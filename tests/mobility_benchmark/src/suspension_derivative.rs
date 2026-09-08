//! Explicit offline velocity reconstruction for acquired suspension positions.
//! This is not a causal controller sensor or an automatic acquisition converter.

use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};

use crate::suspension_acquisition::{
    SuspensionEvidenceFileRef, SuspensionPhysicalAcquisitionManifest, SuspensionSignalOrigin,
};
use crate::suspension_identification::SuspensionIdentificationDataset;
use crate::suspension_runs::MAX_SUSPENSION_RUN_BYTES;
use crate::suspension_uncertainty::{
    propagate_acquired_derived_errors, SuspensionAcquiredErrorRequest, SuspensionErrorPropagation,
};

/// Exact file-bound inputs for derivative-aware additive error propagation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionDerivedErrorRequest {
    /// Acquisitions, seeded error assumptions and per-factor calibration references.
    pub errors: SuspensionAcquiredErrorRequest,
    /// One explicit derivative interpretation per training acquisition, in order.
    pub bindings: Vec<SuspensionDerivativeBinding>,
}

/// Replayable derivative diagnostics, without a coverage or budget-match verdict.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionDerivedErrorEvidence {
    /// Must be `rne_suspension_derived_error_evidence`.
    pub kind: String,
    /// Independent evidence schema, currently 1.
    pub schema_version: u32,
    /// Exact source declarations and numerical assumptions.
    pub request: SuspensionDerivedErrorRequest,
    /// Baseline and every draw, including invalid and nonphysical fits.
    pub propagation: SuspensionErrorPropagation,
}

impl SuspensionDerivedErrorRequest {
    /// Verifies retained files and executes all draws, without discarding failures.
    pub fn evaluate(&self, root: &std::path::Path) -> Result<SuspensionDerivedErrorEvidence> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "derived error request too large"
        );
        self.errors.validate()?;
        let propagation = propagate_acquired_derived_errors(
            &self.errors.acquisitions,
            &self.errors.model,
            &self.bindings,
            root,
        )?;
        let evidence = SuspensionDerivedErrorEvidence {
            kind: "rne_suspension_derived_error_evidence".into(),
            schema_version: 1,
            request: self.clone(),
            propagation,
        };
        ensure!(
            serde_json::to_vec(&evidence)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "derived error evidence too large"
        );
        Ok(evidence)
    }
}

impl SuspensionDerivedErrorEvidence {
    /// Reopens retained files and recomputes the entire evidence, not just a hash.
    pub fn verify(&self, root: &std::path::Path) -> Result<()> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "derived error evidence too large"
        );
        ensure!(
            *self == self.request.evaluate(root)?,
            "derived error evidence replay mismatch"
        );
        Ok(())
    }
}

/// Strict 8 MiB-bounded decoding followed by file checks and actual recomputation.
pub fn decode_suspension_derived_errors(
    bytes: &[u8],
    root: &std::path::Path,
) -> Result<SuspensionDerivedErrorEvidence> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "derived error evidence too large"
    );
    let evidence: SuspensionDerivedErrorEvidence = serde_json::from_slice(bytes)?;
    evidence.verify(root)?;
    Ok(evidence)
}

/// Verifies complete evidence before compact serialization.
pub fn encode_suspension_derived_errors(
    evidence: &SuspensionDerivedErrorEvidence,
    root: &std::path::Path,
) -> Result<Vec<u8>> {
    evidence.verify(root)?;
    Ok(serde_json::to_vec(evidence)?)
}

/// Caller-declared executable interpretation of one retained velocity procedure.
/// The file reference binds bytes, not the truth of the caller's interpretation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionDerivativeBinding {
    /// Exact capture identity from the acquisition manifest.
    pub capture_id: String,
    /// Exact derived velocity calibration/procedure reference from the manifest.
    pub procedure: SuspensionEvidenceFileRef,
    /// Explicit numerical procedure; never inferred from a source label.
    pub operator: SuspensionDerivativeOperator,
    /// Caller-declared inclusive maximum absolute reconstruction error (m/s).
    pub absolute_tolerance_m_s: f64,
}

impl SuspensionDerivativeBinding {
    /// Validates declarations and reconstructs every velocity without changing inputs.
    /// Does not read retained files; use `verify_files` for file-bound verification.
    /// A mismatch, including at an endpoint, rejects the entire acquisition.
    pub fn validate(
        &self,
        dataset: &SuspensionIdentificationDataset,
        manifest: &SuspensionPhysicalAcquisitionManifest,
    ) -> Result<Vec<f64>> {
        manifest.validate(dataset)?;
        // Manifest validation fixes the position/velocity/force order and size.
        let velocity = &manifest.signals[1];
        ensure!(
            self.capture_id == manifest.capture_id,
            "derivative capture mismatch"
        );
        ensure!(
            velocity.origin == SuspensionSignalOrigin::Derived
                && self.procedure == velocity.calibration_artifact,
            "derivative procedure binding mismatch"
        );
        ensure!(
            self.absolute_tolerance_m_s.is_finite() && self.absolute_tolerance_m_s >= 0.0,
            "invalid derivative reconstruction tolerance"
        );
        let times: Vec<_> = dataset.samples.iter().map(|s| s.capture_time_s).collect();
        let positions: Vec<_> = dataset.samples.iter().map(|s| s.position_m).collect();
        let velocities = self.operator.reconstruct(&times, &positions)?;
        for (row, (reconstructed, sample)) in velocities.iter().zip(&dataset.samples).enumerate() {
            let error_m_s = (reconstructed - sample.velocity_m_s).abs();
            ensure!(
                error_m_s.is_finite() && error_m_s <= self.absolute_tolerance_m_s,
                "derived velocity mismatch at row {row}"
            );
        }
        Ok(velocities)
    }

    /// Checks all retained raw/procedure/calibration bytes and nominal reconstruction.
    /// This proves neither calibration authenticity nor physical model acceptance.
    pub fn verify_files(
        &self,
        dataset: &SuspensionIdentificationDataset,
        manifest: &SuspensionPhysicalAcquisitionManifest,
        root: &std::path::Path,
    ) -> Result<Vec<f64>> {
        let velocities = self.validate(dataset, manifest)?;
        manifest.verify_files(dataset, root)?;
        Ok(velocities)
    }
}

/// Independent latent source coupling elapsed-time, position and force corrections.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAffineFactor {
    /// Strictly increasing unique stream identity in the model.
    pub factor_id: u64,
    /// Explicit zero-mean unit-variance distribution.
    pub distribution: crate::suspension_uncertainty::SuspensionErrorDistribution,
    /// Shared training or per-acquisition; sample scope is rejected for affine corrections.
    pub scope: crate::suspension_uncertainty::SuspensionErrorScope,
    /// Signed dimensionless elapsed-time scale loading about the nominal correction.
    pub time_scale_loading: f64,
    /// Signed timestamp translation loading in seconds.
    pub time_offset_loading_s: f64,
    /// Signed dimensionless position scale loading.
    pub position_scale_loading: f64,
    /// Signed position translation loading in metres.
    pub position_offset_loading_m: f64,
    /// Signed dimensionless force scale loading.
    pub force_scale_loading: f64,
    /// Signed force translation loading in newtons.
    pub force_offset_loading_n: f64,
}

/// Seeded affine diagnostics only; does not establish calibration provenance.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAffineErrorModel {
    /// Must be `rne_suspension_affine_error_model`.
    pub kind: String,
    /// Currently 1; independent of the additive error model.
    pub schema_version: u32,
    /// Explicit WorldRandom seed.
    pub seed: u64,
    /// Number of realizations, in 1..=4096.
    pub draws: usize,
    /// One nominal correction and explicit derivative operator per training run.
    pub nominal: Vec<(SuspensionAffineCorrection, SuspensionDerivativeOperator)>,
    /// Independent sources, whose signed loadings create within-source correlation.
    pub factors: Vec<SuspensionAffineFactor>,
}

impl SuspensionAffineErrorModel {
    /// Perturbs nominal corrections, reconstructs velocity and refits every draw.
    /// Holdout values are validated but never used in fitting or random sampling.
    /// Nonpositive sampled scales and numerical failures are retained, not redrawn.
    /// No automatic expanded-uncertainty conversion, clipping or coverage claim.
    pub fn propagate(
        &self,
        request: &crate::suspension_runs::SuspensionRunRequest,
    ) -> Result<SuspensionErrorPropagation> {
        use crate::suspension_uncertainty::{draw_rng, latent, SuspensionErrorScope};
        request.validate()?;
        ensure!(
            self.kind == "rne_suspension_affine_error_model" && self.schema_version == 1,
            "affine model kind/schema drift"
        );
        ensure!(
            (1..=4096).contains(&self.draws) && (1..=16).contains(&self.factors.len()),
            "invalid affine workload"
        );
        ensure!(
            self.nominal.len() == request.training.len(),
            "affine nominal count mismatch"
        );
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "affine model too large"
        );
        let rows: usize = request
            .training
            .iter()
            .map(|r| r.dataset.samples.len())
            .sum();
        ensure!(
            rows.saturating_mul(self.draws)
                .saturating_mul(self.factors.len())
                <= 10_000_000,
            "affine work budget exceeded"
        );
        ensure!(
            self.factors
                .windows(2)
                .all(|pair| pair[0].factor_id < pair[1].factor_id),
            "affine factors must have increasing unique IDs"
        );
        for factor in &self.factors {
            ensure!(
                factor.scope != SuspensionErrorScope::Sample,
                "affine sample scope unsupported"
            );
            ensure!(
                [
                    factor.time_scale_loading,
                    factor.time_offset_loading_s,
                    factor.position_scale_loading,
                    factor.position_offset_loading_m,
                    factor.force_scale_loading,
                    factor.force_offset_loading_n
                ]
                .into_iter()
                .all(f64::is_finite),
                "nonfinite affine loading"
            );
        }
        let runs: Vec<_> = request
            .training
            .iter()
            .map(|run| rne_robot::SuspensionIdentificationRun {
                acquisition_id: run.acquisition_id,
                samples: &run.dataset.samples,
            })
            .collect();
        // Malformed nominal corrections are request errors, not stochastic outcomes.
        for (run, (correction, operator)) in runs.iter().zip(&self.nominal) {
            correction.apply(run.samples, *operator)?;
        }
        let baseline = SuspensionAffineCorrection::fit_training(request.spec, &runs, &self.nominal);
        let random = rne_world::WorldRandom::new(self.seed);
        let mut draws = Vec::with_capacity(self.draws);
        for draw in 0..self.draws {
            let mut corrections = self.nominal.clone();
            for factor in &self.factors {
                let mut rng = draw_rng(&random, factor.factor_id, draw);
                let shared = latent(&mut rng, factor.distribution);
                for (correction, _) in &mut corrections {
                    let value = match factor.scope {
                        SuspensionErrorScope::SharedTraining => shared,
                        SuspensionErrorScope::Acquisition => latent(&mut rng, factor.distribution),
                        SuspensionErrorScope::Sample => unreachable!("validated affine scope"),
                    };
                    correction.time_scale += value * factor.time_scale_loading;
                    correction.time_offset_s += value * factor.time_offset_loading_s;
                    correction.position_scale += value * factor.position_scale_loading;
                    correction.position_offset_m += value * factor.position_offset_loading_m;
                    correction.force_scale += value * factor.force_scale_loading;
                    correction.force_offset_n += value * factor.force_offset_loading_n;
                }
            }
            draws.push(SuspensionAffineCorrection::fit_training(
                request.spec,
                &runs,
                &corrections,
            ));
        }
        Ok(SuspensionErrorPropagation { baseline, draws })
    }
}

/// One explicitly applied affine correction, not an instrument-error estimate.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAffineCorrection {
    /// Reference timestamp for elapsed-time scaling, in seconds.
    pub time_reference_s: f64,
    /// Positive elapsed-time multiplier; not an oscillator frequency offset.
    pub time_scale: f64,
    /// Common timestamp translation in seconds, applied after scaling.
    pub time_offset_s: f64,
    /// Positive position multiplier about coordinate zero.
    pub position_scale: f64,
    /// Position translation after scaling, in metres.
    pub position_offset_m: f64,
    /// Positive force multiplier about force zero.
    pub force_scale: f64,
    /// Force translation after scaling, in newtons.
    pub force_offset_n: f64,
}

impl SuspensionAffineCorrection {
    /// Reconstructs corrected runs and fits their combined training rows.
    /// One correction and operator must be supplied per acquisition, in order.
    /// This has no holdout input and does not infer calibration or distributions.
    /// Invalid transformed samples are reported as `InvalidSample`; estimator
    /// errors (including nonphysical coefficients) are preserved unchanged.
    pub fn fit_training(
        spec: rne_robot::SuspensionIdentificationSpec,
        runs: &[rne_robot::SuspensionIdentificationRun<'_>],
        corrections: &[(Self, SuspensionDerivativeOperator)],
    ) -> std::result::Result<
        rne_robot::SuspensionTrainingCoefficients,
        rne_robot::SuspensionIdentificationError,
    > {
        use rne_robot::{SuspensionIdentificationError, SuspensionIdentificationRun};
        if runs.is_empty() || runs.len() > 64 || runs.len() != corrections.len() {
            return Err(SuspensionIdentificationError::InvalidSample);
        }
        let rows = runs
            .iter()
            .try_fold(0usize, |total, run| total.checked_add(run.samples.len()));
        if rows.is_none_or(|rows| rows > 100_000) {
            return Err(SuspensionIdentificationError::InvalidSample);
        }
        let samples: Vec<_> = runs
            .iter()
            .zip(corrections)
            .map(|(run, (correction, operator))| {
                correction
                    .apply(run.samples, *operator)
                    .map_err(|_| SuspensionIdentificationError::InvalidSample)
            })
            .collect::<std::result::Result<_, _>>()?;
        let corrected: Vec<_> = runs
            .iter()
            .zip(&samples)
            .map(|(run, samples)| SuspensionIdentificationRun {
                acquisition_id: run.acquisition_id,
                samples,
            })
            .collect();
        rne_robot::fit_suspension_training_runs(spec, &corrected)
    }

    /// Applies a single correction to one acquisition without mutating the input.
    /// Velocity is reconstructed, not independently scaled from the input column.
    /// Rejects nonfinite inputs/arithmetic, nonpositive scales and collapsed or
    /// reversed clocks. Does not sample uncertainty, clip values or remove rows.
    pub fn apply(
        self,
        samples: &[rne_robot::SuspensionForceSample],
        operator: SuspensionDerivativeOperator,
    ) -> Result<Vec<rne_robot::SuspensionForceSample>> {
        ensure!(
            (3..=100_000).contains(&samples.len()),
            "invalid affine sample count"
        );
        ensure!(
            [
                self.time_reference_s,
                self.time_offset_s,
                self.position_offset_m,
                self.force_offset_n
            ]
            .into_iter()
            .all(f64::is_finite),
            "non-finite affine offset"
        );
        ensure!(
            [self.time_scale, self.position_scale, self.force_scale]
                .into_iter()
                .all(|scale| scale.is_finite() && scale > 0.0),
            "invalid affine scale"
        );
        ensure!(
            samples.iter().all(
                |s| [s.capture_time_s, s.position_m, s.velocity_m_s, s.force_n]
                    .into_iter()
                    .all(f64::is_finite)
            ),
            "non-finite affine input"
        );
        ensure!(
            samples
                .windows(2)
                .all(|s| s[0].capture_time_s < s[1].capture_time_s),
            "invalid affine input clock"
        );
        let mut corrected = Vec::with_capacity(samples.len());
        for sample in samples {
            let capture_time_s = self.time_reference_s
                + self.time_scale * (sample.capture_time_s - self.time_reference_s)
                + self.time_offset_s;
            let position_m = self.position_scale * sample.position_m + self.position_offset_m;
            let force_n = self.force_scale * sample.force_n + self.force_offset_n;
            ensure!(
                [capture_time_s, position_m, force_n]
                    .into_iter()
                    .all(f64::is_finite),
                "non-finite affine result"
            );
            corrected.push(rne_robot::SuspensionForceSample {
                capture_time_s,
                position_m,
                force_n,
                velocity_m_s: 0.0,
            });
        }
        let times: Vec<_> = corrected.iter().map(|s| s.capture_time_s).collect();
        let positions: Vec<_> = corrected.iter().map(|s| s.position_m).collect();
        for (sample, velocity) in corrected
            .iter_mut()
            .zip(operator.reconstruct(&times, &positions)?)
        {
            sample.velocity_m_s = velocity;
        }
        Ok(corrected)
    }
}

/// Versioned numerical derivative, with no implicit filtering or resampling.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionDerivativeOperator {
    /// Three-point quadratic derivative at interior samples using actual times;
    /// the two endpoints use their adjacent two-point secant. Requires >= 3 rows.
    /// Interior values need one future position: offline use only.
    NonuniformThreePointSecantEndsV1,
}

impl SuspensionDerivativeOperator {
    /// Returns one velocity (m/s) per input timestamp (s), in unchanged row order.
    ///
    /// Position is in metres. Clocks must be finite and strictly increasing
    /// within this single acquisition. No sorting, trimming, filtering, nominal
    /// rate substitution, or cross-acquisition differencing is performed.
    /// Invalid inputs or non-finite arithmetic fail the entire realization.
    pub fn reconstruct(self, timestamps_s: &[f64], positions_m: &[f64]) -> Result<Vec<f64>> {
        ensure!(
            timestamps_s.len() == positions_m.len() && (3..=100_000).contains(&timestamps_s.len()),
            "derivative requires 3..=100000 matching time/position rows"
        );
        ensure!(
            timestamps_s
                .iter()
                .chain(positions_m)
                .all(|x| x.is_finite()),
            "non-finite derivative input"
        );
        let mut intervals_s = Vec::with_capacity(timestamps_s.len() - 1);
        let mut slopes_m_s = Vec::with_capacity(timestamps_s.len() - 1);
        for (t, x) in timestamps_s.windows(2).zip(positions_m.windows(2)) {
            let dt_s = t[1] - t[0];
            ensure!(
                dt_s.is_finite() && dt_s > 0.0,
                "invalid derivative clock interval"
            );
            let slope_m_s = (x[1] - x[0]) / dt_s;
            ensure!(slope_m_s.is_finite(), "non-finite derivative secant");
            intervals_s.push(dt_s);
            slopes_m_s.push(slope_m_s);
        }
        let mut velocities_m_s = Vec::with_capacity(timestamps_s.len());
        velocities_m_s.push(slopes_m_s[0]);
        for i in 1..timestamps_s.len() - 1 {
            let left_s = intervals_s[i - 1];
            let right_s = intervals_s[i];
            // Scale before summation to avoid overflow for large valid intervals.
            let scale_s = left_s.max(right_s);
            let left = left_s / scale_s;
            let right = right_s / scale_s;
            // Derivative of the interpolating quadratic, expressed as weighted
            // adjacent secants to cancel a constant position offset naturally.
            let velocity_m_s = (right / (left + right)) * slopes_m_s[i - 1]
                + (left / (left + right)) * slopes_m_s[i];
            ensure!(velocity_m_s.is_finite(), "non-finite derivative result");
            velocities_m_s.push(velocity_m_s);
        }
        velocities_m_s.push(slopes_m_s[slopes_m_s.len() - 1]);
        Ok(velocities_m_s)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const OP: SuspensionDerivativeOperator =
        SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1;

    fn close(actual: &[f64], expected: &[f64]) {
        assert_eq!(actual.len(), expected.len());
        for (a, b) in actual.iter().zip(expected) {
            assert!((a - b).abs() < 1e-12, "{a} != {b}");
        }
    }

    #[test]
    fn affine_force_law_fit_matches_analytic_coefficient_transform() {
        use crate::suspension_identification::suspension_identification_spec;
        use rne_robot::{
            fit_suspension_training_runs, SuspensionForceSample, SuspensionIdentificationRun,
        };
        let mut samples: Vec<_> = (0..200)
            .map(|i| {
                let t = 0.01 * i as f64 + 0.0001 * (i % 2) as f64;
                SuspensionForceSample {
                    capture_time_s: t,
                    position_m: -0.06 + 0.02 * (7.0 * t).sin() + 0.003 * (13.0 * t).cos(),
                    velocity_m_s: 0.0,
                    force_n: 0.0,
                }
            })
            .collect();
        let times: Vec<_> = samples.iter().map(|s| s.capture_time_s).collect();
        let positions: Vec<_> = samples.iter().map(|s| s.position_m).collect();
        for (sample, velocity) in samples
            .iter_mut()
            .zip(OP.reconstruct(&times, &positions).unwrap())
        {
            sample.velocity_m_s = velocity;
            sample.force_n = -200_000.0 * (sample.position_m + 0.06) - 15_000.0 * velocity;
        }
        let identity = SuspensionAffineCorrection {
            time_reference_s: 0.5,
            time_scale: 1.0,
            time_offset_s: 0.0,
            position_scale: 1.0,
            position_offset_m: 0.0,
            force_scale: 1.0,
            force_offset_n: 0.0,
        };
        for correction in [
            identity,
            SuspensionAffineCorrection {
                time_offset_s: 4.0,
                ..identity
            },
            SuspensionAffineCorrection {
                time_scale: 1.25,
                ..identity
            },
            SuspensionAffineCorrection {
                time_scale: 1.01,
                time_offset_s: 2.0,
                position_scale: 1.2,
                position_offset_m: 0.005,
                force_scale: 0.9,
                force_offset_n: 100.0,
                ..identity
            },
        ] {
            let corrected = correction.apply(&samples, OP).unwrap();
            let fitted = fit_suspension_training_runs(
                suspension_identification_spec(),
                &[SuspensionIdentificationRun {
                    acquisition_id: 1,
                    samples: &corrected,
                }],
            )
            .unwrap();
            let expected_k = correction.force_scale * 200_000.0 / correction.position_scale;
            let expected_c = correction.force_scale * correction.time_scale * 15_000.0
                / correction.position_scale;
            let expected_e = correction.position_scale * -0.06
                + correction.position_offset_m
                + correction.force_offset_n / expected_k;
            assert!((fitted.stiffness_n_per_m - expected_k).abs() < 1e-5);
            assert!((fitted.damping_n_s_per_m - expected_c).abs() < 1e-6);
            assert!((fitted.equilibrium_position_m - expected_e).abs() < 1e-12);
            let runs = [
                SuspensionIdentificationRun {
                    acquisition_id: 1,
                    samples: &samples,
                },
                SuspensionIdentificationRun {
                    acquisition_id: 2,
                    samples: &samples,
                },
            ];
            let fit = SuspensionAffineCorrection::fit_training(
                suspension_identification_spec(),
                &runs,
                &[(correction, OP); 2],
            )
            .unwrap();
            assert!((fit.stiffness_n_per_m - expected_k).abs() < 1e-5);
            assert!((fit.damping_n_s_per_m - expected_c).abs() < 1e-6);
            assert!((fit.equilibrium_position_m - expected_e).abs() < 1e-12);
            use rne_robot::SuspensionIdentificationError as Error;
            let outcomes: Vec<_> = [
                correction,
                SuspensionAffineCorrection {
                    time_scale: 0.0,
                    ..correction
                },
                SuspensionAffineCorrection {
                    force_scale: 100.0,
                    ..correction
                },
                correction,
            ]
            .into_iter()
            .map(|value| {
                SuspensionAffineCorrection::fit_training(
                    suspension_identification_spec(),
                    &runs,
                    &[(value, OP); 2],
                )
            })
            .collect();
            assert_eq!(
                outcomes,
                vec![
                    Ok(fit),
                    Err(Error::InvalidSample),
                    Err(Error::NonPhysicalResult),
                    Ok(fit)
                ]
            );
            assert_eq!(
                SuspensionAffineCorrection::fit_training(
                    suspension_identification_spec(),
                    &runs,
                    &[(correction, OP)],
                ),
                Err(Error::InvalidSample)
            );
            let duplicates = [runs[0], runs[0]];
            assert_eq!(
                SuspensionAffineCorrection::fit_training(
                    suspension_identification_spec(),
                    &duplicates,
                    &[(correction, OP); 2],
                ),
                Err(Error::DuplicateAcquisition)
            );
        }
    }

    #[test]
    fn affine_correction_reconstructs_velocity_and_preserves_source() {
        let samples: Vec<_> = [0.0, 0.25, 1.0, 2.0]
            .into_iter()
            .map(|t| rne_robot::SuspensionForceSample {
                capture_time_s: t,
                position_m: t * t,
                velocity_m_s: 999.0,
                force_n: t + 1.0,
            })
            .collect();
        let correction = SuspensionAffineCorrection {
            time_reference_s: 0.0,
            time_scale: 2.0,
            time_offset_s: 16.0,
            position_scale: 3.0,
            position_offset_m: 8.0,
            force_scale: 4.0,
            force_offset_n: 5.0,
        };
        let result = correction.apply(&samples, OP).unwrap();
        close(
            &result.iter().map(|s| s.velocity_m_s).collect::<Vec<_>>(),
            &[0.375, 0.75, 3.0, 4.5],
        );
        close(
            &result.iter().map(|s| s.capture_time_s).collect::<Vec<_>>(),
            &[16.0, 16.5, 18.0, 20.0],
        );
        close(
            &result.iter().map(|s| s.force_n).collect::<Vec<_>>(),
            &[9.0, 10.0, 13.0, 17.0],
        );
        assert!(samples.iter().all(|s| s.velocity_m_s == 999.0));
        assert_eq!(result, correction.apply(&samples, OP).unwrap());
        for scale in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(SuspensionAffineCorrection {
                time_scale: scale,
                ..correction
            }
            .apply(&samples, OP)
            .is_err());
            assert!(SuspensionAffineCorrection {
                position_scale: scale,
                ..correction
            }
            .apply(&samples, OP)
            .is_err());
            assert!(SuspensionAffineCorrection {
                force_scale: scale,
                ..correction
            }
            .apply(&samples, OP)
            .is_err());
        }
        // Large finite translation collapses distinguishable timestamps: reject,
        // never sort or remove rows to manufacture a valid realization.
        assert!(SuspensionAffineCorrection {
            time_offset_s: 1e30,
            ..correction
        }
        .apply(&samples, OP)
        .is_err());
        assert!(SuspensionAffineCorrection {
            force_scale: f64::MAX,
            ..correction
        }
        .apply(&samples, OP)
        .is_err());
        let encoded = serde_json::to_vec(&correction).unwrap();
        assert_eq!(
            serde_json::from_slice::<SuspensionAffineCorrection>(&encoded).unwrap(),
            correction
        );
    }

    #[test]
    fn nonuniform_quadratic_and_explicit_secant_endpoints() {
        let time = [0.0, 0.25, 1.0, 2.0];
        let position = time.map(|t| t * t);
        let velocity = OP.reconstruct(&time, &position).unwrap();
        close(&velocity, &[0.25, 0.5, 2.0, 3.0]);
        assert_eq!(velocity, OP.reconstruct(&time, &position).unwrap());
        let local_clock = time.map(|t| t + 128.0);
        close(&OP.reconstruct(&local_clock, &position).unwrap(), &velocity);
    }

    #[test]
    fn common_offset_sample_noise_and_clock_scale_are_distinct() {
        let time = [0.0, 1.0, 2.0, 3.0, 4.0];
        let position = [0.0; 5];
        close(
            &OP.reconstruct(&time, &position.map(|x| x + 8.0)).unwrap(),
            &[0.0; 5],
        );
        // One position error affects two neighbouring velocities with opposite
        // signs: independent velocity draws would miss this coupling.
        let noisy = [0.0, 0.0, 2.0, 0.0, 0.0];
        close(
            &OP.reconstruct(&time, &noisy).unwrap(),
            &[0.0, 1.0, 0.0, -1.0, 0.0],
        );
        close(
            &OP.reconstruct(&time.map(|t| t * 2.0), &noisy).unwrap(),
            &[0.0, 0.5, 0.0, -0.5, 0.0],
        );
    }

    #[test]
    fn malformed_or_overflowing_realization_is_not_trimmed() {
        for time in [
            [0.0, 0.0, 1.0],
            [0.0, 2.0, 1.0],
            [0.0, f64::NAN, 2.0],
            [-f64::MAX, f64::MAX, f64::MAX],
        ] {
            assert!(OP.reconstruct(&time, &[0.0; 3]).is_err());
        }
        assert!(OP
            .reconstruct(&[0.0, 1.0, 2.0], &[0.0, f64::INFINITY, 0.0])
            .is_err());
        assert!(OP
            .reconstruct(&[0.0, 1.0, 2.0], &[-f64::MAX, f64::MAX, 0.0])
            .is_err());
        assert!(OP
            .reconstruct(&[0.0, 1e-320, 1.0], &[0.0, 1.0, 2.0])
            .is_err());
        assert!(OP.reconstruct(&[0.0, 1.0], &[0.0, 1.0]).is_err());
        assert!(OP.reconstruct(&[0.0, 1.0, 2.0], &[0.0, 1.0]).is_err());
        assert!(OP
            .reconstruct(&vec![0.0; 100_001], &vec![0.0; 100_001])
            .is_err());
    }

    #[test]
    fn operator_identity_is_explicit_and_versioned() {
        let bytes = serde_json::to_vec(&OP).unwrap();
        assert_eq!(
            serde_json::from_slice::<SuspensionDerivativeOperator>(&bytes).unwrap(),
            OP
        );
        assert!(serde_json::from_str::<SuspensionDerivativeOperator>("\"default\"").is_err());
        assert!(serde_json::from_str::<SuspensionDerivativeOperator>(
            "\"nonuniform_three_point_secant_ends_v2\""
        )
        .is_err());
    }
}

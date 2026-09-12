//! Training-only Huber IRLS and separately evaluated, unweighted holdout diagnostics.

use anyhow::{ensure, Result};
use rne_robot::{SuspensionForceSample, SuspensionIdentificationRun};
use serde::{Deserialize, Serialize};

/// Explicit force-residual scale and bounded numerical stopping rule.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionHuberSpec {
    /// Positive quadratic/linear transition in newtons; never inferred from holdout.
    pub delta_n: f64,
    /// One to 256 reweighted solves, excluding the initial unweighted solve.
    pub maximum_iterations: usize,
    /// Maximum change of predicted training force, in newtons, for convergence.
    pub prediction_tolerance_n: f64,
}

/// Last unconstrained force-law iterate, including unsuccessful convergence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionHuberIterate {
    /// Coefficient of position in F = a*x + b*v + d, in newtons per meter.
    pub position_coefficient_n_per_m: f64,
    /// Coefficient of velocity in newton-seconds per meter.
    pub velocity_coefficient_n_s_per_m: f64,
    /// Constant force term in newtons.
    pub intercept_n: f64,
    /// Number of reweighted solves actually performed.
    pub iterations: usize,
    /// Whether the declared prediction-change stopping rule was met.
    pub converged: bool,
    /// Unweighted force RMSE; outlier residuals are not concealed.
    pub training_rmse_n: f64,
    /// Final per-row weights in caller run/sample order; no rows removed.
    pub weights: Vec<f64>,
}

fn predict(beta: [f64; 3], s: &SuspensionForceSample) -> f64 {
    beta[0] * s.position_m + beta[1] * s.velocity_m_s + beta[2]
}

/// Evaluation of a frozen robust iterate without holdout reweighting.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionHuberEvaluation {
    /// Full last iterate; retained even when it is nonconverged or nonphysical.
    pub iterate: SuspensionHuberIterate,
    /// Whether final k, c and equilibrium satisfy the declared physical bounds.
    pub physical_bounds_passed: bool,
    /// Unweighted holdout force RMSE in newtons.
    pub holdout_rmse_n: f64,
    /// Worst absolute holdout force residual in newtons.
    pub maximum_holdout_residual_n: f64,
    /// Unweighted training residuals in input acquisition order.
    pub training_runs: Vec<rne_robot::SuspensionRunResidual>,
    /// Unweighted holdout residuals in input acquisition order.
    pub holdout_runs: Vec<rne_robot::SuspensionRunResidual>,
    /// Requires convergence, physical bounds and unchanged training/holdout RMSE gates.
    pub passed: bool,
    /// Ordinary training-only least-squares result, including nonphysical failures.
    pub ordinary: std::result::Result<
        rne_robot::SuspensionTrainingCoefficients,
        rne_robot::SuspensionIdentificationError,
    >,
}

/// Versioned numerical evidence; acquisition authenticity is not established.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionHuberEvidence {
    /// Must be `rne_suspension_huber_evidence`.
    pub kind: String,
    /// Independent envelope version, currently 1.
    pub schema_version: u32,
    /// Full immutable training/holdout split and physical acceptance gates.
    pub request: crate::suspension_runs::SuspensionRunRequest,
    /// Explicit scale and stopping assumptions.
    pub robust_spec: SuspensionHuberSpec,
    /// Complete result including failed gates, weights and the ordinary fit.
    pub evaluation: SuspensionHuberEvaluation,
}

const MAX_HUBER_EVIDENCE_BYTES: usize = 8 * 1024 * 1024;

/// Huber result bound to declared acquisition manifests and retained file bytes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAcquiredHuberRequest {
    /// Exact datasets and declared source/calibration file references.
    pub acquisitions: crate::suspension_runs::SuspensionAcquiredRunRequest,
    /// Explicit numerical assumptions.
    pub robust_spec: SuspensionHuberSpec,
}

/// Huber result bound to declared acquisition manifests and retained file bytes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAcquiredHuberEvidence {
    /// Must be `rne_suspension_acquired_huber_evidence`.
    pub kind: String,
    /// Independent envelope version, currently 1.
    pub schema_version: u32,
    /// Exact acquisition declarations and training/holdout data.
    pub acquisitions: crate::suspension_runs::SuspensionAcquiredRunRequest,
    /// Explicit numerical assumptions, not inferred from calibration certificates.
    pub robust_spec: SuspensionHuberSpec,
    /// Full evaluation, including unsuccessful physical or residual gates.
    pub evaluation: SuspensionHuberEvaluation,
}

impl SuspensionAcquiredHuberEvidence {
    /// Streams and hashes declared external files before evaluating training data.
    /// Does not authenticate certificates or execute a declared processing procedure.
    pub fn evaluate(
        acquisitions: &crate::suspension_runs::SuspensionAcquiredRunRequest,
        robust_spec: SuspensionHuberSpec,
        root: &std::path::Path,
    ) -> Result<Self> {
        acquisitions.verify_files(root)?;
        let evidence = Self {
            kind: "rne_suspension_acquired_huber_evidence".into(),
            schema_version: 1,
            evaluation: evaluate_suspension_huber(&acquisitions.runs, robust_spec)?,
            acquisitions: acquisitions.clone(),
            robust_spec,
        };
        ensure!(
            serde_json::to_vec(&evidence)?.len() <= MAX_HUBER_EVIDENCE_BYTES,
            "acquired Huber evidence exceeds byte limit"
        );
        Ok(evidence)
    }

    /// Rechecks file bytes and the entire fit; stored verdicts are not trusted.
    pub fn verify(&self, root: &std::path::Path) -> Result<()> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_HUBER_EVIDENCE_BYTES,
            "acquired Huber evidence exceeds byte limit"
        );
        ensure!(
            self.kind == "rne_suspension_acquired_huber_evidence" && self.schema_version == 1,
            "acquired Huber kind/schema drift"
        );
        ensure!(
            *self == Self::evaluate(&self.acquisitions, self.robust_spec, root)?,
            "acquired Huber replay mismatch"
        );
        Ok(())
    }
}

/// Strict bounded acquired evidence intake with external-byte and numerical replay.
pub fn decode_suspension_acquired_huber(
    bytes: &[u8],
    root: &std::path::Path,
) -> Result<SuspensionAcquiredHuberEvidence> {
    ensure!(
        bytes.len() <= MAX_HUBER_EVIDENCE_BYTES,
        "acquired Huber evidence exceeds byte limit"
    );
    let evidence: SuspensionAcquiredHuberEvidence = serde_json::from_slice(bytes)?;
    evidence.verify(root)?;
    Ok(evidence)
}

impl SuspensionHuberEvidence {
    /// Recomputes the full evaluation; does not trust stored flags or coefficients.
    pub fn verify(&self) -> Result<()> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_HUBER_EVIDENCE_BYTES,
            "Huber evidence exceeds byte limit"
        );
        ensure!(
            self.kind == "rne_suspension_huber_evidence" && self.schema_version == 1,
            "Huber evidence kind/schema drift"
        );
        ensure!(
            self.evaluation == evaluate_suspension_huber(&self.request, self.robust_spec)?,
            "Huber evidence replay mismatch"
        );
        Ok(())
    }
}

/// Creates bounded numerical evidence without acquisition-file verification.
pub fn identify_suspension_huber(
    request: &crate::suspension_runs::SuspensionRunRequest,
    robust_spec: SuspensionHuberSpec,
) -> Result<SuspensionHuberEvidence> {
    let evidence = SuspensionHuberEvidence {
        kind: "rne_suspension_huber_evidence".into(),
        schema_version: 1,
        evaluation: evaluate_suspension_huber(request, robust_spec)?,
        request: request.clone(),
        robust_spec,
    };
    ensure!(
        serde_json::to_vec(&evidence)?.len() <= MAX_HUBER_EVIDENCE_BYTES,
        "Huber evidence exceeds byte limit"
    );
    Ok(evidence)
}

/// Strict bounded decoding followed by actual full numerical replay.
pub fn decode_suspension_huber(bytes: &[u8]) -> Result<SuspensionHuberEvidence> {
    ensure!(
        bytes.len() <= MAX_HUBER_EVIDENCE_BYTES,
        "Huber evidence exceeds byte limit"
    );
    let evidence: SuspensionHuberEvidence = serde_json::from_slice(bytes)?;
    evidence.verify()?;
    Ok(evidence)
}

/// Encodes compact JSON only after revalidation, including unsuccessful verdicts.
pub fn encode_suspension_huber(evidence: &SuspensionHuberEvidence) -> Result<Vec<u8>> {
    evidence.verify()?;
    Ok(serde_json::to_vec(evidence)?)
}

/// Evaluates a validated whole-acquisition split. Holdout values never enter IRLS.
/// Does not verify physical acquisition files or grant calibration qualification.
pub fn evaluate_suspension_huber(
    request: &crate::suspension_runs::SuspensionRunRequest,
    robust_spec: SuspensionHuberSpec,
) -> Result<SuspensionHuberEvaluation> {
    request.validate()?;
    let spec = request.spec;
    let training: Vec<_> = request
        .training
        .iter()
        .map(|run| SuspensionIdentificationRun {
            acquisition_id: run.acquisition_id,
            samples: &run.dataset.samples,
        })
        .collect();
    let training_count: usize = training.iter().map(|run| run.samples.len()).sum();
    let holdout_count: usize = request
        .holdout
        .iter()
        .map(|run| run.dataset.samples.len())
        .sum();
    ensure!(
        training_count >= spec.minimum_training_samples
            && holdout_count >= spec.minimum_holdout_samples,
        "insufficient training or holdout samples"
    );
    let iterate = fit_suspension_huber_iterate(robust_spec, &training)?;
    let ordinary = rne_robot::fit_suspension_training_runs(spec, &training);
    let k = -iterate.position_coefficient_n_per_m;
    let c = -iterate.velocity_coefficient_n_s_per_m;
    let equilibrium = iterate.intercept_n / k;
    let physical_bounds_passed = k.is_finite()
        && c.is_finite()
        && equilibrium.is_finite()
        && (spec.stiffness_bounds_n_per_m[0]..=spec.stiffness_bounds_n_per_m[1]).contains(&k)
        && (spec.damping_bounds_n_s_per_m[0]..=spec.damping_bounds_n_s_per_m[1]).contains(&c)
        && (spec.equilibrium_position_bounds_m[0]..=spec.equilibrium_position_bounds_m[1])
            .contains(&equilibrium);
    let beta = [
        iterate.position_coefficient_n_per_m,
        iterate.velocity_coefficient_n_s_per_m,
        iterate.intercept_n,
    ];
    let residuals = |runs: &[crate::suspension_runs::SuspensionRunInput], limit_n: f64| {
        runs.iter()
            .map(|run| {
                let mut squared = 0.0;
                let mut maximum = 0.0_f64;
                for sample in &run.dataset.samples {
                    let residual = (predict(beta, sample) - sample.force_n).abs();
                    ensure!(residual.is_finite(), "nonfinite acquisition residual");
                    squared += residual * residual;
                    maximum = maximum.max(residual);
                }
                let rmse_n = (squared / run.dataset.samples.len() as f64).sqrt();
                ensure!(rmse_n.is_finite(), "nonfinite acquisition RMSE");
                Ok(rne_robot::SuspensionRunResidual {
                    acquisition_id: run.acquisition_id,
                    sample_count: run.dataset.samples.len(),
                    rmse_n,
                    maximum_absolute_residual_n: maximum,
                    passed: rmse_n <= limit_n,
                })
            })
            .collect::<Result<Vec<_>>>()
    };
    let training_runs = residuals(&request.training, spec.maximum_training_rmse_n)?;
    let holdout_runs = residuals(&request.holdout, spec.maximum_holdout_rmse_n)?;
    let mut squared = 0.0;
    let mut maximum_holdout_residual_n = 0.0_f64;
    for sample in request.holdout.iter().flat_map(|run| &run.dataset.samples) {
        let residual = (predict(beta, sample) - sample.force_n).abs();
        ensure!(residual.is_finite(), "nonfinite holdout residual");
        squared += residual * residual;
        maximum_holdout_residual_n = maximum_holdout_residual_n.max(residual);
    }
    let holdout_rmse_n = (squared / holdout_count as f64).sqrt();
    ensure!(holdout_rmse_n.is_finite(), "nonfinite holdout RMSE");
    let passed = iterate.converged
        && physical_bounds_passed
        && iterate.training_rmse_n <= spec.maximum_training_rmse_n
        && holdout_rmse_n <= spec.maximum_holdout_rmse_n
        && training_runs
            .iter()
            .chain(&holdout_runs)
            .all(|run| run.passed);
    Ok(SuspensionHuberEvaluation {
        iterate,
        ordinary,
        physical_bounds_passed,
        holdout_rmse_n,
        maximum_holdout_residual_n,
        training_runs,
        holdout_runs,
        passed,
    })
}

fn solve(samples: &[SuspensionForceSample], weights: &[f64]) -> Result<[f64; 3]> {
    let mut sums = [0.0; 4];
    for (s, w) in samples.iter().zip(weights) {
        sums[0] += w;
        sums[1] += w * s.position_m;
        sums[2] += w * s.velocity_m_s;
        sums[3] += w * s.force_n;
    }
    ensure!(
        sums.iter().all(|v| v.is_finite()) && sums[0] > 0.0,
        "invalid weighted means"
    );
    let means = [sums[1] / sums[0], sums[2] / sums[0], sums[3] / sums[0]];
    let mut moments = [0.0; 5];
    for (s, w) in samples.iter().zip(weights) {
        let x = s.position_m - means[0];
        let v = s.velocity_m_s - means[1];
        let f = s.force_n - means[2];
        moments[0] += w * x * x;
        moments[1] += w * v * v;
        moments[2] += w * x * v;
        moments[3] += w * x * f;
        moments[4] += w * v * f;
    }
    let [xx, vv, xv, xf, vf] = moments;
    let determinant = xx * vv - xv * xv;
    ensure!(
        moments.iter().all(|v| v.is_finite())
            && determinant.is_finite()
            && xx > 0.0
            && vv > 0.0
            && determinant > 1e-12 * xx * vv,
        "rank-deficient or nonfinite weighted excitation"
    );
    let a = (xf * vv - vf * xv) / determinant;
    let b = (vf * xx - xf * xv) / determinant;
    let beta = [a, b, means[2] - a * means[0] - b * means[1]];
    ensure!(beta.iter().all(|v| v.is_finite()), "nonfinite coefficients");
    Ok(beta)
}

/// Fits unconstrained coefficients using training runs only, without sorting or trimming.
/// Caps 64 unique runs, 100,000 rows and 10 million row-iteration evaluations.
/// Nonconvergence returns the last iterate with `converged=false`; numerical errors
/// reject the request. Physical bounds, holdout evaluation and calibration binding
/// must be applied separately before these coefficients can qualify a force model.
pub fn fit_suspension_huber_iterate(
    spec: SuspensionHuberSpec,
    runs: &[SuspensionIdentificationRun<'_>],
) -> Result<SuspensionHuberIterate> {
    ensure!(
        spec.delta_n.is_finite()
            && spec.delta_n > 0.0
            && spec.prediction_tolerance_n.is_finite()
            && spec.prediction_tolerance_n > 0.0
            && (1..=256).contains(&spec.maximum_iterations),
        "invalid Huber spec"
    );
    ensure!((1..=64).contains(&runs.len()), "invalid training run count");
    let mut ids = std::collections::BTreeSet::new();
    let mut samples = Vec::new();
    for run in runs {
        ensure!(
            ids.insert(run.acquisition_id) && !run.samples.is_empty(),
            "duplicate or empty run"
        );
        ensure!(
            samples.len() + run.samples.len() <= 100_000,
            "too many training rows"
        );
        ensure!(
            run.samples.iter().all(
                |s| [s.capture_time_s, s.position_m, s.velocity_m_s, s.force_n]
                    .iter()
                    .all(|v| v.is_finite())
            ),
            "nonfinite sample"
        );
        ensure!(
            run.samples.windows(2).all(|p| {
                let dt = p[1].capture_time_s - p[0].capture_time_s;
                dt.is_finite() && dt > 0.0
            }),
            "invalid capture order"
        );
        samples.extend_from_slice(run.samples);
    }
    ensure!(
        samples.len() >= 3 && samples.len().saturating_mul(spec.maximum_iterations) <= 10_000_000,
        "invalid Huber workload"
    );
    let mut weights = vec![1.0; samples.len()];
    let mut beta = solve(&samples, &weights)?;
    let mut converged = false;
    let mut iterations = 0;
    let reweight = |beta, weights: &mut [f64]| -> Result<()> {
        for (s, w) in samples.iter().zip(weights) {
            let residual = (predict(beta, s) - s.force_n).abs();
            ensure!(residual.is_finite(), "nonfinite force residual");
            *w = if residual <= spec.delta_n {
                1.0
            } else {
                spec.delta_n / residual
            };
            ensure!(*w > 0.0, "underflowed Huber weight");
        }
        Ok(())
    };
    for iteration in 1..=spec.maximum_iterations {
        reweight(beta, &mut weights)?;
        let next = solve(&samples, &weights)?;
        let mut change = 0.0_f64;
        for s in &samples {
            let delta = (predict(next, s) - predict(beta, s)).abs();
            ensure!(delta.is_finite(), "nonfinite prediction change");
            change = change.max(delta);
        }
        beta = next;
        iterations = iteration;
        if change <= spec.prediction_tolerance_n {
            converged = true;
            break;
        }
    }
    reweight(beta, &mut weights)?;
    let squared: f64 = samples
        .iter()
        .map(|s| (predict(beta, s) - s.force_n).powi(2))
        .sum();
    let training_rmse_n = (squared / samples.len() as f64).sqrt();
    ensure!(training_rmse_n.is_finite(), "nonfinite unweighted RMSE");
    Ok(SuspensionHuberIterate {
        position_coefficient_n_per_m: beta[0],
        velocity_coefficient_n_s_per_m: beta[1],
        intercept_n: beta[2],
        iterations,
        converged,
        training_rmse_n,
        weights,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn holdout_is_not_reweighted_or_used_to_fit() {
        use crate::suspension_identification::{
            suspension_identification_spec, synthetic_suspension_identification_dataset,
        };
        use crate::suspension_runs::{SuspensionRunInput, SuspensionRunRequest};
        let first = synthetic_suspension_identification_dataset().unwrap();
        let mut second = first.clone();
        second.dataset_id = "synthetic.robust.holdout".into();
        for s in &mut second.samples {
            s.force_n += 1.0;
        }
        second.seal().unwrap();
        let mut request = SuspensionRunRequest {
            kind: "rne_suspension_run_request".into(),
            schema_version: 1,
            spec: suspension_identification_spec(),
            training: vec![SuspensionRunInput {
                acquisition_id: 1,
                dataset: first,
            }],
            holdout: vec![SuspensionRunInput {
                acquisition_id: 2,
                dataset: second,
            }],
        };
        let spec = SuspensionHuberSpec {
            delta_n: 10.0,
            maximum_iterations: 100,
            prediction_tolerance_n: 1e-7,
        };
        let baseline = evaluate_suspension_huber(&request, spec).unwrap();
        assert!(baseline.passed);
        let mut small = request.holdout[0].clone();
        small.acquisition_id = 3;
        small.dataset.dataset_id = "synthetic.small.failed.holdout".into();
        small.dataset.samples.truncate(4);
        for sample in &mut small.dataset.samples {
            sample.force_n += 40.0;
        }
        small.dataset.seal().unwrap();
        request.holdout.push(small);
        let mixed = evaluate_suspension_huber(&request, spec).unwrap();
        assert!(mixed.holdout_rmse_n < request.spec.maximum_holdout_rmse_n);
        assert!(mixed.holdout_runs[0].passed);
        assert!(!mixed.holdout_runs[1].passed);
        assert_eq!(mixed.holdout_runs[1].acquisition_id, 3);
        assert_eq!(mixed.holdout_runs[1].sample_count, 4);
        assert!(!mixed.passed);
        assert_eq!(mixed.iterate, baseline.iterate);
        request.holdout.pop();
        for s in &mut request.holdout[0].dataset.samples {
            s.force_n += 1000.0;
        }
        request.holdout[0].dataset.seal().unwrap();
        let changed = evaluate_suspension_huber(&request, spec).unwrap();
        assert_eq!(baseline.iterate, changed.iterate);
        assert_eq!(baseline.ordinary, changed.ordinary);
        assert!(changed.physical_bounds_passed);
        assert!(changed.holdout_rmse_n > 900.0);
        assert!(!changed.passed);
        let evidence = identify_suspension_huber(&request, spec).unwrap();
        let bytes = encode_suspension_huber(&evidence).unwrap();
        assert_eq!(decode_suspension_huber(&bytes).unwrap(), evidence);
        for mutation in 0..4 {
            let mut forged = evidence.clone();
            match mutation {
                0 => forged.evaluation.passed = true,
                1 => forged.evaluation.iterate.weights.clear(),
                2 => forged.evaluation.holdout_runs.clear(),
                _ => forged.schema_version = 2,
            }
            assert!(encode_suspension_huber(&forged).is_err());
            assert!(decode_suspension_huber(&serde_json::to_vec(&forged).unwrap()).is_err());
        }
        assert_eq!(changed, evaluate_suspension_huber(&request, spec).unwrap());
        for run in request.training.iter_mut().chain(&mut request.holdout) {
            for sample in &mut run.dataset.samples {
                sample.force_n = 200_000.0 * sample.position_m
                    + 15_000.0 * sample.velocity_m_s
                    + 12_000.0
                    + run.acquisition_id as f64;
            }
            run.dataset.seal().unwrap();
        }
        let nonphysical = evaluate_suspension_huber(&request, spec).unwrap();
        assert!(nonphysical.iterate.converged);
        assert!(nonphysical.holdout_rmse_n < request.spec.maximum_holdout_rmse_n);
        assert!(!nonphysical.physical_bounds_passed);
        assert!(!nonphysical.passed);
        assert_eq!(
            nonphysical.ordinary,
            Err(rne_robot::SuspensionIdentificationError::NonPhysicalResult)
        );
    }

    #[test]
    fn malformed_and_rank_deficient_training_is_rejected() {
        let spec = SuspensionHuberSpec {
            delta_n: 10.0,
            maximum_iterations: 100,
            prediction_tolerance_n: 1e-7,
        };
        let samples: Vec<_> = (0..10)
            .map(|i| {
                let t = i as f64;
                SuspensionForceSample {
                    capture_time_s: t,
                    position_m: t,
                    velocity_m_s: t * t,
                    force_n: -2.0 * t - t * t,
                }
            })
            .collect();
        fn run(samples: &[SuspensionForceSample]) -> SuspensionIdentificationRun<'_> {
            SuspensionIdentificationRun {
                acquisition_id: 1,
                samples,
            }
        }
        assert!(fit_suspension_huber_iterate(spec, &[run(&samples)]).is_ok());
        assert!(fit_suspension_huber_iterate(spec, &[]).is_err());
        assert!(fit_suspension_huber_iterate(spec, &[run(&samples), run(&samples)]).is_err());
        assert!(fit_suspension_huber_iterate(spec, &[run(&[])]).is_err());
        for delta_n in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert!(fit_suspension_huber_iterate(
                SuspensionHuberSpec { delta_n, ..spec },
                &[run(&samples)]
            )
            .is_err());
        }
        for maximum_iterations in [0, 257] {
            assert!(fit_suspension_huber_iterate(
                SuspensionHuberSpec {
                    maximum_iterations,
                    ..spec
                },
                &[run(&samples)]
            )
            .is_err());
        }
        let mut bad = samples.clone();
        bad[1].capture_time_s = bad[0].capture_time_s;
        assert!(fit_suspension_huber_iterate(spec, &[run(&bad)]).is_err());
        for invalid in [f64::NAN, f64::INFINITY, f64::MAX] {
            bad = samples.clone();
            bad[1].force_n = invalid;
            assert!(fit_suspension_huber_iterate(spec, &[run(&bad)]).is_err());
        }
        bad = samples.clone();
        for s in &mut bad {
            s.velocity_m_s = s.position_m;
        }
        assert!(fit_suspension_huber_iterate(spec, &[run(&bad)]).is_err());
    }

    #[test]
    fn huber_recovers_force_law_without_hiding_outlier_or_nonconvergence() {
        let mut samples: Vec<_> = (0..200)
            .map(|i| {
                let t = i as f64 * 0.01;
                let x = 0.02 * (3.0 * t).sin();
                let v = 0.06 * (3.0 * t).cos();
                SuspensionForceSample {
                    capture_time_s: t,
                    position_m: x,
                    velocity_m_s: v,
                    force_n: -200_000.0 * x - 15_000.0 * v - 12_000.0,
                }
            })
            .collect();
        let spec = SuspensionHuberSpec {
            delta_n: 10.0,
            maximum_iterations: 100,
            prediction_tolerance_n: 1e-7,
        };
        let fit = |samples: &[SuspensionForceSample], spec| {
            fit_suspension_huber_iterate(
                spec,
                &[SuspensionIdentificationRun {
                    acquisition_id: 1,
                    samples,
                }],
            )
            .unwrap()
        };
        let clean = fit(&samples, spec);
        assert!(clean.converged);
        assert!((clean.position_coefficient_n_per_m + 200_000.0).abs() < 1e-6);
        samples[70].force_n += 10_000_000.0;
        let ols = solve(&samples, &vec![1.0; samples.len()]).unwrap();
        assert!(
            ols[0] > 0.0,
            "contaminated OLS has negative spring stiffness"
        );
        let robust = fit(&samples, spec);
        assert!(robust.converged);
        assert!((robust.position_coefficient_n_per_m + 200_000.0).abs() < 100.0);
        assert!(robust.training_rmse_n > 1000.0);
        assert!(robust.weights[70] < 0.001);
        assert_eq!(robust, fit(&samples, spec));
        let limited = fit(
            &samples,
            SuspensionHuberSpec {
                maximum_iterations: 1,
                ..spec
            },
        );
        assert!(!limited.converged);
        assert_eq!(limited.iterations, 1);
        // A grossly wrong predictor has leverage: robust force residuals are not
        // a replacement for position calibration or errors-in-variables modeling.
        samples[70].position_m = 1_000_000.0;
        samples[70].force_n = 200_000.0 * samples[70].position_m;
        let leverage = fit(&samples, spec);
        assert!(leverage.position_coefficient_n_per_m > 0.0);
        assert!(leverage.training_rmse_n > 1000.0);
    }
}

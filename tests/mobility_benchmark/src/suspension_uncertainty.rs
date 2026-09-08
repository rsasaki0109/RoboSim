//! Bounded propagation of declared additive measurement errors through the actual estimator.
//!
//! This evaluates assumptions, not calibration authenticity or interval coverage.

use crate::suspension_acquisition::{SuspensionEvidenceFileRef, SuspensionSignalKind};
use crate::suspension_runs::SuspensionRunRequest;
use crate::suspension_runs::{SuspensionAcquiredRunRequest, MAX_SUSPENSION_RUN_BYTES};
use anyhow::{ensure, Result};
use rne_core::DeterministicRng;
use rne_robot::{
    fit_suspension_training_runs, SuspensionIdentificationError, SuspensionIdentificationRun,
    SuspensionTrainingCoefficients,
};
use rne_world::{RandomStreamId, WorldRandom};
use serde::{Deserialize, Serialize};

/// Complete replay input for file-bound additive uncertainty diagnostics.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionUncertaintyRequest {
    /// File-bound acquisition and error assumptions with their own version.
    pub errors: SuspensionAcquiredErrorRequest,
    /// Explicit coverage-factor interpretations for every training channel.
    pub interpretations: Vec<SuspensionCoverageInterpretation>,
}

/// Recomputable diagnostics, not a certificate of physical coverage.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionUncertaintyEvidence {
    /// Must be `rne_suspension_uncertainty_evidence`.
    pub kind: String,
    /// Independent evidence schema, currently 1.
    pub schema_version: u32,
    /// Exact acquisitions, calibration references, random seed and assumptions.
    pub request: SuspensionUncertaintyRequest,
    /// Every training-channel comparison, including mismatches.
    pub budget: Vec<SuspensionChannelBudgetComparison>,
    /// Every actual estimator outcome, including unsuccessful draws.
    pub propagation: SuspensionErrorPropagation,
}

impl SuspensionUncertaintyRequest {
    /// Checks files and evaluates both diagnostics without filtering mismatches or failures.
    pub fn evaluate(&self, root: &std::path::Path) -> Result<SuspensionUncertaintyEvidence> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "uncertainty request too large"
        );
        // The audit validates the entire error request and retained files first.
        let budget = self.errors.audit_budget(root, &self.interpretations)?;
        let propagation =
            propagate_suspension_errors(&self.errors.acquisitions.runs, &self.errors.model)?;
        let evidence = SuspensionUncertaintyEvidence {
            kind: "rne_suspension_uncertainty_evidence".into(),
            schema_version: 1,
            request: self.clone(),
            budget,
            propagation,
        };
        ensure!(
            serde_json::to_vec(&evidence)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "uncertainty evidence too large"
        );
        Ok(evidence)
    }
}

impl SuspensionUncertaintyEvidence {
    /// Rechecks files and recomputes all fields; stored outcomes are never trusted.
    pub fn verify(&self, root: &std::path::Path) -> Result<()> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "uncertainty evidence too large"
        );
        ensure!(
            *self == self.request.evaluate(root)?,
            "uncertainty evidence replay mismatch"
        );
        Ok(())
    }
}

/// Bounded strict evidence decoding followed by retained-file checks and actual refits.
pub fn decode_suspension_uncertainty(
    bytes: &[u8],
    root: &std::path::Path,
) -> Result<SuspensionUncertaintyEvidence> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "uncertainty evidence too large"
    );
    let evidence: SuspensionUncertaintyEvidence = serde_json::from_slice(bytes)?;
    evidence.verify(root)?;
    Ok(evidence)
}

/// Recomputes and verifies before emitting bounded compact JSON.
pub fn encode_suspension_uncertainty(
    evidence: &SuspensionUncertaintyEvidence,
    root: &std::path::Path,
) -> Result<Vec<u8>> {
    evidence.verify(root)?;
    Ok(serde_json::to_vec(evidence)?)
}

/// Calibration reference for one affected channel of one training acquisition.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionFactorCalibrationBinding {
    /// Error factor whose signed SI loading is justified by this declaration.
    pub factor_id: u64,
    /// Training acquisition identity; holdout is never an error-model fitting input.
    pub acquisition_id: u64,
    /// Channel with a nonzero loading for this factor.
    pub signal: SuspensionSignalKind,
    /// Exact calibration reference from the bound acquisition manifest.
    pub calibration_artifact: SuspensionEvidenceFileRef,
    /// Caller explanation of the standard-uncertainty loading, distribution and sharing.
    /// This is retained provenance, not a machine-verified certificate interpretation.
    pub interpretation: String,
}

/// Explicit interpretation of a retained expanded uncertainty; no default coverage factor.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionCoverageInterpretation {
    /// Training acquisition identity in request order.
    pub acquisition_id: u64,
    /// Channel identity, in canonical position/velocity/force order per acquisition.
    pub signal: SuspensionSignalKind,
    /// Explicit positive multiplier k in the retained declaration U = k*u.
    /// This does not imply any particular coverage probability or distribution.
    pub coverage_factor: f64,
    /// Caller-declared absolute tolerance in this channel's SI unit.
    pub absolute_tolerance_si: f64,
}

/// Marginal uncertainty budget comparison; mismatch is retained, not hidden.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionChannelBudgetComparison {
    /// Exact interpretation used to convert the manifest's expanded uncertainty.
    pub interpretation: SuspensionCoverageInterpretation,
    /// Declared standard uncertainty U/k, in the named channel's SI unit.
    pub declared_standard_uncertainty_si: f64,
    /// Root sum of squared independent factor loadings, in that same unit.
    pub modeled_standard_uncertainty_si: f64,
    /// Absolute discrepancy between modeled and declared standard uncertainty.
    pub absolute_difference_si: f64,
    /// Whether discrepancy is within the declared tolerance; not physical qualification.
    pub matched: bool,
}

/// File-bound additive error assumptions; neither a complete budget nor certification.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAcquiredErrorRequest {
    /// Must be `rne_suspension_acquired_error_request`.
    pub kind: String,
    /// Independent request schema, currently 1.
    pub schema_version: u32,
    /// Exact acquisitions and retained raw/procedure/calibration references.
    pub acquisitions: SuspensionAcquiredRunRequest,
    /// Explicit error assumptions; no automatic conversion from expanded uncertainty.
    pub model: SuspensionErrorModel,
    /// One binding per factor/nonzero channel/training acquisition, in that order.
    pub calibration: Vec<SuspensionFactorCalibrationBinding>,
}

impl SuspensionAcquiredErrorRequest {
    /// Validates exhaustive ordered bindings before any file reads or simulation work.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_suspension_acquired_error_request" && self.schema_version == 1,
            "acquired error request kind/schema drift"
        );
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "acquired error request too large"
        );
        self.acquisitions.validate()?;
        validate_error_model(&self.acquisitions.runs, &self.model)?;
        let mut bindings = self.calibration.iter();
        for factor in &self.model.factors {
            for (signal, loading) in [
                (SuspensionSignalKind::Position, factor.position_loading_m),
                (SuspensionSignalKind::Velocity, factor.velocity_loading_m_s),
                (SuspensionSignalKind::Force, factor.force_loading_n),
            ] {
                if loading == 0.0 {
                    continue;
                }
                for (run, manifest) in self
                    .acquisitions
                    .runs
                    .training
                    .iter()
                    .zip(&self.acquisitions.training)
                {
                    let binding = bindings
                        .next()
                        .ok_or_else(|| anyhow::anyhow!("missing factor calibration binding"))?;
                    let channel = manifest
                        .signals
                        .iter()
                        .find(|entry| entry.signal == signal)
                        .ok_or_else(|| anyhow::anyhow!("missing acquisition channel"))?;
                    ensure!(
                        binding.factor_id == factor.factor_id
                            && binding.acquisition_id == run.acquisition_id
                            && binding.signal == signal
                            && binding.calibration_artifact == channel.calibration_artifact,
                        "factor calibration binding mismatch"
                    );
                    ensure!(
                        !binding.interpretation.trim().is_empty()
                            && binding.interpretation.len() <= 1024
                            && !binding.interpretation.chars().any(char::is_control),
                        "invalid calibration interpretation"
                    );
                }
            }
        }
        ensure!(
            bindings.next().is_none(),
            "unexpected factor calibration binding"
        );
        Ok(())
    }

    /// Verifies all retained files before propagating; byte identity is not authenticity.
    pub fn propagate(&self, root: &std::path::Path) -> Result<SuspensionErrorPropagation> {
        self.validate()?;
        self.acquisitions.verify_files(root)?;
        propagate_suspension_errors(&self.acquisitions.runs, &self.model)
    }

    /// Compares pointwise marginal uncertainty budgets against retained declarations.
    ///
    /// Requires every training channel, including channels with zero modeled error.
    /// Shared errors are not divided by sample count. This does not validate k against
    /// certificate text, joint coverage, temporal covariance, or the completeness of
    /// the measurement model. All referenced acquisition files are verified on success.
    pub fn audit_budget(
        &self,
        root: &std::path::Path,
        interpretations: &[SuspensionCoverageInterpretation],
    ) -> Result<Vec<SuspensionChannelBudgetComparison>> {
        self.validate()?;
        ensure!(
            interpretations.len() == self.acquisitions.training.len() * 3,
            "uncertainty interpretation count mismatch"
        );
        let mut declarations = interpretations.iter();
        let mut comparisons = Vec::with_capacity(interpretations.len());
        for (run, manifest) in self
            .acquisitions
            .runs
            .training
            .iter()
            .zip(&self.acquisitions.training)
        {
            for channel in &manifest.signals {
                let declaration = declarations
                    .next()
                    .ok_or_else(|| anyhow::anyhow!("missing interpretation"))?;
                ensure!(
                    declaration.acquisition_id == run.acquisition_id
                        && declaration.signal == channel.signal,
                    "uncertainty interpretation identity/order mismatch"
                );
                ensure!(
                    declaration.coverage_factor.is_finite()
                        && declaration.coverage_factor > 0.0
                        && declaration.absolute_tolerance_si.is_finite()
                        && declaration.absolute_tolerance_si >= 0.0,
                    "invalid uncertainty interpretation"
                );
                let declared = channel.expanded_uncertainty_si / declaration.coverage_factor;
                let modeled = self.model.factors.iter().fold(0.0_f64, |total, factor| {
                    let loading = match channel.signal {
                        SuspensionSignalKind::Position => factor.position_loading_m,
                        SuspensionSignalKind::Velocity => factor.velocity_loading_m_s,
                        SuspensionSignalKind::Force => factor.force_loading_n,
                    };
                    total.hypot(loading)
                });
                ensure!(
                    declared.is_finite() && modeled.is_finite(),
                    "uncertainty budget overflow"
                );
                let difference = (declared - modeled).abs();
                comparisons.push(SuspensionChannelBudgetComparison {
                    interpretation: declaration.clone(),
                    declared_standard_uncertainty_si: declared,
                    modeled_standard_uncertainty_si: modeled,
                    absolute_difference_si: difference,
                    matched: difference <= declaration.absolute_tolerance_si,
                });
            }
        }
        self.acquisitions.verify_files(root)?;
        Ok(comparisons)
    }
}

/// Unit-variance, zero-mean latent distribution, explicitly chosen by the caller.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionErrorDistribution {
    /// Standard normal; SI loadings are standard deviations, not expanded uncertainties.
    Normal,
    /// Uniform on [-sqrt(3), sqrt(3)); SI loadings are standard deviations.
    Rectangular,
}

/// Which measurements share one realization of an error source.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionErrorScope {
    /// One value across all samples and all training acquisitions in this draw.
    SharedTraining,
    /// One independent value per acquisition, shared by its samples.
    Acquisition,
    /// Independent per sample; use only when justified by measurement evidence.
    Sample,
}

/// One independent latent source; signed loadings couple channels within that source.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionErrorFactor {
    /// Unique factor identity used for seeded stream derivation.
    pub factor_id: u64,
    /// Declared zero-mean, unit-variance distribution.
    pub distribution: SuspensionErrorDistribution,
    /// Temporal/acquisition sharing of this factor.
    pub scope: SuspensionErrorScope,
    /// Signed additive position loading in meters per unit latent value.
    pub position_loading_m: f64,
    /// Signed additive velocity loading in meters/second per unit latent value.
    pub velocity_loading_m_s: f64,
    /// Signed additive force loading in newtons per unit latent value.
    pub force_loading_n: f64,
}

/// Frozen assumptions and bounded Monte Carlo workload; no implicit U-to-sigma conversion.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionErrorModel {
    /// Must be `rne_suspension_additive_error_model`.
    pub kind: String,
    /// Current model and random-stream algorithm version, 1.
    pub schema_version: u32,
    /// Explicit root seed used to create WorldRandom.
    pub seed: u64,
    /// Number of draws, 1 through 4096; failed draws are not replaced.
    pub draws: usize,
    /// At most 16 independent factors, in strictly increasing factor-ID order.
    /// Cross-channel covariance is the sum of loading outer products.
    pub factors: Vec<SuspensionErrorFactor>,
}

/// All estimator outcomes, without a success-conditioned confidence interval.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionErrorPropagation {
    /// Unperturbed training fit, including failure.
    pub baseline: Result<SuspensionTrainingCoefficients, SuspensionIdentificationError>,
    /// One outcome per draw in draw order; failures remain in their original slots.
    pub draws: Vec<Result<SuspensionTrainingCoefficients, SuspensionIdentificationError>>,
}

pub(crate) fn latent(rng: &mut DeterministicRng, distribution: SuspensionErrorDistribution) -> f64 {
    match distribution {
        SuspensionErrorDistribution::Normal => {
            // 1-u is strictly positive since uniform_f64 excludes its upper bound.
            let radius = (-2.0 * (1.0 - rng.uniform_f64(0.0, 1.0)).ln()).sqrt();
            radius * (std::f64::consts::TAU * rng.uniform_f64(0.0, 1.0)).cos()
        }
        SuspensionErrorDistribution::Rectangular => rng.uniform_f64(-1.0, 1.0) * 3.0_f64.sqrt(),
    }
}

pub(crate) fn draw_rng(random: &WorldRandom, factor_id: u64, draw: usize) -> DeterministicRng {
    // Hierarchical derivation avoids symmetric XOR collisions between factor and draw IDs.
    let factor_random = WorldRandom::new(random.stream_seed(RandomStreamId::new(factor_id)));
    factor_random.stream(RandomStreamId::new(draw as u64 ^ 0x5355_5350_4552_5231))
}

/// Adds declared error realizations to training samples and executes the same estimator.
///
/// Holds capture timestamps and holdout samples fixed. Factors can encode correlated
/// additive errors, but not gain errors, clock errors, derivative/filter operators or
/// model discrepancy. Never claim this limited model is a complete uncertainty budget.
/// Input is bounded by the whole-run contract and 10 million sample-factor evaluations.
/// This function does not read or authenticate calibration files.
pub fn propagate_suspension_errors(
    request: &SuspensionRunRequest,
    model: &SuspensionErrorModel,
) -> Result<SuspensionErrorPropagation> {
    validate_error_model(request, model)?;
    propagate_validated_errors(request, model)
}

fn validate_error_model(
    request: &SuspensionRunRequest,
    model: &SuspensionErrorModel,
) -> Result<()> {
    request.validate()?;
    ensure!(
        model.kind == "rne_suspension_additive_error_model" && model.schema_version == 1,
        "error model kind/schema drift"
    );
    ensure!(
        (1..=4096).contains(&model.draws) && (1..=16).contains(&model.factors.len()),
        "invalid propagation workload"
    );
    ensure!(
        model
            .factors
            .windows(2)
            .all(|pair| pair[0].factor_id < pair[1].factor_id),
        "error factors must have unique ordered IDs"
    );
    ensure!(
        model.factors.iter().all(|factor| [
            factor.position_loading_m,
            factor.velocity_loading_m_s,
            factor.force_loading_n
        ]
        .into_iter()
        .all(f64::is_finite)),
        "nonfinite error loading"
    );
    let count: usize = request
        .training
        .iter()
        .map(|run| run.dataset.samples.len())
        .sum();
    let work = count
        .checked_mul(model.draws)
        .and_then(|n| n.checked_mul(model.factors.len()));
    ensure!(
        work.is_some_and(|n| n <= 10_000_000),
        "propagation workload too large"
    );
    Ok(())
}

fn propagate_validated_errors(
    request: &SuspensionRunRequest,
    model: &SuspensionErrorModel,
) -> Result<SuspensionErrorPropagation> {
    propagate_with_derivatives(request, model, None)
}

/// Verifies retained acquisitions and reconstructs training velocity in every draw.
/// Bindings must cover every training run in order. Additive velocity loadings
/// must be zero: velocity error comes exclusively from differentiating position.
/// Timestamps remain fixed; clock/gain/filter error propagation is not implemented.
/// Failures stay in their draw slots. This is not a complete uncertainty budget.
pub fn propagate_acquired_derived_errors(
    acquisitions: &SuspensionAcquiredRunRequest,
    model: &SuspensionErrorModel,
    bindings: &[crate::suspension_derivative::SuspensionDerivativeBinding],
    root: &std::path::Path,
) -> Result<SuspensionErrorPropagation> {
    validate_error_model(&acquisitions.runs, model)?;
    acquisitions.validate()?;
    ensure!(
        bindings.len() == acquisitions.training.len(),
        "derivative binding count mismatch"
    );
    ensure!(
        model
            .factors
            .iter()
            .all(|factor| factor.velocity_loading_m_s == 0.0),
        "derived velocity forbids independent velocity error loadings"
    );
    for ((binding, run), manifest) in bindings
        .iter()
        .zip(&acquisitions.runs.training)
        .zip(&acquisitions.training)
    {
        binding.validate(&run.dataset, manifest)?;
    }
    acquisitions.verify_files(root)?;
    let operators: Vec<_> = bindings.iter().map(|binding| binding.operator).collect();
    propagate_with_derivatives(&acquisitions.runs, model, Some(&operators))
}

fn reconstruct_runs(
    samples: &mut [Vec<rne_robot::SuspensionForceSample>],
    operators: &[crate::suspension_derivative::SuspensionDerivativeOperator],
) -> Result<(), SuspensionIdentificationError> {
    for (run, operator) in samples.iter_mut().zip(operators) {
        let times: Vec<_> = run.iter().map(|sample| sample.capture_time_s).collect();
        let positions: Vec<_> = run.iter().map(|sample| sample.position_m).collect();
        let velocities = operator
            .reconstruct(&times, &positions)
            .map_err(|_| SuspensionIdentificationError::InvalidSample)?;
        for (sample, velocity) in run.iter_mut().zip(velocities) {
            sample.velocity_m_s = velocity;
        }
    }
    Ok(())
}

fn propagate_with_derivatives(
    request: &SuspensionRunRequest,
    model: &SuspensionErrorModel,
    operators: Option<&[crate::suspension_derivative::SuspensionDerivativeOperator]>,
) -> Result<SuspensionErrorPropagation> {
    let mut nominal: Vec<_> = request
        .training
        .iter()
        .map(|run| run.dataset.samples.clone())
        .collect();
    if let Some(operators) = operators {
        ensure!(
            operators.len() == nominal.len(),
            "derivative operator count mismatch"
        );
        reconstruct_runs(&mut nominal, operators)?;
    }
    let training: Vec<_> = request
        .training
        .iter()
        .zip(&nominal)
        .map(|(run, samples)| SuspensionIdentificationRun {
            acquisition_id: run.acquisition_id,
            samples,
        })
        .collect();
    let baseline = fit_suspension_training_runs(request.spec, &training);
    let world_random = WorldRandom::new(model.seed);
    let mut outcomes = Vec::with_capacity(model.draws);
    for draw in 0..model.draws {
        let mut samples: Vec<_> = request
            .training
            .iter()
            .map(|run| run.dataset.samples.clone())
            .collect();
        for factor in &model.factors {
            let mut rng = draw_rng(&world_random, factor.factor_id, draw);
            let shared = latent(&mut rng, factor.distribution);
            for run in &mut samples {
                let acquisition = match factor.scope {
                    SuspensionErrorScope::SharedTraining => shared,
                    _ => latent(&mut rng, factor.distribution),
                };
                for sample in run {
                    let value = match factor.scope {
                        SuspensionErrorScope::Sample => latent(&mut rng, factor.distribution),
                        _ => acquisition,
                    };
                    sample.position_m += value * factor.position_loading_m;
                    sample.velocity_m_s += value * factor.velocity_loading_m_s;
                    sample.force_n += value * factor.force_loading_n;
                }
            }
        }
        if let Some(operators) = operators {
            if let Err(error) = reconstruct_runs(&mut samples, operators) {
                outcomes.push(Err(error));
                continue;
            }
        }
        let runs: Vec<_> = training
            .iter()
            .zip(&samples)
            .map(|(run, samples)| SuspensionIdentificationRun {
                acquisition_id: run.acquisition_id,
                samples,
            })
            .collect();
        outcomes.push(fit_suspension_training_runs(request.spec, &runs));
    }
    Ok(SuspensionErrorPropagation {
        baseline,
        draws: outcomes,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::suspension_identification::{
        suspension_identification_spec, synthetic_suspension_identification_dataset,
    };
    use crate::suspension_runs::SuspensionRunInput;

    fn fixture() -> (SuspensionRunRequest, SuspensionErrorModel) {
        let first = synthetic_suspension_identification_dataset().unwrap();
        let mut holdout = first.clone();
        holdout.dataset_id = "synthetic.uncertainty.holdout".into();
        holdout.samples[0].force_n += 1.0;
        holdout.seal().unwrap();
        (
            SuspensionRunRequest {
                kind: "rne_suspension_run_request".into(),
                schema_version: 1,
                spec: suspension_identification_spec(),
                training: vec![SuspensionRunInput {
                    acquisition_id: 1,
                    dataset: first,
                }],
                holdout: vec![SuspensionRunInput {
                    acquisition_id: 2,
                    dataset: holdout,
                }],
            },
            SuspensionErrorModel {
                kind: "rne_suspension_additive_error_model".into(),
                schema_version: 1,
                seed: 42,
                draws: 32,
                factors: vec![SuspensionErrorFactor {
                    factor_id: 1,
                    distribution: SuspensionErrorDistribution::Normal,
                    scope: SuspensionErrorScope::SharedTraining,
                    position_loading_m: 0.001,
                    velocity_loading_m_s: 0.0,
                    force_loading_n: 100.0,
                }],
            },
        )
    }

    #[test]
    fn timestamp_draws_refit_and_keep_failed_clock_realizations() {
        use crate::suspension_derivative::SuspensionDerivativeOperator;
        use crate::suspension_sampling::{
            SuspensionTimingErrorModel, SuspensionTimingFactor, SuspensionTimingScope,
        };
        let (mut request, _) = fixture();
        let operators = [SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1];
        let mut model = SuspensionTimingErrorModel {
            kind: "rne_suspension_timing_error_model".into(),
            schema_version: 1,
            seed: 42,
            factors: vec![SuspensionTimingFactor {
                factor_id: 7,
                distribution: SuspensionErrorDistribution::Normal,
                scope: SuspensionTimingScope::IndependentSamples,
                physical_loading_s: 0.0,
                reported_loading_s: 0.0001,
            }],
        };
        let result = model.propagate_labels(&request, &operators, 8).unwrap();
        assert!(result.baseline.is_ok());
        assert_eq!(
            result,
            model.propagate_labels(&request, &operators, 8).unwrap()
        );
        assert!(result.draws.iter().any(|draw| *draw != result.baseline));
        assert_eq!(
            &model
                .propagate_labels(&request, &operators, 16)
                .unwrap()
                .draws[..8],
            &result.draws
        );
        request.holdout[0].dataset.samples[0].force_n += 100.0;
        request.holdout[0].dataset.seal().unwrap();
        assert_eq!(
            result,
            model.propagate_labels(&request, &operators, 8).unwrap()
        );
        model.factors[0].reported_loading_s = 10.0;
        let failed = model.propagate_labels(&request, &operators, 8).unwrap();
        assert_eq!(
            failed.draws,
            vec![Err(SuspensionIdentificationError::InvalidSample); 8]
        );
        model.factors[0].reported_loading_s = 0.0;
        let zero = model.propagate_labels(&request, &operators, 8).unwrap();
        assert!(zero.draws.iter().all(|draw| *draw == zero.baseline));
        model.factors[0].physical_loading_s = 1e-9;
        assert!(model.propagate_labels(&request, &operators, 8).is_err());
        model.factors[0].physical_loading_s = 0.0;
        assert!(model.propagate_labels(&request, &[], 8).is_err());
        assert!(model.propagate_labels(&request, &operators, 0).is_err());
        assert!(model.propagate_labels(&request, &operators, 4097).is_err());
    }

    #[test]
    fn affine_draws_replay_scale_damping_and_preserve_failed_slots() {
        use crate::suspension_derivative::{
            SuspensionAffineCorrection, SuspensionAffineErrorModel, SuspensionAffineFactor,
            SuspensionDerivativeOperator,
        };
        let (mut request, _) = fixture();
        let op = SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1;
        let nominal = SuspensionAffineCorrection {
            time_reference_s: 0.0,
            time_scale: 1.0,
            time_offset_s: 0.0,
            position_scale: 1.0,
            position_offset_m: 0.0,
            force_scale: 1.0,
            force_offset_n: 0.0,
        };
        let mut model = SuspensionAffineErrorModel {
            kind: "rne_suspension_affine_error_model".into(),
            schema_version: 1,
            seed: 42,
            draws: 16,
            nominal: vec![(nominal, op)],
            factors: vec![SuspensionAffineFactor {
                factor_id: 7,
                distribution: SuspensionErrorDistribution::Rectangular,
                scope: SuspensionErrorScope::SharedTraining,
                time_scale_loading: 0.01,
                time_offset_loading_s: 0.0,
                position_scale_loading: 0.0,
                position_offset_loading_m: 0.0,
                force_scale_loading: 0.0,
                force_offset_loading_n: 0.0,
            }],
        };
        let result = model.propagate(&request).unwrap();
        assert_eq!(result, model.propagate(&request).unwrap());
        let base = result.baseline.unwrap();
        // All six signed loadings share one latent value. An independent-channel
        // implementation would violate these per-draw coefficient identities.
        let mut joint = model.clone();
        joint.factors[0].time_offset_loading_s = 0.02;
        joint.factors[0].position_scale_loading = -0.015;
        joint.factors[0].position_offset_loading_m = 0.001;
        joint.factors[0].force_scale_loading = 0.025;
        joint.factors[0].force_offset_loading_n = -20.0;
        for distribution in [
            SuspensionErrorDistribution::Normal,
            SuspensionErrorDistribution::Rectangular,
        ] {
            joint.factors[0].distribution = distribution;
            let propagated = joint.propagate(&request).unwrap();
            assert_eq!(propagated, joint.propagate(&request).unwrap());
            for (i, fitted) in propagated.draws.iter().enumerate() {
                let mut rng = draw_rng(&WorldRandom::new(42), 7, i);
                let z = latent(&mut rng, distribution);
                let at = 1.0 + 0.01 * z;
                let ax = 1.0 - 0.015 * z;
                let af = 1.0 + 0.025 * z;
                let expected_k = af / ax * base.stiffness_n_per_m;
                let expected_c = af * at / ax * base.damping_n_s_per_m;
                let expected_e =
                    ax * base.equilibrium_position_m + 0.001 * z - 20.0 * z / expected_k;
                let fitted = fitted.as_ref().unwrap();
                assert!((fitted.stiffness_n_per_m - expected_k).abs() < 1e-5);
                assert!((fitted.damping_n_s_per_m - expected_c).abs() < 1e-6);
                assert!((fitted.equilibrium_position_m - expected_e).abs() < 1e-12);
            }
        }
        for invalid in [f64::NAN, f64::INFINITY] {
            joint.factors[0].force_scale_loading = invalid;
            assert!(joint.propagate(&request).is_err());
        }
        joint = model.clone();
        joint.schema_version = 2;
        assert!(joint.propagate(&request).is_err());
        joint = model.clone();
        joint.draws = 4097;
        assert!(joint.propagate(&request).is_err());
        joint = model.clone();
        joint.nominal.clear();
        assert!(joint.propagate(&request).is_err());
        for (i, fitted) in result.draws.iter().enumerate() {
            let mut rng = draw_rng(&WorldRandom::new(42), 7, i);
            let scale = 1.0 + 0.01 * latent(&mut rng, SuspensionErrorDistribution::Rectangular);
            let fitted = fitted.as_ref().unwrap();
            assert!((fitted.stiffness_n_per_m - base.stiffness_n_per_m).abs() < 1e-5);
            assert!((fitted.damping_n_s_per_m - scale * base.damping_n_s_per_m).abs() < 1e-6);
        }
        model.draws = 32;
        assert_eq!(
            &model.propagate(&request).unwrap().draws[..16],
            &result.draws
        );
        request.holdout[0].dataset.samples[0].force_n += 100.0;
        request.holdout[0].dataset.seal().unwrap();
        model.draws = 16;
        assert_eq!(result, model.propagate(&request).unwrap());
        model.factors[0].time_scale_loading = 100.0;
        let failed = model.propagate(&request).unwrap();
        assert_eq!(failed.draws.len(), 16);
        assert!(failed
            .draws
            .contains(&Err(SuspensionIdentificationError::InvalidSample)));
        assert!(failed
            .draws
            .contains(&Err(SuspensionIdentificationError::NonPhysicalResult)));
        assert_eq!(failed, model.propagate(&request).unwrap());
        model.factors[0].scope = SuspensionErrorScope::Sample;
        assert!(model.propagate(&request).is_err());
        model.factors[0].scope = SuspensionErrorScope::Acquisition;
        model.factors[0].time_scale_loading = 0.01;
        assert_ne!(result.draws, model.propagate(&request).unwrap().draws);
        model.factors.push(model.factors[0].clone());
        assert!(model.propagate(&request).is_err());
        model.factors.pop();
        let mut second = request.training[0].clone();
        second.acquisition_id = 3;
        second.dataset.dataset_id = "synthetic.affine.second".into();
        for sample in &mut second.dataset.samples {
            sample.capture_time_s += 1.0;
        }
        second.dataset.seal().unwrap();
        request.training.push(second);
        model.nominal.push((nominal, op));
        let acquisition = model.propagate(&request).unwrap();
        assert!(acquisition.draws.iter().all(Result::is_ok));
        model.factors[0].scope = SuspensionErrorScope::SharedTraining;
        let shared = model.propagate(&request).unwrap();
        assert_ne!(acquisition.draws, shared.draws);
        assert_eq!(shared, model.propagate(&request).unwrap());
        model.seed += 1;
        assert_ne!(shared.draws, model.propagate(&request).unwrap().draws);
        model.factors[0].time_scale_loading = 0.0;
        let zero = model.propagate(&request).unwrap();
        assert!(zero.draws.iter().all(|draw| *draw == zero.baseline));
        let encoded = serde_json::to_vec(&model).unwrap();
        assert_eq!(
            model,
            serde_json::from_slice::<SuspensionAffineErrorModel>(&encoded).unwrap()
        );
        model.nominal[0].0.time_scale = 0.0;
        assert!(model.propagate(&request).is_err());
    }

    #[test]
    fn derivative_draws_recompute_velocity_and_retain_failures() {
        use crate::suspension_derivative::SuspensionDerivativeOperator;
        let (mut request, mut model) = fixture();
        let operators = [SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1];
        let mut nominal = vec![request.training[0].dataset.samples.clone()];
        reconstruct_runs(&mut nominal, &operators).unwrap();
        request.training[0].dataset.samples = nominal.remove(0);
        request.training[0].dataset.seal().unwrap();
        model.factors[0].force_loading_n = 0.0;
        model.factors[0].scope = SuspensionErrorScope::Sample;
        let propagated = propagate_with_derivatives(&request, &model, Some(&operators)).unwrap();
        assert_eq!(
            propagated,
            propagate_with_derivatives(&request, &model, Some(&operators)).unwrap()
        );
        let independent = propagate_suspension_errors(&request, &model).unwrap();
        assert_eq!(propagated.baseline, independent.baseline);
        assert_ne!(propagated.draws, independent.draws);
        for sample in &mut request.holdout[0].dataset.samples {
            sample.force_n += 1000.0;
        }
        request.holdout[0].dataset.seal().unwrap();
        assert_eq!(
            propagated,
            propagate_with_derivatives(&request, &model, Some(&operators)).unwrap()
        );
        model.factors[0].position_loading_m = f64::MAX;
        let failed = propagate_with_derivatives(&request, &model, Some(&operators)).unwrap();
        assert_eq!(failed.draws.len(), model.draws);
        assert!(failed
            .draws
            .contains(&Err(SuspensionIdentificationError::InvalidSample)));
        assert!(propagate_with_derivatives(&request, &model, Some(&[])).is_err());
    }

    #[test]
    fn seeded_common_error_is_reproducible_and_holdout_independent() {
        let (mut request, model) = fixture();
        let result = propagate_suspension_errors(&request, &model).unwrap();
        assert_eq!(
            result,
            propagate_suspension_errors(&request, &model).unwrap()
        );
        let baseline = result.baseline.unwrap();
        assert!(result.draws.iter().all(|fit| fit.is_ok()));
        assert!(result
            .draws
            .iter()
            .any(
                |fit| (fit.unwrap().equilibrium_position_m - baseline.equilibrium_position_m).abs()
                    > 0.0001
            ));
        for fit in &result.draws {
            assert!((fit.unwrap().stiffness_n_per_m - baseline.stiffness_n_per_m).abs() < 1e-6);
        }
        request.holdout[0].dataset.samples[0].force_n += 10_000.0;
        request.holdout[0].dataset.seal().unwrap();
        assert_eq!(
            result,
            propagate_suspension_errors(&request, &model).unwrap()
        );
        let mut changed = model.clone();
        changed.seed += 1;
        assert_ne!(
            result.draws,
            propagate_suspension_errors(&request, &changed)
                .unwrap()
                .draws
        );
    }

    #[test]
    fn factor_and_draw_streams_have_no_symmetric_xor_alias() {
        let random = WorldRandom::new(42);
        let domain = 0x5355_5350_4552_5231;
        // Mixing each key then XORing would give the same stream for these pairs.
        let a = draw_rng(&random, domain ^ 1, 0).next_u64();
        let b = draw_rng(&random, domain, 1).next_u64();
        assert_ne!(a, b);
        assert_eq!(a, draw_rng(&random, domain ^ 1, 0).next_u64());
    }

    #[test]
    fn draw_prefix_is_stable_and_invalid_models_are_rejected() {
        let (request, mut model) = fixture();
        for scope in [
            SuspensionErrorScope::SharedTraining,
            SuspensionErrorScope::Acquisition,
            SuspensionErrorScope::Sample,
        ] {
            model.factors[0].scope = scope;
            model.draws = 8;
            let short = propagate_suspension_errors(&request, &model).unwrap();
            model.draws = 16;
            let long = propagate_suspension_errors(&request, &model).unwrap();
            assert_eq!(short.draws, long.draws[..8]);
        }
        model.schema_version = 2;
        assert!(propagate_suspension_errors(&request, &model).is_err());
        model.schema_version = 1;
        model.draws = 0;
        assert!(propagate_suspension_errors(&request, &model).is_err());
        model.draws = 4096;
        let factor = model.factors[0].clone();
        model.factors = (0..16)
            .map(|factor_id| SuspensionErrorFactor {
                factor_id,
                ..factor.clone()
            })
            .collect();
        assert!(propagate_suspension_errors(&request, &model)
            .unwrap_err()
            .to_string()
            .contains("workload too large"));
        model.draws = 1;
        model.factors[1].factor_id = model.factors[0].factor_id;
        assert!(propagate_suspension_errors(&request, &model).is_err());
        let mut json = serde_json::to_value(&model).unwrap();
        json["coverage_probability"] = serde_json::json!(0.95);
        assert!(serde_json::from_value::<SuspensionErrorModel>(json).is_err());
    }

    #[test]
    fn failed_draws_are_not_replaced_and_work_is_bounded() {
        let (request, mut model) = fixture();
        model.factors[0].distribution = SuspensionErrorDistribution::Rectangular;
        model.factors[0].force_loading_n = 1e8;
        let result = propagate_suspension_errors(&request, &model).unwrap();
        assert_eq!(result.draws.len(), model.draws);
        assert!(result.draws.iter().any(|fit| fit.is_err()));
        model.draws = 4097;
        assert!(propagate_suspension_errors(&request, &model).is_err());
        model.draws = 32;
        model.factors[0].position_loading_m = f64::NAN;
        assert!(propagate_suspension_errors(&request, &model).is_err());
    }

    #[test]
    fn shared_factor_survives_repeated_capture_and_signed_channels_can_cancel() {
        let (mut request, mut model) = fixture();
        let original = propagate_suspension_errors(&request, &model).unwrap();
        let mut repeated = request.training[0].clone();
        repeated.acquisition_id = 3;
        repeated.dataset.dataset_id = "synthetic.repeated.not.independent".into();
        for sample in &mut repeated.dataset.samples {
            sample.capture_time_s += 100.0;
        }
        repeated.dataset.seal().unwrap();
        request.training.push(repeated);
        let doubled = propagate_suspension_errors(&request, &model).unwrap();
        for (a, b) in original.draws.iter().zip(&doubled.draws) {
            assert!(
                (a.unwrap().equilibrium_position_m - b.unwrap().equilibrium_position_m).abs()
                    < 1e-10
            );
        }
        let baseline = doubled.baseline.unwrap();
        model.factors[0].force_loading_n =
            -baseline.stiffness_n_per_m * model.factors[0].position_loading_m;
        let cancelled = propagate_suspension_errors(&request, &model).unwrap();
        for fit in cancelled.draws {
            assert!(
                (fit.unwrap().equilibrium_position_m - baseline.equilibrium_position_m).abs()
                    < 1e-10
            );
        }
        // Changing sharing is a different model, not a reordering optimization.
        model.factors[0].scope = SuspensionErrorScope::Acquisition;
        let per_run = propagate_suspension_errors(&request, &model).unwrap();
        model.factors[0].scope = SuspensionErrorScope::Sample;
        let per_sample = propagate_suspension_errors(&request, &model).unwrap();
        assert_ne!(per_run.draws, per_sample.draws);
    }
}

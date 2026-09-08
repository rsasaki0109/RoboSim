//! File-bound interpretations for affine suspension corrections and error factors.

use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};

use crate::suspension_acquisition::SuspensionEvidenceFileRef;
use crate::suspension_derivative::{SuspensionAffineErrorModel, SuspensionDerivativeBinding};
use crate::suspension_runs::{SuspensionAcquiredRunRequest, MAX_SUSPENSION_RUN_BYTES};
use crate::suspension_uncertainty::SuspensionErrorPropagation;

/// Calibration domain; timebase scale is not inter-channel synchronization error.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionAffineDomain {
    /// Explicit caller-declared installed acquisition clock.
    Timebase,
    /// Position channel from the acquisition manifest.
    Position,
    /// Force channel from the acquisition manifest.
    Force,
}

/// One explicit interpretation of a retained calibration document.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAffineCalibrationBinding {
    /// None binds the nominal correction; Some binds that exact random factor.
    pub factor_id: Option<u64>,
    /// Exact training acquisition identity.
    pub acquisition_id: u64,
    /// Exact capture identity from the manifest.
    pub capture_id: String,
    /// Correction domain; both scale and offset are covered by the interpretation.
    pub domain: SuspensionAffineDomain,
    /// Manifest sensor ID, or caller-declared installed clock ID for timebase.
    pub instrument_id: String,
    /// Retained calibration bytes; timebase references are independent of sync bounds.
    pub calibration_artifact: SuspensionEvidenceFileRef,
    /// Explicit explanation of nominal correction or factor loadings and sharing.
    pub interpretation: String,
}

/// Complete inputs for file-verified affine propagation, not a physical certificate.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAcquiredAffineRequest {
    /// Must be `rne_suspension_acquired_affine_request`.
    pub kind: String,
    /// Currently 1, independent of additive request schemas.
    pub schema_version: u32,
    /// Complete training and holdout source declarations.
    pub acquisitions: SuspensionAcquiredRunRequest,
    /// Seeded corrections and explicit numerical derivative operators.
    pub model: SuspensionAffineErrorModel,
    /// Nominal velocity procedure per training acquisition, in order.
    pub derivatives: Vec<SuspensionDerivativeBinding>,
    /// Nominal first, then factors in model order; domain timebase/position/force,
    /// then acquisition order. Nominal requires all domains, including identity.
    /// Factors require every domain with a nonzero scale or offset loading.
    pub calibration: Vec<SuspensionAffineCalibrationBinding>,
}

impl SuspensionAcquiredAffineRequest {
    /// Verifies files and retains the exact request plus every fitted outcome.
    pub fn evaluate(&self, root: &std::path::Path) -> Result<SuspensionAffineEvidence> {
        let propagation = self.propagate(root)?;
        let evidence = SuspensionAffineEvidence {
            kind: "rne_suspension_affine_evidence".into(),
            schema_version: 1,
            request: self.clone(),
            propagation,
        };
        ensure!(
            serde_json::to_vec(&evidence)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "affine evidence too large"
        );
        Ok(evidence)
    }

    /// Verifies exhaustive bindings and all source/calibration bytes before fitting.
    /// The clock identity and certificate interpretation remain caller assertions;
    /// this cannot establish traceability, device installation or coverage.
    pub fn propagate(&self, root: &std::path::Path) -> Result<SuspensionErrorPropagation> {
        use SuspensionAffineDomain::{Force, Position, Timebase};
        ensure!(
            self.kind == "rne_suspension_acquired_affine_request" && self.schema_version == 1,
            "acquired affine kind/schema drift"
        );
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "acquired affine request too large"
        );
        self.acquisitions.validate()?;
        let count = self.acquisitions.training.len();
        ensure!(
            self.derivatives.len() == count && self.model.nominal.len() == count,
            "acquired affine derivative count mismatch"
        );
        for ((run, manifest), (derivative, (_, operator))) in self
            .acquisitions
            .runs
            .training
            .iter()
            .zip(&self.acquisitions.training)
            .zip(self.derivatives.iter().zip(&self.model.nominal))
        {
            ensure!(
                derivative.operator == *operator,
                "affine derivative operator mismatch"
            );
            derivative.validate(&run.dataset, manifest)?;
        }
        let mut expected = vec![(None, [true; 3])];
        for factor in &self.model.factors {
            expected.push((
                Some(factor.factor_id),
                [
                    factor.time_scale_loading != 0.0 || factor.time_offset_loading_s != 0.0,
                    factor.position_scale_loading != 0.0 || factor.position_offset_loading_m != 0.0,
                    factor.force_scale_loading != 0.0 || factor.force_offset_loading_n != 0.0,
                ],
            ));
        }
        let mut bindings = self.calibration.iter();
        let mut clocks = Vec::with_capacity(count);
        for (factor_id, required) in expected {
            for (domain, required) in [Timebase, Position, Force].into_iter().zip(required) {
                if !required {
                    continue;
                }
                for (index, (run, manifest)) in self
                    .acquisitions
                    .runs
                    .training
                    .iter()
                    .zip(&self.acquisitions.training)
                    .enumerate()
                {
                    let binding = bindings
                        .next()
                        .ok_or_else(|| anyhow::anyhow!("missing affine calibration binding"))?;
                    ensure!(
                        binding.factor_id == factor_id
                            && binding.domain == domain
                            && binding.acquisition_id == run.acquisition_id
                            && binding.capture_id == manifest.capture_id,
                        "affine calibration identity mismatch"
                    );
                    ensure!(
                        !binding.instrument_id.trim().is_empty()
                            && binding.instrument_id.len() <= 256
                            && !binding.instrument_id.chars().any(char::is_control),
                        "invalid affine instrument ID"
                    );
                    ensure!(
                        !binding.interpretation.trim().is_empty()
                            && binding.interpretation.len() <= 1024
                            && !binding.interpretation.chars().any(char::is_control),
                        "invalid affine interpretation"
                    );
                    binding.calibration_artifact.validate()?;
                    if domain == Timebase {
                        if factor_id.is_none() {
                            clocks.push(binding);
                        } else {
                            ensure!(
                                binding.instrument_id == clocks[index].instrument_id
                                    && binding.calibration_artifact
                                        == clocks[index].calibration_artifact,
                                "affine clock calibration mismatch"
                            );
                        }
                    } else {
                        let signal = &manifest.signals[if domain == Position { 0 } else { 2 }];
                        ensure!(
                            binding.instrument_id == signal.sensor_id
                                && binding.calibration_artifact == signal.calibration_artifact,
                            "affine channel calibration mismatch"
                        );
                    }
                }
            }
        }
        ensure!(
            bindings.next().is_none(),
            "unexpected affine calibration binding"
        );
        self.acquisitions.verify_files(root)?;
        // Position and force references are already verified through the manifest.
        for clock in clocks {
            clock.calibration_artifact.verify(root)?;
        }
        self.model.propagate(&self.acquisitions.runs)
    }
}

/// Replayable affine diagnostics; neither calibration authentication nor coverage.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionAffineEvidence {
    /// Must be `rne_suspension_affine_evidence`.
    pub kind: String,
    /// Currently 1.
    pub schema_version: u32,
    /// Exact file-bound acquisition, correction, factor and interpretation inputs.
    pub request: SuspensionAcquiredAffineRequest,
    /// Baseline and all draw outcomes, retaining every failure.
    pub propagation: SuspensionErrorPropagation,
}

impl SuspensionAffineEvidence {
    /// Reopens all retained files and reruns propagation; compares every field.
    pub fn verify(&self, root: &std::path::Path) -> Result<()> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "affine evidence too large"
        );
        ensure!(
            *self == self.request.evaluate(root)?,
            "affine evidence replay mismatch"
        );
        Ok(())
    }
}

/// Strict bounded decoding followed by file verification and complete recomputation.
pub fn decode_suspension_affine(
    bytes: &[u8],
    root: &std::path::Path,
) -> Result<SuspensionAffineEvidence> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "affine evidence too large"
    );
    let evidence: SuspensionAffineEvidence = serde_json::from_slice(bytes)?;
    evidence.verify(root)?;
    Ok(evidence)
}

/// Serializes only after file verification and complete recomputation.
pub fn encode_suspension_affine(
    evidence: &SuspensionAffineEvidence,
    root: &std::path::Path,
) -> Result<Vec<u8>> {
    evidence.verify(root)?;
    Ok(serde_json::to_vec(evidence)?)
}

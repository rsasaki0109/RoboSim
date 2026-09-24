//! File-bound timestamp-label uncertainty; physical sampling errors are excluded.

use crate::suspension_affine_acquisition::{
    SuspensionAffineCalibrationBinding, SuspensionAffineDomain,
};
use crate::suspension_derivative::SuspensionDerivativeBinding;
use crate::suspension_runs::{SuspensionAcquiredRunRequest, MAX_SUSPENSION_RUN_BYTES};
use crate::suspension_sampling::SuspensionTimingErrorModel;
use crate::suspension_uncertainty::SuspensionErrorPropagation;
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};

/// Exact inputs and exhaustive retained clock interpretations for timestamp errors.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionTimestampRequest {
    /// Complete acquisition manifests and whole-run split.
    pub acquisitions: SuspensionAcquiredRunRequest,
    /// Explicit seeded timing assumptions; physical-time loadings must be zero.
    pub model: SuspensionTimingErrorModel,
    /// Number of realizations; bounded by the numerical propagation contract.
    pub draws: usize,
    /// Executable nominal velocity procedure per training acquisition, in order.
    pub derivatives: Vec<SuspensionDerivativeBinding>,
    /// Timebase-only bindings: nominal (`factor_id` None) per acquisition, followed
    /// by each nonzero reported-time factor per acquisition, in model/run order.
    /// Nominal binds the installed clock and interpretation of unchanged labels.
    pub clocks: Vec<SuspensionAffineCalibrationBinding>,
}

impl SuspensionTimestampRequest {
    /// Verifies identities, derivative procedures and retained bytes, then refits.
    /// Installed-clock identity and certificate interpretation remain assertions;
    /// hashing does not authenticate a certificate or establish physical coverage.
    pub fn evaluate(&self, root: &std::path::Path) -> Result<SuspensionTimestampEvidence> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "timestamp request too large"
        );
        self.acquisitions.validate()?;
        let count = self.acquisitions.training.len();
        ensure!(
            self.derivatives.len() == count,
            "timestamp derivative count mismatch"
        );
        ensure!(
            self.model
                .factors
                .iter()
                .all(|factor| factor.physical_loading_s == 0.0),
            "physical timing errors require a declared signal"
        );
        let mut operators = Vec::with_capacity(count);
        for ((run, manifest), derivative) in self
            .acquisitions
            .runs
            .training
            .iter()
            .zip(&self.acquisitions.training)
            .zip(&self.derivatives)
        {
            derivative.validate(&run.dataset, manifest)?;
            operators.push(derivative.operator);
        }
        let required = std::iter::once(None).chain(
            self.model
                .factors
                .iter()
                .filter(|factor| factor.reported_loading_s != 0.0)
                .map(|factor| Some(factor.factor_id)),
        );
        let mut bindings = self.clocks.iter();
        let mut nominal = Vec::with_capacity(count);
        for factor_id in required {
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
                    .ok_or_else(|| anyhow::anyhow!("missing timestamp clock binding"))?;
                ensure!(
                    binding.domain == SuspensionAffineDomain::Timebase
                        && binding.factor_id == factor_id
                        && binding.acquisition_id == run.acquisition_id
                        && binding.capture_id == manifest.capture_id,
                    "timestamp clock binding mismatch"
                );
                ensure!(
                    !binding.instrument_id.trim().is_empty()
                        && binding.instrument_id.len() <= 256
                        && !binding.instrument_id.chars().any(char::is_control),
                    "invalid timestamp clock ID"
                );
                ensure!(
                    !binding.interpretation.trim().is_empty()
                        && binding.interpretation.len() <= 1024
                        && !binding.interpretation.chars().any(char::is_control),
                    "invalid timestamp interpretation"
                );
                binding.calibration_artifact.validate()?;
                if factor_id.is_none() {
                    nominal.push(binding);
                } else {
                    ensure!(
                        binding.instrument_id == nominal[index].instrument_id
                            && binding.calibration_artifact == nominal[index].calibration_artifact,
                        "timestamp factor clock mismatch"
                    );
                }
            }
        }
        ensure!(
            bindings.next().is_none(),
            "unexpected timestamp clock binding"
        );
        self.acquisitions.verify_files(root)?;
        for clock in nominal {
            clock.calibration_artifact.verify(root)?;
        }
        let propagation =
            self.model
                .propagate_labels(&self.acquisitions.runs, &operators, self.draws)?;
        let evidence = SuspensionTimestampEvidence {
            kind: "rne_suspension_timestamp_evidence".into(),
            schema_version: 1,
            request: self.clone(),
            propagation,
        };
        ensure!(
            serde_json::to_vec(&evidence)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "timestamp evidence too large"
        );
        Ok(evidence)
    }
}

/// Replayable timestamp diagnostics with all failed realizations retained.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionTimestampEvidence {
    /// Must be `rne_suspension_timestamp_evidence`.
    pub kind: String,
    /// Currently 1.
    pub schema_version: u32,
    /// Exact file-bound and seeded input assumptions.
    pub request: SuspensionTimestampRequest,
    /// Baseline and every original draw outcome.
    pub propagation: SuspensionErrorPropagation,
}

impl SuspensionTimestampEvidence {
    /// Checks retained files and recomputes all outcomes before comparing every field.
    pub fn verify(&self, root: &std::path::Path) -> Result<()> {
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "timestamp evidence too large"
        );
        ensure!(
            *self == self.request.evaluate(root)?,
            "timestamp evidence replay mismatch"
        );
        Ok(())
    }
}

/// Strict bounded decode, file verification and complete numerical replay.
pub fn decode_suspension_timestamp(
    bytes: &[u8],
    root: &std::path::Path,
) -> Result<SuspensionTimestampEvidence> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "timestamp evidence too large"
    );
    let evidence: SuspensionTimestampEvidence = serde_json::from_slice(bytes)?;
    evidence.verify(root)?;
    Ok(evidence)
}

/// Validates retained bytes and numerical outcomes before compact serialization.
pub fn encode_suspension_timestamp(
    evidence: &SuspensionTimestampEvidence,
    root: &std::path::Path,
) -> Result<Vec<u8>> {
    evidence.verify(root)?;
    Ok(serde_json::to_vec(evidence)?)
}

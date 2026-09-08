//! Bounded, replay-verified whole-acquisition suspension identification.

use crate::suspension_identification::SuspensionIdentificationDataset;
use anyhow::{ensure, Result};
use rne_robot::{
    identify_suspension_strut_runs_report, SuspensionIdentificationRun,
    SuspensionIdentificationSpec, SuspensionRunIdentificationReport,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Whole-run evidence with training-only excitation diagnostics; not uncertainty.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionExcitationEvidence {
    /// Must be `rne_suspension_excitation_evidence`.
    pub kind: String,
    /// Independent envelope schema, currently 1.
    pub schema_version: u32,
    /// Unchanged v1 fit, run split and residual verdicts.
    pub run_evidence: SuspensionRunEvidence,
    /// Recomputed solely from embedded training acquisitions.
    pub training_excitation: rne_robot::systems::SuspensionExcitationDiagnostics,
}

/// Fits a checked request and retains excitation without changing residual gates.
pub fn identify_suspension_excitation(
    request: &SuspensionRunRequest,
) -> Result<SuspensionExcitationEvidence> {
    let run_evidence = identify_suspension_runs(request)?;
    let training: Vec<_> = request
        .training
        .iter()
        .map(|run| SuspensionIdentificationRun {
            acquisition_id: run.acquisition_id,
            samples: &run.dataset.samples,
        })
        .collect();
    let training_excitation = rne_robot::systems::suspension_training_excitation(&training)?;
    let evidence = SuspensionExcitationEvidence {
        kind: "rne_suspension_excitation_evidence".into(),
        schema_version: 1,
        run_evidence,
        training_excitation,
    };
    ensure!(
        serde_json::to_vec(&evidence)?.len() <= MAX_SUSPENSION_RUN_BYTES,
        "excitation evidence too large"
    );
    Ok(evidence)
}

/// Reexecutes both fit and excitation, then emits bounded compact JSON.
pub fn encode_suspension_excitation(evidence: &SuspensionExcitationEvidence) -> Result<Vec<u8>> {
    ensure!(
        *evidence == identify_suspension_excitation(&evidence.run_evidence.request)?,
        "excitation evidence replay mismatch"
    );
    let bytes = serde_json::to_vec(evidence)?;
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "excitation evidence too large"
    );
    Ok(bytes)
}

/// Bounded decoder that recomputes diagnostics, input binding and run verdicts.
pub fn decode_suspension_excitation(bytes: &[u8]) -> Result<SuspensionExcitationEvidence> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "excitation evidence too large"
    );
    let evidence: SuspensionExcitationEvidence = serde_json::from_slice(bytes)?;
    ensure!(
        evidence == identify_suspension_excitation(&evidence.run_evidence.request)?,
        "excitation evidence replay mismatch"
    );
    Ok(evidence)
}

/// Maximum serialized request or evidence size (including embedded samples).
pub const MAX_SUSPENSION_RUN_BYTES: usize = 8 * 1024 * 1024;

/// One complete acquisition assigned to a frozen split role.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionRunInput {
    /// Stable numeric identity used in the per-run solver report.
    pub acquisition_id: u64,
    /// Existing v1 dataset, retaining source declarations and SI samples.
    pub dataset: SuspensionIdentificationDataset,
}

/// Explicit whole-run split; source labels do not establish physical qualification.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionRunRequest {
    /// Must be `rne_suspension_run_request`.
    pub kind: String,
    /// Must be 1; separate from the existing stride-split artifact kind.
    pub schema_version: u32,
    /// Frozen parameter and residual gates; stride is validated but unused.
    pub spec: SuspensionIdentificationSpec,
    /// Complete training acquisitions, in accumulation order.
    pub training: Vec<SuspensionRunInput>,
    /// Complete holdout acquisitions, never used in coefficient fitting.
    pub holdout: Vec<SuspensionRunInput>,
}

impl SuspensionRunRequest {
    /// Checks bounded shape and rejects duplicate identities or exact sample copies.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == "rne_suspension_run_request" && self.schema_version == 1,
            "suspension run request kind/schema drift"
        );
        ensure!(self.spec.is_valid(), "invalid run identification spec");
        ensure!(
            !self.training.is_empty()
                && !self.holdout.is_empty()
                && self.training.len() + self.holdout.len() <= 64,
            "invalid run count"
        );
        let mut ids = BTreeSet::new();
        let mut names = BTreeSet::new();
        let mut sample_hashes = BTreeSet::new();
        let mut count = 0usize;
        for run in self.training.iter().chain(&self.holdout) {
            ensure!(ids.insert(run.acquisition_id), "duplicate acquisition ID");
            ensure!(
                names.insert(&run.dataset.dataset_id),
                "duplicate dataset ID"
            );
            count = count
                .checked_add(run.dataset.samples.len())
                .ok_or_else(|| anyhow::anyhow!("sample count overflow"))?;
            ensure!(count <= 100_000, "too many combined samples");
            run.dataset.validate()?;
            ensure!(
                sample_hashes.insert(hash(&run.dataset.samples)?),
                "duplicate sample capture"
            );
        }
        ensure!(
            serde_json::to_vec(self)?.len() <= MAX_SUSPENSION_RUN_BYTES,
            "run request too large"
        );
        Ok(())
    }
}

/// Embedded inputs plus recomputable report, with SHA-256 integrity (not authentication).
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionRunEvidence {
    /// Must be `rne_suspension_run_evidence`.
    pub kind: String,
    /// Separate artifact schema, currently 1.
    pub schema_version: u32,
    /// Exact declared acquisition split and gates.
    pub request: SuspensionRunRequest,
    /// SHA-256 of typed compact JSON for the request.
    pub request_sha256: String,
    /// Recomputed fit and per-run verdicts; a false verdict is retained.
    pub report: SuspensionRunIdentificationReport,
}

fn hash(value: &impl Serialize) -> Result<String> {
    Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(value)?)))
}

/// Executes a checked split without upgrading source declarations to qualification.
pub fn identify_suspension_runs(request: &SuspensionRunRequest) -> Result<SuspensionRunEvidence> {
    request.validate()?;
    // Borrow samples directly; no concatenation or interpolation changes their clocks.
    let training: Vec<_> = request
        .training
        .iter()
        .map(|run| SuspensionIdentificationRun {
            acquisition_id: run.acquisition_id,
            samples: &run.dataset.samples,
        })
        .collect();
    let holdout: Vec<_> = request
        .holdout
        .iter()
        .map(|run| SuspensionIdentificationRun {
            acquisition_id: run.acquisition_id,
            samples: &run.dataset.samples,
        })
        .collect();
    let report = identify_suspension_strut_runs_report(request.spec, &training, &holdout)?;
    let evidence = SuspensionRunEvidence {
        kind: "rne_suspension_run_evidence".into(),
        schema_version: 1,
        request: request.clone(),
        request_sha256: hash(request)?,
        report,
    };
    ensure!(
        serde_json::to_vec(&evidence)?.len() <= MAX_SUSPENSION_RUN_BYTES,
        "run evidence too large"
    );
    Ok(evidence)
}

/// Bounded request decoder, rejecting unknown fields before execution.
pub fn decode_suspension_run_request(bytes: &[u8]) -> Result<SuspensionRunRequest> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "run request too large"
    );
    let request: SuspensionRunRequest = serde_json::from_slice(bytes)?;
    request.validate()?;
    Ok(request)
}

/// Revalidates and encodes compact JSON within the decoder's exact byte bound.
///
/// No whitespace or trailing newline is added, so formatting cannot turn an
/// accepted artifact into an oversized, unreadable output.
pub fn encode_suspension_run_evidence(evidence: &SuspensionRunEvidence) -> Result<Vec<u8>> {
    ensure!(
        *evidence == identify_suspension_runs(&evidence.request)?,
        "run evidence replay mismatch"
    );
    let bytes = serde_json::to_vec(evidence)?;
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "run evidence too large"
    );
    Ok(bytes)
}

/// Bounded evidence decoder that reruns identification rather than trusting verdicts.
pub fn decode_suspension_run_evidence(bytes: &[u8]) -> Result<SuspensionRunEvidence> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_RUN_BYTES,
        "run evidence too large"
    );
    let evidence: SuspensionRunEvidence = serde_json::from_slice(bytes)?;
    ensure!(
        evidence == identify_suspension_runs(&evidence.request)?,
        "run evidence replay mismatch"
    );
    Ok(evidence)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::suspension_identification::{
        suspension_identification_spec, synthetic_suspension_identification_dataset,
    };

    fn request() -> SuspensionRunRequest {
        let first = synthetic_suspension_identification_dataset().unwrap();
        let mut second = first.clone();
        second.dataset_id = "synthetic.second".into();
        for sample in &mut second.samples {
            sample.force_n += 1.0;
        }
        second.seal().unwrap();
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
                dataset: second,
            }],
        }
    }

    #[test]
    fn excitation_evidence_recomputes_training_only_and_keeps_v1_unchanged() {
        let mut request = request();
        let evidence = identify_suspension_excitation(&request).unwrap();
        assert_eq!(
            evidence.run_evidence,
            identify_suspension_runs(&request).unwrap()
        );
        let bytes = encode_suspension_excitation(&evidence).unwrap();
        assert_eq!(decode_suspension_excitation(&bytes).unwrap(), evidence);
        for sample in &mut request.holdout[0].dataset.samples {
            sample.force_n += 1.0;
        }
        request.holdout[0].dataset.seal().unwrap();
        let changed = identify_suspension_excitation(&request).unwrap();
        assert_eq!(changed.training_excitation, evidence.training_excitation);
        assert_ne!(
            changed.run_evidence.request_sha256,
            evidence.run_evidence.request_sha256
        );
        let mut forged = evidence;
        forged.training_excitation.position_rms_m *= 2.0;
        assert!(encode_suspension_excitation(&forged).is_err());
        assert!(decode_suspension_excitation(&serde_json::to_vec(&forged).unwrap()).is_err());
    }

    #[test]
    fn suspension_runs_roundtrip_recomputes_and_rejects_tampering() {
        let request = request();
        let decoded =
            decode_suspension_run_request(&serde_json::to_vec(&request).unwrap()).unwrap();
        let mut evidence = identify_suspension_runs(&decoded).unwrap();
        assert!(evidence.report.passed);
        let encoded = encode_suspension_run_evidence(&evidence).unwrap();
        assert!(encoded.len() <= MAX_SUSPENSION_RUN_BYTES);
        assert_eq!(decode_suspension_run_evidence(&encoded).unwrap(), evidence);
        assert_eq!(
            decode_suspension_run_evidence(&serde_json::to_vec(&evidence).unwrap()).unwrap(),
            evidence
        );
        evidence.report.holdout_runs[0].rmse_n += 1.0;
        assert!(encode_suspension_run_evidence(&evidence).is_err());
        assert!(decode_suspension_run_evidence(&serde_json::to_vec(&evidence).unwrap()).is_err());
    }

    #[test]
    fn suspension_runs_preserve_failed_verdict_and_reject_rehashed_split_drift() {
        let mut request = request();
        let mut bad = request.holdout[0].clone();
        bad.acquisition_id = 3;
        bad.dataset.dataset_id = "synthetic.bad.short".into();
        bad.dataset.samples.truncate(20);
        for sample in &mut bad.dataset.samples {
            sample.force_n += 49.0;
        }
        bad.dataset.seal().unwrap();
        request.holdout.push(bad);
        let evidence = identify_suspension_runs(&request).unwrap();
        assert!(evidence.report.fit.holdout_rmse_n < request.spec.maximum_holdout_rmse_n);
        assert!(!evidence.report.passed);
        assert!(!evidence.report.holdout_runs[1].passed);
        let bytes = serde_json::to_vec(&evidence).unwrap();
        assert_eq!(decode_suspension_run_evidence(&bytes).unwrap(), evidence);
        let mut forged = evidence.clone();
        forged.report.passed = true;
        assert!(decode_suspension_run_evidence(&serde_json::to_vec(&forged).unwrap()).is_err());
        let mut forged = evidence.clone();
        forged.request.holdout.swap(0, 1);
        forged.request_sha256 = hash(&forged.request).unwrap();
        assert!(decode_suspension_run_evidence(&serde_json::to_vec(&forged).unwrap()).is_err());
        let mut forged = evidence.clone();
        forged.schema_version = 2;
        assert!(decode_suspension_run_evidence(&serde_json::to_vec(&forged).unwrap()).is_err());
        let mut forged = evidence;
        forged.request_sha256 = "0".repeat(64);
        assert!(decode_suspension_run_evidence(&serde_json::to_vec(&forged).unwrap()).is_err());
    }

    #[test]
    fn suspension_runs_reject_duplicate_capture_even_with_new_ids() {
        let mut request = request();
        request.holdout[0].dataset.samples = request.training[0].dataset.samples.clone();
        request.holdout[0].dataset.seal().unwrap();
        assert!(request
            .validate()
            .unwrap_err()
            .to_string()
            .contains("duplicate sample"));
        assert!(decode_suspension_run_request(&vec![b' '; MAX_SUSPENSION_RUN_BYTES + 1]).is_err());
        let mut value = serde_json::to_value(self::request()).unwrap();
        value["unexpected"] = true.into();
        assert!(decode_suspension_run_request(&serde_json::to_vec(&value).unwrap()).is_err());
    }
}

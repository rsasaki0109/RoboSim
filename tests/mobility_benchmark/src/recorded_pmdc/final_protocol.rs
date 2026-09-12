//! Frozen contract for sealing and evaluating the two untouched PMDC final runs.

use super::{pmdc_identification_protocol, PMDC_PROTOCOL_SHA256};
use anyhow::{ensure, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Stable artifact kind for the final evaluation contract.
pub const PMDC_FINAL_PROTOCOL_KIND: &str = "rne_pmdc_final_evaluation_protocol";
/// Training-only selected-model evidence bound by final protocol v1.
pub const PMDC_TRAINING_IDENTIFICATION_CONTENT_SHA256: &str =
    "dc9995c3c0b76abeb03b6dfbce8e696ea66091221fbbf7b082c81f0785af361d";
/// Exposed one-shot development evidence bound by final protocol v1.
pub const PMDC_DEVELOPMENT_EVALUATION_CONTENT_SHA256: &str =
    "1cf7b5769d7e3a6f86fc6fc9d86a67ea821ce437d09948b0ddbe87635f024c71";
/// Exact byte length of the exclusively created final-partition artifact.
pub const PMDC_FINAL_ARTIFACT_BYTES: usize = 1_261_025;
/// SHA-256 of the complete final-partition JSONL artifact, including manifest and trailer.
pub const PMDC_FINAL_ARTIFACT_SHA256: &str =
    "94c2829014d9e82adbd101f6c78f15257d71c24147d539f6fdc0a401bd6772d2";
/// SHA-256 of the 4,018 canonical final record lines, excluding manifest and trailer.
pub const PMDC_FINAL_RECORDS_SHA256: &str =
    "722af6217cb801bb4a56557095f2924a6f4931415be6f87a950978092df11409";

/// Aggregation level at which a final metric must pass.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PmdcFinalAggregation {
    /// Each complete final run is evaluated independently.
    PerRun,
    /// Residual sums are pooled before normalization.
    Pooled,
    /// Maximum normalized value across the two complete runs.
    WorstRun,
}

/// One final metric with unchanged development threshold and explicit aggregation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcFinalMetricSpec {
    /// Metric identifier from the development protocol.
    pub id: String,
    /// Required aggregation level.
    pub aggregation: PmdcFinalAggregation,
    /// Inclusive maximum accepted value.
    pub maximum: f64,
}

/// Complete final protocol frozen while both final response runs remain unread.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct PmdcFinalEvaluationProtocol {
    /// Artifact discriminator.
    pub kind: String,
    /// Protocol schema version.
    pub schema_version: u32,
    /// Parent identification/development protocol identity.
    pub parent_protocol_sha256: String,
    /// Exact selected training model evidence.
    pub training_identification_content_sha256: String,
    /// Exact exposed development result; it may not be rerun.
    pub development_evaluation_content_sha256: String,
    /// Frozen untouched source trials.
    pub final_trials: Vec<u8>,
    /// Exact source header cells in evaluation order.
    pub final_header_cells: Vec<String>,
    /// Expected complete sample count in each final run.
    pub samples_per_run: usize,
    /// Refit/model-selection prohibition after development exposure.
    pub refit_allowed: bool,
    /// Threshold-change prohibition after development exposure.
    pub threshold_change_allowed: bool,
    /// Final source conversion must create a new artifact exactly once.
    pub conversion_create_new: bool,
    /// Final evaluator must create a new artifact exactly once.
    pub evaluation_create_new: bool,
    /// All per-run, pooled and worst-run metric gates.
    pub metrics: Vec<PmdcFinalMetricSpec>,
    /// Current common capsule cannot honestly encode the nonuniform recorded clock.
    pub common_failure_capsule_supported: bool,
    /// Required replay clock for a future portable Failure Capsule.
    pub required_failure_replay_clock: String,
    /// A failed final result must still be retained even before capsule support exists.
    pub retain_failed_evaluation_evidence: bool,
    /// Physical motor constants remain outside this empirical final gate.
    pub qualification_scope: String,
}

impl PmdcFinalEvaluationProtocol {
    /// Reject any mutation of the contract frozen before final conversion.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self == &pmdc_final_evaluation_protocol(),
            "PMDC final evaluation protocol drift"
        );
        Ok(())
    }

    /// SHA-256 of deterministic serde JSON for conversion/evaluation binding.
    pub fn sha256(&self) -> Result<String> {
        self.validate()?;
        Ok(format!("{:x}", Sha256::digest(serde_json::to_vec(self)?)))
    }
}

/// Construct final protocol v1 without opening either final response run.
pub fn pmdc_final_evaluation_protocol() -> PmdcFinalEvaluationProtocol {
    let parent = pmdc_identification_protocol();
    let metrics = parent
        .development_metrics
        .iter()
        .flat_map(|metric| {
            [
                PmdcFinalMetricSpec {
                    id: metric.id.clone(),
                    aggregation: PmdcFinalAggregation::PerRun,
                    maximum: metric.maximum,
                },
                PmdcFinalMetricSpec {
                    id: metric.id.clone(),
                    aggregation: PmdcFinalAggregation::Pooled,
                    maximum: metric.maximum,
                },
                PmdcFinalMetricSpec {
                    id: metric.id.clone(),
                    aggregation: PmdcFinalAggregation::WorstRun,
                    maximum: metric.maximum,
                },
            ]
        })
        .collect();
    PmdcFinalEvaluationProtocol {
        kind: PMDC_FINAL_PROTOCOL_KIND.into(),
        schema_version: 1,
        parent_protocol_sha256: PMDC_PROTOCOL_SHA256.into(),
        training_identification_content_sha256: PMDC_TRAINING_IDENTIFICATION_CONTENT_SHA256.into(),
        development_evaluation_content_sha256: PMDC_DEVELOPMENT_EVALUATION_CONTENT_SHA256.into(),
        final_trials: vec![10, 11],
        final_header_cells: vec!["A18101".into(), "A20112".into()],
        samples_per_run: 2009,
        refit_allowed: false,
        threshold_change_allowed: false,
        conversion_create_new: true,
        evaluation_create_new: true,
        metrics,
        common_failure_capsule_supported: false,
        required_failure_replay_clock: "recorded_source_timestamp_us_nonuniform".into(),
        retain_failed_evaluation_evidence: true,
        qualification_scope:
            "effective_model_final_generalization_not_individual_physical_constants".into(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn final_contract_keeps_runs_sealed_and_all_aggregations_frozen() {
        let protocol = pmdc_final_evaluation_protocol();
        protocol.validate().unwrap();
        assert_eq!(protocol.final_trials, [10, 11]);
        assert_eq!(protocol.final_header_cells, ["A18101", "A20112"]);
        assert_eq!(protocol.metrics.len(), 12);
        for chunk in protocol.metrics.chunks_exact(3) {
            assert_eq!(chunk[0].aggregation, PmdcFinalAggregation::PerRun);
            assert_eq!(chunk[1].aggregation, PmdcFinalAggregation::Pooled);
            assert_eq!(chunk[2].aggregation, PmdcFinalAggregation::WorstRun);
            assert_eq!(chunk[0].maximum, chunk[1].maximum);
            assert_eq!(chunk[0].maximum, chunk[2].maximum);
        }
        assert!(!protocol.refit_allowed);
        assert!(!protocol.threshold_change_allowed);
        assert!(!protocol.common_failure_capsule_supported);
        assert!(protocol.retain_failed_evaluation_evidence);
    }

    #[test]
    fn final_contract_rejects_threshold_or_capsule_overclaim() {
        let mut changed = pmdc_final_evaluation_protocol();
        changed.metrics[0].maximum = 1.0;
        assert!(changed.validate().is_err());
        let mut changed = pmdc_final_evaluation_protocol();
        changed.common_failure_capsule_supported = true;
        assert!(changed.validate().is_err());
        let mut changed = pmdc_final_evaluation_protocol();
        changed.final_trials.reverse();
        assert!(changed.validate().is_err());
    }

    #[test]
    fn final_seal_identity_is_frozen_separately_from_the_protocol() {
        assert_eq!(PMDC_FINAL_ARTIFACT_BYTES, 1_261_025);
        assert_eq!(PMDC_FINAL_ARTIFACT_SHA256.len(), 64);
        assert_eq!(PMDC_FINAL_RECORDS_SHA256.len(), 64);
        assert_ne!(PMDC_FINAL_ARTIFACT_SHA256, PMDC_FINAL_RECORDS_SHA256);
        assert_eq!(
            pmdc_final_evaluation_protocol().sha256().unwrap(),
            "c8ed0ce2f34fa90fd1797b42dee02f2aa721760f504d9590c1e3b659220fca53"
        );
    }
}

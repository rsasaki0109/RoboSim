//! Owned staged tire load-sensitivity datasets and replayable fit evidence.

use crate::tire_identification::{
    identify_tire_dataset, synthetic_tire_identification_dataset, TireDatasetSourceKind,
    TireIdentificationDataset, TireIdentificationEvidence,
};
use anyhow::{ensure, Result};
use rne_robot::{
    evaluate_combined_slip_tire_steady_force, identify_tire_load_sensitivity, CombinedSlipTireSpec,
    TireForceIdentificationSample, TireIdentificationRun, TireLoadSensitivityIdentificationResult,
    TireLoadSensitivityIdentificationSpec,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Stable owned load-sensitivity dataset discriminator.
pub const TIRE_LOAD_SENSITIVITY_DATASET_KIND: &str = "rne_mobility_tire_load_sensitivity_dataset";
/// Stable load-sensitivity evidence discriminator.
pub const TIRE_LOAD_SENSITIVITY_RESULT_KIND: &str = "rne_mobility_tire_load_sensitivity_result";
/// Current dataset and result schema.
pub const TIRE_LOAD_SENSITIVITY_SCHEMA_VERSION: u32 = 1;
/// Maximum accepted serialized dataset size.
pub const MAX_TIRE_LOAD_SENSITIVITY_DATASET_BYTES: usize = 32 * 1024 * 1024;

/// One complete load-sweep acquisition owned by a frozen split.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityRunData {
    /// Stable acquisition identity unique across train and holdout.
    pub acquisition_id: u64,
    /// Stable condition identity used for worst-condition holdout reporting.
    pub condition_id: u64,
    /// Independently established road-friction scale.
    pub road_friction_scale: f64,
    /// Strictly time-ordered combined-slip force observations.
    pub samples: Vec<TireForceIdentificationSample>,
}

impl TireLoadSensitivityRunData {
    fn borrowed(&self) -> TireIdentificationRun<'_> {
        TireIdentificationRun {
            acquisition_id: self.acquisition_id,
            condition_id: self.condition_id,
            road_friction_scale: self.road_friction_scale,
            samples: &self.samples,
        }
    }
}

/// Owned load-sensitivity input that replays the exact preceding steady fit.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityDataset {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Stable bounded dataset identity.
    pub dataset_id: String,
    /// Honest origin class.
    pub source_kind: TireDatasetSourceKind,
    /// Human-readable source description; never physical proof by itself.
    pub source_description: String,
    /// Exact staged force-law identity.
    pub force_model: String,
    /// Owned steady dataset whose four reference-load parameters remain frozen.
    pub steady_dataset: TireIdentificationDataset,
    /// Replayable steady identification evidence.
    pub steady_identification: TireIdentificationEvidence,
    /// Complete acquisitions used exclusively for the load-sensitivity fit.
    pub training_runs: Vec<TireLoadSensitivityRunData>,
    /// Complete acquisitions used exclusively for holdout.
    pub holdout_runs: Vec<TireLoadSensitivityRunData>,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireLoadSensitivityDataset {
    /// Recomputes and stores the self-excluding dataset digest.
    pub fn seal(&mut self) -> Result<()> {
        self.content_sha256 = dataset_digest(self)?;
        Ok(())
    }

    /// Validates ownership, split structure, physical domains, and integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == TIRE_LOAD_SENSITIVITY_DATASET_KIND
                && self.schema_version == TIRE_LOAD_SENSITIVITY_SCHEMA_VERSION,
            "tire load-sensitivity dataset kind/schema drift"
        );
        ensure!(
            valid_id(&self.dataset_id),
            "invalid load-sensitivity dataset identity"
        );
        ensure!(
            !self.source_description.is_empty() && self.source_description.len() <= 512,
            "invalid load-sensitivity source description"
        );
        ensure!(
            self.force_model == "rne_combined_slip_tanh_ellipse_load_sensitivity_v1",
            "unsupported load-sensitivity force model"
        );
        self.steady_identification.validate(&self.steady_dataset)?;
        ensure!(
            self.source_kind == self.steady_dataset.source_kind
                && self.source_kind == self.steady_identification.source_kind,
            "load-sensitivity provenance drift"
        );
        ensure!(
            !self.training_runs.is_empty() && !self.holdout_runs.is_empty(),
            "empty load-sensitivity split"
        );
        let mut acquisition_ids = BTreeSet::new();
        let mut sample_count = 0_usize;
        for run in self.training_runs.iter().chain(&self.holdout_runs) {
            ensure!(
                acquisition_ids.insert(run.acquisition_id),
                "duplicate load-sensitivity acquisition"
            );
            ensure!(
                run.road_friction_scale.is_finite()
                    && run.road_friction_scale > 0.0
                    && !run.samples.is_empty(),
                "invalid load-sensitivity run"
            );
            sample_count = sample_count
                .checked_add(run.samples.len())
                .ok_or_else(|| anyhow::anyhow!("load-sensitivity sample count overflow"))?;
            ensure!(
                sample_count <= 100_000,
                "unbounded load-sensitivity samples"
            );
            let tire = self.steady_identification.result.tire_spec;
            ensure!(
                run.samples.iter().all(|sample| {
                    [
                        sample.capture_time_s,
                        sample.longitudinal_slip_ratio,
                        sample.lateral_slip_tangent,
                        sample.normal_load_n,
                        sample.longitudinal_force_n,
                        sample.lateral_force_n,
                    ]
                    .iter()
                    .all(|value| value.is_finite())
                        && sample.normal_load_n > 0.0
                        && sample.normal_load_n <= tire.reference_load_n * tire.maximum_load_ratio
                }) && run
                    .samples
                    .windows(2)
                    .all(|pair| pair[0].capture_time_s < pair[1].capture_time_s),
                "invalid load-sensitivity samples"
            );
        }
        ensure!(
            self.content_sha256 == dataset_digest(self)?,
            "load-sensitivity dataset digest drift"
        );
        Ok(())
    }
}

/// Recomputable load-sensitivity result bound to one exact owned dataset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact input dataset SHA-256.
    pub dataset_content_sha256: String,
    /// Input provenance copied into the result.
    pub source_kind: TireDatasetSourceKind,
    /// Source-label claim only; retained-file qualification remains separate.
    pub recorded_source_claim: bool,
    /// Frozen search, excitation, and residual contract.
    pub identification_spec: TireLoadSensitivityIdentificationSpec,
    /// Deterministically recomputed fit and holdout evidence.
    pub result: TireLoadSensitivityIdentificationResult,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireLoadSensitivityEvidence {
    /// Replays the staged fit and verifies provenance, split, and content bindings.
    pub fn validate(&self, dataset: &TireLoadSensitivityDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == TIRE_LOAD_SENSITIVITY_RESULT_KIND
                && self.schema_version == TIRE_LOAD_SENSITIVITY_SCHEMA_VERSION,
            "load-sensitivity result kind/schema drift"
        );
        ensure!(
            self.dataset_content_sha256 == dataset.content_sha256
                && self.source_kind == dataset.source_kind
                && self.recorded_source_claim == dataset.source_kind.is_physical_measurement(),
            "load-sensitivity result provenance drift"
        );
        ensure!(
            self.result == identify(dataset, self.identification_spec)?,
            "load-sensitivity result drift"
        );
        ensure!(
            self.content_sha256 == evidence_digest(self)?,
            "load-sensitivity result digest drift"
        );
        Ok(())
    }
}

/// Returns the frozen v1 one-parameter search and acceptance contract.
pub fn tire_load_sensitivity_identification_spec() -> TireLoadSensitivityIdentificationSpec {
    TireLoadSensitivityIdentificationSpec {
        load_sensitivity_bounds_per_load_ratio: [0.0, 0.4],
        minimum_combined_axis_slip: 0.15,
        maximum_abs_slip: 1.0,
        minimum_training_load_ratio_span: 0.8,
        minimum_training_samples: 9,
        minimum_holdout_samples: 6,
        minimum_holdout_samples_per_condition: 3,
        grid_points: 17,
        refinement_passes: 4,
        maximum_training_rms_n: 1.0e-8,
        maximum_holdout_rms_n: 1.0e-8,
        maximum_worst_condition_rms_n: 1.0e-8,
    }
}

/// Decodes and validates one bounded owned dataset.
pub fn decode_tire_load_sensitivity_dataset(bytes: &[u8]) -> Result<TireLoadSensitivityDataset> {
    ensure!(
        bytes.len() <= MAX_TIRE_LOAD_SENSITIVITY_DATASET_BYTES,
        "load-sensitivity dataset exceeds byte limit"
    );
    let dataset: TireLoadSensitivityDataset = serde_json::from_slice(bytes)?;
    dataset.validate()?;
    Ok(dataset)
}

/// Fits and seals one verified owned dataset.
pub fn identify_tire_load_sensitivity_dataset(
    dataset: &TireLoadSensitivityDataset,
) -> Result<TireLoadSensitivityEvidence> {
    dataset.validate()?;
    let identification_spec = tire_load_sensitivity_identification_spec();
    let result = identify(dataset, identification_spec)?;
    let mut evidence = TireLoadSensitivityEvidence {
        kind: TIRE_LOAD_SENSITIVITY_RESULT_KIND.into(),
        schema_version: TIRE_LOAD_SENSITIVITY_SCHEMA_VERSION,
        dataset_content_sha256: dataset.content_sha256.clone(),
        source_kind: dataset.source_kind,
        recorded_source_claim: dataset.source_kind.is_physical_measurement(),
        identification_spec,
        result,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = evidence_digest(&evidence)?;
    evidence.validate(dataset)?;
    Ok(evidence)
}

/// Builds the deterministic non-physical fixture for the staged software path.
pub fn synthetic_tire_load_sensitivity_dataset() -> Result<TireLoadSensitivityDataset> {
    let steady_dataset = synthetic_tire_identification_dataset()?;
    let steady_identification = identify_tire_dataset(&steady_dataset)?;
    let true_tire = CombinedSlipTireSpec {
        load_sensitivity_per_load_ratio: 0.2,
        ..steady_identification.result.tire_spec
    };
    let samples = |loads_n: &[f64], road_friction_scale: f64| -> Result<Vec<_>> {
        loads_n
            .iter()
            .flat_map(|load_n| {
                [(0.50, 0.25), (-0.45, 0.30), (0.35, -0.40)]
                    .into_iter()
                    .map(move |slip| (*load_n, slip))
            })
            .enumerate()
            .map(|(index, (load_n, (longitudinal, lateral)))| {
                let force = evaluate_combined_slip_tire_steady_force(
                    true_tire,
                    longitudinal,
                    lateral,
                    load_n,
                    road_friction_scale,
                )?;
                Ok(TireForceIdentificationSample {
                    capture_time_s: index as f64 * 0.01,
                    longitudinal_slip_ratio: longitudinal,
                    lateral_slip_tangent: lateral,
                    normal_load_n: load_n,
                    longitudinal_force_n: force.0,
                    lateral_force_n: force.1,
                })
            })
            .collect()
    };
    let run = |acquisition_id, condition_id, loads_n: &[f64], road_friction_scale| {
        Ok::<_, anyhow::Error>(TireLoadSensitivityRunData {
            acquisition_id,
            condition_id,
            road_friction_scale,
            samples: samples(loads_n, road_friction_scale)?,
        })
    };
    let mut dataset = TireLoadSensitivityDataset {
        kind: TIRE_LOAD_SENSITIVITY_DATASET_KIND.into(),
        schema_version: TIRE_LOAD_SENSITIVITY_SCHEMA_VERSION,
        dataset_id: "rne.synthetic.tire.load_sensitivity.v1".into(),
        source_kind: TireDatasetSourceKind::SyntheticFixture,
        source_description: "deterministic load sweep fixture; not physical data".into(),
        force_model: "rne_combined_slip_tanh_ellipse_load_sensitivity_v1".into(),
        steady_dataset,
        steady_identification,
        training_runs: vec![run(101, 1_010, &[600.0, 1_000.0, 1_600.0], 1.0)?],
        holdout_runs: vec![
            run(102, 1_020, &[800.0], 1.0)?,
            run(103, 1_030, &[1_400.0], 0.7)?,
        ],
        content_sha256: String::new(),
    };
    dataset.seal()?;
    dataset.validate()?;
    Ok(dataset)
}

fn identify(
    dataset: &TireLoadSensitivityDataset,
    spec: TireLoadSensitivityIdentificationSpec,
) -> Result<TireLoadSensitivityIdentificationResult> {
    let training = dataset
        .training_runs
        .iter()
        .map(TireLoadSensitivityRunData::borrowed)
        .collect::<Vec<_>>();
    let holdout = dataset
        .holdout_runs
        .iter()
        .map(TireLoadSensitivityRunData::borrowed)
        .collect::<Vec<_>>();
    Ok(identify_tire_load_sensitivity(
        spec,
        dataset.steady_identification.result.tire_spec,
        &training,
        &holdout,
    )?)
}

fn dataset_digest(dataset: &TireLoadSensitivityDataset) -> Result<String> {
    let mut canonical = dataset.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn evidence_digest(evidence: &TireLoadSensitivityEvidence) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn sha256(bytes: &[u8]) -> Result<String> {
    Ok(format!("sha256:{:x}", Sha256::digest(bytes)))
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_dataset_recovers_only_load_sensitivity_and_replays() {
        let dataset = synthetic_tire_load_sensitivity_dataset().unwrap();
        let evidence = identify_tire_load_sensitivity_dataset(&dataset).unwrap();
        let repeat = identify_tire_load_sensitivity_dataset(&dataset).unwrap();
        assert_eq!(evidence, repeat);
        assert_eq!(evidence.result.load_sensitivity_per_load_ratio, 0.2);
        assert_eq!(
            evidence.result.tire_spec,
            CombinedSlipTireSpec {
                load_sensitivity_per_load_ratio: 0.2,
                ..dataset.steady_identification.result.tire_spec
            }
        );
        assert!(!evidence.recorded_source_claim);
        evidence.validate(&dataset).unwrap();
    }

    #[test]
    fn dataset_and_result_tampering_fail_replay() {
        let mut dataset = synthetic_tire_load_sensitivity_dataset().unwrap();
        dataset.training_runs[0].samples[0].normal_load_n += 1.0;
        assert!(dataset.validate().is_err());

        let dataset = synthetic_tire_load_sensitivity_dataset().unwrap();
        let mut evidence = identify_tire_load_sensitivity_dataset(&dataset).unwrap();
        evidence.result.load_sensitivity_per_load_ratio += 0.01;
        assert!(evidence.validate(&dataset).is_err());
    }

    #[test]
    fn bounded_decoder_rejects_unknown_fields_and_oversize() {
        let dataset = synthetic_tire_load_sensitivity_dataset().unwrap();
        let mut value = serde_json::to_value(&dataset).unwrap();
        value["unexpected"] = serde_json::json!(true);
        assert!(
            decode_tire_load_sensitivity_dataset(&serde_json::to_vec(&value).unwrap()).is_err()
        );
        assert!(decode_tire_load_sensitivity_dataset(&vec![
            b' ';
            MAX_TIRE_LOAD_SENSITIVITY_DATASET_BYTES
                + 1
        ])
        .is_err());
    }
}

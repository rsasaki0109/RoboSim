//! Bounded, self-verifying steady combined-slip tire-identification artifacts.

use anyhow::{ensure, Result};
use rne_robot::{
    evaluate_combined_slip_tire_steady_force, identify_combined_slip_tire_steady,
    CombinedSlipTireIdentificationResult, CombinedSlipTireSpec, TireForceIdentificationSample,
    TireIdentificationRun, TireIdentificationSpec,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Stable input artifact kind.
pub const TIRE_IDENTIFICATION_DATASET_KIND: &str = "rne_mobility_tire_identification_dataset";
/// Stable result artifact kind.
pub const TIRE_IDENTIFICATION_RESULT_KIND: &str = "rne_mobility_tire_identification_result";
/// Identification artifact schema.
pub const TIRE_IDENTIFICATION_SCHEMA_VERSION: u32 = 1;
/// Result schema separating a recorded-source claim from physical qualification.
pub const TIRE_IDENTIFICATION_RESULT_SCHEMA_VERSION: u32 = 2;
/// Maximum accepted serialized dataset size.
pub const MAX_TIRE_IDENTIFICATION_DATASET_BYTES: usize = 32 * 1024 * 1024;

/// Honest origin class for tire-force observations.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireDatasetSourceKind {
    /// Deterministic generated samples used only to verify the software path.
    SyntheticFixture,
    /// Measurements from a tire or vehicle test bench.
    RecordedBench,
    /// Measurements captured on a physical vehicle.
    RecordedVehicle,
}

impl TireDatasetSourceKind {
    /// Returns whether this origin is a physical measurement.
    pub fn is_physical_measurement(self) -> bool {
        matches!(self, Self::RecordedBench | Self::RecordedVehicle)
    }
}

/// One owned acquisition assigned wholly to training or holdout.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireIdentificationRunData {
    /// Stable acquisition identity; cannot occur in both splits.
    pub acquisition_id: u64,
    /// Stable road/tire condition identity.
    pub condition_id: u64,
    /// Independently established dimensionless road-friction multiplier.
    pub road_friction_scale: f64,
    /// Strictly time-ordered SI-unit force samples.
    pub samples: Vec<TireForceIdentificationSample>,
}

impl TireIdentificationRunData {
    fn borrowed(&self) -> TireIdentificationRun<'_> {
        TireIdentificationRun {
            acquisition_id: self.acquisition_id,
            condition_id: self.condition_id,
            road_friction_scale: self.road_friction_scale,
            samples: &self.samples,
        }
    }
}

/// Unit-explicit, split-frozen steady tire-force identification input.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireIdentificationDataset {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Bounded stable dataset identity.
    pub dataset_id: String,
    /// Honest origin class.
    pub source_kind: TireDatasetSourceKind,
    /// Human-readable source description; not integrity evidence by itself.
    pub source_description: String,
    /// Exact force-law identity expected by the identifier.
    pub force_model: String,
    /// Fixed non-identified tire parameters and initial values.
    pub template: CombinedSlipTireSpec,
    /// Complete pure-slip acquisitions used for fitting.
    pub training_runs: Vec<TireIdentificationRunData>,
    /// Complete combined-slip acquisitions never used for fitting.
    pub holdout_runs: Vec<TireIdentificationRunData>,
    /// SHA-256 over canonical JSON with this field empty.
    pub content_sha256: String,
}

impl TireIdentificationDataset {
    /// Recomputes the integrity digest after populating the dataset.
    pub fn seal(&mut self) -> Result<()> {
        self.content_sha256 = dataset_digest(self)?;
        Ok(())
    }

    /// Validates bounds, sequence shape, provenance labels, and integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == TIRE_IDENTIFICATION_DATASET_KIND
                && self.schema_version == TIRE_IDENTIFICATION_SCHEMA_VERSION,
            "tire identification dataset kind/schema drift"
        );
        ensure!(valid_id(&self.dataset_id), "invalid tire dataset identity");
        ensure!(
            !self.source_description.is_empty() && self.source_description.len() <= 512,
            "invalid tire source description"
        );
        ensure!(
            self.force_model == "rne_combined_slip_tanh_ellipse_steady_v1",
            "unsupported tire force model"
        );
        ensure!(
            evaluate_combined_slip_tire_steady_force(
                self.template,
                0.0,
                0.0,
                self.template.reference_load_n,
                1.0,
            )
            .is_ok(),
            "invalid tire template"
        );
        ensure!(
            !self.training_runs.is_empty() && !self.holdout_runs.is_empty(),
            "empty tire identification split"
        );
        let run_count = self.training_runs.len() + self.holdout_runs.len();
        let sample_count = self
            .training_runs
            .iter()
            .chain(&self.holdout_runs)
            .try_fold(0_usize, |sum, run| sum.checked_add(run.samples.len()));
        ensure!(run_count <= 10_000, "unbounded tire run count");
        ensure!(
            matches!(sample_count, Some(1..=100_000)),
            "unbounded tire sample count"
        );
        ensure!(
            self.training_runs
                .iter()
                .chain(&self.holdout_runs)
                .all(|run| {
                    !run.samples.is_empty()
                        && run.road_friction_scale.is_finite()
                        && run.road_friction_scale > 0.0
                        && run.samples.iter().all(|sample| {
                            sample.capture_time_s.is_finite()
                                && sample.longitudinal_slip_ratio.is_finite()
                                && sample.lateral_slip_tangent.is_finite()
                                && sample.normal_load_n.is_finite()
                                && sample.longitudinal_force_n.is_finite()
                                && sample.lateral_force_n.is_finite()
                        })
                        && run
                            .samples
                            .windows(2)
                            .all(|pair| pair[0].capture_time_s < pair[1].capture_time_s)
                }),
            "invalid tire sample sequence"
        );
        ensure!(
            self.content_sha256 == dataset_digest(self)?,
            "tire dataset digest drift"
        );
        Ok(())
    }
}

/// Recomputable fit evidence bound to one exact dataset and split.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireIdentificationEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact input artifact SHA-256.
    pub dataset_content_sha256: String,
    /// Input provenance copied into the result.
    pub source_kind: TireDatasetSourceKind,
    /// Source-label claim only; physical qualification requires an acquisition artifact.
    pub recorded_source_claim: bool,
    /// Exact parameter bounds, excitation requirements, and residual gates.
    pub identification_spec: TireIdentificationSpec,
    /// Deterministically recomputed parameters and residuals.
    pub result: CombinedSlipTireIdentificationResult,
    /// SHA-256 over canonical JSON with this field empty.
    pub content_sha256: String,
}

impl TireIdentificationEvidence {
    /// Recomputes the fit and verifies input binding, provenance, and integrity.
    pub fn validate(&self, dataset: &TireIdentificationDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == TIRE_IDENTIFICATION_RESULT_KIND
                && self.schema_version == TIRE_IDENTIFICATION_RESULT_SCHEMA_VERSION,
            "tire identification result kind/schema drift"
        );
        ensure!(
            self.dataset_content_sha256 == dataset.content_sha256
                && self.source_kind == dataset.source_kind
                && self.recorded_source_claim == dataset.source_kind.is_physical_measurement(),
            "tire identification provenance drift"
        );
        let recomputed = identify(dataset, self.identification_spec)?;
        ensure!(
            self.result == recomputed,
            "tire identification result drift"
        );
        ensure!(
            self.content_sha256 == evidence_digest(self)?,
            "tire identification evidence digest drift"
        );
        Ok(())
    }
}

/// Decodes and verifies one bounded JSON dataset before allocation is trusted downstream.
pub fn decode_tire_identification_dataset(bytes: &[u8]) -> Result<TireIdentificationDataset> {
    ensure!(
        bytes.len() <= MAX_TIRE_IDENTIFICATION_DATASET_BYTES,
        "tire identification dataset exceeds byte limit"
    );
    let dataset: TireIdentificationDataset = serde_json::from_slice(bytes)?;
    dataset.validate()?;
    Ok(dataset)
}

/// Returns the frozen v1 search and acceptance contract.
pub fn tire_identification_spec() -> TireIdentificationSpec {
    TireIdentificationSpec {
        longitudinal_stiffness_bounds_n: [6_000.0, 10_000.0],
        lateral_stiffness_bounds_n: [5_000.0, 9_000.0],
        longitudinal_peak_friction_bounds: [0.5, 1.3],
        lateral_peak_friction_bounds: [0.4, 1.2],
        pure_slip_tolerance: 0.001,
        minimum_excited_slip: 0.02,
        maximum_linear_slip: 0.06,
        minimum_peak_slip: 0.4,
        maximum_abs_slip: 1.0,
        minimum_training_samples_per_axis: 6,
        minimum_combined_holdout_samples: 6,
        minimum_holdout_samples_per_condition: 3,
        grid_points_per_axis: 9,
        refinement_passes: 3,
        maximum_training_rms_n: 1.0e-8,
        maximum_holdout_rms_n: 1.0e-8,
        maximum_worst_condition_rms_n: 1.0e-8,
    }
}

/// Builds the deterministic non-physical fixture used to test the artifact path.
pub fn synthetic_tire_identification_dataset() -> Result<TireIdentificationDataset> {
    let identified = CombinedSlipTireSpec {
        longitudinal_stiffness_n: 8_000.0,
        lateral_stiffness_n: 7_000.0,
        longitudinal_peak_friction: 0.9,
        lateral_peak_friction: 0.8,
        ..CombinedSlipTireSpec::default()
    };
    let sample = |index: usize, longitudinal: f64, lateral: f64, load: f64, road: f64| {
        let (longitudinal_force_n, lateral_force_n) = evaluate_combined_slip_tire_steady_force(
            identified,
            longitudinal,
            lateral,
            load,
            road,
        )?;
        Ok::<_, anyhow::Error>(TireForceIdentificationSample {
            capture_time_s: index as f64 * 0.01,
            longitudinal_slip_ratio: longitudinal,
            lateral_slip_tangent: lateral,
            normal_load_n: load,
            longitudinal_force_n,
            lateral_force_n,
        })
    };
    let slips = [-0.6, -0.2, -0.05, 0.05, 0.2, 0.6];
    let training_samples = slips
        .into_iter()
        .map(|slip| (slip, 0.0))
        .chain(slips.into_iter().map(|slip| (0.0, slip)))
        .enumerate()
        .map(|(index, (longitudinal, lateral))| sample(index, longitudinal, lateral, 1_000.0, 1.0))
        .collect::<Result<Vec<_>>>()?;
    let combined = |road: f64| {
        [(-0.35, 0.20), (0.25, 0.30), (0.45, -0.25)]
            .into_iter()
            .enumerate()
            .map(|(index, (longitudinal, lateral))| {
                sample(
                    index,
                    longitudinal,
                    lateral,
                    800.0 + index as f64 * 200.0,
                    road,
                )
            })
            .collect::<Result<Vec<_>>>()
    };
    let mut dataset = TireIdentificationDataset {
        kind: TIRE_IDENTIFICATION_DATASET_KIND.to_string(),
        schema_version: TIRE_IDENTIFICATION_SCHEMA_VERSION,
        dataset_id: "rne.synthetic.tire.combined_slip.v1".to_string(),
        source_kind: TireDatasetSourceKind::SyntheticFixture,
        source_description: "deterministic generated contract fixture; not physical data"
            .to_string(),
        force_model: "rne_combined_slip_tanh_ellipse_steady_v1".to_string(),
        template: CombinedSlipTireSpec::default(),
        training_runs: vec![TireIdentificationRunData {
            acquisition_id: 1,
            condition_id: 10,
            road_friction_scale: 1.0,
            samples: training_samples,
        }],
        holdout_runs: vec![
            TireIdentificationRunData {
                acquisition_id: 2,
                condition_id: 20,
                road_friction_scale: 1.0,
                samples: combined(1.0)?,
            },
            TireIdentificationRunData {
                acquisition_id: 3,
                condition_id: 30,
                road_friction_scale: 0.6,
                samples: combined(0.6)?,
            },
        ],
        content_sha256: String::new(),
    };
    dataset.seal()?;
    dataset.validate()?;
    Ok(dataset)
}

/// Fits a verified dataset and binds the exact result to it.
pub fn identify_tire_dataset(
    dataset: &TireIdentificationDataset,
) -> Result<TireIdentificationEvidence> {
    dataset.validate()?;
    let identification_spec = tire_identification_spec();
    let result = identify(dataset, identification_spec)?;
    let mut evidence = TireIdentificationEvidence {
        kind: TIRE_IDENTIFICATION_RESULT_KIND.to_string(),
        schema_version: TIRE_IDENTIFICATION_RESULT_SCHEMA_VERSION,
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

fn identify(
    dataset: &TireIdentificationDataset,
    spec: TireIdentificationSpec,
) -> Result<CombinedSlipTireIdentificationResult> {
    let training = dataset
        .training_runs
        .iter()
        .map(TireIdentificationRunData::borrowed)
        .collect::<Vec<_>>();
    let holdout = dataset
        .holdout_runs
        .iter()
        .map(TireIdentificationRunData::borrowed)
        .collect::<Vec<_>>();
    Ok(identify_combined_slip_tire_steady(
        spec,
        dataset.template,
        &training,
        &holdout,
    )?)
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

fn dataset_digest(dataset: &TireIdentificationDataset) -> Result<String> {
    let mut canonical = dataset.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn evidence_digest(evidence: &TireIdentificationEvidence) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn sha256(bytes: &[u8]) -> Result<String> {
    let digest = Sha256::digest(bytes);
    Ok(format!("sha256:{digest:x}"))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_dataset_is_honest_identifiable_and_repeatable() {
        let dataset = synthetic_tire_identification_dataset().unwrap();
        assert!(!dataset.source_kind.is_physical_measurement());
        let first = identify_tire_dataset(&dataset).unwrap();
        let second = identify_tire_dataset(&dataset).unwrap();
        assert_eq!(first, second);
        assert!(!first.recorded_source_claim);
        assert_eq!(first.result.tire_spec.longitudinal_stiffness_n, 8_000.0);
        assert_eq!(first.result.tire_spec.lateral_stiffness_n, 7_000.0);
        assert!(first.result.holdout_rms_n < 1.0e-10);
    }

    #[test]
    fn dataset_and_result_tampering_are_rejected() {
        let dataset = synthetic_tire_identification_dataset().unwrap();
        let evidence = identify_tire_dataset(&dataset).unwrap();
        let mut tampered_dataset = dataset.clone();
        tampered_dataset.holdout_runs[0].samples[0].longitudinal_force_n += 1.0;
        assert!(tampered_dataset.validate().is_err());
        let mut tampered_evidence = evidence;
        tampered_evidence.recorded_source_claim = true;
        assert!(tampered_evidence.validate(&dataset).is_err());
    }

    #[test]
    fn bounded_decoder_rejects_unknown_fields_and_oversize() {
        let dataset = synthetic_tire_identification_dataset().unwrap();
        let mut value = serde_json::to_value(dataset).unwrap();
        value["unknown"] = serde_json::json!(true);
        assert!(decode_tire_identification_dataset(
            serde_json::to_string(&value).unwrap().as_bytes()
        )
        .is_err());
        assert!(decode_tire_identification_dataset(&vec![
            b' ';
            MAX_TIRE_IDENTIFICATION_DATASET_BYTES + 1
        ])
        .is_err());
    }
}

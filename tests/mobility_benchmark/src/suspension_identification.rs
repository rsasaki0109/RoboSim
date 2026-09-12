//! Self-verifying suspension-identification dataset and result artifacts.

use anyhow::{ensure, Result};
use rne_robot::{
    identify_suspension_strut, SuspensionForceSample, SuspensionIdentificationResult,
    SuspensionIdentificationSpec,
};
use serde::{Deserialize, Serialize};

/// Stable input artifact kind.
pub const SUSPENSION_IDENTIFICATION_DATASET_KIND: &str =
    "rne_mobility_suspension_identification_dataset";
/// Stable result artifact kind.
pub const SUSPENSION_IDENTIFICATION_RESULT_KIND: &str =
    "rne_mobility_suspension_identification_result";
/// Identification artifact schema.
pub const SUSPENSION_IDENTIFICATION_SCHEMA_VERSION: u32 = 1;
/// Maximum accepted serialized dataset size.
pub const MAX_SUSPENSION_IDENTIFICATION_DATASET_BYTES: usize = 8 * 1024 * 1024;

/// Explicit provenance class that keeps the built-in fixture labelled as synthetic.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionDatasetSourceKind {
    /// Deterministic generated samples used only to verify the contract and solver.
    SyntheticFixture,
    /// Measurements from a suspension test bench.
    RecordedBench,
    /// Measurements captured on a physical vehicle.
    RecordedVehicle,
}

impl SuspensionDatasetSourceKind {
    /// Returns whether this provenance class represents a physical measurement.
    pub fn is_physical_measurement(self) -> bool {
        matches!(self, Self::RecordedBench | Self::RecordedVehicle)
    }
}

/// Ordered, unit-explicit suspension-force identification input.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionIdentificationDataset {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Bounded stable dataset identity.
    pub dataset_id: String,
    /// Honest origin class.
    pub source_kind: SuspensionDatasetSourceKind,
    /// Human-readable source description; not used as integrity evidence.
    pub source_description: String,
    /// Exact force model expected by the identifier.
    pub force_model: String,
    /// Strictly time-ordered SI-unit samples.
    pub samples: Vec<SuspensionForceSample>,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl SuspensionIdentificationDataset {
    /// Recomputes the integrity digest after a producer has populated the dataset.
    pub fn seal(&mut self) -> Result<()> {
        self.content_digest = dataset_digest(self)?;
        Ok(())
    }

    /// Validates shape, provenance, timestamps, and content integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == SUSPENSION_IDENTIFICATION_DATASET_KIND
                && self.schema_version == SUSPENSION_IDENTIFICATION_SCHEMA_VERSION,
            "suspension identification dataset kind/schema drift"
        );
        ensure!(valid_id(&self.dataset_id), "invalid dataset identity");
        ensure!(
            !self.source_description.is_empty() && self.source_description.len() <= 512,
            "invalid source description"
        );
        ensure!(
            self.force_model
                == "force_n=k_n_per_m*(equilibrium_m-position_m)-c_n_s_per_m*velocity_m_s",
            "unsupported suspension force model"
        );
        ensure!(
            self.samples.len() >= 4 && self.samples.len() <= 100_000,
            "unbounded suspension sample count"
        );
        ensure!(
            self.samples.iter().all(|sample| {
                sample.capture_time_s.is_finite()
                    && sample.position_m.is_finite()
                    && sample.velocity_m_s.is_finite()
                    && sample.force_n.is_finite()
            }) && self
                .samples
                .windows(2)
                .all(|pair| pair[0].capture_time_s < pair[1].capture_time_s),
            "invalid suspension sample sequence"
        );
        ensure!(
            self.content_digest == dataset_digest(self)?,
            "suspension dataset digest drift"
        );
        Ok(())
    }
}

/// Self-verifying identification output bound to the exact input and split.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionIdentificationEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact input artifact digest.
    pub dataset_content_digest: String,
    /// Input provenance copied into the content-bound result.
    pub source_kind: SuspensionDatasetSourceKind,
    /// True only for bench or vehicle measurements, never for the generated fixture.
    pub physical_measurement: bool,
    /// Exact split, parameter bounds, and residual gates.
    pub identification_spec: SuspensionIdentificationSpec,
    /// Deterministically recomputed fit and residuals.
    pub result: SuspensionIdentificationResult,
    /// FNV-1a digest with this field empty.
    pub content_digest: String,
}

impl SuspensionIdentificationEvidence {
    /// Recomputes the fit and verifies provenance, input binding, and integrity.
    pub fn validate(&self, dataset: &SuspensionIdentificationDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == SUSPENSION_IDENTIFICATION_RESULT_KIND
                && self.schema_version == SUSPENSION_IDENTIFICATION_SCHEMA_VERSION,
            "suspension identification result kind/schema drift"
        );
        ensure!(
            self.dataset_content_digest == dataset.content_digest
                && self.source_kind == dataset.source_kind
                && self.physical_measurement == dataset.source_kind.is_physical_measurement(),
            "suspension identification provenance drift"
        );
        let recomputed = identify_suspension_strut(self.identification_spec, &dataset.samples)?;
        ensure!(
            self.result == recomputed,
            "suspension identification result drift"
        );
        ensure!(
            self.content_digest == evidence_digest(self)?,
            "suspension identification evidence digest drift"
        );
        Ok(())
    }
}

/// Decodes a bounded JSON dataset and verifies it before use.
pub fn decode_suspension_identification_dataset(
    bytes: &[u8],
) -> Result<SuspensionIdentificationDataset> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_IDENTIFICATION_DATASET_BYTES,
        "suspension identification dataset exceeds byte limit"
    );
    let dataset: SuspensionIdentificationDataset = serde_json::from_slice(bytes)?;
    dataset.validate()?;
    Ok(dataset)
}

/// Builds the deterministic non-physical fixture used to test the data path.
pub fn synthetic_suspension_identification_dataset() -> Result<SuspensionIdentificationDataset> {
    let stiffness_n_per_m = 200_000.0;
    let damping_n_s_per_m = 15_000.0;
    let equilibrium_position_m = -0.061;
    let samples = (0..400)
        .map(|index| {
            let capture_time_s = index as f64 * 0.005;
            let fast_phase = std::f64::consts::TAU * 1.2 * capture_time_s;
            let slow_phase = std::f64::consts::TAU * 0.37 * capture_time_s;
            let position_m = -0.055 + 0.010 * fast_phase.sin() + 0.004 * slow_phase.sin();
            let velocity_m_s = 0.010 * std::f64::consts::TAU * 1.2 * fast_phase.cos()
                + 0.004 * std::f64::consts::TAU * 0.37 * slow_phase.cos();
            let noise_n = ((index * 17 % 11) as f64 - 5.0) * 0.2;
            SuspensionForceSample {
                capture_time_s,
                position_m,
                velocity_m_s,
                force_n: stiffness_n_per_m * (equilibrium_position_m - position_m)
                    - damping_n_s_per_m * velocity_m_s
                    + noise_n,
            }
        })
        .collect();
    let mut dataset = SuspensionIdentificationDataset {
        kind: SUSPENSION_IDENTIFICATION_DATASET_KIND.to_string(),
        schema_version: SUSPENSION_IDENTIFICATION_SCHEMA_VERSION,
        dataset_id: "rne.synthetic.suspension.linear.v1".to_string(),
        source_kind: SuspensionDatasetSourceKind::SyntheticFixture,
        source_description: "deterministic generated contract fixture; not physical data"
            .to_string(),
        force_model: "force_n=k_n_per_m*(equilibrium_m-position_m)-c_n_s_per_m*velocity_m_s"
            .to_string(),
        samples,
        content_digest: String::new(),
    };
    dataset.seal()?;
    dataset.validate()?;
    Ok(dataset)
}

/// Returns the frozen v1 fit/split bounds.
pub fn suspension_identification_spec() -> SuspensionIdentificationSpec {
    SuspensionIdentificationSpec {
        holdout_stride: 5,
        minimum_training_samples: 80,
        minimum_holdout_samples: 20,
        stiffness_bounds_n_per_m: [50_000.0, 500_000.0],
        damping_bounds_n_s_per_m: [500.0, 50_000.0],
        equilibrium_position_bounds_m: [-0.20, 0.10],
        maximum_training_rmse_n: 25.0,
        maximum_holdout_rmse_n: 25.0,
    }
}

/// Fits one verified dataset and binds the exact result to it.
pub fn identify_suspension_dataset(
    dataset: &SuspensionIdentificationDataset,
) -> Result<SuspensionIdentificationEvidence> {
    dataset.validate()?;
    let identification_spec = suspension_identification_spec();
    let result = identify_suspension_strut(identification_spec, &dataset.samples)?;
    let mut evidence = SuspensionIdentificationEvidence {
        kind: SUSPENSION_IDENTIFICATION_RESULT_KIND.to_string(),
        schema_version: SUSPENSION_IDENTIFICATION_SCHEMA_VERSION,
        dataset_content_digest: dataset.content_digest.clone(),
        source_kind: dataset.source_kind,
        physical_measurement: dataset.source_kind.is_physical_measurement(),
        identification_spec,
        result,
        content_digest: String::new(),
    };
    evidence.content_digest = evidence_digest(&evidence)?;
    evidence.validate(dataset)?;
    Ok(evidence)
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

fn dataset_digest(dataset: &SuspensionIdentificationDataset) -> Result<String> {
    let mut canonical = dataset.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn evidence_digest(evidence: &SuspensionIdentificationEvidence) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_digest.clear();
    Ok(fnv1a64(&serde_json::to_vec(&canonical)?))
}

fn fnv1a64(bytes: &[u8]) -> String {
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    format!("fnv1a64:{digest:016x}")
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn synthetic_dataset_is_honest_identifiable_and_repeatable() {
        let dataset = synthetic_suspension_identification_dataset().unwrap();
        assert!(!dataset.source_kind.is_physical_measurement());
        let first = identify_suspension_dataset(&dataset).unwrap();
        let second = identify_suspension_dataset(&dataset).unwrap();
        assert_eq!(first, second);
        assert!(!first.physical_measurement);
        assert!((first.result.stiffness_n_per_m - 200_000.0).abs() < 20.0);
        assert!((first.result.damping_n_s_per_m - 15_000.0).abs() < 2.0);
        assert!(first.result.holdout_rmse_n < 1.0);
    }

    #[test]
    fn decoded_finite_dataset_rejects_overflowing_holdout_prediction() {
        let mut dataset = synthetic_suspension_identification_dataset().unwrap();
        let index = suspension_identification_spec().holdout_stride - 1;
        dataset.samples[index].position_m = 1.0e308;
        dataset.samples[index].velocity_m_s = -1.0e308;
        dataset.seal().unwrap();
        let bytes = serde_json::to_vec(&dataset).unwrap();
        let decoded = decode_suspension_identification_dataset(&bytes).unwrap();
        let error = identify_suspension_dataset(&decoded).unwrap_err();
        assert_eq!(
            error.downcast_ref::<rne_robot::systems::SuspensionIdentificationError>(),
            Some(&rne_robot::systems::SuspensionIdentificationError::ResidualExceeded)
        );
    }

    #[test]
    fn dataset_and_result_tampering_are_rejected() {
        let dataset = synthetic_suspension_identification_dataset().unwrap();
        let evidence = identify_suspension_dataset(&dataset).unwrap();

        let mut tampered_dataset = dataset.clone();
        tampered_dataset.samples[10].force_n += 1.0;
        assert!(tampered_dataset.validate().is_err());

        let mut tampered_evidence = evidence;
        tampered_evidence.physical_measurement = true;
        assert!(tampered_evidence.validate(&dataset).is_err());
    }

    #[test]
    fn bounded_decoder_rejects_unknown_fields_and_oversize() {
        let dataset = synthetic_suspension_identification_dataset().unwrap();
        let mut value = serde_json::to_value(dataset).unwrap();
        value["unknown"] = serde_json::json!(true);
        assert!(decode_suspension_identification_dataset(
            serde_json::to_string(&value).unwrap().as_bytes()
        )
        .is_err());
        assert!(decode_suspension_identification_dataset(&vec![
            b' ';
            MAX_SUSPENSION_IDENTIFICATION_DATASET_BYTES
                + 1
        ])
        .is_err());
    }
}

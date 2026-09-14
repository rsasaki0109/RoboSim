//! Physical load-sweep acquisition, synchronization, and calibration evidence.
//!
//! This is the fourth acquisition manifest required to physically qualify tire profile
//! schema v2 (steady + load sensitivity + longitudinal relaxation + lateral relaxation).
//! Its run-level shape and five-channel signal contract deliberately reuse
//! [`crate::tire_acquisition::TireSignalKind`], [`crate::tire_acquisition::TireSignalEvidence`],
//! and [`crate::tire_acquisition::TireEvidenceFileRef`] because the load-sweep dataset records
//! the exact same longitudinal-slip/lateral-slip/normal-load/longitudinal-force/lateral-force
//! channels as the steady dataset; only the owning dataset run type differs.

use crate::tire_acquisition::{
    valid_git_revision, valid_id, valid_sha256, valid_text, verify_file, RoadFrictionEvidenceKind,
    TireCaptureClockKind, TireEvidenceFileRef, TireRawCaptureFormat, TireSignalEvidence,
    TireSignalKind,
};
use crate::tire_identification::TireDatasetSourceKind;
use crate::tire_load_sensitivity::{
    identify_tire_load_sensitivity_dataset, TireLoadSensitivityDataset,
    TireLoadSensitivityEvidence, TireLoadSensitivityRunData,
};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

/// Stable load-sweep acquisition-manifest artifact discriminator.
pub const TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_KIND: &str =
    "rne_mobility_tire_load_sensitivity_acquisition_manifest";
/// Stable load-sweep physical-qualification artifact discriminator.
pub const TIRE_LOAD_SENSITIVITY_PHYSICAL_QUALIFICATION_KIND: &str =
    "rne_mobility_tire_load_sensitivity_physical_qualification";
/// Current schema shared by the load-sweep acquisition artifact family.
pub const TIRE_LOAD_SENSITIVITY_ACQUISITION_SCHEMA_VERSION: u32 = 1;
/// Maximum accepted serialized load-sweep acquisition manifest size.
pub const MAX_TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_BYTES: usize = 4 * 1024 * 1024;
const MAX_TIMESTAMP_UNCERTAINTY_S: f64 = 0.001;

/// Physical evidence for one complete load-sweep dataset acquisition.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityRunAcquisitionEvidence {
    /// Exact dataset acquisition identity.
    pub acquisition_id: u64,
    /// Exact road/tire condition identity.
    pub condition_id: u64,
    /// Exact road-friction scale copied from the dataset.
    pub road_friction_scale: f64,
    /// Stable tire/wheel installation identity.
    pub tire_id: String,
    /// Clock synchronization mechanism.
    pub clock_kind: TireCaptureClockKind,
    /// True only when every required channel uses the synchronized domain.
    pub all_channels_synchronized: bool,
    /// Conservative upper bound on inter-channel timestamp error.
    pub maximum_timestamp_uncertainty_s: f64,
    /// Raw retained container format.
    pub raw_capture_format: TireRawCaptureFormat,
    /// Content-addressed raw bytes.
    pub raw_capture: TireEvidenceFileRef,
    /// Stable segment identity within the raw container.
    pub raw_segment_id: String,
    /// Independent method used to establish road scale.
    pub road_friction_evidence_kind: RoadFrictionEvidenceKind,
    /// Content-addressed road characterization evidence.
    pub road_friction_artifact: TireEvidenceFileRef,
    /// Exactly five required converted signals in canonical order.
    pub signals: Vec<TireSignalEvidence>,
}

impl TireLoadSensitivityRunAcquisitionEvidence {
    fn validate(&self, dataset_run: &TireLoadSensitivityRunData) -> Result<()> {
        ensure!(
            self.acquisition_id == dataset_run.acquisition_id
                && self.condition_id == dataset_run.condition_id
                && self.road_friction_scale.to_bits() == dataset_run.road_friction_scale.to_bits(),
            "tire load-sensitivity acquisition run binding drift"
        );
        ensure!(
            valid_id(&self.tire_id),
            "invalid tire load-sensitivity installation identity"
        );
        ensure!(
            self.all_channels_synchronized
                && self.maximum_timestamp_uncertainty_s.is_finite()
                && (0.0..=MAX_TIMESTAMP_UNCERTAINTY_S)
                    .contains(&self.maximum_timestamp_uncertainty_s),
            "unqualified tire load-sensitivity timestamp synchronization"
        );
        self.raw_capture.validate()?;
        ensure!(
            valid_id(&self.raw_segment_id),
            "invalid tire load-sensitivity raw segment identity"
        );
        self.road_friction_artifact.validate()?;
        let required = [
            TireSignalKind::LongitudinalSlipRatio,
            TireSignalKind::LateralSlipTangent,
            TireSignalKind::NormalLoad,
            TireSignalKind::LongitudinalForce,
            TireSignalKind::LateralForce,
        ];
        ensure!(
            self.signals.len() == required.len()
                && self.signals.iter().map(|signal| signal.signal).eq(required),
            "physical tire load-sensitivity channels are incomplete or unordered"
        );
        for signal in &self.signals {
            signal.validate()?;
        }
        let reference_rate_hz = self.signals[0].converted_sample_rate_hz;
        ensure!(
            self.signals
                .iter()
                .all(|signal| signal.converted_sample_rate_hz == reference_rate_hz),
            "physical tire load-sensitivity converted sample-rate mismatch"
        );
        Ok(())
    }

    fn artifacts(&self) -> impl Iterator<Item = &TireEvidenceFileRef> {
        std::iter::once(&self.raw_capture)
            .chain(std::iter::once(&self.road_friction_artifact))
            .chain(
                self.signals
                    .iter()
                    .map(|signal| &signal.calibration_artifact),
            )
    }
}

/// Dataset-bound physical acquisition manifest for staged load-sweep identification.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityAcquisitionManifest {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Stable capture campaign identity.
    pub capture_id: String,
    /// Physical source class; synthetic fixtures are forbidden.
    pub source_kind: TireDatasetSourceKind,
    /// Exact converted load-sweep dataset SHA-256.
    pub dataset_content_sha256: String,
    /// Stable vehicle or test-rig identity.
    pub vehicle_id: String,
    /// Stable DAQ/logger hardware identity.
    pub data_logger_id: String,
    /// Exact logger software and version.
    pub logger_software: String,
    /// Content-addressed acquisition and installation procedure.
    pub acquisition_procedure: TireEvidenceFileRef,
    /// One entry for every dataset run, sorted by acquisition ID.
    pub runs: Vec<TireLoadSensitivityRunAcquisitionEvidence>,
    /// Full source commit used for conversion and identification.
    pub rne_commit: String,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireLoadSensitivityAcquisitionManifest {
    /// Recomputes and stores the self-excluding manifest digest.
    pub fn seal(&mut self) -> Result<()> {
        self.content_sha256 = digest_without_hash(self)?;
        Ok(())
    }

    /// Validates metadata and exact binding to a physical-source load-sweep dataset.
    pub fn validate(&self, dataset: &TireLoadSensitivityDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_KIND
                && self.schema_version == TIRE_LOAD_SENSITIVITY_ACQUISITION_SCHEMA_VERSION,
            "tire load-sensitivity acquisition kind/schema drift"
        );
        ensure!(
            dataset.source_kind.is_physical_measurement()
                && self.source_kind == dataset.source_kind,
            "physical tire load-sensitivity source mismatch"
        );
        ensure!(
            self.dataset_content_sha256 == dataset.content_sha256,
            "physical tire load-sensitivity dataset digest mismatch"
        );
        ensure!(
            [
                self.capture_id.as_str(),
                self.vehicle_id.as_str(),
                self.data_logger_id.as_str(),
            ]
            .into_iter()
            .all(valid_id),
            "invalid physical tire load-sensitivity identity"
        );
        ensure!(
            valid_text(&self.logger_software),
            "invalid tire load-sensitivity logger software"
        );
        self.acquisition_procedure.validate()?;
        let dataset_runs = dataset
            .training_runs
            .iter()
            .chain(&dataset.holdout_runs)
            .collect::<Vec<_>>();
        ensure!(
            self.runs.len() == dataset_runs.len()
                && self
                    .runs
                    .windows(2)
                    .all(|pair| pair[0].acquisition_id < pair[1].acquisition_id),
            "physical tire load-sensitivity run coverage is incomplete or unordered"
        );
        let by_id = dataset_runs
            .into_iter()
            .map(|run| (run.acquisition_id, run))
            .collect::<BTreeMap<_, _>>();
        ensure!(
            by_id.len() == self.runs.len(),
            "duplicate tire load-sensitivity dataset acquisition identity"
        );
        for run in &self.runs {
            let dataset_run = by_id
                .get(&run.acquisition_id)
                .context("manifest contains unknown tire load-sensitivity acquisition")?;
            run.validate(dataset_run)?;
        }
        let mut raw_segments = BTreeSet::new();
        ensure!(
            self.runs.iter().all(|run| raw_segments
                .insert((run.raw_capture.path.as_str(), run.raw_segment_id.as_str()))),
            "tire load-sensitivity raw segment reused across acquisitions"
        );
        let mut references = BTreeMap::new();
        for artifact in self.artifacts() {
            if let Some(previous) = references.insert(artifact.path.as_str(), artifact) {
                ensure!(
                    previous == artifact,
                    "conflicting tire load-sensitivity evidence references for {}",
                    artifact.path
                );
            }
        }
        ensure!(valid_git_revision(&self.rne_commit), "invalid RNE commit");
        ensure!(
            valid_sha256(&self.content_sha256) && self.content_sha256 == digest_without_hash(self)?,
            "tire load-sensitivity acquisition manifest digest mismatch"
        );
        Ok(())
    }

    /// Streams and verifies every referenced file under an explicit root.
    pub fn verify_files(
        &self,
        dataset: &TireLoadSensitivityDataset,
        evidence_root: &Path,
    ) -> Result<()> {
        self.validate(dataset)?;
        let canonical_root = evidence_root
            .canonicalize()
            .with_context(|| format!("canonicalize {}", evidence_root.display()))?;
        ensure!(
            canonical_root.is_dir(),
            "tire load-sensitivity evidence root is not a directory"
        );
        let mut paths = BTreeSet::new();
        for artifact in self.artifacts() {
            if paths.insert(artifact.path.as_str()) {
                verify_file(&canonical_root, artifact)?;
            }
        }
        Ok(())
    }

    fn artifacts(&self) -> impl Iterator<Item = &TireEvidenceFileRef> {
        std::iter::once(&self.acquisition_procedure).chain(
            self.runs
                .iter()
                .flat_map(TireLoadSensitivityRunAcquisitionEvidence::artifacts),
        )
    }
}

/// Recomputable proof that load-sensitivity identification and physical acquisition passed
/// together.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireLoadSensitivityPhysicalQualificationEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact converted dataset SHA-256.
    pub dataset_content_sha256: String,
    /// Deterministically recomputed load-sensitivity identification evidence.
    pub identification: TireLoadSensitivityEvidence,
    /// Exact acquisition-manifest SHA-256.
    pub acquisition_manifest_sha256: String,
    /// True only for this joined, successfully verified artifact.
    pub physical_measurement: bool,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireLoadSensitivityPhysicalQualificationEvidence {
    /// Replays identification and streams the complete acquisition evidence set.
    pub fn validate(
        &self,
        dataset: &TireLoadSensitivityDataset,
        manifest: &TireLoadSensitivityAcquisitionManifest,
        evidence_root: &Path,
    ) -> Result<()> {
        ensure!(
            self.kind == TIRE_LOAD_SENSITIVITY_PHYSICAL_QUALIFICATION_KIND
                && self.schema_version == TIRE_LOAD_SENSITIVITY_ACQUISITION_SCHEMA_VERSION,
            "tire load-sensitivity qualification kind/schema drift"
        );
        ensure!(
            self.physical_measurement
                && self.dataset_content_sha256 == dataset.content_sha256
                && self.identification.recorded_source_claim
                && self.acquisition_manifest_sha256 == manifest.content_sha256,
            "tire load-sensitivity qualification binding drift"
        );
        self.identification.validate(dataset)?;
        manifest.verify_files(dataset, evidence_root)?;
        ensure!(
            self.content_sha256 == digest_without_hash(self)?,
            "tire load-sensitivity qualification digest drift"
        );
        Ok(())
    }
}

/// Runs load-sensitivity identification and physical-file verification as one transaction.
pub fn qualify_tire_load_sensitivity_dataset(
    dataset: &TireLoadSensitivityDataset,
    manifest: &TireLoadSensitivityAcquisitionManifest,
    evidence_root: &Path,
) -> Result<TireLoadSensitivityPhysicalQualificationEvidence> {
    manifest.verify_files(dataset, evidence_root)?;
    let identification = identify_tire_load_sensitivity_dataset(dataset)?;
    ensure!(
        identification.recorded_source_claim,
        "tire load-sensitivity qualification requires a recorded source claim"
    );
    let mut evidence = TireLoadSensitivityPhysicalQualificationEvidence {
        kind: TIRE_LOAD_SENSITIVITY_PHYSICAL_QUALIFICATION_KIND.into(),
        schema_version: TIRE_LOAD_SENSITIVITY_ACQUISITION_SCHEMA_VERSION,
        dataset_content_sha256: dataset.content_sha256.clone(),
        identification,
        acquisition_manifest_sha256: manifest.content_sha256.clone(),
        physical_measurement: true,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = digest_without_hash(&evidence)?;
    evidence.validate(dataset, manifest, evidence_root)?;
    Ok(evidence)
}

/// Decodes and validates one bounded manifest against the supplied load-sweep dataset.
pub fn decode_tire_load_sensitivity_acquisition_manifest(
    bytes: &[u8],
    dataset: &TireLoadSensitivityDataset,
) -> Result<TireLoadSensitivityAcquisitionManifest> {
    ensure!(
        bytes.len() <= MAX_TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_BYTES,
        "tire load-sensitivity acquisition manifest exceeds byte limit"
    );
    let manifest: TireLoadSensitivityAcquisitionManifest = serde_json::from_slice(bytes)?;
    manifest.validate(dataset)?;
    Ok(manifest)
}

fn digest_without_hash<T>(value: &T) -> Result<String>
where
    T: Clone + Serialize + ClearContentHash,
{
    let mut canonical = value.clone();
    canonical.clear_content_hash();
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(&canonical)?)
    ))
}

trait ClearContentHash {
    fn clear_content_hash(&mut self);
}

macro_rules! impl_clear_content_hash {
    ($($ty:ty),+ $(,)?) => {$(
        impl ClearContentHash for $ty {
            fn clear_content_hash(&mut self) {
                self.content_sha256.clear();
            }
        }
    )+};
}

impl_clear_content_hash!(
    TireLoadSensitivityAcquisitionManifest,
    TireLoadSensitivityPhysicalQualificationEvidence,
);

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tire_acquisition::{TireCalibrationKind, TireSignalOrigin};
    use crate::tire_identification::identify_tire_dataset;
    use crate::tire_load_sensitivity::synthetic_tire_load_sensitivity_dataset;
    use std::fs;

    fn file_ref(root: &Path, name: &str, bytes: &[u8]) -> TireEvidenceFileRef {
        fs::write(root.join(name), bytes).unwrap();
        TireEvidenceFileRef {
            path: name.to_string(),
            size_bytes: bytes.len() as u64,
            sha256: format!("sha256:{:x}", Sha256::digest(bytes)),
        }
    }

    fn signals(calibration: &TireEvidenceFileRef) -> Vec<TireSignalEvidence> {
        [
            TireSignalKind::LongitudinalSlipRatio,
            TireSignalKind::LateralSlipTangent,
            TireSignalKind::NormalLoad,
            TireSignalKind::LongitudinalForce,
            TireSignalKind::LateralForce,
        ]
        .into_iter()
        .map(|signal| TireSignalEvidence {
            signal,
            sensor_id: format!("sensor.load.{signal:?}"),
            unit: signal.unit().to_string(),
            convention: signal.convention().to_string(),
            origin: TireSignalOrigin::Measured,
            source_sample_rate_hz: if signal == TireSignalKind::LateralSlipTangent {
                400.0
            } else {
                100.0
            },
            converted_sample_rate_hz: 100.0,
            resolution_si: 1.0e-6,
            expanded_uncertainty_si: 1.0e-4,
            calibration_kind: TireCalibrationKind::Iso17025,
            calibration_artifact: calibration.clone(),
        })
        .collect()
    }

    fn physical_fixture(
        root: &Path,
    ) -> (
        TireLoadSensitivityDataset,
        TireLoadSensitivityAcquisitionManifest,
    ) {
        let mut dataset = synthetic_tire_load_sensitivity_dataset().unwrap();
        dataset.steady_dataset.source_kind = TireDatasetSourceKind::RecordedBench;
        dataset.steady_dataset.source_description =
            "test-only recorded bench label; not real evidence".into();
        dataset.steady_dataset.seal().unwrap();
        dataset.steady_identification = identify_tire_dataset(&dataset.steady_dataset).unwrap();
        dataset.source_kind = TireDatasetSourceKind::RecordedBench;
        dataset.source_description =
            "test-only load-sweep bench-shaped fixture; not real evidence".into();
        dataset.seal().unwrap();
        let raw = file_ref(root, "load_capture.mcap", b"test raw load-sweep capture");
        let calibration = file_ref(
            root,
            "load_calibration.txt",
            b"test load calibration record",
        );
        let road = file_ref(root, "load_road.txt", b"test load road characterization");
        let procedure = file_ref(
            root,
            "load_procedure.txt",
            b"test load acquisition procedure",
        );
        let mut runs = dataset
            .training_runs
            .iter()
            .chain(&dataset.holdout_runs)
            .map(|run| TireLoadSensitivityRunAcquisitionEvidence {
                acquisition_id: run.acquisition_id,
                condition_id: run.condition_id,
                road_friction_scale: run.road_friction_scale,
                tire_id: "tire.front_left.serial_1".into(),
                clock_kind: TireCaptureClockKind::SharedHardware,
                all_channels_synchronized: true,
                maximum_timestamp_uncertainty_s: 0.0001,
                raw_capture_format: TireRawCaptureFormat::Mcap,
                raw_capture: raw.clone(),
                raw_segment_id: format!("load.segment.{}", run.acquisition_id),
                road_friction_evidence_kind: RoadFrictionEvidenceKind::CalibratedBenchSurface,
                road_friction_artifact: road.clone(),
                signals: signals(&calibration),
            })
            .collect::<Vec<_>>();
        runs.sort_by_key(|run| run.acquisition_id);
        let mut manifest = TireLoadSensitivityAcquisitionManifest {
            kind: TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_KIND.into(),
            schema_version: TIRE_LOAD_SENSITIVITY_ACQUISITION_SCHEMA_VERSION,
            capture_id: "campaign.tire.load.test.1".into(),
            source_kind: TireDatasetSourceKind::RecordedBench,
            dataset_content_sha256: dataset.content_sha256.clone(),
            vehicle_id: "rig.tire.load.test.1".into(),
            data_logger_id: "logger.load.test.1".into(),
            logger_software: "test load logger 1.0".into(),
            acquisition_procedure: procedure,
            runs,
            rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
            content_sha256: String::new(),
        };
        manifest.seal().unwrap();
        (dataset, manifest)
    }

    #[test]
    fn physical_manifest_binds_every_run_and_streams_files() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, manifest) = physical_fixture(root.path());
        manifest.validate(&dataset).unwrap();
        manifest.verify_files(&dataset, root.path()).unwrap();
        let qualification =
            qualify_tire_load_sensitivity_dataset(&dataset, &manifest, root.path()).unwrap();
        assert!(qualification.physical_measurement);
        qualification
            .validate(&dataset, &manifest, root.path())
            .unwrap();
        let bytes = serde_json::to_vec(&manifest).unwrap();
        assert_eq!(
            decode_tire_load_sensitivity_acquisition_manifest(&bytes, &dataset).unwrap(),
            manifest
        );
    }

    #[test]
    fn repeated_qualification_is_deterministic() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, manifest) = physical_fixture(root.path());
        let first =
            qualify_tire_load_sensitivity_dataset(&dataset, &manifest, root.path()).unwrap();
        let second =
            qualify_tire_load_sensitivity_dataset(&dataset, &manifest, root.path()).unwrap();
        assert_eq!(first, second);
    }

    #[test]
    fn synthetic_incomplete_and_run_drift_never_qualify() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, manifest) = physical_fixture(root.path());

        let mut synthetic = dataset.clone();
        synthetic.source_kind = TireDatasetSourceKind::SyntheticFixture;
        synthetic.seal().unwrap();
        assert!(manifest.validate(&synthetic).is_err());

        let mut incomplete = manifest.clone();
        incomplete.runs[0].signals.pop();
        incomplete.seal().unwrap();
        assert!(incomplete.validate(&dataset).is_err());

        let mut missing_run = manifest.clone();
        missing_run.runs.pop();
        missing_run.seal().unwrap();
        assert!(missing_run.validate(&dataset).is_err());

        let mut duplicate_run = manifest.clone();
        duplicate_run.runs.push(duplicate_run.runs[0].clone());
        duplicate_run.runs.sort_by_key(|run| run.acquisition_id);
        duplicate_run.seal().unwrap();
        assert!(duplicate_run.validate(&dataset).is_err());

        let mut drifted = manifest.clone();
        drifted.runs[1].road_friction_scale += 0.01;
        drifted.seal().unwrap();
        assert!(drifted.validate(&dataset).is_err());

        let (_, mut reused_segment) = physical_fixture(root.path());
        reused_segment.runs[1].raw_segment_id = reused_segment.runs[0].raw_segment_id.clone();
        reused_segment.seal().unwrap();
        assert!(reused_segment.validate(&dataset).is_err());

        let (_, mut invalid_derivation) = physical_fixture(root.path());
        invalid_derivation.runs[0].signals[0].origin = TireSignalOrigin::Derived;
        invalid_derivation.seal().unwrap();
        assert!(invalid_derivation.validate(&dataset).is_err());

        let (_, mut upsampled) = physical_fixture(root.path());
        upsampled.runs[0].signals[0].source_sample_rate_hz = 50.0;
        upsampled.seal().unwrap();
        assert!(upsampled.validate(&dataset).is_err());

        let mut wrong_dataset_digest = manifest;
        wrong_dataset_digest.dataset_content_sha256 = "sha256:00".to_string() + &"0".repeat(62);
        wrong_dataset_digest.seal().unwrap();
        assert!(wrong_dataset_digest.validate(&dataset).is_err());
    }

    #[test]
    fn path_escape_is_rejected() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, mut manifest) = physical_fixture(root.path());
        manifest.runs[0].raw_capture.path = "../escape.mcap".into();
        manifest.seal().unwrap();
        assert!(manifest.validate(&dataset).is_err());
    }

    #[test]
    fn file_tampering_unknown_fields_and_oversize_are_rejected() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, manifest) = physical_fixture(root.path());
        fs::write(
            root.path().join("load_capture.mcap"),
            b"changed raw load-sweep capture",
        )
        .unwrap();
        assert!(manifest.verify_files(&dataset, root.path()).is_err());

        let mut value = serde_json::to_value(&manifest).unwrap();
        value["unknown"] = serde_json::json!(true);
        assert!(decode_tire_load_sensitivity_acquisition_manifest(
            serde_json::to_string(&value).unwrap().as_bytes(),
            &dataset
        )
        .is_err());
        assert!(decode_tire_load_sensitivity_acquisition_manifest(
            &vec![b' '; MAX_TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_BYTES + 1],
            &dataset
        )
        .is_err());
    }

    #[test]
    fn retained_file_one_byte_tamper_invalidates_qualification() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, manifest) = physical_fixture(root.path());
        let qualification =
            qualify_tire_load_sensitivity_dataset(&dataset, &manifest, root.path()).unwrap();
        let calibration_path = root.path().join("load_calibration.txt");
        let mut bytes = fs::read(&calibration_path).unwrap();
        bytes[0] ^= 0x01;
        fs::write(&calibration_path, bytes).unwrap();
        assert!(qualification
            .validate(&dataset, &manifest, root.path())
            .is_err());
    }

    #[test]
    fn qualification_digest_tamper_is_rejected() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, manifest) = physical_fixture(root.path());
        let mut qualification =
            qualify_tire_load_sensitivity_dataset(&dataset, &manifest, root.path()).unwrap();
        // Flip one hex digit of the self-excluding digest itself; every other bound field
        // (including the recomputed identification result) stays exactly as qualified.
        let last = qualification.content_sha256.pop().unwrap();
        qualification
            .content_sha256
            .push(if last == '0' { '1' } else { '0' });
        assert!(qualification
            .validate(&dataset, &manifest, root.path())
            .is_err());
    }
}

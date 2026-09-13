//! Physical tire-force acquisition, synchronization, and calibration evidence.

use crate::tire_identification::{
    identify_tire_dataset, TireDatasetSourceKind, TireIdentificationDataset,
    TireIdentificationEvidence, TireIdentificationRunData,
};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

/// Stable artifact discriminator.
pub const TIRE_ACQUISITION_MANIFEST_KIND: &str = "rne_mobility_tire_acquisition_manifest";
/// Physical tire acquisition schema.
pub const TIRE_ACQUISITION_SCHEMA_VERSION: u32 = 1;
/// Stable physical-qualification artifact discriminator.
pub const TIRE_PHYSICAL_QUALIFICATION_KIND: &str = "rne_mobility_tire_physical_qualification";
/// Maximum accepted serialized manifest size.
pub const MAX_TIRE_ACQUISITION_MANIFEST_BYTES: usize = 4 * 1024 * 1024;
/// Maximum size of one referenced immutable artifact.
pub const MAX_TIRE_ACQUISITION_ARTIFACT_BYTES: u64 = 64 * 1024 * 1024 * 1024;
const MAX_TIMESTAMP_UNCERTAINTY_S: f64 = 0.001;

/// Retained raw measurement container.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireRawCaptureFormat {
    /// ASAM MDF 4.x measurement container.
    Mdf4,
    /// MCAP robotics message container.
    Mcap,
    /// ROS bag 2 storage.
    Rosbag2,
    /// Documented immutable CSV export.
    Csv,
}

/// Clock relationship shared by every channel in one acquisition.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireCaptureClockKind {
    /// All channels share one hardware clock and trigger domain.
    SharedHardware,
    /// IEEE 1588 PTP synchronized hardware clocks.
    PtpIeee1588,
    /// Hardware timestamps disciplined by GNSS.
    GnssDisciplined,
}

/// Evidence class used to establish the declared road-friction scale.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RoadFrictionEvidenceKind {
    /// Instrumented reference tire or force transducer.
    InstrumentedReferenceTire,
    /// Calibrated continuous-friction or locked-wheel trailer.
    CalibratedFrictionTrailer,
    /// Calibrated tire test bench surface.
    CalibratedBenchSurface,
}

/// Required signal identity after conversion to the identifier convention.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireSignalKind {
    /// Positive driven-wheel slip ratio.
    LongitudinalSlipRatio,
    /// Negative carrier lateral velocity divided by transport speed.
    LateralSlipTangent,
    /// Contact-normal load on the wheel.
    NormalLoad,
    /// Tire force along positive wheel-forward.
    LongitudinalForce,
    /// Tire force along positive wheel-lateral.
    LateralForce,
}

impl TireSignalKind {
    /// Returns the exact SI-unit symbol required in artifact metadata.
    pub fn unit(self) -> &'static str {
        match self {
            Self::LongitudinalSlipRatio | Self::LateralSlipTangent => "1",
            Self::NormalLoad | Self::LongitudinalForce | Self::LateralForce => "N",
        }
    }

    /// Returns the exact positive-axis convention required by RNE's tire law.
    pub fn convention(self) -> &'static str {
        match self {
            Self::LongitudinalSlipRatio => "positive_wheel_surface_speed_minus_carrier_forward",
            Self::LateralSlipTangent => "negative_carrier_lateral_velocity",
            Self::NormalLoad => "positive_contact_normal_on_wheel",
            Self::LongitudinalForce => "positive_wheel_forward",
            Self::LateralForce => "positive_wheel_lateral",
        }
    }
}

/// Whether a converted channel was measured or deterministically reconstructed.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireSignalOrigin {
    /// Direct calibrated transducer output.
    Measured,
    /// Deterministic reconstruction from retained raw inputs.
    Derived,
}

/// Traceability class for a calibration or derivation record.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireCalibrationKind {
    /// ISO/IEC 17025 accredited calibration.
    Iso17025,
    /// Calibration traceable to a national metrology institute.
    NationalMetrologyTraceable,
    /// Retained in-situ zero/span/shunt verification.
    InSituVerification,
    /// Versioned deterministic reconstruction procedure.
    DerivedSignalProcedure,
}

/// Immutable file reference relative to an explicitly supplied evidence root.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireEvidenceFileRef {
    /// Canonical slash-separated relative path.
    pub path: String,
    /// Exact file length.
    pub size_bytes: u64,
    /// Lowercase SHA-256 with a `sha256:` prefix.
    pub sha256: String,
}

impl TireEvidenceFileRef {
    /// Validates portable path, bounded size, and digest syntax.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            valid_relative_path(&self.path),
            "invalid tire evidence path"
        );
        ensure!(
            self.size_bytes > 0 && self.size_bytes <= MAX_TIRE_ACQUISITION_ARTIFACT_BYTES,
            "invalid tire evidence file size"
        );
        ensure!(valid_sha256(&self.sha256), "invalid tire evidence SHA-256");
        Ok(())
    }
}

/// One unit-, convention-, uncertainty-, and calibration-bound signal.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireSignalEvidence {
    /// Canonical signal identity; entries are strictly ordered by this field.
    pub signal: TireSignalKind,
    /// Stable installed transducer or derived-channel identity.
    pub sensor_id: String,
    /// Exact required SI unit.
    pub unit: String,
    /// Exact RNE positive-axis convention.
    pub convention: String,
    /// Direct measurement or deterministic reconstruction.
    pub origin: TireSignalOrigin,
    /// Nominal raw source sampling rate.
    pub source_sample_rate_hz: f64,
    /// Common rate of the converted identification rows.
    pub converted_sample_rate_hz: f64,
    /// Smallest declared output increment in SI units.
    pub resolution_si: f64,
    /// Expanded uncertainty in the same SI unit.
    pub expanded_uncertainty_si: f64,
    /// Calibration or derivation traceability class.
    pub calibration_kind: TireCalibrationKind,
    /// Content-addressed calibration or derivation record.
    pub calibration_artifact: TireEvidenceFileRef,
}

impl TireSignalEvidence {
    fn validate(&self) -> Result<()> {
        ensure!(valid_id(&self.sensor_id), "invalid tire sensor identity");
        ensure!(self.unit == self.signal.unit(), "tire signal unit drift");
        ensure!(
            self.convention == self.signal.convention(),
            "tire signal convention drift"
        );
        ensure!(
            self.source_sample_rate_hz.is_finite()
                && (1.0..=100_000.0).contains(&self.source_sample_rate_hz)
                && self.converted_sample_rate_hz.is_finite()
                && (1.0..=self.source_sample_rate_hz).contains(&self.converted_sample_rate_hz),
            "invalid tire source or converted signal rate"
        );
        ensure!(
            self.resolution_si.is_finite() && self.resolution_si > 0.0,
            "invalid tire signal resolution"
        );
        ensure!(
            self.expanded_uncertainty_si.is_finite() && self.expanded_uncertainty_si >= 0.0,
            "invalid tire signal uncertainty"
        );
        ensure!(
            (self.origin == TireSignalOrigin::Derived)
                == (self.calibration_kind == TireCalibrationKind::DerivedSignalProcedure),
            "derived tire signal calibration mismatch"
        );
        self.calibration_artifact.validate()
    }
}

/// Physical evidence for one complete dataset acquisition.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRunAcquisitionEvidence {
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

impl TireRunAcquisitionEvidence {
    fn validate(&self, dataset_run: &TireIdentificationRunData) -> Result<()> {
        ensure!(
            self.acquisition_id == dataset_run.acquisition_id
                && self.condition_id == dataset_run.condition_id
                && self.road_friction_scale.to_bits() == dataset_run.road_friction_scale.to_bits(),
            "tire acquisition run binding drift"
        );
        ensure!(
            valid_id(&self.tire_id),
            "invalid tire installation identity"
        );
        ensure!(
            self.all_channels_synchronized
                && self.maximum_timestamp_uncertainty_s.is_finite()
                && (0.0..=MAX_TIMESTAMP_UNCERTAINTY_S)
                    .contains(&self.maximum_timestamp_uncertainty_s),
            "unqualified tire timestamp synchronization"
        );
        self.raw_capture.validate()?;
        ensure!(
            valid_id(&self.raw_segment_id),
            "invalid tire raw segment identity"
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
            "physical tire channels are incomplete or unordered"
        );
        for signal in &self.signals {
            signal.validate()?;
        }
        let reference_rate_hz = self.signals[0].converted_sample_rate_hz;
        ensure!(
            self.signals
                .iter()
                .all(|signal| signal.converted_sample_rate_hz == reference_rate_hz),
            "physical tire converted sample-rate mismatch"
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

/// Dataset-bound physical acquisition manifest.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TirePhysicalAcquisitionManifest {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Stable capture campaign identity.
    pub capture_id: String,
    /// Physical source class; synthetic fixtures are forbidden.
    pub source_kind: TireDatasetSourceKind,
    /// Exact converted dataset SHA-256.
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
    pub runs: Vec<TireRunAcquisitionEvidence>,
    /// Full source commit used for conversion and identification.
    pub rne_commit: String,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TirePhysicalAcquisitionManifest {
    /// Recomputes and stores the self-excluding manifest digest.
    pub fn seal(&mut self) -> Result<()> {
        self.content_sha256 = self.computed_content_sha256()?;
        Ok(())
    }

    /// Returns the self-excluding manifest digest.
    pub fn computed_content_sha256(&self) -> Result<String> {
        let mut canonical = self.clone();
        canonical.content_sha256.clear();
        Ok(format!(
            "sha256:{:x}",
            Sha256::digest(serde_json::to_vec(&canonical)?)
        ))
    }

    /// Validates metadata and exact binding to a physical-source dataset.
    pub fn validate(&self, dataset: &TireIdentificationDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == TIRE_ACQUISITION_MANIFEST_KIND
                && self.schema_version == TIRE_ACQUISITION_SCHEMA_VERSION,
            "tire acquisition kind/schema drift"
        );
        ensure!(
            dataset.source_kind.is_physical_measurement()
                && self.source_kind == dataset.source_kind,
            "physical tire source mismatch"
        );
        ensure!(
            self.dataset_content_sha256 == dataset.content_sha256,
            "physical tire dataset digest mismatch"
        );
        ensure!(
            [
                self.capture_id.as_str(),
                self.vehicle_id.as_str(),
                self.data_logger_id.as_str(),
            ]
            .into_iter()
            .all(valid_id),
            "invalid physical tire identity"
        );
        ensure!(
            valid_text(&self.logger_software),
            "invalid tire logger software"
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
            "physical tire run coverage is incomplete or unordered"
        );
        let by_id = dataset_runs
            .into_iter()
            .map(|run| (run.acquisition_id, run))
            .collect::<BTreeMap<_, _>>();
        ensure!(
            by_id.len() == self.runs.len(),
            "duplicate tire dataset acquisition identity"
        );
        for run in &self.runs {
            let dataset_run = by_id
                .get(&run.acquisition_id)
                .context("manifest contains unknown tire acquisition")?;
            run.validate(dataset_run)?;
        }
        let mut raw_segments = BTreeSet::new();
        ensure!(
            self.runs.iter().all(|run| raw_segments
                .insert((run.raw_capture.path.as_str(), run.raw_segment_id.as_str()))),
            "tire raw segment reused across acquisitions"
        );
        let mut references = BTreeMap::new();
        for artifact in self.artifacts() {
            if let Some(previous) = references.insert(artifact.path.as_str(), artifact) {
                ensure!(
                    previous == artifact,
                    "conflicting tire evidence references for {}",
                    artifact.path
                );
            }
        }
        ensure!(valid_git_revision(&self.rne_commit), "invalid RNE commit");
        ensure!(
            valid_sha256(&self.content_sha256)
                && self.content_sha256 == self.computed_content_sha256()?,
            "tire acquisition manifest digest mismatch"
        );
        Ok(())
    }

    /// Streams and verifies every referenced file under an explicit root.
    pub fn verify_files(
        &self,
        dataset: &TireIdentificationDataset,
        evidence_root: &Path,
    ) -> Result<()> {
        self.validate(dataset)?;
        let canonical_root = evidence_root
            .canonicalize()
            .with_context(|| format!("canonicalize {}", evidence_root.display()))?;
        ensure!(
            canonical_root.is_dir(),
            "tire evidence root is not a directory"
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
                .flat_map(TireRunAcquisitionEvidence::artifacts),
        )
    }
}

/// Recomputable proof that identification and physical acquisition passed together.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TirePhysicalQualificationEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact converted dataset SHA-256.
    pub dataset_content_sha256: String,
    /// Deterministically recomputed identification evidence.
    pub identification: TireIdentificationEvidence,
    /// Exact acquisition-manifest SHA-256.
    pub acquisition_manifest_sha256: String,
    /// True only for this joined, successfully verified artifact.
    pub physical_measurement: bool,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TirePhysicalQualificationEvidence {
    /// Replays identification and streams the complete acquisition evidence set.
    pub fn validate(
        &self,
        dataset: &TireIdentificationDataset,
        manifest: &TirePhysicalAcquisitionManifest,
        evidence_root: &Path,
    ) -> Result<()> {
        ensure!(
            self.kind == TIRE_PHYSICAL_QUALIFICATION_KIND
                && self.schema_version == TIRE_ACQUISITION_SCHEMA_VERSION,
            "tire qualification kind/schema drift"
        );
        ensure!(
            self.physical_measurement
                && self.dataset_content_sha256 == dataset.content_sha256
                && self.identification.recorded_source_claim
                && self.acquisition_manifest_sha256 == manifest.content_sha256,
            "tire qualification binding drift"
        );
        self.identification.validate(dataset)?;
        manifest.verify_files(dataset, evidence_root)?;
        ensure!(
            self.content_sha256 == qualification_digest(self)?,
            "tire qualification digest drift"
        );
        Ok(())
    }
}

/// Runs identification and physical-file verification as one qualification transaction.
pub fn qualify_tire_dataset(
    dataset: &TireIdentificationDataset,
    manifest: &TirePhysicalAcquisitionManifest,
    evidence_root: &Path,
) -> Result<TirePhysicalQualificationEvidence> {
    manifest.verify_files(dataset, evidence_root)?;
    let identification = identify_tire_dataset(dataset)?;
    ensure!(
        identification.recorded_source_claim,
        "tire qualification requires a recorded source claim"
    );
    let mut evidence = TirePhysicalQualificationEvidence {
        kind: TIRE_PHYSICAL_QUALIFICATION_KIND.into(),
        schema_version: TIRE_ACQUISITION_SCHEMA_VERSION,
        dataset_content_sha256: dataset.content_sha256.clone(),
        identification,
        acquisition_manifest_sha256: manifest.content_sha256.clone(),
        physical_measurement: true,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = qualification_digest(&evidence)?;
    evidence.validate(dataset, manifest, evidence_root)?;
    Ok(evidence)
}

/// Decodes and validates one bounded manifest against the supplied dataset.
pub fn decode_tire_acquisition_manifest(
    bytes: &[u8],
    dataset: &TireIdentificationDataset,
) -> Result<TirePhysicalAcquisitionManifest> {
    ensure!(
        bytes.len() <= MAX_TIRE_ACQUISITION_MANIFEST_BYTES,
        "tire acquisition manifest exceeds byte limit"
    );
    let manifest: TirePhysicalAcquisitionManifest = serde_json::from_slice(bytes)?;
    manifest.validate(dataset)?;
    Ok(manifest)
}

pub(crate) fn verify_file(root: &Path, artifact: &TireEvidenceFileRef) -> Result<()> {
    artifact.validate()?;
    let candidate = root.join(PathBuf::from(&artifact.path));
    let canonical = candidate
        .canonicalize()
        .with_context(|| format!("canonicalize {}", candidate.display()))?;
    ensure!(
        canonical.starts_with(root),
        "tire evidence path escaped root"
    );
    let file = File::open(&canonical).with_context(|| format!("open {}", canonical.display()))?;
    let metadata = file
        .metadata()
        .with_context(|| format!("inspect {}", canonical.display()))?;
    ensure!(
        metadata.is_file() && metadata.len() == artifact.size_bytes,
        "tire evidence file size mismatch"
    );
    let mut reader = file.take(artifact.size_bytes + 1);
    let mut count = 0_u64;
    let mut hasher = Sha256::new();
    let mut buffer = vec![0_u8; 64 * 1024];
    loop {
        let read = reader.read(&mut buffer)?;
        if read == 0 {
            break;
        }
        count += read as u64;
        ensure!(count <= artifact.size_bytes, "tire evidence file grew");
        hasher.update(&buffer[..read]);
    }
    ensure!(count == artifact.size_bytes, "tire evidence file shrank");
    ensure!(
        format!("sha256:{:x}", hasher.finalize()) == artifact.sha256,
        "tire evidence file SHA-256 mismatch"
    );
    Ok(())
}

pub(crate) fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

pub(crate) fn valid_text(value: &str) -> bool {
    !value.is_empty() && value.len() <= 1_024 && !value.chars().any(char::is_control)
}

fn valid_relative_path(value: &str) -> bool {
    valid_text(value)
        && !value.contains('\\')
        && !value.starts_with('/')
        && !value.contains(':')
        && !value
            .split('/')
            .any(|component| component.is_empty() || matches!(component, "." | ".."))
}

pub(crate) fn valid_sha256(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

pub(crate) fn valid_git_revision(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

fn qualification_digest(evidence: &TirePhysicalQualificationEvidence) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_sha256.clear();
    Ok(format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(&canonical)?)
    ))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::tire_identification::synthetic_tire_identification_dataset;
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
            sensor_id: format!("sensor.{signal:?}"),
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
    ) -> (TireIdentificationDataset, TirePhysicalAcquisitionManifest) {
        let mut dataset = synthetic_tire_identification_dataset().unwrap();
        dataset.source_kind = TireDatasetSourceKind::RecordedBench;
        dataset.source_description = "test-only bench-shaped fixture; not real evidence".into();
        dataset.seal().unwrap();
        let raw = file_ref(root, "capture.mcap", b"test raw tire capture");
        let calibration = file_ref(root, "calibration.txt", b"test calibration record");
        let road = file_ref(root, "road.txt", b"test road characterization");
        let procedure = file_ref(root, "procedure.txt", b"test acquisition procedure");
        let mut runs = dataset
            .training_runs
            .iter()
            .chain(&dataset.holdout_runs)
            .map(|run| TireRunAcquisitionEvidence {
                acquisition_id: run.acquisition_id,
                condition_id: run.condition_id,
                road_friction_scale: run.road_friction_scale,
                tire_id: "tire.front_left.serial_1".into(),
                clock_kind: TireCaptureClockKind::SharedHardware,
                all_channels_synchronized: true,
                maximum_timestamp_uncertainty_s: 0.0001,
                raw_capture_format: TireRawCaptureFormat::Mcap,
                raw_capture: raw.clone(),
                raw_segment_id: format!("segment.{}", run.acquisition_id),
                road_friction_evidence_kind: RoadFrictionEvidenceKind::CalibratedBenchSurface,
                road_friction_artifact: road.clone(),
                signals: signals(&calibration),
            })
            .collect::<Vec<_>>();
        runs.sort_by_key(|run| run.acquisition_id);
        let mut manifest = TirePhysicalAcquisitionManifest {
            kind: TIRE_ACQUISITION_MANIFEST_KIND.into(),
            schema_version: TIRE_ACQUISITION_SCHEMA_VERSION,
            capture_id: "campaign.tire.test.1".into(),
            source_kind: TireDatasetSourceKind::RecordedBench,
            dataset_content_sha256: dataset.content_sha256.clone(),
            vehicle_id: "rig.tire.test.1".into(),
            data_logger_id: "logger.test.1".into(),
            logger_software: "test logger 1.0".into(),
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
        let qualification = qualify_tire_dataset(&dataset, &manifest, root.path()).unwrap();
        assert!(qualification.physical_measurement);
        qualification
            .validate(&dataset, &manifest, root.path())
            .unwrap();
        let bytes = serde_json::to_vec(&manifest).unwrap();
        assert_eq!(
            decode_tire_acquisition_manifest(&bytes, &dataset).unwrap(),
            manifest
        );
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

        let mut drifted = manifest;
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
    }

    #[test]
    fn file_tampering_unknown_fields_and_oversize_are_rejected() {
        let root = tempfile::tempdir().unwrap();
        let (dataset, manifest) = physical_fixture(root.path());
        fs::write(
            root.path().join("capture.mcap"),
            b"changed raw tire capture",
        )
        .unwrap();
        assert!(manifest.verify_files(&dataset, root.path()).is_err());

        let mut value = serde_json::to_value(&manifest).unwrap();
        value["unknown"] = serde_json::json!(true);
        assert!(decode_tire_acquisition_manifest(
            serde_json::to_string(&value).unwrap().as_bytes(),
            &dataset
        )
        .is_err());
        assert!(decode_tire_acquisition_manifest(
            &vec![b' '; MAX_TIRE_ACQUISITION_MANIFEST_BYTES + 1],
            &dataset
        )
        .is_err());
    }
}

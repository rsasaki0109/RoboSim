//! Physical suspension-log acquisition and calibration evidence contract.

use crate::suspension_identification::{
    SuspensionDatasetSourceKind, SuspensionIdentificationDataset,
};
use anyhow::{ensure, Context, Result};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::fs::File;
use std::io::Read;
use std::path::{Path, PathBuf};

/// Stable artifact discriminator for a physical suspension capture manifest.
pub const SUSPENSION_ACQUISITION_MANIFEST_KIND: &str =
    "rne_mobility_suspension_acquisition_manifest";
/// Physical suspension capture manifest schema.
pub const SUSPENSION_ACQUISITION_SCHEMA_VERSION: u32 = 1;
/// Maximum serialized acquisition-manifest size accepted by the CLI.
pub const MAX_SUSPENSION_ACQUISITION_MANIFEST_BYTES: usize = 1024 * 1024;
/// Maximum size of any referenced raw or calibration artifact.
pub const MAX_SUSPENSION_ACQUISITION_ARTIFACT_BYTES: u64 = 64 * 1024 * 1024 * 1024;
const MAX_TIMESTAMP_UNCERTAINTY_S: f64 = 0.001;

/// Container used for the retained raw measurement stream.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionRawCaptureFormat {
    /// ASAM MDF 4.x measurement data.
    Mdf4,
    /// MCAP robotics message container.
    Mcap,
    /// A documented, immutable CSV export.
    Csv,
}

/// Synchronization source shared by the required channels.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionCaptureClockKind {
    /// Channels are sampled from one hardware clock and trigger domain.
    SharedHardware,
    /// IEEE 1588 PTP synchronized hardware clocks.
    PtpIeee1588,
    /// Hardware timestamps disciplined by a GNSS time source.
    GnssDisciplined,
}

/// Required physical or derived signal identity.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionSignalKind {
    /// Strut coordinate along the declared positive axis.
    Position,
    /// Time derivative of the strut coordinate.
    Velocity,
    /// Generalized strut force along the declared positive axis.
    Force,
}

impl SuspensionSignalKind {
    fn unit(self) -> &'static str {
        match self {
            Self::Position => "m",
            Self::Velocity => "m/s",
            Self::Force => "N",
        }
    }
}

/// Whether a channel was directly measured or deterministically derived.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionSignalOrigin {
    /// Directly sampled transducer output.
    Measured,
    /// Deterministic processing of another retained channel.
    Derived,
}

/// Calibration traceability class declared by the retained certificate or procedure.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SuspensionCalibrationKind {
    /// Calibration performed by an ISO/IEC 17025 accredited laboratory.
    Iso17025,
    /// Calibration traceable to a national metrology institute.
    NationalMetrologyTraceable,
    /// In-situ bridge shunt/zero verification retained with the capture.
    InSituShunt,
    /// Deterministic derivation procedure for a calculated channel.
    DerivedSignalProcedure,
}

/// Immutable file reference verified relative to an explicitly supplied evidence root.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionEvidenceFileRef {
    /// Canonical slash-separated relative path.
    pub path: String,
    /// Exact file length.
    pub size_bytes: u64,
    /// Lowercase SHA-256 with a `sha256:` prefix.
    pub sha256: String,
}

impl SuspensionEvidenceFileRef {
    /// Verifies a standalone retained calibration file under an explicit root.
    /// Enforces the same path confinement, regular-file, exact-size and streaming
    /// SHA-256 checks as acquisition manifests. Byte identity is not authenticity.
    pub fn verify(&self, evidence_root: &Path) -> Result<()> {
        self.validate()?;
        let root = evidence_root
            .canonicalize()
            .with_context(|| format!("canonicalize {}", evidence_root.display()))?;
        ensure!(root.is_dir(), "suspension evidence root is not a directory");
        verify_file(&root, self)
    }

    /// Validates portable path, bounded size, and digest syntax without reading the file.
    pub fn validate(&self) -> Result<()> {
        ensure!(valid_relative_path(&self.path), "invalid evidence path");
        ensure!(
            self.size_bytes > 0 && self.size_bytes <= MAX_SUSPENSION_ACQUISITION_ARTIFACT_BYTES,
            "invalid evidence file size"
        );
        ensure!(valid_sha256(&self.sha256), "invalid evidence SHA-256");
        Ok(())
    }
}

/// One unit-, direction-, uncertainty-, and calibration-bound input channel.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionSignalEvidence {
    /// Canonical signal identity; entries must be sorted by this field.
    pub signal: SuspensionSignalKind,
    /// Stable installed transducer or derived-channel identity.
    pub sensor_id: String,
    /// Exact SI unit required for this signal.
    pub unit: String,
    /// Direct measurement or deterministic derivation.
    pub origin: SuspensionSignalOrigin,
    /// Positive values point along the strut axis used by `SuspensionStrutSpec`.
    pub positive_along_strut_axis: bool,
    /// Nominal sample rate.
    pub sample_rate_hz: f64,
    /// Smallest declared output increment in SI units.
    pub resolution_si: f64,
    /// Expanded measurement uncertainty in the same SI unit.
    pub expanded_uncertainty_si: f64,
    /// Traceability class of the retained calibration or derivation record.
    pub calibration_kind: SuspensionCalibrationKind,
    /// Content-addressed certificate, shunt result, or derivation procedure.
    pub calibration_artifact: SuspensionEvidenceFileRef,
}

impl SuspensionSignalEvidence {
    fn validate(&self) -> Result<()> {
        ensure!(valid_id(&self.sensor_id), "invalid suspension sensor ID");
        ensure!(
            self.unit == self.signal.unit(),
            "suspension signal unit drift"
        );
        ensure!(
            self.positive_along_strut_axis,
            "suspension signal direction is not canonical"
        );
        ensure!(
            self.sample_rate_hz.is_finite() && (1.0..=100_000.0).contains(&self.sample_rate_hz),
            "invalid suspension sample rate"
        );
        ensure!(
            self.resolution_si.is_finite() && self.resolution_si > 0.0,
            "invalid suspension signal resolution"
        );
        ensure!(
            self.expanded_uncertainty_si.is_finite() && self.expanded_uncertainty_si >= 0.0,
            "invalid suspension signal uncertainty"
        );
        ensure!(
            (self.origin == SuspensionSignalOrigin::Derived)
                == (self.calibration_kind == SuspensionCalibrationKind::DerivedSignalProcedure),
            "derived signal calibration class mismatch"
        );
        self.calibration_artifact.validate()
    }
}

/// Content-bound acquisition metadata required before a log can qualify as physical evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SuspensionPhysicalAcquisitionManifest {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Stable capture identity.
    pub capture_id: String,
    /// Physical source class; synthetic fixtures are forbidden.
    pub source_kind: SuspensionDatasetSourceKind,
    /// Exact converted dataset digest.
    pub dataset_content_digest: String,
    /// Stable vehicle or test-rig identity.
    pub vehicle_id: String,
    /// Stable corner/strut identity on that vehicle or rig.
    pub strut_id: String,
    /// Stable DAQ/logger hardware identity.
    pub data_logger_id: String,
    /// Exact logger software and version.
    pub logger_software: String,
    /// Clock synchronization mechanism.
    pub clock_kind: SuspensionCaptureClockKind,
    /// True only when every channel uses the declared synchronized clock domain.
    pub all_channels_synchronized: bool,
    /// Conservative upper bound on inter-channel timestamp error.
    pub maximum_timestamp_uncertainty_s: f64,
    /// Raw retained measurement container.
    pub raw_capture_format: SuspensionRawCaptureFormat,
    /// Content-addressed raw measurement bytes.
    pub raw_capture: SuspensionEvidenceFileRef,
    /// Content-addressed acquisition and installation procedure.
    pub acquisition_procedure: SuspensionEvidenceFileRef,
    /// Exactly position, velocity, and force, in canonical order.
    pub signals: Vec<SuspensionSignalEvidence>,
    /// Full source commit used to convert and identify the capture.
    pub rne_commit: String,
    /// SHA-256 of compact JSON with this field empty.
    pub content_sha256: String,
}

impl SuspensionPhysicalAcquisitionManifest {
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

    /// Validates metadata and binds the manifest to an exact physical-source dataset.
    pub fn validate(&self, dataset: &SuspensionIdentificationDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == SUSPENSION_ACQUISITION_MANIFEST_KIND
                && self.schema_version == SUSPENSION_ACQUISITION_SCHEMA_VERSION,
            "suspension acquisition kind/schema drift"
        );
        ensure!(
            matches!(
                self.source_kind,
                SuspensionDatasetSourceKind::RecordedBench
                    | SuspensionDatasetSourceKind::RecordedVehicle
            ) && self.source_kind == dataset.source_kind,
            "physical suspension source mismatch"
        );
        ensure!(
            self.dataset_content_digest == dataset.content_digest,
            "physical suspension dataset digest mismatch"
        );
        ensure!(
            [
                self.capture_id.as_str(),
                self.vehicle_id.as_str(),
                self.strut_id.as_str(),
                self.data_logger_id.as_str(),
            ]
            .into_iter()
            .all(valid_id),
            "invalid physical suspension identity"
        );
        ensure!(valid_text(&self.logger_software), "invalid logger software");
        ensure!(
            self.all_channels_synchronized
                && self.maximum_timestamp_uncertainty_s.is_finite()
                && (0.0..=MAX_TIMESTAMP_UNCERTAINTY_S)
                    .contains(&self.maximum_timestamp_uncertainty_s),
            "unqualified suspension timestamp synchronization"
        );
        self.raw_capture.validate()?;
        self.acquisition_procedure.validate()?;
        ensure!(
            self.signals.len() == 3
                && self.signals.iter().map(|signal| signal.signal).eq([
                    SuspensionSignalKind::Position,
                    SuspensionSignalKind::Velocity,
                    SuspensionSignalKind::Force,
                ]),
            "physical suspension channels are incomplete or unordered"
        );
        for signal in &self.signals {
            signal.validate()?;
        }
        // A shared calibration file is valid only with one consistent byte identity.
        // Check before deduplicating file reads, including cross-role references.
        let mut references = BTreeMap::new();
        for artifact in self.artifacts() {
            if let Some(previous) = references.insert(artifact.path.as_str(), artifact) {
                ensure!(
                    previous == artifact,
                    "conflicting suspension evidence references for {}",
                    artifact.path
                );
            }
        }
        let reference_rate_hz = self.signals[0].sample_rate_hz;
        ensure!(
            self.signals
                .iter()
                .all(|signal| signal.sample_rate_hz == reference_rate_hz),
            "physical suspension channel sample-rate mismatch"
        );
        ensure!(valid_git_revision(&self.rne_commit), "invalid RNE commit");
        ensure!(
            valid_sha256(&self.content_sha256)
                && self.content_sha256 == self.computed_content_sha256()?,
            "suspension acquisition manifest digest mismatch"
        );
        Ok(())
    }

    /// Validates the contract and streams every referenced file from an external root.
    pub fn verify_files(
        &self,
        dataset: &SuspensionIdentificationDataset,
        evidence_root: &Path,
    ) -> Result<()> {
        self.validate(dataset)?;
        let canonical_root = evidence_root
            .canonicalize()
            .with_context(|| format!("canonicalize {}", evidence_root.display()))?;
        ensure!(
            canonical_root.is_dir(),
            "suspension evidence root is not a directory"
        );
        let mut paths = BTreeSet::new();
        for artifact in self.artifacts() {
            if paths.insert(artifact.path.as_str()) {
                verify_file(&canonical_root, artifact)?;
            }
        }
        Ok(())
    }

    fn artifacts(&self) -> impl Iterator<Item = &SuspensionEvidenceFileRef> {
        std::iter::once(&self.raw_capture)
            .chain(std::iter::once(&self.acquisition_procedure))
            .chain(
                self.signals
                    .iter()
                    .map(|signal| &signal.calibration_artifact),
            )
    }
}

/// Decodes one bounded manifest and validates its binding to the supplied dataset.
pub fn decode_suspension_acquisition_manifest(
    bytes: &[u8],
    dataset: &SuspensionIdentificationDataset,
) -> Result<SuspensionPhysicalAcquisitionManifest> {
    ensure!(
        bytes.len() <= MAX_SUSPENSION_ACQUISITION_MANIFEST_BYTES,
        "suspension acquisition manifest exceeds byte limit"
    );
    let manifest: SuspensionPhysicalAcquisitionManifest = serde_json::from_slice(bytes)?;
    manifest.validate(dataset)?;
    Ok(manifest)
}

fn verify_file(root: &Path, artifact: &SuspensionEvidenceFileRef) -> Result<()> {
    artifact.validate()?;
    let candidate = root.join(PathBuf::from(&artifact.path));
    let canonical = candidate
        .canonicalize()
        .with_context(|| format!("canonicalize {}", candidate.display()))?;
    ensure!(
        canonical.starts_with(root),
        "suspension evidence path escaped root"
    );
    let file = File::open(&canonical).with_context(|| format!("open {}", canonical.display()))?;
    let metadata = file
        .metadata()
        .with_context(|| format!("inspect {}", canonical.display()))?;
    ensure!(
        metadata.is_file() && metadata.len() == artifact.size_bytes,
        "suspension evidence file size mismatch"
    );
    verify_stream(file, artifact).with_context(|| format!("read {}", canonical.display()))
}

fn verify_stream(reader: impl Read, artifact: &SuspensionEvidenceFileRef) -> Result<()> {
    artifact.validate()?;
    let mut file = reader.take(artifact.size_bytes + 1);
    let mut bytes_read = 0_u64;
    let mut hasher = Sha256::new();
    // Windows executable main threads can have a 1 MiB stack. Keep streamed
    // evidence independent of that limit, including nested CLI verification.
    let mut buffer = vec![0_u8; 64 * 1024];
    loop {
        let count = file.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        bytes_read += count as u64;
        ensure!(
            bytes_read <= artifact.size_bytes,
            "suspension evidence file grew during verification"
        );
        hasher.update(&buffer[..count]);
    }
    ensure!(
        bytes_read == artifact.size_bytes,
        "suspension evidence file shrank during verification"
    );
    ensure!(
        format!("sha256:{:x}", hasher.finalize()) == artifact.sha256,
        "suspension evidence file SHA-256 mismatch"
    );
    Ok(())
}

fn valid_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
}

fn valid_text(value: &str) -> bool {
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

fn valid_sha256(value: &str) -> bool {
    value.strip_prefix("sha256:").is_some_and(|hex| {
        hex.len() == 64
            && hex
                .bytes()
                .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    })
}

fn valid_git_revision(value: &str) -> bool {
    value.len() == 40
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::suspension_identification::synthetic_suspension_identification_dataset;
    use std::fs;

    fn recorded_dataset() -> SuspensionIdentificationDataset {
        let mut dataset = synthetic_suspension_identification_dataset().unwrap();
        dataset.dataset_id = "rne.recorded.suspension.bench.v1".to_string();
        dataset.source_kind = SuspensionDatasetSourceKind::RecordedBench;
        dataset.source_description = "test-only byte fixture standing in for a bench export".into();
        dataset.seal().unwrap();
        dataset
    }

    fn file_ref(path: &str, bytes: &[u8]) -> SuspensionEvidenceFileRef {
        SuspensionEvidenceFileRef {
            path: path.to_string(),
            size_bytes: bytes.len() as u64,
            sha256: format!("sha256:{:x}", Sha256::digest(bytes)),
        }
    }

    fn manifest(
        dataset: &SuspensionIdentificationDataset,
    ) -> SuspensionPhysicalAcquisitionManifest {
        let certificate = file_ref("calibration.txt", b"test-only calibration bytes");
        let mut manifest = SuspensionPhysicalAcquisitionManifest {
            kind: SUSPENSION_ACQUISITION_MANIFEST_KIND.to_string(),
            schema_version: SUSPENSION_ACQUISITION_SCHEMA_VERSION,
            capture_id: "bench.capture.001".into(),
            source_kind: SuspensionDatasetSourceKind::RecordedBench,
            dataset_content_digest: dataset.content_digest.clone(),
            vehicle_id: "quarter.car.rig.01".into(),
            strut_id: "front.left".into(),
            data_logger_id: "daq.01".into(),
            logger_software: "test logger 1.0".into(),
            clock_kind: SuspensionCaptureClockKind::SharedHardware,
            all_channels_synchronized: true,
            maximum_timestamp_uncertainty_s: 0.000_01,
            raw_capture_format: SuspensionRawCaptureFormat::Csv,
            raw_capture: file_ref("raw.csv", b"test-only raw capture bytes"),
            acquisition_procedure: file_ref("procedure.txt", b"test-only procedure bytes"),
            signals: vec![
                signal(SuspensionSignalKind::Position, certificate.clone()),
                SuspensionSignalEvidence {
                    origin: SuspensionSignalOrigin::Derived,
                    calibration_kind: SuspensionCalibrationKind::DerivedSignalProcedure,
                    ..signal(SuspensionSignalKind::Velocity, certificate.clone())
                },
                signal(SuspensionSignalKind::Force, certificate),
            ],
            rne_commit: "a".repeat(40),
            content_sha256: String::new(),
        };
        manifest.seal().unwrap();
        manifest
    }

    fn signal(
        kind: SuspensionSignalKind,
        calibration_artifact: SuspensionEvidenceFileRef,
    ) -> SuspensionSignalEvidence {
        SuspensionSignalEvidence {
            signal: kind,
            sensor_id: format!("sensor.{kind:?}").to_ascii_lowercase(),
            unit: kind.unit().into(),
            origin: SuspensionSignalOrigin::Measured,
            positive_along_strut_axis: true,
            sample_rate_hz: 200.0,
            resolution_si: 0.000_001,
            expanded_uncertainty_si: 0.000_01,
            calibration_kind: SuspensionCalibrationKind::Iso17025,
            calibration_artifact,
        }
    }

    #[test]
    fn derived_velocity_binding_checks_nominal_rows_and_retained_procedure() {
        use crate::suspension_derivative::{
            SuspensionDerivativeBinding, SuspensionDerivativeOperator,
        };
        let mut dataset = recorded_dataset();
        let operator = SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1;
        let times: Vec<_> = dataset.samples.iter().map(|s| s.capture_time_s).collect();
        let positions: Vec<_> = dataset.samples.iter().map(|s| s.position_m).collect();
        let velocities = operator.reconstruct(&times, &positions).unwrap();
        for (sample, velocity) in dataset.samples.iter_mut().zip(&velocities) {
            sample.velocity_m_s = *velocity;
        }
        dataset.seal().unwrap();
        let mut manifest = manifest(&dataset);
        let binding = SuspensionDerivativeBinding {
            capture_id: manifest.capture_id.clone(),
            procedure: manifest.signals[1].calibration_artifact.clone(),
            operator,
            absolute_tolerance_m_s: 0.0,
        };
        assert_eq!(binding.validate(&dataset, &manifest).unwrap(), velocities);
        let root =
            std::env::temp_dir().join(format!("rne-derived-velocity-{}", std::process::id()));
        fs::create_dir(&root).unwrap();
        fs::write(root.join("raw.csv"), b"test-only raw capture bytes").unwrap();
        fs::write(root.join("procedure.txt"), b"test-only procedure bytes").unwrap();
        fs::write(root.join("calibration.txt"), b"test-only calibration bytes").unwrap();
        assert_eq!(
            binding.verify_files(&dataset, &manifest, &root).unwrap(),
            velocities
        );
        fs::write(root.join("calibration.txt"), b"tampered").unwrap();
        assert!(binding.verify_files(&dataset, &manifest, &root).is_err());
        // The owned test-only directory contains no external dataset.
        fs::remove_dir_all(&root).unwrap();
        let mut wrong = binding.clone();
        wrong.capture_id.push_str("-other");
        assert!(wrong.validate(&dataset, &manifest).is_err());
        wrong = binding.clone();
        wrong.procedure.sha256 = format!("sha256:{}", "0".repeat(64));
        assert!(wrong.validate(&dataset, &manifest).is_err());
        for tolerance in [-1.0, f64::NAN, f64::INFINITY] {
            wrong = binding.clone();
            wrong.absolute_tolerance_m_s = tolerance;
            assert!(wrong.validate(&dataset, &manifest).is_err());
        }
        dataset.samples[0].velocity_m_s += 0.001;
        dataset.seal().unwrap();
        manifest.dataset_content_digest = dataset.content_digest.clone();
        manifest.seal().unwrap();
        assert!(binding.validate(&dataset, &manifest).is_err());
        wrong = binding.clone();
        wrong.absolute_tolerance_m_s = 0.002;
        assert!(wrong.validate(&dataset, &manifest).is_ok());
        manifest.signals[1].origin = SuspensionSignalOrigin::Measured;
        manifest.signals[1].calibration_kind = SuspensionCalibrationKind::Iso17025;
        manifest.seal().unwrap();
        assert!(wrong.validate(&dataset, &manifest).is_err());
    }

    #[test]
    fn physical_manifest_requires_physical_source_complete_channels_and_digest() {
        let dataset = recorded_dataset();
        let manifest = manifest(&dataset);
        manifest.validate(&dataset).unwrap();

        let synthetic = synthetic_suspension_identification_dataset().unwrap();
        assert!(manifest.validate(&synthetic).is_err());
        let mut unordered = manifest.clone();
        unordered.signals.swap(0, 1);
        unordered.seal().unwrap();
        assert!(unordered.validate(&dataset).is_err());
        let mut bad_clock = manifest;
        bad_clock.maximum_timestamp_uncertainty_s = 0.002;
        bad_clock.seal().unwrap();
        assert!(bad_clock.validate(&dataset).is_err());
    }

    #[test]
    fn acquired_run_binding_rejects_reused_raw_capture_and_mixed_subjects() {
        use crate::suspension_runs::{
            SuspensionAcquiredRunRequest, SuspensionRunInput, SuspensionRunRequest,
        };
        let first = recorded_dataset();
        let mut second = first.clone();
        second.dataset_id = "recorded.second".into();
        second.samples[0].force_n += 1.0;
        second.seal().unwrap();
        let training = manifest(&first);
        let mut holdout = manifest(&second);
        holdout.capture_id = "bench.capture.002".into();
        holdout.seal().unwrap();
        let mut request = SuspensionAcquiredRunRequest {
            runs: SuspensionRunRequest {
                kind: "rne_suspension_run_request".into(),
                schema_version: 1,
                spec: crate::suspension_identification::suspension_identification_spec(),
                training: vec![SuspensionRunInput {
                    acquisition_id: 1,
                    dataset: first,
                }],
                holdout: vec![SuspensionRunInput {
                    acquisition_id: 2,
                    dataset: second,
                }],
            },
            training: vec![training],
            holdout: vec![holdout],
        };
        assert!(request
            .validate()
            .unwrap_err()
            .to_string()
            .contains("raw capture"));
        request.holdout[0].raw_capture = file_ref("second.csv", b"different test-only capture");
        request.holdout[0].seal().unwrap();
        request.validate().unwrap();
        let request_bytes = serde_json::to_vec(&request).unwrap();
        assert_eq!(
            crate::suspension_runs::decode_suspension_acquired_request(&request_bytes).unwrap(),
            request
        );
        let mut unknown = serde_json::to_value(&request).unwrap();
        unknown["verified"] = serde_json::json!(true);
        assert!(crate::suspension_runs::decode_suspension_acquired_request(
            &serde_json::to_vec(&unknown).unwrap()
        )
        .is_err());
        assert!(
            crate::suspension_runs::decode_suspension_acquired_request(&vec![
                b' ';
                crate::suspension_runs::MAX_SUSPENSION_RUN_BYTES
                    + 1
            ])
            .is_err()
        );
        let mut swapped = request.clone();
        std::mem::swap(&mut swapped.training, &mut swapped.holdout);
        assert!(swapped.validate().is_err());
        let root =
            std::env::temp_dir().join(format!("rne-acquired-evidence-{}", std::process::id()));
        fs::create_dir(&root).unwrap();
        fs::write(root.join("raw.csv"), b"test-only raw capture bytes").unwrap();
        fs::write(root.join("second.csv"), b"different test-only capture").unwrap();
        fs::write(root.join("procedure.txt"), b"test-only procedure bytes").unwrap();
        fs::write(root.join("calibration.txt"), b"test-only calibration bytes").unwrap();
        let evidence = request.identify(&root, 1e-9).unwrap();
        let standalone = request.training[0].signals[2].calibration_artifact.clone();
        standalone.verify(&root).unwrap();
        assert!(standalone.verify(&root.join("raw.csv")).is_err());
        let mut invalid = standalone.clone();
        invalid.path = "../calibration.txt".into();
        assert!(invalid.verify(&root).is_err());
        invalid = standalone.clone();
        invalid.size_bytes += 1;
        assert!(invalid.verify(&root).is_err());
        fs::write(root.join("calibration.txt"), b"TEST-ONLY calibration bytes").unwrap();
        assert!(standalone.verify(&root).is_err());
        fs::write(root.join("calibration.txt"), b"test-only calibration bytes").unwrap();
        standalone.verify(&root).unwrap();
        use crate::suspension_uncertainty::*;
        let error_request = SuspensionAcquiredErrorRequest {
            kind: "rne_suspension_acquired_error_request".into(),
            schema_version: 1,
            acquisitions: request.clone(),
            model: SuspensionErrorModel {
                kind: "rne_suspension_additive_error_model".into(),
                schema_version: 1,
                seed: 42,
                draws: 8,
                factors: vec![SuspensionErrorFactor {
                    factor_id: 7,
                    distribution: SuspensionErrorDistribution::Normal,
                    scope: SuspensionErrorScope::SharedTraining,
                    position_loading_m: 0.0,
                    velocity_loading_m_s: 0.0,
                    force_loading_n: 100.0,
                }],
            },
            calibration: vec![SuspensionFactorCalibrationBinding {
                factor_id: 7,
                acquisition_id: request.runs.training[0].acquisition_id,
                signal: SuspensionSignalKind::Force,
                calibration_artifact: request.training[0].signals[2].calibration_artifact.clone(),
                interpretation:
                    "Synthetic test-only 100 N shared Gaussian assumption, not certificate-derived."
                        .into(),
            }],
        };
        let propagated = error_request.propagate(&root).unwrap();
        {
            use crate::suspension_derivative::{
                SuspensionDerivativeBinding, SuspensionDerivativeOperator,
            };
            let mut derived = request.clone();
            let operator = SuspensionDerivativeOperator::NonuniformThreePointSecantEndsV1;
            let dataset = &mut derived.runs.training[0].dataset;
            let times: Vec<_> = dataset.samples.iter().map(|s| s.capture_time_s).collect();
            let positions: Vec<_> = dataset.samples.iter().map(|s| s.position_m).collect();
            for (sample, velocity) in dataset
                .samples
                .iter_mut()
                .zip(operator.reconstruct(&times, &positions).unwrap())
            {
                sample.velocity_m_s = velocity;
            }
            dataset.seal().unwrap();
            derived.training[0].dataset_content_digest = dataset.content_digest.clone();
            derived.training[0].seal().unwrap();
            let bindings = vec![SuspensionDerivativeBinding {
                capture_id: derived.training[0].capture_id.clone(),
                procedure: derived.training[0].signals[1].calibration_artifact.clone(),
                operator,
                absolute_tolerance_m_s: 0.0,
            }];
            let mut model = error_request.model.clone();
            {
                use crate::suspension_affine_acquisition::*;
                use crate::suspension_derivative::{
                    SuspensionAffineCorrection, SuspensionAffineErrorModel, SuspensionAffineFactor,
                };
                use SuspensionAffineDomain::{Force, Position, Timebase};
                let mut affine = SuspensionAcquiredAffineRequest {
                    kind: "rne_suspension_acquired_affine_request".into(),
                    schema_version: 1,
                    acquisitions: derived.clone(),
                    derivatives: bindings.clone(),
                    model: SuspensionAffineErrorModel {
                        kind: "rne_suspension_affine_error_model".into(),
                        schema_version: 1,
                        seed: 42,
                        draws: 8,
                        nominal: vec![(
                            SuspensionAffineCorrection {
                                time_reference_s: 0.0,
                                time_scale: 1.0,
                                time_offset_s: 0.0,
                                position_scale: 1.0,
                                position_offset_m: 0.0,
                                force_scale: 1.0,
                                force_offset_n: 0.0,
                            },
                            operator,
                        )],
                        factors: vec![SuspensionAffineFactor {
                            factor_id: 7,
                            distribution: SuspensionErrorDistribution::Normal,
                            scope: SuspensionErrorScope::SharedTraining,
                            time_scale_loading: 0.01,
                            time_offset_loading_s: 0.0,
                            position_scale_loading: 0.01,
                            position_offset_loading_m: 0.0,
                            force_scale_loading: 0.01,
                            force_offset_loading_n: 0.0,
                        }],
                    },
                    calibration: vec![],
                };
                // Dedicated synthetic clock bytes, not a synchronization declaration.
                fs::write(root.join("clock.txt"), b"test-only calibration bytes").unwrap();
                for factor_id in [None, Some(7)] {
                    for domain in [Timebase, Position, Force] {
                        let signal =
                            &derived.training[0].signals[if domain == Force { 2 } else { 0 }];
                        let mut artifact = signal.calibration_artifact.clone();
                        if domain == Timebase {
                            artifact.path = "clock.txt".into();
                        }
                        affine.calibration.push(SuspensionAffineCalibrationBinding {
                            factor_id,
                            domain,
                            acquisition_id: derived.runs.training[0].acquisition_id,
                            capture_id: derived.training[0].capture_id.clone(),
                            instrument_id: if domain == Timebase {
                                "test-clock".into()
                            } else {
                                signal.sensor_id.clone()
                            },
                            calibration_artifact: artifact,
                            interpretation: "Synthetic test assumption, not physical calibration."
                                .into(),
                        });
                    }
                }
                let propagated = affine.propagate(&root).unwrap();
                let envelope = affine.evaluate(&root).unwrap();
                let bytes = encode_suspension_affine(&envelope, &root).unwrap();
                assert_eq!(envelope, decode_suspension_affine(&bytes, &root).unwrap());
                let mut forged = envelope.clone();
                forged.propagation.draws.clear();
                assert!(forged.verify(&root).is_err());
                forged = envelope.clone();
                forged.schema_version = 2;
                assert!(forged.verify(&root).is_err());
                let mut unknown = serde_json::to_value(&envelope).unwrap();
                unknown["qualified"] = serde_json::json!(true);
                assert!(
                    decode_suspension_affine(&serde_json::to_vec(&unknown).unwrap(), &root)
                        .is_err()
                );
                assert_eq!(propagated, affine.model.propagate(&derived.runs).unwrap());
                assert_eq!(propagated, affine.propagate(&root).unwrap());
                let mut bad = affine.clone();
                bad.calibration.pop();
                assert!(bad.propagate(&root).is_err());
                bad = affine.clone();
                bad.calibration[3].instrument_id = "other-clock".into();
                assert!(bad.propagate(&root).is_err());
                bad = affine.clone();
                bad.calibration[4].capture_id = "other-capture".into();
                assert!(bad.propagate(&root).is_err());
                bad = affine.clone();
                bad.calibration[1].instrument_id = "other-position".into();
                assert!(bad.propagate(&root).is_err());
                fs::write(root.join("clock.txt"), b"TEST-ONLY calibration bytes").unwrap();
                assert!(affine.propagate(&root).is_err());
                assert!(decode_suspension_affine(&bytes, &root).is_err());
                fs::write(root.join("clock.txt"), b"test-only calibration bytes").unwrap();
                assert_eq!(propagated, affine.propagate(&root).unwrap());
            }
            model.factors[0].position_loading_m = 0.001;
            model.factors[0].scope = SuspensionErrorScope::Sample;
            let result =
                propagate_acquired_derived_errors(&derived, &model, &bindings, &root).unwrap();
            assert_eq!(result.draws.len(), model.draws);
            assert_eq!(
                result,
                propagate_acquired_derived_errors(&derived, &model, &bindings, &root).unwrap()
            );
            assert!(propagate_acquired_derived_errors(&derived, &model, &[], &root).is_err());
            model.factors[0].velocity_loading_m_s = 1e-9;
            assert!(
                propagate_acquired_derived_errors(&derived, &model, &bindings, &root)
                    .unwrap_err()
                    .to_string()
                    .contains("independent velocity")
            );
            model.factors[0].velocity_loading_m_s = 0.0;
            use crate::suspension_derivative::{
                decode_suspension_derived_errors, encode_suspension_derived_errors,
                SuspensionDerivedErrorRequest,
            };
            let mut errors = error_request.clone();
            errors.acquisitions = derived.clone();
            errors.model = model.clone();
            let mut position_binding = errors.calibration[0].clone();
            position_binding.signal = SuspensionSignalKind::Position;
            position_binding.calibration_artifact =
                derived.training[0].signals[0].calibration_artifact.clone();
            errors.calibration.insert(0, position_binding);
            let envelope = SuspensionDerivedErrorRequest {
                errors,
                bindings: bindings.clone(),
            }
            .evaluate(&root)
            .unwrap();
            assert_eq!(envelope.propagation, result);
            let bytes = encode_suspension_derived_errors(&envelope, &root).unwrap();
            assert_eq!(
                decode_suspension_derived_errors(&bytes, &root).unwrap(),
                envelope
            );
            let mut forged = envelope.clone();
            forged.propagation.draws.clear();
            assert!(forged.verify(&root).is_err());
            forged = envelope.clone();
            forged.schema_version = 2;
            assert!(forged.verify(&root).is_err());
            let mut unknown = serde_json::to_value(&envelope).unwrap();
            unknown["qualified"] = serde_json::json!(true);
            assert!(decode_suspension_derived_errors(
                &serde_json::to_vec(&unknown).unwrap(),
                &root
            )
            .is_err());
            fs::write(
                root.join("calibration.txt"),
                b"changed derivative procedure",
            )
            .unwrap();
            assert!(propagate_acquired_derived_errors(&derived, &model, &bindings, &root).is_err());
            assert!(decode_suspension_derived_errors(&bytes, &root).is_err());
            fs::write(root.join("calibration.txt"), b"test-only calibration bytes").unwrap();
            assert_eq!(
                result,
                propagate_acquired_derived_errors(&derived, &model, &bindings, &root).unwrap()
            );
        }
        let interpretations: Vec<_> = [
            SuspensionSignalKind::Position,
            SuspensionSignalKind::Velocity,
            SuspensionSignalKind::Force,
        ]
        .into_iter()
        .map(|signal| SuspensionCoverageInterpretation {
            acquisition_id: request.runs.training[0].acquisition_id,
            signal,
            coverage_factor: 2.0,
            absolute_tolerance_si: 1e-12,
        })
        .collect();
        let budget = error_request.audit_budget(&root, &interpretations).unwrap();
        assert_eq!(budget.len(), 3);
        assert!(budget.iter().all(|entry| !entry.matched));
        assert_eq!(budget[2].modeled_standard_uncertainty_si, 100.0);
        let mut uncertainty_request = SuspensionUncertaintyRequest {
            errors: error_request.clone(),
            interpretations: interpretations.clone(),
        };
        uncertainty_request.errors.model.factors[0].force_loading_n = 1e8;
        let uncertainty = uncertainty_request.evaluate(&root).unwrap();
        assert!(uncertainty
            .propagation
            .draws
            .iter()
            .any(|draw| draw.is_err()));
        let uncertainty_bytes = encode_suspension_uncertainty(&uncertainty, &root).unwrap();
        assert_eq!(
            decode_suspension_uncertainty(&uncertainty_bytes, &root).unwrap(),
            uncertainty
        );
        let mut corrupted = uncertainty.clone();
        corrupted.propagation.draws.pop();
        assert!(encode_suspension_uncertainty(&corrupted, &root).is_err());
        assert!(
            decode_suspension_uncertainty(&serde_json::to_vec(&corrupted).unwrap(), &root).is_err()
        );
        corrupted = uncertainty.clone();
        corrupted.budget[0].matched = true;
        assert!(corrupted.verify(&root).is_err());
        corrupted = uncertainty.clone();
        corrupted.schema_version = 2;
        assert!(corrupted.verify(&root).is_err());
        let mut unknown = serde_json::to_value(&uncertainty).unwrap();
        unknown["qualified"] = serde_json::json!(true);
        assert!(
            decode_suspension_uncertainty(&serde_json::to_vec(&unknown).unwrap(), &root).is_err()
        );
        assert!(decode_suspension_uncertainty(
            &vec![b' '; crate::suspension_runs::MAX_SUSPENSION_RUN_BYTES + 1],
            &root
        )
        .is_err());
        let mut consistent = error_request.clone();
        consistent.model.factors[0].force_loading_n = 3e-6;
        let mut extra = consistent.model.factors[0].clone();
        extra.factor_id = 8;
        extra.force_loading_n = -4e-6;
        consistent.model.factors.push(extra);
        let mut binding = consistent.calibration[0].clone();
        binding.factor_id = 8;
        consistent.calibration.push(binding);
        let budget = consistent.audit_budget(&root, &interpretations).unwrap();
        assert!(budget[2].matched);
        assert!((budget[2].modeled_standard_uncertainty_si - 5e-6).abs() < 1e-15);
        assert!(!budget[0].matched); // Unmodeled position uncertainty remains visible.
        assert!(consistent
            .audit_budget(&root, &interpretations[..2])
            .is_err());
        let mut invalid = interpretations.clone();
        invalid[0].coverage_factor = 0.0;
        assert!(consistent.audit_budget(&root, &invalid).is_err());
        assert_eq!(
            propagated,
            propagate_suspension_errors(&request.runs, &error_request.model).unwrap()
        );
        let mut missing = error_request.clone();
        missing.calibration.clear();
        assert!(missing.validate().is_err());
        let mut wrong = error_request.clone();
        wrong.calibration[0].calibration_artifact.sha256 = format!("sha256:{}", "0".repeat(64));
        assert!(wrong.validate().is_err());
        wrong = error_request.clone();
        wrong.calibration[0].interpretation.clear();
        assert!(wrong.validate().is_err());
        fs::write(root.join("calibration.txt"), b"corrupted calibration bytes").unwrap();
        assert!(error_request.propagate(&root).is_err());
        assert!(error_request.audit_budget(&root, &interpretations).is_err());
        assert!(decode_suspension_uncertainty(&uncertainty_bytes, &root).is_err());
        fs::write(root.join("calibration.txt"), b"test-only calibration bytes").unwrap();
        assert_eq!(error_request.propagate(&root).unwrap(), propagated);
        let bytes =
            crate::suspension_runs::encode_suspension_acquired_evidence(&evidence, &root).unwrap();
        assert_eq!(
            crate::suspension_runs::decode_suspension_acquired_evidence(&bytes, &root).unwrap(),
            evidence
        );
        let mut forged = evidence.clone();
        forged.schema_version = 2;
        assert!(forged.verify(&root).is_err());
        forged = evidence.clone();
        forged.timing.holdout[0].mean_residual_n += 1.0;
        assert!(forged.verify(&root).is_err());
        fs::write(root.join("second.csv"), b"changed capture").unwrap();
        assert!(
            crate::suspension_runs::decode_suspension_acquired_evidence(&bytes, &root).is_err()
        );
        fs::remove_dir_all(&root).unwrap();
        assert!(request.identify(&root, 1e-9).is_err());
        request.holdout[0].strut_id = "rear.right".into();
        request.holdout[0].seal().unwrap();
        assert!(request
            .validate()
            .unwrap_err()
            .to_string()
            .contains("subjects"));
        request.holdout.clear();
        assert!(request
            .validate()
            .unwrap_err()
            .to_string()
            .contains("count"));
    }

    #[test]
    fn stream_verification_bounds_growth_and_rejects_truncation() {
        use std::io::Cursor;
        let reference = file_ref("sample.bin", b"abc");
        verify_stream(Cursor::new(b"abc"), &reference).unwrap();
        let mut growing = Cursor::new(b"abc-extra-bytes");
        assert!(verify_stream(&mut growing, &reference)
            .unwrap_err()
            .to_string()
            .contains("grew"));
        assert_eq!(growing.position(), reference.size_bytes + 1);
        assert!(verify_stream(Cursor::new(b"ab"), &reference)
            .unwrap_err()
            .to_string()
            .contains("shrank"));
        assert!(verify_stream(Cursor::new(b"abd"), &reference)
            .unwrap_err()
            .to_string()
            .contains("SHA-256"));
        struct Broken;
        impl Read for Broken {
            fn read(&mut self, _: &mut [u8]) -> std::io::Result<usize> {
                Err(std::io::Error::other("injected read failure"))
            }
        }
        assert!(verify_stream(Broken, &reference)
            .unwrap_err()
            .to_string()
            .contains("injected"));
    }

    #[test]
    fn shared_evidence_paths_require_identical_size_and_digest() {
        let dataset = recorded_dataset();
        let original = manifest(&dataset);
        original.validate(&dataset).unwrap();
        for change_size in [false, true] {
            let mut conflicting = original.clone();
            // The first calibration reference remains valid; the later one must
            // not escape validation merely because its path was already visited.
            if change_size {
                conflicting.signals[1].calibration_artifact.size_bytes += 1;
            } else {
                conflicting.signals[1].calibration_artifact.sha256 =
                    format!("sha256:{}", "0".repeat(64));
            }
            conflicting.seal().unwrap();
            assert!(conflicting
                .validate(&dataset)
                .unwrap_err()
                .to_string()
                .contains("conflicting"));
            assert!(decode_suspension_acquisition_manifest(
                &serde_json::to_vec(&conflicting).unwrap(),
                &dataset
            )
            .is_err());
        }
        let mut cross_role = original.clone();
        cross_role.acquisition_procedure = cross_role.raw_capture.clone();
        cross_role.seal().unwrap();
        cross_role.validate(&dataset).unwrap();
        cross_role.acquisition_procedure.size_bytes += 1;
        cross_role.seal().unwrap();
        assert!(cross_role
            .validate(&dataset)
            .unwrap_err()
            .to_string()
            .contains("conflicting"));
    }

    #[test]
    fn external_files_are_streamed_and_tampering_is_detected() {
        let dataset = recorded_dataset();
        let manifest = manifest(&dataset);
        let root = std::env::temp_dir().join(format!(
            "rne-suspension-acquisition-{}-{}",
            std::process::id(),
            dataset.samples.len()
        ));
        if root.exists() {
            fs::remove_dir_all(&root).unwrap();
        }
        fs::create_dir(&root).unwrap();
        fs::write(root.join("raw.csv"), b"test-only raw capture bytes").unwrap();
        fs::write(root.join("procedure.txt"), b"test-only procedure bytes").unwrap();
        fs::write(root.join("calibration.txt"), b"test-only calibration bytes").unwrap();

        manifest.verify_files(&dataset, &root).unwrap();
        let mut conflicting = manifest.clone();
        conflicting.signals[1].calibration_artifact.sha256 = format!("sha256:{}", "0".repeat(64));
        conflicting.seal().unwrap();
        assert!(conflicting
            .verify_files(&dataset, &root)
            .unwrap_err()
            .to_string()
            .contains("conflicting"));
        fs::write(root.join("calibration.txt"), b"tampered calibration bytes").unwrap();
        assert!(manifest.verify_files(&dataset, &root).is_err());
        fs::remove_dir_all(&root).unwrap();
    }

    #[test]
    fn bounded_decoder_rejects_unknown_fields_and_oversize() {
        let dataset = recorded_dataset();
        let manifest = manifest(&dataset);
        let bytes = serde_json::to_vec(&manifest).unwrap();
        assert_eq!(
            decode_suspension_acquisition_manifest(&bytes, &dataset).unwrap(),
            manifest
        );

        let mut value = serde_json::to_value(&manifest).unwrap();
        value["unexpected"] = serde_json::json!(true);
        assert!(decode_suspension_acquisition_manifest(
            &serde_json::to_vec(&value).unwrap(),
            &dataset
        )
        .is_err());
        assert!(decode_suspension_acquisition_manifest(
            &vec![b' '; MAX_SUSPENSION_ACQUISITION_MANIFEST_BYTES + 1],
            &dataset
        )
        .is_err());
    }
}

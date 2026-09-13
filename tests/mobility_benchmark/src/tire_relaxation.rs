//! Owned transient tire datasets, fit evidence, and physical acquisition binding.

use crate::tire_acquisition::{
    valid_git_revision, valid_id, valid_sha256, valid_text, verify_file, RoadFrictionEvidenceKind,
    TireCalibrationKind, TireCaptureClockKind, TireEvidenceFileRef, TireRawCaptureFormat,
    TireSignalOrigin,
};
use crate::tire_identification::{
    identify_tire_dataset, synthetic_tire_identification_dataset, TireDatasetSourceKind,
    TireIdentificationDataset, TireIdentificationEvidence,
};
use anyhow::{ensure, Context, Result};
use rne_robot::{
    evaluate_combined_slip_tire_steady_force, identify_tire_relaxation_length,
    CombinedSlipTireSpec, TireRelaxationAxis, TireRelaxationIdentificationResult,
    TireRelaxationIdentificationRun, TireRelaxationIdentificationSample,
    TireRelaxationIdentificationSpec,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

/// Stable input artifact discriminator.
pub const TIRE_RELAXATION_DATASET_KIND: &str = "rne_mobility_tire_relaxation_dataset";
/// Stable fit-evidence discriminator.
pub const TIRE_RELAXATION_RESULT_KIND: &str = "rne_mobility_tire_relaxation_result";
/// Stable joined physical-qualification discriminator.
pub const TIRE_RELAXATION_PHYSICAL_QUALIFICATION_KIND: &str =
    "rne_mobility_tire_relaxation_physical_qualification";
/// Stable transient acquisition-manifest discriminator.
pub const TIRE_RELAXATION_ACQUISITION_MANIFEST_KIND: &str =
    "rne_mobility_tire_relaxation_acquisition_manifest";
/// Current schema shared by the first transient tire artifact family.
pub const TIRE_RELAXATION_SCHEMA_VERSION: u32 = 1;
/// Maximum accepted serialized transient dataset size.
pub const MAX_TIRE_RELAXATION_DATASET_BYTES: usize = 32 * 1024 * 1024;
/// Maximum accepted serialized transient acquisition manifest size.
pub const MAX_TIRE_RELAXATION_ACQUISITION_MANIFEST_BYTES: usize = 4 * 1024 * 1024;
const MAX_TIMESTAMP_UNCERTAINTY_S: f64 = 0.001;

/// One owned transient acquisition assigned wholly to training or holdout.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationRunData {
    /// Stable acquisition identity.
    pub acquisition_id: u64,
    /// Stable speed/road/tire condition identity.
    pub condition_id: u64,
    /// Strictly ordered observable rows.
    pub samples: Vec<TireRelaxationIdentificationSample>,
}

impl TireRelaxationRunData {
    fn borrowed(&self) -> TireRelaxationIdentificationRun<'_> {
        TireRelaxationIdentificationRun {
            acquisition_id: self.acquisition_id,
            condition_id: self.condition_id,
            samples: &self.samples,
        }
    }
}

/// Unit-explicit, split-frozen input for one tire relaxation axis.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationDataset {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Stable dataset identity.
    pub dataset_id: String,
    /// Honest source class.
    pub source_kind: TireDatasetSourceKind,
    /// Human-readable source description; not integrity evidence by itself.
    pub source_description: String,
    /// Exact transient force-law identity.
    pub force_model: String,
    /// Axis fitted by this dataset.
    pub axis: TireRelaxationAxis,
    /// Owned steady-force dataset whose split and provenance are replayed first.
    pub steady_dataset: TireIdentificationDataset,
    /// Verified steady-force fit that supplies the only tire law used here.
    pub steady_identification: TireIdentificationEvidence,
    /// Complete acquisitions used for fitting.
    pub training_runs: Vec<TireRelaxationRunData>,
    /// Complete acquisitions used only for validation.
    pub holdout_runs: Vec<TireRelaxationRunData>,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireRelaxationDataset {
    /// Recomputes and stores the self-excluding dataset digest.
    pub fn seal(&mut self) -> Result<()> {
        self.content_sha256 = digest_without_hash(self)?;
        Ok(())
    }

    /// Validates provenance, split ownership, shape, units, and integrity.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == TIRE_RELAXATION_DATASET_KIND
                && self.schema_version == TIRE_RELAXATION_SCHEMA_VERSION,
            "tire relaxation dataset kind/schema drift"
        );
        ensure!(
            valid_id(&self.dataset_id),
            "invalid tire relaxation dataset identity"
        );
        ensure!(
            valid_text(&self.source_description),
            "invalid tire relaxation source description"
        );
        ensure!(
            self.force_model == "rne_combined_slip_tanh_ellipse_relaxation_v1",
            "unsupported tire relaxation force model"
        );
        self.steady_identification.validate(&self.steady_dataset)?;
        ensure!(
            self.source_kind == self.steady_dataset.source_kind,
            "steady and transient tire provenance differ"
        );
        let steady_tire_spec = self.steady_identification.result.tire_spec;
        ensure!(
            evaluate_combined_slip_tire_steady_force(
                steady_tire_spec,
                0.0,
                0.0,
                steady_tire_spec.reference_load_n,
                1.0,
            )
            .is_ok(),
            "invalid identified steady tire spec"
        );
        ensure!(
            !self.training_runs.is_empty() && !self.holdout_runs.is_empty(),
            "empty tire relaxation split"
        );
        let runs = self.training_runs.iter().chain(&self.holdout_runs);
        let mut ids = BTreeSet::new();
        let mut sample_count = 0_usize;
        for run in runs {
            ensure!(
                ids.insert(run.acquisition_id),
                "duplicate tire relaxation acquisition identity"
            );
            ensure!(run.samples.len() >= 2, "short tire relaxation acquisition");
            sample_count = sample_count
                .checked_add(run.samples.len())
                .context("tire relaxation sample count overflow")?;
            ensure!(
                run.samples.iter().all(valid_sample)
                    && run
                        .samples
                        .windows(2)
                        .all(|pair| pair[0].capture_time_s < pair[1].capture_time_s),
                "invalid tire relaxation sample sequence"
            );
        }
        ensure!(
            ids.len() <= 10_000 && sample_count <= 100_000,
            "unbounded tire relaxation dataset"
        );
        ensure!(
            valid_sha256(&self.content_sha256) && self.content_sha256 == digest_without_hash(self)?,
            "tire relaxation dataset digest drift"
        );
        Ok(())
    }
}

/// Recomputable fit evidence bound to one exact transient dataset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact input dataset SHA-256.
    pub dataset_content_sha256: String,
    /// Input provenance copied into the result.
    pub source_kind: TireDatasetSourceKind,
    /// Source-label claim only; physical qualification needs the joined manifest gate.
    pub recorded_source_claim: bool,
    /// Fitted axis.
    pub axis: TireRelaxationAxis,
    /// Frozen fit bounds and acceptance gates.
    pub identification_spec: TireRelaxationIdentificationSpec,
    /// Deterministically recomputed fit and residuals.
    pub result: TireRelaxationIdentificationResult,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireRelaxationEvidence {
    /// Replays the fit and verifies provenance, exact input binding, and integrity.
    pub fn validate(&self, dataset: &TireRelaxationDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == TIRE_RELAXATION_RESULT_KIND
                && self.schema_version == TIRE_RELAXATION_SCHEMA_VERSION,
            "tire relaxation result kind/schema drift"
        );
        ensure!(
            self.dataset_content_sha256 == dataset.content_sha256
                && self.source_kind == dataset.source_kind
                && self.recorded_source_claim == dataset.source_kind.is_physical_measurement()
                && self.axis == dataset.axis,
            "tire relaxation provenance drift"
        );
        ensure!(
            self.result == identify(dataset, self.identification_spec)?,
            "tire relaxation result drift"
        );
        ensure!(
            self.content_sha256 == digest_without_hash(self)?,
            "tire relaxation result digest drift"
        );
        Ok(())
    }
}

/// Required transient signal after conversion into the identifier convention.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TireRelaxationSignalKind {
    /// Positive regularized wheel-carrier transport speed.
    TransportSpeed,
    /// Kinematic slip target for the selected axis.
    TargetSlip,
    /// Positive contact-normal load.
    NormalLoad,
    /// Independently measured force for the selected axis.
    MeasuredAxisForce,
}

impl TireRelaxationSignalKind {
    /// Returns the exact SI-unit symbol required in artifact metadata.
    pub fn unit(self) -> &'static str {
        match self {
            Self::TransportSpeed => "m/s",
            Self::TargetSlip => "1",
            Self::NormalLoad | Self::MeasuredAxisForce => "N",
        }
    }

    /// Returns the exact positive-axis convention required for the selected fit axis.
    pub fn convention(self, axis: TireRelaxationAxis) -> &'static str {
        match (self, axis) {
            (Self::TransportSpeed, _) => "positive_regularized_contact_transport_speed",
            (Self::TargetSlip, TireRelaxationAxis::Longitudinal) => {
                "positive_wheel_surface_speed_minus_carrier_forward"
            }
            (Self::TargetSlip, TireRelaxationAxis::Lateral) => {
                "negative_carrier_lateral_velocity_divided_by_transport_speed"
            }
            (Self::NormalLoad, _) => "positive_contact_normal_on_wheel",
            (Self::MeasuredAxisForce, TireRelaxationAxis::Longitudinal) => "positive_wheel_forward",
            (Self::MeasuredAxisForce, TireRelaxationAxis::Lateral) => "positive_wheel_lateral",
        }
    }
}

/// Calibration- and uncertainty-bound transient signal evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationSignalEvidence {
    /// Canonical signal identity.
    pub signal: TireRelaxationSignalKind,
    /// Stable installed sensor or derived-channel identity.
    pub sensor_id: String,
    /// Exact SI unit.
    pub unit: String,
    /// Exact RNE positive-axis convention.
    pub convention: String,
    /// Directly measured or deterministically reconstructed.
    pub origin: TireSignalOrigin,
    /// Raw source sampling rate.
    pub source_sample_rate_hz: f64,
    /// Converted common-row sampling rate.
    pub converted_sample_rate_hz: f64,
    /// Smallest output increment in the declared unit.
    pub resolution_si: f64,
    /// Expanded uncertainty in the declared unit.
    pub expanded_uncertainty_si: f64,
    /// Traceability class.
    pub calibration_kind: TireCalibrationKind,
    /// Content-addressed calibration or derivation record.
    pub calibration_artifact: TireEvidenceFileRef,
}

impl TireRelaxationSignalEvidence {
    fn validate(&self, axis: TireRelaxationAxis) -> Result<()> {
        ensure!(
            valid_id(&self.sensor_id),
            "invalid tire relaxation sensor identity"
        );
        ensure!(
            self.unit == self.signal.unit(),
            "tire relaxation signal unit drift"
        );
        ensure!(
            self.convention == self.signal.convention(axis),
            "tire relaxation signal convention drift"
        );
        ensure!(
            self.source_sample_rate_hz.is_finite()
                && (1.0..=100_000.0).contains(&self.source_sample_rate_hz)
                && self.converted_sample_rate_hz.is_finite()
                && (1.0..=self.source_sample_rate_hz).contains(&self.converted_sample_rate_hz),
            "invalid tire relaxation sample rate"
        );
        ensure!(
            self.resolution_si.is_finite()
                && self.resolution_si > 0.0
                && self.expanded_uncertainty_si.is_finite()
                && self.expanded_uncertainty_si >= 0.0,
            "invalid tire relaxation signal resolution or uncertainty"
        );
        ensure!(
            (self.origin == TireSignalOrigin::Derived)
                == (self.calibration_kind == TireCalibrationKind::DerivedSignalProcedure),
            "derived tire relaxation signal calibration mismatch"
        );
        self.calibration_artifact.validate()
    }
}

/// Physical records for one complete transient dataset acquisition.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationRunAcquisitionEvidence {
    /// Exact dataset acquisition identity.
    pub acquisition_id: u64,
    /// Exact dataset condition identity.
    pub condition_id: u64,
    /// Stable tire/wheel installation identity.
    pub tire_id: String,
    /// Clock synchronization mechanism.
    pub clock_kind: TireCaptureClockKind,
    /// True only when all required channels share the synchronized domain.
    pub all_channels_synchronized: bool,
    /// Conservative inter-channel timestamp uncertainty bound.
    pub maximum_timestamp_uncertainty_s: f64,
    /// Raw retained container format.
    pub raw_capture_format: TireRawCaptureFormat,
    /// Content-addressed raw bytes.
    pub raw_capture: TireEvidenceFileRef,
    /// Stable segment identity within the raw container.
    pub raw_segment_id: String,
    /// Independent method establishing road friction.
    pub road_friction_evidence_kind: RoadFrictionEvidenceKind,
    /// Content-addressed road characterization record.
    pub road_friction_artifact: TireEvidenceFileRef,
    /// Exactly four required transient channels in canonical order.
    pub signals: Vec<TireRelaxationSignalEvidence>,
}

impl TireRelaxationRunAcquisitionEvidence {
    fn validate(&self, axis: TireRelaxationAxis, run: &TireRelaxationRunData) -> Result<()> {
        ensure!(
            self.acquisition_id == run.acquisition_id && self.condition_id == run.condition_id,
            "tire relaxation acquisition run binding drift"
        );
        ensure!(
            valid_id(&self.tire_id) && valid_id(&self.raw_segment_id),
            "invalid tire relaxation run identity"
        );
        ensure!(
            self.all_channels_synchronized
                && self.maximum_timestamp_uncertainty_s.is_finite()
                && (0.0..=MAX_TIMESTAMP_UNCERTAINTY_S)
                    .contains(&self.maximum_timestamp_uncertainty_s),
            "unqualified tire relaxation timestamp synchronization"
        );
        self.raw_capture.validate()?;
        self.road_friction_artifact.validate()?;
        let required = [
            TireRelaxationSignalKind::TransportSpeed,
            TireRelaxationSignalKind::TargetSlip,
            TireRelaxationSignalKind::NormalLoad,
            TireRelaxationSignalKind::MeasuredAxisForce,
        ];
        ensure!(
            self.signals.len() == required.len()
                && self.signals.iter().map(|signal| signal.signal).eq(required),
            "physical tire relaxation channels are incomplete or unordered"
        );
        for signal in &self.signals {
            signal.validate(axis)?;
        }
        let rate_hz = self.signals[0].converted_sample_rate_hz;
        ensure!(
            self.signals
                .iter()
                .all(|signal| signal.converted_sample_rate_hz == rate_hz),
            "physical tire relaxation converted sample-rate mismatch"
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

/// Dataset-bound physical acquisition manifest for transient tire identification.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationAcquisitionManifest {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Stable capture campaign identity.
    pub capture_id: String,
    /// Physical source class; synthetic fixtures are forbidden.
    pub source_kind: TireDatasetSourceKind,
    /// Exact converted transient dataset SHA-256.
    pub dataset_content_sha256: String,
    /// Stable vehicle or test-rig identity.
    pub vehicle_id: String,
    /// Stable DAQ/logger hardware identity.
    pub data_logger_id: String,
    /// Exact logger software and version.
    pub logger_software: String,
    /// Content-addressed acquisition and installation procedure.
    pub acquisition_procedure: TireEvidenceFileRef,
    /// One entry for every dataset run, sorted by acquisition identity.
    pub runs: Vec<TireRelaxationRunAcquisitionEvidence>,
    /// Full RNE source commit used for conversion and identification.
    pub rne_commit: String,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireRelaxationAcquisitionManifest {
    /// Recomputes and stores the self-excluding manifest digest.
    pub fn seal(&mut self) -> Result<()> {
        self.content_sha256 = digest_without_hash(self)?;
        Ok(())
    }

    /// Validates exact dataset/run binding and acquisition metadata.
    pub fn validate(&self, dataset: &TireRelaxationDataset) -> Result<()> {
        dataset.validate()?;
        ensure!(
            self.kind == TIRE_RELAXATION_ACQUISITION_MANIFEST_KIND
                && self.schema_version == TIRE_RELAXATION_SCHEMA_VERSION,
            "tire relaxation acquisition kind/schema drift"
        );
        ensure!(
            dataset.source_kind.is_physical_measurement()
                && self.source_kind == dataset.source_kind
                && self.dataset_content_sha256 == dataset.content_sha256,
            "physical tire relaxation source or dataset mismatch"
        );
        ensure!(
            [
                self.capture_id.as_str(),
                self.vehicle_id.as_str(),
                self.data_logger_id.as_str()
            ]
            .into_iter()
            .all(valid_id),
            "invalid physical tire relaxation identity"
        );
        ensure!(
            valid_text(&self.logger_software),
            "invalid tire relaxation logger software"
        );
        ensure!(valid_git_revision(&self.rne_commit), "invalid RNE commit");
        self.acquisition_procedure.validate()?;
        let dataset_runs = dataset.training_runs.iter().chain(&dataset.holdout_runs);
        let by_id = dataset_runs
            .map(|run| (run.acquisition_id, run))
            .collect::<BTreeMap<_, _>>();
        ensure!(
            by_id.len() == self.runs.len()
                && self
                    .runs
                    .windows(2)
                    .all(|pair| pair[0].acquisition_id < pair[1].acquisition_id),
            "physical tire relaxation run coverage is incomplete or unordered"
        );
        for run in &self.runs {
            run.validate(
                dataset.axis,
                by_id
                    .get(&run.acquisition_id)
                    .context("manifest contains unknown tire relaxation acquisition")?,
            )?;
        }
        let mut raw_segments = BTreeSet::new();
        ensure!(
            self.runs.iter().all(|run| raw_segments
                .insert((run.raw_capture.path.as_str(), run.raw_segment_id.as_str()))),
            "tire relaxation raw segment reused across acquisitions"
        );
        let mut references = BTreeMap::new();
        for artifact in self.artifacts() {
            if let Some(previous) = references.insert(artifact.path.as_str(), artifact) {
                ensure!(
                    previous == artifact,
                    "conflicting tire relaxation evidence reference"
                );
            }
        }
        ensure!(
            valid_sha256(&self.content_sha256) && self.content_sha256 == digest_without_hash(self)?,
            "tire relaxation acquisition manifest digest drift"
        );
        Ok(())
    }

    /// Streams every referenced record under an explicitly supplied evidence root.
    pub fn verify_files(
        &self,
        dataset: &TireRelaxationDataset,
        evidence_root: &Path,
    ) -> Result<()> {
        self.validate(dataset)?;
        let root = evidence_root
            .canonicalize()
            .with_context(|| format!("canonicalize {}", evidence_root.display()))?;
        ensure!(
            root.is_dir(),
            "tire relaxation evidence root is not a directory"
        );
        let mut paths = BTreeSet::new();
        for artifact in self.artifacts() {
            if paths.insert(artifact.path.as_str()) {
                verify_file(&root, artifact)?;
            }
        }
        Ok(())
    }

    fn artifacts(&self) -> impl Iterator<Item = &TireEvidenceFileRef> {
        std::iter::once(&self.acquisition_procedure).chain(
            self.runs
                .iter()
                .flat_map(TireRelaxationRunAcquisitionEvidence::artifacts),
        )
    }
}

/// Joined proof that transient identification and retained physical records passed together.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TireRelaxationPhysicalQualificationEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact converted dataset SHA-256.
    pub dataset_content_sha256: String,
    /// Recomputed transient fit evidence.
    pub identification: TireRelaxationEvidence,
    /// Exact acquisition-manifest SHA-256.
    pub acquisition_manifest_sha256: String,
    /// True only on this joined, successfully verified artifact.
    pub physical_measurement: bool,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl TireRelaxationPhysicalQualificationEvidence {
    /// Replays identification and streams the complete acquisition evidence set.
    pub fn validate(
        &self,
        dataset: &TireRelaxationDataset,
        manifest: &TireRelaxationAcquisitionManifest,
        evidence_root: &Path,
    ) -> Result<()> {
        ensure!(
            self.kind == TIRE_RELAXATION_PHYSICAL_QUALIFICATION_KIND
                && self.schema_version == TIRE_RELAXATION_SCHEMA_VERSION,
            "tire relaxation qualification kind/schema drift"
        );
        ensure!(
            self.physical_measurement
                && self.dataset_content_sha256 == dataset.content_sha256
                && self.identification.recorded_source_claim
                && self.acquisition_manifest_sha256 == manifest.content_sha256,
            "tire relaxation qualification binding drift"
        );
        self.identification.validate(dataset)?;
        manifest.verify_files(dataset, evidence_root)?;
        ensure!(
            self.content_sha256 == digest_without_hash(self)?,
            "tire relaxation qualification digest drift"
        );
        Ok(())
    }
}

/// Decodes one bounded transient dataset and verifies it before downstream use.
pub fn decode_tire_relaxation_dataset(bytes: &[u8]) -> Result<TireRelaxationDataset> {
    ensure!(
        bytes.len() <= MAX_TIRE_RELAXATION_DATASET_BYTES,
        "tire relaxation dataset exceeds byte limit"
    );
    let dataset: TireRelaxationDataset = serde_json::from_slice(bytes)?;
    dataset.validate()?;
    Ok(dataset)
}

/// Decodes one bounded acquisition manifest and verifies exact dataset binding.
pub fn decode_tire_relaxation_acquisition_manifest(
    bytes: &[u8],
    dataset: &TireRelaxationDataset,
) -> Result<TireRelaxationAcquisitionManifest> {
    ensure!(
        bytes.len() <= MAX_TIRE_RELAXATION_ACQUISITION_MANIFEST_BYTES,
        "tire relaxation acquisition manifest exceeds byte limit"
    );
    let manifest: TireRelaxationAcquisitionManifest = serde_json::from_slice(bytes)?;
    manifest.validate(dataset)?;
    Ok(manifest)
}

/// Returns the frozen v1 transient search and acceptance contract.
pub fn tire_relaxation_identification_spec() -> TireRelaxationIdentificationSpec {
    TireRelaxationIdentificationSpec {
        relaxation_length_bounds_m: [0.10, 0.60],
        minimum_transport_speed_m_s: 1.0,
        minimum_slip_excitation: 0.01,
        maximum_abs_slip: 1.0,
        maximum_force_utilization: 0.95,
        minimum_training_transitions: 40,
        minimum_holdout_transitions: 80,
        minimum_holdout_transitions_per_condition: 40,
        grid_points: 11,
        refinement_passes: 3,
        maximum_training_rms_slip: 1.0e-12,
        maximum_holdout_rms_slip: 1.0e-12,
        maximum_worst_condition_rms_slip: 1.0e-12,
    }
}

/// Builds a deterministic non-physical fixture for the owned artifact path.
pub fn synthetic_tire_relaxation_dataset() -> Result<TireRelaxationDataset> {
    synthetic_tire_relaxation_dataset_for_axis(TireRelaxationAxis::Longitudinal)
}

/// Builds a deterministic non-physical fixture for one explicitly selected axis.
pub fn synthetic_tire_relaxation_dataset_for_axis(
    axis: TireRelaxationAxis,
) -> Result<TireRelaxationDataset> {
    let steady_dataset = synthetic_tire_identification_dataset()?;
    let steady_identification = identify_tire_dataset(&steady_dataset)?;
    let tire = steady_identification.result.tire_spec;
    let run = |phase| synthetic_run(tire, axis, 0.35, phase);
    let mut dataset = TireRelaxationDataset {
        kind: TIRE_RELAXATION_DATASET_KIND.into(),
        schema_version: TIRE_RELAXATION_SCHEMA_VERSION,
        dataset_id: match axis {
            TireRelaxationAxis::Longitudinal => {
                "rne.synthetic.tire.relaxation.longitudinal.v1".into()
            }
            TireRelaxationAxis::Lateral => "rne.synthetic.tire.relaxation.lateral.v1".into(),
        },
        source_kind: TireDatasetSourceKind::SyntheticFixture,
        source_description: "deterministic generated transient fixture; not physical data".into(),
        force_model: "rne_combined_slip_tanh_ellipse_relaxation_v1".into(),
        axis,
        steady_dataset,
        steady_identification,
        training_runs: vec![TireRelaxationRunData {
            acquisition_id: 101,
            condition_id: 10,
            samples: run(0)?,
        }],
        holdout_runs: vec![
            TireRelaxationRunData {
                acquisition_id: 102,
                condition_id: 20,
                samples: run(1)?,
            },
            TireRelaxationRunData {
                acquisition_id: 103,
                condition_id: 30,
                samples: run(2)?,
            },
        ],
        content_sha256: String::new(),
    };
    dataset.seal()?;
    dataset.validate()?;
    Ok(dataset)
}

/// Fits and seals a verified transient dataset.
pub fn identify_tire_relaxation_dataset(
    dataset: &TireRelaxationDataset,
) -> Result<TireRelaxationEvidence> {
    dataset.validate()?;
    let identification_spec = tire_relaxation_identification_spec();
    let result = identify(dataset, identification_spec)?;
    let mut evidence = TireRelaxationEvidence {
        kind: TIRE_RELAXATION_RESULT_KIND.into(),
        schema_version: TIRE_RELAXATION_SCHEMA_VERSION,
        dataset_content_sha256: dataset.content_sha256.clone(),
        source_kind: dataset.source_kind,
        recorded_source_claim: dataset.source_kind.is_physical_measurement(),
        axis: dataset.axis,
        identification_spec,
        result,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = digest_without_hash(&evidence)?;
    evidence.validate(dataset)?;
    Ok(evidence)
}

/// Runs transient identification and physical-file verification as one transaction.
pub fn qualify_tire_relaxation_dataset(
    dataset: &TireRelaxationDataset,
    manifest: &TireRelaxationAcquisitionManifest,
    evidence_root: &Path,
) -> Result<TireRelaxationPhysicalQualificationEvidence> {
    manifest.verify_files(dataset, evidence_root)?;
    let identification = identify_tire_relaxation_dataset(dataset)?;
    ensure!(
        identification.recorded_source_claim,
        "tire relaxation qualification requires recorded source"
    );
    let mut evidence = TireRelaxationPhysicalQualificationEvidence {
        kind: TIRE_RELAXATION_PHYSICAL_QUALIFICATION_KIND.into(),
        schema_version: TIRE_RELAXATION_SCHEMA_VERSION,
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

fn identify(
    dataset: &TireRelaxationDataset,
    spec: TireRelaxationIdentificationSpec,
) -> Result<TireRelaxationIdentificationResult> {
    let training = dataset
        .training_runs
        .iter()
        .map(TireRelaxationRunData::borrowed)
        .collect::<Vec<_>>();
    let holdout = dataset
        .holdout_runs
        .iter()
        .map(TireRelaxationRunData::borrowed)
        .collect::<Vec<_>>();
    Ok(identify_tire_relaxation_length(
        spec,
        dataset.steady_identification.result.tire_spec,
        dataset.axis,
        &training,
        &holdout,
    )?)
}

fn valid_sample(sample: &TireRelaxationIdentificationSample) -> bool {
    sample.capture_time_s.is_finite()
        && sample.transport_speed_m_s.is_finite()
        && sample.target_slip.is_finite()
        && sample.normal_load_n.is_finite()
        && sample.road_friction_scale.is_finite()
        && sample.measured_force_n.is_finite()
}

fn synthetic_run(
    tire: CombinedSlipTireSpec,
    axis: TireRelaxationAxis,
    relaxation_length_m: f64,
    phase: usize,
) -> Result<Vec<TireRelaxationIdentificationSample>> {
    let targets = [0.15, -0.12, 0.08, -0.15];
    let dt_s = 0.01;
    let mut relaxed_slip = 0.0;
    (0..240)
        .map(|index| {
            let target_slip = targets[((index / 30) + phase) % targets.len()];
            let transport_speed_m_s = 4.0 + (index % 17) as f64 * 0.02;
            let (longitudinal_slip, lateral_slip) = match axis {
                TireRelaxationAxis::Longitudinal => (relaxed_slip, 0.0),
                TireRelaxationAxis::Lateral => (0.0, relaxed_slip),
            };
            let force = evaluate_combined_slip_tire_steady_force(
                tire,
                longitudinal_slip,
                lateral_slip,
                1_000.0,
                1.0,
            )?;
            let sample = TireRelaxationIdentificationSample {
                capture_time_s: index as f64 * dt_s,
                transport_speed_m_s,
                target_slip,
                normal_load_n: 1_000.0,
                road_friction_scale: 1.0,
                measured_force_n: match axis {
                    TireRelaxationAxis::Longitudinal => force.0,
                    TireRelaxationAxis::Lateral => force.1,
                },
            };
            relaxed_slip = target_slip
                + (relaxed_slip - target_slip)
                    * (-transport_speed_m_s * dt_s / relaxation_length_m).exp();
            Ok(sample)
        })
        .collect()
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
    TireRelaxationDataset,
    TireRelaxationEvidence,
    TireRelaxationAcquisitionManifest,
    TireRelaxationPhysicalQualificationEvidence,
);

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    fn file_ref(root: &Path, name: &str, bytes: &[u8]) -> TireEvidenceFileRef {
        fs::write(root.join(name), bytes).unwrap();
        TireEvidenceFileRef {
            path: name.into(),
            size_bytes: bytes.len() as u64,
            sha256: format!("sha256:{:x}", Sha256::digest(bytes)),
        }
    }

    fn signal(
        kind: TireRelaxationSignalKind,
        axis: TireRelaxationAxis,
        artifact: &TireEvidenceFileRef,
    ) -> TireRelaxationSignalEvidence {
        let derived = matches!(
            kind,
            TireRelaxationSignalKind::TransportSpeed | TireRelaxationSignalKind::TargetSlip
        );
        TireRelaxationSignalEvidence {
            signal: kind,
            sensor_id: format!("test.{kind:?}").to_ascii_lowercase(),
            unit: kind.unit().into(),
            convention: kind.convention(axis).into(),
            origin: if derived {
                TireSignalOrigin::Derived
            } else {
                TireSignalOrigin::Measured
            },
            source_sample_rate_hz: 1_000.0,
            converted_sample_rate_hz: 100.0,
            resolution_si: 1.0e-6,
            expanded_uncertainty_si: 1.0e-4,
            calibration_kind: if derived {
                TireCalibrationKind::DerivedSignalProcedure
            } else {
                TireCalibrationKind::InSituVerification
            },
            calibration_artifact: artifact.clone(),
        }
    }

    #[test]
    fn synthetic_artifact_is_honest_repeatable_and_self_verifying() {
        let dataset = synthetic_tire_relaxation_dataset().unwrap();
        let first = identify_tire_relaxation_dataset(&dataset).unwrap();
        let second = identify_tire_relaxation_dataset(&dataset).unwrap();
        assert_eq!(first, second);
        assert!(!first.recorded_source_claim);
        assert!((first.result.relaxation_length_m - 0.35).abs() < 1.0e-12);
        first.validate(&dataset).unwrap();
    }

    #[test]
    fn dataset_and_evidence_tampering_are_rejected() {
        let dataset = synthetic_tire_relaxation_dataset().unwrap();
        let evidence = identify_tire_relaxation_dataset(&dataset).unwrap();
        let mut tampered_dataset = dataset.clone();
        tampered_dataset.holdout_runs[0].samples[0].measured_force_n += 1.0;
        assert!(tampered_dataset.validate().is_err());
        let mut tampered_evidence = evidence;
        tampered_evidence.recorded_source_claim = true;
        assert!(tampered_evidence.validate(&dataset).is_err());
    }

    #[test]
    fn bounded_decoder_rejects_unknown_fields_and_oversize() {
        let dataset = synthetic_tire_relaxation_dataset().unwrap();
        let mut value = serde_json::to_value(dataset).unwrap();
        value["unknown"] = serde_json::json!(true);
        assert!(
            decode_tire_relaxation_dataset(serde_json::to_string(&value).unwrap().as_bytes())
                .is_err()
        );
        assert!(
            decode_tire_relaxation_dataset(&vec![b' '; MAX_TIRE_RELAXATION_DATASET_BYTES + 1])
                .is_err()
        );
    }

    #[test]
    fn joined_physical_gate_streams_files_and_rejects_escape() {
        let root = tempfile::tempdir().unwrap();
        let raw = file_ref(root.path(), "capture.mf4", b"test-only transient capture");
        let road = file_ref(root.path(), "road.txt", b"test-only road characterization");
        let calibration = file_ref(root.path(), "calibration.txt", b"test-only calibration");
        let procedure = file_ref(root.path(), "procedure.txt", b"test-only procedure");
        let mut dataset = synthetic_tire_relaxation_dataset().unwrap();
        dataset.source_kind = TireDatasetSourceKind::RecordedBench;
        dataset.source_description =
            "unit-test bytes labelled as bench records; not release evidence".into();
        dataset.steady_dataset.source_kind = TireDatasetSourceKind::RecordedBench;
        dataset.steady_dataset.source_description =
            "unit-test bytes labelled as bench records; not release evidence".into();
        dataset.steady_dataset.seal().unwrap();
        dataset.steady_identification = identify_tire_dataset(&dataset.steady_dataset).unwrap();
        dataset.seal().unwrap();
        let kinds = [
            TireRelaxationSignalKind::TransportSpeed,
            TireRelaxationSignalKind::TargetSlip,
            TireRelaxationSignalKind::NormalLoad,
            TireRelaxationSignalKind::MeasuredAxisForce,
        ];
        let runs = dataset
            .training_runs
            .iter()
            .chain(&dataset.holdout_runs)
            .map(|run| TireRelaxationRunAcquisitionEvidence {
                acquisition_id: run.acquisition_id,
                condition_id: run.condition_id,
                tire_id: "test.tire.installation".into(),
                clock_kind: TireCaptureClockKind::SharedHardware,
                all_channels_synchronized: true,
                maximum_timestamp_uncertainty_s: 0.000_1,
                raw_capture_format: TireRawCaptureFormat::Mdf4,
                raw_capture: raw.clone(),
                raw_segment_id: format!("segment.{}", run.acquisition_id),
                road_friction_evidence_kind: RoadFrictionEvidenceKind::CalibratedBenchSurface,
                road_friction_artifact: road.clone(),
                signals: kinds
                    .into_iter()
                    .map(|kind| signal(kind, dataset.axis, &calibration))
                    .collect(),
            })
            .collect();
        let mut manifest = TireRelaxationAcquisitionManifest {
            kind: TIRE_RELAXATION_ACQUISITION_MANIFEST_KIND.into(),
            schema_version: TIRE_RELAXATION_SCHEMA_VERSION,
            capture_id: "test.capture".into(),
            source_kind: TireDatasetSourceKind::RecordedBench,
            dataset_content_sha256: dataset.content_sha256.clone(),
            vehicle_id: "test.rig".into(),
            data_logger_id: "test.logger".into(),
            logger_software: "unit-test logger 1.0".into(),
            acquisition_procedure: procedure,
            runs,
            rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
            content_sha256: String::new(),
        };
        manifest.seal().unwrap();
        let qualification =
            qualify_tire_relaxation_dataset(&dataset, &manifest, root.path()).unwrap();
        assert!(qualification.physical_measurement);
        qualification
            .validate(&dataset, &manifest, root.path())
            .unwrap();

        manifest.runs[0].raw_capture.path = "../escape.mf4".into();
        manifest.seal().unwrap();
        assert!(manifest.validate(&dataset).is_err());
    }
}

//! File-verified tire qualification bound to cross-backend execution evidence.

use crate::identified_tire_backend::{
    run_identified_tire_backend_comparison, IdentifiedTireBackendComparison,
    IdentifiedTireProfileEvidence, IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION,
    IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1,
};
use crate::tire_acquisition::{
    qualify_tire_dataset, TirePhysicalAcquisitionManifest, TirePhysicalQualificationEvidence,
};
use crate::tire_load_sensitivity_acquisition::{
    qualify_tire_load_sensitivity_dataset, TireLoadSensitivityAcquisitionManifest,
    TireLoadSensitivityPhysicalQualificationEvidence,
};
use crate::tire_relaxation::{
    qualify_tire_relaxation_dataset, TireRelaxationAcquisitionManifest,
    TireRelaxationPhysicalQualificationEvidence,
};
use anyhow::{ensure, Context, Result};
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use rne_robot::TireRelaxationAxis;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::path::Path;

/// Stable physical application-request discriminator.
pub const PHYSICAL_TIRE_APPLICATION_REQUEST_KIND: &str =
    "rne_mobility_physical_tire_application_request";
/// Stable file-verified profile discriminator.
pub const PHYSICALLY_QUALIFIED_TIRE_PROFILE_KIND: &str =
    "rne_mobility_physically_qualified_tire_profile";
/// Stable file-verified cross-backend execution discriminator.
pub const PHYSICAL_TIRE_BACKEND_COMPARISON_KIND: &str =
    "rne_mobility_physical_tire_backend_comparison";
/// Legacy schema binding profile v1 to exactly three acquisition manifests.
pub const PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION_V1: u32 = 1;
/// Current schema binding profile v2 to all four acquisition manifests.
pub const PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION: u32 = 2;
/// Maximum accepted serialized physical application request.
pub const MAX_PHYSICAL_TIRE_APPLICATION_REQUEST_BYTES: usize = 80 * 1024 * 1024;

/// The acquisition manifests required before an identified tire can claim physical use.
///
/// Schema v1 binds profile v1 to exactly the steady, longitudinal-relaxation, and
/// lateral-relaxation manifests, byte-compatible with every existing three-manifest request.
/// Schema v2 additionally requires the load-sweep acquisition manifest bound to profile v2's
/// owned load-sensitivity dataset. The two versions are mutually exclusive and fail closed on
/// any mismatch between `schema_version`, `profile.schema_version`, and the presence of
/// `load_sensitivity_acquisition`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicalTireApplicationRequest {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema: `1` for the legacy three-manifest request, `2` for the four-manifest
    /// load-sensitive request.
    pub schema_version: u32,
    /// Exact profile that will be executed; v1 or v2 depending on `schema_version`.
    pub profile: IdentifiedTireProfileEvidence,
    /// Physical acquisition manifest for the owned steady-force dataset.
    pub steady_acquisition: TirePhysicalAcquisitionManifest,
    /// Physical acquisition manifest for the owned load-sweep dataset; schema v2 only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub load_sensitivity_acquisition: Option<TireLoadSensitivityAcquisitionManifest>,
    /// Physical acquisition manifest for longitudinal relaxation.
    pub longitudinal_acquisition: TireRelaxationAcquisitionManifest,
    /// Physical acquisition manifest for lateral relaxation.
    pub lateral_acquisition: TireRelaxationAcquisitionManifest,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl PhysicalTireApplicationRequest {
    /// Recomputes and stores the self-excluding request digest.
    pub fn seal(&mut self) -> Result<()> {
        self.content_sha256 = request_digest(self)?;
        Ok(())
    }

    /// Validates all fit and manifest bindings without asserting that files were read.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == PHYSICAL_TIRE_APPLICATION_REQUEST_KIND
                && matches!(
                    self.schema_version,
                    PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION_V1
                        | PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION
                ),
            "physical tire application request kind/schema drift"
        );
        self.profile.validate()?;
        ensure!(
            self.profile.longitudinal_dataset.axis == TireRelaxationAxis::Longitudinal
                && self.profile.lateral_dataset.axis == TireRelaxationAxis::Lateral,
            "physical tire application axis drift"
        );
        match self.schema_version {
            PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION_V1 => {
                ensure!(
                    self.profile.schema_version == IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1,
                    "three-manifest physical gate cannot qualify load-sensitive profile v2"
                );
                ensure!(
                    self.load_sensitivity_acquisition.is_none(),
                    "three-manifest physical request cannot carry a load-sensitivity manifest"
                );
            }
            PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION => {
                ensure!(
                    self.profile.schema_version == IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION,
                    "four-manifest physical gate requires load-sensitive profile v2"
                );
                let load_dataset = self
                    .profile
                    .load_sensitivity_dataset
                    .as_ref()
                    .context("load-sensitive profile v2 lacks an owned load-sweep dataset")?;
                let load_acquisition = self.load_sensitivity_acquisition.as_ref().context(
                    "four-manifest physical request requires a load-sensitivity manifest",
                )?;
                load_acquisition.validate(load_dataset)?;
                ensure!(
                    load_acquisition.source_kind == load_dataset.source_kind,
                    "physical tire load-sensitivity acquisition source mismatch"
                );
            }
            _ => unreachable!("schema_version checked above"),
        }
        let steady_dataset = &self.profile.longitudinal_dataset.steady_dataset;
        self.steady_acquisition.validate(steady_dataset)?;
        self.longitudinal_acquisition
            .validate(&self.profile.longitudinal_dataset)?;
        self.lateral_acquisition
            .validate(&self.profile.lateral_dataset)?;
        ensure!(
            self.steady_acquisition.source_kind == self.profile.longitudinal_dataset.source_kind
                && self.longitudinal_acquisition.source_kind
                    == self.profile.longitudinal_dataset.source_kind
                && self.lateral_acquisition.source_kind == self.profile.lateral_dataset.source_kind,
            "physical tire acquisition source mismatch"
        );
        ensure!(
            self.content_sha256 == request_digest(self)?,
            "physical tire application request digest drift"
        );
        Ok(())
    }
}

/// Binds one identified profile v1 to its steady and two axis-specific acquisition manifests.
///
/// This is the legacy three-manifest construction path, retained byte-compatible with every
/// existing profile-v1 physical request. It validates every artifact relationship before
/// sealing the request, but deliberately does not open retained evidence files;
/// [`qualify_physical_tire_profile`] performs that separate step.
pub fn build_physical_tire_application_request(
    profile: IdentifiedTireProfileEvidence,
    steady_acquisition: TirePhysicalAcquisitionManifest,
    longitudinal_acquisition: TireRelaxationAcquisitionManifest,
    lateral_acquisition: TireRelaxationAcquisitionManifest,
) -> Result<PhysicalTireApplicationRequest> {
    profile.validate()?;
    ensure!(
        profile.schema_version == IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1,
        "three-manifest physical gate cannot qualify load-sensitive profile v2"
    );
    steady_acquisition.validate(&profile.longitudinal_dataset.steady_dataset)?;
    longitudinal_acquisition.validate(&profile.longitudinal_dataset)?;
    lateral_acquisition.validate(&profile.lateral_dataset)?;
    let mut request = PhysicalTireApplicationRequest {
        kind: PHYSICAL_TIRE_APPLICATION_REQUEST_KIND.into(),
        schema_version: PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION_V1,
        profile,
        steady_acquisition,
        load_sensitivity_acquisition: None,
        longitudinal_acquisition,
        lateral_acquisition,
        content_sha256: String::new(),
    };
    request.seal()?;
    request.validate()?;
    Ok(request)
}

/// Binds one identified profile v2 to its steady, load-sweep, and two axis-specific manifests.
///
/// This is the four-manifest construction path required to physically qualify load-sensitive
/// profile v2. It validates every artifact relationship, including the load-sweep manifest's
/// binding to the profile's owned load-sensitivity dataset, before sealing the request.
pub fn build_load_sensitive_physical_tire_application_request(
    profile: IdentifiedTireProfileEvidence,
    steady_acquisition: TirePhysicalAcquisitionManifest,
    load_sensitivity_acquisition: TireLoadSensitivityAcquisitionManifest,
    longitudinal_acquisition: TireRelaxationAcquisitionManifest,
    lateral_acquisition: TireRelaxationAcquisitionManifest,
) -> Result<PhysicalTireApplicationRequest> {
    profile.validate()?;
    ensure!(
        profile.schema_version == IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION,
        "four-manifest physical gate requires load-sensitive profile v2"
    );
    let load_dataset = profile
        .load_sensitivity_dataset
        .as_ref()
        .context("load-sensitive profile v2 lacks an owned load-sweep dataset")?;
    steady_acquisition.validate(&profile.longitudinal_dataset.steady_dataset)?;
    load_sensitivity_acquisition.validate(load_dataset)?;
    longitudinal_acquisition.validate(&profile.longitudinal_dataset)?;
    lateral_acquisition.validate(&profile.lateral_dataset)?;
    let mut request = PhysicalTireApplicationRequest {
        kind: PHYSICAL_TIRE_APPLICATION_REQUEST_KIND.into(),
        schema_version: PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION,
        profile,
        steady_acquisition,
        load_sensitivity_acquisition: Some(load_sensitivity_acquisition),
        longitudinal_acquisition,
        lateral_acquisition,
        content_sha256: String::new(),
    };
    request.seal()?;
    request.validate()?;
    Ok(request)
}

/// Joined, file-verified qualification for steady, load sensitivity, and both transient axes.
///
/// `load_sensitivity_qualification` is present only for a schema-v2 request; validation fails
/// closed if its presence disagrees with `schema_version`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicallyQualifiedTireProfile {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema; copied from the qualified request.
    pub schema_version: u32,
    /// Exact request digest qualified by this transaction.
    pub request_content_sha256: String,
    /// Exact identified profile eligible for execution.
    pub profile: IdentifiedTireProfileEvidence,
    /// Recomputed steady-force physical qualification.
    pub steady_qualification: TirePhysicalQualificationEvidence,
    /// Recomputed load-sensitivity physical qualification; schema v2 only.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub load_sensitivity_qualification: Option<TireLoadSensitivityPhysicalQualificationEvidence>,
    /// Recomputed longitudinal-relaxation physical qualification.
    pub longitudinal_qualification: TireRelaxationPhysicalQualificationEvidence,
    /// Recomputed lateral-relaxation physical qualification.
    pub lateral_qualification: TireRelaxationPhysicalQualificationEvidence,
    /// True only after every required manifest closure has been streamed and verified.
    pub physical_measurement: bool,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl PhysicallyQualifiedTireProfile {
    /// Replays all fits and streams every retained file beneath the supplied root.
    pub fn validate(&self, request: &PhysicalTireApplicationRequest, root: &Path) -> Result<()> {
        request.validate()?;
        ensure!(
            self.kind == PHYSICALLY_QUALIFIED_TIRE_PROFILE_KIND
                && self.schema_version == request.schema_version,
            "physical tire qualification kind/schema drift"
        );
        ensure!(
            self.physical_measurement
                && self.request_content_sha256 == request.content_sha256
                && self.profile == request.profile,
            "physical tire qualification request binding drift"
        );
        let steady_dataset = &request.profile.longitudinal_dataset.steady_dataset;
        self.steady_qualification
            .validate(steady_dataset, &request.steady_acquisition, root)?;
        self.longitudinal_qualification.validate(
            &request.profile.longitudinal_dataset,
            &request.longitudinal_acquisition,
            root,
        )?;
        self.lateral_qualification.validate(
            &request.profile.lateral_dataset,
            &request.lateral_acquisition,
            root,
        )?;
        match self.schema_version {
            PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION_V1 => {
                ensure!(
                    self.load_sensitivity_qualification.is_none(),
                    "three-manifest qualification cannot carry a load-sensitivity qualification"
                );
            }
            PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION => {
                let load_dataset = request
                    .profile
                    .load_sensitivity_dataset
                    .as_ref()
                    .context("load-sensitive profile v2 lacks an owned load-sweep dataset")?;
                let load_acquisition = request
                    .load_sensitivity_acquisition
                    .as_ref()
                    .context("four-manifest request lacks a load-sensitivity manifest")?;
                let load_qualification = self.load_sensitivity_qualification.as_ref().context(
                    "four-manifest qualification lacks a load-sensitivity qualification",
                )?;
                load_qualification.validate(load_dataset, load_acquisition, root)?;
                ensure!(
                    load_qualification.identification
                        == request
                            .profile
                            .load_sensitivity_identification
                            .clone()
                            .context("load-sensitive profile v2 lacks load-sensitivity evidence")?,
                    "physical tire load-sensitivity qualification fit binding drift"
                );
            }
            _ => unreachable!("schema_version checked by request.validate()"),
        }
        ensure!(
            self.steady_qualification.identification
                == request.profile.longitudinal_dataset.steady_identification
                && self.longitudinal_qualification.identification
                    == request.profile.longitudinal_identification
                && self.lateral_qualification.identification
                    == request.profile.lateral_identification,
            "physical tire qualification fit binding drift"
        );
        ensure!(
            self.content_sha256 == qualification_digest(self)?,
            "physical tire qualification digest drift"
        );
        Ok(())
    }
}

/// Physical qualification plus same-profile cross-backend execution.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicalTireBackendComparison {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema; copied from the qualified request.
    pub schema_version: u32,
    /// File-verified profile qualification.
    pub qualification: PhysicallyQualifiedTireProfile,
    /// Rapier/MuJoCo same-TaskSpec execution evidence.
    pub execution: IdentifiedTireBackendComparison,
    /// True only when qualification and both backend executions are bound and passing.
    pub physical_measurement: bool,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl PhysicalTireBackendComparison {
    /// Revalidates physical files, fit chains, exact plant application, and SI tolerances.
    pub fn validate(&self, request: &PhysicalTireApplicationRequest, root: &Path) -> Result<()> {
        ensure!(
            self.kind == PHYSICAL_TIRE_BACKEND_COMPARISON_KIND
                && self.schema_version == request.schema_version,
            "physical tire backend comparison kind/schema drift"
        );
        self.qualification.validate(request, root)?;
        self.execution.validate()?;
        ensure!(
            self.physical_measurement
                && self.qualification.schema_version == self.schema_version
                && self.execution.profile == self.qualification.profile
                && self.execution.comparison.passed,
            "physical tire backend execution binding drift"
        );
        ensure!(
            self.content_sha256 == comparison_digest(self)?,
            "physical tire backend comparison digest drift"
        );
        Ok(())
    }
}

/// Decodes and validates one bounded request before any external files are opened.
pub fn decode_physical_tire_application_request(
    bytes: &[u8],
) -> Result<PhysicalTireApplicationRequest> {
    ensure!(
        bytes.len() <= MAX_PHYSICAL_TIRE_APPLICATION_REQUEST_BYTES,
        "physical tire application request exceeds byte limit"
    );
    let request: PhysicalTireApplicationRequest = serde_json::from_slice(bytes)?;
    request.validate()?;
    Ok(request)
}

/// Streams every required acquisition closure and emits one executable physical qualification.
///
/// Schema v1 streams exactly the steady, longitudinal, and lateral manifests; schema v2
/// additionally streams the load-sweep manifest. Every retained file is re-hashed here,
/// regardless of any previously serialized `physical_measurement` value.
pub fn qualify_physical_tire_profile(
    request: &PhysicalTireApplicationRequest,
    root: &Path,
) -> Result<PhysicallyQualifiedTireProfile> {
    request.validate()?;
    let steady_qualification = qualify_tire_dataset(
        &request.profile.longitudinal_dataset.steady_dataset,
        &request.steady_acquisition,
        root,
    )?;
    let load_sensitivity_qualification = match request.schema_version {
        PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION => {
            let load_dataset = request
                .profile
                .load_sensitivity_dataset
                .as_ref()
                .context("load-sensitive profile v2 lacks an owned load-sweep dataset")?;
            let load_acquisition = request
                .load_sensitivity_acquisition
                .as_ref()
                .context("four-manifest request lacks a load-sensitivity manifest")?;
            Some(qualify_tire_load_sensitivity_dataset(
                load_dataset,
                load_acquisition,
                root,
            )?)
        }
        _ => None,
    };
    let longitudinal_qualification = qualify_tire_relaxation_dataset(
        &request.profile.longitudinal_dataset,
        &request.longitudinal_acquisition,
        root,
    )?;
    let lateral_qualification = qualify_tire_relaxation_dataset(
        &request.profile.lateral_dataset,
        &request.lateral_acquisition,
        root,
    )?;
    let mut qualification = PhysicallyQualifiedTireProfile {
        kind: PHYSICALLY_QUALIFIED_TIRE_PROFILE_KIND.into(),
        schema_version: request.schema_version,
        request_content_sha256: request.content_sha256.clone(),
        profile: request.profile.clone(),
        steady_qualification,
        load_sensitivity_qualification,
        longitudinal_qualification,
        lateral_qualification,
        physical_measurement: true,
        content_sha256: String::new(),
    };
    qualification.content_sha256 = qualification_digest(&qualification)?;
    qualification.validate(request, root)?;
    Ok(qualification)
}

/// Qualifies the files, then executes the exact profile on two distinct backends.
pub fn run_physical_tire_backend_comparison<B1: PhysicsBackend, B2: PhysicsBackend>(
    first_backend: B1,
    first_manifest: PhysicsBackendManifest,
    second_backend: B2,
    second_manifest: PhysicsBackendManifest,
    request: &PhysicalTireApplicationRequest,
    root: &Path,
) -> Result<PhysicalTireBackendComparison> {
    let qualification = qualify_physical_tire_profile(request, root)?;
    let execution = run_identified_tire_backend_comparison(
        first_backend,
        first_manifest,
        second_backend,
        second_manifest,
        qualification.profile.clone(),
    )?;
    ensure!(
        execution.comparison.passed,
        "physical tire backend tolerance exceeded"
    );
    let mut evidence = PhysicalTireBackendComparison {
        kind: PHYSICAL_TIRE_BACKEND_COMPARISON_KIND.into(),
        schema_version: qualification.schema_version,
        qualification,
        execution,
        physical_measurement: true,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = comparison_digest(&evidence)?;
    evidence.validate(request, root)?;
    Ok(evidence)
}

fn request_digest(request: &PhysicalTireApplicationRequest) -> Result<String> {
    let mut canonical = request.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn qualification_digest(qualification: &PhysicallyQualifiedTireProfile) -> Result<String> {
    let mut canonical = qualification.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn comparison_digest(comparison: &PhysicalTireBackendComparison) -> Result<String> {
    let mut canonical = comparison.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn sha256(bytes: &[u8]) -> Result<String> {
    Ok(format!("sha256:{:x}", Sha256::digest(bytes)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identified_tire_backend::{
        build_identified_tire_profile, build_load_sensitive_identified_tire_profile,
        run_identified_tire_backend_trace, synthetic_identified_tire_profile,
        synthetic_load_sensitive_identified_tire_profile,
    };
    use crate::tire_acquisition::{
        RoadFrictionEvidenceKind, TireCalibrationKind, TireCaptureClockKind, TireEvidenceFileRef,
        TireRawCaptureFormat, TireRunAcquisitionEvidence, TireSignalEvidence, TireSignalKind,
        TireSignalOrigin, TIRE_ACQUISITION_MANIFEST_KIND, TIRE_ACQUISITION_SCHEMA_VERSION,
    };
    use crate::tire_identification::{
        identify_tire_dataset, synthetic_tire_identification_dataset, TireDatasetSourceKind,
    };
    use crate::tire_load_sensitivity::synthetic_tire_load_sensitivity_dataset;
    use crate::tire_load_sensitivity_acquisition::{
        TireLoadSensitivityRunAcquisitionEvidence, TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_KIND,
        TIRE_LOAD_SENSITIVITY_ACQUISITION_SCHEMA_VERSION,
    };
    use crate::tire_relaxation::{
        synthetic_tire_relaxation_dataset_for_axis, TireRelaxationRunAcquisitionEvidence,
        TireRelaxationSignalEvidence, TireRelaxationSignalKind,
        TIRE_RELAXATION_ACQUISITION_MANIFEST_KIND, TIRE_RELAXATION_SCHEMA_VERSION,
    };
    use rne_physics_rapier::RapierBackend;
    use std::fs;

    fn file_ref(root: &Path, name: &str, bytes: &[u8]) -> TireEvidenceFileRef {
        fs::write(root.join(name), bytes).unwrap();
        TireEvidenceFileRef {
            path: name.into(),
            size_bytes: bytes.len() as u64,
            sha256: format!("sha256:{:x}", Sha256::digest(bytes)),
        }
    }

    fn recorded_profile() -> IdentifiedTireProfileEvidence {
        let mut steady = synthetic_tire_identification_dataset().unwrap();
        steady.source_kind = TireDatasetSourceKind::RecordedBench;
        steady.source_description = "test-only recorded bench label; not physical evidence".into();
        steady.seal().unwrap();
        let steady_fit = identify_tire_dataset(&steady).unwrap();
        let transient = |axis| {
            let mut dataset = synthetic_tire_relaxation_dataset_for_axis(axis).unwrap();
            dataset.source_kind = TireDatasetSourceKind::RecordedBench;
            dataset.source_description =
                "test-only recorded transient label; not physical evidence".into();
            dataset.steady_dataset = steady.clone();
            dataset.steady_identification = steady_fit.clone();
            dataset.seal().unwrap();
            dataset
        };
        build_identified_tire_profile(
            transient(TireRelaxationAxis::Longitudinal),
            transient(TireRelaxationAxis::Lateral),
        )
        .unwrap()
    }

    fn recorded_load_sensitive_profile() -> IdentifiedTireProfileEvidence {
        let mut steady = synthetic_tire_identification_dataset().unwrap();
        steady.source_kind = TireDatasetSourceKind::RecordedBench;
        steady.source_description = "test-only recorded bench label; not physical evidence".into();
        steady.seal().unwrap();
        let steady_fit = identify_tire_dataset(&steady).unwrap();
        let transient = |axis| {
            let mut dataset = synthetic_tire_relaxation_dataset_for_axis(axis).unwrap();
            dataset.source_kind = TireDatasetSourceKind::RecordedBench;
            dataset.source_description =
                "test-only recorded transient label; not physical evidence".into();
            dataset.steady_dataset = steady.clone();
            dataset.steady_identification = steady_fit.clone();
            dataset.seal().unwrap();
            dataset
        };
        let mut load_dataset = synthetic_tire_load_sensitivity_dataset().unwrap();
        load_dataset.source_kind = TireDatasetSourceKind::RecordedBench;
        load_dataset.source_description =
            "test-only recorded load-sweep label; not physical evidence".into();
        load_dataset.steady_dataset = steady.clone();
        load_dataset.steady_identification = steady_fit.clone();
        load_dataset.seal().unwrap();
        build_load_sensitive_identified_tire_profile(
            load_dataset,
            transient(TireRelaxationAxis::Longitudinal),
            transient(TireRelaxationAxis::Lateral),
        )
        .unwrap()
    }

    fn steady_signals(calibration: &TireEvidenceFileRef) -> Vec<TireSignalEvidence> {
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
            sensor_id: format!("test.{signal:?}"),
            unit: signal.unit().into(),
            convention: signal.convention().into(),
            origin: TireSignalOrigin::Measured,
            source_sample_rate_hz: 1_000.0,
            converted_sample_rate_hz: 100.0,
            resolution_si: 1.0e-6,
            expanded_uncertainty_si: 1.0e-4,
            calibration_kind: TireCalibrationKind::InSituVerification,
            calibration_artifact: calibration.clone(),
        })
        .collect()
    }

    fn transient_signals(
        axis: TireRelaxationAxis,
        calibration: &TireEvidenceFileRef,
    ) -> Vec<TireRelaxationSignalEvidence> {
        [
            TireRelaxationSignalKind::TransportSpeed,
            TireRelaxationSignalKind::TargetSlip,
            TireRelaxationSignalKind::NormalLoad,
            TireRelaxationSignalKind::MeasuredAxisForce,
        ]
        .into_iter()
        .map(|signal| {
            let derived = matches!(
                signal,
                TireRelaxationSignalKind::TransportSpeed | TireRelaxationSignalKind::TargetSlip
            );
            TireRelaxationSignalEvidence {
                signal,
                sensor_id: format!("test.{signal:?}"),
                unit: signal.unit().into(),
                convention: signal.convention(axis).into(),
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
                calibration_artifact: calibration.clone(),
            }
        })
        .collect()
    }

    fn request_fixture(root: &Path) -> PhysicalTireApplicationRequest {
        let profile = recorded_profile();
        let raw = file_ref(root, "capture.mf4", b"test austere raw capture bytes");
        let road = file_ref(root, "road.txt", b"test road characterization");
        let calibration = file_ref(root, "calibration.txt", b"test calibration records");
        let procedure = file_ref(root, "procedure.txt", b"test acquisition procedure");
        let steady_dataset = &profile.longitudinal_dataset.steady_dataset;
        let mut steady_acquisition = TirePhysicalAcquisitionManifest {
            kind: TIRE_ACQUISITION_MANIFEST_KIND.into(),
            schema_version: TIRE_ACQUISITION_SCHEMA_VERSION,
            capture_id: "test.steady.capture".into(),
            source_kind: TireDatasetSourceKind::RecordedBench,
            dataset_content_sha256: steady_dataset.content_sha256.clone(),
            vehicle_id: "test.rig".into(),
            data_logger_id: "test.logger".into(),
            logger_software: "test logger 1.0".into(),
            acquisition_procedure: procedure.clone(),
            runs: steady_dataset
                .training_runs
                .iter()
                .chain(&steady_dataset.holdout_runs)
                .map(|run| TireRunAcquisitionEvidence {
                    acquisition_id: run.acquisition_id,
                    condition_id: run.condition_id,
                    road_friction_scale: run.road_friction_scale,
                    tire_id: "test.tire".into(),
                    clock_kind: TireCaptureClockKind::SharedHardware,
                    all_channels_synchronized: true,
                    maximum_timestamp_uncertainty_s: 0.000_1,
                    raw_capture_format: TireRawCaptureFormat::Mdf4,
                    raw_capture: raw.clone(),
                    raw_segment_id: format!("steady.{}", run.acquisition_id),
                    road_friction_evidence_kind: RoadFrictionEvidenceKind::CalibratedBenchSurface,
                    road_friction_artifact: road.clone(),
                    signals: steady_signals(&calibration),
                })
                .collect(),
            rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
            content_sha256: String::new(),
        };
        steady_acquisition.seal().unwrap();
        let transient_manifest = |dataset: &crate::tire_relaxation::TireRelaxationDataset,
                                  capture_id: &str| {
            let mut manifest = TireRelaxationAcquisitionManifest {
                kind: TIRE_RELAXATION_ACQUISITION_MANIFEST_KIND.into(),
                schema_version: TIRE_RELAXATION_SCHEMA_VERSION,
                capture_id: capture_id.into(),
                source_kind: TireDatasetSourceKind::RecordedBench,
                dataset_content_sha256: dataset.content_sha256.clone(),
                vehicle_id: "test.rig".into(),
                data_logger_id: "test.logger".into(),
                logger_software: "test logger 1.0".into(),
                acquisition_procedure: procedure.clone(),
                runs: dataset
                    .training_runs
                    .iter()
                    .chain(&dataset.holdout_runs)
                    .map(|run| TireRelaxationRunAcquisitionEvidence {
                        acquisition_id: run.acquisition_id,
                        condition_id: run.condition_id,
                        tire_id: "test.tire".into(),
                        clock_kind: TireCaptureClockKind::SharedHardware,
                        all_channels_synchronized: true,
                        maximum_timestamp_uncertainty_s: 0.000_1,
                        raw_capture_format: TireRawCaptureFormat::Mdf4,
                        raw_capture: raw.clone(),
                        raw_segment_id: format!("{capture_id}.{}", run.acquisition_id),
                        road_friction_evidence_kind:
                            RoadFrictionEvidenceKind::CalibratedBenchSurface,
                        road_friction_artifact: road.clone(),
                        signals: transient_signals(dataset.axis, &calibration),
                    })
                    .collect(),
                rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
                content_sha256: String::new(),
            };
            manifest.seal().unwrap();
            manifest
        };
        let longitudinal_acquisition =
            transient_manifest(&profile.longitudinal_dataset, "test.longitudinal.capture");
        let lateral_acquisition =
            transient_manifest(&profile.lateral_dataset, "test.lateral.capture");
        build_physical_tire_application_request(
            profile,
            steady_acquisition,
            longitudinal_acquisition,
            lateral_acquisition,
        )
        .unwrap()
    }

    fn request_fixture_v2(root: &Path) -> PhysicalTireApplicationRequest {
        let profile = recorded_load_sensitive_profile();
        let raw = file_ref(root, "capture.mf4", b"test austere raw capture bytes");
        let road = file_ref(root, "road.txt", b"test road characterization");
        let calibration = file_ref(root, "calibration.txt", b"test calibration records");
        let procedure = file_ref(root, "procedure.txt", b"test acquisition procedure");
        let steady_dataset = &profile.longitudinal_dataset.steady_dataset;
        let mut steady_acquisition = TirePhysicalAcquisitionManifest {
            kind: TIRE_ACQUISITION_MANIFEST_KIND.into(),
            schema_version: TIRE_ACQUISITION_SCHEMA_VERSION,
            capture_id: "test.steady.capture".into(),
            source_kind: TireDatasetSourceKind::RecordedBench,
            dataset_content_sha256: steady_dataset.content_sha256.clone(),
            vehicle_id: "test.rig".into(),
            data_logger_id: "test.logger".into(),
            logger_software: "test logger 1.0".into(),
            acquisition_procedure: procedure.clone(),
            runs: steady_dataset
                .training_runs
                .iter()
                .chain(&steady_dataset.holdout_runs)
                .map(|run| TireRunAcquisitionEvidence {
                    acquisition_id: run.acquisition_id,
                    condition_id: run.condition_id,
                    road_friction_scale: run.road_friction_scale,
                    tire_id: "test.tire".into(),
                    clock_kind: TireCaptureClockKind::SharedHardware,
                    all_channels_synchronized: true,
                    maximum_timestamp_uncertainty_s: 0.000_1,
                    raw_capture_format: TireRawCaptureFormat::Mdf4,
                    raw_capture: raw.clone(),
                    raw_segment_id: format!("steady.{}", run.acquisition_id),
                    road_friction_evidence_kind: RoadFrictionEvidenceKind::CalibratedBenchSurface,
                    road_friction_artifact: road.clone(),
                    signals: steady_signals(&calibration),
                })
                .collect(),
            rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
            content_sha256: String::new(),
        };
        steady_acquisition.seal().unwrap();
        let load_dataset = profile.load_sensitivity_dataset.as_ref().unwrap();
        let mut load_sensitivity_acquisition = TireLoadSensitivityAcquisitionManifest {
            kind: TIRE_LOAD_SENSITIVITY_ACQUISITION_MANIFEST_KIND.into(),
            schema_version: TIRE_LOAD_SENSITIVITY_ACQUISITION_SCHEMA_VERSION,
            capture_id: "test.load.capture".into(),
            source_kind: TireDatasetSourceKind::RecordedBench,
            dataset_content_sha256: load_dataset.content_sha256.clone(),
            vehicle_id: "test.rig".into(),
            data_logger_id: "test.logger".into(),
            logger_software: "test logger 1.0".into(),
            acquisition_procedure: procedure.clone(),
            runs: load_dataset
                .training_runs
                .iter()
                .chain(&load_dataset.holdout_runs)
                .map(|run| TireLoadSensitivityRunAcquisitionEvidence {
                    acquisition_id: run.acquisition_id,
                    condition_id: run.condition_id,
                    road_friction_scale: run.road_friction_scale,
                    tire_id: "test.tire".into(),
                    clock_kind: TireCaptureClockKind::SharedHardware,
                    all_channels_synchronized: true,
                    maximum_timestamp_uncertainty_s: 0.000_1,
                    raw_capture_format: TireRawCaptureFormat::Mdf4,
                    raw_capture: raw.clone(),
                    raw_segment_id: format!("load.{}", run.acquisition_id),
                    road_friction_evidence_kind: RoadFrictionEvidenceKind::CalibratedBenchSurface,
                    road_friction_artifact: road.clone(),
                    signals: steady_signals(&calibration),
                })
                .collect(),
            rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
            content_sha256: String::new(),
        };
        load_sensitivity_acquisition.seal().unwrap();
        let transient_manifest = |dataset: &crate::tire_relaxation::TireRelaxationDataset,
                                  capture_id: &str| {
            let mut manifest = TireRelaxationAcquisitionManifest {
                kind: TIRE_RELAXATION_ACQUISITION_MANIFEST_KIND.into(),
                schema_version: TIRE_RELAXATION_SCHEMA_VERSION,
                capture_id: capture_id.into(),
                source_kind: TireDatasetSourceKind::RecordedBench,
                dataset_content_sha256: dataset.content_sha256.clone(),
                vehicle_id: "test.rig".into(),
                data_logger_id: "test.logger".into(),
                logger_software: "test logger 1.0".into(),
                acquisition_procedure: procedure.clone(),
                runs: dataset
                    .training_runs
                    .iter()
                    .chain(&dataset.holdout_runs)
                    .map(|run| TireRelaxationRunAcquisitionEvidence {
                        acquisition_id: run.acquisition_id,
                        condition_id: run.condition_id,
                        tire_id: "test.tire".into(),
                        clock_kind: TireCaptureClockKind::SharedHardware,
                        all_channels_synchronized: true,
                        maximum_timestamp_uncertainty_s: 0.000_1,
                        raw_capture_format: TireRawCaptureFormat::Mdf4,
                        raw_capture: raw.clone(),
                        raw_segment_id: format!("{capture_id}.{}", run.acquisition_id),
                        road_friction_evidence_kind:
                            RoadFrictionEvidenceKind::CalibratedBenchSurface,
                        road_friction_artifact: road.clone(),
                        signals: transient_signals(dataset.axis, &calibration),
                    })
                    .collect(),
                rne_commit: "0123456789abcdef0123456789abcdef01234567".into(),
                content_sha256: String::new(),
            };
            manifest.seal().unwrap();
            manifest
        };
        let longitudinal_acquisition =
            transient_manifest(&profile.longitudinal_dataset, "test.longitudinal.capture");
        let lateral_acquisition =
            transient_manifest(&profile.lateral_dataset, "test.lateral.capture");
        build_load_sensitive_physical_tire_application_request(
            profile,
            steady_acquisition,
            load_sensitivity_acquisition,
            longitudinal_acquisition,
            lateral_acquisition,
        )
        .unwrap()
    }

    #[test]
    fn builder_rejects_axis_manifest_substitution_before_sealing() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture(root.path());
        assert!(build_physical_tire_application_request(
            request.profile,
            request.steady_acquisition,
            request.lateral_acquisition.clone(),
            request.lateral_acquisition,
        )
        .is_err());
    }

    #[test]
    fn three_manifest_builder_rejects_load_sensitive_profile_v2() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture(root.path());
        assert!(build_physical_tire_application_request(
            synthetic_load_sensitive_identified_tire_profile().unwrap(),
            request.steady_acquisition,
            request.longitudinal_acquisition,
            request.lateral_acquisition,
        )
        .is_err());
    }

    #[test]
    fn three_manifest_gate_streams_files_and_rejects_label_only_profile() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture(root.path());
        let qualification = qualify_physical_tire_profile(&request, root.path()).unwrap();
        assert!(qualification.physical_measurement);
        qualification.validate(&request, root.path()).unwrap();

        let mut label_only = request;
        label_only.profile = synthetic_identified_tire_profile().unwrap();
        label_only.seal().unwrap();
        assert!(label_only.validate().is_err());
    }

    #[test]
    fn retained_file_tampering_invalidates_qualified_profile() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture(root.path());
        let qualification = qualify_physical_tire_profile(&request, root.path()).unwrap();
        fs::write(
            root.path().join("calibration.txt"),
            b"changed calibration bytes",
        )
        .unwrap();
        assert!(qualification.validate(&request, root.path()).is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn physical_gate_precedes_same_profile_rapier_mujoco_execution() {
        use rne_core::SimDuration;
        use rne_physics_mujoco::MuJoCoBackend;

        let root = tempfile::tempdir().unwrap();
        let request = request_fixture(root.path());
        let evidence = run_physical_tire_backend_comparison(
            RapierBackend::new(),
            RapierBackend::manifest(),
            MuJoCoBackend::new(SimDuration::from_ticks(
                crate::backend::BACKEND_MOBILITY_FIXED_DELTA_TICKS,
            ))
            .unwrap(),
            MuJoCoBackend::manifest(),
            &request,
            root.path(),
        )
        .unwrap();
        assert!(evidence.physical_measurement);
        assert!(evidence.execution.comparison.passed);
        evidence.validate(&request, root.path()).unwrap();
    }

    #[test]
    fn four_manifest_builder_requires_load_sensitive_profile_v2() {
        let root = tempfile::tempdir().unwrap();
        let v2_request = request_fixture_v2(root.path());
        assert!(build_load_sensitive_physical_tire_application_request(
            synthetic_identified_tire_profile().unwrap(),
            v2_request.steady_acquisition,
            v2_request.load_sensitivity_acquisition.clone().unwrap(),
            v2_request.longitudinal_acquisition,
            v2_request.lateral_acquisition,
        )
        .is_err());
    }

    #[test]
    fn v2_request_missing_load_manifest_is_rejected() {
        let root = tempfile::tempdir().unwrap();
        let mut request = request_fixture_v2(root.path());
        request.load_sensitivity_acquisition = None;
        request.seal().unwrap();
        assert!(request.validate().is_err());
    }

    #[test]
    fn v1_request_cannot_carry_a_load_manifest() {
        let root = tempfile::tempdir().unwrap();
        let v1 = request_fixture(root.path());
        let v2 = request_fixture_v2(root.path());
        let mut smuggled = v1;
        smuggled.load_sensitivity_acquisition = v2.load_sensitivity_acquisition;
        smuggled.seal().unwrap();
        assert!(smuggled.validate().is_err());
    }

    #[test]
    fn four_manifest_gate_streams_files_and_qualifies_v2_profile() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture_v2(root.path());
        assert_eq!(
            request.schema_version,
            PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION
        );
        let qualification = qualify_physical_tire_profile(&request, root.path()).unwrap();
        assert!(qualification.physical_measurement);
        assert!(qualification.load_sensitivity_qualification.is_some());
        qualification.validate(&request, root.path()).unwrap();

        // Repeating qualification is deterministic.
        let repeat = qualify_physical_tire_profile(&request, root.path()).unwrap();
        assert_eq!(qualification, repeat);
    }

    #[test]
    fn four_manifest_gate_rejects_load_evidence_one_byte_tamper() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture_v2(root.path());
        let qualification = qualify_physical_tire_profile(&request, root.path()).unwrap();
        let raw_path = root.path().join("capture.mf4");
        let mut bytes = fs::read(&raw_path).unwrap();
        bytes[0] ^= 0x01;
        fs::write(&raw_path, bytes).unwrap();
        assert!(qualification.validate(&request, root.path()).is_err());
    }

    #[test]
    fn qualification_digest_tamper_is_rejected_for_v2() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture_v2(root.path());
        let mut qualification = qualify_physical_tire_profile(&request, root.path()).unwrap();
        let last = qualification.content_sha256.pop().unwrap();
        qualification
            .content_sha256
            .push(if last == '0' { '1' } else { '0' });
        assert!(qualification.validate(&request, root.path()).is_err());
    }

    #[test]
    fn headless_rapier_executes_qualified_v2_profile() {
        let root = tempfile::tempdir().unwrap();
        let request = request_fixture_v2(root.path());
        let qualification = qualify_physical_tire_profile(&request, root.path()).unwrap();
        let evidence = run_identified_tire_backend_trace(
            RapierBackend::new(),
            RapierBackend::manifest(),
            qualification.profile.clone(),
        )
        .unwrap();
        assert!(evidence.trace.passed, "{:#?}", evidence.trace.metrics);
        evidence.validate().unwrap();
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn physical_gate_precedes_v2_profile_rapier_mujoco_execution() {
        use rne_core::SimDuration;
        use rne_physics_mujoco::MuJoCoBackend;

        let root = tempfile::tempdir().unwrap();
        let request = request_fixture_v2(root.path());
        let evidence = run_physical_tire_backend_comparison(
            RapierBackend::new(),
            RapierBackend::manifest(),
            MuJoCoBackend::new(SimDuration::from_ticks(
                crate::backend::BACKEND_MOBILITY_FIXED_DELTA_TICKS,
            ))
            .unwrap(),
            MuJoCoBackend::manifest(),
            &request,
            root.path(),
        )
        .unwrap();
        assert!(evidence.physical_measurement);
        assert_eq!(
            evidence.schema_version,
            PHYSICAL_TIRE_APPLICATION_SCHEMA_VERSION
        );
        assert!(evidence.execution.comparison.passed);
        evidence.validate(&request, root.path()).unwrap();
    }
}

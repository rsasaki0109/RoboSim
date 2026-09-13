//! Application of identified steady and transient tire evidence to shared backend tasks.

use crate::backend::{
    backend_mobility_task_spec, backend_plant_spec, compare_backend_mobility_traces,
    run_backend_mobility_trace_configured, BackendMobilityComparison, BackendMobilityTrace,
};
use crate::tire_load_sensitivity::{
    identify_tire_load_sensitivity_dataset, synthetic_tire_load_sensitivity_dataset,
    TireLoadSensitivityDataset, TireLoadSensitivityEvidence,
};
use crate::tire_relaxation::{
    identify_tire_relaxation_dataset, synthetic_tire_relaxation_dataset_for_axis,
    TireRelaxationDataset, TireRelaxationEvidence,
};
use anyhow::{ensure, Context, Result};
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use rne_robot::{CombinedSlipTireSpec, TireRelaxationAxis};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};

/// Stable identified tire-profile artifact discriminator.
pub const IDENTIFIED_TIRE_PROFILE_KIND: &str = "rne_mobility_identified_tire_profile";
/// Stable identified-profile backend trace discriminator.
pub const IDENTIFIED_TIRE_BACKEND_TRACE_KIND: &str = "rne_mobility_identified_tire_backend_trace";
/// Stable identified-profile cross-backend comparison discriminator.
pub const IDENTIFIED_TIRE_BACKEND_COMPARISON_KIND: &str =
    "rne_mobility_identified_tire_backend_comparison";
/// Legacy profile schema containing steady and two-axis relaxation evidence.
pub const IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1: u32 = 1;
/// Current profile schema adding staged load-sensitivity evidence.
pub const IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION: u32 = 2;
/// Current schema for backend execution evidence.
pub const IDENTIFIED_TIRE_BACKEND_SCHEMA_VERSION: u32 = 1;
/// Maximum accepted serialized profile size.
pub const MAX_IDENTIFIED_TIRE_PROFILE_BYTES: usize = 128 * 1024 * 1024;

/// Self-contained staged tire profile with both transient axes and optional v2 load evidence.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentifiedTireProfileEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Owned longitudinal transient dataset.
    pub longitudinal_dataset: TireRelaxationDataset,
    /// Replayed longitudinal relaxation fit.
    pub longitudinal_identification: TireRelaxationEvidence,
    /// Owned lateral transient dataset.
    pub lateral_dataset: TireRelaxationDataset,
    /// Replayed lateral relaxation fit.
    pub lateral_identification: TireRelaxationEvidence,
    /// Owned load-sweep dataset for profile schema v2.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub load_sensitivity_dataset: Option<TireLoadSensitivityDataset>,
    /// Replayed load-sensitivity fit for profile schema v2.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub load_sensitivity_identification: Option<TireLoadSensitivityEvidence>,
    /// Exact steady parameters plus the staged load and longitudinal/lateral fits.
    pub tire_spec: CombinedSlipTireSpec,
    /// Source-label claim only; this is not physical acquisition qualification.
    pub recorded_source_claim: bool,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl IdentifiedTireProfileEvidence {
    /// Replays both fit chains and verifies common steady evidence and output parameters.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == IDENTIFIED_TIRE_PROFILE_KIND
                && matches!(
                    self.schema_version,
                    IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1
                        | IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION
                ),
            "identified tire profile kind/schema drift"
        );
        self.longitudinal_identification
            .validate(&self.longitudinal_dataset)
            .context("longitudinal tire relaxation evidence")?;
        self.lateral_identification
            .validate(&self.lateral_dataset)
            .context("lateral tire relaxation evidence")?;
        ensure!(
            self.longitudinal_dataset.axis == TireRelaxationAxis::Longitudinal
                && self.longitudinal_identification.axis == TireRelaxationAxis::Longitudinal
                && self.lateral_dataset.axis == TireRelaxationAxis::Lateral
                && self.lateral_identification.axis == TireRelaxationAxis::Lateral,
            "identified tire profile axis mismatch"
        );
        ensure!(
            self.longitudinal_dataset.steady_dataset == self.lateral_dataset.steady_dataset
                && self.longitudinal_dataset.steady_identification
                    == self.lateral_dataset.steady_identification,
            "relaxation axes do not share exact steady tire evidence"
        );
        let mut expected = self
            .longitudinal_dataset
            .steady_identification
            .result
            .tire_spec;
        let load_recorded_source_claim = match self.schema_version {
            IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1 => {
                ensure!(
                    self.load_sensitivity_dataset.is_none()
                        && self.load_sensitivity_identification.is_none(),
                    "legacy identified tire profile contains load-sensitivity evidence"
                );
                true
            }
            IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION => {
                let load_dataset = self
                    .load_sensitivity_dataset
                    .as_ref()
                    .context("load-sensitive profile lacks owned load-sweep dataset")?;
                let load_identification = self
                    .load_sensitivity_identification
                    .as_ref()
                    .context("load-sensitive profile lacks load-sensitivity evidence")?;
                load_identification
                    .validate(load_dataset)
                    .context("tire load-sensitivity evidence")?;
                ensure!(
                    load_dataset.steady_dataset == self.longitudinal_dataset.steady_dataset
                        && load_dataset.steady_identification
                            == self.longitudinal_dataset.steady_identification,
                    "load sensitivity and relaxation do not share exact steady tire evidence"
                );
                expected.load_sensitivity_per_load_ratio =
                    load_identification.result.load_sensitivity_per_load_ratio;
                load_identification.recorded_source_claim
            }
            _ => unreachable!("profile schema checked above"),
        };
        expected.longitudinal_relaxation_length_m =
            self.longitudinal_identification.result.relaxation_length_m;
        expected.lateral_relaxation_length_m =
            self.lateral_identification.result.relaxation_length_m;
        ensure!(
            self.tire_spec == expected,
            "identified tire parameter application drift"
        );
        let expected_recorded = self.longitudinal_identification.recorded_source_claim
            && self.lateral_identification.recorded_source_claim
            && load_recorded_source_claim;
        ensure!(
            self.recorded_source_claim == expected_recorded,
            "identified tire source claim drift"
        );
        ensure!(
            self.content_sha256 == profile_digest(self)?,
            "identified tire profile digest drift"
        );
        Ok(())
    }
}

/// One identified tire profile applied to one backend-neutral task execution.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentifiedTireBackendTrace {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact identified profile applied to the plant.
    pub profile: IdentifiedTireProfileEvidence,
    /// Complete backend trace whose plant is checked against the profile.
    pub trace: BackendMobilityTrace,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl IdentifiedTireBackendTrace {
    /// Verifies the fit chain, TaskSpec, applied plant, execution trace, and digest.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == IDENTIFIED_TIRE_BACKEND_TRACE_KIND
                && self.schema_version == IDENTIFIED_TIRE_BACKEND_SCHEMA_VERSION,
            "identified tire backend trace kind/schema drift"
        );
        self.profile.validate()?;
        self.trace.validate_execution()?;
        ensure!(
            self.trace.task_spec == backend_mobility_task_spec()
                && self.trace.plant == applied_plant(self.profile.tire_spec)
                && self.trace.seed == 0,
            "identified tire backend execution binding drift"
        );
        ensure!(
            self.content_sha256 == trace_digest(self)?,
            "identified tire backend trace digest drift"
        );
        Ok(())
    }
}

/// Same identified tire profile executed on two distinct physics backends.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentifiedTireBackendComparison {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact profile applied to both backend plants.
    pub profile: IdentifiedTireProfileEvidence,
    /// Complete same-TaskSpec comparison with SI-unit tolerances.
    pub comparison: BackendMobilityComparison,
    /// SHA-256 over compact JSON with this field empty.
    pub content_sha256: String,
}

impl IdentifiedTireBackendComparison {
    /// Verifies both executions use the exact profile and shared TaskSpec.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == IDENTIFIED_TIRE_BACKEND_COMPARISON_KIND
                && self.schema_version == IDENTIFIED_TIRE_BACKEND_SCHEMA_VERSION,
            "identified tire backend comparison kind/schema drift"
        );
        self.profile.validate()?;
        self.comparison.validate()?;
        let plant = applied_plant(self.profile.tire_spec);
        ensure!(
            self.comparison.first.task_spec == backend_mobility_task_spec()
                && self.comparison.second.task_spec == backend_mobility_task_spec()
                && self.comparison.first.plant == plant
                && self.comparison.second.plant == plant,
            "identified tire cross-backend application drift"
        );
        ensure!(
            self.content_sha256 == comparison_digest(self)?,
            "identified tire comparison digest drift"
        );
        Ok(())
    }
}

/// Builds and verifies the deterministic non-physical two-axis profile fixture.
pub fn synthetic_identified_tire_profile() -> Result<IdentifiedTireProfileEvidence> {
    let longitudinal_dataset =
        synthetic_tire_relaxation_dataset_for_axis(TireRelaxationAxis::Longitudinal)?;
    let lateral_dataset = synthetic_tire_relaxation_dataset_for_axis(TireRelaxationAxis::Lateral)?;
    build_identified_tire_profile(longitudinal_dataset, lateral_dataset)
}

/// Builds and verifies a deterministic non-physical v2 profile including load sensitivity.
pub fn synthetic_load_sensitive_identified_tire_profile() -> Result<IdentifiedTireProfileEvidence> {
    let load_sensitivity_dataset = synthetic_tire_load_sensitivity_dataset()?;
    let longitudinal_dataset =
        synthetic_tire_relaxation_dataset_for_axis(TireRelaxationAxis::Longitudinal)?;
    let lateral_dataset = synthetic_tire_relaxation_dataset_for_axis(TireRelaxationAxis::Lateral)?;
    build_load_sensitive_identified_tire_profile(
        load_sensitivity_dataset,
        longitudinal_dataset,
        lateral_dataset,
    )
}

/// Replays two axis datasets and assembles the only accepted combined tire profile.
pub fn build_identified_tire_profile(
    longitudinal_dataset: TireRelaxationDataset,
    lateral_dataset: TireRelaxationDataset,
) -> Result<IdentifiedTireProfileEvidence> {
    let longitudinal_identification = identify_tire_relaxation_dataset(&longitudinal_dataset)?;
    let lateral_identification = identify_tire_relaxation_dataset(&lateral_dataset)?;
    ensure!(
        longitudinal_dataset.axis == TireRelaxationAxis::Longitudinal
            && lateral_dataset.axis == TireRelaxationAxis::Lateral,
        "identified tire profile requires longitudinal then lateral datasets"
    );
    ensure!(
        longitudinal_dataset.steady_dataset == lateral_dataset.steady_dataset
            && longitudinal_dataset.steady_identification == lateral_dataset.steady_identification,
        "relaxation datasets do not share exact steady evidence"
    );
    let mut tire_spec = longitudinal_dataset.steady_identification.result.tire_spec;
    tire_spec.longitudinal_relaxation_length_m =
        longitudinal_identification.result.relaxation_length_m;
    tire_spec.lateral_relaxation_length_m = lateral_identification.result.relaxation_length_m;
    let recorded_source_claim = longitudinal_identification.recorded_source_claim
        && lateral_identification.recorded_source_claim;
    let mut profile = IdentifiedTireProfileEvidence {
        kind: IDENTIFIED_TIRE_PROFILE_KIND.into(),
        schema_version: IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1,
        longitudinal_dataset,
        longitudinal_identification,
        lateral_dataset,
        lateral_identification,
        load_sensitivity_dataset: None,
        load_sensitivity_identification: None,
        tire_spec,
        recorded_source_claim,
        content_sha256: String::new(),
    };
    profile.content_sha256 = profile_digest(&profile)?;
    profile.validate()?;
    Ok(profile)
}

/// Replays load-sweep and two-axis relaxation datasets into one v2 executable profile.
pub fn build_load_sensitive_identified_tire_profile(
    load_sensitivity_dataset: TireLoadSensitivityDataset,
    longitudinal_dataset: TireRelaxationDataset,
    lateral_dataset: TireRelaxationDataset,
) -> Result<IdentifiedTireProfileEvidence> {
    let load_sensitivity_identification =
        identify_tire_load_sensitivity_dataset(&load_sensitivity_dataset)?;
    let longitudinal_identification = identify_tire_relaxation_dataset(&longitudinal_dataset)?;
    let lateral_identification = identify_tire_relaxation_dataset(&lateral_dataset)?;
    ensure!(
        longitudinal_dataset.axis == TireRelaxationAxis::Longitudinal
            && lateral_dataset.axis == TireRelaxationAxis::Lateral,
        "load-sensitive profile requires longitudinal then lateral datasets"
    );
    ensure!(
        longitudinal_dataset.steady_dataset == lateral_dataset.steady_dataset
            && longitudinal_dataset.steady_identification == lateral_dataset.steady_identification
            && load_sensitivity_dataset.steady_dataset == longitudinal_dataset.steady_dataset
            && load_sensitivity_dataset.steady_identification
                == longitudinal_dataset.steady_identification,
        "load-sensitive profile inputs do not share exact steady evidence"
    );
    let mut tire_spec = longitudinal_dataset.steady_identification.result.tire_spec;
    tire_spec.load_sensitivity_per_load_ratio = load_sensitivity_identification
        .result
        .load_sensitivity_per_load_ratio;
    tire_spec.longitudinal_relaxation_length_m =
        longitudinal_identification.result.relaxation_length_m;
    tire_spec.lateral_relaxation_length_m = lateral_identification.result.relaxation_length_m;
    let recorded_source_claim = load_sensitivity_identification.recorded_source_claim
        && longitudinal_identification.recorded_source_claim
        && lateral_identification.recorded_source_claim;
    let mut profile = IdentifiedTireProfileEvidence {
        kind: IDENTIFIED_TIRE_PROFILE_KIND.into(),
        schema_version: IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION,
        longitudinal_dataset,
        longitudinal_identification,
        lateral_dataset,
        lateral_identification,
        load_sensitivity_dataset: Some(load_sensitivity_dataset),
        load_sensitivity_identification: Some(load_sensitivity_identification),
        tire_spec,
        recorded_source_claim,
        content_sha256: String::new(),
    };
    profile.content_sha256 = profile_digest(&profile)?;
    profile.validate()?;
    Ok(profile)
}

/// Decodes and verifies one bounded identified tire profile.
pub fn decode_identified_tire_profile(bytes: &[u8]) -> Result<IdentifiedTireProfileEvidence> {
    ensure!(
        bytes.len() <= MAX_IDENTIFIED_TIRE_PROFILE_BYTES,
        "identified tire profile exceeds byte limit"
    );
    let profile: IdentifiedTireProfileEvidence = serde_json::from_slice(bytes)?;
    profile.validate()?;
    Ok(profile)
}

/// Executes one exact identified profile through a backend-neutral Mobility plant.
pub fn run_identified_tire_backend_trace<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    profile: IdentifiedTireProfileEvidence,
) -> Result<IdentifiedTireBackendTrace> {
    profile.validate()?;
    let trace = run_backend_mobility_trace_configured(
        backend,
        manifest,
        backend_mobility_task_spec(),
        applied_plant(profile.tire_spec),
        0,
    )?;
    let mut evidence = IdentifiedTireBackendTrace {
        kind: IDENTIFIED_TIRE_BACKEND_TRACE_KIND.into(),
        schema_version: IDENTIFIED_TIRE_BACKEND_SCHEMA_VERSION,
        profile,
        trace,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = trace_digest(&evidence)?;
    evidence.validate()?;
    Ok(evidence)
}

/// Executes one profile on two backends and retains explicit SI-unit tolerances.
pub fn run_identified_tire_backend_comparison<B1: PhysicsBackend, B2: PhysicsBackend>(
    first_backend: B1,
    first_manifest: PhysicsBackendManifest,
    second_backend: B2,
    second_manifest: PhysicsBackendManifest,
    profile: IdentifiedTireProfileEvidence,
) -> Result<IdentifiedTireBackendComparison> {
    profile.validate()?;
    let plant = applied_plant(profile.tire_spec);
    let first = run_backend_mobility_trace_configured(
        first_backend,
        first_manifest,
        backend_mobility_task_spec(),
        plant,
        0,
    )?;
    let second = run_backend_mobility_trace_configured(
        second_backend,
        second_manifest,
        backend_mobility_task_spec(),
        plant,
        0,
    )?;
    let comparison = compare_backend_mobility_traces(first, second)?;
    let mut evidence = IdentifiedTireBackendComparison {
        kind: IDENTIFIED_TIRE_BACKEND_COMPARISON_KIND.into(),
        schema_version: IDENTIFIED_TIRE_BACKEND_SCHEMA_VERSION,
        profile,
        comparison,
        content_sha256: String::new(),
    };
    evidence.content_sha256 = comparison_digest(&evidence)?;
    evidence.validate()?;
    Ok(evidence)
}

fn applied_plant(tire_spec: CombinedSlipTireSpec) -> rne_robot::LongitudinalMobilityPlantSpec {
    let mut plant = backend_plant_spec();
    plant.tire = tire_spec;
    plant
}

fn profile_digest(profile: &IdentifiedTireProfileEvidence) -> Result<String> {
    let mut canonical = profile.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn trace_digest(trace: &IdentifiedTireBackendTrace) -> Result<String> {
    let mut canonical = trace.clone();
    canonical.content_sha256.clear();
    sha256(&serde_json::to_vec(&canonical)?)
}

fn comparison_digest(comparison: &IdentifiedTireBackendComparison) -> Result<String> {
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
    use rne_physics_rapier::RapierBackend;

    #[test]
    fn profile_replays_both_axes_and_rejects_parameter_substitution() {
        let profile = synthetic_identified_tire_profile().unwrap();
        assert_eq!(
            profile.schema_version,
            IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION_V1
        );
        assert!(profile.load_sensitivity_dataset.is_none());
        assert!(!profile.recorded_source_claim);
        assert!((profile.tire_spec.longitudinal_relaxation_length_m - 0.35).abs() < 1.0e-12);
        assert!((profile.tire_spec.lateral_relaxation_length_m - 0.35).abs() < 1.0e-12);
        let mut tampered = profile;
        tampered.tire_spec.longitudinal_relaxation_length_m += 0.01;
        assert!(tampered.validate().is_err());
    }

    #[test]
    fn v2_profile_replays_load_sensitivity_and_rejects_substitution() {
        let profile = synthetic_load_sensitive_identified_tire_profile().unwrap();
        assert_eq!(
            profile.schema_version,
            IDENTIFIED_TIRE_PROFILE_SCHEMA_VERSION
        );
        assert!(profile.load_sensitivity_dataset.is_some());
        assert!(profile.load_sensitivity_identification.is_some());
        assert_eq!(profile.tire_spec.load_sensitivity_per_load_ratio, 0.2);
        profile.validate().unwrap();

        let mut tampered = profile;
        tampered.tire_spec.load_sensitivity_per_load_ratio += 0.01;
        assert!(tampered.validate().is_err());

        let mut incomplete = synthetic_load_sensitive_identified_tire_profile().unwrap();
        incomplete.load_sensitivity_identification = None;
        incomplete.content_sha256 = profile_digest(&incomplete).unwrap();
        assert!(incomplete.validate().is_err());
    }

    #[test]
    fn legacy_v1_profile_roundtrips_without_v2_fields() {
        let profile = synthetic_identified_tire_profile().unwrap();
        let bytes = serde_json::to_vec(&profile).unwrap();
        let json = std::str::from_utf8(&bytes).unwrap();
        assert!(!json.contains("load_sensitivity_dataset"));
        assert!(!json.contains("load_sensitivity_identification"));
        assert_eq!(decode_identified_tire_profile(&bytes).unwrap(), profile);
    }

    #[test]
    fn rapier_trace_binds_identified_profile_into_executed_plant() {
        let profile = synthetic_load_sensitive_identified_tire_profile().unwrap();
        let evidence = run_identified_tire_backend_trace(
            RapierBackend::new(),
            RapierBackend::manifest(),
            profile.clone(),
        )
        .unwrap();
        assert!(evidence.trace.passed, "{:#?}", evidence.trace.metrics);
        assert_eq!(evidence.trace.plant.tire, profile.tire_spec);
        assert_eq!(evidence.trace.task_spec, backend_mobility_task_spec());
        evidence.validate().unwrap();
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_execute_same_identified_profile_and_task() {
        use rne_core::SimDuration;
        use rne_physics_mujoco::MuJoCoBackend;

        let evidence = run_identified_tire_backend_comparison(
            RapierBackend::new(),
            RapierBackend::manifest(),
            MuJoCoBackend::new(SimDuration::from_ticks(
                crate::backend::BACKEND_MOBILITY_FIXED_DELTA_TICKS,
            ))
            .unwrap(),
            MuJoCoBackend::manifest(),
            synthetic_load_sensitive_identified_tire_profile().unwrap(),
        )
        .unwrap();
        assert!(
            evidence.comparison.passed,
            "{:#?}",
            evidence.comparison.metrics
        );
        evidence.validate().unwrap();
    }
}

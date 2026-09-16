//! Content-bound combined application of identified suspension and tire profiles.
//!
//! Each identification family previously exited into simulation against the other
//! element's baseline fixture: identified suspension ran only with the frozen
//! road-excitation tire, and identified tire ran only in a suspension-free
//! longitudinal plant. This module applies both identified specs to the exact same
//! suspended four-wheel road-excitation task on two physics backends and binds the
//! complete chain into one self-verifying artifact.
//!
//! The combined artifact is a software application bridge. It never asserts
//! physical qualification: the tire side carries only a `recorded_source_claim`,
//! and only the tire physical gate can promote that to `physical_measurement`.

use crate::ackermann_suspension::wheel_plant_spec;
use crate::identified_suspension_road::suspension_spec_from_identification;
use crate::identified_tire_backend::IdentifiedTireProfileEvidence;
use crate::road_excitation::{
    compare_road_excitation_traces, run_road_excitation_trace_with_specs, RoadExcitationComparison,
    RoadExcitationTrace,
};
use crate::suspension_identification::{
    identify_suspension_dataset, SuspensionIdentificationDataset, SuspensionIdentificationEvidence,
};
use anyhow::{ensure, Result};
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use rne_robot::{CombinedSlipTireSpec, LongitudinalMobilityPlantSpec, SuspensionStrutSpec};
use serde::{Deserialize, Serialize};

/// Stable artifact kind for combined identification-to-simulation evidence.
pub const IDENTIFIED_SUSPENSION_TIRE_KIND: &str =
    "rne_mobility_identified_suspension_tire_evidence";
/// Stable artifact kind for a single-backend combined application trace.
pub const IDENTIFIED_SUSPENSION_TIRE_TRACE_KIND: &str =
    "rne_mobility_identified_suspension_tire_trace";
/// Combined identification-to-simulation artifact schema.
pub const IDENTIFIED_SUSPENSION_TIRE_SCHEMA_VERSION: u32 = 1;

/// Self-verifying chain from a suspension force log and a tire profile to two backend responses.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentifiedSuspensionTireEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact ordered suspension identification input, including its source declaration.
    pub suspension_dataset: SuspensionIdentificationDataset,
    /// Deterministically recomputed suspension fit and train/holdout residual evidence.
    pub suspension_identification: SuspensionIdentificationEvidence,
    /// Exact identified tire profile replayed from its own identification chain.
    pub tire_profile: IdentifiedTireProfileEvidence,
    /// Portable strut contract actually supplied to both physics backends.
    pub applied_suspension_spec: SuspensionStrutSpec,
    /// Portable tire contract actually supplied to both physics backends.
    pub applied_tire_spec: CombinedSlipTireSpec,
    /// True only when the suspension dataset declares recorded bench or vehicle measurements.
    pub suspension_physical_measurement: bool,
    /// True only when every tire dataset declares a recorded source.
    ///
    /// This is the standalone profile's provenance claim, not physical qualification.
    pub tire_recorded_source_claim: bool,
    /// True only when both sides declare a recorded source.
    ///
    /// Even then this is a declaration, not a qualified physical measurement. Only the
    /// separate tire and suspension acquisition gates can qualify a real capture.
    pub physical_measurement: bool,
    /// Complete SI-unit road-response comparison from the two backends.
    pub road_comparison: RoadExcitationComparison,
    /// True only when identification validation and the road comparison pass.
    pub passed: bool,
    /// FNV-1a integrity digest with this field empty; this is not a signature.
    pub content_digest: String,
}

impl IdentifiedSuspensionTireEvidence {
    /// Recomputes both identification chains and verifies their exact joint application.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == IDENTIFIED_SUSPENSION_TIRE_KIND
                && self.schema_version == IDENTIFIED_SUSPENSION_TIRE_SCHEMA_VERSION,
            "identified suspension tire kind/schema drift"
        );
        self.suspension_dataset.validate()?;
        self.suspension_identification
            .validate(&self.suspension_dataset)?;
        self.tire_profile.validate()?;
        let (suspension, tire) = suspension_tire_specs(
            &self.suspension_dataset,
            &self.suspension_identification,
            &self.tire_profile,
        )?;
        ensure!(
            self.applied_suspension_spec == suspension,
            "applied suspension does not match identification"
        );
        ensure!(
            self.applied_tire_spec == tire,
            "applied tire does not match identified profile"
        );
        ensure!(
            self.suspension_physical_measurement
                == self.suspension_identification.physical_measurement,
            "suspension physical measurement declaration drift"
        );
        ensure!(
            self.tire_recorded_source_claim == self.tire_profile.recorded_source_claim,
            "tire recorded source claim drift"
        );
        ensure!(
            self.physical_measurement
                == (self.suspension_physical_measurement && self.tire_recorded_source_claim),
            "combined physical measurement declaration drift"
        );
        self.road_comparison.validate()?;
        for trace in [&self.road_comparison.first, &self.road_comparison.second] {
            ensure!(
                trace.suspension_spec == self.applied_suspension_spec,
                "backend trace did not use identified suspension"
            );
            ensure!(
                trace.wheel_plant_spec.tire == self.applied_tire_spec,
                "backend trace did not use identified tire"
            );
        }
        ensure!(
            self.passed == self.road_comparison.passed,
            "aggregate verdict drift"
        );
        ensure!(
            self.content_digest == evidence_digest(self)?,
            "identified suspension tire digest drift"
        );
        Ok(())
    }
}

/// Derives the applied strut and tire while retaining the benchmark's declared geometry.
pub fn suspension_tire_specs(
    dataset: &SuspensionIdentificationDataset,
    identification: &SuspensionIdentificationEvidence,
    tire_profile: &IdentifiedTireProfileEvidence,
) -> Result<(SuspensionStrutSpec, CombinedSlipTireSpec)> {
    let suspension = suspension_spec_from_identification(dataset, identification)?;
    tire_profile.validate()?;
    let tire = tire_profile.tire_spec;
    ensure!(tire.is_valid(), "identified tire is invalid");
    Ok((suspension, tire))
}

/// Replaces only the road plant's tire element with the identified contract.
pub fn applied_wheel_plant(tire_spec: CombinedSlipTireSpec) -> LongitudinalMobilityPlantSpec {
    let mut plant = wheel_plant_spec();
    plant.tire = tire_spec;
    plant
}

/// Self-verifying single-backend response to the combined identified plant.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentifiedSuspensionTireTrace {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact ordered suspension identification input, including its source declaration.
    pub suspension_dataset: SuspensionIdentificationDataset,
    /// Deterministically recomputed suspension fit and train/holdout residual evidence.
    pub suspension_identification: SuspensionIdentificationEvidence,
    /// Exact identified tire profile replayed from its own identification chain.
    pub tire_profile: IdentifiedTireProfileEvidence,
    /// Portable strut contract actually supplied to the backend.
    pub applied_suspension_spec: SuspensionStrutSpec,
    /// Portable tire contract actually supplied to the backend.
    pub applied_tire_spec: CombinedSlipTireSpec,
    /// True only when the suspension dataset declares recorded bench or vehicle measurements.
    pub suspension_physical_measurement: bool,
    /// True only when every tire dataset declares a recorded source.
    pub tire_recorded_source_claim: bool,
    /// True only when both sides declare a recorded source. Still not physical qualification.
    pub physical_measurement: bool,
    /// Complete SI-unit road response from the backend.
    pub trace: RoadExcitationTrace,
    /// True only when the executed task metrics pass.
    pub passed: bool,
    /// FNV-1a integrity digest with this field empty; this is not a signature.
    pub content_digest: String,
}

impl IdentifiedSuspensionTireTrace {
    /// Recomputes both identification chains and verifies their exact joint application.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == IDENTIFIED_SUSPENSION_TIRE_TRACE_KIND
                && self.schema_version == IDENTIFIED_SUSPENSION_TIRE_SCHEMA_VERSION,
            "identified suspension tire trace kind/schema drift"
        );
        self.suspension_dataset.validate()?;
        self.suspension_identification
            .validate(&self.suspension_dataset)?;
        self.tire_profile.validate()?;
        let (suspension, tire) = suspension_tire_specs(
            &self.suspension_dataset,
            &self.suspension_identification,
            &self.tire_profile,
        )?;
        ensure!(
            self.applied_suspension_spec == suspension,
            "applied suspension does not match identification"
        );
        ensure!(
            self.applied_tire_spec == tire,
            "applied tire does not match identified profile"
        );
        ensure!(
            self.suspension_physical_measurement
                == self.suspension_identification.physical_measurement
                && self.tire_recorded_source_claim == self.tire_profile.recorded_source_claim
                && self.physical_measurement
                    == (self.suspension_physical_measurement && self.tire_recorded_source_claim),
            "physical measurement declaration drift"
        );
        self.trace.validate()?;
        ensure!(
            self.trace.suspension_spec == self.applied_suspension_spec,
            "backend trace did not use identified suspension"
        );
        ensure!(
            self.trace.wheel_plant_spec.tire == self.applied_tire_spec,
            "backend trace did not use identified tire"
        );
        ensure!(self.passed == self.trace.passed, "aggregate verdict drift");
        ensure!(
            self.content_digest == trace_evidence_digest(self)?,
            "identified suspension tire trace digest drift"
        );
        Ok(())
    }
}

/// Fits one suspension dataset, replays one tire profile, and executes them on one backend.
pub fn run_identified_suspension_tire_trace<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    suspension_dataset: SuspensionIdentificationDataset,
    tire_profile: IdentifiedTireProfileEvidence,
) -> Result<IdentifiedSuspensionTireTrace> {
    let suspension_identification = identify_suspension_dataset(&suspension_dataset)?;
    let (applied_suspension_spec, applied_tire_spec) = suspension_tire_specs(
        &suspension_dataset,
        &suspension_identification,
        &tire_profile,
    )?;
    let trace = run_road_excitation_trace_with_specs(
        backend,
        manifest,
        applied_suspension_spec,
        applied_wheel_plant(applied_tire_spec),
    )?;
    let passed = trace.passed;
    let suspension_physical_measurement = suspension_identification.physical_measurement;
    let tire_recorded_source_claim = tire_profile.recorded_source_claim;
    let mut evidence = IdentifiedSuspensionTireTrace {
        kind: IDENTIFIED_SUSPENSION_TIRE_TRACE_KIND.to_string(),
        schema_version: IDENTIFIED_SUSPENSION_TIRE_SCHEMA_VERSION,
        suspension_dataset,
        suspension_identification,
        tire_profile,
        applied_suspension_spec,
        applied_tire_spec,
        suspension_physical_measurement,
        tire_recorded_source_claim,
        physical_measurement: suspension_physical_measurement && tire_recorded_source_claim,
        trace,
        passed,
        content_digest: String::new(),
    };
    evidence.content_digest = trace_evidence_digest(&evidence)?;
    evidence.validate()?;
    Ok(evidence)
}

/// Fits one suspension dataset, replays one tire profile, and binds their joint execution.
///
/// Both backends receive the exact same finite-difference task, identified strut, and
/// identified tire. The comparison retains unit-bearing cross-backend tolerances rather
/// than claiming bitwise state equality across solvers.
pub fn run_identified_suspension_tire_evidence<First, Second>(
    suspension_dataset: SuspensionIdentificationDataset,
    tire_profile: IdentifiedTireProfileEvidence,
    first_backend: First,
    first_manifest: PhysicsBackendManifest,
    second_backend: Second,
    second_manifest: PhysicsBackendManifest,
) -> Result<IdentifiedSuspensionTireEvidence>
where
    First: PhysicsBackend,
    Second: PhysicsBackend,
{
    let suspension_identification = identify_suspension_dataset(&suspension_dataset)?;
    let (applied_suspension_spec, applied_tire_spec) = suspension_tire_specs(
        &suspension_dataset,
        &suspension_identification,
        &tire_profile,
    )?;
    let plant = applied_wheel_plant(applied_tire_spec);
    let first = run_road_excitation_trace_with_specs(
        first_backend,
        first_manifest,
        applied_suspension_spec,
        plant,
    )?;
    let second = run_road_excitation_trace_with_specs(
        second_backend,
        second_manifest,
        applied_suspension_spec,
        plant,
    )?;
    let road_comparison = compare_road_excitation_traces(first, second)?;
    let passed = road_comparison.passed;
    let suspension_physical_measurement = suspension_identification.physical_measurement;
    let tire_recorded_source_claim = tire_profile.recorded_source_claim;
    let mut evidence = IdentifiedSuspensionTireEvidence {
        kind: IDENTIFIED_SUSPENSION_TIRE_KIND.to_string(),
        schema_version: IDENTIFIED_SUSPENSION_TIRE_SCHEMA_VERSION,
        suspension_dataset,
        suspension_identification,
        tire_profile,
        applied_suspension_spec,
        applied_tire_spec,
        suspension_physical_measurement,
        tire_recorded_source_claim,
        physical_measurement: suspension_physical_measurement && tire_recorded_source_claim,
        road_comparison,
        passed,
        content_digest: String::new(),
    };
    evidence.content_digest = evidence_digest(&evidence)?;
    evidence.validate()?;
    Ok(evidence)
}

fn evidence_digest(evidence: &IdentifiedSuspensionTireEvidence) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_digest.clear();
    fnv1a64(&serde_json::to_vec(&canonical)?)
}

fn trace_evidence_digest(evidence: &IdentifiedSuspensionTireTrace) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_digest.clear();
    fnv1a64(&serde_json::to_vec(&canonical)?)
}

fn fnv1a64(bytes: &[u8]) -> Result<String> {
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{digest:016x}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::ackermann_suspension::suspension_spec;
    use crate::identified_tire_backend::synthetic_identified_tire_profile;
    use crate::suspension_identification::synthetic_suspension_identification_dataset;
    use rne_physics_rapier::RapierBackend;

    #[test]
    fn combined_specs_replace_only_identified_terms() {
        let dataset = synthetic_suspension_identification_dataset().unwrap();
        let identification = identify_suspension_dataset(&dataset).unwrap();
        let profile = synthetic_identified_tire_profile().unwrap();
        let baseline_suspension = suspension_spec();
        let baseline_tire = wheel_plant_spec().tire;

        let (suspension, tire) =
            suspension_tire_specs(&dataset, &identification, &profile).unwrap();

        assert_eq!(
            suspension.stiffness_n_per_m,
            identification.result.stiffness_n_per_m
        );
        assert_eq!(
            suspension.damping_n_s_per_m,
            identification.result.damping_n_s_per_m
        );
        assert_eq!(
            suspension.equilibrium_position_m,
            identification.result.equilibrium_position_m
        );
        assert_eq!(suspension.axis_body, baseline_suspension.axis_body);
        assert_eq!(
            suspension.maximum_force_n,
            baseline_suspension.maximum_force_n
        );
        assert_eq!(
            suspension.unsprung_mass_kg,
            baseline_suspension.unsprung_mass_kg
        );
        assert_eq!(tire, profile.tire_spec);
        assert_ne!(tire, baseline_tire);
    }

    #[test]
    fn tampered_tire_profile_is_rejected_before_application() {
        let dataset = synthetic_suspension_identification_dataset().unwrap();
        let identification = identify_suspension_dataset(&dataset).unwrap();
        let mut profile = synthetic_identified_tire_profile().unwrap();
        profile.tire_spec.longitudinal_stiffness_n += 1.0;
        assert!(suspension_tire_specs(&dataset, &identification, &profile).is_err());
    }

    #[test]
    fn rapier_trace_binds_both_identified_specs() {
        let dataset = synthetic_suspension_identification_dataset().unwrap();
        let profile = synthetic_identified_tire_profile().unwrap();
        let evidence = run_identified_suspension_tire_trace(
            RapierBackend::new(),
            RapierBackend::manifest(),
            dataset,
            profile.clone(),
        )
        .unwrap();
        evidence.validate().unwrap();
        assert_eq!(evidence.applied_tire_spec, profile.tire_spec);
        assert_eq!(evidence.trace.wheel_plant_spec.tire, profile.tire_spec);
        assert!(!evidence.physical_measurement);

        let mut tampered = evidence;
        tampered.applied_tire_spec.lateral_stiffness_n += 1.0;
        assert!(tampered.validate().is_err());
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn rapier_and_mujoco_execute_the_same_combined_identification() {
        use crate::road_excitation::ROAD_EXCITATION_FIXED_DELTA_TICKS;
        use rne_core::SimDuration;
        use rne_physics_mujoco::MuJoCoBackend;

        let evidence = run_identified_suspension_tire_evidence(
            synthetic_suspension_identification_dataset().unwrap(),
            synthetic_identified_tire_profile().unwrap(),
            RapierBackend::new(),
            RapierBackend::manifest(),
            MuJoCoBackend::new(SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS)).unwrap(),
            MuJoCoBackend::manifest(),
        )
        .unwrap();
        assert!(evidence.passed, "{:#?}", evidence.road_comparison.metrics);
        assert!(!evidence.physical_measurement);
        evidence.validate().unwrap();

        let mut tampered = evidence;
        tampered.applied_tire_spec.lateral_stiffness_n += 1.0;
        assert!(tampered.validate().is_err());
    }
}

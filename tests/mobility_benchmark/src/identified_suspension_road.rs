//! Content-bound evidence that applies identified suspension parameters to rigid-road simulation.

use crate::ackermann_suspension::{suspension_spec, wheel_plant_spec};
use crate::road_excitation::{
    compare_road_excitation_traces, run_road_excitation_trace_with_suspension,
    RoadExcitationComparison,
};
use crate::suspension_identification::{
    identify_suspension_dataset, SuspensionIdentificationDataset, SuspensionIdentificationEvidence,
};
use anyhow::{ensure, Result};
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use rne_robot::SuspensionStrutSpec;
use serde::{Deserialize, Serialize};

/// Stable artifact kind for identification-to-simulation evidence.
pub const IDENTIFIED_SUSPENSION_ROAD_KIND: &str =
    "rne_mobility_identified_suspension_road_evidence";
/// Identification-to-simulation artifact schema.
pub const IDENTIFIED_SUSPENSION_ROAD_SCHEMA_VERSION: u32 = 1;

/// Self-verifying chain from a force log through fitted parameters to two backend responses.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct IdentifiedSuspensionRoadEvidence {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema.
    pub schema_version: u32,
    /// Exact ordered identification input, including its source declaration.
    pub dataset: SuspensionIdentificationDataset,
    /// Deterministically recomputed fit and train/holdout residual evidence.
    pub identification: SuspensionIdentificationEvidence,
    /// Portable strut contract actually supplied to both physics backends.
    pub applied_suspension_spec: SuspensionStrutSpec,
    /// True only when the input declares recorded bench or vehicle measurements.
    pub physical_measurement: bool,
    /// Complete SI-unit road-response comparison from the two backends.
    pub road_comparison: RoadExcitationComparison,
    /// True only when identification validation and the road comparison pass.
    pub passed: bool,
    /// FNV-1a integrity digest with this field empty; this is not a signature.
    pub content_digest: String,
}

impl IdentifiedSuspensionRoadEvidence {
    /// Recomputes the fit and verifies its exact application to both backend traces.
    pub fn validate(&self) -> Result<()> {
        ensure!(
            self.kind == IDENTIFIED_SUSPENSION_ROAD_KIND
                && self.schema_version == IDENTIFIED_SUSPENSION_ROAD_SCHEMA_VERSION,
            "identified suspension road kind/schema drift"
        );
        self.dataset.validate()?;
        self.identification.validate(&self.dataset)?;
        ensure!(
            self.applied_suspension_spec
                == suspension_spec_from_identification(&self.dataset, &self.identification)?,
            "applied suspension does not match identification"
        );
        ensure!(
            self.physical_measurement == self.identification.physical_measurement,
            "physical measurement declaration drift"
        );
        self.road_comparison.validate()?;
        ensure!(
            self.road_comparison.first.suspension_spec == self.applied_suspension_spec
                && self.road_comparison.second.suspension_spec == self.applied_suspension_spec,
            "backend trace did not use identified suspension"
        );
        ensure!(
            self.road_comparison.first.wheel_plant_spec == wheel_plant_spec()
                && self.road_comparison.second.wheel_plant_spec == wheel_plant_spec(),
            "backend trace did not use the baseline wheel plant"
        );
        ensure!(
            self.passed == self.road_comparison.passed,
            "aggregate verdict drift"
        );
        ensure!(
            self.content_digest == evidence_digest(self)?,
            "identified suspension road digest drift"
        );
        Ok(())
    }
}

/// Applies the fitted force-law terms while retaining the benchmark's geometry and limits.
pub fn suspension_spec_from_identification(
    dataset: &SuspensionIdentificationDataset,
    identification: &SuspensionIdentificationEvidence,
) -> Result<SuspensionStrutSpec> {
    identification.validate(dataset)?;
    let mut applied = suspension_spec();
    applied.stiffness_n_per_m = identification.result.stiffness_n_per_m;
    applied.damping_n_s_per_m = identification.result.damping_n_s_per_m;
    applied.equilibrium_position_m = identification.result.equilibrium_position_m;
    ensure!(applied.is_valid(), "identified suspension is invalid");
    Ok(applied)
}

/// Fits one dataset, runs the exact fitted strut on two backends, and binds all evidence.
pub fn run_identified_suspension_road_evidence<First, Second>(
    dataset: SuspensionIdentificationDataset,
    first_backend: First,
    first_manifest: PhysicsBackendManifest,
    second_backend: Second,
    second_manifest: PhysicsBackendManifest,
) -> Result<IdentifiedSuspensionRoadEvidence>
where
    First: PhysicsBackend,
    Second: PhysicsBackend,
{
    let identification = identify_suspension_dataset(&dataset)?;
    let applied_suspension_spec = suspension_spec_from_identification(&dataset, &identification)?;
    let first = run_road_excitation_trace_with_suspension(
        first_backend,
        first_manifest,
        applied_suspension_spec,
    )?;
    let second = run_road_excitation_trace_with_suspension(
        second_backend,
        second_manifest,
        applied_suspension_spec,
    )?;
    let road_comparison = compare_road_excitation_traces(first, second)?;
    let mut evidence = IdentifiedSuspensionRoadEvidence {
        kind: IDENTIFIED_SUSPENSION_ROAD_KIND.to_string(),
        schema_version: IDENTIFIED_SUSPENSION_ROAD_SCHEMA_VERSION,
        dataset,
        physical_measurement: identification.physical_measurement,
        identification,
        applied_suspension_spec,
        passed: road_comparison.passed,
        road_comparison,
        content_digest: String::new(),
    };
    evidence.content_digest = evidence_digest(&evidence)?;
    evidence.validate()?;
    Ok(evidence)
}

fn evidence_digest(evidence: &IdentifiedSuspensionRoadEvidence) -> Result<String> {
    let mut canonical = evidence.clone();
    canonical.content_digest.clear();
    let bytes = serde_json::to_vec(&canonical)?;
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    Ok(format!("fnv1a64:{digest:016x}"))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::suspension_identification::synthetic_suspension_identification_dataset;

    #[test]
    fn fitted_terms_replace_only_the_force_law() {
        let dataset = synthetic_suspension_identification_dataset().unwrap();
        let identification = identify_suspension_dataset(&dataset).unwrap();
        let baseline = suspension_spec();
        let applied = suspension_spec_from_identification(&dataset, &identification).unwrap();

        assert_eq!(
            applied.stiffness_n_per_m,
            identification.result.stiffness_n_per_m
        );
        assert_eq!(
            applied.damping_n_s_per_m,
            identification.result.damping_n_s_per_m
        );
        assert_eq!(
            applied.equilibrium_position_m,
            identification.result.equilibrium_position_m
        );
        assert_eq!(applied.axis_body, baseline.axis_body);
        assert_eq!(applied.minimum_position_m, baseline.minimum_position_m);
        assert_eq!(applied.maximum_position_m, baseline.maximum_position_m);
        assert_eq!(applied.maximum_force_n, baseline.maximum_force_n);
        assert_eq!(applied.unsprung_mass_kg, baseline.unsprung_mass_kg);
    }
}

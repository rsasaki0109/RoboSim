//! Machine-verifiable exit evidence for the LeKiwi physical reference run.
//!
//! The schema is filesystem-neutral. It records bounded identifiers,
//! attestations, and content-addressed relative file references; `xtask`
//! performs the filesystem and artifact-specific replay checks.

use crate::{
    LEKIWI_PHYSICAL_DEVICE_ID_PREFIX, LEKIWI_UPSTREAM_REVISION, LEKIWI_UPSTREAM_WATCHDOG_TIMEOUT_MS,
};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;

/// Schema version for [`LeKiwiPhysicalEvidenceManifest`].
pub const LEKIWI_PHYSICAL_EVIDENCE_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`LeKiwiPhysicalEvidenceManifest`].
pub const LEKIWI_PHYSICAL_EVIDENCE_KIND: &str = "rne_lekiwi_physical_evidence_manifest";

/// Schema version shared by physical operator diagnostics.
pub const LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for [`PowerIsolationDiagnostic`].
pub const LEKIWI_POWER_ISOLATION_DIAGNOSTIC_KIND: &str = "rne_lekiwi_power_isolation_diagnostic";

/// Stable discriminator for [`HostTerminationDiagnostic`].
pub const LEKIWI_HOST_TERMINATION_DIAGNOSTIC_KIND: &str = "rne_lekiwi_host_termination_diagnostic";

/// Evidence-reference kind for the pinned upstream calibration JSON.
pub const LEKIWI_CALIBRATION_EVIDENCE_KIND: &str = "lerobot_lekiwi_calibration";

/// Evidence-reference kind for an RNE dataset bundle manifest.
pub const LEKIWI_CAMERA_DATASET_MANIFEST_KIND: &str = "rne_dataset_bundle_manifest";

/// Evidence-reference kind for a headless depth-pair evaluation report.
pub const LEKIWI_CAMERA_OFFLINE_EVALUATION_KIND: &str = "rne_dataset_depth_pair_evaluation";

/// Evidence-reference kind for clean-host reproduction output.
pub const LEKIWI_CLEAN_HOST_REPRODUCTION_KIND: &str = "rne_lekiwi_clean_host_reproduction";

/// Minimum continuous samples required by the elevated shadow exit stage.
pub const LEKIWI_ELEVATED_SHADOW_MIN_SAMPLES: usize = 1_800;

/// Maximum base-axis speed admitted by the first floor-live evidence stage.
pub const LEKIWI_FLOOR_LIVE_MAX_LINEAR_SPEED_M_S: f64 = 0.02;

/// One immutable artifact relative to the physical-evidence manifest.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct EvidenceFileRef {
    /// Stable format discriminator expected at this path.
    pub kind: String,
    /// Expected artifact schema version.
    pub schema_version: u32,
    /// Canonical slash-separated relative path.
    pub path: String,
    /// Lowercase SHA-256 with the `sha256:` prefix.
    pub sha256: String,
}

impl EvidenceFileRef {
    /// Creates a file reference. [`Self::validate`] rejects invalid values.
    pub fn new(
        kind: impl Into<String>,
        schema_version: u32,
        path: impl Into<String>,
        sha256: impl Into<String>,
    ) -> Self {
        Self {
            kind: kind.into(),
            schema_version,
            path: path.into(),
            sha256: sha256.into(),
        }
    }

    /// Validates portable path and digest syntax without reading a filesystem.
    pub fn validate(&self) -> Result<(), LeKiwiPhysicalEvidenceError> {
        validate_identifier("artifact.kind", &self.kind)?;
        if self.schema_version == 0 {
            return Err(invalid(
                "artifact.schema_version",
                "must be greater than zero",
            ));
        }
        validate_relative_path("artifact.path", &self.path)?;
        validate_sha256("artifact.sha256", &self.sha256)
    }
}

/// Stable identity for one operator participating in the physical run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PhysicalOperator {
    /// Pseudonymous identifier stable within this evidence set.
    pub operator_id: String,
    /// Declared responsibility such as `primary` or `safety`.
    pub role: String,
}

/// Exact physical unit and host environment used for the evidence set.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LeKiwiPhysicalInventory {
    /// Device ID returned by the physical bridge Ready response.
    pub device_id: String,
    /// Calibration identity supplied to the upstream LeRobot configuration.
    pub robot_id: String,
    /// Stable Raspberry Pi or base-computer serial.
    pub base_controller_id: String,
    /// Stable arm motor-bus adapter serial.
    pub arm_bus_id: String,
    /// Stable front-camera device identity.
    pub front_camera_id: String,
    /// Stable wrist-camera device identity.
    pub wrist_camera_id: String,
    /// Reproducible operating-system image identifier.
    pub os_image: String,
    /// Exact Python runtime version.
    pub python_version: String,
    /// Content-addressed upstream calibration file.
    pub calibration: EvidenceFileRef,
}

/// Two-person evidence that physical actuator power can be removed.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PowerIsolationAttestation {
    /// Operator controlling the host session.
    pub primary_operator_id: String,
    /// Different operator controlling the reachable physical cutoff.
    pub safety_operator_id: String,
    /// True only after the physical cutoff has actually been exercised.
    pub tested: bool,
    /// Content-addressed diagnostic or checklist captured during the test.
    pub diagnostic: EvidenceFileRef,
}

/// External observation of the independent device watchdog after host loss.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HostTerminationAttestation {
    /// Operator who observed physical wheel stop independently of the host.
    pub observer_operator_id: String,
    /// Session identity whose host process was deliberately terminated.
    pub terminated_session_id: String,
    /// True only when physical zero output was observed.
    pub safe_stop_observed: bool,
    /// Measured upper estimate from host loss to physical stop.
    pub stop_latency_ms: u64,
    /// Non-negative measurement uncertainty added to the latency estimate.
    pub measurement_uncertainty_ms: u64,
    /// Content-addressed diagnostic carrying the raw observation.
    pub diagnostic: EvidenceFileRef,
}

/// Machine-readable record of the two-person physical cutoff exercise.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PowerIsolationDiagnostic {
    /// Stable diagnostic discriminator.
    pub kind: String,
    /// Diagnostic schema version.
    pub schema_version: u32,
    /// Physical evidence run identity.
    pub run_id: String,
    /// Exact bridge device identity under test.
    pub device_id: String,
    /// Host operator identity.
    pub primary_operator_id: String,
    /// Independent physical-cutoff operator identity.
    pub safety_operator_id: String,
    /// True only when actuator power was physically removed.
    pub physical_power_removed: bool,
    /// True only when the safety operator observed actuator output stop.
    pub actuator_output_stopped: bool,
    /// Bounded description of the observation method or instrument.
    pub observation_method: String,
}

impl PowerIsolationDiagnostic {
    /// Validates the standalone diagnostic contract.
    pub fn validate(&self) -> Result<(), LeKiwiPhysicalEvidenceError> {
        if self.kind != LEKIWI_POWER_ISOLATION_DIAGNOSTIC_KIND
            || self.schema_version != LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION
        {
            return Err(invalid(
                "power_isolation_diagnostic",
                "unsupported kind or schema version",
            ));
        }
        validate_identifier("power_isolation_diagnostic.run_id", &self.run_id)?;
        validate_identifier(
            "power_isolation_diagnostic.primary_operator_id",
            &self.primary_operator_id,
        )?;
        validate_identifier(
            "power_isolation_diagnostic.safety_operator_id",
            &self.safety_operator_id,
        )?;
        validate_physical_device_id("power_isolation_diagnostic.device_id", &self.device_id)?;
        validate_text(
            "power_isolation_diagnostic.observation_method",
            &self.observation_method,
        )?;
        if self.primary_operator_id == self.safety_operator_id
            || !self.physical_power_removed
            || !self.actuator_output_stopped
        {
            return Err(invalid(
                "power_isolation_diagnostic",
                "requires distinct operators, physical power removal, and observed stop",
            ));
        }
        Ok(())
    }
}

/// Machine-readable observation of independent stop after host termination.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct HostTerminationDiagnostic {
    /// Stable diagnostic discriminator.
    pub kind: String,
    /// Diagnostic schema version.
    pub schema_version: u32,
    /// Physical evidence run identity.
    pub run_id: String,
    /// Exact bridge device identity under test.
    pub device_id: String,
    /// Physical observer identity.
    pub observer_operator_id: String,
    /// Session whose host process was terminated without a terminal response.
    pub terminated_session_id: String,
    /// Fresh completed HIL session used after reconnect and rearm.
    pub reconnect_session_id: String,
    /// True only when physical zero output was independently observed.
    pub safe_stop_observed: bool,
    /// Measured upper estimate from host loss to physical stop.
    pub stop_latency_ms: u64,
    /// Measurement uncertainty added to the latency estimate.
    pub measurement_uncertainty_ms: u64,
    /// Bounded description of the observation method or instrument.
    pub observation_method: String,
}

impl HostTerminationDiagnostic {
    /// Validates identifiers and the pinned independent-watchdog deadline.
    pub fn validate(&self) -> Result<(), LeKiwiPhysicalEvidenceError> {
        if self.kind != LEKIWI_HOST_TERMINATION_DIAGNOSTIC_KIND
            || self.schema_version != LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION
        {
            return Err(invalid(
                "host_termination_diagnostic",
                "unsupported kind or schema version",
            ));
        }
        validate_identifier("host_termination_diagnostic.run_id", &self.run_id)?;
        validate_identifier(
            "host_termination_diagnostic.observer_operator_id",
            &self.observer_operator_id,
        )?;
        validate_identifier(
            "host_termination_diagnostic.terminated_session_id",
            &self.terminated_session_id,
        )?;
        validate_identifier(
            "host_termination_diagnostic.reconnect_session_id",
            &self.reconnect_session_id,
        )?;
        validate_physical_device_id("host_termination_diagnostic.device_id", &self.device_id)?;
        validate_text(
            "host_termination_diagnostic.observation_method",
            &self.observation_method,
        )?;
        if self.terminated_session_id == self.reconnect_session_id
            || !self.safe_stop_observed
            || !stop_timing_within_watchdog(self.stop_latency_ms, self.measurement_uncertainty_ms)
        {
            return Err(invalid(
                "host_termination_diagnostic",
                "requires distinct sessions and an observed stop within 500 ms",
            ));
        }
        Ok(())
    }
}

/// Required artifacts for every LeKiwi v1 physical exit claim.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LeKiwiPhysicalEvidenceArtifacts {
    /// Exact portable TaskSpec used by every run.
    pub task_spec: EvidenceFileRef,
    /// Exact LeKiwi reference profile.
    pub reference_profile: EvidenceFileRef,
    /// Complete 1,800-sample-or-longer elevated shadow session.
    pub elevated_shadow_session: EvidenceFileRef,
    /// Passing TaskSpec-ordered shadow comparison.
    pub elevated_shadow_comparison: EvidenceFileRef,
    /// Physical command-deadline stop session.
    pub command_deadline_session: EvidenceFileRef,
    /// Physical stale-command/device-watchdog stop session.
    pub device_watchdog_session: EvidenceFileRef,
    /// Physical actuator-limit stop session.
    pub actuator_limit_session: EvidenceFileRef,
    /// Physical software emergency-stop session.
    pub emergency_stop_session: EvidenceFileRef,
    /// Fresh completed HIL session after the host-loss safety trip.
    pub reconnect_session: EvidenceFileRef,
    /// Completed low-speed floor-live session.
    pub low_speed_live_success_session: EvidenceFileRef,
    /// Deliberately safety-stopped floor-live session.
    pub low_speed_live_failure_session: EvidenceFileRef,
    /// Front-and-wrist camera dataset bundle manifest.
    pub camera_dataset_manifest: EvidenceFileRef,
    /// Passing headless offline camera evaluation report.
    pub camera_offline_evaluation: EvidenceFileRef,
    /// `capsule.json` from a fully verifiable Failure Capsule directory.
    pub failure_capsule_manifest: EvidenceFileRef,
    /// Output of reproduction from a clean host checkout.
    pub clean_host_reproduction: EvidenceFileRef,
}

impl LeKiwiPhysicalEvidenceArtifacts {
    /// Returns every required artifact with its stable semantic role.
    pub fn all(&self) -> [(&'static str, &EvidenceFileRef); 15] {
        [
            ("task_spec", &self.task_spec),
            ("reference_profile", &self.reference_profile),
            ("elevated_shadow_session", &self.elevated_shadow_session),
            (
                "elevated_shadow_comparison",
                &self.elevated_shadow_comparison,
            ),
            ("command_deadline_session", &self.command_deadline_session),
            ("device_watchdog_session", &self.device_watchdog_session),
            ("actuator_limit_session", &self.actuator_limit_session),
            ("emergency_stop_session", &self.emergency_stop_session),
            ("reconnect_session", &self.reconnect_session),
            (
                "low_speed_live_success_session",
                &self.low_speed_live_success_session,
            ),
            (
                "low_speed_live_failure_session",
                &self.low_speed_live_failure_session,
            ),
            ("camera_dataset_manifest", &self.camera_dataset_manifest),
            ("camera_offline_evaluation", &self.camera_offline_evaluation),
            ("failure_capsule_manifest", &self.failure_capsule_manifest),
            ("clean_host_reproduction", &self.clean_host_reproduction),
        ]
    }

    /// Returns mutable access to every required artifact and its stable role.
    pub fn all_mut(&mut self) -> [(&'static str, &mut EvidenceFileRef); 15] {
        [
            ("task_spec", &mut self.task_spec),
            ("reference_profile", &mut self.reference_profile),
            ("elevated_shadow_session", &mut self.elevated_shadow_session),
            (
                "elevated_shadow_comparison",
                &mut self.elevated_shadow_comparison,
            ),
            (
                "command_deadline_session",
                &mut self.command_deadline_session,
            ),
            ("device_watchdog_session", &mut self.device_watchdog_session),
            ("actuator_limit_session", &mut self.actuator_limit_session),
            ("emergency_stop_session", &mut self.emergency_stop_session),
            ("reconnect_session", &mut self.reconnect_session),
            (
                "low_speed_live_success_session",
                &mut self.low_speed_live_success_session,
            ),
            (
                "low_speed_live_failure_session",
                &mut self.low_speed_live_failure_session,
            ),
            ("camera_dataset_manifest", &mut self.camera_dataset_manifest),
            (
                "camera_offline_evaluation",
                &mut self.camera_offline_evaluation,
            ),
            (
                "failure_capsule_manifest",
                &mut self.failure_capsule_manifest,
            ),
            ("clean_host_reproduction", &mut self.clean_host_reproduction),
        ]
    }
}

/// Self-contained index for the complete LeKiwi physical reference run.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct LeKiwiPhysicalEvidenceManifest {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Manifest schema version.
    pub schema_version: u32,
    /// Stable identity shared by the evidence collection procedure.
    pub run_id: String,
    /// Full source commit used to build the RNE host.
    pub rne_commit: String,
    /// Exact pinned LeRobot source revision.
    pub upstream_revision: String,
    /// Exact physical device inventory.
    pub inventory: LeKiwiPhysicalInventory,
    /// At least two distinct physical-run participants.
    pub operators: Vec<PhysicalOperator>,
    /// Physical power-isolation exercise and role separation.
    pub power_isolation: PowerIsolationAttestation,
    /// Independent watchdog observation after terminating the host.
    pub host_termination: HostTerminationAttestation,
    /// True only when the recorded reproduction started from a clean checkout.
    pub clean_host_checkout: bool,
    /// Complete role-specific artifact set.
    pub artifacts: LeKiwiPhysicalEvidenceArtifacts,
    /// SHA-256 of compact JSON with this field empty.
    pub content_sha256: String,
}

impl LeKiwiPhysicalEvidenceManifest {
    /// Computes the self-excluding deterministic manifest digest.
    pub fn computed_content_sha256(&self) -> Result<String, LeKiwiPhysicalEvidenceError> {
        let mut canonical = self.clone();
        canonical.content_sha256.clear();
        let bytes = serde_json::to_vec(&canonical).map_err(|error| {
            LeKiwiPhysicalEvidenceError::Serialization {
                reason: error.to_string(),
            }
        })?;
        Ok(format!("sha256:{:x}", Sha256::digest(bytes)))
    }

    /// Recomputes and stores the self-excluding manifest digest.
    pub fn seal(&mut self) -> Result<(), LeKiwiPhysicalEvidenceError> {
        self.content_sha256 = self.computed_content_sha256()?;
        Ok(())
    }

    /// Validates the complete schema and all cross-field invariants.
    pub fn validate(&self) -> Result<(), LeKiwiPhysicalEvidenceError> {
        if self.kind != LEKIWI_PHYSICAL_EVIDENCE_KIND {
            return Err(invalid("kind", "unsupported artifact discriminator"));
        }
        if self.schema_version != LEKIWI_PHYSICAL_EVIDENCE_SCHEMA_VERSION {
            return Err(invalid("schema_version", "unsupported schema version"));
        }
        validate_identifier("run_id", &self.run_id)?;
        validate_git_revision("rne_commit", &self.rne_commit)?;
        if self.upstream_revision != LEKIWI_UPSTREAM_REVISION {
            return Err(invalid(
                "upstream_revision",
                "must equal the pinned reference-profile revision",
            ));
        }
        self.validate_inventory()?;
        self.validate_operators()?;
        if !self.clean_host_checkout {
            return Err(invalid(
                "clean_host_checkout",
                "must be true for physical exit evidence",
            ));
        }

        let operator_ids = self
            .operators
            .iter()
            .map(|operator| operator.operator_id.as_str())
            .collect::<BTreeSet<_>>();
        for (field, operator_id) in [
            (
                "power_isolation.primary_operator_id",
                self.power_isolation.primary_operator_id.as_str(),
            ),
            (
                "power_isolation.safety_operator_id",
                self.power_isolation.safety_operator_id.as_str(),
            ),
            (
                "host_termination.observer_operator_id",
                self.host_termination.observer_operator_id.as_str(),
            ),
        ] {
            if !operator_ids.contains(operator_id) {
                return Err(invalid(field, "must reference a declared operator"));
            }
        }
        if self.power_isolation.primary_operator_id == self.power_isolation.safety_operator_id {
            return Err(invalid(
                "power_isolation.safety_operator_id",
                "must differ from the primary operator",
            ));
        }
        if !self.power_isolation.tested {
            return Err(invalid(
                "power_isolation.tested",
                "physical power isolation must be exercised",
            ));
        }
        if !self.host_termination.safe_stop_observed {
            return Err(invalid(
                "host_termination.safe_stop_observed",
                "independent physical stop must be observed",
            ));
        }
        validate_identifier(
            "host_termination.terminated_session_id",
            &self.host_termination.terminated_session_id,
        )?;
        if !stop_timing_within_watchdog(
            self.host_termination.stop_latency_ms,
            self.host_termination.measurement_uncertainty_ms,
        ) {
            return Err(invalid(
                "host_termination.stop_latency_ms",
                "latency plus uncertainty must be in 1..=500 ms",
            ));
        }

        let mut paths = BTreeSet::new();
        for (role, artifact) in self.artifacts.all().into_iter().chain([
            ("calibration", &self.inventory.calibration),
            (
                "power_isolation_diagnostic",
                &self.power_isolation.diagnostic,
            ),
            (
                "host_termination_diagnostic",
                &self.host_termination.diagnostic,
            ),
        ]) {
            artifact.validate()?;
            if !paths.insert(artifact.path.as_str()) {
                return Err(invalid(role, "artifact paths must be unique"));
            }
        }
        validate_sha256("content_sha256", &self.content_sha256)?;
        if self.content_sha256 != self.computed_content_sha256()? {
            return Err(LeKiwiPhysicalEvidenceError::DigestMismatch);
        }
        Ok(())
    }

    fn validate_inventory(&self) -> Result<(), LeKiwiPhysicalEvidenceError> {
        validate_identifier("inventory.robot_id", &self.inventory.robot_id)?;
        let expected_device_id = format!(
            "{LEKIWI_PHYSICAL_DEVICE_ID_PREFIX}{}",
            self.inventory.robot_id
        );
        if self.inventory.device_id != expected_device_id {
            return Err(invalid(
                "inventory.device_id",
                "must be the physical prefix followed by robot_id",
            ));
        }
        for (field, value) in [
            (
                "inventory.base_controller_id",
                self.inventory.base_controller_id.as_str(),
            ),
            ("inventory.arm_bus_id", self.inventory.arm_bus_id.as_str()),
            (
                "inventory.front_camera_id",
                self.inventory.front_camera_id.as_str(),
            ),
            (
                "inventory.wrist_camera_id",
                self.inventory.wrist_camera_id.as_str(),
            ),
            ("inventory.os_image", self.inventory.os_image.as_str()),
            (
                "inventory.python_version",
                self.inventory.python_version.as_str(),
            ),
        ] {
            validate_text(field, value)?;
        }
        if self.inventory.front_camera_id == self.inventory.wrist_camera_id {
            return Err(invalid(
                "inventory.wrist_camera_id",
                "front and wrist cameras must have distinct identities",
            ));
        }
        Ok(())
    }

    fn validate_operators(&self) -> Result<(), LeKiwiPhysicalEvidenceError> {
        if self.operators.len() < 2 || self.operators.len() > 16 {
            return Err(invalid("operators", "must contain 2..=16 operators"));
        }
        let mut ids = BTreeSet::new();
        for operator in &self.operators {
            validate_identifier("operators.operator_id", &operator.operator_id)?;
            validate_text("operators.role", &operator.role)?;
            if !ids.insert(operator.operator_id.as_str()) {
                return Err(invalid("operators", "operator IDs must be unique"));
            }
        }
        Ok(())
    }
}

/// Failure validating a LeKiwi physical evidence manifest.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum LeKiwiPhysicalEvidenceError {
    /// One field violates its bounded portable contract.
    #[error("invalid LeKiwi physical evidence field {field}: {reason}")]
    Invalid {
        /// Field path.
        field: &'static str,
        /// Stable failure reason.
        reason: &'static str,
    },
    /// The self-excluding manifest digest differs from the declared digest.
    #[error("LeKiwi physical evidence manifest content digest mismatch")]
    DigestMismatch,
    /// Canonical serialization failed.
    #[error("could not serialize LeKiwi physical evidence manifest: {reason}")]
    Serialization {
        /// Serialization error without a foreign public error type.
        reason: String,
    },
}

fn invalid(field: &'static str, reason: &'static str) -> LeKiwiPhysicalEvidenceError {
    LeKiwiPhysicalEvidenceError::Invalid { field, reason }
}

fn validate_identifier(
    field: &'static str,
    value: &str,
) -> Result<(), LeKiwiPhysicalEvidenceError> {
    if value.is_empty()
        || value.len() > 128
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_alphanumeric() || matches!(byte, b'.' | b'_' | b'-'))
    {
        return Err(invalid(
            field,
            "must contain 1..=128 ASCII identifier characters",
        ));
    }
    Ok(())
}

fn validate_text(field: &'static str, value: &str) -> Result<(), LeKiwiPhysicalEvidenceError> {
    if value.is_empty() || value.len() > 1_024 || value.chars().any(char::is_control) {
        return Err(invalid(
            field,
            "must contain 1..=1024 non-control UTF-8 bytes",
        ));
    }
    Ok(())
}

fn validate_relative_path(
    field: &'static str,
    value: &str,
) -> Result<(), LeKiwiPhysicalEvidenceError> {
    validate_text(field, value)?;
    if value.contains('\\')
        || value.starts_with('/')
        || value.contains(':')
        || value
            .split('/')
            .any(|component| component.is_empty() || matches!(component, "." | ".."))
    {
        return Err(invalid(field, "must be a canonical relative slash path"));
    }
    Ok(())
}

fn validate_sha256(field: &'static str, value: &str) -> Result<(), LeKiwiPhysicalEvidenceError> {
    let hex = value.strip_prefix("sha256:").unwrap_or_default();
    if hex.len() != 64
        || !hex
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(invalid(
            field,
            "must be lowercase sha256: followed by 64 hexadecimal digits",
        ));
    }
    Ok(())
}

fn validate_git_revision(
    field: &'static str,
    value: &str,
) -> Result<(), LeKiwiPhysicalEvidenceError> {
    if value.len() != 40
        || !value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(invalid(field, "must be 40 lowercase hexadecimal digits"));
    }
    Ok(())
}

fn validate_physical_device_id(
    field: &'static str,
    value: &str,
) -> Result<(), LeKiwiPhysicalEvidenceError> {
    let suffix = value
        .strip_prefix(LEKIWI_PHYSICAL_DEVICE_ID_PREFIX)
        .ok_or_else(|| invalid(field, "must use the physical LeKiwi device prefix"))?;
    validate_identifier(field, suffix)
}

fn stop_timing_within_watchdog(stop_latency_ms: u64, measurement_uncertainty_ms: u64) -> bool {
    stop_latency_ms
        .checked_add(measurement_uncertainty_ms)
        .is_some_and(|upper_bound_ms| {
            upper_bound_ms > 0 && upper_bound_ms <= LEKIWI_UPSTREAM_WATCHDOG_TIMEOUT_MS
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn artifact(path: &str) -> EvidenceFileRef {
        EvidenceFileRef::new("rne_fixture", 1, path, format!("sha256:{}", "a".repeat(64)))
    }

    fn manifest() -> LeKiwiPhysicalEvidenceManifest {
        let mut manifest = LeKiwiPhysicalEvidenceManifest {
            kind: LEKIWI_PHYSICAL_EVIDENCE_KIND.to_string(),
            schema_version: LEKIWI_PHYSICAL_EVIDENCE_SCHEMA_VERSION,
            run_id: "rne.lekiwi.physical.001".to_string(),
            rne_commit: "1".repeat(40),
            upstream_revision: LEKIWI_UPSTREAM_REVISION.to_string(),
            inventory: LeKiwiPhysicalInventory {
                device_id: format!("{LEKIWI_PHYSICAL_DEVICE_ID_PREFIX}unit-001"),
                robot_id: "unit-001".to_string(),
                base_controller_id: "base-serial-001".to_string(),
                arm_bus_id: "arm-bus-001".to_string(),
                front_camera_id: "front-camera-001".to_string(),
                wrist_camera_id: "wrist-camera-001".to_string(),
                os_image: "raspios-2026-08".to_string(),
                python_version: "3.12.4".to_string(),
                calibration: artifact("inventory/calibration.json"),
            },
            operators: vec![
                PhysicalOperator {
                    operator_id: "operator-a".to_string(),
                    role: "primary".to_string(),
                },
                PhysicalOperator {
                    operator_id: "operator-b".to_string(),
                    role: "safety".to_string(),
                },
            ],
            power_isolation: PowerIsolationAttestation {
                primary_operator_id: "operator-a".to_string(),
                safety_operator_id: "operator-b".to_string(),
                tested: true,
                diagnostic: artifact("diagnostics/power-isolation.txt"),
            },
            host_termination: HostTerminationAttestation {
                observer_operator_id: "operator-b".to_string(),
                terminated_session_id: "host-loss-session".to_string(),
                safe_stop_observed: true,
                stop_latency_ms: 450,
                measurement_uncertainty_ms: 25,
                diagnostic: artifact("diagnostics/host-termination.txt"),
            },
            clean_host_checkout: true,
            artifacts: LeKiwiPhysicalEvidenceArtifacts {
                task_spec: artifact("contracts/task.json"),
                reference_profile: artifact("contracts/profile.json"),
                elevated_shadow_session: artifact("sessions/shadow.json"),
                elevated_shadow_comparison: artifact("reports/shadow.json"),
                command_deadline_session: artifact("sessions/deadline.json"),
                device_watchdog_session: artifact("sessions/watchdog.json"),
                actuator_limit_session: artifact("sessions/limit.json"),
                emergency_stop_session: artifact("sessions/emergency.json"),
                reconnect_session: artifact("sessions/reconnect.json"),
                low_speed_live_success_session: artifact("sessions/live-success.json"),
                low_speed_live_failure_session: artifact("sessions/live-failure.json"),
                camera_dataset_manifest: artifact("dataset/manifest.json"),
                camera_offline_evaluation: artifact("dataset/evaluation.json"),
                failure_capsule_manifest: artifact("capsule/capsule.json"),
                clean_host_reproduction: artifact("diagnostics/clean-host.txt"),
            },
            content_sha256: String::new(),
        };
        manifest.seal().expect("seal manifest");
        manifest
    }

    #[test]
    fn validates_complete_physical_manifest() {
        manifest().validate().expect("valid manifest");
        PowerIsolationDiagnostic {
            kind: LEKIWI_POWER_ISOLATION_DIAGNOSTIC_KIND.to_string(),
            schema_version: LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION,
            run_id: "rne.lekiwi.physical.001".to_string(),
            device_id: format!("{LEKIWI_PHYSICAL_DEVICE_ID_PREFIX}unit-001"),
            primary_operator_id: "operator-a".to_string(),
            safety_operator_id: "operator-b".to_string(),
            physical_power_removed: true,
            actuator_output_stopped: true,
            observation_method: "direct wheel observation".to_string(),
        }
        .validate()
        .expect("valid power diagnostic");
        HostTerminationDiagnostic {
            kind: LEKIWI_HOST_TERMINATION_DIAGNOSTIC_KIND.to_string(),
            schema_version: LEKIWI_PHYSICAL_DIAGNOSTIC_SCHEMA_VERSION,
            run_id: "rne.lekiwi.physical.001".to_string(),
            device_id: format!("{LEKIWI_PHYSICAL_DEVICE_ID_PREFIX}unit-001"),
            observer_operator_id: "operator-b".to_string(),
            terminated_session_id: "host-loss-session".to_string(),
            reconnect_session_id: "reconnect-session".to_string(),
            safe_stop_observed: true,
            stop_latency_ms: 450,
            measurement_uncertainty_ms: 25,
            observation_method: "frame-counted video".to_string(),
        }
        .validate()
        .expect("valid host diagnostic");
    }

    #[test]
    fn rejects_mock_identity_and_tampering() {
        let mut wrong_device = manifest();
        wrong_device.inventory.device_id = crate::LEKIWI_MOCK_DEVICE_ID.to_string();
        wrong_device.seal().expect("reseal");
        assert!(wrong_device.validate().is_err());

        let mut tampered = manifest();
        tampered.run_id = "changed".to_string();
        assert_eq!(
            tampered.validate(),
            Err(LeKiwiPhysicalEvidenceError::DigestMismatch)
        );
    }

    #[test]
    fn rejects_missing_role_separation_and_late_watchdog() {
        let mut same_operator = manifest();
        same_operator.power_isolation.safety_operator_id = "operator-a".to_string();
        same_operator.seal().expect("reseal");
        assert!(same_operator.validate().is_err());

        let mut late = manifest();
        late.host_termination.stop_latency_ms = 480;
        late.host_termination.measurement_uncertainty_ms = 21;
        late.seal().expect("reseal");
        assert!(late.validate().is_err());
    }

    #[test]
    fn rejects_duplicate_or_escaping_artifact_paths() {
        let mut duplicate = manifest();
        duplicate.artifacts.reference_profile.path = duplicate.artifacts.task_spec.path.clone();
        duplicate.seal().expect("reseal");
        assert!(duplicate.validate().is_err());

        let mut escaping = manifest();
        escaping.artifacts.reference_profile.path = "../profile.json".to_string();
        escaping.seal().expect("reseal");
        assert!(escaping.validate().is_err());
    }
}

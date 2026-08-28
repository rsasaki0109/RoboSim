//! Fixed-step, process-isolated contracts for external simulators.
//!
//! This boundary is intentionally distinct from the hardware gateway. An
//! external simulator owns its native world, physics, sensors, and transport;
//! RNE owns the portable [`rne_ai::TaskSpec`], action ordering, reset seed,
//! fixed simulation time, and evidence identities. Simulator SDK, ROS 2, DDS,
//! and vendor message types never cross this module.

pub mod conformance;
pub mod mock;
pub mod wire;

use serde::{Deserialize, Serialize};

/// Current schema for an external-simulator runtime manifest.
pub const SIMULATOR_RUNTIME_MANIFEST_SCHEMA_VERSION: u32 = 1;

/// Stable discriminator for an external-simulator runtime manifest.
pub const SIMULATOR_RUNTIME_MANIFEST_KIND: &str = "rne_external_simulator_runtime_manifest";

/// Required role of one file bound into an external simulator run.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum SimulatorArtifactRole {
    /// World geometry, lights, gravity, and physics configuration.
    World,
    /// Robot model and its joints, collision shapes, and sensors.
    RobotModel,
    /// Adapter mapping between TaskSpec fields and simulator entities.
    AdapterConfig,
}

/// One content-addressed world, robot model, or adapter configuration file.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorRuntimeArtifact {
    /// Stable artifact role.
    pub role: SimulatorArtifactRole,
    /// Portable file label without a machine-specific parent path.
    pub file: String,
    /// Exact file size.
    pub size_bytes: u64,
    /// Lowercase SHA-256 hex digest of the file bytes.
    pub sha256: String,
}

/// Simulator identity and all files required to reproduce its task mapping.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct SimulatorRuntimeManifest {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Manifest schema version.
    pub schema_version: u32,
    /// Stable simulator family, such as `gazebo_sim`.
    pub simulator_id: String,
    /// Exact simulator runtime version.
    pub simulator_version: String,
    /// Distribution or release train, such as `harmonic`.
    pub distribution: String,
    /// Expected simulation-time ticks per TaskSpec action step.
    pub fixed_delta_ticks: u64,
    /// Canonically ordered world, robot model, and adapter configuration files.
    pub artifacts: Vec<SimulatorRuntimeArtifact>,
}

impl SimulatorRuntimeManifest {
    /// Validates identity, fixed-step timing, artifact roles, order, and hashes.
    pub fn validate(&self) -> Result<(), SimulatorRuntimeManifestError> {
        if self.kind != SIMULATOR_RUNTIME_MANIFEST_KIND {
            return Err(SimulatorRuntimeManifestError::InvalidKind);
        }
        if self.schema_version != SIMULATOR_RUNTIME_MANIFEST_SCHEMA_VERSION {
            return Err(SimulatorRuntimeManifestError::UnsupportedSchemaVersion(
                self.schema_version,
            ));
        }
        for (field, value) in [
            ("simulator_id", self.simulator_id.as_str()),
            ("simulator_version", self.simulator_version.as_str()),
            ("distribution", self.distribution.as_str()),
        ] {
            if !valid_identifier(value) {
                return Err(SimulatorRuntimeManifestError::InvalidIdentity(field));
            }
        }
        if self.fixed_delta_ticks == 0 {
            return Err(SimulatorRuntimeManifestError::InvalidFixedDelta);
        }
        let expected = [
            SimulatorArtifactRole::World,
            SimulatorArtifactRole::RobotModel,
            SimulatorArtifactRole::AdapterConfig,
        ];
        if self.artifacts.len() != expected.len()
            || self
                .artifacts
                .iter()
                .map(|artifact| artifact.role)
                .ne(expected)
        {
            return Err(SimulatorRuntimeManifestError::InvalidArtifactCatalog);
        }
        for artifact in &self.artifacts {
            if artifact.file.trim().is_empty()
                || artifact.file.contains('/')
                || artifact.file.contains('\\')
                || artifact.file == "."
                || artifact.file == ".."
                || artifact.size_bytes == 0
                || !is_sha256_hex(&artifact.sha256)
            {
                return Err(SimulatorRuntimeManifestError::InvalidArtifact(
                    artifact.file.clone(),
                ));
            }
        }
        Ok(())
    }
}

/// Invalid external-simulator runtime identity or artifact catalog.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SimulatorRuntimeManifestError {
    /// Stable kind differs from the v1 contract.
    #[error("external simulator runtime manifest kind is invalid")]
    InvalidKind,
    /// Reader does not support the declared schema.
    #[error("unsupported external simulator runtime manifest schema {0}")]
    UnsupportedSchemaVersion(u32),
    /// A required simulator identity field is malformed.
    #[error("external simulator runtime manifest field {0} is invalid")]
    InvalidIdentity(&'static str),
    /// Fixed simulation delta is zero.
    #[error("external simulator fixed_delta_ticks must be greater than zero")]
    InvalidFixedDelta,
    /// Required roles are absent, duplicated, or not canonical.
    #[error("external simulator artifacts must be world, robot_model, adapter_config in order")]
    InvalidArtifactCatalog,
    /// One artifact label, size, or digest is malformed.
    #[error("external simulator runtime artifact is invalid: {0}")]
    InvalidArtifact(String),
}

pub(crate) fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

pub(crate) fn valid_identifier(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= 128
        && value.bytes().all(|byte| {
            byte.is_ascii_alphanumeric() || matches!(byte, b'_' | b'-' | b'.' | b':' | b'/')
        })
}

#[cfg(test)]
mod tests {
    use super::*;

    fn manifest() -> SimulatorRuntimeManifest {
        SimulatorRuntimeManifest {
            kind: SIMULATOR_RUNTIME_MANIFEST_KIND.to_string(),
            schema_version: SIMULATOR_RUNTIME_MANIFEST_SCHEMA_VERSION,
            simulator_id: "gazebo_sim".to_string(),
            simulator_version: "8.9.0".to_string(),
            distribution: "harmonic".to_string(),
            fixed_delta_ticks: 16_666_666,
            artifacts: [
                SimulatorArtifactRole::World,
                SimulatorArtifactRole::RobotModel,
                SimulatorArtifactRole::AdapterConfig,
            ]
            .into_iter()
            .enumerate()
            .map(|(index, role)| SimulatorRuntimeArtifact {
                role,
                file: format!("artifact-{index}.bin"),
                size_bytes: 1,
                sha256: format!("{index:064x}"),
            })
            .collect(),
        }
    }

    #[test]
    fn runtime_manifest_requires_canonical_complete_artifacts() {
        let mut value = manifest();
        value.validate().unwrap();
        value.artifacts.swap(0, 1);
        assert_eq!(
            value.validate(),
            Err(SimulatorRuntimeManifestError::InvalidArtifactCatalog)
        );
    }
}

//! Versioned artifacts for deterministic OpenSCENARIO runs.

use crate::{ScenarioRunOptions, ScenarioRunResult};
use rne_core::ControlCommand;
use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use thiserror::Error;

/// Discriminator stored in every scenario replay artifact.
pub const SCENARIO_REPLAY_KIND: &str = "rne-scenario-replay";

/// Current scenario replay artifact schema version.
pub const SCENARIO_REPLAY_SCHEMA_VERSION: u32 = 3;

const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x100000001b3;

/// Computes a stable FNV-1a digest for a replay input file.
///
/// Scenario replay artifacts store this digest for both the OpenSCENARIO XML
/// and resolved traffic-network source so replay fails clearly when either
/// input changed after the artifact was recorded.
pub fn stable_replay_input_digest(bytes: &[u8]) -> u64 {
    let mut digest = FNV_OFFSET_BASIS;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(FNV_PRIME);
    }
    digest
}

/// Errors raised while reading or validating a scenario replay artifact.
#[derive(Debug, Error)]
pub enum ScenarioReplayArtifactError {
    /// The artifact file could not be read or written.
    #[error("scenario replay artifact I/O error: {0}")]
    Io(#[from] std::io::Error),
    /// The artifact JSON was malformed.
    #[error("scenario replay artifact JSON error: {0}")]
    Json(#[from] serde_json::Error),
    /// The artifact schema differs from the version this runtime supports.
    #[error("unsupported scenario replay schema version: expected {expected}, got {actual}")]
    UnsupportedVersion {
        /// Schema version supported by this crate.
        expected: u32,
        /// Schema version found in the artifact.
        actual: u32,
    },
    /// The artifact is structurally invalid.
    #[error("invalid scenario replay artifact: {0}")]
    Invalid(String),
}

/// A deterministic scenario run record that can be verified by the CLI.
///
/// Paths use the same working-directory-relative convention as the native
/// robot replay artifact. The command transcript contains the commands
/// consumed by the runner, in order, including `reset` and a transport's
/// synthesized `quit` when it is consumed. An empty transcript represents a
/// non-interactive fixed-step run.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioReplayArtifact {
    /// Artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u32,
    /// OpenSCENARIO XML path used for the run.
    pub scenario_path: String,
    /// Stable digest of the OpenSCENARIO XML bytes used for the run.
    pub scenario_digest: u64,
    /// Traffic network path used for the run.
    pub network_path: String,
    /// Stable digest of the resolved traffic-network source bytes.
    pub network_digest: u64,
    /// RNE crate version that produced this artifact.
    pub engine_version: String,
    /// Fixed-step settings used for the run.
    pub options: ScenarioRunOptions,
    /// Number of steps completed in the final episode.
    pub executed_steps: u64,
    /// Whether `rne-asset replay` can reproduce this record automatically.
    ///
    /// This is always `true` for schema version 3 artifacts. It remains an
    /// explicit field so consumers can distinguish a verified artifact from
    /// an older or externally produced record.
    pub replayable: bool,
    /// Runner commands consumed during the scenario execution.
    pub control_commands: Vec<ControlCommand>,
    /// Final deterministic scenario result.
    pub result: ScenarioRunResult,
}

impl ScenarioReplayArtifact {
    /// Creates a validated scenario replay artifact.
    pub fn new(
        scenario_path: impl Into<String>,
        scenario_digest: u64,
        network_path: impl Into<String>,
        network_digest: u64,
        options: ScenarioRunOptions,
        executed_steps: u64,
        control_commands: Vec<ControlCommand>,
        result: ScenarioRunResult,
    ) -> Self {
        Self {
            kind: SCENARIO_REPLAY_KIND.to_string(),
            schema_version: SCENARIO_REPLAY_SCHEMA_VERSION,
            scenario_path: scenario_path.into(),
            scenario_digest,
            network_path: network_path.into(),
            network_digest,
            engine_version: env!("CARGO_PKG_VERSION").to_string(),
            options,
            executed_steps,
            replayable: true,
            control_commands,
            result,
        }
    }

    /// Validates the discriminator, schema, paths, and fixed-step metadata.
    pub fn validate(&self) -> Result<(), ScenarioReplayArtifactError> {
        if self.kind != SCENARIO_REPLAY_KIND {
            return Err(ScenarioReplayArtifactError::Invalid(format!(
                "expected kind `{SCENARIO_REPLAY_KIND}`, got `{}`",
                self.kind
            )));
        }
        if self.schema_version != SCENARIO_REPLAY_SCHEMA_VERSION {
            return Err(ScenarioReplayArtifactError::UnsupportedVersion {
                expected: SCENARIO_REPLAY_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.scenario_path.is_empty() {
            return Err(ScenarioReplayArtifactError::Invalid(
                "scenario_path must not be empty".to_string(),
            ));
        }
        if self.network_path.is_empty() {
            return Err(ScenarioReplayArtifactError::Invalid(
                "network_path must not be empty".to_string(),
            ));
        }
        if self.engine_version.trim().is_empty() {
            return Err(ScenarioReplayArtifactError::Invalid(
                "engine_version must not be empty".to_string(),
            ));
        }
        if !self.replayable {
            return Err(ScenarioReplayArtifactError::Invalid(
                "schema version 3 scenario replay artifacts must be replayable".to_string(),
            ));
        }
        if !self.options.hz.is_finite() || self.options.hz <= 0.0 {
            return Err(ScenarioReplayArtifactError::Invalid(
                "options.hz must be finite and positive".to_string(),
            ));
        }
        if self.executed_steps > self.options.steps {
            return Err(ScenarioReplayArtifactError::Invalid(format!(
                "executed_steps={} exceeds options.steps={}",
                self.executed_steps, self.options.steps
            )));
        }
        if self.result.steps != self.executed_steps {
            return Err(ScenarioReplayArtifactError::Invalid(format!(
                "result.steps={} does not match executed_steps={}",
                self.result.steps, self.executed_steps
            )));
        }
        if !self.result.route_length_m.is_finite() || self.result.route_length_m < 0.0 {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.route_length_m must be finite and non-negative".to_string(),
            ));
        }
        if !self.result.average_speed_m_s.is_finite() || self.result.average_speed_m_s < 0.0 {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.average_speed_m_s must be finite and non-negative".to_string(),
            ));
        }
        if self
            .result
            .final_positions_m
            .iter()
            .flatten()
            .any(|value| !value.is_finite())
        {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.final_positions_m must contain only finite values".to_string(),
            ));
        }
        if self
            .control_commands
            .iter()
            .any(|command| matches!(command, ControlCommand::Step { frames: 0 }))
        {
            return Err(ScenarioReplayArtifactError::Invalid(
                "control step commands must request at least one frame".to_string(),
            ));
        }
        Ok(())
    }

    /// Serializes the artifact as pretty JSON after validation.
    pub fn to_json(&self) -> Result<String, ScenarioReplayArtifactError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates a scenario replay artifact from JSON.
    pub fn from_json(text: &str) -> Result<Self, ScenarioReplayArtifactError> {
        let value: serde_json::Value = serde_json::from_str(text)?;
        if let Some(actual) = value
            .get("schema_version")
            .and_then(serde_json::Value::as_u64)
        {
            let actual = u32::try_from(actual).map_err(|_| {
                ScenarioReplayArtifactError::Invalid(format!(
                    "schema_version={actual} does not fit in u32"
                ))
            })?;
            if actual != SCENARIO_REPLAY_SCHEMA_VERSION {
                return Err(ScenarioReplayArtifactError::UnsupportedVersion {
                    expected: SCENARIO_REPLAY_SCHEMA_VERSION,
                    actual,
                });
            }
        }
        let artifact: Self = serde_json::from_value(value)?;
        artifact.validate()?;
        Ok(artifact)
    }

    /// Writes the artifact as pretty JSON, creating its parent directory.
    pub fn write_json(&self, path: impl AsRef<Path>) -> Result<(), ScenarioReplayArtifactError> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }
        fs::write(path, self.to_json()?)?;
        Ok(())
    }

    /// Reads and validates a scenario replay artifact from a file.
    pub fn read_json(path: impl AsRef<Path>) -> Result<Self, ScenarioReplayArtifactError> {
        let text = fs::read_to_string(path)?;
        Self::from_json(&text)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn result() -> ScenarioRunResult {
        ScenarioRunResult {
            stable_hash: 0x1234,
            signal_violations: 0,
            collisions: 0,
            final_positions_m: vec![[1.0, 0.0, 2.0]],
            route_length_m: 10.0,
            average_speed_m_s: 2.0,
            steps: 4,
        }
    }

    #[test]
    fn artifact_roundtrips_and_validates() {
        let artifact = ScenarioReplayArtifact::new(
            "scenario.xosc",
            stable_replay_input_digest(b"scenario"),
            "network.rne.traffic.json",
            stable_replay_input_digest(b"network"),
            ScenarioRunOptions { steps: 4, hz: 60.0 },
            4,
            vec![],
            result(),
        );
        let json = artifact.to_json().expect("serialize artifact");
        let loaded = ScenarioReplayArtifact::from_json(&json).expect("parse artifact");
        assert_eq!(loaded, artifact);
    }

    #[test]
    fn control_transcript_roundtrips() {
        let artifact = ScenarioReplayArtifact::new(
            "scenario.xosc",
            stable_replay_input_digest(b"scenario"),
            "network.rne.traffic.json",
            stable_replay_input_digest(b"network"),
            ScenarioRunOptions {
                steps: 10,
                hz: 60.0,
            },
            4,
            vec![ControlCommand::Step { frames: 4 }, ControlCommand::Quit],
            result(),
        );
        artifact.validate().expect("control record metadata");
        let json = artifact.to_json().expect("serialize control transcript");
        let loaded = ScenarioReplayArtifact::from_json(&json).expect("parse control transcript");
        assert_eq!(loaded.control_commands, artifact.control_commands);
        assert!(loaded.replayable);
    }

    #[test]
    fn input_digest_has_a_fixed_vector() {
        assert_eq!(
            stable_replay_input_digest(b"RNE scenario replay"),
            0xf5c9_1bfc_e251_9ffd
        );
    }

    #[test]
    fn result_step_mismatch_is_rejected() {
        let artifact = ScenarioReplayArtifact::new(
            "scenario.xosc",
            stable_replay_input_digest(b"scenario"),
            "network.rne.traffic.json",
            stable_replay_input_digest(b"network"),
            ScenarioRunOptions { steps: 4, hz: 60.0 },
            3,
            vec![],
            result(),
        );
        assert!(matches!(
            artifact.validate(),
            Err(ScenarioReplayArtifactError::Invalid(message))
                if message.contains("result.steps")
        ));
    }

    #[test]
    fn older_schema_is_reported_before_new_required_fields() {
        assert!(matches!(
            ScenarioReplayArtifact::from_json(r#"{"schema_version":2}"#),
            Err(ScenarioReplayArtifactError::UnsupportedVersion {
                expected: SCENARIO_REPLAY_SCHEMA_VERSION,
                actual: 2,
            })
        ));
    }
}

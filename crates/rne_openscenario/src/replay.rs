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
pub const SCENARIO_REPLAY_SCHEMA_VERSION: u32 = 2;

/// Errors raised while reading or validating a scenario replay artifact.
#[derive(Debug, Error)]
pub enum ScenarioReplayArtifactError {
    /// The artifact file could not be read or written.
    #[error("scenario replay artifact I/O error: {0}")]
    Io(#[from] std::io::Error),
    /// The artifact JSON was malformed.
    #[error("scenario replay artifact JSON error: {0}")]
    Json(#[from] serde_json::Error),
    /// The artifact schema is newer than this runtime supports.
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
    /// Traffic network path used for the run.
    pub network_path: String,
    /// Fixed-step settings used for the run.
    pub options: ScenarioRunOptions,
    /// Number of steps completed in the final episode.
    pub executed_steps: u64,
    /// Whether `rne-asset replay` can reproduce this record automatically.
    ///
    /// This is always `true` for schema version 2 artifacts. It remains an
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
        network_path: impl Into<String>,
        options: ScenarioRunOptions,
        executed_steps: u64,
        control_commands: Vec<ControlCommand>,
        result: ScenarioRunResult,
    ) -> Self {
        Self {
            kind: SCENARIO_REPLAY_KIND.to_string(),
            schema_version: SCENARIO_REPLAY_SCHEMA_VERSION,
            scenario_path: scenario_path.into(),
            network_path: network_path.into(),
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
        if !self.replayable {
            return Err(ScenarioReplayArtifactError::Invalid(
                "schema version 2 scenario replay artifacts must be replayable".to_string(),
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
        Ok(())
    }

    /// Serializes the artifact as pretty JSON after validation.
    pub fn to_json(&self) -> Result<String, ScenarioReplayArtifactError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates a scenario replay artifact from JSON.
    pub fn from_json(text: &str) -> Result<Self, ScenarioReplayArtifactError> {
        let artifact: Self = serde_json::from_str(text)?;
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
            "network.rne.traffic.json",
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
            "network.rne.traffic.json",
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
}

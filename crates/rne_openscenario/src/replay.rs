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
pub const SCENARIO_REPLAY_SCHEMA_VERSION: u32 = 4;

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

/// Exact files used to execute and later verify a scenario replay.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct ScenarioReplayInputs {
    /// OpenSCENARIO XML path used for the run.
    pub scenario_path: String,
    /// Stable digest of the OpenSCENARIO XML bytes used for the run.
    pub scenario_digest: u64,
    /// Traffic network path used for the run.
    pub network_path: String,
    /// Stable digest of the resolved traffic-network source bytes.
    pub network_digest: u64,
}

impl ScenarioReplayInputs {
    /// Creates replay-input metadata from resolved paths and byte digests.
    pub fn new(
        scenario_path: impl Into<String>,
        scenario_digest: u64,
        network_path: impl Into<String>,
        network_digest: u64,
    ) -> Self {
        Self {
            scenario_path: scenario_path.into(),
            scenario_digest,
            network_path: network_path.into(),
            network_digest,
        }
    }
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
    /// This is always `true` for schema version 4 artifacts. It remains an
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
        inputs: ScenarioReplayInputs,
        options: ScenarioRunOptions,
        executed_steps: u64,
        control_commands: Vec<ControlCommand>,
        result: ScenarioRunResult,
    ) -> Self {
        Self {
            kind: SCENARIO_REPLAY_KIND.to_string(),
            schema_version: SCENARIO_REPLAY_SCHEMA_VERSION,
            scenario_path: inputs.scenario_path,
            scenario_digest: inputs.scenario_digest,
            network_path: inputs.network_path,
            network_digest: inputs.network_digest,
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
                "schema version 4 scenario replay artifacts must be replayable".to_string(),
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
        if self.result.final_actors.len() != self.result.final_positions_m.len() {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.final_actors must correspond one-to-one with final_positions_m".to_string(),
            ));
        }
        if self
            .result
            .final_actors
            .windows(2)
            .any(|window| window[0].stable_uuid >= window[1].stable_uuid)
        {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.final_actors must have unique UUIDs in canonical order".to_string(),
            ));
        }
        let mut names = self
            .result
            .final_actors
            .iter()
            .map(|actor| actor.name.as_str())
            .collect::<Vec<_>>();
        names.sort_unstable();
        if names.windows(2).any(|window| window[0] == window[1]) {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.final_actors must have unique names".to_string(),
            ));
        }
        for (actor, position_m) in self
            .result
            .final_actors
            .iter()
            .zip(&self.result.final_positions_m)
        {
            if actor.name.trim().is_empty()
                || uuid::Uuid::parse_str(&actor.stable_uuid).is_err()
                || actor.route_id.trim().is_empty()
                || actor
                    .final_position_m
                    .iter()
                    .any(|value| !value.is_finite())
                || !actor.final_heading_rad.is_finite()
                || !actor.final_speed_m_s.is_finite()
                || actor.final_speed_m_s < 0.0
            {
                return Err(ScenarioReplayArtifactError::Invalid(
                    "result.final_actors must contain named finite non-negative states".to_string(),
                ));
            }
            if actor.final_position_m != *position_m {
                return Err(ScenarioReplayArtifactError::Invalid(
                    "result.final_actors positions must match final_positions_m".to_string(),
                ));
            }
        }
        if self
            .result
            .minimum_observed_gap_m
            .is_some_and(|gap_m| !gap_m.is_finite())
        {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.minimum_observed_gap_m must be finite".to_string(),
            ));
        }
        let ownership = self.result.ownership;
        if ownership.runtime_owned_actor_count + ownership.external_owned_actor_count
            != ownership.total_actor_count
            || ownership.runtime_advanced_actor_count > ownership.runtime_owned_actor_count
            || ownership.external_observed_actor_count > ownership.external_owned_actor_count
            || ownership.invalid_actor_count != 0
            || (self.executed_steps != 0
                && ownership.total_actor_count != self.result.final_actors.len())
        {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.ownership counts are inconsistent".to_string(),
            ));
        }
        if self.result.action_evidence.windows(2).any(|window| {
            let left = &window[0];
            let right = &window[1];
            left.start_time_s
                .total_cmp(&right.start_time_s)
                .then_with(|| left.entity_name.cmp(&right.entity_name))
                .then_with(|| left.source_action_index.cmp(&right.source_action_index))
                .is_gt()
        }) || self.result.action_evidence.iter().any(|evidence| {
            !evidence.start_time_s.is_finite()
                || evidence.start_time_s < 0.0
                || evidence.entity_name.trim().is_empty()
                || evidence.applied_step == 0
                || evidence.applied_step > self.executed_steps
        }) {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.action_evidence must be finite and canonically ordered".to_string(),
            ));
        }
        let expected_result_digest = crate::runtime::scenario_result_digest(
            &self.result.final_actors,
            &self.result.action_evidence,
        )
        .map_err(|error| ScenarioReplayArtifactError::Invalid(error.to_string()))?;
        if self.result.result_digest != expected_result_digest {
            return Err(ScenarioReplayArtifactError::Invalid(
                "result.result_digest does not match actor/action evidence".to_string(),
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
        let final_actors = vec![crate::ScenarioActorResult {
            name: "ego".to_string(),
            stable_uuid: uuid::Uuid::from_u128(0x0001_0000_0000_0000_0000_0000_0000_0000)
                .to_string(),
            kind: crate::ScenarioEntityKind::MotorVehicle,
            pose_source: rne_traffic::TrafficPoseSource::Runtime,
            route_id: "route:scenario:motor_vehicle".to_string(),
            final_position_m: [1.0, 0.0, 2.0],
            final_heading_rad: 0.0,
            final_speed_m_s: 2.0,
        }];
        let action_evidence = Vec::new();
        let result_digest = crate::runtime::scenario_result_digest(&final_actors, &action_evidence)
            .expect("result digest");
        ScenarioRunResult {
            stable_hash: 0x1234,
            result_digest,
            signal_violations: 0,
            collisions: 0,
            final_positions_m: vec![[1.0, 0.0, 2.0]],
            final_actors,
            action_evidence,
            unapplied_action_count: 0,
            minimum_observed_gap_m: None,
            ownership: rne_traffic::TrafficOwnershipMetrics {
                total_actor_count: 1,
                runtime_owned_actor_count: 1,
                external_owned_actor_count: 0,
                runtime_advanced_actor_count: 1,
                external_observed_actor_count: 0,
                invalid_actor_count: 0,
            },
            route_length_m: 10.0,
            average_speed_m_s: 2.0,
            steps: 4,
        }
    }

    fn inputs() -> ScenarioReplayInputs {
        ScenarioReplayInputs::new(
            "scenario.xosc",
            stable_replay_input_digest(b"scenario"),
            "network.rne.traffic.json",
            stable_replay_input_digest(b"network"),
        )
    }

    #[test]
    fn artifact_roundtrips_and_validates() {
        let artifact = ScenarioReplayArtifact::new(
            inputs(),
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
    fn replay_comparison_accepts_sub_nanounit_json_rounding() {
        let mut recorded = result();
        recorded.minimum_observed_gap_m = Some(15.599_999_999_986_355);
        let mut actual = recorded.clone();
        actual.minimum_observed_gap_m = Some(15.599_999_999_986_357);

        assert_ne!(actual, recorded);
        assert!(actual.replay_matches(&recorded));
    }

    #[test]
    fn control_transcript_roundtrips() {
        let artifact = ScenarioReplayArtifact::new(
            inputs(),
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
            inputs(),
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
    fn finite_negative_gap_is_preserved_as_violation_evidence() {
        let mut overlapping = result();
        overlapping.minimum_observed_gap_m = Some(-0.25);
        overlapping.collisions = 1;
        let artifact = ScenarioReplayArtifact::new(
            inputs(),
            ScenarioRunOptions { steps: 4, hz: 60.0 },
            4,
            vec![],
            overlapping,
        );

        artifact
            .validate()
            .expect("a failing run must retain its finite overlap gap");
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

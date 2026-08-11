//! Versioned failure replay, minimization, and semantic diff support for Behavior CI.

use crate::behavior::{
    run_behavior_scenarios_with_replays, BehaviorContractDescriptor, BehaviorScenario,
    BehaviorViolation,
};
use serde::{Deserialize, Serialize};
use serde_json::Value;
use std::collections::BTreeSet;
use std::fmt;
use std::fs;
use std::io;
use std::path::Path;
use thiserror::Error;

/// Stable artifact-kind discriminator for Behavior CI failure replays.
pub const BEHAVIOR_REPLAY_KIND: &str = "rne_behavior_replay";
/// Current Behavior CI replay schema version.
pub const BEHAVIOR_REPLAY_SCHEMA_VERSION: u32 = 1;
/// Current typed behavior-contract schema version.
pub const BEHAVIOR_CONTRACT_SCHEMA_VERSION: u32 = 2;
/// Current deterministic seed-manifest schema version.
pub const BEHAVIOR_SEED_MANIFEST_SCHEMA_VERSION: u32 = 1;
/// Current standalone failure-case schema version.
pub const BEHAVIOR_FAILURE_CASE_SCHEMA_VERSION: u32 = 1;
/// Absolute tolerance for replayed floating-point observation fields.
///
/// Discrete fields, timestamps, actions, compatibility metadata, and state
/// digests remain exact. This tolerance only absorbs sub-picometer arithmetic
/// drift in derived `f64` observations across otherwise identical processes.
pub const BEHAVIOR_REPLAY_FLOAT_TOLERANCE: f64 = 1.0e-12;
/// Number of deterministic bisection rounds used for active numeric dimensions.
pub const BEHAVIOR_MINIMIZATION_BISECTION_STEPS: u32 = 16;

/// Replay artifact I/O, compatibility, validation, or verification failure.
#[derive(Debug, Error)]
pub enum BehaviorReplayError {
    /// The artifact could not be read or written.
    #[error(transparent)]
    Io(#[from] io::Error),
    /// The artifact could not be serialized or deserialized.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    /// The artifact uses an unsupported schema version.
    #[error("unsupported behavior replay schema: expected {expected}, got {actual}")]
    UnsupportedVersion {
        /// Schema version supported by this engine.
        expected: u32,
        /// Schema version found in the artifact.
        actual: u32,
    },
    /// The deterministic seed manifest uses an unsupported schema version.
    #[error("unsupported behavior seed manifest schema: expected {expected}, got {actual}")]
    UnsupportedSeedManifestVersion {
        /// Schema version supported by this engine.
        expected: u32,
        /// Schema version found in the manifest.
        actual: u32,
    },
    /// The standalone failure case uses an unsupported schema version.
    #[error("unsupported behavior failure-case schema: expected {expected}, got {actual}")]
    UnsupportedFailureCaseVersion {
        /// Schema version supported by this engine.
        expected: u32,
        /// Schema version found in the failure case.
        actual: u32,
    },
    /// The artifact was produced by an incompatible engine version.
    #[error("incompatible behavior replay engine: expected {expected}, got {actual}")]
    IncompatibleEngineVersion {
        /// Engine version running the verifier.
        expected: String,
        /// Engine version recorded in the artifact.
        actual: String,
    },
    /// The artifact uses an incompatible typed-contract schema.
    #[error("incompatible behavior contract schema: expected {expected}, got {actual}")]
    IncompatibleContractSchema {
        /// Contract schema supported by this engine.
        expected: u32,
        /// Contract schema recorded in the artifact.
        actual: u32,
    },
    /// The artifact requests a floating-point tolerance other than the engine policy.
    #[error("incompatible behavior replay float tolerance: expected {expected}, got {actual}")]
    IncompatibleFloatTolerance {
        /// Floating-point tolerance supported by this engine.
        expected: f64,
        /// Floating-point tolerance recorded in the artifact.
        actual: f64,
    },
    /// The artifact violates a deterministic schema invariant.
    #[error("invalid behavior replay: {0}")]
    Invalid(String),
    /// The recorded failure did not recur during verification.
    #[error("behavior replay did not reproduce contract `{contract}`")]
    ExpectedFailureDidNotRecur {
        /// Contract expected to fail.
        contract: String,
    },
    /// Re-execution diverged from the recorded replay.
    #[error("behavior replay diverged: {0}")]
    Diverged(BehaviorReplayDiff),
}

/// One stable, named randomization or scenario-override value.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(untagged)]
pub enum BehaviorDimensionValue {
    /// Boolean feature or override switch.
    Boolean(bool),
    /// Finite continuous value.
    Number(f64),
    /// Stable discrete choice label.
    Text(String),
}

impl BehaviorDimensionValue {
    fn validate(&self, field: &str) -> Result<(), BehaviorReplayError> {
        match self {
            Self::Number(value) if !value.is_finite() => Err(BehaviorReplayError::Invalid(
                format!("{field} must be finite"),
            )),
            Self::Text(value) if value.trim().is_empty() => Err(BehaviorReplayError::Invalid(
                format!("{field} must not be empty"),
            )),
            Self::Boolean(_) | Self::Number(_) | Self::Text(_) => Ok(()),
        }
    }

    fn same_kind(&self, other: &Self) -> bool {
        matches!(
            (self, other),
            (Self::Boolean(_), Self::Boolean(_))
                | (Self::Number(_), Self::Number(_))
                | (Self::Text(_), Self::Text(_))
        )
    }
}

/// Stable named randomization dimension with its neutral baseline.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorDimension {
    /// Stable dimension name.
    pub name: String,
    /// Value used by this run.
    pub value: BehaviorDimensionValue,
    /// Neutral value used while minimizing a failure.
    pub baseline: BehaviorDimensionValue,
}

impl BehaviorDimension {
    /// Creates and validates one behavior dimension.
    pub fn new(
        name: impl Into<String>,
        value: BehaviorDimensionValue,
        baseline: BehaviorDimensionValue,
    ) -> Result<Self, BehaviorReplayError> {
        let dimension = Self {
            name: name.into(),
            value,
            baseline,
        };
        dimension.validate()?;
        Ok(dimension)
    }

    /// Creates a finite continuous dimension.
    pub fn number(
        name: impl Into<String>,
        value: f64,
        baseline: f64,
    ) -> Result<Self, BehaviorReplayError> {
        Self::new(
            name,
            BehaviorDimensionValue::Number(value),
            BehaviorDimensionValue::Number(baseline),
        )
    }

    /// Creates a boolean dimension.
    pub fn boolean(
        name: impl Into<String>,
        value: bool,
        baseline: bool,
    ) -> Result<Self, BehaviorReplayError> {
        Self::new(
            name,
            BehaviorDimensionValue::Boolean(value),
            BehaviorDimensionValue::Boolean(baseline),
        )
    }

    /// Creates a discrete text dimension.
    pub fn text(
        name: impl Into<String>,
        value: impl Into<String>,
        baseline: impl Into<String>,
    ) -> Result<Self, BehaviorReplayError> {
        Self::new(
            name,
            BehaviorDimensionValue::Text(value.into()),
            BehaviorDimensionValue::Text(baseline.into()),
        )
    }

    /// Returns true when this dimension differs from its neutral baseline.
    pub fn is_active(&self) -> bool {
        self.value != self.baseline
    }

    fn validate(&self) -> Result<(), BehaviorReplayError> {
        if self.name.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "behavior dimension name must not be empty".to_string(),
            ));
        }
        self.value
            .validate(&format!("dimension `{}` value", self.name))?;
        self.baseline
            .validate(&format!("dimension `{}` baseline", self.name))?;
        if !self.value.same_kind(&self.baseline) {
            return Err(BehaviorReplayError::Invalid(format!(
                "dimension `{}` value and baseline must have the same type",
                self.name
            )));
        }
        Ok(())
    }
}

/// Deterministic list of seeds requested by one Behavior CI invocation.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorSeedManifest {
    /// Manifest schema version.
    pub schema_version: u32,
    /// Stable scenario name.
    pub scenario: String,
    /// Sorted, unique seeds.
    pub seeds: Vec<u64>,
}

impl BehaviorSeedManifest {
    /// Creates a sorted and deduplicated seed manifest.
    pub fn new(scenario: impl Into<String>, seeds: impl IntoIterator<Item = u64>) -> Self {
        Self {
            schema_version: BEHAVIOR_SEED_MANIFEST_SCHEMA_VERSION,
            scenario: scenario.into(),
            seeds: seeds
                .into_iter()
                .collect::<BTreeSet<_>>()
                .into_iter()
                .collect(),
        }
    }

    /// Validates schema, scenario identity, and sorted unique seed ordering.
    pub fn validate(&self) -> Result<(), BehaviorReplayError> {
        if self.schema_version != BEHAVIOR_SEED_MANIFEST_SCHEMA_VERSION {
            return Err(BehaviorReplayError::UnsupportedSeedManifestVersion {
                expected: BEHAVIOR_SEED_MANIFEST_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.scenario.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "behavior seed manifest scenario must not be empty".to_string(),
            ));
        }
        if self.seeds.windows(2).any(|seeds| seeds[0] >= seeds[1]) {
            return Err(BehaviorReplayError::Invalid(
                "behavior seed manifest seeds must be sorted and unique".to_string(),
            ));
        }
        Ok(())
    }
}

/// Scripted transition represented by one behavior replay frame.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum BehaviorReplayAction {
    /// Capture of scenario state before the first transition.
    InitialObservation,
    /// One deterministic call to [`BehaviorScenario::advance`].
    Advance,
}

/// One recorded action, observation, and deterministic state digest.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorReplayFrame {
    /// Zero-based evaluated behavior step, including step zero.
    pub step: u64,
    /// Simulation timestamp represented as stable integer ticks.
    pub sim_time_ticks: u64,
    /// Scripted behavior action that produced this frame.
    pub action: BehaviorReplayAction,
    /// Backend-neutral serialized task observation.
    pub observation: Value,
    /// Stable same-backend state digest after this step.
    pub state_digest: u64,
}

/// Contract and first violation reproduced by a failure replay.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorReplayFailure {
    /// Versioned contract descriptor.
    pub contract: BehaviorContractDescriptor,
    /// First observed violation.
    pub violation: BehaviorViolation,
}

/// Provenance recorded after deterministic failure minimization.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorMinimizationMetadata {
    /// Digest at the original failure step.
    pub source_state_digest: u64,
    /// Number of reproduction attempts made by the minimizer.
    pub attempts: u32,
    /// Number of non-baseline dimensions in the source replay.
    pub active_dimensions_before: usize,
    /// Number of non-baseline dimensions in the minimized replay.
    pub active_dimensions_after: usize,
}

/// Compact, self-contained Behavior CI failure replay.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorReplayArtifact {
    /// Artifact schema version.
    pub schema_version: u32,
    /// Stable artifact-kind discriminator.
    pub kind: String,
    /// Producing RNE engine version.
    pub engine_version: String,
    /// Typed behavior-contract schema version.
    pub contract_schema_version: u32,
    /// Stable digest of the ordered contract descriptors.
    pub contract_digest: u64,
    /// Stable scenario name.
    pub scenario: String,
    /// Stable digest of the scenario input.
    pub scenario_digest: u64,
    /// Deterministic scenario seed.
    pub seed: u64,
    /// Fixed simulation duration per evaluated step, in ticks.
    pub fixed_delta_ticks: u64,
    /// Absolute tolerance used only for floating-point observation fields.
    pub observation_numeric_tolerance: f64,
    /// Sorted stable randomization dimensions used by the run.
    pub dimensions: Vec<BehaviorDimension>,
    /// Contracts in declaration order.
    pub contracts: Vec<BehaviorContractDescriptor>,
    /// Frames from step zero through the first violating step.
    pub frames: Vec<BehaviorReplayFrame>,
    /// First behavior failure selected by step then declaration order.
    pub failure: BehaviorReplayFailure,
    /// Minimization provenance, when this is a minimized artifact.
    #[serde(default, skip_serializing_if = "Option::is_none")]
    pub minimization: Option<BehaviorMinimizationMetadata>,
}

impl BehaviorReplayArtifact {
    /// Creates and validates one compact failure replay.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        scenario: impl Into<String>,
        scenario_digest: u64,
        seed: u64,
        fixed_delta_ticks: u64,
        mut dimensions: Vec<BehaviorDimension>,
        contracts: Vec<BehaviorContractDescriptor>,
        frames: Vec<BehaviorReplayFrame>,
        failure: BehaviorReplayFailure,
    ) -> Result<Self, BehaviorReplayError> {
        dimensions.sort_by(|left, right| left.name.cmp(&right.name));
        let contract_digest = digest_serializable(&contracts)?;
        let artifact = Self {
            schema_version: BEHAVIOR_REPLAY_SCHEMA_VERSION,
            kind: BEHAVIOR_REPLAY_KIND.to_string(),
            engine_version: env!("CARGO_PKG_VERSION").to_string(),
            contract_schema_version: BEHAVIOR_CONTRACT_SCHEMA_VERSION,
            contract_digest,
            scenario: scenario.into(),
            scenario_digest,
            seed,
            fixed_delta_ticks,
            observation_numeric_tolerance: BEHAVIOR_REPLAY_FLOAT_TOLERANCE,
            dimensions,
            contracts,
            frames,
            failure,
            minimization: None,
        };
        artifact.validate()?;
        Ok(artifact)
    }

    /// Returns a filesystem-safe deterministic artifact filename.
    pub fn file_name(&self) -> String {
        format!(
            "{}-seed-{}.rne-replay",
            sanitize_file_component(&self.scenario),
            self.seed
        )
    }

    /// Returns a filesystem-safe deterministic minimized artifact filename.
    pub fn minimized_file_name(&self) -> String {
        format!(
            "{}-seed-{}-minimized.rne-replay",
            sanitize_file_component(&self.scenario),
            self.seed
        )
    }

    /// Validates schema and deterministic ordering invariants.
    pub fn validate(&self) -> Result<(), BehaviorReplayError> {
        if self.schema_version != BEHAVIOR_REPLAY_SCHEMA_VERSION {
            return Err(BehaviorReplayError::UnsupportedVersion {
                expected: BEHAVIOR_REPLAY_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.kind != BEHAVIOR_REPLAY_KIND {
            return Err(BehaviorReplayError::Invalid(format!(
                "kind must be `{BEHAVIOR_REPLAY_KIND}`"
            )));
        }
        if self.engine_version.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "engine_version must not be empty".to_string(),
            ));
        }
        if self.contract_schema_version == 0 {
            return Err(BehaviorReplayError::Invalid(
                "contract_schema_version must be greater than zero".to_string(),
            ));
        }
        if self.scenario.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "scenario must not be empty".to_string(),
            ));
        }
        if self.fixed_delta_ticks == 0 {
            return Err(BehaviorReplayError::Invalid(
                "fixed_delta_ticks must be greater than zero".to_string(),
            ));
        }
        if !self.observation_numeric_tolerance.is_finite()
            || self.observation_numeric_tolerance < 0.0
        {
            return Err(BehaviorReplayError::Invalid(
                "observation_numeric_tolerance must be finite and non-negative".to_string(),
            ));
        }
        validate_dimensions(&self.dimensions)?;
        validate_contracts(&self.contracts)?;
        let expected_contract_digest = digest_serializable(&self.contracts)?;
        if self.contract_digest != expected_contract_digest {
            return Err(BehaviorReplayError::Invalid(format!(
                "contract_digest mismatch: expected {expected_contract_digest:#018x}, got {:#018x}",
                self.contract_digest
            )));
        }
        let matching_contract = self
            .contracts
            .iter()
            .find(|contract| contract.name == self.failure.contract.name)
            .ok_or_else(|| {
                BehaviorReplayError::Invalid(format!(
                    "failure contract `{}` is absent from the contract manifest",
                    self.failure.contract.name
                ))
            })?;
        if matching_contract != &self.failure.contract {
            return Err(BehaviorReplayError::Invalid(format!(
                "failure contract `{}` does not match its manifest descriptor",
                self.failure.contract.name
            )));
        }
        if self.failure.violation.entities != self.failure.contract.entities {
            return Err(BehaviorReplayError::Invalid(
                "failure violation entities do not match the contract descriptor".to_string(),
            ));
        }
        if self.failure.violation.message.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "failure violation message must not be empty".to_string(),
            ));
        }
        if self.frames.is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "failure replay must contain at least step zero".to_string(),
            ));
        }
        for (index, frame) in self.frames.iter().enumerate() {
            let expected_step = index as u64;
            if frame.step != expected_step {
                return Err(BehaviorReplayError::Invalid(format!(
                    "frame {index} records step {}",
                    frame.step
                )));
            }
            let expected_ticks = self
                .fixed_delta_ticks
                .checked_mul(expected_step)
                .ok_or_else(|| {
                    BehaviorReplayError::Invalid(
                        "frame timestamp overflowed fixed-step clock".to_string(),
                    )
                })?;
            if frame.sim_time_ticks != expected_ticks {
                return Err(BehaviorReplayError::Invalid(format!(
                    "frame {index} has sim_time_ticks={}, expected {expected_ticks}",
                    frame.sim_time_ticks
                )));
            }
            let expected_action = if index == 0 {
                BehaviorReplayAction::InitialObservation
            } else {
                BehaviorReplayAction::Advance
            };
            if frame.action != expected_action {
                return Err(BehaviorReplayError::Invalid(format!(
                    "frame {index} has action {:?}, expected {expected_action:?}",
                    frame.action
                )));
            }
        }
        let final_frame = self.frames.last().expect("checked non-empty");
        if final_frame.step != self.failure.violation.step {
            return Err(BehaviorReplayError::Invalid(format!(
                "final frame step {} does not match violation step {}",
                final_frame.step, self.failure.violation.step
            )));
        }
        if final_frame.sim_time_ticks != self.failure.violation.sim_time_ticks {
            return Err(BehaviorReplayError::Invalid(
                "final frame timestamp does not match violation timestamp".to_string(),
            ));
        }
        if final_frame.state_digest != self.failure.violation.state_digest {
            return Err(BehaviorReplayError::Invalid(
                "final frame digest does not match violation digest".to_string(),
            ));
        }
        if let Some(minimization) = &self.minimization {
            if minimization.active_dimensions_after > minimization.active_dimensions_before {
                return Err(BehaviorReplayError::Invalid(
                    "minimization increased the active dimension count".to_string(),
                ));
            }
        }
        Ok(())
    }

    /// Rejects artifacts produced by an incompatible engine or contract schema.
    pub fn validate_compatibility(&self) -> Result<(), BehaviorReplayError> {
        self.validate()?;
        let current_engine = env!("CARGO_PKG_VERSION");
        if self.engine_version != current_engine {
            return Err(BehaviorReplayError::IncompatibleEngineVersion {
                expected: current_engine.to_string(),
                actual: self.engine_version.clone(),
            });
        }
        if self.contract_schema_version != BEHAVIOR_CONTRACT_SCHEMA_VERSION {
            return Err(BehaviorReplayError::IncompatibleContractSchema {
                expected: BEHAVIOR_CONTRACT_SCHEMA_VERSION,
                actual: self.contract_schema_version,
            });
        }
        if self.observation_numeric_tolerance != BEHAVIOR_REPLAY_FLOAT_TOLERANCE {
            return Err(BehaviorReplayError::IncompatibleFloatTolerance {
                expected: BEHAVIOR_REPLAY_FLOAT_TOLERANCE,
                actual: self.observation_numeric_tolerance,
            });
        }
        Ok(())
    }

    /// Serializes a validated artifact as human-readable JSON.
    pub fn to_json_pretty(&self) -> Result<String, BehaviorReplayError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates an artifact from JSON.
    pub fn from_json(text: &str) -> Result<Self, BehaviorReplayError> {
        let artifact: Self = serde_json::from_str(text)?;
        artifact.validate()?;
        Ok(artifact)
    }

    /// Writes a validated artifact to disk.
    pub fn write_json(&self, path: impl AsRef<Path>) -> Result<(), BehaviorReplayError> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        fs::write(path, self.to_json_pretty()?)?;
        Ok(())
    }

    /// Reads and validates an artifact from disk.
    pub fn read_json(path: impl AsRef<Path>) -> Result<Self, BehaviorReplayError> {
        Self::from_json(&fs::read_to_string(path)?)
    }
}

/// Standalone scenario override used to reproduce a known behavior failure.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorFailureCase {
    /// Failure-case schema version.
    pub schema_version: u32,
    /// Stable scenario name.
    pub scenario: String,
    /// Deterministic scenario seed.
    pub seed: u64,
    /// Named dimensions and overrides applied to the scenario.
    pub dimensions: Vec<BehaviorDimension>,
    /// Contract expected to fail.
    pub expected_contract: String,
}

impl BehaviorFailureCase {
    /// Creates a standalone case from a replay artifact.
    pub fn from_replay(artifact: &BehaviorReplayArtifact) -> Self {
        Self {
            schema_version: BEHAVIOR_FAILURE_CASE_SCHEMA_VERSION,
            scenario: artifact.scenario.clone(),
            seed: artifact.seed,
            dimensions: artifact.dimensions.clone(),
            expected_contract: artifact.failure.contract.name.clone(),
        }
    }

    /// Validates schema and dimension ordering.
    pub fn validate(&self) -> Result<(), BehaviorReplayError> {
        if self.schema_version != BEHAVIOR_FAILURE_CASE_SCHEMA_VERSION {
            return Err(BehaviorReplayError::UnsupportedFailureCaseVersion {
                expected: BEHAVIOR_FAILURE_CASE_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        if self.scenario.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "failure case scenario must not be empty".to_string(),
            ));
        }
        if self.expected_contract.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "failure case expected_contract must not be empty".to_string(),
            ));
        }
        validate_dimensions(&self.dimensions)
    }

    /// Serializes a validated case as human-readable JSON.
    pub fn to_json_pretty(&self) -> Result<String, BehaviorReplayError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates a case from JSON.
    pub fn from_json(text: &str) -> Result<Self, BehaviorReplayError> {
        let case: Self = serde_json::from_str(text)?;
        case.validate()?;
        Ok(case)
    }

    /// Writes a validated case to disk.
    pub fn write_json(&self, path: impl AsRef<Path>) -> Result<(), BehaviorReplayError> {
        let path = path.as_ref();
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        fs::write(path, self.to_json_pretty()?)?;
        Ok(())
    }

    /// Reads and validates a case from disk.
    pub fn read_json(path: impl AsRef<Path>) -> Result<Self, BehaviorReplayError> {
        Self::from_json(&fs::read_to_string(path)?)
    }
}

/// One named field that differs between two behavior replay artifacts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorFieldDiff {
    /// Stable JSON-style field path.
    pub path: String,
    /// Recorded value.
    pub expected: Value,
    /// Re-executed or candidate value.
    pub actual: Value,
    /// Absolute numeric delta when both values are numeric.
    pub absolute_delta: Option<f64>,
}

/// First semantic divergence between two behavior replay artifacts.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorReplayDiff {
    /// First divergent behavior step, or `None` for manifest-level divergence.
    pub first_divergent_step: Option<u64>,
    /// Recorded state digest at the divergent step.
    pub expected_state_digest: Option<u64>,
    /// Candidate state digest at the divergent step.
    pub actual_state_digest: Option<u64>,
    /// Bounded, deterministically ordered field differences.
    pub fields: Vec<BehaviorFieldDiff>,
}

impl fmt::Display for BehaviorReplayDiff {
    fn fmt(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self.first_divergent_step {
            Some(step) => write!(formatter, "first divergent step {step}")?,
            None => formatter.write_str("manifest divergence")?,
        }
        if let Some(field) = self.fields.first() {
            write!(
                formatter,
                " at {}: expected {}, actual {}",
                field.path, field.expected, field.actual
            )?;
            if let Some(delta) = field.absolute_delta {
                write!(formatter, ", absolute_delta={delta}")?;
            }
        }
        Ok(())
    }
}

/// Successful deterministic replay verification summary.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorReplayVerification {
    /// Reproduced scenario seed.
    pub seed: u64,
    /// Reproduced contract name.
    pub contract: String,
    /// Reproduced first violating step.
    pub step: u64,
    /// Number of exactly matched replay frames.
    pub matched_frames: usize,
    /// Reproduced failure-state digest.
    pub state_digest: u64,
}

/// Result of deterministic dimension minimization.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct BehaviorMinimizationResult {
    /// Number of reproduction attempts made by the minimizer.
    pub attempts: u32,
    /// Number of active dimensions before minimization.
    pub active_dimensions_before: usize,
    /// Number of active dimensions after minimization.
    pub active_dimensions_after: usize,
    /// Minimized replay that still names the same failed contract.
    pub artifact: BehaviorReplayArtifact,
}

/// Compares two behavior replays and returns their first meaningful divergence.
pub fn diff_behavior_replays(
    expected: &BehaviorReplayArtifact,
    actual: &BehaviorReplayArtifact,
    numeric_tolerance: f64,
) -> Result<Option<BehaviorReplayDiff>, BehaviorReplayError> {
    if !numeric_tolerance.is_finite() || numeric_tolerance < 0.0 {
        return Err(BehaviorReplayError::Invalid(
            "behavior replay diff tolerance must be finite and non-negative".to_string(),
        ));
    }
    expected.validate()?;
    actual.validate()?;

    let expected_manifest = replay_manifest_value(expected);
    let actual_manifest = replay_manifest_value(actual);
    let mut fields = Vec::new();
    collect_json_diffs(
        "$",
        &expected_manifest,
        &actual_manifest,
        numeric_tolerance,
        &mut fields,
    );
    if !fields.is_empty() {
        return Ok(Some(BehaviorReplayDiff {
            first_divergent_step: None,
            expected_state_digest: None,
            actual_state_digest: None,
            fields,
        }));
    }

    let shared_frames = expected.frames.len().min(actual.frames.len());
    for index in 0..shared_frames {
        let expected_frame = &expected.frames[index];
        let actual_frame = &actual.frames[index];
        let mut frame_fields = Vec::new();
        if expected_frame.step != actual_frame.step {
            frame_fields.push(field_diff(
                format!("$.frames[{index}].step"),
                Value::from(expected_frame.step),
                Value::from(actual_frame.step),
            ));
        }
        if expected_frame.sim_time_ticks != actual_frame.sim_time_ticks {
            frame_fields.push(field_diff(
                format!("$.frames[{index}].sim_time_ticks"),
                Value::from(expected_frame.sim_time_ticks),
                Value::from(actual_frame.sim_time_ticks),
            ));
        }
        if expected_frame.action != actual_frame.action {
            frame_fields.push(field_diff(
                format!("$.frames[{index}].action"),
                serde_json::to_value(expected_frame.action)?,
                serde_json::to_value(actual_frame.action)?,
            ));
        }
        if expected_frame.state_digest != actual_frame.state_digest {
            frame_fields.push(field_diff(
                format!("$.frames[{index}].state_digest"),
                Value::from(expected_frame.state_digest),
                Value::from(actual_frame.state_digest),
            ));
        }
        collect_json_diffs(
            &format!("$.frames[{index}].observation"),
            &expected_frame.observation,
            &actual_frame.observation,
            numeric_tolerance,
            &mut frame_fields,
        );
        if !frame_fields.is_empty() {
            frame_fields.truncate(32);
            return Ok(Some(BehaviorReplayDiff {
                first_divergent_step: Some(expected_frame.step.min(actual_frame.step)),
                expected_state_digest: Some(expected_frame.state_digest),
                actual_state_digest: Some(actual_frame.state_digest),
                fields: frame_fields,
            }));
        }
    }
    if expected.frames.len() != actual.frames.len() {
        return Ok(Some(BehaviorReplayDiff {
            first_divergent_step: Some(shared_frames as u64),
            expected_state_digest: expected
                .frames
                .get(shared_frames)
                .map(|frame| frame.state_digest),
            actual_state_digest: actual
                .frames
                .get(shared_frames)
                .map(|frame| frame.state_digest),
            fields: vec![field_diff(
                "$.frames.length".to_string(),
                Value::from(expected.frames.len()),
                Value::from(actual.frames.len()),
            )],
        }));
    }

    let expected_failure = serde_json::to_value(&expected.failure)?;
    let actual_failure = serde_json::to_value(&actual.failure)?;
    collect_json_diffs(
        "$.failure",
        &expected_failure,
        &actual_failure,
        numeric_tolerance,
        &mut fields,
    );
    if fields.is_empty() {
        Ok(None)
    } else {
        fields.truncate(32);
        Ok(Some(BehaviorReplayDiff {
            first_divergent_step: Some(expected.failure.violation.step),
            expected_state_digest: Some(expected.failure.violation.state_digest),
            actual_state_digest: Some(actual.failure.violation.state_digest),
            fields,
        }))
    }
}

/// Re-executes a behavior replay and verifies the same frames and first violation.
pub fn verify_behavior_replay<S, E>(
    expected: &BehaviorReplayArtifact,
    mut factory: impl FnMut(u64, &[BehaviorDimension]) -> Result<S, E>,
) -> Result<BehaviorReplayVerification, BehaviorReplayError>
where
    S: BehaviorScenario,
    S::Observation: Serialize,
    E: fmt::Display,
{
    expected.validate_compatibility()?;
    let run =
        run_behavior_scenarios_with_replays(expected.scenario.clone(), [expected.seed], |seed| {
            factory(seed, &expected.dimensions)
        })?;
    if let Some(error) = run
        .report
        .seeds
        .first()
        .and_then(|seed| seed.setup_error.as_ref())
    {
        return Err(BehaviorReplayError::Invalid(format!(
            "behavior replay setup failed: {error}"
        )));
    }
    let actual = run.failure_replays.into_iter().next().ok_or_else(|| {
        BehaviorReplayError::ExpectedFailureDidNotRecur {
            contract: expected.failure.contract.name.clone(),
        }
    })?;
    if let Some(diff) =
        diff_behavior_replays(expected, &actual, expected.observation_numeric_tolerance)?
    {
        return Err(BehaviorReplayError::Diverged(diff));
    }
    Ok(BehaviorReplayVerification {
        seed: expected.seed,
        contract: expected.failure.contract.name.clone(),
        step: expected.failure.violation.step,
        matched_frames: expected.frames.len(),
        state_digest: expected.failure.violation.state_digest,
    })
}

/// Minimizes dimensions in stable name order while preserving the failed contract.
pub fn minimize_behavior_failure<E>(
    original: &BehaviorReplayArtifact,
    mut reproduce: impl FnMut(&[BehaviorDimension]) -> Result<Option<BehaviorReplayArtifact>, E>,
) -> Result<BehaviorMinimizationResult, BehaviorReplayError>
where
    E: fmt::Display,
{
    original.validate_compatibility()?;
    let active_dimensions_before = original
        .dimensions
        .iter()
        .filter(|dimension| dimension.is_active())
        .count();
    let mut dimensions = original.dimensions.clone();
    let mut best = original.clone();
    let mut attempts = 0_u32;

    for index in 0..dimensions.len() {
        if !dimensions[index].is_active() {
            continue;
        }
        let original_value = dimensions[index].value.clone();
        let baseline = dimensions[index].baseline.clone();
        let mut candidate = dimensions.clone();
        candidate[index].value = baseline.clone();
        attempts = attempts.saturating_add(1);
        if let Some(replay) =
            reproduce_candidate(&candidate, &original.failure.contract.name, &mut reproduce)?
        {
            dimensions = candidate;
            best = replay;
            continue;
        }

        let (
            BehaviorDimensionValue::Number(mut failing),
            BehaviorDimensionValue::Number(mut passing),
        ) = (original_value, baseline)
        else {
            continue;
        };
        for _ in 0..BEHAVIOR_MINIMIZATION_BISECTION_STEPS {
            let midpoint = passing + (failing - passing) * 0.5;
            if midpoint == passing || midpoint == failing {
                break;
            }
            let mut numeric_candidate = dimensions.clone();
            numeric_candidate[index].value = BehaviorDimensionValue::Number(midpoint);
            attempts = attempts.saturating_add(1);
            if let Some(replay) = reproduce_candidate(
                &numeric_candidate,
                &original.failure.contract.name,
                &mut reproduce,
            )? {
                failing = midpoint;
                dimensions = numeric_candidate;
                best = replay;
            } else {
                passing = midpoint;
            }
        }
    }

    let active_dimensions_after = best
        .dimensions
        .iter()
        .filter(|dimension| dimension.is_active())
        .count();
    best.minimization = Some(BehaviorMinimizationMetadata {
        source_state_digest: original.failure.violation.state_digest,
        attempts,
        active_dimensions_before,
        active_dimensions_after,
    });
    best.validate()?;
    Ok(BehaviorMinimizationResult {
        attempts,
        active_dimensions_before,
        active_dimensions_after,
        artifact: best,
    })
}

/// Computes a stable FNV-1a digest for behavior metadata and semantic state.
pub fn stable_behavior_digest(bytes: &[u8]) -> u64 {
    let mut digest = 0xcbf29ce484222325_u64;
    for byte in bytes {
        digest ^= u64::from(*byte);
        digest = digest.wrapping_mul(0x100000001b3);
    }
    digest
}

fn validate_dimensions(dimensions: &[BehaviorDimension]) -> Result<(), BehaviorReplayError> {
    let mut previous_name: Option<&str> = None;
    for dimension in dimensions {
        dimension.validate()?;
        if previous_name.is_some_and(|previous| previous >= dimension.name.as_str()) {
            return Err(BehaviorReplayError::Invalid(
                "behavior dimensions must be sorted by unique name".to_string(),
            ));
        }
        previous_name = Some(&dimension.name);
    }
    Ok(())
}

fn validate_contracts(contracts: &[BehaviorContractDescriptor]) -> Result<(), BehaviorReplayError> {
    if contracts.is_empty() {
        return Err(BehaviorReplayError::Invalid(
            "behavior replay contract manifest must not be empty".to_string(),
        ));
    }
    let mut names = BTreeSet::new();
    for contract in contracts {
        if contract.name.trim().is_empty() {
            return Err(BehaviorReplayError::Invalid(
                "behavior replay contract name must not be empty".to_string(),
            ));
        }
        if !names.insert(contract.name.as_str()) {
            return Err(BehaviorReplayError::Invalid(format!(
                "duplicate behavior replay contract `{}`",
                contract.name
            )));
        }
        if contract
            .entities
            .iter()
            .any(|entity| entity.trim().is_empty())
        {
            return Err(BehaviorReplayError::Invalid(format!(
                "behavior replay contract `{}` has an empty entity",
                contract.name
            )));
        }
        if matches!(
            contract.kind,
            crate::behavior::BehaviorContractKind::Consecutive { steps: 0 }
        ) {
            return Err(BehaviorReplayError::Invalid(format!(
                "behavior replay contract `{}` has zero consecutive steps",
                contract.name
            )));
        }
    }
    Ok(())
}

fn digest_serializable(value: &impl Serialize) -> Result<u64, BehaviorReplayError> {
    Ok(stable_behavior_digest(&serde_json::to_vec(value)?))
}

fn sanitize_file_component(value: &str) -> String {
    let mut sanitized = value
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric() || matches!(character, '-' | '_') {
                character
            } else {
                '-'
            }
        })
        .collect::<String>();
    while sanitized.contains("--") {
        sanitized = sanitized.replace("--", "-");
    }
    let sanitized = sanitized.trim_matches('-');
    if sanitized.is_empty() {
        "behavior".to_string()
    } else {
        sanitized.to_string()
    }
}

fn replay_manifest_value(artifact: &BehaviorReplayArtifact) -> Value {
    serde_json::json!({
        "schema_version": artifact.schema_version,
        "kind": artifact.kind,
        "engine_version": artifact.engine_version,
        "contract_schema_version": artifact.contract_schema_version,
        "contract_digest": artifact.contract_digest,
        "scenario": artifact.scenario,
        "scenario_digest": artifact.scenario_digest,
        "seed": artifact.seed,
        "fixed_delta_ticks": artifact.fixed_delta_ticks,
        "observation_numeric_tolerance": artifact.observation_numeric_tolerance,
        "dimensions": artifact.dimensions,
        "contracts": artifact.contracts,
    })
}

fn collect_json_diffs(
    path: &str,
    expected: &Value,
    actual: &Value,
    tolerance: f64,
    fields: &mut Vec<BehaviorFieldDiff>,
) {
    if fields.len() >= 32 {
        return;
    }
    match (expected, actual) {
        (Value::Object(expected), Value::Object(actual)) => {
            let keys = expected
                .keys()
                .chain(actual.keys())
                .collect::<BTreeSet<_>>();
            for key in keys {
                let field_path = format!("{path}.{key}");
                match (expected.get(key), actual.get(key)) {
                    (Some(expected), Some(actual)) => {
                        collect_json_diffs(&field_path, expected, actual, tolerance, fields)
                    }
                    (Some(expected), None) => {
                        fields.push(field_diff(field_path, expected.clone(), missing_value()))
                    }
                    (None, Some(actual)) => {
                        fields.push(field_diff(field_path, missing_value(), actual.clone()))
                    }
                    (None, None) => unreachable!("key came from at least one object"),
                }
                if fields.len() >= 32 {
                    break;
                }
            }
        }
        (Value::Array(expected), Value::Array(actual)) => {
            let shared = expected.len().min(actual.len());
            for index in 0..shared {
                collect_json_diffs(
                    &format!("{path}[{index}]"),
                    &expected[index],
                    &actual[index],
                    tolerance,
                    fields,
                );
                if fields.len() >= 32 {
                    return;
                }
            }
            if expected.len() != actual.len() {
                fields.push(field_diff(
                    format!("{path}.length"),
                    Value::from(expected.len()),
                    Value::from(actual.len()),
                ));
            }
        }
        (Value::Number(expected), Value::Number(actual)) => {
            let equal = if expected == actual {
                true
            } else if (expected.is_i64() || expected.is_u64())
                && (actual.is_i64() || actual.is_u64())
            {
                false
            } else {
                match (expected.as_f64(), actual.as_f64()) {
                    (Some(expected), Some(actual)) => (expected - actual).abs() <= tolerance,
                    _ => false,
                }
            };
            if !equal {
                fields.push(field_diff(
                    path.to_string(),
                    Value::Number(expected.clone()),
                    Value::Number(actual.clone()),
                ));
            }
        }
        _ if expected == actual => {}
        _ => fields.push(field_diff(
            path.to_string(),
            expected.clone(),
            actual.clone(),
        )),
    }
}

fn field_diff(path: String, expected: Value, actual: Value) -> BehaviorFieldDiff {
    let absolute_delta = expected
        .as_f64()
        .zip(actual.as_f64())
        .map(|(expected, actual)| (expected - actual).abs());
    BehaviorFieldDiff {
        path,
        expected,
        actual,
        absolute_delta,
    }
}

fn missing_value() -> Value {
    Value::String("<missing>".to_string())
}

fn reproduce_candidate<E>(
    dimensions: &[BehaviorDimension],
    expected_contract: &str,
    reproduce: &mut impl FnMut(&[BehaviorDimension]) -> Result<Option<BehaviorReplayArtifact>, E>,
) -> Result<Option<BehaviorReplayArtifact>, BehaviorReplayError>
where
    E: fmt::Display,
{
    let Some(replay) = reproduce(dimensions).map_err(|error| {
        BehaviorReplayError::Invalid(format!("minimizer setup failed: {error}"))
    })?
    else {
        return Ok(None);
    };
    replay.validate_compatibility()?;
    if replay.dimensions != dimensions {
        return Err(BehaviorReplayError::Invalid(
            "minimizer reproduction changed the requested dimensions".to_string(),
        ));
    }
    Ok((replay.failure.contract.name == expected_contract).then_some(replay))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::behavior::BehaviorContractKind;

    fn descriptor() -> BehaviorContractDescriptor {
        BehaviorContractDescriptor {
            name: "must_hold".to_string(),
            kind: BehaviorContractKind::Always,
            entities: vec!["payload".to_string()],
        }
    }

    fn sample_artifact(value: f64) -> BehaviorReplayArtifact {
        let contract = descriptor();
        let violation = BehaviorViolation {
            step: 1,
            sim_time_ticks: 10,
            state_digest: value.to_bits(),
            entities: vec!["payload".to_string()],
            message: "predicate was false".to_string(),
        };
        BehaviorReplayArtifact::new(
            "sample",
            stable_behavior_digest(b"sample scenario"),
            7,
            10,
            vec![BehaviorDimension::number("offset_m", value, 0.0).expect("dimension")],
            vec![contract.clone()],
            vec![
                BehaviorReplayFrame {
                    step: 0,
                    sim_time_ticks: 0,
                    action: BehaviorReplayAction::InitialObservation,
                    observation: serde_json::json!({"value": 1.0}),
                    state_digest: 1,
                },
                BehaviorReplayFrame {
                    step: 1,
                    sim_time_ticks: 10,
                    action: BehaviorReplayAction::Advance,
                    observation: serde_json::json!({"value": value}),
                    state_digest: value.to_bits(),
                },
            ],
            BehaviorReplayFailure {
                contract,
                violation,
            },
        )
        .expect("artifact")
    }

    #[test]
    fn artifact_round_trip_rejects_schema_and_engine_mismatch() {
        let artifact = sample_artifact(2.0);
        let json = artifact.to_json_pretty().expect("JSON");
        assert_eq!(BehaviorReplayArtifact::from_json(&json).unwrap(), artifact);

        let mut unsupported = artifact.clone();
        unsupported.schema_version += 1;
        assert!(matches!(
            unsupported.validate(),
            Err(BehaviorReplayError::UnsupportedVersion { .. })
        ));

        let mut incompatible = artifact;
        incompatible.engine_version = "0.0.0".to_string();
        assert!(matches!(
            incompatible.validate_compatibility(),
            Err(BehaviorReplayError::IncompatibleEngineVersion { .. })
        ));

        let mut incompatible = sample_artifact(2.0);
        incompatible.observation_numeric_tolerance = 1.0e-6;
        assert!(matches!(
            incompatible.validate_compatibility(),
            Err(BehaviorReplayError::IncompatibleFloatTolerance { .. })
        ));
    }

    #[test]
    fn replay_diff_reports_first_named_field() {
        let expected = sample_artifact(2.0);
        let actual = sample_artifact(2.25);
        let diff = diff_behavior_replays(&expected, &actual, 0.0)
            .expect("diff")
            .expect("divergence");
        assert_eq!(diff.first_divergent_step, None);
        assert_eq!(diff.fields[0].path, "$.dimensions[0].value");

        let mut observation_only = expected.clone();
        observation_only.frames[1].observation = serde_json::json!({"value": 2.0001});
        observation_only.frames[1].state_digest = expected.frames[1].state_digest;
        observation_only.failure.violation.state_digest = expected.frames[1].state_digest;
        let diff = diff_behavior_replays(&expected, &observation_only, 0.0)
            .expect("diff")
            .expect("divergence");
        assert_eq!(diff.first_divergent_step, Some(1));
        assert_eq!(diff.fields[0].path, "$.frames[1].observation.value");
        assert!(diff_behavior_replays(&expected, &observation_only, 0.001)
            .expect("diff")
            .is_none());
    }

    #[test]
    fn minimizer_bisects_numeric_dimension_deterministically() {
        let original = sample_artifact(2.0);
        let minimized = minimize_behavior_failure(&original, |dimensions| {
            let BehaviorDimensionValue::Number(value) = dimensions[0].value else {
                unreachable!()
            };
            Ok::<_, &str>((value >= 1.0).then(|| sample_artifact(value)))
        })
        .expect("minimize");
        assert_eq!(minimized.attempts, 17);
        assert_eq!(minimized.active_dimensions_before, 1);
        assert_eq!(minimized.active_dimensions_after, 1);
        let BehaviorDimensionValue::Number(value) = minimized.artifact.dimensions[0].value else {
            unreachable!()
        };
        assert!((1.0..1.000_1).contains(&value), "{value}");
        assert_eq!(
            minimized.artifact.failure.contract.name,
            original.failure.contract.name
        );
        assert!(minimized.artifact.minimization.is_some());
    }

    #[test]
    fn failure_case_round_trip_is_stable() {
        let artifact = sample_artifact(2.0);
        let case = BehaviorFailureCase::from_replay(&artifact);
        let loaded = BehaviorFailureCase::from_json(&case.to_json_pretty().unwrap()).unwrap();
        assert_eq!(loaded, case);
        assert_eq!(loaded.expected_contract, "must_hold");
    }

    #[test]
    fn seed_manifest_is_sorted_unique_and_validated() {
        let manifest = BehaviorSeedManifest::new("sample", [9, 2, 9, 4]);
        assert_eq!(manifest.seeds, vec![2, 4, 9]);
        manifest.validate().expect("manifest");

        let mut invalid = manifest;
        invalid.seeds = vec![2, 2];
        assert!(invalid.validate().is_err());
    }

    #[test]
    fn json_diff_distinguishes_missing_values_and_large_integers() {
        let mut fields = Vec::new();
        collect_json_diffs(
            "$",
            &serde_json::json!({"literal": "<missing>"}),
            &serde_json::json!({}),
            0.0,
            &mut fields,
        );
        assert_eq!(fields.len(), 1);
        assert_eq!(fields[0].path, "$.literal");

        fields.clear();
        collect_json_diffs(
            "$.digest",
            &Value::from(u64::MAX),
            &Value::from(u64::MAX - 1),
            f64::MAX,
            &mut fields,
        );
        assert_eq!(fields.len(), 1);
    }

    #[test]
    fn already_minimal_failure_allows_zero_attempts() {
        let original = sample_artifact(0.0);
        original.validate().expect("source artifact");
        let minimized = minimize_behavior_failure::<&str>(&original, |_| {
            unreachable!("no active dimension should be reproduced")
        })
        .expect("zero-attempt minimization");
        assert_eq!(minimized.attempts, 0);
        assert_eq!(
            minimized
                .artifact
                .minimization
                .as_ref()
                .expect("metadata")
                .attempts,
            0
        );
    }
}

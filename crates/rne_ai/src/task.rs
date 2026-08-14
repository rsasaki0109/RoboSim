//! Versioned, framework-neutral reinforcement-learning task contracts.

use serde::{Deserialize, Serialize};
use std::collections::{BTreeMap, BTreeSet};

/// Stable artifact kind written by [`TaskSpec`] v1.
pub const TASK_SPEC_KIND: &str = "rne_task_spec";

/// Task-specification schema version supported by this crate.
pub const TASK_SPEC_SCHEMA_VERSION: u32 = 1;

const SPLITMIX64_GAMMA: u64 = 0x9e37_79b9_7f4a_7c15;
const LANE_SEED_DOMAIN: u64 = 0x524e_452d_4c41_4e45;
const EPISODE_SEED_DOMAIN: u64 = 0x524e_452d_4550_4953;

/// Derives the v1 seed for one stable batch lane and its lane-local episode.
///
/// The result depends only on the three arguments, so changing batch width or
/// resetting another lane cannot alter a lane's episode-seed sequence. A
/// single environment uses lane ID zero.
pub fn derive_episode_seed(root_seed: u64, lane_id: u64, episode_index: u64) -> u64 {
    let lane = splitmix64(lane_id ^ LANE_SEED_DOMAIN);
    let episode = splitmix64(episode_index ^ EPISODE_SEED_DOMAIN);
    splitmix64(root_seed ^ lane ^ episode)
}

fn splitmix64(value: u64) -> u64 {
    let mut mixed = value.wrapping_add(SPLITMIX64_GAMMA);
    mixed = (mixed ^ (mixed >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
    mixed = (mixed ^ (mixed >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
    mixed ^ (mixed >> 31)
}

/// Scalar element type used by a portable tensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(rename_all = "snake_case")]
pub enum TensorDType {
    /// IEEE-754 binary32.
    F32,
    /// IEEE-754 binary64.
    F64,
    /// Signed 32-bit integer.
    I32,
    /// Signed 64-bit integer.
    I64,
    /// Unsigned 8-bit integer.
    U8,
    /// Boolean value.
    Bool,
}

/// Logical element ordering of a portable tensor.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(rename_all = "snake_case")]
pub enum TensorLayout {
    /// Last dimension is contiguous, matching NumPy C order and Rust row-major arrays.
    RowMajor,
}

/// Optional numeric bounds for every flattened tensor element.
///
/// Each side contains either one broadcast value or one value per element in
/// row-major order.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct TensorBounds {
    /// Inclusive lower bound values.
    pub lower: Vec<f64>,
    /// Inclusive upper bound values.
    pub upper: Vec<f64>,
}

impl TensorBounds {
    /// Creates numeric tensor bounds.
    pub fn new(lower: Vec<f64>, upper: Vec<f64>) -> Self {
        Self { lower, upper }
    }

    /// Creates one lower and upper bound broadcast to every element.
    pub fn broadcast(lower: f64, upper: f64) -> Self {
        Self::new(vec![lower], vec![upper])
    }
}

/// One named fixed-shape tensor in an observation or action space.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct TensorSpec {
    /// Stable machine-readable tensor name.
    pub name: String,
    /// Scalar element type.
    pub dtype: TensorDType,
    /// Tensor dimensions. An empty shape represents one scalar.
    pub shape: Vec<usize>,
    /// Unit symbol, or `"1"` for a dimensionless value.
    pub unit: String,
    /// Logical element ordering.
    pub layout: TensorLayout,
    /// Optional numeric bounds.
    pub bounds: Option<TensorBounds>,
}

impl TensorSpec {
    /// Creates an unbounded row-major tensor specification.
    pub fn new(
        name: impl Into<String>,
        dtype: TensorDType,
        shape: Vec<usize>,
        unit: impl Into<String>,
    ) -> Self {
        Self {
            name: name.into(),
            dtype,
            shape,
            unit: unit.into(),
            layout: TensorLayout::RowMajor,
            bounds: None,
        }
    }

    /// Adds inclusive numeric bounds.
    pub fn with_bounds(mut self, bounds: TensorBounds) -> Self {
        self.bounds = Some(bounds);
        self
    }
}

/// Ordered observation tensors returned after reset and each task step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct ObservationSpec {
    /// Tensors in stable consumer-visible order.
    pub tensors: Vec<TensorSpec>,
}

impl ObservationSpec {
    /// Creates an ordered observation space.
    pub fn new(tensors: Vec<TensorSpec>) -> Self {
        Self { tensors }
    }
}

/// Ordered action tensors accepted by a task step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct ActionSpec {
    /// Tensors in stable consumer-visible order.
    pub tensors: Vec<TensorSpec>,
}

impl ActionSpec {
    /// Creates an ordered action space.
    pub fn new(tensors: Vec<TensorSpec>) -> Self {
        Self { tensors }
    }
}

/// How declared reward terms form the scalar task reward.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(rename_all = "snake_case")]
pub enum RewardAggregation {
    /// Sum each term after multiplying it by its declared weight.
    WeightedSum,
}

/// One ordered component of a scalar reward.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct RewardTermSpec {
    /// Stable machine-readable term name.
    pub name: String,
    /// Multiplier applied before aggregation.
    pub weight: f64,
    /// Unit of the raw term, or `"1"` when dimensionless.
    pub unit: String,
}

impl RewardTermSpec {
    /// Creates a weighted reward term.
    pub fn new(name: impl Into<String>, weight: f64, unit: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            weight,
            unit: unit.into(),
        }
    }
}

/// Ordered scalar reward contract.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct RewardSpec {
    /// Aggregation applied to the ordered terms.
    pub aggregation: RewardAggregation,
    /// Reward terms in stable evaluation and reporting order.
    pub terms: Vec<RewardTermSpec>,
}

impl RewardSpec {
    /// Creates a weighted-sum reward contract.
    pub fn weighted_sum(terms: Vec<RewardTermSpec>) -> Self {
        Self {
            aggregation: RewardAggregation::WeightedSum,
            terms,
        }
    }
}

/// Semantic result produced by a terminal condition.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(rename_all = "snake_case")]
pub enum TerminationKind {
    /// The task objective was completed.
    Success,
    /// The episode ended because of a task failure.
    Failure,
}

/// One named condition that ends an episode without truncating it.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct TerminationConditionSpec {
    /// Stable machine-readable condition name.
    pub name: String,
    /// Semantic result of satisfying the condition.
    pub kind: TerminationKind,
}

impl TerminationConditionSpec {
    /// Creates a terminal condition.
    pub fn new(name: impl Into<String>, kind: TerminationKind) -> Self {
        Self {
            name: name.into(),
            kind,
        }
    }
}

/// Episode termination and step-budget contract.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct TerminationSpec {
    /// Conditions that produce `terminated = true`, in evaluation order.
    pub conditions: Vec<TerminationConditionSpec>,
    /// Optional action-step budget that produces `truncated = true`.
    pub max_episode_steps: Option<u64>,
}

impl TerminationSpec {
    /// Creates a termination contract.
    pub fn new(conditions: Vec<TerminationConditionSpec>, max_episode_steps: Option<u64>) -> Self {
        Self {
            conditions,
            max_episode_steps,
        }
    }
}

/// Deterministic seed derivation used by reset streams.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(rename_all = "snake_case")]
pub enum EpisodeSeedStrategy {
    /// SplitMix64 derivation from root seed, stable lane ID, and lane-local episode index.
    SplitMix64LaneEpisodeV1,
}

/// Reset behavior promised by the task runtime.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct ResetSpec {
    /// Algorithm mapping a root seed and lane episode identity to an episode seed.
    pub seed_strategy: EpisodeSeedStrategy,
    /// Whether selected lanes can reset without advancing any other lane.
    pub supports_partial_reset: bool,
}

impl ResetSpec {
    /// Creates a reset contract using the v1 lane/episode seed derivation.
    pub fn splitmix64(supports_partial_reset: bool) -> Self {
        Self {
            seed_strategy: EpisodeSeedStrategy::SplitMix64LaneEpisodeV1,
            supports_partial_reset,
        }
    }
}

/// Unit-bearing scalar curriculum parameter.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct TaskParameterSpec {
    /// Parameter value.
    pub value: f64,
    /// Unit symbol, or `"1"` when dimensionless.
    pub unit: String,
}

impl TaskParameterSpec {
    /// Creates a unit-bearing scalar parameter.
    pub fn new(value: f64, unit: impl Into<String>) -> Self {
        Self {
            value,
            unit: unit.into(),
        }
    }
}

/// One deterministic curriculum stage.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct CurriculumStageSpec {
    /// Stable machine-readable stage name.
    pub name: String,
    /// Lane-local episode index at which the stage becomes active.
    pub starts_at_episode: u64,
    /// Task-owned scalar parameter values in canonical key order.
    pub parameters: BTreeMap<String, TaskParameterSpec>,
}

impl CurriculumStageSpec {
    /// Creates a curriculum stage.
    pub fn new(
        name: impl Into<String>,
        starts_at_episode: u64,
        parameters: BTreeMap<String, TaskParameterSpec>,
    ) -> Self {
        Self {
            name: name.into(),
            starts_at_episode,
            parameters,
        }
    }
}

/// Ordered episode-index curriculum.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct CurriculumSpec {
    /// Stages ordered by strictly increasing `starts_at_episode`.
    pub stages: Vec<CurriculumStageSpec>,
}

impl CurriculumSpec {
    /// Creates an episode-index curriculum.
    pub fn new(stages: Vec<CurriculumStageSpec>) -> Self {
        Self { stages }
    }
}

/// Distribution sampled from a deterministic task randomization stream.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum RandomDistributionSpec {
    /// A fixed scalar value that still appears in randomization evidence.
    Constant {
        /// Fixed value.
        value: f64,
    },
    /// Uniform distribution including both declared endpoints.
    Uniform {
        /// Inclusive minimum.
        minimum: f64,
        /// Inclusive maximum.
        maximum: f64,
    },
    /// Normal distribution with a positive standard deviation.
    Normal {
        /// Distribution mean.
        mean: f64,
        /// Positive standard deviation.
        standard_deviation: f64,
    },
}

/// One named and unit-bearing randomization stream.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct RandomizationParameterSpec {
    /// Stable machine-readable stream name.
    pub name: String,
    /// Unit symbol, or `"1"` when dimensionless.
    pub unit: String,
    /// Distribution sampled once per lane-local episode reset.
    pub distribution: RandomDistributionSpec,
}

impl RandomizationParameterSpec {
    /// Creates a randomization stream declaration.
    pub fn new(
        name: impl Into<String>,
        unit: impl Into<String>,
        distribution: RandomDistributionSpec,
    ) -> Self {
        Self {
            name: name.into(),
            unit: unit.into(),
            distribution,
        }
    }
}

/// Ordered deterministic domain-randomization contract.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct RandomizationSpec {
    /// Streams in stable sampling and evidence order.
    pub parameters: Vec<RandomizationParameterSpec>,
}

impl RandomizationSpec {
    /// Creates an ordered randomization contract.
    pub fn new(parameters: Vec<RandomizationParameterSpec>) -> Self {
        Self { parameters }
    }
}

/// Portable task identity and its complete consumer-visible data contract.
///
/// Array order is the order serialized in each observation, action, reward,
/// termination, curriculum, and randomization list. Framework-specific types
/// are intentionally absent. Call [`Self::validate`] after deserializing an
/// untrusted artifact.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct TaskSpec {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Schema version for compatibility checks.
    pub schema_version: u32,
    /// Stable machine-readable task identity.
    pub task_id: String,
    /// Simulation-time duration represented by one action step.
    pub control_step_s: f64,
    /// Ordered observation contract.
    pub observation: ObservationSpec,
    /// Ordered action contract.
    pub action: ActionSpec,
    /// Scalar reward contract.
    pub reward: RewardSpec,
    /// Termination and truncation contract.
    pub termination: TerminationSpec,
    /// Deterministic reset contract.
    pub reset: ResetSpec,
    /// Optional lane-local curriculum.
    pub curriculum: Option<CurriculumSpec>,
    /// Optional per-episode domain randomization.
    pub randomization: Option<RandomizationSpec>,
}

impl TaskSpec {
    /// Creates a v1 portable task without curriculum or randomization.
    pub fn new(
        task_id: impl Into<String>,
        control_step_s: f64,
        observation: ObservationSpec,
        action: ActionSpec,
        reward: RewardSpec,
        termination: TerminationSpec,
        reset: ResetSpec,
    ) -> Self {
        Self {
            kind: TASK_SPEC_KIND.to_string(),
            schema_version: TASK_SPEC_SCHEMA_VERSION,
            task_id: task_id.into(),
            control_step_s,
            observation,
            action,
            reward,
            termination,
            reset,
            curriculum: None,
            randomization: None,
        }
    }

    /// Adds a deterministic lane-local curriculum.
    pub fn with_curriculum(mut self, curriculum: CurriculumSpec) -> Self {
        self.curriculum = Some(curriculum);
        self
    }

    /// Adds deterministic per-episode domain randomization.
    pub fn with_randomization(mut self, randomization: RandomizationSpec) -> Self {
        self.randomization = Some(randomization);
        self
    }

    /// Validates version, identifiers, shapes, units, bounds, and ordering invariants.
    ///
    /// # Errors
    ///
    /// Returns a field-addressed error for the first invalid contract value.
    pub fn validate(&self) -> Result<(), TaskSpecValidationError> {
        if self.kind != TASK_SPEC_KIND {
            return Err(TaskSpecValidationError::InvalidKind {
                expected: TASK_SPEC_KIND,
                actual: self.kind.clone(),
            });
        }
        if self.schema_version != TASK_SPEC_SCHEMA_VERSION {
            return Err(TaskSpecValidationError::UnsupportedSchemaVersion {
                expected: TASK_SPEC_SCHEMA_VERSION,
                actual: self.schema_version,
            });
        }
        validate_identifier("task_id", &self.task_id)?;
        if !self.control_step_s.is_finite() || self.control_step_s <= 0.0 {
            return invalid("control_step_s", "must be finite and greater than zero");
        }
        validate_tensors("observation.tensors", &self.observation.tensors)?;
        validate_tensors("action.tensors", &self.action.tensors)?;
        validate_reward(&self.reward)?;
        validate_termination(&self.termination)?;
        if let Some(curriculum) = &self.curriculum {
            validate_curriculum(curriculum)?;
        }
        if let Some(randomization) = &self.randomization {
            validate_randomization(randomization)?;
        }
        Ok(())
    }
}

/// Failure validating a portable task contract.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum TaskSpecValidationError {
    /// The artifact discriminator does not identify an RNE task specification.
    #[error("task kind must be {expected:?}, got {actual:?}")]
    InvalidKind {
        /// Required artifact discriminator.
        expected: &'static str,
        /// Artifact discriminator that was read.
        actual: String,
    },
    /// The artifact uses a schema version not supported by this loader.
    #[error("task schema version must be {expected}, got {actual}")]
    UnsupportedSchemaVersion {
        /// Schema version supported by this loader.
        expected: u32,
        /// Schema version that was read.
        actual: u32,
    },
    /// One field violates a v1 structural or numeric invariant.
    #[error("invalid task field {field}: {reason}")]
    InvalidField {
        /// Dot-and-index path to the invalid field.
        field: String,
        /// Human-readable invariant violation.
        reason: String,
    },
}

fn invalid<T>(
    field: impl Into<String>,
    reason: impl Into<String>,
) -> Result<T, TaskSpecValidationError> {
    Err(TaskSpecValidationError::InvalidField {
        field: field.into(),
        reason: reason.into(),
    })
}

fn validate_identifier(field: &str, value: &str) -> Result<(), TaskSpecValidationError> {
    if value.is_empty()
        || !value.bytes().all(|byte| {
            byte.is_ascii_lowercase() || byte.is_ascii_digit() || b"._-/".contains(&byte)
        })
    {
        return invalid(
            field,
            "must use only lowercase ASCII letters, digits, '.', '_', '-', or '/'",
        );
    }
    Ok(())
}

fn validate_unit(field: &str, value: &str) -> Result<(), TaskSpecValidationError> {
    if value.is_empty() || value.bytes().any(|byte| byte.is_ascii_whitespace()) {
        return invalid(field, "must be a non-empty unit symbol without whitespace");
    }
    Ok(())
}

fn validate_tensors(field: &str, tensors: &[TensorSpec]) -> Result<(), TaskSpecValidationError> {
    if tensors.is_empty() {
        return invalid(field, "must contain at least one tensor");
    }
    let mut names = BTreeSet::new();
    for (index, tensor) in tensors.iter().enumerate() {
        let path = format!("{field}[{index}]");
        validate_identifier(&format!("{path}.name"), &tensor.name)?;
        if !names.insert(tensor.name.as_str()) {
            return invalid(format!("{path}.name"), "must be unique within its space");
        }
        validate_unit(&format!("{path}.unit"), &tensor.unit)?;
        if tensor.shape.contains(&0) {
            return invalid(
                format!("{path}.shape"),
                "dimensions must be greater than zero",
            );
        }
        let Some(elements) = tensor
            .shape
            .iter()
            .try_fold(1_usize, |total, dimension| total.checked_mul(*dimension))
        else {
            return invalid(format!("{path}.shape"), "element count overflows usize");
        };
        if let Some(bounds) = &tensor.bounds {
            if tensor.dtype == TensorDType::Bool {
                return invalid(
                    format!("{path}.bounds"),
                    "boolean tensors cannot have numeric bounds",
                );
            }
            validate_bound_side(&format!("{path}.bounds.lower"), &bounds.lower, elements)?;
            validate_bound_side(&format!("{path}.bounds.upper"), &bounds.upper, elements)?;
            for element in 0..elements {
                let lower = bounds.lower[if bounds.lower.len() == 1 { 0 } else { element }];
                let upper = bounds.upper[if bounds.upper.len() == 1 { 0 } else { element }];
                if lower > upper {
                    return invalid(
                        format!("{path}.bounds[{element}]"),
                        "lower bound must not exceed upper bound",
                    );
                }
            }
        }
    }
    Ok(())
}

fn validate_bound_side(
    field: &str,
    values: &[f64],
    elements: usize,
) -> Result<(), TaskSpecValidationError> {
    if values.len() != 1 && values.len() != elements {
        return invalid(
            field,
            format!("must contain one broadcast value or {elements} flattened values"),
        );
    }
    if values.iter().any(|value| !value.is_finite()) {
        return invalid(field, "values must be finite");
    }
    Ok(())
}

fn validate_reward(reward: &RewardSpec) -> Result<(), TaskSpecValidationError> {
    if reward.terms.is_empty() {
        return invalid("reward.terms", "must contain at least one term");
    }
    let mut names = BTreeSet::new();
    for (index, term) in reward.terms.iter().enumerate() {
        let path = format!("reward.terms[{index}]");
        validate_identifier(&format!("{path}.name"), &term.name)?;
        if !names.insert(term.name.as_str()) {
            return invalid(format!("{path}.name"), "must be unique");
        }
        if !term.weight.is_finite() {
            return invalid(format!("{path}.weight"), "must be finite");
        }
        validate_unit(&format!("{path}.unit"), &term.unit)?;
    }
    Ok(())
}

fn validate_termination(termination: &TerminationSpec) -> Result<(), TaskSpecValidationError> {
    if termination.conditions.is_empty() {
        return invalid(
            "termination.conditions",
            "must contain at least one condition",
        );
    }
    let mut names = BTreeSet::new();
    for (index, condition) in termination.conditions.iter().enumerate() {
        let path = format!("termination.conditions[{index}].name");
        validate_identifier(&path, &condition.name)?;
        if !names.insert(condition.name.as_str()) {
            return invalid(path, "must be unique");
        }
    }
    if termination.max_episode_steps == Some(0) {
        return invalid(
            "termination.max_episode_steps",
            "must be greater than zero when present",
        );
    }
    Ok(())
}

fn validate_curriculum(curriculum: &CurriculumSpec) -> Result<(), TaskSpecValidationError> {
    if curriculum.stages.is_empty() {
        return invalid("curriculum.stages", "must contain at least one stage");
    }
    if curriculum.stages[0].starts_at_episode != 0 {
        return invalid(
            "curriculum.stages[0].starts_at_episode",
            "the first stage must start at episode zero",
        );
    }
    let mut names = BTreeSet::new();
    for (index, stage) in curriculum.stages.iter().enumerate() {
        let path = format!("curriculum.stages[{index}]");
        validate_identifier(&format!("{path}.name"), &stage.name)?;
        if !names.insert(stage.name.as_str()) {
            return invalid(format!("{path}.name"), "must be unique");
        }
        if index > 0 && stage.starts_at_episode <= curriculum.stages[index - 1].starts_at_episode {
            return invalid(
                format!("{path}.starts_at_episode"),
                "must be strictly greater than the preceding stage",
            );
        }
        if stage.parameters.is_empty() {
            return invalid(format!("{path}.parameters"), "must not be empty");
        }
        for (name, parameter) in &stage.parameters {
            validate_identifier(&format!("{path}.parameters.{name}"), name)?;
            if !parameter.value.is_finite() {
                return invalid(format!("{path}.parameters.{name}.value"), "must be finite");
            }
            validate_unit(&format!("{path}.parameters.{name}.unit"), &parameter.unit)?;
        }
    }
    Ok(())
}

fn validate_randomization(
    randomization: &RandomizationSpec,
) -> Result<(), TaskSpecValidationError> {
    if randomization.parameters.is_empty() {
        return invalid(
            "randomization.parameters",
            "must contain at least one stream",
        );
    }
    let mut names = BTreeSet::new();
    for (index, parameter) in randomization.parameters.iter().enumerate() {
        let path = format!("randomization.parameters[{index}]");
        validate_identifier(&format!("{path}.name"), &parameter.name)?;
        if !names.insert(parameter.name.as_str()) {
            return invalid(format!("{path}.name"), "must be unique");
        }
        validate_unit(&format!("{path}.unit"), &parameter.unit)?;
        match parameter.distribution {
            RandomDistributionSpec::Constant { value } if !value.is_finite() => {
                return invalid(format!("{path}.distribution.value"), "must be finite");
            }
            RandomDistributionSpec::Uniform { minimum, maximum }
                if !minimum.is_finite() || !maximum.is_finite() || minimum > maximum =>
            {
                return invalid(
                    format!("{path}.distribution"),
                    "uniform bounds must be finite and ordered",
                );
            }
            RandomDistributionSpec::Normal {
                mean,
                standard_deviation,
            } if !mean.is_finite()
                || !standard_deviation.is_finite()
                || standard_deviation <= 0.0 =>
            {
                return invalid(
                    format!("{path}.distribution"),
                    "normal mean must be finite and standard deviation must be positive",
                );
            }
            _ => {}
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn reference_task() -> TaskSpec {
        let mut curriculum_parameters = BTreeMap::new();
        curriculum_parameters.insert(
            "goal_distance_m".to_string(),
            TaskParameterSpec::new(1.0, "m"),
        );
        TaskSpec::new(
            "rne.diff_drive.goal.v1",
            0.02,
            ObservationSpec::new(vec![
                TensorSpec::new("base_position_m", TensorDType::F64, vec![2], "m"),
                TensorSpec::new("base_yaw_rad", TensorDType::F64, vec![], "rad"),
                TensorSpec::new("target_position_m", TensorDType::F64, vec![2], "m"),
            ]),
            ActionSpec::new(vec![TensorSpec::new(
                "wheel_velocity_rad_s",
                TensorDType::F64,
                vec![2],
                "rad/s",
            )
            .with_bounds(TensorBounds::broadcast(-10.0, 10.0))]),
            RewardSpec::weighted_sum(vec![
                RewardTermSpec::new("goal_progress", 1.0, "1"),
                RewardTermSpec::new("action_effort", -0.01, "1"),
            ]),
            TerminationSpec::new(
                vec![
                    TerminationConditionSpec::new("goal_reached", TerminationKind::Success),
                    TerminationConditionSpec::new("out_of_bounds", TerminationKind::Failure),
                ],
                Some(500),
            ),
            ResetSpec::splitmix64(true),
        )
        .with_curriculum(CurriculumSpec::new(vec![CurriculumStageSpec::new(
            "baseline",
            0,
            curriculum_parameters,
        )]))
        .with_randomization(RandomizationSpec::new(vec![
            RandomizationParameterSpec::new(
                "initial_yaw_rad",
                "rad",
                RandomDistributionSpec::Uniform {
                    minimum: -std::f64::consts::PI,
                    maximum: std::f64::consts::PI,
                },
            ),
        ]))
    }

    #[test]
    fn task_spec_v1_matches_committed_golden() {
        let task = reference_task();
        task.validate().expect("reference task must validate");
        let json = serde_json::to_string_pretty(&task).expect("serialize task spec");
        let golden = include_str!("../../../tests/golden/tasks/task-spec-v1.json");
        assert_eq!(json, golden.trim_end());
        let decoded: TaskSpec = serde_json::from_str(golden).expect("parse golden task spec");
        decoded.validate().expect("golden task must validate");
        assert_eq!(decoded, task);
    }

    #[test]
    fn invalid_shape_and_bound_cardinality_are_rejected() {
        let mut task = reference_task();
        task.observation.tensors[0].shape = vec![0];
        assert!(matches!(
            task.validate(),
            Err(TaskSpecValidationError::InvalidField { field, .. })
                if field == "observation.tensors[0].shape"
        ));

        let mut task = reference_task();
        task.action.tensors[0].bounds = Some(TensorBounds::new(vec![-1.0, -2.0, -3.0], vec![1.0]));
        assert!(matches!(
            task.validate(),
            Err(TaskSpecValidationError::InvalidField { field, .. })
                if field == "action.tensors[0].bounds.lower"
        ));
    }

    #[test]
    fn batch_sensitive_curriculum_and_invalid_randomization_are_rejected() {
        let mut task = reference_task();
        task.curriculum.as_mut().unwrap().stages[0].starts_at_episode = 1;
        assert!(task.validate().is_err());

        let mut task = reference_task();
        task.randomization.as_mut().unwrap().parameters[0].distribution =
            RandomDistributionSpec::Normal {
                mean: 0.0,
                standard_deviation: 0.0,
            };
        assert!(task.validate().is_err());
    }

    #[test]
    fn unknown_fields_and_future_schema_versions_are_rejected() {
        let mut value = serde_json::to_value(reference_task()).expect("serialize task");
        value["unknown"] = serde_json::json!(true);
        assert!(serde_json::from_value::<TaskSpec>(value).is_err());

        let mut task = reference_task();
        task.schema_version += 1;
        assert_eq!(
            task.validate().unwrap_err(),
            TaskSpecValidationError::UnsupportedSchemaVersion {
                expected: TASK_SPEC_SCHEMA_VERSION,
                actual: TASK_SPEC_SCHEMA_VERSION + 1,
            }
        );
    }

    #[test]
    fn episode_seed_is_independent_of_batch_width_and_other_lanes() {
        let lane_zero = (0..4)
            .map(|episode| derive_episode_seed(42, 0, episode))
            .collect::<Vec<_>>();
        assert_eq!(
            lane_zero,
            vec![
                1_298_720_818_104_676_741,
                6_147_948_423_359_611_076,
                17_925_233_603_215_598_159,
                2_375_635_680_555_833_453,
            ]
        );
        let wider_batch_lane_zero = (0..4)
            .map(|episode| derive_episode_seed(42, 0, episode))
            .collect::<Vec<_>>();
        assert_eq!(lane_zero, wider_batch_lane_zero);
        assert_ne!(lane_zero[0], derive_episode_seed(42, 1, 0));
        assert_ne!(lane_zero[1], derive_episode_seed(42, 0, 0));
    }
}

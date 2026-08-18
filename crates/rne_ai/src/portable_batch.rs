//! TaskSpec-bound deterministic CPU reference batching for reinforcement-learning episodes.

use crate::{
    derive_episode_seed, episode::Episode, EpisodeSeedStrategy, TaskSpec, TaskSpecValidationError,
};
use serde::{Deserialize, Serialize};

/// Replay-checkpoint schema emitted by [`PortableBatchRunner`].
pub const PORTABLE_BATCH_CHECKPOINT_VERSION: u32 = 2;

const FNV_OFFSET_BASIS: u64 = 0xcbf2_9ce4_8422_2325;
const FNV_PRIME: u64 = 0x0000_0100_0000_01b3;

/// Configuration shared by a vectorized episode batch.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PortableBatchConfig {
    /// Number of episode instances in the batch.
    pub num_envs: usize,
    /// Root seed supplied to seeded episode factories.
    pub seed: u64,
    /// Whether a completed episode resets during its next batch step.
    pub auto_reset: bool,
}

impl Default for PortableBatchConfig {
    fn default() -> Self {
        Self {
            num_envs: 1,
            seed: 1,
            auto_reset: true,
        }
    }
}

/// Result of resetting or stepping a deterministic episode batch.
#[derive(Clone, Debug, PartialEq)]
pub struct PortableBatchStep<O> {
    /// Stable lane IDs in the same order as every other field.
    pub lane_ids: Vec<u64>,
    /// Lane-local episode index associated with each result.
    pub episode_indices: Vec<u64>,
    /// Derived episode seed, or `None` for caller-managed episodes.
    pub episode_seeds: Vec<Option<u64>>,
    /// True for results produced by reset rather than action application.
    pub resets: Vec<bool>,
    /// Observations in stable lane-ID order.
    pub observations: Vec<O>,
    /// Rewards in stable lane-ID order.
    pub rewards: Vec<f64>,
    /// Termination flags in stable lane-ID order.
    pub terminated: Vec<bool>,
    /// Truncation flags in stable lane-ID order.
    pub truncated: Vec<bool>,
}

impl<O> PortableBatchStep<O> {
    /// Returns true when every returned lane ended during this batch step.
    pub fn all_done(&self) -> bool {
        self.terminated
            .iter()
            .zip(&self.truncated)
            .all(|(terminated, truncated)| *terminated || *truncated)
    }

    /// Returns the number of returned lanes that reported termination.
    pub fn success_count(&self) -> usize {
        self.terminated.iter().filter(|value| **value).count()
    }
}

/// One replayable operation after the implicit initial full reset.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(tag = "type", rename_all = "snake_case", deny_unknown_fields)]
pub enum PortableBatchOperation<A> {
    /// Apply one action per batch lane.
    Step {
        /// Actions in stable lane-ID order.
        actions: Vec<A>,
    },
    /// Explicitly reset selected lanes in strictly increasing ID order.
    ResetLanes {
        /// Stable lane IDs to reset.
        lane_ids: Vec<u64>,
    },
}

/// Replay state recorded for one stable batch lane.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[non_exhaustive]
#[serde(deny_unknown_fields)]
pub struct PortableBatchLaneCheckpoint {
    /// Stable zero-based lane identity.
    pub lane_id: u64,
    /// Lane-local episode index.
    pub episode_index: u64,
    /// Derived episode seed, or `None` for caller-managed episodes.
    pub episode_seed: Option<u64>,
    /// Whether the next batch step auto-resets this lane.
    pub pending_auto_reset: bool,
    /// Stable same-build digest of this lane's replay history.
    pub replay_digest: u64,
}

/// Error validating, restoring, or partially resetting a deterministic batch.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
#[non_exhaustive]
pub enum PortableBatchError {
    /// The checkpoint schema is not supported by this engine.
    #[error("vectorized checkpoint schema must be {expected}, got {actual}")]
    UnsupportedSchemaVersion {
        /// Schema version supported by this engine.
        expected: u32,
        /// Schema version stored in the checkpoint.
        actual: u32,
    },
    /// The checkpoint was created with a different number of environments.
    #[error("vectorized checkpoint has {actual} lanes; expected {expected}")]
    EnvCountMismatch {
        /// Number of environments in the current batch.
        expected: usize,
        /// Number of environments recorded by the checkpoint.
        actual: usize,
    },
    /// Seeded and caller-managed construction modes do not match.
    #[error("vectorized checkpoint seeded mode is {actual}; expected {expected}")]
    SeedModeMismatch {
        /// Whether the current runner uses the v1 seeded factory.
        expected: bool,
        /// Whether the checkpoint uses the v1 seeded factory.
        actual: bool,
    },
    /// The checkpoint and runner do not describe the same portable task.
    #[error("vectorized checkpoint TaskSpec does not match the runner")]
    TaskSpecMismatch,
    /// The portable task contract forbids partial lane reset.
    #[error("TaskSpec does not declare partial-reset support")]
    PartialResetUnsupported,
    /// A partial-reset request contains no lanes.
    #[error("partial reset requires at least one lane ID")]
    EmptyLaneSelection,
    /// A checkpoint, step, or partial reset was requested before full reset.
    #[error("vectorized batch must receive a full reset first")]
    NotReset,
    /// Lane IDs are not strictly increasing, unique, and in range.
    #[error("invalid lane ID {lane_id} at position {position} for {num_envs} lanes")]
    InvalidLaneId {
        /// Position in the supplied lane-ID list.
        position: usize,
        /// Invalid or non-canonical lane ID.
        lane_id: u64,
        /// Number of lanes in the batch.
        num_envs: usize,
    },
    /// A replay action batch has the wrong number of actions.
    #[error("operation {operation} has {actual} actions; expected {expected}")]
    ActionBatchMismatch {
        /// Zero-based operation containing the malformed batch.
        operation: usize,
        /// Number of actions required by the current batch.
        expected: usize,
        /// Number of actions supplied by the checkpoint.
        actual: usize,
    },
    /// Replaying the checkpoint did not reproduce one lane's metadata or digest.
    #[error("checkpoint replay diverged in lane {lane_id}")]
    LaneStateMismatch {
        /// Stable lane ID that diverged.
        lane_id: u64,
    },
    /// Replaying the checkpoint did not reproduce its aggregate digest.
    #[error("checkpoint replay digest is {actual:#018x}; expected {expected:#018x}")]
    ReplayDigestMismatch {
        /// Digest stored in the checkpoint.
        expected: u64,
        /// Digest produced by the restored replay.
        actual: u64,
    },
}

/// Versioned replay checkpoint for a deterministic vectorized episode batch.
///
/// A full reset is implicit. Subsequent steps and partial resets are stored in
/// chronological order, so restoration also reproduces lane-local episode
/// indices, episode seeds, pending auto-reset state, and random streams owned by
/// a deterministically constructed episode.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PortableBatchCheckpoint<A> {
    /// Checkpoint schema version.
    pub schema_version: u32,
    /// Root seed associated with the vectorized batch.
    pub seed: u64,
    /// Number of episode instances in the original batch.
    pub num_envs: usize,
    /// Whether completed episodes reset on the next batch step.
    pub auto_reset: bool,
    /// Seed derivation used by a seeded factory, or `None` for caller-managed episodes.
    pub seed_strategy: Option<EpisodeSeedStrategy>,
    /// Portable task contract bound to the runner, when supplied.
    pub task_spec: Option<TaskSpec>,
    /// Per-lane state in stable lane-ID order.
    pub lanes: Vec<PortableBatchLaneCheckpoint>,
    /// Operations after the implicit initial full reset.
    pub operations: Vec<PortableBatchOperation<A>>,
    /// Stable same-build digest combining all lane digests in lane-ID order.
    pub replay_digest: u64,
}

type EpisodeFactory<E> = Box<dyn Fn(u64) -> E + Send + Sync + 'static>;

/// Generic deterministic CPU reference runner around an [`Episode`] implementation.
///
/// [`Self::from_seeded`] is the portable v1 path. It reconstructs a lane from
/// `derive_episode_seed(root_seed, lane_id, episode_index)` on every reset.
/// [`Self::from_episodes`] remains available for existing episode types whose
/// reset stream is caller-managed; those lanes report no derived episode seed.
pub struct PortableBatchRunner<E>
where
    E: Episode,
{
    episodes: Vec<E>,
    factory: Option<EpisodeFactory<E>>,
    task_spec: Option<TaskSpec>,
    seed: u64,
    auto_reset: bool,
    has_reset: bool,
    episode_indices: Vec<u64>,
    episode_seeds: Vec<Option<u64>>,
    pending_auto_reset: Vec<bool>,
    operations: Vec<PortableBatchOperation<E::Action>>,
    lane_digests: Vec<u64>,
    replay_digest: u64,
}

impl<E> PortableBatchRunner<E>
where
    E: Episode,
    E::Action: Clone + Serialize,
    E::Observation: Serialize,
{
    /// Creates a caller-managed batch from already-constructed episode instances.
    ///
    /// The root seed is retained as replay metadata, but the runner cannot
    /// reconstruct these episodes with a derived lane/episode seed. Prefer
    /// [`Self::from_seeded`] for portable tasks.
    ///
    /// # Panics
    ///
    /// Panics when `episodes` is empty or its length differs from `config.num_envs`.
    pub fn from_episodes(episodes: Vec<E>, config: PortableBatchConfig) -> Self {
        assert!(config.num_envs > 0, "num_envs must be positive");
        assert_eq!(
            episodes.len(),
            config.num_envs,
            "episode count must match num_envs"
        );
        Self::new(episodes, None, None, config)
    }

    /// Creates a portable batch from a deterministic episode factory.
    ///
    /// Lane `i` and lane-local episode `j` are always reconstructed with
    /// `derive_episode_seed(config.seed, i, j)`. The factory must not read
    /// wall-clock time or other process-global state.
    ///
    /// # Panics
    ///
    /// Panics when `config.num_envs` is zero.
    pub fn from_seeded<F>(config: PortableBatchConfig, factory: F) -> Self
    where
        F: Fn(u64) -> E + Send + Sync + 'static,
    {
        assert!(config.num_envs > 0, "num_envs must be positive");
        let factory: EpisodeFactory<E> = Box::new(factory);
        let episodes = (0..config.num_envs)
            .map(|index| factory(derive_episode_seed(config.seed, index as u64, 0)))
            .collect();
        Self::new(episodes, Some(factory), None, config)
    }

    /// Creates a seeded runner bound to a validated portable task contract.
    ///
    /// The full [`TaskSpec`] is retained in checkpoints so a replay cannot be
    /// restored under a task with different shape, dtype, unit, ordering,
    /// reward, termination, or reset semantics.
    ///
    /// # Errors
    ///
    /// Returns the first [`TaskSpecValidationError`] in `task_spec`.
    pub fn from_task_spec<F>(
        task_spec: TaskSpec,
        config: PortableBatchConfig,
        factory: F,
    ) -> Result<Self, TaskSpecValidationError>
    where
        F: Fn(u64) -> E + Send + Sync + 'static,
    {
        task_spec.validate()?;
        let mut runner = Self::from_seeded(config, factory);
        runner.task_spec = Some(task_spec);
        Ok(runner)
    }

    fn new(
        episodes: Vec<E>,
        factory: Option<EpisodeFactory<E>>,
        task_spec: Option<TaskSpec>,
        config: PortableBatchConfig,
    ) -> Self {
        let lane_digests = (0..config.num_envs)
            .map(|index| initial_lane_digest(config.seed, index as u64))
            .collect::<Vec<_>>();
        let replay_digest = aggregate_digest(config.seed, &lane_digests);
        Self {
            episodes,
            factory,
            task_spec,
            seed: config.seed,
            auto_reset: config.auto_reset,
            has_reset: false,
            episode_indices: vec![0; config.num_envs],
            episode_seeds: vec![None; config.num_envs],
            pending_auto_reset: vec![false; config.num_envs],
            operations: Vec::new(),
            lane_digests,
            replay_digest,
        }
    }

    /// Returns the number of environments in the batch.
    pub fn num_envs(&self) -> usize {
        self.episodes.len()
    }

    /// Returns whether completed episodes reset on their next batch step.
    pub fn auto_reset(&self) -> bool {
        self.auto_reset
    }

    /// Returns the portable task contract bound to this runner, when supplied.
    pub fn task_spec(&self) -> Option<&TaskSpec> {
        self.task_spec.as_ref()
    }

    /// Returns the stable digest combining all lanes in lane-ID order.
    pub fn replay_digest(&self) -> u64 {
        self.replay_digest
    }

    /// Returns one lane's digest, independent of the runner's batch width.
    pub fn lane_replay_digest(&self, lane_id: u64) -> Option<u64> {
        usize::try_from(lane_id)
            .ok()
            .and_then(|index| self.lane_digests.get(index).copied())
    }

    /// Returns the lane-local episode index.
    pub fn lane_episode_index(&self, lane_id: u64) -> Option<u64> {
        usize::try_from(lane_id)
            .ok()
            .and_then(|index| self.episode_indices.get(index).copied())
    }

    /// Returns the current derived episode seed, or `None` for caller-managed lanes.
    pub fn lane_episode_seed(&self, lane_id: u64) -> Option<u64> {
        usize::try_from(lane_id)
            .ok()
            .and_then(|index| self.episode_seeds.get(index).copied().flatten())
    }

    /// Resets every lane to episode index zero and starts a new replay history.
    pub fn reset(&mut self) -> PortableBatchStep<E::Observation> {
        self.operations.clear();
        self.has_reset = true;
        self.episode_indices.fill(0);
        self.pending_auto_reset.fill(false);
        self.lane_digests = (0..self.episodes.len())
            .map(|index| initial_lane_digest(self.seed, index as u64))
            .collect();

        let mut results = Vec::with_capacity(self.episodes.len());
        for index in 0..self.episodes.len() {
            let result = self.reset_lane(index, false);
            results.push((index, result, true));
        }
        self.update_aggregate_digest();
        self.collect_step(results)
    }

    /// Resets selected lanes without advancing any other lane.
    ///
    /// Lane IDs must be unique and strictly increasing. Results contain only
    /// the selected lanes, in the supplied canonical order.
    ///
    /// # Errors
    ///
    /// Returns an error before mutation if full reset has not occurred or a
    /// lane ID is duplicated, out of order, or out of range.
    pub fn reset_lanes(
        &mut self,
        lane_ids: &[u64],
    ) -> Result<PortableBatchStep<E::Observation>, PortableBatchError> {
        if !self.has_reset {
            return Err(PortableBatchError::NotReset);
        }
        if self
            .task_spec
            .as_ref()
            .is_some_and(|spec| !spec.reset.supports_partial_reset)
        {
            return Err(PortableBatchError::PartialResetUnsupported);
        }
        let indices = validate_lane_ids(lane_ids, self.episodes.len())?;
        let mut results = Vec::with_capacity(indices.len());
        for index in indices {
            let result = self.reset_lane(index, true);
            results.push((index, result, true));
        }
        self.operations.push(PortableBatchOperation::ResetLanes {
            lane_ids: lane_ids.to_vec(),
        });
        self.update_aggregate_digest();
        Ok(self.collect_step(results))
    }

    /// Steps every lane with the corresponding action.
    ///
    /// A lane that ended on the previous call resets here when auto-reset is
    /// enabled; its corresponding action is ignored and `resets` is true. The
    /// terminal result is therefore never overwritten by a reset observation.
    ///
    /// # Panics
    ///
    /// Panics if called before [`Self::reset`] or when the action count does
    /// not match [`Self::num_envs`].
    pub fn step(&mut self, actions: &[E::Action]) -> PortableBatchStep<E::Observation> {
        assert!(self.has_reset, "reset must be called before step");
        assert_eq!(
            actions.len(),
            self.episodes.len(),
            "action batch size must match num_envs"
        );

        let mut results = Vec::with_capacity(self.episodes.len());
        for (index, action) in actions.iter().enumerate() {
            if self.auto_reset && self.pending_auto_reset[index] {
                let result = self.reset_lane(index, true);
                results.push((index, result, true));
            } else {
                let result = self.episodes[index].step(action.clone());
                self.pending_auto_reset[index] = result.terminated || result.truncated;
                absorb_marker(&mut self.lane_digests[index], "step");
                absorb_serialized(&mut self.lane_digests[index], action);
                absorb_episode_step(&mut self.lane_digests[index], &result);
                results.push((index, result, false));
            }
        }
        self.operations.push(PortableBatchOperation::Step {
            actions: actions.to_vec(),
        });
        self.update_aggregate_digest();
        self.collect_step(results)
    }

    /// Returns an immutable reference to one episode.
    pub fn episode(&self, index: usize) -> &E {
        &self.episodes[index]
    }

    /// Returns a replay checkpoint for the current operation history.
    ///
    /// # Errors
    ///
    /// Returns [`PortableBatchError::NotReset`] before full reset.
    pub fn checkpoint(&self) -> Result<PortableBatchCheckpoint<E::Action>, PortableBatchError> {
        if !self.has_reset {
            return Err(PortableBatchError::NotReset);
        }
        Ok(PortableBatchCheckpoint {
            schema_version: PORTABLE_BATCH_CHECKPOINT_VERSION,
            seed: self.seed,
            num_envs: self.episodes.len(),
            auto_reset: self.auto_reset,
            seed_strategy: self
                .factory
                .as_ref()
                .map(|_| EpisodeSeedStrategy::SplitMix64LaneEpisodeV1),
            task_spec: self.task_spec.clone(),
            lanes: self.lane_checkpoints(),
            operations: self.operations.clone(),
            replay_digest: self.replay_digest,
        })
    }

    /// Restores a checkpoint through full reset and chronological replay.
    ///
    /// # Errors
    ///
    /// Returns an error when schema, lane count, seed mode, operation shape,
    /// lane state, or deterministic digest does not match.
    pub fn restore_checkpoint(
        &mut self,
        checkpoint: &PortableBatchCheckpoint<E::Action>,
    ) -> Result<(), PortableBatchError> {
        if checkpoint.schema_version != PORTABLE_BATCH_CHECKPOINT_VERSION {
            return Err(PortableBatchError::UnsupportedSchemaVersion {
                expected: PORTABLE_BATCH_CHECKPOINT_VERSION,
                actual: checkpoint.schema_version,
            });
        }
        if checkpoint.num_envs != self.episodes.len() {
            return Err(PortableBatchError::EnvCountMismatch {
                expected: self.episodes.len(),
                actual: checkpoint.num_envs,
            });
        }
        if checkpoint.lanes.len() != self.episodes.len() {
            return Err(PortableBatchError::EnvCountMismatch {
                expected: self.episodes.len(),
                actual: checkpoint.lanes.len(),
            });
        }
        let checkpoint_seeded = checkpoint.seed_strategy.is_some();
        let runner_seeded = self.factory.is_some();
        if checkpoint_seeded != runner_seeded {
            return Err(PortableBatchError::SeedModeMismatch {
                expected: runner_seeded,
                actual: checkpoint_seeded,
            });
        }
        if checkpoint.task_spec != self.task_spec {
            return Err(PortableBatchError::TaskSpecMismatch);
        }
        for (index, lane) in checkpoint.lanes.iter().enumerate() {
            if lane.lane_id != index as u64 {
                return Err(PortableBatchError::InvalidLaneId {
                    position: index,
                    lane_id: lane.lane_id,
                    num_envs: self.episodes.len(),
                });
            }
        }

        self.seed = checkpoint.seed;
        self.auto_reset = checkpoint.auto_reset;
        self.reset();
        for (operation_index, operation) in checkpoint.operations.iter().enumerate() {
            match operation {
                PortableBatchOperation::Step { actions } => {
                    if actions.len() != self.episodes.len() {
                        return Err(PortableBatchError::ActionBatchMismatch {
                            operation: operation_index,
                            expected: self.episodes.len(),
                            actual: actions.len(),
                        });
                    }
                    self.step(actions);
                }
                PortableBatchOperation::ResetLanes { lane_ids } => {
                    self.reset_lanes(lane_ids)?;
                }
            }
        }
        for (expected, actual) in checkpoint.lanes.iter().zip(self.lane_checkpoints()) {
            if expected != &actual {
                return Err(PortableBatchError::LaneStateMismatch {
                    lane_id: expected.lane_id,
                });
            }
        }
        if self.replay_digest != checkpoint.replay_digest {
            return Err(PortableBatchError::ReplayDigestMismatch {
                expected: checkpoint.replay_digest,
                actual: self.replay_digest,
            });
        }
        Ok(())
    }

    fn reset_lane(
        &mut self,
        index: usize,
        advance_episode: bool,
    ) -> crate::episode::EpisodeStep<E::Observation> {
        if advance_episode {
            self.episode_indices[index] = self.episode_indices[index]
                .checked_add(1)
                .expect("lane-local episode index exhausted");
        }
        let episode_index = self.episode_indices[index];
        let episode_seed = self.factory.as_ref().map(|factory| {
            let seed = derive_episode_seed(self.seed, index as u64, episode_index);
            self.episodes[index] = factory(seed);
            seed
        });
        self.episode_seeds[index] = episode_seed;
        self.pending_auto_reset[index] = false;
        let result = self.episodes[index].reset();
        absorb_marker(&mut self.lane_digests[index], "reset");
        absorb_serialized(&mut self.lane_digests[index], &episode_index);
        absorb_serialized(&mut self.lane_digests[index], &episode_seed);
        absorb_episode_step(&mut self.lane_digests[index], &result);
        result
    }

    fn collect_step(
        &self,
        results: Vec<(usize, crate::episode::EpisodeStep<E::Observation>, bool)>,
    ) -> PortableBatchStep<E::Observation> {
        let mut step = PortableBatchStep {
            lane_ids: Vec::with_capacity(results.len()),
            episode_indices: Vec::with_capacity(results.len()),
            episode_seeds: Vec::with_capacity(results.len()),
            resets: Vec::with_capacity(results.len()),
            observations: Vec::with_capacity(results.len()),
            rewards: Vec::with_capacity(results.len()),
            terminated: Vec::with_capacity(results.len()),
            truncated: Vec::with_capacity(results.len()),
        };
        for (index, result, reset) in results {
            step.lane_ids.push(index as u64);
            step.episode_indices.push(self.episode_indices[index]);
            step.episode_seeds.push(self.episode_seeds[index]);
            step.resets.push(reset);
            step.observations.push(result.observation);
            step.rewards.push(result.reward);
            step.terminated.push(result.terminated);
            step.truncated.push(result.truncated);
        }
        step
    }

    fn lane_checkpoints(&self) -> Vec<PortableBatchLaneCheckpoint> {
        (0..self.episodes.len())
            .map(|index| PortableBatchLaneCheckpoint {
                lane_id: index as u64,
                episode_index: self.episode_indices[index],
                episode_seed: self.episode_seeds[index],
                pending_auto_reset: self.pending_auto_reset[index],
                replay_digest: self.lane_digests[index],
            })
            .collect()
    }

    fn update_aggregate_digest(&mut self) {
        self.replay_digest = aggregate_digest(self.seed, &self.lane_digests);
    }
}

fn validate_lane_ids(lane_ids: &[u64], num_envs: usize) -> Result<Vec<usize>, PortableBatchError> {
    if lane_ids.is_empty() {
        return Err(PortableBatchError::EmptyLaneSelection);
    }
    let mut indices = Vec::with_capacity(lane_ids.len());
    let mut previous = None;
    for (position, lane_id) in lane_ids.iter().copied().enumerate() {
        let index = usize::try_from(lane_id).ok();
        if index.is_none_or(|index| index >= num_envs)
            || previous.is_some_and(|previous| lane_id <= previous)
        {
            return Err(PortableBatchError::InvalidLaneId {
                position,
                lane_id,
                num_envs,
            });
        }
        indices.push(index.expect("validated lane ID"));
        previous = Some(lane_id);
    }
    Ok(indices)
}

fn initial_lane_digest(seed: u64, lane_id: u64) -> u64 {
    let mut digest = FNV_OFFSET_BASIS;
    absorb_marker(&mut digest, "rne_vectorized_lane_v2");
    absorb_serialized(&mut digest, &seed);
    absorb_serialized(&mut digest, &lane_id);
    digest
}

fn aggregate_digest(seed: u64, lane_digests: &[u64]) -> u64 {
    let mut digest = FNV_OFFSET_BASIS;
    absorb_marker(&mut digest, "rne_vectorized_batch_v2");
    absorb_serialized(&mut digest, &seed);
    for (lane_id, lane_digest) in lane_digests.iter().enumerate() {
        absorb_serialized(&mut digest, &(lane_id as u64));
        absorb_serialized(&mut digest, lane_digest);
    }
    digest
}

fn absorb_episode_step<O: Serialize>(digest: &mut u64, step: &crate::episode::EpisodeStep<O>) {
    absorb_serialized(digest, &step.observation);
    absorb_serialized(digest, &step.reward);
    absorb_serialized(digest, &step.terminated);
    absorb_serialized(digest, &step.truncated);
}

fn absorb_marker(digest: &mut u64, marker: &str) {
    absorb_bytes(digest, marker.as_bytes());
}

fn absorb_serialized<T: Serialize>(digest: &mut u64, value: &T) {
    let bytes = serde_json::to_vec(value).expect("task value must serialize to JSON");
    absorb_bytes(digest, &bytes);
}

fn absorb_bytes(digest: &mut u64, bytes: &[u8]) {
    for byte in bytes {
        *digest ^= u64::from(*byte);
        *digest = digest.wrapping_mul(FNV_PRIME);
    }
    *digest ^= 0xff;
    *digest = digest.wrapping_mul(FNV_PRIME);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        ActionSpec, Episode, EpisodeStep, ObservationSpec, ResetSpec, RewardSpec, RewardTermSpec,
        TensorDType, TensorSpec, TerminationConditionSpec, TerminationKind, TerminationSpec,
    };

    #[derive(Clone, Debug)]
    struct ToyEpisode {
        seed: u64,
        value: i64,
        step: u32,
    }

    impl ToyEpisode {
        fn new(seed: u64) -> Self {
            Self {
                seed,
                value: 0,
                step: 0,
            }
        }
    }

    impl Episode for ToyEpisode {
        type Observation = (u64, i64);
        type Action = i64;

        fn reset(&mut self) -> EpisodeStep<Self::Observation> {
            self.value = (self.seed % 97) as i64;
            self.step = 0;
            EpisodeStep {
                observation: (self.seed, self.value),
                reward: 0.0,
                terminated: false,
                truncated: false,
            }
        }

        fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
            self.value += action;
            self.step += 1;
            EpisodeStep {
                observation: (self.seed, self.value),
                reward: self.value as f64,
                terminated: self.value % 11 == 0,
                truncated: self.step >= 3,
            }
        }

        fn episode_index(&self) -> u32 {
            0
        }

        fn step_in_episode(&self) -> u64 {
            u64::from(self.step)
        }
    }

    fn runner(num_envs: usize, auto_reset: bool) -> PortableBatchRunner<ToyEpisode> {
        PortableBatchRunner::from_seeded(
            PortableBatchConfig {
                num_envs,
                seed: 42,
                auto_reset,
            },
            ToyEpisode::new,
        )
    }

    fn task_spec(partial_reset: bool) -> TaskSpec {
        TaskSpec::new(
            "rne.test.toy.v1",
            0.01,
            ObservationSpec::new(vec![TensorSpec::new(
                "state",
                TensorDType::I64,
                vec![2],
                "1",
            )]),
            ActionSpec::new(vec![TensorSpec::new(
                "delta",
                TensorDType::I64,
                vec![],
                "1",
            )]),
            RewardSpec::weighted_sum(vec![RewardTermSpec::new("value", 1.0, "1")]),
            TerminationSpec::new(
                vec![TerminationConditionSpec::new(
                    "done",
                    TerminationKind::Success,
                )],
                Some(3),
            ),
            ResetSpec::splitmix64(partial_reset),
        )
    }

    #[test]
    fn batch_lane_zero_matches_single_runner_and_width_does_not_change_seed() {
        let mut single = runner(1, false);
        let mut batch = runner(4, false);
        let single_reset = single.reset();
        let batch_reset = batch.reset();
        assert_eq!(single_reset.observations[0], batch_reset.observations[0]);
        assert_eq!(single_reset.episode_seeds[0], batch_reset.episode_seeds[0]);
        assert_eq!(single.lane_replay_digest(0), batch.lane_replay_digest(0));

        let single_step = single.step(&[3]);
        let batch_step = batch.step(&[3, 4, 5, 6]);
        assert_eq!(single_step.observations[0], batch_step.observations[0]);
        assert_eq!(single_step.rewards[0], batch_step.rewards[0]);
        assert_eq!(single.lane_replay_digest(0), batch.lane_replay_digest(0));

        let lane_zero_digest = batch.lane_replay_digest(0);
        batch.reset_lanes(&[2]).expect("partial reset lane 2");
        assert_eq!(batch.lane_replay_digest(0), lane_zero_digest);
        assert_eq!(batch.lane_episode_index(2), Some(1));
        assert_eq!(
            batch.lane_episode_seed(2),
            Some(derive_episode_seed(42, 2, 1))
        );

        let single_lane_zero = single.reset_lanes(&[0]).expect("single reset lane 0");
        let batch_lane_zero = batch.reset_lanes(&[0]).expect("batch reset lane 0");
        assert_eq!(single_lane_zero.observations, batch_lane_zero.observations);
        assert_eq!(
            single_lane_zero.episode_seeds,
            batch_lane_zero.episode_seeds
        );
        assert_eq!(single.lane_replay_digest(0), batch.lane_replay_digest(0));
    }

    #[test]
    fn partial_reset_requires_canonical_in_range_lane_ids() {
        let mut batch = runner(3, false);
        assert_eq!(
            batch.reset_lanes(&[1]).unwrap_err(),
            PortableBatchError::NotReset
        );
        batch.reset();
        assert_eq!(
            batch.reset_lanes(&[]).unwrap_err(),
            PortableBatchError::EmptyLaneSelection
        );
        assert!(matches!(
            batch.reset_lanes(&[1, 1]),
            Err(PortableBatchError::InvalidLaneId {
                position: 1,
                lane_id: 1,
                ..
            })
        ));
        assert!(batch.reset_lanes(&[2, 1]).is_err());
        assert!(batch.reset_lanes(&[3]).is_err());
    }

    #[test]
    fn auto_reset_preserves_terminal_transition_until_the_next_step() {
        let mut batch = runner(1, true);
        batch.reset();
        let mut terminal = None;
        for _ in 0..3 {
            let step = batch.step(&[1]);
            if step.terminated[0] || step.truncated[0] {
                terminal = Some(step);
                break;
            }
        }
        let terminal = terminal.expect("toy episode must end within three steps");
        assert!(!terminal.resets[0]);
        let next = batch.step(&[999]);
        assert!(next.resets[0]);
        assert!(!next.terminated[0]);
        assert_eq!(next.episode_indices[0], 1);
        assert_eq!(next.episode_seeds[0], Some(derive_episode_seed(42, 0, 1)));
    }

    #[test]
    fn checkpoint_restores_partial_reset_auto_reset_and_lane_digests() {
        let mut batch = runner(2, true);
        batch.reset();
        batch.step(&[1, 2]);
        batch.reset_lanes(&[1]).expect("partial reset");
        batch.step(&[3, 4]);
        batch.step(&[5, 6]);
        batch.step(&[7, 8]);
        let checkpoint = batch.checkpoint().expect("checkpoint");
        let json = serde_json::to_string(&checkpoint).expect("serialize checkpoint");
        let decoded: PortableBatchCheckpoint<i64> =
            serde_json::from_str(&json).expect("decode checkpoint");
        assert_eq!(decoded, checkpoint);

        let expected_next = batch.step(&[9, 10]);
        let mut restored = runner(2, true);
        restored
            .restore_checkpoint(&checkpoint)
            .expect("restore checkpoint");
        assert_eq!(restored.checkpoint().unwrap(), checkpoint);
        assert_eq!(restored.step(&[9, 10]), expected_next);
    }

    #[test]
    fn task_bound_runner_checkpoints_identity_and_enforces_reset_contract() {
        let config = PortableBatchConfig {
            num_envs: 2,
            seed: 42,
            auto_reset: false,
        };
        let spec = task_spec(true);
        let mut runner = PortableBatchRunner::from_task_spec(spec.clone(), config, ToyEpisode::new)
            .expect("valid TaskSpec");
        assert_eq!(runner.task_spec(), Some(&spec));
        runner.reset();
        runner.reset_lanes(&[1]).expect("declared partial reset");
        assert_eq!(runner.checkpoint().unwrap().task_spec, Some(spec));

        let mut no_partial =
            PortableBatchRunner::from_task_spec(task_spec(false), config, ToyEpisode::new)
                .expect("valid TaskSpec");
        no_partial.reset();
        assert_eq!(
            no_partial.reset_lanes(&[0]).unwrap_err(),
            PortableBatchError::PartialResetUnsupported
        );
    }

    #[test]
    fn checkpoint_v2_schema_matches_committed_golden() {
        let golden = include_str!("../../../tests/golden/tasks/vectorized-checkpoint-v2.json");
        let checkpoint: PortableBatchCheckpoint<i64> =
            serde_json::from_str(golden).expect("parse checkpoint golden");
        assert_eq!(checkpoint.schema_version, PORTABLE_BATCH_CHECKPOINT_VERSION);
        assert_eq!(
            serde_json::to_string_pretty(&checkpoint).expect("serialize checkpoint golden"),
            golden.trim_end()
        );
    }
}

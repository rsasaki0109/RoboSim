//! Generic deterministic batching for reinforcement-learning episodes.

use crate::episode::Episode;
use serde::{Deserialize, Serialize};
use std::fmt::Debug;

/// Current schema version for generic vectorized episode replay checkpoints.
///
/// This is a distinct contract from [`crate::PORTABLE_BATCH_CHECKPOINT_VERSION`].
/// Generic vectorized checkpoints retain the original action-replay format,
/// while portable batch checkpoints additionally bind lane state and TaskSpec.
pub const VECTORIZED_EPISODE_CHECKPOINT_VERSION: u32 = 1;
const FNV_OFFSET_BASIS: u64 = 0xcbf29ce484222325;
const FNV_PRIME: u64 = 0x100000001b3;

/// Configuration shared by a vectorized episode batch.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct VectorizedEpisodeConfig {
    /// Number of episode instances in the batch.
    pub num_envs: usize,
    /// Root seed supplied to seeded episode factories.
    pub seed: u64,
    /// Whether a completed episode is reset during the next batch step.
    pub auto_reset: bool,
}

impl Default for VectorizedEpisodeConfig {
    fn default() -> Self {
        Self {
            num_envs: 1,
            seed: 1,
            auto_reset: true,
        }
    }
}

/// Result of resetting or stepping a vectorized episode batch.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedEpisodeStep<O> {
    /// Observations in stable environment-index order.
    pub observations: Vec<O>,
    /// Rewards in stable environment-index order.
    pub rewards: Vec<f64>,
    /// Success termination flags in stable environment-index order.
    pub terminated: Vec<bool>,
    /// Truncation flags in stable environment-index order.
    pub truncated: Vec<bool>,
}

impl<O> VectorizedEpisodeStep<O> {
    /// Returns true when every environment ended during this batch step.
    pub fn all_done(&self) -> bool {
        self.terminated
            .iter()
            .zip(&self.truncated)
            .all(|(terminated, truncated)| *terminated || *truncated)
    }

    /// Returns the number of environments that reported success.
    pub fn success_count(&self) -> usize {
        self.terminated.iter().filter(|value| **value).count()
    }
}

/// Error restoring a deterministic vectorized episode replay checkpoint.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum VectorizedEpisodeCheckpointError {
    /// The checkpoint schema is newer than this engine understands.
    UnsupportedSchemaVersion {
        /// Schema version supported by this engine.
        expected: u32,
        /// Schema version stored in the checkpoint.
        actual: u32,
    },
    /// The checkpoint was created with a different number of environments.
    EnvCountMismatch {
        /// Number of environments in the current batch.
        expected: usize,
        /// Number of environments recorded by the checkpoint.
        actual: usize,
    },
    /// A checkpoint was requested before the batch had been reset.
    NotReset,
    /// A replay action batch has the wrong number of actions.
    ActionBatchMismatch {
        /// Zero-based replay step containing the malformed batch.
        step: usize,
        /// Number of actions required by the current batch.
        expected: usize,
        /// Number of actions supplied by the checkpoint.
        actual: usize,
    },
    /// Replaying the checkpoint did not reproduce its digest.
    ReplayDigestMismatch {
        /// Digest stored in the checkpoint.
        expected: u64,
        /// Digest produced by the restored replay.
        actual: u64,
    },
}

/// In-memory replay checkpoint for a vectorized episode batch.
///
/// The checkpoint stores the action batches rather than backend-specific ECS
/// internals. Restoring reconstructs each episode from its reset state and
/// replays those actions, which keeps the API usable for both lightweight
/// episodes and URDF articulations. This is intentionally a replay checkpoint,
/// not a claim that every physics backend has a portable binary snapshot.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VectorizedEpisodeCheckpoint<A> {
    /// Checkpoint schema version.
    pub schema_version: u32,
    /// Root seed associated with the vectorized batch.
    pub seed: u64,
    /// Number of episode instances in the original batch.
    pub num_envs: usize,
    /// Whether completed episodes were automatically reset while recording.
    pub auto_reset: bool,
    /// Whether the original batch had received its initial reset.
    pub has_reset: bool,
    /// Action batches in chronological order.
    pub actions: Vec<Vec<A>>,
    /// Stable digest of reset observations and replay transitions.
    pub replay_digest: u64,
}

/// Generic deterministic batch wrapper around an [`Episode`] implementation.
pub struct VectorizedEpisode<E>
where
    E: Episode,
{
    episodes: Vec<E>,
    seed: u64,
    auto_reset: bool,
    has_reset: bool,
    action_history: Vec<Vec<E::Action>>,
    replay_digest: u64,
}

impl<E> VectorizedEpisode<E>
where
    E: Episode,
    E::Action: Clone + Debug,
    E::Observation: Debug,
{
    /// Creates a batch from already-constructed episode instances.
    ///
    /// The caller controls episode construction; `seed` is retained in the
    /// replay metadata and should match the factory seed used by the caller.
    ///
    /// # Panics
    ///
    /// Panics when `episodes` is empty or its length differs from `config.num_envs`.
    pub fn from_episodes(episodes: Vec<E>, config: VectorizedEpisodeConfig) -> Self {
        assert!(config.num_envs > 0, "num_envs must be positive");
        assert_eq!(
            episodes.len(),
            config.num_envs,
            "episode count must match num_envs"
        );
        Self {
            episodes,
            seed: config.seed,
            auto_reset: config.auto_reset,
            has_reset: false,
            action_history: Vec::new(),
            replay_digest: initial_digest(config.seed, config.num_envs),
        }
    }

    /// Creates a batch from a deterministic factory, offsetting the seed by
    /// the environment index.
    ///
    /// The factory is used for initial construction. Each episode's own
    /// [`Episode::reset`] implementation is used for subsequent resets, so
    /// episode-specific seed state should be retained by that implementation.
    ///
    /// # Panics
    ///
    /// Panics when `config.num_envs` is zero.
    pub fn from_seeded<F>(config: VectorizedEpisodeConfig, mut factory: F) -> Self
    where
        F: FnMut(u64) -> E,
    {
        assert!(config.num_envs > 0, "num_envs must be positive");
        let episodes = (0..config.num_envs)
            .map(|index| factory(config.seed.wrapping_add(index as u64)))
            .collect();
        Self::from_episodes(episodes, config)
    }

    /// Returns the number of environments in the batch.
    pub fn num_envs(&self) -> usize {
        self.episodes.len()
    }

    /// Returns whether completed episodes are automatically reset.
    pub fn auto_reset(&self) -> bool {
        self.auto_reset
    }

    /// Returns the stable digest of all observations and transitions seen so far.
    pub fn replay_digest(&self) -> u64 {
        self.replay_digest
    }

    /// Resets every episode and starts a new replay history.
    pub fn reset(&mut self) -> VectorizedEpisodeStep<E::Observation> {
        self.action_history.clear();
        self.has_reset = true;
        self.replay_digest = initial_digest(self.seed, self.episodes.len());

        let step = collect_step(self.episodes.iter_mut().map(Episode::reset));
        self.absorb_step(None, &step);
        step
    }

    /// Steps every episode with the corresponding action.
    ///
    /// # Panics
    ///
    /// Panics if called before [`Self::reset`] or when the action count does
    /// not match [`Self::num_envs`].
    pub fn step(&mut self, actions: &[E::Action]) -> VectorizedEpisodeStep<E::Observation> {
        assert!(self.has_reset, "reset must be called before step");
        assert_eq!(
            actions.len(),
            self.episodes.len(),
            "action batch size must match num_envs"
        );

        let mut results = Vec::with_capacity(self.episodes.len());
        for (episode, action) in self.episodes.iter_mut().zip(actions) {
            let mut result = episode.step(action.clone());
            if self.auto_reset && (result.terminated || result.truncated) {
                result = episode.reset();
            }
            results.push(result);
        }
        let step = collect_step(results.into_iter());
        self.action_history.push(actions.to_vec());
        self.absorb_step(Some(actions), &step);
        step
    }

    /// Returns an immutable reference to one episode.
    pub fn episode(&self, index: usize) -> &E {
        &self.episodes[index]
    }

    /// Returns a replay checkpoint for the completed action history.
    ///
    /// # Errors
    ///
    /// Returns [`VectorizedEpisodeCheckpointError::NotReset`] when called before
    /// the first batch reset.
    pub fn checkpoint(
        &self,
    ) -> Result<VectorizedEpisodeCheckpoint<E::Action>, VectorizedEpisodeCheckpointError> {
        if !self.has_reset {
            return Err(VectorizedEpisodeCheckpointError::NotReset);
        }
        Ok(VectorizedEpisodeCheckpoint {
            schema_version: VECTORIZED_EPISODE_CHECKPOINT_VERSION,
            seed: self.seed,
            num_envs: self.episodes.len(),
            auto_reset: self.auto_reset,
            has_reset: self.has_reset,
            actions: self.action_history.clone(),
            replay_digest: self.replay_digest,
        })
    }

    /// Restores a replay checkpoint by resetting and replaying every action batch.
    ///
    /// # Errors
    ///
    /// Returns an error when the schema, environment count, action batch shape,
    /// or resulting deterministic digest does not match.
    pub fn restore_checkpoint(
        &mut self,
        checkpoint: &VectorizedEpisodeCheckpoint<E::Action>,
    ) -> Result<(), VectorizedEpisodeCheckpointError> {
        if checkpoint.schema_version != VECTORIZED_EPISODE_CHECKPOINT_VERSION {
            return Err(VectorizedEpisodeCheckpointError::UnsupportedSchemaVersion {
                expected: VECTORIZED_EPISODE_CHECKPOINT_VERSION,
                actual: checkpoint.schema_version,
            });
        }
        if checkpoint.num_envs != self.episodes.len() {
            return Err(VectorizedEpisodeCheckpointError::EnvCountMismatch {
                expected: self.episodes.len(),
                actual: checkpoint.num_envs,
            });
        }
        if !checkpoint.has_reset {
            return Err(VectorizedEpisodeCheckpointError::NotReset);
        }
        self.auto_reset = checkpoint.auto_reset;
        self.seed = checkpoint.seed;
        self.reset();
        for (step_index, actions) in checkpoint.actions.iter().enumerate() {
            if actions.len() != self.episodes.len() {
                return Err(VectorizedEpisodeCheckpointError::ActionBatchMismatch {
                    step: step_index,
                    expected: self.episodes.len(),
                    actual: actions.len(),
                });
            }
            self.step(actions);
        }
        if self.replay_digest != checkpoint.replay_digest {
            return Err(VectorizedEpisodeCheckpointError::ReplayDigestMismatch {
                expected: checkpoint.replay_digest,
                actual: self.replay_digest,
            });
        }
        Ok(())
    }

    fn absorb_step(
        &mut self,
        actions: Option<&[E::Action]>,
        step: &VectorizedEpisodeStep<E::Observation>,
    ) {
        if let Some(actions) = actions {
            for action in actions {
                absorb_debug(&mut self.replay_digest, action);
            }
        }
        for (((observation, reward), terminated), truncated) in step
            .observations
            .iter()
            .zip(&step.rewards)
            .zip(&step.terminated)
            .zip(&step.truncated)
        {
            absorb_debug(&mut self.replay_digest, observation);
            absorb_debug(&mut self.replay_digest, reward);
            absorb_debug(&mut self.replay_digest, terminated);
            absorb_debug(&mut self.replay_digest, truncated);
        }
    }
}

fn collect_step<O>(
    results: impl Iterator<Item = crate::episode::EpisodeStep<O>>,
) -> VectorizedEpisodeStep<O> {
    let mut step = VectorizedEpisodeStep {
        observations: Vec::new(),
        rewards: Vec::new(),
        terminated: Vec::new(),
        truncated: Vec::new(),
    };
    for result in results {
        step.observations.push(result.observation);
        step.rewards.push(result.reward);
        step.terminated.push(result.terminated);
        step.truncated.push(result.truncated);
    }
    step
}

fn initial_digest(seed: u64, num_envs: usize) -> u64 {
    let mut digest = FNV_OFFSET_BASIS;
    absorb_debug(&mut digest, &seed);
    absorb_debug(&mut digest, &num_envs);
    digest
}

fn absorb_debug<T: Debug>(digest: &mut u64, value: &T) {
    for byte in format!("{value:?}").as_bytes() {
        *digest ^= u64::from(*byte);
        *digest = digest.wrapping_mul(FNV_PRIME);
    }
    *digest ^= 0xff;
    *digest = digest.wrapping_mul(FNV_PRIME);
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{Episode, EpisodeStep};

    #[derive(Clone, Debug)]
    struct ToyEpisode {
        value: i32,
        step: u32,
    }

    impl Episode for ToyEpisode {
        type Observation = i32;
        type Action = i32;

        fn reset(&mut self) -> EpisodeStep<Self::Observation> {
            self.value = 0;
            self.step = 0;
            EpisodeStep {
                observation: self.value,
                reward: 0.0,
                terminated: false,
                truncated: false,
            }
        }

        fn step(&mut self, action: Self::Action) -> EpisodeStep<Self::Observation> {
            self.value += action;
            self.step += 1;
            EpisodeStep {
                observation: self.value,
                reward: f64::from(self.value),
                terminated: false,
                truncated: self.step >= 4,
            }
        }

        fn episode_index(&self) -> u32 {
            0
        }

        fn step_in_episode(&self) -> u64 {
            u64::from(self.step)
        }
    }

    #[test]
    fn vectorized_batch_is_seeded_and_ordered() {
        let mut env = VectorizedEpisode::from_seeded(
            VectorizedEpisodeConfig {
                num_envs: 2,
                seed: 42,
                auto_reset: false,
            },
            |_seed| ToyEpisode { value: 0, step: 0 },
        );
        let reset = env.reset();
        assert_eq!(reset.observations, vec![0, 0]);
        let step = env.step(&[2, 3]);
        assert_eq!(step.observations, vec![2, 3]);
        assert_ne!(env.replay_digest(), initial_digest(42, 2));
    }

    #[test]
    fn replay_checkpoint_restores_digest() {
        let config = VectorizedEpisodeConfig {
            num_envs: 2,
            seed: 7,
            auto_reset: true,
        };
        let mut env =
            VectorizedEpisode::from_seeded(config, |_seed| ToyEpisode { value: 0, step: 0 });
        env.reset();
        env.step(&[1, 2]);
        let checkpoint = env.checkpoint().unwrap();
        let checkpoint_json = serde_json::to_string(&checkpoint).unwrap();
        let checkpoint_from_json: VectorizedEpisodeCheckpoint<i32> =
            serde_json::from_str(&checkpoint_json).unwrap();
        assert_eq!(checkpoint_from_json, checkpoint);
        let digest = env.replay_digest();
        env.step(&[4, 5]);
        env.restore_checkpoint(&checkpoint).unwrap();
        assert_eq!(env.replay_digest(), digest);
    }
}

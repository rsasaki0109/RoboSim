//! Batched differential-drive episodes for parallel RL rollouts.

use super::{
    DiffDriveEpisode, DiffDriveEpisodeConfig, DiffDriveEpisodeSnapshot,
    DiffDriveEpisodeSnapshotError,
};
use crate::action::DiffDriveAction;
use crate::episode::Episode;
use crate::observation::DiffDriveObservation;
use serde::{Deserialize, Serialize};

const VECTORIZED_DIFF_DRIVE_SNAPSHOT_VERSION: u32 = 1;

/// Error restoring or creating a vectorized diff-drive checkpoint.
#[derive(Clone, Debug, PartialEq)]
pub enum VectorizedDiffDriveSnapshotError {
    /// Snapshot payload schema is not supported by this engine.
    UnsupportedSchemaVersion {
        /// Expected snapshot schema version.
        expected: u32,
        /// Actual snapshot schema version.
        actual: u32,
    },
    /// Snapshot contains a different number of environments.
    EnvCountMismatch {
        /// Expected number of environments.
        expected: usize,
        /// Actual number of checkpoint entries.
        actual: usize,
    },
    /// One episode checkpoint failed.
    Episode {
        /// Environment index that failed.
        index: usize,
        /// Underlying episode checkpoint error.
        error: DiffDriveEpisodeSnapshotError,
    },
}

/// Completed-tick checkpoint of a [`VectorizedDiffDriveEnv`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VectorizedDiffDriveSnapshot {
    /// Snapshot payload schema version.
    pub schema_version: u32,
    /// Whether finished environments auto-reset after a step.
    pub auto_reset: bool,
    /// Per-environment episode checkpoints in environment index order.
    pub episodes: Vec<DiffDriveEpisodeSnapshot>,
}

/// Configuration for a vectorized diff-drive environment.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedDiffDriveConfig {
    /// Shared episode template; per-env RNG seeds are offset by env index.
    pub episode: DiffDriveEpisodeConfig,
    /// Number of parallel environments.
    pub num_envs: usize,
    /// When true, finished environments reset automatically at the next step.
    pub auto_reset: bool,
}

impl Default for VectorizedDiffDriveConfig {
    fn default() -> Self {
        Self {
            episode: DiffDriveEpisodeConfig::default(),
            num_envs: 1,
            auto_reset: true,
        }
    }
}

/// Batch result from a vectorized reset or step.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedDiffDriveStep {
    /// Observations for each environment.
    pub observations: Vec<DiffDriveObservation>,
    /// Rewards for each environment.
    pub rewards: Vec<f64>,
    /// Success flags for each environment.
    pub terminated: Vec<bool>,
    /// Timeout flags for each environment.
    pub truncated: Vec<bool>,
}

impl VectorizedDiffDriveStep {
    /// Returns true when every environment has ended.
    pub fn all_done(&self) -> bool {
        self.terminated
            .iter()
            .zip(&self.truncated)
            .all(|(terminated, truncated)| *terminated || *truncated)
    }

    /// Returns the number of environments that reached a terminal success state.
    pub fn success_count(&self) -> usize {
        self.terminated.iter().filter(|value| **value).count()
    }
}

/// Parallel wrapper around multiple [`DiffDriveEpisode`] instances.
pub struct VectorizedDiffDriveEnv {
    episodes: Vec<DiffDriveEpisode>,
    auto_reset: bool,
}

impl VectorizedDiffDriveEnv {
    /// Creates a vectorized environment from the given configuration.
    pub fn new(config: VectorizedDiffDriveConfig) -> Self {
        assert!(config.num_envs > 0, "num_envs must be positive");
        let episodes = (0..config.num_envs)
            .map(|index| {
                DiffDriveEpisode::new(DiffDriveEpisodeConfig {
                    rng_seed: config.episode.rng_seed.wrapping_add(index as u64),
                    ..config.episode.clone()
                })
            })
            .collect();
        Self {
            episodes,
            auto_reset: config.auto_reset,
        }
    }

    /// Returns the number of parallel environments.
    pub fn num_envs(&self) -> usize {
        self.episodes.len()
    }

    /// Resets every environment and returns the initial batch.
    pub fn reset(&mut self) -> VectorizedDiffDriveStep {
        let mut step = VectorizedDiffDriveStep {
            observations: Vec::with_capacity(self.episodes.len()),
            rewards: Vec::with_capacity(self.episodes.len()),
            terminated: Vec::with_capacity(self.episodes.len()),
            truncated: Vec::with_capacity(self.episodes.len()),
        };

        for episode in &mut self.episodes {
            let result = episode.reset();
            step.observations.push(result.observation);
            step.rewards.push(result.reward);
            step.terminated.push(result.terminated);
            step.truncated.push(result.truncated);
        }

        step
    }

    /// Steps all environments with the corresponding actions.
    pub fn step(&mut self, actions: &[DiffDriveAction]) -> VectorizedDiffDriveStep {
        assert_eq!(
            actions.len(),
            self.episodes.len(),
            "action batch size must match num_envs"
        );

        let mut step = VectorizedDiffDriveStep {
            observations: Vec::with_capacity(self.episodes.len()),
            rewards: Vec::with_capacity(self.episodes.len()),
            terminated: Vec::with_capacity(self.episodes.len()),
            truncated: Vec::with_capacity(self.episodes.len()),
        };

        for (episode, action) in self.episodes.iter_mut().zip(actions) {
            let mut result = episode.step(*action);
            if self.auto_reset && result.is_done() {
                result = episode.reset();
            }
            step.observations.push(result.observation);
            step.rewards.push(result.reward);
            step.terminated.push(result.terminated);
            step.truncated.push(result.truncated);
        }

        step
    }

    /// Returns a completed-tick checkpoint for every environment.
    ///
    /// # Errors
    ///
    /// Returns [`VectorizedDiffDriveSnapshotError::Episode`] if any underlying
    /// episode cannot be snapshotted.
    pub fn checkpoint(
        &self,
    ) -> Result<VectorizedDiffDriveSnapshot, VectorizedDiffDriveSnapshotError> {
        let mut episodes = Vec::with_capacity(self.episodes.len());
        for (index, episode) in self.episodes.iter().enumerate() {
            episodes.push(
                episode
                    .checkpoint()
                    .map_err(|error| VectorizedDiffDriveSnapshotError::Episode { index, error })?,
            );
        }
        Ok(VectorizedDiffDriveSnapshot {
            schema_version: VECTORIZED_DIFF_DRIVE_SNAPSHOT_VERSION,
            auto_reset: self.auto_reset,
            episodes,
        })
    }

    /// Restores every environment from a completed-tick checkpoint.
    ///
    /// # Errors
    ///
    /// Returns an error if the snapshot schema is unsupported, if the number of
    /// environments differs, or if any underlying episode checkpoint is
    /// incompatible with its environment.
    pub fn restore_checkpoint(
        &mut self,
        snapshot: &VectorizedDiffDriveSnapshot,
    ) -> Result<(), VectorizedDiffDriveSnapshotError> {
        if snapshot.schema_version != VECTORIZED_DIFF_DRIVE_SNAPSHOT_VERSION {
            return Err(VectorizedDiffDriveSnapshotError::UnsupportedSchemaVersion {
                expected: VECTORIZED_DIFF_DRIVE_SNAPSHOT_VERSION,
                actual: snapshot.schema_version,
            });
        }
        if snapshot.episodes.len() != self.episodes.len() {
            return Err(VectorizedDiffDriveSnapshotError::EnvCountMismatch {
                expected: self.episodes.len(),
                actual: snapshot.episodes.len(),
            });
        }

        for (index, (episode, checkpoint)) in
            self.episodes.iter_mut().zip(&snapshot.episodes).enumerate()
        {
            episode
                .restore_checkpoint(checkpoint)
                .map_err(|error| VectorizedDiffDriveSnapshotError::Episode { index, error })?;
        }
        self.auto_reset = snapshot.auto_reset;
        Ok(())
    }

    /// Returns read access to one underlying episode.
    pub fn episode(&self, index: usize) -> &DiffDriveEpisode {
        &self.episodes[index]
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::action::DiffDriveAction;
    use crate::domain_randomization::DiffDriveDomainRandomization;

    #[test]
    fn vectorized_env_runs_parallel_episodes() {
        let mut env = VectorizedDiffDriveEnv::new(VectorizedDiffDriveConfig {
            episode: DiffDriveEpisodeConfig {
                max_steps: 350,
                goal_x_m: 0.5,
                ..DiffDriveEpisodeConfig::default()
            },
            num_envs: 4,
            auto_reset: false,
        });

        env.reset();
        let actions = vec![DiffDriveAction::forward(6.0); env.num_envs()];
        let mut successes = 0;

        for _ in 0..360 {
            let step = env.step(&actions);
            successes += step.success_count();
            if step.all_done() {
                break;
            }
        }

        assert!(successes > 0);
    }

    #[test]
    fn vectorized_domain_randomization_differs_by_env_index() {
        let mut env = VectorizedDiffDriveEnv::new(VectorizedDiffDriveConfig {
            episode: DiffDriveEpisodeConfig {
                domain_randomization: Some(DiffDriveDomainRandomization::forward_goal_training()),
                rng_seed: 99,
                ..DiffDriveEpisodeConfig::default()
            },
            num_envs: 3,
            auto_reset: false,
        });
        env.reset();

        let goals: Vec<_> = (0..env.num_envs())
            .map(|index| env.episode(index).goal_x_m())
            .collect();
        assert!(goals.windows(2).any(|pair| pair[0] != pair[1]));
    }

    #[test]
    fn checkpoint_restores_vectorized_episode_state() {
        let mut env = VectorizedDiffDriveEnv::new(VectorizedDiffDriveConfig {
            episode: DiffDriveEpisodeConfig {
                max_steps: 120,
                goal_x_m: 0.75,
                ..DiffDriveEpisodeConfig::default()
            },
            num_envs: 2,
            auto_reset: false,
        });
        env.reset();
        env.step(&[DiffDriveAction::forward(2.0), DiffDriveAction::forward(4.0)]);

        let checkpoint = env.checkpoint().unwrap();

        env.step(&[
            DiffDriveAction::forward(-1.0),
            DiffDriveAction::forward(1.0),
        ]);
        env.restore_checkpoint(&checkpoint).unwrap();

        assert_eq!(env.checkpoint().unwrap(), checkpoint);
    }

    #[test]
    fn checkpoint_rejects_env_count_mismatch() {
        let mut env = VectorizedDiffDriveEnv::new(VectorizedDiffDriveConfig {
            num_envs: 2,
            auto_reset: false,
            ..VectorizedDiffDriveConfig::default()
        });
        env.reset();
        let mut checkpoint = env.checkpoint().unwrap();
        checkpoint.episodes.pop();

        assert_eq!(
            env.restore_checkpoint(&checkpoint),
            Err(VectorizedDiffDriveSnapshotError::EnvCountMismatch {
                expected: 2,
                actual: 1
            })
        );
    }
}

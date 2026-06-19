//! Batched mobile manipulator episodes for population-based / parallel RL rollouts.

use super::{
    MobileManipulatorEpisode, MobileManipulatorEpisodeConfig, MobileManipulatorEpisodeSnapshot,
    MobileManipulatorEpisodeSnapshotError,
};
use crate::action::MobileManipulatorAction;
use crate::episode::Episode;
use crate::observation::MobileManipulatorObservation;
use serde::{Deserialize, Serialize};

const VECTORIZED_MOBILE_MANIPULATOR_SNAPSHOT_VERSION: u32 = 1;

/// Error restoring a vectorized mobile-manipulator checkpoint.
#[derive(Clone, Debug, PartialEq)]
pub enum VectorizedMobileManipulatorSnapshotError {
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
        error: MobileManipulatorEpisodeSnapshotError,
    },
}

/// Completed-tick checkpoint of a [`VectorizedMobileManipulatorEnv`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VectorizedMobileManipulatorSnapshot {
    /// Snapshot payload schema version.
    pub schema_version: u32,
    /// Whether finished environments auto-reset after a step.
    pub auto_reset: bool,
    /// Per-environment episode checkpoints in environment index order.
    pub episodes: Vec<MobileManipulatorEpisodeSnapshot>,
}

/// Configuration for a vectorized mobile manipulator environment.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedMobileManipulatorConfig {
    /// Shared episode template applied to every environment.
    pub episode: MobileManipulatorEpisodeConfig,
    /// Number of environments stepped in lock-step.
    pub num_envs: usize,
    /// When true, finished environments reset automatically at the next step.
    pub auto_reset: bool,
}

impl VectorizedMobileManipulatorConfig {
    /// Creates a configuration for `num_envs` copies of `episode` (no auto-reset).
    pub fn new(episode: MobileManipulatorEpisodeConfig, num_envs: usize) -> Self {
        Self {
            episode,
            num_envs,
            auto_reset: false,
        }
    }
}

/// Batch result from a vectorized reset or step.
#[derive(Clone, Debug, PartialEq)]
pub struct VectorizedMobileManipulatorStep {
    /// Observations for each environment.
    pub observations: Vec<MobileManipulatorObservation>,
    /// Rewards for each environment.
    pub rewards: Vec<f64>,
    /// Success flags for each environment.
    pub terminated: Vec<bool>,
    /// Timeout flags for each environment.
    pub truncated: Vec<bool>,
}

impl VectorizedMobileManipulatorStep {
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

/// Wrapper around multiple [`MobileManipulatorEpisode`] instances stepped together.
///
/// All environments share the same deterministic configuration; they diverge only
/// through the per-environment actions, which makes this a convenient way to evaluate a
/// population of policies (e.g. for the Cross-Entropy Method) in lock-step.
pub struct VectorizedMobileManipulatorEnv {
    episodes: Vec<MobileManipulatorEpisode>,
    auto_reset: bool,
}

impl VectorizedMobileManipulatorEnv {
    /// Creates a vectorized environment from the given configuration.
    pub fn new(config: VectorizedMobileManipulatorConfig) -> Self {
        assert!(config.num_envs > 0, "num_envs must be positive");
        let episodes = (0..config.num_envs)
            .map(|_| MobileManipulatorEpisode::new(config.episode.clone()))
            .collect();
        Self {
            episodes,
            auto_reset: config.auto_reset,
        }
    }

    /// Returns the number of environments.
    pub fn num_envs(&self) -> usize {
        self.episodes.len()
    }

    /// Resets every environment and returns the initial batch.
    pub fn reset(&mut self) -> VectorizedMobileManipulatorStep {
        let mut step = self.empty_step();
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
    pub fn step(&mut self, actions: &[MobileManipulatorAction]) -> VectorizedMobileManipulatorStep {
        assert_eq!(
            actions.len(),
            self.episodes.len(),
            "action batch size must match num_envs"
        );

        let mut step = self.empty_step();
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
    pub fn checkpoint(&self) -> VectorizedMobileManipulatorSnapshot {
        VectorizedMobileManipulatorSnapshot {
            schema_version: VECTORIZED_MOBILE_MANIPULATOR_SNAPSHOT_VERSION,
            auto_reset: self.auto_reset,
            episodes: self
                .episodes
                .iter()
                .map(MobileManipulatorEpisode::checkpoint)
                .collect(),
        }
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
        snapshot: &VectorizedMobileManipulatorSnapshot,
    ) -> Result<(), VectorizedMobileManipulatorSnapshotError> {
        if snapshot.schema_version != VECTORIZED_MOBILE_MANIPULATOR_SNAPSHOT_VERSION {
            return Err(
                VectorizedMobileManipulatorSnapshotError::UnsupportedSchemaVersion {
                    expected: VECTORIZED_MOBILE_MANIPULATOR_SNAPSHOT_VERSION,
                    actual: snapshot.schema_version,
                },
            );
        }
        if snapshot.episodes.len() != self.episodes.len() {
            return Err(VectorizedMobileManipulatorSnapshotError::EnvCountMismatch {
                expected: self.episodes.len(),
                actual: snapshot.episodes.len(),
            });
        }

        for (index, (episode, checkpoint)) in
            self.episodes.iter_mut().zip(&snapshot.episodes).enumerate()
        {
            episode.restore_checkpoint(checkpoint).map_err(|error| {
                VectorizedMobileManipulatorSnapshotError::Episode { index, error }
            })?;
        }
        self.auto_reset = snapshot.auto_reset;
        Ok(())
    }

    /// Returns read access to one underlying episode (e.g. for its total reward).
    pub fn episode(&self, index: usize) -> &MobileManipulatorEpisode {
        &self.episodes[index]
    }

    fn empty_step(&self) -> VectorizedMobileManipulatorStep {
        VectorizedMobileManipulatorStep {
            observations: Vec::with_capacity(self.episodes.len()),
            rewards: Vec::with_capacity(self.episodes.len()),
            terminated: Vec::with_capacity(self.episodes.len()),
            truncated: Vec::with_capacity(self.episodes.len()),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn vectorized_reach_diverges_by_action() {
        let mut env = VectorizedMobileManipulatorEnv::new(VectorizedMobileManipulatorConfig::new(
            MobileManipulatorEpisodeConfig::reach(),
            2,
        ));
        env.reset();
        // Env 0 drives toward the target; env 1 does nothing.
        let solving = MobileManipulatorAction {
            shoulder_velocity_rad_s: -3.0,
            ..MobileManipulatorAction::default()
        };
        let idle = MobileManipulatorAction::default();
        for _ in 0..300 {
            let step = env.step(&[solving, idle]);
            if step.terminated[0] {
                break;
            }
        }
        assert!(
            env.episode(0).total_reward() > env.episode(1).total_reward(),
            "the driven environment should out-reward the idle one"
        );
    }

    #[test]
    fn checkpoint_restores_vectorized_episode_state() {
        let mut env = VectorizedMobileManipulatorEnv::new(VectorizedMobileManipulatorConfig::new(
            MobileManipulatorEpisodeConfig::reach_randomized(5),
            2,
        ));
        env.reset();
        env.step(&[
            MobileManipulatorAction {
                shoulder_velocity_rad_s: 0.5,
                ..MobileManipulatorAction::default()
            },
            MobileManipulatorAction {
                elbow_velocity_rad_s: -0.25,
                ..MobileManipulatorAction::default()
            },
        ]);

        let checkpoint = env.checkpoint();

        env.step(&[
            MobileManipulatorAction {
                shoulder_velocity_rad_s: -1.0,
                ..MobileManipulatorAction::default()
            },
            MobileManipulatorAction {
                elbow_velocity_rad_s: 1.0,
                ..MobileManipulatorAction::default()
            },
        ]);
        env.restore_checkpoint(&checkpoint).unwrap();

        assert_eq!(env.checkpoint(), checkpoint);
    }

    #[test]
    fn checkpoint_rejects_env_count_mismatch() {
        let mut env = VectorizedMobileManipulatorEnv::new(VectorizedMobileManipulatorConfig::new(
            MobileManipulatorEpisodeConfig::reach(),
            2,
        ));
        env.reset();
        let mut checkpoint = env.checkpoint();
        checkpoint.episodes.pop();

        assert_eq!(
            env.restore_checkpoint(&checkpoint),
            Err(VectorizedMobileManipulatorSnapshotError::EnvCountMismatch {
                expected: 2,
                actual: 1
            })
        );
    }
}

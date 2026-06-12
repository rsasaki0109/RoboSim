//! Batched differential-drive episodes for parallel RL rollouts.

use super::{DiffDriveEpisode, DiffDriveEpisodeConfig};
use crate::action::DiffDriveAction;
use crate::episode::Episode;
use crate::observation::DiffDriveObservation;

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
}

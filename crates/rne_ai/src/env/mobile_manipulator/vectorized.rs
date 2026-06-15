//! Batched mobile manipulator episodes for population-based / parallel RL rollouts.

use super::{MobileManipulatorEpisode, MobileManipulatorEpisodeConfig};
use crate::action::MobileManipulatorAction;
use crate::episode::Episode;
use crate::observation::MobileManipulatorObservation;

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
}

//! Deterministic proximal policy optimization for continuous Gaussian policies.
//!
//! PPO is the on-policy optimizer the native learning core was missing. It builds
//! directly on [`crate::neural::NeuralNet`] and [`crate::neural::Adam`]: a policy
//! network outputs the Gaussian mean, a value network supplies the baseline, and
//! generalized advantage estimation drives the clipped surrogate loss. Action
//! sampling is seeded, rollouts are processed in a fixed order, and no external
//! ML dependency is used, so training is reproducible.

use crate::neural::{Adam, NeuralError, NeuralNet};
use crate::policy_artifact::{Activation, PolicyArtifact};
use crate::rng::DeterministicRng;

const LOG_2PI: f64 = 1.837_877_066_409_345_4;

/// One environment transition returned by [`PpoEnv::step`].
#[derive(Clone, Debug, PartialEq)]
pub struct PpoStep {
    /// Observation after the action.
    pub observation: Vec<f64>,
    /// Reward for the action.
    pub reward: f64,
    /// Whether the episode ended.
    pub terminated: bool,
}

/// Minimal environment interface for PPO.
pub trait PpoEnv {
    /// Observation vector width.
    fn observation_size(&self) -> usize;
    /// Continuous action vector width.
    fn action_size(&self) -> usize;
    /// Starts a new episode and returns the first observation.
    fn reset(&mut self) -> Vec<f64>;
    /// Applies an action and returns the next transition.
    fn step(&mut self, action: &[f64]) -> PpoStep;
}

/// PPO hyperparameters.
#[derive(Clone, Debug, PartialEq)]
pub struct PpoConfig {
    /// Hidden layer widths shared by the policy and value networks.
    pub hidden_sizes: Vec<usize>,
    /// Policy network learning rate.
    pub policy_learning_rate: f64,
    /// Value network learning rate.
    pub value_learning_rate: f64,
    /// Discount factor.
    pub gamma: f64,
    /// GAE smoothing factor.
    pub lambda: f64,
    /// PPO clipping range.
    pub clip: f64,
    /// Optimization epochs per rollout.
    pub epochs: usize,
    /// Environment steps per rollout.
    pub rollout_steps: usize,
    /// Entropy bonus coefficient.
    pub entropy_coef: f64,
    /// Value loss coefficient.
    pub value_coef: f64,
    /// Initial log standard deviation of the Gaussian policy.
    pub log_std_init: f64,
    /// Seed for weight initialization and action sampling.
    pub seed: u64,
}

impl Default for PpoConfig {
    fn default() -> Self {
        Self {
            hidden_sizes: vec![32, 32],
            policy_learning_rate: 3.0e-4,
            value_learning_rate: 1.0e-3,
            gamma: 0.99,
            lambda: 0.95,
            clip: 0.2,
            epochs: 10,
            rollout_steps: 256,
            entropy_coef: 0.0,
            value_coef: 0.5,
            log_std_init: 0.0,
            seed: 0,
        }
    }
}

/// PPO training failure.
#[derive(Debug, thiserror::Error)]
pub enum PpoError {
    /// Network construction failed.
    #[error("PPO network error: {0}")]
    Neural(#[from] NeuralError),
    /// The environment sizes did not match the trainer.
    #[error("PPO environment mismatch: expected {expected_obs}/{expected_act}, got {actual_obs}/{actual_act}")]
    EnvironmentMismatch {
        /// Trainer observation width.
        expected_obs: usize,
        /// Trainer action width.
        expected_act: usize,
        /// Environment observation width.
        actual_obs: usize,
        /// Environment action width.
        actual_act: usize,
    },
}

/// Summary of a PPO run.
#[derive(Clone, Debug, PartialEq)]
pub struct PpoReport {
    /// Mean rollout reward per training iteration.
    pub mean_reward_history: Vec<f64>,
    /// Number of iterations executed.
    pub iterations: usize,
}

impl PpoReport {
    /// Mean reward of the last iteration.
    pub fn final_mean_reward(&self) -> f64 {
        self.mean_reward_history.last().copied().unwrap_or(0.0)
    }
}

struct Transition {
    observation: Vec<f64>,
    action: Vec<f64>,
    log_prob: f64,
    reward: f64,
    value: f64,
    done: bool,
}

/// A PPO trainer holding a policy and value network.
#[derive(Clone, Debug)]
pub struct PpoTrainer {
    policy: NeuralNet,
    value: NeuralNet,
    log_std: Vec<f64>,
    adam_policy: Adam,
    adam_value: Adam,
    adam_log_std: Adam,
    rng: DeterministicRng,
    config: PpoConfig,
    observation_size: usize,
    action_size: usize,
    current_observation: Vec<f64>,
}

impl PpoTrainer {
    /// Creates a trainer for the given observation and action widths.
    pub fn new(
        observation_size: usize,
        action_size: usize,
        config: PpoConfig,
    ) -> Result<Self, PpoError> {
        let mut policy_sizes = vec![observation_size];
        policy_sizes.extend_from_slice(&config.hidden_sizes);
        policy_sizes.push(action_size);
        let mut value_sizes = vec![observation_size];
        value_sizes.extend_from_slice(&config.hidden_sizes);
        value_sizes.push(1);
        let policy = NeuralNet::new(&policy_sizes, Activation::Tanh, config.seed)?;
        let value = NeuralNet::new(&value_sizes, Activation::Tanh, config.seed ^ 0x9e37_79b9)?;
        let log_std = vec![config.log_std_init; action_size];
        let adam_policy = Adam::new(&policy, config.policy_learning_rate);
        let adam_value = Adam::new(&value, config.value_learning_rate);
        let adam_log_std = Adam::with_count(action_size, config.policy_learning_rate);
        Ok(Self {
            policy,
            value,
            log_std,
            adam_policy,
            adam_value,
            adam_log_std,
            rng: DeterministicRng::new(config.seed ^ 0x1234_5678),
            config,
            observation_size,
            action_size,
            current_observation: Vec::new(),
        })
    }

    /// The trained policy network (Gaussian mean).
    pub fn policy(&self) -> &NeuralNet {
        &self.policy
    }

    /// Exports the deterministic mean policy as a [`PolicyArtifact`].
    pub fn to_policy_artifact(
        &self,
        name: impl Into<String>,
        source: impl Into<String>,
        action_lower: Vec<f64>,
        action_upper: Vec<f64>,
    ) -> Result<PolicyArtifact, PpoError> {
        Ok(self
            .policy
            .to_policy_artifact(name, source, action_lower, action_upper)?)
    }

    /// Trains for `iterations` rollouts.
    pub fn train<E: PpoEnv>(
        &mut self,
        env: &mut E,
        iterations: usize,
    ) -> Result<PpoReport, PpoError> {
        if env.observation_size() != self.observation_size || env.action_size() != self.action_size
        {
            return Err(PpoError::EnvironmentMismatch {
                expected_obs: self.observation_size,
                expected_act: self.action_size,
                actual_obs: env.observation_size(),
                actual_act: env.action_size(),
            });
        }
        if self.current_observation.is_empty() {
            self.current_observation = env.reset();
        }

        let mut history = Vec::with_capacity(iterations);
        for _ in 0..iterations {
            let mut transitions = Vec::with_capacity(self.config.rollout_steps);
            let mut reward_sum = 0.0;
            for _ in 0..self.config.rollout_steps {
                let observation = self.current_observation.clone();
                let (action, log_prob) = self.sample_action(&observation);
                let value = self.value.forward(&observation)[0];
                let step = env.step(&action);
                reward_sum += step.reward;
                transitions.push(Transition {
                    observation: observation.clone(),
                    action,
                    log_prob,
                    reward: step.reward,
                    value,
                    done: step.terminated,
                });
                self.current_observation = if step.terminated {
                    env.reset()
                } else {
                    step.observation
                };
            }
            history.push(reward_sum / self.config.rollout_steps as f64);

            let (advantages, returns) = self.gae(&transitions);
            let (adv_mean, adv_std) = mean_std(&advantages);
            let advantages: Vec<f64> = advantages
                .iter()
                .map(|value| (value - adv_mean) / (adv_std + 1.0e-8))
                .collect();

            for _ in 0..self.config.epochs {
                self.optimize_epoch(&transitions, &advantages, &returns);
            }
        }

        Ok(PpoReport {
            mean_reward_history: history,
            iterations,
        })
    }

    #[allow(clippy::needless_range_loop)]
    fn sample_action(&mut self, observation: &[f64]) -> (Vec<f64>, f64) {
        let mean = self.policy.forward(observation);
        let mut action = Vec::with_capacity(self.action_size);
        let mut log_prob = 0.0;
        for index in 0..self.action_size {
            let sigma = self.log_std[index].exp();
            let noise = standard_normal(&mut self.rng);
            let value = mean[index] + sigma * noise;
            let z = (value - mean[index]) / sigma;
            log_prob += -0.5 * z * z - self.log_std[index] - 0.5 * LOG_2PI;
            action.push(value);
        }
        (action, log_prob)
    }

    fn gae(&self, transitions: &[Transition]) -> (Vec<f64>, Vec<f64>) {
        let mut advantages = vec![0.0; transitions.len()];
        let mut returns = vec![0.0; transitions.len()];
        let mut last_value = self.value.forward(&self.current_observation)[0];
        let mut running = 0.0;
        for index in (0..transitions.len()).rev() {
            let transition = &transitions[index];
            let non_terminal = if transition.done { 0.0 } else { 1.0 };
            let delta = transition.reward + self.config.gamma * last_value * non_terminal
                - transition.value;
            running = delta + self.config.gamma * self.config.lambda * non_terminal * running;
            advantages[index] = running;
            returns[index] = running + transition.value;
            last_value = transition.value;
        }
        (advantages, returns)
    }

    fn optimize_epoch(&mut self, transitions: &[Transition], advantages: &[f64], returns: &[f64]) {
        let batch = transitions.len() as f64;
        let mut policy_gradients = vec![0.0; self.policy.parameter_count()];
        let mut value_gradients = vec![0.0; self.value.parameter_count()];
        let mut log_std_gradients = vec![0.0; self.action_size];

        for (index, transition) in transitions.iter().enumerate() {
            let mean = self.policy.forward(&transition.observation);
            let advantage = advantages[index];
            let mut new_log_prob = 0.0;
            let mut grad_mean = vec![0.0; self.action_size];
            for action_index in 0..self.action_size {
                let sigma = self.log_std[action_index].exp();
                let difference = transition.action[action_index] - mean[action_index];
                let z = difference / sigma;
                new_log_prob += -0.5 * z * z - self.log_std[action_index] - 0.5 * LOG_2PI;
                grad_mean[action_index] = z / sigma;
                // Entropy bonus only; the policy term is added below.
                log_std_gradients[action_index] += -self.config.entropy_coef;
            }
            let ratio = (new_log_prob - transition.log_prob).exp();
            let unclipped = ratio * advantage;
            let clipped = ratio.clamp(1.0 - self.config.clip, 1.0 + self.config.clip) * advantage;
            // d(-min(unclipped, clipped))/d log_prob.
            let d_loss_d_log_prob = if unclipped <= clipped {
                -advantage * ratio
            } else {
                0.0
            };
            for action_index in 0..self.action_size {
                grad_mean[action_index] *= d_loss_d_log_prob;
                let sigma = self.log_std[action_index].exp();
                let difference = transition.action[action_index] - mean[action_index];
                let z = difference / sigma;
                log_std_gradients[action_index] += d_loss_d_log_prob * (z * z - 1.0);
            }

            for (accumulated, grad) in policy_gradients.iter_mut().zip(flat_gradients(
                &self.policy.backward(&transition.observation, &grad_mean),
            )) {
                *accumulated += grad;
            }

            let value = self.value.forward(&transition.observation)[0];
            let grad_value = self.config.value_coef * (value - returns[index]);
            for (accumulated, grad) in value_gradients.iter_mut().zip(flat_gradients(
                &self.value.backward(&transition.observation, &[grad_value]),
            )) {
                *accumulated += grad;
            }
        }

        for gradient in &mut policy_gradients {
            *gradient /= batch;
        }
        for gradient in &mut value_gradients {
            *gradient /= batch;
        }
        for gradient in &mut log_std_gradients {
            *gradient /= batch;
        }

        self.adam_policy.step(&mut self.policy, &policy_gradients);
        self.adam_value.step(&mut self.value, &value_gradients);
        self.adam_log_std
            .step_slice(&mut self.log_std, &log_std_gradients);
    }
}

fn flat_gradients(gradients: &[crate::neural::LayerGradient]) -> Vec<f64> {
    let mut out = Vec::new();
    for gradient in gradients {
        out.extend_from_slice(&gradient.weights);
        out.extend_from_slice(&gradient.biases);
    }
    out
}

fn mean_std(values: &[f64]) -> (f64, f64) {
    if values.is_empty() {
        return (0.0, 0.0);
    }
    let mean = values.iter().sum::<f64>() / values.len() as f64;
    let variance = values
        .iter()
        .map(|value| (value - mean).powi(2))
        .sum::<f64>()
        / values.len() as f64;
    (mean, variance.sqrt())
}

fn standard_normal(rng: &mut DeterministicRng) -> f64 {
    let u1 = ((rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64).max(f64::MIN_POSITIVE);
    let u2 = (rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A one-step continuous bandit: reward peaks at `target`.
    struct Bandit {
        target: f64,
    }

    impl PpoEnv for Bandit {
        fn observation_size(&self) -> usize {
            1
        }
        fn action_size(&self) -> usize {
            1
        }
        fn reset(&mut self) -> Vec<f64> {
            vec![1.0]
        }
        fn step(&mut self, action: &[f64]) -> PpoStep {
            PpoStep {
                observation: vec![1.0],
                reward: -(action[0] - self.target).powi(2),
                terminated: true,
            }
        }
    }

    fn config() -> PpoConfig {
        PpoConfig {
            hidden_sizes: vec![16, 16],
            policy_learning_rate: 0.02,
            value_learning_rate: 0.05,
            gamma: 1.0,
            lambda: 1.0,
            clip: 0.2,
            epochs: 5,
            rollout_steps: 32,
            entropy_coef: 0.0,
            value_coef: 0.5,
            log_std_init: 0.5,
            seed: 3,
        }
    }

    #[test]
    fn ppo_learns_a_bandit_target() {
        let mut env = Bandit { target: 1.0 };
        let mut trainer = PpoTrainer::new(1, 1, config()).expect("trainer");
        let initial = trainer.policy().forward(&[1.0])[0];
        let report = trainer.train(&mut env, 300).expect("train");
        let final_mean = trainer.policy().forward(&[1.0])[0];
        assert!(
            (final_mean - 1.0).abs() < (initial - 1.0).abs(),
            "mean {initial} -> {final_mean} did not approach 1.0"
        );
        assert!(
            (final_mean - 1.0).abs() < 0.2,
            "final mean {final_mean} too far from the target"
        );
        assert!(report.final_mean_reward() > -0.05);
    }

    #[test]
    fn ppo_is_deterministic() {
        let run = || {
            let mut env = Bandit { target: -0.5 };
            let mut trainer = PpoTrainer::new(1, 1, config()).expect("trainer");
            trainer.train(&mut env, 40).expect("train");
            trainer.policy().parameters()
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn ppo_rejects_mismatched_environment() {
        let mut env = Bandit { target: 0.0 };
        let mut trainer = PpoTrainer::new(2, 1, config()).expect("trainer");
        assert!(matches!(
            trainer.train(&mut env, 1),
            Err(PpoError::EnvironmentMismatch { .. })
        ));
    }
}

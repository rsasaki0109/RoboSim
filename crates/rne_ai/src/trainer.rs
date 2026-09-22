//! Deterministic, dependency-free native policy training.
//!
//! The workspace's learned controllers were produced by ad-hoc CEM loops copied
//! into examples. This module lifts that into a reusable, seeded trainer that
//! optimizes a flat parameter vector against a caller-supplied deterministic
//! fitness function, then exports the winner as a [`PolicyArtifact`].
//!
//! Cross-entropy method (CEM) is a gradient-free black-box optimizer: it samples
//! a Gaussian population, keeps the elite, and refits the search distribution.
//! Everything is plain `f64` arithmetic with one seeded [`DeterministicRng`], so
//! a training run is reproducible bit-for-bit. Neural PPO/SAC with analytic
//! gradients is a later increment; the artifact export path is shared.

use crate::policy_artifact::{
    Activation, DenseLayer, PolicyArtifact, PolicyArtifactError, POLICY_ARTIFACT_KIND,
    POLICY_ARTIFACT_SCHEMA_VERSION,
};
use crate::rng::DeterministicRng;
use serde::{Deserialize, Serialize};

/// Cross-entropy method configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CemConfig {
    /// Number of candidates evaluated per iteration (even, `>= 4`).
    pub population: usize,
    /// Fraction of the population kept as the elite, in `(0, 1)`.
    pub elite_fraction: f64,
    /// Number of iterations.
    pub iterations: usize,
    /// Initial per-coordinate sampling standard deviation.
    pub initial_std: f64,
    /// Floor applied to the fitted per-coordinate standard deviation.
    pub min_std: f64,
    /// Seed for the deterministic sampler.
    pub seed: u64,
}

impl Default for CemConfig {
    fn default() -> Self {
        Self {
            population: 32,
            elite_fraction: 0.25,
            iterations: 50,
            initial_std: 1.0,
            min_std: 1e-3,
            seed: 0,
        }
    }
}

impl CemConfig {
    fn validate(&self) -> Result<(), CemTrainerError> {
        if self.population < 4 || !self.population.is_multiple_of(2) {
            return Err(CemTrainerError::InvalidConfig(
                "population must be even and at least 4".into(),
            ));
        }
        if !(self.elite_fraction.is_finite()
            && self.elite_fraction > 0.0
            && self.elite_fraction < 1.0)
        {
            return Err(CemTrainerError::InvalidConfig(
                "elite_fraction must be in (0, 1)".into(),
            ));
        }
        if self.iterations == 0 {
            return Err(CemTrainerError::InvalidConfig(
                "iterations must be non-zero".into(),
            ));
        }
        if !self.initial_std.is_finite() || self.initial_std <= 0.0 {
            return Err(CemTrainerError::InvalidConfig(
                "initial_std must be finite and positive".into(),
            ));
        }
        if !self.min_std.is_finite() || self.min_std <= 0.0 || self.min_std > self.initial_std {
            return Err(CemTrainerError::InvalidConfig(
                "min_std must be finite, positive, and no larger than initial_std".into(),
            ));
        }
        Ok(())
    }
}

/// Result of a CEM training run.
#[derive(Clone, Debug, PartialEq)]
pub struct CemResult {
    /// Best parameter vector seen across all iterations.
    pub best_parameters: Vec<f64>,
    /// Fitness of [`Self::best_parameters`].
    pub best_fitness: f64,
    /// Mean elite fitness per iteration, in evaluation order.
    pub elite_fitness_history: Vec<f64>,
    /// Number of iterations executed.
    pub iterations: usize,
}

/// Description of a dense MLP policy whose flat parameters CEM optimizes.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct MlpPolicyTemplate {
    /// Observation vector width.
    pub observation_size: u32,
    /// Action vector width.
    pub action_size: u32,
    /// Hidden layer widths, in order.
    pub hidden_sizes: Vec<u32>,
    /// Activation used by every hidden layer (the output layer is linear).
    pub hidden_activation: Activation,
    /// Inclusive lower action clamp per output.
    pub action_lower: Vec<f64>,
    /// Inclusive upper action clamp per output.
    pub action_upper: Vec<f64>,
}

impl MlpPolicyTemplate {
    /// Number of scalar parameters (weights then biases, layer by layer).
    pub fn parameter_count(&self) -> usize {
        let mut count = 0usize;
        let mut input = self.observation_size as usize;
        for output in self
            .hidden_sizes
            .iter()
            .copied()
            .chain(std::iter::once(self.action_size))
        {
            let output = output as usize;
            count += input * output + output;
            input = output;
        }
        count
    }

    /// Packs a flat parameter vector into a validated [`PolicyArtifact`].
    pub fn to_artifact(
        &self,
        name: impl Into<String>,
        source: impl Into<String>,
        parameters: &[f64],
    ) -> Result<PolicyArtifact, CemTrainerError> {
        if parameters.len() != self.parameter_count() {
            return Err(CemTrainerError::ParameterCount {
                expected: self.parameter_count(),
                actual: parameters.len(),
            });
        }
        let mut layers = Vec::with_capacity(self.hidden_sizes.len() + 1);
        let mut input = self.observation_size;
        let mut offset = 0usize;
        let width_count = self.hidden_sizes.len();
        for (index, output) in self
            .hidden_sizes
            .iter()
            .copied()
            .chain(std::iter::once(self.action_size))
            .enumerate()
        {
            let input_width = input as usize;
            let output_width = output as usize;
            let weight_count = input_width * output_width;
            let weights = parameters[offset..offset + weight_count].to_vec();
            offset += weight_count;
            let biases = parameters[offset..offset + output_width].to_vec();
            offset += output_width;
            let activation = if index < width_count {
                self.hidden_activation
            } else {
                Activation::Identity
            };
            layers.push(DenseLayer {
                input_size: input,
                output_size: output,
                weights,
                biases,
                activation,
            });
            input = output;
        }
        let artifact = PolicyArtifact {
            kind: POLICY_ARTIFACT_KIND.to_string(),
            schema_version: POLICY_ARTIFACT_SCHEMA_VERSION,
            name: name.into(),
            source: source.into(),
            observation_size: self.observation_size,
            action_size: self.action_size,
            layers,
            action_lower: self.action_lower.clone(),
            action_upper: self.action_upper.clone(),
        };
        artifact.validate()?;
        Ok(artifact)
    }
}

/// Training failure.
#[derive(Debug, thiserror::Error)]
pub enum CemTrainerError {
    /// The configuration was invalid.
    #[error("invalid CEM configuration: {0}")]
    InvalidConfig(String),
    /// The optimized dimension was zero.
    #[error("CEM dimension must be non-zero")]
    EmptyDimension,
    /// The parameter vector did not match the template.
    #[error("expected {expected} parameters, got {actual}")]
    ParameterCount {
        /// Parameters required by the template.
        expected: usize,
        /// Parameters provided.
        actual: usize,
    },
    /// The exported artifact was invalid.
    #[error("policy artifact export failed: {0}")]
    Artifact(#[from] PolicyArtifactError),
}

/// Runs the cross-entropy method on `fitness`, maximizing it.
///
/// `fitness` is called once per candidate per iteration in a fixed order with
/// the seeded sampler, so results are reproducible for a given `config.seed`.
pub fn cem_train<F>(
    mut fitness: F,
    dimension: usize,
    config: CemConfig,
) -> Result<CemResult, CemTrainerError>
where
    F: FnMut(&[f64]) -> f64,
{
    if dimension == 0 {
        return Err(CemTrainerError::EmptyDimension);
    }
    config.validate()?;

    let elite_count = ((config.population as f64 * config.elite_fraction).ceil() as usize)
        .clamp(1, config.population);
    let mut rng = DeterministicRng::new(config.seed);
    let mut mean = vec![0.0_f64; dimension];
    let mut std = vec![config.initial_std; dimension];
    let mut best_parameters = mean.clone();
    let mut best_fitness = f64::NEG_INFINITY;
    let mut elite_fitness_history = Vec::with_capacity(config.iterations);

    for _ in 0..config.iterations {
        let mut samples: Vec<(f64, Vec<f64>)> = Vec::with_capacity(config.population);
        for _ in 0..config.population {
            let candidate: Vec<f64> = mean
                .iter()
                .zip(std.iter())
                .map(|(center, scale)| center + scale * standard_normal(&mut rng))
                .collect();
            let value = fitness(&candidate);
            if value > best_fitness {
                best_fitness = value;
                best_parameters = candidate.clone();
            }
            samples.push((value, candidate));
        }
        samples.sort_by(|left, right| {
            right
                .0
                .total_cmp(&left.0)
                .then_with(|| compare_vectors(&left.1, &right.1))
        });

        let elite = &samples[..elite_count];
        for coordinate in 0..dimension {
            let center = elite
                .iter()
                .map(|(_, candidate)| candidate[coordinate])
                .sum::<f64>()
                / elite_count as f64;
            mean[coordinate] = center;
        }
        for coordinate in 0..dimension {
            let variance = elite
                .iter()
                .map(|(_, candidate)| (candidate[coordinate] - mean[coordinate]).powi(2))
                .sum::<f64>()
                / elite_count as f64;
            std[coordinate] = variance.sqrt().max(config.min_std);
        }
        elite_fitness_history
            .push(elite.iter().map(|(value, _)| *value).sum::<f64>() / elite_count as f64);
    }

    Ok(CemResult {
        best_parameters,
        best_fitness,
        elite_fitness_history,
        iterations: config.iterations,
    })
}

fn compare_vectors(left: &[f64], right: &[f64]) -> std::cmp::Ordering {
    for (a, b) in left.iter().zip(right.iter()) {
        let ordering = a.total_cmp(b);
        if ordering != std::cmp::Ordering::Equal {
            return ordering;
        }
    }
    left.len().cmp(&right.len())
}

fn standard_normal(rng: &mut DeterministicRng) -> f64 {
    // Box-Muller transform over two deterministic [0, 1) draws.
    let u1 = ((rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64).max(f64::MIN_POSITIVE);
    let u2 = (rng.next_u64() >> 11) as f64 / (1u64 << 53) as f64;
    (-2.0 * u1.ln()).sqrt() * (2.0 * std::f64::consts::PI * u2).cos()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn quadratic_target(config: CemConfig) -> CemResult {
        let target = [0.5, -1.0, 2.0, 0.25];
        cem_train(
            |parameters| {
                -parameters
                    .iter()
                    .zip(target.iter())
                    .map(|(value, target)| (value - target).powi(2))
                    .sum::<f64>()
            },
            target.len(),
            config,
        )
        .expect("train")
    }

    #[test]
    fn cem_converges_on_a_quadratic_objective() {
        let result = quadratic_target(CemConfig {
            population: 64,
            elite_fraction: 0.25,
            iterations: 200,
            initial_std: 1.0,
            min_std: 1e-4,
            seed: 7,
        });
        assert!(
            result.best_fitness > -1e-4,
            "fitness {}",
            result.best_fitness
        );
        assert_eq!(result.elite_fitness_history.len(), 200);
        // Elite fitness is monotonically improving on the first iteration.
        assert!(result.elite_fitness_history[199] >= result.elite_fitness_history[0]);
    }

    #[test]
    fn cem_is_deterministic_for_a_seed() {
        let config = CemConfig {
            population: 16,
            iterations: 25,
            seed: 123,
            ..CemConfig::default()
        };
        let first = quadratic_target(config);
        let second = quadratic_target(config);
        assert_eq!(first, second);
    }

    #[test]
    fn template_parameter_count_and_export() {
        let template = MlpPolicyTemplate {
            observation_size: 3,
            action_size: 2,
            hidden_sizes: vec![4, 4],
            hidden_activation: Activation::Tanh,
            action_lower: vec![-1.0, -1.0],
            action_upper: vec![1.0, 1.0],
        };
        // (3*4+4) + (4*4+4) + (4*2+2) = 16 + 20 + 10 = 46.
        assert_eq!(template.parameter_count(), 46);
        let parameters = vec![0.0; 46];
        let artifact = template
            .to_artifact("test", "cem", &parameters)
            .expect("export");
        assert_eq!(artifact.layers.len(), 3);
        assert_eq!(artifact.layers[0].input_size, 3);
        assert_eq!(artifact.layers[2].output_size, 2);
        assert_eq!(artifact.layers[2].activation, Activation::Identity);
        let action = artifact.evaluate(&[0.1, 0.2, 0.3]).expect("evaluate");
        assert_eq!(action, vec![0.0, 0.0]);

        assert!(matches!(
            template.to_artifact("test", "cem", &[0.0; 45]),
            Err(CemTrainerError::ParameterCount { .. })
        ));
    }

    #[test]
    fn invalid_configurations_are_rejected() {
        assert!(matches!(
            cem_train(|_| 0.0, 0, CemConfig::default()),
            Err(CemTrainerError::EmptyDimension)
        ));
        assert!(matches!(
            cem_train(
                |_| 0.0,
                2,
                CemConfig {
                    population: 5,
                    ..CemConfig::default()
                }
            ),
            Err(CemTrainerError::InvalidConfig(_))
        ));
        assert!(matches!(
            cem_train(
                |_| 0.0,
                2,
                CemConfig {
                    elite_fraction: 1.0,
                    ..CemConfig::default()
                }
            ),
            Err(CemTrainerError::InvalidConfig(_))
        ));
        assert!(matches!(
            cem_train(
                |_| 0.0,
                2,
                CemConfig {
                    min_std: 2.0,
                    ..CemConfig::default()
                }
            ),
            Err(CemTrainerError::InvalidConfig(_))
        ));
    }
}

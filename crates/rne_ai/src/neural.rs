//! Small differentiable dense networks with hand-written backpropagation.
//!
//! This is the native learning core the crate previously lacked: a deterministic
//! multilayer perceptron with an explicit backward pass, an Adam optimizer, and
//! export to a [`PolicyArtifact`]. There is no external ML dependency, so the
//! whole pipeline stays reproducible and headless. On-policy optimizers (PPO/SAC)
//! build on this module; this increment provides the differentiable network and
//! verifies its gradients against finite differences.

use crate::policy_artifact::{
    Activation, DenseLayer, PolicyArtifact, PolicyArtifactError, POLICY_ARTIFACT_KIND,
    POLICY_ARTIFACT_SCHEMA_VERSION,
};
use crate::rng::DeterministicRng;

/// Neural network construction or training failure.
#[derive(Debug, thiserror::Error)]
pub enum NeuralError {
    /// The layer sizes were degenerate.
    #[error("invalid network topology: {0}")]
    InvalidTopology(String),
    /// A parameter vector did not match the network.
    #[error("expected {expected} parameters, got {actual}")]
    ParameterCount {
        /// Parameters required by the network.
        expected: usize,
        /// Parameters provided.
        actual: usize,
    },
    /// Exporting a policy artifact failed.
    #[error("policy artifact export failed: {0}")]
    Artifact(#[from] PolicyArtifactError),
}

/// One dense layer with its activation.
#[derive(Clone, Debug, PartialEq)]
struct Layer {
    input_size: usize,
    output_size: usize,
    weights: Vec<f64>,
    biases: Vec<f64>,
    activation: Activation,
}

/// Per-layer gradients for dense-layer weights and biases.
#[derive(Clone, Debug, PartialEq)]
pub struct LayerGradient {
    /// Gradient with respect to the weights, row-major.
    pub weights: Vec<f64>,
    /// Gradient with respect to the biases.
    pub biases: Vec<f64>,
}

/// A deterministic dense multilayer perceptron.
#[derive(Clone, Debug, PartialEq)]
pub struct NeuralNet {
    layers: Vec<Layer>,
}

/// Forward activations cached for backpropagation.
#[derive(Clone, Debug)]
struct ForwardCache {
    /// Activation input to each layer (`activations[0]` is the network input).
    activations: Vec<Vec<f64>>,
    /// Pre-activation `z` for each layer.
    pre_activations: Vec<Vec<f64>>,
}

impl NeuralNet {
    /// Builds a network with the given sizes (input, hidden..., output).
    ///
    /// Hidden layers use `hidden_activation`; the output layer is linear.
    /// Weights are seeded uniformly in `[-1/sqrt(in), 1/sqrt(in)]`.
    pub fn new(
        sizes: &[usize],
        hidden_activation: Activation,
        seed: u64,
    ) -> Result<Self, NeuralError> {
        if sizes.len() < 2 || sizes.contains(&0) {
            return Err(NeuralError::InvalidTopology(
                "sizes must have at least an input and an output and be non-zero".into(),
            ));
        }
        let mut rng = DeterministicRng::new(seed);
        let mut layers = Vec::with_capacity(sizes.len() - 1);
        for pair in sizes.windows(2) {
            let (input_size, output_size) = (pair[0], pair[1]);
            let limit = 1.0 / (input_size as f64).sqrt();
            let weights = (0..input_size * output_size)
                .map(|_| rng.uniform_f64(-limit, limit))
                .collect();
            let biases = vec![0.0; output_size];
            layers.push(Layer {
                input_size,
                output_size,
                weights,
                biases,
                activation: if layers.len() + 1 == sizes.len() - 1 {
                    Activation::Identity
                } else {
                    hidden_activation
                },
            });
        }
        Ok(Self { layers })
    }

    /// Number of scalar parameters.
    pub fn parameter_count(&self) -> usize {
        self.layers
            .iter()
            .map(|layer| layer.weights.len() + layer.biases.len())
            .sum()
    }

    /// Flattens all parameters (weights then biases, layer by layer).
    pub fn parameters(&self) -> Vec<f64> {
        let mut out = Vec::with_capacity(self.parameter_count());
        for layer in &self.layers {
            out.extend_from_slice(&layer.weights);
            out.extend_from_slice(&layer.biases);
        }
        out
    }

    /// Overwrites all parameters from a flat vector.
    pub fn set_parameters(&mut self, parameters: &[f64]) -> Result<(), NeuralError> {
        if parameters.len() != self.parameter_count() {
            return Err(NeuralError::ParameterCount {
                expected: self.parameter_count(),
                actual: parameters.len(),
            });
        }
        let mut offset = 0;
        for layer in &mut self.layers {
            let weight_count = layer.weights.len();
            layer
                .weights
                .copy_from_slice(&parameters[offset..offset + weight_count]);
            offset += weight_count;
            let bias_count = layer.biases.len();
            layer
                .biases
                .copy_from_slice(&parameters[offset..offset + bias_count]);
            offset += bias_count;
        }
        Ok(())
    }

    /// Runs a forward pass and returns the output.
    pub fn forward(&self, input: &[f64]) -> Vec<f64> {
        let mut current = input.to_vec();
        for layer in &self.layers {
            current = layer_forward(layer, &current);
        }
        current
    }

    fn forward_with_cache(&self, input: &[f64]) -> ForwardCache {
        let mut activations = vec![input.to_vec()];
        let mut pre_activations = Vec::with_capacity(self.layers.len());
        let mut current = input.to_vec();
        for layer in &self.layers {
            let mut z = Vec::with_capacity(layer.output_size);
            for output in 0..layer.output_size {
                let row =
                    &layer.weights[output * layer.input_size..(output + 1) * layer.input_size];
                let mut sum = layer.biases[output];
                for (weight, value) in row.iter().zip(current.iter()) {
                    sum += weight * value;
                }
                z.push(sum);
            }
            let activated = z
                .iter()
                .map(|value| layer.activation.apply(*value))
                .collect::<Vec<_>>();
            pre_activations.push(z);
            activations.push(activated.clone());
            current = activated;
        }
        ForwardCache {
            activations,
            pre_activations,
        }
    }

    /// Backpropagates `grad_output` (dL/doutput) and returns per-layer gradients.
    #[allow(clippy::needless_range_loop)]
    pub fn backward(&self, input: &[f64], grad_output: &[f64]) -> Vec<LayerGradient> {
        let cache = self.forward_with_cache(input);
        let mut gradients = vec![
            LayerGradient {
                weights: Vec::new(),
                biases: Vec::new(),
            };
            self.layers.len()
        ];
        let mut grad = grad_output.to_vec();
        for index in (0..self.layers.len()).rev() {
            let layer = &self.layers[index];
            let previous = &cache.activations[index];
            // dL/dz = dL/da * f'(z).
            let mut grad_z = vec![0.0; layer.output_size];
            for output in 0..layer.output_size {
                let derivative = layer
                    .activation
                    .derivative(cache.pre_activations[index][output]);
                grad_z[output] = grad[output] * derivative;
            }
            // dW = dL/dz * a_prev^T, db = dL/dz.
            let mut weight_grad = vec![0.0; layer.weights.len()];
            for output in 0..layer.output_size {
                for input_index in 0..layer.input_size {
                    weight_grad[output * layer.input_size + input_index] =
                        grad_z[output] * previous[input_index];
                }
            }
            // dL/da_prev = W^T dL/dz.
            let mut grad_previous = vec![0.0; layer.input_size];
            for output in 0..layer.output_size {
                for input_index in 0..layer.input_size {
                    grad_previous[input_index] +=
                        layer.weights[output * layer.input_size + input_index] * grad_z[output];
                }
            }
            gradients[index] = LayerGradient {
                weights: weight_grad,
                biases: grad_z,
            };
            grad = grad_previous;
        }
        gradients
    }

    /// Applies gradients with plain stochastic gradient descent.
    pub fn apply_sgd(&mut self, gradients: &[LayerGradient], learning_rate: f64) {
        for (layer, gradient) in self.layers.iter_mut().zip(gradients) {
            for (weight, grad) in layer.weights.iter_mut().zip(&gradient.weights) {
                *weight -= learning_rate * grad;
            }
            for (bias, grad) in layer.biases.iter_mut().zip(&gradient.biases) {
                *bias -= learning_rate * grad;
            }
        }
    }

    /// Exports the network as a linear-output [`PolicyArtifact`].
    pub fn to_policy_artifact(
        &self,
        name: impl Into<String>,
        source: impl Into<String>,
        action_lower: Vec<f64>,
        action_upper: Vec<f64>,
    ) -> Result<PolicyArtifact, NeuralError> {
        let observation_size = self.layers.first().map_or(0, |layer| layer.input_size);
        let action_size = self.layers.last().map_or(0, |layer| layer.output_size);
        let layers = self
            .layers
            .iter()
            .map(|layer| DenseLayer {
                input_size: layer.input_size as u32,
                output_size: layer.output_size as u32,
                weights: layer.weights.clone(),
                biases: layer.biases.clone(),
                activation: layer.activation,
            })
            .collect();
        let artifact = PolicyArtifact {
            kind: POLICY_ARTIFACT_KIND.to_string(),
            schema_version: POLICY_ARTIFACT_SCHEMA_VERSION,
            name: name.into(),
            source: source.into(),
            observation_size: observation_size as u32,
            action_size: action_size as u32,
            layers,
            action_lower,
            action_upper,
        };
        artifact.validate()?;
        Ok(artifact)
    }
}

fn layer_forward(layer: &Layer, input: &[f64]) -> Vec<f64> {
    let mut out = Vec::with_capacity(layer.output_size);
    for output in 0..layer.output_size {
        let row = &layer.weights[output * layer.input_size..(output + 1) * layer.input_size];
        let mut sum = layer.biases[output];
        for (weight, value) in row.iter().zip(input.iter()) {
            sum += weight * value;
        }
        out.push(layer.activation.apply(sum));
    }
    out
}

impl Activation {
    fn derivative(self, pre_activation: f64) -> f64 {
        match self {
            Self::Identity => 1.0,
            Self::Tanh => {
                let tanh = pre_activation.tanh();
                1.0 - tanh * tanh
            }
            Self::Relu => {
                if pre_activation > 0.0 {
                    1.0
                } else {
                    0.0
                }
            }
            Self::Sigmoid => {
                let sigmoid = 1.0 / (1.0 + (-pre_activation).exp());
                sigmoid * (1.0 - sigmoid)
            }
        }
    }
}

/// Adam optimizer state for a [`NeuralNet`].
#[derive(Clone, Debug)]
pub struct Adam {
    first_moment: Vec<f64>,
    second_moment: Vec<f64>,
    step: u64,
    learning_rate: f64,
    beta1: f64,
    beta2: f64,
    epsilon: f64,
}

impl Adam {
    /// Creates Adam state matching `net`'s parameter count.
    pub fn new(net: &NeuralNet, learning_rate: f64) -> Self {
        Self::with_count(net.parameter_count(), learning_rate)
    }

    /// Creates Adam state for an arbitrary parameter vector length.
    pub fn with_count(count: usize, learning_rate: f64) -> Self {
        Self {
            first_moment: vec![0.0; count],
            second_moment: vec![0.0; count],
            step: 0,
            learning_rate,
            beta1: 0.9,
            beta2: 0.999,
            epsilon: 1.0e-8,
        }
    }

    /// Applies one Adam update from flat gradients.
    pub fn step(&mut self, net: &mut NeuralNet, gradients: &[f64]) {
        let mut parameters = net.parameters();
        self.step_slice(&mut parameters, gradients);
        let _ = net.set_parameters(&parameters);
    }

    /// Applies one Adam update in place to a parameter slice.
    pub fn step_slice(&mut self, values: &mut [f64], gradients: &[f64]) {
        self.step += 1;
        let bias_correction1 = 1.0 - self.beta1.powi(self.step as i32);
        let bias_correction2 = 1.0 - self.beta2.powi(self.step as i32);
        for (index, gradient) in gradients.iter().enumerate() {
            self.first_moment[index] =
                self.beta1 * self.first_moment[index] + (1.0 - self.beta1) * gradient;
            self.second_moment[index] =
                self.beta2 * self.second_moment[index] + (1.0 - self.beta2) * gradient * gradient;
            let m_hat = self.first_moment[index] / bias_correction1;
            let v_hat = self.second_moment[index] / bias_correction2;
            values[index] -= self.learning_rate * m_hat / (v_hat.sqrt() + self.epsilon);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn mean_squared_error(net: &NeuralNet, input: &[f64], target: &[f64]) -> f64 {
        net.forward(input)
            .iter()
            .zip(target)
            .map(|(out, target)| (out - target).powi(2))
            .sum::<f64>()
            / target.len() as f64
    }

    fn flat_gradients(gradients: &[LayerGradient]) -> Vec<f64> {
        let mut out = Vec::new();
        for gradient in gradients {
            out.extend_from_slice(&gradient.weights);
            out.extend_from_slice(&gradient.biases);
        }
        out
    }

    #[test]
    fn gradients_match_finite_difference() {
        let net = NeuralNet::new(&[3, 5, 2], Activation::Tanh, 7).expect("net");
        let input = [0.3, -0.7, 0.2];
        let target = [0.5, -0.4];
        let output = net.forward(&input);
        let count = target.len() as f64;
        let grad_output: Vec<f64> = output
            .iter()
            .zip(&target)
            .map(|(out, target)| 2.0 * (out - target) / count)
            .collect();
        let analytic = flat_gradients(&net.backward(&input, &grad_output));

        let base = net.parameters();
        let epsilon = 1.0e-6;
        for index in 0..base.len() {
            let mut plus = base.clone();
            plus[index] += epsilon;
            let mut minus = base.clone();
            minus[index] -= epsilon;
            let mut plus_net = net.clone();
            let mut minus_net = net.clone();
            plus_net.set_parameters(&plus).unwrap();
            minus_net.set_parameters(&minus).unwrap();
            let numeric = (mean_squared_error(&plus_net, &input, &target)
                - mean_squared_error(&minus_net, &input, &target))
                / (2.0 * epsilon);
            assert!(
                (analytic[index] - numeric).abs() < 1.0e-6,
                "param {index}: analytic {} numeric {}",
                analytic[index],
                numeric
            );
        }
    }

    #[test]
    fn adam_reduces_regression_loss() {
        let mut net = NeuralNet::new(&[1, 8, 1], Activation::Tanh, 42).expect("net");
        let mut adam = Adam::new(&net, 0.05);
        let samples: Vec<(f64, f64)> = (-10..=10)
            .map(|i| {
                let x = i as f64 * 0.1;
                (x, 2.0 * x + 1.0)
            })
            .collect();
        let loss_before = samples
            .iter()
            .map(|(x, y)| mean_squared_error(&net, &[*x], &[*y]))
            .sum::<f64>()
            / samples.len() as f64;
        for _ in 0..500 {
            let mut gradients = vec![0.0; net.parameter_count()];
            for (x, y) in &samples {
                let output = net.forward(&[*x]);
                let grad_output = vec![2.0 * (output[0] - y) / samples.len() as f64];
                for (accumulated, grad) in gradients
                    .iter_mut()
                    .zip(flat_gradients(&net.backward(&[*x], &grad_output)))
                {
                    *accumulated += grad;
                }
            }
            adam.step(&mut net, &gradients);
        }
        let loss_after = samples
            .iter()
            .map(|(x, y)| mean_squared_error(&net, &[*x], &[*y]))
            .sum::<f64>()
            / samples.len() as f64;
        assert!(
            loss_after < loss_before * 0.05,
            "loss {loss_before} -> {loss_after}"
        );
    }

    #[test]
    fn export_matches_forward() {
        let net = NeuralNet::new(&[2, 4, 1], Activation::Relu, 3).expect("net");
        let artifact = net
            .to_policy_artifact("net", "test", vec![-1.0], vec![1.0])
            .expect("export");
        for input in [[0.1, -0.2], [1.0, 1.0], [-0.5, 0.7]] {
            let expected = net.forward(&input);
            let actual = artifact.evaluate(&input).expect("evaluate");
            assert_eq!(expected, actual);
        }
    }

    #[test]
    fn parameters_round_trip() {
        let mut net = NeuralNet::new(&[3, 4, 2], Activation::Tanh, 9).expect("net");
        let parameters = net.parameters();
        net.set_parameters(&parameters).expect("set");
        assert_eq!(net.parameters(), parameters);
        assert!(matches!(
            net.set_parameters(&[0.0]),
            Err(NeuralError::ParameterCount { .. })
        ));
    }
}

//! Versioned, backend-neutral learned-policy artifacts.
//!
//! Training in this workspace (dependency-free CEM in examples, or external
//! PPO) previously froze its winning weights as Rust constants. This module
//! defines a small, deterministic artifact (`.rne.policy.json`) so learned
//! policies are loadable data instead of pinned source constants.
//!
//! The format is a dense feed-forward network with explicit per-layer
//! activations and output clamps. Inference is plain `f64` arithmetic with a
//! fixed evaluation order, so replaying the same artifact and observation
//! always yields bit-identical actions.

use crate::action::DiffDriveAction;
use crate::env::DiffDriveEpisode;
use crate::observation::DiffDriveObservation;
use crate::policy::{LocomotionPolicy, Policy};
use serde::{Deserialize, Serialize};
use std::path::Path;

/// Stable kind identifier for learned-policy artifacts.
pub const POLICY_ARTIFACT_KIND: &str = "rne_policy";

/// Learned-policy artifact schema version.
pub const POLICY_ARTIFACT_SCHEMA_VERSION: u16 = 1;

/// Fixed observation vector width for [`DiffDriveArtifactPolicy`].
pub const DIFF_DRIVE_OBSERVATION_WIDTH: usize = 12;

/// Fixed action vector width for [`DiffDriveArtifactPolicy`].
pub const DIFF_DRIVE_ACTION_WIDTH: usize = 2;

/// Per-layer activation function.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Activation {
    /// Linear passthrough.
    Identity,
    /// Hyperbolic tangent.
    Tanh,
    /// Rectified linear unit.
    Relu,
    /// Logistic sigmoid.
    Sigmoid,
}

impl Activation {
    pub(crate) fn apply(self, value: f64) -> f64 {
        match self {
            Self::Identity => value,
            Self::Tanh => value.tanh(),
            Self::Relu => value.max(0.0),
            Self::Sigmoid => 1.0 / (1.0 + (-value).exp()),
        }
    }
}

/// One fully connected layer, row-major `output_size x input_size` weights.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct DenseLayer {
    /// Number of inputs.
    pub input_size: u32,
    /// Number of outputs.
    pub output_size: u32,
    /// Row-major weights, length `input_size * output_size`.
    pub weights: Vec<f64>,
    /// Bias per output, length `output_size`.
    pub biases: Vec<f64>,
    /// Activation applied after the affine transform.
    pub activation: Activation,
}

/// A versioned dense learned policy.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyArtifact {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u16,
    /// Human-readable policy name.
    pub name: String,
    /// Provenance of the weights (for example `cem`, `ppo`, `linear`).
    pub source: String,
    /// Observation vector width.
    pub observation_size: u32,
    /// Action vector width.
    pub action_size: u32,
    /// Ordered dense layers.
    pub layers: Vec<DenseLayer>,
    /// Inclusive lower action clamp per output, length `action_size`.
    pub action_lower: Vec<f64>,
    /// Inclusive upper action clamp per output, length `action_size`.
    pub action_upper: Vec<f64>,
}

/// Learned-policy artifact error.
#[derive(Debug, thiserror::Error)]
pub enum PolicyArtifactError {
    /// Artifact kind or schema version did not match.
    #[error("invalid policy artifact: {0}")]
    InvalidArtifact(String),
    /// A layer, weight, bias, or bound was malformed.
    #[error("invalid policy layer: {0}")]
    InvalidLayer(String),
    /// The observation width did not match the artifact.
    #[error("expected observation of width {expected}, got {actual}")]
    ObservationSize {
        /// Expected width.
        expected: usize,
        /// Provided width.
        actual: usize,
    },
    /// Reading or writing an artifact failed.
    #[error("policy artifact I/O failed: {0}")]
    Io(#[from] std::io::Error),
    /// JSON encoding or decoding failed.
    #[error("policy artifact JSON failed: {0}")]
    Json(#[from] serde_json::Error),
}

impl PolicyArtifact {
    /// Builds a single-layer linear policy with identity activation.
    pub fn linear(name: impl Into<String>, weights: Vec<f64>, biases: Vec<f64>) -> Self {
        let output_size = biases.len() as u32;
        let input_size = if output_size == 0 {
            0
        } else {
            (weights.len() / output_size as usize) as u32
        };
        Self {
            kind: POLICY_ARTIFACT_KIND.to_string(),
            schema_version: POLICY_ARTIFACT_SCHEMA_VERSION,
            name: name.into(),
            source: "linear".to_string(),
            observation_size: input_size,
            action_size: output_size,
            layers: vec![DenseLayer {
                input_size,
                output_size,
                weights,
                biases,
                activation: Activation::Identity,
            }],
            action_lower: vec![f64::NEG_INFINITY; output_size as usize],
            action_upper: vec![f64::INFINITY; output_size as usize],
        }
    }

    /// Validates the artifact's shape, weights, and bounds.
    pub fn validate(&self) -> Result<(), PolicyArtifactError> {
        if self.kind != POLICY_ARTIFACT_KIND {
            return Err(PolicyArtifactError::InvalidArtifact("kind mismatch".into()));
        }
        if self.schema_version != POLICY_ARTIFACT_SCHEMA_VERSION {
            return Err(PolicyArtifactError::InvalidArtifact(
                "schema version mismatch".into(),
            ));
        }
        if self.observation_size == 0 || self.action_size == 0 {
            return Err(PolicyArtifactError::InvalidArtifact(
                "observation and action sizes must be non-zero".into(),
            ));
        }
        if self.layers.is_empty() {
            return Err(PolicyArtifactError::InvalidArtifact(
                "artifact has no layers".into(),
            ));
        }
        let mut expected_input = self.observation_size;
        for (index, layer) in self.layers.iter().enumerate() {
            if layer.input_size != expected_input {
                return Err(PolicyArtifactError::InvalidLayer(format!(
                    "layer {index} input {expected_input} does not match declared {}",
                    layer.input_size
                )));
            }
            if layer.output_size == 0 {
                return Err(PolicyArtifactError::InvalidLayer(format!(
                    "layer {index} has zero outputs"
                )));
            }
            let expected_weights = layer.input_size as usize * layer.output_size as usize;
            if layer.weights.len() != expected_weights {
                return Err(PolicyArtifactError::InvalidLayer(format!(
                    "layer {index} has {} weights, expected {expected_weights}",
                    layer.weights.len()
                )));
            }
            if layer.biases.len() != layer.output_size as usize {
                return Err(PolicyArtifactError::InvalidLayer(format!(
                    "layer {index} has {} biases, expected {}",
                    layer.biases.len(),
                    layer.output_size
                )));
            }
            if layer
                .weights
                .iter()
                .chain(layer.biases.iter())
                .any(|value| !value.is_finite())
            {
                return Err(PolicyArtifactError::InvalidLayer(format!(
                    "layer {index} contains non-finite values"
                )));
            }
            expected_input = layer.output_size;
        }
        if expected_input != self.action_size {
            return Err(PolicyArtifactError::InvalidArtifact(format!(
                "output width {expected_input} does not match action size {}",
                self.action_size
            )));
        }
        let action_size = self.action_size as usize;
        if self.action_lower.len() != action_size || self.action_upper.len() != action_size {
            return Err(PolicyArtifactError::InvalidArtifact(
                "action bounds must match action size".into(),
            ));
        }
        for index in 0..action_size {
            let (lower, upper) = (self.action_lower[index], self.action_upper[index]);
            if lower.is_nan() || upper.is_nan() || lower > upper {
                return Err(PolicyArtifactError::InvalidArtifact(format!(
                    "action bound {index} is invalid ({lower}..{upper})"
                )));
            }
        }
        Ok(())
    }

    /// Runs a deterministic forward pass and clamps the action.
    pub fn evaluate(&self, observation: &[f64]) -> Result<Vec<f64>, PolicyArtifactError> {
        self.validate()?;
        if observation.len() != self.observation_size as usize {
            return Err(PolicyArtifactError::ObservationSize {
                expected: self.observation_size as usize,
                actual: observation.len(),
            });
        }
        let mut current = observation.to_vec();
        for layer in &self.layers {
            let outputs = layer.output_size as usize;
            let inputs = layer.input_size as usize;
            let mut next = Vec::with_capacity(outputs);
            for output in 0..outputs {
                let row = &layer.weights[output * inputs..(output + 1) * inputs];
                let mut sum = layer.biases[output];
                for (weight, value) in row.iter().zip(current.iter()) {
                    sum += weight * value;
                }
                next.push(layer.activation.apply(sum));
            }
            current = next;
        }
        for (value, (lower, upper)) in current
            .iter_mut()
            .zip(self.action_lower.iter().zip(self.action_upper.iter()))
        {
            *value = value.clamp(*lower, *upper);
        }
        Ok(current)
    }

    /// Serializes the artifact as stable pretty JSON with a trailing newline.
    pub fn to_json_pretty(&self) -> Result<String, PolicyArtifactError> {
        self.validate()?;
        let mut json = serde_json::to_string_pretty(self)?;
        json.push('\n');
        Ok(json)
    }

    /// Writes the artifact to `path`, validating first.
    pub fn save(&self, path: &Path) -> Result<(), PolicyArtifactError> {
        std::fs::write(path, self.to_json_pretty()?)?;
        Ok(())
    }

    /// Reads and validates an artifact from `path`.
    pub fn load(path: &Path) -> Result<Self, PolicyArtifactError> {
        let bytes = std::fs::read(path)?;
        let artifact: Self = serde_json::from_slice(&bytes)?;
        artifact.validate()?;
        Ok(artifact)
    }
}

/// Encodes a diff-drive observation into the fixed [`DIFF_DRIVE_OBSERVATION_WIDTH`] vector.
pub fn diff_drive_observation_vector(observation: &DiffDriveObservation) -> Vec<f64> {
    vec![
        observation.base_x_m,
        observation.base_y_m,
        observation.base_z_m,
        observation.base_yaw_rad,
        observation.left_wheel_velocity_rad_s,
        observation.right_wheel_velocity_rad_s,
        observation.imu_ay_m_s2,
        observation.lidar_points as f64,
        observation.goal_delta_x_m.unwrap_or(0.0),
        observation.peer_delta_x_m.unwrap_or(0.0),
        observation.peer_delta_z_m.unwrap_or(0.0),
        observation.peer_separation_m.unwrap_or(0.0),
    ]
}

/// A learned diff-drive policy driven by a [`PolicyArtifact`].
#[derive(Clone, Debug, PartialEq)]
pub struct DiffDriveArtifactPolicy {
    artifact: PolicyArtifact,
}

impl DiffDriveArtifactPolicy {
    /// Wraps a validated artifact. Returns an error when it is malformed or has
    /// the wrong observation/action width.
    pub fn new(artifact: PolicyArtifact) -> Result<Self, PolicyArtifactError> {
        artifact.validate()?;
        if artifact.observation_size as usize != DIFF_DRIVE_OBSERVATION_WIDTH
            || artifact.action_size as usize != DIFF_DRIVE_ACTION_WIDTH
        {
            return Err(PolicyArtifactError::InvalidArtifact(format!(
                "diff-drive policy must map {DIFF_DRIVE_OBSERVATION_WIDTH} observations to {DIFF_DRIVE_ACTION_WIDTH} actions"
            )));
        }
        Ok(Self { artifact })
    }

    /// Returns the wrapped artifact.
    pub fn artifact(&self) -> &PolicyArtifact {
        &self.artifact
    }

    /// Evaluates the artifact, falling back to zero action on shape error.
    pub fn act(&self, observation: &DiffDriveObservation) -> DiffDriveAction {
        let vector = diff_drive_observation_vector(observation);
        match self.artifact.evaluate(&vector) {
            Ok(action) => DiffDriveAction {
                left_velocity_rad_s: action[0],
                right_velocity_rad_s: action[1],
            },
            Err(_) => DiffDriveAction {
                left_velocity_rad_s: 0.0,
                right_velocity_rad_s: 0.0,
            },
        }
    }
}

impl LocomotionPolicy for DiffDriveArtifactPolicy {
    type Observation = DiffDriveObservation;
    type Action = DiffDriveAction;

    fn act(&mut self, observation: &Self::Observation) -> Self::Action {
        DiffDriveArtifactPolicy::act(self, observation)
    }
}

impl Policy<DiffDriveEpisode> for DiffDriveArtifactPolicy {
    fn act(&mut self, observation: &DiffDriveObservation) -> DiffDriveAction {
        DiffDriveArtifactPolicy::act(self, observation)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn linear_gain_artifact() -> PolicyArtifact {
        // Two outputs from two inputs: forward gain 0.5, turn gain 1.5.
        PolicyArtifact {
            kind: POLICY_ARTIFACT_KIND.to_string(),
            schema_version: POLICY_ARTIFACT_SCHEMA_VERSION,
            name: "test".to_string(),
            source: "unit".to_string(),
            observation_size: 2,
            action_size: 2,
            layers: vec![DenseLayer {
                input_size: 2,
                output_size: 2,
                weights: vec![0.5, 0.0, 0.0, 1.5],
                biases: vec![0.0, 0.0],
                activation: Activation::Identity,
            }],
            action_lower: vec![-1.0, -1.0],
            action_upper: vec![1.0, 1.0],
        }
    }

    #[test]
    fn forward_pass_clamps_and_is_deterministic() {
        let artifact = linear_gain_artifact();
        let first = artifact.evaluate(&[2.0, -2.0]).unwrap();
        let second = artifact.evaluate(&[2.0, -2.0]).unwrap();
        assert_eq!(first, second);
        assert_eq!(first, vec![1.0, -1.0]);
    }

    #[test]
    fn tanh_layer_is_bounded() {
        let artifact = PolicyArtifact {
            kind: POLICY_ARTIFACT_KIND.to_string(),
            schema_version: POLICY_ARTIFACT_SCHEMA_VERSION,
            name: "tanh".to_string(),
            source: "unit".to_string(),
            observation_size: 1,
            action_size: 1,
            layers: vec![DenseLayer {
                input_size: 1,
                output_size: 1,
                weights: vec![100.0],
                biases: vec![0.0],
                activation: Activation::Tanh,
            }],
            action_lower: vec![f64::NEG_INFINITY],
            action_upper: vec![f64::INFINITY],
        };
        let action = artifact.evaluate(&[1.0]).unwrap();
        assert!((action[0] - 1.0).abs() < 1e-12);
    }

    #[test]
    fn malformed_artifacts_are_rejected() {
        let mut wrong_weights = linear_gain_artifact();
        wrong_weights.layers[0].weights.pop();
        assert!(matches!(
            wrong_weights.validate(),
            Err(PolicyArtifactError::InvalidLayer(_))
        ));

        let mut wrong_bounds = linear_gain_artifact();
        wrong_bounds.action_lower.pop();
        assert!(matches!(
            wrong_bounds.validate(),
            Err(PolicyArtifactError::InvalidArtifact(_))
        ));

        let mut inverted = linear_gain_artifact();
        inverted.action_lower[0] = 2.0;
        inverted.action_upper[0] = 1.0;
        assert!(matches!(
            inverted.validate(),
            Err(PolicyArtifactError::InvalidArtifact(_))
        ));

        assert!(matches!(
            linear_gain_artifact().evaluate(&[1.0]),
            Err(PolicyArtifactError::ObservationSize { .. })
        ));
    }

    #[test]
    fn artifact_round_trips_through_json() {
        let artifact = linear_gain_artifact();
        let json = artifact.to_json_pretty().unwrap();
        let decoded: PolicyArtifact = serde_json::from_str(&json).unwrap();
        assert_eq!(decoded, artifact);
    }

    #[test]
    fn diff_drive_policy_uses_the_fixed_encoding() {
        let artifact = PolicyArtifact {
            kind: POLICY_ARTIFACT_KIND.to_string(),
            schema_version: POLICY_ARTIFACT_SCHEMA_VERSION,
            name: "diff_drive".to_string(),
            source: "unit".to_string(),
            observation_size: DIFF_DRIVE_OBSERVATION_WIDTH as u32,
            action_size: DIFF_DRIVE_ACTION_WIDTH as u32,
            layers: vec![DenseLayer {
                input_size: DIFF_DRIVE_OBSERVATION_WIDTH as u32,
                output_size: DIFF_DRIVE_ACTION_WIDTH as u32,
                // Row 0: forward is +10 * goal_delta_x. Row 1: negative.
                weights: {
                    let mut weights = vec![0.0; DIFF_DRIVE_OBSERVATION_WIDTH * 2];
                    weights[8] = 10.0;
                    weights[DIFF_DRIVE_OBSERVATION_WIDTH + 8] = -10.0;
                    weights
                },
                biases: vec![0.0, 0.0],
                activation: Activation::Identity,
            }],
            action_lower: vec![f64::NEG_INFINITY; 2],
            action_upper: vec![f64::INFINITY; 2],
        };
        let policy = DiffDriveArtifactPolicy::new(artifact).expect("policy");
        let observation = DiffDriveObservation {
            goal_delta_x_m: Some(0.2),
            ..DiffDriveObservation::default()
        };
        let action = policy.act(&observation);
        assert!((action.left_velocity_rad_s - 2.0).abs() < 1e-12);
        assert!((action.right_velocity_rad_s + 2.0).abs() < 1e-12);
    }

    #[test]
    fn diff_drive_policy_rejects_wrong_width() {
        let artifact = linear_gain_artifact();
        assert!(DiffDriveArtifactPolicy::new(artifact).is_err());
    }
}

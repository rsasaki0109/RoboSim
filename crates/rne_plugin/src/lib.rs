//! Plugin manifest and controller-plugin boundary for Robot Native Engine.
//!
//! The controller-plugin boundary separates the fixed-step runner from policy
//! implementations: a [`ControllerPlugin`] maps an observed joint state to
//! velocity commands, and the runner invokes it through the trait rather than
//! inlining the policy. A [`PluginManifest`] names and classifies a plugin for
//! discovery, [`VelocityServoController`] is a built-in reference
//! implementation, and [`cabi::load_controller_library`] loads controller
//! plugins from shared libraries through a versioned C ABI.

#![deny(missing_docs)]

use serde::{Deserialize, Serialize};

pub mod cabi;

pub use cabi::{
    load_controller_library, LoadedControllerPlugin, PluginLoadError, RneJointPosition,
    RneJointVelocity, RNE_PLUGIN_ABI_VERSION,
};

/// Plugin kind used for discovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PluginKind {
    /// A controller that maps observations to actuator commands.
    Controller,
}

/// Versioned plugin manifest.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PluginManifest {
    /// Plugin name used to select the implementation.
    pub name: String,
    /// Plugin kind.
    pub kind: PluginKind,
}

impl PluginManifest {
    /// Creates a controller plugin manifest.
    pub fn controller(name: impl Into<String>) -> Self {
        Self {
            name: name.into(),
            kind: PluginKind::Controller,
        }
    }

    /// Validates the manifest invariants.
    pub fn validate(&self) -> Result<(), PluginError> {
        if self.name.trim().is_empty() {
            return Err(PluginError::Invalid(
                "plugin name must not be empty".to_string(),
            ));
        }
        Ok(())
    }

    /// Serializes a validated manifest as pretty JSON.
    pub fn to_json(&self) -> Result<String, PluginError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }
}

/// Plugin manifest validation failure.
#[derive(Debug, thiserror::Error)]
pub enum PluginError {
    /// The manifest is invalid.
    #[error("invalid plugin manifest: {0}")]
    Invalid(String),
    /// The manifest could not be serialized.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

/// Boundary implemented by controller plugins.
///
/// The runner calls [`ControllerPlugin::joint_velocity_commands`] once per
/// fixed step with the observed joint positions and expects velocity commands
/// in return. Implementations must be deterministic for a given observation.
pub trait ControllerPlugin: Send + Sync + std::fmt::Debug {
    /// Plugin name reported to the runner.
    fn name(&self) -> &str;

    /// Computes joint velocity commands from the observed joint positions.
    ///
    /// `joint_names` and `positions_rad` are parallel arrays. Returned commands
    /// are `(joint name, velocity rad/s)` pairs; unknown names are ignored by
    /// the runner.
    fn joint_velocity_commands(
        &self,
        joint_names: &[&str],
        positions_rad: &[f64],
    ) -> Vec<(String, f64)>;
}

/// Reference controller plugin that drives one joint toward a target angle.
///
/// The commanded velocity is a proportional error term, `gain * (target - position)`,
/// clamped to the configured maximum. This is a state-dependent policy: the same
/// command only holds while the joint is away from the target.
#[derive(Clone, Debug, PartialEq)]
pub struct VelocityServoController {
    /// Plugin name.
    name: String,
    /// Joint to drive.
    pub joint: String,
    /// Target joint angle in radians.
    pub target_rad: f64,
    /// Proportional gain.
    pub gain: f64,
    /// Maximum commanded velocity in radians per second.
    pub max_velocity_rad_s: f64,
}

impl VelocityServoController {
    /// Creates a velocity-servo controller.
    pub fn new(
        name: impl Into<String>,
        joint: impl Into<String>,
        target_rad: f64,
        gain: f64,
        max_velocity_rad_s: f64,
    ) -> Result<Self, PluginError> {
        let controller = Self {
            name: name.into(),
            joint: joint.into(),
            target_rad,
            gain,
            max_velocity_rad_s,
        };
        if !target_rad.is_finite() || !gain.is_finite() || !max_velocity_rad_s.is_finite() {
            return Err(PluginError::Invalid(
                "velocity servo parameters must be finite".to_string(),
            ));
        }
        if gain < 0.0 || max_velocity_rad_s < 0.0 {
            return Err(PluginError::Invalid(
                "velocity servo gain and max velocity must be non-negative".to_string(),
            ));
        }
        Ok(controller)
    }
}

impl ControllerPlugin for VelocityServoController {
    fn name(&self) -> &str {
        &self.name
    }

    fn joint_velocity_commands(
        &self,
        joint_names: &[&str],
        positions_rad: &[f64],
    ) -> Vec<(String, f64)> {
        for (index, name) in joint_names.iter().enumerate() {
            if *name == self.joint {
                let position = positions_rad.get(index).copied().unwrap_or(0.0);
                let error = self.target_rad - position;
                let velocity =
                    (self.gain * error).clamp(-self.max_velocity_rad_s, self.max_velocity_rad_s);
                return vec![(self.joint.clone(), velocity)];
            }
        }
        Vec::new()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn controller_manifest_roundtrips() {
        let manifest = PluginManifest::controller("velocity_servo");
        let json = manifest.to_json().expect("serialize");
        let loaded: PluginManifest = serde_json::from_str(&json).expect("parse");
        assert_eq!(loaded, manifest);
        assert!(json.contains("\"kind\": \"controller\""));
    }

    #[test]
    fn velocity_servo_drives_toward_target_and_stops() {
        let controller =
            VelocityServoController::new("velocity_servo", "shoulder_joint", 1.0, 2.0, 5.0)
                .expect("controller");

        let near = controller.joint_velocity_commands(&["shoulder_joint"], &[0.25]);
        assert_eq!(near, vec![("shoulder_joint".to_string(), 1.5)]);

        let far = controller.joint_velocity_commands(&["shoulder_joint"], &[-10.0]);
        assert_eq!(far, vec![("shoulder_joint".to_string(), 5.0)]);

        let at_target = controller.joint_velocity_commands(&["shoulder_joint"], &[1.0]);
        assert_eq!(at_target, vec![("shoulder_joint".to_string(), 0.0)]);

        let unknown = controller.joint_velocity_commands(&["other_joint"], &[0.0]);
        assert!(unknown.is_empty());
    }
}

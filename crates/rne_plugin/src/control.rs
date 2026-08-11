//! Versioned, backend-neutral controller observation and action schemas.

use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Current robot-native controller observation/action schema version.
pub const CONTROLLER_SCHEMA_VERSION: u32 = 1;

/// One named joint observation exposed to a controller.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerJointObservation {
    /// Stable joint name.
    pub name: String,
    /// Joint position in radians.
    pub position_rad: f64,
    /// Joint velocity in radians per second when the source provides it.
    pub velocity_rad_s: Option<f64>,
}

impl ControllerJointObservation {
    /// Creates one position-only joint observation.
    pub fn position(name: impl Into<String>, position_rad: f64) -> Self {
        Self {
            name: name.into(),
            position_rad,
            velocity_rad_s: None,
        }
    }

    /// Creates one joint observation with position and velocity.
    pub fn position_velocity(
        name: impl Into<String>,
        position_rad: f64,
        velocity_rad_s: f64,
    ) -> Self {
        Self {
            name: name.into(),
            position_rad,
            velocity_rad_s: Some(velocity_rad_s),
        }
    }

    fn validate(&self, robot_id: &str) -> Result<(), ControllerSchemaError> {
        validate_identifier("joint name", &self.name)?;
        if !self.position_rad.is_finite() {
            return Err(ControllerSchemaError::Invalid(format!(
                "robot `{robot_id}` joint `{}` position_rad must be finite",
                self.name
            )));
        }
        if self.velocity_rad_s.is_some_and(|value| !value.is_finite()) {
            return Err(ControllerSchemaError::Invalid(format!(
                "robot `{robot_id}` joint `{}` velocity_rad_s must be finite when present",
                self.name
            )));
        }
        Ok(())
    }
}

/// Deterministically ordered observations for one robot.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerRobotObservation {
    /// Stable robot identity, independent of ECS entity allocation.
    pub robot_id: String,
    /// Joint observations sorted by joint name.
    pub joints: Vec<ControllerJointObservation>,
}

impl ControllerRobotObservation {
    /// Creates and validates one robot observation, sorting joints by name.
    pub fn new(
        robot_id: impl Into<String>,
        mut joints: Vec<ControllerJointObservation>,
    ) -> Result<Self, ControllerSchemaError> {
        joints.sort_by(|left, right| left.name.cmp(&right.name));
        let observation = Self {
            robot_id: robot_id.into(),
            joints,
        };
        observation.validate()?;
        Ok(observation)
    }

    /// Returns one joint observation by stable name.
    pub fn joint(&self, name: &str) -> Option<&ControllerJointObservation> {
        self.joints
            .binary_search_by(|joint| joint.name.as_str().cmp(name))
            .ok()
            .map(|index| &self.joints[index])
    }

    fn validate(&self) -> Result<(), ControllerSchemaError> {
        validate_identifier("robot_id", &self.robot_id)?;
        validate_strict_order(
            self.joints.iter().map(|joint| joint.name.as_str()),
            &format!("robot `{}` joint names", self.robot_id),
        )?;
        for joint in &self.joints {
            joint.validate(&self.robot_id)?;
        }
        Ok(())
    }
}

/// Versioned observation frame passed to a controller at one fixed step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerObservationFrame {
    /// Observation/action schema version.
    pub schema_version: u32,
    /// Zero-based fixed simulation step.
    pub step: u64,
    /// Simulation timestamp represented as stable integer ticks.
    pub sim_time_ticks: u64,
    /// Robot observations sorted by stable robot ID.
    pub robots: Vec<ControllerRobotObservation>,
}

impl ControllerObservationFrame {
    /// Creates and validates one frame, sorting robots by stable ID.
    pub fn new(
        step: u64,
        sim_time_ticks: u64,
        mut robots: Vec<ControllerRobotObservation>,
    ) -> Result<Self, ControllerSchemaError> {
        robots.sort_by(|left, right| left.robot_id.cmp(&right.robot_id));
        let frame = Self {
            schema_version: CONTROLLER_SCHEMA_VERSION,
            step,
            sim_time_ticks,
            robots,
        };
        frame.validate()?;
        Ok(frame)
    }

    /// Validates schema compatibility, ordering, identities, and numeric fields.
    pub fn validate(&self) -> Result<(), ControllerSchemaError> {
        validate_schema_version(self.schema_version)?;
        validate_strict_order(
            self.robots.iter().map(|robot| robot.robot_id.as_str()),
            "observation robot IDs",
        )?;
        for robot in &self.robots {
            robot.validate()?;
        }
        Ok(())
    }

    /// Returns one robot observation by stable ID.
    pub fn robot(&self, robot_id: &str) -> Option<&ControllerRobotObservation> {
        self.robots
            .binary_search_by(|robot| robot.robot_id.as_str().cmp(robot_id))
            .ok()
            .map(|index| &self.robots[index])
    }

    /// Serializes a validated observation frame as human-readable JSON.
    pub fn to_json_pretty(&self) -> Result<String, ControllerSchemaError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates an observation frame from JSON.
    pub fn from_json(text: &str) -> Result<Self, ControllerSchemaError> {
        let frame: Self = serde_json::from_str(text)?;
        frame.validate()?;
        Ok(frame)
    }
}

/// One named joint-velocity command returned by a controller.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerJointVelocityCommand {
    /// Stable joint name.
    pub name: String,
    /// Commanded joint velocity in radians per second.
    pub velocity_rad_s: f64,
}

impl ControllerJointVelocityCommand {
    /// Creates one joint-velocity command.
    pub fn new(name: impl Into<String>, velocity_rad_s: f64) -> Self {
        Self {
            name: name.into(),
            velocity_rad_s,
        }
    }

    fn validate(&self, robot_id: &str) -> Result<(), ControllerSchemaError> {
        validate_identifier("joint command name", &self.name)?;
        if !self.velocity_rad_s.is_finite() {
            return Err(ControllerSchemaError::Invalid(format!(
                "robot `{robot_id}` joint `{}` velocity command must be finite",
                self.name
            )));
        }
        Ok(())
    }
}

/// Deterministically ordered actions for one robot.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerRobotAction {
    /// Stable robot identity targeted by these commands.
    pub robot_id: String,
    /// Joint-velocity commands sorted by joint name.
    pub joint_velocities: Vec<ControllerJointVelocityCommand>,
}

impl ControllerRobotAction {
    /// Creates and validates one robot action, sorting commands by joint name.
    pub fn new(
        robot_id: impl Into<String>,
        mut joint_velocities: Vec<ControllerJointVelocityCommand>,
    ) -> Result<Self, ControllerSchemaError> {
        joint_velocities.sort_by(|left, right| left.name.cmp(&right.name));
        let action = Self {
            robot_id: robot_id.into(),
            joint_velocities,
        };
        action.validate()?;
        Ok(action)
    }

    fn validate(&self) -> Result<(), ControllerSchemaError> {
        validate_identifier("robot_id", &self.robot_id)?;
        validate_strict_order(
            self.joint_velocities
                .iter()
                .map(|command| command.name.as_str()),
            &format!("robot `{}` joint command names", self.robot_id),
        )?;
        for command in &self.joint_velocities {
            command.validate(&self.robot_id)?;
        }
        Ok(())
    }
}

/// Versioned action frame returned by a controller for one fixed step.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ControllerActionFrame {
    /// Observation/action schema version.
    pub schema_version: u32,
    /// Fixed simulation step this action answers.
    pub step: u64,
    /// Robot actions sorted by stable robot ID.
    pub robots: Vec<ControllerRobotAction>,
}

impl ControllerActionFrame {
    /// Creates and validates one action frame, sorting robots by stable ID.
    pub fn new(
        step: u64,
        mut robots: Vec<ControllerRobotAction>,
    ) -> Result<Self, ControllerSchemaError> {
        robots.sort_by(|left, right| left.robot_id.cmp(&right.robot_id));
        let frame = Self {
            schema_version: CONTROLLER_SCHEMA_VERSION,
            step,
            robots,
        };
        frame.validate()?;
        Ok(frame)
    }

    /// Validates schema compatibility, ordering, identities, and numeric fields.
    pub fn validate(&self) -> Result<(), ControllerSchemaError> {
        validate_schema_version(self.schema_version)?;
        validate_strict_order(
            self.robots.iter().map(|robot| robot.robot_id.as_str()),
            "action robot IDs",
        )?;
        for robot in &self.robots {
            robot.validate()?;
        }
        Ok(())
    }

    /// Validates that this action answers the supplied observation exactly.
    pub fn validate_against(
        &self,
        observation: &ControllerObservationFrame,
    ) -> Result<(), ControllerSchemaError> {
        self.validate()?;
        observation.validate()?;
        if self.step != observation.step {
            return Err(ControllerSchemaError::Invalid(format!(
                "action step {} does not match observation step {}",
                self.step, observation.step
            )));
        }
        for robot_action in &self.robots {
            let robot_observation = observation.robot(&robot_action.robot_id).ok_or_else(|| {
                ControllerSchemaError::UnknownRobot(robot_action.robot_id.clone())
            })?;
            for command in &robot_action.joint_velocities {
                if robot_observation.joint(&command.name).is_none() {
                    return Err(ControllerSchemaError::UnknownJoint {
                        robot_id: robot_action.robot_id.clone(),
                        joint: command.name.clone(),
                    });
                }
            }
        }
        Ok(())
    }

    /// Serializes a validated action frame as human-readable JSON.
    pub fn to_json_pretty(&self) -> Result<String, ControllerSchemaError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates an action frame from JSON.
    pub fn from_json(text: &str) -> Result<Self, ControllerSchemaError> {
        let frame: Self = serde_json::from_str(text)?;
        frame.validate()?;
        Ok(frame)
    }
}

/// Controller schema validation or compatibility failure.
#[derive(Debug, thiserror::Error)]
pub enum ControllerSchemaError {
    /// The frame uses an unsupported schema version.
    #[error("unsupported controller schema: expected {expected}, got {actual}")]
    UnsupportedVersion {
        /// Schema version supported by this runtime.
        expected: u32,
        /// Schema version found in the frame.
        actual: u32,
    },
    /// A schema invariant is invalid.
    #[error("invalid controller schema: {0}")]
    Invalid(String),
    /// An action targets a robot absent from the paired observation.
    #[error("controller action targets unknown robot `{0}`")]
    UnknownRobot(String),
    /// An action targets a joint absent from the paired robot observation.
    #[error("controller action targets unknown joint `{joint}` on robot `{robot_id}`")]
    UnknownJoint {
        /// Stable robot identity.
        robot_id: String,
        /// Stable joint name.
        joint: String,
    },
    /// JSON serialization or deserialization failed.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
}

fn validate_schema_version(schema_version: u32) -> Result<(), ControllerSchemaError> {
    if schema_version != CONTROLLER_SCHEMA_VERSION {
        return Err(ControllerSchemaError::UnsupportedVersion {
            expected: CONTROLLER_SCHEMA_VERSION,
            actual: schema_version,
        });
    }
    Ok(())
}

fn validate_identifier(field: &str, value: &str) -> Result<(), ControllerSchemaError> {
    if value.trim().is_empty() {
        return Err(ControllerSchemaError::Invalid(format!(
            "{field} must not be empty"
        )));
    }
    if value.contains('\0') {
        return Err(ControllerSchemaError::Invalid(format!(
            "{field} must not contain a NUL byte"
        )));
    }
    Ok(())
}

fn validate_strict_order<'a>(
    values: impl IntoIterator<Item = &'a str>,
    field: &str,
) -> Result<(), ControllerSchemaError> {
    let values = values.into_iter().collect::<Vec<_>>();
    let unique = values.iter().copied().collect::<BTreeSet<_>>();
    if unique.len() != values.len() {
        return Err(ControllerSchemaError::Invalid(format!(
            "{field} must be unique"
        )));
    }
    if values.windows(2).any(|pair| pair[0] >= pair[1]) {
        return Err(ControllerSchemaError::Invalid(format!(
            "{field} must be in strict ascending order"
        )));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn observation() -> ControllerObservationFrame {
        ControllerObservationFrame::new(
            7,
            700,
            vec![
                ControllerRobotObservation::new(
                    "robot_b",
                    vec![ControllerJointObservation::position("wheel", 0.5)],
                )
                .unwrap(),
                ControllerRobotObservation::new(
                    "robot_a",
                    vec![
                        ControllerJointObservation::position("joint_z", 0.25),
                        ControllerJointObservation::position_velocity("joint_a", -0.5, 1.0),
                    ],
                )
                .unwrap(),
            ],
        )
        .unwrap()
    }

    #[test]
    fn constructors_canonicalize_robot_and_joint_order() {
        let observation = observation();
        assert_eq!(observation.robots[0].robot_id, "robot_a");
        assert_eq!(observation.robots[0].joints[0].name, "joint_a");
        assert_eq!(observation.robots[1].robot_id, "robot_b");

        let json = observation.to_json_pretty().unwrap();
        assert_eq!(
            ControllerObservationFrame::from_json(&json).unwrap(),
            observation
        );
    }

    #[test]
    fn deserialized_noncanonical_order_is_rejected() {
        let mut observation = observation();
        observation.robots.swap(0, 1);
        let json = serde_json::to_string(&observation).unwrap();
        assert!(ControllerObservationFrame::from_json(&json).is_err());
    }

    #[test]
    fn duplicate_identifiers_and_non_finite_values_are_rejected() {
        let duplicate = ControllerRobotObservation::new(
            "robot",
            vec![
                ControllerJointObservation::position("joint", 0.0),
                ControllerJointObservation::position("joint", 1.0),
            ],
        );
        assert!(duplicate.is_err());

        let non_finite = ControllerRobotAction::new(
            "robot",
            vec![ControllerJointVelocityCommand::new("joint", f64::NAN)],
        );
        assert!(non_finite.is_err());
    }

    #[test]
    fn action_must_match_observation_step_robot_and_joint() {
        let observation = observation();
        let valid = ControllerActionFrame::new(
            7,
            vec![ControllerRobotAction::new(
                "robot_a",
                vec![ControllerJointVelocityCommand::new("joint_a", 2.0)],
            )
            .unwrap()],
        )
        .unwrap();
        valid.validate_against(&observation).unwrap();

        let wrong_step = ControllerActionFrame::new(8, valid.robots.clone()).unwrap();
        assert!(wrong_step.validate_against(&observation).is_err());
        let unknown_robot = ControllerActionFrame::new(
            7,
            vec![ControllerRobotAction::new("robot_c", Vec::new()).unwrap()],
        )
        .unwrap();
        assert!(matches!(
            unknown_robot.validate_against(&observation),
            Err(ControllerSchemaError::UnknownRobot(_))
        ));
        let unknown_joint = ControllerActionFrame::new(
            7,
            vec![ControllerRobotAction::new(
                "robot_a",
                vec![ControllerJointVelocityCommand::new("missing", 0.0)],
            )
            .unwrap()],
        )
        .unwrap();
        assert!(matches!(
            unknown_joint.validate_against(&observation),
            Err(ControllerSchemaError::UnknownJoint { .. })
        ));
    }

    #[test]
    fn unsupported_schema_versions_are_rejected() {
        let mut observation = observation();
        observation.schema_version += 1;
        assert!(matches!(
            observation.validate(),
            Err(ControllerSchemaError::UnsupportedVersion { .. })
        ));
    }
}

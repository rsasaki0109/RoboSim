//! A `ros2_control`-style boundary for wheeled robots.
//!
//! This module maps a planar body command to per-joint `ros2_control` command
//! interfaces (position, velocity, effort, acceleration) for a differential
//! drive, and interpolates a joint trajectory the way
//! `joint_trajectory_controller` does. It is deliberately free of any ROS 2 types
//! so the boundary can be reused by an adapter without changing it.

use serde::{Deserialize, Serialize};

/// Errors raised by the control boundary.
#[derive(Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum Ros2ControlError {
    /// A geometry, limit, or effort value was non-finite or non-positive.
    #[error("invalid ros2_control configuration")]
    InvalidConfig,
    /// A command or time step was non-finite.
    #[error("non-finite ros2_control input")]
    NonFiniteInput,
    /// The time step was not strictly positive.
    #[error("time step must be finite and positive")]
    NonPositiveTime,
    /// Trajectory points disagreed on the number of joints.
    #[error("trajectory point dimension mismatch")]
    DimensionMismatch,
    /// The trajectory had no points.
    #[error("trajectory must not be empty")]
    EmptyTrajectory,
    /// Trajectory point times were not strictly increasing.
    #[error("trajectory point times must strictly increase")]
    NonMonotonicTime,
}

/// Configuration for [`DiffDriveWheelController`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveControllerConfig {
    /// Wheel radius in meters.
    pub wheel_radius_m: f64,
    /// Track width in meters.
    pub wheel_track_m: f64,
    /// Maximum absolute wheel angular velocity in radians per second.
    pub max_wheel_velocity_rad_s: f64,
    /// Maximum absolute wheel angular acceleration in radians per second squared.
    pub max_wheel_accel_rad_s2: f64,
    /// Feed-forward effort applied per wheel in newton-meters.
    pub wheel_effort_nm: f64,
    /// Joint name of the left wheel.
    pub left_joint: String,
    /// Joint name of the right wheel.
    pub right_joint: String,
}

impl Default for DiffDriveControllerConfig {
    fn default() -> Self {
        Self {
            wheel_radius_m: 0.1,
            wheel_track_m: 0.45,
            max_wheel_velocity_rad_s: 10.0,
            max_wheel_accel_rad_s2: 20.0,
            wheel_effort_nm: 0.0,
            left_joint: "left_wheel_joint".into(),
            right_joint: "right_wheel_joint".into(),
        }
    }
}

impl DiffDriveControllerConfig {
    fn is_valid(&self) -> bool {
        self.wheel_radius_m.is_finite()
            && self.wheel_radius_m > 0.0
            && self.wheel_track_m.is_finite()
            && self.wheel_track_m > 0.0
            && self.max_wheel_velocity_rad_s.is_finite()
            && self.max_wheel_velocity_rad_s > 0.0
            && self.max_wheel_accel_rad_s2.is_finite()
            && self.max_wheel_accel_rad_s2 > 0.0
            && self.wheel_effort_nm.is_finite()
            && !self.left_joint.is_empty()
            && !self.right_joint.is_empty()
    }
}

/// One `ros2_control` joint command.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointCommand {
    /// Joint name.
    pub joint_name: String,
    /// Commanded position in radians.
    pub position_rad: f64,
    /// Commanded velocity in radians per second.
    pub velocity_rad_s: f64,
    /// Commanded acceleration in radians per second squared.
    pub acceleration_rad_s2: f64,
    /// Feed-forward effort in newton-meters.
    pub effort_nm: f64,
}

/// A pair of wheel joint commands.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct DiffDriveCommand {
    /// Left wheel command.
    pub left: JointCommand,
    /// Right wheel command.
    pub right: JointCommand,
    /// Whether a wheel velocity was clamped to its limit.
    pub saturated: bool,
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct WheelState {
    position_rad: f64,
    velocity_rad_s: f64,
}

/// Maps planar body commands to differential-drive wheel joint commands.
#[derive(Clone, Debug, PartialEq)]
pub struct DiffDriveWheelController {
    config: DiffDriveControllerConfig,
    left: WheelState,
    right: WheelState,
}

impl DiffDriveWheelController {
    /// Creates a controller after validating the configuration.
    pub fn new(config: DiffDriveControllerConfig) -> Result<Self, Ros2ControlError> {
        if !config.is_valid() {
            return Err(Ros2ControlError::InvalidConfig);
        }
        Ok(Self {
            config,
            left: WheelState::default(),
            right: WheelState::default(),
        })
    }

    /// The configuration.
    pub fn config(&self) -> &DiffDriveControllerConfig {
        &self.config
    }

    /// Resets accumulated wheel positions and velocities to zero.
    pub fn reset(&mut self) {
        self.left = WheelState::default();
        self.right = WheelState::default();
    }

    /// Current wheel positions in radians.
    pub fn positions_rad(&self) -> (f64, f64) {
        (self.left.position_rad, self.right.position_rad)
    }

    /// Maps `(linear_m_s, angular_rad_s)` to wheel commands over `dt_s`.
    ///
    /// Wheel velocities are clamped to `max_wheel_velocity_rad_s` and their
    /// change is bounded by `max_wheel_accel_rad_s2`; `saturated` reports
    /// whether clamping occurred.
    pub fn command(
        &mut self,
        linear_m_s: f64,
        angular_rad_s: f64,
        dt_s: f64,
    ) -> Result<DiffDriveCommand, Ros2ControlError> {
        if !linear_m_s.is_finite() || !angular_rad_s.is_finite() {
            return Err(Ros2ControlError::NonFiniteInput);
        }
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(Ros2ControlError::NonPositiveTime);
        }

        let half_track = self.config.wheel_track_m * 0.5;
        let radius = self.config.wheel_radius_m;
        let desired_left = (linear_m_s - angular_rad_s * half_track) / radius;
        let desired_right = (linear_m_s + angular_rad_s * half_track) / radius;

        let mut saturated = false;
        let left_target = Self::clamp_velocity(
            desired_left,
            self.config.max_wheel_velocity_rad_s,
            &mut saturated,
        );
        let right_target = Self::clamp_velocity(
            desired_right,
            self.config.max_wheel_velocity_rad_s,
            &mut saturated,
        );

        let left_accel = Self::step_wheel(
            &mut self.left,
            left_target,
            dt_s,
            self.config.max_wheel_accel_rad_s2,
            &mut saturated,
        );
        let right_accel = Self::step_wheel(
            &mut self.right,
            right_target,
            dt_s,
            self.config.max_wheel_accel_rad_s2,
            &mut saturated,
        );

        Ok(DiffDriveCommand {
            left: JointCommand {
                joint_name: self.config.left_joint.clone(),
                position_rad: self.left.position_rad,
                velocity_rad_s: self.left.velocity_rad_s,
                acceleration_rad_s2: left_accel,
                effort_nm: self.config.wheel_effort_nm,
            },
            right: JointCommand {
                joint_name: self.config.right_joint.clone(),
                position_rad: self.right.position_rad,
                velocity_rad_s: self.right.velocity_rad_s,
                acceleration_rad_s2: right_accel,
                effort_nm: self.config.wheel_effort_nm,
            },
            saturated,
        })
    }

    fn clamp_velocity(value: f64, limit: f64, saturated: &mut bool) -> f64 {
        let clamped = value.clamp(-limit, limit);
        if (clamped - value).abs() > 0.0 {
            *saturated = true;
        }
        clamped
    }

    fn step_wheel(
        wheel: &mut WheelState,
        target_velocity: f64,
        dt_s: f64,
        max_accel_rad_s2: f64,
        saturated: &mut bool,
    ) -> f64 {
        let max_step = max_accel_rad_s2 * dt_s;
        let step = (target_velocity - wheel.velocity_rad_s).clamp(-max_step, max_step);
        if (step - (target_velocity - wheel.velocity_rad_s)).abs() > 0.0 {
            *saturated = true;
        }
        let acceleration = step / dt_s;
        wheel.velocity_rad_s += step;
        wheel.position_rad += wheel.velocity_rad_s * dt_s;
        acceleration
    }
}

/// One `joint_trajectory_controller` waypoint.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointTrajectoryPoint {
    /// Joint positions in radians.
    pub positions_rad: Vec<f64>,
    /// Joint velocities in radians per second.
    pub velocities_rad_s: Vec<f64>,
    /// Time from the trajectory start in seconds.
    pub time_from_start_s: f64,
}

/// A multi-joint trajectory.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointTrajectory {
    /// Joint names, in the order used by every point.
    pub joint_names: Vec<String>,
    /// Waypoints in strictly increasing time order.
    pub points: Vec<JointTrajectoryPoint>,
}

impl JointTrajectory {
    /// Validates joint names, dimensions, and time ordering.
    pub fn validate(&self) -> Result<(), Ros2ControlError> {
        if self.joint_names.is_empty() {
            return Err(Ros2ControlError::InvalidConfig);
        }
        if self.points.is_empty() {
            return Err(Ros2ControlError::EmptyTrajectory);
        }
        let dimension = self.joint_names.len();
        let mut previous_time = f64::NEG_INFINITY;
        for point in &self.points {
            if point.positions_rad.len() != dimension || point.velocities_rad_s.len() != dimension {
                return Err(Ros2ControlError::DimensionMismatch);
            }
            if !point.time_from_start_s.is_finite()
                || point.positions_rad.iter().any(|value| !value.is_finite())
                || point
                    .velocities_rad_s
                    .iter()
                    .any(|value| !value.is_finite())
            {
                return Err(Ros2ControlError::NonFiniteInput);
            }
            if point.time_from_start_s <= previous_time {
                return Err(Ros2ControlError::NonMonotonicTime);
            }
            previous_time = point.time_from_start_s;
        }
        Ok(())
    }

    /// Samples joint positions by linear interpolation at `time_s`.
    ///
    /// Times before the first point or after the last clamp to the nearest
    /// waypoint, matching a controller that holds the final setpoint.
    pub fn sample_positions(&self, time_s: f64) -> Result<Vec<f64>, Ros2ControlError> {
        self.validate()?;
        if !time_s.is_finite() {
            return Err(Ros2ControlError::NonFiniteInput);
        }
        if time_s <= self.points[0].time_from_start_s {
            return Ok(self.points[0].positions_rad.clone());
        }
        let last = self.points.last().expect("validated non-empty");
        if time_s >= last.time_from_start_s {
            return Ok(last.positions_rad.clone());
        }
        for pair in self.points.windows(2) {
            let (start, end) = (&pair[0], &pair[1]);
            if time_s <= end.time_from_start_s {
                let span = end.time_from_start_s - start.time_from_start_s;
                let fraction = (time_s - start.time_from_start_s) / span;
                return Ok(start
                    .positions_rad
                    .iter()
                    .zip(&end.positions_rad)
                    .map(|(a, b)| a + (b - a) * fraction)
                    .collect());
            }
        }
        Ok(last.positions_rad.clone())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn straight_command_gives_equal_wheel_velocities() {
        let mut controller =
            DiffDriveWheelController::new(DiffDriveControllerConfig::default()).unwrap();
        let command = controller.command(1.0, 0.0, 1.0).unwrap();
        assert_relative_eq!(command.left.velocity_rad_s, 10.0, epsilon = 1e-9);
        assert_relative_eq!(command.right.velocity_rad_s, 10.0, epsilon = 1e-9);
        assert_eq!(command.left.joint_name, "left_wheel_joint");
        assert!(!command.saturated);
    }

    #[test]
    fn spin_gives_opposite_wheel_velocities() {
        let mut controller =
            DiffDriveWheelController::new(DiffDriveControllerConfig::default()).unwrap();
        let command = controller.command(0.0, 1.0, 1.0).unwrap();
        assert!(command.left.velocity_rad_s < 0.0);
        assert!(command.right.velocity_rad_s > 0.0);
    }

    #[test]
    fn clamps_velocity_and_reports_saturation() {
        let config = DiffDriveControllerConfig {
            wheel_radius_m: 0.1,
            max_wheel_velocity_rad_s: 5.0,
            max_wheel_accel_rad_s2: 1000.0,
            ..DiffDriveControllerConfig::default()
        };
        let mut controller = DiffDriveWheelController::new(config).unwrap();
        let command = controller.command(1.0, 0.0, 1.0).unwrap();
        assert!(command.saturated);
        assert_relative_eq!(command.left.velocity_rad_s, 5.0, epsilon = 1e-9);
    }

    #[test]
    fn accel_limit_bounds_the_velocity_ramp() {
        let config = DiffDriveControllerConfig {
            wheel_radius_m: 0.1,
            max_wheel_velocity_rad_s: 100.0,
            max_wheel_accel_rad_s2: 2.0,
            ..DiffDriveControllerConfig::default()
        };
        let mut controller = DiffDriveWheelController::new(config).unwrap();
        let command = controller.command(1.0, 0.0, 0.1).unwrap();
        assert!(command.saturated);
        assert_relative_eq!(command.left.velocity_rad_s, 0.2, epsilon = 1e-9);
        let (_, _) = controller.positions_rad();
    }

    #[test]
    fn trajectory_interpolates_and_clamps() {
        let trajectory = JointTrajectory {
            joint_names: vec!["left".into(), "right".into()],
            points: vec![
                JointTrajectoryPoint {
                    positions_rad: vec![0.0, 0.0],
                    velocities_rad_s: vec![0.0, 0.0],
                    time_from_start_s: 0.0,
                },
                JointTrajectoryPoint {
                    positions_rad: vec![1.0, 2.0],
                    velocities_rad_s: vec![1.0, 1.0],
                    time_from_start_s: 1.0,
                },
            ],
        };
        assert_eq!(trajectory.sample_positions(-0.5).unwrap(), vec![0.0, 0.0]);
        assert_relative_eq!(
            trajectory.sample_positions(0.5).unwrap()[0],
            0.5,
            epsilon = 1e-9
        );
        assert_relative_eq!(
            trajectory.sample_positions(0.5).unwrap()[1],
            1.0,
            epsilon = 1e-9
        );
        assert_eq!(trajectory.sample_positions(5.0).unwrap(), vec![1.0, 2.0]);
    }

    #[test]
    fn rejects_bad_config_and_trajectory() {
        let bad = DiffDriveControllerConfig {
            wheel_radius_m: 0.0,
            ..DiffDriveControllerConfig::default()
        };
        assert_eq!(
            DiffDriveWheelController::new(bad),
            Err(Ros2ControlError::InvalidConfig)
        );

        let trajectory = JointTrajectory {
            joint_names: vec!["left".into(), "right".into()],
            points: vec![JointTrajectoryPoint {
                positions_rad: vec![0.0],
                velocities_rad_s: vec![0.0],
                time_from_start_s: 0.0,
            }],
        };
        assert_eq!(
            trajectory.sample_positions(0.0),
            Err(Ros2ControlError::DimensionMismatch)
        );
    }
}

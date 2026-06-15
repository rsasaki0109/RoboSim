//! Action types for robot-native environments.

/// Wheel velocity command for a differential drive robot.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DiffDriveAction {
    /// Left wheel angular velocity in radians per second.
    pub left_velocity_rad_s: f64,
    /// Right wheel angular velocity in radians per second.
    pub right_velocity_rad_s: f64,
}

impl DiffDriveAction {
    /// Creates equal wheel velocities for straight-line motion.
    pub fn forward(velocity_rad_s: f64) -> Self {
        Self {
            left_velocity_rad_s: velocity_rad_s,
            right_velocity_rad_s: velocity_rad_s,
        }
    }
}

/// Joint velocity command for a mobile manipulator (optional base wheels + arm).
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MobileManipulatorAction {
    /// Left wheel angular velocity in radians per second.
    pub left_wheel_velocity_rad_s: f64,
    /// Right wheel angular velocity in radians per second.
    pub right_wheel_velocity_rad_s: f64,
    /// Shoulder joint angular velocity in radians per second.
    pub shoulder_velocity_rad_s: f64,
    /// Elbow joint angular velocity in radians per second.
    pub elbow_velocity_rad_s: f64,
    /// Parallel gripper open/close velocity in radians per second (both fingers).
    pub gripper_velocity_rad_s: f64,
    /// Vertical lift (prismatic column) velocity in meters per second. Positive
    /// raises the arm. Only the lift-equipped robot acts on this; other robots
    /// ignore it.
    pub lift_velocity_m_s: f64,
}

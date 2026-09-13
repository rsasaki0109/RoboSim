//! Action types for robot-native environments.

use crate::mm_lift_kinematics::MmLiftJointTarget;

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
    /// SO101 `shoulder_pan` angular velocity in radians per second.
    pub shoulder_pan_velocity_rad_s: f64,
    /// SO101 `shoulder_lift` angular velocity in radians per second.
    /// Falls back to `shoulder_velocity_rad_s` when zero.
    pub shoulder_lift_velocity_rad_s: f64,
    /// SO101 `elbow_flex` angular velocity in radians per second.
    /// Falls back to `elbow_velocity_rad_s` when zero.
    pub elbow_flex_velocity_rad_s: f64,
    /// SO101 `wrist_flex` angular velocity in radians per second.
    pub wrist_flex_velocity_rad_s: f64,
    /// SO101 `wrist_roll` angular velocity in radians per second.
    pub wrist_roll_velocity_rad_s: f64,
    /// SO101 `gripper` open/close velocity in radians per second.
    /// Follows the shared close-negative convention (negative closes the jaw
    /// toward the fixed frame); the sign is flipped on the motor wire to match
    /// the URDF close-positive travel. Falls back to `gripper_velocity_rad_s`
    /// when zero.
    pub so101_gripper_velocity_rad_s: f64,
    /// Parallel gripper open/close velocity in radians per second (both fingers).
    pub gripper_velocity_rad_s: f64,
    /// Parallel linear-gripper open/close velocity in meters per second. Positive
    /// opens the fingers and negative closes them. Revolute grippers ignore it.
    pub gripper_velocity_m_s: f64,
    /// Vertical lift (prismatic column) velocity in meters per second. Positive
    /// raises the arm. Only the lift-equipped robot acts on this; other robots
    /// ignore it.
    pub lift_velocity_m_s: f64,
    /// When set on the `mm_lift` robot, drives lift / shoulder / elbow position
    /// motors directly to these targets instead of integrating velocity commands.
    pub lift_joint_target: Option<MmLiftJointTarget>,
    /// Optional wrist-yaw position target in radians. Robots without an actuated
    /// wrist ignore it.
    pub wrist_yaw_target_rad: Option<f64>,
    /// Optional absolute SO101 arm joint targets (from IK). When set on the
    /// `mm_mobile_so101` variant, the arm joints are driven to these positions
    /// instead of integrating the velocity commands; the jaw still follows
    /// `so101_gripper_velocity_rad_s`/`gripper_velocity_rad_s`.
    pub so101_joint_target: Option<crate::so101_kinematics::So101JointTarget>,
}

impl MobileManipulatorAction {
    /// Creates an action that holds the lift arm at absolute joint targets.
    pub fn hold_lift_joints(target: MmLiftJointTarget) -> Self {
        Self {
            lift_joint_target: Some(target),
            wrist_yaw_target_rad: Some(-(target.shoulder_rad + target.elbow_rad)),
            ..Self::default()
        }
    }

    /// Attaches a lift joint-space target to an existing velocity command.
    pub fn with_lift_joint_target(mut self, target: MmLiftJointTarget) -> Self {
        self.lift_joint_target = Some(target);
        self.wrist_yaw_target_rad = Some(-(target.shoulder_rad + target.elbow_rad));
        self
    }
}

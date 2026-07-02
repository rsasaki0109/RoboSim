//! Differential-drive velocity conversion for mobile manipulator bases.

use rne_ai::{mm_mobile_twist_to_wheel_velocities, MobileManipulatorAction};

/// Converts a planar twist command into wheel joint velocities (rad/s).
pub fn twist_to_wheel_velocities(linear_x_m_s: f64, angular_z_rad_s: f64) -> (f64, f64) {
    mm_mobile_twist_to_wheel_velocities(linear_x_m_s, angular_z_rad_s)
}

/// Builds a mobile manipulator action from base twist and arm joint velocities.
pub fn mobile_action_from_twist_and_arm(
    linear_x_m_s: f64,
    angular_z_rad_s: f64,
    shoulder_velocity_rad_s: f64,
    elbow_velocity_rad_s: f64,
) -> MobileManipulatorAction {
    let (left, right) = twist_to_wheel_velocities(linear_x_m_s, angular_z_rad_s);
    MobileManipulatorAction {
        left_wheel_velocity_rad_s: left,
        right_wheel_velocity_rad_s: right,
        shoulder_velocity_rad_s,
        elbow_velocity_rad_s,
        gripper_velocity_rad_s: 0.0,
        lift_velocity_m_s: 0.0,
        ..MobileManipulatorAction::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_line_commands_equal_wheels() {
        let (left, right) = twist_to_wheel_velocities(0.6, 0.0);
        assert!((left - right).abs() < f64::EPSILON);
        assert!(left > 0.0);
    }

    #[test]
    fn turn_commands_differ_wheel_speeds() {
        let (left, right) = twist_to_wheel_velocities(0.0, 1.0);
        assert!((left - right).abs() > 0.1);
    }
}

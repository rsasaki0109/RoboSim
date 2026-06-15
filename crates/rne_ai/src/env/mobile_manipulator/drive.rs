//! Diff-drive helpers for `mm_mobile` wheel joints.

/// Track width in meters (matches `mm_mobile` URDF defaults).
pub const MM_MOBILE_TRACK_WIDTH_M: f64 = 0.45;
/// Wheel radius in meters (matches `mm_mobile` URDF defaults).
pub const MM_MOBILE_WHEEL_RADIUS_M: f64 = 0.1;
/// Positive command velocity means forward (+X); URDF Z-axis joints need this sign at the motor.
pub const MM_MOBILE_WHEEL_JOINT_SIGN: f64 = -1.0;

/// Converts a planar twist command into semantic wheel velocities (rad/s, forward = positive).
pub fn mm_mobile_twist_to_wheel_velocities(linear_x_m_s: f64, angular_z_rad_s: f64) -> (f64, f64) {
    let v_left_m_s = linear_x_m_s - angular_z_rad_s * MM_MOBILE_TRACK_WIDTH_M * 0.5;
    let v_right_m_s = linear_x_m_s + angular_z_rad_s * MM_MOBILE_TRACK_WIDTH_M * 0.5;
    (
        v_left_m_s / MM_MOBILE_WHEEL_RADIUS_M,
        v_right_m_s / MM_MOBILE_WHEEL_RADIUS_M,
    )
}

/// Maps a semantic wheel velocity command to the joint motor setpoint.
pub fn wheel_command_to_motor_rad_s(command_rad_s: f64) -> f64 {
    MM_MOBILE_WHEEL_JOINT_SIGN * command_rad_s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn twist_straight_commands_equal_wheels() {
        let (left, right) = mm_mobile_twist_to_wheel_velocities(0.6, 0.0);
        assert!((left - right).abs() < f64::EPSILON);
        assert!(left > 0.0);
    }

    #[test]
    fn motor_sign_inverts_positive_command() {
        assert!(wheel_command_to_motor_rad_s(6.0) < 0.0);
    }
}

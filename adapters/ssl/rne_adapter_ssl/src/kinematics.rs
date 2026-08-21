//! Map SSL move commands into planar / differential-drive kinematics.
//!
//! Keeps core crates free of SSL types. Callers convert the returned wheel
//! speeds into their own actuator actions (for example `DiffDriveAction`).

use crate::SslMoveCommand;

/// Default SSL stand-in wheel radius used by the bundled 2v2 robots (meters).
pub const SSL_STAND_IN_WHEEL_RADIUS_M: f64 = 0.05;
/// Default SSL stand-in track width used by the bundled 2v2 robots (meters).
pub const SSL_STAND_IN_TRACK_WIDTH_M: f64 = 0.14;

/// Differential-drive wheel speeds in radians per second.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SslDiffDriveWheelSpeeds {
    /// Left wheel angular velocity.
    pub left_rad_s: f64,
    /// Right wheel angular velocity.
    pub right_rad_s: f64,
}

/// Convert robot-local SSL velocity into RNE Y-up planar velocity.
///
/// SSL `forward`/`left` are robot-local. RNE uses +X forward and +Z left in the
/// yaw frame, so local left becomes world +Z after yaw.
#[must_use]
pub fn ssl_local_velocity_to_rne_planar(
    forward_m_s: f64,
    left_m_s: f64,
    angular_rad_s: f64,
    yaw_rad: f64,
) -> (f64, f64, f64) {
    let cos = yaw_rad.cos();
    let sin = yaw_rad.sin();
    let vx_m_s = forward_m_s * cos - left_m_s * sin;
    let vz_m_s = forward_m_s * sin + left_m_s * cos;
    (vx_m_s, vz_m_s, angular_rad_s)
}

/// Convert SSL field-frame velocity into RNE Y-up planar velocity.
///
/// SSL field `x` is along the pitch length; SSL `y` is leftward across the
/// pitch and maps onto RNE +Z.
#[must_use]
pub fn ssl_global_velocity_to_rne_planar(
    x_m_s: f64,
    y_m_s: f64,
    angular_rad_s: f64,
) -> (f64, f64, f64) {
    (x_m_s, y_m_s, angular_rad_s)
}

/// Map planar body velocity to differential-drive wheel speeds.
#[must_use]
pub fn planar_velocity_to_diff_drive(
    forward_m_s: f64,
    angular_rad_s: f64,
    wheel_radius_m: f64,
    track_width_m: f64,
) -> SslDiffDriveWheelSpeeds {
    let radius = wheel_radius_m.max(1.0e-6);
    let half_track = track_width_m * 0.5;
    SslDiffDriveWheelSpeeds {
        left_rad_s: (forward_m_s - angular_rad_s * half_track) / radius,
        right_rad_s: (forward_m_s + angular_rad_s * half_track) / radius,
    }
}

/// Map an SSL move command into stand-in differential-drive wheel speeds.
///
/// `LocalVelocity.left` is folded into the planar twist but pure lateral motion
/// without yaw cannot be realized on a non-holonomic stand-in; callers should
/// treat large leftward commands as approximate.
#[must_use]
pub fn ssl_move_to_diff_drive(
    command: SslMoveCommand,
    yaw_rad: f64,
    wheel_radius_m: f64,
    track_width_m: f64,
) -> SslDiffDriveWheelSpeeds {
    match command {
        SslMoveCommand::WheelVelocity {
            front_right_m_s,
            back_right_m_s,
            back_left_m_s,
            front_left_m_s,
        } => {
            let left_m_s = 0.5 * (f64::from(front_left_m_s) + f64::from(back_left_m_s));
            let right_m_s = 0.5 * (f64::from(front_right_m_s) + f64::from(back_right_m_s));
            let radius = wheel_radius_m.max(1.0e-6);
            SslDiffDriveWheelSpeeds {
                left_rad_s: left_m_s / radius,
                right_rad_s: right_m_s / radius,
            }
        }
        SslMoveCommand::LocalVelocity {
            forward_m_s,
            left_m_s: _,
            angular_rad_s,
        } => planar_velocity_to_diff_drive(
            f64::from(forward_m_s),
            f64::from(angular_rad_s),
            wheel_radius_m,
            track_width_m,
        ),
        SslMoveCommand::GlobalVelocity {
            x_m_s,
            y_m_s,
            angular_rad_s,
        } => {
            let (vx, vz, omega) = ssl_global_velocity_to_rne_planar(
                f64::from(x_m_s),
                f64::from(y_m_s),
                f64::from(angular_rad_s),
            );
            let forward = vx * yaw_rad.cos() + vz * yaw_rad.sin();
            planar_velocity_to_diff_drive(forward, omega, wheel_radius_m, track_width_m)
        }
    }
}

/// Map SSL ball teleport coordinates into RNE Y-up meters.
///
/// SSL `x`/`y` are the pitch plane; SSL `z` is up and becomes RNE `y`.
#[must_use]
pub fn ssl_ball_teleport_to_rne_m(x_m: f32, y_m: f32, z_m: f32) -> [f64; 3] {
    [f64::from(x_m), f64::from(z_m), f64::from(y_m)]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn local_forward_maps_to_equal_wheel_speeds() {
        let wheels = ssl_move_to_diff_drive(
            SslMoveCommand::LocalVelocity {
                forward_m_s: 0.5,
                left_m_s: 0.0,
                angular_rad_s: 0.0,
            },
            0.0,
            SSL_STAND_IN_WHEEL_RADIUS_M,
            SSL_STAND_IN_TRACK_WIDTH_M,
        );
        assert!((wheels.left_rad_s - 10.0).abs() < 1.0e-9);
        assert!((wheels.right_rad_s - 10.0).abs() < 1.0e-9);
    }

    #[test]
    fn ball_teleport_swaps_ssl_up_into_rne_y() {
        let mapped = ssl_ball_teleport_to_rne_m(4.6, 0.0, 0.0215);
        assert!((mapped[0] - 4.6).abs() < 1.0e-6);
        assert!((mapped[1] - 0.0215).abs() < 1.0e-6);
        assert!((mapped[2] - 0.0).abs() < 1.0e-6);
    }
}

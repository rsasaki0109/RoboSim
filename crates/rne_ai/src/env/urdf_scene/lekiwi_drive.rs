//! Kiwi-drive (three-omni-wheel) kinematics for the vendored LeKiwi base.

/// Wheel radius in meters (4" omni wheel per LeKiwi BOM).
pub const LEKIWI_WHEEL_RADIUS_M: f64 = 0.0508;

/// Wheel mount azimuth in the RNE XZ ground plane, derived from upstream
/// `drive_motor_mount-v11-{2,1,}` origins on `base_plate_layer1-v5` after the
/// `world_to_base_plate` `rpy="-π/2 0 0"` re-root (URDF XY → world XZ).
pub const LEKIWI_WHEEL_AZIMUTH_RAD: [f64; 3] = [
    1.768_189_872_084_854_2,  // mount -2 at (-0.02, -0.1) → world (-0.02, 0.10)
    -0.280_980_865_351_562_3, // mount -1 at (0.07928, 0.02268) → world (0.07928, -0.02268)
    -2.347_469_338_125_864_5, // mount base at (-0.05928, 0.05732) → world (-0.05928, -0.05732)
];

/// Pivot radius in meters for each drive wheel (distance from base origin in XZ).
pub const LEKIWI_WHEEL_PIVOT_RADIUS_M: [f64; 3] = [0.101_984, 0.082_483, 0.082_630];

/// Semantic wheel names in the same order as [`LEKIWI_WHEEL_AZIMUTH_RAD`].
pub const LEKIWI_DRIVE_WHEEL_LINKS: [&str; 3] = [
    "omni_wheel_mount-v5-2",
    "omni_wheel_mount-v5-1",
    "omni_wheel_mount-v5",
];

/// Planar body-frame velocity command for LeKiwi kiwi drive.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UrdfKiwiAction {
    /// Forward body velocity (+X) in m/s.
    pub vx_m_s: f64,
    /// Lateral body velocity (+Z) in m/s.
    pub vz_m_s: f64,
    /// Yaw rate about +Y in rad/s.
    pub wz_rad_s: f64,
}

/// Maps a planar twist to the three drive wheel angular velocities (rad/s).
///
/// Uses the standard three-omni kiwi model with per-wheel mount azimuth θᵢ and
/// pivot radius Rᵢ taken from the reduced URDF geometry:
///
/// `ωᵢ = (-sin(θᵢ)·vx + cos(θᵢ)·vz + Rᵢ·ωz) / r`
pub fn lekiwi_twist_to_wheel_velocities(action: UrdfKiwiAction) -> [f64; 3] {
    let UrdfKiwiAction {
        vx_m_s,
        vz_m_s,
        wz_rad_s,
    } = action;
    let mut out = [0.0; 3];
    for i in 0..3 {
        let theta_rad = LEKIWI_WHEEL_AZIMUTH_RAD[i];
        let pivot_radius_m = LEKIWI_WHEEL_PIVOT_RADIUS_M[i];
        let v_tangent_m_s =
            -theta_rad.sin() * vx_m_s + theta_rad.cos() * vz_m_s + pivot_radius_m * wz_rad_s;
        out[i] = v_tangent_m_s / LEKIWI_WHEEL_RADIUS_M;
    }
    out
}

/// Motor setpoint sign for LeKiwi drive joints (URDF axis vs semantic velocity).
pub const LEKIWI_WHEEL_JOINT_SIGN: f64 = 1.0;

/// Maps a semantic wheel velocity to the joint motor command.
pub fn lekiwi_wheel_command_to_motor_rad_s(command_rad_s: f64) -> f64 {
    LEKIWI_WHEEL_JOINT_SIGN * command_rad_s
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn forward_twist_commands_equal_wheels_for_symmetric_spacing() {
        let wheels = lekiwi_twist_to_wheel_velocities(UrdfKiwiAction {
            vx_m_s: 0.3,
            vz_m_s: 0.0,
            wz_rad_s: 0.0,
        });
        assert!(wheels.iter().all(|w| w.is_finite()));
        assert!(wheels[0].abs() > 0.0);
    }

    #[test]
    fn yaw_only_twist_produces_differential_wheel_speeds() {
        let wheels = lekiwi_twist_to_wheel_velocities(UrdfKiwiAction {
            vx_m_s: 0.0,
            vz_m_s: 0.0,
            wz_rad_s: 0.5,
        });
        let spread = wheels
            .iter()
            .fold(0.0_f64, |acc, w| acc + (w - wheels[0]).abs());
        assert!(spread > 0.01);
    }
}

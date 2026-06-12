//! Sensor noise models.

use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Simple additive noise model for sensor outputs.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct NoiseModel {
    /// Gaussian standard deviation for angular velocity.
    pub angular_stddev_rad_s: f64,
    /// Gaussian standard deviation for linear acceleration.
    pub linear_stddev_m_s2: f64,
    /// Constant bias added to linear acceleration.
    pub linear_bias_m_s2: Vec3,
}

impl NoiseModel {
    /// Applies noise to an IMU sample deterministically from a seed.
    pub fn apply_imu(
        &self,
        angular_velocity_rad_s: Vec3,
        linear_acceleration_m_s2: Vec3,
        seed: u64,
    ) -> (Vec3, Vec3) {
        let n = deterministic_unit(seed);
        let angular = angular_velocity_rad_s + n * self.angular_stddev_rad_s;
        let linear = linear_acceleration_m_s2 + self.linear_bias_m_s2 + n * self.linear_stddev_m_s2;
        (angular, linear)
    }
}

fn deterministic_unit(seed: u64) -> Vec3 {
    let x = pseudo_random(seed.wrapping_mul(0x9E37_79B9_7F4A_7C15)) * 2.0 - 1.0;
    let y = pseudo_random(seed.wrapping_mul(0xBF58_476D_1CE4_E5B9)) * 2.0 - 1.0;
    let z = pseudo_random(seed.wrapping_mul(0x94D0_49BB_1331_11EB)) * 2.0 - 1.0;
    Vec3::new(x, y, z)
}

fn pseudo_random(value: u64) -> f64 {
    let mixed = value ^ (value >> 33);
    let mixed = mixed.wrapping_mul(0xff51_afd7_ed55_8ccd);
    let mixed = mixed ^ (mixed >> 33);
    (mixed as f64) / (u64::MAX as f64)
}

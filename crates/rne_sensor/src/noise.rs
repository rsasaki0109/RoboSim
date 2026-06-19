//! Sensor noise models.

use rne_core::{mix64, KeyedRandom};
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

const IMU_NOISE_DOMAIN_V1: u64 = 0x314E_756D_6945_4E52;

/// Stable coordinate for stateless sensor noise samples.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SensorNoiseKey {
    /// Root world seed for the simulation.
    pub root_seed: u64,
    /// Sensor-local seed or salt.
    pub sensor_seed: u64,
    /// Stable sensor identifier, such as a DataBus stream id.
    pub stable_sensor_id: u64,
    /// Deterministic sample counter for this sensor.
    pub sample_index: u64,
}

impl SensorNoiseKey {
    /// Creates a stateless sensor noise key.
    pub const fn new(
        root_seed: u64,
        sensor_seed: u64,
        stable_sensor_id: u64,
        sample_index: u64,
    ) -> Self {
        Self {
            root_seed,
            sensor_seed,
            stable_sensor_id,
            sample_index,
        }
    }
}

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

    /// Applies IMU noise from a stateless deterministic sample key.
    pub fn apply_imu_keyed(
        &self,
        angular_velocity_rad_s: Vec3,
        linear_acceleration_m_s2: Vec3,
        key: SensorNoiseKey,
    ) -> (Vec3, Vec3) {
        let n = keyed_unit(key);
        let angular = angular_velocity_rad_s + n * self.angular_stddev_rad_s;
        let linear = linear_acceleration_m_s2 + self.linear_bias_m_s2 + n * self.linear_stddev_m_s2;
        (angular, linear)
    }
}

fn keyed_unit(key: SensorNoiseKey) -> Vec3 {
    let random = KeyedRandom::new(key.root_seed, IMU_NOISE_DOMAIN_V1 ^ mix64(key.sensor_seed));
    Vec3::new(
        random.sample_signed_f64(key.stable_sensor_id, key.sample_index, 0),
        random.sample_signed_f64(key.stable_sensor_id, key.sample_index, 1),
        random.sample_signed_f64(key.stable_sensor_id, key.sample_index, 2),
    )
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn keyed_imu_noise_is_repeatable() {
        let model = NoiseModel {
            angular_stddev_rad_s: 0.1,
            linear_stddev_m_s2: 0.2,
            linear_bias_m_s2: Vec3::new(0.0, 1.0, 0.0),
        };
        let key = SensorNoiseKey::new(42, 3, 100, 7);

        let first = model.apply_imu_keyed(Vec3::ZERO, Vec3::ZERO, key);
        let second = model.apply_imu_keyed(Vec3::ZERO, Vec3::ZERO, key);

        assert_eq!(first, second);
    }

    #[test]
    fn keyed_imu_noise_changes_by_sample_index() {
        let model = NoiseModel {
            angular_stddev_rad_s: 0.1,
            linear_stddev_m_s2: 0.2,
            linear_bias_m_s2: Vec3::ZERO,
        };

        let first =
            model.apply_imu_keyed(Vec3::ZERO, Vec3::ZERO, SensorNoiseKey::new(42, 3, 100, 7));
        let second =
            model.apply_imu_keyed(Vec3::ZERO, Vec3::ZERO, SensorNoiseKey::new(42, 3, 100, 8));

        assert_ne!(first, second);
    }
}

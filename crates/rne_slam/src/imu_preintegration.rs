//! IMU preintegration for LiDAR-inertial odometry.
//!
//! Between two keyframes, high-rate gyroscope and accelerometer samples are
//! folded into a single relative motion measurement on the SE(3)/R^3 manifold
//! (Forster et al., 2017). The preintegrated delta is independent of the
//! starting pose, so a keyframe graph can recompute predicted poses cheaply and
//! correct acceleration/gyroscope bias without replaying every raw sample.
//!
//! Bias is treated as an additive estimate that is subtracted from each sample;
//! a first-order bias-correction Jacobian is a later refinement.

use crate::se3::{so3_exp, Se3};
use rne_math::{Quat, Vec3};
use serde::{Deserialize, Serialize};

/// One inertial measurement with its integration interval.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ImuSample {
    /// Integration time since the previous sample, in seconds.
    pub dt_s: f64,
    /// Specific force (accelerometer) reading in the IMU frame, m/s^2.
    pub accel_m_s2: Vec3,
    /// Angular velocity (gyroscope) reading in the IMU frame, rad/s.
    pub gyro_rad_s: Vec3,
}

/// Additive IMU bias estimate subtracted from raw samples.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct ImuBias {
    /// Accelerometer bias in m/s^2.
    pub accel_m_s2: Vec3,
    /// Gyroscope bias in rad/s.
    pub gyro_rad_s: Vec3,
}

/// Pose-independent preintegrated motion between two keyframes.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PreintegratedDelta {
    /// Integrated duration in seconds.
    pub dt_s: f64,
    /// Preintegrated rotation `delta_R`.
    pub delta_rotation: Quat,
    /// Preintegrated velocity `delta_v`.
    pub delta_velocity: Vec3,
    /// Preintegrated position `delta_p`.
    pub delta_position: Vec3,
}

impl Default for PreintegratedDelta {
    fn default() -> Self {
        Self {
            dt_s: 0.0,
            delta_rotation: Quat::IDENTITY,
            delta_velocity: Vec3::ZERO,
            delta_position: Vec3::ZERO,
        }
    }
}

/// IMU preintegration failure.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum ImuPreintegrationError {
    /// The sample interval was non-positive or non-finite.
    #[error("IMU sample dt must be finite and positive")]
    InvalidDt,
    /// A sample contained a non-finite value.
    #[error("IMU sample must be finite")]
    NonFinite,
}

/// Accumulates IMU samples into a [`PreintegratedDelta`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ImuPreintegrator {
    bias: ImuBias,
    delta: PreintegratedDelta,
}

impl ImuPreintegrator {
    /// Creates an integrator with the given bias estimate.
    pub fn new(bias: ImuBias) -> Self {
        Self {
            bias,
            delta: PreintegratedDelta::default(),
        }
    }

    /// Resets the accumulated delta while keeping the bias estimate.
    pub fn reset(&mut self) {
        self.delta = PreintegratedDelta::default();
    }

    /// Current bias estimate.
    pub const fn bias(&self) -> ImuBias {
        self.bias
    }

    /// Updates the bias estimate without changing the accumulated delta.
    pub fn set_bias(&mut self, bias: ImuBias) {
        self.bias = bias;
    }

    /// Accumulated delta.
    pub const fn delta(&self) -> PreintegratedDelta {
        self.delta
    }

    /// Folds one bias-corrected sample into the delta.
    pub fn integrate(&mut self, sample: &ImuSample) -> Result<(), ImuPreintegrationError> {
        if !sample.dt_s.is_finite() || sample.dt_s <= 0.0 {
            return Err(ImuPreintegrationError::InvalidDt);
        }
        if !sample.accel_m_s2.is_finite() || !sample.gyro_rad_s.is_finite() {
            return Err(ImuPreintegrationError::NonFinite);
        }
        let dt = sample.dt_s;
        let gyro = sample.gyro_rad_s - self.bias.gyro_rad_s;
        let accel = sample.accel_m_s2 - self.bias.accel_m_s2;

        // Rotate the body-frame specific force into the (preintegrated) start frame.
        let accel_in_start = self.delta.delta_rotation.mul_vec3(accel);
        self.delta.delta_position +=
            self.delta.delta_velocity * dt + accel_in_start * (0.5 * dt * dt);
        self.delta.delta_velocity += accel_in_start * dt;
        self.delta.delta_rotation = (self.delta.delta_rotation * so3_exp(gyro * dt)).normalize();
        self.delta.dt_s += dt;
        Ok(())
    }

    /// Predicts the end pose and velocity from a start pose and velocity.
    ///
    /// `gravity_m_s2` is the gravity vector in the world frame (for example
    /// `(0, -9.81, 0)`); the start rotation maps the IMU frame to the world.
    pub fn predict(&self, pose: Se3, velocity_m_s: Vec3, gravity_m_s2: Vec3) -> (Se3, Vec3) {
        let dt = self.delta.dt_s;
        let rotation = (pose.rotation * self.delta.delta_rotation).normalize();
        let rotated_velocity = pose.rotation.mul_vec3(self.delta.delta_velocity);
        let rotated_position = pose.rotation.mul_vec3(self.delta.delta_position);
        let velocity = velocity_m_s + gravity_m_s2 * dt + rotated_velocity;
        let position = pose.translation
            + velocity_m_s * dt
            + gravity_m_s2 * (0.5 * dt * dt)
            + rotated_position;
        (
            Se3 {
                rotation,
                translation: position,
            },
            velocity,
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn integrate_constant(accel: Vec3, gyro: Vec3, dt: f64, steps: usize) -> ImuPreintegrator {
        let mut integrator = ImuPreintegrator::new(ImuBias::default());
        for _ in 0..steps {
            integrator
                .integrate(&ImuSample {
                    dt_s: dt,
                    accel_m_s2: accel,
                    gyro_rad_s: gyro,
                })
                .expect("sample");
        }
        integrator
    }

    #[test]
    fn pure_rotation_integrates_yaw() {
        // Y is up in RNE, so yaw is rotation about the Y axis.
        let integrator = integrate_constant(Vec3::ZERO, Vec3::new(0.0, 1.0, 0.0), 0.01, 100);
        let delta = integrator.delta();
        assert_relative_eq!(delta.dt_s, 1.0, epsilon = 1e-12);
        let (pose, velocity) =
            integrator.predict(Se3::IDENTITY, Vec3::ZERO, Vec3::new(0.0, -9.81, 0.0));
        // Zero linear acceleration, so gravity is the only velocity source.
        assert_relative_eq!(velocity.y, -9.81, epsilon = 1e-9);
        let yaw = rne_math::yaw_rad(pose.rotation);
        assert_relative_eq!(yaw, 1.0, epsilon = 1e-9);
    }

    #[test]
    fn constant_body_acceleration_integrates_position() {
        let integrator = integrate_constant(Vec3::new(1.0, 0.0, 0.0), Vec3::ZERO, 0.01, 100);
        let (pose, velocity) = integrator.predict(Se3::IDENTITY, Vec3::ZERO, Vec3::ZERO);
        assert_relative_eq!(velocity.x, 1.0, epsilon = 1e-9);
        assert_relative_eq!(pose.translation.x, 0.5, epsilon = 1e-6);
    }

    #[test]
    fn free_fall_is_gravity_cancelled() {
        // Accelerometer reads +g upward while gravity pulls -g down.
        let gravity = Vec3::new(0.0, -9.81, 0.0);
        let integrator = integrate_constant(-gravity, Vec3::ZERO, 0.01, 100);
        let (pose, velocity) = integrator.predict(Se3::IDENTITY, Vec3::ZERO, gravity);
        assert_relative_eq!(velocity.x, 0.0, epsilon = 1e-9);
        assert_relative_eq!(velocity.y, 0.0, epsilon = 1e-9);
        assert_relative_eq!(pose.translation.y, 0.0, epsilon = 1e-6);
    }

    #[test]
    fn invalid_samples_are_rejected() {
        let mut integrator = ImuPreintegrator::new(ImuBias::default());
        assert!(matches!(
            integrator.integrate(&ImuSample {
                dt_s: 0.0,
                accel_m_s2: Vec3::ZERO,
                gyro_rad_s: Vec3::ZERO,
            }),
            Err(ImuPreintegrationError::InvalidDt)
        ));
        assert!(matches!(
            integrator.integrate(&ImuSample {
                dt_s: 0.01,
                accel_m_s2: Vec3::new(f64::NAN, 0.0, 0.0),
                gyro_rad_s: Vec3::ZERO,
            }),
            Err(ImuPreintegrationError::NonFinite)
        ));
    }
}

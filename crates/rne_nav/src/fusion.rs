//! Deterministic 2D error-state-free EKF fusing odometry, IMU, and GPS.
//!
//! [`EkfFusion`] estimates the five-state vector
//! `[x_m, y_m, yaw_rad, v_m_s, yaw_rate_rad_s]` with a constant-velocity motion
//! model. Measurements are supplied as observations rather than control inputs,
//! which keeps the filter reusable across backends:
//!
//! * [`EkfFusion::predict`] advances the constant-velocity model by `dt_s`.
//! * [`EkfFusion::update_odometry`] fuses a forward velocity and yaw rate.
//! * [`EkfFusion::update_yaw_rate`] fuses an IMU yaw rate only.
//! * [`EkfFusion::update_position`] fuses a planar position fix (e.g. GPS).
//!
//! All arithmetic is plain `f64` with index-ordered updates and no random
//! numbers, so a recorded sequence of measurements replays bit-for-bit.

use crate::control::VelocityCommand2d;
use crate::navsat::NavSatTransform;
use crate::pose2d::Pose2d;
use serde::{Deserialize, Serialize};

const STATE_DIM: usize = 5;

/// A planar pose with its `[x, y, yaw]` covariance.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PoseWithCovariance2d {
    /// The fused pose.
    pub pose: Pose2d,
    /// Row-major 3x3 covariance in `[x, y, yaw]` order.
    pub covariance: [[f64; 3]; 3],
}

/// Covariance tuning for [`EkfFusion`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct EkfConfig {
    /// Process noise added to position per second.
    pub process_noise_position: f64,
    /// Process noise added to yaw per second.
    pub process_noise_yaw: f64,
    /// Process noise added to linear velocity per second.
    pub process_noise_velocity: f64,
    /// Process noise added to yaw rate per second.
    pub process_noise_yaw_rate: f64,
    /// Initial position variance in meters squared.
    pub initial_position_variance_m2: f64,
    /// Initial yaw variance in radians squared.
    pub initial_yaw_variance_rad2: f64,
    /// Initial linear velocity variance.
    pub initial_velocity_variance: f64,
    /// Initial yaw rate variance.
    pub initial_yaw_rate_variance: f64,
}

impl Default for EkfConfig {
    fn default() -> Self {
        Self {
            process_noise_position: 0.01,
            process_noise_yaw: 0.005,
            process_noise_velocity: 0.05,
            process_noise_yaw_rate: 0.05,
            initial_position_variance_m2: 1.0,
            initial_yaw_variance_rad2: 0.1,
            initial_velocity_variance: 0.25,
            initial_yaw_rate_variance: 0.25,
        }
    }
}

impl EkfConfig {
    fn is_valid(&self) -> bool {
        let noises = [
            self.process_noise_position,
            self.process_noise_yaw,
            self.process_noise_velocity,
            self.process_noise_yaw_rate,
        ];
        let variances = [
            self.initial_position_variance_m2,
            self.initial_yaw_variance_rad2,
            self.initial_velocity_variance,
            self.initial_yaw_rate_variance,
        ];
        noises.iter().all(|n| n.is_finite() && *n >= 0.0)
            && variances.iter().all(|v| v.is_finite() && *v > 0.0)
    }
}

/// Errors raised by the EKF.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum FusionError {
    /// The configuration contains a non-finite or non-positive value.
    #[error("invalid EKF configuration")]
    InvalidConfig,
    /// A measurement or time step was non-finite.
    #[error("non-finite EKF input")]
    NonFiniteInput,
    /// The time step was not strictly positive.
    #[error("time step must be finite and positive")]
    NonPositiveTime,
    /// The innovation covariance was singular and could not be inverted.
    #[error("innovation covariance is singular")]
    SingularInnovation,
    /// A measurement variance was not strictly positive.
    #[error("measurement variance must be positive")]
    InvalidVariance,
}

/// A five-state planar extended Kalman filter.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EkfFusion {
    config: EkfConfig,
    state: [f64; STATE_DIM],
    covariance: [[f64; STATE_DIM]; STATE_DIM],
}

impl EkfFusion {
    /// Creates a filter initialized at the identity pose with zero velocity.
    pub fn new(config: EkfConfig) -> Result<Self, FusionError> {
        if !config.is_valid() {
            return Err(FusionError::InvalidConfig);
        }
        let mut covariance = [[0.0; STATE_DIM]; STATE_DIM];
        covariance[0][0] = config.initial_position_variance_m2;
        covariance[1][1] = config.initial_position_variance_m2;
        covariance[2][2] = config.initial_yaw_variance_rad2;
        covariance[3][3] = config.initial_velocity_variance;
        covariance[4][4] = config.initial_yaw_rate_variance;
        Ok(Self {
            config,
            state: [0.0; STATE_DIM],
            covariance,
        })
    }

    /// Creates a filter initialized with an explicit pose and velocity.
    pub fn with_initial_state(
        config: EkfConfig,
        pose: Pose2d,
        velocity: VelocityCommand2d,
    ) -> Result<Self, FusionError> {
        if !pose.is_finite()
            || !velocity.linear_m_s.is_finite()
            || !velocity.angular_rad_s.is_finite()
        {
            return Err(FusionError::NonFiniteInput);
        }
        let mut filter = Self::new(config)?;
        filter.state = [
            pose.x_m,
            pose.y_m,
            wrap_angle(pose.yaw_rad),
            velocity.linear_m_s,
            velocity.angular_rad_s,
        ];
        Ok(filter)
    }

    /// The current fused pose estimate.
    pub fn pose(&self) -> Pose2d {
        Pose2d::new(self.state[0], self.state[1], self.state[2])
    }

    /// The current fused velocity estimate.
    pub fn velocity(&self) -> VelocityCommand2d {
        VelocityCommand2d::new(self.state[3], self.state[4])
    }

    /// The fused pose with its `[x, y, yaw]` covariance sub-block.
    pub fn pose_with_covariance(&self) -> PoseWithCovariance2d {
        PoseWithCovariance2d {
            pose: self.pose(),
            covariance: [
                [
                    self.covariance[0][0],
                    self.covariance[0][1],
                    self.covariance[0][2],
                ],
                [
                    self.covariance[1][0],
                    self.covariance[1][1],
                    self.covariance[1][2],
                ],
                [
                    self.covariance[2][0],
                    self.covariance[2][1],
                    self.covariance[2][2],
                ],
            ],
        }
    }

    /// The full state vector `[x, y, yaw, v, yaw_rate]`.
    pub fn state(&self) -> [f64; STATE_DIM] {
        self.state
    }

    /// The state covariance matrix.
    pub fn covariance(&self) -> [[f64; STATE_DIM]; STATE_DIM] {
        self.covariance
    }

    /// Advances the constant-velocity motion model by `dt_s`.
    pub fn predict(&mut self, dt_s: f64) -> Result<(), FusionError> {
        if !dt_s.is_finite() {
            return Err(FusionError::NonFiniteInput);
        }
        if dt_s <= 0.0 {
            return Err(FusionError::NonPositiveTime);
        }

        let (x, y, yaw, v, omega) = (
            self.state[0],
            self.state[1],
            self.state[2],
            self.state[3],
            self.state[4],
        );
        let (sin, cos) = yaw.sin_cos();

        self.state[0] = x + v * cos * dt_s;
        self.state[1] = y + v * sin * dt_s;
        self.state[2] = wrap_angle(yaw + omega * dt_s);

        let jacobian = [
            [1.0, 0.0, -v * sin * dt_s, cos * dt_s, 0.0],
            [0.0, 1.0, v * cos * dt_s, sin * dt_s, 0.0],
            [0.0, 0.0, 1.0, 0.0, dt_s],
            [0.0, 0.0, 0.0, 1.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0],
        ];
        self.covariance = mul_5x5(&jacobian, &self.covariance);
        self.covariance = mul_5x5(&self.covariance, &transpose_5x5(&jacobian));

        let q = [
            self.config.process_noise_position * dt_s,
            self.config.process_noise_position * dt_s,
            self.config.process_noise_yaw * dt_s,
            self.config.process_noise_velocity * dt_s,
            self.config.process_noise_yaw_rate * dt_s,
        ];
        for (i, noise) in q.iter().enumerate() {
            self.covariance[i][i] += noise;
        }
        self.symmetrize();
        Ok(())
    }

    /// Fuses a forward velocity and yaw rate measurement.
    pub fn update_odometry(
        &mut self,
        linear_m_s: f64,
        angular_rad_s: f64,
        linear_variance: f64,
        angular_variance: f64,
    ) -> Result<(), FusionError> {
        if !linear_m_s.is_finite() || !angular_rad_s.is_finite() {
            return Err(FusionError::NonFiniteInput);
        }
        if linear_variance <= 0.0 || angular_variance <= 0.0 {
            return Err(FusionError::InvalidVariance);
        }
        let innovation = [linear_m_s - self.state[3], angular_rad_s - self.state[4]];
        let s = [
            [
                self.covariance[3][3] + linear_variance,
                self.covariance[3][4],
            ],
            [
                self.covariance[4][3],
                self.covariance[4][4] + angular_variance,
            ],
        ];
        let det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
        if det.abs() < f64::MIN_POSITIVE {
            return Err(FusionError::SingularInnovation);
        }
        let inv = [
            [s[1][1] / det, -s[0][1] / det],
            [-s[1][0] / det, s[0][0] / det],
        ];

        let mut gain = [[0.0; 2]; STATE_DIM];
        for (i, row) in gain.iter_mut().enumerate() {
            row[0] = self.covariance[i][3] * inv[0][0] + self.covariance[i][4] * inv[1][0];
            row[1] = self.covariance[i][3] * inv[0][1] + self.covariance[i][4] * inv[1][1];
        }

        let row3 = self.covariance[3];
        let row4 = self.covariance[4];
        for (i, gain_row) in gain.iter().enumerate() {
            self.state[i] += gain_row[0] * innovation[0] + gain_row[1] * innovation[1];
            for (j, cell) in self.covariance[i].iter_mut().enumerate() {
                *cell -= gain_row[0] * row3[j] + gain_row[1] * row4[j];
            }
        }
        self.state[2] = wrap_angle(self.state[2]);
        self.symmetrize();
        Ok(())
    }

    /// Fuses an IMU yaw rate measurement.
    pub fn update_yaw_rate(
        &mut self,
        angular_rad_s: f64,
        variance: f64,
    ) -> Result<(), FusionError> {
        if !angular_rad_s.is_finite() {
            return Err(FusionError::NonFiniteInput);
        }
        if variance <= 0.0 {
            return Err(FusionError::InvalidVariance);
        }
        let innovation = angular_rad_s - self.state[4];
        let s = self.covariance[4][4] + variance;
        if s.abs() < f64::MIN_POSITIVE {
            return Err(FusionError::SingularInnovation);
        }
        let row: [f64; STATE_DIM] = self.covariance[4];
        for i in 0..STATE_DIM {
            let gain = self.covariance[i][4] / s;
            self.state[i] += gain * innovation;
            for (j, value) in row.iter().enumerate() {
                self.covariance[i][j] -= gain * value;
            }
        }
        self.state[2] = wrap_angle(self.state[2]);
        self.symmetrize();
        Ok(())
    }

    /// Fuses a planar position fix (e.g. GPS) in the map frame.
    pub fn update_position(
        &mut self,
        x_m: f64,
        y_m: f64,
        variance_m2: f64,
    ) -> Result<(), FusionError> {
        if !x_m.is_finite() || !y_m.is_finite() {
            return Err(FusionError::NonFiniteInput);
        }
        if variance_m2 <= 0.0 {
            return Err(FusionError::InvalidVariance);
        }
        let innovation = [x_m - self.state[0], y_m - self.state[1]];
        let s = [
            [self.covariance[0][0] + variance_m2, self.covariance[0][1]],
            [self.covariance[1][0], self.covariance[1][1] + variance_m2],
        ];
        let det = s[0][0] * s[1][1] - s[0][1] * s[1][0];
        if det.abs() < f64::MIN_POSITIVE {
            return Err(FusionError::SingularInnovation);
        }
        let inv = [
            [s[1][1] / det, -s[0][1] / det],
            [-s[1][0] / det, s[0][0] / det],
        ];

        let mut gain = [[0.0; 2]; STATE_DIM];
        for (i, row) in gain.iter_mut().enumerate() {
            row[0] = self.covariance[i][0] * inv[0][0] + self.covariance[i][1] * inv[1][0];
            row[1] = self.covariance[i][0] * inv[0][1] + self.covariance[i][1] * inv[1][1];
        }

        for (i, gain_row) in gain.iter().enumerate() {
            self.state[i] += gain_row[0] * innovation[0] + gain_row[1] * innovation[1];
            self.covariance[i][0] -= gain_row[0];
            self.covariance[i][1] -= gain_row[1];
        }
        self.state[2] = wrap_angle(self.state[2]);
        self.symmetrize();
        Ok(())
    }

    /// Fuses a geographic fix by projecting it through a `NavSat` transform.
    pub fn update_navsat(
        &mut self,
        navsat: &NavSatTransform,
        latitude_rad: f64,
        longitude_rad: f64,
        altitude_m: f64,
        variance_m2: f64,
    ) -> Result<(), FusionError> {
        let point = navsat
            .datum_to_map(latitude_rad, longitude_rad, altitude_m)
            .map_err(|_| FusionError::NonFiniteInput)?;
        self.update_position(point.x, point.y, variance_m2)
    }

    fn symmetrize(&mut self) {
        for i in 0..STATE_DIM {
            for j in (i + 1)..STATE_DIM {
                let mean = 0.5 * (self.covariance[i][j] + self.covariance[j][i]);
                self.covariance[i][j] = mean;
                self.covariance[j][i] = mean;
            }
        }
    }
}

/// Computes the `map -> odom` transform from a fused map pose and an odom pose.
///
/// Publishing this as the `map -> odom` edge places the drifting odom frame
/// under the corrected map frame, as `robot_localization` does.
pub fn map_from_odom(fused_pose: Pose2d, odom_pose: Pose2d) -> Pose2d {
    fused_pose.compose(odom_pose.inverse())
}

/// Wraps an angle to `(-pi, pi]`.
pub fn wrap_angle(angle_rad: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let mut wrapped = angle_rad % two_pi;
    if wrapped > std::f64::consts::PI {
        wrapped -= two_pi;
    } else if wrapped <= -std::f64::consts::PI {
        wrapped += two_pi;
    }
    wrapped
}

fn transpose_5x5(matrix: &[[f64; STATE_DIM]; STATE_DIM]) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut out = [[0.0; STATE_DIM]; STATE_DIM];
    for (i, row) in matrix.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            out[j][i] = *value;
        }
    }
    out
}

fn mul_5x5(
    a: &[[f64; STATE_DIM]; STATE_DIM],
    b: &[[f64; STATE_DIM]; STATE_DIM],
) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut out = [[0.0; STATE_DIM]; STATE_DIM];
    for i in 0..STATE_DIM {
        for k in 0..STATE_DIM {
            let aik = a[i][k];
            if aik == 0.0 {
                continue;
            }
            for j in 0..STATE_DIM {
                out[i][j] += aik * b[k][j];
            }
        }
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn gps_pulls_a_dead_reckoned_estimate_back() {
        let config = EkfConfig::default();
        let mut filter = EkfFusion::new(config).unwrap();
        let dt = 0.1;
        let mut odom_only = 0.0;
        for step in 0..100 {
            filter.predict(dt).unwrap();
            filter.update_odometry(1.0, 0.0, 0.01, 0.01).unwrap();
            odom_only += 1.0 * dt;
            let truth = 1.0 * dt * (step + 1) as f64;
            filter.update_position(truth, 0.0, 0.01).unwrap();
        }
        let pose = filter.pose();
        assert_relative_eq!(pose.x_m, odom_only, epsilon = 1e-6);
        assert_relative_eq!(pose.y_m, 0.0, epsilon = 1e-6);
        assert_relative_eq!(filter.velocity().linear_m_s, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn position_fix_beats_pure_dead_reckoning_under_bias() {
        let mut fused = EkfFusion::new(EkfConfig::default()).unwrap();
        let dt = 0.1;
        let steps = 200;
        let mut dead_reckoning = Pose2d::IDENTITY;
        for step in 0..steps {
            fused.predict(dt).unwrap();
            fused.update_odometry(0.9, 0.0, 0.01, 0.01).unwrap();
            dead_reckoning.x_m += 0.9 * dt;
            let truth_x = 1.0 * dt * (step + 1) as f64;
            fused.update_position(truth_x, 0.0, 0.0004).unwrap();
        }
        let truth_x = 1.0 * dt * steps as f64;
        let fused_error = (fused.pose().x_m - truth_x).abs();
        let dead_error = (dead_reckoning.x_m - truth_x).abs();
        assert!(
            fused_error < dead_error,
            "fused {fused_error} dead {dead_error}"
        );
        assert!(fused_error < 0.05);
    }

    #[test]
    fn yaw_rate_update_tracks_rotation() {
        let mut filter = EkfFusion::new(EkfConfig::default()).unwrap();
        filter.update_yaw_rate(0.5, 0.001).unwrap();
        assert!((filter.velocity().angular_rad_s - 0.5).abs() < 0.01);
        assert!(filter.velocity().angular_rad_s > 0.49);
    }

    #[test]
    fn yaw_wraps_across_pi() {
        let mut filter = EkfFusion::with_initial_state(
            EkfConfig::default(),
            Pose2d::new(0.0, 0.0, 3.0),
            VelocityCommand2d::new(0.0, 1.0),
        )
        .unwrap();
        for _ in 0..10 {
            filter.predict(0.1).unwrap();
        }
        assert!(filter.pose().yaw_rad <= std::f64::consts::PI);
        assert!(filter.pose().yaw_rad > -std::f64::consts::PI);
        assert_relative_eq!(filter.pose().yaw_rad, wrap_angle(4.0), epsilon = 1e-9);
    }

    #[test]
    fn rejects_bad_inputs() {
        let mut filter = EkfFusion::new(EkfConfig::default()).unwrap();
        assert_eq!(filter.predict(0.0), Err(FusionError::NonPositiveTime));
        assert_eq!(filter.predict(f64::NAN), Err(FusionError::NonFiniteInput));
        assert_eq!(
            filter.update_position(0.0, 0.0, 0.0),
            Err(FusionError::InvalidVariance)
        );
        assert_eq!(
            filter.update_odometry(f64::NAN, 0.0, 0.1, 0.1),
            Err(FusionError::NonFiniteInput)
        );
    }

    #[test]
    fn replay_is_bit_identical() {
        let run = || {
            let mut filter = EkfFusion::new(EkfConfig::default()).unwrap();
            for step in 0..50 {
                filter.predict(0.05).unwrap();
                filter.update_odometry(0.5, 0.1, 0.02, 0.02).unwrap();
                filter.update_yaw_rate(0.1, 0.01).unwrap();
                if step % 5 == 0 {
                    filter
                        .update_position(0.025 * step as f64, 0.0, 0.01)
                        .unwrap();
                }
            }
            filter
        };
        let a = run();
        let b = run();
        assert_eq!(a.state(), b.state());
        assert_eq!(a.covariance(), b.covariance());
    }

    #[test]
    fn pose_with_covariance_exposes_the_planar_block() {
        let mut filter = EkfFusion::new(EkfConfig::default()).unwrap();
        filter.predict(0.1).unwrap();
        let estimate = filter.pose_with_covariance();
        assert_eq!(estimate.pose.x_m, filter.state()[0]);
        assert_eq!(estimate.covariance[0][0], filter.covariance()[0][0]);
        assert!(estimate.covariance[2][2] > 0.0);
    }

    #[test]
    fn fuses_a_geographic_fix_through_navsat() {
        let navsat = NavSatTransform::new(0.0, 0.0, 0.0).unwrap();
        let mut filter = EkfFusion::new(EkfConfig::default()).unwrap();
        // 1e-5 rad east/north is about 63.7 m.
        filter
            .update_navsat(&navsat, 1.0e-5, 1.0e-5, 0.0, 0.0004)
            .unwrap();
        let pose = filter.pose();
        assert!(pose.x_m > 0.0 && pose.y_m > 0.0);
        assert!((pose.x_m - pose.y_m).abs() < 1.0e-6);
    }

    #[test]
    fn map_from_odom_composes_back_to_the_fused_pose() {
        let fused = Pose2d::new(3.0, -1.0, 0.8);
        let odom = Pose2d::new(2.7, -0.9, 0.75);
        let map_from_odom_pose = map_from_odom(fused, odom);
        let recomposed = map_from_odom_pose.compose(odom);
        assert_relative_eq!(recomposed.x_m, fused.x_m, epsilon = 1e-12);
        assert_relative_eq!(recomposed.y_m, fused.y_m, epsilon = 1e-12);
        assert_relative_eq!(recomposed.yaw_rad, fused.yaw_rad, epsilon = 1e-12);
    }
}

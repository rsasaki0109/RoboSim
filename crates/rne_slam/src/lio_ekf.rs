//! Covariance-aware LiDAR-inertial odometry (pose EKF).
//!
//! Extends the predict-then-scan-match front-end with an explicit 6-DoF pose
//! covariance. IMU preintegration predicts the pose and inflates the covariance;
//! a point-to-plane scan-to-map match supplies a pose measurement whose
//! information diagonal comes from the registration normal equations, and a
//! Kalman update fuses them.
//!
//! This is a loosely-coupled pose EKF: the measurement is the scan-match pose
//! rather than the raw point residuals. It propagates and reduces covariance
//! deterministically; a tightly-coupled iterated EKF that keeps point residuals
//! in the filter is a later increment.

use crate::imu_preintegration::{ImuBias, ImuPreintegrator, ImuSample};
use crate::point_to_plane::{
    estimate_normals, IcpPointToPlane, PointToPlaneConfig, PointToPlaneError,
};
use crate::se3::Se3;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

const POSE_DIM: usize = 6;

/// Covariance-aware LiDAR-inertial odometry configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LioEkfConfig {
    /// Gravity vector in meters per second squared.
    pub gravity_m_s2: Vec3,
    /// IMU bias estimate.
    pub imu_bias: ImuBias,
    /// Point-to-plane registration configuration.
    pub icp: PointToPlaneConfig,
    /// Neighbourhood radius for local-map normals, in meters.
    pub normal_radius_m: f64,
    /// Voxel size for local-map downsampling, in meters.
    pub map_voxel_size_m: f64,
    /// Maximum number of points retained in the local map.
    pub max_map_points: usize,
    /// Process gyroscope noise density, rad/s per sqrt(s).
    pub process_gyro_noise_rad_s: f64,
    /// Process accelerometer noise density, m/s^2 per sqrt(s).
    pub process_accel_noise_m_s2: f64,
    /// Floor applied to the inverse of a registration information entry.
    pub min_measurement_variance: f64,
}

impl Default for LioEkfConfig {
    fn default() -> Self {
        Self {
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
            imu_bias: ImuBias::default(),
            icp: PointToPlaneConfig {
                target_voxel_size_m: 0.5,
                ..PointToPlaneConfig::default()
            },
            normal_radius_m: 0.35,
            map_voxel_size_m: 0.2,
            max_map_points: 50_000,
            process_gyro_noise_rad_s: 0.01,
            process_accel_noise_m_s2: 0.1,
            min_measurement_variance: 1.0e-4,
        }
    }
}

impl LioEkfConfig {
    fn is_valid(&self) -> bool {
        self.gravity_m_s2.is_finite()
            && self.normal_radius_m.is_finite()
            && self.normal_radius_m > 0.0
            && self.map_voxel_size_m.is_finite()
            && self.map_voxel_size_m > 0.0
            && self.max_map_points > 0
            && self.process_gyro_noise_rad_s.is_finite()
            && self.process_gyro_noise_rad_s >= 0.0
            && self.process_accel_noise_m_s2.is_finite()
            && self.process_accel_noise_m_s2 >= 0.0
            && self.min_measurement_variance.is_finite()
            && self.min_measurement_variance > 0.0
    }
}

/// LiDAR-inertial odometry failure.
#[derive(Debug, thiserror::Error)]
pub enum LioEkfError {
    /// The configuration was invalid.
    #[error("invalid LIO EKF configuration")]
    InvalidConfig,
    /// The scan was empty.
    #[error("scan cloud is empty")]
    EmptyScan,
    /// A scan sample was not finite.
    #[error("scan cloud contained a non-finite value")]
    NonFinite,
    /// The point-to-plane registration failed.
    #[error("scan registration failed: {0}")]
    Registration(#[from] PointToPlaneError),
}

/// Result of registering one scan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LioEkfUpdate {
    /// Corrected world pose of the sensor.
    pub pose: Se3,
    /// Estimated linear velocity in the world frame.
    pub velocity_m_s: Vec3,
    /// Trace of the corrected pose covariance.
    pub covariance_trace: f64,
    /// Point-to-plane correspondences used.
    pub correspondences: usize,
    /// Registration root-mean-square residual in meters.
    pub rmse_m: f64,
}

/// A covariance-aware LiDAR-inertial odometry front-end.
#[derive(Clone, Debug)]
pub struct LioEkf {
    config: LioEkfConfig,
    integrator: ImuPreintegrator,
    pose: Se3,
    velocity_m_s: Vec3,
    covariance: [[f64; POSE_DIM]; POSE_DIM],
    map_points: Vec<Vec3>,
    map_normals: Vec<Vec3>,
    map_voxels: BTreeSet<[i64; 3]>,
    initialized: bool,
}

impl LioEkf {
    /// Creates an odometry filter at the origin with a small initial covariance.
    pub fn new(config: LioEkfConfig) -> Result<Self, LioEkfError> {
        if !config.is_valid() {
            return Err(LioEkfError::InvalidConfig);
        }
        let mut covariance = [[0.0; POSE_DIM]; POSE_DIM];
        for (i, row) in covariance.iter_mut().enumerate() {
            // A large prior so the scan-to-map pose measurement is trusted.
            row[i] = 1.0;
        }
        Ok(Self {
            config,
            integrator: ImuPreintegrator::new(config.imu_bias),
            pose: Se3::IDENTITY,
            velocity_m_s: Vec3::ZERO,
            covariance,
            map_points: Vec::new(),
            map_normals: Vec::new(),
            map_voxels: BTreeSet::new(),
            initialized: false,
        })
    }

    /// Folds one IMU sample into the motion model.
    pub fn process_imu(&mut self, sample: &ImuSample) -> Result<(), LioEkfError> {
        self.integrator
            .integrate(sample)
            .map_err(|_| LioEkfError::NonFinite)
    }

    /// Current world pose.
    pub const fn pose(&self) -> Se3 {
        self.pose
    }

    /// Current world-frame linear velocity.
    pub const fn velocity_m_s(&self) -> Vec3 {
        self.velocity_m_s
    }

    /// Current 6x6 pose covariance in the rotation-first tangent space.
    pub const fn pose_covariance(&self) -> &[[f64; POSE_DIM]; POSE_DIM] {
        &self.covariance
    }

    /// Trace the covariance would have after inflating by the pending IMU interval.
    pub fn predicted_covariance_trace(&self) -> f64 {
        let dt = self.integrator.delta().dt_s;
        let rotation_variance = (self.config.process_gyro_noise_rad_s * dt).powi(2);
        let translation_variance = (self.config.process_accel_noise_m_s2 * 0.5 * dt * dt).powi(2);
        (0..POSE_DIM).map(|i| self.covariance[i][i]).sum::<f64>()
            + 3.0 * rotation_variance
            + 3.0 * translation_variance
    }

    /// Number of points in the local map.
    pub fn local_map_len(&self) -> usize {
        self.map_points.len()
    }

    /// Registers one scan (points in the sensor frame) and fuses it with IMU prediction.
    pub fn register_scan(&mut self, points: &[Vec3]) -> Result<LioEkfUpdate, LioEkfError> {
        if points.is_empty() {
            return Err(LioEkfError::EmptyScan);
        }
        if points.iter().any(|point| !point.is_finite()) {
            return Err(LioEkfError::NonFinite);
        }

        let dt = self.integrator.delta().dt_s;
        let (predicted_pose, predicted_velocity) =
            self.integrator
                .predict(self.pose, self.velocity_m_s, self.config.gravity_m_s2);

        // Inflate the covariance with process noise.
        if dt > 0.0 {
            let rotation_variance = (self.config.process_gyro_noise_rad_s * dt).powi(2);
            let translation_variance =
                (self.config.process_accel_noise_m_s2 * 0.5 * dt * dt).powi(2);
            for i in 0..3 {
                self.covariance[i][i] += rotation_variance;
                self.covariance[i + 3][i + 3] += translation_variance;
            }
        }

        let first_scan = !self.initialized || self.map_points.is_empty();
        let (corrected, correspondences, rmse_m) = if first_scan {
            (predicted_pose, 0, 0.0)
        } else {
            let result = IcpPointToPlane::align(
                points,
                &self.map_points,
                &self.map_normals,
                predicted_pose,
                &self.config.icp,
            )?;
            let (pose, covariance) =
                self.ekf_update(predicted_pose, result.transform, &result.information);
            self.covariance = covariance;
            (pose, result.correspondences, result.rmse_m)
        };

        let velocity = if dt > 0.0 {
            (corrected.translation - self.pose.translation) / dt
        } else {
            predicted_velocity
        };
        self.pose = corrected;
        self.velocity_m_s = velocity;
        self.initialized = true;

        self.integrate_scan_into_map(points, corrected);
        self.integrator.reset();

        Ok(LioEkfUpdate {
            pose: corrected,
            velocity_m_s: velocity,
            covariance_trace: (0..POSE_DIM).map(|i| self.covariance[i][i]).sum(),
            correspondences,
            rmse_m,
        })
    }

    /// Applies the Kalman update for a direct pose measurement.
    #[allow(clippy::needless_range_loop)]
    fn ekf_update(
        &self,
        predicted: Se3,
        measurement: Se3,
        information: &[f64; POSE_DIM],
    ) -> (Se3, [[f64; POSE_DIM]; POSE_DIM]) {
        // Innovation in the predicted pose's tangent space.
        let innovation = predicted.inverse().compose(measurement).log();
        // Measurement noise is the inverse of the registration information.
        let mut s = self.covariance;
        for i in 0..POSE_DIM {
            let variance =
                (1.0 / information[i].max(1.0e-9)).max(self.config.min_measurement_variance);
            s[i][i] += variance;
        }
        let Some(s_inverse) = invert6(&s) else {
            return (predicted, self.covariance);
        };
        // K = P * S^-1.
        let mut gain = [[0.0; POSE_DIM]; POSE_DIM];
        for i in 0..POSE_DIM {
            for j in 0..POSE_DIM {
                gain[i][j] = (0..POSE_DIM)
                    .map(|k| self.covariance[i][k] * s_inverse[k][j])
                    .sum();
            }
        }
        let mut delta = [0.0; POSE_DIM];
        for i in 0..POSE_DIM {
            delta[i] = (0..POSE_DIM).map(|j| gain[i][j] * innovation[j]).sum();
        }
        let corrected_pose = predicted.compose(Se3::exp(delta));
        // P = (I - K) P.
        let mut covariance = [[0.0; POSE_DIM]; POSE_DIM];
        for i in 0..POSE_DIM {
            for j in 0..POSE_DIM {
                let identity = if i == j { 1.0 } else { 0.0 };
                covariance[i][j] = (0..POSE_DIM)
                    .map(|k| (identity - gain[i][k]) * self.covariance[k][j])
                    .sum();
            }
        }
        (corrected_pose, covariance)
    }

    fn integrate_scan_into_map(&mut self, points: &[Vec3], pose: Se3) {
        let voxel = self.config.map_voxel_size_m;
        for point in points {
            let world = pose.transform_point(*point);
            if !world.is_finite() {
                continue;
            }
            if self.map_voxels.insert(voxel_key(world, voxel)) {
                self.map_points.push(world);
            }
        }
        if self.map_points.len() > self.config.max_map_points {
            let excess = self.map_points.len() - self.config.max_map_points;
            self.map_points.drain(0..excess);
            let retained = self.map_points.clone();
            self.map_voxels = retained.iter().map(|p| voxel_key(*p, voxel)).collect();
        }
        self.map_normals = estimate_normals(&self.map_points, self.config.normal_radius_m)
            .into_iter()
            .map(|normal| normal.unwrap_or(Vec3::Y))
            .collect();
    }
}

fn voxel_key(point: Vec3, voxel_size_m: f64) -> [i64; 3] {
    [
        (point.x / voxel_size_m).floor() as i64,
        (point.y / voxel_size_m).floor() as i64,
        (point.z / voxel_size_m).floor() as i64,
    ]
}

/// Inverts a symmetric positive-definite 6x6 matrix via Cholesky.
#[allow(clippy::needless_range_loop)]
pub(crate) fn invert6(matrix: &[[f64; POSE_DIM]; POSE_DIM]) -> Option<[[f64; POSE_DIM]; POSE_DIM]> {
    let mut jittered = *matrix;
    let trace: f64 = (0..POSE_DIM).map(|i| matrix[i][i]).sum();
    let jitter = 1.0e-9 * trace.abs().max(1.0);
    for i in 0..POSE_DIM {
        jittered[i][i] += jitter;
    }
    let matrix = &jittered;
    // Cholesky factor L with matrix = L L^T.
    let mut lower = [[0.0; POSE_DIM]; POSE_DIM];
    for row in 0..POSE_DIM {
        for col in 0..=row {
            let mut sum = matrix[row][col];
            for k in 0..col {
                sum -= lower[row][k] * lower[col][k];
            }
            if row == col {
                if sum <= 1.0e-18 {
                    return None;
                }
                lower[row][col] = sum.sqrt();
            } else {
                lower[row][col] = sum / lower[col][col];
            }
        }
    }
    // Invert the lower factor, then form the inverse.
    let mut lower_inverse = [[0.0; POSE_DIM]; POSE_DIM];
    for i in 0..POSE_DIM {
        lower_inverse[i][i] = 1.0 / lower[i][i];
        for j in 0..i {
            let mut sum = 0.0;
            for k in j..i {
                sum += lower[i][k] * lower_inverse[k][j];
            }
            lower_inverse[i][j] = -sum / lower[i][i];
        }
    }
    let mut inverse = [[0.0; POSE_DIM]; POSE_DIM];
    for i in 0..POSE_DIM {
        for j in 0..=i {
            let value = (j..=i)
                .map(|k| lower_inverse[k][j] * lower_inverse[k][i])
                .sum();
            inverse[i][j] = value;
            inverse[j][i] = value;
        }
    }
    Some(inverse)
}

/// Solves a symmetric positive-definite 6x6 system via Cholesky.
#[allow(clippy::needless_range_loop)]
pub(crate) fn solve6(
    matrix: &[[f64; POSE_DIM]; POSE_DIM],
    rhs: &[f64; POSE_DIM],
) -> Option<[f64; POSE_DIM]> {
    let mut jittered = *matrix;
    let trace: f64 = (0..POSE_DIM).map(|i| matrix[i][i]).sum();
    let jitter = 1.0e-9 * trace.abs().max(1.0);
    for i in 0..POSE_DIM {
        jittered[i][i] += jitter;
    }
    let matrix = &jittered;
    let mut lower = [[0.0; POSE_DIM]; POSE_DIM];
    for row in 0..POSE_DIM {
        for col in 0..=row {
            let mut sum = matrix[row][col];
            for k in 0..col {
                sum -= lower[row][k] * lower[col][k];
            }
            if row == col {
                if sum <= 1.0e-18 {
                    return None;
                }
                lower[row][col] = sum.sqrt();
            } else {
                lower[row][col] = sum / lower[col][col];
            }
        }
    }
    let mut y = [0.0; POSE_DIM];
    for row in 0..POSE_DIM {
        let mut sum = rhs[row];
        for k in 0..row {
            sum -= lower[row][k] * y[k];
        }
        y[row] = sum / lower[row][row];
    }
    let mut x = [0.0; POSE_DIM];
    for row in (0..POSE_DIM).rev() {
        let mut sum = y[row];
        for k in (row + 1)..POSE_DIM {
            sum -= lower[k][row] * x[k];
        }
        x[row] = sum / lower[row][row];
    }
    if x.iter().all(|value| value.is_finite()) {
        Some(x)
    } else {
        None
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::Quat;

    fn scene() -> Vec<Vec3> {
        let mut points = Vec::new();
        for i in 0..25 {
            for j in 0..25 {
                points.push(Vec3::new(i as f64 * 0.2, 0.0, j as f64 * 0.2));
            }
        }
        for i in 0..25 {
            for k in 0..8 {
                points.push(Vec3::new(2.0, i as f64 * 0.2, k as f64 * 0.2));
                points.push(Vec3::new(i as f64 * 0.2, k as f64 * 0.2, 2.0));
            }
        }
        points
    }

    fn scan_from(scene: &[Vec3], sensor_pose: Se3) -> Vec<Vec3> {
        let inverse = sensor_pose.inverse();
        scene
            .iter()
            .map(|point| inverse.transform_point(*point))
            .collect()
    }

    fn config() -> LioEkfConfig {
        LioEkfConfig {
            icp: PointToPlaneConfig {
                target_voxel_size_m: 0.3,
                max_correspondence_distance_m: 0.6,
                ..PointToPlaneConfig::default()
            },
            map_voxel_size_m: 0.15,
            ..LioEkfConfig::default()
        }
    }

    #[test]
    fn covariance_grows_with_prediction_and_shrinks_with_a_scan() {
        let scene = scene();
        let mut lio = LioEkf::new(config()).expect("lio");
        lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
            .expect("first");
        let after_first = lio.pose_covariance()[0][0];

        // IMU-only prediction inflates the covariance.
        lio.process_imu(&ImuSample {
            dt_s: 0.1,
            accel_m_s2: Vec3::new(0.0, 9.81, 0.0),
            gyro_rad_s: Vec3::ZERO,
        })
        .expect("imu");
        let predicted_trace = lio.predicted_covariance_trace();
        assert!(predicted_trace > after_first);
        let predicted_pose = lio
            .integrator
            .predict(lio.pose(), lio.velocity_m_s(), lio.config.gravity_m_s2)
            .0;

        // A scan update reduces the covariance again.
        let update = lio
            .register_scan(&scan_from(&scene, predicted_pose))
            .expect("second");
        assert!(update.covariance_trace < predicted_trace);
    }

    #[test]
    fn tracks_a_known_motion() {
        let scene = scene();
        let mut lio = LioEkf::new(config()).expect("lio");
        lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
            .expect("first");
        let truth = Se3::new(Quat::from_rotation_y(0.03), Vec3::new(0.12, 0.0, 0.06));
        let update = lio
            .register_scan(&scan_from(&scene, truth))
            .expect("second");
        assert!(
            (update.pose.translation - truth.translation).length() < 0.03,
            "pose error {}",
            (update.pose.translation - truth.translation).length()
        );
        assert!(update.covariance_trace > 0.0);
    }

    #[test]
    fn replay_is_deterministic() {
        let scene = scene();
        let run = || {
            let mut lio = LioEkf::new(config()).expect("lio");
            let poses = [
                Se3::IDENTITY,
                Se3::new(Quat::from_rotation_y(0.02), Vec3::new(0.1, 0.0, 0.05)),
                Se3::new(Quat::from_rotation_y(0.04), Vec3::new(0.2, 0.0, 0.1)),
            ];
            let mut out = Vec::new();
            for pose in poses {
                let update = lio.register_scan(&scan_from(&scene, pose)).expect("scan");
                out.push((update.pose, update.covariance_trace));
            }
            out
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn rejects_bad_inputs() {
        assert!(LioEkf::new(LioEkfConfig {
            normal_radius_m: 0.0,
            ..LioEkfConfig::default()
        })
        .is_err());
        let mut lio = LioEkf::new(config()).expect("lio");
        assert!(matches!(
            lio.register_scan(&[]),
            Err(LioEkfError::EmptyScan)
        ));
        assert_relative_eq!(Vec3::ZERO.x, 0.0);
    }
}

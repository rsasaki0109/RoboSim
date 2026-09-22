//! Tightly-coupled iterated EKF for LiDAR-inertial odometry.
//!
//! Unlike the loosely-coupled [`crate::lio_ekf::LioEkf`], this filter feeds the
//! **raw point-to-plane residuals** directly into the update. The pose prior comes
//! from IMU preintegration; each scan is fused by an iterated information-form
//! Gauss-Newton step: re-linearize the residuals at the current pose estimate,
//! accumulate `H = sum J J^T` and `g = sum J r`, and solve
//! `(P^-1 + H/sigma^2) delta = g/sigma^2`, updating `pose <- pose * Exp(delta)`
//! and `P <- (P^-1 + H/sigma^2)^-1`. Everything is deterministic and needs no
//! NxN matrix inversion.
//!
//! The state is pose-only (6-DoF). Velocity and IMU-bias estimation in the filter
//! is a later increment.

use crate::imu_preintegration::{ImuBias, ImuPreintegrator, ImuSample};
use crate::lio_ekf::{invert6, solve6};
use crate::point_to_plane::{
    estimate_normals, PointToPlaneConfig, PointToPlaneError, VoxelPointIndex,
};
use crate::se3::Se3;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

const POSE_DIM: usize = 6;

/// Tightly-coupled iEKF configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LioIekfConfig {
    /// Gravity vector in meters per second squared.
    pub gravity_m_s2: Vec3,
    /// IMU bias estimate.
    pub imu_bias: ImuBias,
    /// Registration gating and stride configuration.
    pub icp: PointToPlaneConfig,
    /// Neighbourhood radius for local-map normals, in meters.
    pub normal_radius_m: f64,
    /// Maximum number of points retained in the local map.
    pub max_map_points: usize,
    /// Local-map voxel size in meters.
    pub map_voxel_size_m: f64,
    /// Point-to-plane residual variance in square meters.
    pub point_noise_variance_m2: f64,
    /// Iterated update steps per scan.
    pub iterations: usize,
    /// Twist step below which the iteration converges.
    pub step_tolerance: f64,
    /// Initial pose covariance diagonal.
    pub initial_covariance: f64,
    /// Process gyroscope noise density, rad/s per sqrt(s).
    pub process_gyro_noise_rad_s: f64,
    /// Process accelerometer noise density, m/s^2 per sqrt(s).
    pub process_accel_noise_m_s2: f64,
}

impl Default for LioIekfConfig {
    fn default() -> Self {
        Self {
            gravity_m_s2: Vec3::new(0.0, -9.81, 0.0),
            imu_bias: ImuBias::default(),
            icp: PointToPlaneConfig {
                target_voxel_size_m: 0.5,
                ..PointToPlaneConfig::default()
            },
            normal_radius_m: 0.35,
            max_map_points: 50_000,
            map_voxel_size_m: 0.2,
            point_noise_variance_m2: 1.0e-4,
            iterations: 3,
            step_tolerance: 1.0e-6,
            initial_covariance: 1.0,
            process_gyro_noise_rad_s: 0.01,
            process_accel_noise_m_s2: 0.1,
        }
    }
}

impl LioIekfConfig {
    fn is_valid(&self) -> bool {
        self.gravity_m_s2.is_finite()
            && self.normal_radius_m.is_finite()
            && self.normal_radius_m > 0.0
            && self.map_voxel_size_m.is_finite()
            && self.map_voxel_size_m > 0.0
            && self.max_map_points > 0
            && self.point_noise_variance_m2.is_finite()
            && self.point_noise_variance_m2 > 0.0
            && self.iterations > 0
            && self.step_tolerance.is_finite()
            && self.step_tolerance > 0.0
            && self.initial_covariance.is_finite()
            && self.initial_covariance > 0.0
            && self.process_gyro_noise_rad_s.is_finite()
            && self.process_gyro_noise_rad_s >= 0.0
            && self.process_accel_noise_m_s2.is_finite()
            && self.process_accel_noise_m_s2 >= 0.0
    }
}

/// Tightly-coupled iEKF failure.
#[derive(Debug, thiserror::Error)]
pub enum LioIekfError {
    /// The configuration was invalid.
    #[error("invalid LIO iEKF configuration")]
    InvalidConfig,
    /// The scan was empty.
    #[error("scan cloud is empty")]
    EmptyScan,
    /// A scan sample was not finite.
    #[error("scan cloud contained a non-finite value")]
    NonFinite,
    /// The information system was singular.
    #[error("LIO iEKF information system is singular")]
    Singular,
    /// A registration helper failed.
    #[error("registration failed: {0}")]
    Registration(#[from] PointToPlaneError),
}

/// Result of registering one scan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LioIekfUpdate {
    /// Corrected world pose of the sensor.
    pub pose: Se3,
    /// Estimated linear velocity in the world frame.
    pub velocity_m_s: Vec3,
    /// Trace of the corrected pose covariance.
    pub covariance_trace: f64,
    /// Point-to-plane correspondences used.
    pub correspondences: usize,
    /// Root-mean-square point-to-plane residual in meters.
    pub rmse_m: f64,
}

/// A tightly-coupled iterated-EKF LiDAR-inertial front-end.
#[derive(Clone, Debug)]
pub struct LioIekf {
    config: LioIekfConfig,
    integrator: ImuPreintegrator,
    pose: Se3,
    velocity_m_s: Vec3,
    covariance: [[f64; POSE_DIM]; POSE_DIM],
    map_points: Vec<Vec3>,
    map_normals: Vec<Vec3>,
    map_voxels: BTreeSet<[i64; 3]>,
    initialized: bool,
}

impl LioIekf {
    /// Creates a filter at the origin with the configured initial covariance.
    pub fn new(config: LioIekfConfig) -> Result<Self, LioIekfError> {
        if !config.is_valid() {
            return Err(LioIekfError::InvalidConfig);
        }
        let mut covariance = [[0.0; POSE_DIM]; POSE_DIM];
        for (i, row) in covariance.iter_mut().enumerate() {
            row[i] = config.initial_covariance;
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
    pub fn process_imu(&mut self, sample: &ImuSample) -> Result<(), LioIekfError> {
        self.integrator
            .integrate(sample)
            .map_err(|_| LioIekfError::NonFinite)
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

    /// Number of points in the local map.
    pub fn local_map_len(&self) -> usize {
        self.map_points.len()
    }

    /// Registers one scan (points in the sensor frame) with a raw-residual update.
    pub fn register_scan(&mut self, points: &[Vec3]) -> Result<LioIekfUpdate, LioIekfError> {
        if points.is_empty() {
            return Err(LioIekfError::EmptyScan);
        }
        if points.iter().any(|point| !point.is_finite()) {
            return Err(LioIekfError::NonFinite);
        }

        let dt = self.integrator.delta().dt_s;
        let (predicted_pose, predicted_velocity) =
            self.integrator
                .predict(self.pose, self.velocity_m_s, self.config.gravity_m_s2);

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
            self.iterated_update(points, predicted_pose)?
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

        Ok(LioIekfUpdate {
            pose: corrected,
            velocity_m_s: velocity,
            covariance_trace: (0..POSE_DIM).map(|i| self.covariance[i][i]).sum(),
            correspondences,
            rmse_m,
        })
    }

    /// Runs the iterated information-form update using raw point residuals.
    #[allow(clippy::needless_range_loop)]
    fn iterated_update(
        &mut self,
        points: &[Vec3],
        predicted_pose: Se3,
    ) -> Result<(Se3, usize, f64), LioIekfError> {
        let index = VoxelPointIndex::new(&self.map_points, self.config.icp.target_voxel_size_m);
        let prior_information = invert6(&self.covariance).ok_or(LioIekfError::Singular)?;
        let inverse_variance = 1.0 / self.config.point_noise_variance_m2;
        let mut current = predicted_pose;
        let mut correspondences = 0;
        let mut rmse_m = 0.0;
        let mut last_information = prior_information;

        for _ in 0..self.config.iterations {
            let mut h = [[0.0; POSE_DIM]; POSE_DIM];
            let mut g = [0.0; POSE_DIM];
            let mut residual_squared = 0.0;
            let mut inliers = 0;
            for point in points.iter().step_by(self.config.icp.source_stride.max(1)) {
                let world = current.transform_point(*point);
                let Some((nearest, distance)) = index.nearest(world) else {
                    break;
                };
                if distance > self.config.icp.max_correspondence_distance_m {
                    continue;
                }
                let normal_world = self.map_normals[nearest];
                let length = normal_world.length();
                if !length.is_finite() || length < 1.0e-9 {
                    continue;
                }
                let normal_world = normal_world / length;
                let normal_local = current.rotation.conjugate().mul_vec3(normal_world);
                let residual = normal_world.dot(world - self.map_points[nearest]);
                let point_normal = point.cross(normal_local);
                let jacobian = [
                    point_normal.x,
                    point_normal.y,
                    point_normal.z,
                    normal_local.x,
                    normal_local.y,
                    normal_local.z,
                ];
                for row in 0..POSE_DIM {
                    for col in 0..POSE_DIM {
                        h[row][col] += jacobian[row] * jacobian[col];
                    }
                    g[row] += jacobian[row] * residual;
                }
                residual_squared += residual * residual;
                inliers += 1;
            }
            correspondences = inliers;
            if inliers < self.config.icp.min_correspondences {
                break;
            }
            rmse_m = (residual_squared / inliers as f64).sqrt();

            let mut information = prior_information;
            let mut rhs = [0.0; POSE_DIM];
            for row in 0..POSE_DIM {
                for col in 0..POSE_DIM {
                    information[row][col] += inverse_variance * h[row][col];
                }
                rhs[row] = -inverse_variance * g[row];
            }
            last_information = information;
            let delta = solve6(&information, &rhs).ok_or(LioIekfError::Singular)?;
            current = current.compose(Se3::exp(delta));
            let max_step = delta
                .iter()
                .fold(0.0_f64, |max, value| max.max(value.abs()));
            if max_step <= self.config.step_tolerance {
                break;
            }
        }

        let mut covariance = invert6(&last_information).ok_or(LioIekfError::Singular)?;
        // Keep the covariance symmetric and strictly positive so later
        // information updates stay well conditioned.
        for i in 0..POSE_DIM {
            covariance[i][i] = covariance[i][i].max(1.0e-6);
            for j in 0..i {
                let symmetric = 0.5 * (covariance[i][j] + covariance[j][i]);
                covariance[i][j] = symmetric;
                covariance[j][i] = symmetric;
            }
        }
        self.covariance = covariance;
        Ok((current, correspondences, rmse_m))
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

    fn config() -> LioIekfConfig {
        LioIekfConfig {
            icp: PointToPlaneConfig {
                target_voxel_size_m: 0.3,
                max_correspondence_distance_m: 0.6,
                ..PointToPlaneConfig::default()
            },
            map_voxel_size_m: 0.15,
            ..LioIekfConfig::default()
        }
    }

    #[test]
    fn tracks_a_known_motion_tightly() {
        let scene = scene();
        let mut lio = LioIekf::new(config()).expect("lio");
        lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
            .expect("first");
        let truth = Se3::new(Quat::from_rotation_y(0.03), Vec3::new(0.12, 0.0, 0.06));
        let update = lio
            .register_scan(&scan_from(&scene, truth))
            .expect("second");
        let error = (update.pose.translation - truth.translation).length();
        assert!(error < 0.02, "pose error {error}");
        assert!(update.rmse_m < 0.02);
        assert!(update.covariance_trace > 0.0);
    }

    #[test]
    fn covariance_shrinks_with_a_scan() {
        let scene = scene();
        let mut lio = LioIekf::new(config()).expect("lio");
        lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
            .expect("first");
        let before: f64 = (0..POSE_DIM).map(|i| lio.pose_covariance()[i][i]).sum();
        lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
            .expect("second");
        let after: f64 = (0..POSE_DIM).map(|i| lio.pose_covariance()[i][i]).sum();
        assert!(after < before, "covariance {before} -> {after}");
    }

    #[test]
    fn replay_is_deterministic() {
        let scene = scene();
        let run = || {
            let mut lio = LioIekf::new(config()).expect("lio");
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
        assert!(LioIekf::new(LioIekfConfig {
            iterations: 0,
            ..LioIekfConfig::default()
        })
        .is_err());
        let mut lio = LioIekf::new(config()).expect("lio");
        assert!(matches!(
            lio.register_scan(&[]),
            Err(LioIekfError::EmptyScan)
        ));
        assert_relative_eq!(Vec3::ZERO.x, 0.0);
    }
}

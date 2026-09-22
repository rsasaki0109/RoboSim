//! Velocity- and bias-augmented tightly-coupled iEKF.
//!
//! Extends the pose-only [`crate::lio_iekf::LioIekf`] with velocity and IMU-bias
//! states. The error state is `[rotation(3), translation(3), velocity(3),
//! gyro_bias(3), accel_bias(3)]`. IMU samples propagate pose, velocity, and the
//! 15x15 error-state covariance; each scan feeds raw point-to-plane residuals
//! into the iterated information-form update. Pose-only residuals observe the
//! biases only through the covariance correlation built up during propagation,
//! so bias estimation needs a short window of motion or scan mismatch.

use crate::imu_preintegration::ImuBias;
use crate::point_to_plane::{
    estimate_normals, PointToPlaneConfig, PointToPlaneError, VoxelPointIndex,
};
use crate::se3::Se3;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

const STATE_DIM: usize = 15;

/// Velocity- and bias-augmented iEKF configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LioInertialConfig {
    /// Gravity vector in meters per second squared.
    pub gravity_m_s2: Vec3,
    /// Initial IMU bias estimate.
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
    /// Initial position covariance diagonal.
    pub initial_position_variance: f64,
    /// Initial velocity covariance diagonal.
    pub initial_velocity_variance: f64,
    /// Initial gyro/accel bias covariance diagonal.
    pub initial_bias_variance: f64,
    /// Process gyroscope noise density, rad/s per sqrt(s).
    pub process_gyro_noise_rad_s: f64,
    /// Process accelerometer noise density, m/s^2 per sqrt(s).
    pub process_accel_noise_m_s2: f64,
    /// Gyro bias random-walk density, rad/s per sqrt(s).
    pub process_gyro_bias_noise_rad_s: f64,
    /// Accel bias random-walk density, m/s^2 per sqrt(s).
    pub process_accel_bias_noise_m_s2: f64,
}

impl Default for LioInertialConfig {
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
            initial_position_variance: 1.0,
            initial_velocity_variance: 1.0,
            initial_bias_variance: 1.0e-2,
            process_gyro_noise_rad_s: 0.01,
            process_accel_noise_m_s2: 0.1,
            process_gyro_bias_noise_rad_s: 1.0e-4,
            process_accel_bias_noise_m_s2: 1.0e-3,
        }
    }
}

impl LioInertialConfig {
    /// Returns true when every parameter is finite and valid.
    pub fn is_valid(&self) -> bool {
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
            && self.initial_position_variance.is_finite()
            && self.initial_position_variance > 0.0
            && self.initial_velocity_variance.is_finite()
            && self.initial_velocity_variance > 0.0
            && self.initial_bias_variance.is_finite()
            && self.initial_bias_variance > 0.0
            && self.process_gyro_noise_rad_s.is_finite()
            && self.process_gyro_noise_rad_s >= 0.0
            && self.process_accel_noise_m_s2.is_finite()
            && self.process_accel_noise_m_s2 >= 0.0
            && self.process_gyro_bias_noise_rad_s.is_finite()
            && self.process_gyro_bias_noise_rad_s >= 0.0
            && self.process_accel_bias_noise_m_s2.is_finite()
            && self.process_accel_bias_noise_m_s2 >= 0.0
    }
}

/// Velocity- and bias-augmented iEKF failure.
#[derive(Debug, thiserror::Error)]
pub enum LioInertialError {
    /// The configuration was invalid.
    #[error("invalid inertial LIO configuration")]
    InvalidConfig,
    /// The scan was empty.
    #[error("scan cloud is empty")]
    EmptyScan,
    /// A scan or IMU sample was not finite.
    #[error("input contained a non-finite value")]
    NonFinite,
    /// A sample interval was non-positive.
    #[error("IMU sample dt must be finite and positive")]
    InvalidDt,
    /// The information system was singular.
    #[error("inertial LIO information system is singular")]
    Singular,
    /// A registration helper failed.
    #[error("registration failed: {0}")]
    Registration(#[from] PointToPlaneError),
}

/// Result of registering one scan.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LioInertialUpdate {
    /// Corrected world pose of the sensor.
    pub pose: Se3,
    /// Estimated world-frame linear velocity.
    pub velocity_m_s: Vec3,
    /// Estimated gyroscope bias.
    pub gyro_bias_rad_s: Vec3,
    /// Estimated accelerometer bias.
    pub accel_bias_m_s2: Vec3,
    /// Trace of the 15x15 error-state covariance.
    pub covariance_trace: f64,
    /// Point-to-plane correspondences used.
    pub correspondences: usize,
    /// Root-mean-square point-to-plane residual in meters.
    pub rmse_m: f64,
}

/// A velocity- and bias-augmented tightly-coupled iEKF.
#[derive(Clone, Debug)]
pub struct LioInertialEkf {
    config: LioInertialConfig,
    pose: Se3,
    velocity_m_s: Vec3,
    gyro_bias_rad_s: Vec3,
    accel_bias_m_s2: Vec3,
    covariance: [[f64; STATE_DIM]; STATE_DIM],
    map_points: Vec<Vec3>,
    map_normals: Vec<Vec3>,
    initialized: bool,
}

impl LioInertialEkf {
    /// Creates a filter at rest at the origin.
    #[allow(clippy::needless_range_loop)]
    pub fn new(config: LioInertialConfig) -> Result<Self, LioInertialError> {
        if !config.is_valid() {
            return Err(LioInertialError::InvalidConfig);
        }
        let mut covariance = [[0.0; STATE_DIM]; STATE_DIM];
        for i in 0..6 {
            covariance[i][i] = config.initial_position_variance;
        }
        for i in 6..9 {
            covariance[i][i] = config.initial_velocity_variance;
        }
        for i in 9..STATE_DIM {
            covariance[i][i] = config.initial_bias_variance;
        }
        Ok(Self {
            config,
            pose: Se3::IDENTITY,
            velocity_m_s: Vec3::ZERO,
            gyro_bias_rad_s: config.imu_bias.gyro_rad_s,
            accel_bias_m_s2: config.imu_bias.accel_m_s2,
            covariance,
            map_points: Vec::new(),
            map_normals: Vec::new(),
            initialized: false,
        })
    }

    /// Current world pose.
    pub const fn pose(&self) -> Se3 {
        self.pose
    }

    /// Current world-frame linear velocity.
    pub const fn velocity_m_s(&self) -> Vec3 {
        self.velocity_m_s
    }

    /// Current gyroscope bias estimate.
    pub const fn gyro_bias_rad_s(&self) -> Vec3 {
        self.gyro_bias_rad_s
    }

    /// Current accelerometer bias estimate.
    pub const fn accel_bias_m_s2(&self) -> Vec3 {
        self.accel_bias_m_s2
    }

    /// Current 15x15 error-state covariance.
    pub const fn covariance(&self) -> &[[f64; STATE_DIM]; STATE_DIM] {
        &self.covariance
    }

    /// Number of points in the local map.
    pub fn local_map_len(&self) -> usize {
        self.map_points.len()
    }

    /// Propagates pose, velocity, biases, and covariance by one IMU sample.
    #[allow(clippy::needless_range_loop)]
    pub fn process_imu(
        &mut self,
        dt_s: f64,
        accel_m_s2: Vec3,
        gyro_rad_s: Vec3,
    ) -> Result<(), LioInertialError> {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return Err(LioInertialError::InvalidDt);
        }
        if !accel_m_s2.is_finite() || !gyro_rad_s.is_finite() {
            return Err(LioInertialError::NonFinite);
        }
        let gyro = gyro_rad_s - self.gyro_bias_rad_s;
        let accel = accel_m_s2 - self.accel_bias_m_s2;

        let rotation_delta = crate::se3::so3_exp(gyro * dt_s);
        let body_accel_world = self.pose.rotation.mul_vec3(accel);
        let world_accel = body_accel_world + self.config.gravity_m_s2;

        self.pose = Se3 {
            rotation: (self.pose.rotation * rotation_delta).normalize(),
            translation: self.pose.translation
                + self.velocity_m_s * dt_s
                + world_accel * (0.5 * dt_s * dt_s),
        };
        self.velocity_m_s += world_accel * dt_s;

        // Error-state transition F (ordering rotation, translation, velocity,
        // gyro bias, accel bias).
        let skew = skew(body_accel_world);
        let rotation = self.pose.rotation;
        let rotation_matrix = [
            [
                rotation.mul_vec3(Vec3::X).x,
                rotation.mul_vec3(Vec3::Y).x,
                rotation.mul_vec3(Vec3::Z).x,
            ],
            [
                rotation.mul_vec3(Vec3::X).y,
                rotation.mul_vec3(Vec3::Y).y,
                rotation.mul_vec3(Vec3::Z).y,
            ],
            [
                rotation.mul_vec3(Vec3::X).z,
                rotation.mul_vec3(Vec3::Y).z,
                rotation.mul_vec3(Vec3::Z).z,
            ],
        ];
        let mut f = identity();
        for row in 0..3 {
            for col in 0..3 {
                f[row + 3][col + 6] = if row == col { dt_s } else { 0.0 };
                f[row + 6][col] = -skew[row][col] * dt_s;
                f[row + 6][col + 12] = -rotation_matrix[row][col] * dt_s;
            }
            f[row][row + 9] = -dt_s;
        }
        // P = F P F^T + Q.
        let fp = mat_mul(&f, &self.covariance);
        let ft = transpose(&f);
        let mut covariance = mat_mul(&fp, &ft);
        let rotation_variance = (self.config.process_gyro_noise_rad_s * dt_s).powi(2);
        let velocity_variance = (self.config.process_accel_noise_m_s2 * dt_s).powi(2);
        let position_variance = (self.config.process_accel_noise_m_s2 * 0.5 * dt_s * dt_s).powi(2);
        let gyro_bias_variance = (self.config.process_gyro_bias_noise_rad_s * dt_s).powi(2);
        let accel_bias_variance = (self.config.process_accel_bias_noise_m_s2 * dt_s).powi(2);
        for i in 0..3 {
            covariance[i][i] += rotation_variance;
            covariance[i + 3][i + 3] += position_variance;
            covariance[i + 6][i + 6] += velocity_variance;
            covariance[i + 9][i + 9] += gyro_bias_variance;
            covariance[i + 12][i + 12] += accel_bias_variance;
        }
        self.covariance = symmetrize(covariance);
        Ok(())
    }

    /// Registers one scan (points in the sensor frame) with a raw-residual update.
    pub fn register_scan(
        &mut self,
        points: &[Vec3],
    ) -> Result<LioInertialUpdate, LioInertialError> {
        if points.is_empty() {
            return Err(LioInertialError::EmptyScan);
        }
        if points.iter().any(|point| !point.is_finite()) {
            return Err(LioInertialError::NonFinite);
        }
        let first_scan = !self.initialized || self.map_points.is_empty();
        let (correspondences, rmse_m) = if first_scan {
            self.initialized = true;
            (0, 0.0)
        } else {
            self.iterated_update(points)?
        };

        self.integrate_scan_into_map(points);
        Ok(LioInertialUpdate {
            pose: self.pose,
            velocity_m_s: self.velocity_m_s,
            gyro_bias_rad_s: self.gyro_bias_rad_s,
            accel_bias_m_s2: self.accel_bias_m_s2,
            covariance_trace: (0..STATE_DIM).map(|i| self.covariance[i][i]).sum(),
            correspondences,
            rmse_m,
        })
    }

    #[allow(clippy::needless_range_loop)]
    fn iterated_update(&mut self, points: &[Vec3]) -> Result<(usize, f64), LioInertialError> {
        let index = VoxelPointIndex::new(&self.map_points, self.config.icp.target_voxel_size_m);
        let prior_information = invert(&self.covariance).ok_or(LioInertialError::Singular)?;
        let inverse_variance = 1.0 / self.config.point_noise_variance_m2;
        let mut pose = self.pose;
        let mut velocity = self.velocity_m_s;
        let mut gyro_bias = self.gyro_bias_rad_s;
        let mut accel_bias = self.accel_bias_m_s2;
        let mut correspondences = 0;
        let mut rmse_m = 0.0;
        let mut last_information = prior_information;

        for _ in 0..self.config.iterations {
            let mut h = [[0.0; STATE_DIM]; STATE_DIM];
            let mut g = [0.0; STATE_DIM];
            let mut residual_squared = 0.0;
            let mut inliers = 0;
            for point in points.iter().step_by(self.config.icp.source_stride.max(1)) {
                let world = pose.transform_point(*point);
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
                let normal_local = pose.rotation.conjugate().mul_vec3(normal_world);
                let residual = normal_world.dot(world - self.map_points[nearest]);
                let point_normal = point.cross(normal_local);
                let mut jacobian = [0.0; STATE_DIM];
                jacobian[0] = point_normal.x;
                jacobian[1] = point_normal.y;
                jacobian[2] = point_normal.z;
                jacobian[3] = normal_local.x;
                jacobian[4] = normal_local.y;
                jacobian[5] = normal_local.z;
                for row in 0..STATE_DIM {
                    for col in 0..STATE_DIM {
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
            let mut rhs = [0.0; STATE_DIM];
            for row in 0..STATE_DIM {
                for col in 0..STATE_DIM {
                    information[row][col] += inverse_variance * h[row][col];
                }
                rhs[row] = -inverse_variance * g[row];
            }
            last_information = information;
            let delta = solve(&information, &rhs).ok_or(LioInertialError::Singular)?;
            let mut twist = [0.0; 6];
            twist.copy_from_slice(&delta[0..6]);
            pose = pose.compose(Se3::exp(twist));
            velocity += Vec3::new(delta[6], delta[7], delta[8]);
            gyro_bias += Vec3::new(delta[9], delta[10], delta[11]);
            accel_bias += Vec3::new(delta[12], delta[13], delta[14]);
            let max_step = delta
                .iter()
                .fold(0.0_f64, |max, value| max.max(value.abs()));
            if max_step <= self.config.step_tolerance {
                break;
            }
        }

        self.pose = pose;
        self.velocity_m_s = velocity;
        self.gyro_bias_rad_s = gyro_bias;
        self.accel_bias_m_s2 = accel_bias;
        let covariance = invert(&last_information).ok_or(LioInertialError::Singular)?;
        self.covariance = ensure_positive_definite(symmetrize(covariance));
        Ok((correspondences, rmse_m))
    }

    fn integrate_scan_into_map(&mut self, points: &[Vec3]) {
        let voxel = self.config.map_voxel_size_m;
        let mut keys = std::collections::BTreeSet::new();
        for point in points {
            let world = self.pose.transform_point(*point);
            if !world.is_finite() {
                continue;
            }
            let key = [
                (world.x / voxel).floor() as i64,
                (world.y / voxel).floor() as i64,
                (world.z / voxel).floor() as i64,
            ];
            if keys.insert(key) {
                self.map_points.push(world);
            }
        }
        if self.map_points.len() > self.config.max_map_points {
            let excess = self.map_points.len() - self.config.max_map_points;
            self.map_points.drain(0..excess);
        }
        self.map_normals = estimate_normals(&self.map_points, self.config.normal_radius_m)
            .into_iter()
            .map(|normal| normal.unwrap_or(Vec3::Y))
            .collect();
    }
}

fn identity() -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut out = [[0.0; STATE_DIM]; STATE_DIM];
    for (i, row) in out.iter_mut().enumerate() {
        row[i] = 1.0;
    }
    out
}

fn transpose(matrix: &[[f64; STATE_DIM]; STATE_DIM]) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut out = [[0.0; STATE_DIM]; STATE_DIM];
    for row in 0..STATE_DIM {
        for col in 0..STATE_DIM {
            out[col][row] = matrix[row][col];
        }
    }
    out
}

fn mat_mul(
    left: &[[f64; STATE_DIM]; STATE_DIM],
    right: &[[f64; STATE_DIM]; STATE_DIM],
) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut out = [[0.0; STATE_DIM]; STATE_DIM];
    for row in 0..STATE_DIM {
        for col in 0..STATE_DIM {
            out[row][col] = (0..STATE_DIM).map(|k| left[row][k] * right[k][col]).sum();
        }
    }
    out
}

fn symmetrize(matrix: [[f64; STATE_DIM]; STATE_DIM]) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut out = matrix;
    for row in 0..STATE_DIM {
        for col in 0..row {
            let symmetric = 0.5 * (matrix[row][col] + matrix[col][row]);
            out[row][col] = symmetric;
            out[col][row] = symmetric;
        }
    }
    out
}

#[allow(clippy::needless_range_loop)]
fn ensure_positive_definite(
    matrix: [[f64; STATE_DIM]; STATE_DIM],
) -> [[f64; STATE_DIM]; STATE_DIM] {
    // Gershgorin lower bound on the smallest eigenvalue; shift the spectrum so
    // the covariance stays positive definite under later information inversion.
    let mut lower_bound = f64::INFINITY;
    for row in 0..STATE_DIM {
        let off_diagonal: f64 = (0..STATE_DIM)
            .filter(|&col| col != row)
            .map(|col| matrix[row][col].abs())
            .sum();
        lower_bound = lower_bound.min(matrix[row][row] - off_diagonal);
    }
    let floor = 1.0e-12 * diagonal_scale(&matrix);
    let loading = (-lower_bound).max(0.0) + floor;
    let mut out = matrix;
    for (i, row) in out.iter_mut().enumerate() {
        row[i] += loading;
    }
    symmetrize(out)
}

fn skew(vector: Vec3) -> [[f64; 3]; 3] {
    [
        [0.0, -vector.z, vector.y],
        [vector.z, 0.0, -vector.x],
        [-vector.y, vector.x, 0.0],
    ]
}

#[allow(clippy::needless_range_loop)]
fn cholesky(matrix: &[[f64; STATE_DIM]; STATE_DIM]) -> Option<[[f64; STATE_DIM]; STATE_DIM]> {
    let mut lower = [[0.0; STATE_DIM]; STATE_DIM];
    for row in 0..STATE_DIM {
        for col in 0..=row {
            let mut sum = matrix[row][col];
            for k in 0..col {
                sum -= lower[row][k] * lower[col][k];
            }
            if row == col {
                if sum <= 0.0 || sum.is_nan() {
                    return None;
                }
                lower[row][col] = sum.sqrt();
            } else {
                lower[row][col] = sum / lower[col][col];
            }
        }
    }
    Some(lower)
}

#[allow(clippy::needless_range_loop)]
fn regularize(
    matrix: &[[f64; STATE_DIM]; STATE_DIM],
    jitter: f64,
) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut out = *matrix;
    for i in 0..STATE_DIM {
        out[i][i] += jitter;
    }
    out
}

fn diagonal_scale(matrix: &[[f64; STATE_DIM]; STATE_DIM]) -> f64 {
    let scale = (0..STATE_DIM)
        .map(|i| matrix[i][i].abs())
        .fold(0.0_f64, f64::max);
    if scale > 0.0 {
        scale
    } else {
        1.0
    }
}

#[allow(clippy::needless_range_loop)]
fn invert(matrix: &[[f64; STATE_DIM]; STATE_DIM]) -> Option<[[f64; STATE_DIM]; STATE_DIM]> {
    let scale = diagonal_scale(matrix);
    let lower = [1.0e-12, 1.0e-9, 1.0e-6, 1.0e-3]
        .iter()
        .find_map(|factor| cholesky(&regularize(matrix, factor * scale)))?;
    let mut lower_inverse = [[0.0; STATE_DIM]; STATE_DIM];
    for i in 0..STATE_DIM {
        lower_inverse[i][i] = 1.0 / lower[i][i];
        for j in 0..i {
            let mut sum = 0.0;
            for k in j..i {
                sum += lower[i][k] * lower_inverse[k][j];
            }
            lower_inverse[i][j] = -sum / lower[i][i];
        }
    }
    let mut inverse = [[0.0; STATE_DIM]; STATE_DIM];
    for i in 0..STATE_DIM {
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

#[allow(clippy::needless_range_loop)]
fn solve(
    matrix: &[[f64; STATE_DIM]; STATE_DIM],
    rhs: &[f64; STATE_DIM],
) -> Option<[f64; STATE_DIM]> {
    let scale = diagonal_scale(matrix);
    let lower = [1.0e-12, 1.0e-9, 1.0e-6, 1.0e-3]
        .iter()
        .find_map(|factor| cholesky(&regularize(matrix, factor * scale)))?;
    let mut y = [0.0; STATE_DIM];
    for row in 0..STATE_DIM {
        let mut sum = rhs[row];
        for k in 0..row {
            sum -= lower[row][k] * y[k];
        }
        y[row] = sum / lower[row][row];
    }
    let mut x = [0.0; STATE_DIM];
    for row in (0..STATE_DIM).rev() {
        let mut sum = y[row];
        for k in (row + 1)..STATE_DIM {
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

    fn config() -> LioInertialConfig {
        LioInertialConfig {
            icp: PointToPlaneConfig {
                target_voxel_size_m: 0.3,
                max_correspondence_distance_m: 0.6,
                ..PointToPlaneConfig::default()
            },
            map_voxel_size_m: 0.15,
            max_map_points: 2_000,
            ..LioInertialConfig::default()
        }
    }

    #[test]
    fn velocity_tracks_constant_acceleration() {
        let scene = scene();
        let mut lio = LioInertialEkf::new(config()).expect("lio");
        lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
            .expect("first");
        // Body acceleration cancels gravity and adds 1 m/s^2 along +x.
        for _ in 0..10 {
            lio.process_imu(0.1, Vec3::new(1.0, 9.81, 0.0), Vec3::ZERO)
                .expect("imu");
        }
        let truth = Se3::new(Quat::IDENTITY, Vec3::new(0.5, 0.0, 0.0));
        let update = lio
            .register_scan(&scan_from(&scene, truth))
            .expect("second");
        assert!(
            (update.velocity_m_s.x - 1.0).abs() < 0.05,
            "velocity {}",
            update.velocity_m_s.x
        );
        assert!(update.covariance_trace > 0.0);
    }

    #[test]
    fn pose_tracks_a_known_motion() {
        let scene = scene();
        let mut lio = LioInertialEkf::new(config()).expect("lio");
        lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
            .expect("first");
        let truth = Se3::new(Quat::from_rotation_y(0.03), Vec3::new(0.12, 0.0, 0.06));
        let update = lio
            .register_scan(&scan_from(&scene, truth))
            .expect("second");
        let error = (update.pose.translation - truth.translation).length();
        assert!(error < 0.03, "pose error {error}");
    }

    #[test]
    fn accel_bias_is_estimated() {
        let scene = scene();
        let true_bias = 0.5;
        let mut lio = LioInertialEkf::new(config()).expect("lio");
        for _ in 0..60 {
            lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
                .expect("scan");
            for _ in 0..10 {
                // Stationary robot with an x accelerometer bias.
                lio.process_imu(0.01, Vec3::new(true_bias, 9.81, 0.0), Vec3::ZERO)
                    .expect("imu");
            }
        }
        assert!(
            (lio.accel_bias_m_s2().x - true_bias).abs() < 0.15,
            "accel bias {}",
            lio.accel_bias_m_s2().x
        );
        assert!(
            lio.velocity_m_s().length() < 0.1,
            "velocity {:?}",
            lio.velocity_m_s()
        );
    }

    #[test]
    fn replay_is_deterministic() {
        let scene = scene();
        let run = || {
            let mut lio = LioInertialEkf::new(config()).expect("lio");
            lio.register_scan(&scan_from(&scene, Se3::IDENTITY))
                .expect("scan");
            for _ in 0..5 {
                lio.process_imu(0.05, Vec3::new(0.2, 9.81, 0.0), Vec3::ZERO)
                    .expect("imu");
            }
            let update = lio
                .register_scan(&scan_from(
                    &scene,
                    Se3::new(Quat::IDENTITY, Vec3::new(0.05, 0.0, 0.0)),
                ))
                .expect("scan")
                .pose;
            (update, lio.velocity_m_s(), lio.accel_bias_m_s2())
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn rejects_bad_inputs() {
        assert!(LioInertialEkf::new(LioInertialConfig {
            iterations: 0,
            ..LioInertialConfig::default()
        })
        .is_err());
        let mut lio = LioInertialEkf::new(config()).expect("lio");
        assert!(matches!(
            lio.register_scan(&[]),
            Err(LioInertialError::EmptyScan)
        ));
        assert!(matches!(
            lio.process_imu(0.0, Vec3::ZERO, Vec3::ZERO),
            Err(LioInertialError::InvalidDt)
        ));
    }
}

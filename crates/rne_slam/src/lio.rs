//! Deterministic LiDAR-inertial odometry front-end.
//!
//! Between scans, high-rate IMU samples are preintegrated to predict the next
//! pose. Each scan is then registered to a maintained local map with
//! point-to-plane ICP, initialized from that prediction, and the corrected pose
//! is committed. Keyframes and their relative-pose constraints are recorded as
//! [`PoseGraph3dEdge`]s for the back-end.
//!
//! The pipeline is intentionally modest: it is a predict-then-scan-match
//! front-end rather than a tightly-coupled iterated EKF. The scan-to-map update
//! uses the point-to-plane normal equations directly, and IMU prediction supplies
//! the initial guess and velocity. Everything is deterministic.

use crate::imu_preintegration::{ImuBias, ImuPreintegrator, ImuSample};
use crate::point_to_plane::{
    estimate_normals, IcpPointToPlane, PointToPlaneConfig, PointToPlaneError,
};
use crate::pose_graph3d::PoseGraph3dEdge;
use crate::se3::Se3;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// LiDAR-inertial odometry configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LioConfig {
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
    /// Translation since the last keyframe that starts a new one, in meters.
    pub keyframe_translation_m: f64,
    /// Rotation since the last keyframe that starts a new one, in radians.
    pub keyframe_rotation_rad: f64,
}

impl Default for LioConfig {
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
            keyframe_translation_m: 0.05,
            keyframe_rotation_rad: 0.05,
        }
    }
}

impl LioConfig {
    fn is_valid(&self) -> bool {
        self.gravity_m_s2.is_finite()
            && self.normal_radius_m.is_finite()
            && self.normal_radius_m > 0.0
            && self.map_voxel_size_m.is_finite()
            && self.map_voxel_size_m > 0.0
            && self.max_map_points > 0
            && self.keyframe_translation_m.is_finite()
            && self.keyframe_translation_m >= 0.0
            && self.keyframe_rotation_rad.is_finite()
            && self.keyframe_rotation_rad >= 0.0
    }
}

/// LiDAR-inertial odometry failure.
#[derive(Debug, thiserror::Error)]
pub enum LioError {
    /// The configuration was invalid.
    #[error("invalid LIO configuration")]
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
pub struct LioUpdate {
    /// Corrected world pose of the sensor.
    pub pose: Se3,
    /// Estimated linear velocity in the world frame.
    pub velocity_m_s: Vec3,
    /// Whether this scan started a new keyframe.
    pub keyframe: bool,
    /// Point-to-plane correspondences used.
    pub correspondences: usize,
    /// Registration root-mean-square residual in meters.
    pub rmse_m: f64,
}

/// A LiDAR-inertial odometry front-end.
#[derive(Clone, Debug)]
pub struct LioOdometry {
    config: LioConfig,
    integrator: ImuPreintegrator,
    pose: Se3,
    velocity_m_s: Vec3,
    map_points: Vec<Vec3>,
    map_normals: Vec<Vec3>,
    map_voxels: BTreeSet<[i64; 3]>,
    keyframes: Vec<Se3>,
    edges: Vec<PoseGraph3dEdge>,
    initialized: bool,
}

impl LioOdometry {
    /// Creates an odometry front-end at the origin.
    pub fn new(config: LioConfig) -> Result<Self, LioError> {
        if !config.is_valid() {
            return Err(LioError::InvalidConfig);
        }
        Ok(Self {
            config,
            integrator: ImuPreintegrator::new(config.imu_bias),
            pose: Se3::IDENTITY,
            velocity_m_s: Vec3::ZERO,
            map_points: Vec::new(),
            map_normals: Vec::new(),
            map_voxels: BTreeSet::new(),
            keyframes: Vec::new(),
            edges: Vec::new(),
            initialized: false,
        })
    }

    /// Folds one IMU sample into the motion model.
    pub fn process_imu(&mut self, sample: &ImuSample) -> Result<(), LioError> {
        self.integrator
            .integrate(sample)
            .map_err(|_| LioError::NonFinite)
    }

    /// Current world pose.
    pub const fn pose(&self) -> Se3 {
        self.pose
    }

    /// Current world-frame linear velocity.
    pub const fn velocity_m_s(&self) -> Vec3 {
        self.velocity_m_s
    }

    /// Number of points in the local map.
    pub fn local_map_len(&self) -> usize {
        self.map_points.len()
    }

    /// Keyframe poses recorded so far.
    pub fn keyframes(&self) -> &[Se3] {
        &self.keyframes
    }

    /// Relative-pose constraints between keyframes, for the back-end.
    pub fn edges(&self) -> &[PoseGraph3dEdge] {
        &self.edges
    }

    /// Registers one scan (points in the sensor frame) and returns the correction.
    pub fn register_scan(&mut self, points: &[Vec3]) -> Result<LioUpdate, LioError> {
        if points.is_empty() {
            return Err(LioError::EmptyScan);
        }
        if points.iter().any(|point| !point.is_finite()) {
            return Err(LioError::NonFinite);
        }

        let dt = self.integrator.delta().dt_s;
        let (predicted_pose, predicted_velocity) =
            self.integrator
                .predict(self.pose, self.velocity_m_s, self.config.gravity_m_s2);

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
            (result.transform, result.correspondences, result.rmse_m)
        };

        let velocity = if dt > 0.0 {
            (corrected.translation - self.pose.translation) / dt
        } else {
            predicted_velocity
        };

        self.pose = corrected;
        self.velocity_m_s = velocity;
        self.initialized = true;

        let keyframe = self.keyframes.is_empty() || {
            let last = self.keyframes.last().copied().unwrap_or(Se3::IDENTITY);
            let relative = last.inverse().compose(corrected);
            relative.translation.length() >= self.config.keyframe_translation_m
                || (2.0 * relative.rotation.w.abs().min(1.0).acos())
                    >= self.config.keyframe_rotation_rad
        };
        if keyframe {
            if let Some(last) = self.keyframes.last().copied() {
                let from = self.keyframes.len() - 1;
                let to = self.keyframes.len();
                self.edges.push(PoseGraph3dEdge::odometry(
                    from,
                    to,
                    last.inverse().compose(corrected),
                ));
            }
            self.keyframes.push(corrected);
        }

        self.integrate_scan_into_map(points, corrected);
        self.integrator.reset();

        Ok(LioUpdate {
            pose: corrected,
            velocity_m_s: velocity,
            keyframe,
            correspondences,
            rmse_m,
        })
    }

    fn integrate_scan_into_map(&mut self, points: &[Vec3], pose: Se3) {
        let voxel = self.config.map_voxel_size_m;
        for point in points {
            let world = pose.transform_point(*point);
            if !world.is_finite() {
                continue;
            }
            let key = voxel_key(world, voxel);
            if self.map_voxels.insert(key) {
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

    fn config() -> LioConfig {
        LioConfig {
            icp: PointToPlaneConfig {
                target_voxel_size_m: 0.3,
                max_correspondence_distance_m: 0.6,
                ..PointToPlaneConfig::default()
            },
            normal_radius_m: 0.35,
            map_voxel_size_m: 0.15,
            keyframe_translation_m: 0.05,
            keyframe_rotation_rad: 0.05,
            ..LioConfig::default()
        }
    }

    #[test]
    fn initializes_on_the_first_scan() {
        let mut lio = LioOdometry::new(config()).expect("lio");
        let update = lio.register_scan(&scene()).expect("scan");
        assert!(lio.local_map_len() > 0);
        assert_eq!(lio.keyframes().len(), 1);
        assert_eq!(update.pose, Se3::IDENTITY);
    }

    #[test]
    fn tracks_a_known_motion_between_scans() {
        let scene = scene();
        let mut lio = LioOdometry::new(config()).expect("lio");
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
        assert!(update.keyframe);
        assert!(!lio.edges().is_empty());
    }

    #[test]
    fn replay_is_deterministic() {
        let scene = scene();
        let run = || {
            let mut lio = LioOdometry::new(config()).expect("lio");
            let poses = [
                Se3::IDENTITY,
                Se3::new(Quat::from_rotation_y(0.02), Vec3::new(0.1, 0.0, 0.05)),
                Se3::new(Quat::from_rotation_y(0.04), Vec3::new(0.2, 0.0, 0.1)),
            ];
            let mut out = Vec::new();
            for pose in poses {
                out.push(
                    lio.register_scan(&scan_from(&scene, pose))
                        .expect("scan")
                        .pose,
                );
            }
            out
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn rejects_bad_inputs() {
        let mut lio = LioOdometry::new(config()).expect("lio");
        assert!(matches!(lio.register_scan(&[]), Err(LioError::EmptyScan)));
        assert!(LioOdometry::new(LioConfig {
            normal_radius_m: 0.0,
            ..LioConfig::default()
        })
        .is_err());
        assert_relative_eq!(Vec3::ZERO.x, 0.0);
    }
}

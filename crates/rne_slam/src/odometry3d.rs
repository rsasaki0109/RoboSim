//! 3D LiDAR odometry from chained point-to-point ICP.
//!
//! [`IcpOdometry`] keeps a downsampled map cloud and estimates the sensor pose
//! by aligning each new scan to it, using the odometry delta as the motion
//! prediction. It is the 3D front-end that feeds the `rne_nav` elevation map
//! for outdoor mapping; combined with a pose graph it provides loop closure.
//! Voxel downsampling and correspondence search are deterministic, so a scan
//! sequence replays identically.

use crate::icp::{Icp3d, IcpConfig, IcpError};
use rne_math::{Transform3, Vec3};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

/// Configuration for [`IcpOdometry`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct IcpOdometryConfig {
    /// ICP refinement settings.
    pub icp: IcpConfig,
    /// Voxel size for downsampling scans and the map, in meters.
    pub voxel_size_m: f64,
    /// Minimum correspondences required to accept an ICP correction.
    pub min_correspondences: usize,
    /// Map size above which the map is re-downsampled.
    pub max_map_points: usize,
}

impl Default for IcpOdometryConfig {
    fn default() -> Self {
        Self {
            icp: IcpConfig::default(),
            voxel_size_m: 0.05,
            min_correspondences: 10,
            max_map_points: 200_000,
        }
    }
}

impl IcpOdometryConfig {
    /// Whether the configuration is finite and well ordered.
    pub fn is_valid(&self) -> bool {
        self.voxel_size_m.is_finite()
            && self.voxel_size_m > 0.0
            && self.min_correspondences >= 3
            && self.max_map_points > 0
    }
}

/// Result of one odometry update.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct IcpOdometryUpdate {
    /// The estimated sensor pose in the map frame.
    pub pose: Transform3,
    /// Correspondences used by the accepted ICP correction.
    pub correspondences: usize,
    /// Mean ICP residual in meters for the accepted correction.
    pub mean_residual_m: f64,
    /// Whether ICP reported convergence.
    pub converged: bool,
    /// Whether an ICP correction was applied (rather than the odometry guess).
    pub corrected: bool,
}

/// Deterministic 3D ICP odometry.
#[derive(Clone, Debug, PartialEq)]
pub struct IcpOdometry {
    config: IcpOdometryConfig,
    pose: Transform3,
    map_points: Vec<Vec3>,
}

impl IcpOdometry {
    /// Creates an empty odometry estimator.
    pub fn new(config: IcpOdometryConfig) -> Result<Self, IcpError> {
        if !config.is_valid() {
            return Err(IcpError::InvalidConfig);
        }
        Ok(Self {
            config,
            pose: Transform3::IDENTITY,
            map_points: Vec::new(),
        })
    }

    /// Seeds the map with an initial cloud (e.g. the first scan in world frame).
    pub fn seed_map(&mut self, points_world_m: &[Vec3]) {
        self.map_points = voxel_downsample(points_world_m, self.config.voxel_size_m);
    }

    /// The current sensor pose in the map frame.
    pub fn pose(&self) -> Transform3 {
        self.pose
    }

    /// Overrides the estimated pose (used after a loop-closure correction).
    pub fn set_pose(&mut self, pose: Transform3) {
        self.pose = pose;
    }

    /// Number of points in the downsampled map.
    pub fn map_len(&self) -> usize {
        self.map_points.len()
    }

    /// Aligns `scan_sensor` to the map using `odom_delta` as the prediction and
    /// then integrates it into the map.
    pub fn update(
        &mut self,
        scan_sensor_m: &[Vec3],
        odom_delta: Transform3,
        sensor_from_base: Transform3,
    ) -> Result<IcpOdometryUpdate, IcpError> {
        let scan = voxel_downsample(scan_sensor_m, self.config.voxel_size_m);
        if scan.is_empty() {
            return Err(IcpError::EmptySource);
        }

        let predicted = odom_delta.mul_transform(&self.pose);
        let predicted_sensor = predicted.mul_transform(&sensor_from_base);
        let source_world: Vec<Vec3> = scan
            .iter()
            .map(|point| predicted_sensor.transform_point(*point))
            .collect();

        let mut update = IcpOdometryUpdate {
            pose: predicted,
            correspondences: 0,
            mean_residual_m: 0.0,
            converged: false,
            corrected: false,
        };

        if self.map_points.len() >= 3 {
            let result = Icp3d::align(
                &source_world,
                &self.map_points,
                Transform3::IDENTITY,
                &self.config.icp,
            )?;
            if result.correspondences >= self.config.min_correspondences {
                self.pose = result.transform.mul_transform(&predicted);
                update.pose = self.pose;
                update.correspondences = result.correspondences;
                update.mean_residual_m = result.mean_residual_m;
                update.converged = result.converged;
                update.corrected = true;
            } else {
                self.pose = predicted;
            }
        } else {
            self.pose = predicted;
        }

        let sensor_world = self.pose.mul_transform(&sensor_from_base);
        for point in &scan {
            self.map_points.push(sensor_world.transform_point(*point));
        }
        if self.map_points.len() > self.config.max_map_points {
            self.map_points = voxel_downsample(&self.map_points, self.config.voxel_size_m);
        }
        Ok(update)
    }
}

/// Downsamples points to one centroid per voxel, in deterministic voxel order.
pub fn voxel_downsample(points_m: &[Vec3], voxel_size_m: f64) -> Vec<Vec3> {
    let size = voxel_size_m.max(f64::MIN_POSITIVE);
    let mut buckets: BTreeMap<[i64; 3], (Vec3, u64)> = BTreeMap::new();
    for point in points_m {
        if !point.is_finite() {
            continue;
        }
        let key = [
            (point.x / size).floor() as i64,
            (point.y / size).floor() as i64,
            (point.z / size).floor() as i64,
        ];
        let entry = buckets.entry(key).or_insert((Vec3::ZERO, 0));
        entry.0 += *point;
        entry.1 += 1;
    }
    buckets
        .values()
        .map(|(sum, count)| *sum / *count as f64)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::Quat;

    fn world_cloud() -> Vec<Vec3> {
        let mut points = Vec::new();
        for i in 0..10 {
            for j in 0..10 {
                for k in 0..3 {
                    points.push(Vec3::new(
                        0.2 * i as f64,
                        0.2 * j as f64,
                        0.3 * k as f64 + 0.05 * ((i + j) % 3) as f64,
                    ));
                }
            }
        }
        points
    }

    #[test]
    fn downsample_collapses_duplicates() {
        let downsampled = voxel_downsample(
            &[
                Vec3::new(0.01, 0.01, 0.01),
                Vec3::new(0.02, 0.02, 0.02),
                Vec3::new(1.0, 1.0, 1.0),
            ],
            0.05,
        );
        assert_eq!(downsampled.len(), 2);
    }

    #[test]
    fn recovers_a_pose_from_a_seeded_map() {
        let world = world_cloud();
        let mut odometry = IcpOdometry::new(IcpOdometryConfig::default()).unwrap();
        odometry.seed_map(&world);

        let truth = Transform3::from_translation_rotation(
            Vec3::new(0.4, 0.0, 0.3),
            Quat::from_rotation_y(0.2),
        );
        // The scan is the world seen from the true pose (inverse transform).
        let truth_inverse = truth.inverse();
        let scan: Vec<Vec3> = world
            .iter()
            .map(|point| truth_inverse.transform_point(*point))
            .collect();
        let odom_guess = Transform3::from_translation_rotation(
            Vec3::new(0.35, 0.02, 0.28),
            Quat::from_rotation_y(0.15),
        );

        let update = odometry
            .update(&scan, odom_guess, Transform3::IDENTITY)
            .unwrap();
        assert!(update.corrected);
        let pose = odometry.pose();
        assert_relative_eq!(pose.translation.x, truth.translation.x, epsilon = 1e-3);
        assert_relative_eq!(pose.translation.z, truth.translation.z, epsilon = 1e-3);
        let probe = Vec3::new(0.1, 0.2, 0.3);
        let expected = truth.rotation * probe;
        let actual = pose.rotation * probe;
        assert_relative_eq!(actual.x, expected.x, epsilon = 1e-3);
        assert_relative_eq!(actual.z, expected.z, epsilon = 1e-3);
    }

    #[test]
    fn first_scan_adopts_the_odometry_guess() {
        let mut odometry = IcpOdometry::new(IcpOdometryConfig::default()).unwrap();
        let delta = Transform3::from_translation_rotation(Vec3::new(0.1, 0.0, 0.0), Quat::IDENTITY);
        let update = odometry
            .update(&world_cloud(), delta, Transform3::IDENTITY)
            .unwrap();
        assert!(!update.corrected);
        assert_relative_eq!(update.pose.translation.x, 0.1, epsilon = 1e-12);
        assert!(odometry.map_len() > 0);
    }

    #[test]
    fn rejects_invalid_config() {
        let config = IcpOdometryConfig {
            voxel_size_m: 0.0,
            ..IcpOdometryConfig::default()
        };
        assert_eq!(IcpOdometry::new(config), Err(IcpError::InvalidConfig));
    }
}

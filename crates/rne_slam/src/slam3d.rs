//! 3D SLAM back-end: keyframes, ICP loop closure, and elevation re-integration.
//!
//! [`Slam3d`] wraps [`IcpOdometry`] as the front
//! end and adds the mapping back-end:
//!
//! * keyframes are added when the robot has moved far enough,
//! * a revisit within `loop_search_radius_m` is verified by aligning the current
//!   scan to the candidate keyframe with [`Icp3d`]; a low-residual match adds a
//!   loop-closure edge and re-optimizes the pose graph,
//! * the [`ElevationMap`] is rebuilt from every keyframe after optimization.
//!
//! Ground robots use the 2.5D graph pose `(x, z, yaw)` (world Y is up) so the
//! existing planar `PoseGraph` applies; everything is deterministic.

use crate::icp::{Icp3d, IcpError};
use crate::odometry3d::{voxel_downsample, IcpOdometry, IcpOdometryConfig};
use crate::pose_graph::{PoseGraph, PoseGraphEdge, PoseGraphError};
use rne_math::{yaw_rad, Quat, Transform3, Vec3};
use rne_nav::pose2d::Pose2d;
use rne_nav::{ElevationConfig, ElevationMap};
use serde::{Deserialize, Serialize};

/// Configuration for [`Slam3d`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Slam3dConfig {
    /// Front-end ICP odometry settings.
    pub odometry: IcpOdometryConfig,
    /// Minimum translation between keyframes, in meters.
    pub keyframe_translation_m: f64,
    /// Minimum yaw change between keyframes, in radians.
    pub keyframe_rotation_rad: f64,
    /// Search radius for loop-closure candidates, in meters.
    pub loop_search_radius_m: f64,
    /// Maximum ICP residual for an accepted loop closure, in meters.
    pub loop_max_residual_m: f64,
    /// Pose-graph optimization iterations after a loop closure.
    pub optimize_iterations: usize,
}

impl Default for Slam3dConfig {
    fn default() -> Self {
        Self {
            odometry: IcpOdometryConfig::default(),
            keyframe_translation_m: 0.2,
            keyframe_rotation_rad: 0.2,
            loop_search_radius_m: 0.5,
            loop_max_residual_m: 0.15,
            optimize_iterations: 10,
        }
    }
}

impl Slam3dConfig {
    /// Whether every threshold is finite and positive.
    pub fn is_valid(&self) -> bool {
        self.keyframe_translation_m.is_finite()
            && self.keyframe_translation_m > 0.0
            && self.keyframe_rotation_rad.is_finite()
            && self.keyframe_rotation_rad > 0.0
            && self.loop_search_radius_m.is_finite()
            && self.loop_search_radius_m > 0.0
            && self.loop_max_residual_m.is_finite()
            && self.loop_max_residual_m > 0.0
            && self.optimize_iterations > 0
    }
}

/// Errors raised by the 3D SLAM back-end.
#[derive(Clone, Copy, Debug, PartialEq, Eq, thiserror::Error)]
pub enum SlamError {
    /// The configuration was degenerate.
    #[error("invalid SLAM configuration")]
    InvalidConfig,
    /// The ICP front end failed.
    #[error("ICP front end failed: {0}")]
    Icp(#[from] IcpError),
    /// Pose-graph optimization failed.
    #[error("pose graph failed: {0}")]
    Graph(#[from] PoseGraphError),
}

/// Result of one [`Slam3d::process`] call.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Slam3dUpdate {
    /// Corrected sensor pose in the map frame.
    pub pose: Transform3,
    /// Whether a keyframe was added this step.
    pub keyframe_added: bool,
    /// Whether a loop closure was accepted this step.
    pub loop_closure: bool,
    /// Total loop closures accepted so far.
    pub loop_closures: usize,
    /// Correspondences used by the ICP front end.
    pub correspondences: usize,
    /// Whether the ICP front end corrected the odometry prediction.
    pub odometry_corrected: bool,
}

#[derive(Debug)]
struct Keyframe3d {
    pose: Pose2d,
    cloud_sensor: Vec<Vec3>,
}

/// A 3D SLAM back-end over the elevation map.
#[derive(Debug)]
pub struct Slam3d {
    config: Slam3dConfig,
    odometry: IcpOdometry,
    keyframes: Vec<Keyframe3d>,
    graph: PoseGraph,
    elevation: ElevationMap,
    geometry: (usize, usize, f64, Pose2d, ElevationConfig),
    loop_closures: usize,
}

impl Slam3d {
    /// Creates a back-end with a pre-sized elevation map.
    pub fn new(elevation: ElevationMap, config: Slam3dConfig) -> Result<Self, SlamError> {
        if !config.is_valid() || !config.odometry.is_valid() {
            return Err(SlamError::InvalidConfig);
        }
        let odometry = IcpOdometry::new(config.odometry)?;
        let geometry = (
            elevation.width(),
            elevation.height(),
            elevation.resolution_m(),
            elevation.origin(),
            elevation.config(),
        );
        Ok(Self {
            config,
            odometry,
            keyframes: Vec::new(),
            graph: PoseGraph::new(),
            elevation,
            geometry,
            loop_closures: 0,
        })
    }

    /// The current corrected sensor pose.
    pub fn pose(&self) -> Transform3 {
        self.odometry.pose()
    }

    /// The accumulated elevation map.
    pub fn elevation(&self) -> &ElevationMap {
        &self.elevation
    }

    /// Number of keyframes.
    pub fn keyframe_count(&self) -> usize {
        self.keyframes.len()
    }

    /// Total loop closures accepted.
    pub fn loop_closures(&self) -> usize {
        self.loop_closures
    }

    /// The optimized pose graph.
    pub fn graph(&self) -> &PoseGraph {
        &self.graph
    }

    /// Integrates one scan and runs keyframe/loop-closure bookkeeping.
    pub fn process(
        &mut self,
        scan_sensor_m: &[Vec3],
        odom_delta: Transform3,
        sensor_from_base: Transform3,
    ) -> Result<Slam3dUpdate, SlamError> {
        let front_end = self
            .odometry
            .update(scan_sensor_m, odom_delta, sensor_from_base)?;
        let cloud = voxel_downsample(scan_sensor_m, self.config.odometry.voxel_size_m);
        let pose3 = self.odometry.pose();
        let pose2 = pose3_to_pose2d(pose3);

        let mut keyframe_added = false;
        let mut loop_closure = false;
        if self.keyframes.is_empty() {
            self.graph.add_node(pose2);
            self.keyframes.push(Keyframe3d {
                pose: pose2,
                cloud_sensor: cloud.clone(),
            });
            keyframe_added = true;
        } else if self.moved_enough(pose2) {
            let index = self.graph.add_node(pose2);
            if index > 0 {
                let previous = self.graph.node(index - 1).unwrap_or(pose2);
                self.graph.add_edge(PoseGraphEdge::odometry(
                    index - 1,
                    index,
                    previous.inverse().compose(pose2),
                ));
            }
            self.keyframes.push(Keyframe3d {
                pose: pose2,
                cloud_sensor: cloud.clone(),
            });
            keyframe_added = true;
            loop_closure = self.try_loop_closure(index, sensor_from_base)?;
        }

        let sensor_world = pose3.mul_transform(&sensor_from_base);
        let world_points: Vec<Vec3> = cloud
            .iter()
            .map(|point| sensor_world.transform_point(*point))
            .collect();
        self.elevation.integrate(&world_points);

        Ok(Slam3dUpdate {
            pose: self.odometry.pose(),
            keyframe_added,
            loop_closure,
            loop_closures: self.loop_closures,
            correspondences: front_end.correspondences,
            odometry_corrected: front_end.corrected,
        })
    }

    fn moved_enough(&self, pose: Pose2d) -> bool {
        let Some(last) = self.keyframes.last() else {
            return true;
        };
        let translation =
            ((pose.x_m - last.pose.x_m).powi(2) + (pose.y_m - last.pose.y_m).powi(2)).sqrt();
        let rotation = wrap_angle(pose.yaw_rad - last.pose.yaw_rad).abs();
        translation >= self.config.keyframe_translation_m
            || rotation >= self.config.keyframe_rotation_rad
    }

    fn try_loop_closure(
        &mut self,
        index: usize,
        sensor_from_base: Transform3,
    ) -> Result<bool, SlamError> {
        let current_pose = self.keyframes[index].pose;
        let mut candidate: Option<(usize, f64)> = None;
        for (i, keyframe) in self
            .keyframes
            .iter()
            .enumerate()
            .take(index.saturating_sub(3))
        {
            let distance = ((keyframe.pose.x_m - current_pose.x_m).powi(2)
                + (keyframe.pose.y_m - current_pose.y_m).powi(2))
            .sqrt();
            if distance <= self.config.loop_search_radius_m
                && candidate.map(|(_, best)| distance < best).unwrap_or(true)
            {
                candidate = Some((i, distance));
            }
        }
        let Some((candidate_index, _)) = candidate else {
            return Ok(false);
        };

        let current_world = pose2d_to_transform3(current_pose).mul_transform(&sensor_from_base);
        let candidate_world = pose2d_to_transform3(self.keyframes[candidate_index].pose)
            .mul_transform(&sensor_from_base);
        let source = world_points(&self.keyframes[index].cloud_sensor, &current_world);
        let target = world_points(
            &self.keyframes[candidate_index].cloud_sensor,
            &candidate_world,
        );
        let initial = candidate_world.mul_transform(&current_world.inverse());
        let result = Icp3d::align(&source, &target, initial, &self.config.odometry.icp)?;
        if result.correspondences < self.config.odometry.min_correspondences
            || result.mean_residual_m > self.config.loop_max_residual_m
        {
            return Ok(false);
        }

        // The ICP transform maps the current sensor points onto the candidate.
        let corrected_current_sensor = result.transform.mul_transform(&current_world);
        let corrected_current_base =
            corrected_current_sensor.mul_transform(&sensor_from_base.inverse());
        let measurement = pose3_to_pose2d(
            pose2d_to_transform3(self.keyframes[candidate_index].pose)
                .inverse()
                .mul_transform(&corrected_current_base),
        );
        self.graph.add_edge(PoseGraphEdge::loop_closure(
            candidate_index,
            index,
            measurement,
            (1.0, 1.0, 1.0),
        ));
        self.graph
            .optimize_robust(self.config.optimize_iterations, 1.0e-6, 0, 0.3)?;
        self.sync_keyframe_poses();
        self.rebuild_elevation();
        self.loop_closures += 1;
        Ok(true)
    }

    fn sync_keyframe_poses(&mut self) {
        for (i, keyframe) in self.keyframes.iter_mut().enumerate() {
            if let Some(pose) = self.graph.node(i) {
                keyframe.pose = pose;
            }
        }
        if let Some(last) = self.keyframes.last() {
            self.odometry.set_pose(pose2d_to_transform3(last.pose));
        }
    }

    fn rebuild_elevation(&mut self) {
        let (width, height, resolution, origin, config) = self.geometry;
        let Ok(mut elevation) = ElevationMap::new(width, height, resolution, origin, config) else {
            return;
        };
        for (i, keyframe) in self.keyframes.iter().enumerate() {
            let pose = self.graph.node(i).unwrap_or(keyframe.pose);
            let world = pose2d_to_transform3(pose);
            let points: Vec<Vec3> = keyframe
                .cloud_sensor
                .iter()
                .map(|point| world.transform_point(*point))
                .collect();
            elevation.integrate(&points);
        }
        self.elevation = elevation;
    }
}

fn world_points(cloud_sensor: &[Vec3], sensor_world: &Transform3) -> Vec<Vec3> {
    cloud_sensor
        .iter()
        .map(|point| sensor_world.transform_point(*point))
        .collect()
}

/// Converts a planar graph pose into a world transform (`yaw` about world Y).
pub fn pose2d_to_transform3(pose: Pose2d) -> Transform3 {
    Transform3::from_translation_rotation(
        Vec3::new(pose.x_m, 0.0, pose.y_m),
        Quat::from_rotation_y(pose.yaw_rad),
    )
}

/// Projects a world transform into the planar graph pose `(x, z, yaw_y)`.
pub fn pose3_to_pose2d(pose: Transform3) -> Pose2d {
    Pose2d::new(
        pose.translation.x,
        pose.translation.z,
        yaw_rad(pose.rotation),
    )
}

fn wrap_angle(angle_rad: f64) -> f64 {
    let two_pi = std::f64::consts::TAU;
    let mut wrapped = angle_rad % two_pi;
    if wrapped > std::f64::consts::PI {
        wrapped -= two_pi;
    } else if wrapped <= -std::f64::consts::PI {
        wrapped += two_pi;
    }
    wrapped
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn elevation() -> ElevationMap {
        ElevationMap::new(
            200,
            200,
            0.1,
            Pose2d::new(-10.0, -10.0, 0.0),
            ElevationConfig::default(),
        )
        .unwrap()
    }

    fn cloud() -> Vec<Vec3> {
        let mut points = Vec::new();
        for i in 0..8 {
            for j in 0..8 {
                for k in 0..3 {
                    points.push(Vec3::new(0.25 * i as f64, 0.2 * j as f64, 0.3 * k as f64));
                }
            }
        }
        points
    }

    #[test]
    fn first_scan_seeds_a_keyframe_and_elevation() {
        let mut slam = Slam3d::new(elevation(), Slam3dConfig::default()).unwrap();
        let update = slam
            .process(&cloud(), Transform3::IDENTITY, Transform3::IDENTITY)
            .unwrap();
        assert!(update.keyframe_added);
        assert_eq!(slam.keyframe_count(), 1);
        assert!(slam.elevation().cells().iter().any(|cell| cell.is_known()));
    }

    #[test]
    fn replay_is_bit_identical() {
        let run = || {
            let mut slam = Slam3d::new(elevation(), Slam3dConfig::default()).unwrap();
            for step in 0..12 {
                let delta = Transform3::from_translation_rotation(
                    Vec3::new(0.1, 0.0, 0.0),
                    Quat::from_rotation_y(0.05 * step as f64),
                );
                slam.process(&cloud(), delta, Transform3::IDENTITY).unwrap();
            }
            (
                slam.keyframe_count(),
                slam.graph().node_count(),
                slam.pose().translation.x.to_bits(),
                slam.pose().translation.z.to_bits(),
                slam.elevation()
                    .cells()
                    .iter()
                    .map(|cell| cell.count)
                    .sum::<u32>(),
            )
        };
        assert_eq!(run(), run());
    }

    #[test]
    fn pose_conversion_round_trips() {
        let pose = Pose2d::new(1.2, -0.7, 0.9);
        let back = pose3_to_pose2d(pose2d_to_transform3(pose));
        assert_relative_eq!(back.x_m, pose.x_m, epsilon = 1e-12);
        assert_relative_eq!(back.y_m, pose.y_m, epsilon = 1e-12);
        assert_relative_eq!(back.yaw_rad, pose.yaw_rad, epsilon = 1e-12);
    }

    #[test]
    fn rejects_invalid_config() {
        let config = Slam3dConfig {
            loop_search_radius_m: 0.0,
            ..Slam3dConfig::default()
        };
        assert!(matches!(
            Slam3d::new(elevation(), config),
            Err(SlamError::InvalidConfig)
        ));
    }
}

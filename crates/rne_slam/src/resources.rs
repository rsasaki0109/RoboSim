//! ECS resources for online SLAM.

use crate::slam::Slam2d;
use bevy_ecs::prelude::Resource;
use rne_nav::{LaserScan2d, Pose2d};

/// Online SLAM estimator resource.
#[derive(Resource, Clone, Debug)]
pub struct SlamState {
    /// The estimator.
    pub estimator: Slam2d,
}

impl SlamState {
    /// Wraps an estimator.
    pub fn new(estimator: Slam2d) -> Self {
        Self { estimator }
    }
}

/// One scan queued for SLAM with its odometry pose and sensor mounting.
#[derive(Clone, Debug, PartialEq)]
pub struct SlamScanInput {
    /// Scan payload.
    pub scan: LaserScan2d,
    /// Odometry base pose at acquisition time.
    pub odom_pose: Pose2d,
    /// Sensor pose relative to the base at acquisition time.
    pub sensor_from_base: Pose2d,
}

/// Queue of scans awaiting SLAM processing.
#[derive(Resource, Clone, Debug, Default)]
pub struct PendingSlamScans(pub Vec<SlamScanInput>);

impl PendingSlamScans {
    /// Creates an empty queue.
    pub fn new() -> Self {
        Self::default()
    }

    /// Pushes a scan onto the queue.
    pub fn push(&mut self, scan: LaserScan2d, odom_pose: Pose2d, sensor_from_base: Pose2d) {
        self.0.push(SlamScanInput {
            scan,
            odom_pose,
            sensor_from_base,
        });
    }

    /// Number of queued scans.
    pub fn len(&self) -> usize {
        self.0.len()
    }

    /// Whether the queue is empty.
    pub fn is_empty(&self) -> bool {
        self.0.is_empty()
    }
}

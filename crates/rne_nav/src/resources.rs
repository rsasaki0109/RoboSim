//! ECS resources bundling the navigation map and transform tree.

use crate::costmap::{Costmap, CostmapConfig};
use crate::grid::{GridError, OccupancyGrid};
use crate::pose2d::Pose2d;
use crate::scan::{integrate_scan, LaserScan2d, ScanIntegrationConfig, ScanIntegrationReport};
use crate::tf::TfBuffer;
use bevy_ecs::prelude::Resource;

/// An occupancy grid, its derived costmap, and update configuration.
#[derive(Resource, Clone, Debug)]
pub struct NavMap {
    /// Current occupancy grid.
    pub grid: OccupancyGrid,
    /// Costmap derived from `grid`.
    pub costmap: Costmap,
    /// Configuration used to rebuild the costmap.
    pub costmap_config: CostmapConfig,
    /// Configuration applied when integrating scans.
    pub integration: ScanIntegrationConfig,
}

impl NavMap {
    /// Creates a map and its initial costmap.
    pub fn new(
        grid: OccupancyGrid,
        costmap_config: CostmapConfig,
        integration: ScanIntegrationConfig,
    ) -> Result<Self, GridError> {
        let costmap = Costmap::from_occupancy(&grid, &costmap_config)?;
        Ok(Self {
            grid,
            costmap,
            costmap_config,
            integration,
        })
    }

    /// Integrates one scan into the grid without rebuilding the costmap.
    pub fn integrate(
        &mut self,
        scan: &LaserScan2d,
        sensor_pose_world: Pose2d,
    ) -> Result<ScanIntegrationReport, GridError> {
        integrate_scan(&mut self.grid, scan, sensor_pose_world, &self.integration)
    }

    /// Rebuilds the costmap from the current grid.
    pub fn refresh_costmap(&mut self) -> Result<(), GridError> {
        self.costmap = Costmap::from_occupancy(&self.grid, &self.costmap_config)?;
        Ok(())
    }

    /// Grid dimensions in cells.
    pub fn dimensions(&self) -> (usize, usize) {
        (self.grid.width(), self.grid.height())
    }
}

/// A scan queued for integration with its resolved world pose.
#[derive(Clone, Debug, PartialEq)]
pub struct PendingScan {
    /// Scan payload.
    pub scan: LaserScan2d,
    /// Sensor pose in the world/map frame.
    pub sensor_pose_world: Pose2d,
}

/// Queue of scans awaiting integration.
#[derive(Resource, Clone, Debug, Default)]
pub struct PendingScans(pub Vec<PendingScan>);

impl PendingScans {
    /// Creates an empty queue.
    pub fn new() -> Self {
        Self::default()
    }

    /// Pushes a scan onto the queue.
    pub fn push(&mut self, scan: LaserScan2d, sensor_pose_world: Pose2d) {
        self.0.push(PendingScan {
            scan,
            sensor_pose_world,
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

/// Transform tree resource.
#[derive(Resource, Clone, Debug, Default)]
pub struct TfTree(pub TfBuffer);

impl TfTree {
    /// Creates an empty transform tree.
    pub fn new() -> Self {
        Self::default()
    }
}

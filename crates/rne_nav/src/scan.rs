//! 2D laser scan payload and occupancy integration.

use crate::grid::{GridError, OccupancyGrid};
use crate::pose2d::Pose2d;
use crate::tf::FrameId;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// A planar laser scan in a named sensor frame.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct LaserScan2d {
    /// Acquisition time in seconds.
    pub time_s: f64,
    /// Sensor frame id.
    pub frame: FrameId,
    /// Angle of the first beam in radians.
    pub angle_min_rad: f64,
    /// Angular step between beams in radians.
    pub angle_increment_rad: f64,
    /// Minimum valid range in meters.
    pub range_min_m: f64,
    /// Maximum valid range in meters.
    pub range_max_m: f64,
    /// Measured ranges in meters; non-finite or out-of-range values are no-return.
    pub ranges_m: Vec<f64>,
}

impl LaserScan2d {
    /// Number of beams.
    pub fn beam_count(&self) -> usize {
        self.ranges_m.len()
    }

    /// Angle of the beam at `index` in radians, if the index is valid.
    pub fn angle_at(&self, index: usize) -> Option<f64> {
        (index < self.ranges_m.len())
            .then_some(self.angle_min_rad + self.angle_increment_rad * index as f64)
    }

    /// Whether the geometry fields are finite and consistent.
    pub fn is_valid(&self) -> bool {
        self.time_s.is_finite()
            && self.angle_min_rad.is_finite()
            && self.angle_increment_rad.is_finite()
            && self.range_min_m.is_finite()
            && self.range_max_m.is_finite()
            && self.range_min_m >= 0.0
            && self.range_max_m >= self.range_min_m
    }
}

/// Configuration for integrating scans into an occupancy grid.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ScanIntegrationConfig {
    /// Log-odds increment for an occupied return.
    pub occupied_log_odds: f64,
    /// Log-odds increment for a free-space traversal.
    pub free_log_odds: f64,
    /// Whether rays clear free space along their path.
    pub mark_free: bool,
    /// Process every `beam_stride`-th beam; values below one are treated as one.
    pub beam_stride: usize,
    /// Cast length for no-return beams, or `None` to use the scan's max range.
    pub no_return_range_m: Option<f64>,
}

impl Default for ScanIntegrationConfig {
    fn default() -> Self {
        Self {
            occupied_log_odds: crate::grid::DEFAULT_OCCUPIED_LOG_ODDS,
            free_log_odds: crate::grid::DEFAULT_FREE_LOG_ODDS,
            mark_free: true,
            beam_stride: 1,
            no_return_range_m: None,
        }
    }
}

/// Summary of one scan integration pass.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ScanIntegrationReport {
    /// Beams that produced at least one cell.
    pub beams_processed: usize,
    /// Beams skipped because they were invalid or outside range.
    pub beams_skipped: usize,
    /// Cells updated as free.
    pub free_updates: usize,
    /// Cells updated as occupied.
    pub occupied_updates: usize,
}

/// Integrates a 2D scan into an occupancy grid using the sensor's world pose.
///
/// The pose resolves the sensor frame to the world (or map) frame; the
/// transform tree is not consulted here so integration is a pure function.
pub fn integrate_scan(
    grid: &mut OccupancyGrid,
    scan: &LaserScan2d,
    sensor_pose_world: Pose2d,
    config: &ScanIntegrationConfig,
) -> Result<ScanIntegrationReport, GridError> {
    if !scan.is_valid() || !sensor_pose_world.is_finite() {
        return Err(GridError::NonFinite);
    }
    if !config.occupied_log_odds.is_finite() || !config.free_log_odds.is_finite() {
        return Err(GridError::NonFinite);
    }

    let cast_limit = config
        .no_return_range_m
        .filter(|value| value.is_finite())
        .map(|value| value.min(scan.range_max_m))
        .unwrap_or(scan.range_max_m);
    let stride = config.beam_stride.max(1);
    let sensor_origin = sensor_pose_world.transform_point(Vec3::ZERO);

    let mut report = ScanIntegrationReport::default();
    let mut index = 0;
    while index < scan.ranges_m.len() {
        let range = scan.ranges_m[index];
        let angle = scan.angle_min_rad + scan.angle_increment_rad * index as f64;
        index += stride;

        if !range.is_finite() || range < scan.range_min_m {
            report.beams_skipped += 1;
            continue;
        }
        let hit = range <= scan.range_max_m;
        let cast_range = if hit { range } else { cast_limit };
        if !cast_range.is_finite() || cast_range <= 0.0 {
            report.beams_skipped += 1;
            continue;
        }

        let local_dir = Vec3::new(angle.cos(), angle.sin(), 0.0);
        let end = sensor_pose_world.transform_point(local_dir * cast_range);
        let cells = grid.raycast(sensor_origin, end);
        if cells.is_empty() {
            report.beams_skipped += 1;
            continue;
        }

        if config.mark_free {
            let free_end = if hit { cells.len() - 1 } else { cells.len() };
            for coord in &cells[..free_end] {
                if grid.apply_free(*coord, config.free_log_odds) {
                    report.free_updates += 1;
                }
            }
        }
        if hit && grid.apply_occupied(cells[cells.len() - 1], config.occupied_log_odds) {
            report.occupied_updates += 1;
        }
        report.beams_processed += 1;
    }

    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::{CELL_FREE, CELL_OCCUPIED};

    fn ring_scan(radius_m: f64) -> LaserScan2d {
        let beam_count = 72;
        LaserScan2d {
            time_s: 0.0,
            frame: FrameId::new("laser"),
            angle_min_rad: 0.0,
            angle_increment_rad: std::f64::consts::TAU / beam_count as f64,
            range_min_m: 0.1,
            range_max_m: 10.0,
            ranges_m: vec![radius_m; beam_count],
        }
    }

    #[test]
    fn integration_marks_center_free_and_ring_occupied() {
        let mut grid = OccupancyGrid::new(21, 21, 0.2, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
        // Two passes so free cells cross the 0.35 probability threshold.
        for _ in 0..2 {
            integrate_scan(
                &mut grid,
                &ring_scan(1.0),
                Pose2d::IDENTITY,
                &ScanIntegrationConfig::default(),
            )
            .unwrap();
        }
        // Cell containing the sensor is cleared.
        let center = grid.world_to_grid(Vec3::new(0.0, 0.0, 0.0)).unwrap();
        assert_eq!(grid.cell_value(center), CELL_FREE);
        // A cell near the ring is occupied.
        let ring = grid.world_to_grid(Vec3::new(1.0, 0.0, 0.0)).unwrap();
        assert_eq!(grid.cell_value(ring), CELL_OCCUPIED);
    }

    #[test]
    fn beam_stride_skips_beams() {
        let mut grid = OccupancyGrid::new(21, 21, 0.2, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
        let config = ScanIntegrationConfig {
            beam_stride: 8,
            ..ScanIntegrationConfig::default()
        };
        let report = integrate_scan(&mut grid, &ring_scan(1.0), Pose2d::IDENTITY, &config).unwrap();
        assert_eq!(report.beams_processed, 9);
    }

    #[test]
    fn rejects_non_finite_scan() {
        let mut grid = OccupancyGrid::new(4, 4, 1.0, Pose2d::IDENTITY).unwrap();
        let mut scan = ring_scan(1.0);
        scan.angle_increment_rad = f64::NAN;
        assert_eq!(
            integrate_scan(
                &mut grid,
                &scan,
                Pose2d::IDENTITY,
                &ScanIntegrationConfig::default()
            ),
            Err(GridError::NonFinite)
        );
    }
}

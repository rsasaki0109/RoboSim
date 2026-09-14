//! Project a 3D point cloud into a planar occupancy grid.
//!
//! This is the outdoor mapping path: points inside a ground-relative height
//! band are treated as obstacles and projected onto the navigation plane
//! (`world X-Z`), while rays from the sensor clear free space behind them. It is
//! the 3D counterpart of [`crate::scan::integrate_scan`].

use crate::grid::{GridError, OccupancyGrid};
use crate::pose2d::Pose2d;
use crate::scan::{ScanIntegrationConfig, ScanIntegrationReport};
use rne_math::Vec3;

/// Integrates world-frame points whose height falls within `[min_height_m, max_height_m]`.
///
/// Height is the world `Y` axis; the navigation plane is `world X-Z`, so a
/// point `(x, y, z)` projects to grid `(x, z)`.
pub fn integrate_point_cloud(
    grid: &mut OccupancyGrid,
    points_world_m: &[Vec3],
    sensor_pose_world: Pose2d,
    min_height_m: f64,
    max_height_m: f64,
    config: &ScanIntegrationConfig,
) -> Result<ScanIntegrationReport, GridError> {
    if !sensor_pose_world.is_finite()
        || !min_height_m.is_finite()
        || !max_height_m.is_finite()
        || max_height_m < min_height_m
        || !config.occupied_log_odds.is_finite()
        || !config.free_log_odds.is_finite()
    {
        return Err(GridError::NonFinite);
    }

    let origin = sensor_pose_world.transform_point(Vec3::ZERO);
    let origin_flat = Vec3::new(origin.x, origin.z, 0.0);
    let mut report = ScanIntegrationReport::default();

    for point in points_world_m {
        if point.y < min_height_m || point.y > max_height_m {
            continue;
        }
        let end = Vec3::new(point.x, point.z, 0.0);
        let cells = grid.raycast(origin_flat, end);
        if cells.is_empty() {
            report.beams_skipped += 1;
            continue;
        }
        if config.mark_free {
            for coord in &cells[..cells.len() - 1] {
                if grid.apply_free(*coord, config.free_log_odds) {
                    report.free_updates += 1;
                }
            }
        }
        if grid.apply_occupied(cells[cells.len() - 1], config.occupied_log_odds) {
            report.occupied_updates += 1;
        }
        report.beams_processed += 1;
    }

    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn projects_points_within_the_height_band() {
        let mut grid = OccupancyGrid::new(20, 20, 0.1, Pose2d::new(-1.0, -1.0, 0.0)).unwrap();
        let points = [
            Vec3::new(0.5, 0.5, 0.5),  // within band
            Vec3::new(0.5, 5.0, 0.5),  // above band, ignored
            Vec3::new(-0.5, 0.2, 0.5), // within band
        ];
        let report = integrate_point_cloud(
            &mut grid,
            &points,
            Pose2d::IDENTITY,
            0.0,
            1.0,
            &ScanIntegrationConfig::default(),
        )
        .unwrap();
        assert_eq!(report.beams_processed, 2);
        let near = grid.world_to_grid(Vec3::new(0.5, 0.5, 0.0)).unwrap();
        assert!(grid.is_occupied(near));
        let other = grid.world_to_grid(Vec3::new(-0.5, 0.5, 0.0)).unwrap();
        assert!(grid.is_occupied(other));
    }

    #[test]
    fn rejects_invalid_height_band() {
        let mut grid = OccupancyGrid::new(4, 4, 1.0, Pose2d::IDENTITY).unwrap();
        assert_eq!(
            integrate_point_cloud(
                &mut grid,
                &[],
                Pose2d::IDENTITY,
                2.0,
                1.0,
                &ScanIntegrationConfig::default()
            ),
            Err(GridError::NonFinite)
        );
    }
}

//! Multi-session occupancy-map merging.
//!
//! [`merge_maps`] fuses two axis-aligned occupancy grids into one spanning their
//! union. Overlapping cells sum their fixed-point log-odds (clamped to the grid
//! storage limits), so evidence from both sessions accumulates; cells covered by
//! only one map keep that map's value. This is the map-side counterpart of
//! combining pose graphs across sessions.

use crate::grid::{GridError, OccupancyGrid, MAX_LOG_ODDS, MIN_LOG_ODDS};
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use thiserror::Error;

/// Errors raised by map merging.
#[derive(Debug, Error)]
pub enum MapMergeError {
    /// The two maps used different resolutions.
    #[error("map merge requires equal resolutions")]
    ResolutionMismatch,
    /// An origin carried a non-zero yaw, which the merge does not support.
    #[error("map merge requires axis-aligned maps")]
    RotatedOrigin,
    /// An origin was not finite.
    #[error("map origin is not finite")]
    NonFinite,
    /// The merged geometry was invalid.
    #[error("invalid map geometry: {0}")]
    Grid(#[from] GridError),
}

/// Fuses two occupancy grids over the union of their extents.
pub fn merge_maps(a: &OccupancyGrid, b: &OccupancyGrid) -> Result<OccupancyGrid, MapMergeError> {
    if !a.origin().is_finite() || !b.origin().is_finite() {
        return Err(MapMergeError::NonFinite);
    }
    if (a.resolution_m() - b.resolution_m()).abs() > f64::EPSILON {
        return Err(MapMergeError::ResolutionMismatch);
    }
    if a.origin().yaw_rad.abs() > 1.0e-9 || b.origin().yaw_rad.abs() > 1.0e-9 {
        return Err(MapMergeError::RotatedOrigin);
    }

    let resolution = a.resolution_m();
    let a_min_x = a.origin().x_m;
    let a_min_y = a.origin().y_m;
    let b_min_x = b.origin().x_m;
    let b_min_y = b.origin().y_m;
    let a_max_x = a_min_x + a.width() as f64 * resolution;
    let a_max_y = a_min_y + a.height() as f64 * resolution;
    let b_max_x = b_min_x + b.width() as f64 * resolution;
    let b_max_y = b_min_y + b.height() as f64 * resolution;

    let min_x = a_min_x.min(b_min_x);
    let min_y = a_min_y.min(b_min_y);
    let max_x = a_max_x.max(b_max_x);
    let max_y = a_max_y.max(b_max_y);
    let width = (((max_x - min_x) / resolution).ceil() as usize).max(1);
    let height = (((max_y - min_y) / resolution).ceil() as usize).max(1);
    let origin = Pose2d::new(min_x, min_y, 0.0);

    let mut log_odds = Vec::with_capacity(width * height);
    for y in 0..height {
        for x in 0..width {
            let center = Vec3::new(
                min_x + (x as f64 + 0.5) * resolution,
                min_y + (y as f64 + 0.5) * resolution,
                0.0,
            );
            let value = sample(a, center).unwrap_or(0) + sample(b, center).unwrap_or(0);
            log_odds.push(value.clamp(MIN_LOG_ODDS, MAX_LOG_ODDS));
        }
    }
    Ok(OccupancyGrid::from_log_odds(
        width, height, resolution, origin, log_odds,
    )?)
}

fn sample(grid: &OccupancyGrid, point_m: Vec3) -> Option<i16> {
    let coord = grid.world_to_grid(point_m)?;
    let index = coord.y as usize * grid.width() + coord.x as usize;
    grid.log_odds().get(index).copied()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::grid::GridCoord;

    #[test]
    fn merges_overlapping_maps() {
        let mut a = OccupancyGrid::new(2, 2, 1.0, Pose2d::IDENTITY).unwrap();
        a.apply_occupied(GridCoord { x: 0, y: 0 }, 0.85);
        let b = OccupancyGrid::new(2, 2, 1.0, Pose2d::IDENTITY).unwrap();
        let merged = merge_maps(&a, &b).unwrap();
        assert_eq!(merged.width(), 2);
        assert_eq!(merged.height(), 2);
        assert!(merged.is_occupied(GridCoord { x: 0, y: 0 }));
    }

    #[test]
    fn spans_the_union_of_offset_maps() {
        let a = OccupancyGrid::new(2, 2, 1.0, Pose2d::IDENTITY).unwrap();
        let mut b = OccupancyGrid::new(2, 2, 1.0, Pose2d::new(1.0, 0.0, 0.0)).unwrap();
        b.apply_occupied(GridCoord { x: 0, y: 0 }, 0.85);
        let merged = merge_maps(&a, &b).unwrap();
        assert_eq!(merged.width(), 3);
        assert_eq!(merged.height(), 2);
        assert!(merged.is_occupied(GridCoord { x: 1, y: 0 }));
    }

    #[test]
    fn sums_evidence_in_overlap() {
        let mut a = OccupancyGrid::new(1, 1, 1.0, Pose2d::IDENTITY).unwrap();
        let mut b = OccupancyGrid::new(1, 1, 1.0, Pose2d::IDENTITY).unwrap();
        a.apply_occupied(GridCoord { x: 0, y: 0 }, 0.5);
        b.apply_occupied(GridCoord { x: 0, y: 0 }, 0.5);
        let merged = merge_maps(&a, &b).unwrap();
        assert!(merged.log_odds()[0] > a.log_odds()[0]);
    }

    #[test]
    fn rejects_mismatched_resolution_and_rotation() {
        let a = OccupancyGrid::new(2, 2, 1.0, Pose2d::IDENTITY).unwrap();
        let b = OccupancyGrid::new(2, 2, 0.5, Pose2d::IDENTITY).unwrap();
        assert!(matches!(
            merge_maps(&a, &b),
            Err(MapMergeError::ResolutionMismatch)
        ));
        let rotated = OccupancyGrid::new(2, 2, 1.0, Pose2d::new(0.0, 0.0, 0.5)).unwrap();
        assert!(matches!(
            merge_maps(&a, &rotated),
            Err(MapMergeError::RotatedOrigin)
        ));
    }
}

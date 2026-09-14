//! Deterministic 2D occupancy grid with log-odds updates and grid ray casting.

use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use thiserror::Error;

/// Fixed-point scale for log-odds stored as `i16`.
pub const LOG_ODDS_SCALE: f64 = 1000.0;
/// Lower clamp for accumulated log-odds.
pub const MIN_LOG_ODDS: i16 = -2000;
/// Upper clamp for accumulated log-odds.
pub const MAX_LOG_ODDS: i16 = 3500;
/// Default log-odds increment applied to an occupied return.
pub const DEFAULT_OCCUPIED_LOG_ODDS: f64 = 0.85;
/// Default log-odds increment applied to a free-space traversal.
pub const DEFAULT_FREE_LOG_ODDS: f64 = -0.4;
/// Probability at or above which a cell is reported occupied by default.
pub const DEFAULT_OCCUPIED_PROBABILITY: f64 = 0.65;
/// Probability at or below which a cell is reported free by default.
pub const DEFAULT_FREE_PROBABILITY: f64 = 0.35;
/// Map cell value for an unknown cell, matching the ROS occupancy grid convention.
pub const CELL_UNKNOWN: i8 = -1;
/// Map cell value for free space.
pub const CELL_FREE: i8 = 0;
/// Map cell value for occupied space.
pub const CELL_OCCUPIED: i8 = 100;

/// Integer grid coordinate.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, PartialOrd, Ord, Serialize, Deserialize)]
pub struct GridCoord {
    /// Column index.
    pub x: isize,
    /// Row index.
    pub y: isize,
}

/// Error returned by grid construction or projection.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Error)]
pub enum GridError {
    /// Resolution was not finite and strictly positive.
    #[error("grid resolution must be finite and positive")]
    InvalidResolution,
    /// Width or height was zero.
    #[error("grid dimensions must be non-zero")]
    InvalidSize,
    /// A pose or point contained a non-finite value.
    #[error("grid input contained a non-finite value")]
    NonFinite,
}

/// Row-major 2D occupancy grid in log-odds space.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct OccupancyGrid {
    width: usize,
    height: usize,
    resolution_m: f64,
    origin: Pose2d,
    log_odds: Vec<i16>,
}

impl OccupancyGrid {
    /// Creates an unknown grid whose cell `(0, 0)` center sits at `origin`.
    pub fn new(
        width: usize,
        height: usize,
        resolution_m: f64,
        origin: Pose2d,
    ) -> Result<Self, GridError> {
        if width == 0 || height == 0 {
            return Err(GridError::InvalidSize);
        }
        if !resolution_m.is_finite() || resolution_m <= 0.0 {
            return Err(GridError::InvalidResolution);
        }
        if !origin.is_finite() {
            return Err(GridError::NonFinite);
        }
        Ok(Self {
            width,
            height,
            resolution_m,
            origin,
            log_odds: vec![0; width * height],
        })
    }

    /// Restores a grid from raw fixed-point log-odds values.
    ///
    /// The value count must equal `width * height`; values outside the storage
    /// clamp are accepted as-is so a map round-trips exactly.
    pub fn from_log_odds(
        width: usize,
        height: usize,
        resolution_m: f64,
        origin: Pose2d,
        log_odds: Vec<i16>,
    ) -> Result<Self, GridError> {
        let mut grid = Self::new(width, height, resolution_m, origin)?;
        if log_odds.len() != width * height {
            return Err(GridError::InvalidSize);
        }
        grid.log_odds = log_odds;
        Ok(grid)
    }

    /// Grid width in cells.
    pub fn width(&self) -> usize {
        self.width
    }

    /// Grid height in cells.
    pub fn height(&self) -> usize {
        self.height
    }

    /// Cell size in meters.
    pub fn resolution_m(&self) -> f64 {
        self.resolution_m
    }

    /// World pose of cell `(0, 0)`'s center.
    pub fn origin(&self) -> Pose2d {
        self.origin
    }

    /// Number of cells.
    pub fn len(&self) -> usize {
        self.log_odds.len()
    }

    /// Whether the grid has no cells (always false for a valid grid).
    pub fn is_empty(&self) -> bool {
        self.log_odds.is_empty()
    }

    /// Raw log-odds values in row-major order.
    pub fn log_odds(&self) -> &[i16] {
        &self.log_odds
    }

    /// Whether a coordinate lies inside the grid.
    pub fn contains(&self, coord: GridCoord) -> bool {
        coord.x >= 0
            && coord.y >= 0
            && (coord.x as usize) < self.width
            && (coord.y as usize) < self.height
    }

    /// Projects a world point onto integer cell coordinates, if inside.
    pub fn world_to_grid(&self, point_m: Vec3) -> Option<GridCoord> {
        let (x, y) = self.world_to_grid_continuous(point_m);
        let coord = GridCoord {
            x: x.floor() as isize,
            y: y.floor() as isize,
        };
        if self.contains(coord) {
            Some(coord)
        } else {
            None
        }
    }

    /// World-space center of a cell.
    pub fn grid_to_world(&self, coord: GridCoord) -> Vec3 {
        self.origin.transform_point(Vec3::new(
            (coord.x as f64 + 0.5) * self.resolution_m,
            (coord.y as f64 + 0.5) * self.resolution_m,
            0.0,
        ))
    }

    /// Occupancy probability of a cell, or `None` when outside the grid.
    pub fn probability(&self, coord: GridCoord) -> Option<f64> {
        self.contains(coord)
            .then_some(self.probability_at_index(self.index(coord.x as usize, coord.y as usize)))
    }

    /// Whether a cell is occupied above the default threshold.
    pub fn is_occupied(&self, coord: GridCoord) -> bool {
        self.probability(coord)
            .map(|p| p >= DEFAULT_OCCUPIED_PROBABILITY)
            .unwrap_or(false)
    }

    /// Whether a cell is free below the default threshold.
    pub fn is_free(&self, coord: GridCoord) -> bool {
        self.probability(coord)
            .map(|p| p <= DEFAULT_FREE_PROBABILITY)
            .unwrap_or(false)
    }

    /// Whether a cell has been observed.
    pub fn is_known(&self, coord: GridCoord) -> bool {
        self.is_occupied(coord) || self.is_free(coord)
    }

    /// Map-style cell value: `-1` unknown, `0` free, `100` occupied.
    pub fn cell_value(&self, coord: GridCoord) -> i8 {
        if self.is_occupied(coord) {
            CELL_OCCUPIED
        } else if self.is_free(coord) {
            CELL_FREE
        } else {
            CELL_UNKNOWN
        }
    }

    /// Applies an occupied update to a cell, returning whether it was in bounds.
    pub fn apply_occupied(&mut self, coord: GridCoord, log_odds: f64) -> bool {
        self.apply(coord, log_odds)
    }

    /// Applies a free update to a cell, returning whether it was in bounds.
    pub fn apply_free(&mut self, coord: GridCoord, log_odds: f64) -> bool {
        self.apply(coord, log_odds)
    }

    /// Marks a cell occupied with the default increment.
    pub fn mark_occupied(&mut self, coord: GridCoord) -> bool {
        self.apply_occupied(coord, DEFAULT_OCCUPIED_LOG_ODDS)
    }

    /// Marks a cell free with the default increment.
    pub fn mark_free(&mut self, coord: GridCoord) -> bool {
        self.apply_free(coord, DEFAULT_FREE_LOG_ODDS)
    }

    /// Resets a cell to unknown.
    pub fn reset(&mut self, coord: GridCoord) {
        if self.contains(coord) {
            let index = self.index(coord.x as usize, coord.y as usize);
            self.log_odds[index] = 0;
        }
    }

    /// Resets the whole grid to unknown.
    pub fn clear(&mut self) {
        self.log_odds.fill(0);
    }

    /// Casts a world-space ray and returns the in-bounds cells it crosses.
    ///
    /// The returned cells are ordered from `from_m` toward `to_m` and include
    /// both endpoints when they are inside the grid. Out-of-bounds cells are
    /// skipped, so the last returned cell is where the ray leaves the grid (or
    /// the requested endpoint).
    pub fn raycast(&self, from_m: Vec3, to_m: Vec3) -> Vec<GridCoord> {
        let (start_x, start_y) = self.world_to_grid_continuous(from_m);
        let (end_x, end_y) = self.world_to_grid_continuous(to_m);
        if !start_x.is_finite() || !start_y.is_finite() || !end_x.is_finite() || !end_y.is_finite()
        {
            return Vec::new();
        }

        let end_coord = GridCoord {
            x: end_x.floor() as isize,
            y: end_y.floor() as isize,
        };
        let mut coord = GridCoord {
            x: start_x.floor() as isize,
            y: start_y.floor() as isize,
        };

        let dx = end_x - start_x;
        let dy = end_y - start_y;
        let step_x = dx.signum() as isize;
        let step_y = dy.signum() as isize;
        let inv_dx = if dx != 0.0 { 1.0 / dx } else { f64::INFINITY };
        let inv_dy = if dy != 0.0 { 1.0 / dy } else { f64::INFINITY };
        let mut t_max_x = if dx != 0.0 {
            ((if step_x > 0 {
                coord.x as f64 + 1.0
            } else {
                coord.x as f64
            }) - start_x)
                * inv_dx
        } else {
            f64::INFINITY
        };
        let mut t_max_y = if dy != 0.0 {
            ((if step_y > 0 {
                coord.y as f64 + 1.0
            } else {
                coord.y as f64
            }) - start_y)
                * inv_dy
        } else {
            f64::INFINITY
        };
        let t_delta_x = if dx != 0.0 {
            (step_x as f64 * inv_dx).abs()
        } else {
            f64::INFINITY
        };
        let t_delta_y = if dy != 0.0 {
            (step_y as f64 * inv_dy).abs()
        } else {
            f64::INFINITY
        };

        let mut cells = Vec::new();
        let max_steps = self.width + self.height + 4;
        for _ in 0..max_steps {
            if self.contains(coord) {
                cells.push(coord);
            }
            if coord == end_coord {
                break;
            }
            if t_max_x < t_max_y {
                coord.x += step_x;
                t_max_x += t_delta_x;
            } else if t_max_y.is_finite() {
                coord.y += step_y;
                t_max_y += t_delta_y;
            } else {
                break;
            }
        }
        cells
    }

    fn apply(&mut self, coord: GridCoord, log_odds: f64) -> bool {
        if !self.contains(coord) || !log_odds.is_finite() {
            return false;
        }
        let index = self.index(coord.x as usize, coord.y as usize);
        let quantized = (log_odds * LOG_ODDS_SCALE).round() as i32;
        let updated = (self.log_odds[index] as i32 + quantized)
            .clamp(MIN_LOG_ODDS as i32, MAX_LOG_ODDS as i32);
        self.log_odds[index] = updated as i16;
        true
    }

    fn index(&self, x: usize, y: usize) -> usize {
        y * self.width + x
    }

    fn probability_at_index(&self, index: usize) -> f64 {
        let log_odds = self.log_odds[index] as f64 / LOG_ODDS_SCALE;
        let odds = log_odds.exp();
        odds / (1.0 + odds)
    }

    fn world_to_grid_continuous(&self, point_m: Vec3) -> (f64, f64) {
        let local = self.origin.inverse_transform_point(point_m);
        (local.x / self.resolution_m, local.y / self.resolution_m)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn grid() -> OccupancyGrid {
        OccupancyGrid::new(10, 10, 1.0, Pose2d::IDENTITY).unwrap()
    }

    #[test]
    fn world_grid_round_trip() {
        let grid = OccupancyGrid::new(10, 10, 0.5, Pose2d::new(-2.0, -2.0, 0.0)).unwrap();
        let coord = GridCoord { x: 4, y: 6 };
        let world = grid.grid_to_world(coord);
        assert_eq!(grid.world_to_grid(world), Some(coord));
    }

    #[test]
    fn occupied_updates_raise_probability() {
        let mut grid = grid();
        let coord = GridCoord { x: 3, y: 3 };
        assert_eq!(grid.cell_value(coord), CELL_UNKNOWN);
        grid.mark_occupied(coord);
        assert!(grid.is_occupied(coord));
        assert_relative_eq!(grid.probability(coord).unwrap(), 0.7, epsilon = 0.05);
        for _ in 0..4 {
            grid.mark_free(coord);
        }
        assert!(grid.is_free(coord));
    }

    #[test]
    fn raycast_visits_a_straight_line() {
        let grid = grid();
        let cells = grid.raycast(Vec3::new(0.5, 0.5, 0.0), Vec3::new(4.5, 0.5, 0.0));
        let xs: Vec<isize> = cells.iter().map(|coord| coord.x).collect();
        assert_eq!(xs, vec![0, 1, 2, 3, 4]);
        assert!(cells.iter().all(|coord| coord.y == 0));
    }

    #[test]
    fn reset_clears_a_cell() {
        let mut grid = grid();
        let coord = GridCoord { x: 1, y: 1 };
        grid.mark_occupied(coord);
        grid.reset(coord);
        assert!(!grid.is_known(coord));
    }
}

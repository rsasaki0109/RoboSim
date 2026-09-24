//! Deterministic 2.5D elevation map for 3D LiDAR and point-cloud mapping.
//!
//! [`ElevationMap`] stores a running minimum, maximum, and mean height per grid
//! cell. Points are projected onto the navigation plane (`world X-Z`) and keep
//! their `world Y` height, so slope and step queries describe traversability for
//! ground robots. Iteration is index-ordered and no random numbers are used, so
//! integrating a recorded point sequence reproduces the same map.

use crate::grid::{GridCoord, GridError, OccupancyGrid};
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};

/// Tunables for an elevation map.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ElevationConfig {
    /// Maximum intra-cell height difference still considered traversable, in meters.
    pub max_step_m: f64,
}

impl Default for ElevationConfig {
    fn default() -> Self {
        Self { max_step_m: 0.3 }
    }
}

/// Aggregated height statistics for one cell.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct ElevationCell {
    /// Lowest observed height in meters (`+inf` when empty).
    pub min_y_m: f64,
    /// Highest observed height in meters (`-inf` when empty).
    pub max_y_m: f64,
    /// Running mean height in meters.
    pub mean_y_m: f64,
    /// Number of points accumulated.
    pub count: u32,
}

impl ElevationCell {
    /// An empty cell.
    pub const EMPTY: Self = Self {
        min_y_m: f64::INFINITY,
        max_y_m: f64::NEG_INFINITY,
        mean_y_m: 0.0,
        count: 0,
    };

    /// Whether the cell has received at least one point.
    pub fn is_known(&self) -> bool {
        self.count > 0
    }

    /// Intra-cell height range in meters, or `None` when empty.
    pub fn range_m(&self) -> Option<f64> {
        self.is_known().then_some(self.max_y_m - self.min_y_m)
    }
}

/// Aggregate result of integrating a point cloud.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct ElevationReport {
    /// Points that were inside the map and finite.
    pub points_processed: usize,
    /// Points outside the map or non-finite.
    pub points_skipped: usize,
    /// Cells touched by at least one point.
    pub cells_updated: usize,
}

/// A row-major 2.5D elevation map in the navigation plane.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct ElevationMap {
    width: usize,
    height: usize,
    resolution_m: f64,
    origin: Pose2d,
    config: ElevationConfig,
    cells: Vec<ElevationCell>,
}

impl ElevationMap {
    /// Creates an empty elevation map whose cell `(0, 0)` center sits at `origin`.
    pub fn new(
        width: usize,
        height: usize,
        resolution_m: f64,
        origin: Pose2d,
        config: ElevationConfig,
    ) -> Result<Self, GridError> {
        if width == 0 || height == 0 {
            return Err(GridError::InvalidSize);
        }
        if !resolution_m.is_finite() || resolution_m <= 0.0 {
            return Err(GridError::InvalidResolution);
        }
        if !origin.is_finite() || !config.max_step_m.is_finite() || config.max_step_m < 0.0 {
            return Err(GridError::NonFinite);
        }
        Ok(Self {
            width,
            height,
            resolution_m,
            origin,
            config,
            cells: vec![ElevationCell::EMPTY; width * height],
        })
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

    /// The configuration.
    pub fn config(&self) -> ElevationConfig {
        self.config
    }

    /// Number of cells.
    pub fn len(&self) -> usize {
        self.cells.len()
    }

    /// Whether the map has no cells (always false for a valid map).
    pub fn is_empty(&self) -> bool {
        self.cells.is_empty()
    }

    /// Cell statistics in row-major order.
    pub fn cells(&self) -> &[ElevationCell] {
        &self.cells
    }

    /// Whether a coordinate lies inside the map.
    pub fn contains(&self, coord: GridCoord) -> bool {
        coord.x >= 0
            && coord.y >= 0
            && (coord.x as usize) < self.width
            && (coord.y as usize) < self.height
    }

    /// Projects a world point onto integer cell coordinates, if inside.
    ///
    /// The navigation plane is `world X-Z`; `world Y` is the height.
    pub(crate) fn world_to_cell(&self, point_m: Vec3) -> Option<GridCoord> {
        if !point_m.is_finite() {
            return None;
        }
        let flat = Vec3::new(point_m.x, point_m.z, 0.0);
        let local = self.origin.inverse_transform_point(flat);
        let coord = GridCoord {
            x: (local.x / self.resolution_m).floor() as isize,
            y: (local.y / self.resolution_m).floor() as isize,
        };
        self.contains(coord).then_some(coord)
    }

    /// Integrates finite world-frame points, returning the aggregate report.
    pub fn integrate(&mut self, points_world_m: &[Vec3]) -> ElevationReport {
        let mut report = ElevationReport::default();
        for point in points_world_m {
            let Some(coord) = self.world_to_cell(*point) else {
                report.points_skipped += 1;
                continue;
            };
            let index = self.index(coord.x as usize, coord.y as usize);
            let cell = &mut self.cells[index];
            if !cell.is_known() {
                report.cells_updated += 1;
            }
            let height = point.y;
            if cell.count == 0 {
                cell.min_y_m = height;
                cell.max_y_m = height;
                cell.mean_y_m = height;
            } else {
                cell.min_y_m = cell.min_y_m.min(height);
                cell.max_y_m = cell.max_y_m.max(height);
                let count = cell.count as f64;
                cell.mean_y_m += (height - cell.mean_y_m) / (count + 1.0);
            }
            cell.count += 1;
            report.points_processed += 1;
        }
        report
    }

    /// Cell statistics at integer coordinates, if inside.
    pub fn cell(&self, coord: GridCoord) -> Option<&ElevationCell> {
        self.contains(coord)
            .then(|| &self.cells[self.index(coord.x as usize, coord.y as usize)])
    }

    /// Mean height at integer coordinates, if the cell is known.
    pub fn height_at(&self, coord: GridCoord) -> Option<f64> {
        self.cell(coord)
            .filter(|c| c.is_known())
            .map(|c| c.mean_y_m)
    }

    /// Maximum slope to the `+x` and `+y` neighbors, in radians.
    ///
    /// Returns `None` when the cell or one of its forward neighbors is unknown.
    pub fn slope_at(&self, coord: GridCoord) -> Option<f64> {
        let here = self.height_at(coord)?;
        let right = self.height_at(GridCoord {
            x: coord.x + 1,
            y: coord.y,
        });
        let down = self.height_at(GridCoord {
            x: coord.x,
            y: coord.y + 1,
        });
        let mut dz_dx = 0.0;
        let mut dz_dy = 0.0;
        let mut known = false;
        if let Some(right) = right {
            dz_dx = (right - here) / self.resolution_m;
            known = true;
        }
        if let Some(down) = down {
            dz_dy = (down - here) / self.resolution_m;
            known = true;
        }
        known.then(|| (dz_dx * dz_dx + dz_dy * dz_dy).sqrt().atan())
    }

    /// Whether a cell is known, has a small step, and is within the slope limit.
    pub fn is_traversable(&self, coord: GridCoord, max_slope_rad: f64) -> bool {
        let Some(cell) = self.cell(coord).filter(|c| c.is_known()) else {
            return false;
        };
        if cell.range_m().unwrap_or(f64::INFINITY) > self.config.max_step_m {
            return false;
        }
        self.slope_at(coord)
            .map(|slope| slope <= max_slope_rad)
            .unwrap_or(true)
    }

    /// Rebuilds a planar occupancy grid marking cells whose step exceeds the
    /// traversability threshold.
    pub fn to_obstacle_grid(&self) -> Result<OccupancyGrid, GridError> {
        let mut grid = OccupancyGrid::new(self.width, self.height, self.resolution_m, self.origin)?;
        for y in 0..self.height {
            for x in 0..self.width {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                let cell = self.cells[self.index(x, y)];
                if cell.is_known() && cell.range_m().unwrap_or(0.0) > self.config.max_step_m {
                    grid.apply_occupied(coord, 0.85);
                } else if cell.is_known() {
                    grid.apply_free(coord, -0.4);
                }
            }
        }
        Ok(grid)
    }

    fn index(&self, x: usize, y: usize) -> usize {
        y * self.width + x
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn flat_map() -> ElevationMap {
        ElevationMap::new(10, 10, 0.5, Pose2d::IDENTITY, ElevationConfig::default()).unwrap()
    }

    #[test]
    fn integrates_running_mean_and_range() {
        let mut map = flat_map();
        let points = [
            Vec3::new(0.25, 1.0, 0.25),
            Vec3::new(0.25, 3.0, 0.25),
            Vec3::new(5.0, 1.0, 5.0), // outside the 5 m x 5 m frame
        ];
        let report = map.integrate(&points);
        assert_eq!(report.points_processed, 2);
        assert_eq!(report.points_skipped, 1);
        let coord = GridCoord { x: 0, y: 0 };
        let cell = map.cell(coord).unwrap();
        assert_eq!(cell.count, 2);
        assert_relative_eq!(cell.mean_y_m, 2.0, epsilon = 1e-12);
        assert_relative_eq!(cell.range_m().unwrap(), 2.0, epsilon = 1e-12);
    }

    #[test]
    fn slope_detects_a_ramp() {
        let mut map = flat_map();
        for i in 0..4 {
            let x = 0.25 + i as f64 * 0.5;
            map.integrate(&[Vec3::new(x, i as f64 * 0.5, 0.25)]);
        }
        let slope = map.slope_at(GridCoord { x: 0, y: 0 }).unwrap();
        assert_relative_eq!(slope, std::f64::consts::FRAC_PI_4, epsilon = 1e-9);
        assert!(!map.is_traversable(GridCoord { x: 0, y: 0 }, 0.1));
        assert!(map.is_traversable(GridCoord { x: 0, y: 0 }, 1.0));
    }

    #[test]
    fn unknown_cells_are_not_traversable() {
        let map = flat_map();
        assert_eq!(map.height_at(GridCoord { x: 3, y: 3 }), None);
        assert!(!map.is_traversable(GridCoord { x: 3, y: 3 }, 1.0));
        assert_eq!(map.slope_at(GridCoord { x: 3, y: 3 }), None);
    }

    #[test]
    fn obstacle_grid_marks_large_steps() {
        let mut map = flat_map();
        map.integrate(&[Vec3::new(0.25, 0.0, 0.25)]);
        map.integrate(&[Vec3::new(0.25, 1.5, 0.25)]);
        let grid = map.to_obstacle_grid().unwrap();
        assert!(grid.is_occupied(GridCoord { x: 0, y: 0 }));
    }

    #[test]
    fn rejects_invalid_parameters() {
        assert_eq!(
            ElevationMap::new(0, 1, 0.5, Pose2d::IDENTITY, ElevationConfig::default()),
            Err(GridError::InvalidSize)
        );
        assert_eq!(
            ElevationMap::new(1, 1, 0.0, Pose2d::IDENTITY, ElevationConfig::default()),
            Err(GridError::InvalidResolution)
        );
    }
}

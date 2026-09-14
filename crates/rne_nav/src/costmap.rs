//! Planar costmap derived from an occupancy grid.
//!
//! Costs follow the ROS navigation convention: `0` is free, `253` is the
//! inscribed radius, `254` is lethal, and `255` is unknown. Inflation decays
//! exponentially from the inscribed radius to the configured inflation radius,
//! which keeps the map deterministic and planner-friendly.

use crate::grid::{GridCoord, GridError, OccupancyGrid};
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::f64::consts::SQRT_2;

/// Cost of free space.
pub const COST_FREE: u8 = 0;
/// Cost at the robot's inscribed radius.
pub const COST_INSCRIBED: u8 = 253;
/// Cost of an occupied cell.
pub const COST_LETHAL: u8 = 254;
/// Cost of an unobserved cell.
pub const COST_NO_INFORMATION: u8 = 255;

/// Configuration for costmap generation.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct CostmapConfig {
    /// Probability at or above which a cell is lethal.
    pub occupied_probability: f64,
    /// Probability at or below which a cell is free.
    pub free_probability: f64,
    /// Radius in meters within which cells are at least inscribed cost.
    pub inscribed_radius_m: f64,
    /// Radius in meters at which inflated cost reaches zero.
    pub inflation_radius_m: f64,
    /// Exponential decay factor for inflated cost.
    pub cost_scaling_factor: f64,
}

impl Default for CostmapConfig {
    fn default() -> Self {
        Self {
            occupied_probability: 0.65,
            free_probability: 0.35,
            inscribed_radius_m: 0.20,
            inflation_radius_m: 0.55,
            cost_scaling_factor: 3.0,
        }
    }
}

/// A row-major 2D cost grid aligned with an [`OccupancyGrid`].
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct Costmap {
    width: usize,
    height: usize,
    resolution_m: f64,
    origin: Pose2d,
    costs: Vec<u8>,
}

impl Costmap {
    /// Builds a costmap from an occupancy grid and configuration.
    pub fn from_occupancy(grid: &OccupancyGrid, config: &CostmapConfig) -> Result<Self, GridError> {
        if !config.inscribed_radius_m.is_finite()
            || !config.inflation_radius_m.is_finite()
            || !config.cost_scaling_factor.is_finite()
            || config.inscribed_radius_m < 0.0
            || config.inflation_radius_m < config.inscribed_radius_m
            || config.cost_scaling_factor <= 0.0
        {
            return Err(GridError::NonFinite);
        }

        let width = grid.width();
        let height = grid.height();
        let resolution = grid.resolution_m();

        let mut lethal = vec![false; width * height];
        let mut known = vec![false; width * height];
        for y in 0..height {
            for x in 0..width {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                let index = y * width + x;
                let probability = grid.probability(coord).unwrap_or(0.5);
                if probability >= config.occupied_probability {
                    lethal[index] = true;
                    known[index] = true;
                } else if probability <= config.free_probability {
                    known[index] = true;
                }
            }
        }
        let distances = chamfer_distance(&lethal, width, height);

        let mut costs = vec![COST_NO_INFORMATION; width * height];
        for y in 0..height {
            for x in 0..width {
                let index = y * width + x;
                if lethal[index] {
                    costs[index] = COST_LETHAL;
                    continue;
                }
                if !known[index] {
                    continue;
                }
                let distance_m = distances[index] * resolution;
                if distance_m <= config.inscribed_radius_m {
                    costs[index] = COST_INSCRIBED;
                } else if distance_m < config.inflation_radius_m {
                    let decay =
                        -config.cost_scaling_factor * (distance_m - config.inscribed_radius_m);
                    let cost = (COST_INSCRIBED as f64 * decay.exp()).round();
                    costs[index] = cost.clamp(1.0, (COST_INSCRIBED - 1) as f64) as u8;
                } else {
                    costs[index] = COST_FREE;
                }
            }
        }

        Ok(Self {
            width,
            height,
            resolution_m: resolution,
            origin: grid.origin(),
            costs,
        })
    }

    /// Costmap width in cells.
    pub fn width(&self) -> usize {
        self.width
    }

    /// Costmap height in cells.
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

    /// Raw costs in row-major order.
    pub fn costs(&self) -> &[u8] {
        &self.costs
    }

    /// Whether a coordinate lies inside the costmap.
    pub fn contains(&self, coord: GridCoord) -> bool {
        coord.x >= 0
            && coord.y >= 0
            && (coord.x as usize) < self.width
            && (coord.y as usize) < self.height
    }

    /// Cost at a cell, or `None` when outside the costmap.
    pub fn cost_at(&self, coord: GridCoord) -> Option<u8> {
        self.contains(coord)
            .then_some(self.costs[coord.y as usize * self.width + coord.x as usize])
    }

    /// Projects a world point onto integer cell coordinates, if inside.
    pub fn world_to_grid(&self, point_m: Vec3) -> Option<GridCoord> {
        let local = self.origin.inverse_transform_point(point_m);
        let coord = GridCoord {
            x: (local.x / self.resolution_m).floor() as isize,
            y: (local.y / self.resolution_m).floor() as isize,
        };
        self.contains(coord).then_some(coord)
    }

    /// World-space center of a cell.
    pub fn grid_to_world(&self, coord: GridCoord) -> Vec3 {
        self.origin.transform_point(Vec3::new(
            (coord.x as f64 + 0.5) * self.resolution_m,
            (coord.y as f64 + 0.5) * self.resolution_m,
            0.0,
        ))
    }

    /// Whether a cell is lethal.
    pub fn is_lethal(&self, coord: GridCoord) -> bool {
        self.cost_at(coord) == Some(COST_LETHAL)
    }

    /// Whether a cell is traversable free space.
    pub fn is_free(&self, coord: GridCoord) -> bool {
        self.cost_at(coord) == Some(COST_FREE)
    }

    /// Number of lethal cells.
    pub fn lethal_count(&self) -> usize {
        self.costs
            .iter()
            .filter(|cost| **cost == COST_LETHAL)
            .count()
    }

    /// Merges a layer cost into a cell, returning whether it was in bounds.
    ///
    /// Unknown cells adopt the layer cost, lethal always wins, and otherwise the
    /// cell keeps the larger of its current cost and the layer cost. This lets a
    /// terrain or obstacle layer raise cost without lowering existing
    /// inflation or lethality.
    pub fn apply_cost(&mut self, coord: GridCoord, cost: u8) -> bool {
        if !self.contains(coord) {
            return false;
        }
        let index = coord.y as usize * self.width + coord.x as usize;
        let current = self.costs[index];
        self.costs[index] = if cost == COST_LETHAL {
            COST_LETHAL
        } else if current == COST_NO_INFORMATION || current == COST_FREE {
            cost
        } else {
            current.max(cost)
        };
        true
    }
}

/// Two-pass chamfer distance transform in cell units from the nearest lethal cell.
fn chamfer_distance(lethal: &[bool], width: usize, height: usize) -> Vec<f64> {
    let mut distance = vec![f64::INFINITY; lethal.len()];
    for (cell, is_lethal) in distance.iter_mut().zip(lethal) {
        if *is_lethal {
            *cell = 0.0;
        }
    }

    for y in 0..height {
        for x in 0..width {
            let index = y * width + x;
            if distance[index] == 0.0 {
                continue;
            }
            let mut best = distance[index];
            if x > 0 {
                best = best.min(distance[index - 1] + 1.0);
            }
            if y > 0 {
                best = best.min(distance[index - width] + 1.0);
                if x > 0 {
                    best = best.min(distance[index - width - 1] + SQRT_2);
                }
                if x + 1 < width {
                    best = best.min(distance[index - width + 1] + SQRT_2);
                }
            }
            distance[index] = best;
        }
    }

    for y in (0..height).rev() {
        for x in (0..width).rev() {
            let index = y * width + x;
            if distance[index] == 0.0 {
                continue;
            }
            let mut best = distance[index];
            if x + 1 < width {
                best = best.min(distance[index + 1] + 1.0);
            }
            if y + 1 < height {
                best = best.min(distance[index + width] + 1.0);
                if x + 1 < width {
                    best = best.min(distance[index + width + 1] + SQRT_2);
                }
                if x > 0 {
                    best = best.min(distance[index + width - 1] + SQRT_2);
                }
            }
            distance[index] = best;
        }
    }
    distance
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn grid_with_obstacle_at(center: GridCoord) -> OccupancyGrid {
        let mut grid = OccupancyGrid::new(20, 20, 0.1, Pose2d::IDENTITY).unwrap();
        for y in 0..grid.height() {
            for x in 0..grid.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                grid.mark_free(coord);
                grid.mark_free(coord);
            }
        }
        grid.reset(center);
        grid.mark_occupied(center);
        grid
    }

    #[test]
    fn lethal_and_inscribed_costs() {
        let grid = grid_with_obstacle_at(GridCoord { x: 10, y: 10 });
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();

        assert_eq!(
            costmap.cost_at(GridCoord { x: 10, y: 10 }),
            Some(COST_LETHAL)
        );
        assert_eq!(
            costmap.cost_at(GridCoord { x: 11, y: 10 }),
            Some(COST_INSCRIBED)
        );
        assert_relative_eq!(costmap.resolution_m(), 0.1);
    }

    #[test]
    fn inflation_decays_toward_free() {
        let grid = grid_with_obstacle_at(GridCoord { x: 10, y: 10 });
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
        let near = costmap.cost_at(GridCoord { x: 13, y: 10 }).unwrap();
        let far = costmap.cost_at(GridCoord { x: 19, y: 19 }).unwrap();
        assert!(near > 0 && near < COST_INSCRIBED, "near={near}");
        assert_eq!(far, COST_FREE);
    }

    #[test]
    fn unknown_cells_stay_unknown() {
        let mut grid = OccupancyGrid::new(4, 4, 1.0, Pose2d::IDENTITY).unwrap();
        grid.mark_occupied(GridCoord { x: 1, y: 1 });
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
        assert_eq!(
            costmap.cost_at(GridCoord { x: 3, y: 3 }),
            Some(COST_NO_INFORMATION)
        );
    }

    #[test]
    fn rejects_invalid_configuration() {
        let grid = OccupancyGrid::new(4, 4, 1.0, Pose2d::IDENTITY).unwrap();
        let config = CostmapConfig {
            inflation_radius_m: 0.1,
            inscribed_radius_m: 0.5,
            ..CostmapConfig::default()
        };
        assert_eq!(
            Costmap::from_occupancy(&grid, &config),
            Err(GridError::NonFinite)
        );
    }
}

//! Occupancy likelihood field for scan matching.

use rne_math::Vec3;
use rne_nav::{GridCoord, GridError, OccupancyGrid, Pose2d};
use serde::{Deserialize, Serialize};
use std::f64::consts::SQRT_2;

/// Configuration for building a likelihood field.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct LikelihoodConfig {
    /// Probability at or above which a cell is treated as an obstacle.
    pub occupied_probability: f64,
    /// Gaussian width in meters; larger values give smoother basins.
    pub sigma_m: f64,
    /// Distance in meters beyond which the likelihood is zero.
    pub max_distance_m: f64,
}

impl Default for LikelihoodConfig {
    fn default() -> Self {
        Self {
            occupied_probability: 0.6,
            sigma_m: 0.2,
            max_distance_m: 1.0,
        }
    }
}

/// A precomputed obstacle-distance likelihood over a grid.
///
/// The field stores `exp(-d² / (2σ²))` at every cell, where `d` is the distance
/// to the nearest obstacle cell, clipped to [`LikelihoodConfig::max_distance_m`].
/// Scan endpoints that land on an obstacle score near `1.0`.
#[derive(Clone, Debug, PartialEq)]
pub struct LikelihoodField {
    width: usize,
    height: usize,
    resolution_m: f64,
    origin: Pose2d,
    values: Vec<f64>,
}

impl LikelihoodField {
    /// Builds a likelihood field from an occupancy grid.
    pub fn from_occupancy(
        grid: &OccupancyGrid,
        config: &LikelihoodConfig,
    ) -> Result<Self, GridError> {
        if !config.sigma_m.is_finite()
            || config.sigma_m <= 0.0
            || !config.max_distance_m.is_finite()
            || config.max_distance_m <= 0.0
            || !config.occupied_probability.is_finite()
        {
            return Err(GridError::NonFinite);
        }
        let width = grid.width();
        let height = grid.height();
        let resolution = grid.resolution_m();

        let mut obstacles = vec![false; width * height];
        for y in 0..height {
            for x in 0..width {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                let probability = grid.probability(coord).unwrap_or(0.5);
                obstacles[y * width + x] = probability >= config.occupied_probability;
            }
        }
        let distances = chamfer_distance(&obstacles, width, height);
        let two_sigma_squared = 2.0 * config.sigma_m * config.sigma_m;
        let max_distance = config.max_distance_m;
        let values = distances
            .iter()
            .map(|cells| {
                let distance_m = cells * resolution;
                if distance_m >= max_distance {
                    0.0
                } else {
                    (-(distance_m * distance_m) / two_sigma_squared).exp()
                }
            })
            .collect();

        Ok(Self {
            width,
            height,
            resolution_m: resolution,
            origin: grid.origin(),
            values,
        })
    }

    /// Width in cells.
    pub fn width(&self) -> usize {
        self.width
    }

    /// Height in cells.
    pub fn height(&self) -> usize {
        self.height
    }

    /// Cell size in meters.
    pub fn resolution_m(&self) -> f64 {
        self.resolution_m
    }

    /// Likelihood at a world point in `[0, 1]`; zero when outside the field.
    pub fn score(&self, point_m: Vec3) -> f64 {
        match self.world_to_grid(point_m) {
            Some(coord) => self.values[coord.y as usize * self.width + coord.x as usize],
            None => 0.0,
        }
    }

    /// Projects a world point onto integer cell coordinates, if inside.
    pub fn world_to_grid(&self, point_m: Vec3) -> Option<GridCoord> {
        let local = self.origin.inverse_transform_point(point_m);
        let coord = GridCoord {
            x: (local.x / self.resolution_m).floor() as isize,
            y: (local.y / self.resolution_m).floor() as isize,
        };
        (coord.x >= 0
            && coord.y >= 0
            && (coord.x as usize) < self.width
            && (coord.y as usize) < self.height)
            .then_some(coord)
    }

    /// Returns a coarser field by max-pooling `factor` by `factor` blocks.
    ///
    /// The origin is unchanged; the resolution scales by `factor`. A `factor`
    /// of one returns a clone. This is the coarse level of a multi-resolution
    /// scan match: it widens the basin at each pose candidate.
    pub fn downsample(&self, factor: usize) -> Self {
        let factor = factor.max(1);
        if factor == 1 {
            return self.clone();
        }
        let width = self.width.div_ceil(factor);
        let height = self.height.div_ceil(factor);
        let mut values = vec![0.0_f64; width * height];
        for block_y in 0..height {
            for block_x in 0..width {
                let mut best = 0.0_f64;
                for dy in 0..factor {
                    for dx in 0..factor {
                        let x = block_x * factor + dx;
                        let y = block_y * factor + dy;
                        if x < self.width && y < self.height {
                            best = best.max(self.values[y * self.width + x]);
                        }
                    }
                }
                values[block_y * width + block_x] = best;
            }
        }
        Self {
            width,
            height,
            resolution_m: self.resolution_m * factor as f64,
            origin: self.origin,
            values,
        }
    }
}

fn chamfer_distance(obstacles: &[bool], width: usize, height: usize) -> Vec<f64> {
    let mut distance = vec![f64::INFINITY; obstacles.len()];
    for (cell, is_obstacle) in distance.iter_mut().zip(obstacles) {
        if *is_obstacle {
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

    #[test]
    fn obstacle_cells_score_near_one() {
        let mut grid = OccupancyGrid::new(20, 20, 0.1, Pose2d::IDENTITY).unwrap();
        let obstacle = GridCoord { x: 10, y: 10 };
        grid.mark_occupied(obstacle);
        let field = LikelihoodField::from_occupancy(&grid, &LikelihoodConfig::default()).unwrap();
        let center = grid.grid_to_world(obstacle);
        assert_relative_eq!(field.score(center), 1.0, epsilon = 1e-9);
        let far = grid.grid_to_world(GridCoord { x: 0, y: 0 });
        assert_relative_eq!(field.score(far), 0.0, epsilon = 1e-9);
    }

    #[test]
    fn downsample_scales_resolution_and_keeps_peak() {
        let mut grid = OccupancyGrid::new(20, 20, 0.1, Pose2d::IDENTITY).unwrap();
        let obstacle = GridCoord { x: 5, y: 5 };
        grid.mark_occupied(obstacle);
        let field = LikelihoodField::from_occupancy(&grid, &LikelihoodConfig::default()).unwrap();
        let coarse = field.downsample(2);
        assert_eq!(coarse.width(), 10);
        assert_eq!(coarse.height(), 10);
        assert_relative_eq!(coarse.resolution_m(), 0.2, epsilon = 1e-12);
        // The obstacle peak survives the max-pool at the coarse cell center.
        let center = grid.grid_to_world(obstacle);
        assert!(coarse.score(center) > 0.5, "score={}", coarse.score(center));
    }
}

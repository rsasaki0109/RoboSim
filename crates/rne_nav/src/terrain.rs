//! Terrain-aware cost layering.
//!
//! [`apply_terrain_layer`] folds a 2.5D [`ElevationMap`] into a planar
//! [`Costmap`]. Cells whose intra-cell step reaches `lethal_step_m`, or whose
//! slope reaches `lethal_slope_rad`, become lethal; slopes between
//! `max_traversable_slope_rad` and `lethal_slope_rad` raise a graded cost that
//! the planner's `cost_weight` will prefer to avoid. This is how outdoor and
//! uneven-terrain navigation reuses the 2D planner.

use crate::costmap::{Costmap, COST_LETHAL};
use crate::elevation::ElevationMap;
use crate::grid::{GridCoord, GridError};
use serde::{Deserialize, Serialize};

/// Slope and step thresholds for terrain cost layering.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TerrainConfig {
    /// Slope up to which the terrain is free, in radians.
    pub max_traversable_slope_rad: f64,
    /// Slope at or above which the terrain is lethal, in radians.
    pub lethal_slope_rad: f64,
    /// Intra-cell step at or above which the cell is lethal, in meters.
    pub lethal_step_m: f64,
    /// Maximum graded slope cost added below the lethal slope.
    pub slope_cost: f64,
}

impl Default for TerrainConfig {
    fn default() -> Self {
        Self {
            max_traversable_slope_rad: 0.5,
            lethal_slope_rad: 1.0,
            lethal_step_m: 0.3,
            slope_cost: 200.0,
        }
    }
}

impl TerrainConfig {
    /// Whether every threshold is finite and ordered.
    pub fn is_valid(&self) -> bool {
        self.max_traversable_slope_rad.is_finite()
            && self.max_traversable_slope_rad >= 0.0
            && self.lethal_slope_rad.is_finite()
            && self.lethal_slope_rad >= self.max_traversable_slope_rad
            && self.lethal_step_m.is_finite()
            && self.lethal_step_m > 0.0
            && self.slope_cost.is_finite()
            && self.slope_cost >= 0.0
            && self.slope_cost < f64::from(COST_LETHAL)
    }
}

/// Aggregate result of terrain layering.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TerrainReport {
    /// Cells marked lethal by step or slope.
    pub lethal_cells: usize,
    /// Cells that received a graded slope cost.
    pub cost_cells: usize,
}

/// Folds `elevation` into `costmap` using `config`.
///
/// The two maps must share dimensions, resolution, and origin.
pub fn apply_terrain_layer(
    costmap: &mut Costmap,
    elevation: &ElevationMap,
    config: &TerrainConfig,
) -> Result<TerrainReport, GridError> {
    if !config.is_valid() {
        return Err(GridError::NonFinite);
    }
    if costmap.width() != elevation.width()
        || costmap.height() != elevation.height()
        || (costmap.resolution_m() - elevation.resolution_m()).abs() > f64::EPSILON
    {
        return Err(GridError::InvalidSize);
    }

    let mut report = TerrainReport::default();
    let slope_span =
        (config.lethal_slope_rad - config.max_traversable_slope_rad).max(f64::MIN_POSITIVE);
    for y in 0..costmap.height() {
        for x in 0..costmap.width() {
            let coord = GridCoord {
                x: x as isize,
                y: y as isize,
            };
            let Some(cell) = elevation.cell(coord).filter(|cell| cell.is_known()) else {
                continue;
            };
            if cell.range_m().unwrap_or(f64::INFINITY) >= config.lethal_step_m {
                if costmap.apply_cost(coord, COST_LETHAL) {
                    report.lethal_cells += 1;
                }
                continue;
            }
            let Some(slope) = elevation.slope_at(coord) else {
                continue;
            };
            if slope >= config.lethal_slope_rad {
                if costmap.apply_cost(coord, COST_LETHAL) {
                    report.lethal_cells += 1;
                }
            } else if slope > config.max_traversable_slope_rad {
                let fraction =
                    ((slope - config.max_traversable_slope_rad) / slope_span).clamp(0.0, 1.0);
                let cost = (fraction * config.slope_cost).round();
                let cost = (cost as u8).clamp(1, COST_LETHAL - 1);
                if costmap.apply_cost(coord, cost) {
                    report.cost_cells += 1;
                }
            }
        }
    }
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::CostmapConfig;
    use crate::elevation::ElevationConfig;
    use crate::grid::OccupancyGrid;
    use crate::pose2d::Pose2d;
    use rne_math::Vec3;

    fn maps() -> (Costmap, ElevationMap) {
        let origin = Pose2d::IDENTITY;
        let grid = OccupancyGrid::new(40, 40, 0.1, origin).unwrap();
        let costmap = Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap();
        let elevation = ElevationMap::new(40, 40, 0.1, origin, ElevationConfig::default()).unwrap();
        (costmap, elevation)
    }

    #[test]
    fn steep_slope_becomes_lethal() {
        let (mut costmap, mut elevation) = maps();
        for i in 0..5 {
            let x = 0.05 + i as f64 * 0.1;
            elevation.integrate(&[Vec3::new(x, i as f64 * 0.5, 0.05)]);
        }
        let report =
            apply_terrain_layer(&mut costmap, &elevation, &TerrainConfig::default()).unwrap();
        assert!(report.lethal_cells > 0);
        assert!(costmap.is_lethal(GridCoord { x: 0, y: 0 }));
    }

    #[test]
    fn gentle_slope_adds_graded_cost() {
        let (mut costmap, mut elevation) = maps();
        let config = TerrainConfig {
            lethal_step_m: 10.0,
            ..TerrainConfig::default()
        };
        // A shallow ramp: 0.08 m rise per 0.1 m cell (~0.67 rad, below lethal 1.0).
        for i in 0..6 {
            let x = 0.05 + i as f64 * 0.1;
            elevation.integrate(&[Vec3::new(x, i as f64 * 0.08, 0.05)]);
        }
        let report = apply_terrain_layer(&mut costmap, &elevation, &config).unwrap();
        assert!(report.cost_cells > 0);
        let cost = costmap.cost_at(GridCoord { x: 0, y: 0 }).unwrap();
        assert!(cost > 0 && cost < COST_LETHAL);
    }

    #[test]
    fn flat_terrain_stays_free() {
        let (mut costmap, mut elevation) = maps();
        elevation.integrate(&[Vec3::new(0.05, 0.0, 0.05)]);
        let report =
            apply_terrain_layer(&mut costmap, &elevation, &TerrainConfig::default()).unwrap();
        assert_eq!(report, TerrainReport::default());
        assert!(!costmap.is_lethal(GridCoord { x: 0, y: 0 }));
    }

    #[test]
    fn rejects_mismatched_geometry_and_bad_config() {
        let (mut costmap, elevation) = maps();
        let origin = Pose2d::new(0.0, 0.0, 0.0);
        let small = ElevationMap::new(10, 10, 0.1, origin, ElevationConfig::default()).unwrap();
        assert_eq!(
            apply_terrain_layer(&mut costmap, &small, &TerrainConfig::default()),
            Err(GridError::InvalidSize)
        );
        let bad = TerrainConfig {
            max_traversable_slope_rad: 2.0,
            lethal_slope_rad: 1.0,
            ..TerrainConfig::default()
        };
        assert_eq!(
            apply_terrain_layer(&mut costmap, &elevation, &bad),
            Err(GridError::NonFinite)
        );
    }
}

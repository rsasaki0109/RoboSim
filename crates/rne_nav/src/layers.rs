//! Perception layers for the navigation costmap.
//!
//! Three deterministic layers compose with the occupancy- and terrain-derived
//! costmap:
//!
//! * [`VoxelLayer`] accumulates 3D points into a sparse voxel set and projects
//!   the occupied columns onto a planar [`Costmap`] or occupancy grid. This is
//!   the depth-camera / 3D-LiDAR obstacle layer.
//! * [`KeepoutZone`] marks a rectangular region lethal so the planner avoids it.
//! * [`SpeedFilter`] caps the commanded speed inside slow zones.
//!
//! All storage uses ordered containers and index-ordered iteration, so a point
//! stream replays deterministically.

use crate::costmap::{Costmap, COST_LETHAL};
use crate::grid::{GridCoord, GridError};
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use serde::{Deserialize, Serialize};
use std::collections::BTreeSet;

/// Configuration for a [`VoxelLayer`].
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct VoxelConfig {
    /// Voxel edge length in meters.
    pub voxel_size_m: f64,
    /// Minimum world height in meters; points below are ignored.
    pub min_height_m: f64,
    /// Maximum world height in meters; points above are ignored.
    pub max_height_m: f64,
}

impl Default for VoxelConfig {
    fn default() -> Self {
        Self {
            voxel_size_m: 0.1,
            min_height_m: 0.1,
            max_height_m: 2.0,
        }
    }
}

impl VoxelConfig {
    /// Whether the configuration is finite and well ordered.
    pub fn is_valid(&self) -> bool {
        self.voxel_size_m.is_finite()
            && self.voxel_size_m > 0.0
            && self.min_height_m.is_finite()
            && self.max_height_m.is_finite()
            && self.max_height_m >= self.min_height_m
    }
}

/// A sparse 3D obstacle layer.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct VoxelLayer {
    config: VoxelConfig,
    occupied: BTreeSet<[i32; 3]>,
}

impl VoxelLayer {
    /// Creates an empty layer.
    pub fn new(config: VoxelConfig) -> Result<Self, GridError> {
        if !config.is_valid() {
            return Err(GridError::NonFinite);
        }
        Ok(Self {
            config,
            occupied: BTreeSet::new(),
        })
    }

    /// The configuration.
    pub fn config(&self) -> VoxelConfig {
        self.config
    }

    /// Number of occupied voxels.
    pub fn len(&self) -> usize {
        self.occupied.len()
    }

    /// Whether the layer has no occupied voxels.
    pub fn is_empty(&self) -> bool {
        self.occupied.is_empty()
    }

    /// Marks voxels for finite points inside the height band.
    ///
    /// Returns the number of points that were integrated.
    pub fn integrate(&mut self, points_world_m: &[Vec3]) -> usize {
        let size = self.config.voxel_size_m;
        let mut integrated = 0;
        for point in points_world_m {
            if !point.is_finite()
                || point.y < self.config.min_height_m
                || point.y > self.config.max_height_m
            {
                continue;
            }
            let key = [
                (point.x / size).floor() as i32,
                (point.y / size).floor() as i32,
                (point.z / size).floor() as i32,
            ];
            self.occupied.insert(key);
            integrated += 1;
        }
        integrated
    }

    /// Removes every occupied voxel.
    pub fn clear(&mut self) {
        self.occupied.clear();
    }

    /// Marks the planar [`Costmap`] column under each occupied voxel lethal.
    ///
    /// Voxels project through the navigation plane (`world X-Z` → grid `x, y`),
    /// matching `integrate_point_cloud`.
    pub fn to_costmap_layer(&self, costmap: &mut Costmap) -> usize {
        let mut marked = BTreeSet::new();
        for key in &self.occupied {
            let world = self.voxel_center(*key);
            let Some(coord) = costmap.world_to_grid(Vec3::new(world.x, world.z, 0.0)) else {
                continue;
            };
            if costmap.apply_cost(coord, COST_LETHAL) {
                marked.insert((coord.x, coord.y));
            }
        }
        marked.len()
    }

    fn voxel_center(&self, key: [i32; 3]) -> Vec3 {
        let size = self.config.voxel_size_m;
        Vec3::new(
            (key[0] as f64 + 0.5) * size,
            (key[1] as f64 + 0.5) * size,
            (key[2] as f64 + 0.5) * size,
        )
    }
}

/// A rectangular keepout region in the costmap plane (`world X-Y`).
///
/// Voxel and point-cloud layers project `world (x, z)` onto the map `x, y`, so
/// callers with 3D world points pass `Vec3::new(world.x, world.z, 0.0)`.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct KeepoutZone {
    /// Minimum world x in meters.
    pub min_x_m: f64,
    /// Maximum world x in meters.
    pub max_x_m: f64,
    /// Minimum world z in meters.
    pub min_z_m: f64,
    /// Maximum world z in meters.
    pub max_z_m: f64,
}

impl KeepoutZone {
    /// Whether a map-plane point lies inside the zone.
    pub fn contains(&self, point_m: Vec3) -> bool {
        point_m.x >= self.min_x_m
            && point_m.x <= self.max_x_m
            && point_m.y >= self.min_z_m
            && point_m.y <= self.max_z_m
    }

    /// Marks every costmap cell inside the zone lethal, returning the count.
    pub fn apply(&self, costmap: &mut Costmap) -> usize {
        let mut marked = 0;
        for y in 0..costmap.height() {
            for x in 0..costmap.width() {
                let coord = GridCoord {
                    x: x as isize,
                    y: y as isize,
                };
                let world = costmap.grid_to_world(coord);
                if self.contains(world) && costmap.apply_cost(coord, COST_LETHAL) {
                    marked += 1;
                }
            }
        }
        marked
    }
}

/// A rectangular region with a maximum speed.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct SpeedLimitZone {
    /// The zone boundary.
    pub zone: KeepoutZone,
    /// Maximum speed inside the zone in meters per second.
    pub max_speed_m_s: f64,
}

/// Caps the commanded speed inside slow zones.
#[derive(Clone, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct SpeedFilter {
    /// Outside-zone speed cap in meters per second.
    pub default_max_speed_m_s: f64,
    /// Ordered list of speed-limit zones.
    pub zones: Vec<SpeedLimitZone>,
}

impl SpeedFilter {
    /// Creates a filter with an outside-zone cap and no zones.
    pub fn new(default_max_speed_m_s: f64) -> Self {
        Self {
            default_max_speed_m_s,
            zones: Vec::new(),
        }
    }

    /// Adds a slow zone.
    pub fn add_zone(&mut self, zone: KeepoutZone, max_speed_m_s: f64) {
        self.zones.push(SpeedLimitZone {
            zone,
            max_speed_m_s,
        });
    }

    /// The speed cap for a robot pose.
    pub fn limit_at(&self, pose: Pose2d) -> f64 {
        let point = Vec3::new(pose.x_m, pose.y_m, 0.0);
        let mut limit = self.default_max_speed_m_s;
        for zone in &self.zones {
            if zone.zone.contains(point) {
                limit = limit.min(zone.max_speed_m_s);
            }
        }
        limit
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::costmap::CostmapConfig;
    use crate::grid::OccupancyGrid;
    use approx::assert_relative_eq;

    fn costmap() -> Costmap {
        let grid = OccupancyGrid::new(40, 40, 0.1, Pose2d::IDENTITY).unwrap();
        Costmap::from_occupancy(&grid, &CostmapConfig::default()).unwrap()
    }

    #[test]
    fn voxel_layer_dedupes_and_projects() {
        let mut layer = VoxelLayer::new(VoxelConfig::default()).unwrap();
        let integrated = layer.integrate(&[
            Vec3::new(0.25, 0.5, 0.25),
            Vec3::new(0.26, 0.51, 0.26),
            Vec3::new(0.3, 5.0, 0.3), // above the height band
        ]);
        assert_eq!(integrated, 2);
        assert_eq!(layer.len(), 1);

        let mut map = costmap();
        let marked = layer.to_costmap_layer(&mut map);
        assert_eq!(marked, 1);
        assert!(map.is_lethal(GridCoord { x: 2, y: 2 }));
    }

    #[test]
    fn keepout_zone_marks_cells_lethal() {
        let mut map = costmap();
        let zone = KeepoutZone {
            min_x_m: 0.2,
            max_x_m: 0.5,
            min_z_m: 0.2,
            max_z_m: 0.5,
        };
        assert!(zone.contains(Vec3::new(0.3, 0.3, 0.0)));
        assert!(!zone.contains(Vec3::new(1.0, 0.3, 0.0)));
        let marked = zone.apply(&mut map);
        assert!(marked >= 4);
        assert!(map.is_lethal(GridCoord { x: 2, y: 2 }));
        assert!(!map.is_lethal(GridCoord { x: 10, y: 10 }));
    }

    #[test]
    fn speed_filter_returns_the_tightest_limit() {
        let mut filter = SpeedFilter::new(1.5);
        filter.add_zone(
            KeepoutZone {
                min_x_m: -0.5,
                max_x_m: 0.5,
                min_z_m: -0.5,
                max_z_m: 0.5,
            },
            0.3,
        );
        assert_relative_eq!(
            filter.limit_at(Pose2d::new(0.0, 0.0, 0.0)),
            0.3,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            filter.limit_at(Pose2d::new(5.0, 0.0, 0.0)),
            1.5,
            epsilon = 1e-12
        );
    }

    #[test]
    fn rejects_invalid_voxel_config() {
        let bad = VoxelConfig {
            voxel_size_m: 0.0,
            ..VoxelConfig::default()
        };
        assert_eq!(VoxelLayer::new(bad), Err(GridError::NonFinite));
    }
}

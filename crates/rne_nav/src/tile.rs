//! Lazily allocated tiled occupancy grid for large maps.
//!
//! A [`TiledOccupancyGrid`] stores square [`OccupancyGrid`] tiles keyed by tile
//! index, creating them on demand as updates arrive. World queries route to the
//! owning tile, so a map can grow without allocating the full bounding box.

use crate::grid::{GridCoord, GridError, OccupancyGrid};
use crate::pose2d::Pose2d;
use rne_math::Vec3;
use std::collections::BTreeMap;

/// A sparse grid of fixed-size occupancy tiles.
#[derive(Clone, Debug)]
pub struct TiledOccupancyGrid {
    resolution_m: f64,
    tile_size_cells: usize,
    origin: Pose2d,
    tiles: BTreeMap<(i64, i64), OccupancyGrid>,
}

impl TiledOccupancyGrid {
    /// Creates an empty tiled grid. `origin` is the world pose of cell `(0, 0)`
    /// in tile `(0, 0)`.
    pub fn new(
        resolution_m: f64,
        tile_size_cells: usize,
        origin: Pose2d,
    ) -> Result<Self, GridError> {
        if tile_size_cells == 0 {
            return Err(GridError::InvalidSize);
        }
        // Validate resolution and origin via a throwaway tile.
        OccupancyGrid::new(1, 1, resolution_m, origin)?;
        Ok(Self {
            resolution_m,
            tile_size_cells,
            origin,
            tiles: BTreeMap::new(),
        })
    }

    /// Cell size in meters.
    pub fn resolution_m(&self) -> f64 {
        self.resolution_m
    }

    /// Tile edge length in cells.
    pub fn tile_size_cells(&self) -> usize {
        self.tile_size_cells
    }

    /// Number of allocated tiles.
    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// Tile indices currently allocated, sorted.
    pub fn tile_indices(&self) -> Vec<(i64, i64)> {
        self.tiles.keys().copied().collect()
    }

    /// Applies an occupied update at a world point, allocating a tile if needed.
    pub fn apply_occupied(&mut self, world_m: Vec3, log_odds: f64) -> bool {
        self.apply(world_m, log_odds, true)
    }

    /// Applies a free update at a world point, allocating a tile if needed.
    pub fn apply_free(&mut self, world_m: Vec3, log_odds: f64) -> bool {
        self.apply(world_m, log_odds, false)
    }

    /// Occupancy probability at a world point, if the owning tile exists.
    pub fn probability_at(&self, world_m: Vec3) -> Option<f64> {
        let (tile, coord) = self.locate(world_m);
        self.tiles.get(&tile)?.probability(coord)
    }

    /// Whether a world point is occupied.
    pub fn is_occupied(&self, world_m: Vec3) -> bool {
        self.probability_at(world_m)
            .map(|probability| probability >= crate::grid::DEFAULT_OCCUPIED_PROBABILITY)
            .unwrap_or(false)
    }

    /// World centers of occupied cells across all tiles.
    pub fn occupied_world_cells(&self) -> Vec<Vec3> {
        let mut cells = Vec::new();
        for (tile_index, tile) in &self.tiles {
            for y in 0..tile.height() {
                for x in 0..tile.width() {
                    let coord = GridCoord {
                        x: x as isize,
                        y: y as isize,
                    };
                    if tile.is_occupied(coord) {
                        let local = tile_world_origin(self, *tile_index);
                        let world = self.origin.transform_point(
                            local
                                + Vec3::new(
                                    x as f64 * self.resolution_m,
                                    y as f64 * self.resolution_m,
                                    0.0,
                                ),
                        );
                        cells.push(world);
                    }
                }
            }
        }
        cells
    }

    fn apply(&mut self, world_m: Vec3, log_odds: f64, occupied: bool) -> bool {
        let (tile_index, coord) = self.locate(world_m);
        if !self.tiles.contains_key(&tile_index) {
            let tile = self.make_tile(tile_index);
            self.tiles.insert(tile_index, tile);
        }
        let tile = self.tiles.get_mut(&tile_index).expect("tile just inserted");
        if occupied {
            tile.apply_occupied(coord, log_odds)
        } else {
            tile.apply_free(coord, log_odds)
        }
    }

    fn tile_world_size(&self) -> f64 {
        self.tile_size_cells as f64 * self.resolution_m
    }

    fn locate(&self, world_m: Vec3) -> ((i64, i64), GridCoord) {
        let local = self.origin.inverse_transform_point(world_m);
        let size = self.tile_world_size();
        let tile = (
            (local.x / size).floor() as i64,
            (local.y / size).floor() as i64,
        );
        let within = Vec3::new(
            local.x - tile.0 as f64 * size,
            local.y - tile.1 as f64 * size,
            0.0,
        );
        let coord = GridCoord {
            x: (within.x / self.resolution_m).floor() as isize,
            y: (within.y / self.resolution_m).floor() as isize,
        };
        (tile, coord)
    }

    fn make_tile(&self, tile_index: (i64, i64)) -> OccupancyGrid {
        let origin_local = tile_world_origin(self, tile_index)
            + Vec3::new(self.resolution_m * 0.5, self.resolution_m * 0.5, 0.0);
        let origin_world = self.origin.transform_point(origin_local);
        let origin = Pose2d::from_translation_yaw(origin_world, self.origin.yaw_rad);
        OccupancyGrid::new(
            self.tile_size_cells,
            self.tile_size_cells,
            self.resolution_m,
            origin,
        )
        .expect("tile dimensions were validated")
    }
}

fn tile_world_origin(grid: &TiledOccupancyGrid, tile_index: (i64, i64)) -> Vec3 {
    let size = grid.tile_world_size();
    Vec3::new(tile_index.0 as f64 * size, tile_index.1 as f64 * size, 0.0)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn allocates_tiles_on_demand() {
        let mut grid = TiledOccupancyGrid::new(0.1, 10, Pose2d::IDENTITY).unwrap();
        assert_eq!(grid.tile_count(), 0);
        // Two points 2 m apart land in different 1 m tiles.
        grid.apply_occupied(Vec3::new(0.05, 0.05, 0.0), 0.85);
        grid.apply_occupied(Vec3::new(2.05, 0.05, 0.0), 0.85);
        assert_eq!(grid.tile_count(), 2);
        assert_eq!(grid.tile_indices(), vec![(0, 0), (2, 0)]);
        assert!(grid.is_occupied(Vec3::new(0.05, 0.05, 0.0)));
        assert!(grid.is_occupied(Vec3::new(2.05, 0.05, 0.0)));
        assert!(!grid.is_occupied(Vec3::new(1.05, 0.05, 0.0)));
    }

    #[test]
    fn updates_share_a_tile() {
        let mut grid = TiledOccupancyGrid::new(0.1, 10, Pose2d::IDENTITY).unwrap();
        for x in [0.05, 0.45, 0.85] {
            grid.apply_free(Vec3::new(x, 0.05, 0.0), -0.4);
        }
        assert_eq!(grid.tile_count(), 1);
        assert!(grid.probability_at(Vec3::new(0.45, 0.05, 0.0)).is_some());
    }

    #[test]
    fn rejects_zero_tile_size() {
        assert_eq!(
            TiledOccupancyGrid::new(0.1, 0, Pose2d::IDENTITY).err(),
            Some(GridError::InvalidSize)
        );
    }
}

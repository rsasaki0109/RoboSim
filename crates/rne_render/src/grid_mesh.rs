//! Flat cell meshes for drawing grid-shaped data on the floor.
//!
//! Occupancy grids, costmaps, elevation maps and coverage heatmaps are all the
//! same picture: a regular grid of cells, some of which should be drawn. The
//! engine could render robots and props but had no way to show any of it, so a
//! navigating robot appeared to move for no reason — the map it built, the
//! costs it avoided and the path it chose were all invisible.
//!
//! [`grid_mesh`] closes that gap without teaching the renderer anything about
//! navigation. It takes the grid's dimensions and a predicate over cell indices,
//! and returns the selected cells as upward-facing quads. Colour is not part of
//! the mesh, because a render item carries one colour: a caller draws each band
//! it wants — free, occupied, unknown, or a cost range — as its own mesh and
//! gives each its own colour.
//!
//! Cells lie in the world X/Z plane at a fixed height, matching the engine's
//! Y-up convention, and are emitted in row-major order so the same grid always
//! produces the same mesh.

use crate::mesh::TriangleMesh;
use rne_math::Vec3;

/// Placement and spacing of a grid to be drawn.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct GridMeshSpec {
    /// Number of cells along the world X axis.
    pub columns: usize,
    /// Number of cells along the world Z axis.
    pub rows: usize,
    /// Edge length of one square cell, in meters.
    pub cell_size_m: f64,
    /// World position of the minimum corner of cell `(0, 0)`.
    pub origin_m: Vec3,
    /// Height above `origin_m.y` at which the cells are drawn, in meters.
    ///
    /// A small positive value keeps the overlay from z-fighting with the floor
    /// it describes.
    pub height_m: f64,
    /// Fraction of a cell actually filled, in `(0, 1]`.
    ///
    /// Values below one leave a gap between cells, which reads as a grid rather
    /// than a solid sheet and makes individual cells countable.
    pub fill: f64,
}

impl Default for GridMeshSpec {
    fn default() -> Self {
        Self {
            columns: 0,
            rows: 0,
            cell_size_m: 0.05,
            origin_m: Vec3::ZERO,
            height_m: 0.01,
            fill: 0.9,
        }
    }
}

impl GridMeshSpec {
    /// Returns whether the specification describes a drawable grid.
    pub fn is_valid(&self) -> bool {
        self.columns > 0
            && self.rows > 0
            && self.cell_size_m.is_finite()
            && self.cell_size_m > 0.0
            && self.origin_m.is_finite()
            && self.height_m.is_finite()
            && self.fill.is_finite()
            && self.fill > 0.0
            && self.fill <= 1.0
    }

    /// Returns the world centre of one cell, or `None` when out of range.
    pub fn cell_center_m(&self, column: usize, row: usize) -> Option<Vec3> {
        (column < self.columns && row < self.rows).then(|| {
            Vec3::new(
                self.origin_m.x + (column as f64 + 0.5) * self.cell_size_m,
                self.origin_m.y + self.height_m,
                self.origin_m.z + (row as f64 + 0.5) * self.cell_size_m,
            )
        })
    }
}

/// Builds an upward-facing quad mesh over the cells `selected` accepts.
///
/// `selected` is called once per cell as `(column, row)` in row-major order.
/// An invalid specification, or a selector that accepts nothing, yields an empty
/// mesh rather than an error: an empty map is a normal thing to draw, and a
/// caller that renders it gets nothing on screen instead of a failure.
pub fn grid_mesh(
    spec: &GridMeshSpec,
    mut selected: impl FnMut(usize, usize) -> bool,
) -> TriangleMesh {
    let mut mesh = TriangleMesh {
        positions: Vec::new(),
        normals: Vec::new(),
        texcoords: Vec::new(),
        indices: Vec::new(),
        skinning: None,
    };
    if !spec.is_valid() {
        return mesh;
    }
    let inset = spec.cell_size_m * (1.0 - spec.fill) * 0.5;
    let y = (spec.origin_m.y + spec.height_m) as f32;

    for row in 0..spec.rows {
        for column in 0..spec.columns {
            if !selected(column, row) {
                continue;
            }
            let min_x = spec.origin_m.x + column as f64 * spec.cell_size_m + inset;
            let max_x = spec.origin_m.x + (column + 1) as f64 * spec.cell_size_m - inset;
            let min_z = spec.origin_m.z + row as f64 * spec.cell_size_m + inset;
            let max_z = spec.origin_m.z + (row + 1) as f64 * spec.cell_size_m - inset;

            let base = mesh.positions.len() as u32;
            for (x, z) in [
                (min_x, min_z),
                (max_x, min_z),
                (max_x, max_z),
                (min_x, max_z),
            ] {
                mesh.positions.push([x as f32, y, z as f32]);
                mesh.normals.push([0.0, 1.0, 0.0]);
                mesh.texcoords.push([0.0, 0.0]);
            }
            // Counter-clockwise seen from above, so the quad faces up.
            mesh.indices
                .extend_from_slice(&[base, base + 3, base + 2, base, base + 2, base + 1]);
        }
    }
    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    fn spec(columns: usize, rows: usize) -> GridMeshSpec {
        GridMeshSpec {
            columns,
            rows,
            cell_size_m: 1.0,
            origin_m: Vec3::new(0.0, 0.0, 0.0),
            height_m: 0.0,
            fill: 1.0,
        }
    }

    #[test]
    fn only_selected_cells_become_quads_and_the_order_is_row_major() {
        let mut visited = Vec::new();
        let mesh = grid_mesh(&spec(3, 2), |column, row| {
            visited.push((column, row));
            column == 1
        });
        assert_eq!(
            visited,
            vec![(0, 0), (1, 0), (2, 0), (0, 1), (1, 1), (2, 1)],
            "cells must be offered row-major so the mesh is reproducible"
        );
        // One quad per selected cell: four vertices, two triangles.
        assert_eq!(mesh.positions.len(), 8);
        assert_eq!(mesh.triangle_count(), 4);

        // Selecting nothing is an empty mesh, not an error.
        assert_eq!(grid_mesh(&spec(3, 2), |_, _| false).positions.len(), 0);
    }

    #[test]
    fn quads_sit_flat_at_the_requested_height_and_face_up() {
        let placed = GridMeshSpec {
            origin_m: Vec3::new(10.0, 2.0, -4.0),
            height_m: 0.02,
            ..spec(1, 1)
        };
        let mesh = grid_mesh(&placed, |_, _| true);
        assert!(mesh
            .positions
            .iter()
            .all(|position| (position[1] - 2.02).abs() < 1.0e-6));
        assert!(mesh.normals.iter().all(|normal| *normal == [0.0, 1.0, 0.0]));

        // The single cell spans one cell size from the origin corner.
        let xs: Vec<f32> = mesh.positions.iter().map(|p| p[0]).collect();
        let zs: Vec<f32> = mesh.positions.iter().map(|p| p[2]).collect();
        assert!((xs.iter().cloned().fold(f32::MAX, f32::min) - 10.0).abs() < 1.0e-6);
        assert!((xs.iter().cloned().fold(f32::MIN, f32::max) - 11.0).abs() < 1.0e-6);
        assert!((zs.iter().cloned().fold(f32::MAX, f32::min) + 4.0).abs() < 1.0e-6);
        assert!((zs.iter().cloned().fold(f32::MIN, f32::max) + 3.0).abs() < 1.0e-6);

        // Winding is counter-clockwise from above, so the face is not culled.
        let index = |i: usize| mesh.positions[mesh.indices[i] as usize];
        let (a, b, c) = (index(0), index(1), index(2));
        let edge1 = [b[0] - a[0], b[2] - a[2]];
        let edge2 = [c[0] - a[0], c[2] - a[2]];
        let cross = edge1[0] * edge2[1] - edge1[1] * edge2[0];
        assert!(
            cross < 0.0,
            "expected an upward-facing winding, got {cross}"
        );
    }

    #[test]
    fn fill_insets_each_cell_without_moving_its_centre() {
        let gapped = GridMeshSpec {
            fill: 0.5,
            ..spec(1, 1)
        };
        let mesh = grid_mesh(&gapped, |_, _| true);
        let xs: Vec<f32> = mesh.positions.iter().map(|p| p[0]).collect();
        let min = xs.iter().cloned().fold(f32::MAX, f32::min);
        let max = xs.iter().cloned().fold(f32::MIN, f32::max);
        assert!((max - min - 0.5).abs() < 1.0e-6, "half-filled cell");
        assert!(
            ((min + max) / 2.0 - 0.5).abs() < 1.0e-6,
            "centred on the cell"
        );
    }

    #[test]
    fn a_degenerate_specification_draws_nothing_rather_than_failing() {
        for broken in [
            GridMeshSpec {
                columns: 0,
                ..spec(1, 1)
            },
            GridMeshSpec {
                rows: 0,
                ..spec(1, 1)
            },
            GridMeshSpec {
                cell_size_m: 0.0,
                ..spec(1, 1)
            },
            GridMeshSpec {
                cell_size_m: f64::NAN,
                ..spec(1, 1)
            },
            GridMeshSpec {
                fill: 0.0,
                ..spec(1, 1)
            },
            GridMeshSpec {
                fill: 1.5,
                ..spec(1, 1)
            },
            GridMeshSpec {
                height_m: f64::INFINITY,
                ..spec(1, 1)
            },
            GridMeshSpec {
                origin_m: Vec3::new(f64::NAN, 0.0, 0.0),
                ..spec(1, 1)
            },
        ] {
            assert!(!broken.is_valid());
            assert_eq!(grid_mesh(&broken, |_, _| true).positions.len(), 0);
        }
    }

    #[test]
    fn cell_centres_are_reported_for_placing_labels_and_markers() {
        let placed = GridMeshSpec {
            origin_m: Vec3::new(1.0, 0.5, 2.0),
            height_m: 0.01,
            cell_size_m: 0.25,
            ..spec(4, 4)
        };
        assert_eq!(
            placed.cell_center_m(0, 0),
            Some(Vec3::new(1.125, 0.51, 2.125))
        );
        assert_eq!(
            placed.cell_center_m(3, 3),
            Some(Vec3::new(1.875, 0.51, 2.875))
        );
        assert_eq!(placed.cell_center_m(4, 0), None);
        assert_eq!(placed.cell_center_m(0, 4), None);
    }
}

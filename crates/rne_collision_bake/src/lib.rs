//! Deterministic offline approximate convex decomposition for RNE colliders.
//!
//! This crate is an offline authoring tool. It converts a triangle mesh into a
//! [`ColliderShape::Compound`] of axis-aligned boxes using a deterministic
//! voxel-and-merge decomposition. The result is serialized as an
//! `.rne.collision.json` sidecar that importers load at runtime; no runtime
//! crate depends on this decomposition.
//!
//! The decomposition is intentionally simple and dependency-free so it stays
//! bit-for-bit reproducible. A higher-quality convex-hull backend (for example
//! CoACD) can be added behind a feature without changing the artifact format.

#![deny(missing_docs)]

use rne_math::{Quat, Vec3};
use rne_physics::{ColliderShape, CompoundPart};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};
use std::path::Path;
use std::sync::Arc;

/// Stable kind identifier for baked collision artifacts.
pub const COLLISION_BAKE_KIND: &str = "rne_collision_bake";

/// Baked collision artifact schema version.
pub const COLLISION_BAKE_SCHEMA_VERSION: u16 = 1;

/// Largest grid extent accepted per axis, protecting the offline tool.
pub const MAX_CELLS_PER_AXIS_LIMIT: u32 = 512;

/// Largest voxel count accepted before decomposition is rejected.
pub const MAX_VOXELS: usize = 4_000_000;

/// File suffix for collision sidecars written next to a source mesh.
pub const COLLISION_SIDECAR_SUFFIX: &str = ".collision.json";

/// Voxel decomposition configuration.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VoxelBakeConfig {
    /// Maximum voxel count along the mesh's longest axis (clamped to `1..=512`).
    pub max_cells_per_axis: u32,
    /// Maximum number of merged boxes before decomposition fails.
    pub max_parts: u32,
}

impl Default for VoxelBakeConfig {
    fn default() -> Self {
        Self {
            max_cells_per_axis: 16,
            max_parts: 2048,
        }
    }
}

/// A baked collision artifact.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct CollisionBake {
    /// Stable artifact discriminator.
    pub kind: String,
    /// Artifact schema version.
    pub schema_version: u16,
    /// Number of source triangles consumed by the decomposition.
    pub source_triangle_count: u64,
    /// Configuration used to produce the artifact.
    pub config: VoxelBakeConfig,
    /// Number of convex parts in `shape`.
    pub part_count: u32,
    /// Baked collider shape in the source mesh's local frame.
    pub shape: ColliderShape,
}

/// Collision bake failure.
#[derive(Debug, thiserror::Error)]
pub enum CollisionBakeError {
    /// The mesh has no vertices.
    #[error("collision bake requires a non-empty vertex list")]
    EmptyMesh,
    /// The index list is empty or not a flat multiple of three.
    #[error("collision bake requires a non-empty flat triangle index list")]
    InvalidIndices,
    /// An index pointed outside the vertex list.
    #[error("triangle index {index} is out of range for {vertices} vertices")]
    IndexOutOfRange {
        /// Offending index.
        index: u32,
        /// Number of available vertices.
        vertices: usize,
    },
    /// A vertex coordinate was not finite.
    #[error("mesh vertex {index} is not finite")]
    NonFiniteVertex {
        /// Offending vertex index.
        index: usize,
    },
    /// The mesh bounding box was degenerate.
    #[error("mesh bounding box is degenerate")]
    DegenerateExtent,
    /// The requested grid exceeded the voxel budget.
    #[error("voxel grid of {voxels} cells exceeds the {limit} cell budget")]
    ResolutionTooFine {
        /// Requested voxel count.
        voxels: usize,
        /// Configured budget.
        limit: usize,
    },
    /// The merge produced more parts than configured.
    #[error("decomposition produced {parts} parts, exceeding the {limit} part budget")]
    TooManyParts {
        /// Produced part count.
        parts: usize,
        /// Configured budget.
        limit: u32,
    },
    /// Deserializing or validating an artifact failed.
    #[error("invalid collision bake artifact: {0}")]
    InvalidArtifact(String),
    /// Reading or writing an artifact failed.
    #[error("collision bake I/O failed: {0}")]
    Io(#[from] std::io::Error),
    /// JSON encoding or decoding failed.
    #[error("collision bake JSON failed: {0}")]
    Json(#[from] serde_json::Error),
}

/// Decomposes a triangle mesh into a deterministic compound of axis-aligned boxes.
///
/// Vertices are in the mesh's local frame. The output shape uses the same frame,
/// so callers apply their own placement separately.
pub fn bake_voxel_decomposition(
    positions: &[[f32; 3]],
    indices: &[u32],
    config: VoxelBakeConfig,
) -> Result<CollisionBake, CollisionBakeError> {
    if positions.is_empty() {
        return Err(CollisionBakeError::EmptyMesh);
    }
    if indices.is_empty() || !indices.len().is_multiple_of(3) {
        return Err(CollisionBakeError::InvalidIndices);
    }

    let mut min = Vec3::splat(f64::INFINITY);
    let mut max = Vec3::splat(f64::NEG_INFINITY);
    for (index, position) in positions.iter().enumerate() {
        let vertex = Vec3::new(
            f64::from(position[0]),
            f64::from(position[1]),
            f64::from(position[2]),
        );
        if !vertex.is_finite() {
            return Err(CollisionBakeError::NonFiniteVertex { index });
        }
        min = min.min(vertex);
        max = max.max(vertex);
    }
    let extent = max - min;
    let longest_m = extent.max_element();
    if longest_m <= 0.0 {
        return Err(CollisionBakeError::DegenerateExtent);
    }

    let cells_per_axis = config.max_cells_per_axis.clamp(1, MAX_CELLS_PER_AXIS_LIMIT);
    let cell_m = longest_m / f64::from(cells_per_axis);
    let dims = [
        (((extent.x / cell_m).ceil() as usize).max(1)).min(MAX_CELLS_PER_AXIS_LIMIT as usize),
        (((extent.y / cell_m).ceil() as usize).max(1)).min(MAX_CELLS_PER_AXIS_LIMIT as usize),
        (((extent.z / cell_m).ceil() as usize).max(1)).min(MAX_CELLS_PER_AXIS_LIMIT as usize),
    ];
    let voxel_count = dims[0] * dims[1] * dims[2];
    if voxel_count > MAX_VOXELS {
        return Err(CollisionBakeError::ResolutionTooFine {
            voxels: voxel_count,
            limit: MAX_VOXELS,
        });
    }

    let stride_y = dims[0];
    let stride_z = dims[0] * dims[1];
    let index_of = |x: usize, y: usize, z: usize| z * stride_z + y * stride_y + x;
    let cell_axis = |value: f64, dim: usize| -> usize {
        let raw = (value / cell_m).floor();
        if raw <= 0.0 {
            0
        } else {
            (raw as usize).min(dim.saturating_sub(1))
        }
    };

    let mut occupied = vec![false; voxel_count];
    for triangle in indices.chunks_exact(3) {
        let mut triangle_min = Vec3::splat(f64::INFINITY);
        let mut triangle_max = Vec3::splat(f64::NEG_INFINITY);
        for corner in triangle {
            let index = *corner as usize;
            let position = positions
                .get(index)
                .ok_or(CollisionBakeError::IndexOutOfRange {
                    index: *corner,
                    vertices: positions.len(),
                })?;
            let vertex = Vec3::new(
                f64::from(position[0]),
                f64::from(position[1]),
                f64::from(position[2]),
            );
            triangle_min = triangle_min.min(vertex);
            triangle_max = triangle_max.max(vertex);
        }
        let low = [
            cell_axis(triangle_min.x - min.x, dims[0]),
            cell_axis(triangle_min.y - min.y, dims[1]),
            cell_axis(triangle_min.z - min.z, dims[2]),
        ];
        let high = [
            cell_axis(triangle_max.x - min.x, dims[0]),
            cell_axis(triangle_max.y - min.y, dims[1]),
            cell_axis(triangle_max.z - min.z, dims[2]),
        ];
        for z in low[2]..=high[2] {
            for y in low[1]..=high[1] {
                for x in low[0]..=high[0] {
                    let center = min
                        + Vec3::new(
                            (x as f64 + 0.5) * cell_m,
                            (y as f64 + 0.5) * cell_m,
                            (z as f64 + 0.5) * cell_m,
                        );
                    let corners = [
                        vertex_at(positions, triangle[0])?,
                        vertex_at(positions, triangle[1])?,
                        vertex_at(positions, triangle[2])?,
                    ];
                    if triangle_box_overlap(
                        corners[0],
                        corners[1],
                        corners[2],
                        center,
                        Vec3::splat(cell_m * 0.5),
                    ) {
                        occupied[index_of(x, y, z)] = true;
                    }
                }
            }
        }
    }

    let mut visited = vec![false; voxel_count];
    let mut parts: Vec<CompoundPart> = Vec::new();
    for z in 0..dims[2] {
        for y in 0..dims[1] {
            for x in 0..dims[0] {
                if !occupied[index_of(x, y, z)] || visited[index_of(x, y, z)] {
                    continue;
                }
                let mut end_x = x;
                while end_x + 1 < dims[0]
                    && occupied[index_of(end_x + 1, y, z)]
                    && !visited[index_of(end_x + 1, y, z)]
                {
                    end_x += 1;
                }
                let mut end_z = z;
                while end_z + 1 < dims[2]
                    && row_free(&occupied, &visited, dims, x, end_x, y, end_z + 1)
                {
                    end_z += 1;
                }
                let mut end_y = y;
                while end_y + 1 < dims[1]
                    && slab_free(&occupied, &visited, dims, x, end_x, z, end_z, end_y + 1)
                {
                    end_y += 1;
                }
                for sz in z..=end_z {
                    for sy in y..=end_y {
                        for sx in x..=end_x {
                            visited[index_of(sx, sy, sz)] = true;
                        }
                    }
                }
                let counts = [
                    (end_x - x + 1) as f64,
                    (end_y - y + 1) as f64,
                    (end_z - z + 1) as f64,
                ];
                let half_extents_m = Vec3::new(
                    counts[0] * cell_m * 0.5,
                    counts[1] * cell_m * 0.5,
                    counts[2] * cell_m * 0.5,
                );
                let center_m = min
                    + Vec3::new(
                        (x as f64 + counts[0] * 0.5) * cell_m,
                        (y as f64 + counts[1] * 0.5) * cell_m,
                        (z as f64 + counts[2] * 0.5) * cell_m,
                    );
                parts.push(CompoundPart {
                    shape: ColliderShape::Cuboid { half_extents_m },
                    local_offset: Transform3::from_translation_rotation(center_m, Quat::IDENTITY),
                });
                if parts.len() as u64 > u64::from(config.max_parts) {
                    return Err(CollisionBakeError::TooManyParts {
                        parts: parts.len(),
                        limit: config.max_parts,
                    });
                }
            }
        }
    }

    let part_count = parts.len() as u32;
    Ok(CollisionBake {
        kind: COLLISION_BAKE_KIND.to_string(),
        schema_version: COLLISION_BAKE_SCHEMA_VERSION,
        source_triangle_count: (indices.len() / 3) as u64,
        config,
        part_count,
        shape: ColliderShape::Compound {
            parts: Arc::from(parts),
        },
    })
}

fn row_free(
    occupied: &[bool],
    visited: &[bool],
    dims: [usize; 3],
    x_start: usize,
    x_end: usize,
    y: usize,
    z: usize,
) -> bool {
    let stride_y = dims[0];
    let stride_z = dims[0] * dims[1];
    (x_start..=x_end).all(|x| {
        let index = z * stride_z + y * stride_y + x;
        occupied[index] && !visited[index]
    })
}

#[allow(clippy::too_many_arguments)]
fn slab_free(
    occupied: &[bool],
    visited: &[bool],
    dims: [usize; 3],
    x_start: usize,
    x_end: usize,
    z_start: usize,
    z_end: usize,
    y: usize,
) -> bool {
    (z_start..=z_end).all(|z| row_free(occupied, visited, dims, x_start, x_end, y, z))
}

fn vertex_at(positions: &[[f32; 3]], index: u32) -> Result<Vec3, CollisionBakeError> {
    let position = positions
        .get(index as usize)
        .ok_or(CollisionBakeError::IndexOutOfRange {
            index,
            vertices: positions.len(),
        })?;
    Ok(Vec3::new(
        f64::from(position[0]),
        f64::from(position[1]),
        f64::from(position[2]),
    ))
}

/// Separating-axis test between a triangle and an axis-aligned box.
fn triangle_box_overlap(v0: Vec3, v1: Vec3, v2: Vec3, center: Vec3, half: Vec3) -> bool {
    let a = v0 - center;
    let b = v1 - center;
    let c = v2 - center;

    let min = a.min(b).min(c);
    let max = a.max(b).max(c);
    if min.x > half.x
        || max.x < -half.x
        || min.y > half.y
        || max.y < -half.y
        || min.z > half.z
        || max.z < -half.z
    {
        return false;
    }

    let normal = (b - a).cross(c - a);
    let distance = normal.dot(a);
    let radius = half.x * normal.x.abs() + half.y * normal.y.abs() + half.z * normal.z.abs();
    if distance.abs() > radius {
        return false;
    }

    let edges = [b - a, c - b, a - c];
    for edge in edges {
        for axis in [Vec3::X, Vec3::Y, Vec3::Z] {
            let test = edge.cross(axis);
            let projection_min = test.dot(a).min(test.dot(b)).min(test.dot(c));
            let projection_max = test.dot(a).max(test.dot(b)).max(test.dot(c));
            let projected_radius =
                half.x * test.x.abs() + half.y * test.y.abs() + half.z * test.z.abs();
            if projection_min > projected_radius || projection_max < -projected_radius {
                return false;
            }
        }
    }
    true
}

impl CollisionBake {
    /// Validates artifact shape and metadata invariants.
    pub fn validate(&self) -> Result<(), CollisionBakeError> {
        if self.kind != COLLISION_BAKE_KIND {
            return Err(CollisionBakeError::InvalidArtifact("kind mismatch".into()));
        }
        if self.schema_version != COLLISION_BAKE_SCHEMA_VERSION {
            return Err(CollisionBakeError::InvalidArtifact(
                "schema version mismatch".into(),
            ));
        }
        let part_count = match &self.shape {
            ColliderShape::Compound { parts } => parts.len(),
            _ => {
                return Err(CollisionBakeError::InvalidArtifact(
                    "baked shape must be a compound".into(),
                ))
            }
        };
        if part_count == 0 {
            return Err(CollisionBakeError::InvalidArtifact(
                "baked shape has no parts".into(),
            ));
        }
        if part_count as u32 != self.part_count {
            return Err(CollisionBakeError::InvalidArtifact(
                "part count mismatch".into(),
            ));
        }
        Ok(())
    }
}

/// Serializes a bake as stable pretty JSON with a trailing newline.
pub fn bake_to_json(bake: &CollisionBake) -> Result<String, CollisionBakeError> {
    bake.validate()?;
    let mut json = serde_json::to_string_pretty(bake)?;
    json.push('\n');
    Ok(json)
}

/// Writes a bake artifact to `path`.
pub fn save_bake(path: &Path, bake: &CollisionBake) -> Result<(), CollisionBakeError> {
    std::fs::write(path, bake_to_json(bake)?)?;
    Ok(())
}

/// Reads and validates a bake artifact from `path`.
pub fn load_bake(path: &Path) -> Result<CollisionBake, CollisionBakeError> {
    let bytes = std::fs::read(path)?;
    let bake: CollisionBake = serde_json::from_slice(&bytes)?;
    bake.validate()?;
    Ok(bake)
}

/// Returns the collision sidecar path for a source mesh path.
pub fn sidecar_path(mesh_path: &Path) -> std::path::PathBuf {
    let mut path = mesh_path.as_os_str().to_os_string();
    path.push(COLLISION_SIDECAR_SUFFIX);
    std::path::PathBuf::from(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn unit_cube() -> (Vec<[f32; 3]>, Vec<u32>) {
        let positions = vec![
            [0.0, 0.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0],
            [1.0, 1.0, 0.0],
            [1.0, 0.0, 1.0],
            [0.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
        ];
        // 12 triangles over the cube surface.
        let indices = vec![
            0, 2, 1, 1, 2, 3, 4, 5, 6, 5, 7, 6, 0, 1, 4, 1, 5, 4, 2, 6, 3, 3, 6, 7, 0, 4, 2, 2, 4,
            6, 1, 3, 5, 3, 7, 5,
        ];
        (positions, indices)
    }

    #[test]
    fn cube_bakes_to_parts_within_bounds() {
        let (positions, indices) = unit_cube();
        let bake = bake_voxel_decomposition(&positions, &indices, VoxelBakeConfig::default())
            .expect("bake");
        bake.validate().expect("valid");
        assert!(bake.part_count >= 1);
        assert_eq!(bake.source_triangle_count, 12);
        // Every part must stay inside the unit cube.
        if let ColliderShape::Compound { parts } = &bake.shape {
            for part in parts.iter() {
                if let ColliderShape::Cuboid { half_extents_m } = part.shape {
                    let center = part.local_offset.translation;
                    assert!(center.x - half_extents_m.x >= -1e-9);
                    assert!(center.x + half_extents_m.x <= 1.0 + 1e-9);
                    assert!(center.y - half_extents_m.y >= -1e-9);
                    assert!(center.y + half_extents_m.y <= 1.0 + 1e-9);
                    assert!(center.z - half_extents_m.z >= -1e-9);
                    assert!(center.z + half_extents_m.z <= 1.0 + 1e-9);
                } else {
                    panic!("voxel decomposition must emit cuboids");
                }
            }
        } else {
            panic!("bake must be a compound");
        }
    }

    #[test]
    fn bake_is_deterministic() {
        let (positions, indices) = unit_cube();
        let config = VoxelBakeConfig {
            max_cells_per_axis: 8,
            max_parts: 512,
        };
        let first = bake_voxel_decomposition(&positions, &indices, config).expect("first");
        let second = bake_voxel_decomposition(&positions, &indices, config).expect("second");
        assert_eq!(first, second);
        assert_eq!(
            bake_to_json(&first).unwrap(),
            bake_to_json(&second).unwrap()
        );
    }

    #[test]
    fn concave_l_prism_splits_into_multiple_parts() {
        // L-shaped polygon extruded in Z. The notch must block a single box.
        let positions = vec![
            [0.0, 0.0, 0.0],
            [2.0, 0.0, 0.0],
            [2.0, 1.0, 0.0],
            [1.0, 1.0, 0.0],
            [1.0, 2.0, 0.0],
            [0.0, 2.0, 0.0],
            [0.0, 0.0, 1.0],
            [2.0, 0.0, 1.0],
            [2.0, 1.0, 1.0],
            [1.0, 1.0, 1.0],
            [1.0, 2.0, 1.0],
            [0.0, 2.0, 1.0],
        ];
        let indices = vec![
            0, 1, 2, 0, 2, 3, 0, 3, 4, 0, 4, 5, // bottom
            6, 8, 7, 6, 9, 8, 6, 10, 9, 6, 11, 10, // top
            0, 1, 7, 0, 7, 6, 1, 2, 8, 1, 8, 7, 2, 3, 9, 2, 9, 8, // sides
            3, 4, 10, 3, 10, 9, 4, 5, 11, 4, 11, 10, 5, 0, 6, 5, 6, 11,
        ];
        let bake = bake_voxel_decomposition(&positions, &indices, VoxelBakeConfig::default())
            .expect("bake");
        assert!(
            bake.part_count >= 2,
            "concave mesh should need more than one box, got {}",
            bake.part_count
        );
    }

    #[test]
    fn malformed_inputs_are_rejected() {
        assert!(matches!(
            bake_voxel_decomposition(&[], &[0, 1, 2], VoxelBakeConfig::default()),
            Err(CollisionBakeError::EmptyMesh)
        ));
        assert!(matches!(
            bake_voxel_decomposition(&[[0.0, 0.0, 0.0]], &[0, 1], VoxelBakeConfig::default()),
            Err(CollisionBakeError::InvalidIndices)
        ));
        assert!(matches!(
            bake_voxel_decomposition(
                &[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]],
                &[0, 1, 9],
                VoxelBakeConfig::default()
            ),
            Err(CollisionBakeError::IndexOutOfRange { .. })
        ));
    }
}

//! Convert parsed URDF geometry into RNE components.

use crate::parse::rpy_to_quat;
use crate::schema::{UrdfGeometry, UrdfGeometryElement, UrdfLink};
use rne_math::{Quat, Vec3};
use rne_physics::{Collider, ColliderShape};
use rne_render::{load_stl, resolve_package_uri, TriangleMesh, Visual, VisualShape};
use rne_world::Transform3;
use std::path::Path;

/// Returns the extra rotation needed when mapping a URDF Z cylinder to a Y capsule.
pub fn cylinder_collider_rotation() -> Quat {
    Quat::from_rotation_x(std::f64::consts::FRAC_PI_2)
}

/// Converts a URDF geometry element into a collider component.
///
/// Mesh collision elements require `assets_root` to resolve and load STL files. When
/// `assets_root` is `None`, mesh collisions return `None` (preserving legacy behavior).
pub fn collider_from_element(
    element: &UrdfGeometryElement,
    assets_root: Option<&Path>,
) -> Option<Collider> {
    let mut local_offset = local_offset_from_element(element);
    let shape = match &element.geometry {
        UrdfGeometry::Box { size_m } => ColliderShape::Cuboid {
            half_extents_m: *size_m * 0.5,
        },
        UrdfGeometry::Sphere { radius_m } => ColliderShape::Sphere {
            radius_m: *radius_m,
        },
        UrdfGeometry::Cylinder { radius_m, length_m } => {
            local_offset.rotation *= cylinder_collider_rotation();
            cylinder_to_capsule(*radius_m, *length_m)
        }
        UrdfGeometry::Mesh { path, scale } => {
            let root = assets_root?;
            let (center_m, half_extents_m) = mesh_aabb_collider(path, *scale, root)?;
            local_offset.translation += center_m;
            ColliderShape::Cuboid { half_extents_m }
        }
    };

    Some(Collider {
        shape,
        material: Default::default(),
        local_offset,
    })
}

/// Builds a single collider for a link by merging all collision-element shapes.
///
/// Multiple primitives or mesh AABBs are unioned into one axis-aligned cuboid.
pub fn collider_from_link(link: &UrdfLink, assets_root: Option<&Path>) -> Option<Collider> {
    if link.collisions.is_empty() {
        return None;
    }
    if link.collisions.len() == 1 {
        return collider_from_element(&link.collisions[0], assets_root);
    }

    let mut bounds: Option<(Vec3, Vec3)> = None;
    let mut material = Default::default();

    for element in &link.collisions {
        let collider = collider_from_element(element, assets_root)?;
        material = collider.material;
        let (min, max) = collider_local_aabb(&collider)?;
        bounds = Some(match bounds {
            Some((bmin, bmax)) => (bmin.min(min), bmax.max(max)),
            None => (min, max),
        });
    }

    let (min, max) = bounds?;
    let center_m = (min + max) * 0.5;
    let half_extents_m = (max - min) * 0.5;
    Some(Collider {
        shape: ColliderShape::Cuboid { half_extents_m },
        material,
        local_offset: Transform3::from_translation_rotation(center_m, Quat::IDENTITY),
    })
}

/// Converts a URDF geometry element into a visual component.
pub fn visual_from_element(element: &UrdfGeometryElement, color_rgba: [f32; 4]) -> Visual {
    Visual {
        shape: visual_shape_from_geometry(&element.geometry),
        local_offset: local_offset_from_element(element),
        color_rgba: element.material_rgba.unwrap_or(color_rgba),
    }
}

/// Computes an axis-aligned bounding box for an STL mesh in the element's local frame.
pub fn mesh_aabb_collider(uri: &str, scale: Vec3, assets_root: &Path) -> Option<(Vec3, Vec3)> {
    let path = resolve_package_uri(uri, assets_root);
    let mesh = load_stl(&path).ok()?;
    mesh_aabb(&mesh, scale)
}

/// Returns the AABB center and positive half-extents for a scaled mesh.
pub fn mesh_aabb(mesh: &TriangleMesh, scale: Vec3) -> Option<(Vec3, Vec3)> {
    if mesh.positions.is_empty() {
        return None;
    }

    let mut min = Vec3::splat(f64::INFINITY);
    let mut max = Vec3::splat(f64::NEG_INFINITY);
    for position in &mesh.positions {
        let scaled = Vec3::new(
            f64::from(position[0]) * scale.x,
            f64::from(position[1]) * scale.y,
            f64::from(position[2]) * scale.z,
        );
        min = min.min(scaled);
        max = max.max(scaled);
    }

    let center_m = (min + max) * 0.5;
    let half_extents_m = (max - min) * 0.5;
    if !half_extents_m.x.is_finite() || half_extents_m.x <= 0.0 {
        return None;
    }
    Some((center_m, half_extents_m))
}

fn collider_local_aabb(collider: &Collider) -> Option<(Vec3, Vec3)> {
    let offset = collider.local_offset.translation;
    let (half, extra_rotation) = match collider.shape {
        ColliderShape::Cuboid { half_extents_m } => (half_extents_m, Quat::IDENTITY),
        ColliderShape::Sphere { radius_m } => {
            let r = Vec3::splat(radius_m);
            (r, Quat::IDENTITY)
        }
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => {
            let y = half_height_m + radius_m;
            (Vec3::new(radius_m, y, radius_m), Quat::IDENTITY)
        }
        ColliderShape::Plane { .. } => return None,
    };

    let rotated_half =
        rotate_aabb_half_extents(extra_rotation * collider.local_offset.rotation, half);
    let min = offset - rotated_half;
    let max = offset + rotated_half;
    Some((min, max))
}

fn rotate_aabb_half_extents(rotation: Quat, half_extents_m: Vec3) -> Vec3 {
    let corners = [
        Vec3::new(half_extents_m.x, half_extents_m.y, half_extents_m.z),
        Vec3::new(half_extents_m.x, half_extents_m.y, -half_extents_m.z),
        Vec3::new(half_extents_m.x, -half_extents_m.y, half_extents_m.z),
        Vec3::new(half_extents_m.x, -half_extents_m.y, -half_extents_m.z),
        Vec3::new(-half_extents_m.x, half_extents_m.y, half_extents_m.z),
        Vec3::new(-half_extents_m.x, half_extents_m.y, -half_extents_m.z),
        Vec3::new(-half_extents_m.x, -half_extents_m.y, half_extents_m.z),
        Vec3::new(-half_extents_m.x, -half_extents_m.y, -half_extents_m.z),
    ];
    let mut min = Vec3::splat(f64::INFINITY);
    let mut max = Vec3::splat(f64::NEG_INFINITY);
    for corner in corners {
        let rotated = rotation * corner;
        min = min.min(rotated);
        max = max.max(rotated);
    }
    (max - min) * 0.5
}

fn local_offset_from_element(element: &UrdfGeometryElement) -> Transform3 {
    Transform3::from_translation_rotation(element.origin_xyz, rpy_to_quat(element.origin_rpy))
}

fn visual_shape_from_geometry(geometry: &UrdfGeometry) -> VisualShape {
    match geometry {
        UrdfGeometry::Box { size_m } => VisualShape::Box { size_m: *size_m },
        UrdfGeometry::Sphere { radius_m } => VisualShape::Sphere {
            radius_m: *radius_m,
        },
        UrdfGeometry::Cylinder { radius_m, length_m } => VisualShape::Cylinder {
            radius_m: *radius_m,
            length_m: *length_m,
        },
        UrdfGeometry::Mesh { path, scale } => VisualShape::Mesh {
            path: path.clone(),
            scale: *scale,
        },
    }
}

fn cylinder_to_capsule(radius_m: f64, length_m: f64) -> ColliderShape {
    let half_height_m = (length_m * 0.5 - radius_m).max(radius_m * 0.1);
    ColliderShape::Capsule {
        half_height_m,
        radius_m,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::schema::UrdfGeometry;
    use rne_math::Vec3;
    use std::path::PathBuf;

    #[test]
    fn box_geometry_maps_to_cuboid_collider() {
        let element = UrdfGeometryElement {
            origin_xyz: Vec3::ZERO,
            origin_rpy: Vec3::ZERO,
            material_rgba: None,
            geometry: UrdfGeometry::Box {
                size_m: Vec3::new(1.0, 2.0, 3.0),
            },
        };
        let collider = collider_from_element(&element, None).expect("collider");
        assert_eq!(
            collider.shape,
            ColliderShape::Cuboid {
                half_extents_m: Vec3::new(0.5, 1.0, 1.5),
            }
        );
    }

    #[test]
    fn mesh_collision_without_assets_root_is_skipped() {
        let element = UrdfGeometryElement {
            origin_xyz: Vec3::ZERO,
            origin_rpy: Vec3::ZERO,
            material_rgba: None,
            geometry: UrdfGeometry::Mesh {
                path: "package://robot/mesh.stl".into(),
                scale: Vec3::ONE,
            },
        };
        assert!(collider_from_element(&element, None).is_none());
        assert!(matches!(
            visual_from_element(&element, [1.0, 1.0, 1.0, 1.0]).shape,
            VisualShape::Mesh { .. }
        ));
    }

    #[test]
    fn mesh_collision_uses_stl_aabb_cuboid() {
        let package_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("tests/fixtures/mesh_diff_drive_package");
        let element = UrdfGeometryElement {
            origin_xyz: Vec3::ZERO,
            origin_rpy: Vec3::ZERO,
            material_rgba: None,
            geometry: UrdfGeometry::Mesh {
                path: "package://mesh_diff_drive/meshes/base_link.stl".into(),
                scale: Vec3::ONE,
            },
        };
        let collider =
            collider_from_element(&element, Some(package_root.as_path())).expect("mesh collider");
        assert!(matches!(collider.shape, ColliderShape::Cuboid { .. }));
        if let ColliderShape::Cuboid { half_extents_m } = collider.shape {
            assert!(half_extents_m.x > 0.2);
            assert!(half_extents_m.y > 0.1);
            assert!(half_extents_m.z > 0.1);
        }
    }

    #[test]
    fn visual_material_color_overrides_spawn_fallback() {
        let element = UrdfGeometryElement {
            origin_xyz: Vec3::ZERO,
            origin_rpy: Vec3::ZERO,
            material_rgba: Some([0.1, 0.2, 0.3, 1.0]),
            geometry: UrdfGeometry::Sphere { radius_m: 0.25 },
        };
        assert_eq!(
            visual_from_element(&element, [1.0, 1.0, 1.0, 1.0]).color_rgba,
            [0.1, 0.2, 0.3, 1.0]
        );
    }
}

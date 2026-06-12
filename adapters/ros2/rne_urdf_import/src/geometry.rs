//! Convert parsed URDF geometry into RNE components.

use crate::parse::rpy_to_quat;
use crate::schema::{UrdfGeometry, UrdfGeometryElement};
use rne_math::Quat;
use rne_physics::{Collider, ColliderShape};
use rne_render::{Visual, VisualShape};
use rne_world::Transform3;

/// Returns the extra rotation needed when mapping a URDF Z cylinder to a Y capsule.
pub fn cylinder_collider_rotation() -> Quat {
    Quat::from_rotation_x(std::f64::consts::FRAC_PI_2)
}

/// Converts a URDF geometry element into a collider component.
pub fn collider_from_element(element: &UrdfGeometryElement) -> Option<Collider> {
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
        UrdfGeometry::Mesh { .. } => return None,
    };

    Some(Collider {
        shape,
        material: Default::default(),
        local_offset,
    })
}

/// Converts a URDF geometry element into a visual component.
pub fn visual_from_element(element: &UrdfGeometryElement, color_rgba: [f32; 4]) -> Visual {
    Visual {
        shape: visual_shape_from_geometry(&element.geometry),
        local_offset: local_offset_from_element(element),
        color_rgba,
    }
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

    #[test]
    fn box_geometry_maps_to_cuboid_collider() {
        let element = UrdfGeometryElement {
            origin_xyz: Vec3::ZERO,
            origin_rpy: Vec3::ZERO,
            geometry: UrdfGeometry::Box {
                size_m: Vec3::new(1.0, 2.0, 3.0),
            },
        };
        let collider = collider_from_element(&element).expect("collider");
        assert_eq!(
            collider.shape,
            ColliderShape::Cuboid {
                half_extents_m: Vec3::new(0.5, 1.0, 1.5),
            }
        );
    }

    #[test]
    fn mesh_collision_is_skipped() {
        let element = UrdfGeometryElement {
            origin_xyz: Vec3::ZERO,
            origin_rpy: Vec3::ZERO,
            geometry: UrdfGeometry::Mesh {
                path: "package://robot/mesh.stl".into(),
                scale: Vec3::ONE,
            },
        };
        assert!(collider_from_element(&element).is_none());
        assert!(matches!(
            visual_from_element(&element, [1.0, 1.0, 1.0, 1.0]).shape,
            VisualShape::Mesh { .. }
        ));
    }
}

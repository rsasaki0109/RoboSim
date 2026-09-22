//! Type conversion between RNE math and Rapier.

use rapier3d::na::{DMatrix, Point3, Unit, UnitQuaternion, Vector3};
use rapier3d::prelude::{Isometry, SharedShape};
use rne_math::{Quat, Vec3};
use rne_physics::{ColliderShape, PhysicsError, RigidBodyType};
use rne_world::Transform3;

pub fn vec3_to_rapier(v: Vec3) -> Vector3<f32> {
    Vector3::new(v.x as f32, v.y as f32, v.z as f32)
}

pub fn vec3_from_rapier(v: Vector3<f32>) -> Vec3 {
    Vec3::new(v.x as f64, v.y as f64, v.z as f64)
}

pub fn vec3_to_point(v: Vec3) -> Point3<f32> {
    Point3::new(v.x as f32, v.y as f32, v.z as f32)
}

pub fn vec3_from_point(v: Point3<f32>) -> Vec3 {
    Vec3::new(v.x as f64, v.y as f64, v.z as f64)
}

pub fn quat_to_rapier(q: Quat) -> UnitQuaternion<f32> {
    UnitQuaternion::from_quaternion(rapier3d::na::Quaternion::new(
        q.w as f32, q.x as f32, q.y as f32, q.z as f32,
    ))
}

pub fn quat_from_rapier(q: UnitQuaternion<f32>) -> Quat {
    let coords = q.coords;
    Quat::from_xyzw(
        coords[0] as f64,
        coords[1] as f64,
        coords[2] as f64,
        coords[3] as f64,
    )
}

pub fn transform_to_isometry(transform: &Transform3) -> Isometry<f32> {
    Isometry::from_parts(
        vec3_to_rapier(transform.translation).into(),
        quat_to_rapier(transform.rotation),
    )
}

pub fn isometry_to_transform(isometry: &Isometry<f32>) -> Transform3 {
    Transform3 {
        translation: vec3_from_rapier(isometry.translation.vector),
        rotation: quat_from_rapier(isometry.rotation),
        scale: Vec3::ONE,
    }
}

pub fn shape_to_shared(shape: &ColliderShape) -> Result<SharedShape, PhysicsError> {
    Ok(match shape {
        ColliderShape::Sphere { radius_m } => SharedShape::ball(*radius_m as f32),
        ColliderShape::Cuboid { half_extents_m } => SharedShape::cuboid(
            half_extents_m.x as f32,
            half_extents_m.y as f32,
            half_extents_m.z as f32,
        ),
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => SharedShape::capsule_y(*half_height_m as f32, *radius_m as f32),
        ColliderShape::Plane { normal } => {
            let mut n = vec3_to_rapier(*normal);
            if n.norm_squared() <= f32::EPSILON {
                n = Vector3::y();
            } else {
                n.normalize_mut();
            }
            SharedShape::halfspace(Unit::new_normalize(n))
        }
        ColliderShape::ConvexHull { points } => {
            // parry's convex-hull builder panics on too few points, so guard
            // before calling it and surface a typed backend error instead.
            if points.len() < 4 {
                return Err(PhysicsError::InvalidColliderShape {
                    reason: "convex hull requires at least four points",
                });
            }
            let vertices: Vec<Point3<f32>> =
                points.iter().map(|point| vec3_to_point(*point)).collect();
            SharedShape::convex_hull(&vertices).ok_or(PhysicsError::InvalidColliderShape {
                reason: "convex hull points are degenerate",
            })?
        }
        ColliderShape::TriMesh { vertices, indices } => {
            if vertices.len() < 3 || indices.len() < 3 || indices.len() % 3 != 0 {
                return Err(PhysicsError::InvalidColliderShape {
                    reason: "triangle mesh requires vertices and flat index triples",
                });
            }
            let points: Vec<Point3<f32>> =
                vertices.iter().map(|point| vec3_to_point(*point)).collect();
            let faces: Vec<[u32; 3]> = indices
                .chunks_exact(3)
                .map(|face| [face[0], face[1], face[2]])
                .collect();
            SharedShape::trimesh(points, faces)
        }
        ColliderShape::HeightField {
            nrows,
            ncols,
            heights_m,
            scale,
        } => {
            let expected = (*nrows as usize) * (*ncols as usize);
            if *nrows < 2 || *ncols < 2 || heights_m.len() != expected {
                return Err(PhysicsError::InvalidColliderShape {
                    reason: "height field requires nrows>=2, ncols>=2, and nrows*ncols heights",
                });
            }
            // parry indexes rows along local Z and columns along local X.
            let data: Vec<f32> = heights_m.iter().map(|height| *height as f32).collect();
            let heights = DMatrix::from_row_slice(*nrows as usize, *ncols as usize, &data);
            SharedShape::heightfield(heights, vec3_to_rapier(*scale))
        }
        ColliderShape::Compound { parts } => {
            if parts.is_empty() {
                return Err(PhysicsError::InvalidColliderShape {
                    reason: "compound collider requires at least one part",
                });
            }
            let mut children = Vec::with_capacity(parts.len());
            for part in parts.iter() {
                children.push((
                    transform_to_isometry(&part.local_offset),
                    shape_to_shared(&part.shape)?,
                ));
            }
            SharedShape::compound(children)
        }
    })
}

pub fn body_type_to_rapier(body_type: RigidBodyType) -> rapier3d::prelude::RigidBodyType {
    match body_type {
        RigidBodyType::Dynamic => rapier3d::prelude::RigidBodyType::Dynamic,
        RigidBodyType::Fixed => rapier3d::prelude::RigidBodyType::Fixed,
        RigidBodyType::Kinematic => rapier3d::prelude::RigidBodyType::KinematicPositionBased,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics::CompoundPart;
    use std::sync::Arc;

    fn hull(points: Vec<Vec3>) -> ColliderShape {
        ColliderShape::ConvexHull {
            points: Arc::from(points.into_boxed_slice()),
        }
    }

    #[test]
    fn primitive_shapes_convert() {
        assert!(shape_to_shared(&ColliderShape::Sphere { radius_m: 0.5 }).is_ok());
        assert!(shape_to_shared(&ColliderShape::Cuboid {
            half_extents_m: Vec3::splat(0.5),
        })
        .is_ok());
        assert!(shape_to_shared(&ColliderShape::Plane { normal: Vec3::Y }).is_ok());
    }

    #[test]
    fn tetrahedral_hull_converts() {
        let shape = hull(vec![
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ]);
        assert!(shape_to_shared(&shape).is_ok());
    }

    #[test]
    fn empty_hull_is_rejected() {
        let shape = hull(Vec::new());
        assert!(matches!(
            shape_to_shared(&shape),
            Err(PhysicsError::InvalidColliderShape { .. })
        ));
    }

    #[test]
    fn trimesh_converts_and_rejects_ragged_indices() {
        let vertices: Arc<[Vec3]> = vec![
            Vec3::ZERO,
            Vec3::new(1.0, 0.0, 0.0),
            Vec3::new(0.0, 1.0, 0.0),
            Vec3::new(0.0, 0.0, 1.0),
        ]
        .into();
        let indices: Arc<[u32]> = vec![0, 1, 2, 0, 2, 3].into();
        let shape = ColliderShape::TriMesh {
            vertices: vertices.clone(),
            indices,
        };
        assert!(shape_to_shared(&shape).is_ok());

        let ragged = ColliderShape::TriMesh {
            vertices,
            indices: Arc::from(vec![0_u32, 1, 2, 3]),
        };
        assert!(matches!(
            shape_to_shared(&ragged),
            Err(PhysicsError::InvalidColliderShape { .. })
        ));
    }

    #[test]
    fn heightfield_converts_and_validates_extent() {
        let shape = ColliderShape::HeightField {
            nrows: 2,
            ncols: 2,
            heights_m: Arc::from(vec![0.0_f64, 0.0, 0.0, 0.1]),
            scale: Vec3::new(1.0, 1.0, 1.0),
        };
        assert!(shape_to_shared(&shape).is_ok());

        let wrong_len = ColliderShape::HeightField {
            nrows: 3,
            ncols: 3,
            heights_m: Arc::from(vec![0.0_f64, 0.0]),
            scale: Vec3::ONE,
        };
        assert!(matches!(
            shape_to_shared(&wrong_len),
            Err(PhysicsError::InvalidColliderShape { .. })
        ));
    }

    #[test]
    fn compound_converts_and_rejects_empty() {
        let part = |x: f64| CompoundPart {
            shape: ColliderShape::Sphere { radius_m: 0.1 },
            local_offset: Transform3::from_translation_rotation(
                Vec3::new(x, 0.0, 0.0),
                Quat::IDENTITY,
            ),
        };
        let shape = ColliderShape::Compound {
            parts: Arc::from(vec![part(-0.2), part(0.2)]),
        };
        assert!(shape_to_shared(&shape).is_ok());

        let empty = ColliderShape::Compound {
            parts: Arc::from(Vec::<CompoundPart>::new()),
        };
        assert!(matches!(
            shape_to_shared(&empty),
            Err(PhysicsError::InvalidColliderShape { .. })
        ));
    }
}

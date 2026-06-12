//! Type conversion between RNE math and Rapier.

use rapier3d::na::{Point3, Unit, UnitQuaternion, Vector3};
use rapier3d::prelude::{Isometry, SharedShape};
use rne_math::{Quat, Vec3};
use rne_physics::{ColliderShape, RigidBodyType};
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

pub fn shape_to_shared(shape: ColliderShape) -> SharedShape {
    match shape {
        ColliderShape::Sphere { radius_m } => SharedShape::ball(radius_m as f32),
        ColliderShape::Cuboid { half_extents_m } => SharedShape::cuboid(
            half_extents_m.x as f32,
            half_extents_m.y as f32,
            half_extents_m.z as f32,
        ),
        ColliderShape::Capsule {
            half_height_m,
            radius_m,
        } => SharedShape::capsule_y(half_height_m as f32, radius_m as f32),
        ColliderShape::Plane { normal } => {
            let mut n = vec3_to_rapier(normal);
            if n.norm_squared() <= f32::EPSILON {
                n = Vector3::y();
            } else {
                n.normalize_mut();
            }
            SharedShape::halfspace(Unit::new_normalize(n))
        }
    }
}

pub fn body_type_to_rapier(body_type: RigidBodyType) -> rapier3d::prelude::RigidBodyType {
    match body_type {
        RigidBodyType::Dynamic => rapier3d::prelude::RigidBodyType::Dynamic,
        RigidBodyType::Fixed => rapier3d::prelude::RigidBodyType::Fixed,
        RigidBodyType::Kinematic => rapier3d::prelude::RigidBodyType::KinematicPositionBased,
    }
}

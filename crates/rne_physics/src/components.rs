//! Physics ECS components.

use bevy_ecs::prelude::Component;
use rne_math::Vec3;
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

/// Rigid body motion type.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
pub enum RigidBodyType {
    /// Fully simulated dynamic body.
    #[default]
    Dynamic,
    /// Immovable static body.
    Fixed,
    /// User-driven body with collision response.
    Kinematic,
}

/// Rigid body simulation properties.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct RigidBody {
    /// Motion type.
    pub body_type: RigidBodyType,
    /// Mass in kilograms. Ignored for fixed bodies.
    pub mass_kg: f64,
    /// Linear velocity in meters per second.
    pub linear_velocity_m_s: Vec3,
    /// Angular velocity in radians per second.
    pub angular_velocity_rad_s: Vec3,
}

impl Default for RigidBody {
    fn default() -> Self {
        Self {
            body_type: RigidBodyType::Dynamic,
            mass_kg: 1.0,
            linear_velocity_m_s: Vec3::ZERO,
            angular_velocity_rad_s: Vec3::ZERO,
        }
    }
}

/// Collision shape definition.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub enum ColliderShape {
    /// Sphere with radius in meters.
    Sphere {
        /// Radius in meters.
        radius_m: f64,
    },
    /// Axis-aligned box half extents in meters.
    Cuboid {
        /// Half extents in meters.
        half_extents_m: Vec3,
    },
    /// Capsule aligned with the Y axis.
    Capsule {
        /// Half height in meters (excluding hemispheres).
        half_height_m: f64,
        /// Radius in meters.
        radius_m: f64,
    },
    /// Infinite plane with outward normal.
    Plane {
        /// Unit normal vector.
        normal: Vec3,
    },
}

impl Default for ColliderShape {
    fn default() -> Self {
        Self::Cuboid {
            half_extents_m: Vec3::splat(0.5),
        }
    }
}

/// Collider attached to an entity.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Collider {
    /// Shape definition.
    pub shape: ColliderShape,
    /// Surface material properties.
    pub material: PhysicsMaterial,
    /// Pose relative to the entity transform.
    pub local_offset: Transform3,
}

impl Default for Collider {
    fn default() -> Self {
        Self {
            shape: ColliderShape::default(),
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
        }
    }
}

impl Collider {
    /// Creates a cuboid collider with the given half extents.
    pub fn cuboid(half_extents_m: Vec3) -> Self {
        Self {
            shape: ColliderShape::Cuboid { half_extents_m },
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
        }
    }

    /// Creates a sphere collider with the given radius.
    pub fn sphere(radius_m: f64) -> Self {
        Self {
            shape: ColliderShape::Sphere { radius_m },
            material: PhysicsMaterial::default(),
            local_offset: Transform3::IDENTITY,
        }
    }
}

/// Physical surface material.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct PhysicsMaterial {
    /// Coulomb friction coefficient.
    pub friction: f32,
    /// Coefficient of restitution.
    pub restitution: f32,
}

impl Default for PhysicsMaterial {
    fn default() -> Self {
        Self {
            friction: 0.5,
            restitution: 0.0,
        }
    }
}

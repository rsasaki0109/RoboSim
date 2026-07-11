//! Physics ECS components.

use bevy_ecs::prelude::Component;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
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

/// Pairwise collider filtering masks.
///
/// Two colliders interact when each collider's membership overlaps the other
/// collider's filter. Missing components are treated as [`Self::default`].
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct CollisionGroups {
    /// Groups this collider belongs to.
    pub memberships: u32,
    /// Groups this collider accepts interactions from.
    pub filter: u32,
}

impl Default for CollisionGroups {
    fn default() -> Self {
        Self {
            memberships: u32::MAX,
            filter: u32::MAX,
        }
    }
}

impl CollisionGroups {
    /// Creates masks that disable interactions with colliders in the same group.
    pub const fn without_self_collision(group_bit: u32) -> Self {
        Self {
            memberships: group_bit,
            filter: !group_bit,
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

/// Revolute joint description for physics backends.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct RevoluteJointDesc {
    /// Parent rigid body entity.
    pub parent: Entity,
    /// Joint axis in parent-local coordinates.
    pub axis: Vec3,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Optional lower angle limit in radians.
    pub lower_rad: Option<f64>,
    /// Optional upper angle limit in radians.
    pub upper_rad: Option<f64>,
}

/// Prismatic (linear sliding) joint description for physics backends.
///
/// The single free degree of freedom translates the child body along `axis`,
/// expressed in the parent body's local frame.
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct PrismaticJointDesc {
    /// Parent rigid body entity.
    pub parent: Entity,
    /// Sliding axis in parent-local coordinates.
    pub axis: Vec3,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
}

/// Fixed (weld) joint description for physics backends.
///
/// Rigidly locks the child body to the parent at the relative pose implied by the
/// anchors and `relative_rotation`, removing all six relative degrees of freedom.
/// Inserting this component attaches the weld on the next sync; removing it releases
/// the weld. Intended for attach-on-contact grasping (weld a grasped object to the
/// gripper at its current relative pose so it neither snaps nor drifts).
#[derive(Component, Clone, Copy, Debug, PartialEq)]
pub struct FixedJointDesc {
    /// Parent rigid body entity (e.g. the gripper link).
    pub parent: Entity,
    /// Anchor point in the parent body's local frame.
    pub anchor_parent_m: Vec3,
    /// Anchor point in the child body's local frame.
    pub anchor_child_m: Vec3,
    /// Orientation of the child frame relative to the parent frame.
    pub relative_rotation: Quat,
}

/// Marks a rigid body and its joint as part of a reduced-coordinate multibody.
///
/// Backends that support multibodies use this instead of an impulse joint. The
/// marker may also be placed on the root so it is simulated without a collider.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct MultibodyLink;

/// Velocity motor command applied to a joint before each physics step.
///
/// The value is interpreted as an angular velocity (rad/s) for revolute joints
/// and as a linear velocity (m/s) for prismatic joints.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointMotor {
    /// Target velocity: radians per second (revolute) or meters per second (prismatic).
    pub velocity_rad_s: f64,
    /// Velocity-tracking gain (motor damping factor). Higher values track the target
    /// velocity more stiffly under load — e.g. a joint holding weight against gravity —
    /// up to the backend's motor force cap. Defaults to `1.0`.
    #[serde(default = "default_motor_gain")]
    pub gain: f64,
    /// Position-tracking stiffness. When `0.0` (the default) the motor is a pure
    /// velocity motor. When positive, the motor also pulls the joint toward
    /// [`Self::target_position`] like a spring (with `gain` acting as its damping),
    /// which lets a joint *hold* a load against gravity without drift — required
    /// for a stable vertical lift carrying a multi-link arm.
    #[serde(default)]
    pub stiffness: f64,
    /// Position target the motor pulls toward when [`Self::stiffness`] is positive:
    /// radians (revolute) or meters along the slide axis (prismatic).
    #[serde(default)]
    pub target_position: f64,
    /// Maximum force/torque the motor may apply. `0.0` (the default) uses the backend's
    /// per-joint-type cap; a positive value overrides it (e.g. a heavier arm joint that
    /// needs more torque to track its target quickly).
    #[serde(default)]
    pub max_force: f64,
}

fn default_motor_gain() -> f64 {
    1.0
}

impl Default for JointMotor {
    fn default() -> Self {
        Self {
            velocity_rad_s: 0.0,
            gain: default_motor_gain(),
            stiffness: 0.0,
            target_position: 0.0,
            max_force: 0.0,
        }
    }
}

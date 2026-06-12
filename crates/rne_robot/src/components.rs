//! Robot entity components.

use bevy_ecs::prelude::Component;
use rne_ecs::Entity;
use rne_math::{Quat, Vec3};
use serde::{Deserialize, Serialize};
use uuid::Uuid;

/// Stable robot identifier.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct RobotId(pub Uuid);

impl RobotId {
    /// Creates a new random robot id.
    pub fn new_v4() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Default for RobotId {
    fn default() -> Self {
        Self::new_v4()
    }
}

/// Top-level robot entity marker.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Robot {
    /// Stable robot identifier.
    pub robot_id: RobotId,
    /// Human-readable model name.
    pub model_name: String,
    /// Base link entity.
    pub base_link: Entity,
}

/// Physical link on a robot.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Link {
    /// Owning robot entity.
    pub robot: Entity,
    /// Link name.
    pub name: String,
}

/// Joint type between two links.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum JointKind {
    /// Fixed joint with no degrees of freedom.
    Fixed,
    /// Revolute joint about one axis.
    Revolute,
    /// Continuous revolute joint without limits.
    Continuous,
    /// Prismatic joint sliding along one axis.
    Prismatic,
}

/// Joint limit specification.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct JointLimits {
    /// Lower position limit in radians or meters.
    pub lower: f64,
    /// Upper position limit in radians or meters.
    pub upper: f64,
    /// Maximum velocity in radians per second or meters per second.
    pub max_velocity: f64,
    /// Maximum effort in newton-meters or newtons.
    pub max_effort: f64,
}

impl Default for JointLimits {
    fn default() -> Self {
        Self {
            lower: -f64::INFINITY,
            upper: f64::INFINITY,
            max_velocity: f64::INFINITY,
            max_effort: f64::INFINITY,
        }
    }
}

/// Joint connecting parent and child links.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Joint {
    /// Owning robot entity.
    pub robot: Entity,
    /// Parent link entity.
    pub parent_link: Entity,
    /// Child link entity.
    pub child_link: Entity,
    /// Joint type.
    pub kind: JointKind,
    /// Joint limits.
    pub limits: JointLimits,
    /// Joint axis in parent frame.
    pub axis: Vec3,
    /// Current joint position in radians or meters.
    pub position: f64,
    /// Current joint velocity.
    pub velocity: f64,
}

/// Actuator driving a joint or wheel.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct Actuator {
    /// Owning robot entity.
    pub robot: Entity,
    /// Driven joint entity, if any.
    pub joint: Option<Entity>,
    /// Actuator name.
    pub name: String,
    /// Current control mode.
    pub mode: crate::actuator::ControlMode,
    /// Current command target.
    pub target: crate::actuator::ActuatorTarget,
    /// Safety and saturation limits.
    pub limits: crate::actuator::ActuatorLimits,
}

/// Inertial properties for a link.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct Inertial {
    /// Mass in kilograms.
    pub mass_kg: f64,
    /// Center of mass offset in meters.
    pub center_of_mass_m: Vec3,
    /// Orientation of inertial frame.
    pub orientation: Quat,
}

impl Default for Inertial {
    fn default() -> Self {
        Self {
            mass_kg: 1.0,
            center_of_mass_m: Vec3::ZERO,
            orientation: Quat::IDENTITY,
        }
    }
}

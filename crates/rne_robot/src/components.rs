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

/// Deterministic kinematic Ackermann drive state and safety limits.
///
/// The drive uses the entity's local `+X` axis as its forward direction. Commands
/// are clamped to the configured speed and steering limits. The integration
/// system ignores an entity when its limits are non-finite or physically invalid.
#[derive(Component, Clone, Debug, PartialEq)]
pub struct AckermannDrive {
    /// Distance between front and rear axles in meters.
    pub wheelbase_m: f64,
    /// Maximum absolute forward or reverse speed in meters per second.
    pub max_speed_m_s: f64,
    /// Maximum absolute front-wheel steering angle in radians.
    pub max_steering_rad: f64,
    /// Maximum speed increase per second in meters per second squared.
    pub max_acceleration_m_s2: f64,
    /// Maximum braking or direction-change rate in meters per second squared.
    pub max_deceleration_m_s2: f64,
    /// Maximum steering-angle change in radians per second.
    pub max_steering_rate_rad_s: f64,
    /// Current signed longitudinal speed in meters per second.
    pub speed_m_s: f64,
    /// Current front-wheel steering angle in radians.
    pub steering_rad: f64,
    /// Clamped target signed longitudinal speed in meters per second.
    pub target_speed_m_s: f64,
    /// Clamped target front-wheel steering angle in radians.
    pub target_steering_rad: f64,
}

impl Default for AckermannDrive {
    fn default() -> Self {
        Self {
            wheelbase_m: 2.7,
            max_speed_m_s: 13.9,
            max_steering_rad: 0.6,
            max_acceleration_m_s2: 2.5,
            max_deceleration_m_s2: 5.0,
            max_steering_rate_rad_s: 0.8,
            speed_m_s: 0.0,
            steering_rad: 0.0,
            target_speed_m_s: 0.0,
            target_steering_rad: 0.0,
        }
    }
}

impl AckermannDrive {
    /// Returns whether all limits and state values are finite and physically valid.
    pub fn is_valid(&self) -> bool {
        [
            self.wheelbase_m,
            self.max_speed_m_s,
            self.max_steering_rad,
            self.max_acceleration_m_s2,
            self.max_deceleration_m_s2,
            self.max_steering_rate_rad_s,
            self.speed_m_s,
            self.steering_rad,
            self.target_speed_m_s,
            self.target_steering_rad,
        ]
        .iter()
        .all(|value| value.is_finite())
            && self.wheelbase_m > 0.0
            && self.max_speed_m_s >= 0.0
            && self.max_steering_rad >= 0.0
            && self.max_acceleration_m_s2 >= 0.0
            && self.max_deceleration_m_s2 >= 0.0
            && self.max_steering_rate_rad_s >= 0.0
    }
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

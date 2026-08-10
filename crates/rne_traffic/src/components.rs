//! Traffic ECS components.

use bevy_ecs::prelude::Component;
use serde::{Deserialize, Serialize};

use crate::TrafficId;

/// Marks the root ECS entity for one loaded traffic network.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct TrafficNetworkRoot;

/// Classifies a road user without constraining its robot or policy model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrafficActorKind {
    /// A motor vehicle, including a car, bus, or truck.
    MotorVehicle,
    /// A bicycle or another lightweight cycle.
    Bicycle,
    /// A pedestrian.
    Pedestrian,
}

/// Marks an entity as a road user that participates in traffic integration.
///
/// Externally visible iteration uses the entity's [`rne_ecs::EntityUuid`],
/// never ECS insertion order.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TrafficActor {
    /// Broad road-user classification.
    pub kind: TrafficActorKind,
}

impl TrafficActor {
    /// Creates a motor-vehicle traffic actor.
    pub const fn motor_vehicle() -> Self {
        Self {
            kind: TrafficActorKind::MotorVehicle,
        }
    }
}

/// Identifies the subsystem that owns an actor's [`TrafficPose`].
///
/// Actors without this component, or with [`TrafficPoseSource::Runtime`], are
/// advanced by the deterministic traffic runtime. An external adapter can
/// attach [`TrafficPoseSource::External`] when it supplies the pose each
/// simulation step; traffic runtime systems then leave that actor untouched.
#[derive(Component, Clone, Copy, Debug, Default, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum TrafficPoseSource {
    /// The RNE traffic runtime owns route progress and pose updates.
    #[default]
    Runtime,
    /// An external simulator or adapter owns route progress and pose updates.
    External,
}

/// Kinematic progress of one actor along a catalogued traffic route.
#[derive(Component, Clone, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrafficRouteFollower {
    /// Stable route identifier resolved through `TrafficRouteCatalog`.
    pub route_id: TrafficId,
    /// Distance traveled from the beginning of the route.
    pub distance_m: f64,
    /// Current longitudinal speed.
    pub speed_m_s: f64,
    /// Free-flow target speed before headway constraints.
    pub desired_speed_m_s: f64,
    /// Actor bumper-to-bumper length.
    pub length_m: f64,
}

/// Optional simulation-time gate for a scheduled traffic departure.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrafficDeparture {
    /// Earliest simulation time at which the actor may move.
    pub departure_time_s: f64,
}

/// Backend-neutral pose sampled from a traffic route.
#[derive(Component, Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
pub struct TrafficPose {
    /// Position in the route coordinate frame.
    pub position_m: [f64; 3],
    /// Heading around the positive Y axis in radians.
    pub yaw_rad: f64,
}

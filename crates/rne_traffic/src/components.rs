//! Traffic ECS components.

use bevy_ecs::prelude::Component;
use serde::{Deserialize, Serialize};

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

/// Marks an entity as a road user managed by traffic runtime systems.
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

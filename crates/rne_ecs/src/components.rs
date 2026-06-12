//! Shared ECS components.

use bevy_ecs::prelude::Component;
use serde::{Deserialize, Serialize};
use smallvec::SmallVec;
use uuid::Uuid;

/// Stable UUID for logging, replay, and import/export.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct EntityUuid(pub Uuid);

impl EntityUuid {
    /// Creates a new random UUID.
    pub fn new_v4() -> Self {
        Self(Uuid::new_v4())
    }
}

impl Default for EntityUuid {
    fn default() -> Self {
        Self::new_v4()
    }
}

/// Human-readable entity name.
#[derive(Component, Clone, Debug, Default, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct Name(pub String);

impl Name {
    /// Creates a new name.
    pub fn new(name: impl Into<String>) -> Self {
        Self(name.into())
    }
}

/// Parent entity in the spatial hierarchy.
#[derive(Component, Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub struct Parent(pub bevy_ecs::entity::Entity);

/// Child entities in the spatial hierarchy.
#[derive(Component, Clone, Debug, Default, PartialEq, Eq, Hash)]
pub struct Children(pub SmallVec<[bevy_ecs::entity::Entity; 8]>);

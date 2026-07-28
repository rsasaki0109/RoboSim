//! Deterministic, backend-neutral traffic semantics and runtime state.
//!
//! Traffic actors augment Robot and Agent Entities but do not replace them.
//! This crate does not depend on a renderer, physics backend, geospatial
//! importer, robotics adapter, or external traffic simulator.

#![deny(missing_docs)]

pub mod components;
pub mod events;
pub mod resources;
pub mod systems;

pub use components::{TrafficActor, TrafficActorKind, TrafficNetworkRoot};
pub use events::TrafficStepCompleted;
pub use resources::TrafficRuntime;
pub use systems::{
    advance_traffic_step, traffic_actors_in_stable_order, MissingTrafficActorStableId,
};

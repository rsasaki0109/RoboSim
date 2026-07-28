//! Deterministic, backend-neutral traffic semantics and runtime state.
//!
//! Traffic actors augment Robot and Agent Entities but do not replace them.
//! This crate does not depend on a renderer, physics backend, geospatial
//! importer, robotics adapter, or external traffic simulator.

#![deny(missing_docs)]

pub mod asset;
pub mod components;
pub mod error;
pub mod events;
pub mod id;
pub mod io;
pub mod resources;
pub mod systems;

pub use asset::{
    Accuracy, AccuracyClass, AuthorityClass, AxisConvention, CoordinateFrame, Junction,
    JunctionKind, Lane, LaneKind, MovementKind, Provenance, SignalAspect, SignalGroup,
    SignalGroupAspect, SignalPhase, SignalProgram, SourceReference, TrafficAsset,
    TrafficConnection, TrafficNetwork, TrafficSignal, TRAFFIC_ASSET_SCHEMA,
    TRAFFIC_ASSET_SCHEMA_VERSION,
};
pub use components::{TrafficActor, TrafficActorKind, TrafficNetworkRoot};
pub use error::{TrafficAssetError, TrafficIdError};
pub use events::TrafficStepCompleted;
pub use id::TrafficId;
pub use io::{
    canonical_traffic_asset_bytes, load_traffic_asset, parse_traffic_asset, save_traffic_asset,
};
pub use resources::TrafficRuntime;
pub use systems::{
    advance_traffic_step, traffic_actors_in_stable_order, MissingTrafficActorStableId,
};

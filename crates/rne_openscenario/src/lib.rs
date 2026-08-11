//! OpenSCENARIO 1.0 scenario import for Robot Native Engine.
//!
//! RNE's scenario workflow takes a fixed-step run (see `rne_asset_cli`) and,
//! when needed, a traffic network (see `rne_traffic`). This crate bridges the
//! industry-standard [ASAM OpenSCENARIO 1.0] storyboard into a versioned RNE
//! scenario document so a scenario file can drive the same deterministic
//! runtime as native manifests.
//!
//! The importer intentionally supports a strict subset of OpenSCENARIO 1.0 and
//! rejects everything else with a clear error instead of silently dropping it:
//!
//! - `FileHeader` with `revMajor`/`revMinor` exactly `1.0`
//! - `ParameterDeclarations` with `${name}` reference substitution
//! - `CatalogLocations` `VehicleCatalog` directories with `CatalogReference`
//!   entity lookup
//! - `RoadNetwork/LogicFile@filepath` recorded as the road-network reference
//! - `Entities/ScenarioObject` declaring `Vehicle`, `Bicycle`, or `Pedestrian`
//! - `Storyboard/Init` `TeleportAction` `WorldPosition` spawn poses
//! - storyboard `SpeedAction` events with an `AbsoluteTargetSpeed` and a
//!   `SimulationTimeCondition` start time, `LaneChangeAction` events with a
//!   `RelativeTargetLane` offset, and `AssignRouteAction` waypoint routes
//! - the road network's fixed-time `TrafficSignal` programs drive stop lines
//!   during scenario execution
//!
//! Controller bindings are not yet supported.
//!
//! [ASAM OpenSCENARIO 1.0]: https://www.asam.net/standards/detail/openscenario/

#![deny(missing_docs)]

pub mod parser;
pub mod replay;
pub mod runtime;
pub mod scenario;

pub use parser::{
    parse_openscenario_xml, parse_openscenario_xml_file, parse_openscenario_xml_with_source,
    parse_openscenario_xml_with_source_at,
};
pub use replay::{
    stable_replay_input_digest, ScenarioReplayArtifact, ScenarioReplayArtifactError,
    ScenarioReplayInputs, SCENARIO_REPLAY_KIND, SCENARIO_REPLAY_SCHEMA_VERSION,
};
pub use runtime::{
    actor_length_m, execute_scenario, execute_scenario_with_control, ScenarioActionEvidence,
    ScenarioActorResult, ScenarioRunOptions, ScenarioRunResult,
};
pub use scenario::{
    ScenarioAction, ScenarioDocument, ScenarioEntity, ScenarioEntityKind, ScenarioError,
    ScenarioTimedAction, SCENARIO_DOCUMENT_VERSION,
};

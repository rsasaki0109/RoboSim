//! Versioned scenario documents imported from OpenSCENARIO.

use serde::{Deserialize, Serialize};
use std::fs;
use std::path::Path;
use thiserror::Error;

/// Current `.rne.scenario.json` schema version.
pub const SCENARIO_DOCUMENT_VERSION: u32 = 1;

/// Scenario import, serialization, or validation failure.
#[derive(Debug, Error)]
pub enum ScenarioError {
    /// The scenario file or document could not be read or written.
    #[error(transparent)]
    Io(#[from] std::io::Error),
    /// The scenario document could not be serialized or deserialized.
    #[error(transparent)]
    Json(#[from] serde_json::Error),
    /// The document uses a schema version unsupported by this engine.
    #[error("unsupported scenario version: expected {expected}, got {actual}")]
    UnsupportedVersion {
        /// Schema version supported by this engine.
        expected: u32,
        /// Schema version found in the document.
        actual: u32,
    },
    /// The OpenSCENARIO file uses a revision unsupported by this importer.
    #[error("unsupported OpenSCENARIO revision: expected {expected}.{minor}, got {actual}.{actual_minor}")]
    UnsupportedRevision {
        /// Supported major revision.
        expected: u32,
        /// Supported minor revision.
        minor: u32,
        /// Major revision found in the file.
        actual: u32,
        /// Minor revision found in the file.
        actual_minor: u32,
    },
    /// The file contains an element or attribute the importer cannot handle.
    #[error("unsupported OpenSCENARIO element `{element}`: {reason}")]
    UnsupportedElement {
        /// XML element name.
        element: String,
        /// Why the element cannot be imported.
        reason: String,
    },
    /// The scenario file or document is malformed.
    #[error("invalid scenario: {0}")]
    Invalid(String),
}

/// Classifies a scenario road user without constraining its robot or policy model.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ScenarioEntityKind {
    /// A motor vehicle, including a car, bus, or truck.
    MotorVehicle,
    /// A bicycle or another lightweight cycle.
    Bicycle,
    /// A pedestrian.
    Pedestrian,
}

/// One scenario entity declared in `Entities`.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioEntity {
    /// Scenario entity name.
    pub name: String,
    /// Broad road-user classification.
    pub kind: ScenarioEntityKind,
    /// Initial world position in metres from a `TeleportAction`, when present.
    #[serde(default)]
    pub initial_world_position_m: Option<[f64; 3]>,
    /// Initial world heading in radians, when present.
    #[serde(default)]
    pub initial_heading_rad: Option<f64>,
}

/// One supported storyboard action.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(tag = "kind", rename_all = "snake_case", deny_unknown_fields)]
pub enum ScenarioAction {
    /// Absolute longitudinal speed target.
    AbsoluteSpeed {
        /// Target speed in metres per second.
        target_m_s: f64,
    },
    /// Lateral lane change to a relative lane offset.
    ///
    /// The executor switches the actor to a synthetic parallel route offset
    /// one lane width to the right (+1) or left (-1); the manoeuvre is a snap,
    /// not a continuous lateral animation.
    LaneChange {
        /// Relative target lane offset, `+1` or `-1`.
        target_lane_offset: i64,
    },
    /// Assign a scripted route of world waypoints.
    ///
    /// The executor builds a polyline route through the waypoints and switches
    /// the actor's route follower onto it (snapping to the nearest point).
    AssignRoute {
        /// Ordered world waypoints in metres.
        waypoints: Vec<[f64; 3]>,
    },
}

/// A storyboard action scheduled at a simulation time.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioTimedAction {
    /// Entity the action applies to.
    pub entity: String,
    /// Simulation time at which the action starts, in seconds.
    pub start_time_s: f64,
    /// Action to apply.
    pub action: ScenarioAction,
}

/// A self-contained scenario imported from one OpenSCENARIO file.
///
/// The document keeps the source path, the road-network reference from
/// `RoadNetwork/LogicFile@filepath`, the declared entities, and the storyboard
/// actions. It does not embed the road network itself; a runner resolves the
/// network reference against the scene assets.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct ScenarioDocument {
    /// Scenario document schema version.
    pub version: u32,
    /// Source `.xosc` path or name.
    pub source: String,
    /// Road-network reference from `RoadNetwork/LogicFile@filepath`.
    pub road_network_logic_file: String,
    /// Scenario entities declared in `Entities`.
    pub entities: Vec<ScenarioEntity>,
    /// Timed storyboard actions.
    pub actions: Vec<ScenarioTimedAction>,
}

impl ScenarioDocument {
    /// Creates a scenario document from its parts.
    pub fn new(
        source: impl Into<String>,
        road_network_logic_file: impl Into<String>,
        entities: Vec<ScenarioEntity>,
        actions: Vec<ScenarioTimedAction>,
    ) -> Self {
        Self {
            version: SCENARIO_DOCUMENT_VERSION,
            source: source.into(),
            road_network_logic_file: road_network_logic_file.into(),
            entities,
            actions,
        }
    }

    /// Validates the schema and cross-reference invariants of this document.
    pub fn validate(&self) -> Result<(), ScenarioError> {
        if self.version != SCENARIO_DOCUMENT_VERSION {
            return Err(ScenarioError::UnsupportedVersion {
                expected: SCENARIO_DOCUMENT_VERSION,
                actual: self.version,
            });
        }
        if self.source.trim().is_empty() {
            return Err(ScenarioError::Invalid(
                "source must not be empty".to_string(),
            ));
        }
        if self.road_network_logic_file.trim().is_empty() {
            return Err(ScenarioError::Invalid(
                "road_network_logic_file must not be empty".to_string(),
            ));
        }
        let mut names = self
            .entities
            .iter()
            .map(|entity| entity.name.clone())
            .collect::<Vec<_>>();
        if names.iter().any(|name| name.trim().is_empty()) {
            return Err(ScenarioError::Invalid(
                "entity names must not be empty".to_string(),
            ));
        }
        names.sort_unstable();
        if names.windows(2).any(|window| window[0] == window[1]) {
            return Err(ScenarioError::Invalid(
                "entity names must be unique".to_string(),
            ));
        }
        for entity in &self.entities {
            if let Some(position) = entity.initial_world_position_m {
                if position.iter().any(|value| !value.is_finite()) {
                    return Err(ScenarioError::Invalid(format!(
                        "entity `{}` initial position must be finite",
                        entity.name
                    )));
                }
            }
            if let Some(heading) = entity.initial_heading_rad {
                if !heading.is_finite() {
                    return Err(ScenarioError::Invalid(format!(
                        "entity `{}` initial heading must be finite",
                        entity.name
                    )));
                }
            }
        }
        for action in &self.actions {
            if !self
                .entities
                .iter()
                .any(|entity| entity.name == action.entity)
            {
                return Err(ScenarioError::Invalid(format!(
                    "action targets unknown entity `{}`",
                    action.entity
                )));
            }
            if !action.start_time_s.is_finite() || action.start_time_s < 0.0 {
                return Err(ScenarioError::Invalid(format!(
                    "action for entity `{}` must have a finite non-negative start time",
                    action.entity
                )));
            }
            match &action.action {
                ScenarioAction::AbsoluteSpeed { target_m_s } => {
                    if !target_m_s.is_finite() || *target_m_s < 0.0 {
                        return Err(ScenarioError::Invalid(format!(
                            "action for entity `{}` must have a finite non-negative speed target",
                            action.entity
                        )));
                    }
                }
                ScenarioAction::LaneChange { target_lane_offset } => {
                    if *target_lane_offset != 1 && *target_lane_offset != -1 {
                        return Err(ScenarioError::Invalid(format!(
                            "action for entity `{}` lane change offset must be +1 or -1",
                            action.entity
                        )));
                    }
                }
                ScenarioAction::AssignRoute { waypoints } => {
                    if waypoints.len() < 2 {
                        return Err(ScenarioError::Invalid(format!(
                            "action for entity `{}` assigned route requires at least two waypoints",
                            action.entity
                        )));
                    }
                    if waypoints
                        .iter()
                        .any(|waypoint| waypoint.iter().any(|value| !value.is_finite()))
                    {
                        return Err(ScenarioError::Invalid(format!(
                            "action for entity `{}` assigned route waypoints must be finite",
                            action.entity
                        )));
                    }
                }
            }
        }
        Ok(())
    }

    /// Serializes a validated scenario document as pretty JSON.
    pub fn to_json(&self) -> Result<String, ScenarioError> {
        self.validate()?;
        Ok(serde_json::to_string_pretty(self)?)
    }

    /// Parses and validates a scenario document from JSON text.
    pub fn from_json(text: &str) -> Result<Self, ScenarioError> {
        let document: Self = serde_json::from_str(text)?;
        document.validate()?;
        Ok(document)
    }

    /// Writes a validated scenario document to a JSON file.
    pub fn write_json(&self, path: impl AsRef<Path>) -> Result<(), ScenarioError> {
        let path = path.as_ref();
        let text = self.to_json()?;
        if let Some(parent) = path.parent() {
            if !parent.as_os_str().is_empty() {
                fs::create_dir_all(parent)?;
            }
        }
        fs::write(path, text)?;
        Ok(())
    }

    /// Loads and validates a scenario document from a JSON file.
    pub fn read_json(path: impl AsRef<Path>) -> Result<Self, ScenarioError> {
        let text = fs::read_to_string(path)?;
        Self::from_json(&text)
    }
}

/// Validates the OpenSCENARIO `FileHeader` revision fields.
pub(crate) fn check_revision(rev_major: u32, rev_minor: u32) -> Result<(), ScenarioError> {
    if (rev_major, rev_minor) != (1, 0) {
        return Err(ScenarioError::UnsupportedRevision {
            expected: 1,
            minor: 0,
            actual: rev_major,
            actual_minor: rev_minor,
        });
    }
    Ok(())
}

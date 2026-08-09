//! Minimal SUMO `.net.xml` road-network import for Robot Native Engine.
//!
//! This offline importer converts a strict subset of the SUMO road-network
//! format into an RNE traffic asset (`rne_traffic`). The importer maps SUMO
//! `edge`/`lane` geometry and semantics onto directed RNE lanes, converts SUMO
//! east/north coordinates into the RNE Y-up frame, and then lets
//! [`rne_traffic::build_traffic_topology`] deterministically derive junctions
//! and lane connections from the lane endpoints, exactly as it does for native
//! networks.
//!
//! The importer supports a strict subset of `.net.xml` and rejects everything
//! else with a clear error:
//!
//! - `edge` elements with `function` absent or `normal` (internal and
//!   connector edges are skipped), each containing `lane` elements
//! - lane `id`, `shape` (2D `x,y` pairs, with an optional `z`), `width`,
//!   `speed`, and `allow`/`disallow` road-user classes
//! - `connection` elements (directed lane-to-lane movements with optional
//!   `tl` and `linkIndex`) and `tlLogic` fixed-time signal programs; the
//!   importer overlays them onto the derived connections, so signalized SUMO
//!   networks drive RNE stop-line control
//! - SUMO coordinates are `x` = east and `y` = north; the RNE Y-up frame maps
//!   them to `[x, z, -y]`
//!
//! SUMO `junction` elements are intentionally not parsed: junctions and
//! connections are derived deterministically from lane geometry, and signal
//! groups are matched to those derived connections by lane pair.

#![deny(missing_docs)]

use rne_traffic::{
    build_traffic_topology, Accuracy, AccuracyClass, AuthorityClass, CoordinateFrame, Lane,
    LaneKind, Provenance, SignalAspect, SignalGroup, SignalGroupAspect, SignalPhase, SignalProgram,
    SourceReference, TopologyBuildConfig, TrafficActorKind, TrafficAsset, TrafficId,
    TrafficNetwork, TrafficSignal,
};
use roxmltree::{Document, Node};
use std::collections::BTreeSet;
use std::fs;
use std::path::Path;

/// Current importer feature set version.
pub const SUMO_IMPORT_VERSION: u32 = 1;

/// Counts describing one completed SUMO import.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SumoImportStats {
    /// Number of lanes converted from `edge`/`lane` elements.
    pub lane_count: usize,
    /// Number of junctions derived by the topology builder.
    pub junction_count: usize,
    /// Number of lane connections derived by the topology builder.
    pub connection_count: usize,
}

/// Imported lane network before topology derivation.
#[derive(Clone, Debug, PartialEq)]
pub struct LaneNetworkImport {
    /// Lane-only network in the RNE Y-up frame.
    pub network: TrafficNetwork,
    /// Import counts for the parsed lanes.
    pub stats: SumoImportStats,
    /// Optional SUMO connections and fixed-time signal programs.
    pub signal_plan: SumoSignalPlan,
}

/// One SUMO `connection` element (a directed lane-to-lane movement).
#[derive(Clone, Debug, PartialEq)]
pub struct SumoConnection {
    /// Incoming lane id as written in the `.net.xml` (for example
    /// `northbound_0`).
    pub incoming_lane_id: String,
    /// Outgoing lane id as written in the `.net.xml`.
    pub outgoing_lane_id: String,
    /// `tlLogic` id controlling this connection, if any.
    pub tl: Option<String>,
    /// Signal link index used to index the `tlLogic` phase states.
    pub link_index: Option<u32>,
}

/// One `phase` inside a SUMO `tlLogic`.
#[derive(Clone, Debug, PartialEq)]
pub struct SumoPhase {
    /// Phase duration in seconds.
    pub duration_s: f64,
    /// Per-link state string, one character per controlled link index.
    pub state: String,
}

/// One SUMO `tlLogic` fixed-time signal program.
#[derive(Clone, Debug, PartialEq)]
pub struct SumoTlLogic {
    /// `tlLogic` id.
    pub id: String,
    /// Ordered phases.
    pub phases: Vec<SumoPhase>,
}

/// SUMO connections and fixed-time signal programs parsed from a `.net.xml`.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SumoSignalPlan {
    /// Parsed `connection` elements.
    pub connections: Vec<SumoConnection>,
    /// Parsed `tlLogic` fixed-time signal programs.
    pub logics: Vec<SumoTlLogic>,
}

/// SUMO `.net.xml` import failure.
#[derive(Debug, thiserror::Error)]
pub enum SumoImportError {
    /// The input file could not be read.
    #[error("read SUMO network {path}: {message}")]
    Io {
        /// File path.
        path: String,
        /// Underlying I/O error.
        message: String,
    },
    /// The input is not valid UTF-8 XML.
    #[error("SUMO network is not valid UTF-8")]
    Utf8,
    /// The input is not well-formed XML.
    #[error("SUMO network XML is invalid: {0}")]
    Xml(String),
    /// The root element is not `net`.
    #[error("expected a SUMO `<net>` root element, got `{0}`")]
    Root(String),
    /// A required element attribute is missing.
    #[error("SUMO `<{element}>` is missing attribute `{attribute}`")]
    MissingAttribute {
        /// Element name.
        element: &'static str,
        /// Attribute name.
        attribute: &'static str,
    },
    /// A numeric attribute could not be parsed.
    #[error("SUMO attribute `{field}` has invalid number `{value}`")]
    InvalidNumber {
        /// Attribute path.
        field: String,
        /// Raw attribute value.
        value: String,
    },
    /// An identifier could not be represented as a stable traffic ID.
    #[error("SUMO identifier could not be represented: {0}")]
    InvalidId(String),
    /// Topology derivation failed.
    #[error("derive SUMO topology: {0}")]
    Topology(String),
    /// The generated asset failed schema validation.
    #[error("validate imported SUMO network: {0}")]
    Asset(String),
}

/// Parses a SUMO `.net.xml` document into a lane-only RNE network.
pub fn parse_sumo_net_xml(bytes: &[u8]) -> Result<LaneNetworkImport, SumoImportError> {
    let text = std::str::from_utf8(bytes).map_err(|_| SumoImportError::Utf8)?;
    let document =
        Document::parse(text).map_err(|error| SumoImportError::Xml(error.to_string()))?;
    let root = document.root_element();
    if root.tag_name().name() != "net" {
        return Err(SumoImportError::Root(root.tag_name().name().into()));
    }
    let coordinate_frame = CoordinateFrame {
        frame_id: "map".into(),
        axis_convention: rne_traffic::AxisConvention::RneYUp,
        origin_m: [0.0, 0.0, 0.0],
        source_crs: root.attribute("projection").map(str::to_string),
    };
    let mut lanes = Vec::new();
    for edge in elements_named(root, "edge") {
        if matches!(edge.attribute("function"), Some("internal" | "connector")) {
            continue;
        }
        let edge_id = required_attribute(edge, "edge", "id")?;
        for lane_node in elements_named(edge, "lane") {
            lanes.push(convert_lane(lane_node, &edge_id)?);
        }
    }
    if lanes.is_empty() {
        return Err(SumoImportError::Asset(
            "SUMO network contains no convertible lanes".into(),
        ));
    }
    let network = TrafficNetwork {
        id: TrafficId::new("sumo:lanes")
            .map_err(|error| SumoImportError::InvalidId(error.to_string()))?,
        provenance: sumo_provenance(None),
        coordinate_frame,
        lanes,
        junctions: Vec::new(),
        connections: Vec::new(),
        signals: Vec::new(),
    };
    let signal_plan = parse_signal_plan(root)?;
    Ok(LaneNetworkImport {
        stats: SumoImportStats {
            lane_count: network.lanes.len(),
            junction_count: 0,
            connection_count: 0,
        },
        network,
        signal_plan,
    })
}

/// Parses a SUMO `.net.xml` file into a lane-only RNE network.
pub fn parse_sumo_net_file(path: &Path) -> Result<LaneNetworkImport, SumoImportError> {
    let bytes = fs::read(path).map_err(|error| SumoImportError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    parse_sumo_net_xml(&bytes)
}

/// Imports a SUMO `.net.xml` document and derives the full traffic asset.
///
/// The lane network is converted into the RNE Y-up frame and then passed to
/// [`rne_traffic::build_traffic_topology`], which deterministically derives
/// junctions and lane connections from the lane endpoints. When the document
/// contains `connection` and `tlLogic` elements, fixed-time signal programs
/// are overlaid onto the derived connections by matching `(incoming, outgoing)`
/// lane pairs, so a signalized SUMO network drives RNE stop-line control.
pub fn import_sumo_net_xml(
    network_id: &TrafficId,
    bytes: &[u8],
) -> Result<TrafficAsset, SumoImportError> {
    let parsed = parse_sumo_net_xml(bytes)?;
    let mut result = build_traffic_topology(
        network_id.clone(),
        &[parsed.network],
        TopologyBuildConfig::default(),
    )
    .map_err(|error| SumoImportError::Topology(error.to_string()))?;
    result.network.signals = apply_signal_plan(&mut result.network, &parsed.signal_plan)?;
    let asset = TrafficAsset::new(result.network).canonicalized();
    asset
        .validate()
        .map_err(|error| SumoImportError::Asset(error.to_string()))?;
    Ok(asset)
}

/// Imports a SUMO `.net.xml` file and derives the full traffic asset.
pub fn import_sumo_net_file(
    network_id: &TrafficId,
    path: &Path,
) -> Result<TrafficAsset, SumoImportError> {
    let bytes = fs::read(path).map_err(|error| SumoImportError::Io {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    import_sumo_net_xml(network_id, &bytes)
}

fn convert_lane<'a, 'b>(lane_node: Node<'a, 'b>, edge_id: &str) -> Result<Lane, SumoImportError> {
    let lane_id = required_attribute(lane_node, "lane", "id")?;
    let shape = required_attribute(lane_node, "lane", "shape")?;
    let centerline_m = parse_shape(&shape)?;
    if centerline_m.len() < 2 {
        return Err(SumoImportError::InvalidNumber {
            field: format!("lane {lane_id} shape"),
            value: shape,
        });
    }
    let width_m = parse_f64_attribute(lane_node, "width")?.unwrap_or(3.2);
    let speed_limit_m_s = parse_f64_attribute(lane_node, "speed")?;
    let (kind, allowed_actors) = classify_lane(
        lane_node.attribute("allow").unwrap_or("all"),
        lane_node.attribute("disallow"),
    );
    Ok(Lane {
        id: stable_lane_id(&lane_id)?,
        provenance: sumo_provenance(Some(&lane_id)),
        kind,
        allowed_actors,
        centerline_m,
        width_m,
        speed_limit_m_s,
        road_class: Some(edge_id.to_string()),
        road_functions: Vec::new(),
    })
}

/// Converts a SUMO east/north lane id into a stable RNE traffic id.
fn stable_lane_id(lane_id: &str) -> Result<TrafficId, SumoImportError> {
    let sanitized = lane_id
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric()
                || matches!(character, '-' | '_' | '.' | '~' | ':' | '/' | '#')
            {
                character
            } else {
                '_'
            }
        })
        .collect::<String>();
    TrafficId::new(format!("sumo:{sanitized}"))
        .map_err(|error| SumoImportError::InvalidId(error.to_string()))
}

/// Parses a SUMO shape into RNE Y-up frame points `[x, z, -y]`.
fn parse_shape(shape: &str) -> Result<Vec<[f64; 3]>, SumoImportError> {
    shape
        .split_whitespace()
        .map(|token| {
            let mut parts = token.split(',');
            let x = parts
                .next()
                .ok_or_else(|| invalid_shape(shape))?
                .parse::<f64>()
                .map_err(|_| invalid_shape(shape))?;
            let y = parts
                .next()
                .ok_or_else(|| invalid_shape(shape))?
                .parse::<f64>()
                .map_err(|_| invalid_shape(shape))?;
            let z = parts
                .next()
                .map(|value| value.parse::<f64>())
                .transpose()
                .map_err(|_| invalid_shape(shape))?
                .unwrap_or(0.0);
            Ok([x, z, -y])
        })
        .collect()
}

fn invalid_shape(shape: &str) -> SumoImportError {
    SumoImportError::InvalidNumber {
        field: "lane shape".into(),
        value: shape.to_string(),
    }
}

/// Maps SUMO `allow`/`disallow` classes onto RNE lane semantics.
fn classify_lane(allow: &str, disallow: Option<&str>) -> (LaneKind, Vec<TrafficActorKind>) {
    let allowed: BTreeSet<&str> = allow
        .split(',')
        .map(str::trim)
        .filter(|token| !token.is_empty())
        .collect();
    let disallowed: BTreeSet<&str> = disallow
        .unwrap_or("")
        .split(',')
        .map(str::trim)
        .filter(|token| !token.is_empty())
        .collect();
    let mut actors = Vec::new();
    let mut add = |actor: TrafficActorKind, token: &str| {
        if !disallowed.contains(token) && !actors.contains(&actor) {
            actors.push(actor);
        }
    };
    if allowed.is_empty() || allowed.contains("all") {
        add(TrafficActorKind::MotorVehicle, "private");
        add(TrafficActorKind::MotorVehicle, "passenger");
        add(TrafficActorKind::MotorVehicle, "vehicle");
    } else {
        for token in &allowed {
            match *token {
                "pedestrian" => add(TrafficActorKind::Pedestrian, "pedestrian"),
                "bicycle" => add(TrafficActorKind::Bicycle, "bicycle"),
                _ => add(TrafficActorKind::MotorVehicle, "private"),
            }
        }
    }
    if actors.is_empty() {
        actors.push(TrafficActorKind::MotorVehicle);
    }
    let kind = if actors == [TrafficActorKind::Pedestrian] {
        LaneKind::Sidewalk
    } else if actors == [TrafficActorKind::Bicycle] {
        LaneKind::Bicycle
    } else {
        LaneKind::Driving
    };
    (kind, actors)
}

fn sumo_provenance(feature_id: Option<&str>) -> Provenance {
    Provenance {
        authority: AuthorityClass::Authoritative,
        accuracy: Accuracy {
            class: AccuracyClass::Modeled,
            horizontal_m: None,
            vertical_m: None,
        },
        sources: vec![SourceReference {
            dataset: "SUMO .net.xml".into(),
            feature_id: feature_id.map(str::to_string),
            uri: None,
        }],
        method: Some("SUMO road network import".into()),
    }
}

/// Parses SUMO `connection` and `tlLogic` elements into a signal plan.
fn parse_signal_plan(root: Node) -> Result<SumoSignalPlan, SumoImportError> {
    let mut connections = Vec::new();
    for connection in elements_named(root, "connection") {
        let from = required_attribute(connection, "connection", "from")?;
        let from_lane = required_attribute(connection, "connection", "fromLane")?;
        let to = required_attribute(connection, "connection", "to")?;
        let to_lane = required_attribute(connection, "connection", "toLane")?;
        let link_index = match connection.attribute("linkIndex") {
            Some(value) => {
                Some(
                    value
                        .parse::<u32>()
                        .map_err(|_| SumoImportError::InvalidNumber {
                            field: "connection@linkIndex".into(),
                            value: value.to_string(),
                        })?,
                )
            }
            None => None,
        };
        connections.push(SumoConnection {
            incoming_lane_id: format!("{from}_{from_lane}"),
            outgoing_lane_id: format!("{to}_{to_lane}"),
            tl: connection.attribute("tl").map(str::to_string),
            link_index,
        });
    }
    let mut logics = Vec::new();
    for logic in elements_named(root, "tlLogic") {
        let id = required_attribute(logic, "tlLogic", "id")?;
        let mut phases = Vec::new();
        for phase in elements_named(logic, "phase") {
            let duration_s = parse_f64_attribute(phase, "duration")?.ok_or(
                SumoImportError::MissingAttribute {
                    element: "phase",
                    attribute: "duration",
                },
            )?;
            let state = required_attribute(phase, "phase", "state")?;
            phases.push(SumoPhase { duration_s, state });
        }
        logics.push(SumoTlLogic { id, phases });
    }
    Ok(SumoSignalPlan {
        connections,
        logics,
    })
}

/// Maps a SUMO phase-state character to an RNE signal aspect.
fn aspect_from_char(character: char) -> SignalAspect {
    match character {
        'G' | 'g' => SignalAspect::Green,
        'y' | 'Y' => SignalAspect::Yellow,
        'r' | 'R' => SignalAspect::Red,
        _ => SignalAspect::Off,
    }
}

/// Overlays fixed-time signal programs onto derived connections.
///
/// SUMO connections are matched to the derived connections by `(incoming,
/// outgoing)` lane pairs; each `tlLogic` becomes a [`rne_traffic::TrafficSignal`]
/// with one group per matched link index and one phase per parsed phase. The
/// phase-state character at a group's link index becomes the group's aspect.
/// Unmatched connections stay unsignaled.
fn apply_signal_plan(
    network: &mut TrafficNetwork,
    plan: &SumoSignalPlan,
) -> Result<Vec<rne_traffic::TrafficSignal>, SumoImportError> {
    if plan.logics.is_empty() {
        return Ok(Vec::new());
    }
    let mut connection_by_pair = std::collections::BTreeMap::new();
    for (index, connection) in network.connections.iter().enumerate() {
        connection_by_pair.insert(
            (
                connection.incoming_lane_id.as_str().to_string(),
                connection.outgoing_lane_id.as_str().to_string(),
            ),
            index,
        );
    }

    let mut signals = Vec::new();
    for logic in &plan.logics {
        // Group matched plan connections by link index.
        let mut links = std::collections::BTreeMap::<u32, Vec<usize>>::new();
        for connection in &plan.connections {
            if connection.tl.as_deref() != Some(logic.id.as_str()) {
                continue;
            }
            let Some(link_index) = connection.link_index else {
                continue;
            };
            let incoming = stable_lane_id(&connection.incoming_lane_id)?;
            let outgoing = stable_lane_id(&connection.outgoing_lane_id)?;
            if let Some(index) = connection_by_pair
                .get(&(incoming.as_str().to_string(), outgoing.as_str().to_string()))
                .copied()
            {
                links.entry(link_index).or_default().push(index);
            }
        }
        if links.is_empty() {
            continue;
        }

        // One signal group per matched link index.
        let signal_id = stable_signal_id(&logic.id)?;
        let mut groups = Vec::new();
        let mut group_by_link = std::collections::BTreeMap::new();
        for (link_index, connection_indices) in &links {
            let mut group_connections = connection_indices
                .iter()
                .map(|index| network.connections[*index].id.clone())
                .collect::<Vec<_>>();
            group_connections.sort();
            group_connections.dedup();
            let group_id = TrafficId::new(format!("sumo:{}:group:{link_index}", logic.id))
                .map_err(|error| SumoImportError::InvalidId(error.to_string()))?;
            group_by_link.insert(*link_index, group_id.clone());
            groups.push(SignalGroup {
                id: group_id,
                connection_ids: group_connections,
            });
        }

        // Wire the matched connections to their groups.
        for (link_index, connection_indices) in &links {
            let group_id = &group_by_link[link_index];
            for index in connection_indices {
                network.connections[*index].signal_group_id = Some(group_id.clone());
            }
        }

        // One phase per parsed phase, with an aspect for every group.
        let mut phases = Vec::new();
        for (phase_index, phase) in logic.phases.iter().enumerate() {
            let mut group_aspects = Vec::new();
            for (link_index, group_id) in &group_by_link {
                let aspect = phase
                    .state
                    .chars()
                    .nth(*link_index as usize)
                    .map(aspect_from_char)
                    .unwrap_or(SignalAspect::Off);
                group_aspects.push(SignalGroupAspect {
                    group_id: group_id.clone(),
                    aspect,
                });
            }
            phases.push(SignalPhase {
                id: TrafficId::new(format!("sumo:{}:phase:{phase_index}", logic.id))
                    .map_err(|error| SumoImportError::InvalidId(error.to_string()))?,
                duration_s: phase.duration_s,
                group_aspects,
            });
        }

        let first_link = links.values().next().expect("non-empty links");
        let junction_id = network.connections[first_link[0]].junction_id.clone();
        signals.push(TrafficSignal {
            id: signal_id,
            provenance: Provenance {
                authority: AuthorityClass::Authoritative,
                accuracy: Accuracy {
                    class: AccuracyClass::Modeled,
                    horizontal_m: None,
                    vertical_m: None,
                },
                sources: vec![SourceReference {
                    dataset: "SUMO .net.xml".into(),
                    feature_id: Some(logic.id.clone()),
                    uri: None,
                }],
                method: Some("SUMO signal program import".into()),
            },
            junction_id,
            position_m: None,
            facing_yaw_rad: None,
            groups,
            program: Some(SignalProgram {
                provenance: Provenance {
                    authority: AuthorityClass::Authoritative,
                    accuracy: Accuracy {
                        class: AccuracyClass::Modeled,
                        horizontal_m: None,
                        vertical_m: None,
                    },
                    sources: vec![SourceReference {
                        dataset: "SUMO .net.xml".into(),
                        feature_id: Some(logic.id.clone()),
                        uri: None,
                    }],
                    method: Some("SUMO signal program import".into()),
                },
                offset_s: 0.0,
                phases,
            }),
        });
    }
    signals.sort_by(|left, right| left.id.cmp(&right.id));
    Ok(signals)
}

fn stable_signal_id(logic_id: &str) -> Result<TrafficId, SumoImportError> {
    let sanitized = logic_id
        .chars()
        .map(|character| {
            if character.is_ascii_alphanumeric()
                || matches!(character, '-' | '_' | '.' | '~' | ':' | '/' | '#')
            {
                character
            } else {
                '_'
            }
        })
        .collect::<String>();
    TrafficId::new(format!("sumo:{sanitized}"))
        .map_err(|error| SumoImportError::InvalidId(error.to_string()))
}

fn required_attribute<'a, 'b>(
    node: Node<'a, 'b>,
    element: &'static str,
    attribute: &'static str,
) -> Result<String, SumoImportError> {
    node.attribute(attribute)
        .map(str::to_string)
        .ok_or(SumoImportError::MissingAttribute { element, attribute })
}

fn parse_f64_attribute<'a, 'b>(
    node: Node<'a, 'b>,
    attribute: &str,
) -> Result<Option<f64>, SumoImportError> {
    let Some(value) = node.attribute(attribute) else {
        return Ok(None);
    };
    value
        .parse::<f64>()
        .map(Some)
        .map_err(|_| SumoImportError::InvalidNumber {
            field: format!("{}[{}]", node.tag_name().name(), attribute),
            value: value.to_string(),
        })
}

fn elements_named<'a, 'b>(node: Node<'a, 'b>, name: &str) -> Vec<Node<'a, 'b>> {
    node.children()
        .filter(|child| child.is_element() && child.tag_name().name() == name)
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_traffic::{LaneKind, SignalAspect, TrafficActorKind};

    const FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../assets/networks/minimal_cross.net.xml"
    );

    const SIGNALIZED_FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../assets/networks/signalized_cross.net.xml"
    );

    #[test]
    fn imports_signal_programs_and_wires_connections() {
        let id = TrafficId::new("sumo:signalized").expect("network id");
        let asset = import_sumo_net_file(&id, Path::new(SIGNALIZED_FIXTURE)).expect("import");
        asset.validate().expect("asset stays valid");

        assert_eq!(asset.network.signals.len(), 1);
        let signal = &asset.network.signals[0];
        assert_eq!(signal.id.as_str(), "sumo:0");
        assert_eq!(signal.groups.len(), 7, "one group per matched link index");

        let group_0 = signal
            .groups
            .iter()
            .find(|group| group.id.as_str().ends_with(":group:0"))
            .expect("link group 0");
        let wired = asset
            .network
            .connections
            .iter()
            .find(|connection| connection.signal_group_id.as_ref() == Some(&group_0.id))
            .expect("wired connection");
        assert_eq!(wired.incoming_lane_id.as_str(), "sumo:northbound_0");
        assert_eq!(wired.outgoing_lane_id.as_str(), "sumo:southbound_0");

        let program = signal.program.as_ref().expect("signal program");
        assert_eq!(program.phases.len(), 2);
        let phase_0 = &program.phases[0];
        assert_eq!(phase_0.duration_s, 20.0);
        let group_4 = signal
            .groups
            .iter()
            .find(|group| group.id.as_str().ends_with(":group:4"))
            .expect("link group 4");
        assert_eq!(
            phase_0
                .group_aspects
                .iter()
                .find(|aspect| aspect.group_id == group_0.id)
                .expect("group 0 aspect")
                .aspect,
            SignalAspect::Green
        );
        assert_eq!(
            phase_0
                .group_aspects
                .iter()
                .find(|aspect| aspect.group_id == group_4.id)
                .expect("group 4 aspect")
                .aspect,
            SignalAspect::Red
        );
        assert_eq!(
            program.phases[1]
                .group_aspects
                .iter()
                .find(|aspect| aspect.group_id == group_4.id)
                .expect("group 4 aspect")
                .aspect,
            SignalAspect::Green
        );
    }

    #[test]
    fn unsignalized_networks_import_without_signals() {
        let id = TrafficId::new("sumo:plain").expect("network id");
        let asset = import_sumo_net_file(&id, Path::new(FIXTURE)).expect("import");
        assert!(asset.network.signals.is_empty());
    }

    #[test]
    fn parses_lanes_into_the_rne_y_up_frame() {
        let parsed = parse_sumo_net_file(Path::new(FIXTURE)).expect("parse fixture");
        assert_eq!(parsed.network.lanes.len(), 8);
        assert_eq!(parsed.stats.lane_count, 8);

        let northbound = parsed
            .network
            .lanes
            .iter()
            .find(|lane| lane.id.as_str() == "sumo:northbound_0")
            .expect("northbound_0");
        assert_eq!(
            northbound.centerline_m,
            vec![[200.0, 0.0, -300.0], [200.0, 0.0, -200.0]]
        );
        assert_eq!(northbound.width_m, 3.5);
        assert_eq!(northbound.speed_limit_m_s, Some(13.89));
        assert_eq!(northbound.kind, LaneKind::Driving);
        assert_eq!(
            northbound.allowed_actors,
            vec![TrafficActorKind::MotorVehicle]
        );

        let pedestrian = parsed
            .network
            .lanes
            .iter()
            .find(|lane| lane.id.as_str() == "sumo:pedestrian_path_0")
            .expect("pedestrian lane");
        assert_eq!(pedestrian.kind, LaneKind::Sidewalk);
        assert_eq!(
            pedestrian.allowed_actors,
            vec![TrafficActorKind::Pedestrian]
        );

        let bicycle = parsed
            .network
            .lanes
            .iter()
            .find(|lane| lane.id.as_str() == "sumo:bike_path_0")
            .expect("bicycle lane");
        assert_eq!(bicycle.kind, LaneKind::Bicycle);
        assert_eq!(bicycle.allowed_actors, vec![TrafficActorKind::Bicycle]);
    }

    #[test]
    fn derives_a_cross_intersection_from_the_lane_network() {
        let id = TrafficId::new("sumo:minimal_cross").expect("network id");
        let asset = import_sumo_net_xml(&id, &fs::read(FIXTURE).expect("read fixture"))
            .expect("import fixture");
        assert_eq!(asset.network.lanes.len(), 8);
        assert!(
            !asset.network.junctions.is_empty(),
            "the lane endpoints must cluster into junctions"
        );
        assert!(
            asset.network.connections.len() >= 4,
            "the approaches must connect through the junction"
        );
        asset.validate().expect("asset stays valid");
    }

    #[test]
    fn rejects_malformed_input() {
        let error = parse_sumo_net_xml(b"<not-a-net/>").expect_err("root must be net");
        assert!(matches!(error, SumoImportError::Root(_)));

        let error = parse_sumo_net_xml(b"<net>").expect_err("bad xml must be rejected");
        assert!(matches!(error, SumoImportError::Xml(_)));

        let error = parse_sumo_net_xml(
            b"<net><edge id=\"e\"><lane id=\"e_0\" shape=\"x,y\"/></edge></net>",
        )
        .expect_err("bad shape must be rejected");
        assert!(matches!(error, SumoImportError::InvalidNumber { .. }));
    }

    #[test]
    fn skips_internal_and_connector_edges() {
        let xml = br#"<net>
            <edge id="e0"><lane id="e0_0" shape="0,0 10,0"/></edge>
            <edge id="e1" function="internal"><lane id="e1_0" shape="0,0 10,0"/></edge>
            <edge id="e2" function="connector"><lane id="e2_0" shape="0,0 10,0"/></edge>
        </net>"#;
        let parsed = parse_sumo_net_xml(xml).expect("parse");
        assert_eq!(parsed.network.lanes.len(), 1);
        assert_eq!(parsed.network.lanes[0].id.as_str(), "sumo:e0_0");
    }
}

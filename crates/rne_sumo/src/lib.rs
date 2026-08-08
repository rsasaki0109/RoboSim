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
//! - SUMO coordinates are `x` = east and `y` = north; the RNE Y-up frame maps
//!   them to `[x, z, -y]`
//!
//! SUMO `junction`, `connection`, and `tlLogic` elements are intentionally not
//! parsed: junctions and connections are derived deterministically, and signal
//! programs are out of scope for this minimal importer.

#![deny(missing_docs)]

use rne_traffic::{
    build_traffic_topology, Accuracy, AccuracyClass, AuthorityClass, CoordinateFrame, Lane,
    LaneKind, Provenance, SourceReference, TopologyBuildConfig, TrafficActorKind, TrafficAsset,
    TrafficId, TrafficNetwork,
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
    Ok(LaneNetworkImport {
        stats: SumoImportStats {
            lane_count: network.lanes.len(),
            junction_count: 0,
            connection_count: 0,
        },
        network,
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
/// junctions and lane connections from the lane endpoints.
pub fn import_sumo_net_xml(
    network_id: &TrafficId,
    bytes: &[u8],
) -> Result<TrafficAsset, SumoImportError> {
    let parsed = parse_sumo_net_xml(bytes)?;
    let result = build_traffic_topology(
        network_id.clone(),
        &[parsed.network],
        TopologyBuildConfig::default(),
    )
    .map_err(|error| SumoImportError::Topology(error.to_string()))?;
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
    use rne_traffic::{LaneKind, TrafficActorKind};

    const FIXTURE: &str = concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../assets/networks/minimal_cross.net.xml"
    );

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

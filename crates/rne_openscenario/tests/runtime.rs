//! Scenario execution over the traffic runtime integration tests.

use rne_openscenario::{
    execute_scenario, parse_openscenario_xml_with_source, ScenarioDocument, ScenarioEntityKind,
    ScenarioRunOptions,
};
use rne_traffic::{
    Accuracy, AccuracyClass, AuthorityClass, AxisConvention, CoordinateFrame, Junction,
    JunctionKind, Lane, LaneKind, MovementKind, Provenance, TrafficActorKind, TrafficConnection,
    TrafficId, TrafficNetwork, TrafficSignal,
};
use std::fs;
use std::path::Path;

const FIXTURE_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("stable ID")
}

fn synthetic(feature_id: &str) -> Provenance {
    Provenance {
        authority: AuthorityClass::Synthetic,
        accuracy: Accuracy {
            class: AccuracyClass::ScenarioAuthored,
            horizontal_m: None,
            vertical_m: None,
        },
        sources: Vec::new(),
        method: Some(feature_id.to_string()),
    }
}

fn corridor_network() -> TrafficNetwork {
    TrafficNetwork {
        id: id("runtime:corridor"),
        provenance: synthetic("runtime-corridor"),
        coordinate_frame: CoordinateFrame {
            frame_id: "map".into(),
            axis_convention: AxisConvention::RneYUp,
            origin_m: [0.0, 0.0, 0.0],
            source_crs: Some("local".into()),
        },
        lanes: vec![
            Lane {
                id: id("corridor:west"),
                provenance: synthetic("corridor-west"),
                kind: LaneKind::Driving,
                allowed_actors: vec![TrafficActorKind::MotorVehicle],
                centerline_m: vec![[-20.0, 0.0, 1.75], [-4.0, 0.0, 1.75]],
                width_m: 3.25,
                speed_limit_m_s: Some(15.0),
                road_class: Some("道路".into()),
                road_functions: vec!["一般道路".into()],
            },
            Lane {
                id: id("corridor:east"),
                provenance: synthetic("corridor-east"),
                kind: LaneKind::Driving,
                allowed_actors: vec![TrafficActorKind::MotorVehicle],
                centerline_m: vec![[4.0, 0.0, 1.75], [20.0, 0.0, 1.75]],
                width_m: 3.25,
                speed_limit_m_s: Some(15.0),
                road_class: Some("道路".into()),
                road_functions: vec!["一般道路".into()],
            },
        ],
        junctions: vec![Junction {
            id: id("corridor:junction"),
            provenance: synthetic("corridor-junction"),
            kind: JunctionKind::CrossIntersection,
            center_m: [0.0, 0.0, 0.0],
        }],
        connections: vec![TrafficConnection {
            id: id("corridor:connect-west-east"),
            provenance: synthetic("corridor-connect"),
            incoming_lane_id: id("corridor:west"),
            outgoing_lane_id: id("corridor:east"),
            junction_id: Some(id("corridor:junction")),
            movement: MovementKind::Straight,
            path_m: vec![[-4.0, 0.0, 1.75], [0.0, 0.0, 1.75], [4.0, 0.0, 1.75]],
            conflict_connection_ids: Vec::new(),
            signal_group_id: None,
        }],
        signals: Vec::<TrafficSignal>::new(),
    }
}

fn scenario() -> ScenarioDocument {
    let text = fs::read_to_string(Path::new(FIXTURE_DIR).join("runtime_speed.xosc"))
        .expect("read fixture");
    parse_openscenario_xml_with_source("runtime_speed.xosc", &text).expect("parse scenario")
}

#[test]
fn executes_speed_scenario_deterministically() {
    let document = scenario();
    let network = corridor_network();
    let options = ScenarioRunOptions {
        steps: 300,
        hz: 60.0,
    };

    let first = execute_scenario(&document, &network, &options).expect("run scenario");
    let replay = execute_scenario(&document, &network, &options).expect("rerun scenario");

    assert_eq!(first, replay, "scenario execution must be deterministic");
    assert_eq!(first.final_positions_m.len(), 1);
    assert_eq!(first.collisions, 0);
    assert_eq!(first.signal_violations, 0);
    assert!(first.route_length_m > 0.0);
    assert!(
        first.average_speed_m_s > 0.0,
        "the speed action should move the ego forward"
    );
    let final_x = first.final_positions_m[0][0];
    assert!(final_x > 0.0, "ego should travel east along the corridor");
    assert_ne!(first.stable_hash, 0);
}

#[test]
fn rejects_unsupported_entity_kind_route() {
    let mut document = scenario();
    document.entities[0].kind = ScenarioEntityKind::Pedestrian;
    let network = corridor_network();
    let error = execute_scenario(&document, &network, &ScenarioRunOptions::default())
        .expect_err("pedestrians must not route on a motor-only corridor");
    assert!(error.to_string().contains("no lanes that allow"));
}

#[test]
fn empty_scenario_is_rejected() {
    let mut document = scenario();
    document.entities.clear();
    document.actions.clear();
    let error = execute_scenario(
        &document,
        &corridor_network(),
        &ScenarioRunOptions::default(),
    )
    .expect_err("empty scenario");
    assert!(error.to_string().contains("no entities"));
}

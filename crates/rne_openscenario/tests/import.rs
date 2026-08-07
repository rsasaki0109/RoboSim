//! OpenSCENARIO import integration tests.

use rne_openscenario::{
    parse_openscenario_xml_with_source, ScenarioAction, ScenarioDocument, ScenarioEntityKind,
};
use std::fs;
use std::path::Path;

const FIXTURE_DIR: &str = concat!(env!("CARGO_MANIFEST_DIR"), "/tests/fixtures");

fn fixture(name: &str) -> String {
    fs::read_to_string(Path::new(FIXTURE_DIR).join(name)).expect("read fixture")
}

#[test]
fn imports_minimal_speed_scenario() {
    let document =
        parse_openscenario_xml_with_source("minimal_speed.xosc", &fixture("minimal_speed.xosc"))
            .expect("parse scenario");

    assert_eq!(document.version, 1);
    assert_eq!(document.source, "minimal_speed.xosc");
    assert_eq!(
        document.road_network_logic_file,
        "assets/traffic/sanjo.rne.traffic.json"
    );

    assert_eq!(document.entities.len(), 3);
    let ego = &document.entities[0];
    assert_eq!(ego.name, "ego");
    assert_eq!(ego.kind, ScenarioEntityKind::MotorVehicle);
    assert_eq!(ego.initial_world_position_m, Some([0.0, 0.0, 0.0]));
    let heading_rad = ego.initial_heading_rad.expect("heading");
    assert!((heading_rad - 90.0_f64.to_radians()).abs() < 1e-12);
    assert_eq!(document.entities[1].kind, ScenarioEntityKind::Bicycle);
    assert_eq!(document.entities[2].kind, ScenarioEntityKind::Pedestrian);
    assert!(document.entities[1].initial_world_position_m.is_none());

    assert_eq!(document.actions.len(), 1);
    let action = &document.actions[0];
    assert_eq!(action.entity, "ego");
    assert_eq!(action.start_time_s, 2.0);
    assert_eq!(
        action.action,
        ScenarioAction::AbsoluteSpeed { target_m_s: 10.0 }
    );
}

#[test]
fn scenario_document_roundtrips_json() {
    let document =
        parse_openscenario_xml_with_source("minimal_speed.xosc", &fixture("minimal_speed.xosc"))
            .expect("parse scenario");

    let json = document.to_json().expect("serialize");
    let loaded = ScenarioDocument::from_json(&json).expect("parse json");
    assert_eq!(loaded, document);
}

#[test]
fn canonical_json_is_stable() {
    let document =
        parse_openscenario_xml_with_source("minimal_speed.xosc", &fixture("minimal_speed.xosc"))
            .expect("parse scenario");
    let actual = document.to_json().expect("serialize");
    let expected = fs::read_to_string(Path::new(FIXTURE_DIR).join("minimal_speed.scenario.json"))
        .expect("read golden")
        .replace("\r\n", "\n");
    assert_eq!(actual.trim_end(), expected.trim_end());
}

#[test]
fn rejects_unsupported_revision() {
    let text = fixture("minimal_speed.xosc").replace(
        "revMajor=\"1\" revMinor=\"0\"",
        "revMajor=\"1\" revMinor=\"1\"",
    );
    let error =
        parse_openscenario_xml_with_source("unsupported.xosc", &text).expect_err("revision");
    assert!(error
        .to_string()
        .contains("unsupported OpenSCENARIO revision"));
}

#[test]
fn rejects_action_for_unknown_entity() {
    let text = fixture("minimal_speed.xosc").replace("entityRef=\"ego\"", "entityRef=\"ghost\"");
    let error = parse_openscenario_xml_with_source("bad.xosc", &text).expect_err("unknown entity");
    assert!(error.to_string().contains("unknown entity"));
}

#[test]
fn imports_lane_change_scenario() {
    use rne_openscenario::ScenarioAction;

    let text =
        fs::read_to_string(Path::new(FIXTURE_DIR).join("lane_change.xosc")).expect("read fixture");
    let document =
        parse_openscenario_xml_with_source("lane_change.xosc", &text).expect("parse scenario");

    assert_eq!(document.actions.len(), 2);
    assert!(matches!(
        document.actions[0].action,
        ScenarioAction::AbsoluteSpeed { target_m_s: 5.0 }
    ));
    assert!(matches!(
        document.actions[1].action,
        ScenarioAction::LaneChange {
            target_lane_offset: 1
        }
    ));
    assert_eq!(document.actions[1].start_time_s, 1.0);
}

#[test]
fn imports_assigned_route_action() {
    use rne_openscenario::ScenarioAction;

    let text = fs::read_to_string(Path::new(FIXTURE_DIR).join("assigned_route.xosc"))
        .expect("read fixture");
    let document =
        parse_openscenario_xml_with_source("assigned_route.xosc", &text).expect("parse scenario");

    let route_action = document
        .actions
        .iter()
        .find(|action| matches!(action.action, ScenarioAction::AssignRoute { .. }))
        .expect("assign route action");
    match &route_action.action {
        ScenarioAction::AssignRoute { waypoints } => {
            assert_eq!(waypoints.len(), 2);
            assert_eq!(waypoints[0], [0.0, 0.0, 0.0]);
            assert_eq!(waypoints[1], [0.0, 0.0, 30.0]);
        }
        other => panic!("expected assign route action, got {other:?}"),
    }
}

#[test]
fn substitutes_parameter_declarations() {
    use rne_openscenario::ScenarioAction;

    let text =
        fs::read_to_string(Path::new(FIXTURE_DIR).join("parameters.xosc")).expect("read fixture");
    let document =
        parse_openscenario_xml_with_source("parameters.xosc", &text).expect("parse scenario");

    assert_eq!(document.actions.len(), 1);
    assert!(matches!(
        document.actions[0].action,
        ScenarioAction::AbsoluteSpeed { target_m_s: 10.0 }
    ));
    assert_eq!(document.actions[0].start_time_s, 2.0);
}

#[test]
fn rejects_missing_road_network() {
    let text = fixture("minimal_speed.xosc").replace(
        "<LogicFile filepath=\"assets/traffic/sanjo.rne.traffic.json\"/>",
        "",
    );
    let error = parse_openscenario_xml_with_source("bad.xosc", &text).expect_err("road network");
    assert!(error.to_string().contains("LogicFile"));
}

//! Scenario execution over the traffic runtime integration tests.

use rne_core::control::{ControlCommand, RunControl, RunnerControl};
use rne_openscenario::{
    execute_scenario, execute_scenario_with_control, parse_openscenario_xml_with_source,
    ScenarioAction, ScenarioDocument, ScenarioEntity, ScenarioEntityKind, ScenarioRunOptions,
    ScenarioTimedAction,
};
use rne_traffic::{
    Accuracy, AccuracyClass, AuthorityClass, AxisConvention, CoordinateFrame, Junction,
    JunctionKind, Lane, LaneKind, MovementKind, Provenance, SignalAspect, SignalGroup,
    SignalGroupAspect, SignalPhase, SignalProgram, TrafficActorKind, TrafficConnection, TrafficId,
    TrafficNetwork, TrafficSignal,
};
use std::collections::VecDeque;
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

fn entity(name: &str, kind: ScenarioEntityKind, position_m: [f64; 3]) -> ScenarioEntity {
    ScenarioEntity {
        name: name.to_string(),
        kind,
        initial_world_position_m: Some(position_m),
        initial_heading_rad: Some(0.0),
    }
}

fn action(entity: &str, action: ScenarioAction) -> ScenarioTimedAction {
    ScenarioTimedAction {
        entity: entity.to_string(),
        start_time_s: 0.0,
        action,
    }
}

struct ScriptedControl {
    commands: VecDeque<ControlCommand>,
    status_steps: Vec<u64>,
}

impl RunnerControl for ScriptedControl {
    fn try_poll(&mut self) -> Option<ControlCommand> {
        self.commands.pop_front()
    }

    fn wait_command(&mut self) -> ControlCommand {
        self.commands.pop_front().unwrap_or(ControlCommand::Quit)
    }

    fn report_status(&mut self, step: u64, _sim_time_s: f64, _snapshot: &[u8]) {
        self.status_steps.push(step);
    }
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
fn multi_kind_execution_is_entity_order_independent() {
    let mut network = corridor_network();
    network.lanes.push(Lane {
        id: id("corridor:walk"),
        provenance: synthetic("corridor-walk"),
        kind: LaneKind::Sidewalk,
        allowed_actors: vec![TrafficActorKind::Pedestrian],
        centerline_m: vec![[-20.0, 0.0, 10.0], [20.0, 0.0, 10.0]],
        width_m: 2.0,
        speed_limit_m_s: Some(2.0),
        road_class: None,
        road_functions: Vec::new(),
    });
    let ego = entity("ego", ScenarioEntityKind::MotorVehicle, [-18.0, 0.0, 1.75]);
    let walker = entity("walker", ScenarioEntityKind::Pedestrian, [-18.0, 0.0, 10.0]);
    let ego_speed = action("ego", ScenarioAction::AbsoluteSpeed { target_m_s: 5.0 });
    let walker_speed = action("walker", ScenarioAction::AbsoluteSpeed { target_m_s: 1.0 });
    let forward = ScenarioDocument::new(
        "multi_kind.xosc",
        "network.rne.traffic.json",
        vec![ego.clone(), walker.clone()],
        vec![ego_speed.clone(), walker_speed.clone()],
    );
    let reverse = ScenarioDocument::new(
        "multi_kind.xosc",
        "network.rne.traffic.json",
        vec![walker, ego],
        vec![walker_speed, ego_speed],
    );
    let options = ScenarioRunOptions {
        steps: 120,
        hz: 60.0,
    };

    let forward = execute_scenario(&forward, &network, &options).expect("forward multi-kind run");
    let reverse = execute_scenario(&reverse, &network, &options).expect("reverse multi-kind run");

    assert_eq!(forward, reverse);
    assert_eq!(forward.final_actors.len(), 2);
    assert_eq!(forward.final_actors[0].name, "ego");
    assert_eq!(
        forward.final_actors[0].kind,
        ScenarioEntityKind::MotorVehicle
    );
    assert_eq!(forward.final_actors[1].name, "walker");
    assert_eq!(forward.final_actors[1].kind, ScenarioEntityKind::Pedestrian);
    assert!(forward.final_actors[0].final_speed_m_s > 0.0);
    assert!(forward.final_actors[1].final_speed_m_s > 0.0);
}

#[test]
fn simultaneous_assigned_routes_remain_actor_specific() {
    let entities = vec![
        entity("north", ScenarioEntityKind::MotorVehicle, [0.0, 0.0, 0.0]),
        entity("south", ScenarioEntityKind::MotorVehicle, [10.0, 0.0, 0.0]),
    ];
    let actions = vec![
        action(
            "north",
            ScenarioAction::AssignRoute {
                waypoints: vec![[0.0, 0.0, 0.0], [0.0, 0.0, 30.0]],
            },
        ),
        action("north", ScenarioAction::AbsoluteSpeed { target_m_s: 4.0 }),
        action(
            "south",
            ScenarioAction::AssignRoute {
                waypoints: vec![[10.0, 0.0, 0.0], [10.0, 0.0, -30.0]],
            },
        ),
        action("south", ScenarioAction::AbsoluteSpeed { target_m_s: 4.0 }),
    ];
    let document = ScenarioDocument::new(
        "independent_routes.xosc",
        "network.rne.traffic.json",
        entities,
        actions,
    );
    let result = execute_scenario(
        &document,
        &corridor_network(),
        &ScenarioRunOptions {
            steps: 300,
            hz: 60.0,
        },
    )
    .expect("independent assigned routes");

    assert_eq!(result.final_actors[0].name, "north");
    assert!(result.final_actors[0].final_position_m[2] > 10.0);
    assert_eq!(result.final_actors[1].name, "south");
    assert!(result.final_actors[1].final_position_m[2] < -10.0);
    assert_eq!(result.collisions, 0);
}

#[test]
fn controlled_scenario_supports_step_reset_and_quit() {
    let document = scenario();
    let network = corridor_network();
    let options = ScenarioRunOptions {
        steps: 30,
        hz: 60.0,
    };
    let commands = vec![
        ControlCommand::Step { frames: 5 },
        ControlCommand::Reset,
        ControlCommand::Step { frames: 3 },
        ControlCommand::Quit,
    ];
    let mut transport = ScriptedControl {
        commands: commands.clone().into(),
        status_steps: Vec::new(),
    };
    let (controlled, recorded_commands) = {
        let mut control = RunControl::paused(&mut transport);
        let result =
            execute_scenario_with_control(&document, &network, &options, Some(&mut control))
                .expect("controlled scenario");
        (result, control.recorded_commands().to_vec())
    };
    assert_eq!(recorded_commands, commands);

    let mut replay_transport = ScriptedControl {
        commands: recorded_commands.into(),
        status_steps: Vec::new(),
    };
    let replayed = {
        let mut control = RunControl::paused(&mut replay_transport);
        execute_scenario_with_control(&document, &network, &options, Some(&mut control))
            .expect("replay controlled scenario")
    };

    let baseline = execute_scenario(
        &document,
        &network,
        &ScenarioRunOptions { steps: 3, hz: 60.0 },
    )
    .expect("baseline scenario");
    assert_eq!(
        controlled, baseline,
        "reset must restart deterministic state"
    );
    assert_eq!(
        controlled, replayed,
        "the command transcript must replay exactly"
    );
    assert_eq!(transport.status_steps.len(), 8);
    assert_eq!(&transport.status_steps[..5], &[1, 2, 3, 4, 5]);
    assert_eq!(&transport.status_steps[5..], &[1, 2, 3]);
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

#[test]
fn lane_change_switches_the_actor_to_the_parallel_route() {
    let text =
        fs::read_to_string(Path::new(FIXTURE_DIR).join("lane_change.xosc")).expect("read fixture");
    let document =
        parse_openscenario_xml_with_source("lane_change.xosc", &text).expect("parse scenario");
    assert_eq!(document.actions.len(), 2);

    let options = ScenarioRunOptions {
        steps: 300,
        hz: 60.0,
    };
    let result = execute_scenario(&document, &corridor_network(), &options)
        .expect("run lane-change scenario");

    let final_z = result.final_positions_m[0][2];
    // The corridor sits at z = 1.75; a +1 lane change offsets it one lane width
    // (3.5 m) laterally.
    assert!(
        (final_z - 5.25).abs() < 1.0,
        "lane change should move the ego laterally (got z={final_z})"
    );

    let replay =
        execute_scenario(&document, &corridor_network(), &options).expect("rerun lane-change");
    assert_eq!(result, replay, "lane change must be deterministic");
}

/// Corridor network with a fixed-time signal on the through movement.
///
/// The program holds Red for 4 s then Green for 1 s, cyclically, so an actor
/// arriving before t = 4 s is held at the stop line.
fn signaled_corridor_network() -> TrafficNetwork {
    let mut network = corridor_network();
    let group_id = id("runtime:signal/group-through");
    for connection in &mut network.connections {
        connection.signal_group_id = Some(group_id.clone());
    }
    network.signals = vec![TrafficSignal {
        id: id("runtime:signal-main"),
        provenance: synthetic("runtime-signal"),
        junction_id: Some(id("corridor:junction")),
        position_m: Some([0.0, 0.0, 1.75]),
        facing_yaw_rad: Some(0.0),
        groups: vec![SignalGroup {
            id: group_id.clone(),
            connection_ids: vec![id("corridor:connect-west-east")],
        }],
        program: Some(SignalProgram {
            provenance: synthetic("runtime-signal-program"),
            offset_s: 0.0,
            phases: vec![
                SignalPhase {
                    id: id("runtime:signal/phase-red"),
                    duration_s: 4.0,
                    group_aspects: vec![SignalGroupAspect {
                        group_id: group_id.clone(),
                        aspect: SignalAspect::Red,
                    }],
                },
                SignalPhase {
                    id: id("runtime:signal/phase-green"),
                    duration_s: 1.0,
                    group_aspects: vec![SignalGroupAspect {
                        group_id,
                        aspect: SignalAspect::Green,
                    }],
                },
            ],
        }),
    }];
    network
}

#[test]
fn network_signal_stops_then_releases_the_actor() {
    let text = fs::read_to_string(Path::new(FIXTURE_DIR).join("signaled_speed.xosc"))
        .expect("read fixture");
    let document =
        parse_openscenario_xml_with_source("signaled_speed.xosc", &text).expect("parse scenario");
    let options = ScenarioRunOptions {
        steps: 300,
        hz: 60.0,
    };

    let free =
        execute_scenario(&document, &corridor_network(), &options).expect("run without signal");
    let signaled = execute_scenario(&document, &signaled_corridor_network(), &options)
        .expect("run with signal");

    assert_eq!(
        signaled.signal_violations, 0,
        "the actor must respect the red"
    );
    assert_eq!(signaled.collisions, 0);
    let free_x = free.final_positions_m[0][0];
    let signaled_x = signaled.final_positions_m[0][0];
    assert!(
        signaled_x < free_x,
        "a red phase should delay the actor (free_x={free_x}, signaled_x={signaled_x})"
    );

    let replay = execute_scenario(&document, &signaled_corridor_network(), &options)
        .expect("rerun with signal");
    assert_eq!(signaled, replay, "signal timing must be deterministic");
}

#[test]
fn assigned_route_follows_scripted_waypoints() {
    let text = fs::read_to_string(Path::new(FIXTURE_DIR).join("assigned_route.xosc"))
        .expect("read fixture");
    let document =
        parse_openscenario_xml_with_source("assigned_route.xosc", &text).expect("parse scenario");
    let options = ScenarioRunOptions {
        steps: 300,
        hz: 60.0,
    };

    let result = execute_scenario(&document, &corridor_network(), &options)
        .expect("run assigned-route scenario");

    let final_x = result.final_positions_m[0][0];
    let final_z = result.final_positions_m[0][2];
    assert!(
        final_x.abs() < 0.5,
        "assigned route keeps the actor near x=0 (got x={final_x})"
    );
    assert!(
        final_z > 5.0,
        "the actor should travel along the assigned +z route (got z={final_z})"
    );

    let replay = execute_scenario(&document, &corridor_network(), &options)
        .expect("rerun assigned-route scenario");
    assert_eq!(result, replay, "assigned route must be deterministic");
}

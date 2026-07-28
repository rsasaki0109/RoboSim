//! Runs a renderer-free 100-vehicle urban traffic acceptance replay.

use bevy_ecs::prelude::World;
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use rne_traffic::{
    advance_controlled_kinematic_traffic, shortest_lane_route, Accuracy, AccuracyClass,
    AuthorityClass, AxisConvention, CoordinateFrame, KinematicTrafficConfig, Lane, LaneKind,
    MovementKind, Provenance, SignalAspect, TrafficActor, TrafficActorKind, TrafficConnection,
    TrafficId, TrafficNetwork, TrafficPose, TrafficRoute, TrafficRouteCatalog,
    TrafficRouteFollower, TrafficRuntime, TrafficSignalControl, TrafficSignalControls,
};
use std::time::Instant;
use uuid::Uuid;

const ACTOR_COUNT: usize = 100;
const RED_STEP_COUNT: u64 = 600;
const STEP_COUNT: u64 = 720;

fn main() {
    let planned = shortest_lane_route(
        &routing_network(),
        &id("lane:start"),
        &id("lane:goal"),
        TrafficActorKind::MotorVehicle,
    )
    .expect("deterministic shortest route");
    assert_eq!(
        planned.connection_ids,
        vec![id("connection:left"), id("connection:right")]
    );

    let start = Instant::now();
    let forward = replay(false);
    let elapsed = start.elapsed();
    let reverse = replay(true);
    assert_eq!(forward.stable_hash, reverse.stable_hash);
    assert_eq!(forward.signal_violations, 0);
    assert_eq!(forward.collisions, 0);
    assert!(forward.minimum_gap_m >= 2.0);
    assert!(forward.left_turns > 0 && forward.right_turns > 0);
    let simulated_steps_per_s = STEP_COUNT as f64 / elapsed.as_secs_f64();
    assert!(
        simulated_steps_per_s >= 60.0,
        "headless throughput {simulated_steps_per_s:.1} Hz is below 60 Hz"
    );
    println!(
        "traffic acceptance passed: actors={ACTOR_COUNT} steps={STEP_COUNT} \
         shortest_route_m={:.2} left_turns={} right_turns={} violations=0 collisions=0 \
         minimum_gap_m={:.3} stable_hash={} throughput_hz={simulated_steps_per_s:.1}",
        planned.distance_m,
        forward.left_turns,
        forward.right_turns,
        forward.minimum_gap_m,
        forward.stable_hash,
    );
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ReplayResult {
    stable_hash: u64,
    signal_violations: usize,
    collisions: usize,
    minimum_gap_m: f64,
    left_turns: usize,
    right_turns: usize,
}

fn replay(reverse_spawn_order: bool) -> ReplayResult {
    let route_id = id("route:urban-tile");
    let route = TrafficRoute::new(
        route_id.clone(),
        vec![
            [0.0, 0.0, 0.0],
            [500.0, 0.0, 0.0],
            [500.0, 0.0, 300.0],
            [300.0, 0.0, 300.0],
            [300.0, 0.0, 100.0],
            [100.0, 0.0, 100.0],
            [100.0, 0.0, 300.0],
            [0.0, 0.0, 300.0],
        ],
        true,
    )
    .expect("urban route");
    assert_eq!(route.total_length_m(), 2_000.0);
    let (left_turns, right_turns) = turn_counts(route.path_m());
    let mut routes = TrafficRouteCatalog::default();
    routes.insert(route).expect("insert urban route");
    let signal_id = id("signal:urban-main");
    let mut controls = TrafficSignalControls::default();
    controls
        .insert(TrafficSignalControl {
            id: signal_id.clone(),
            route_id: route_id.clone(),
            stop_distance_m: 450.0,
            aspect: SignalAspect::Red,
        })
        .expect("insert signal");
    let mut world = World::new();
    let indices: Vec<_> = if reverse_spawn_order {
        (0..ACTOR_COUNT).rev().collect()
    } else {
        (0..ACTOR_COUNT).collect()
    };
    for index in indices {
        let distance_m = index as f64 * 20.0;
        let pose = routes
            .get(&route_id)
            .expect("urban route")
            .sample(distance_m);
        world.spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(index as u128 + 1)),
            TrafficRouteFollower {
                route_id: route_id.clone(),
                distance_m,
                speed_m_s: 0.0,
                desired_speed_m_s: 10.0 + (index % 5) as f64 * 0.25,
                length_m: 4.4,
            },
            TrafficPose {
                position_m: pose.position_m,
                yaw_rad: pose.yaw_rad,
            },
        ));
    }
    let delta = SimDuration::from_ticks(16_666_666);
    let mut runtime = TrafficRuntime::default();
    let mut violations = 0;
    let mut collisions = 0;
    let mut minimum_gap_m = f64::INFINITY;
    let mut stable_hash = 0;
    for step in 1..=STEP_COUNT {
        if step == RED_STEP_COUNT + 1 {
            controls
                .set_aspect(&signal_id, SignalAspect::Green)
                .expect("release signal");
        }
        let report = advance_controlled_kinematic_traffic(
            &mut world,
            &routes,
            &controls,
            &mut runtime,
            SimTime::from_ticks(step * delta.ticks()),
            delta,
            KinematicTrafficConfig::default(),
        )
        .expect("traffic replay step");
        violations += report.signal_violation_count;
        collisions += report.collision_count;
        if let Some(gap_m) = report.minimum_observed_gap_m {
            minimum_gap_m = minimum_gap_m.min(gap_m);
        }
        stable_hash = report.stable_state_hash;
    }
    ReplayResult {
        stable_hash,
        signal_violations: violations,
        collisions,
        minimum_gap_m,
        left_turns,
        right_turns,
    }
}

fn turn_counts(path_m: &[[f64; 3]]) -> (usize, usize) {
    let mut left = 0;
    let mut right = 0;
    let mut points = path_m.to_vec();
    points.extend_from_slice(&path_m[..2]);
    for window in points.windows(3) {
        let incoming = [window[1][0] - window[0][0], window[1][2] - window[0][2]];
        let outgoing = [window[2][0] - window[1][0], window[2][2] - window[1][2]];
        let cross_y = incoming[1] * outgoing[0] - incoming[0] * outgoing[1];
        if cross_y > 0.0 {
            left += 1;
        } else if cross_y < 0.0 {
            right += 1;
        }
    }
    (left, right)
}

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("stable ID")
}

fn provenance(feature_id: &str) -> Provenance {
    Provenance {
        authority: AuthorityClass::Synthetic,
        accuracy: Accuracy {
            class: AccuracyClass::ScenarioAuthored,
            horizontal_m: None,
            vertical_m: None,
        },
        sources: Vec::new(),
        method: Some(format!("Example 47 {feature_id}")),
    }
}

fn routing_lane(lane_id: &str, start: [f64; 3], end: [f64; 3]) -> Lane {
    Lane {
        id: id(lane_id),
        provenance: provenance(lane_id),
        kind: LaneKind::Driving,
        allowed_actors: vec![TrafficActorKind::MotorVehicle],
        centerline_m: vec![start, end],
        width_m: 3.0,
        speed_limit_m_s: Some(10.0),
        road_class: None,
        road_functions: Vec::new(),
    }
}

fn routing_connection(
    connection_id: &str,
    incoming: &str,
    outgoing: &str,
    movement: MovementKind,
    path_m: Vec<[f64; 3]>,
) -> TrafficConnection {
    TrafficConnection {
        id: id(connection_id),
        provenance: provenance(connection_id),
        incoming_lane_id: id(incoming),
        outgoing_lane_id: id(outgoing),
        junction_id: None,
        movement,
        path_m,
        conflict_connection_ids: Vec::new(),
        signal_group_id: None,
    }
}

fn routing_network() -> TrafficNetwork {
    TrafficNetwork {
        id: id("network:example-47"),
        provenance: provenance("network"),
        coordinate_frame: CoordinateFrame {
            frame_id: "map".into(),
            axis_convention: AxisConvention::RneYUp,
            origin_m: [0.0, 0.0, 0.0],
            source_crs: None,
        },
        lanes: vec![
            routing_lane("lane:start", [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]),
            routing_lane("lane:middle", [11.0, 0.0, 1.0], [20.0, 0.0, 10.0]),
            routing_lane("lane:goal", [21.0, 0.0, 10.0], [31.0, 0.0, 10.0]),
        ],
        junctions: Vec::new(),
        connections: vec![
            routing_connection(
                "connection:left",
                "lane:start",
                "lane:middle",
                MovementKind::Left,
                vec![[10.0, 0.0, 0.0], [11.0, 0.0, 1.0]],
            ),
            routing_connection(
                "connection:right",
                "lane:middle",
                "lane:goal",
                MovementKind::Right,
                vec![[20.0, 0.0, 10.0], [21.0, 0.0, 10.0]],
            ),
        ],
        signals: Vec::new(),
    }
}

use bevy_ecs::prelude::World;
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use rne_traffic::{
    advance_reserved_kinematic_traffic, materialize_lane_route, Accuracy, AccuracyClass,
    AuthorityClass, AxisConvention, CoordinateFrame, Junction, JunctionKind,
    KinematicTrafficConfig, KinematicTrafficControls, Lane, LaneKind, LaneRoute, MovementKind,
    Provenance, SourceReference, TrafficActor, TrafficActorKind, TrafficConflictControls,
    TrafficConnection, TrafficFlowMetrics, TrafficId, TrafficNetwork, TrafficPose,
    TrafficRouteCatalog, TrafficRouteFollower, TrafficRuntime, TrafficSignalControls,
};
use uuid::Uuid;

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("fixture ID")
}

fn provenance(feature: &str) -> Provenance {
    Provenance {
        authority: AuthorityClass::Derived,
        accuracy: Accuracy {
            class: AccuracyClass::Derived,
            horizontal_m: Some(0.1),
            vertical_m: Some(0.1),
        },
        sources: vec![SourceReference {
            dataset: "conflict fixture".into(),
            feature_id: Some(feature.into()),
            uri: None,
        }],
        method: Some("orthogonal conflict fixture".into()),
    }
}

fn lane(lane_id: &str, start: [f64; 3], end: [f64; 3]) -> Lane {
    Lane {
        id: id(lane_id),
        provenance: provenance(lane_id),
        kind: LaneKind::Driving,
        allowed_actors: vec![TrafficActorKind::MotorVehicle],
        centerline_m: vec![start, end],
        width_m: 3.0,
        speed_limit_m_s: Some(8.0),
        road_class: None,
        road_functions: Vec::new(),
    }
}

fn crossing_network() -> TrafficNetwork {
    let east_id = id("connection:east");
    let north_id = id("connection:north");
    TrafficNetwork {
        id: id("network:crossing"),
        provenance: provenance("network"),
        coordinate_frame: CoordinateFrame {
            frame_id: "map".into(),
            axis_convention: AxisConvention::RneYUp,
            origin_m: [0.0; 3],
            source_crs: None,
        },
        lanes: vec![
            lane("lane:west", [-20.0, 0.0, 0.0], [-2.0, 0.0, 0.0]),
            lane("lane:east", [2.0, 0.0, 0.0], [20.0, 0.0, 0.0]),
            lane("lane:south", [0.0, 0.0, -20.0], [0.0, 0.0, -2.0]),
            lane("lane:north", [0.0, 0.0, 2.0], [0.0, 0.0, 20.0]),
        ],
        junctions: vec![Junction {
            id: id("junction:center"),
            provenance: provenance("junction"),
            kind: JunctionKind::CrossIntersection,
            center_m: [0.0; 3],
        }],
        connections: vec![
            TrafficConnection {
                id: east_id.clone(),
                provenance: provenance("east"),
                incoming_lane_id: id("lane:west"),
                outgoing_lane_id: id("lane:east"),
                junction_id: Some(id("junction:center")),
                movement: MovementKind::Straight,
                path_m: vec![[-2.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
                conflict_connection_ids: vec![north_id.clone()],
                signal_group_id: None,
            },
            TrafficConnection {
                id: north_id,
                provenance: provenance("north"),
                incoming_lane_id: id("lane:south"),
                outgoing_lane_id: id("lane:north"),
                junction_id: Some(id("junction:center")),
                movement: MovementKind::Straight,
                path_m: vec![[0.0, 0.0, -2.0], [0.0, 0.0, 2.0]],
                conflict_connection_ids: vec![east_id],
                signal_group_id: None,
            },
            TrafficConnection {
                id: id("connection:bypass"),
                provenance: provenance("bypass"),
                incoming_lane_id: id("lane:west"),
                outgoing_lane_id: id("lane:east"),
                junction_id: Some(id("junction:center")),
                movement: MovementKind::Straight,
                path_m: vec![[-2.0, 0.0, 0.5], [2.0, 0.0, 0.5]],
                conflict_connection_ids: Vec::new(),
                signal_group_id: None,
            },
        ],
        signals: Vec::new(),
    }
}

fn routes(network: &TrafficNetwork) -> TrafficRouteCatalog {
    let mut routes = TrafficRouteCatalog::default();
    for (route_id, lanes, connection) in [
        (
            "route:east",
            vec![id("lane:west"), id("lane:east")],
            id("connection:east"),
        ),
        (
            "route:north",
            vec![id("lane:south"), id("lane:north")],
            id("connection:north"),
        ),
        (
            "route:bypass",
            vec![id("lane:west"), id("lane:east")],
            id("connection:bypass"),
        ),
    ] {
        routes
            .insert(
                materialize_lane_route(
                    network,
                    &LaneRoute {
                        lane_ids: lanes,
                        connection_ids: vec![connection],
                        distance_m: 40.0,
                    },
                    id(route_id),
                    false,
                )
                .expect("materialize crossing route"),
            )
            .expect("insert crossing route");
    }
    routes
}

fn replay(reverse_spawn_order: bool) -> (u64, usize, usize, [f64; 2], TrafficFlowMetrics) {
    let network = crossing_network();
    let routes = routes(&network);
    let mut conflicts = TrafficConflictControls::from_network_routes(&network, &routes, 12.0)
        .expect("build conflict controls");
    assert_eq!(conflicts.len(), 3);
    assert_eq!(
        conflicts
            .iter()
            .map(|control| control.conflict_group_id.clone())
            .collect::<std::collections::BTreeSet<_>>()
            .len(),
        1,
        "all movements at a controlled junction share one reservation group"
    );
    let mut world = World::new();
    let actor_specs = [
        (1_u128, id("route:east"), None),
        (2_u128, id("route:north"), Some(0.5)),
    ];
    let indices: Vec<_> = if reverse_spawn_order {
        (0..actor_specs.len()).rev().collect()
    } else {
        (0..actor_specs.len()).collect()
    };
    for index in indices {
        let (uuid, route_id, departure_time_s) = &actor_specs[index];
        let sample = routes.get(route_id).expect("route").sample(10.0);
        let mut entity = world.spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(*uuid)),
            TrafficRouteFollower {
                route_id: route_id.clone(),
                distance_m: 10.0,
                speed_m_s: 0.0,
                desired_speed_m_s: 6.0,
                length_m: 4.2,
            },
            TrafficPose {
                position_m: sample.position_m,
                yaw_rad: sample.yaw_rad,
            },
        ));
        if let Some(departure_time_s) = departure_time_s {
            entity.insert(rne_traffic::TrafficDeparture {
                departure_time_s: *departure_time_s,
            });
        }
    }
    let delta = SimDuration::from_ticks(16_666_666);
    let mut runtime = TrafficRuntime::default();
    let signals = TrafficSignalControls::default();
    let mut stable_hash = 0;
    let mut collision_count = 0;
    let mut maximum_reservations = 0;
    let mut flow = TrafficFlowMetrics::default();
    for step in 1..=600 {
        let report = advance_reserved_kinematic_traffic(
            &mut world,
            &routes,
            KinematicTrafficControls::new(&signals, &mut conflicts),
            &mut runtime,
            SimTime::from_ticks(step * delta.ticks()),
            delta,
            KinematicTrafficConfig::default(),
        )
        .expect("advance reserved crossing");
        stable_hash = report.stable_state_hash;
        collision_count += report.collision_count;
        maximum_reservations = maximum_reservations.max(report.active_reservation_count);
        flow = report.flow;
    }
    let mut final_distances = [0.0; 2];
    for (uuid, route_id, _) in actor_specs {
        let distance_m = world
            .query::<(&EntityUuid, &TrafficRouteFollower)>()
            .iter(&world)
            .find(|(entity_uuid, _)| entity_uuid.0 == Uuid::from_u128(uuid))
            .expect("actor")
            .1
            .distance_m;
        assert_eq!(
            world
                .query::<(&EntityUuid, &TrafficRouteFollower)>()
                .iter(&world)
                .find(|(entity_uuid, _)| entity_uuid.0 == Uuid::from_u128(uuid))
                .expect("actor")
                .1
                .route_id,
            route_id
        );
        final_distances[(uuid - 1) as usize] = distance_m;
    }
    (
        stable_hash,
        collision_count,
        maximum_reservations,
        final_distances,
        flow,
    )
}

#[test]
fn conflicting_routes_reserve_one_owner_and_replay_independently_of_spawn_order() {
    let forward = replay(false);
    let reverse = replay(true);

    assert_eq!(forward, reverse);
    assert_eq!(forward.1, 0);
    assert_eq!(forward.2, 1);
    assert!(forward.3.iter().all(|distance_m| *distance_m > 35.0));
    assert_eq!(forward.4.completed_trip_count, 2);
    assert!(forward.4.cumulative_waiting_time_s > 0.0);
}

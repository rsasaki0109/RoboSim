use rne_traffic::{
    shortest_lane_route, Accuracy, AccuracyClass, AuthorityClass, AxisConvention, CoordinateFrame,
    Lane, LaneKind, MovementKind, Provenance, SourceReference, TrafficActorKind, TrafficConnection,
    TrafficId, TrafficNetwork,
};

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("fixture ID")
}

fn provenance(feature_id: &str) -> Provenance {
    Provenance {
        authority: AuthorityClass::Derived,
        accuracy: Accuracy {
            class: AccuracyClass::Derived,
            horizontal_m: Some(0.1),
            vertical_m: Some(0.1),
        },
        sources: vec![SourceReference {
            dataset: "routing fixture".into(),
            feature_id: Some(feature_id.into()),
            uri: None,
        }],
        method: Some("test topology".into()),
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
        speed_limit_m_s: Some(10.0),
        road_class: None,
        road_functions: Vec::new(),
    }
}

fn connection(
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

fn fixture(scrambled: bool) -> TrafficNetwork {
    let mut lanes = vec![
        lane("lane:start", [0.0, 0.0, 0.0], [10.0, 0.0, 0.0]),
        lane("lane:short", [11.0, 0.0, 1.0], [20.0, 0.0, 10.0]),
        lane("lane:long", [11.0, 0.0, -1.0], [31.0, 0.0, -21.0]),
        lane("lane:goal", [21.0, 0.0, 10.0], [31.0, 0.0, 10.0]),
    ];
    let mut connections = vec![
        connection(
            "connection:start-short",
            "lane:start",
            "lane:short",
            MovementKind::Left,
            vec![[10.0, 0.0, 0.0], [11.0, 0.0, 1.0]],
        ),
        connection(
            "connection:short-goal",
            "lane:short",
            "lane:goal",
            MovementKind::Right,
            vec![[20.0, 0.0, 10.0], [21.0, 0.0, 10.0]],
        ),
        connection(
            "connection:start-long",
            "lane:start",
            "lane:long",
            MovementKind::Right,
            vec![[10.0, 0.0, 0.0], [11.0, 0.0, -1.0]],
        ),
        connection(
            "connection:long-goal",
            "lane:long",
            "lane:goal",
            MovementKind::Left,
            vec![[31.0, 0.0, -21.0], [21.0, 0.0, 10.0]],
        ),
    ];
    if scrambled {
        lanes.reverse();
        connections.reverse();
    }
    TrafficNetwork {
        id: id("network:routing"),
        provenance: provenance("network"),
        coordinate_frame: CoordinateFrame {
            frame_id: "map".into(),
            axis_convention: AxisConvention::RneYUp,
            origin_m: [0.0, 0.0, 0.0],
            source_crs: None,
        },
        lanes,
        junctions: Vec::new(),
        connections,
        signals: Vec::new(),
    }
}

#[test]
fn shortest_route_is_deterministic_and_contains_left_and_right_turns() {
    let expected = shortest_lane_route(
        &fixture(false),
        &id("lane:start"),
        &id("lane:goal"),
        TrafficActorKind::MotorVehicle,
    )
    .expect("shortest route");
    let scrambled = shortest_lane_route(
        &fixture(true),
        &id("lane:start"),
        &id("lane:goal"),
        TrafficActorKind::MotorVehicle,
    )
    .expect("scrambled shortest route");

    assert_eq!(expected, scrambled);
    assert_eq!(
        expected.lane_ids,
        vec![id("lane:start"), id("lane:short"), id("lane:goal")]
    );
    assert_eq!(
        expected.connection_ids,
        vec![id("connection:start-short"), id("connection:short-goal")]
    );
}

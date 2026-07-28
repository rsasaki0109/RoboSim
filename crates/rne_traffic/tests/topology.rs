use rne_traffic::{
    build_traffic_topology, canonical_traffic_asset_bytes, Accuracy, AccuracyClass, AuthorityClass,
    AxisConvention, CoordinateFrame, JunctionKind, Lane, LaneKind, MovementKind, Provenance,
    SourceReference, TopologyBuildConfig, TopologyBuilder, TopologyError, TrafficActorKind,
    TrafficAsset, TrafficId, TrafficNetwork,
};
fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("fixture ID")
}

fn provenance(feature_id: &str) -> Provenance {
    Provenance {
        authority: AuthorityClass::Authoritative,
        accuracy: Accuracy {
            class: AccuracyClass::Modeled,
            horizontal_m: Some(0.2),
            vertical_m: Some(0.2),
        },
        sources: vec![SourceReference {
            dataset: "synthetic topology fixture".into(),
            feature_id: Some(feature_id.into()),
            uri: None,
        }],
        method: None,
    }
}

fn lane(lane_id: &str, points: Vec<[f64; 3]>) -> Lane {
    Lane {
        id: id(lane_id),
        provenance: provenance(lane_id),
        kind: LaneKind::Driving,
        allowed_actors: vec![TrafficActorKind::MotorVehicle],
        centerline_m: points,
        width_m: 3.0,
        speed_limit_m_s: Some(13.0),
        road_class: Some("test road".into()),
        road_functions: vec!["traffic".into()],
    }
}

fn network(network_id: &str, lanes: Vec<Lane>) -> TrafficNetwork {
    TrafficNetwork {
        id: id(network_id),
        provenance: provenance(network_id),
        coordinate_frame: CoordinateFrame {
            frame_id: "map".into(),
            axis_convention: AxisConvention::RneYUp,
            origin_m: [0.0, 0.0, 0.0],
            source_crs: Some("EPSG:6697".into()),
        },
        lanes,
        junctions: Vec::new(),
        connections: Vec::new(),
        signals: Vec::new(),
    }
}

fn cross_lanes() -> Vec<Lane> {
    vec![
        lane("lane:west-in", vec![[-20.0, 0.0, 0.75], [-2.0, 0.0, 0.75]]),
        lane(
            "lane:west-out",
            vec![[-2.0, 0.0, -0.75], [-20.0, 0.0, -0.75]],
        ),
        lane("lane:east-in", vec![[20.0, 0.0, -0.75], [2.0, 0.0, -0.75]]),
        lane("lane:east-out", vec![[2.0, 0.0, 0.75], [20.0, 0.0, 0.75]]),
        lane("lane:south-in", vec![[-0.75, 0.0, 20.0], [-0.75, 0.0, 2.0]]),
        lane("lane:south-out", vec![[0.75, 0.0, 2.0], [0.75, 0.0, 20.0]]),
        lane("lane:north-in", vec![[0.75, 0.0, -20.0], [0.75, 0.0, -2.0]]),
        lane(
            "lane:north-out",
            vec![[-0.75, 0.0, -2.0], [-0.75, 0.0, -20.0]],
        ),
    ]
}

#[test]
fn builds_cross_intersection_turns_curves_and_symmetric_conflicts() {
    let result = build_traffic_topology(
        id("network:cross-topology"),
        &[network("network:cross-source", cross_lanes())],
        TopologyBuildConfig::default(),
    )
    .expect("cross topology");

    assert_eq!(result.stats.junction_count, 1);
    assert_eq!(result.stats.connection_count, 12);
    assert!(result.stats.conflict_pair_count > 0);
    assert_eq!(
        result.network.junctions[0].kind,
        JunctionKind::CrossIntersection
    );

    let count = |movement| {
        result
            .network
            .connections
            .iter()
            .filter(|connection| connection.movement == movement)
            .count()
    };
    assert_eq!(count(MovementKind::Straight), 4);
    assert_eq!(count(MovementKind::Left), 4);
    assert_eq!(count(MovementKind::Right), 4);
    assert_eq!(count(MovementKind::UTurn), 0);

    for connection in &result.network.connections {
        assert_eq!(connection.path_m.len(), 13);
        let incoming = result
            .network
            .lanes
            .iter()
            .find(|lane| lane.id == connection.incoming_lane_id)
            .expect("incoming lane");
        let outgoing = result
            .network
            .lanes
            .iter()
            .find(|lane| lane.id == connection.outgoing_lane_id)
            .expect("outgoing lane");
        assert_eq!(connection.path_m.first(), incoming.centerline_m.last());
        assert_eq!(connection.path_m.last(), outgoing.centerline_m.first());
        for conflict_id in &connection.conflict_connection_ids {
            let conflict = result
                .network
                .connections
                .iter()
                .find(|other| &other.id == conflict_id)
                .expect("conflicting connection");
            assert!(conflict.conflict_connection_ids.contains(&connection.id));
        }
    }
}

#[test]
fn builds_t_intersection() {
    let lanes = cross_lanes()
        .into_iter()
        .filter(|lane| !lane.id.as_str().contains("north"))
        .collect();
    let result = build_traffic_topology(
        id("network:t-topology"),
        &[network("network:t-source", lanes)],
        TopologyBuildConfig::default(),
    )
    .expect("T topology");

    assert_eq!(result.stats.junction_count, 1);
    assert_eq!(result.stats.connection_count, 6);
    assert_eq!(
        result.network.junctions[0].kind,
        JunctionKind::TIntersection
    );
}

#[test]
fn stitches_a_tile_boundary_with_stable_provenance() {
    let west = network(
        "network:tile-west",
        vec![lane(
            "lane:tile-west-in",
            vec![[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
        )],
    );
    let east = network(
        "network:tile-east",
        vec![lane(
            "lane:tile-east-out",
            vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]],
        )],
    );
    let result = build_traffic_topology(
        id("network:stitched"),
        &[west, east],
        TopologyBuildConfig::default(),
    )
    .expect("tile-boundary topology");

    assert_eq!(result.stats.tile_boundary_count, 1);
    assert_eq!(result.stats.connection_count, 1);
    assert_eq!(result.network.junctions[0].kind, JunctionKind::TileBoundary);
    assert_eq!(result.network.junctions[0].provenance.sources.len(), 2);
}

#[test]
fn separates_grade_crossings() {
    let lanes = vec![
        lane("lane:low-in", vec![[-10.0, 0.0, 0.0], [0.0, 0.0, 0.0]]),
        lane("lane:low-out", vec![[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]]),
        lane("lane:high-in", vec![[0.0, 5.0, -10.0], [0.0, 5.0, 0.0]]),
        lane("lane:high-out", vec![[0.0, 5.0, 0.0], [0.0, 5.0, 10.0]]),
    ];
    let result = build_traffic_topology(
        id("network:grade-separated"),
        &[network("network:grade-source", lanes)],
        TopologyBuildConfig::default(),
    )
    .expect("grade-separated topology");

    assert_eq!(result.stats.junction_count, 2);
    assert_eq!(result.stats.connection_count, 2);
    assert_eq!(result.stats.conflict_pair_count, 0);
    assert_ne!(
        result.network.connections[0].junction_id,
        result.network.connections[1].junction_id
    );
}

#[test]
fn curved_lane_heading_controls_movement_and_curve_endpoints() {
    let lanes = vec![
        lane(
            "lane:curved-in",
            vec![[-12.0, 0.0, -4.0], [-6.0, 0.0, -2.0], [-2.0, 0.0, 0.0]],
        ),
        lane(
            "lane:curved-out",
            vec![[0.0, 0.0, 2.0], [0.0, 0.0, 8.0], [2.0, 0.0, 14.0]],
        ),
    ];
    let result = build_traffic_topology(
        id("network:curve-topology"),
        &[network("network:curve-source", lanes)],
        TopologyBuildConfig::default(),
    )
    .expect("curved topology");

    assert_eq!(result.stats.connection_count, 1);
    let connection = &result.network.connections[0];
    assert_eq!(connection.movement, MovementKind::Right);
    assert_eq!(connection.path_m[0], [-2.0, 0.0, 0.0]);
    assert_eq!(
        *connection.path_m.last().expect("curve end"),
        [0.0, 0.0, 2.0]
    );
}

#[test]
fn output_is_byte_identical_for_scrambled_network_and_lane_order() {
    let mut lanes = cross_lanes();
    let split = lanes.split_off(4);
    let first = network("network:source-a", lanes);
    let second = network("network:source-b", split);
    let canonical = build_traffic_topology(
        id("network:stable"),
        &[first.clone(), second.clone()],
        TopologyBuildConfig::default(),
    )
    .expect("canonical topology");

    let mut reversed_first = first;
    reversed_first.lanes.reverse();
    let mut reversed_second = second;
    reversed_second.lanes.reverse();
    let scrambled = build_traffic_topology(
        id("network:stable"),
        &[reversed_second, reversed_first],
        TopologyBuildConfig::default(),
    )
    .expect("scrambled topology");

    assert_eq!(
        canonical_traffic_asset_bytes(&TrafficAsset::new(canonical.network))
            .expect("canonical bytes"),
        canonical_traffic_asset_bytes(&TrafficAsset::new(scrambled.network))
            .expect("scrambled bytes")
    );
}

#[test]
fn rejects_invalid_config_before_building() {
    let config = TopologyBuildConfig {
        turn_curve_segments: 1,
        ..TopologyBuildConfig::default()
    };
    assert!(matches!(
        TopologyBuilder::new(config),
        Err(TopologyError::InvalidConfig {
            field: "turn_curve_segments",
            ..
        })
    ));
}

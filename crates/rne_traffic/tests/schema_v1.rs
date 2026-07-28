use rne_traffic::{
    canonical_traffic_asset_bytes, load_traffic_asset, parse_traffic_asset, save_traffic_asset,
    Accuracy, AccuracyClass, AuthorityClass, AxisConvention, CoordinateFrame, Junction,
    JunctionKind, Lane, LaneKind, MovementKind, Provenance, SignalAspect, SignalGroup,
    SignalGroupAspect, SignalPhase, SignalProgram, SourceReference, TrafficActorKind, TrafficAsset,
    TrafficAssetError, TrafficConnection, TrafficId, TrafficNetwork, TrafficSignal,
};
use std::path::PathBuf;

fn id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("fixture ID")
}

fn plateau_source(feature_id: &str) -> SourceReference {
    SourceReference {
        dataset: "PLATEAU 53394525 (CityGML 3.0)".into(),
        feature_id: Some(feature_id.into()),
        uri: Some("https://www.geospatial.jp/ckan/dataset/plateau-53394525".into()),
    }
}

fn authoritative(feature_id: &str) -> Provenance {
    Provenance {
        authority: AuthorityClass::Authoritative,
        accuracy: Accuracy {
            class: AccuracyClass::Modeled,
            horizontal_m: Some(0.2),
            vertical_m: Some(0.3),
        },
        sources: vec![plateau_source(feature_id)],
        method: None,
    }
}

fn derived(feature_id: &str, method: &str) -> Provenance {
    Provenance {
        authority: AuthorityClass::Derived,
        accuracy: Accuracy {
            class: AccuracyClass::Derived,
            horizontal_m: Some(0.35),
            vertical_m: Some(0.3),
        },
        sources: vec![plateau_source(feature_id)],
        method: Some(method.into()),
    }
}

fn lane(lane_id: &str, source_id: &str, points: [[f64; 3]; 2]) -> Lane {
    Lane {
        id: id(lane_id),
        provenance: authoritative(source_id),
        kind: LaneKind::Driving,
        allowed_actors: vec![TrafficActorKind::MotorVehicle, TrafficActorKind::Bicycle],
        centerline_m: points.into(),
        width_m: 3.25,
        speed_limit_m_s: Some(11.111_111_111_111_11),
        road_class: Some("道路".into()),
        road_functions: vec!["一般道路".into(), "緊急輸送道路".into()],
    }
}

fn fixture(scrambled: bool) -> TrafficAsset {
    let junction_id = id("plateau:53394525/junction-main");
    let eastbound_id = id("plateau:53394525/connection-eastbound");
    let northbound_id = id("plateau:53394525/connection-northbound");
    let eastbound_group_id = id("scenario:signal-main/group-eastbound");
    let northbound_group_id = id("scenario:signal-main/group-northbound");

    let mut lanes = vec![
        lane(
            "plateau:53394525/road-west#lane-0",
            "road-west",
            [[-20.0, 0.0, 1.75], [-4.0, 0.0, 1.75]],
        ),
        lane(
            "plateau:53394525/road-east#lane-0",
            "road-east",
            [[4.0, 0.0, 1.75], [20.0, 0.0, 1.75]],
        ),
        lane(
            "plateau:53394525/road-south#lane-0",
            "road-south",
            [[-1.75, 0.0, 20.0], [-1.75, 0.0, 4.0]],
        ),
        lane(
            "plateau:53394525/road-north#lane-0",
            "road-north",
            [[-1.75, 0.0, -4.0], [-1.75, 0.0, -20.0]],
        ),
    ];
    let mut connections = vec![
        TrafficConnection {
            id: eastbound_id.clone(),
            provenance: derived("junction-main", "connect directed lane endpoints"),
            incoming_lane_id: id("plateau:53394525/road-west#lane-0"),
            outgoing_lane_id: id("plateau:53394525/road-east#lane-0"),
            junction_id: Some(junction_id.clone()),
            movement: MovementKind::Straight,
            path_m: vec![[-4.0, 0.0, 1.75], [0.0, 0.0, 1.75], [4.0, 0.0, 1.75]],
            conflict_connection_ids: vec![northbound_id.clone()],
            signal_group_id: Some(eastbound_group_id.clone()),
        },
        TrafficConnection {
            id: northbound_id.clone(),
            provenance: derived("junction-main", "connect directed lane endpoints"),
            incoming_lane_id: id("plateau:53394525/road-south#lane-0"),
            outgoing_lane_id: id("plateau:53394525/road-north#lane-0"),
            junction_id: Some(junction_id.clone()),
            movement: MovementKind::Straight,
            path_m: vec![[-1.75, 0.0, 4.0], [-1.75, 0.0, 0.0], [-1.75, 0.0, -4.0]],
            conflict_connection_ids: vec![eastbound_id.clone()],
            signal_group_id: Some(northbound_group_id.clone()),
        },
    ];
    let mut groups = vec![
        SignalGroup {
            id: eastbound_group_id.clone(),
            connection_ids: vec![eastbound_id.clone()],
        },
        SignalGroup {
            id: northbound_group_id.clone(),
            connection_ids: vec![northbound_id.clone()],
        },
    ];

    let synthetic_program = Provenance {
        authority: AuthorityClass::Synthetic,
        accuracy: Accuracy {
            class: AccuracyClass::ScenarioAuthored,
            horizontal_m: None,
            vertical_m: None,
        },
        sources: Vec::new(),
        method: Some("RNE fixed-time reference scenario".into()),
    };
    let mut eastbound_aspects = vec![
        SignalGroupAspect {
            group_id: eastbound_group_id.clone(),
            aspect: SignalAspect::Green,
        },
        SignalGroupAspect {
            group_id: northbound_group_id.clone(),
            aspect: SignalAspect::Red,
        },
    ];
    let mut northbound_aspects = vec![
        SignalGroupAspect {
            group_id: eastbound_group_id,
            aspect: SignalAspect::Red,
        },
        SignalGroupAspect {
            group_id: northbound_group_id,
            aspect: SignalAspect::Green,
        },
    ];

    if scrambled {
        lanes.reverse();
        connections.reverse();
        groups.reverse();
        eastbound_aspects.reverse();
        northbound_aspects.reverse();
        for lane in &mut lanes {
            lane.allowed_actors.reverse();
            lane.road_functions.reverse();
        }
    }

    TrafficAsset::new(TrafficNetwork {
        id: id("plateau:53394525/network"),
        provenance: authoritative("CityModel-53394525"),
        coordinate_frame: CoordinateFrame {
            frame_id: "map".into(),
            axis_convention: AxisConvention::RneYUp,
            origin_m: [-0.0, 0.0, -0.0],
            source_crs: Some("EPSG:6697".into()),
        },
        lanes,
        junctions: vec![Junction {
            id: junction_id.clone(),
            provenance: derived("junction-main", "intersect at-grade lane centerlines"),
            kind: JunctionKind::CrossIntersection,
            center_m: [0.0, 0.0, 0.0],
        }],
        connections,
        signals: vec![TrafficSignal {
            id: id("scenario:signal-main"),
            provenance: Provenance {
                authority: AuthorityClass::Synthetic,
                accuracy: Accuracy {
                    class: AccuracyClass::ScenarioAuthored,
                    horizontal_m: None,
                    vertical_m: None,
                },
                sources: Vec::new(),
                method: Some("reference intersection signal placement".into()),
            },
            junction_id: Some(junction_id),
            position_m: Some([-3.5, 3.0, 3.5]),
            facing_yaw_rad: Some(0.0),
            groups,
            program: Some(SignalProgram {
                provenance: synthetic_program,
                offset_s: -0.0,
                phases: vec![
                    SignalPhase {
                        id: id("scenario:signal-main/phase-eastbound"),
                        duration_s: 12.0,
                        group_aspects: eastbound_aspects,
                    },
                    SignalPhase {
                        id: id("scenario:signal-main/phase-northbound"),
                        duration_s: 10.0,
                        group_aspects: northbound_aspects,
                    },
                ],
            }),
        }],
    })
}

fn golden_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../tests/golden/traffic/schema_v1_reference.rne.traffic.json")
}

#[test]
fn schema_v1_matches_byte_identical_golden() {
    let bytes = canonical_traffic_asset_bytes(&fixture(false)).expect("canonical asset");
    let expected = std::fs::read(golden_path()).expect("read traffic golden");
    assert_eq!(bytes, expected);
}

#[test]
fn set_order_and_negative_zero_do_not_change_bytes() {
    let canonical = fixture(false);
    let scrambled = fixture(true);

    assert_eq!(
        canonical_traffic_asset_bytes(&canonical).expect("canonical order"),
        canonical_traffic_asset_bytes(&scrambled).expect("scrambled order")
    );
}

#[test]
fn canonical_json_round_trips_and_restores_order() {
    let bytes = canonical_traffic_asset_bytes(&fixture(false)).expect("serialize");
    let parsed = parse_traffic_asset(&bytes).expect("parse");
    assert_eq!(
        canonical_traffic_asset_bytes(&parsed).expect("reserialize"),
        bytes
    );
    assert!(parsed
        .network
        .lanes
        .windows(2)
        .all(|lanes| lanes[0].id < lanes[1].id));
}

#[test]
fn file_io_writes_and_loads_canonical_bytes() {
    let path = std::env::temp_dir().join(format!(
        "rne-traffic-schema-v1-{}.rne.traffic.json",
        std::process::id()
    ));
    let asset = fixture(true);

    save_traffic_asset(&path, &asset).expect("save canonical asset");
    let disk_bytes = std::fs::read(&path).expect("read saved bytes");
    let loaded = load_traffic_asset(&path).expect("load canonical asset");

    assert_eq!(
        disk_bytes,
        canonical_traffic_asset_bytes(&asset).expect("expected bytes")
    );
    assert_eq!(
        canonical_traffic_asset_bytes(&loaded).expect("loaded bytes"),
        disk_bytes
    );
    std::fs::remove_file(path).expect("remove temp traffic asset");
}

#[test]
fn missing_lane_reference_is_rejected() {
    let mut asset = fixture(false);
    asset.network.connections[0].outgoing_lane_id = id("plateau:missing-lane");

    assert!(matches!(
        canonical_traffic_asset_bytes(&asset),
        Err(TrafficAssetError::MissingReference {
            target_kind: "lane",
            ..
        })
    ));
}

#[test]
fn asymmetric_conflict_is_rejected() {
    let mut asset = fixture(false);
    asset.network.connections[0].conflict_connection_ids.clear();

    assert!(matches!(
        canonical_traffic_asset_bytes(&asset),
        Err(TrafficAssetError::InvalidValue {
            field: "conflict_connection_ids",
            ..
        })
    ));
}

#[test]
fn duplicate_ids_across_record_kinds_are_rejected() {
    let mut asset = fixture(false);
    asset.network.junctions[0].id = asset.network.lanes[0].id.clone();

    assert!(matches!(
        canonical_traffic_asset_bytes(&asset),
        Err(TrafficAssetError::DuplicateId {
            first_kind: "lane",
            second_kind: "junction",
            ..
        })
    ));
}

#[test]
fn signal_phase_must_cover_every_group_once() {
    let mut asset = fixture(false);
    asset.network.signals[0]
        .program
        .as_mut()
        .expect("program")
        .phases[0]
        .group_aspects
        .pop();

    assert!(matches!(
        canonical_traffic_asset_bytes(&asset),
        Err(TrafficAssetError::InvalidValue {
            field: "group_aspects",
            ..
        })
    ));
}

#[test]
fn nonpositive_metric_values_are_rejected() {
    let mut asset = fixture(false);
    asset.network.lanes[0].width_m = 0.0;

    assert!(matches!(
        canonical_traffic_asset_bytes(&asset),
        Err(TrafficAssetError::InvalidValue {
            field: "width_m",
            ..
        })
    ));
}

#[test]
fn invalid_stable_id_is_rejected_during_json_parse() {
    let bytes = canonical_traffic_asset_bytes(&fixture(false)).expect("serialize");
    let invalid = String::from_utf8(bytes).expect("UTF-8").replace(
        "plateau:53394525/network",
        "plateau:53394525/network with space",
    );

    assert!(matches!(
        parse_traffic_asset(invalid.as_bytes()),
        Err(TrafficAssetError::Json(_))
    ));
}

#[test]
fn unknown_schema_fields_are_rejected() {
    let bytes = canonical_traffic_asset_bytes(&fixture(false)).expect("serialize");
    let invalid = String::from_utf8(bytes).expect("UTF-8").replace(
        "\"schema_version\": 1,",
        "\"schema_version\": 1,\n  \"schema_typo\": true,",
    );

    assert!(matches!(
        parse_traffic_asset(invalid.as_bytes()),
        Err(TrafficAssetError::Json(_))
    ));
}

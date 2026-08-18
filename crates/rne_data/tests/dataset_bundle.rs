use rne_core::{SimDuration, SimTime};
use rne_data::{
    decode_dataset_action, decode_dataset_annotation, decode_dataset_imu,
    decode_dataset_task_outcome, decode_dataset_transform, DatasetActionSample, DatasetAsset,
    DatasetBundle, DatasetBundleWriter, DatasetCalibration, DatasetError, DatasetFieldSpec,
    DatasetGapPolicy, DatasetGroundTruthAnnotation, DatasetLatencyModel, DatasetLatencySpec,
    DatasetManifest, DatasetNoiseSpec, DatasetRandomizationDecision, DatasetRandomizationValue,
    DatasetRecordKind, DatasetStreamKind, DatasetStreamSpec, DatasetTaskOutcomeSample,
    DatasetTimingSpec, DepthPairEvaluationReport, DepthPairMetricSpec, Frame, ImageDepth,
    ImuSample, PoseSample, StreamId, DATASET_ACTION_ENCODING, DATASET_ANNOTATION_ENCODING,
    DATASET_IMU_ENCODING, DATASET_TASK_OUTCOME_ENCODING, DATASET_TRANSFORM_ENCODING,
};
use rne_math::Vec3;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fs::{self, File};
use std::io::{Read, Seek, SeekFrom, Write};
use std::path::Path;

const TASK_DIGEST: &str = "sha256:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";
const ASSET_DIGEST: &str =
    "sha256:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb";

#[test]
fn bundle_round_trip_and_headless_depth_evaluation_match_goldens() {
    let temp = tempfile::tempdir().unwrap();
    let (manifest, report) = write_reference_bundle(temp.path());

    let manifest_json = pretty_json(&manifest);
    let report_json = pretty_json(&report);
    if let Some(directory) = std::env::var_os("RNE_DATA_GOLDEN_OUTPUT") {
        let directory = Path::new(&directory);
        fs::create_dir_all(directory).unwrap();
        fs::write(directory.join("bundle-manifest-v1.json"), &manifest_json).unwrap();
        fs::write(
            directory.join("depth-pair-evaluation-v1.json"),
            &report_json,
        )
        .unwrap();
    }
    assert_eq!(
        manifest_json,
        include_str!("../../../tests/golden/datasets/bundle-manifest-v1.json")
    );
    assert_eq!(
        report_json,
        include_str!("../../../tests/golden/datasets/depth-pair-evaluation-v1.json")
    );
}

#[test]
fn missing_sequence_is_rejected_but_an_explicit_gap_is_verified() {
    let missing = tempfile::tempdir().unwrap();
    let mut missing_writer =
        DatasetBundleWriter::create(missing.path().join("bundle"), reference_manifest()).unwrap();
    let (mut world, entity) = source_entity();
    let sequence_one = depth_frame(&mut world, entity, StreamId::new(10), 1, 10, vec![1.0]);
    assert!(matches!(
        missing_writer.write_image_depth(&sequence_one),
        Err(DatasetError::SequenceMismatch {
            expected: 0,
            actual: 1,
            ..
        })
    ));

    let explicit = tempfile::tempdir().unwrap();
    let bundle_path = explicit.path().join("bundle");
    let mut writer = DatasetBundleWriter::create(&bundle_path, reference_manifest()).unwrap();
    writer.write_gap(StreamId::new(10), 0, 3, 0, 2).unwrap();
    writer.write_gap(StreamId::new(11), 0, 3, 0, 2).unwrap();
    assert_eq!(writer.record_count(), 2);
    writer.finish().unwrap();
    let verification = DatasetBundle::open(bundle_path).unwrap().verify().unwrap();
    assert_eq!(verification.record_count, 2);
    assert_eq!(verification.sample_count, 0);
    assert_eq!(verification.dropped_count, 6);
}

#[test]
fn payload_digest_corruption_is_detected_during_streaming_verification() {
    let temp = tempfile::tempdir().unwrap();
    let bundle_path = temp.path().join("bundle");
    write_reference_bundle_at(&bundle_path);
    let shard = bundle_path.join("records.rnedata");
    let mut file = File::options().read(true).write(true).open(shard).unwrap();
    file.seek(SeekFrom::Start(16 + 48)).unwrap();
    let mut byte = [0_u8; 1];
    file.read_exact(&mut byte).unwrap();
    file.seek(SeekFrom::Start(16 + 48)).unwrap();
    file.write_all(&[byte[0] ^ 1]).unwrap();
    file.sync_all().unwrap();

    let bundle = DatasetBundle::open(&bundle_path).unwrap();
    assert!(matches!(
        bundle.verify(),
        Err(DatasetError::DigestMismatch(_))
    ));
}

#[test]
fn unknown_manifest_fields_fail_closed() {
    let temp = tempfile::tempdir().unwrap();
    let bundle_path = temp.path().join("bundle");
    write_reference_bundle_at(&bundle_path);
    let path = bundle_path.join("manifest.json");
    let mut value: serde_json::Value = serde_json::from_slice(&fs::read(&path).unwrap()).unwrap();
    value
        .as_object_mut()
        .unwrap()
        .insert("wall_clock_time".to_string(), serde_json::json!(123));
    fs::write(path, serde_json::to_vec_pretty(&value).unwrap()).unwrap();

    assert!(matches!(
        DatasetBundle::open(bundle_path),
        Err(DatasetError::Json(_))
    ));
}

#[test]
fn fixed_latency_and_sensor_calibration_are_enforced() {
    let mut invalid_manifest = reference_manifest();
    invalid_manifest.streams[0].calibration = None;
    assert!(matches!(
        invalid_manifest.validate(),
        Err(DatasetError::InvalidField {
            field: "streams.calibration",
            ..
        })
    ));

    let temp = tempfile::tempdir().unwrap();
    let mut writer =
        DatasetBundleWriter::create(temp.path().join("bundle"), reference_manifest()).unwrap();
    let (world, entity) = source_entity();
    let frame = Frame::new(
        StreamId::new(10),
        entity,
        0,
        SimTime::from_ticks(0),
        ImageDepth::new(1, 1, vec![1.0]),
    );
    assert!(matches!(
        writer.write_image_depth(&frame),
        Err(DatasetError::InvalidField {
            field: "record.available_ticks",
            ..
        })
    ));
    let _ = world;
}

#[test]
fn report_tampering_is_detected_even_when_the_verdict_is_changed() {
    let temp = tempfile::tempdir().unwrap();
    let (_, mut report) = write_reference_bundle(temp.path());
    assert!(report.passed);
    report.passed = false;
    assert!(matches!(
        report.validate(),
        Err(DatasetError::InvalidField {
            field: "metrics",
            ..
        })
    ));
}

#[test]
fn bundle_recomputation_rejects_a_self_consistent_forged_report() {
    let temp = tempfile::tempdir().unwrap();
    let bundle_path = temp.path().join("bundle");
    let (_, mut report) = write_reference_bundle_at(&bundle_path);
    report.mean_absolute_error_m = 0.0;
    report.root_mean_square_error_m = 0.0;
    report.max_absolute_error_m = 0.0;
    report.passed = true;
    report.content_sha256.clear();
    report.content_sha256 = format!(
        "sha256:{:x}",
        Sha256::digest(serde_json::to_vec(&report).unwrap())
    );
    report.validate().unwrap();

    let bundle = DatasetBundle::open(bundle_path).unwrap();
    assert!(matches!(
        bundle.verify_depth_pair_report(&report),
        Err(DatasetError::InvalidField {
            field: "offline_evaluation",
            ..
        })
    ));
}

#[test]
fn all_run_streams_have_versioned_round_trip_codecs() {
    let temp = tempfile::tempdir().unwrap();
    let bundle_path = temp.path().join("bundle");
    let mut writer = DatasetBundleWriter::create(&bundle_path, run_stream_manifest()).unwrap();
    let (_world, entity) = source_entity();
    let capture = SimTime::from_ticks(100);
    let latency = SimDuration::from_ticks(1);

    let imu = ImuSample {
        angular_velocity_rad_s: Vec3::new(0.1, -0.2, 0.3),
        linear_acceleration_m_s2: Vec3::new(1.0, 2.0, 9.81),
    };
    writer
        .write_imu(&Frame::new(StreamId::new(20), entity, 0, capture, imu).with_latency(latency))
        .unwrap();
    let transform = PoseSample {
        position_m: Vec3::new(1.0, 2.0, 3.0),
        yaw_rad: 0.25,
    };
    writer
        .write_transform(
            &Frame::new(StreamId::new(21), entity, 0, capture, transform).with_latency(latency),
        )
        .unwrap();
    let action = DatasetActionSample {
        values: vec![0.5, -0.25],
    };
    writer
        .write_action(
            &Frame::new(StreamId::new(22), entity, 0, capture, action.clone())
                .with_latency(latency),
        )
        .unwrap();
    let outcome = DatasetTaskOutcomeSample {
        episode_index: 7,
        step_in_episode: 12,
        reward: 1.5,
        cumulative_reward: 9.0,
        terminated: true,
        truncated: false,
        success: Some(true),
    };
    writer
        .write_task_outcome(
            &Frame::new(StreamId::new(23), entity, 0, capture, outcome).with_latency(latency),
        )
        .unwrap();
    let annotation = DatasetGroundTruthAnnotation {
        class_id: 3,
        instance_id: 99,
        values: vec![1.0, 2.0, 0.5, 0.75],
    };
    writer
        .write_ground_truth_annotation(
            &Frame::new(StreamId::new(24), entity, 0, capture, annotation.clone())
                .with_latency(latency),
        )
        .unwrap();
    let manifest = writer.finish().unwrap();
    assert_eq!(manifest.shards[0].record_count, 5);
    assert_eq!(manifest.shards[0].sample_count, 5);
    assert_eq!(
        manifest.shards[0].sha256,
        "sha256:09341137a0c4722b29b080f2eb46bb7a6537bc4585c2f4a18971d289ecb4b98f"
    );

    let bundle = DatasetBundle::open(bundle_path).unwrap();
    let verification = bundle.verify().unwrap();
    assert_eq!(verification.stream_count, 5);
    assert_eq!(verification.record_count, 5);
    let records = bundle
        .records()
        .unwrap()
        .collect::<Result<Vec<_>, _>>()
        .unwrap();
    assert_eq!(records.len(), 5);
    assert_eq!(records[0].kind, DatasetRecordKind::Imu);
    assert_eq!(decode_dataset_imu(&records[0].payload).unwrap().1, imu);
    assert_eq!(records[1].kind, DatasetRecordKind::Transform);
    assert_eq!(
        decode_dataset_transform(&records[1].payload).unwrap().1,
        transform
    );
    assert_eq!(records[2].kind, DatasetRecordKind::Action);
    assert_eq!(
        decode_dataset_action(&records[2].payload).unwrap().1,
        action
    );
    assert_eq!(records[3].kind, DatasetRecordKind::TaskOutcome);
    assert_eq!(
        decode_dataset_task_outcome(&records[3].payload).unwrap().1,
        outcome
    );
    assert_eq!(records[4].kind, DatasetRecordKind::GroundTruthAnnotation);
    assert_eq!(
        decode_dataset_annotation(&records[4].payload).unwrap().1,
        annotation
    );
}

#[test]
fn non_finite_action_and_trailing_payload_bytes_fail_closed() {
    let temp = tempfile::tempdir().unwrap();
    let mut writer =
        DatasetBundleWriter::create(temp.path().join("bundle"), run_stream_manifest()).unwrap();
    let (_world, entity) = source_entity();
    let frame = Frame::new(
        StreamId::new(22),
        entity,
        0,
        SimTime::from_ticks(0),
        DatasetActionSample {
            values: vec![f64::NAN],
        },
    )
    .with_latency(SimDuration::from_ticks(1));
    assert!(matches!(
        writer.write_action(&frame),
        Err(DatasetError::Transport(_))
    ));

    let metadata = rne_data::transport::SensorFrameMetadata {
        stream_id: 22,
        sensor_sequence: 0,
        capture_ticks: 0,
        available_ticks: 1,
    };
    let mut payload =
        rne_data::encode_dataset_action(metadata, &DatasetActionSample { values: vec![0.0] })
            .unwrap();
    payload.push(0);
    assert!(matches!(
        decode_dataset_action(&payload),
        Err(rne_data::transport::TransportError::TrailingBytes)
    ));
}

fn write_reference_bundle(root: &Path) -> (DatasetManifest, DepthPairEvaluationReport) {
    let bundle_path = root.join("bundle");
    write_reference_bundle_at(&bundle_path)
}

fn write_reference_bundle_at(bundle_path: &Path) -> (DatasetManifest, DepthPairEvaluationReport) {
    let mut writer = DatasetBundleWriter::create(bundle_path, reference_manifest()).unwrap();
    let (mut world, entity) = source_entity();
    for (stream, values) in [
        (StreamId::new(10), vec![1.0, 2.0, 3.0, 4.0]),
        (StreamId::new(11), vec![1.0, 2.1, 3.0, 3.9]),
    ] {
        writer
            .write_image_depth(&depth_frame(&mut world, entity, stream, 0, 0, values))
            .unwrap();
    }
    writer.write_gap(StreamId::new(10), 1, 1, 10, 12).unwrap();
    writer.write_gap(StreamId::new(11), 1, 1, 10, 12).unwrap();
    for (stream, values) in [
        (StreamId::new(10), vec![5.0, 6.0]),
        (StreamId::new(11), vec![5.05, 5.95]),
    ] {
        writer
            .write_image_depth(&depth_frame(&mut world, entity, stream, 2, 20, values))
            .unwrap();
    }
    assert_eq!(writer.record_count(), 6);
    let manifest = writer.finish().unwrap();
    let bundle = DatasetBundle::open(bundle_path).unwrap();
    let verification = bundle.verify().unwrap();
    assert_eq!(verification.stream_count, 2);
    assert_eq!(verification.record_count, 6);
    assert_eq!(verification.sample_count, 4);
    assert_eq!(verification.dropped_count, 2);
    let report = bundle
        .evaluate_depth_pair(DepthPairMetricSpec {
            predicted_stream: StreamId::new(10),
            ground_truth_stream: StreamId::new(11),
            tolerance_m: 0.11,
        })
        .unwrap();
    assert_eq!(report.compared_frames, 2);
    assert_eq!(report.compared_pixels, 6);
    assert_eq!(report.dropped_pairs, 1);
    assert!(report.passed);
    (manifest, report)
}

fn reference_manifest() -> DatasetManifest {
    let streams = [
        (10, "predicted_depth", 100),
        (11, "ground_truth_depth", 101),
    ]
    .into_iter()
    .map(|(stream_id, name, seed)| DatasetStreamSpec {
        stream_id: StreamId::new(stream_id),
        name: name.to_string(),
        kind: DatasetStreamKind::DepthF32,
        payload_encoding: "rne.transport.image_depth_f32.v1".to_string(),
        source_entity: "reference_camera".to_string(),
        frame_id: "camera_optical".to_string(),
        fields: vec![DatasetFieldSpec {
            name: "depth_m".to_string(),
            dtype: "f32[]".to_string(),
            unit: "m".to_string(),
        }],
        calibration: Some(DatasetCalibration {
            model: "pinhole.v1".to_string(),
            reference_frame: "camera_optical".to_string(),
            parameters: BTreeMap::from([
                ("cx_px".to_string(), 0.5),
                ("cy_px".to_string(), 0.5),
                ("fx_px".to_string(), 100.0),
                ("fy_px".to_string(), 100.0),
            ]),
        }),
        timing: DatasetTimingSpec {
            nominal_period_ticks: 10,
            latency: DatasetLatencySpec {
                model: DatasetLatencyModel::Fixed,
                fixed_ticks: Some(2),
                max_ticks: 2,
            },
            gap_policy: DatasetGapPolicy::ExplicitRecords,
        },
        noise: Some(DatasetNoiseSpec {
            model: "none.v1".to_string(),
            seed,
            parameters: BTreeMap::new(),
        }),
    })
    .collect();
    let mut manifest =
        DatasetManifest::new("rne.reference.depth-pair.v1", TASK_DIGEST, 10, 42, streams);
    manifest.assets.push(DatasetAsset {
        role: "scene".to_string(),
        path: "assets/reference-room.rne.json".to_string(),
        sha256: ASSET_DIGEST.to_string(),
    });
    manifest.randomization.push(DatasetRandomizationDecision {
        key: "lighting.intensity".to_string(),
        seed: 9001,
        value: DatasetRandomizationValue::Scalar {
            value: 1.25,
            unit: "unitless".to_string(),
        },
    });
    manifest
}

fn run_stream_manifest() -> DatasetManifest {
    let latency = DatasetLatencySpec {
        model: DatasetLatencyModel::Fixed,
        fixed_ticks: Some(1),
        max_ticks: 1,
    };
    let timing = || DatasetTimingSpec {
        nominal_period_ticks: 10,
        latency: latency.clone(),
        gap_policy: DatasetGapPolicy::ExplicitRecords,
    };
    let scalar_field = |name: &str, dtype: &str, unit: &str| DatasetFieldSpec {
        name: name.to_string(),
        dtype: dtype.to_string(),
        unit: unit.to_string(),
    };
    let imu = DatasetStreamSpec {
        stream_id: StreamId::new(20),
        name: "imu".to_string(),
        kind: DatasetStreamKind::Imu,
        payload_encoding: DATASET_IMU_ENCODING.to_string(),
        source_entity: "reference_camera".to_string(),
        frame_id: "imu_link".to_string(),
        fields: vec![
            scalar_field("angular_velocity_rad_s", "f64[3]", "rad_s"),
            scalar_field("linear_acceleration_m_s2", "f64[3]", "m_s2"),
        ],
        calibration: Some(DatasetCalibration {
            model: "imu_intrinsic.v1".to_string(),
            reference_frame: "imu_link".to_string(),
            parameters: BTreeMap::new(),
        }),
        timing: timing(),
        noise: Some(DatasetNoiseSpec {
            model: "none.v1".to_string(),
            seed: 20,
            parameters: BTreeMap::new(),
        }),
    };
    let plain = |stream_id, name: &str, kind, encoding: &str, fields| DatasetStreamSpec {
        stream_id: StreamId::new(stream_id),
        name: name.to_string(),
        kind,
        payload_encoding: encoding.to_string(),
        source_entity: "reference_camera".to_string(),
        frame_id: "world".to_string(),
        fields,
        calibration: None,
        timing: timing(),
        noise: None,
    };
    DatasetManifest::new(
        "rne.reference.run-streams.v1",
        TASK_DIGEST,
        10,
        42,
        vec![
            imu,
            plain(
                21,
                "transform",
                DatasetStreamKind::Transform,
                DATASET_TRANSFORM_ENCODING,
                vec![
                    scalar_field("position_m", "f64[3]", "m"),
                    scalar_field("yaw_rad", "f64", "rad"),
                ],
            ),
            plain(
                22,
                "action",
                DatasetStreamKind::Action,
                DATASET_ACTION_ENCODING,
                vec![scalar_field("values", "f64[]", "task_spec")],
            ),
            plain(
                23,
                "task_outcome",
                DatasetStreamKind::TaskOutcome,
                DATASET_TASK_OUTCOME_ENCODING,
                vec![
                    scalar_field("reward", "f64", "reward"),
                    scalar_field("terminated", "bool", "boolean"),
                    scalar_field("truncated", "bool", "boolean"),
                ],
            ),
            plain(
                24,
                "annotation",
                DatasetStreamKind::GroundTruthAnnotation,
                DATASET_ANNOTATION_ENCODING,
                vec![scalar_field("values", "f64[]", "stream_declared")],
            ),
        ],
    )
}

fn source_entity() -> (rne_ecs::World, rne_ecs::Entity) {
    let mut world = rne_ecs::World::new();
    let entity = rne_ecs::spawn_named(&mut world, "reference_camera");
    (world, entity)
}

fn depth_frame(
    _world: &mut rne_ecs::World,
    entity: rne_ecs::Entity,
    stream: StreamId,
    sequence: u64,
    capture_ticks: u64,
    values: Vec<f32>,
) -> Frame<ImageDepth> {
    let width = u32::try_from(values.len()).unwrap();
    Frame::new(
        stream,
        entity,
        sequence,
        SimTime::from_ticks(capture_ticks),
        ImageDepth::new(width, 1, values),
    )
    .with_latency(SimDuration::from_ticks(2))
}

fn pretty_json<T: serde::Serialize>(value: &T) -> String {
    let mut json = serde_json::to_string_pretty(value).unwrap();
    json.push('\n');
    json
}

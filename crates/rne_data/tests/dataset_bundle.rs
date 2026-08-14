use rne_core::{SimDuration, SimTime};
use rne_data::{
    DatasetAsset, DatasetBundle, DatasetBundleWriter, DatasetCalibration, DatasetError,
    DatasetFieldSpec, DatasetGapPolicy, DatasetLatencyModel, DatasetLatencySpec, DatasetManifest,
    DatasetNoiseSpec, DatasetRandomizationDecision, DatasetRandomizationValue, DatasetStreamKind,
    DatasetStreamSpec, DatasetTimingSpec, DepthPairEvaluationReport, DepthPairMetricSpec, Frame,
    ImageDepth, StreamId,
};
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

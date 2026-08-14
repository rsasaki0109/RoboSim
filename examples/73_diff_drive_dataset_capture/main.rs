//! Captures a real headless diff-drive episode into a streaming dataset bundle.

use rne_ai::{
    diff_drive_goal_task_spec, DiffDriveAction, DiffDriveEpisode, DiffDriveEpisodeConfig,
    DiffDriveRewardConfig, Episode, TaskSpec,
};
use rne_data::{
    DataBus, DatasetActionSample, DatasetAsset, DatasetBundle, DatasetBundleWriter,
    DatasetCalibration, DatasetFieldSpec, DatasetGapPolicy, DatasetLatencyModel,
    DatasetLatencySpec, DatasetManifest, DatasetNoiseSpec, DatasetRandomizationDecision,
    DatasetRandomizationValue, DatasetStreamKind, DatasetStreamSpec, DatasetTaskOutcomeSample,
    DatasetTimingSpec, Frame, ImuSample, PointCloud, PoseSample, StreamId, SubscriptionCursor,
    DATASET_ACTION_ENCODING, DATASET_IMU_ENCODING, DATASET_TASK_OUTCOME_ENCODING,
    DATASET_TRANSFORM_ENCODING,
};
use rne_sensor::{Sensor, SensorKind};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::error::Error;
use std::fs;
use std::io;
use std::path::{Path, PathBuf};

const MAX_EPISODE_STEPS: u64 = 180;
const IMU_STREAM: StreamId = StreamId::new(100);
const LIDAR_STREAM: StreamId = StreamId::new(200);
const ACTION_STREAM: StreamId = StreamId::new(300);
const OUTCOME_STREAM: StreamId = StreamId::new(301);
const TRANSFORM_STREAM: StreamId = StreamId::new(302);
const LIDAR_POINT_RESOLUTION_M: f64 = 0.000_001;
const LIDAR_INTENSITY_RESOLUTION: f64 = 0.000_001;

#[derive(Debug, Serialize)]
struct CaptureSummary {
    schema_version: u32,
    dataset_id: String,
    task_spec_sha256: String,
    manifest_sha256: String,
    shard_sha256: String,
    stream_count: u64,
    record_count: u64,
    sample_count: u64,
    dropped_count: u64,
    imu_samples: u64,
    lidar_samples: u64,
    action_samples: u64,
    outcome_samples: u64,
    transform_samples: u64,
    terminated: bool,
    truncated: bool,
}

fn main() -> Result<(), Box<dyn Error>> {
    let mut arguments = std::env::args_os().skip(1);
    let output = arguments.next().map(PathBuf::from).ok_or_else(|| {
        io::Error::other("usage: 73_diff_drive_dataset_capture OUTPUT_DIR [--verify-golden]")
    })?;
    let verify_golden = match arguments.next() {
        None => false,
        Some(argument) if argument == "--verify-golden" => true,
        Some(argument) => {
            return Err(io::Error::other(format!(
                "unknown argument {}; expected --verify-golden",
                PathBuf::from(argument).display()
            ))
            .into());
        }
    };
    if arguments.next().is_some() {
        return Err(io::Error::other("too many arguments").into());
    }

    let workspace = Path::new(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = workspace.join("assets/scenes/dataset_diff_drive.rne.scene.toml");
    let robot_path = workspace.join("assets/robots/dataset_diff_drive.rne.robot.toml");
    let task_path = workspace.join("assets/tasks/diff_drive_goal.task.json");
    let reward = DiffDriveRewardConfig::default();
    let task_bytes = fs::read(&task_path)?;
    let committed_task: TaskSpec = serde_json::from_slice(&task_bytes)?;
    committed_task.validate()?;
    let expected_task = diff_drive_goal_task_spec(MAX_EPISODE_STEPS, reward);
    if committed_task != expected_task {
        return Err(io::Error::other("committed TaskSpec does not match the Rust contract").into());
    }

    let mut environment = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
        max_steps: MAX_EPISODE_STEPS,
        goal_x_m: 1.0,
        reward,
        scene_path: Some(scene_path.clone()),
        rng_seed: 42,
        ..DiffDriveEpisodeConfig::default()
    });
    let initial = environment.reset();
    if initial.is_done() {
        return Err(io::Error::other("reference episode ended during reset").into());
    }

    let simulation = environment.simulation();
    let robot = *simulation.robot();
    let imu_sensor = simulation
        .world()
        .get::<Sensor>(robot.base_link)
        .cloned()
        .ok_or_else(|| io::Error::other("reference robot has no IMU"))?;
    let lidar_entity = simulation
        .lidar_mounts()
        .first()
        .map(|mount| mount.lidar)
        .ok_or_else(|| io::Error::other("reference robot has no LiDAR"))?;
    let lidar_sensor = simulation
        .world()
        .get::<Sensor>(lidar_entity)
        .cloned()
        .ok_or_else(|| io::Error::other("reference LiDAR has no Sensor component"))?;
    if imu_sensor.stream_id != IMU_STREAM || lidar_sensor.stream_id != LIDAR_STREAM {
        return Err(io::Error::other("reference sensor stream IDs changed").into());
    }

    let mut manifest = DatasetManifest::new(
        "rne-diff-drive-reference-v1",
        sha256(&task_bytes),
        simulation.fixed_delta().ticks(),
        environment.world_seed(),
        stream_specs(&imu_sensor, &lidar_sensor)?,
    );
    manifest.assets = vec![
        dataset_asset(
            "robot_model",
            "assets/robots/dataset_diff_drive.rne.robot.toml",
            &robot_path,
        )?,
        dataset_asset(
            "scene",
            "assets/scenes/dataset_diff_drive.rne.scene.toml",
            &scene_path,
        )?,
        dataset_asset(
            "task_spec",
            "assets/tasks/diff_drive_goal.task.json",
            &task_path,
        )?,
    ];
    manifest.randomization = vec![DatasetRandomizationDecision {
        key: "goal_x_m".into(),
        seed: 42,
        value: DatasetRandomizationValue::Scalar {
            value: 1.0,
            unit: "m".into(),
        },
    }];
    if let Some(parent) = output
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }
    let mut writer = DatasetBundleWriter::create(&output, manifest)?;
    let mut imu_cursor = SubscriptionCursor::default();
    let mut lidar_cursor = SubscriptionCursor::default();
    let mut imu_sequence = 0_u64;
    let mut lidar_sequence = 0_u64;
    let mut terminal = None;

    for sequence in 0..MAX_EPISODE_STEPS {
        let action = DiffDriveAction::forward(6.0);
        let action_time = environment.simulation().sim_time();
        writer.write_action(&Frame::new(
            ACTION_STREAM,
            robot.robot,
            sequence,
            action_time,
            DatasetActionSample {
                values: vec![action.left_velocity_rad_s, action.right_velocity_rad_s],
            },
        ))?;

        let step = environment.step(action);
        drain_sensor_frames(
            environment.simulation().data_bus(),
            &mut writer,
            &mut imu_cursor,
            &mut lidar_cursor,
            &mut imu_sequence,
            &mut lidar_sequence,
        )?;
        let completed_time = environment.simulation().sim_time();
        writer.write_transform(&Frame::new(
            TRANSFORM_STREAM,
            robot.base_link,
            sequence,
            completed_time,
            PoseSample {
                position_m: rne_math::Vec3::new(
                    step.observation.base_x_m,
                    step.observation.base_y_m,
                    step.observation.base_z_m,
                ),
                yaw_rad: step.observation.base_yaw_rad,
            },
        ))?;
        writer.write_task_outcome(&Frame::new(
            OUTCOME_STREAM,
            robot.robot,
            sequence,
            completed_time,
            DatasetTaskOutcomeSample {
                episode_index: 0,
                step_in_episode: sequence + 1,
                reward: step.reward,
                cumulative_reward: environment.total_reward(),
                terminated: step.terminated,
                truncated: step.truncated,
                success: Some(step.terminated),
            },
        ))?;
        if step.is_done() {
            terminal = Some((step.terminated, step.truncated));
            break;
        }
    }

    let (terminated, truncated) = terminal
        .ok_or_else(|| io::Error::other("reference episode did not reach a terminal state"))?;
    if !terminated || truncated || lidar_sequence == 0 || imu_sequence == 0 {
        return Err(io::Error::other("reference capture acceptance criteria failed").into());
    }
    writer.finish()?;

    let bundle = DatasetBundle::open(&output)?;
    let verification = bundle.verify()?;
    let manifest = bundle.manifest();
    let shard = &manifest.shards[0];
    let summary = CaptureSummary {
        schema_version: 1,
        dataset_id: manifest.dataset_id.clone(),
        task_spec_sha256: manifest.task_spec_sha256.clone(),
        manifest_sha256: manifest.content_sha256.clone(),
        shard_sha256: shard.sha256.clone(),
        stream_count: verification.stream_count,
        record_count: verification.record_count,
        sample_count: verification.sample_count,
        dropped_count: verification.dropped_count,
        imu_samples: stream_samples(manifest, IMU_STREAM),
        lidar_samples: stream_samples(manifest, LIDAR_STREAM),
        action_samples: stream_samples(manifest, ACTION_STREAM),
        outcome_samples: stream_samples(manifest, OUTCOME_STREAM),
        transform_samples: stream_samples(manifest, TRANSFORM_STREAM),
        terminated,
        truncated,
    };
    let summary_json = format!("{}\n", serde_json::to_string_pretty(&summary)?);
    if verify_golden {
        let golden = fs::read_to_string(
            workspace.join("tests/golden/datasets/diff-drive-reference-summary-v1.json"),
        )?;
        if summary_json != golden {
            return Err(io::Error::other(format!(
                "reference capture does not match the cross-platform golden\n{summary_json}"
            ))
            .into());
        }
    }
    print!("{summary_json}");
    Ok(())
}

fn drain_sensor_frames(
    bus: &rne_data::InMemoryDataBus,
    writer: &mut DatasetBundleWriter,
    imu_cursor: &mut SubscriptionCursor,
    lidar_cursor: &mut SubscriptionCursor,
    imu_sequence: &mut u64,
    lidar_sequence: &mut u64,
) -> Result<(), Box<dyn Error>> {
    while let Some(mut frame) = bus.next::<ImuSample>(IMU_STREAM, imu_cursor) {
        frame.sequence = *imu_sequence;
        writer.write_imu(&frame)?;
        *imu_sequence += 1;
    }
    while let Some(mut frame) = bus.next::<PointCloud>(LIDAR_STREAM, lidar_cursor) {
        frame.sequence = *lidar_sequence;
        canonicalize_lidar(&mut frame.payload);
        writer.write_lidar_point_cloud(&frame)?;
        *lidar_sequence += 1;
    }
    Ok(())
}

fn stream_specs(imu: &Sensor, lidar: &Sensor) -> Result<Vec<DatasetStreamSpec>, Box<dyn Error>> {
    let SensorKind::Imu(imu_spec) = &imu.kind else {
        return Err(io::Error::other("stream 100 is not an IMU").into());
    };
    let SensorKind::Lidar(lidar_spec) = &lidar.kind else {
        return Err(io::Error::other("stream 200 is not a LiDAR").into());
    };
    if !imu_spec.is_ideal() {
        return Err(io::Error::other("reference IMU must remain ideal").into());
    }

    let fixed_step_ticks = 1_000_000_000 / 60;
    Ok(vec![
        DatasetStreamSpec {
            stream_id: IMU_STREAM,
            name: "base_imu".into(),
            kind: DatasetStreamKind::Imu,
            payload_encoding: DATASET_IMU_ENCODING.into(),
            source_entity: "base_link".into(),
            frame_id: "base_link".into(),
            fields: vec![
                field("angular_velocity_rad_s", "f64[3]", "rad/s"),
                field("linear_acceleration_m_s2", "f64[3]", "m/s^2"),
            ],
            calibration: Some(DatasetCalibration {
                model: "body_aligned.v1".into(),
                reference_frame: "base_link".into(),
                parameters: BTreeMap::new(),
            }),
            timing: timing(imu.period().ticks(), imu.latency_ticks),
            noise: Some(DatasetNoiseSpec {
                model: "rne.imu.ideal.v1".into(),
                seed: imu_spec.seed,
                parameters: BTreeMap::new(),
            }),
        },
        DatasetStreamSpec {
            stream_id: LIDAR_STREAM,
            name: "base_lidar".into(),
            kind: DatasetStreamKind::LidarPointCloud,
            payload_encoding: "rne.transport.lidar_point_cloud.v1".into(),
            source_entity: "lidar".into(),
            frame_id: "lidar".into(),
            fields: vec![
                field("points_m", "f64[][3]", "m"),
                field("intensities", "f32[]", "1"),
                field("ray_indices", "u32[]", "1"),
                field("return_indices", "u8[]", "1"),
                field("channel_indices", "u16[]", "1"),
                field("timestamps_s", "f64[]", "s"),
            ],
            calibration: Some(DatasetCalibration {
                model: "spherical_lidar.v1".into(),
                reference_frame: "lidar".into(),
                parameters: BTreeMap::from([
                    ("channel_count".into(), f64::from(lidar_spec.channel_count)),
                    ("max_angle_rad".into(), lidar_spec.max_angle_rad),
                    ("max_range_m".into(), lidar_spec.max_range_m),
                    ("min_angle_rad".into(), lidar_spec.min_angle_rad),
                    ("min_range_m".into(), lidar_spec.min_range_m),
                    ("point_resolution_m".into(), LIDAR_POINT_RESOLUTION_M),
                    ("ray_count".into(), f64::from(lidar_spec.ray_count)),
                    ("rotation_period_s".into(), lidar_spec.rotation_period_s),
                ]),
            }),
            timing: timing(lidar.period().ticks(), lidar.latency_ticks),
            noise: Some(DatasetNoiseSpec {
                model: "rne.lidar.physical.v1".into(),
                seed: lidar_spec.seed,
                parameters: BTreeMap::from([
                    ("dropout_probability".into(), lidar_spec.dropout_probability),
                    (
                        "intensity_noise_stddev".into(),
                        lidar_spec.intensity_noise_stddev,
                    ),
                    ("intensity_resolution".into(), LIDAR_INTENSITY_RESOLUTION),
                    (
                        "range_noise_stddev_m".into(),
                        lidar_spec.range_noise_stddev_m,
                    ),
                    ("solar_noise_floor".into(), lidar_spec.solar_noise_floor),
                ]),
            }),
        },
        plain_stream(
            ACTION_STREAM,
            "wheel_action",
            DatasetStreamKind::Action,
            DATASET_ACTION_ENCODING,
            "dataset_diff_drive",
            vec![field("wheel_velocity_rad_s", "f64[2]", "rad/s")],
            fixed_step_ticks,
        ),
        plain_stream(
            OUTCOME_STREAM,
            "task_outcome",
            DatasetStreamKind::TaskOutcome,
            DATASET_TASK_OUTCOME_ENCODING,
            "dataset_diff_drive",
            vec![
                field("reward", "f64", "reward"),
                field("cumulative_reward", "f64", "reward"),
                field("terminated", "bool", "boolean"),
                field("truncated", "bool", "boolean"),
                field("success", "optional_bool", "boolean"),
            ],
            fixed_step_ticks,
        ),
        plain_stream(
            TRANSFORM_STREAM,
            "base_transform",
            DatasetStreamKind::Transform,
            DATASET_TRANSFORM_ENCODING,
            "base_link",
            vec![
                field("position_m", "f64[3]", "m"),
                field("yaw_rad", "f64", "rad"),
            ],
            fixed_step_ticks,
        ),
    ])
}

fn plain_stream(
    stream_id: StreamId,
    name: &str,
    kind: DatasetStreamKind,
    encoding: &str,
    source_entity: &str,
    fields: Vec<DatasetFieldSpec>,
    nominal_period_ticks: u64,
) -> DatasetStreamSpec {
    DatasetStreamSpec {
        stream_id,
        name: name.into(),
        kind,
        payload_encoding: encoding.into(),
        source_entity: source_entity.into(),
        frame_id: "world".into(),
        fields,
        calibration: None,
        timing: timing(nominal_period_ticks, 0),
        noise: None,
    }
}

fn timing(nominal_period_ticks: u64, latency_ticks: u64) -> DatasetTimingSpec {
    DatasetTimingSpec {
        nominal_period_ticks,
        latency: DatasetLatencySpec {
            model: DatasetLatencyModel::Fixed,
            fixed_ticks: Some(latency_ticks),
            max_ticks: latency_ticks,
        },
        gap_policy: DatasetGapPolicy::ExplicitRecords,
    }
}

fn field(name: &str, dtype: &str, unit: &str) -> DatasetFieldSpec {
    DatasetFieldSpec {
        name: name.into(),
        dtype: dtype.into(),
        unit: unit.into(),
    }
}

fn dataset_asset(role: &str, path: &str, source: &Path) -> Result<DatasetAsset, io::Error> {
    Ok(DatasetAsset {
        role: role.into(),
        path: path.into(),
        sha256: sha256(&fs::read(source)?),
    })
}

fn stream_samples(manifest: &rne_data::DatasetManifest, stream_id: StreamId) -> u64 {
    manifest.shards[0]
        .streams
        .iter()
        .find(|summary| summary.stream_id == stream_id)
        .map_or(0, |summary| summary.sample_count)
}

fn canonicalize_lidar(cloud: &mut PointCloud) {
    for point in &mut cloud.points_m {
        point.x = quantize(point.x, LIDAR_POINT_RESOLUTION_M);
        point.y = quantize(point.y, LIDAR_POINT_RESOLUTION_M);
        point.z = quantize(point.z, LIDAR_POINT_RESOLUTION_M);
    }
    for intensity in &mut cloud.intensities {
        *intensity = quantize(f64::from(*intensity), LIDAR_INTENSITY_RESOLUTION) as f32;
    }
}

fn quantize(value: f64, resolution: f64) -> f64 {
    (value / resolution).round() * resolution
}

fn sha256(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

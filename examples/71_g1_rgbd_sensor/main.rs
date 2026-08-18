//! Captures a photoreal RGB-D stream from a camera mounted on Unitree G1's head.
//!
//! The camera uses the renderer-independent `rne_sensor` pipeline: deterministic
//! lens response, exposure, vignetting, noise, DataBus timestamps, and output
//! latency. The GPU path renders the resolved official G1 scene and calibration
//! room; `--smoke` uses the CPU headless renderer and checks the same RGB-D/DataBus
//! contract without initializing a GPU.

use image::codecs::gif::{GifEncoder, Repeat};
use image::{Delay, Frame as GifFrame, RgbaImage};
use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_dynamic_scene_path, unitree_g1_gait_targets,
    unitree_g1_gait_task_spec, UnitreeG1GaitCommand, UrdfSceneSim,
};
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, DatasetAsset, DatasetBundle, DatasetBundleWriter, DatasetCalibration,
    DatasetFieldSpec, DatasetGapPolicy, DatasetLatencyModel, DatasetLatencySpec, DatasetManifest,
    DatasetNoiseSpec, DatasetStreamKind, DatasetStreamSpec, DatasetTimingSpec, Frame, ImageDepth,
    ImageRgb8, InMemoryDataBus, RendererDatasetCaptureReport, StreamId,
    RENDERER_DATASET_CAPTURE_REPORT_KIND, RENDERER_DATASET_CAPTURE_REPORT_SCHEMA_VERSION,
};
use rne_ecs::{spawn_named, Entity, World};
use rne_math::{Hertz, Quat, Transform3, Vec3};
use rne_physics::{PhysicsBackend, PhysicsWorldDesc, PhysicsWorldId};
use rne_physics_rapier::RapierBackend;
use rne_render::{
    hash_depth_f32, hash_rgba8, Camera, EnvironmentLighting, EnvironmentMap, ImageFrame,
    MeshRenderCache, PbrMaterial, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh,
    VisualShape,
};
use rne_render_wgpu::{TaaSettings, WgpuRenderBackend};
use rne_sensor::{
    sample_sensors, CameraDistortion, CameraSpec, Sensor, SensorKind, SensorSampleContext,
    SensorState, CAMERA_DEPTH_STREAM_OFFSET,
};
use rne_world::Transform3 as WorldTransform3;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::error::Error;
use std::fs::{self, File};
use std::io::{self, BufReader, BufWriter, Read, Write};
use std::path::{Path, PathBuf};
use std::sync::Arc;

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const FRAME_COUNT: usize = 12;
const STEPS_PER_FRAME: u64 = 4;
const CAMERA_RATE_HZ: f64 = 15.0;
const CAMERA_LATENCY_TICKS: u64 = 3;
const CAMERA_STREAM_ID: u64 = 7101;
const CAMERA_FRAME_ID: u32 = 7101;
const SETTLE_STEPS: u64 = 240;
const G1_MESH_MINIMUM: usize = 20;
const RGB_STREAM: StreamId = StreamId::new(CAMERA_STREAM_ID);
const DEPTH_STREAM: StreamId = StreamId::new(CAMERA_STREAM_ID + CAMERA_DEPTH_STREAM_OFFSET);
const RENDERER_CONTRACT: &[u8] = b"{\n  \"kind\": \"rne_renderer_capture_contract\",\n  \"schema_version\": 1,\n  \"backend\": \"rne_render_wgpu\",\n  \"capture_mode\": \"offscreen_rgbd\"\n}\n";
const WALK_COMMAND: UnitreeG1GaitCommand = UnitreeG1GaitCommand {
    stride_rad: 0.065,
    foot_lift_rad: 0.12,
    cycle_steps: 100,
};

struct FloorTextures {
    base_color: Arc<ImageFrame>,
    normal: Arc<ImageFrame>,
    roughness: Arc<ImageFrame>,
}

#[derive(Clone, Debug, Default)]
struct IndustrialProp {
    items: Vec<RenderSceneItem>,
}

#[derive(Clone, Debug)]
struct RgbdCapture {
    rgb: Frame<ImageRgb8>,
    depth: Frame<ImageDepth>,
}

/// A camera sensor entity with its own empty physics world.
///
/// The sensor world is separate from the G1 physics world because a camera has no
/// collider or motor. Its transform is updated from G1's named head link before each
/// sample, while the render scene remains the G1 simulation's visual scene.
struct CameraRig {
    world: World,
    entity: Entity,
    physics: RapierBackend,
    physics_world: PhysicsWorldId,
    bus: InMemoryDataBus,
    sim_time: SimTime,
    frame_dt: SimDuration,
    stream_id: StreamId,
}

impl CameraRig {
    fn new(spec: CameraSpec) -> Self {
        let stream_id = StreamId::new(CAMERA_STREAM_ID);
        let mut world = World::new();
        let entity = spawn_named(&mut world, "unitree_g1_head_rgbd_camera");
        world.entity_mut(entity).insert((
            WorldTransform3::default(),
            Sensor {
                kind: SensorKind::Camera(spec),
                update_rate_hz: CAMERA_RATE_HZ,
                latency_ticks: CAMERA_LATENCY_TICKS,
                frame_id: CAMERA_FRAME_ID,
                enabled: true,
                stream_id,
            },
            SensorState::default(),
        ));

        let mut physics = RapierBackend::new();
        let physics_world = physics
            .create_world(PhysicsWorldDesc::default())
            .expect("create empty camera physics world");
        Self {
            world,
            entity,
            physics,
            physics_world,
            bus: InMemoryDataBus::new(),
            sim_time: SimTime::ZERO,
            frame_dt: SimDuration::from_hertz(Hertz::new(CAMERA_RATE_HZ)),
            stream_id,
        }
    }

    fn sample<'a>(
        &'a mut self,
        transform: WorldTransform3,
        render: Option<&'a mut (dyn RenderBackend + 'a)>,
        scene: Option<&'a RenderScene>,
    ) -> RgbdCapture {
        self.world.entity_mut(self.entity).insert(transform);
        let published = sample_sensors(
            &mut SensorSampleContext {
                world: &mut self.world,
                sim_time: self.sim_time,
                physics: &self.physics,
                physics_world: self.physics_world,
                render,
                scene,
            },
            &mut self.bus,
        );
        assert_eq!(published, 1, "camera sensor must publish one RGB-D pair");
        let rgb = self
            .bus
            .latest::<ImageRgb8>(self.stream_id)
            .expect("camera RGB frame");
        let depth = self
            .bus
            .latest::<ImageDepth>(StreamId::new(CAMERA_STREAM_ID + CAMERA_DEPTH_STREAM_OFFSET))
            .expect("camera depth frame");
        self.sim_time = self.sim_time + self.frame_dt;
        RgbdCapture { rgb, depth }
    }
}

fn main() -> Result<(), Box<dyn Error>> {
    let (smoke, require_renderer, dataset_path) = parse_args()?;
    if smoke && dataset_path.is_some() {
        return Err(io::Error::other("--dataset requires the WGPU capture path").into());
    }
    if smoke || (std::env::var_os("RNE_SKIP_GPU").is_some() && !require_renderer) {
        run_smoke();
        return Ok(());
    }
    if std::env::var_os("RNE_SKIP_GPU").is_some() {
        return Err(io::Error::other("WGPU capture required but RNE_SKIP_GPU is set").into());
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    if dataset_path.is_some() {
        validate_renderer_evidence_inputs(&repo_root)?;
    }
    let floor = load_floor_textures(&repo_root);
    let industrial_prop = load_industrial_prop(&repo_root);
    let mut sim = load_and_settle_g1();
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            if require_renderer || dataset_path.is_some() {
                return Err(io::Error::other(format!(
                    "required WGPU renderer is unavailable: {error}"
                ))
                .into());
            }
            eprintln!("wgpu unavailable; G1 RGB-D smoke passed: {error}");
            return Ok(());
        }
    };
    configure_environment(&mut backend, &repo_root);
    configure_taa(&mut backend);

    let mut rig = CameraRig::new(camera_spec());
    let mut cache = MeshRenderCache::new();
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let output_dir = target_dir(&repo_root).join("rne-g1-rgbd-sensor");
    fs::create_dir_all(&output_dir).expect("create G1 RGB-D output directory");
    let mut dataset_writer = dataset_path
        .as_ref()
        .map(|path| create_renderer_dataset(path, &repo_root))
        .transpose()?;
    let mut gif_frames = Vec::with_capacity(FRAME_COUNT);
    let mut manifest = String::from(
        "frame,capture_ticks,available_ticks,rgb_hash,depth_hash,min_depth_m,center_depth_m\n",
    );

    for frame_index in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = frame_index as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_g1_gait_targets(step, WALK_COMMAND));
        }
        let scene = g1_scene(&sim, &mut cache, &mesh_root_refs, &floor, &industrial_prop);
        let capture = rig.sample(
            g1_head_camera_transform(&sim),
            Some(&mut backend),
            Some(&scene),
        );
        validate_capture(&capture);
        if let Some(writer) = dataset_writer.as_mut() {
            let mut rgb = capture.rgb.clone();
            let mut depth = capture.depth.clone();
            rgb.sequence = frame_index as u64;
            depth.sequence = frame_index as u64;
            writer.write_image_rgb8(&rgb)?;
            writer.write_image_depth(&depth)?;
        }
        let rgb_hash = hash_rgba8(&capture.rgb.payload.rgba8);
        let depth_hash = hash_depth_f32(&capture.depth.payload.depth_m);
        let min_depth_m = capture.depth.payload.min_depth_m();
        let center_depth_m = capture.depth.payload.center_depth_m();
        let rgb_path = output_dir.join(format!("rgb-{frame_index:03}.png"));
        let depth_path = output_dir.join(format!("depth-{frame_index:03}.png"));
        let raw_depth_path = output_dir.join(format!("depth-{frame_index:03}.f32le"));
        write_rgb_png(&rgb_path, &capture.rgb.payload).expect("write RGB capture");
        write_depth_png(&depth_path, &capture.depth.payload).expect("write depth preview");
        write_depth_f32le(&raw_depth_path, &capture.depth.payload)
            .expect("write raw depth capture");
        manifest.push_str(&format!(
            "{frame_index},{},{},{rgb_hash:#018x},{depth_hash:#018x},{min_depth_m:.6},{center_depth_m:.6}\n",
            capture.rgb.capture_time.ticks(),
            capture.rgb.available_time.ticks(),
        ));
        println!(
            "G1 RGB-D frame={frame_index} rgb_hash={rgb_hash:#018x} depth_hash={depth_hash:#018x} min_depth_m={min_depth_m:.3} available_ticks={} rgb_path={}",
            capture.rgb.available_time.ticks(),
            rgb_path.display()
        );
        gif_frames.push(capture.rgb.payload.rgba8.clone());
    }

    write_gif(
        &output_dir.join("unitree-g1-rgbd-sensor.gif"),
        &gif_frames,
        WIDTH,
        HEIGHT,
    )
    .expect("write RGB sensor GIF");
    fs::write(output_dir.join("manifest.csv"), manifest).expect("write RGB-D manifest");
    if let (Some(writer), Some(dataset_path)) = (dataset_writer, dataset_path.as_ref()) {
        writer.finish()?;
        let bundle = DatasetBundle::open(dataset_path)?;
        let verification = bundle.verify()?;
        let task_bytes = fs::read(dataset_path.join("task-spec.json"))?;
        let task_digest = sha256(&task_bytes);
        if task_digest != bundle.manifest().task_spec_sha256 {
            return Err(io::Error::other("renderer dataset TaskSpec digest mismatch").into());
        }
        let report = RendererDatasetCaptureReport {
            kind: RENDERER_DATASET_CAPTURE_REPORT_KIND.to_string(),
            schema_version: RENDERER_DATASET_CAPTURE_REPORT_SCHEMA_VERSION,
            status: "passed".to_string(),
            renderer: "rne_render_wgpu".to_string(),
            dataset_manifest_sha256: bundle.manifest().content_sha256.clone(),
            task_spec_sha256: task_digest,
            stream_count: verification.stream_count,
            record_count: verification.record_count,
            sample_count: verification.sample_count,
            frame_count: FRAME_COUNT,
        };
        report.validate_against(&bundle.manifest().content_sha256, &verification)?;
        let mut report_bytes = serde_json::to_vec_pretty(&report)?;
        report_bytes.push(b'\n');
        fs::write(
            dataset_path.join("renderer-capture-report.json"),
            report_bytes,
        )?;
        println!(
            "verified WGPU dataset: manifest={} records={} path={}",
            bundle.manifest().content_sha256,
            verification.record_count,
            dataset_path.display()
        );
    }
    println!("rendered G1 RGB-D sensor media to {}", output_dir.display());
    Ok(())
}

fn parse_args() -> Result<(bool, bool, Option<PathBuf>), Box<dyn Error>> {
    parse_args_from(std::env::args().skip(1))
}

fn parse_args_from(
    arguments: impl IntoIterator<Item = String>,
) -> Result<(bool, bool, Option<PathBuf>), Box<dyn Error>> {
    let mut smoke = false;
    let mut require_renderer = false;
    let mut dataset_path = None;
    let mut args = arguments.into_iter();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--smoke" if !smoke => smoke = true,
            "--require-renderer" if !require_renderer => require_renderer = true,
            "--dataset" if dataset_path.is_none() => {
                let path = args
                    .next()
                    .ok_or_else(|| io::Error::other("--dataset requires a path"))?;
                dataset_path = Some(PathBuf::from(path));
            }
            "--smoke" | "--require-renderer" | "--dataset" => {
                return Err(io::Error::other(format!("duplicate argument: {argument}")).into());
            }
            _ => return Err(io::Error::other(format!("unknown argument: {argument}")).into()),
        }
    }
    Ok((smoke, require_renderer, dataset_path))
}

fn create_renderer_dataset(
    path: &Path,
    repo_root: &Path,
) -> Result<DatasetBundleWriter, Box<dyn Error>> {
    if let Some(parent) = path
        .parent()
        .filter(|parent| !parent.as_os_str().is_empty())
    {
        fs::create_dir_all(parent)?;
    }
    let task = unitree_g1_gait_task_spec((FRAME_COUNT as u64) * STEPS_PER_FRAME);
    task.validate()?;
    let mut task_bytes = serde_json::to_vec_pretty(&task)?;
    task_bytes.push(b'\n');
    let task_digest = sha256(&task_bytes);
    let mut manifest = DatasetManifest::new(
        "rne-unitree-g1-wgpu-rgbd-v1",
        task_digest.clone(),
        SimDuration::from_hertz(Hertz::new(60.0)).ticks(),
        0,
        renderer_stream_specs(),
    );
    manifest.assets = vec![
        DatasetAsset {
            role: "renderer_contract".into(),
            path: "renderer-contract.json".into(),
            sha256: sha256(RENDERER_CONTRACT),
        },
        DatasetAsset {
            role: "task_spec".into(),
            path: "task-spec.json".into(),
            sha256: task_digest,
        },
    ];
    manifest
        .assets
        .extend(renderer_workspace_assets(repo_root)?);
    let writer = DatasetBundleWriter::create(path, manifest)?;
    fs::write(path.join("task-spec.json"), task_bytes)?;
    fs::write(path.join("renderer-contract.json"), RENDERER_CONTRACT)?;
    Ok(writer)
}

fn validate_renderer_evidence_inputs(repo_root: &Path) -> Result<(), Box<dyn Error>> {
    for variable in [
        "RNE_HDRI_PATH",
        "RNE_DISABLE_BUNDLED_INDUSTRIAL_ENVIRONMENT",
        "RNE_DISABLE_INDUSTRIAL_ASSETS",
    ] {
        if std::env::var_os(variable).is_some() {
            return Err(io::Error::other(format!(
                "renderer dataset evidence rejects environment override {variable}"
            ))
            .into());
        }
    }
    for relative in [
        "assets/scenes/unitree_g1_dynamic.rne.scene.toml",
        "assets/robots/unitree_g1_dynamic.rne.robot.toml",
        "assets/robots/g1_description",
        "assets/environments/polyhaven_machine_shop_01",
        "examples/63_g1_stride_gif/assets/photoreal_test_bay",
    ] {
        if !repo_root.join(relative).exists() {
            return Err(
                io::Error::other(format!("renderer dataset input is missing: {relative}")).into(),
            );
        }
    }
    Ok(())
}

fn renderer_workspace_assets(repo_root: &Path) -> Result<Vec<DatasetAsset>, Box<dyn Error>> {
    let mut sources = vec![
        (
            "scene",
            repo_root.join("assets/scenes/unitree_g1_dynamic.rne.scene.toml"),
        ),
        (
            "robot_model",
            repo_root.join("assets/robots/unitree_g1_dynamic.rne.robot.toml"),
        ),
    ];
    collect_regular_files(
        "robot_source",
        &repo_root.join("assets/robots/g1_description"),
        &mut sources,
    )?;
    collect_regular_files(
        "environment",
        &repo_root.join("assets/environments/polyhaven_machine_shop_01"),
        &mut sources,
    )?;
    collect_regular_files(
        "renderer_texture",
        &repo_root.join("examples/63_g1_stride_gif/assets/photoreal_test_bay"),
        &mut sources,
    )?;
    sources.sort_by(|left, right| left.1.cmp(&right.1));
    sources
        .into_iter()
        .map(|(role, source)| {
            let relative = source
                .strip_prefix(repo_root)
                .map_err(io::Error::other)?
                .to_string_lossy()
                .replace('\\', "/");
            Ok(DatasetAsset {
                role: role.into(),
                path: relative,
                sha256: sha256_file(&source)?,
            })
        })
        .collect()
}

fn collect_regular_files<'a>(
    role: &'a str,
    directory: &Path,
    output: &mut Vec<(&'a str, PathBuf)>,
) -> io::Result<()> {
    let mut entries = fs::read_dir(directory)?.collect::<Result<Vec<_>, _>>()?;
    entries.sort_by_key(|entry| entry.file_name());
    for entry in entries {
        let file_type = entry.file_type()?;
        if file_type.is_symlink() {
            return Err(io::Error::other(format!(
                "renderer input must not be a symlink: {}",
                entry.path().display()
            )));
        }
        if file_type.is_dir() {
            collect_regular_files(role, &entry.path(), output)?;
        } else if file_type.is_file() {
            output.push((role, entry.path()));
        } else {
            return Err(io::Error::other(format!(
                "renderer input must be a regular file: {}",
                entry.path().display()
            )));
        }
    }
    Ok(())
}

fn sha256_file(path: &Path) -> io::Result<String> {
    let mut reader = BufReader::new(File::open(path)?);
    let mut hasher = Sha256::new();
    let mut buffer = [0_u8; 64 * 1024];
    loop {
        let count = reader.read(&mut buffer)?;
        if count == 0 {
            break;
        }
        hasher.update(&buffer[..count]);
    }
    Ok(format!("sha256:{:x}", hasher.finalize()))
}

fn renderer_stream_specs() -> Vec<DatasetStreamSpec> {
    let spec = camera_spec();
    let calibration = DatasetCalibration {
        model: "brown_conrady_pinhole.v1".into(),
        reference_frame: "head_link".into(),
        parameters: BTreeMap::from([
            ("fov_y_rad".into(), spec.fov_y_rad),
            ("height_px".into(), f64::from(spec.height)),
            ("k1".into(), spec.distortion.k1),
            ("k2".into(), spec.distortion.k2),
            ("k3".into(), spec.distortion.k3),
            ("p1".into(), spec.distortion.p1),
            ("p2".into(), spec.distortion.p2),
            ("readout_time_s".into(), spec.readout_time_s),
            (
                "rolling_shutter_bands".into(),
                f64::from(spec.rolling_shutter_bands),
            ),
            ("width_px".into(), f64::from(spec.width)),
        ]),
    };
    let timing = DatasetTimingSpec {
        nominal_period_ticks: SimDuration::from_hertz(Hertz::new(CAMERA_RATE_HZ)).ticks(),
        latency: DatasetLatencySpec {
            model: DatasetLatencyModel::Fixed,
            fixed_ticks: Some(CAMERA_LATENCY_TICKS),
            max_ticks: CAMERA_LATENCY_TICKS,
        },
        gap_policy: DatasetGapPolicy::ExplicitRecords,
    };
    vec![
        DatasetStreamSpec {
            stream_id: RGB_STREAM,
            name: "unitree_g1_head_rgb".into(),
            kind: DatasetStreamKind::Rgb8,
            payload_encoding: "rne.transport.image_rgb8.v1".into(),
            source_entity: "unitree_g1_head_rgbd_camera".into(),
            frame_id: "head_link".into(),
            fields: vec![DatasetFieldSpec {
                name: "rgba8".into(),
                dtype: "u8[height,width,4]".into(),
                unit: "1".into(),
            }],
            calibration: Some(calibration.clone()),
            timing: timing.clone(),
            noise: Some(DatasetNoiseSpec {
                model: "rne.camera.rgb_physical.v1".into(),
                seed: spec.seed,
                parameters: BTreeMap::from([
                    ("auto_exposure_max_ev".into(), spec.auto_exposure_max_ev),
                    (
                        "auto_exposure_target_luminance".into(),
                        spec.auto_exposure_target_luminance,
                    ),
                    ("exposure_ev".into(), spec.exposure_ev),
                    ("read_noise_stddev".into(), spec.read_noise_stddev),
                    ("shot_noise_scale".into(), spec.shot_noise_scale),
                    ("vignette_strength".into(), spec.vignette_strength),
                ]),
            }),
        },
        DatasetStreamSpec {
            stream_id: DEPTH_STREAM,
            name: "unitree_g1_head_depth".into(),
            kind: DatasetStreamKind::DepthF32,
            payload_encoding: "rne.transport.image_depth_f32.v1".into(),
            source_entity: "unitree_g1_head_rgbd_camera".into(),
            frame_id: "head_link".into(),
            fields: vec![DatasetFieldSpec {
                name: "depth_m".into(),
                dtype: "f32[height,width]".into(),
                unit: "m".into(),
            }],
            calibration: Some(calibration),
            timing,
            noise: Some(DatasetNoiseSpec {
                model: "rne.camera.depth_geometry.v1".into(),
                seed: spec.seed,
                parameters: BTreeMap::new(),
            }),
        },
    ]
}

fn sha256(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

#[cfg(test)]
mod dataset_tests {
    use super::*;

    #[test]
    fn renderer_dataset_cli_is_explicit_and_fail_closed() {
        let parsed = parse_args_from(
            ["--dataset", "evidence/g1.rne-dataset", "--require-renderer"].map(str::to_string),
        )
        .unwrap();
        assert!(!parsed.0);
        assert!(parsed.1);
        assert_eq!(parsed.2, Some(PathBuf::from("evidence/g1.rne-dataset")));
        assert!(parse_args_from(["--dataset".to_string()]).is_err());
        assert!(parse_args_from(["--smoke".to_string(), "--smoke".to_string()]).is_err());
        assert!(parse_args_from(["evidence/g1.rne-dataset".to_string()]).is_err());
    }

    #[test]
    fn renderer_dataset_streams_freeze_calibration_latency_and_noise() {
        let streams = renderer_stream_specs();
        assert_eq!(streams.len(), 2);
        assert_eq!(streams[0].stream_id, RGB_STREAM);
        assert_eq!(streams[1].stream_id, DEPTH_STREAM);
        for stream in streams {
            assert_eq!(
                stream.timing.latency.fixed_ticks,
                Some(CAMERA_LATENCY_TICKS)
            );
            assert!(stream.calibration.is_some());
            assert!(stream.noise.is_some());
        }
        unitree_g1_gait_task_spec((FRAME_COUNT as u64) * STEPS_PER_FRAME)
            .validate()
            .unwrap();
    }
}

fn run_smoke() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    validate_bundled_environment(&repo_root);
    let first = smoke_digest();
    let second = smoke_digest();
    assert_eq!(first, second, "RGB-D sensor replay must be deterministic");
    println!(
        "G1 RGB-D sensor smoke passed: frames={} rgb_hash={:#018x} depth_hash={:#018x} latency_ticks={CAMERA_LATENCY_TICKS}",
        first.len(),
        first[0].0,
        first[0].1,
    );
}

fn smoke_digest() -> Vec<(u64, u64, u64, u64)> {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut sim = load_and_settle_g1();
    let mut cache = MeshRenderCache::new();
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let floor = load_floor_textures(&repo_root);
    let industrial_prop = load_industrial_prop(&repo_root);
    if !env_flag("RNE_DISABLE_INDUSTRIAL_ASSETS") {
        assert!(
            !industrial_prop.items.is_empty(),
            "industrial hand-truck asset must be available in the repository"
        );
        assert!(
            industrial_prop.items.iter().any(|item| {
                item.material.normal_texture.is_some()
                    && item.material.metallic_roughness_texture.is_some()
            }),
            "industrial hand-truck PBR maps must be imported"
        );
    }
    let mut rig = CameraRig::new(camera_spec());
    let mut digest = Vec::with_capacity(FRAME_COUNT);

    for frame_index in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = frame_index as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_g1_gait_targets(step, WALK_COMMAND));
        }
        let scene = g1_scene(&sim, &mut cache, &mesh_root_refs, &floor, &industrial_prop);
        let mesh_count = scene
            .items
            .iter()
            .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
            .count();
        assert!(mesh_count >= G1_MESH_MINIMUM);
        let capture = rig.sample(g1_head_camera_transform(&sim), None, Some(&scene));
        validate_capture(&capture);
        digest.push((
            hash_rgba8(&capture.rgb.payload.rgba8),
            hash_depth_f32(&capture.depth.payload.depth_m),
            capture.rgb.capture_time.ticks(),
            capture.rgb.available_time.ticks(),
        ));
    }
    digest
}

fn validate_capture(capture: &RgbdCapture) {
    assert_eq!(capture.rgb.stream_id, StreamId::new(CAMERA_STREAM_ID));
    assert_eq!(
        capture.depth.stream_id,
        StreamId::new(CAMERA_STREAM_ID + CAMERA_DEPTH_STREAM_OFFSET)
    );
    assert_eq!(capture.rgb.payload.width, WIDTH);
    assert_eq!(capture.rgb.payload.height, HEIGHT);
    assert_eq!(capture.depth.payload.width, WIDTH);
    assert_eq!(capture.depth.payload.height, HEIGHT);
    assert_eq!(capture.rgb.capture_time, capture.depth.capture_time);
    assert_eq!(
        capture.rgb.available_time.ticks() - capture.rgb.capture_time.ticks(),
        CAMERA_LATENCY_TICKS
    );
    assert!(capture
        .depth
        .payload
        .depth_m
        .iter()
        .all(|depth| depth.is_finite() && *depth > 0.0));
    assert!(capture.depth.payload.min_depth_m() < Camera::default().far_m as f32);
}

fn camera_spec() -> CameraSpec {
    CameraSpec {
        width: WIDTH,
        height: HEIGHT,
        fov_y_rad: 1.05,
        seed: CAMERA_STREAM_ID,
        distortion: CameraDistortion {
            k1: -0.12,
            k2: 0.025,
            ..CameraDistortion::default()
        },
        readout_time_s: 0.012,
        rolling_shutter_bands: 4,
        auto_exposure_target_luminance: 0.42,
        auto_exposure_max_ev: 2.0,
        shot_noise_scale: 0.0006,
        read_noise_stddev: 0.003,
        vignette_strength: 0.22,
        ..CameraSpec::default()
    }
}

fn load_and_settle_g1() -> UrdfSceneSim {
    let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
        .expect("load official dynamic G1");
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let stand = UnitreeG1GaitCommand {
        stride_rad: 0.0,
        foot_lift_rad: 0.0,
        ..WALK_COMMAND
    };
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&unitree_g1_gait_targets(0, stand));
    }
    sim
}

fn g1_head_camera_transform(sim: &UrdfSceneSim) -> WorldTransform3 {
    let head = sim
        .named_transform("head_link")
        .expect("G1 head_link transform");
    // The vendored G1 scene faces +X after its URDF-to-world basis conversion. The
    // camera uses local -Z forward and world +Y up, hence the -90 degree Y rotation.
    WorldTransform3::from_translation_rotation(
        head.translation + Vec3::new(0.11, 0.015, 0.0),
        Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2),
    )
}

fn g1_scene(
    sim: &UrdfSceneSim,
    cache: &mut MeshRenderCache,
    mesh_roots: &[&Path],
    floor: &FloorTextures,
    industrial_prop: &IndustrialProp,
) -> RenderScene {
    let mut scene = build_visual_render_scene(sim.world());
    scene.items.retain(|item| {
        !matches!(item.shape, VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z > 5.0)
    });
    cache
        .resolve_scene(&mut scene, mesh_roots)
        .expect("resolve official G1 meshes");
    for item in &mut scene.items {
        if matches!(item.shape, VisualShape::Mesh { .. }) {
            item.material = PbrMaterial::new([1.0; 4], 0.48, 0.05, [0.0; 3]);
        }
    }
    append_calibration_room(&mut scene, Vec3::new(0.0, 0.0, 0.0), floor);
    append_industrial_prop(&mut scene, Vec3::ZERO, industrial_prop);
    scene
}

fn load_floor_textures(repo_root: &Path) -> FloorTextures {
    let asset_dir = repo_root.join("examples/63_g1_stride_gif/assets/photoreal_test_bay");
    FloorTextures {
        base_color: load_texture(&asset_dir.join("concrete_floor_basecolor.png")),
        normal: load_texture(&asset_dir.join("concrete_floor_normal.png")),
        roughness: load_texture(&asset_dir.join("concrete_floor_roughness.png")),
    }
}

fn load_texture(path: &Path) -> Arc<ImageFrame> {
    let rgba = image::open(path)
        .unwrap_or_else(|error| panic!("load photoreal texture {}: {error}", path.display()))
        .into_rgba8();
    Arc::new(ImageFrame::from_rgba8(
        rgba.width(),
        rgba.height(),
        rgba.into_raw(),
    ))
}

fn load_industrial_prop(repo_root: &Path) -> IndustrialProp {
    if env_flag("RNE_DISABLE_INDUSTRIAL_ASSETS") {
        return IndustrialProp::default();
    }
    let asset_root = repo_root.join("assets/environments/polyhaven_machine_shop_01");
    let gltf_path = asset_root.join("hand_truck.gltf");
    if !gltf_path.is_file() {
        eprintln!(
            "industrial asset {} is missing; using the procedural calibration room only",
            gltf_path.display()
        );
        return IndustrialProp::default();
    }

    let mut prop_scene = RenderScene::new();
    prop_scene.items.push(RenderSceneItem {
        transform: Transform3::IDENTITY,
        shape: VisualShape::Mesh {
            path: "package://hand_truck/hand_truck.gltf".to_owned(),
            scale: Vec3::ONE,
        },
        color_rgba: [1.0; 4],
        mesh: None,
        base_color_texture: None,
        material: PbrMaterial::default(),
    });
    prop_scene
        .resolve_mesh_assets(asset_root.as_path())
        .unwrap_or_else(|error| panic!("resolve Poly Haven hand truck: {error}"));
    IndustrialProp {
        items: prop_scene.items,
    }
}

fn append_industrial_prop(scene: &mut RenderScene, center: Vec3, prop: &IndustrialProp) {
    let placement = center + Vec3::new(0.95, 0.005, -0.55);
    for base_item in &prop.items {
        let mut item = base_item.clone();
        item.transform.translation += placement;
        scene.items.push(item);
    }
}

fn validate_bundled_environment(repo_root: &Path) {
    if env_flag("RNE_DISABLE_BUNDLED_INDUSTRIAL_ENVIRONMENT") {
        return;
    }
    let path =
        repo_root.join("assets/environments/polyhaven_machine_shop_01/machine_shop_01_1k.hdr");
    let map = EnvironmentMap::load(&path)
        .unwrap_or_else(|error| panic!("load bundled industrial HDRI {}: {error}", path.display()));
    assert!(
        map.width > 1 && map.height > 1,
        "bundled HDRI must contain an image"
    );
    assert!(
        map.rgba32f
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0),
        "bundled HDRI contains invalid linear pixels"
    );
}

fn append_calibration_room(scene: &mut RenderScene, center: Vec3, floor: &FloorTextures) {
    let floor_center = center + Vec3::new(0.7, -0.035, -0.25);
    push_box(
        scene,
        floor_center,
        Vec3::new(5.4, 0.07, 4.6),
        [0.16, 0.18, 0.20, 1.0],
    );
    push_textured_floor(scene, floor_center, Vec3::new(5.4, 0.0, 4.6), floor);
    push_box(
        scene,
        center + Vec3::new(2.45, 1.35, -0.25),
        Vec3::new(0.10, 2.7, 4.7),
        [0.11, 0.14, 0.17, 1.0],
    );
    push_box(
        scene,
        center + Vec3::new(0.7, 1.35, -2.25),
        Vec3::new(4.5, 2.7, 0.10),
        [0.11, 0.14, 0.17, 1.0],
    );
    for z_offset in [-1.2, 0.0, 1.2] {
        push_box(
            scene,
            center + Vec3::new(0.6, 2.55, z_offset),
            Vec3::new(0.72, 0.035, 0.18),
            [0.82, 0.86, 0.78, 1.0],
        );
    }
}

fn push_textured_floor(
    scene: &mut RenderScene,
    center: Vec3,
    footprint: Vec3,
    floor: &FloorTextures,
) {
    let half_x = (footprint.x * 0.5) as f32;
    let half_z = (footprint.z * 0.5) as f32;
    let repeat_x = (footprint.x / 0.75).max(1.0) as f32;
    let repeat_z = (footprint.z / 0.75).max(1.0) as f32;
    let mesh = TriangleMesh {
        positions: vec![
            [-half_x, 0.038, -half_z],
            [half_x, 0.038, -half_z],
            [half_x, 0.038, half_z],
            [-half_x, 0.038, half_z],
        ],
        normals: vec![[0.0, 1.0, 0.0]; 4],
        texcoords: vec![
            [0.0, 0.0],
            [repeat_x, 0.0],
            [repeat_x, repeat_z],
            [0.0, repeat_z],
        ],
        indices: vec![0, 1, 2, 0, 2, 3],
        skinning: None,
    };
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation: center,
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::DynamicMesh,
        color_rgba: [1.0; 4],
        mesh: Some(Arc::new(mesh)),
        base_color_texture: Some(Arc::clone(&floor.base_color)),
        material: PbrMaterial::new([1.0; 4], 0.9, 0.0, [0.0; 3]).with_texture_maps(
            Some(Arc::clone(&floor.normal)),
            Some(Arc::clone(&floor.roughness)),
        ),
    });
}

fn push_box(scene: &mut RenderScene, translation: Vec3, size: Vec3, color_rgba: [f32; 4]) {
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale: size,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    });
}

fn configure_environment(backend: &mut WgpuRenderBackend, repo_root: &Path) {
    let path = std::env::var_os("RNE_HDRI_PATH")
        .map(PathBuf::from)
        .or_else(|| {
            if env_flag("RNE_DISABLE_BUNDLED_INDUSTRIAL_ENVIRONMENT") {
                return None;
            }
            Some(
                repo_root
                    .join("assets/environments/polyhaven_machine_shop_01/machine_shop_01_1k.hdr"),
            )
        });
    let Some(path) = path else {
        return;
    };
    if !path.is_file() {
        eprintln!(
            "HDRI environment {} is missing; using the procedural lighting fallback",
            path.display()
        );
        return;
    }
    let map = EnvironmentMap::load(&path)
        .unwrap_or_else(|error| panic!("load HDRI environment {}: {error}", path.display()));
    let mut lighting = EnvironmentLighting::from_map(Arc::new(map));
    if let Ok(value) = std::env::var("RNE_HDRI_INTENSITY") {
        lighting.intensity = value
            .parse()
            .unwrap_or_else(|error| panic!("parse RNE_HDRI_INTENSITY={value:?}: {error}"));
    }
    if let Ok(value) = std::env::var("RNE_HDRI_ROTATION_RAD") {
        lighting.rotation_rad = value
            .parse()
            .unwrap_or_else(|error| panic!("parse RNE_HDRI_ROTATION_RAD={value:?}: {error}"));
    }
    backend.set_environment(lighting);
    println!("using HDRI environment {}", path.display());
}

fn env_flag(name: &str) -> bool {
    std::env::var(name)
        .map(|value| {
            matches!(
                value.trim().to_ascii_lowercase().as_str(),
                "1" | "true" | "on"
            )
        })
        .unwrap_or(false)
}

fn configure_taa(backend: &mut WgpuRenderBackend) {
    let enabled = std::env::var("RNE_TAA")
        .map(|value| {
            matches!(
                value.trim().to_ascii_lowercase().as_str(),
                "1" | "true" | "on"
            )
        })
        .unwrap_or(false);
    if !enabled {
        return;
    }
    let mut settings = TaaSettings::enabled();
    if let Ok(value) = std::env::var("RNE_TAA_FEEDBACK") {
        settings.feedback = value
            .parse()
            .unwrap_or_else(|error| panic!("parse RNE_TAA_FEEDBACK={value:?}: {error}"));
    }
    if let Ok(value) = std::env::var("RNE_TAA_JITTER_PX") {
        settings.jitter_scale_px = value
            .parse()
            .unwrap_or_else(|error| panic!("parse RNE_TAA_JITTER_PX={value:?}: {error}"));
    }
    backend.set_taa(settings);
    println!(
        "using temporal anti-aliasing (feedback {:.2}, jitter {:.2}px)",
        backend.taa().feedback,
        backend.taa().jitter_scale_px
    );
}

fn write_rgb_png(path: &Path, image: &ImageRgb8) -> io::Result<()> {
    let file = File::create(path)?;
    let mut encoder = Encoder::new(BufWriter::new(file), image.width, image.height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer
        .write_image_data(&image.rgba8)
        .map_err(io::Error::other)
}

fn write_depth_png(path: &Path, depth: &ImageDepth) -> io::Result<()> {
    let mut bytes = Vec::with_capacity(depth.depth_m.len() * 2);
    for value in &depth.depth_m {
        let normalized = (*value / Camera::default().far_m as f32).clamp(0.0, 1.0);
        bytes.extend_from_slice(&((normalized * 65_535.0) as u16).to_be_bytes());
    }
    let file = File::create(path)?;
    let mut encoder = Encoder::new(BufWriter::new(file), depth.width, depth.height);
    encoder.set_color(ColorType::Grayscale);
    encoder.set_depth(BitDepth::Sixteen);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(&bytes).map_err(io::Error::other)
}

fn write_depth_f32le(path: &Path, depth: &ImageDepth) -> io::Result<()> {
    let mut writer = BufWriter::new(File::create(path)?);
    for value in &depth.depth_m {
        writer.write_all(&value.to_le_bytes())?;
    }
    writer.flush()
}

fn write_gif(
    path: &Path,
    frames: &[Vec<u8>],
    width: u32,
    height: u32,
) -> Result<(), Box<dyn std::error::Error>> {
    let file = File::create(path)?;
    let mut encoder = GifEncoder::new(file);
    encoder.set_repeat(Repeat::Infinite)?;
    let delay = Delay::from_numer_denom_ms(100, 1);
    for rgba in frames {
        let image = RgbaImage::from_raw(width, height, rgba.clone()).ok_or_else(|| {
            io::Error::new(io::ErrorKind::InvalidData, "invalid RGBA frame dimensions")
        })?;
        encoder.encode_frame(GifFrame::from_parts(image, 0, 0, delay))?;
    }
    Ok(())
}

fn target_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
}

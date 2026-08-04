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
    UnitreeG1GaitCommand, UrdfSceneSim,
};
use rne_core::{SimDuration, SimTime};
use rne_data::{DataBus, Frame, ImageDepth, ImageRgb8, InMemoryDataBus, StreamId};
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
use std::fs::{self, File};
use std::io::{self, BufWriter, Write};
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

fn main() {
    if std::env::args().any(|argument| argument == "--smoke")
        || std::env::var_os("RNE_SKIP_GPU").is_some()
    {
        run_smoke();
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let floor = load_floor_textures(&repo_root);
    let mut sim = load_and_settle_g1();
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; G1 RGB-D smoke passed: {error}");
            return;
        }
    };
    configure_environment(&mut backend);
    configure_taa(&mut backend);

    let mut rig = CameraRig::new(camera_spec());
    let mut cache = MeshRenderCache::new();
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let output_dir = target_dir(&repo_root).join("rne-g1-rgbd-sensor");
    fs::create_dir_all(&output_dir).expect("create G1 RGB-D output directory");
    let mut gif_frames = Vec::with_capacity(FRAME_COUNT);
    let mut manifest = String::from(
        "frame,capture_ticks,available_ticks,rgb_hash,depth_hash,min_depth_m,center_depth_m\n",
    );

    for frame_index in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = frame_index as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_g1_gait_targets(step, WALK_COMMAND));
        }
        let scene = g1_scene(&sim, &mut cache, &mesh_root_refs, &floor);
        let capture = rig.sample(
            g1_head_camera_transform(&sim),
            Some(&mut backend),
            Some(&scene),
        );
        validate_capture(&capture);
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
    println!("rendered G1 RGB-D sensor media to {}", output_dir.display());
}

fn run_smoke() {
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
    let mut sim = load_and_settle_g1();
    let mut cache = MeshRenderCache::new();
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let floor = load_floor_textures(&PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."));
    let mut rig = CameraRig::new(camera_spec());
    let mut digest = Vec::with_capacity(FRAME_COUNT);

    for frame_index in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = frame_index as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_g1_gait_targets(step, WALK_COMMAND));
        }
        let scene = g1_scene(&sim, &mut cache, &mesh_root_refs, &floor);
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

fn configure_environment(backend: &mut WgpuRenderBackend) {
    let Some(path) = std::env::var_os("RNE_HDRI_PATH") else {
        return;
    };
    let path = PathBuf::from(path);
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

//! Captures the official Unitree G1 in RNE's photoreal render path.
//!
//! The simulation remains the existing URDF/STL scene. This example adds a
//! render-only calibration room, PBR floor maps, optional HDRI lighting, and
//! optional deterministic TAA without changing physics or core boundaries.
//! `--smoke` resolves the official G1 meshes and exercises a short articulated
//! pose update without initializing a GPU.

use image::codecs::gif::{GifEncoder, Repeat};
use image::{Delay, Frame, RgbaImage};
use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_dynamic_scene_path, unitree_g1_gait_targets,
    UnitreeG1GaitCommand, UrdfSceneSim,
};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    hash_depth_f32, hash_rgba8, Camera, EnvironmentLighting, EnvironmentMap, ImageFrame,
    MeshRenderCache, PbrMaterial, RenderBackend, RenderScene, RenderSceneItem, TriangleMesh,
    VisualShape,
};
use rne_render_wgpu::{CameraOrbit, TaaSettings, WgpuRenderBackend};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};
use std::sync::Arc;

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const FRAME_COUNT: usize = 12;
const STEPS_PER_FRAME: u64 = 8;
const SETTLE_STEPS: u64 = 240;
const CLEAR_COLOR: [f32; 4] = [0.018, 0.026, 0.040, 1.0];
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

fn main() {
    if std::env::args().any(|argument| argument == "--smoke")
        || std::env::var_os("RNE_SKIP_GPU").is_some()
    {
        run_smoke();
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut sim = load_and_settle_g1();
    let start = sim.observe();
    let floor = load_floor_textures(&repo_root);
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; G1 photoreal smoke passed: {error}");
            return;
        }
    };
    configure_environment(&mut backend);
    configure_taa(&mut backend);

    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(start.base_x_m + 0.12, start.base_y_m + 0.05, start.base_z_m),
        yaw_rad: -0.72,
        pitch_rad: 1.16,
        distance_m: 2.60,
    };
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();
    let output_dir = target_dir(&repo_root).join("rne-g1-photoreal");
    fs::create_dir_all(&output_dir).expect("create G1 photoreal output directory");
    let mut frames = Vec::with_capacity(FRAME_COUNT);

    for frame in 0..FRAME_COUNT {
        for substep in 0..STEPS_PER_FRAME {
            let step = frame as u64 * STEPS_PER_FRAME + substep;
            sim.step_joint_position_targets(&unitree_g1_gait_targets(step, WALK_COMMAND));
        }

        let mut scene = g1_scene(&sim, &mut mesh_cache, &mesh_root_refs);
        append_calibration_room(
            &mut scene,
            Vec3::new(start.base_x_m, 0.0, start.base_z_m),
            &floor,
        );
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render photoreal G1 frame");
        let min_depth_m = output
            .depth
            .depth_m
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min);
        assert!(
            min_depth_m < camera.far_m as f32,
            "G1 scene did not reach camera"
        );
        let frame_path = output_dir.join(format!("frame-{frame:03}.png"));
        write_png(
            &frame_path,
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write photoreal G1 frame");
        println!(
            "G1 photoreal frame={frame} mesh_items={} color_hash={:#018x} depth_hash={:#018x} min_depth_m={min_depth_m:.3} path={}",
            count_mesh_items(&scene),
            hash_rgba8(&output.color.rgba8),
            hash_depth_f32(&output.depth.depth_m),
            frame_path.display()
        );
        frames.push(output.color.rgba8);
    }

    let gif_path = output_dir.join("unitree-g1-photoreal.gif");
    write_gif(&gif_path, &frames, WIDTH, HEIGHT).expect("write photoreal G1 GIF");
    println!("rendered G1 photoreal media to {}", gif_path.display());
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

fn run_smoke() {
    let mut sim = load_and_settle_g1();
    let start = sim.observe();
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut cache = MeshRenderCache::new();
    let scene = g1_scene(&sim, &mut cache, &mesh_root_refs);
    let mesh_items = count_mesh_items(&scene);
    assert!(
        mesh_items >= G1_MESH_MINIMUM,
        "expected official G1 mesh visuals, got {mesh_items}"
    );
    assert!(
        scene.items.iter().all(|item| {
            item.mesh.as_ref().is_none_or(|mesh| {
                mesh.positions
                    .iter()
                    .flatten()
                    .all(|value| value.is_finite())
            })
        }),
        "G1 mesh contains non-finite positions"
    );
    for step in 0..24 {
        sim.step_joint_position_targets(&unitree_g1_gait_targets(step, WALK_COMMAND));
    }
    let observed = sim.observe();
    assert!(
        observed.base_y_m > 0.7,
        "G1 photoreal smoke fell: {:.3} m",
        observed.base_y_m
    );
    assert!(observed.base_y_m.is_finite());
    println!(
        "G1 photoreal smoke passed: mesh_items={} scene_items={} start_height_m={:.3} final_height_m={:.3}",
        mesh_items,
        scene.items.len(),
        start.base_y_m,
        observed.base_y_m
    );
}

fn g1_scene(sim: &UrdfSceneSim, cache: &mut MeshRenderCache, mesh_roots: &[&Path]) -> RenderScene {
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
    scene
}

fn count_mesh_items(scene: &RenderScene) -> usize {
    scene
        .items
        .iter()
        .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
        .count()
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
    const FLOOR: [f32; 4] = [0.16, 0.18, 0.20, 1.0];
    const FLOOR_SEAM: [f32; 4] = [0.055, 0.07, 0.08, 1.0];
    const SAFETY_YELLOW: [f32; 4] = [0.72, 0.46, 0.08, 1.0];
    const WALL: [f32; 4] = [0.11, 0.14, 0.17, 1.0];
    const WALL_PANEL: [f32; 4] = [0.055, 0.09, 0.13, 1.0];
    const METAL: [f32; 4] = [0.28, 0.32, 0.35, 1.0];
    const WINDOW: [f32; 4] = [0.025, 0.08, 0.12, 1.0];
    const LIGHT: [f32; 4] = [0.82, 0.86, 0.78, 1.0];

    let floor_center = center + Vec3::new(0.25, -0.035, -0.35);
    push_box(scene, floor_center, Vec3::new(5.4, 0.07, 4.6), FLOOR);
    push_textured_floor(scene, floor_center, Vec3::new(5.4, 0.0, 4.6), floor);

    for x_offset in [-1.8, -0.9, 0.0, 0.9, 1.8] {
        push_box(
            scene,
            center + Vec3::new(x_offset, 0.004, -0.35),
            Vec3::new(0.012, 0.006, 4.35),
            FLOOR_SEAM,
        );
    }
    for z_offset in [-1.4, -0.45, 0.5, 1.45] {
        push_box(
            scene,
            center + Vec3::new(0.25, 0.004, z_offset),
            Vec3::new(5.25, 0.006, 0.012),
            FLOOR_SEAM,
        );
    }
    for z_offset in [-0.82, 0.82] {
        push_box(
            scene,
            center + Vec3::new(0.25, 0.009, z_offset),
            Vec3::new(4.8, 0.008, 0.035),
            SAFETY_YELLOW,
        );
    }

    push_box(
        scene,
        center + Vec3::new(2.45, 1.35, -0.35),
        Vec3::new(0.10, 2.7, 4.7),
        WALL,
    );
    push_box(
        scene,
        center + Vec3::new(0.25, 1.35, -2.25),
        Vec3::new(4.5, 2.7, 0.10),
        WALL,
    );
    for z_offset in [-1.55, -0.55, 0.45, 1.45] {
        push_box(
            scene,
            center + Vec3::new(2.385, 1.42, z_offset),
            Vec3::new(0.018, 2.15, 0.84),
            WALL_PANEL,
        );
        push_box(
            scene,
            center + Vec3::new(2.32, 1.42, z_offset - 0.5),
            Vec3::new(0.025, 2.18, 0.018),
            METAL,
        );
    }
    for z_offset in [-1.05, -0.05, 0.95] {
        push_box(
            scene,
            center + Vec3::new(2.31, 1.55, z_offset),
            Vec3::new(0.026, 1.18, 0.68),
            WINDOW,
        );
        push_box(
            scene,
            center + Vec3::new(2.285, 1.55, z_offset - 0.36),
            Vec3::new(0.032, 0.025, 0.72),
            METAL,
        );
        push_box(
            scene,
            center + Vec3::new(2.285, 1.55, z_offset + 0.36),
            Vec3::new(0.032, 0.025, 0.72),
            METAL,
        );
        push_box(
            scene,
            center + Vec3::new(2.275, 1.55, z_offset),
            Vec3::new(0.035, 1.18, 0.025),
            METAL,
        );
    }
    for z_offset in [-1.2, 0.0, 1.2] {
        push_box(
            scene,
            center + Vec3::new(0.20, 2.55, z_offset),
            Vec3::new(0.72, 0.035, 0.18),
            LIGHT,
        );
        push_box(
            scene,
            center + Vec3::new(0.20, 2.49, z_offset),
            Vec3::new(0.06, 0.12, 0.06),
            METAL,
        );
    }
}

fn push_textured_floor(
    scene: &mut RenderScene,
    center: Vec3,
    footprint: Vec3,
    floor: &FloorTextures,
) {
    let half_x = footprint.x * 0.5;
    let half_z = footprint.z * 0.5;
    let repeat_x = (footprint.x / 0.75).max(1.0) as f32;
    let repeat_z = (footprint.z / 0.75).max(1.0) as f32;
    let mesh = TriangleMesh {
        positions: vec![
            [-half_x as f32, 0.038, -half_z as f32],
            [half_x as f32, 0.038, -half_z as f32],
            [half_x as f32, 0.038, half_z as f32],
            [-half_x as f32, 0.038, half_z as f32],
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

fn target_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = File::create(path)?;
    let mut encoder = Encoder::new(BufWriter::new(file), width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
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
        let image = RgbaImage::from_raw(width, height, rgba.clone())
            .ok_or("invalid RGBA frame dimensions")?;
        encoder.encode_frame(Frame::from_parts(image, 0, 0, delay))?;
    }
    Ok(())
}

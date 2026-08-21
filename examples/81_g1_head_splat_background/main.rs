//! G1 head-camera hybrid capture over a Tsukuba 3DGS sidewalk background.
//!
//! Mounts the camera on the official G1 `head_link`, draws the Gaussian splat
//! environment behind the robot meshes, and writes a small capture report.
//! Contest scoring and the full RGB-D DataBus path stay in examples 75 / 71.

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_dynamic_scene_path, unitree_g1_gait_targets,
    UnitreeG1GaitCommand, UrdfSceneSim,
};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    validate_gaussian_splat_manifest, Camera, HybridRenderScene, MeshRenderCache, PbrMaterial,
    RenderScene, VisualShape,
};
use rne_render_3dgs::{load_gaussian_splat_background, render_hybrid_scene_camera};
use rne_render_wgpu::WgpuRenderBackend;
use rne_world::Transform3 as WorldTransform3;
use serde::Serialize;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const CLEAR_COLOR: [f32; 4] = [0.55, 0.62, 0.68, 1.0];
const SETTLE_STEPS: u64 = 120;
const G1_MESH_MINIMUM: usize = 20;
const HEAD_LINK: &str = "head_link";

#[derive(Clone, Debug, Serialize)]
struct G1HeadSplatCaptureReport {
    kind: &'static str,
    schema_version: u32,
    camera_frame: &'static str,
    environment_id: String,
    renderer_identity: String,
    mesh_items: usize,
    head_translation_m: [f64; 3],
    rgba_hash: Option<u64>,
    png_path: Option<String>,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let splat_manifest = repo_root.join("assets/environments/tsukuba_confirmation.rne.splat.toml");
    let environment = validate_gaussian_splat_manifest(&splat_manifest).expect("splat manifest");

    let sim = load_and_settle_g1();
    let head = g1_head_camera_transform(&sim);
    let mesh_roots = [
        repo_root.join("assets/robots/g1_description"),
        repo_root.join("assets/robots"),
    ];
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut cache = MeshRenderCache::default();
    let foreground = g1_foreground(&sim, &mut cache, &mesh_root_refs);
    assert!(
        foreground.items.len() >= G1_MESH_MINIMUM,
        "expected official G1 meshes, got {}",
        foreground.items.len()
    );

    let report_dir = output_dir(&repo_root);
    fs::create_dir_all(&report_dir).expect("output dir");

    if smoke || std::env::var_os("RNE_SKIP_GPU").is_some() {
        let report = G1HeadSplatCaptureReport {
            kind: "rne_g1_head_splat_capture_report",
            schema_version: 1,
            camera_frame: HEAD_LINK,
            environment_id: environment.environment_id.clone(),
            renderer_identity: environment.renderer_identity.clone(),
            mesh_items: foreground.items.len(),
            head_translation_m: head.translation.to_array(),
            rgba_hash: None,
            png_path: None,
        };
        write_report(&report_dir.join("g1_head_splat_capture.json"), &report);
        println!(
            "smoke: camera={} environment={} meshes={} head=[{:.3}, {:.3}, {:.3}]",
            report.camera_frame,
            report.environment_id,
            report.mesh_items,
            report.head_translation_m[0],
            report.head_translation_m[1],
            report.head_translation_m[2]
        );
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; G1 head splat smoke passed: {error}");
            return;
        }
    };
    let mut background = match load_gaussian_splat_background(backend.device(), &environment) {
        Ok(background) => background,
        Err(error) => {
            eprintln!("splat background unavailable; G1 head splat smoke passed: {error}");
            return;
        }
    };

    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let view = Transform3 {
        translation: head.translation,
        rotation: head.rotation,
        scale: Vec3::ONE,
    };
    let hybrid = HybridRenderScene::new(environment.clone(), foreground);

    let output = match render_hybrid_scene_camera(
        &mut backend,
        &mut background,
        &camera,
        &view,
        &hybrid,
        CLEAR_COLOR,
    ) {
        Ok(output) => output,
        Err(error) => {
            eprintln!("hybrid capture unavailable; G1 head splat smoke passed: {error}");
            return;
        }
    };

    let hash = output.color.hash_pixels();
    assert_ne!(hash, 0);
    let png_path = report_dir.join("g1_head_splat.png");
    write_png(
        &png_path,
        output.color.width,
        output.color.height,
        &output.color.rgba8,
    );
    let report = G1HeadSplatCaptureReport {
        kind: "rne_g1_head_splat_capture_report",
        schema_version: 1,
        camera_frame: HEAD_LINK,
        environment_id: environment.environment_id,
        renderer_identity: environment.renderer_identity,
        mesh_items: hybrid.foreground.items.len(),
        head_translation_m: head.translation.to_array(),
        rgba_hash: Some(hash),
        png_path: Some(png_path.display().to_string()),
    };
    write_report(&report_dir.join("g1_head_splat_capture.json"), &report);
    println!(
        "capture: camera={} environment={} meshes={} rgba_hash={hash} png={}",
        report.camera_frame,
        report.environment_id,
        report.mesh_items,
        png_path.display()
    );
}

fn load_and_settle_g1() -> UrdfSceneSim {
    let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
        .expect("load official dynamic G1");
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let stand = UnitreeG1GaitCommand {
        stride_rad: 0.0,
        foot_lift_rad: 0.0,
        cycle_steps: 100,
    };
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&unitree_g1_gait_targets(0, stand));
    }
    sim
}

fn g1_head_camera_transform(sim: &UrdfSceneSim) -> WorldTransform3 {
    let head = sim
        .named_transform(HEAD_LINK)
        .expect("G1 head_link transform");
    // Match example 71: vendored G1 faces +X; camera uses local -Z forward.
    WorldTransform3::from_translation_rotation(
        head.translation + Vec3::new(0.11, 0.015, 0.0),
        Quat::from_rotation_y(-std::f64::consts::FRAC_PI_2),
    )
}

fn g1_foreground(
    sim: &UrdfSceneSim,
    cache: &mut MeshRenderCache,
    mesh_roots: &[&Path],
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
    scene
}

fn output_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
        .join("rne-g1-head-splat")
}

fn write_report(path: &Path, report: &G1HeadSplatCaptureReport) {
    let json = serde_json::to_string_pretty(report).expect("report json");
    fs::write(path, json).expect("write report");
    println!("report {}", path.display());
}

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) {
    let file = File::create(path).expect("png file");
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().expect("png header");
    png_writer.write_image_data(rgba8).expect("png pixels");
}

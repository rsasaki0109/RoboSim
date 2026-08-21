//! Tsukuba 3DGS proxy-depth spike for hybrid RGB-D captures.
//!
//! Projects Gaussian means into a linear depth buffer, optionally composites
//! with the mesh foreground depth from the hybrid color pass, and writes a
//! capture report. Contest scoring stays in example 75.

use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    hash_depth_f32, validate_gaussian_splat_manifest, Camera, DepthFrame, HybridRenderScene,
    RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_3dgs::{
    composite_mesh_and_splat_depth, load_gaussian_splat_background, render_hybrid_scene_camera,
    splat_proxy_depth_from_ply,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::Serialize;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 160;
const HEIGHT: u32 = 120;
const CLEAR_COLOR: [f32; 4] = [0.55, 0.62, 0.68, 1.0];

#[derive(Clone, Debug, Serialize)]
struct SplatDepthCaptureReport {
    kind: &'static str,
    schema_version: u32,
    environment_id: String,
    renderer_identity: String,
    splat_finite_pixels: usize,
    composite_finite_pixels: usize,
    splat_depth_hash: u64,
    composite_depth_hash: Option<u64>,
    depth_preview_path: Option<String>,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest = repo_root.join("assets/environments/tsukuba_confirmation.rne.splat.toml");
    let environment = validate_gaussian_splat_manifest(&manifest).expect("splat manifest");

    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(3.75, 0.25, 0.0),
        yaw_rad: -1.35,
        pitch_rad: 1.05,
        distance_m: 5.5,
    };
    let view = orbit.camera_transform();

    let splat_depth = splat_proxy_depth_from_ply(
        &environment.ply_path,
        &camera,
        &view,
        &environment.transform,
    )
    .expect("splat proxy depth");
    let splat_finite = count_finite(&splat_depth, camera.far_m as f32);
    assert!(
        splat_finite > 0,
        "expected splat proxy depths from the fixture PLY"
    );
    let splat_hash = splat_depth.hash_depth();
    let again = splat_proxy_depth_from_ply(
        &environment.ply_path,
        &camera,
        &view,
        &environment.transform,
    )
    .expect("splat proxy depth replay");
    assert_eq!(splat_hash, again.hash_depth());

    let out_dir = output_dir(&repo_root);
    fs::create_dir_all(&out_dir).expect("output dir");

    if smoke || std::env::var_os("RNE_SKIP_GPU").is_some() {
        let report = SplatDepthCaptureReport {
            kind: "rne_gaussian_splat_depth_capture_report",
            schema_version: 1,
            environment_id: environment.environment_id.clone(),
            renderer_identity: environment.renderer_identity.clone(),
            splat_finite_pixels: splat_finite,
            composite_finite_pixels: splat_finite,
            splat_depth_hash: splat_hash,
            composite_depth_hash: None,
            depth_preview_path: None,
        };
        write_report(&out_dir.join("tsukuba_splat_depth.json"), &report);
        println!(
            "smoke: environment={} splat_finite={} depth_hash={splat_hash}",
            report.environment_id, report.splat_finite_pixels
        );
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; 3DGS depth smoke passed: {error}");
            return;
        }
    };
    let mut background = match load_gaussian_splat_background(backend.device(), &environment) {
        Ok(background) => background,
        Err(error) => {
            eprintln!("splat background unavailable; 3DGS depth smoke passed: {error}");
            return;
        }
    };
    let hybrid = HybridRenderScene::new(environment.clone(), foreground_box());
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
            eprintln!("hybrid capture unavailable; 3DGS depth smoke passed: {error}");
            return;
        }
    };

    let composite =
        composite_mesh_and_splat_depth(&output.depth, &splat_depth, camera.far_m as f32);
    let composite_finite = count_finite(&composite, camera.far_m as f32);
    assert!(composite_finite >= splat_finite);
    assert_ne!(hash_depth_f32(&composite.depth_m), 0);

    let preview = out_dir.join("tsukuba_splat_depth_preview.png");
    write_depth_preview(&preview, &composite, camera.far_m as f32);
    let report = SplatDepthCaptureReport {
        kind: "rne_gaussian_splat_depth_capture_report",
        schema_version: 1,
        environment_id: environment.environment_id,
        renderer_identity: environment.renderer_identity,
        splat_finite_pixels: splat_finite,
        composite_finite_pixels: composite_finite,
        splat_depth_hash: splat_hash,
        composite_depth_hash: Some(composite.hash_depth()),
        depth_preview_path: Some(preview.display().to_string()),
    };
    write_report(&out_dir.join("tsukuba_splat_depth.json"), &report);
    println!(
        "capture: splat_finite={} composite_finite={} splat_hash={splat_hash} preview={}",
        splat_finite,
        composite_finite,
        preview.display()
    );
}

fn foreground_box() -> RenderScene {
    let mut scene = RenderScene::new();
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation: Vec3::new(3.75, 0.25, 0.0),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::Box {
            size_m: Vec3::new(0.5, 0.5, 0.5),
        },
        color_rgba: [0.35, 0.55, 0.95, 1.0],
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    });
    scene
}

fn count_finite(depth: &DepthFrame, far_m: f32) -> usize {
    depth
        .depth_m
        .iter()
        .filter(|value| value.is_finite() && **value < far_m * 0.999)
        .count()
}

fn output_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
        .join("rne-tsukuba-splat-depth")
}

fn write_report(path: &Path, report: &SplatDepthCaptureReport) {
    let json = serde_json::to_string_pretty(report).expect("report json");
    fs::write(path, json).expect("write report");
    println!("report {}", path.display());
}

fn write_depth_preview(path: &Path, depth: &DepthFrame, far_m: f32) {
    let mut rgba = Vec::with_capacity((depth.width * depth.height * 4) as usize);
    for value in &depth.depth_m {
        let normalized = if value.is_finite() && *value < far_m * 0.999 {
            (1.0 - (*value / far_m).clamp(0.0, 1.0)).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let gray = (normalized * 255.0) as u8;
        rgba.extend_from_slice(&[gray, gray, gray, 255]);
    }
    let file = File::create(path).expect("preview file");
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, depth.width, depth.height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().expect("png header");
    png_writer.write_image_data(&rgba).expect("png pixels");
}

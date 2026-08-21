//! House 3DGS hybrid capture smoke test.
//!
//! The checked-in procedural cloud is a visual-only indoor background. This
//! example validates the manifest and PLY, computes deterministic proxy depth,
//! and exercises the same hybrid splat-background + mesh-foreground path used
//! by showcase captures. `--smoke` is safe on machines without a GPU: it still
//! verifies the CPU hybrid contract and reports that GPU capture was skipped.
//!
//! ```text
//! cargo run -p house_3dgs_hybrid --example 88_house_3dgs_hybrid -- --smoke
//! ```

use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    validate_gaussian_splat_manifest, Camera, DepthFrame, HybridRenderScene, RenderScene,
    RenderSceneItem, VisualShape,
};
use rne_render_3dgs::{
    composite_mesh_and_splat_depth, load_gaussian_splat_background, render_hybrid_scene_camera,
    splat_proxy_depth_from_ply,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 320;
const HEIGHT: u32 = 240;
const FAR_M: f32 = 40.0;
const CLEAR_COLOR: [f32; 4] = [0.12, 0.15, 0.18, 1.0];

#[derive(Clone, Debug, Serialize)]
struct HouseCaptureReport {
    kind: &'static str,
    schema_version: u32,
    environment_id: String,
    renderer_identity: String,
    ply_sha256: String,
    ply_bytes: u64,
    proxy_depth_finite_pixels: usize,
    proxy_depth_hash: u64,
    composite_depth_hash: u64,
    hybrid_foreground_items: usize,
    hybrid_render: &'static str,
    rgba_hash: Option<u64>,
    png_path: Option<String>,
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest_path = repo_root.join("assets/environments/house_3dgs/house_3dgs.rne.splat.toml");
    let environment =
        validate_gaussian_splat_manifest(&manifest_path).expect("House splat manifest");
    let ply_bytes = fs::metadata(&environment.ply_path)
        .expect("House fixture metadata")
        .len();
    let ply_sha256 = sha256_file(&environment.ply_path);

    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let view = CameraOrbit {
        focus: Vec3::new(0.0, 1.05, 0.0),
        yaw_rad: 0.55,
        pitch_rad: 0.92,
        distance_m: 7.2,
    }
    .camera_transform();
    let splat_depth = splat_proxy_depth_from_ply(
        &environment.ply_path,
        &camera,
        &view,
        &environment.transform,
    )
    .expect("House splat proxy depth");
    let proxy_finite = finite_depth_pixels(&splat_depth, FAR_M);
    assert!(
        proxy_finite > 1_000,
        "expected a dense indoor proxy depth, got {proxy_finite}"
    );
    let proxy_hash = splat_depth.hash_depth();
    let replay = splat_proxy_depth_from_ply(
        &environment.ply_path,
        &camera,
        &view,
        &environment.transform,
    )
    .expect("House proxy depth replay");
    assert_eq!(
        proxy_hash,
        replay.hash_depth(),
        "proxy depth must be deterministic"
    );

    let foreground = house_foreground();
    let hybrid = HybridRenderScene::new(environment.clone(), foreground);
    let far_depth = DepthFrame::new(WIDTH, HEIGHT, vec![FAR_M; (WIDTH * HEIGHT) as usize]);
    let composite = composite_mesh_and_splat_depth(&far_depth, &splat_depth, FAR_M);
    assert_eq!(finite_depth_pixels(&composite, FAR_M), proxy_finite);
    let composite_hash = composite.hash_depth();

    let out_dir = output_dir(&repo_root);
    fs::create_dir_all(&out_dir).expect("House capture output directory");
    let gpu_capture = if std::env::var_os("RNE_SKIP_GPU").is_some() {
        None
    } else {
        try_gpu_capture(&environment, &hybrid, &camera, &view, &out_dir)
    };
    let (hybrid_render, rgba_hash, png_path) = match gpu_capture {
        Some((hash, path)) => ("gpu", Some(hash), Some(path)),
        None => ("cpu_contract", None, None),
    };

    let report = HouseCaptureReport {
        kind: "rne_house_3dgs_hybrid_capture_report",
        schema_version: 1,
        environment_id: environment.environment_id,
        renderer_identity: environment.renderer_identity,
        ply_sha256,
        ply_bytes,
        proxy_depth_finite_pixels: proxy_finite,
        proxy_depth_hash: proxy_hash,
        composite_depth_hash: composite_hash,
        hybrid_foreground_items: hybrid.foreground.items.len(),
        hybrid_render,
        rgba_hash,
        png_path: png_path.as_ref().map(|path| path.display().to_string()),
    };
    write_report(&out_dir.join("house_3dgs_capture.json"), &report);
    println!(
        "{}: environment={} points_bytes={} proxy_finite={} depth_hash={} hybrid={} items={}",
        if smoke { "smoke" } else { "capture" },
        report.environment_id,
        report.ply_bytes,
        report.proxy_depth_finite_pixels,
        report.proxy_depth_hash,
        report.hybrid_render,
        report.hybrid_foreground_items,
    );
}

fn house_foreground() -> RenderScene {
    let mut scene = RenderScene::new();
    // A small, deliberately generic foreground marks the hybrid contract. The
    // mobile-manipulator visual package can replace these items in the hero
    // capture without changing the House environment or depth path.
    scene.items.push(box_item(
        [0.0, 0.24, 1.75],
        [0.85, 0.42, 0.72],
        [0.12, 0.35, 0.68, 1.0],
    ));
    scene.items.push(box_item(
        [0.0, 0.74, 1.75],
        [0.22, 0.82, 0.22],
        [0.18, 0.22, 0.27, 1.0],
    ));
    scene.items.push(box_item(
        [0.0, 1.28, 1.75],
        [0.52, 0.16, 0.16],
        [0.74, 0.47, 0.12, 1.0],
    ));
    scene.items.push(box_item(
        [0.0, 1.22, 1.46],
        [0.16, 0.16, 0.46],
        [0.80, 0.28, 0.10, 1.0],
    ));
    scene
}

fn box_item(center: [f64; 3], size_m: [f64; 3], color: [f32; 4]) -> RenderSceneItem {
    RenderSceneItem {
        transform: Transform3 {
            translation: Vec3::from_array(center),
            rotation: Quat::IDENTITY,
            scale: Vec3::ONE,
        },
        shape: VisualShape::Box {
            size_m: Vec3::from_array(size_m),
        },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    }
}

fn try_gpu_capture(
    environment: &rne_render::GaussianSplatEnvironment,
    hybrid: &HybridRenderScene,
    camera: &Camera,
    view: &Transform3,
    out_dir: &Path,
) -> Option<(u64, PathBuf)> {
    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("House GPU hybrid unavailable; CPU contract retained: {error}");
            return None;
        }
    };
    let mut background = match load_gaussian_splat_background(backend.device(), environment) {
        Ok(background) => background,
        Err(error) => {
            eprintln!("House 3DGS background unavailable; CPU contract retained: {error}");
            return None;
        }
    };
    let output = match render_hybrid_scene_camera(
        &mut backend,
        &mut background,
        camera,
        view,
        hybrid,
        CLEAR_COLOR,
    ) {
        Ok(output) => output,
        Err(error) => {
            eprintln!("House hybrid render unavailable; CPU contract retained: {error}");
            return None;
        }
    };
    let hash = output.color.hash_pixels();
    if hash == 0 {
        return None;
    }
    let png_path = out_dir.join("house_3dgs_hybrid.png");
    write_png(
        &png_path,
        output.color.width,
        output.color.height,
        &output.color.rgba8,
    );
    Some((hash, png_path))
}

fn finite_depth_pixels(depth: &DepthFrame, far_m: f32) -> usize {
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
        .join("rne-house-3dgs")
}

fn sha256_file(path: &Path) -> String {
    let digest = Sha256::digest(fs::read(path).expect("read House fixture"));
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn write_report(path: &Path, report: &HouseCaptureReport) {
    fs::write(
        path,
        serde_json::to_string_pretty(report).expect("House report JSON"),
    )
    .expect("write House report");
    println!("report {}", path.display());
}

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) {
    let file = File::create(path).expect("House PNG");
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().expect("House PNG header");
    png_writer
        .write_image_data(rgba8)
        .expect("House PNG pixels");
}

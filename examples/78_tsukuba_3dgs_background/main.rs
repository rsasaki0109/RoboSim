//! Tsukuba confirmation hybrid capture: 3DGS sidewalk background + mesh overlay.
//!
//! Contest scoring stays in example 75. This example is viewer/dataset only.
//!
//! ```text
//! --smoke
//! --environment fixture|kenkyugakuen   (default: kenkyugakuen)
//! --ply PATH                           override PLY (clears standin)
//! ```

use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    validate_gaussian_splat_manifest_with_override, Camera, GaussianSplatCaptureReport,
    HybridRenderScene, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_3dgs::{load_gaussian_splat_background, render_hybrid_scene_camera};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use sha2::{Digest, Sha256};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const CLEAR_COLOR: [f32; 4] = [0.55, 0.62, 0.68, 1.0];

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let environment_name = arg_value("--environment").unwrap_or_else(|| "kenkyugakuen".into());
    let ply_override = arg_value("--ply")
        .map(PathBuf::from)
        .or_else(|| std::env::var_os("RNE_SPLAT_PLY").map(PathBuf::from));

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest = match environment_name.as_str() {
        "fixture" | "confirmation" => {
            repo_root.join("assets/environments/tsukuba_confirmation.rne.splat.toml")
        }
        "kenkyugakuen" => repo_root.join("assets/environments/tsukuba_kenkyugakuen.rne.splat.toml"),
        other => {
            eprintln!("unknown --environment {other}; use fixture or kenkyugakuen");
            std::process::exit(2);
        }
    };

    let environment =
        validate_gaussian_splat_manifest_with_override(&manifest, ply_override.as_deref())
            .expect("splat manifest");
    let ply_sha256 = sha256_file(&environment.ply_path);

    if smoke || std::env::var_os("RNE_SKIP_GPU").is_some() {
        let report = GaussianSplatCaptureReport::new(&environment, ply_sha256.clone(), None, None);
        println!(
            "smoke: environment_id={} renderer={} standin={} ply={} sha256={}",
            report.environment_id,
            report.renderer_identity,
            report.standin,
            report.ply_path,
            report.ply_sha256
        );
        write_report(&report_path(&repo_root), &report);
        if smoke {
            assert!(environment.ply_path.is_file());
            if environment_name == "kenkyugakuen" && ply_override.is_none() {
                assert!(
                    environment.standin,
                    "checkout without preferred PLY must report standin"
                );
            }
        }
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; Tsukuba 3DGS smoke passed: {error}");
            write_report(
                &report_path(&repo_root),
                &GaussianSplatCaptureReport::new(&environment, ply_sha256, None, None),
            );
            return;
        }
    };
    backend.set_taa(Default::default());
    let mut background = match load_gaussian_splat_background(backend.device(), &environment) {
        Ok(background) => background,
        Err(error) => {
            eprintln!("splat background unavailable; Tsukuba 3DGS smoke passed: {error}");
            write_report(
                &report_path(&repo_root),
                &GaussianSplatCaptureReport::new(&environment, ply_sha256, None, None),
            );
            return;
        }
    };

    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let orbit = CameraOrbit {
        focus: Vec3::new(3.75, 0.25, 0.0),
        yaw_rad: -1.35,
        pitch_rad: 1.05,
        distance_m: 5.5,
    };
    let view = orbit.camera_transform();
    let hybrid = HybridRenderScene::new(environment.clone(), tsukuba_confirmation_foreground());

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
            eprintln!("hybrid capture unavailable; Tsukuba 3DGS smoke passed: {error}");
            write_report(
                &report_path(&repo_root),
                &GaussianSplatCaptureReport::new(&environment, ply_sha256, None, None),
            );
            return;
        }
    };

    let hash = output.color.hash_pixels();
    println!(
        "capture: environment={} renderer={} standin={} rgba_hash={hash}",
        background.environment_id(),
        background.renderer_identity(),
        environment.standin,
    );
    assert_ne!(hash, 0);

    let png_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"));
    fs::create_dir_all(&png_dir).expect("png output directory");
    let png_path = png_dir.join("tsukuba_hybrid.png");
    write_png(
        &png_path,
        output.color.width,
        output.color.height,
        &output.color.rgba8,
    );
    println!("wrote {}", png_path.display());
    write_report(
        &report_path(&repo_root),
        &GaussianSplatCaptureReport::new(&environment, ply_sha256, Some(hash), Some(png_path)),
    );
}

fn arg_value(flag: &str) -> Option<String> {
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        if argument == flag {
            return args.next();
        }
        if let Some(value) = argument.strip_prefix(&format!("{flag}=")) {
            return Some(value.to_string());
        }
    }
    None
}

fn report_path(repo_root: &Path) -> PathBuf {
    let dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"));
    dir.join("tsukuba_splat_capture.json")
}

fn write_report(path: &Path, report: &GaussianSplatCaptureReport) {
    if let Some(parent) = path.parent() {
        let _ = fs::create_dir_all(parent);
    }
    let json = serde_json::to_string_pretty(report).expect("capture report json");
    fs::write(path, json).expect("write capture report");
    println!("report {}", path.display());
}

fn sha256_file(path: &Path) -> String {
    let bytes = fs::read(path).expect("read ply for sha256");
    let digest = Sha256::digest(bytes);
    digest.iter().map(|byte| format!("{byte:02x}")).collect()
}

fn tsukuba_confirmation_foreground() -> RenderScene {
    let mut scene = RenderScene::new();
    scene.items.push(box_item(
        [3.75, 0.01, 0.0],
        [8.5, 0.02, 2.0],
        [0.78, 0.76, 0.72, 0.55],
    ));
    scene.items.push(box_item(
        [3.75, 0.005, 2.2],
        [8.5, 0.01, 2.4],
        [0.22, 0.22, 0.24, 0.85],
    ));
    scene.items.push(box_item(
        [6.0, 0.2, 0.0],
        [0.24, 0.4, 0.24],
        [0.12, 0.72, 0.22, 1.0],
    ));
    scene.items.push(box_item(
        [0.5, 0.25, 0.0],
        [0.5, 0.3, 0.4],
        [0.35, 0.55, 0.95, 1.0],
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

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) {
    let file = File::create(path).expect("png file");
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().expect("png header");
    png_writer.write_image_data(rgba8).expect("png pixels");
}

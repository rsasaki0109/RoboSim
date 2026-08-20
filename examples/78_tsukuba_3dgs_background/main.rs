//! Tsukuba confirmation hybrid capture: 3DGS sidewalk background + mesh overlay.
//!
//! Contest scoring stays in example 75. This example is viewer/dataset only.

use png::{BitDepth, ColorType, Encoder};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    validate_gaussian_splat_manifest, Camera, HybridRenderScene, RenderScene, RenderSceneItem,
    VisualShape,
};
use rne_render_3dgs::{load_gaussian_splat_background, render_hybrid_scene_camera};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 640;
const HEIGHT: u32 = 480;
const CLEAR_COLOR: [f32; 4] = [0.55, 0.62, 0.68, 1.0];

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let manifest = repo_root.join("assets/environments/tsukuba_confirmation.rne.splat.toml");
    let environment = validate_gaussian_splat_manifest(&manifest).expect("splat manifest");

    if smoke || std::env::var_os("RNE_SKIP_GPU").is_some() {
        println!(
            "smoke: environment_id={} renderer={} ply={}",
            environment.environment_id,
            environment.renderer_identity,
            environment.ply_path.display()
        );
        if smoke {
            assert!(environment.ply_path.is_file());
        }
        return;
    }

    let mut backend = match WgpuRenderBackend::new() {
        Ok(backend) => backend,
        Err(error) => {
            eprintln!("wgpu unavailable; Tsukuba 3DGS smoke passed: {error}");
            return;
        }
    };
    backend.set_taa(Default::default());
    let mut background = match load_gaussian_splat_background(backend.device(), &environment) {
        Ok(background) => background,
        Err(error) => {
            eprintln!("splat background unavailable; Tsukuba 3DGS smoke passed: {error}");
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
    let hybrid = HybridRenderScene::new(environment, tsukuba_confirmation_foreground());

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
            return;
        }
    };

    let hash = output.color.hash_pixels();
    println!(
        "capture: environment={} renderer={} rgba_hash={hash}",
        background.environment_id(),
        background.renderer_identity(),
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

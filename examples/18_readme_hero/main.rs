//! Renders README hero media from the mesh diff-drive scene asset.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{build_diff_drive_render_scene, DiffDriveAction, DiffDriveSim};
use rne_math::Vec3;
use rne_render::{hash_rgba8, Camera, MeshRenderCache, RenderBackend, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const CLEAR_COLOR: [f32; 4] = [0.05, 0.08, 0.12, 1.0];
const RENDER_WIDTH: u32 = 320;
const RENDER_HEIGHT: u32 = 240;
const POSTER_WIDTH: u32 = 960;
const POSTER_HEIGHT: u32 = 540;
const DRIVE_STEPS: usize = 60;
const DRIVE_SPEED_RAD_S: f64 = 5.0;

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        eprintln!("RNE_SKIP_GPU set; skipping README hero render");
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let scene_path = repo_root.join("assets/scenes/mesh_diff_drive.rne.scene.toml");
    let media_dir = repo_root.join("docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut sim = DiffDriveSim::from_scene_path(&scene_path).expect("load mesh scene");
    for _ in 0..DRIVE_STEPS {
        sim.step_action(DiffDriveAction::forward(DRIVE_SPEED_RAD_S));
    }

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu backend");
    let mut scene = build_diff_drive_render_scene(sim.world(), sim.robots());
    let mut mesh_cache = MeshRenderCache::new();
    let mesh_roots: Vec<PathBuf> = sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    mesh_cache
        .resolve_scene(&mut scene, &mesh_root_refs)
        .expect("resolve mesh assets");

    let orbit = CameraOrbit {
        focus: robot_focus(&sim),
        ..CameraOrbit::default()
    };
    let camera = Camera::new(RENDER_WIDTH, RENDER_HEIGHT, std::f64::consts::FRAC_PI_4);
    let output = backend
        .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render hero frame");

    let color_hash = hash_rgba8(&output.color.rgba8);
    let mesh_items = scene
        .items
        .iter()
        .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
        .count();

    let raw_path = media_dir.join("rne-hero.raw.png");
    let poster_path = media_dir.join("rne-hero.png");
    write_png(
        &raw_path,
        &output.color.rgba8,
        output.color.width,
        output.color.height,
    )
    .expect("write png");
    upscale_png(&raw_path, &poster_path, POSTER_WIDTH, POSTER_HEIGHT).expect("upscale png");
    let _ = fs::remove_file(raw_path);

    println!(
        "rendered README hero poster to {} (hash={color_hash:#018x}, mesh_items={mesh_items}, base_x={:.2} m)",
        poster_path.display(),
        sim.observe().base_x_m
    );
}

fn robot_focus(sim: &DiffDriveSim) -> Vec3 {
    let obs = sim.observe();
    Vec3::new(obs.base_x_m, 0.25, obs.base_z_m)
}

fn upscale_png(src: &Path, dst: &Path, width: u32, height: u32) -> std::io::Result<()> {
    let src = src.display();
    let dst = dst.display();
    let status = std::process::Command::new("python3")
        .arg("-c")
        .arg(format!(
            "from PIL import Image; im=Image.open('{src}'); im.resize(({width},{height}), Image.Resampling.LANCZOS).save('{dst}')"
        ))
        .status()?;
    if status.success() {
        Ok(())
    } else {
        Err(std::io::Error::other("failed to upscale png"))
    }
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer
        .write_image_data(rgba)
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::Other, error))?;
    Ok(())
}

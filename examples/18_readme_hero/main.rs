//! Renders README hero media from the mesh diff-drive scene asset.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{build_diff_drive_render_scene, DiffDriveAction, DiffDriveSim};
use rne_math::Vec3;
use rne_render::{hash_rgba8, Camera, MeshRenderCache, RenderBackend, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const CLEAR_COLOR: [f32; 4] = [0.05, 0.08, 0.12, 1.0];
const RENDER_WIDTH: u32 = 640;
const RENDER_HEIGHT: u32 = 360;
const POSTER_WIDTH: u32 = 960;
const POSTER_HEIGHT: u32 = 540;
const DRIVE_STEPS: usize = 60;
const DRIVE_SPEED_RAD_S: f64 = 5.0;
const FRAME_COUNT: usize = 24;
const MIN_UNIQUE_COLORS: usize = 2;

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

    let mesh_items = scene
        .items
        .iter()
        .filter(|item| matches!(item.shape, VisualShape::Mesh { .. }))
        .count();
    assert!(mesh_items > 0, "expected at least one mesh visual in hero scene");

    let focus = robot_focus(&sim);
    let camera = Camera::new(RENDER_WIDTH, RENDER_HEIGHT, std::f64::consts::FRAC_PI_4);
    let frames_dir = media_dir.join("hero-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create hero frame directory");

    let mut frame_paths = Vec::with_capacity(FRAME_COUNT);
    for frame in 0..FRAME_COUNT {
        let yaw = (frame as f64 / FRAME_COUNT as f64) * std::f64::consts::TAU * 0.18 - 0.09;
        let orbit = CameraOrbit {
            focus,
            yaw_rad: yaw,
            pitch_rad: 0.52,
            distance_m: 3.6,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render hero frame");

        let unique = unique_colors(&output.color.rgba8);
        let center = (output.depth.height / 2 * output.color.width + output.color.width / 2) as usize;
        let center_depth = output.depth.depth_m[center];
        if frame == 0 {
            assert!(
                unique >= MIN_UNIQUE_COLORS && center_depth < camera.far_m as f32,
                "hero frame invalid (unique_colors={unique}, center_depth={center_depth:.2} m)"
            );
        }

        let frame_path = frames_dir.join(format!("frame-{frame:03}.png"));
        write_png(
            &frame_path,
            &output.color.rgba8,
            output.color.width,
            output.color.height,
        )
        .expect("write frame png");
        frame_paths.push(frame_path);
    }

    let poster_src = &frame_paths[FRAME_COUNT / 4];
    let poster_path = media_dir.join("rne-hero.png");
    upscale_png(poster_src, &poster_path, POSTER_WIDTH, POSTER_HEIGHT).expect("upscale poster");

    let gif_path = media_dir.join("rne-hero.gif");
    build_gif(&frames_dir, FRAME_COUNT, &gif_path).expect("build hero gif");
    let _ = fs::remove_dir_all(&frames_dir);

    let poster_bytes = fs::read(&poster_path).expect("read poster");
    let color_hash = hash_rgba8(&poster_bytes);

    println!(
        "rendered README hero media to {} and {} (poster_hash={color_hash:#018x}, mesh_items={mesh_items}, base_x={:.2} m)",
        poster_path.display(),
        gif_path.display(),
        sim.observe().base_x_m
    );
}

fn robot_focus(sim: &DiffDriveSim) -> Vec3 {
    let obs = sim.observe();
    Vec3::new(obs.base_x_m, 0.25, obs.base_z_m)
}

fn unique_colors(rgba8: &[u8]) -> usize {
    use std::collections::HashSet;
    rgba8
        .chunks_exact(4)
        .map(|px| (px[0], px[1], px[2], px[3]))
        .collect::<HashSet<_>>()
        .len()
}

fn build_gif(frames_dir: &Path, frame_count: usize, gif_path: &Path) -> std::io::Result<()> {
    let input = frames_dir.join("frame-%03d.png");
    let filter = format!(
        "fps=12,scale={POSTER_WIDTH}:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3"
    );
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            "12",
            "-i",
            &input.to_string_lossy(),
            "-frames:v",
            &frame_count.to_string(),
            "-vf",
            &filter,
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    if status.success() {
        Ok(())
    } else {
        Err(std::io::Error::other("ffmpeg gif encode failed"))
    }
}

fn upscale_png(src: &Path, dst: &Path, width: u32, height: u32) -> std::io::Result<()> {
    let image = image::open(src)
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::Other, error))?
        .resize_to_fill(width, height, image::imageops::FilterType::Lanczos3);
    image
        .save(dst)
        .map_err(|error| std::io::Error::new(std::io::ErrorKind::Other, error))
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

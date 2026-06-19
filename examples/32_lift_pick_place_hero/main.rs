//! Renders README hero media from the real 3D `mm_lift_pick` simulation.
//!
//! This is not a synthetic 2D preview: each GIF frame is produced by stepping
//! [`MobileManipulatorSim`] through the same scripted pick-and-place policy as
//! example 31, then rendering the resulting world with the wgpu backend.
//!
//! Run (needs a GPU and ffmpeg; set `RNE_SKIP_GPU=1` to skip):
//!   cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero

use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, mm_lift_pick_scene_path, LiftPickPlacePolicy,
    MobileManipulatorAction, MobileManipulatorSim,
};
use rne_math::{Quat, Vec3};
use rne_render::{Camera, RenderBackend, RenderScene, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_world::Transform3;

const CLEAR_COLOR: [f32; 4] = [0.58, 0.66, 0.78, 1.0];
const RENDER_WIDTH: u32 = 640;
const RENDER_HEIGHT: u32 = 360;
const POSTER_WIDTH: u32 = 960;
const POSTER_HEIGHT: u32 = 540;
const FRAME_COUNT: usize = 48;
const FPS: usize = 12;
const SETTLE_STEPS: usize = 150;
const MIN_CARRY_M: f64 = 0.5;
const MIN_UNIQUE_COLORS: usize = 8;

fn main() {
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        eprintln!("RNE_SKIP_GPU set; skipping 3D lift pick-place hero render");
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut sim = MobileManipulatorSim::from_scene_path(&mm_lift_pick_scene_path())
        .expect("load mm_lift_pick scene");
    for _ in 0..SETTLE_STEPS {
        sim.step(MobileManipulatorAction::default());
    }

    let start_cube = sim.named_translation_m("lift_cube").expect("cube");
    let mut policy = LiftPickPlacePolicy::new();
    let policy_steps = policy.total_steps() as usize;
    let mut policy_step = 0usize;
    let mut grasped_frames = 0usize;

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu backend");
    let camera = Camera::new(RENDER_WIDTH, RENDER_HEIGHT, std::f64::consts::FRAC_PI_4);
    let frames_dir = media_dir.join("hero-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create hero frame directory");

    let mut frame_paths = Vec::with_capacity(FRAME_COUNT);
    for frame in 0..FRAME_COUNT {
        let target_step = frame * policy_steps / (FRAME_COUNT - 1);
        while policy_step < target_step {
            sim.step(policy.next_action());
            policy_step += 1;
        }
        if sim.is_grasping() {
            grasped_frames += 1;
        }

        let mut scene = build_visual_render_scene(sim.world());
        append_hero_context(&mut scene);
        let orbit = CameraOrbit {
            focus: Vec3::new(0.58, 0.50, -0.36),
            yaw_rad: -1.02,
            pitch_rad: 0.24,
            distance_m: 2.25,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render hero frame");

        if frame == 0 || frame == FRAME_COUNT / 2 {
            let unique = unique_colors(&output.color.rgba8);
            let center =
                (output.depth.height / 2 * output.color.width + output.color.width / 2) as usize;
            let center_depth = output.depth.depth_m[center];
            assert!(
                unique >= MIN_UNIQUE_COLORS && center_depth < camera.far_m as f32,
                "3D hero frame invalid (unique_colors={unique}, center_depth={center_depth:.2} m)"
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

    let final_cube = sim.named_translation_m("lift_cube").expect("cube");
    let carried_m =
        ((final_cube.0 - start_cube.0).powi(2) + (final_cube.2 - start_cube.2).powi(2)).sqrt();
    assert!(
        !sim.is_grasping() && carried_m > MIN_CARRY_M && final_cube.1 < 0.1,
        "expected complete 3D pick-place: grasping={} carried={carried_m:.2} final_y={:.2}",
        sim.is_grasping(),
        final_cube.1
    );
    assert!(
        grasped_frames > 0,
        "expected at least one GIF frame while cube is grasped"
    );

    let poster_src = &frame_paths[FRAME_COUNT / 2];
    let poster_path = media_dir.join("rne-hero.png");
    upscale_png(poster_src, &poster_path, POSTER_WIDTH, POSTER_HEIGHT).expect("upscale poster");

    let still_path = media_dir.join("mm-lift-pickplace.png");
    upscale_png(poster_src, &still_path, POSTER_WIDTH, POSTER_HEIGHT).expect("write still poster");

    let gif_path = media_dir.join("rne-hero.gif");
    build_gif(&frames_dir, FRAME_COUNT, &gif_path).expect("build hero gif");
    let _ = fs::remove_dir_all(&frames_dir);

    println!(
        "rendered 3D README hero to {} and {} (frames={FRAME_COUNT}, grasped_frames={grasped_frames}, carried={carried_m:.2} m, final=({:.2}, {:.2}, {:.2}))",
        poster_path.display(),
        gif_path.display(),
        final_cube.0,
        final_cube.1,
        final_cube.2
    );
}

fn append_hero_context(scene: &mut RenderScene) {
    // Floor and target marker are visual context for the rendered scene; the robot
    // and cube state still come from the physics simulation.
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(Vec3::new(0.55, -0.05, -0.25), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(4.2, 0.1, 3.2),
        },
        [0.28, 0.30, 0.32, 1.0],
        Transform3::IDENTITY,
    ));
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(Vec3::new(0.55, 0.01, -0.87), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(0.28, 0.02, 0.28),
        },
        [0.08, 0.58, 0.46, 1.0],
        Transform3::IDENTITY,
    ));
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(Vec3::new(1.21, 0.01, 0.0), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(0.20, 0.015, 0.20),
        },
        [0.70, 0.45, 0.12, 1.0],
        Transform3::IDENTITY,
    ));
}

fn unique_colors(rgba8: &[u8]) -> usize {
    rgba8
        .chunks_exact(4)
        .map(|px| (px[0], px[1], px[2], px[3]))
        .collect::<HashSet<_>>()
        .len()
}

fn build_gif(frames_dir: &Path, frame_count: usize, gif_path: &Path) -> std::io::Result<()> {
    let input = frames_dir.join("frame-%03d.png");
    let filter = format!(
        "fps={FPS},scale={POSTER_WIDTH}:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=128[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3"
    );
    let status = Command::new("ffmpeg")
        .args([
            "-y",
            "-v",
            "error",
            "-framerate",
            &FPS.to_string(),
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
        .map_err(std::io::Error::other)?
        .resize_to_fill(width, height, image::imageops::FilterType::Lanczos3);
    image.save(dst).map_err(std::io::Error::other)
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer
        .write_image_data(rgba)
        .map_err(std::io::Error::other)?;
    Ok(())
}

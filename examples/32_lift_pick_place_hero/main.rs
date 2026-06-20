//! Renders README hero media from the real 3D `mm_mobile` simulation.
//!
//! This is not a synthetic 2D preview: each GIF frame is produced by stepping
//! [`MobileManipulatorSim`] as the differential-drive base navigates and the arm
//! reaches, then rendering the resulting world with the wgpu backend.
//!
//! Run (needs a GPU and ffmpeg; set `RNE_SKIP_GPU=1` to skip):
//!   cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero

use std::collections::HashSet;
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, mm_mobile_scene_path, MobileManipulatorAction,
    MobileManipulatorObservation, MobileManipulatorSim,
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
const SETTLE_STEPS: usize = 120;
const POLICY_STEPS: usize = 520;
const MIN_BASE_TRAVEL_M: f64 = 0.20;
const MIN_EE_TRAVEL_M: f64 = 0.15;
const MIN_UNIQUE_COLORS: usize = 8;
const HERO_DIGEST_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const HERO_DIGEST_PRIME: u64 = 0x0000_0100_0000_01b3;

fn main() {
    if env::args().any(|arg| arg == "--smoke") {
        let metrics = run_hero_smoke();
        let repeat = run_hero_smoke();
        metrics.assert_deterministic_match(&repeat);
        println!(
            "3D hero simulation smoke ok: digest=0x{:016x}, base_travel={:.2} m, ee_travel={:.2} m, base=({:.2}, {:.2}, {:.2}), ee=({:.2}, {:.2}, {:.2})",
            metrics.trajectory_digest,
            metrics.base_travel_m,
            metrics.ee_travel_m,
            metrics.final_base_m[0],
            metrics.final_base_m[1],
            metrics.final_base_m[2],
            metrics.final_ee_m[0],
            metrics.final_ee_m[1],
            metrics.final_ee_m[2]
        );
        return;
    }

    if std::env::var("RNE_SKIP_GPU").is_ok() {
        eprintln!("RNE_SKIP_GPU set; skipping 3D mobile manipulator hero render");
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    fs::create_dir_all(&media_dir).expect("create media directory");

    let mut sim = MobileManipulatorSim::from_scene_path(&mm_mobile_scene_path())
        .expect("load mm_mobile scene");
    for _ in 0..SETTLE_STEPS {
        sim.step(MobileManipulatorAction::default());
    }

    let start = sim.observe();
    let mut trajectory_digest = HERO_DIGEST_OFFSET;
    mix_observation_digest(&mut trajectory_digest, &start);
    let mut policy = MobileReachHeroPolicy::new();
    let mut policy_step = 0usize;

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu backend");
    let camera = Camera::new(RENDER_WIDTH, RENDER_HEIGHT, std::f64::consts::FRAC_PI_4);
    let frames_dir = media_dir.join("hero-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create hero frame directory");

    let mut frame_paths = Vec::with_capacity(FRAME_COUNT);
    let mut base_path: Vec<Vec3> = Vec::with_capacity(FRAME_COUNT);
    for frame in 0..FRAME_COUNT {
        let target_step = frame * POLICY_STEPS / (FRAME_COUNT - 1);
        while policy_step < target_step {
            let obs = sim.step(policy.next_action());
            mix_observation_digest(&mut trajectory_digest, &obs);
            policy_step += 1;
        }

        let obs = sim.observe();
        base_path.push(Vec3::new(obs.base_x_m, 0.0, obs.base_z_m));
        let mut scene = build_visual_render_scene(sim.world());
        append_hero_context(
            &mut scene,
            obs.base_x_m,
            obs.base_z_m,
            obs.ee_y_m,
            &base_path,
        );
        let orbit = CameraOrbit {
            focus: Vec3::new(obs.base_x_m + 0.28, 0.34, obs.base_z_m),
            yaw_rad: -0.88,
            pitch_rad: 0.30,
            distance_m: 2.35,
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

    let final_obs = sim.observe();
    let metrics = HeroSimMetrics::new(
        [start.base_x_m, start.base_y_m, start.base_z_m],
        [start.ee_x_m, start.ee_y_m, start.ee_z_m],
        [final_obs.base_x_m, final_obs.base_y_m, final_obs.base_z_m],
        [final_obs.ee_x_m, final_obs.ee_y_m, final_obs.ee_z_m],
        trajectory_digest,
    );
    metrics.assert_navigation_and_reach();
    write_sim_metadata_if_requested(
        metrics.base_travel_m,
        metrics.ee_travel_m,
        metrics.trajectory_digest,
        metrics.final_base_m,
        metrics.final_ee_m,
    )
    .expect("write hero simulation metadata");

    let poster_src = &frame_paths[FRAME_COUNT / 2];
    let poster_path = media_dir.join("rne-hero.png");
    upscale_png(poster_src, &poster_path, POSTER_WIDTH, POSTER_HEIGHT).expect("upscale poster");

    let gif_path = media_dir.join("rne-hero.gif");
    build_gif(&frames_dir, FRAME_COUNT, &gif_path).expect("build hero gif");
    let _ = fs::remove_dir_all(&frames_dir);

    println!(
        "rendered 3D mobile manipulator hero to {} and {} (frames={FRAME_COUNT}, digest=0x{:016x}, base_travel={:.2} m, ee_travel={:.2} m, base=({:.2}, {:.2}, {:.2}), ee=({:.2}, {:.2}, {:.2}))",
        poster_path.display(),
        gif_path.display(),
        metrics.trajectory_digest,
        metrics.base_travel_m,
        metrics.ee_travel_m,
        metrics.final_base_m[0],
        metrics.final_base_m[1],
        metrics.final_base_m[2],
        metrics.final_ee_m[0],
        metrics.final_ee_m[1],
        metrics.final_ee_m[2]
    );
}

#[derive(Clone, Copy, Debug)]
struct HeroSimMetrics {
    base_travel_m: f64,
    ee_travel_m: f64,
    trajectory_digest: u64,
    final_base_m: [f64; 3],
    final_ee_m: [f64; 3],
}

impl HeroSimMetrics {
    fn new(
        start_base_m: [f64; 3],
        start_ee_m: [f64; 3],
        final_base_m: [f64; 3],
        final_ee_m: [f64; 3],
        trajectory_digest: u64,
    ) -> Self {
        let base_travel_m = ((final_base_m[0] - start_base_m[0]).powi(2)
            + (final_base_m[2] - start_base_m[2]).powi(2))
        .sqrt();
        let ee_travel_m = ((final_ee_m[0] - start_ee_m[0]).powi(2)
            + (final_ee_m[1] - start_ee_m[1]).powi(2)
            + (final_ee_m[2] - start_ee_m[2]).powi(2))
        .sqrt();
        Self {
            base_travel_m,
            ee_travel_m,
            trajectory_digest,
            final_base_m,
            final_ee_m,
        }
    }

    fn assert_navigation_and_reach(&self) {
        assert!(
            self.base_travel_m > MIN_BASE_TRAVEL_M,
            "expected mobile base navigation: base_travel={:.2} m",
            self.base_travel_m
        );
        assert!(
            self.ee_travel_m > MIN_EE_TRAVEL_M,
            "expected manipulator reach: ee_travel={:.2} m",
            self.ee_travel_m
        );
    }

    fn assert_deterministic_match(&self, repeat: &Self) {
        assert_eq!(
            self.trajectory_digest, repeat.trajectory_digest,
            "hero simulation trajectory digest changed between identical runs"
        );
        assert_eq!(
            self.base_travel_m.to_bits(),
            repeat.base_travel_m.to_bits(),
            "hero simulation base travel changed between identical runs"
        );
        assert_eq!(
            self.ee_travel_m.to_bits(),
            repeat.ee_travel_m.to_bits(),
            "hero simulation end-effector travel changed between identical runs"
        );
        assert_eq!(
            self.final_base_m.map(f64::to_bits),
            repeat.final_base_m.map(f64::to_bits),
            "hero simulation final base pose changed between identical runs"
        );
        assert_eq!(
            self.final_ee_m.map(f64::to_bits),
            repeat.final_ee_m.map(f64::to_bits),
            "hero simulation final end-effector pose changed between identical runs"
        );
    }
}

fn run_hero_smoke() -> HeroSimMetrics {
    let mut sim = MobileManipulatorSim::from_scene_path(&mm_mobile_scene_path())
        .expect("load mm_mobile scene");
    for _ in 0..SETTLE_STEPS {
        sim.step(MobileManipulatorAction::default());
    }

    let start = sim.observe();
    let mut trajectory_digest = HERO_DIGEST_OFFSET;
    mix_observation_digest(&mut trajectory_digest, &start);
    let mut policy = MobileReachHeroPolicy::new();
    for _ in 0..POLICY_STEPS {
        let obs = sim.step(policy.next_action());
        mix_observation_digest(&mut trajectory_digest, &obs);
    }

    let final_obs = sim.observe();
    let metrics = HeroSimMetrics::new(
        [start.base_x_m, start.base_y_m, start.base_z_m],
        [start.ee_x_m, start.ee_y_m, start.ee_z_m],
        [final_obs.base_x_m, final_obs.base_y_m, final_obs.base_z_m],
        [final_obs.ee_x_m, final_obs.ee_y_m, final_obs.ee_z_m],
        trajectory_digest,
    );
    metrics.assert_navigation_and_reach();
    metrics
}

fn mix_observation_digest(digest: &mut u64, obs: &MobileManipulatorObservation) {
    for value in [
        obs.base_x_m,
        obs.base_y_m,
        obs.base_z_m,
        obs.base_yaw_rad,
        obs.ee_x_m,
        obs.ee_y_m,
        obs.ee_z_m,
        obs.shoulder_position_rad,
        obs.elbow_position_rad,
        obs.gripper_position_rad,
        obs.target_dx_m,
        obs.target_dy_m,
        obs.target_dz_m,
    ] {
        mix_digest_u64(digest, value.to_bits());
    }
    mix_digest_u64(digest, obs.wrist_camera_pixels as u64);
    mix_digest_u64(digest, obs.joint_state_count as u64);
}

fn mix_digest_u64(digest: &mut u64, value: u64) {
    for byte in value.to_le_bytes() {
        *digest ^= u64::from(byte);
        *digest = digest.wrapping_mul(HERO_DIGEST_PRIME);
    }
}

struct MobileReachHeroPolicy {
    step: usize,
}

impl MobileReachHeroPolicy {
    fn new() -> Self {
        Self { step: 0 }
    }

    fn next_action(&mut self) -> MobileManipulatorAction {
        self.step += 1;
        match self.step {
            0..=90 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 1.2,
                right_wheel_velocity_rad_s: 1.2,
                ..MobileManipulatorAction::default()
            },
            91..=170 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 0.35,
                right_wheel_velocity_rad_s: 1.0,
                shoulder_velocity_rad_s: 0.8,
                ..MobileManipulatorAction::default()
            },
            171..=420 => MobileManipulatorAction {
                shoulder_velocity_rad_s: 1.2,
                elbow_velocity_rad_s: -0.8,
                ..MobileManipulatorAction::default()
            },
            _ => MobileManipulatorAction::default(),
        }
    }
}

fn append_hero_context(
    scene: &mut RenderScene,
    base_x_m: f64,
    base_z_m: f64,
    ee_y_m: f64,
    base_path: &[Vec3],
) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(Vec3::new(base_x_m, -0.05, base_z_m), Quat::IDENTITY),
        VisualShape::Box {
            size_m: Vec3::new(4.2, 0.1, 3.2),
        },
        [0.28, 0.30, 0.32, 1.0],
        Transform3::IDENTITY,
    ));
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(
            Vec3::new(base_x_m + 0.90, ee_y_m, base_z_m + 0.24),
            Quat::IDENTITY,
        ),
        VisualShape::Sphere { radius_m: 0.08 },
        [0.08, 0.78, 0.62, 1.0],
        Transform3::IDENTITY,
    ));
    for point in base_path.iter().step_by(3) {
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(
                Vec3::new(point.x, 0.012, point.z),
                Quat::IDENTITY,
            ),
            VisualShape::Box {
                size_m: Vec3::new(0.10, 0.018, 0.10),
            },
            [0.10, 0.55, 0.92, 1.0],
            Transform3::IDENTITY,
        ));
    }
}

fn write_sim_metadata_if_requested(
    base_travel_m: f64,
    ee_travel_m: f64,
    trajectory_digest: u64,
    final_base_m: [f64; 3],
    final_ee_m: [f64; 3],
) -> std::io::Result<()> {
    let Some(path) = env::var_os("RNE_HERO_SIM_METADATA") else {
        return Ok(());
    };
    let path = PathBuf::from(path);
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent)?;
    }
    let payload = format!(
        concat!(
            "{{\n",
            "  \"base_travel_m\": {:.6},\n",
            "  \"ee_travel_m\": {:.6},\n",
            "  \"trajectory_digest\": \"0x{:016x}\",\n",
            "  \"final_base_m\": [{:.6}, {:.6}, {:.6}],\n",
            "  \"final_ee_m\": [{:.6}, {:.6}, {:.6}],\n",
            "  \"min_base_travel_m\": {:.6},\n",
            "  \"min_ee_travel_m\": {:.6}\n",
            "}}\n"
        ),
        base_travel_m,
        ee_travel_m,
        trajectory_digest,
        final_base_m[0],
        final_base_m[1],
        final_base_m[2],
        final_ee_m[0],
        final_ee_m[1],
        final_ee_m[2],
        MIN_BASE_TRAVEL_M,
        MIN_EE_TRAVEL_M
    );
    fs::write(path, payload)
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

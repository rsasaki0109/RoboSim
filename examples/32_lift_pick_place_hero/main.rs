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
use rne_math::{yaw_rad, Quat, Vec3};
use rne_render::{Camera, RenderBackend, RenderScene, VisualShape};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::Link;
use rne_world::{world_transform_of, Transform3};

const CLEAR_COLOR: [f32; 4] = [0.06, 0.07, 0.07, 1.0];
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
const MAX_FINAL_EE_TARGET_ERROR_M: f64 = 0.05;
const MIN_CONSECUTIVE_FRAME_DELTA_RATIO: f64 = 0.01;
const MIN_FIRST_LAST_FRAME_DELTA_RATIO: f64 = 0.08;
const MIN_UNIQUE_COLORS: usize = 8;
const EXPECTED_BASE_Y_M: f64 = 0.25;
const MAX_BASE_HEIGHT_ERROR_M: f64 = 0.01;
const MIN_BASE_YAW_ONLY_DOT: f64 = 0.999_999;
const HERO_DIGEST_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const HERO_DIGEST_PRIME: u64 = 0x0000_0100_0000_01b3;
const HOUSE_CENTER_M: Vec3 = Vec3::new(1.0, 0.0, -1.1);
const REACH_TARGET_M: Vec3 = Vec3::new(2.17, 0.40, -2.48);

fn main() {
    if env::args().any(|arg| arg == "--smoke") {
        let metrics = run_hero_smoke();
        let repeat = run_hero_smoke();
        metrics.assert_deterministic_match(&repeat);
        println!(
            "3D hero simulation smoke ok: digest=0x{:016x}, base_travel={:.2} m, ee_travel={:.2} m, final_ee_target_error={:.3} m, max_base_height_error={:.4} m, min_base_yaw_only_dot={:.9}, base=({:.2}, {:.2}, {:.2}), ee=({:.2}, {:.2}, {:.2})",
            metrics.trajectory_digest,
            metrics.base_travel_m,
            metrics.ee_travel_m,
            metrics.final_ee_target_error_m,
            metrics.max_base_height_error_m,
            metrics.min_base_yaw_only_dot,
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
    let mut planarity = HeroBasePlanarity::new();
    planarity.observe(&sim);
    let mut policy = MobileReachHeroPolicy::new();
    let mut policy_step = 0usize;

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu backend");
    let camera = Camera::new(RENDER_WIDTH, RENDER_HEIGHT, std::f64::consts::FRAC_PI_4);
    let frames_dir = media_dir.join("hero-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create hero frame directory");

    let mut frame_paths = Vec::with_capacity(FRAME_COUNT);
    let mut base_path: Vec<Vec3> = Vec::with_capacity(FRAME_COUNT);
    let mut render_metrics = HeroRenderMetrics::new();
    for frame in 0..FRAME_COUNT {
        let target_step = frame * POLICY_STEPS / (FRAME_COUNT - 1);
        while policy_step < target_step {
            let obs = sim.step(policy.next_action());
            mix_observation_digest(&mut trajectory_digest, &obs);
            planarity.observe(&sim);
            policy_step += 1;
        }

        let obs = sim.observe();
        base_path.push(Vec3::new(obs.base_x_m, 0.0, obs.base_z_m));
        let mut scene = build_visual_render_scene(sim.world());
        append_hero_context(&mut scene, &base_path);
        let orbit = CameraOrbit {
            focus: Vec3::new(
                obs.base_x_m * 0.62 + REACH_TARGET_M.x * 0.24 + HOUSE_CENTER_M.x * 0.14,
                0.52,
                obs.base_z_m * 0.62 + REACH_TARGET_M.z * 0.24 + HOUSE_CENTER_M.z * 0.14,
            ),
            yaw_rad: -1.06,
            pitch_rad: 1.15,
            distance_m: 3.45,
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
        render_metrics.observe(&output.color.rgba8);

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
        planarity,
    );
    metrics.assert_navigation_and_reach();
    render_metrics.assert_dynamic();
    write_sim_metadata_if_requested(&metrics, &render_metrics)
        .expect("write hero simulation metadata");

    let poster_src = &frame_paths[FRAME_COUNT - 1];
    let poster_path = media_dir.join("rne-hero.png");
    upscale_png(poster_src, &poster_path, POSTER_WIDTH, POSTER_HEIGHT).expect("upscale poster");

    let gif_path = media_dir.join("rne-hero.gif");
    build_gif(&frames_dir, FRAME_COUNT, &gif_path).expect("build hero gif");
    let _ = fs::remove_dir_all(&frames_dir);

    println!(
        "rendered 3D mobile manipulator hero to {} and {} (frames={FRAME_COUNT}, digest=0x{:016x}, base_travel={:.2} m, ee_travel={:.2} m, final_ee_target_error={:.3} m, base=({:.2}, {:.2}, {:.2}), ee=({:.2}, {:.2}, {:.2}))",
        poster_path.display(),
        gif_path.display(),
        metrics.trajectory_digest,
        metrics.base_travel_m,
        metrics.ee_travel_m,
        metrics.final_ee_target_error_m,
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
    max_base_height_error_m: f64,
    min_base_yaw_only_dot: f64,
    final_ee_target_error_m: f64,
    final_base_m: [f64; 3],
    final_ee_m: [f64; 3],
}

#[derive(Clone, Debug)]
struct HeroRenderMetrics {
    frame_count: usize,
    first_frame_rgba8: Vec<u8>,
    previous_frame_rgba8: Vec<u8>,
    min_consecutive_frame_delta_ratio: f64,
    first_last_frame_delta_ratio: f64,
}

impl HeroRenderMetrics {
    fn new() -> Self {
        Self {
            frame_count: 0,
            first_frame_rgba8: Vec::new(),
            previous_frame_rgba8: Vec::new(),
            min_consecutive_frame_delta_ratio: 1.0,
            first_last_frame_delta_ratio: 0.0,
        }
    }

    fn observe(&mut self, rgba8: &[u8]) {
        if self.frame_count == 0 {
            self.first_frame_rgba8 = rgba8.to_vec();
        } else {
            let delta_ratio = frame_delta_ratio(&self.previous_frame_rgba8, rgba8);
            self.min_consecutive_frame_delta_ratio =
                self.min_consecutive_frame_delta_ratio.min(delta_ratio);
            self.first_last_frame_delta_ratio = frame_delta_ratio(&self.first_frame_rgba8, rgba8);
        }
        self.previous_frame_rgba8 = rgba8.to_vec();
        self.frame_count += 1;
    }

    fn assert_dynamic(&self) {
        assert_eq!(
            self.frame_count, FRAME_COUNT,
            "expected render metrics for every hero frame"
        );
        assert!(
            self.min_consecutive_frame_delta_ratio >= MIN_CONSECUTIVE_FRAME_DELTA_RATIO,
            "expected animated hero frames: min_consecutive_frame_delta_ratio={:.4}",
            self.min_consecutive_frame_delta_ratio
        );
        assert!(
            self.first_last_frame_delta_ratio >= MIN_FIRST_LAST_FRAME_DELTA_RATIO,
            "expected visible hero progression: first_last_frame_delta_ratio={:.4}",
            self.first_last_frame_delta_ratio
        );
    }
}

impl HeroSimMetrics {
    fn new(
        start_base_m: [f64; 3],
        start_ee_m: [f64; 3],
        final_base_m: [f64; 3],
        final_ee_m: [f64; 3],
        trajectory_digest: u64,
        planarity: HeroBasePlanarity,
    ) -> Self {
        let base_travel_m = ((final_base_m[0] - start_base_m[0]).powi(2)
            + (final_base_m[2] - start_base_m[2]).powi(2))
        .sqrt();
        let ee_travel_m = ((final_ee_m[0] - start_ee_m[0]).powi(2)
            + (final_ee_m[1] - start_ee_m[1]).powi(2)
            + (final_ee_m[2] - start_ee_m[2]).powi(2))
        .sqrt();
        let final_ee_target_error_m = ((final_ee_m[0] - REACH_TARGET_M.x).powi(2)
            + (final_ee_m[1] - REACH_TARGET_M.y).powi(2)
            + (final_ee_m[2] - REACH_TARGET_M.z).powi(2))
        .sqrt();
        Self {
            base_travel_m,
            ee_travel_m,
            trajectory_digest,
            max_base_height_error_m: planarity.max_base_height_error_m,
            min_base_yaw_only_dot: planarity.min_base_yaw_only_dot,
            final_ee_target_error_m,
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
        assert!(
            self.max_base_height_error_m <= MAX_BASE_HEIGHT_ERROR_M,
            "expected planar mobile base height: max_error={:.4} m",
            self.max_base_height_error_m
        );
        assert!(
            self.min_base_yaw_only_dot >= MIN_BASE_YAW_ONLY_DOT,
            "expected upright mobile base orientation: min_yaw_only_dot={:.9}",
            self.min_base_yaw_only_dot
        );
        assert!(
            self.final_ee_target_error_m <= MAX_FINAL_EE_TARGET_ERROR_M,
            "expected manipulator to reach target: final_ee_target_error={:.3} m",
            self.final_ee_target_error_m
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
            self.max_base_height_error_m.to_bits(),
            repeat.max_base_height_error_m.to_bits(),
            "hero simulation base height error changed between identical runs"
        );
        assert_eq!(
            self.min_base_yaw_only_dot.to_bits(),
            repeat.min_base_yaw_only_dot.to_bits(),
            "hero simulation base upright metric changed between identical runs"
        );
        assert_eq!(
            self.final_ee_target_error_m.to_bits(),
            repeat.final_ee_target_error_m.to_bits(),
            "hero simulation final reach error changed between identical runs"
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
    let mut planarity = HeroBasePlanarity::new();
    planarity.observe(&sim);
    let mut policy = MobileReachHeroPolicy::new();
    for _ in 0..POLICY_STEPS {
        let obs = sim.step(policy.next_action());
        mix_observation_digest(&mut trajectory_digest, &obs);
        planarity.observe(&sim);
    }

    let final_obs = sim.observe();
    let metrics = HeroSimMetrics::new(
        [start.base_x_m, start.base_y_m, start.base_z_m],
        [start.ee_x_m, start.ee_y_m, start.ee_z_m],
        [final_obs.base_x_m, final_obs.base_y_m, final_obs.base_z_m],
        [final_obs.ee_x_m, final_obs.ee_y_m, final_obs.ee_z_m],
        trajectory_digest,
        planarity,
    );
    metrics.assert_navigation_and_reach();
    metrics
}

#[derive(Clone, Copy, Debug)]
struct HeroBasePlanarity {
    max_base_height_error_m: f64,
    min_base_yaw_only_dot: f64,
}

impl HeroBasePlanarity {
    fn new() -> Self {
        Self {
            max_base_height_error_m: 0.0,
            min_base_yaw_only_dot: 1.0,
        }
    }

    fn observe(&mut self, sim: &MobileManipulatorSim) {
        let transform = base_link_transform(sim);
        let yaw_only = Quat::from_rotation_y(yaw_rad(transform.rotation));
        let height_error_m = (transform.translation.y - EXPECTED_BASE_Y_M).abs();
        self.max_base_height_error_m = self.max_base_height_error_m.max(height_error_m);
        self.min_base_yaw_only_dot = self
            .min_base_yaw_only_dot
            .min(transform.rotation.dot(yaw_only).abs());
    }
}

fn base_link_transform(sim: &MobileManipulatorSim) -> Transform3 {
    let base_link = sim
        .world()
        .iter_entities()
        .find_map(|entity| {
            let link = entity.get::<Link>()?;
            (link.name == "base_link").then_some(entity.id())
        })
        .expect("hero simulation should contain a base_link");
    world_transform_of(sim.world(), base_link)
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
            0..=180 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 5.0,
                right_wheel_velocity_rad_s: 5.0,
                ..MobileManipulatorAction::default()
            },
            181..=300 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 2.0,
                right_wheel_velocity_rad_s: 5.0,
                shoulder_velocity_rad_s: 0.4,
                ..MobileManipulatorAction::default()
            },
            301..=420 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 3.0,
                right_wheel_velocity_rad_s: 3.0,
                shoulder_velocity_rad_s: 0.9,
                elbow_velocity_rad_s: -0.5,
                ..MobileManipulatorAction::default()
            },
            _ => MobileManipulatorAction::default(),
        }
    }
}

fn append_hero_context(scene: &mut RenderScene, base_path: &[Vec3]) {
    push_box(
        scene,
        Vec3::new(HOUSE_CENTER_M.x, -0.055, HOUSE_CENTER_M.z),
        Vec3::new(8.0, 0.10, 5.8),
        [0.27, 0.29, 0.29, 1.0],
    );
    push_box(
        scene,
        Vec3::new(HOUSE_CENTER_M.x, 0.22, HOUSE_CENTER_M.z - 2.82),
        Vec3::new(8.0, 0.48, 0.08),
        [0.62, 0.64, 0.60, 1.0],
    );
    push_box(
        scene,
        Vec3::new(HOUSE_CENTER_M.x - 3.95, 0.22, HOUSE_CENTER_M.z),
        Vec3::new(0.08, 0.48, 5.8),
        [0.58, 0.61, 0.62, 1.0],
    );
    push_box(
        scene,
        Vec3::new(HOUSE_CENTER_M.x + 3.95, 0.22, HOUSE_CENTER_M.z),
        Vec3::new(0.08, 0.48, 5.8),
        [0.58, 0.61, 0.62, 1.0],
    );
    push_box(
        scene,
        Vec3::new(HOUSE_CENTER_M.x - 0.35, 0.16, HOUSE_CENTER_M.z - 0.25),
        Vec3::new(0.10, 0.34, 1.80),
        [0.70, 0.70, 0.64, 1.0],
    );
    push_box(
        scene,
        Vec3::new(HOUSE_CENTER_M.x - 1.65, 0.16, HOUSE_CENTER_M.z - 1.35),
        Vec3::new(2.20, 0.34, 0.10),
        [0.70, 0.70, 0.64, 1.0],
    );
    push_box(
        scene,
        Vec3::new(2.10, 0.12, -2.48),
        Vec3::new(0.72, 0.24, 0.56),
        [0.39, 0.33, 0.25, 1.0],
    );
    push_box(
        scene,
        Vec3::new(0.10, 0.14, 0.18),
        Vec3::new(0.82, 0.28, 0.54),
        [0.44, 0.42, 0.35, 1.0],
    );
    push_box(
        scene,
        Vec3::new(-1.65, 0.20, -0.65),
        Vec3::new(0.56, 0.40, 0.80),
        [0.25, 0.40, 0.44, 1.0],
    );
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(REACH_TARGET_M, Quat::IDENTITY),
        VisualShape::Sphere { radius_m: 0.09 },
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

fn push_box(scene: &mut RenderScene, translation_m: Vec3, size_m: Vec3, color_rgba: [f32; 4]) {
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(translation_m, Quat::IDENTITY),
        VisualShape::Box { size_m },
        color_rgba,
        Transform3::IDENTITY,
    ));
}

fn write_sim_metadata_if_requested(
    metrics: &HeroSimMetrics,
    render_metrics: &HeroRenderMetrics,
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
            "  \"max_base_height_error_m\": {:.6},\n",
            "  \"min_base_yaw_only_dot\": {:.9},\n",
            "  \"final_ee_target_error_m\": {:.6},\n",
            "  \"final_base_m\": [{:.6}, {:.6}, {:.6}],\n",
            "  \"final_ee_m\": [{:.6}, {:.6}, {:.6}],\n",
            "  \"min_base_travel_m\": {:.6},\n",
            "  \"min_ee_travel_m\": {:.6},\n",
            "  \"max_final_ee_target_error_m\": {:.6},\n",
            "  \"min_consecutive_frame_delta_ratio\": {:.6},\n",
            "  \"first_last_frame_delta_ratio\": {:.6},\n",
            "  \"min_consecutive_frame_delta_ratio_threshold\": {:.6},\n",
            "  \"min_first_last_frame_delta_ratio_threshold\": {:.6}\n",
            "}}\n"
        ),
        metrics.base_travel_m,
        metrics.ee_travel_m,
        metrics.trajectory_digest,
        metrics.max_base_height_error_m,
        metrics.min_base_yaw_only_dot,
        metrics.final_ee_target_error_m,
        metrics.final_base_m[0],
        metrics.final_base_m[1],
        metrics.final_base_m[2],
        metrics.final_ee_m[0],
        metrics.final_ee_m[1],
        metrics.final_ee_m[2],
        MIN_BASE_TRAVEL_M,
        MIN_EE_TRAVEL_M,
        MAX_FINAL_EE_TARGET_ERROR_M,
        render_metrics.min_consecutive_frame_delta_ratio,
        render_metrics.first_last_frame_delta_ratio,
        MIN_CONSECUTIVE_FRAME_DELTA_RATIO,
        MIN_FIRST_LAST_FRAME_DELTA_RATIO
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

fn frame_delta_ratio(previous_rgba8: &[u8], current_rgba8: &[u8]) -> f64 {
    assert_eq!(
        previous_rgba8.len(),
        current_rgba8.len(),
        "hero frame buffers must have identical dimensions"
    );
    let pixel_count = previous_rgba8.len() / 4;
    if pixel_count == 0 {
        return 0.0;
    }
    let changed_pixels = previous_rgba8
        .chunks_exact(4)
        .zip(current_rgba8.chunks_exact(4))
        .filter(|(previous, current)| previous != current)
        .count();
    changed_pixels as f64 / pixel_count as f64
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

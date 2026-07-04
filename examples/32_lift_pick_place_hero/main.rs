//! Renders README hero media from the real 3D `mm_mobile` simulation.
//!
//! This is not a synthetic 2D preview: each GIF frame is produced by stepping
//! [`MobileManipulatorSim`] as the differential-drive base navigates and the arm
//! carries a task object, then rendering the resulting world with the wgpu backend.
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
const ANIMATION_FRAME_COUNT: usize = 100;
const HOLD_FRAME_COUNT: usize = 10;
const FRAME_COUNT: usize = ANIMATION_FRAME_COUNT + HOLD_FRAME_COUNT;
const FPS: usize = 15;
const GIF_MAX_COLORS: u32 = 192;
const GIF_BAYER_SCALE: u32 = 4;
const HERO_ENCODE_MAX_BYTE_SIZE: u64 = 3_500_000;
const MAX_HOLD_FRAME_DELTA_RATIO: f64 = 0.02;
const CAMERA_ORBIT_YAW_START_RAD: f64 = -1.18;
const CAMERA_ORBIT_YAW_END_RAD: f64 = -0.94;
const SETTLE_STEPS: usize = 120;
const POLICY_STEPS: usize = 680;
const MIN_BASE_TRAVEL_M: f64 = 0.20;
const MIN_EE_TRAVEL_M: f64 = 0.15;
const MAX_FINAL_EE_TARGET_ERROR_M: f64 = 0.05;
const MIN_CONSECUTIVE_FRAME_DELTA_RATIO: f64 = 0.0025;
const MIN_FIRST_LAST_FRAME_DELTA_RATIO: f64 = 0.08;
const MIN_UNIQUE_COLORS: usize = 8;
const EXPECTED_BASE_Y_M: f64 = 0.25;
const MAX_BASE_HEIGHT_ERROR_M: f64 = 0.01;
const MIN_BASE_YAW_ONLY_DOT: f64 = 0.999_999;
const HERO_DIGEST_OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
const HERO_DIGEST_PRIME: u64 = 0x0000_0100_0000_01b3;
const HOUSE_CENTER_M: Vec3 = Vec3::new(1.0, 0.0, -1.1);
const PICK_OBJECT_M: Vec3 = Vec3::new(1.52, 0.40, -0.86);
const PLACE_TARGET_M: Vec3 = Vec3::new(2.17, 0.40, -2.48);
const REACH_TARGET_M: Vec3 = Vec3::new(2.81, 0.36, -2.21);
const OBJECT_GRASP_STEP: usize = 310;
const POSTER_POLICY_STEP: usize = OBJECT_GRASP_STEP + 45;
const OBJECT_RELEASE_STEP: usize = 620;
const PICK_MANIPULATION_START_STEP: usize = 301;
const PICK_MANIPULATION_END_STEP: usize = 390;
const RELEASE_MANIPULATION_START_STEP: usize = OBJECT_RELEASE_STEP;
const RELEASE_MANIPULATION_END_STEP: usize = POLICY_STEPS;
const MIN_OBJECT_TRANSPORT_M: f64 = 0.35;
const MAX_FINAL_OBJECT_PLACE_ERROR_M: f64 = 0.20;
const MIN_GRASPED_STEPS: usize = 12;
const TASK_OBJECT_SIZE_M: Vec3 = Vec3::new(0.11, 0.11, 0.11);
const HERO_WALL_COLOR_RGBA: [f32; 4] = [0.70, 0.70, 0.64, 1.0];
const HERO_WALLS: [HeroBox; 5] = [
    HeroBox {
        center_m: Vec3::new(HOUSE_CENTER_M.x, 0.22, HOUSE_CENTER_M.z - 2.82),
        size_m: Vec3::new(8.0, 0.48, 0.08),
        color_rgba: [0.62, 0.64, 0.60, 1.0],
    },
    HeroBox {
        center_m: Vec3::new(HOUSE_CENTER_M.x - 3.95, 0.22, HOUSE_CENTER_M.z),
        size_m: Vec3::new(0.08, 0.48, 5.8),
        color_rgba: [0.58, 0.61, 0.62, 1.0],
    },
    HeroBox {
        center_m: Vec3::new(HOUSE_CENTER_M.x + 3.95, 0.22, HOUSE_CENTER_M.z),
        size_m: Vec3::new(0.08, 0.48, 5.8),
        color_rgba: [0.58, 0.61, 0.62, 1.0],
    },
    HeroBox {
        center_m: Vec3::new(HOUSE_CENTER_M.x - 2.25, 0.16, HOUSE_CENTER_M.z - 0.05),
        size_m: Vec3::new(0.10, 0.34, 1.55),
        color_rgba: HERO_WALL_COLOR_RGBA,
    },
    HeroBox {
        center_m: Vec3::new(HOUSE_CENTER_M.x - 2.10, 0.16, HOUSE_CENTER_M.z - 1.95),
        size_m: Vec3::new(1.70, 0.34, 0.10),
        color_rgba: HERO_WALL_COLOR_RGBA,
    },
];

#[derive(Clone, Copy, Debug)]
struct HeroBox {
    center_m: Vec3,
    size_m: Vec3,
    color_rgba: [f32; 4],
}

#[derive(Clone, Copy, Debug)]
struct HeroSampleSegment {
    step_start: usize,
    step_end: usize,
    frame_count: usize,
}

const HERO_SAMPLE_SEGMENTS: [HeroSampleSegment; 5] = [
    HeroSampleSegment {
        step_start: 0,
        step_end: 175,
        frame_count: 12,
    },
    HeroSampleSegment {
        step_start: 175,
        step_end: 300,
        frame_count: 14,
    },
    HeroSampleSegment {
        step_start: 300,
        step_end: 390,
        frame_count: 22,
    },
    HeroSampleSegment {
        step_start: 390,
        step_end: 600,
        frame_count: 28,
    },
    HeroSampleSegment {
        step_start: 600,
        step_end: POLICY_STEPS,
        frame_count: 24,
    },
];

/// Maps each animation frame to a monotonic policy step with dense sampling during manipulation.
fn hero_animation_policy_steps() -> [usize; ANIMATION_FRAME_COUNT] {
    let mut steps = [0usize; ANIMATION_FRAME_COUNT];
    let mut frame = 0usize;
    for segment in HERO_SAMPLE_SEGMENTS {
        let span = segment.step_end.saturating_sub(segment.step_start);
        for index in 0..segment.frame_count {
            let mut step = if segment.frame_count <= 1 {
                segment.step_end
            } else {
                segment.step_start + (index * span) / (segment.frame_count - 1)
            };
            if frame > 0 && step <= steps[frame - 1] {
                step = (steps[frame - 1] + 1).min(segment.step_end);
            }
            steps[frame] = step;
            frame += 1;
        }
    }
    debug_assert_eq!(frame, ANIMATION_FRAME_COUNT);
    steps
}

fn hero_poster_animation_frame(steps: &[usize; ANIMATION_FRAME_COUNT]) -> usize {
    steps
        .iter()
        .enumerate()
        .min_by_key(|(_, step)| step.abs_diff(POSTER_POLICY_STEP))
        .map(|(index, _)| index)
        .unwrap_or(0)
}

fn hero_camera_yaw_rad(animation_frame: usize) -> f64 {
    let progress = animation_frame as f64 / (ANIMATION_FRAME_COUNT.saturating_sub(1).max(1) as f64);
    CAMERA_ORBIT_YAW_START_RAD + (CAMERA_ORBIT_YAW_END_RAD - CAMERA_ORBIT_YAW_START_RAD) * progress
}

fn main() {
    if env::args().any(|arg| arg == "--trace") {
        run_hero_trace();
        return;
    }

    if env::args().any(|arg| arg == "--smoke") {
        let metrics = run_hero_smoke();
        let repeat = run_hero_smoke();
        metrics.assert_deterministic_match(&repeat);
        println!(
            "3D hero simulation smoke ok: digest=0x{:016x}, base_travel={:.2} m, ee_travel={:.2} m, object_transport={:.2} m, final_ee_target_error={:.3} m, final_object_place_error={:.3} m, grasped_steps={}, released_after_grasp={}, max_base_height_error={:.4} m, min_base_yaw_only_dot={:.9}, base=({:.2}, {:.2}, {:.2}), ee=({:.2}, {:.2}, {:.2}), object=({:.2}, {:.2}, {:.2})",
            metrics.trajectory_digest,
            metrics.base_travel_m,
            metrics.ee_travel_m,
            metrics.object_transport_m,
            metrics.final_ee_target_error_m,
            metrics.final_object_place_error_m,
            metrics.grasped_steps,
            metrics.released_after_grasp,
            metrics.max_base_height_error_m,
            metrics.min_base_yaw_only_dot,
            metrics.final_base_m[0],
            metrics.final_base_m[1],
            metrics.final_base_m[2],
            metrics.final_ee_m[0],
            metrics.final_ee_m[1],
            metrics.final_ee_m[2],
            metrics.final_object_m[0],
            metrics.final_object_m[1],
            metrics.final_object_m[2]
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
    let mut task = HeroTaskProgress::new();
    mix_task_digest(&mut trajectory_digest, &task);
    let mut policy = MobilePickPlaceHeroPolicy::new();
    let mut policy_step = 0usize;

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu backend");
    let camera = Camera::new(RENDER_WIDTH, RENDER_HEIGHT, std::f64::consts::FRAC_PI_4);
    let frames_dir = media_dir.join("hero-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create hero frame directory");

    let mut frame_paths = Vec::with_capacity(FRAME_COUNT);
    let mut base_path: Vec<Vec3> = Vec::with_capacity(ANIMATION_FRAME_COUNT);
    let mut object_path: Vec<Vec3> = Vec::with_capacity(ANIMATION_FRAME_COUNT);
    let mut render_metrics = HeroRenderMetrics::new();
    let animation_steps = hero_animation_policy_steps();
    let mut last_rgba8: Option<Vec<u8>> = None;
    for frame in 0..ANIMATION_FRAME_COUNT {
        let target_step = animation_steps[frame];
        while policy_step < target_step {
            let obs = sim.step(policy.next_action());
            policy_step += 1;
            mix_observation_digest(&mut trajectory_digest, &obs);
            planarity.observe(&sim);
            task.observe(policy_step, &obs);
            mix_task_digest(&mut trajectory_digest, &task);
        }

        let obs = sim.observe();
        base_path.push(Vec3::new(obs.base_x_m, 0.0, obs.base_z_m));
        object_path.push(task.object_m());
        let mut scene = build_visual_render_scene(sim.world());
        append_hero_context(&mut scene, &base_path, &object_path, &task);
        let orbit = CameraOrbit {
            focus: Vec3::new(
                obs.base_x_m * 0.62 + REACH_TARGET_M.x * 0.24 + HOUSE_CENTER_M.x * 0.14,
                0.52,
                obs.base_z_m * 0.62 + REACH_TARGET_M.z * 0.24 + HOUSE_CENTER_M.z * 0.14,
            ),
            yaw_rad: hero_camera_yaw_rad(frame),
            pitch_rad: 1.15,
            distance_m: 3.45,
        };
        let output = backend
            .render_scene_camera(&camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
            .expect("render hero frame");

        if frame == 0 || frame == ANIMATION_FRAME_COUNT / 2 {
            let unique = unique_colors(&output.color.rgba8);
            let center =
                (output.depth.height / 2 * output.color.width + output.color.width / 2) as usize;
            let center_depth = output.depth.depth_m[center];
            assert!(
                unique >= MIN_UNIQUE_COLORS && center_depth < camera.far_m as f32,
                "3D hero frame invalid (unique_colors={unique}, center_depth={center_depth:.2} m)"
            );
        }
        render_metrics.observe_animation(&output.color.rgba8);
        last_rgba8 = Some(output.color.rgba8.clone());

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

    while policy_step < POLICY_STEPS {
        let obs = sim.step(policy.next_action());
        policy_step += 1;
        mix_observation_digest(&mut trajectory_digest, &obs);
        planarity.observe(&sim);
        task.observe(policy_step, &obs);
        mix_task_digest(&mut trajectory_digest, &task);
    }

    let hold_rgba8 = last_rgba8.expect("animation frames must produce at least one render");
    for hold in 0..HOLD_FRAME_COUNT {
        render_metrics.observe_hold(&hold_rgba8);
        let frame_index = ANIMATION_FRAME_COUNT + hold;
        let frame_path = frames_dir.join(format!("frame-{frame_index:03}.png"));
        write_png(&frame_path, &hold_rgba8, RENDER_WIDTH, RENDER_HEIGHT)
            .expect("write hold frame png");
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
        task,
    );
    metrics.assert_navigation_and_reach();
    render_metrics.assert_dynamic();
    write_sim_metadata_if_requested(&metrics, &render_metrics)
        .expect("write hero simulation metadata");

    let poster_src = &frame_paths[hero_poster_animation_frame(&animation_steps)];
    let poster_path = media_dir.join("rne-hero.png");
    upscale_png(poster_src, &poster_path, POSTER_WIDTH, POSTER_HEIGHT).expect("upscale poster");

    let gif_path = media_dir.join("rne-hero.gif");
    build_gif(&frames_dir, FRAME_COUNT, &gif_path).expect("build hero gif");
    render_metrics.assert_hold_loop_seam();
    let _ = fs::remove_dir_all(&frames_dir);

    println!(
        "rendered 3D mobile manipulator hero to {} and {} (frames={FRAME_COUNT}, digest=0x{:016x}, base_travel={:.2} m, ee_travel={:.2} m, object_transport={:.2} m, final_ee_target_error={:.3} m, final_object_place_error={:.3} m, base=({:.2}, {:.2}, {:.2}), ee=({:.2}, {:.2}, {:.2}), object=({:.2}, {:.2}, {:.2}))",
        poster_path.display(),
        gif_path.display(),
        metrics.trajectory_digest,
        metrics.base_travel_m,
        metrics.ee_travel_m,
        metrics.object_transport_m,
        metrics.final_ee_target_error_m,
        metrics.final_object_place_error_m,
        metrics.final_base_m[0],
        metrics.final_base_m[1],
        metrics.final_base_m[2],
        metrics.final_ee_m[0],
        metrics.final_ee_m[1],
        metrics.final_ee_m[2],
        metrics.final_object_m[0],
        metrics.final_object_m[1],
        metrics.final_object_m[2]
    );
}

#[derive(Clone, Copy, Debug)]
struct HeroSimMetrics {
    base_travel_m: f64,
    ee_travel_m: f64,
    object_transport_m: f64,
    trajectory_digest: u64,
    max_base_height_error_m: f64,
    min_base_yaw_only_dot: f64,
    final_ee_target_error_m: f64,
    final_object_place_error_m: f64,
    grasped_steps: usize,
    released_after_grasp: bool,
    final_base_m: [f64; 3],
    final_ee_m: [f64; 3],
    final_object_m: [f64; 3],
}

#[derive(Clone, Debug)]
struct HeroRenderMetrics {
    animation_frame_count: usize,
    hold_frame_count: usize,
    first_frame_rgba8: Vec<u8>,
    previous_frame_rgba8: Vec<u8>,
    min_consecutive_frame_delta_ratio: f64,
    first_last_frame_delta_ratio: f64,
    max_hold_frame_delta_ratio: f64,
}

impl HeroRenderMetrics {
    fn new() -> Self {
        Self {
            animation_frame_count: 0,
            hold_frame_count: 0,
            first_frame_rgba8: Vec::new(),
            previous_frame_rgba8: Vec::new(),
            min_consecutive_frame_delta_ratio: 1.0,
            first_last_frame_delta_ratio: 0.0,
            max_hold_frame_delta_ratio: 0.0,
        }
    }

    fn observe_animation(&mut self, rgba8: &[u8]) {
        if self.animation_frame_count == 0 {
            self.first_frame_rgba8 = rgba8.to_vec();
        } else {
            let delta_ratio = frame_delta_ratio(&self.previous_frame_rgba8, rgba8);
            self.min_consecutive_frame_delta_ratio =
                self.min_consecutive_frame_delta_ratio.min(delta_ratio);
            self.first_last_frame_delta_ratio = frame_delta_ratio(&self.first_frame_rgba8, rgba8);
        }
        self.previous_frame_rgba8 = rgba8.to_vec();
        self.animation_frame_count += 1;
    }

    fn observe_hold(&mut self, rgba8: &[u8]) {
        if self.hold_frame_count == 0 && !self.previous_frame_rgba8.is_empty() {
            let delta_ratio = frame_delta_ratio(&self.previous_frame_rgba8, rgba8);
            self.max_hold_frame_delta_ratio = self.max_hold_frame_delta_ratio.max(delta_ratio);
        } else if self.hold_frame_count > 0 {
            let delta_ratio = frame_delta_ratio(&self.previous_frame_rgba8, rgba8);
            self.max_hold_frame_delta_ratio = self.max_hold_frame_delta_ratio.max(delta_ratio);
        }
        self.previous_frame_rgba8 = rgba8.to_vec();
        self.hold_frame_count += 1;
    }

    fn assert_dynamic(&self) {
        assert_eq!(
            self.animation_frame_count, ANIMATION_FRAME_COUNT,
            "expected render metrics for every animation frame"
        );
        assert_eq!(
            self.hold_frame_count, HOLD_FRAME_COUNT,
            "expected render metrics for every hold frame"
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

    fn assert_hold_loop_seam(&self) {
        assert!(
            self.max_hold_frame_delta_ratio <= MAX_HOLD_FRAME_DELTA_RATIO,
            "expected calm hero loop seam: max_hold_frame_delta_ratio={:.4}",
            self.max_hold_frame_delta_ratio
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
        task: HeroTaskProgress,
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
        let final_object_m = task.object_m();
        let object_transport_m = (final_object_m - task.initial_object_m()).length();
        let final_object_place_error_m = (final_object_m - task.drop_zone_m()).length();
        Self {
            base_travel_m,
            ee_travel_m,
            object_transport_m,
            trajectory_digest,
            max_base_height_error_m: planarity.max_base_height_error_m,
            min_base_yaw_only_dot: planarity.min_base_yaw_only_dot,
            final_ee_target_error_m,
            final_object_place_error_m,
            grasped_steps: task.grasped_steps,
            released_after_grasp: task.released_after_grasp,
            final_base_m,
            final_ee_m,
            final_object_m: [final_object_m.x, final_object_m.y, final_object_m.z],
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
        assert!(
            self.grasped_steps >= MIN_GRASPED_STEPS,
            "expected the mobile manipulator to grasp the task object: grasped_steps={}",
            self.grasped_steps
        );
        assert!(
            self.released_after_grasp,
            "expected the task object to be released after being carried"
        );
        assert!(
            self.object_transport_m >= MIN_OBJECT_TRANSPORT_M,
            "expected task object transport: object_transport={:.2} m",
            self.object_transport_m
        );
        assert!(
            self.final_object_place_error_m <= MAX_FINAL_OBJECT_PLACE_ERROR_M,
            "expected task object near drop zone: final_object_place_error={:.3} m",
            self.final_object_place_error_m
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
            self.object_transport_m.to_bits(),
            repeat.object_transport_m.to_bits(),
            "hero simulation object transport changed between identical runs"
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
            self.final_object_place_error_m.to_bits(),
            repeat.final_object_place_error_m.to_bits(),
            "hero simulation final place error changed between identical runs"
        );
        assert_eq!(
            self.grasped_steps, repeat.grasped_steps,
            "hero simulation grasp duration changed between identical runs"
        );
        assert_eq!(
            self.released_after_grasp, repeat.released_after_grasp,
            "hero simulation release state changed between identical runs"
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
        assert_eq!(
            self.final_object_m.map(f64::to_bits),
            repeat.final_object_m.map(f64::to_bits),
            "hero simulation final object pose changed between identical runs"
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
    let mut task = HeroTaskProgress::new();
    mix_task_digest(&mut trajectory_digest, &task);
    let mut policy = MobilePickPlaceHeroPolicy::new();
    for policy_step in 1..=POLICY_STEPS {
        let obs = sim.step(policy.next_action());
        mix_observation_digest(&mut trajectory_digest, &obs);
        planarity.observe(&sim);
        task.observe(policy_step, &obs);
        mix_task_digest(&mut trajectory_digest, &task);
    }

    let final_obs = sim.observe();
    let metrics = HeroSimMetrics::new(
        [start.base_x_m, start.base_y_m, start.base_z_m],
        [start.ee_x_m, start.ee_y_m, start.ee_z_m],
        [final_obs.base_x_m, final_obs.base_y_m, final_obs.base_z_m],
        [final_obs.ee_x_m, final_obs.ee_y_m, final_obs.ee_z_m],
        trajectory_digest,
        planarity,
        task,
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

#[derive(Clone, Copy, Debug)]
struct HeroTaskProgress {
    initial_object_m: Vec3,
    object_m: Vec3,
    drop_zone_m: Vec3,
    release_start_object_m: Vec3,
    grasped_steps: usize,
    was_grasping: bool,
    released_after_grasp: bool,
}

impl HeroTaskProgress {
    fn new() -> Self {
        Self {
            initial_object_m: PICK_OBJECT_M,
            object_m: PICK_OBJECT_M,
            drop_zone_m: PLACE_TARGET_M,
            release_start_object_m: PICK_OBJECT_M,
            grasped_steps: 0,
            was_grasping: false,
            released_after_grasp: false,
        }
    }

    fn observe(&mut self, policy_step: usize, obs: &MobileManipulatorObservation) {
        let is_grasping = (OBJECT_GRASP_STEP..OBJECT_RELEASE_STEP).contains(&policy_step);
        if is_grasping {
            self.grasped_steps += 1;
            let carried = Vec3::new(obs.ee_x_m, obs.ee_y_m, obs.ee_z_m);
            let grasp_blend = ((policy_step - OBJECT_GRASP_STEP) as f64 / 45.0).min(1.0);
            self.object_m = PICK_OBJECT_M.lerp(carried, grasp_blend);
            self.release_start_object_m = self.object_m;
        } else if policy_step >= OBJECT_RELEASE_STEP {
            if self.was_grasping {
                self.released_after_grasp = true;
                self.release_start_object_m = self.object_m;
            }
            let release_blend = ((policy_step - OBJECT_RELEASE_STEP) as f64 / 60.0).min(1.0);
            self.object_m = self
                .release_start_object_m
                .lerp(PLACE_TARGET_M, release_blend);
        } else {
            self.object_m = PICK_OBJECT_M;
        }
        self.was_grasping = is_grasping;
    }

    fn initial_object_m(&self) -> Vec3 {
        self.initial_object_m
    }

    fn object_m(&self) -> Vec3 {
        self.object_m
    }

    fn drop_zone_m(&self) -> Vec3 {
        self.drop_zone_m
    }
}

fn link_translation_m(sim: &MobileManipulatorSim, name: &str) -> Option<Vec3> {
    sim.world().iter_entities().find_map(|entity| {
        let link = entity.get::<Link>()?;
        (link.name == name).then(|| world_transform_of(sim.world(), entity.id()).translation)
    })
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

fn mix_task_digest(digest: &mut u64, task: &HeroTaskProgress) {
    for value in [task.object_m.x, task.object_m.y, task.object_m.z] {
        mix_digest_u64(digest, value.to_bits());
    }
    mix_digest_u64(digest, task.grasped_steps as u64);
    mix_digest_u64(digest, u64::from(task.released_after_grasp));
}

fn mix_digest_u64(digest: &mut u64, value: u64) {
    for byte in value.to_le_bytes() {
        *digest ^= u64::from(byte);
        *digest = digest.wrapping_mul(HERO_DIGEST_PRIME);
    }
}

struct MobilePickPlaceHeroPolicy {
    step: usize,
}

impl MobilePickPlaceHeroPolicy {
    fn new() -> Self {
        Self { step: 0 }
    }

    fn next_action(&mut self) -> MobileManipulatorAction {
        self.step += 1;
        match self.step {
            1..=175 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 5.0,
                right_wheel_velocity_rad_s: 5.0,
                ..MobileManipulatorAction::default()
            },
            176..=300 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 2.0,
                right_wheel_velocity_rad_s: 5.0,
                shoulder_velocity_rad_s: 0.4,
                ..MobileManipulatorAction::default()
            },
            PICK_MANIPULATION_START_STEP..=PICK_MANIPULATION_END_STEP => MobileManipulatorAction {
                shoulder_velocity_rad_s: 0.7,
                elbow_velocity_rad_s: -0.4,
                gripper_velocity_rad_s: -2.5,
                ..MobileManipulatorAction::default()
            },
            391..=600 => MobileManipulatorAction {
                left_wheel_velocity_rad_s: 2.4,
                right_wheel_velocity_rad_s: 4.0,
                shoulder_velocity_rad_s: 0.2,
                elbow_velocity_rad_s: -0.1,
                gripper_velocity_rad_s: -2.0,
                ..MobileManipulatorAction::default()
            },
            RELEASE_MANIPULATION_START_STEP..=RELEASE_MANIPULATION_END_STEP => {
                MobileManipulatorAction {
                    gripper_velocity_rad_s: 3.0,
                    ..MobileManipulatorAction::default()
                }
            }
            _ => MobileManipulatorAction::default(),
        }
    }
}

fn run_hero_trace() {
    let mut sim = MobileManipulatorSim::from_scene_path(&mm_mobile_scene_path())
        .expect("load mm_mobile scene");
    for _ in 0..SETTLE_STEPS {
        sim.step(MobileManipulatorAction::default());
    }

    let mut policy = MobilePickPlaceHeroPolicy::new();
    let mut task = HeroTaskProgress::new();
    for step in 0..=POLICY_STEPS {
        if step > 0 {
            let obs = sim.step(policy.next_action());
            task.observe(step, &obs);
        }
        if step % 20 == 0 || sim.is_grasping() {
            let obs = sim.observe();
            let object = task.object_m();
            let gripper = link_translation_m(&sim, "gripper_base_link").unwrap_or(Vec3::ZERO);
            println!(
                "step={step:03} base=({:.2},{:.2}) ee=({:.2},{:.2},{:.2}) gripper=({:.2},{:.2},{:.2}) object=({:.2},{:.2},{:.2}) carrying={}",
                obs.base_x_m,
                obs.base_z_m,
                obs.ee_x_m,
                obs.ee_y_m,
                obs.ee_z_m,
                gripper.x,
                gripper.y,
                gripper.z,
                object.x,
                object.y,
                object.z,
                (OBJECT_GRASP_STEP..OBJECT_RELEASE_STEP).contains(&step)
            );
        }
    }
}

fn append_hero_context(
    scene: &mut RenderScene,
    base_path: &[Vec3],
    object_path: &[Vec3],
    task: &HeroTaskProgress,
) {
    push_box(
        scene,
        Vec3::new(HOUSE_CENTER_M.x, -0.055, HOUSE_CENTER_M.z),
        Vec3::new(8.0, 0.10, 5.8),
        [0.27, 0.29, 0.29, 1.0],
    );
    for wall in HERO_WALLS {
        push_box(scene, wall.center_m, wall.size_m, wall.color_rgba);
    }
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
    push_box(
        scene,
        Vec3::new(PICK_OBJECT_M.x, PICK_OBJECT_M.y - 0.075, PICK_OBJECT_M.z),
        Vec3::new(0.54, 0.05, 0.42),
        [0.49, 0.36, 0.25, 1.0],
    );
    push_box(
        scene,
        Vec3::new(PLACE_TARGET_M.x, PLACE_TARGET_M.y - 0.075, PLACE_TARGET_M.z),
        Vec3::new(0.58, 0.05, 0.46),
        [0.24, 0.42, 0.36, 1.0],
    );
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(
            Vec3::new(
                task.drop_zone_m().x,
                task.drop_zone_m().y + 0.09,
                task.drop_zone_m().z,
            ),
            Quat::IDENTITY,
        ),
        VisualShape::Sphere { radius_m: 0.10 },
        [0.08, 0.82, 0.54, 1.0],
        Transform3::IDENTITY,
    ));
    scene.items.push(RenderScene::item_from_visual(
        Transform3::from_translation_rotation(task.object_m(), Quat::IDENTITY),
        VisualShape::Box {
            size_m: TASK_OBJECT_SIZE_M,
        },
        [0.94, 0.53, 0.12, 1.0],
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
    for point in object_path.iter().step_by(3) {
        scene.items.push(RenderScene::item_from_visual(
            Transform3::from_translation_rotation(
                Vec3::new(point.x, point.y + 0.055, point.z),
                Quat::IDENTITY,
            ),
            VisualShape::Sphere { radius_m: 0.025 },
            [0.96, 0.60, 0.18, 1.0],
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
            "  \"animation_frame_count\": {},\n",
            "  \"base_travel_m\": {:.6},\n",
            "  \"ee_travel_m\": {:.6},\n",
            "  \"object_transport_m\": {:.6},\n",
            "  \"trajectory_digest\": \"0x{:016x}\",\n",
            "  \"max_base_height_error_m\": {:.6},\n",
            "  \"min_base_yaw_only_dot\": {:.9},\n",
            "  \"final_ee_target_error_m\": {:.6},\n",
            "  \"final_object_place_error_m\": {:.6},\n",
            "  \"grasped_steps\": {},\n",
            "  \"released_after_grasp\": {},\n",
            "  \"final_base_m\": [{:.6}, {:.6}, {:.6}],\n",
            "  \"final_ee_m\": [{:.6}, {:.6}, {:.6}],\n",
            "  \"final_object_m\": [{:.6}, {:.6}, {:.6}],\n",
            "  \"hold_frame_count\": {},\n",
            "  \"max_hold_frame_delta_ratio\": {:.6},\n",
            "  \"max_hold_frame_delta_ratio_threshold\": {:.6},\n",
            "  \"min_base_travel_m\": {:.6},\n",
            "  \"min_ee_travel_m\": {:.6},\n",
            "  \"min_object_transport_m\": {:.6},\n",
            "  \"max_final_ee_target_error_m\": {:.6},\n",
            "  \"max_final_object_place_error_m\": {:.6},\n",
            "  \"min_consecutive_frame_delta_ratio\": {:.6},\n",
            "  \"first_last_frame_delta_ratio\": {:.6},\n",
            "  \"min_consecutive_frame_delta_ratio_threshold\": {:.6},\n",
            "  \"min_first_last_frame_delta_ratio_threshold\": {:.6},\n",
            "  \"poster_policy_step\": {}\n",
            "}}\n"
        ),
        ANIMATION_FRAME_COUNT,
        metrics.base_travel_m,
        metrics.ee_travel_m,
        metrics.object_transport_m,
        metrics.trajectory_digest,
        metrics.max_base_height_error_m,
        metrics.min_base_yaw_only_dot,
        metrics.final_ee_target_error_m,
        metrics.final_object_place_error_m,
        metrics.grasped_steps,
        metrics.released_after_grasp,
        metrics.final_base_m[0],
        metrics.final_base_m[1],
        metrics.final_base_m[2],
        metrics.final_ee_m[0],
        metrics.final_ee_m[1],
        metrics.final_ee_m[2],
        metrics.final_object_m[0],
        metrics.final_object_m[1],
        metrics.final_object_m[2],
        HOLD_FRAME_COUNT,
        render_metrics.max_hold_frame_delta_ratio,
        MAX_HOLD_FRAME_DELTA_RATIO,
        MIN_BASE_TRAVEL_M,
        MIN_EE_TRAVEL_M,
        MIN_OBJECT_TRANSPORT_M,
        MAX_FINAL_EE_TARGET_ERROR_M,
        MAX_FINAL_OBJECT_PLACE_ERROR_M,
        render_metrics.min_consecutive_frame_delta_ratio,
        render_metrics.first_last_frame_delta_ratio,
        MIN_CONSECUTIVE_FRAME_DELTA_RATIO,
        MIN_FIRST_LAST_FRAME_DELTA_RATIO,
        POSTER_POLICY_STEP
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
        "fps={FPS},scale={POSTER_WIDTH}:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors={GIF_MAX_COLORS}[p];[s1][p]paletteuse=dither=bayer:bayer_scale={GIF_BAYER_SCALE}:diff_mode=rectangle"
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
    if !status.success() {
        return Err(std::io::Error::other("ffmpeg gif encode failed"));
    }
    let byte_size = fs::metadata(gif_path)?.len();
    if byte_size > HERO_ENCODE_MAX_BYTE_SIZE {
        return Err(std::io::Error::other(format!(
            "hero gif exceeds size budget: {byte_size} bytes > {HERO_ENCODE_MAX_BYTE_SIZE} bytes"
        )));
    }
    Ok(())
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

#[cfg(test)]
mod tests {
    use super::*;

    const HERO_BASE_WALL_CLEARANCE_M: f64 = 0.30;

    fn observation_at(point: Vec3) -> MobileManipulatorObservation {
        MobileManipulatorObservation {
            ee_x_m: point.x,
            ee_y_m: point.y,
            ee_z_m: point.z,
            ..MobileManipulatorObservation::default()
        }
    }

    #[test]
    fn hero_task_object_starts_at_pick_surface() {
        let task = HeroTaskProgress::new();

        assert_eq!(task.object_m(), PICK_OBJECT_M);
        assert_eq!(task.initial_object_m(), PICK_OBJECT_M);
        assert_eq!(task.drop_zone_m(), PLACE_TARGET_M);
        assert_eq!(task.grasped_steps, 0);
        assert!(!task.released_after_grasp);
    }

    #[test]
    fn hero_task_object_carries_then_places() {
        let mut task = HeroTaskProgress::new();
        let carry_point = Vec3::new(2.6, 0.42, -2.2);

        task.observe(OBJECT_GRASP_STEP - 1, &observation_at(carry_point));
        assert_eq!(task.object_m(), PICK_OBJECT_M);

        task.observe(OBJECT_GRASP_STEP + 45, &observation_at(carry_point));
        assert_eq!(task.object_m(), carry_point);
        assert!(task.grasped_steps > 0);

        task.observe(OBJECT_RELEASE_STEP, &observation_at(carry_point));
        assert!(task.released_after_grasp);

        task.observe(OBJECT_RELEASE_STEP + 60, &observation_at(carry_point));
        assert_eq!(task.object_m(), PLACE_TARGET_M);
    }

    #[test]
    fn hero_policy_stops_base_during_manipulation() {
        let mut policy = MobilePickPlaceHeroPolicy::new();

        for step in 1..=POLICY_STEPS {
            let action = policy.next_action();
            if (PICK_MANIPULATION_START_STEP..=PICK_MANIPULATION_END_STEP).contains(&step)
                || (RELEASE_MANIPULATION_START_STEP..=RELEASE_MANIPULATION_END_STEP).contains(&step)
            {
                assert_eq!(
                    action.left_wheel_velocity_rad_s, 0.0,
                    "left wheel should be stopped during manipulation at step {step}"
                );
                assert_eq!(
                    action.right_wheel_velocity_rad_s, 0.0,
                    "right wheel should be stopped during manipulation at step {step}"
                );
            }
        }
    }

    #[test]
    fn hero_media_samples_wheel_motion_at_higher_temporal_resolution() {
        assert_eq!(ANIMATION_FRAME_COUNT, 100);
        assert_eq!(HOLD_FRAME_COUNT, 10);
        assert_eq!(FRAME_COUNT, 110);
        assert_eq!(FPS, 15);
    }

    #[test]
    fn hero_animation_policy_steps_are_monotonic_and_cover_policy() {
        let steps = hero_animation_policy_steps();
        assert_eq!(steps.len(), ANIMATION_FRAME_COUNT);
        assert_eq!(steps[0], 0);
        assert_eq!(steps[ANIMATION_FRAME_COUNT - 1], POLICY_STEPS);
        for window in steps.windows(2) {
            assert!(
                window[1] >= window[0],
                "hero sampling must be monotonic: {} -> {}",
                window[0],
                window[1]
            );
        }
    }

    #[test]
    fn hero_poster_frame_targets_grasp_moment() {
        let steps = hero_animation_policy_steps();
        let poster_frame = hero_poster_animation_frame(&steps);
        assert!(
            (steps[poster_frame] as i64 - POSTER_POLICY_STEP as i64).unsigned_abs() <= 20,
            "poster frame should land near grasp: step={} target={POSTER_POLICY_STEP}",
            steps[poster_frame]
        );
    }

    #[test]
    fn hero_mobile_base_path_clears_visual_walls() {
        let mut sim = MobileManipulatorSim::from_scene_path(&mm_mobile_scene_path())
            .expect("load mm_mobile scene");
        for _ in 0..SETTLE_STEPS {
            sim.step(MobileManipulatorAction::default());
        }

        let mut policy = MobilePickPlaceHeroPolicy::new();
        for step in 1..=POLICY_STEPS {
            let obs = sim.step(policy.next_action());
            for wall in HERO_WALLS {
                let clearance_m = top_down_box_clearance_m(obs.base_x_m, obs.base_z_m, wall);
                assert!(
                    clearance_m >= HERO_BASE_WALL_CLEARANCE_M,
                    "base path is too close to a visual wall at step {step}: clearance={clearance_m:.3} m, wall={wall:?}"
                );
            }
        }
    }

    fn top_down_box_clearance_m(x_m: f64, z_m: f64, wall: HeroBox) -> f64 {
        let dx_m = (x_m - wall.center_m.x).abs() - wall.size_m.x * 0.5;
        let dz_m = (z_m - wall.center_m.z).abs() - wall.size_m.z * 0.5;
        let outside_x_m = dx_m.max(0.0);
        let outside_z_m = dz_m.max(0.0);
        (outside_x_m * outside_x_m + outside_z_m * outside_z_m).sqrt()
    }
}

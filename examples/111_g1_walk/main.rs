//! Headless forward-walk regression and hero GIF for the official Unitree G1.
//!
//! The default and `--smoke` paths drive the validated commanded gait with a
//! pure forward velocity and assert the measured boundary: no fall, an upright
//! body, bounded torque, ground covered, and exact replay. `--gif` runs the same
//! contact-gated hybrid plant frame by frame and writes
//! `docs/media/unitree-g1-walk.gif` (honoured only when `RNE_SKIP_GPU` is unset).
//!
//! Run with `cargo run -p g1_walk --example 111_g1_walk`.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, run_unitree_g1_commanded_gait, unitree_g1_dynamic_scene_path,
    unitree_g1_gait_targets_for_velocity, UnitreeG1CommandedGaitConfig,
    UnitreeG1CommandedTorquePolicy, UnitreeG1GaitCommand, UnitreeG1VelocityCommand,
    UnitreeG1VelocityPolicyInput, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const PANEL_WIDTH: u32 = 960;
const PANEL_HEIGHT: u32 = 540;
const FRAME_COUNT: usize = 120;
const STEPS_PER_FRAME: u64 = 12;
const PREROLL_STEPS: u64 = 480;
const SETTLE_STEPS: u64 = 240;
const KP: f64 = 300.0;
const KD: f64 = 10.0;
const TORQUE_LIMIT_NM: f64 = 88.0;
const SPEED_LIMIT_RAD_S: f64 = 30.0;
const WALK_STRIDE_RAD: f64 = 0.065;
const WALK_FOOT_LIFT_RAD: f64 = 0.12;
const WALK_CYCLE_STEPS: u64 = 100;
const FORWARD_M_S: f64 = 0.0276;
const CLEAR_COLOR: [f32; 4] = [0.03, 0.045, 0.07, 1.0];

const TORQUE_LINKS: [&str; 8] = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
];

fn main() {
    if std::env::args().any(|argument| argument == "--smoke") {
        run_api_gate(true);
        return;
    }
    if !std::env::args().any(|argument| argument == "--gif") {
        run_api_gate(false);
        return;
    }
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    run_gif();
}

fn run_api_gate(smoke: bool) {
    let (settle_steps, rollout_steps) = if smoke { (60, 480) } else { (240, 1440) };

    let config = UnitreeG1CommandedGaitConfig {
        settle_steps,
        rollout_steps,
        command: UnitreeG1VelocityCommand {
            forward_m_s: FORWARD_M_S,
            yaw_rate_rad_s: 0.0,
        },
        ..UnitreeG1CommandedGaitConfig::default()
    };

    let first = run_unitree_g1_commanded_gait(config.clone()).expect("G1 forward walk");
    let second = run_unitree_g1_commanded_gait(config).expect("G1 forward walk replay");
    assert_eq!(
        first, second,
        "G1 forward walk must replay deterministically"
    );

    let min_displacement_m = if smoke { 0.05 } else { 0.2 };
    assert!(!first.fell, "G1 forward walk fell: {first:?}");
    assert!(
        first.min_height_m > 0.75,
        "G1 forward walk dropped too low: {first:?}"
    );
    assert!(
        first.max_tilt_rad < 0.30,
        "G1 forward walk tilted too far: {first:?}"
    );
    assert!(
        first.max_command_nm <= TORQUE_LIMIT_NM,
        "G1 forward walk exceeded the torque limit: {first:?}"
    );
    assert!(
        first.total_displacement_m > min_displacement_m,
        "G1 forward walk must cover ground: {first:?}"
    );

    println!(
        "g1 walk passed: steps={rollout_steps} displacement={:.3} m minH={:.3} m tilt={:.3} rad",
        first.total_displacement_m, first.min_height_m, first.max_tilt_rad,
    );
}

fn walk_command() -> UnitreeG1GaitCommand {
    UnitreeG1GaitCommand {
        stride_rad: WALK_STRIDE_RAD,
        foot_lift_rad: WALK_FOOT_LIFT_RAD,
        cycle_steps: WALK_CYCLE_STEPS,
    }
}

struct G1Walker {
    sim: UrdfSceneSim,
    policy: UnitreeG1CommandedTorquePolicy,
    command: UnitreeG1VelocityCommand,
    step: u64,
    start_x_m: f64,
    min_height_m: f64,
}

impl G1Walker {
    fn new() -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
            .expect("load dynamic G1");
        sim.configure_position_motors(220.0, 24.0, TORQUE_LIMIT_NM);
        let stand = unitree_g1_gait_targets_for_velocity(
            0,
            walk_command(),
            UnitreeG1VelocityCommand::default(),
        );
        for _ in 0..SETTLE_STEPS {
            sim.step_joint_position_targets(&stand);
        }
        let observed = sim.observe();
        Self {
            sim,
            policy: UnitreeG1CommandedTorquePolicy::validated_heading(),
            command: UnitreeG1VelocityCommand {
                forward_m_s: FORWARD_M_S,
                yaw_rate_rad_s: 0.0,
            },
            step: 0,
            start_x_m: observed.base_x_m,
            min_height_m: observed.base_y_m,
        }
    }

    fn step_frame(&mut self, steps: u64) {
        for _ in 0..steps {
            let targets =
                unitree_g1_gait_targets_for_velocity(self.step, walk_command(), self.command);
            let servo: Vec<UrdfJointPositionTarget<'_>> = targets
                .iter()
                .filter(|target| !TORQUE_LINKS.contains(&target.link_name))
                .copied()
                .collect();
            self.sim.set_joint_position_targets(&servo);
            let stance = [
                self.sim.link_contact_impulse_ns("left_ankle_roll_link") > 0.0,
                self.sim.link_contact_impulse_ns("right_ankle_roll_link") > 0.0,
            ];
            let observation = self.sim.observe();
            let world_velocity = Vec3::new(
                observation.base_linear_velocity_x_m_s,
                observation.base_linear_velocity_y_m_s,
                observation.base_linear_velocity_z_m_s,
            );
            let body_rotation = self
                .sim
                .named_transform("pelvis")
                .expect("G1 pelvis pose")
                .rotation;
            let input = UnitreeG1VelocityPolicyInput {
                two_cycle_phase: (self.step % (2 * WALK_CYCLE_STEPS)) as f64
                    / (2 * WALK_CYCLE_STEPS) as f64,
                stance,
                command: self.command,
                measured_forward_velocity_m_s: (body_rotation.inverse() * world_velocity).z,
                measured_yaw_rate_rad_s: observation.base_angular_velocity_y_rad_s,
                target_heading_rad: 0.0,
                measured_heading_rad: 0.0,
                heading_error_rad: 0.0,
                yaw_rate_error_rad_s: self.command.yaw_rate_rad_s
                    - observation.base_angular_velocity_y_rad_s,
            };
            let feed_forward = self.policy.torques_nm_for_command(input, TORQUE_LIMIT_NM);
            let torques: Vec<UrdfJointTorqueTarget<'_>> = TORQUE_LINKS
                .iter()
                .enumerate()
                .map(|(index, link_name)| {
                    let target_position = targets
                        .iter()
                        .find(|target| target.link_name == *link_name)
                        .expect("torque link in gait targets")
                        .position;
                    let q = self
                        .sim
                        .named_joint_position(link_name)
                        .expect("joint position");
                    let qd = self
                        .sim
                        .named_joint_velocity(link_name)
                        .expect("joint velocity");
                    UrdfJointTorqueTarget {
                        link_name,
                        torque_nm: (KP * (target_position - q) - KD * qd + feed_forward[index])
                            .clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM),
                        max_velocity_rad_s: SPEED_LIMIT_RAD_S,
                    }
                })
                .collect();
            self.sim.step_joint_torques(&torques);
            let observed = self.sim.observe();
            assert!(
                observed.base_y_m.is_finite(),
                "G1 walker became non-finite at step {}",
                self.step
            );
            self.min_height_m = self.min_height_m.min(observed.base_y_m);
            self.step += 1;
        }
    }
}

fn run_gif() {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("unitree-g1-walk-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create G1 walk frame directory");

    let mut walker = G1Walker::new();
    walker.step_frame(PREROLL_STEPS);

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = walker.sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        walker.step_frame(STEPS_PER_FRAME);
        let rgba = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            &walker,
        );
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &rgba,
            PANEL_WIDTH,
            PANEL_HEIGHT,
        )
        .expect("write G1 walk frame");
    }

    let observed = walker.sim.observe();
    let forward_m = observed.base_x_m - walker.start_x_m;
    assert!(observed.base_y_m > 0.75, "G1 walk fell while capturing");
    assert!(
        walker.min_height_m > 0.75,
        "G1 walk dropped too low while capturing: {:.3} m",
        walker.min_height_m
    );
    assert!(
        forward_m.abs() > 0.10,
        "G1 walk must cover ground: {forward_m:.3} m"
    );

    let gif_path = media_dir.join("unitree-g1-walk.gif");
    build_gif(&frames_dir, &gif_path).expect("encode G1 walk gif");
    image::open(frames_dir.join(format!("frame-{:03}.png", FRAME_COUNT - 1)))
        .expect("read G1 walk poster frame")
        .save(media_dir.join("unitree-g1-walk.png"))
        .expect("write G1 walk poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered G1 walk media to {} (forward {:.3} m, minH {:.3} m)",
        gif_path.display(),
        forward_m,
        walker.min_height_m,
    );
}

fn render_panel(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    mesh_root_refs: &[&Path],
    walker: &G1Walker,
) -> Vec<u8> {
    let observed = walker.sim.observe();
    let mut scene = build_visual_render_scene(walker.sim.world());
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
    append_checker_floor(&mut scene, observed.base_x_m, observed.base_z_m, 0.12);
    mesh_cache
        .resolve_scene(&mut scene, mesh_root_refs)
        .expect("resolve official G1 meshes");
    let orbit = CameraOrbit {
        focus: Vec3::new(
            observed.base_x_m,
            observed.base_y_m * 0.62,
            observed.base_z_m,
        ),
        yaw_rad: -0.9,
        pitch_rad: 1.38,
        distance_m: 3.0,
    };
    let output = backend
        .render_scene_camera(camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render G1 walk panel");
    output.color.rgba8
}

fn append_checker_floor(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64, tile_m: f64) {
    let snap = |value: f64| (value / (2.0 * tile_m)).floor() * 2.0 * tile_m;
    for row in -8..=8 {
        for column in -8..=8 {
            let color = if (row + column) & 1 == 0 {
                [0.11, 0.15, 0.21, 1.0]
            } else {
                [0.055, 0.075, 0.11, 1.0]
            };
            scene.items.push(RenderSceneItem {
                transform: Transform3 {
                    translation: Vec3::new(
                        snap(center_x_m) + column as f64 * tile_m,
                        -0.008,
                        snap(center_z_m) + row as f64 * tile_m,
                    ),
                    rotation: Quat::IDENTITY,
                    scale: Vec3::new(tile_m * 0.96, 0.008, tile_m * 0.96),
                },
                shape: VisualShape::Box { size_m: Vec3::ONE },
                color_rgba: color,
                mesh: None,
                base_color_texture: None,
                material: Default::default(),
            });
        }
    }
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            "12",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=12,scale=720:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg G1 walk gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}

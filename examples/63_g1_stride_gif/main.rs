//! Renders the commanded G1 locomotion milestone as a side-by-side hero GIF.
//!
//! The blue panel is the plain hybrid stepper and the orange panel is the
//! pinned [`UnitreeG1TorqueOverlay::LEARNED_STRIDE`] under the command
//! boundary's positive differential-steering request. Both panels settle,
//! discard the same 8 s transient, and then render the measured windows.
//! `--smoke` exercises that physics path without initializing a renderer.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_dynamic_scene_path, unitree_g1_gait_targets_for_velocity,
    UnitreeG1CommandedTorquePolicy, UnitreeG1GaitCommand, UnitreeG1TorqueOverlay,
    UnitreeG1VelocityCommand, UnitreeG1VelocityPolicyInput, UrdfJointPositionTarget,
    UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const PANEL_WIDTH: u32 = 480;
const PANEL_HEIGHT: u32 = 520;
const BANNER_HEIGHT: u32 = 10;
const FRAME_COUNT: usize = 80;
const STEPS_PER_FRAME: u64 = 12;
const PREROLL_STEPS: u64 = 480;
const WINDOW_STEPS: u64 = 480;
const CAPTURE_STEPS: u64 = FRAME_COUNT as u64 * STEPS_PER_FRAME;
const SETTLE_STEPS: u64 = 240;
const KP: f64 = 300.0;
const KD: f64 = 10.0;
const TORQUE_LIMIT_NM: f64 = 88.0;
const SPEED_LIMIT_RAD_S: f64 = 30.0;
const WALK_STRIDE_RAD: f64 = 0.065;
const WALK_FOOT_LIFT_RAD: f64 = 0.12;
const WALK_CYCLE_STEPS: u64 = 100;
const TRAIL_EVERY_STEPS: u64 = 36;
const CLEAR_COLOR: [f32; 4] = [0.025, 0.035, 0.05, 1.0];
const COMMAND_MIN_WINDOW_M: f64 = 0.12;

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
    capture_step: u64,
    capture_start_xz_m: [f64; 2],
    window_start_xz_m: [f64; 2],
    window_a_m: f64,
    window_b_m: f64,
    min_height_m: f64,
    trail_m: Vec<[f64; 2]>,
    capture_active: bool,
}

impl G1Walker {
    fn new(overlay: UnitreeG1TorqueOverlay, command: UnitreeG1VelocityCommand) -> Self {
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
            policy: UnitreeG1CommandedTorquePolicy {
                overlay,
                forward_velocity_feedback_gain: 0.0,
                ..UnitreeG1CommandedTorquePolicy::default()
            },
            command,
            step: 0,
            capture_step: 0,
            capture_start_xz_m: [observed.base_x_m, observed.base_z_m],
            window_start_xz_m: [observed.base_x_m, observed.base_z_m],
            window_a_m: 0.0,
            window_b_m: 0.0,
            min_height_m: observed.base_y_m,
            trail_m: Vec::new(),
            capture_active: false,
        }
    }

    fn begin_capture(&mut self) {
        let observed = self.sim.observe();
        self.capture_step = 0;
        self.capture_start_xz_m = [observed.base_x_m, observed.base_z_m];
        self.window_start_xz_m = self.capture_start_xz_m;
        self.window_a_m = 0.0;
        self.window_b_m = 0.0;
        self.min_height_m = observed.base_y_m;
        self.trail_m.clear();
        self.capture_active = true;
        self.record_trail();
    }

    fn step_frame(&mut self, steps: u64) {
        for _ in 0..steps {
            if self.capture_active && self.capture_step.is_multiple_of(TRAIL_EVERY_STEPS) {
                self.record_trail();
            }
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
            self.step += 1;
            if self.capture_active {
                self.capture_step += 1;
                self.min_height_m = self.min_height_m.min(observed.base_y_m);
                let position = [observed.base_x_m, observed.base_z_m];
                if self.capture_step == WINDOW_STEPS {
                    self.window_a_m = distance_m(self.window_start_xz_m, position);
                    self.window_start_xz_m = position;
                } else if self.capture_step == CAPTURE_STEPS {
                    self.window_b_m = distance_m(self.window_start_xz_m, position);
                }
            }
        }
        if self.capture_active {
            self.record_trail();
        }
    }

    fn record_trail(&mut self) {
        let observed = self.sim.observe();
        let position = [observed.base_x_m, observed.base_z_m];
        let should_push = self
            .trail_m
            .last()
            .is_none_or(|last| distance_m(*last, position) > 0.005);
        if should_push {
            self.trail_m.push(position);
        }
    }

    fn minimum_window_m(&self) -> f64 {
        self.window_a_m.min(self.window_b_m)
    }
}

fn distance_m(a: [f64; 2], b: [f64; 2]) -> f64 {
    (b[0] - a[0]).hypot(b[1] - a[1])
}

fn main() {
    if std::env::args().any(|argument| argument == "--smoke") {
        run_smoke();
        return;
    }
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }

    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("unitree-g1-learned-stride-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create learned G1 frame directory");

    let forward = UnitreeG1VelocityCommand {
        forward_m_s: 0.0276,
        yaw_rate_rad_s: 0.0,
    };
    let steering = UnitreeG1VelocityCommand {
        forward_m_s: 0.0276,
        yaw_rate_rad_s: 0.05,
    };
    let mut baseline = G1Walker::new(UnitreeG1TorqueOverlay::ZERO, forward);
    let mut learned = G1Walker::new(UnitreeG1TorqueOverlay::LEARNED_STRIDE, steering);
    baseline.step_frame(PREROLL_STEPS);
    learned.step_frame(PREROLL_STEPS);
    baseline.begin_capture();
    learned.begin_capture();

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = baseline.sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        baseline.step_frame(STEPS_PER_FRAME);
        learned.step_frame(STEPS_PER_FRAME);
        let left = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            &baseline,
            [0.16, 0.58, 0.96, 1.0],
        );
        let right = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            &learned,
            [0.98, 0.54, 0.16, 1.0],
        );
        let composite = composite_side_by_side(&left, &right);
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &composite,
            PANEL_WIDTH * 2,
            PANEL_HEIGHT + BANNER_HEIGHT,
        )
        .expect("write learned G1 frame");
    }

    let baseline_min = baseline.minimum_window_m();
    let learned_min = learned.minimum_window_m();
    assert!(
        learned_min > 2.0 * baseline_min && learned_min > COMMAND_MIN_WINDOW_M,
        "commanded G1 must clear the path bar: {learned_min:.3} m vs stepper {baseline_min:.3} m"
    );
    assert!(
        learned.min_height_m > 0.7,
        "learned G1 should stay upright, min height {:.3} m",
        learned.min_height_m
    );

    let gif_path = media_dir.join("unitree-g1-learned-stride.gif");
    build_gif(&frames_dir, &gif_path).expect("encode learned G1 gif");
    image::open(frames_dir.join(format!("frame-{:03}.png", FRAME_COUNT - 1)))
        .expect("read learned G1 poster frame")
        .save(media_dir.join("unitree-g1-learned-stride.png"))
        .expect("write learned G1 poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered learned G1 media to {} (stepper {:.3}/{:.3} m, learned {:.3}/{:.3} m, minH {:.3} m)",
        gif_path.display(),
        baseline.window_a_m,
        baseline.window_b_m,
        learned.window_a_m,
        learned.window_b_m,
        learned.min_height_m,
    );
}

fn run_smoke() {
    let mut learned = G1Walker::new(
        UnitreeG1TorqueOverlay::LEARNED_STRIDE,
        UnitreeG1VelocityCommand {
            forward_m_s: 0.0276,
            yaw_rate_rad_s: 0.05,
        },
    );
    learned.step_frame(PREROLL_STEPS);
    learned.begin_capture();
    learned.step_frame(96);
    let observed = learned.sim.observe();
    assert!(
        observed.base_y_m > 0.7,
        "learned G1 smoke fell: {:.3} m",
        observed.base_y_m
    );
    println!(
        "learned G1 stride smoke passed at height {:.3} m",
        observed.base_y_m
    );
}

fn render_panel(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    mesh_root_refs: &[&Path],
    walker: &G1Walker,
    trail_color: [f32; 4],
) -> Vec<u8> {
    let observed = walker.sim.observe();
    let mut scene = build_visual_render_scene(walker.sim.world());
    scene.items.retain(|item| {
        !matches!(item.shape, VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z > 5.0)
    });
    append_realistic_test_bay(
        &mut scene,
        walker.capture_start_xz_m[0],
        walker.capture_start_xz_m[1],
    );
    for position in &walker.trail_m {
        scene.items.push(RenderSceneItem {
            transform: Transform3 {
                translation: Vec3::new(position[0], 0.012, position[1]),
                rotation: rne_math::Quat::IDENTITY,
                scale: Vec3::new(0.035, 0.006, 0.035),
            },
            shape: VisualShape::Box { size_m: Vec3::ONE },
            color_rgba: trail_color,
            mesh: None,
            base_color_texture: None,
        });
    }
    mesh_cache
        .resolve_scene(&mut scene, mesh_root_refs)
        .expect("resolve official G1 meshes");
    let orbit = CameraOrbit {
        focus: Vec3::new(
            walker.capture_start_xz_m[0] + 0.20,
            observed.base_y_m + 0.05,
            walker.capture_start_xz_m[1],
        ),
        yaw_rad: -0.72,
        pitch_rad: 1.16,
        distance_m: 2.60,
    };
    let output = backend
        .render_scene_camera(camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render learned G1 panel");
    output.color.rgba8
}

fn append_realistic_test_bay(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64) {
    // The G1 physics scene intentionally stays minimal. These render-only props make the
    // hero capture read as a real robotics test bay without changing contacts or dynamics.
    const FLOOR: [f32; 4] = [0.19, 0.21, 0.22, 1.0];
    const FLOOR_SEAM: [f32; 4] = [0.075, 0.09, 0.10, 1.0];
    const SAFETY_YELLOW: [f32; 4] = [0.72, 0.46, 0.08, 1.0];
    const WALL: [f32; 4] = [0.14, 0.17, 0.20, 1.0];
    const WALL_PANEL: [f32; 4] = [0.09, 0.13, 0.17, 1.0];
    const METAL: [f32; 4] = [0.30, 0.34, 0.36, 1.0];
    const WINDOW: [f32; 4] = [0.035, 0.10, 0.14, 1.0];
    const LIGHT: [f32; 4] = [0.82, 0.86, 0.78, 1.0];
    const STATUS: [f32; 4] = [0.10, 0.74, 0.48, 1.0];

    let floor_center = Vec3::new(center_x_m + 0.25, -0.035, center_z_m - 0.35);
    push_box(scene, floor_center, Vec3::new(5.4, 0.07, 4.6), FLOOR);

    for x_offset in [-1.8, -0.9, 0.0, 0.9, 1.8] {
        push_box(
            scene,
            Vec3::new(center_x_m + x_offset, 0.004, center_z_m - 0.35),
            Vec3::new(0.012, 0.006, 4.35),
            FLOOR_SEAM,
        );
    }
    for z_offset in [-1.4, -0.45, 0.5, 1.45] {
        push_box(
            scene,
            Vec3::new(center_x_m + 0.25, 0.004, center_z_m + z_offset),
            Vec3::new(5.25, 0.006, 0.012),
            FLOOR_SEAM,
        );
    }

    // A pair of inset safety lines gives the walking lane a scale cue and keeps the
    // measured trails readable against the matte floor.
    for z_offset in [-0.82, 0.82] {
        push_box(
            scene,
            Vec3::new(center_x_m + 0.25, 0.009, center_z_m + z_offset),
            Vec3::new(4.8, 0.008, 0.035),
            SAFETY_YELLOW,
        );
    }

    // Corner walls: the camera looks into the corner, so the clear color is only a
    // narrow upper margin rather than an empty flat backdrop.
    push_box(
        scene,
        Vec3::new(center_x_m + 2.45, 1.35, center_z_m - 0.35),
        Vec3::new(0.10, 2.7, 4.7),
        WALL,
    );
    push_box(
        scene,
        Vec3::new(center_x_m + 0.25, 1.35, center_z_m - 2.25),
        Vec3::new(4.5, 2.7, 0.10),
        WALL,
    );

    // Recessed blue wall panels and a few narrow metal mullions suggest a real
    // calibration room while remaining cheap primitive geometry.
    for z_offset in [-1.55, -0.55, 0.45, 1.45] {
        push_box(
            scene,
            Vec3::new(center_x_m + 2.385, 1.42, center_z_m + z_offset),
            Vec3::new(0.018, 2.15, 0.84),
            WALL_PANEL,
        );
        push_box(
            scene,
            Vec3::new(center_x_m + 2.32, 1.42, center_z_m + z_offset - 0.5),
            Vec3::new(0.025, 2.18, 0.018),
            METAL,
        );
    }
    for z_offset in [-1.05, -0.05, 0.95] {
        push_box(
            scene,
            Vec3::new(center_x_m + 2.31, 1.55, center_z_m + z_offset),
            Vec3::new(0.026, 1.18, 0.68),
            WINDOW,
        );
        push_box(
            scene,
            Vec3::new(center_x_m + 2.285, 1.55, center_z_m + z_offset - 0.36),
            Vec3::new(0.032, 0.025, 0.72),
            METAL,
        );
        push_box(
            scene,
            Vec3::new(center_x_m + 2.285, 1.55, center_z_m + z_offset + 0.36),
            Vec3::new(0.032, 0.025, 0.72),
            METAL,
        );
        push_box(
            scene,
            Vec3::new(center_x_m + 2.275, 1.55, center_z_m + z_offset),
            Vec3::new(0.035, 1.18, 0.025),
            METAL,
        );
    }

    // A low service rail and a few status strips add depth behind the robot without
    // competing with the white G1 body.
    push_box(
        scene,
        Vec3::new(center_x_m + 1.95, 0.42, center_z_m - 2.17),
        Vec3::new(0.08, 0.78, 0.08),
        METAL,
    );
    push_box(
        scene,
        Vec3::new(center_x_m + 1.95, 0.78, center_z_m - 2.17),
        Vec3::new(0.08, 0.06, 0.08),
        STATUS,
    );
    push_box(
        scene,
        Vec3::new(center_x_m + 2.38, 0.22, center_z_m - 1.92),
        Vec3::new(0.04, 0.16, 0.52),
        SAFETY_YELLOW,
    );

    // Suspended LED panels catch the existing directional shadows and make the
    // ceiling area feel intentional without adding a second render pass.
    for z_offset in [-1.2, 0.0, 1.2] {
        push_box(
            scene,
            Vec3::new(center_x_m + 0.20, 2.55, center_z_m + z_offset),
            Vec3::new(0.72, 0.035, 0.18),
            LIGHT,
        );
        push_box(
            scene,
            Vec3::new(center_x_m + 0.20, 2.49, center_z_m + z_offset),
            Vec3::new(0.06, 0.12, 0.06),
            METAL,
        );
    }
}

fn push_box(scene: &mut RenderScene, translation: Vec3, size_m: Vec3, color_rgba: [f32; 4]) {
    scene.items.push(RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation: rne_math::Quat::IDENTITY,
            scale: size_m,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba,
        mesh: None,
        base_color_texture: None,
    });
}

fn composite_side_by_side(left: &[u8], right: &[u8]) -> Vec<u8> {
    let width = (PANEL_WIDTH * 2) as usize;
    let height = (PANEL_HEIGHT + BANNER_HEIGHT) as usize;
    let mut composite = vec![0_u8; width * height * 4];
    for y in 0..BANNER_HEIGHT as usize {
        for x in 0..width {
            let offset = (y * width + x) * 4;
            let color = if x < PANEL_WIDTH as usize {
                [66, 140, 220, 255]
            } else {
                [226, 138, 52, 255]
            };
            composite[offset..offset + 4].copy_from_slice(&color);
        }
    }
    for y in 0..PANEL_HEIGHT as usize {
        let row = y + BANNER_HEIGHT as usize;
        let panel_row = y * PANEL_WIDTH as usize * 4;
        let panel_bytes = PANEL_WIDTH as usize * 4;
        let left_offset = row * width * 4;
        composite[left_offset..left_offset + panel_bytes]
            .copy_from_slice(&left[panel_row..panel_row + panel_bytes]);
        let right_offset = left_offset + panel_bytes;
        composite[right_offset..right_offset + panel_bytes]
            .copy_from_slice(&right[panel_row..panel_row + panel_bytes]);
    }
    composite
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
            "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=192[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg learned G1 gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}

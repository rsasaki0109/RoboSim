//! Renders the learned G1 stride as a side-by-side hero GIF.
//!
//! The blue panel is the plain hybrid stepper and the orange panel is the
//! pinned [`UnitreeG1TorqueOverlay::LEARNED_STRIDE`] at the speed-up command
//! used by the headless regression test. Both panels settle, discard the same
//! 8 s transient, and then render the two measured 8 s windows. `--smoke`
//! exercises that physics path without initializing a renderer.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_g1_dynamic_scene_path, unitree_g1_gait_targets,
    UnitreeG1GaitCommand, UnitreeG1TorqueOverlay, UrdfJointPositionTarget, UrdfJointTorqueTarget,
    UrdfSceneSim,
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
const WALK_STRIDE_RAD: f64 = 0.07;
const WALK_FOOT_LIFT_RAD: f64 = 0.10;
const WALK_CYCLE_STEPS: u64 = 100;
const TRAIL_EVERY_STEPS: u64 = 36;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
const SPEEDUP_MIN_WINDOW_M: f64 = 0.20;

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
    overlay: UnitreeG1TorqueOverlay,
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
    fn new(overlay: UnitreeG1TorqueOverlay) -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_dynamic_scene_path())
            .expect("load dynamic G1");
        sim.configure_position_motors(220.0, 24.0, TORQUE_LIMIT_NM);
        let stand = unitree_g1_gait_targets(
            0,
            UnitreeG1GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                cycle_steps: WALK_CYCLE_STEPS,
            },
        );
        for _ in 0..SETTLE_STEPS {
            sim.step_joint_position_targets(&stand);
        }
        let observed = sim.observe();
        Self {
            sim,
            overlay,
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
            let targets = unitree_g1_gait_targets(self.step, walk_command());
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
            let two_cycle_steps = 2 * WALK_CYCLE_STEPS;
            let two_cycle_phase = (self.step % two_cycle_steps) as f64 / two_cycle_steps as f64;
            let feed_forward = self.overlay.torques_nm(two_cycle_phase, stance);
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

    let mut baseline = G1Walker::new(UnitreeG1TorqueOverlay::ZERO);
    let mut learned = G1Walker::new(UnitreeG1TorqueOverlay::LEARNED_STRIDE);
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
        learned_min > 2.0 * baseline_min && learned_min > SPEEDUP_MIN_WINDOW_M,
        "learned G1 must clear the speed-up bar: {learned_min:.3} m vs stepper {baseline_min:.3} m"
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
    let mut learned = G1Walker::new(UnitreeG1TorqueOverlay::LEARNED_STRIDE);
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
    append_checker_floor(
        &mut scene,
        walker.capture_start_xz_m[0],
        walker.capture_start_xz_m[1],
        0.12,
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

fn append_checker_floor(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64, tile_m: f64) {
    let snap = |value: f64| (value / (2.0 * tile_m)).floor() * 2.0 * tile_m;
    for row in -9..=9 {
        for column in -9..=9 {
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
                    rotation: rne_math::Quat::IDENTITY,
                    scale: Vec3::new(tile_m * 0.96, 0.008, tile_m * 0.96),
                },
                shape: VisualShape::Box { size_m: Vec3::ONE },
                color_rgba: color,
                mesh: None,
                base_color_texture: None,
            });
        }
    }
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

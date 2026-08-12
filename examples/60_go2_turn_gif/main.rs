//! Renders the Go2 torque-steering comparison to README media.
//!
//! Both panels run the identical low-bandwidth torque-PD walk (kp 40,
//! kd 0.5) that the 60 Hz discrete stability bound allows. The left robot
//! walks with zero feed-forward and goes straight; the right robot adds the
//! chaos-robust contact-gated torque overlay
//! (`UnitreeGo2TorqueOverlay::LEARNED_ROBUST_TURN`) and carves a visible arc —
//! the turn that no joint-space controller could produce (see
//! `docs/GO2_LOCOMOTION.md`). The same numbers are pinned by
//! `robust_torque_turn_survives_perturbation`.

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_dynamic_scene_path, unitree_go2_trot_targets,
    UnitreeGo2GaitCommand, UnitreeGo2TorqueOverlay, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::{Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const PANEL_WIDTH: u32 = 480;
const PANEL_HEIGHT: u32 = 532;
const BANNER_HEIGHT: u32 = 8;
const FRAME_COUNT: usize = 80;
const STEPS_PER_FRAME: u64 = 12;
/// The overlay winds up (negative) through its first eight seconds, exactly
/// as the pinned tests measure; the GIF walks that transient off camera and
/// records the sustained-turn windows (steps 480..1440).
const PREROLL_STEPS: u64 = 480;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
const SETTLE_STEPS: u64 = 240;
const KP: f64 = 40.0;
const KD: f64 = 0.5;
const TORQUE_LIMIT_NM: f64 = 23.7;
const SPEED_LIMIT_RAD_S: f64 = 30.1;
/// One trail marker per gait cycle keeps the path readable without clutter.
const TRAIL_EVERY_STEPS: u64 = 45;

fn walk_command() -> UnitreeGo2GaitCommand {
    UnitreeGo2GaitCommand {
        stride_rad: 0.24,
        cycle_steps: 45,
        ..UnitreeGo2GaitCommand::default()
    }
}

/// One torque-PD walker with an optional contact-gated torque overlay — the
/// exact protocol of the pinned turn tests.
struct TorqueWalker {
    sim: UrdfSceneSim,
    overlay: UnitreeGo2TorqueOverlay,
    step: u64,
    previous_yaw: f64,
    total_yaw_rad: f64,
    trail_m: Vec<[f64; 2]>,
    capture_start_xz_m: [f64; 2],
}

impl TorqueWalker {
    fn new(overlay: UnitreeGo2TorqueOverlay) -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path())
            .expect("load dynamic Go2");
        sim.configure_position_motors(180.0, 18.0, TORQUE_LIMIT_NM);
        let stand = unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                ..walk_command()
            },
        );
        for _ in 0..SETTLE_STEPS {
            sim.step_joint_position_targets(&stand);
        }
        let previous_yaw = sim.observe().base_relative_yaw_rad;
        Self {
            sim,
            overlay,
            step: 0,
            previous_yaw,
            total_yaw_rad: 0.0,
            trail_m: Vec::new(),
            capture_start_xz_m: [0.0; 2],
        }
    }

    fn begin_capture(&mut self) {
        let observed = self.sim.observe();
        self.capture_start_xz_m = [observed.base_x_m, observed.base_z_m];
        self.total_yaw_rad = 0.0;
        self.previous_yaw = observed.base_relative_yaw_rad;
        self.trail_m.clear();
    }

    fn net_displacement_m(&self) -> f64 {
        let observed = self.sim.observe();
        (observed.base_x_m - self.capture_start_xz_m[0])
            .hypot(observed.base_z_m - self.capture_start_xz_m[1])
    }

    fn step_frame(&mut self, steps: u64) {
        let cycle = walk_command().cycle_steps;
        for _ in 0..steps {
            if self.step.is_multiple_of(TRAIL_EVERY_STEPS) {
                let observed = self.sim.observe();
                self.trail_m.push([observed.base_x_m, observed.base_z_m]);
            }
            let targets = unitree_go2_trot_targets(self.step, walk_command());
            let stance = [
                self.sim.link_contact_impulse_ns("FL_foot") > 0.0,
                self.sim.link_contact_impulse_ns("FR_foot") > 0.0,
                self.sim.link_contact_impulse_ns("RL_foot") > 0.0,
                self.sim.link_contact_impulse_ns("RR_foot") > 0.0,
            ];
            let two_cycle_phase = (self.step % (2 * cycle)) as f64 / (2 * cycle) as f64;
            let feed_forward = self.overlay.torques_nm(two_cycle_phase, stance);
            let torques: Vec<UrdfJointTorqueTarget<'_>> = targets
                .iter()
                .zip(feed_forward.iter())
                .map(|(target, extra)| {
                    let q = self
                        .sim
                        .named_joint_position(target.link_name)
                        .expect("joint position");
                    let qd = self
                        .sim
                        .named_joint_velocity(target.link_name)
                        .expect("joint velocity");
                    UrdfJointTorqueTarget {
                        link_name: target.link_name,
                        torque_nm: (KP * (target.position - q) - KD * qd + extra)
                            .clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM),
                        max_velocity_rad_s: SPEED_LIMIT_RAD_S,
                    }
                })
                .collect();
            self.sim.step_joint_torques(&torques);
            let observed = self.sim.observe();
            let mut delta = observed.base_relative_yaw_rad - self.previous_yaw;
            while delta > std::f64::consts::PI {
                delta -= 2.0 * std::f64::consts::PI;
            }
            while delta < -std::f64::consts::PI {
                delta += 2.0 * std::f64::consts::PI;
            }
            self.total_yaw_rad += delta;
            self.previous_yaw = observed.base_relative_yaw_rad;
            self.step += 1;
        }
    }
}

fn main() {
    if std::env::args().any(|arg| arg == "--smoke") {
        let metrics = run_headless_capture();
        println!(
            "Go2 torque-turn smoke ok: straight_yaw={:+.3} rad, turn_yaw={:+.3} rad, turn_displacement={:.2} m, straight_height={:.3} m, turn_height={:.3} m",
            metrics.straight_yaw_rad,
            metrics.turn_yaw_rad,
            metrics.turn_displacement_m,
            metrics.straight_height_m,
            metrics.turn_height_m,
        );
        return;
    }
    if std::env::var("RNE_SKIP_GPU").is_ok() {
        return;
    }
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let media_dir = repo_root.join("docs/media");
    let frames_dir = media_dir.join("go2-torque-turn-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create torque-turn frame directory");

    let mut straight = TorqueWalker::new(UnitreeGo2TorqueOverlay::ZERO);
    let mut turning = TorqueWalker::new(UnitreeGo2TorqueOverlay::LEARNED_ROBUST_TURN);
    straight.step_frame(PREROLL_STEPS);
    turning.step_frame(PREROLL_STEPS);
    straight.begin_capture();
    turning.begin_capture();

    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(PANEL_WIDTH, PANEL_HEIGHT, std::f64::consts::FRAC_PI_4);
    let mesh_roots: Vec<PathBuf> = straight.sim.mesh_package_roots().to_vec();
    let mesh_root_refs: Vec<&Path> = mesh_roots.iter().map(PathBuf::as_path).collect();
    let mut mesh_cache = MeshRenderCache::new();

    for frame in 0..FRAME_COUNT {
        straight.step_frame(STEPS_PER_FRAME);
        turning.step_frame(STEPS_PER_FRAME);
        let left = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            &straight,
            [0.30, 0.62, 0.95, 1.0],
        );
        let right = render_panel(
            &mut backend,
            &camera,
            &mut mesh_cache,
            &mesh_root_refs,
            &turning,
            [0.95, 0.55, 0.20, 1.0],
        );
        if frame == 0 {
            let unique = left
                .chunks_exact(4)
                .collect::<std::collections::HashSet<_>>()
                .len();
            assert!(unique > 2, "torque-turn frame should contain geometry");
        }
        let composite = composite_side_by_side(&left, &right);
        write_png(
            &frames_dir.join(format!("frame-{frame:03}.png")),
            &composite,
            PANEL_WIDTH * 2,
            PANEL_HEIGHT + BANNER_HEIGHT,
        )
        .expect("write torque-turn frame");
    }

    // The GIF must show the measured physics: the plain torque walk holds its
    // heading while the overlay carves a genuinely sustained arc.
    let metrics = TurnCaptureMetrics::from_walkers(&straight, &turning);
    metrics.assert_showcase_ready();

    let gif_path = media_dir.join("go2-torque-turn.gif");
    build_gif(&frames_dir, &gif_path).expect("encode torque-turn gif");
    let poster = image::open(frames_dir.join(format!("frame-{:03}.png", FRAME_COUNT - 1)))
        .expect("read poster frame");
    poster
        .save(media_dir.join("go2-torque-turn.png"))
        .expect("write torque-turn poster");
    let _ = fs::remove_dir_all(&frames_dir);
    println!(
        "rendered torque-turn media to {} (straight {:+.3} rad, turn {:+.3} rad, turn displacement {:.2} m)",
        gif_path.display(),
        metrics.straight_yaw_rad,
        metrics.turn_yaw_rad,
        metrics.turn_displacement_m,
    );
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct TurnCaptureMetrics {
    straight_yaw_rad: f64,
    turn_yaw_rad: f64,
    turn_displacement_m: f64,
    straight_height_m: f64,
    turn_height_m: f64,
}

impl TurnCaptureMetrics {
    fn from_walkers(straight: &TorqueWalker, turning: &TorqueWalker) -> Self {
        Self {
            straight_yaw_rad: straight.total_yaw_rad,
            turn_yaw_rad: turning.total_yaw_rad,
            turn_displacement_m: turning.net_displacement_m(),
            straight_height_m: straight.sim.observe().base_y_m,
            turn_height_m: turning.sim.observe().base_y_m,
        }
    }

    fn assert_showcase_ready(self) {
        assert!(
            self.straight_yaw_rad.abs() < 0.25,
            "plain torque walk should hold heading, drifted {:+.3} rad",
            self.straight_yaw_rad,
        );
        assert!(
            self.turn_yaw_rad > 0.45,
            "robust overlay should carve an arc, got {:+.3} rad",
            self.turn_yaw_rad,
        );
        assert!(
            self.straight_height_m > 0.12 && self.turn_height_m > 0.12,
            "both walkers should stay up (straight {:.3} m, turn {:.3} m)",
            self.straight_height_m,
            self.turn_height_m,
        );
        assert!(
            self.turn_displacement_m > 1.0,
            "turning torque walk must preserve transport, moved only {:.3} m",
            self.turn_displacement_m,
        );
    }
}

fn run_headless_capture() -> TurnCaptureMetrics {
    let mut straight = TorqueWalker::new(UnitreeGo2TorqueOverlay::ZERO);
    let mut turning = TorqueWalker::new(UnitreeGo2TorqueOverlay::LEARNED_ROBUST_TURN);
    straight.step_frame(PREROLL_STEPS);
    turning.step_frame(PREROLL_STEPS);
    straight.begin_capture();
    turning.begin_capture();
    straight.step_frame(FRAME_COUNT as u64 * STEPS_PER_FRAME);
    turning.step_frame(FRAME_COUNT as u64 * STEPS_PER_FRAME);
    let metrics = TurnCaptureMetrics::from_walkers(&straight, &turning);
    metrics.assert_showcase_ready();
    metrics
}

fn render_panel(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    mesh_root_refs: &[&Path],
    walker: &TorqueWalker,
    trail_color: [f32; 4],
) -> Vec<u8> {
    let observed = walker.sim.observe();
    let mut scene = build_visual_render_scene(walker.sim.world());
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
    append_checker_floor(&mut scene, observed.base_x_m, observed.base_z_m, 0.18);
    for position in &walker.trail_m {
        scene.items.push(RenderSceneItem {
            transform: Transform3 {
                translation: Vec3::new(position[0], 0.010, position[1]),
                rotation: rne_math::Quat::IDENTITY,
                scale: Vec3::new(0.05, 0.006, 0.05),
            },
            shape: VisualShape::Box { size_m: Vec3::ONE },
            color_rgba: trail_color,
            mesh: None,
            base_color_texture: None,
            material: Default::default(),
        });
    }
    mesh_cache
        .resolve_scene(&mut scene, mesh_root_refs)
        .expect("resolve official Go2 meshes");
    // Elevated follow-cam with a world-fixed yaw: the robot's heading change
    // and the trail's curvature both read against the checker grid.
    let orbit = CameraOrbit {
        focus: Vec3::new(observed.base_x_m, 0.10, observed.base_z_m),
        yaw_rad: -1.6,
        pitch_rad: 1.08,
        distance_m: 2.15,
    };
    let output = backend
        .render_scene_camera(camera, &orbit.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render torque-turn panel");
    output.color.rgba8
}

/// Joins the two panels with a colored banner: blue over the straight walk,
/// orange over the turning overlay.
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
        let left_offset = (row * width) * 4;
        composite[left_offset..left_offset + panel_bytes]
            .copy_from_slice(&left[panel_row..panel_row + panel_bytes]);
        let right_offset = left_offset + panel_bytes;
        composite[right_offset..right_offset + panel_bytes]
            .copy_from_slice(&right[panel_row..panel_row + panel_bytes]);
    }
    composite
}

fn append_checker_floor(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64, tile_m: f64) {
    let snap = |value: f64| (value / (2.0 * tile_m)).floor() * 2.0 * tile_m;
    for row in -10..=10 {
        for column in -10..=10 {
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
            "fps=12,scale=960:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg torque-turn gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}

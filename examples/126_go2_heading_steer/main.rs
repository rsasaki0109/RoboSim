//! Walks the official Go2 head first along a commanded heading schedule with a
//! model-based trot, on the model that actually has feet.
//!
//! The walking asset the older Go2 examples use leaves the feet and head as
//! loose bodies on the floor (issue #346). This runs `unitree_go2_jump`, the
//! same URDF with its fixed children welded and its declared masses (16.1 kg).
//! On it, neither the library's scripted trot nor contact-gated thigh torque
//! does the job: the scripted trot walks tail first (its stance sweep lowers
//! the thigh angle, and on the Go2 a higher thigh angle puts the foot further
//! back), and the thigh-torque channel turns it by at most 0.3 rad.
//!
//! So this is a controller of the kind Pinocchio-based quadruped stacks run,
//! at 500 Hz on joint torques:
//!
//! * **Kinematics.** Each leg's joint origins and axes come from the simulated
//!   link frames; the foot's linear Jacobian column for a revolute joint is
//!   `axis x (foot - joint)`, as Pinocchio computes it.
//! * **Stance.** Each planted foot takes its share of the weight plus a
//!   virtual spring-damper on its own hip height -- which levels pitch and
//!   roll leg by leg -- and horizontal force toward the commanded velocity and
//!   yaw rate; joints apply `tau = -J^T f`.
//! * **Swing.** The foot lands where the Raibert heuristic says: under the
//!   hip, plus half a stance of the body velocity, plus a velocity-error
//!   correction and a yaw term; it gets there along a lifted arc by Cartesian
//!   PD, `tau = J^T (Kp e - Kd v)`.
//! * **Steering.** A heading loop turns the heading error into the yaw-rate
//!   command.
//!
//! Torques are clamped to the URDF effort limits (23.7 N·m hip and thigh,
//! 45.43 N·m calf). Heading comes from `base_relative_yaw_rad`, unwrapped.
//!
//! ```text
//! cargo run --release -p go2_heading_steer --example 126_go2_heading_steer -- --smoke
//! cargo run --release -p go2_heading_steer --example 126_go2_heading_steer
//! ```

use std::fs;
use std::path::{Path, PathBuf};

use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    build_visual_render_scene, unitree_go2_trot_targets, UnitreeGo2GaitCommand,
    UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_core::SimDuration;
use rne_math::{Hertz, Quat, Transform3, Vec3};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};

const CONTROL_HZ: f64 = 500.0;
const GRAVITY_M_S2: f64 = 9.81;
/// Declared URDF mass of the model, in kilograms.
const MASS_KG: f64 = 16.1;
/// Effort limits from the URDF: hip, thigh, calf.
const EFFORT_NM: [f64; 3] = [23.7, 23.7, 45.43];
const LEGS: [[&str; 4]; 4] = [
    ["FL_hip", "FL_thigh", "FL_calf", "FL_foot"],
    ["FR_hip", "FR_thigh", "FR_calf", "FR_foot"],
    ["RL_hip", "RL_thigh", "RL_calf", "RL_foot"],
    ["RR_hip", "RR_thigh", "RR_calf", "RR_foot"],
];
/// Trot phase offsets: diagonal pairs FL/RR and FR/RL half a cycle apart.
const PHASE_OFFSETS: [f64; 4] = [0.0, 0.5, 0.5, 0.0];

const GAIT_PERIOD_S: f64 = 0.4;
const DUTY: f64 = 0.5;
const STANCE_HEIGHT_M: f64 = 0.30;
const STEP_HEIGHT_M: f64 = 0.06;
/// Height of a planted foot's centre above the floor, in meters.
const FOOT_GROUND_Y_M: f64 = 0.022;
const FORWARD_SPEED_M_S: f64 = 0.25;
/// Stance-leg gains: hip-height spring and damper, velocity and yaw-rate
/// force gains.
const HEIGHT_STIFFNESS_N_PER_M: f64 = 600.0;
const HEIGHT_DAMPING_N_S_PER_M: f64 = 40.0;
const VELOCITY_GAIN_N_S_PER_M: f64 = 300.0;
const YAW_RATE_GAIN_N_S: f64 = 30.0;
/// Swing-leg Cartesian PD.
const SWING_STIFFNESS_N_PER_M: f64 = 400.0;
const SWING_DAMPING_N_S_PER_M: f64 = 12.0;
/// Raibert velocity-error correction, in seconds.
const RAIBERT_GAIN_S: f64 = 0.1;
/// Heading loop: yaw-rate command per rad of heading error, and its limit.
const HEADING_GAIN_PER_S: f64 = 1.5;
const YAW_RATE_LIMIT_RAD_S: f64 = 0.5;

const DURATION_S: f64 = 46.0;
const SETTLING_S: f64 = 4.0;

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
const FRAME_COUNT: usize = 138;
/// Trail marker spacing, in control steps (0.25 s).
const TRAIL_EVERY_STEPS: u64 = 125;

/// Segment start times in seconds and the heading each commands: straight,
/// left 90 degrees, straight, right 90 degrees, straight.
const SCHEDULE: [(f64, f64); 5] = [
    (0.0, 0.0),
    (8.0, std::f64::consts::FRAC_PI_2),
    (18.0, 0.0),
    (28.0, -std::f64::consts::FRAC_PI_2),
    (38.0, 0.0),
];

fn segment_at(t_s: f64) -> (f64, f64) {
    SCHEDULE
        .iter()
        .rev()
        .find(|(start, _)| t_s >= *start)
        .copied()
        .unwrap_or((0.0, 0.0))
}

fn scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/unitree_go2_jump.rne.scene.toml")
}

/// One leg's kinematic state, read from the simulated link frames the way
/// Pinocchio's forward kinematics computes it.
struct Leg {
    joints: [&'static str; 3],
    origins: [Vec3; 3],
    axes: [Vec3; 3],
    foot: Vec3,
    joint_rates: [f64; 3],
}

impl Leg {
    fn read(sim: &UrdfSceneSim, names: [&'static str; 4]) -> Self {
        // URDF joint axes: hip abduction about x, thigh and calf about y.
        let local_axes = [Vec3::X, Vec3::Y, Vec3::Y];
        let mut origins = [Vec3::ZERO; 3];
        let mut axes = [Vec3::ZERO; 3];
        let mut joint_rates = [0.0; 3];
        for i in 0..3 {
            let pose = sim.named_transform(names[i]).expect("leg link pose");
            origins[i] = pose.translation;
            axes[i] = (pose.rotation * local_axes[i]).normalize();
            joint_rates[i] = sim.named_joint_velocity(names[i]).expect("joint rate");
        }
        let foot = sim
            .named_transform(names[3])
            .expect("foot pose")
            .translation;
        Self {
            joints: [names[0], names[1], names[2]],
            origins,
            axes,
            foot,
            joint_rates,
        }
    }

    /// Linear Jacobian columns of the foot point.
    fn jacobian(&self) -> [Vec3; 3] {
        std::array::from_fn(|i| self.axes[i].cross(self.foot - self.origins[i]))
    }

    /// Foot velocity relative to the base, `J qdot`.
    fn foot_velocity_rel(&self) -> Vec3 {
        let jacobian = self.jacobian();
        (0..3).fold(Vec3::ZERO, |sum, i| sum + jacobian[i] * self.joint_rates[i])
    }

    /// Joint torques that make the foot exert force `f`: `J^T f`, clamped.
    fn torques_for(&self, f: Vec3) -> [f64; 3] {
        let jacobian = self.jacobian();
        std::array::from_fn(|i| jacobian[i].dot(f).clamp(-EFFORT_NM[i], EFFORT_NM[i]))
    }
}

struct Walker {
    sim: UrdfSceneSim,
    step: u64,
    liftoff: [Vec3; 4],
    in_stance: [bool; 4],
    previous_yaw_rad: f64,
    heading_rad: f64,
    along_facing_m: f64,
    trail: Vec<([f64; 2], f64)>,
    held_sq: f64,
    held_count: u32,
    worst_held_rad: f64,
    min_height_m: f64,
    segment_end_heading_rad: Vec<f64>,
}

impl Walker {
    fn new() -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path()).expect("load welded Go2");
        // Stand up on the position servos at the scene's own 60 Hz, then hand
        // over to torque control at the controller's rate.
        sim.configure_position_motors(180.0, 18.0, EFFORT_NM[0]);
        let stand = unitree_go2_trot_targets(
            0,
            UnitreeGo2GaitCommand {
                stride_rad: 0.0,
                foot_lift_rad: 0.0,
                ..UnitreeGo2GaitCommand::default()
            },
        );
        for _ in 0..240 {
            sim.step_joint_position_targets(&stand);
        }
        sim.set_fixed_delta(SimDuration::from_hertz(Hertz::new(CONTROL_HZ)));
        let previous_yaw_rad = sim.observe().base_relative_yaw_rad;
        Self {
            sim,
            step: 0,
            liftoff: [Vec3::ZERO; 4],
            in_stance: [true; 4],
            previous_yaw_rad,
            heading_rad: 0.0,
            along_facing_m: 0.0,
            trail: Vec::new(),
            held_sq: 0.0,
            held_count: 0,
            worst_held_rad: 0.0,
            min_height_m: f64::MAX,
            segment_end_heading_rad: Vec::new(),
        }
    }

    fn t_s(&self) -> f64 {
        self.step as f64 / CONTROL_HZ
    }

    fn step_once(&mut self) {
        let t_s = self.t_s();
        let (segment_start_s, target_rad) = segment_at(t_s);
        let yaw_rate_cmd = (HEADING_GAIN_PER_S * (target_rad - self.heading_rad))
            .clamp(-YAW_RATE_LIMIT_RAD_S, YAW_RATE_LIMIT_RAD_S);

        let observed = self.sim.observe();
        let base = self.sim.named_transform("base").expect("base pose");
        let facing = base.rotation * Vec3::X;
        let flat_facing = Vec3::new(facing.x, 0.0, facing.z).normalize_or_zero();
        let body_velocity = Vec3::new(
            observed.base_linear_velocity_x_m_s,
            observed.base_linear_velocity_y_m_s,
            observed.base_linear_velocity_z_m_s,
        );
        let flat_velocity = Vec3::new(body_velocity.x, 0.0, body_velocity.z);
        let body_center = Vec3::new(observed.base_x_m, 0.0, observed.base_z_m);
        let yaw_rate = observed.base_angular_velocity_y_rad_s;
        let velocity_cmd = flat_facing * FORWARD_SPEED_M_S;
        if self.step.is_multiple_of(TRAIL_EVERY_STEPS) {
            self.trail
                .push(([observed.base_x_m, observed.base_z_m], target_rad));
        }

        let phases: [f64; 4] =
            std::array::from_fn(|i| (t_s / GAIT_PERIOD_S + PHASE_OFFSETS[i]) % 1.0);
        let stance_count = phases.iter().filter(|p| **p < DUTY).count().max(1) as f64;
        let stance_s = GAIT_PERIOD_S * DUTY;
        let mut torques: Vec<UrdfJointTorqueTarget<'_>> = Vec::with_capacity(12);
        for (index, names) in LEGS.iter().enumerate() {
            let leg = Leg::read(&self.sim, *names);
            let hip = leg.origins[1];
            let force = if phases[index] < DUTY {
                self.in_stance[index] = true;
                // Ground reaction wanted at this foot.
                let hip_height = hip.y - leg.foot.y;
                let hip_rise_rate = -leg.foot_velocity_rel().y;
                let lift = MASS_KG * GRAVITY_M_S2 / stance_count
                    + HEIGHT_STIFFNESS_N_PER_M * (STANCE_HEIGHT_M - hip_height)
                    - HEIGHT_DAMPING_N_S_PER_M * hip_rise_rate;
                let velocity_error = velocity_cmd - flat_velocity;
                let lever = Vec3::new(leg.foot.x, 0.0, leg.foot.z) - body_center;
                let tangent = Vec3::Y.cross(lever).normalize_or_zero();
                let push = velocity_error * (VELOCITY_GAIN_N_S_PER_M / stance_count)
                    + tangent * (YAW_RATE_GAIN_N_S * (yaw_rate_cmd - yaw_rate));
                // The foot pushes the ground the other way: tau = -J^T f.
                -Vec3::new(push.x, lift.max(0.0), push.z)
            } else {
                if self.in_stance[index] {
                    self.liftoff[index] = leg.foot;
                    self.in_stance[index] = false;
                }
                let swing = ((phases[index] - DUTY) / (1.0 - DUTY)).clamp(0.0, 1.0);
                let hip_ground = Vec3::new(hip.x, 0.0, hip.z);
                let from_center = hip_ground - body_center;
                let touchdown = hip_ground
                    + flat_velocity * (stance_s * 0.5)
                    + (flat_velocity - velocity_cmd) * RAIBERT_GAIN_S
                    + Vec3::Y.cross(from_center) * (yaw_rate_cmd * stance_s * 0.5);
                let blend = 0.5 - 0.5 * (std::f64::consts::PI * swing).cos();
                let liftoff = self.liftoff[index];
                let target = Vec3::new(
                    liftoff.x + (touchdown.x - liftoff.x) * blend,
                    FOOT_GROUND_Y_M + STEP_HEIGHT_M * (std::f64::consts::PI * swing).sin(),
                    liftoff.z + (touchdown.z - liftoff.z) * blend,
                );
                let foot_velocity = leg.foot_velocity_rel() + body_velocity;
                (target - leg.foot) * SWING_STIFFNESS_N_PER_M
                    - foot_velocity * SWING_DAMPING_N_S_PER_M
            };
            let tau = leg.torques_for(force);
            for (joint, torque) in leg.joints.iter().zip(tau) {
                torques.push(UrdfJointTorqueTarget {
                    link_name: joint,
                    torque_nm: torque,
                    max_velocity_rad_s: 30.1,
                });
            }
        }
        self.sim.step_joint_torques(&torques);

        let after = self.sim.observe();
        let mut delta = after.base_relative_yaw_rad - self.previous_yaw_rad;
        while delta > std::f64::consts::PI {
            delta -= std::f64::consts::TAU;
        }
        while delta < -std::f64::consts::PI {
            delta += std::f64::consts::TAU;
        }
        self.heading_rad += delta;
        self.previous_yaw_rad = after.base_relative_yaw_rad;
        self.along_facing_m += (after.base_linear_velocity_x_m_s * flat_facing.x
            + after.base_linear_velocity_z_m_s * flat_facing.z)
            / CONTROL_HZ;
        if t_s - segment_start_s >= SETTLING_S {
            let held_error = target_rad - self.heading_rad;
            self.held_sq += held_error * held_error;
            self.held_count += 1;
            self.worst_held_rad = self.worst_held_rad.max(held_error.abs());
        }
        self.min_height_m = self.min_height_m.min(after.base_y_m);
        self.step += 1;
        let next_t_s = self.t_s();
        if SCHEDULE
            .iter()
            .skip(1)
            .any(|(start, _)| t_s < *start && next_t_s >= *start)
            || (t_s < DURATION_S && next_t_s >= DURATION_S)
        {
            self.segment_end_heading_rad.push(self.heading_rad);
        }
    }

    fn held_rms_rad(&self) -> f64 {
        (self.held_sq / f64::from(self.held_count.max(1))).sqrt()
    }
}

fn run(mut on_step: impl FnMut(&Walker)) -> Walker {
    let mut walker = Walker::new();
    while walker.t_s() < DURATION_S {
        walker.step_once();
        on_step(&walker);
    }
    walker
}

fn report_and_gate(walker: &Walker) {
    println!(
        "heading held to {:.3} rad RMS (worst {:.3}); walked {:+.2} m along its facing; lowest body {:.3} m",
        walker.held_rms_rad(),
        walker.worst_held_rad,
        walker.along_facing_m,
        walker.min_height_m
    );
    let ends: Vec<String> = walker
        .segment_end_heading_rad
        .iter()
        .zip(SCHEDULE.iter())
        .map(|(heading, (_, target))| format!("{target:+.2}->{heading:+.2}"))
        .collect();
    println!("segment ends (commanded->reached): {}", ends.join(", "));
    assert!(
        walker.min_height_m > 0.2,
        "the walk sagged or fell: lowest body {:.3} m",
        walker.min_height_m
    );
    assert!(
        walker.along_facing_m > 5.0,
        "the robot must walk head first, got {:+.2} m along its facing",
        walker.along_facing_m
    );
    assert!(
        walker.held_rms_rad() < 0.2 && walker.worst_held_rad < 0.5,
        "held headings too loose: {:.3} RMS, worst {:.3}",
        walker.held_rms_rad(),
        walker.worst_held_rad
    );
    assert_eq!(walker.segment_end_heading_rad.len(), SCHEDULE.len());
    for (heading, (_, target)) in walker.segment_end_heading_rad.iter().zip(SCHEDULE.iter()) {
        assert!(
            (heading - target).abs() < 0.25,
            "segment commanding {target:+.2} ended at {heading:+.2}"
        );
    }
}

fn main() {
    if std::env::args().any(|arg| arg == "--smoke") {
        let walker = run(|_| {});
        report_and_gate(&walker);
        println!("smoke ok: the welded Go2 follows the heading schedule head first");
        return;
    }

    // A headless pass first, for the path's extent: one fixed camera over the
    // whole S reads better than a chase view and compresses far better.
    let dry = run(|_| {});
    let (mut min, mut max) = ([f64::MAX; 2], [f64::MIN; 2]);
    for (position, _) in &dry.trail {
        for axis in 0..2 {
            min[axis] = min[axis].min(position[axis]);
            max[axis] = max[axis].max(position[axis]);
        }
    }
    let center = [(min[0] + max[0]) * 0.5, (min[1] + max[1]) * 0.5];
    let span_m = (max[0] - min[0]).max(max[1] - min[1]);
    let view = CameraOrbit {
        focus: Vec3::new(center[0], 0.1, center[1]),
        yaw_rad: -2.2,
        pitch_rad: 0.8,
        distance_m: span_m * 0.8 + 0.9,
    };

    let total_steps = (DURATION_S * CONTROL_HZ) as u64;
    let every = total_steps / FRAME_COUNT as u64;
    let mut backend = WgpuRenderBackend::new().expect("initialize wgpu");
    let camera = Camera::new(WIDTH, HEIGHT, std::f64::consts::FRAC_PI_4);
    let mut mesh_cache = MeshRenderCache::new();
    let frames_dir =
        PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/rne-go2-heading-frames");
    let _ = fs::remove_dir_all(&frames_dir);
    fs::create_dir_all(&frames_dir).expect("create frame directory");
    let mut frame = 0_usize;
    let walker = run(|walker| {
        if !walker.step.is_multiple_of(every) || frame >= FRAME_COUNT {
            return;
        }
        let rgba = render_frame(
            &mut backend,
            &camera,
            &mut mesh_cache,
            walker,
            &view,
            center,
        );
        write_png(&frames_dir.join(format!("frame-{frame:03}.png")), &rgba).expect("write frame");
        frame += 1;
    });
    report_and_gate(&walker);

    let media_dir = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../docs/media");
    let gif_path = media_dir.join("go2-heading-steer.gif");
    build_gif(&frames_dir, &gif_path).expect("encode gif");
    image::open(frames_dir.join(format!("frame-{:03}.png", frame * 2 / 3)))
        .expect("read poster frame")
        .save(media_dir.join("go2-heading-steer.png"))
        .expect("write poster");
    println!("wrote {}", gif_path.display());
}

fn floor_box(translation: Vec3, rotation: Quat, scale: Vec3, color: [f32; 4]) -> RenderSceneItem {
    RenderSceneItem {
        transform: Transform3 {
            translation,
            rotation,
            scale,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material: Default::default(),
    }
}

/// Trail colour by the heading its segment commanded.
fn segment_color(target_rad: f64) -> [f32; 4] {
    if target_rad > 0.1 {
        [0.95, 0.55, 0.20, 1.0]
    } else if target_rad < -0.1 {
        [0.30, 0.62, 0.95, 1.0]
    } else {
        [0.45, 0.85, 0.50, 1.0]
    }
}

fn render_frame(
    backend: &mut WgpuRenderBackend,
    camera: &Camera,
    mesh_cache: &mut MeshRenderCache,
    walker: &Walker,
    view: &CameraOrbit,
    center: [f64; 2],
) -> Vec<u8> {
    let observed = walker.sim.observe();
    let mut scene = build_visual_render_scene(walker.sim.world());
    // Drop the ground plane; the checker floor below replaces it.
    scene
        .items
        .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
    append_checker_floor(&mut scene, center[0], center[1], 0.45);
    for (position, target) in &walker.trail {
        scene.items.push(floor_box(
            Vec3::new(position[0], 0.010, position[1]),
            Quat::IDENTITY,
            Vec3::new(0.07, 0.006, 0.07),
            segment_color(*target),
        ));
    }
    // The commanded heading, drawn on the floor from the robot.
    let (_, target) = segment_at(walker.t_s());
    let rotation = Quat::from_rotation_y(target);
    let direction = rotation * Vec3::X;
    let base = Vec3::new(observed.base_x_m, 0.014, observed.base_z_m);
    let arrow = [0.98, 0.84, 0.25, 1.0];
    scene.items.push(floor_box(
        base + direction * 0.45,
        rotation,
        Vec3::new(0.6, 0.008, 0.04),
        arrow,
    ));
    scene.items.push(floor_box(
        base + direction * 0.78,
        rotation * Quat::from_rotation_y(std::f64::consts::FRAC_PI_4),
        Vec3::new(0.10, 0.008, 0.10),
        arrow,
    ));
    let roots: Vec<&Path> = walker
        .sim
        .mesh_package_roots()
        .iter()
        .map(PathBuf::as_path)
        .collect();
    mesh_cache
        .resolve_scene(&mut scene, &roots)
        .expect("resolve official Go2 meshes");
    backend
        .render_scene_camera(camera, &view.camera_transform(), &scene, CLEAR_COLOR)
        .expect("render frame")
        .color
        .rgba8
}

fn append_checker_floor(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64, tile_m: f64) {
    let snap = |value: f64| (value / (2.0 * tile_m)).floor() * 2.0 * tile_m;
    for row in -12..=12 {
        for column in -12..=12 {
            let color = if (row + column) & 1 == 0 {
                [0.11, 0.15, 0.21, 1.0]
            } else {
                [0.055, 0.075, 0.11, 1.0]
            };
            scene.items.push(floor_box(
                Vec3::new(
                    snap(center_x_m) + f64::from(column) * tile_m,
                    -0.008,
                    snap(center_z_m) + f64::from(row) * tile_m,
                ),
                Quat::IDENTITY,
                Vec3::new(tile_m * 0.96, 0.008, tile_m * 0.96),
                color,
            ));
        }
    }
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args(["-y", "-loglevel", "error", "-framerate", "12", "-i"])
        .arg(frames_dir.join("frame-%03d.png"))
        .args([
            "-vf",
            "split[a][b];[a]palettegen=max_colors=160[p];[b][p]paletteuse=dither=bayer:bayer_scale=3",
        ])
        .arg(gif_path)
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8]) -> std::io::Result<()> {
    let file = fs::File::create(path)?;
    let mut encoder = Encoder::new(file, WIDTH, HEIGHT);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    encoder
        .write_header()?
        .write_image_data(rgba)
        .map_err(std::io::Error::other)
}

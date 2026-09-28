//! Unitree G1 factory-inspection showcase source and capture.
//!
//! The G1 stands at its station and inspects three gauges in turn: one on
//! the station's display, one lower on the station, and one on the safety
//! barrier to its side. The gauge dials and lamps are drawn by the renderer
//! only; they sit on objects of the factory scene. For each, the arm on that side is raised and aimed at
//! it, held, and lowered. The aim is solved on the G1's own kinematic chain:
//! shoulder pitch and roll are searched so the shoulder-to-hand line points
//! at the target, and the simulation then measures where that line actually
//! points. A target's lamp turns green only if the measured pointing error
//! while holding was under [`CONFIRM_DEG`].
//!
//! It stands the whole time. The scripted G1 gait is a near-stationary stepper
//! (see `docs/G1_LOCOMOTION.md`); an earlier version of this showcase ran it
//! between gestures, and it read as marching on the spot, then a jab at
//! nothing straight ahead, then the arm dropping as the cycle restarted.

use super::media::{
    capture_frames, push_box, push_box_material, push_cylinder, CameraEvidence, CaptureFrame,
    ShowcaseMetadata, SimulationEvidence,
};
use anyhow::{Context, Result};
use rne_ai::{
    build_visual_render_scene, unitree_g1_factory_scene_path,
    unitree_g1_gait_targets_with_arm_pose, UnitreeG1ArmPose, UnitreeG1GaitCommand,
    UrdfJointPositionTarget, UrdfSceneSim,
};
use rne_math::{Quat, Vec3};
use rne_physics::hash_physics_state;
use rne_render::{MeshRenderCache, PbrMaterial, RenderScene};
use rne_render_wgpu::CameraOrbit;
use rne_robot::{KinematicModel, Robot};
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "factory";
const SUBJECT: &str = "Unitree G1 standing inspection: points at three targets in turn";
/// Steps standing before the first gesture.
const STAND_STEPS: u64 = 20;
/// Steps per target: raise, hold, lower, rest.
const RAISE_STEPS: u64 = 40;
const HOLD_STEPS: u64 = 40;
const LOWER_STEPS: u64 = 30;
const TARGET_STEPS: u64 = RAISE_STEPS + HOLD_STEPS + LOWER_STEPS + 10;
const CAPTURE_STEPS: u64 = STAND_STEPS + 3 * TARGET_STEPS;
const CAPTURE_FRAME_COUNT: usize = 76;
const CAPTURE_STRIDE: u64 = CAPTURE_STEPS / CAPTURE_FRAME_COUNT as u64;
/// A target is confirmed when the arm points within this many degrees of it.
const CONFIRM_DEG: f64 = 5.0;
/// Elbow bend while pointing: nearly straight.
const POINT_ELBOW_RAD: f64 = 0.15;
const SETTLE_STEPS: u64 = 120;
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: -0.8,
    pitch_rad: 1.2,
    distance_m: 2.9,
};

#[derive(Clone, Copy, PartialEq, Eq)]
enum Side {
    Left,
    Right,
}

impl Side {
    fn prefix(self) -> &'static str {
        match self {
            Side::Left => "left",
            Side::Right => "right",
        }
    }
}

/// Something the G1 inspects: where it is and which arm points at it.
struct Target {
    name: &'static str,
    position: Vec3,
    side: Side,
}

/// Gauges on the station display (ahead, left, shoulder height), lower on
/// the station (ahead, low), and on the safety barrier (to the right, low).
const TARGETS: [Target; 3] = [
    Target {
        name: "display gauge",
        position: Vec3::new(0.93, 1.11, -0.30),
        side: Side::Left,
    },
    Target {
        name: "station gauge",
        position: Vec3::new(0.95, 0.68, -0.12),
        side: Side::Right,
    },
    Target {
        name: "barrier gauge",
        position: Vec3::new(0.15, 0.45, 0.66),
        side: Side::Right,
    },
];

/// Stands the G1 at its station and points at each target in turn, then
/// optionally renders evenly sampled post-step states with wgpu.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(capture)?;
    let replay = rollout(false)?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "Factory replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    for (target, error_deg) in TARGETS.iter().zip(first.pointing_error_deg) {
        anyhow::ensure!(
            error_deg < CONFIRM_DEG,
            "G1 pointed {error_deg:.1} deg off the {}",
            target.name
        );
    }
    anyhow::ensure!(
        first.max_drift_m < 0.05,
        "G1 moved {:.3} m while standing",
        first.max_drift_m
    );
    anyhow::ensure!(
        first.max_tilt_deg < 8.0,
        "G1 tilted {:.1} deg",
        first.max_tilt_deg
    );
    if capture {
        anyhow::ensure!(
            first.frames.len() == CAPTURE_FRAME_COUNT,
            "factory capture must contain exactly {CAPTURE_FRAME_COUNT} sampled frames"
        );
    }
    let evidence = SimulationEvidence {
        scenario: "G1 standing inspection (examples/90_showcase_captures/factory.rs)",
        steps: CAPTURE_STEPS,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: format!(
            "confirmed_gauges=3/3; pointing_error_deg=[{:.1}, {:.1}, {:.1}]; line_miss_m=[{:.3}, {:.3}, {:.3}]; max_drift_m={:.3}; max_tilt_deg={:.1}; official_g1_meshes={}",
            first.pointing_error_deg[0],
            first.pointing_error_deg[1],
            first.pointing_error_deg[2],
            first.line_miss_m[0],
            first.line_miss_m[1],
            first.line_miss_m[2],
            first.max_drift_m,
            first.max_tilt_deg,
            first.mesh_items
        ),
    };
    let capture_evidence = if capture {
        let orbit = CameraOrbit {
            focus: Vec3::new(0.3, 0.85, 0.0),
            yaw_rad: CAMERA.yaw_rad,
            pitch_rad: CAMERA.pitch_rad,
            distance_m: CAMERA.distance_m,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &first.frames,
            orbit,
            [0.040, 0.055, 0.075, 1.0],
            first.frames.len() / 2,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "Official G1 link meshes are rebuilt from the simulation world after each fixed step. Target lamps turn green only when the measured pointing error while holding is under 5 degrees; the pointing ray is drawn from the simulated shoulder through the simulated hand.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/unitree_g1_factory.rne.scene.toml",
            "assets/robots/unitree_g1_dynamic.rne.robot.toml",
            "crates/rne_ai/src/env/urdf_scene/unitree_g1_commanded_gait.rs",
            "examples/90_showcase_captures/factory.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment factory",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment factory",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-factory.json");
        fs::write(&path, to_vec_pretty(&metadata)?)
            .with_context(|| format!("write {}", path.display()))?;
    }
    Ok(metadata)
}

struct Rollout {
    initial_digest: u64,
    final_digest: u64,
    pointing_error_deg: [f64; 3],
    line_miss_m: [f64; 3],
    max_drift_m: f64,
    max_tilt_deg: f64,
    mesh_items: usize,
    frames: Vec<CaptureFrame>,
}

/// Arm joint targets that point one arm at a target.
#[derive(Clone, Copy)]
struct Aim {
    pitch: f64,
    roll: f64,
}

/// Settles the G1 into its stance, as the factory inspection episode does.
fn settle(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let targets = [
        stand_target("left_hip_pitch_link", -0.18),
        stand_target("left_knee_link", 0.36),
        stand_target("left_ankle_pitch_link", -0.18),
        stand_target("right_hip_pitch_link", -0.18),
        stand_target("right_knee_link", 0.36),
        stand_target("right_ankle_pitch_link", -0.18),
    ];
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&targets);
    }
}

fn stand_target(link_name: &'static str, position: f64) -> UrdfJointPositionTarget<'static> {
    UrdfJointPositionTarget {
        link_name,
        position,
    }
}

/// The standing pose with relaxed, hanging arms.
fn standing() -> [UrdfJointPositionTarget<'static>; 23] {
    unitree_g1_gait_targets_with_arm_pose(
        0,
        UnitreeG1GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: 60,
        },
        UnitreeG1ArmPose::Hanging,
    )
}

/// Searches shoulder pitch and roll, with the elbow nearly straight and the
/// shoulder yaw and wrist at rest, for the pose whose shoulder-to-hand line
/// points at `target`, on the G1's kinematic model in its current stance.
fn aim(sim: &UrdfSceneSim, target: &Target) -> Result<Aim> {
    let robot = sim
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<Robot>().map(|_| entity.id()))
        .context("no G1 robot")?;
    let model = KinematicModel::from_robot(sim.world(), robot).context("G1 kinematic model")?;
    let names = model.movable_dof_names();
    let mut q = vec![0.0; model.dof()];
    for (index, name) in names.iter().enumerate() {
        let joint = model.joint_entity_by_name(name).context("joint")?;
        let child = model.joint_child_link(joint).context("child link")?;
        let link = model
            .link_name(model.link_index(child).context("link index")?)
            .context("link name")?;
        q[index] = sim.named_joint_position(link).unwrap_or(0.0);
    }
    let side = target.side.prefix();
    let dof = |joint: &str| -> Result<usize> {
        names
            .iter()
            .position(|name| name == &format!("{side}_{joint}_joint"))
            .with_context(|| format!("no {side}_{joint}_joint"))
    };
    let (pitch, roll, yaw, elbow) = (
        dof("shoulder_pitch")?,
        dof("shoulder_roll")?,
        dof("shoulder_yaw")?,
        dof("elbow")?,
    );
    q[yaw] = 0.0;
    q[elbow] = POINT_ELBOW_RAD;
    let shoulder = model
        .link_entity_by_name(&format!("{side}_shoulder_pitch_link"))
        .context("shoulder link")?;
    let hand = model
        .link_entity_by_name(&format!("{side}_wrist_roll_rubber_hand"))
        .context("hand link")?;
    let limits = model.joint_limits();
    let error = |q: &[f64]| -> Result<f64> {
        let fk = model.forward_kinematics(q)?;
        let s = fk.link_transform(shoulder).context("shoulder")?.translation;
        let h = fk.link_transform(hand).context("hand")?.translation;
        Ok((h - s)
            .normalize()
            .angle_between((target.position - s).normalize()))
    };
    let mut best = (f64::INFINITY, q[pitch], q[roll]);
    // A coarse sweep over each joint's own range, then two finer ones around
    // the best.
    for (span_pitch, span_roll, step) in [
        (None, None, 0.05),
        (Some(0.06), Some(0.06), 0.01),
        (Some(0.012), Some(0.012), 0.002),
    ] {
        let range = |axis: usize, centre: f64, span: Option<f64>| {
            let (lower, upper) = (limits[axis].lower, limits[axis].upper);
            match span {
                None => (lower, upper),
                Some(span) => ((centre - span).max(lower), (centre + span).min(upper)),
            }
        };
        let (p0, p1) = range(pitch, best.1, span_pitch);
        let (r0, r1) = range(roll, best.2, span_roll);
        let mut p = p0;
        while p <= p1 {
            let mut r = r0;
            while r <= r1 {
                q[pitch] = p;
                q[roll] = r;
                let e = error(&q)?;
                if e < best.0 {
                    best = (e, p, r);
                }
                r += step;
            }
            p += step;
        }
    }
    Ok(Aim {
        pitch: best.1,
        roll: best.2,
    })
}

fn smoothstep(value: f64) -> f64 {
    let value = value.clamp(0.0, 1.0);
    value * value * (3.0 - 2.0 * value)
}

/// Which target is being worked at `step`, how far its arm is raised (0 to
/// 1), and whether it is in the hold.
fn schedule(step: u64) -> Option<(usize, f64, bool)> {
    let local = step.checked_sub(STAND_STEPS)?;
    let index = (local / TARGET_STEPS) as usize;
    if index >= TARGETS.len() {
        return None;
    }
    let t = local % TARGET_STEPS;
    let (raised, holding) = if t < RAISE_STEPS {
        (smoothstep(t as f64 / RAISE_STEPS as f64), false)
    } else if t < RAISE_STEPS + HOLD_STEPS {
        (1.0, true)
    } else if t < RAISE_STEPS + HOLD_STEPS + LOWER_STEPS {
        (
            1.0 - smoothstep((t - RAISE_STEPS - HOLD_STEPS) as f64 / LOWER_STEPS as f64),
            false,
        )
    } else {
        (0.0, false)
    };
    Some((index, raised, holding))
}

/// Replaces the pointing arm's targets with a blend from hanging to `aim`.
fn with_gesture(
    mut targets: [UrdfJointPositionTarget<'static>; 23],
    side: Side,
    aim: Aim,
    raised: f64,
) -> [UrdfJointPositionTarget<'static>; 23] {
    let (pitch, roll, yaw, elbow) = match side {
        Side::Left => (
            "left_shoulder_pitch_link",
            "left_shoulder_roll_link",
            "left_shoulder_yaw_link",
            "left_elbow_link",
        ),
        Side::Right => (
            "right_shoulder_pitch_link",
            "right_shoulder_roll_link",
            "right_shoulder_yaw_link",
            "right_elbow_link",
        ),
    };
    for target in &mut targets {
        let goal = if target.link_name == pitch {
            aim.pitch
        } else if target.link_name == roll {
            aim.roll
        } else if target.link_name == yaw {
            0.0
        } else if target.link_name == elbow {
            POINT_ELBOW_RAD
        } else {
            continue;
        };
        target.position += (goal - target.position) * raised;
    }
    targets
}

/// Where the arm on `side` points in the simulation: shoulder, unit
/// direction to the hand, and hand.
fn pointing(sim: &UrdfSceneSim, side: Side) -> Result<(Vec3, Vec3, Vec3)> {
    let at = |link: String| -> Result<Vec3> {
        let (x, y, z) = sim
            .link_translation_m(&link)
            .with_context(|| format!("missing {link}"))?;
        Ok(Vec3::new(x, y, z))
    };
    let shoulder = at(format!("{}_shoulder_pitch_link", side.prefix()))?;
    let hand = at(format!("{}_wrist_roll_rubber_hand", side.prefix()))?;
    Ok((shoulder, (hand - shoulder).normalize(), hand))
}

fn rollout(capture: bool) -> Result<Rollout> {
    let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_factory_scene_path())
        .context("load the G1 factory scene")?;
    settle(&mut sim);
    let aims: Vec<Aim> = TARGETS
        .iter()
        .map(|target| aim(&sim, target))
        .collect::<Result<_>>()?;
    let initial_digest = hash_physics_state(sim.world());
    let pelvis = |sim: &UrdfSceneSim| -> Result<Vec3> {
        let (x, y, z) = sim.link_translation_m("pelvis").context("pelvis")?;
        Ok(Vec3::new(x, y, z))
    };
    let start = pelvis(&sim)?;
    let mut pointing_error_deg = [f64::INFINITY; 3];
    let mut line_miss_m = [f64::INFINITY; 3];
    let mut confirmed = [false; 3];
    let (mut max_drift_m, mut max_tilt_deg) = (0.0_f64, 0.0_f64);
    let mut frames = Vec::new();
    let mut cache = MeshRenderCache::new();
    let mut mesh_items = 0;
    for step in 1..=CAPTURE_STEPS {
        let gesture = schedule(step);
        let targets = match gesture {
            Some((index, raised, _)) => {
                with_gesture(standing(), TARGETS[index].side, aims[index], raised)
            }
            None => standing(),
        };
        sim.step_joint_position_targets(&targets);
        let now = pelvis(&sim)?;
        max_drift_m = max_drift_m.max((now - start).x.hypot((now - start).z));
        if let Some(transform) = sim.named_transform("torso_link") {
            // The model is Z-up inside a Y-up world: the torso's own +z is
            // its up axis.
            let up = transform.rotation * Vec3::Z;
            max_tilt_deg = max_tilt_deg.max(up.y.clamp(-1.0, 1.0).acos().to_degrees());
        }
        let mut ray = None;
        if let Some((index, _, true)) = gesture {
            let (shoulder, direction, hand) = pointing(&sim, TARGETS[index].side)?;
            let to_target = TARGETS[index].position - shoulder;
            let error = direction.angle_between(to_target.normalize()).to_degrees();
            let miss = (to_target - direction * to_target.dot(direction)).length();
            // The hold's last half: the arm has arrived.
            let t = (step - STAND_STEPS) % TARGET_STEPS;
            if t >= RAISE_STEPS + HOLD_STEPS / 2 {
                // The worst error over the settled half of the hold.
                pointing_error_deg[index] = if pointing_error_deg[index].is_finite() {
                    pointing_error_deg[index].max(error)
                } else {
                    error
                };
                line_miss_m[index] = if line_miss_m[index].is_finite() {
                    line_miss_m[index].max(miss)
                } else {
                    miss
                };
                if error < CONFIRM_DEG {
                    confirmed[index] = true;
                }
            }
            ray = Some((
                hand,
                direction,
                to_target.length() - (hand - shoulder).length(),
            ));
        }
        if capture && step % CAPTURE_STRIDE == 0 && frames.len() < CAPTURE_FRAME_COUNT {
            let (scene, current_mesh_items) =
                render_scene(&sim, &mut cache, confirmed, gesture.map(|g| g.0), ray)?;
            mesh_items = mesh_items.max(current_mesh_items);
            frames.push(CaptureFrame {
                step,
                phase: match gesture {
                    Some((index, _, true)) => {
                        format!("point-at-{}", TARGETS[index].name.replace(' ', "-"))
                    }
                    Some((index, raised, false)) if raised > 0.0 => {
                        format!("raise-toward-{}", TARGETS[index].name.replace(' ', "-"))
                    }
                    _ => "stand".into(),
                },
                scene,
            });
        }
    }
    if !capture {
        let (_, current_mesh_items) = render_scene(&sim, &mut cache, confirmed, None, None)?;
        mesh_items = current_mesh_items;
    }
    anyhow::ensure!(
        mesh_items >= 20,
        "factory render resolved only {mesh_items} G1 mesh items"
    );
    Ok(Rollout {
        initial_digest,
        final_digest: hash_physics_state(sim.world()),
        pointing_error_deg,
        line_miss_m,
        max_drift_m,
        max_tilt_deg,
        mesh_items,
        frames,
    })
}

fn render_scene(
    sim: &UrdfSceneSim,
    cache: &mut MeshRenderCache,
    confirmed: [bool; 3],
    active: Option<usize>,
    ray: Option<(Vec3, Vec3, f64)>,
) -> Result<(RenderScene, usize)> {
    let mut scene = build_visual_render_scene(sim.world());
    let roots = sim.mesh_package_roots().to_vec();
    let root_refs = roots
        .iter()
        .map(std::path::PathBuf::as_path)
        .collect::<Vec<_>>();
    cache
        .resolve_scene(&mut scene, &root_refs)
        .map_err(|error| anyhow::anyhow!("resolve official G1/factory meshes: {error}"))?;
    let mesh_items = scene
        .items
        .iter()
        .filter(|item| item.mesh.is_some())
        .count();
    push_targets(&mut scene, confirmed, active);
    if let Some((hand, direction, length)) = ray {
        // The pointing line, from the simulated hand along the simulated
        // shoulder-to-hand direction, as far as the target.
        push_cylinder(
            &mut scene,
            hand + direction * (0.5 * length),
            0.006,
            length.max(0.0),
            Quat::from_rotation_arc(Vec3::Z, direction),
            [1.0, 0.25, 0.15, 1.0],
        );
    }
    push_dressing(&mut scene);
    Ok((scene, mesh_items))
}

/// Each target: a gauge face turned toward the robot, a bezel, and a lamp
/// that is amber until the target is confirmed and green after. The one
/// being inspected gets a brighter ring.
fn push_targets(scene: &mut RenderScene, confirmed: [bool; 3], active: Option<usize>) {
    for (index, target) in TARGETS.iter().enumerate() {
        let toward = (Vec3::new(0.0, target.position.y, 0.0) - target.position).normalize();
        let facing = Quat::from_rotation_arc(Vec3::Z, toward);
        let lamp = if confirmed[index] {
            [0.15, 1.0, 0.45]
        } else {
            [1.0, 0.62, 0.1]
        };
        push_box_material(
            scene,
            target.position - toward * 0.03,
            Vec3::new(0.26, 0.26, 0.04),
            facing,
            [0.18, 0.2, 0.24, 1.0],
            PbrMaterial::new([0.18, 0.2, 0.24, 1.0], 0.4, 0.3, [0.0; 3]),
        );
        push_cylinder(
            scene,
            target.position,
            0.095,
            0.02,
            facing,
            [0.92, 0.93, 0.9, 1.0],
        );
        push_box_material(
            scene,
            target.position + toward * 0.012 + Vec3::new(0.0, 0.03, 0.0),
            Vec3::new(0.012, 0.07, 0.004),
            facing,
            [0.8, 0.1, 0.1, 1.0],
            PbrMaterial::new([0.8, 0.1, 0.1, 1.0], 0.5, 0.0, [0.0; 3]),
        );
        push_box_material(
            scene,
            target.position + Vec3::new(0.0, 0.17, 0.0),
            Vec3::new(0.06, 0.06, 0.06),
            facing,
            [lamp[0], lamp[1], lamp[2], 1.0],
            PbrMaterial::new([lamp[0], lamp[1], lamp[2], 1.0], 0.3, 0.0, lamp),
        );
        if active == Some(index) {
            push_cylinder(
                scene,
                target.position - toward * 0.01,
                0.13,
                0.01,
                facing,
                [1.0, 0.8, 0.2, 1.0],
            );
        }
    }
}

fn push_dressing(scene: &mut RenderScene) {
    // Render-only factory dressing: a tiled floor, overhead light fixtures
    // and a pipe run.
    const FLOOR_TILE: [f32; 4] = [0.40, 0.42, 0.44, 1.0];
    const FLOOR_TILE_ALT: [f32; 4] = [0.20, 0.22, 0.24, 1.0];
    const FIXTURE_BEAM: [f32; 4] = [0.24, 0.28, 0.33, 1.0];
    const FIXTURE_LIGHT: [f32; 4] = [0.94, 0.95, 0.88, 1.0];
    const PIPE: [f32; 4] = [0.36, 0.40, 0.43, 1.0];
    const CONCRETE: [f32; 4] = [0.30, 0.31, 0.32, 1.0];
    const SIDE_WALL: [f32; 4] = [0.34, 0.37, 0.40, 1.0];
    const BACK_WALL: [f32; 4] = [0.28, 0.32, 0.38, 1.0];
    const HAZARD: [f32; 4] = [0.85, 0.66, 0.10, 1.0];
    const CABINET: [f32; 4] = [0.22, 0.30, 0.38, 1.0];
    const LAMP_RED: [f32; 4] = [0.9, 0.15, 0.1, 1.0];
    const LAMP_GREEN: [f32; 4] = [0.2, 0.85, 0.35, 1.0];

    // Painted floor tiles under and around the inspection route replace the
    // flat untextured slab with visible scale and structure. The default
    // collider-derived ground box's top surface sits at world y = 0.0, so
    // these sit a visible few millimeters above it rather than underneath.
    for (index, x_m) in [-0.6, -0.1, 0.4, 0.9].into_iter().enumerate() {
        for (row, z_m) in [-1.0, -0.5, 0.0, 0.5].into_iter().enumerate() {
            let alt = (index + row) % 2 == 0;
            push_box(
                scene,
                Vec3::new(x_m, 0.012, z_m),
                Vec3::new(0.46, 0.024, 0.46),
                if alt { FLOOR_TILE } else { FLOOR_TILE_ALT },
            );
        }
    }
    // A concrete slab around the tiles, and a side wall past the station
    // with a hazard stripe and a switch cabinet, close the bay in.
    push_box(
        scene,
        Vec3::new(0.3, 0.004, 0.2),
        Vec3::new(4.2, 0.008, 3.4),
        CONCRETE,
    );
    push_box(
        scene,
        Vec3::new(1.95, 1.2, 0.3),
        Vec3::new(0.1, 2.4, 2.9),
        SIDE_WALL,
    );
    // The back wall is 3.2 m by 2 m; this carries it to the side wall.
    push_box(
        scene,
        Vec3::new(1.775, 1.2, -1.15),
        Vec3::new(0.35, 2.4, 0.08),
        BACK_WALL,
    );
    push_box(
        scene,
        Vec3::new(0.0, 2.2, -1.15),
        Vec3::new(3.2, 0.4, 0.08),
        BACK_WALL,
    );
    push_box(
        scene,
        Vec3::new(1.89, 0.1, 0.3),
        Vec3::new(0.02, 0.2, 2.9),
        HAZARD,
    );
    push_box(
        scene,
        Vec3::new(1.8, 0.9, 0.55),
        Vec3::new(0.2, 1.2, 0.6),
        CABINET,
    );
    for (dy, colour) in [(0.25, LAMP_RED), (0.18, LAMP_GREEN)] {
        push_box(
            scene,
            Vec3::new(1.69, 0.9 + dy, 0.55),
            Vec3::new(0.02, 0.04, 0.04),
            colour,
        );
    }
    // Overhead light fixtures and a pipe run give the ceiling area depth.
    push_box(
        scene,
        Vec3::new(0.0, 1.62, -1.17),
        Vec3::new(3.3, 0.08, 0.08),
        FIXTURE_BEAM,
    );
    for x_m in [-1.25, -0.4, 0.45, 1.25] {
        push_cylinder(
            scene,
            Vec3::new(x_m, 1.42, -1.08),
            0.022,
            2.6,
            Quat::IDENTITY,
            PIPE,
        );
        push_box(
            scene,
            Vec3::new(x_m, 2.05, 0.0),
            Vec3::new(0.42, 0.04, 0.14),
            FIXTURE_LIGHT,
        );
    }
}

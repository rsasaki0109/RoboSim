//! Unitree G1 factory-inspection showcase source and capture.
//!
//! Parts ride a belt conveyor past the G1's station. For each part the belt
//! stops with the part in front of the robot, the G1 lowers its right hand
//! onto the part's top face until the simulation reports contact between the
//! hand and the part, holds it there, and lifts away; the belt then runs on.
//! A part's lamp turns green only on that measured contact.
//!
//! The belt is a kinematic body driven along the conveyor; the parts are free
//! dynamic bodies that ride it by friction and stop when it stops. The arm is
//! steered by damped least-squares inverse kinematics on the G1's own
//! kinematic chain, and the G1 stands on its own legs throughout.

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
use rne_ecs::{spawn_named, Entity};
use rne_math::{Quat, Vec3};
use rne_physics::{
    hash_physics_state, Collider, ColliderShape, CommandedKinematicPose, PhysicsMaterial,
    RigidBody, RigidBodyType,
};
use rne_render::{MeshRenderCache, PbrMaterial, RenderScene};
use rne_render_wgpu::CameraOrbit;
use rne_robot::{KinematicModel, Robot};
use rne_world::Transform3;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "factory";
const SUBJECT: &str = "Unitree G1 touch inspection of parts on a belt conveyor";
const SETTLE_STEPS: u64 = 120;
/// Steps over which the hands rise to rest after the legs settle.
const RAISE_STEPS: u64 = 60;
const CAPTURE_STEPS: u64 = 1250;
const CAPTURE_FRAME_COUNT: usize = 80;
const CAPTURE_STRIDE: u64 = CAPTURE_STEPS / CAPTURE_FRAME_COUNT as u64;
/// Physics step of the G1 scene.
const DT_S: f64 = 1.0 / 60.0;

/// The conveyor runs along +z in front of the G1, belt top at this height.
const BELT_X_M: f64 = 0.40;
const BELT_TOP_M: f64 = 0.64;
const BELT_HALF_WIDTH_M: f64 = 0.15;
const BELT_HALF_THICKNESS_M: f64 = 0.01;
/// The belt body is long enough that its drawn stretch stays covered over
/// the whole run.
const BELT_HALF_LENGTH_M: f64 = 3.0;
const BELT_START_Z_M: f64 = -1.8;
/// The stretch of conveyor that is drawn.
const CONVEYOR_Z_M: (f64, f64) = (-1.1, 1.6);
const BELT_SPEED_M_S: f64 = 0.16;
const BELT_ACCEL_M_S2: f64 = 0.4;

/// The parts: size, mass, and where they start on the belt.
const PART_HALF_M: Vec3 = Vec3::new(0.045, 0.06, 0.045);
const PART_MASS_KG: f64 = 0.4;
const PART_START_Z_M: [f64; 3] = [-0.32, -0.72, -1.02];
/// Where along the belt a part is stopped for inspection: in front of the
/// right shoulder.
const INSPECT_Z_M: f64 = 0.14;

/// Hand motion: hover this far above the part's top, then descend at this
/// speed until contact, hold, and lift away.
const HOVER_M: f64 = 0.03;
const DESCENT_M_S: f64 = 0.03;
const REACH_STEPS: u64 = 45;
const HOLD_STEPS: u64 = 30;
const RETRACT_STEPS: u64 = 45;
/// Steps for the arm to settle into the hanging pose after a retract.
const SETTLE_ARM_STEPS: u64 = 20;
/// How high over the belt the fingertips travel between the hanging pose and
/// a part.
const ARC_CLEARANCE_M: f64 = 0.12;
/// The hand never goes further below the part's top than this.
const MAX_PRESS_M: f64 = 0.02;
/// Where the fingertips are held relative to the depth at which they met the
/// part: a millimetre into its face, so they rest on it.
const HOLD_LIFT_M: f64 = -0.001;
/// Share of the measured fingertip error added to the command each step.
const TIP_CORRECTION_GAIN: f64 = 0.05;
const ARM_JOINTS: [&str; 4] = [
    "shoulder_pitch_joint",
    "shoulder_roll_joint",
    "shoulder_yaw_joint",
    "elbow_joint",
];
const ARM_LINKS: [&str; 4] = [
    "shoulder_pitch_link",
    "shoulder_roll_link",
    "shoulder_yaw_link",
    "elbow_link",
];
/// The link that carries the fingertip contact box: the right forearm.
const TIP_LINK: &str = "right_elbow_link";
/// A contact box over the fingertips of the right rubber hand, in the
/// forearm's frame: the bounds of the vertices of
/// `right_wrist_roll_rubber_hand.STL` beyond 0.235 m along the hand, shifted
/// by the wrist roll joint's origin 0.100 m along the forearm.
const FINGERTIP_BOX_SIZE_M: [f64; 3] = [0.02, 0.044, 0.079];
const FINGERTIP_BOX_CENTER_M: [f64; 3] = [0.344, 0.018, -0.012];
/// Where each hand rests between inspections: in front of the chest, well
/// above the belt.
const REST_POINT_M: [Vec3; 2] = [Vec3::new(0.22, 0.98, 0.2), Vec3::new(0.22, 0.98, -0.2)];

const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 1.1,
    pitch_rad: 1.18,
    distance_m: 1.9,
};

/// Stands the G1 at the conveyor and inspects three parts by touch, then
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
    for (index, part) in first.parts.iter().enumerate() {
        anyhow::ensure!(
            part.touched_at_step.is_some(),
            "the G1's hand never touched part {index}: {part:?}"
        );
        anyhow::ensure!(
            part.stop_error_m < 0.03,
            "part {index} stopped {:.3} m from the inspection point",
            part.stop_error_m
        );
        anyhow::ensure!(
            part.contact_steps * 5 >= HOLD_STEPS * 4,
            "the hand stayed on part {index} for only {} of {HOLD_STEPS} steps",
            part.contact_steps
        );
        anyhow::ensure!(
            part.pushed_m < 0.005,
            "the touch moved part {index} by {:.3} m",
            part.pushed_m
        );
        anyhow::ensure!(
            part.tip_offset_m < 0.02 && part.tip_height_m.abs() < 0.005,
            "the fingertips met part {index} {:.3} m off the centre of its top and {:.3} m above it",
            part.tip_offset_m,
            part.tip_height_m
        );
    }
    anyhow::ensure!(
        first.parts_left_on_belt == first.parts.len(),
        "a part fell off the belt"
    );
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
    let touched = first
        .parts
        .iter()
        .filter(|part| part.touched_at_step.is_some())
        .count();
    let evidence = SimulationEvidence {
        scenario: "G1 conveyor touch inspection (examples/90_showcase_captures/factory.rs)",
        steps: CAPTURE_STEPS,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: format!(
            "touched_parts={touched}/{}; stop_error_m=[{}]; tip_offset_m=[{}]; tip_height_m=[{}]; touch_push_m=[{}]; hand_contact_steps=[{}]; max_drift_m={:.3}; max_tilt_deg={:.1}; official_g1_meshes={}",
            first.parts.len(),
            join(first.parts.iter().map(|part| format!("{:.3}", part.stop_error_m))),
            join(first.parts.iter().map(|part| format!("{:.3}", part.tip_offset_m))),
            join(first.parts.iter().map(|part| format!("{:.4}", part.tip_height_m))),
            join(first.parts.iter().map(|part| format!("{:.4}", part.pushed_m))),
            join(first.parts.iter().map(|part| part.contact_steps.to_string())),
            first.max_drift_m,
            first.max_tilt_deg,
            first.mesh_items
        ),
    };
    let capture_evidence = if capture {
        let orbit = CameraOrbit {
            focus: Vec3::new(0.3, 0.78, 0.0),
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
        visual_state_sync: "Official G1 link meshes, the belt and the parts are rebuilt from the simulation world after each fixed step. The belt is a kinematic body and the parts are dynamic bodies riding it; a part's lamp turns green only when the simulation reports contact between the G1's hand and that part.",
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

fn join(items: impl Iterator<Item = String>) -> String {
    items.collect::<Vec<_>>().join(", ")
}

#[derive(Clone, Debug, Default)]
struct PartEvidence {
    /// Distance along the belt from the inspection point when the belt
    /// stopped for this part.
    stop_error_m: f64,
    touched_at_step: Option<u64>,
    /// Steps with hand-part contact reported.
    contact_steps: u64,
    /// How far the part moved while the hand was on it.
    pushed_m: f64,
    /// Where the fingertips met the part, relative to the centre of its top
    /// face: across the face, and above it.
    tip_offset_m: f64,
    tip_height_m: f64,
}

struct Rollout {
    initial_digest: u64,
    final_digest: u64,
    parts: Vec<PartEvidence>,
    parts_left_on_belt: usize,
    max_drift_m: f64,
    max_tilt_deg: f64,
    mesh_items: usize,
    frames: Vec<CaptureFrame>,
}

/// Settles the G1 into its stance under the position motors the factory
/// inspection episode uses, then raises its hands to rest over
/// [`RAISE_STEPS`] steps.
fn settle(sim: &mut UrdfSceneSim, from: &[[f64; 4]; 2], rest: &[[f64; 4]; 2]) {
    sim.configure_position_motors(220.0, 24.0, 88.0);
    let legs = [
        stand_target("left_hip_pitch_link", -0.18),
        stand_target("left_knee_link", 0.36),
        stand_target("left_ankle_pitch_link", -0.18),
        stand_target("right_hip_pitch_link", -0.18),
        stand_target("right_knee_link", 0.36),
        stand_target("right_ankle_pitch_link", -0.18),
    ];
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&legs);
    }
    for step in 1..=RAISE_STEPS {
        let t = smoothstep(step as f64 / RAISE_STEPS as f64);
        let blend = |a: [f64; 4], b: [f64; 4]| std::array::from_fn(|k| a[k] + (b[k] - a[k]) * t);
        sim.step_joint_position_targets(&pose(blend(from[0], rest[0]), blend(from[1], rest[1])));
    }
}

/// The standing pose with the four joints of each arm set.
fn pose(right: [f64; 4], left: [f64; 4]) -> [UrdfJointPositionTarget<'static>; 23] {
    let mut targets = standing();
    for target in &mut targets {
        for (slot, link) in ARM_LINKS.iter().enumerate() {
            if target.link_name == format!("right_{link}") {
                target.position = right[slot];
            } else if target.link_name == format!("left_{link}") {
                target.position = left[slot];
            }
        }
    }
    targets
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

/// The conveyor belt: a kinematic body commanded along +z, so the solver
/// knows its velocity and carries what rests on it.
struct Belt {
    entity: Entity,
    travelled_m: f64,
    speed_m_s: f64,
}

impl Belt {
    fn spawn(sim: &mut UrdfSceneSim) -> Self {
        let world = sim.world_mut();
        let entity = spawn_named(world, "conveyor_belt");
        world.entity_mut(entity).insert((
            RigidBody {
                body_type: RigidBodyType::Kinematic,
                ..RigidBody::default()
            },
            CommandedKinematicPose,
            Collider {
                shape: ColliderShape::Cuboid {
                    half_extents_m: Vec3::new(
                        BELT_HALF_WIDTH_M,
                        BELT_HALF_THICKNESS_M,
                        BELT_HALF_LENGTH_M,
                    ),
                },
                material: PhysicsMaterial {
                    friction: 0.9,
                    ..PhysicsMaterial::default()
                },
                ..Collider::default()
            },
            Transform3::from_translation_rotation(Belt::center(0.0), Quat::IDENTITY),
        ));
        Self {
            entity,
            travelled_m: 0.0,
            speed_m_s: 0.0,
        }
    }

    fn center(travelled_m: f64) -> Vec3 {
        Vec3::new(
            BELT_X_M,
            BELT_TOP_M - BELT_HALF_THICKNESS_M,
            BELT_START_Z_M + travelled_m,
        )
    }

    /// Accelerates toward `run_m_s` and commands the next pose.
    fn drive(&mut self, sim: &mut UrdfSceneSim, run_m_s: f64) {
        let change =
            (run_m_s - self.speed_m_s).clamp(-BELT_ACCEL_M_S2 * DT_S, BELT_ACCEL_M_S2 * DT_S);
        self.speed_m_s += change;
        self.travelled_m += self.speed_m_s * DT_S;
        if let Some(mut transform) = sim.world_mut().get_mut::<Transform3>(self.entity) {
            transform.translation = Belt::center(self.travelled_m);
        }
    }
}

fn spawn_part(sim: &mut UrdfSceneSim, index: usize) -> String {
    let name = format!("conveyor_part_{index}");
    let world = sim.world_mut();
    let entity = spawn_named(world, &name);
    world.entity_mut(entity).insert((
        RigidBody {
            body_type: RigidBodyType::Dynamic,
            mass_kg: PART_MASS_KG,
            ..RigidBody::default()
        },
        Collider {
            shape: ColliderShape::Cuboid {
                half_extents_m: PART_HALF_M,
            },
            material: PhysicsMaterial {
                friction: 0.8,
                ..PhysicsMaterial::default()
            },
            ..Collider::default()
        },
        Transform3::from_translation_rotation(
            Vec3::new(
                BELT_X_M,
                BELT_TOP_M + PART_HALF_M.y + 0.002,
                PART_START_Z_M[index],
            ),
            Quat::IDENTITY,
        ),
    ));
    name
}

/// The G1's kinematic model and one arm's degrees of freedom.
struct Arm {
    model: KinematicModel,
    names: Vec<String>,
    dofs: [usize; 4],
    hand: Entity,
    pelvis: Entity,
}

impl Arm {
    fn new(sim: &UrdfSceneSim, side: &str) -> Result<Self> {
        let robot = sim
            .world()
            .iter_entities()
            .find_map(|entity| entity.get::<Robot>().map(|_| entity.id()))
            .context("no G1 robot")?;
        let model = KinematicModel::from_robot(sim.world(), robot).context("G1 kinematic model")?;
        let names = model.movable_dof_names();
        let dof = |joint: &str| -> Result<usize> {
            names
                .iter()
                .position(|name| name == joint)
                .with_context(|| format!("no {joint}"))
        };
        let dofs = [
            dof(&format!("{side}_{}", ARM_JOINTS[0]))?,
            dof(&format!("{side}_{}", ARM_JOINTS[1]))?,
            dof(&format!("{side}_{}", ARM_JOINTS[2]))?,
            dof(&format!("{side}_{}", ARM_JOINTS[3]))?,
        ];
        let hand = model
            .link_entity_by_name(&format!("{side}_elbow_link"))
            .context("forearm link")?;
        let pelvis = model.link_entity_by_name("pelvis").context("pelvis link")?;
        Ok(Self {
            model,
            names,
            dofs,
            hand,
            pelvis,
        })
    }

    /// The current joint vector, read from the simulation.
    fn joints(&self, sim: &UrdfSceneSim) -> Result<Vec<f64>> {
        let mut q = vec![0.0; self.model.dof()];
        for (index, name) in self.names.iter().enumerate() {
            let joint = self.model.joint_entity_by_name(name).context("joint")?;
            let child = self.model.joint_child_link(joint).context("child link")?;
            let link = self
                .model
                .link_name(self.model.link_index(child).context("link index")?)
                .context("link name")?;
            q[index] = sim.named_joint_position(link).unwrap_or(0.0);
        }
        Ok(q)
    }

    /// The touch point of the hand in the model's frame for joint vector `q`.
    fn model_touch_point(&self, q: &[f64], local: Vec3) -> Result<Vec3> {
        let fk = self.model.forward_kinematics(q)?;
        let hand = fk.link_transform(self.hand).context("forearm transform")?;
        Ok(hand.translation + hand.rotation * local)
    }

    /// Maps between the model's frame, whose pelvis stays where the robot
    /// was loaded, and the world, where the standing robot drifts and turns
    /// a little on its feet: `(world from model)`.
    fn world_from_model(&self, sim: &UrdfSceneSim, q: &[f64]) -> Result<(Quat, Vec3, Vec3)> {
        let fk = self.model.forward_kinematics(q)?;
        let model = *fk.link_transform(self.pelvis).context("model pelvis")?;
        let world = sim.named_transform("pelvis").context("pelvis")?;
        let rotation = world.rotation * model.rotation.inverse();
        Ok((rotation, model.translation, world.translation))
    }

    /// The touch point of the hand in the world, read from the simulation.
    fn touch_point(&self, sim: &UrdfSceneSim, q: &[f64]) -> Result<Vec3> {
        let (rotation, model_pelvis, world_pelvis) = self.world_from_model(sim, q)?;
        Ok(world_pelvis
            + rotation * (self.model_touch_point(q, lowest_tip_local(sim)?)? - model_pelvis))
    }

    /// Damped least-squares steps of the four arm joints toward putting the
    /// touch point at `target`, starting from `q`; returns the arm angles.
    fn solve(&self, sim: &UrdfSceneSim, q: &[f64], target: Vec3) -> Result<[f64; 4]> {
        const DAMPING: f64 = 0.05;
        let limits = self.model.joint_limits();
        let (rotation, model_pelvis, world_pelvis) = self.world_from_model(sim, q)?;
        let target = model_pelvis + rotation.inverse() * (target - world_pelvis);
        let local = lowest_tip_local(sim)?;
        let mut q = q.to_vec();
        for _ in 0..30 {
            let error = target - self.model_touch_point(&q, local)?;
            if error.length() < 1e-4 {
                break;
            }
            let jacobian = self.model.jacobian(&q, self.hand, local)?;
            let j: [[f64; 4]; 3] = std::array::from_fn(|row| {
                std::array::from_fn(|col| jacobian.get(row, self.dofs[col]))
            });
            let a: [[f64; 3]; 3] = std::array::from_fn(|r| {
                std::array::from_fn(|c| {
                    (0..4).map(|k| j[r][k] * j[c][k]).sum::<f64>()
                        + if r == c { DAMPING * DAMPING } else { 0.0 }
                })
            });
            let Some(y) = solve3(a, [error.x, error.y, error.z]) else {
                break;
            };
            for (col, &dof) in self.dofs.iter().enumerate() {
                let step: f64 = (0..3).map(|row| j[row][col] * y[row]).sum();
                q[dof] = (q[dof] + step).clamp(limits[dof].lower, limits[dof].upper);
            }
        }
        Ok(self.dofs.map(|dof| q[dof]))
    }
}

fn solve3(a: [[f64; 3]; 3], b: [f64; 3]) -> Option<[f64; 3]> {
    let det = |m: [[f64; 3]; 3]| {
        m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1])
            - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0])
            + m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0])
    };
    let d = det(a);
    if d.abs() < 1e-12 {
        return None;
    }
    let column = |col: usize| {
        let mut m = a;
        for row in 0..3 {
            m[row][col] = b[row];
        }
        det(m) / d
    };
    Some([column(0), column(1), column(2)])
}

/// A point on the path from `from` to `to` that keeps the fingertips clear
/// of the belt: straight up to a height above the belt, across, and down.
fn arc(from: Vec3, to: Vec3, t: f64) -> Vec3 {
    let high = from.y.max(to.y).max(BELT_TOP_M + ARC_CLEARANCE_M);
    let lerp = |a: f64, b: f64, u: f64| a + (b - a) * u.clamp(0.0, 1.0);
    if t < 0.35 {
        Vec3::new(from.x, lerp(from.y, high, t / 0.35), from.z)
    } else if t < 0.8 {
        let u = (t - 0.35) / 0.45;
        Vec3::new(lerp(from.x, to.x, u), high, lerp(from.z, to.z, u))
    } else {
        Vec3::new(to.x, lerp(high, to.y, (t - 0.8) / 0.2), to.z)
    }
}

fn smoothstep(value: f64) -> f64 {
    let value = value.clamp(0.0, 1.0);
    value * value * (3.0 - 2.0 * value)
}

/// What the inspection cell is doing.
#[derive(Clone, Copy, Debug, PartialEq)]
enum Stage {
    /// The belt runs until part `index` reaches the inspection point.
    Convey { index: usize },
    /// The fingertips move over the part, from the point `from`.
    Reach { index: usize, step: u64, from: Vec3 },
    /// The hand descends until it touches.
    Descend { index: usize, depth_m: f64 },
    /// The hand rests on the part.
    Hold { index: usize, step: u64 },
    /// The fingertips return from the point `from`, then the arm hangs.
    Retract { index: usize, step: u64, from: Vec3 },
    /// All parts inspected; the belt carries them on.
    Done,
}

impl Stage {
    fn label(self) -> String {
        match self {
            Stage::Convey { index } => format!("convey-part-{index}"),
            Stage::Reach { index, .. } => format!("reach-part-{index}"),
            Stage::Descend { index, .. } => format!("descend-part-{index}"),
            Stage::Hold { index, .. } => format!("touch-part-{index}"),
            Stage::Retract { index, .. } => format!("retract-part-{index}"),
            Stage::Done => "convey-away".into(),
        }
    }
}

/// The lowest corner of the fingertip contact box, in the forearm's frame,
/// for the forearm's current orientation in the simulation: the part of the
/// fingertips that meets a surface below them first.
fn lowest_tip_local(sim: &UrdfSceneSim) -> Result<Vec3> {
    let forearm = sim.named_transform(TIP_LINK).context("forearm")?;
    let down = forearm.rotation.inverse() * Vec3::NEG_Y;
    let half = Vec3::from(FINGERTIP_BOX_SIZE_M) * 0.5;
    Ok(Vec3::from(FINGERTIP_BOX_CENTER_M)
        + Vec3::new(
            half.x * down.x.signum(),
            half.y * down.y.signum(),
            half.z * down.z.signum(),
        ))
}

/// The fingertips' lowest point in the simulation.
fn sim_tip(sim: &UrdfSceneSim) -> Result<Vec3> {
    let forearm = sim.named_transform(TIP_LINK).context("forearm")?;
    Ok(forearm.translation + forearm.rotation * lowest_tip_local(sim)?)
}

fn translation(sim: &UrdfSceneSim, name: &str) -> Result<Vec3> {
    let (x, y, z) = sim
        .named_translation_m(name)
        .with_context(|| format!("missing {name}"))?;
    Ok(Vec3::new(x, y, z))
}

#[allow(clippy::too_many_lines)]
fn rollout(capture: bool) -> Result<Rollout> {
    let mut sim = UrdfSceneSim::from_scene_path(&unitree_g1_factory_scene_path())
        .context("load the G1 factory scene")?;
    let mut belt = Belt::spawn(&mut sim);
    let parts: Vec<String> = (0..PART_START_Z_M.len())
        .map(|index| spawn_part(&mut sim, index))
        .collect();
    // The URDF's hand collision is its mesh, and this scene loads the G1
    // without mesh colliders; the fingertips get a box. It rides the forearm
    // (the elbow link): the rubber hand only rolls about the forearm's axis,
    // along which the fingertips lie, and giving the hand link a collider of
    // its own made the backend add it as a new body that toppled the robot.
    anyhow::ensure!(
        sim.add_named_box_contact_proxy_at(TIP_LINK, FINGERTIP_BOX_SIZE_M, FINGERTIP_BOX_CENTER_M),
        "could not give the G1's right fingertips a contact box"
    );
    let arm = Arm::new(&sim, "right")?;
    let left_arm = Arm::new(&sim, "left")?;
    let q0 = arm.joints(&sim)?;
    let rest = [
        arm.solve(&sim, &q0, REST_POINT_M[0])?,
        left_arm.solve(&sim, &q0, REST_POINT_M[1])?,
    ];
    let hanging = [
        arm.dofs.map(|dof| q0[dof]),
        left_arm.dofs.map(|dof| q0[dof]),
    ];
    settle(&mut sim, &hanging, &rest);
    let hanging = rest[0];
    let rest_point = REST_POINT_M[0];
    let initial_digest = hash_physics_state(sim.world());
    let pelvis = |sim: &UrdfSceneSim| translation(sim, "pelvis");
    let start = pelvis(&sim)?;
    let mut evidence = vec![PartEvidence::default(); parts.len()];
    let mut part_before_touch = Vec3::ZERO;
    let mut arm_target = hanging;
    // Integral of the measured fingertip error: the arm's position motors
    // sag under gravity, so the fingertip lands below and beside where the
    // kinematic model puts it.
    let mut bias = Vec3::ZERO;
    let mut stage = Stage::Convey { index: 0 };
    let (mut max_drift_m, mut max_tilt_deg) = (0.0_f64, 0.0_f64);
    let mut frames = Vec::new();
    let mut cache = MeshRenderCache::new();
    let mut mesh_items = 0;
    for step in 1..=CAPTURE_STEPS {
        // Belt: run unless a part is being inspected; slow to stop the
        // current part at the inspection point.
        let run_m_s = match stage {
            Stage::Convey { index } => {
                let z = translation(&sim, &parts[index])?.z;
                let remaining = INSPECT_Z_M - z;
                if remaining <= 0.0 {
                    0.0
                } else {
                    // The speed from which the belt can still stop in the
                    // remaining distance.
                    BELT_SPEED_M_S.min((2.0 * BELT_ACCEL_M_S2 * remaining).sqrt())
                }
            }
            Stage::Done => BELT_SPEED_M_S,
            _ => 0.0,
        };
        belt.drive(&mut sim, run_m_s);

        let targets = pose(arm_target, rest[1]);
        sim.step_joint_position_targets(&targets);

        // Advance the inspection.
        stage = match stage {
            Stage::Convey { index } => {
                let z = translation(&sim, &parts[index])?.z;
                if belt.speed_m_s.abs() < 1e-3 && z >= INSPECT_Z_M - 0.03 {
                    evidence[index].stop_error_m = (z - INSPECT_Z_M).abs();
                    Stage::Reach {
                        index,
                        step: 0,
                        from: arm.touch_point(&sim, &arm.joints(&sim)?)?,
                    }
                } else {
                    Stage::Convey { index }
                }
            }
            Stage::Reach { index, step, from } => {
                let part = translation(&sim, &parts[index])?;
                let hover = part + Vec3::new(0.0, PART_HALF_M.y + HOVER_M, 0.0);
                let t = smoothstep(f64::from(step as u32 + 1) / REACH_STEPS as f64);
                let q = arm.joints(&sim)?;
                let goal = arc(from, hover, t);
                bias += (goal - sim_tip(&sim)?) * TIP_CORRECTION_GAIN;
                arm_target = arm.solve(&sim, &q, goal + bias)?;
                if step + 1 >= REACH_STEPS {
                    part_before_touch = part;
                    Stage::Descend {
                        index,
                        depth_m: HOVER_M,
                    }
                } else {
                    Stage::Reach {
                        index,
                        step: step + 1,
                        from,
                    }
                }
            }
            Stage::Descend { index, depth_m } => {
                if sim.named_entities_in_contact(TIP_LINK, &parts[index]) {
                    evidence[index].touched_at_step = Some(step);
                    let top =
                        translation(&sim, &parts[index])? + Vec3::new(0.0, PART_HALF_M.y, 0.0);
                    let tip = sim_tip(&sim)?;
                    evidence[index].tip_offset_m = (tip.x - top.x).hypot(tip.z - top.z);
                    evidence[index].tip_height_m = tip.y - top.y;
                    // Rest the fingertips on the part where they met it.
                    let part = translation(&sim, &parts[index])?;
                    let rest_at = part + Vec3::new(0.0, PART_HALF_M.y + depth_m + HOLD_LIFT_M, 0.0);
                    let q = arm.joints(&sim)?;
                    arm_target = arm.solve(&sim, &q, rest_at + bias)?;
                    Stage::Hold { index, step: 0 }
                } else if depth_m <= -MAX_PRESS_M {
                    // Went the full press depth without a reported contact.
                    Stage::Retract {
                        index,
                        step: 0,
                        from: arm.touch_point(&sim, &arm.joints(&sim)?)?,
                    }
                } else {
                    let depth_m = depth_m - DESCENT_M_S * DT_S;
                    let part = translation(&sim, &parts[index])?;
                    let point = part + Vec3::new(0.0, PART_HALF_M.y + depth_m, 0.0);
                    let q = arm.joints(&sim)?;
                    bias += (point - sim_tip(&sim)?) * TIP_CORRECTION_GAIN;
                    arm_target = arm.solve(&sim, &q, point + bias)?;
                    Stage::Descend { index, depth_m }
                }
            }
            Stage::Hold { index, step } => {
                if sim.named_entities_in_contact(TIP_LINK, &parts[index]) {
                    evidence[index].contact_steps += 1;
                }
                let moved = translation(&sim, &parts[index])? - part_before_touch;
                evidence[index].pushed_m = evidence[index].pushed_m.max(moved.length());
                if step + 1 >= HOLD_STEPS {
                    Stage::Retract {
                        index,
                        step: 0,
                        from: arm.touch_point(&sim, &arm.joints(&sim)?)?,
                    }
                } else {
                    Stage::Hold {
                        index,
                        step: step + 1,
                    }
                }
            }
            Stage::Retract { index, step, from } => {
                // Back along the same raised arc to the hanging hand's
                // position, then let the joints settle to the hanging pose.
                let t = smoothstep(f64::from(step as u32 + 1) / RETRACT_STEPS as f64);
                let q = arm.joints(&sim)?;
                arm_target = if t < 1.0 {
                    arm.solve(&sim, &q, arc(from, rest_point, t))?
                } else {
                    hanging
                };
                if step + 1 >= RETRACT_STEPS + SETTLE_ARM_STEPS {
                    if index + 1 < parts.len() {
                        Stage::Convey { index: index + 1 }
                    } else {
                        Stage::Done
                    }
                } else {
                    Stage::Retract {
                        index,
                        step: step + 1,
                        from,
                    }
                }
            }
            Stage::Done => Stage::Done,
        };

        let now = pelvis(&sim)?;
        max_drift_m = max_drift_m.max((now - start).x.hypot((now - start).z));
        if let Some(transform) = sim.named_transform("torso_link") {
            let up = transform.rotation * Vec3::Z;
            max_tilt_deg = max_tilt_deg.max(up.y.clamp(-1.0, 1.0).acos().to_degrees());
        }
        if capture && step % CAPTURE_STRIDE == 0 && frames.len() < CAPTURE_FRAME_COUNT {
            let lamps: Vec<bool> = evidence
                .iter()
                .map(|part| part.touched_at_step.is_some())
                .collect();
            let (scene, current_mesh_items) =
                render_scene(&sim, &mut cache, &belt, &parts, &lamps, stage)?;
            mesh_items = mesh_items.max(current_mesh_items);
            frames.push(CaptureFrame {
                step,
                phase: stage.label(),
                scene,
            });
        }
    }
    if !capture {
        let lamps = vec![false; parts.len()];
        let (_, current_mesh_items) = render_scene(&sim, &mut cache, &belt, &parts, &lamps, stage)?;
        mesh_items = current_mesh_items;
    }
    anyhow::ensure!(
        mesh_items >= 20,
        "factory render resolved only {mesh_items} G1 mesh items"
    );
    let parts_left_on_belt = parts
        .iter()
        .filter(|name| {
            translation(&sim, name).is_ok_and(|p| {
                (p.x - BELT_X_M).abs() < BELT_HALF_WIDTH_M && p.y > BELT_TOP_M - 0.02
            })
        })
        .count();
    Ok(Rollout {
        initial_digest,
        final_digest: hash_physics_state(sim.world()),
        parts: evidence,
        parts_left_on_belt,
        max_drift_m,
        max_tilt_deg,
        mesh_items,
        frames,
    })
}

fn render_scene(
    sim: &UrdfSceneSim,
    cache: &mut MeshRenderCache,
    belt: &Belt,
    parts: &[String],
    lamps: &[bool],
    stage: Stage,
) -> Result<(RenderScene, usize)> {
    let mut scene = build_visual_render_scene(sim.world());
    // The belt and the parts carry colliders but no visuals, and the scene
    // builder draws such bodies as plain collider boxes; they are drawn in
    // detail below instead.
    let own: Vec<Vec3> = std::iter::once("conveyor_belt")
        .chain(parts.iter().map(String::as_str))
        .filter_map(|name| sim.named_transform(name).map(|t| t.translation))
        .collect();
    scene.items.retain(|item| {
        !own.iter()
            .any(|at| (item.transform.translation - *at).length() < 1e-9)
    });
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
    push_conveyor(&mut scene, belt.travelled_m);
    for (index, name) in parts.iter().enumerate() {
        if let Some(transform) = sim.named_transform(name) {
            push_part(&mut scene, transform.translation, transform.rotation, index);
        }
    }
    let active = match stage {
        Stage::Reach { index, .. }
        | Stage::Descend { index, .. }
        | Stage::Hold { index, .. }
        | Stage::Retract { index, .. } => Some(index),
        _ => None,
    };
    push_inspection_lamps(&mut scene, lamps, active);
    push_dressing(&mut scene);
    Ok((scene, mesh_items))
}

/// The conveyor: a frame on legs, end rollers, side guards, and the belt,
/// whose cleats move with how far the simulated belt has travelled.
fn push_conveyor(scene: &mut RenderScene, travelled_m: f64) {
    const FRAME: [f32; 4] = [0.30, 0.34, 0.38, 1.0];
    const BELT: [f32; 4] = [0.07, 0.075, 0.08, 1.0];
    const CLEAT: [f32; 4] = [0.18, 0.19, 0.20, 1.0];
    const GUARD: [f32; 4] = [0.92, 0.72, 0.12, 1.0];
    let (z0, z1) = CONVEYOR_Z_M;
    let length = z1 - z0;
    let mid = 0.5 * (z0 + z1);
    push_box_material(
        scene,
        Vec3::new(BELT_X_M, BELT_TOP_M - 0.004, mid),
        Vec3::new(2.0 * BELT_HALF_WIDTH_M, 0.008, length),
        Quat::IDENTITY,
        BELT,
        PbrMaterial::new(BELT, 0.8, 0.0, [0.0; 3]),
    );
    let spacing = 0.12;
    let offset = travelled_m.rem_euclid(spacing);
    let mut z = z0 + offset;
    while z < z1 {
        push_box(
            scene,
            Vec3::new(BELT_X_M, BELT_TOP_M + 0.002, z),
            Vec3::new(2.0 * BELT_HALF_WIDTH_M - 0.02, 0.004, 0.012),
            CLEAT,
        );
        z += spacing;
    }
    for side in [-1.0, 1.0] {
        let x = BELT_X_M + side * (BELT_HALF_WIDTH_M + 0.03);
        push_box_material(
            scene,
            Vec3::new(x, BELT_TOP_M - 0.05, mid),
            Vec3::new(0.05, 0.10, length + 0.12),
            Quat::IDENTITY,
            FRAME,
            PbrMaterial::new(FRAME, 0.4, 0.6, [0.0; 3]),
        );
        push_box(
            scene,
            Vec3::new(x, BELT_TOP_M + 0.008, mid),
            Vec3::new(0.012, 0.016, length),
            GUARD,
        );
        for z in [z0 + 0.1, mid, z1 - 0.1] {
            push_box(
                scene,
                Vec3::new(x, 0.5 * (BELT_TOP_M - 0.1), z),
                Vec3::new(0.05, BELT_TOP_M - 0.1, 0.05),
                FRAME,
            );
        }
    }
    for z in [z0, z1] {
        push_cylinder(
            scene,
            Vec3::new(BELT_X_M, BELT_TOP_M - 0.035, z),
            0.035,
            2.0 * BELT_HALF_WIDTH_M + 0.04,
            Quat::from_rotation_arc(Vec3::Z, Vec3::X),
            [0.55, 0.57, 0.6, 1.0],
        );
    }
    // Drive motor at the far end.
    push_box(
        scene,
        Vec3::new(
            BELT_X_M + BELT_HALF_WIDTH_M + 0.12,
            BELT_TOP_M - 0.08,
            z1 - 0.05,
        ),
        Vec3::new(0.12, 0.12, 0.14),
        [0.15, 0.32, 0.55, 1.0],
    );
}

/// A part: a machined block with a darker top plate and a bore.
fn push_part(scene: &mut RenderScene, at: Vec3, rotation: Quat, index: usize) {
    let tones = [
        [0.72, 0.74, 0.78, 1.0],
        [0.80, 0.52, 0.18, 1.0],
        [0.40, 0.58, 0.72, 1.0],
    ];
    let tone = tones[index % tones.len()];
    push_box_material(
        scene,
        at,
        PART_HALF_M * 2.0,
        rotation,
        tone,
        PbrMaterial::new(tone, 0.35, 0.7, [0.0; 3]),
    );
    push_cylinder(
        scene,
        at + rotation * Vec3::new(0.0, PART_HALF_M.y + 0.001, 0.0),
        0.012,
        0.004,
        rotation * Quat::from_rotation_arc(Vec3::Z, Vec3::Y),
        [0.08, 0.08, 0.09, 1.0],
    );
}

/// A stack of three lamps on a post at the inspection point: one per part,
/// amber until the hand is reported touching that part, green after; the
/// one being inspected wears a white ring.
fn push_inspection_lamps(scene: &mut RenderScene, lamps: &[bool], active: Option<usize>) {
    let base = Vec3::new(
        BELT_X_M + BELT_HALF_WIDTH_M + 0.12,
        BELT_TOP_M,
        INSPECT_Z_M + 0.6,
    );
    push_box(
        scene,
        base + Vec3::new(0.0, 0.2, 0.0),
        Vec3::new(0.03, 0.4, 0.03),
        [0.25, 0.27, 0.3, 1.0],
    );
    for (index, touched) in lamps.iter().enumerate() {
        let colour = if *touched {
            [0.15, 1.0, 0.45]
        } else {
            [1.0, 0.62, 0.1]
        };
        let at = base + Vec3::new(0.0, 0.42 + 0.07 * index as f64, 0.0);
        push_cylinder(
            scene,
            at,
            0.035,
            0.06,
            Quat::from_rotation_arc(Vec3::Z, Vec3::Y),
            [colour[0], colour[1], colour[2], 1.0],
        );
        if let Some(last) = scene.items.last_mut() {
            last.material =
                PbrMaterial::new([colour[0], colour[1], colour[2], 1.0], 0.3, 0.0, colour);
        }
        if active == Some(index) {
            push_cylinder(
                scene,
                at,
                0.042,
                0.012,
                Quat::from_rotation_arc(Vec3::Z, Vec3::Y),
                [0.95, 0.95, 0.95, 1.0],
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
    const BACK_WALL: [f32; 4] = [0.28, 0.32, 0.38, 1.0];

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
    // A concrete slab around the tiles, and the back wall carried up and
    // across.
    push_box(
        scene,
        Vec3::new(0.3, 0.004, 0.2),
        Vec3::new(4.2, 0.008, 3.4),
        CONCRETE,
    );
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

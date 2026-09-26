//! OpenArm v2 bimanual-control showcase source and capture.
//!
//! The two official arms run a purposeful pick / relay / place cycle: the
//! right arm reaches down to a block resting on the workbench, closes its
//! gripper, lifts it, carries it to a mid-table relay point and sets it back
//! down, then withdraws; only once it is clear does the left arm reach down,
//! close on the same block, lift it, carry it to a marked target pad, and
//! set it down. Every keypose below is an inverse-kinematics solution
//! (solved offline against the real URDF chain, see
//! `docs/media/showcase-openarm.json` provenance) so the commanded
//! end-effector path actually reaches the block, the relay point, and the
//! pad instead of an arbitrary joint-space sweep. The block is a real
//! dynamic Rapier body throughout: each contact-gated grasp (two distinct
//! fingertip contacts on the block, see
//! [`rne_ai::UrdfSceneSim::named_child_has_distinct_dual_contact`]) switches
//! it to a kinematic pose-follower parented to whichever gripper is holding
//! it, and each release hands it back to ordinary dynamics so it actually
//! drops and settles under gravity, both at the relay point and on the pad.

use super::media::{
    capture_frames, push_box, push_box_material, CameraEvidence, CaptureFrame, ShowcaseMetadata,
    SimulationEvidence,
};
use anyhow::{Context, Result};
use rne_ai::{
    build_visual_render_scene, UrdfJointFeedbackSensorConfig, UrdfJointPdEffortTarget,
    UrdfJointPositionTarget, UrdfSceneSim,
};
use rne_data::{
    DataBus, Frame, InMemoryDataBus, JointCoordinateFeedback, JointFeedback, JointFeedbackStatus,
    StreamId,
};
use rne_math::{Quat, Vec3};
use rne_physics::{hash_physics_state, CollisionGroups};
use rne_render::{MeshRenderCache, PbrMaterial, RenderScene, VisualShape};
use rne_render_wgpu::CameraOrbit;
use rne_sensor::JointFeedbackFault;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::{Path, PathBuf};

const ENVIRONMENT_ID: &str = "openarm";
const SUBJECT: &str = "Official OpenArm v2 bimanual pick, relay, and place";
const CAPTURE_STEPS: u64 = 1_400;
const CAPTURE_FRAME_COUNT: usize = 38;
const CAPTURE_STRIDE: u64 = CAPTURE_STEPS / CAPTURE_FRAME_COUNT as u64;
const PHYSICS_SUBSTEPS_PER_CONTROL_STEP: usize = 19;
const JOINT_FEEDBACK_STREAM: StreamId = StreamId::new(9_090);
// A closer, more side-on shot (vs. a distant near-frontal one) so the arms,
// block, and pad fill most of the frame instead of leaving the composition
// dominated by empty floor and background. The two arm base_link origins
// are only 6.2 cm apart in world Z, but that understates the real shoulder
// spread: joint 1's own fixed offset (URDF `openarm_*_joint1` origin, 6.25 cm
// further out from each base) puts the actual shoulder pivots ~18.7 cm apart,
// widening to ~31 cm between the wrists in the shared, mirrored `READY`
// pose. A near-0 yaw still reads that spread mostly as depth (the two arms
// visually fuse into one silhouette), so yaw stays pushed round toward
// side-on to turn it into visible screen-space width instead.
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.85,
    // Larger pitch is nearer horizontal here. 0.75 looked down steeply enough
    // to flatten the arms onto each other however far apart they actually are;
    // a three-quarter view keeps the near arm in front of the far one rather
    // than on top of it.
    pitch_rad: 1.08,
    // 1.05 m framed the torso out of the top and pushed the robot against the
    // right edge, leaving most of the image empty table.
    distance_m: 1.26,
};

/// Name of the dynamic block the right arm picks up and the left arm places.
const BLOCK_NAME: &str = "openarm_part_right";
/// Palm-equivalent link each gripper's grasp follower frame is parented to.
const RIGHT_PALM: &str = "openarm_right_ee_base_link";
const LEFT_PALM: &str = "openarm_left_ee_base_link";
/// The official finger links carry only a visual-quality collision mesh
/// (`mesh_collisions = false` in the robot manifests deliberately drops mesh
/// colliders, so the fingers otherwise have none). Each of these is a small
/// fixed sensor box parented to the real fingertip via
/// [`rne_ai::UrdfSceneSim::add_named_child_box_sensor`], offset to the same
/// point [`finger_aperture_m`] already measures. It rides along with the
/// finger exactly and is the real collider the grasp gate's contact query
/// reads; it is filtered back out of the render scene (see
/// [`FINGER_SENSOR_SIZE_M`]) so it never appears as a stray box.
const RIGHT_FINGER1: &str = "openarm_right_ee_link1_tip_sensor";
const RIGHT_FINGER2: &str = "openarm_right_ee_link2_tip_sensor";
const LEFT_FINGER1: &str = "openarm_left_ee_link1_tip_sensor";
const LEFT_FINGER2: &str = "openarm_left_ee_link2_tip_sensor";
/// Local offset of each fingertip sensor, matching [`finger_aperture_m`].
const FINGER_TIP_OFFSET_M: [f64; 3] = [0.0, 0.0, -0.065];
/// Generously sized so a few centimeters of IK/tracking slack still closes
/// the contact gate; deliberately distinctive so [`render_scene`] can filter
/// it out of the visible scene by exact size match.
const FINGER_SENSOR_SIZE_M: [f64; 3] = [0.032, 0.030, 0.034];
const RIGHT_GRASP_FRAME: &str = "openarm_right_grasp_frame";
const LEFT_GRASP_FRAME: &str = "openarm_left_grasp_frame";
/// Steps during which the closing right gripper is checked for the
/// contact-gated pick grasp off the workbench.
const RIGHT_GRASP_WINDOW: (u64, u64) = (255, 320);
/// Step at which the right gripper releases the block back to ordinary
/// dynamics at the mid-table relay point. Lands exactly where `RIGHT_MID_OPEN`
/// finishes opening the fingers, so the block drops the instant it is clear
/// of them instead of being dragged a few more steps by an already-open hand.
const RIGHT_RELEASE_STEP: u64 = 620;
/// Steps during which the closing left gripper is checked for the
/// contact-gated re-grasp at the relay point.
const LEFT_REGRASP_WINDOW: (u64, u64) = (835, 890);
/// Step at which the left gripper releases the block back to ordinary
/// dynamics on the target pad, matching `PLACE_L_OPEN`'s finger timing.
const LEFT_RELEASE_STEP: u64 = 1_210;

const LEFT_LINKS: [&str; 9] = [
    "openarm_left_link1",
    "openarm_left_link2",
    "openarm_left_link3",
    "openarm_left_link4",
    "openarm_left_link5",
    "openarm_left_link6",
    "openarm_left_ee_base_link",
    "openarm_left_ee_link1",
    "openarm_left_ee_link2",
];
const RIGHT_LINKS: [&str; 9] = [
    "openarm_right_link1",
    "openarm_right_link2",
    "openarm_right_link3",
    "openarm_right_link4",
    "openarm_right_link5",
    "openarm_right_link6",
    "openarm_right_ee_base_link",
    "openarm_right_ee_link1",
    "openarm_right_ee_link2",
];

#[derive(Clone, Copy, Debug)]
struct ActuatorProfile {
    stiffness_nm_per_rad: f64,
    damping_nm_s_per_rad: f64,
    max_effort_nm: f64,
    max_velocity_rad_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
struct ControlFrameTelemetry {
    observation_age_ticks: u64,
    tracking_error_rad: f64,
    effort_utilization: f64,
    saturated_fraction: f64,
}

#[derive(Clone, Copy, Debug)]
struct BimanualPose {
    left: [f64; 9],
    right: [f64; 9],
}

/// A single arm's 9 commanded joint positions: 7 arm joints followed by the
/// two mimicked finger joints.
type ArmPose = [f64; 9];

// Every named pose below is an inverse-kinematics solution against the real
// OpenArm v2 URDF chain (solved offline; see the module doc comment), so the
// commanded fingertip midpoint actually lands on the block or the pad instead
// of an arbitrary joint-space guess. `READY` is the shared tucked resting
// pose both arms start and end in.
const READY_R: ArmPose = [
    0.0,
    0.0,
    0.0,
    std::f64::consts::FRAC_PI_2,
    0.0,
    0.0,
    0.0,
    -0.55,
    -0.55,
];
const HOVER_PICK_R: ArmPose = [
    0.1520, 1.1987, -0.0588, 1.1824, -0.1735, 0.3151, 1.0023, -0.55, -0.55,
];
const PICK_R_OPEN: ArmPose = [
    0.4922, 0.8083, -0.2360, 0.5538, -0.3318, 0.3847, 0.8179, -0.55, -0.55,
];
const PICK_R_CLOSED: ArmPose = [
    0.6389, 0.8025, -0.2978, 0.4220, -0.3782, 0.4661, 0.9659, -0.08, -0.08,
];
const LIFT_R_CLOSED: ArmPose = [
    0.7663, 1.0899, -0.4171, 0.7954, -0.4356, 0.4834, 1.5374, -0.08, -0.08,
];
// A true mid-air hand-to-hand meet left the two wrists visually fused into
// one silhouette from the showcase camera, no matter how the elbows were
// posed: the two arm bases sit only 6.2 cm apart, so any point both
// grippers can reach at once puts their hands almost exactly on top of each
// other on screen. Instead the right arm carries the block to a mid-table
// relay point and sets it down there (a real gravity drop, released back to
// ordinary dynamics), withdraws, and only then does the left arm reach down
// and pick it up on its own -- two clearly separated, individually legible
// pick/place actions instead of one crowded simultaneous one.
const RIGHT_HOVER_MID_CLOSED: ArmPose = [
    0.1551,
    1.5063,
    -0.9782,
    2.4435,
    -0.0384,
    0.5128,
    std::f64::consts::FRAC_PI_2,
    -0.08,
    -0.08,
];
const RIGHT_MID_CLOSED: ArmPose = [
    -0.1308, 1.2983, -1.2026, 2.2172, -0.5453, 0.6331, 0.9176, -0.08, -0.08,
];
const RIGHT_MID_OPEN: ArmPose = [
    -0.2150, 1.2996, -1.2636, 2.2122, -0.7099, 0.6862, 0.7098, -0.55, -0.55,
];
const RIGHT_HOVER_MID_OPEN: ArmPose = [
    0.1214, 1.6356, -1.0521, 2.4435, 0.1017, 0.4521, 1.2856, -0.55, -0.55,
];

const READY_L: ArmPose = [
    0.0,
    0.0,
    0.0,
    std::f64::consts::FRAC_PI_2,
    0.0,
    0.0,
    0.0,
    0.55,
    0.55,
];
const LEFT_HOVER_MID_OPEN: ArmPose = [
    0.2118, 0.1745, 1.4439, 1.7356, -0.0055, -0.7345, 0.0178, 0.55, 0.55,
];
const LEFT_MID_OPEN: ArmPose = [
    0.0279, 0.1745, 1.5447, 1.2975, -0.0074, -0.5384, 0.0126, 0.55, 0.55,
];
const LEFT_MID_CLOSED: ArmPose = [
    0.0078, 0.1745, 1.5645, 1.3640, -0.0101, -0.6951, 0.0110, 0.08, 0.08,
];
const LEFT_HOVER_MID_CLOSED: ArmPose = [
    -0.2044,
    0.1745,
    std::f64::consts::FRAC_PI_2,
    1.7663,
    0.1181,
    -std::f64::consts::FRAC_PI_4,
    -0.3929,
    0.08,
    0.08,
];
const HOVER_PLACE_L_CLOSED: ArmPose = [
    1.0798,
    0.1745,
    std::f64::consts::FRAC_PI_2,
    1.1568,
    -0.1057,
    -0.3194,
    0.3637,
    0.08,
    0.08,
];
const PLACE_L_CLOSED: ArmPose = [
    0.7501, 0.1745, 1.5697, 0.8454, -0.1359, -0.0208, 0.0404, 0.08, 0.08,
];
const PLACE_L_OPEN: ArmPose = [
    0.7501, 0.1745, 1.5697, 0.8454, -0.1359, -0.0208, 0.0404, 0.55, 0.55,
];
const HOVER_PLACE_L_OPEN: ArmPose = [
    1.0798,
    0.1745,
    std::f64::consts::FRAC_PI_2,
    1.1568,
    -0.1057,
    -0.3194,
    0.3637,
    0.55,
    0.55,
];

/// Right-arm keyframes: reach out and down to the block, close, lift, carry
/// to the mid-table relay point, set it down, withdraw, and return home.
const RIGHT_KEYFRAMES: &[(u64, ArmPose)] = &[
    (0, READY_R),
    (150, HOVER_PICK_R),
    (260, PICK_R_OPEN),
    (320, PICK_R_CLOSED),
    (420, LIFT_R_CLOSED),
    (520, RIGHT_HOVER_MID_CLOSED),
    (580, RIGHT_MID_CLOSED),
    (620, RIGHT_MID_OPEN),
    (680, RIGHT_HOVER_MID_OPEN),
    (760, READY_R),
    (CAPTURE_STEPS, READY_R),
];
/// Left-arm keyframes: wait for the right arm to set the block down and
/// withdraw, reach down to the relay point, close on it, lift, carry to the
/// target pad, place it, release, withdraw, and return home.
const LEFT_KEYFRAMES: &[(u64, ArmPose)] = &[
    (0, READY_L),
    (760, READY_L),
    (800, LEFT_HOVER_MID_OPEN),
    (840, LEFT_MID_OPEN),
    (890, LEFT_MID_CLOSED),
    (970, LEFT_HOVER_MID_CLOSED),
    (1_080, HOVER_PLACE_L_CLOSED),
    (1_150, PLACE_L_CLOSED),
    (1_210, PLACE_L_OPEN),
    (1_270, HOVER_PLACE_L_OPEN),
    (CAPTURE_STEPS, READY_L),
];

/// Evaluates a per-arm keyframe schedule at `step` with quintic-smoothed
/// blending between the surrounding keyframes.
fn arm_pose_at(schedule: &[(u64, ArmPose)], step: u64) -> ArmPose {
    for window in schedule.windows(2) {
        let (start_step, start_pose) = window[0];
        let (end_step, end_pose) = window[1];
        if step <= end_step {
            let alpha = if end_step > start_step {
                (step - start_step) as f64 / (end_step - start_step) as f64
            } else {
                1.0
            };
            return blend_arm_pose(start_pose, end_pose, smooth(alpha));
        }
    }
    schedule.last().map_or(READY_R, |(_, pose)| *pose)
}

fn blend_arm_pose(from: ArmPose, to: ArmPose, alpha: f64) -> ArmPose {
    std::array::from_fn(|i| from[i] + (to[i] - from[i]) * alpha)
}

/// Runs a force-limited OpenArm bimanual joint-space cycle and optionally
/// renders evenly sampled post-step states using the same Rapier rollout.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(repo_root, capture)?;
    let replay = rollout(repo_root, false)?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "OpenArm replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    anyhow::ensure!(
        first.actuated_joint_count == 18,
        "OpenArm must expose 18 actuators"
    );
    anyhow::ensure!(
        first.mesh_items >= 45,
        "OpenArm resolved only {} visual mesh parts",
        first.mesh_items
    );
    anyhow::ensure!(
        first.left_end_effector_travel_m >= 0.16 && first.right_end_effector_travel_m >= 0.16,
        "OpenArm end effectors did not visibly move: left={:.3} m right={:.3} m",
        first.left_end_effector_travel_m,
        first.right_end_effector_travel_m
    );
    anyhow::ensure!(
        first.max_final_proximal_joint_error_rad <= 0.13,
        "OpenArm final proximal-joint tracking error exceeded 0.13 rad: {:.4}",
        first.max_final_proximal_joint_error_rad
    );
    anyhow::ensure!(
        first.left_gripper_aperture_change_m >= 0.015
            && first.right_gripper_aperture_change_m >= 0.015,
        "OpenArm grippers did not visibly actuate: left={:.4} m right={:.4} m",
        first.left_gripper_aperture_change_m,
        first.right_gripper_aperture_change_m
    );
    anyhow::ensure!(
        first.feedback_decisions >= CAPTURE_STEPS - 2,
        "OpenArm typed feedback controlled only {} of {} steps",
        first.feedback_decisions,
        CAPTURE_STEPS
    );
    anyhow::ensure!(
        first.sensor_samples == CAPTURE_STEPS,
        "OpenArm joint sensor emitted {} of {} samples",
        first.sensor_samples,
        CAPTURE_STEPS
    );
    anyhow::ensure!(
        first.max_observation_age_ticks == first.fixed_delta_ticks,
        "OpenArm feedback age must be exactly one control period: age={} period={}",
        first.max_observation_age_ticks,
        first.fixed_delta_ticks
    );
    anyhow::ensure!(
        first.pick_grasp_step.is_some(),
        "OpenArm right gripper never achieved a contact-gated pick grasp on the block"
    );
    anyhow::ensure!(
        first.regrasp_step.is_some(),
        "OpenArm left gripper never achieved a contact-gated re-grasp at the relay point"
    );
    anyhow::ensure!(
        first.released,
        "OpenArm left gripper never released the block back to ordinary dynamics"
    );
    if capture {
        anyhow::ensure!(
            first.frames.len() == CAPTURE_FRAME_COUNT,
            "OpenArm capture must contain {CAPTURE_FRAME_COUNT} frames"
        );
    }

    let capture_evidence = if capture {
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &first.frames,
            CameraOrbit {
                // The arms sit at y = 0.698; focusing 0.20 m below them and
                // 0.25 m along +z put the robot off to one side of its own
                // showcase. Focus between the shoulders and the work area.
                focus: Vec3::new(0.0, 0.60, 0.12),
                yaw_rad: CAMERA.yaw_rad,
                pitch_rad: CAMERA.pitch_rad,
                distance_m: CAMERA.distance_m,
            },
            // A bright, pleasant lab tone instead of a near-black void: the
            // official robot mesh materials are unaffected, so this only
            // lifts the backdrop the grey links read against.
            [0.72, 0.78, 0.85, 1.0],
            22,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "Official OpenArm link visuals, the workbench dressing, and the latency/error/effort HUD are rebuilt from the same post-Rapier fixed-step state; the carried block is a real dynamic Rapier body that switches to a kinematic pose-follower only after a contact-gated grasp and is handed back to ordinary dynamics on release.",
        simulation: SimulationEvidence {
            scenario: "OpenArm v2 bimanual pick / relay / place under delayed joint-feedback and portable PD-effort control",
            steps: CAPTURE_STEPS,
            initial_state_digest: first.initial_digest,
            final_state_digest: first.final_digest,
            replay_final_state_digest: replay.final_digest,
            replay_match: true,
            outcome: format!(
                "actuators={}; typed_feedback_decisions={}; sensor_samples={}; feedback_latency_ticks={}; max_tracking_error_rad={:.5}; saturated_channel_samples={}; left_ee_travel_m={:.4}; right_ee_travel_m={:.4}; left_gripper_aperture_change_m={:.4}; right_gripper_aperture_change_m={:.4}; max_final_proximal_joint_error_rad={:.5}; visual_mesh_parts={}; contact_gated_pick_grasp_step={}; contact_gated_relay_regrasp_step={}; block_released_to_dynamics={}",
                first.actuated_joint_count,
                first.feedback_decisions,
                first.sensor_samples,
                first.max_observation_age_ticks,
                first.max_tracking_error_rad,
                first.saturated_channel_samples,
                first.left_end_effector_travel_m,
                first.right_end_effector_travel_m,
                first.left_gripper_aperture_change_m,
                first.right_gripper_aperture_change_m,
                first.max_final_proximal_joint_error_rad,
                first.mesh_items,
                first.pick_grasp_step.unwrap_or_default(),
                first.regrasp_step.unwrap_or_default(),
                first.released
            ),
        },
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/robots/openarm_description/PROVENANCE.md",
            "assets/robots/openarm_description/LICENSE.openarm_description",
            "assets/robots/openarm_description/openarm_v2.rne.urdf",
            "assets/robots/openarm_v2_left.rne.robot.toml",
            "assets/robots/openarm_v2_right.rne.robot.toml",
            "assets/scenes/openarm_v2_showcase.rne.scene.toml",
            "examples/90_showcase_captures/openarm.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment openarm",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment openarm",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-openarm.json");
        fs::write(&path, to_vec_pretty(&metadata)?)
            .with_context(|| format!("write {}", path.display()))?;
    }
    Ok(metadata)
}

struct Rollout {
    initial_digest: u64,
    final_digest: u64,
    actuated_joint_count: usize,
    left_end_effector_travel_m: f64,
    right_end_effector_travel_m: f64,
    left_gripper_aperture_change_m: f64,
    right_gripper_aperture_change_m: f64,
    max_final_proximal_joint_error_rad: f64,
    fixed_delta_ticks: u64,
    feedback_decisions: u64,
    sensor_samples: u64,
    max_observation_age_ticks: u64,
    max_tracking_error_rad: f64,
    saturated_channel_samples: u64,
    mesh_items: usize,
    pick_grasp_step: Option<u64>,
    regrasp_step: Option<u64>,
    released: bool,
    frames: Vec<CaptureFrame>,
}

/// Which gripper (if any) currently holds the block as a kinematic
/// pose-follower. Transitions are gated on real fingertip contact.
///
/// The block passes through this state machine twice: the right gripper
/// picks it off the workbench, carries it, and sets it back down (real
/// gravity drop) at a mid-table relay point (`Carrier::None` again); only
/// then does the left gripper reach down, re-grasp it under its own
/// contact gate, carry it to the pad, and release it a final time.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Carrier {
    /// Resting freely under ordinary dynamics (workbench, relay point, or pad).
    None,
    /// Kinematically following the right gripper's grasp frame.
    Right,
    /// Kinematically following the left gripper's grasp frame.
    Left,
}

fn rollout(repo_root: &Path, capture: bool) -> Result<Rollout> {
    let scene_path = repo_root.join("assets/scenes/openarm_v2_showcase.rne.scene.toml");
    let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations(&scene_path, 16)
        .context("load official OpenArm v2 bimanual scene")?;
    configure_official_effort_actuators(&mut sim)?;
    disable_cross_arm_collisions(&mut sim)?;
    install_finger_tip_sensors(&mut sim)?;
    let fixed_delta_ticks = sim.fixed_delta().ticks();
    let link_order = LEFT_LINKS
        .iter()
        .chain(RIGHT_LINKS.iter())
        .map(|link| (*link).to_string())
        .collect::<Vec<_>>();
    sim.install_joint_feedback_sensor(UrdfJointFeedbackSensorConfig {
        sensor_name: "openarm_bimanual_joint_feedback".into(),
        link_names: link_order.clone(),
        update_rate_hz: 60.0,
        sample_period_ticks: Some(fixed_delta_ticks),
        phase_offset_ticks: fixed_delta_ticks,
        latency_ticks: fixed_delta_ticks,
        stream_id: JOINT_FEEDBACK_STREAM,
        fault: JointFeedbackFault::None,
    })
    .context("install OpenArm bimanual joint-feedback sensor")?;
    let observation = sim.observe();
    let initial_digest = hash_physics_state(sim.world());
    let left_start = link_position(&sim, "openarm_left_ee_base_link")?;
    let right_start = link_position(&sim, "openarm_right_ee_base_link")?;
    let mut left_travel_m: f64 = 0.0;
    let mut right_travel_m: f64 = 0.0;
    let mut left_aperture_min_m = f64::INFINITY;
    let mut left_aperture_max_m = 0.0_f64;
    let mut right_aperture_min_m = f64::INFINITY;
    let mut right_aperture_max_m = 0.0_f64;
    let mut frames = Vec::new();
    let mut cache = MeshRenderCache::new();
    let mut bus = InMemoryDataBus::new();
    let mut feedback_decisions = 0;
    let mut sensor_samples = 0;
    let mut max_observation_age_ticks = 0;
    let mut max_tracking_error_rad = 0.0_f64;
    let mut saturated_channel_samples = 0;
    let mut mesh_items = 0;
    let mut carrier = Carrier::None;
    let mut pick_grasp_step = None;
    let mut regrasp_step = None;
    let mut released = false;
    for step in 1..=CAPTURE_STEPS {
        let pose = commanded_pose(step);
        let reference_targets = targets_for_pose(&pose);
        let visible_feedback =
            bus.latest_available::<JointFeedback>(JOINT_FEEDBACK_STREAM, sim.sim_time());
        let (controller_targets, observation_age_ticks) = feedback_adjusted_targets(
            &reference_targets,
            visible_feedback.as_ref(),
            sim.sim_time().ticks(),
            &link_order,
        )?;
        if visible_feedback.is_some() {
            feedback_decisions += 1;
            max_observation_age_ticks = max_observation_age_ticks.max(observation_age_ticks);
        }
        let effort_targets = controller_targets
            .iter()
            .enumerate()
            .map(|(index, target)| {
                let profile = actuator_profile(index);
                UrdfJointPdEffortTarget {
                    link_name: target.link_name,
                    target_position_rad: target.position,
                    stiffness_nm_per_rad: profile.stiffness_nm_per_rad,
                    damping_nm_s_per_rad: profile.damping_nm_s_per_rad,
                    max_effort_nm: profile.max_effort_nm,
                    max_velocity_rad_s: profile.max_velocity_rad_s,
                    transmission_efficiency: 1.0,
                }
            })
            .collect::<Vec<_>>();
        let applied = sim
            .step_joint_pd_effort_targets_substeps(
                &effort_targets,
                PHYSICS_SUBSTEPS_PER_CONTROL_STEP,
            )
            .context("step OpenArm portable PD-effort controller")?;
        sensor_samples += sim
            .sample_joint_feedback(&mut bus)
            .context("sample OpenArm bimanual joint feedback")? as u64;
        saturated_channel_samples += applied.iter().filter(|value| value.saturated).count() as u64;
        let carrier_before_step = carrier;
        carrier = advance_grasp_state(&mut sim, carrier, step)?;
        if carrier == Carrier::Right && pick_grasp_step.is_none() {
            pick_grasp_step = Some(step);
        }
        if carrier == Carrier::Left && regrasp_step.is_none() {
            regrasp_step = Some(step);
        }
        if carrier_before_step == Carrier::Left && carrier == Carrier::None {
            released = true;
        }
        let tracking_error_rad = reference_targets
            .iter()
            .map(|target| {
                sim.named_joint_position(target.link_name)
                    .map(|position| (position - target.position).abs())
                    .with_context(|| format!("missing OpenArm joint {}", target.link_name))
            })
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .fold(0.0_f64, f64::max);
        max_tracking_error_rad = max_tracking_error_rad.max(tracking_error_rad);
        let effort_utilization = applied
            .iter()
            .enumerate()
            .map(|(index, value)| value.effort_nm.abs() / actuator_profile(index).max_effort_nm)
            .fold(0.0_f64, f64::max);
        let telemetry = ControlFrameTelemetry {
            observation_age_ticks,
            tracking_error_rad,
            effort_utilization,
            saturated_fraction: applied.iter().filter(|value| value.saturated).count() as f64
                / applied.len() as f64,
        };
        left_travel_m = left_travel_m
            .max((link_position(&sim, "openarm_left_ee_base_link")? - left_start).length());
        right_travel_m = right_travel_m
            .max((link_position(&sim, "openarm_right_ee_base_link")? - right_start).length());
        let left_aperture_m =
            finger_aperture_m(&sim, "openarm_left_ee_link1", "openarm_left_ee_link2")?;
        let right_aperture_m =
            finger_aperture_m(&sim, "openarm_right_ee_link1", "openarm_right_ee_link2")?;
        left_aperture_min_m = left_aperture_min_m.min(left_aperture_m);
        left_aperture_max_m = left_aperture_max_m.max(left_aperture_m);
        right_aperture_min_m = right_aperture_min_m.min(right_aperture_m);
        right_aperture_max_m = right_aperture_max_m.max(right_aperture_m);
        if capture && step % CAPTURE_STRIDE == 0 {
            let (scene, current_mesh_items) = render_scene(&sim, &mut cache, telemetry)?;
            mesh_items = mesh_items.max(current_mesh_items);
            frames.push(CaptureFrame {
                step,
                phase: phase_label(step).to_string(),
                scene,
            });
        }
    }
    if !capture {
        let (_, current_mesh_items) =
            render_scene(&sim, &mut cache, ControlFrameTelemetry::default())?;
        mesh_items = current_mesh_items;
    }
    let final_targets = targets_for_pose(&BimanualPose {
        left: READY_L,
        right: READY_R,
    });
    let max_final_proximal_joint_error_rad = final_targets
        .iter()
        .filter(|target| {
            !target.link_name.contains("ee_link") && !target.link_name.contains("ee_base_link")
        })
        .filter_map(|target| {
            sim.named_joint_position(target.link_name)
                .map(|position| (position - target.position).abs())
        })
        .fold(0.0_f64, f64::max);
    Ok(Rollout {
        initial_digest,
        final_digest: hash_physics_state(sim.world()),
        actuated_joint_count: observation.actuated_joint_count,
        left_end_effector_travel_m: left_travel_m,
        right_end_effector_travel_m: right_travel_m,
        left_gripper_aperture_change_m: left_aperture_max_m - left_aperture_min_m,
        right_gripper_aperture_change_m: right_aperture_max_m - right_aperture_min_m,
        max_final_proximal_joint_error_rad,
        fixed_delta_ticks,
        feedback_decisions,
        sensor_samples,
        max_observation_age_ticks,
        max_tracking_error_rad,
        saturated_channel_samples,
        mesh_items,
        pick_grasp_step,
        regrasp_step,
        released,
        frames,
    })
}

/// Advances the contact-gated pick / relay-drop / re-grasp / place state
/// machine by one control step and returns the (possibly updated) carrier.
///
/// The block is a real dynamic Rapier body throughout. A grasp only occurs
/// once two distinct fingertip sensors of the closing gripper are actually
/// touching it ([`UrdfSceneSim::named_child_has_distinct_dual_contact`]);
/// once gated, the block switches to a kinematic pose-follower parented to
/// that gripper's palm frame so it rides along exactly. Each release hands
/// it back to ordinary dynamics so it actually drops and settles under
/// gravity, both at the mid-table relay point and on the final pad.
fn advance_grasp_state(sim: &mut UrdfSceneSim, carrier: Carrier, step: u64) -> Result<Carrier> {
    let mut carrier = carrier;
    if carrier == Carrier::None
        && (RIGHT_GRASP_WINDOW.0..=RIGHT_GRASP_WINDOW.1).contains(&step)
        && sim.named_child_has_distinct_dual_contact(RIGHT_FINGER1, RIGHT_FINGER2, BLOCK_NAME)
    {
        anyhow::ensure!(
            sim.add_named_child_frame_from_body(RIGHT_PALM, RIGHT_GRASP_FRAME, BLOCK_NAME),
            "OpenArm right grasp follower frame failed"
        );
        anyhow::ensure!(
            sim.set_named_body_kinematic(BLOCK_NAME, true),
            "OpenArm block did not accept kinematic pick grasp"
        );
        anyhow::ensure!(
            sim.set_named_collider_sensor(BLOCK_NAME, true),
            "OpenArm block collider missing for pick grasp"
        );
        carrier = Carrier::Right;
    }
    if carrier == Carrier::Right && step >= RIGHT_RELEASE_STEP {
        anyhow::ensure!(
            sim.set_named_body_kinematic(BLOCK_NAME, false),
            "OpenArm block did not release kinematic hold at the relay point"
        );
        anyhow::ensure!(
            sim.set_named_collider_sensor(BLOCK_NAME, false),
            "OpenArm block collider missing for the relay-point release"
        );
        carrier = Carrier::None;
    }
    if carrier == Carrier::None
        && (LEFT_REGRASP_WINDOW.0..=LEFT_REGRASP_WINDOW.1).contains(&step)
        && sim.named_child_has_distinct_dual_contact(LEFT_FINGER1, LEFT_FINGER2, BLOCK_NAME)
    {
        anyhow::ensure!(
            sim.add_named_child_frame_from_body(LEFT_PALM, LEFT_GRASP_FRAME, BLOCK_NAME),
            "OpenArm left re-grasp follower frame failed"
        );
        anyhow::ensure!(
            sim.set_named_body_kinematic(BLOCK_NAME, true),
            "OpenArm block did not accept kinematic re-grasp"
        );
        anyhow::ensure!(
            sim.set_named_collider_sensor(BLOCK_NAME, true),
            "OpenArm block collider missing for the re-grasp"
        );
        carrier = Carrier::Left;
    }
    if carrier == Carrier::Left && step >= LEFT_RELEASE_STEP {
        anyhow::ensure!(
            sim.set_named_body_kinematic(BLOCK_NAME, false),
            "OpenArm block did not release kinematic hold on the pad"
        );
        anyhow::ensure!(
            sim.set_named_collider_sensor(BLOCK_NAME, false),
            "OpenArm block collider missing for the final release"
        );
        carrier = Carrier::None;
    }
    match carrier {
        Carrier::Right => {
            anyhow::ensure!(
                sim.follow_named_body_to_frame(BLOCK_NAME, RIGHT_GRASP_FRAME),
                "OpenArm block lost its right grasp follower frame"
            );
        }
        Carrier::Left => {
            anyhow::ensure!(
                sim.follow_named_body_to_frame(BLOCK_NAME, LEFT_GRASP_FRAME),
                "OpenArm block lost its left grasp follower frame"
            );
        }
        Carrier::None => {}
    }
    Ok(carrier)
}

fn configure_official_effort_actuators(sim: &mut UrdfSceneSim) -> Result<()> {
    for (links, side) in [(&LEFT_LINKS, "left"), (&RIGHT_LINKS, "right")] {
        for (index, link) in links.iter().enumerate() {
            let profile = actuator_profile(index);
            anyhow::ensure!(
                sim.configure_named_revolute_effort_actuation(link, profile.max_effort_nm),
                "missing OpenArm {side} actuator link {link}"
            );
        }
    }
    Ok(())
}

/// Attaches the four fingertip contact sensors the grasp gate reads (see the
/// doc comment on [`RIGHT_FINGER1`]).
fn install_finger_tip_sensors(sim: &mut UrdfSceneSim) -> Result<()> {
    for (parent, sensor_name) in [
        ("openarm_right_ee_link1", RIGHT_FINGER1),
        ("openarm_right_ee_link2", RIGHT_FINGER2),
        ("openarm_left_ee_link1", LEFT_FINGER1),
        ("openarm_left_ee_link2", LEFT_FINGER2),
    ] {
        anyhow::ensure!(
            sim.add_named_child_box_sensor(
                parent,
                sensor_name,
                FINGER_SENSOR_SIZE_M,
                FINGER_TIP_OFFSET_M,
            ),
            "OpenArm failed to install fingertip contact sensor {sensor_name} on {parent}"
        );
    }
    Ok(())
}

const LEFT_ARM_COLLISION_GROUP: u32 = 1 << 0;
const RIGHT_ARM_COLLISION_GROUP: u32 = 1 << 1;

/// Excludes the two arms from colliding with each other.
///
/// The two arms are not collision-avoidance planned against each other, and
/// their commanded keyposes are not spatially separated by construction (the
/// bases sit only 6.2 cm apart), so an unlucky pair of joint targets could
/// otherwise let their forearms interpenetrate (which made the Rapier
/// contact solver do dramatically more work for the rest of that rollout).
/// The block and every other prop keep the default all-groups mask, so the
/// contact gates the grasp state machine relies on (fingertip-on-block) are
/// unaffected; only arm-vs-arm contact is suppressed.
fn disable_cross_arm_collisions(sim: &mut UrdfSceneSim) -> Result<()> {
    for link in LEFT_LINKS {
        anyhow::ensure!(
            sim.set_named_collision_groups(
                link,
                CollisionGroups {
                    memberships: LEFT_ARM_COLLISION_GROUP,
                    filter: !RIGHT_ARM_COLLISION_GROUP,
                },
            ),
            "missing OpenArm left link {link} for collision group setup"
        );
    }
    for link in RIGHT_LINKS {
        anyhow::ensure!(
            sim.set_named_collision_groups(
                link,
                CollisionGroups {
                    memberships: RIGHT_ARM_COLLISION_GROUP,
                    filter: !LEFT_ARM_COLLISION_GROUP,
                },
            ),
            "missing OpenArm right link {link} for collision group setup"
        );
    }
    Ok(())
}

fn actuator_profile(index: usize) -> ActuatorProfile {
    match index % LEFT_LINKS.len() {
        0 | 1 => ActuatorProfile {
            stiffness_nm_per_rad: 69.0,
            damping_nm_s_per_rad: 0.027,
            max_effort_nm: 40.0,
            max_velocity_rad_s: 16.755,
        },
        2 | 3 => ActuatorProfile {
            stiffness_nm_per_rad: 36.0,
            damping_nm_s_per_rad: 0.2,
            max_effort_nm: 27.0,
            max_velocity_rad_s: 5.4454,
        },
        4 => ActuatorProfile {
            stiffness_nm_per_rad: 36.0,
            damping_nm_s_per_rad: 0.2,
            max_effort_nm: 7.0,
            max_velocity_rad_s: 20.944,
        },
        5 | 6 => ActuatorProfile {
            stiffness_nm_per_rad: 12.0,
            damping_nm_s_per_rad: 0.06,
            max_effort_nm: 7.0,
            max_velocity_rad_s: 20.944,
        },
        _ => ActuatorProfile {
            stiffness_nm_per_rad: 12.0,
            damping_nm_s_per_rad: 0.03,
            max_effort_nm: 0.5,
            max_velocity_rad_s: 5.0,
        },
    }
}

fn feedback_adjusted_targets(
    references: &[UrdfJointPositionTarget<'static>],
    feedback: Option<&Frame<JointFeedback>>,
    consumed_at_ticks: u64,
    link_order: &[String],
) -> Result<(Vec<UrdfJointPositionTarget<'static>>, u64)> {
    let Some(feedback) = feedback else {
        return Ok((references.to_vec(), 0));
    };
    anyhow::ensure!(
        feedback.payload.schema_version == JointFeedback::SCHEMA_VERSION
            && feedback.payload.status == JointFeedbackStatus::Nominal,
        "OpenArm controller requires nominal joint feedback"
    );
    anyhow::ensure!(
        feedback.payload.joints.len() == references.len() && link_order.len() == references.len(),
        "OpenArm joint-feedback width mismatch"
    );
    let adjusted = references
        .iter()
        .zip(&feedback.payload.joints)
        .zip(link_order)
        .enumerate()
        .map(|(index, ((reference, joint), expected_name))| {
            anyhow::ensure!(
                joint.name == *expected_name && reference.link_name == expected_name,
                "OpenArm joint-feedback order mismatch at channel {index}"
            );
            let (position_rad, velocity_rad_s) = match joint.coordinate {
                JointCoordinateFeedback::Revolute {
                    position_rad,
                    velocity_rad_s,
                } => (position_rad, velocity_rad_s),
                _ => anyhow::bail!("OpenArm feedback channel {} is not revolute", joint.name),
            };
            let finger = index % LEFT_LINKS.len() >= 7;
            let position_gain = if finger { 0.15 } else { 0.30 };
            let velocity_damping_s = if finger { 0.001 } else { 0.002 };
            let maximum_correction_rad = if finger { 0.02 } else { 0.04 };
            let correction_rad = (position_gain * (reference.position - position_rad)
                - velocity_damping_s * velocity_rad_s)
                .clamp(-maximum_correction_rad, maximum_correction_rad);
            Ok(UrdfJointPositionTarget {
                link_name: reference.link_name,
                position: reference.position + correction_rad,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok((
        adjusted,
        consumed_at_ticks.saturating_sub(feedback.capture_time.ticks()),
    ))
}

/// Independently evaluates each arm's own keyframe schedule at `step`. The
/// two arms are not required to share phase boundaries: the left arm holds
/// at `READY_L` while the right arm picks the block up and relays it to the
/// mid-table point, then the right arm withdraws home while the left arm
/// reaches in, re-grasps it, and carries it on to the pad.
fn commanded_pose(step: u64) -> BimanualPose {
    BimanualPose {
        left: arm_pose_at(LEFT_KEYFRAMES, step),
        right: arm_pose_at(RIGHT_KEYFRAMES, step),
    }
}

/// Quintic ("smootherstep") ease: zero first *and* second derivative at both
/// ends, giving a minimum-jerk-like glide between keyposes instead of the
/// abrupt acceleration a linear or cubic blend leaves at the endpoints.
fn smooth(alpha: f64) -> f64 {
    let alpha = alpha.clamp(0.0, 1.0);
    alpha * alpha * alpha * (alpha * (alpha * 6.0 - 15.0) + 10.0)
}

fn targets_for_pose(pose: &BimanualPose) -> Vec<UrdfJointPositionTarget<'static>> {
    LEFT_LINKS
        .iter()
        .zip(pose.left)
        .chain(RIGHT_LINKS.iter().zip(pose.right))
        .map(|(link_name, position)| UrdfJointPositionTarget {
            link_name,
            position,
        })
        .collect()
}

fn phase_label(step: u64) -> &'static str {
    match step {
        0..=150 => "reach-to-pick",
        151..=260 => "descend-to-block",
        261..=320 => "close-gripper",
        321..=420 => "lift-block",
        421..=520 => "carry-to-relay",
        521..=580 => "descend-to-relay",
        581..=620 => "release-at-relay",
        621..=760 => "right-withdraw",
        761..=800 => "left-approach-relay",
        801..=840 => "left-descend-to-relay",
        841..=890 => "left-close-gripper",
        891..=970 => "lift-from-relay",
        971..=1_080 => "carry-to-pad",
        1_081..=1_150 => "descend-to-pad",
        1_151..=1_210 => "release-on-pad",
        1_211..=1_270 => "left-withdraw",
        _ => "return-home",
    }
}

fn link_position(sim: &UrdfSceneSim, link: &str) -> Result<Vec3> {
    let (x_m, y_m, z_m) = sim
        .link_translation_m(link)
        .with_context(|| format!("missing OpenArm link {link}"))?;
    Ok(Vec3::new(x_m, y_m, z_m))
}

fn finger_aperture_m(sim: &UrdfSceneSim, first_link: &str, second_link: &str) -> Result<f64> {
    let fingertip = |link: &str| -> Result<Vec3> {
        let transform = sim
            .named_transform(link)
            .with_context(|| format!("missing OpenArm finger link {link}"))?;
        Ok(transform.translation
            + transform.rotation * (Vec3::new(0.0, 0.0, -0.065) * transform.scale))
    };
    Ok((fingertip(first_link)? - fingertip(second_link)?).length())
}

fn render_scene(
    sim: &UrdfSceneSim,
    cache: &mut MeshRenderCache,
    telemetry: ControlFrameTelemetry,
) -> Result<(RenderScene, usize)> {
    let mut scene = build_visual_render_scene(sim.world());
    // The fingertip contact sensors (see `RIGHT_FINGER1`) are real physics
    // colliders with no `Visual`/`LinkVisuals` component, so the generic
    // scene builder would otherwise draw them as a fallback box. Their exact,
    // deliberately distinctive size makes them safe to filter back out here.
    scene.items.retain(|item| {
        !matches!(
            item.shape,
            VisualShape::Box { size_m }
                if (size_m.x - FINGER_SENSOR_SIZE_M[0]).abs() < 1e-9
                    && (size_m.y - FINGER_SENSOR_SIZE_M[1]).abs() < 1e-9
                    && (size_m.z - FINGER_SENSOR_SIZE_M[2]).abs() < 1e-9
        )
    });
    let roots = sim.mesh_package_roots().to_vec();
    let root_refs = roots.iter().map(PathBuf::as_path).collect::<Vec<_>>();
    cache
        .resolve_scene(&mut scene, &root_refs)
        .map_err(|error| anyhow::anyhow!("resolve official OpenArm meshes: {error}"))?;
    let mesh_items = scene
        .items
        .iter()
        .filter(|item| item.mesh.is_some())
        .count();
    push_lab_dressing(&mut scene);
    push_control_panel(&mut scene, sim.fixed_delta().ticks(), telemetry);

    Ok((scene, mesh_items))
}

/// Render-only lab dressing: tiled floor, an aluminium-extrusion workbench
/// frame under the authored tabletop, trim/baseboard detail on the authored
/// partition wall, and a small parts bin so the cell reads as a real
/// workspace instead of an empty void. Everything here is placed to stay
/// inside the showcase camera's frame (see `CAMERA`); none of it
/// participates in physics, and every transform is a fixed function of
/// world constants, so it is exactly reproduced on every replay.
fn push_lab_dressing(scene: &mut RenderScene) {
    const FLOOR_Y_M: f64 = -0.012;
    const FLOOR_COLOR: [f32; 4] = [0.74, 0.76, 0.80, 1.0];
    const GROUT_COLOR: [f32; 4] = [0.60, 0.63, 0.68, 1.0];
    let floor_material = PbrMaterial::new(FLOOR_COLOR, 0.55, 0.30, [0.0; 3]);
    push_box_material(
        scene,
        Vec3::new(0.05, FLOOR_Y_M, 0.15),
        Vec3::new(5.4, 0.02, 5.4),
        Quat::IDENTITY,
        FLOOR_COLOR,
        floor_material,
    );
    for x_m in [-1.5, -0.5, 0.5, 1.5] {
        push_box(
            scene,
            Vec3::new(x_m, FLOOR_Y_M + 0.006, 0.15),
            Vec3::new(0.012, 0.006, 5.4),
            GROUT_COLOR,
        );
    }
    for z_m in [-1.4, -0.2, 1.0, 2.2] {
        push_box(
            scene,
            Vec3::new(0.05, FLOOR_Y_M + 0.006, z_m),
            Vec3::new(5.4, 0.006, 0.012),
            GROUT_COLOR,
        );
    }

    // Aluminium-extrusion workbench frame under the authored tabletop
    // (table top center (0.0, 0.30, 0.48), half-extents 0.725 x 0.03 x 0.31).
    const LEG_COLOR: [f32; 4] = [0.70, 0.72, 0.75, 1.0];
    let leg_material = || PbrMaterial::new(LEG_COLOR, 0.30, 0.75, [0.0; 3]);
    for x_m in [-0.64, 0.64] {
        for z_m in [0.22, 0.74] {
            push_box_material(
                scene,
                Vec3::new(x_m, 0.135, z_m),
                Vec3::new(0.045, 0.27, 0.045),
                Quat::IDENTITY,
                LEG_COLOR,
                leg_material(),
            );
        }
    }
    push_box_material(
        scene,
        Vec3::new(0.0, 0.008, 0.48),
        Vec3::new(1.30, 0.016, 0.56),
        Quat::IDENTITY,
        LEG_COLOR,
        leg_material(),
    );

    // Small parts bin with a few spare stock cubes, low and just past the
    // pedestal on the camera-right side, comfortably inside the frame.
    const BIN_COLOR: [f32; 4] = [0.30, 0.34, 0.40, 1.0];
    push_box(
        scene,
        Vec3::new(0.46, 0.24, 0.16),
        Vec3::new(0.20, 0.10, 0.16),
        BIN_COLOR,
    );
    push_box(
        scene,
        Vec3::new(0.46, 0.30, 0.09),
        Vec3::new(0.20, 0.03, 0.012),
        BIN_COLOR,
    );
    push_box(
        scene,
        Vec3::new(0.46, 0.30, 0.23),
        Vec3::new(0.20, 0.03, 0.012),
        BIN_COLOR,
    );
    push_box(
        scene,
        Vec3::new(0.37, 0.30, 0.16),
        Vec3::new(0.012, 0.03, 0.16),
        BIN_COLOR,
    );
    push_box(
        scene,
        Vec3::new(0.55, 0.30, 0.16),
        Vec3::new(0.012, 0.03, 0.16),
        BIN_COLOR,
    );
    for (offset_x, offset_z, color) in [
        (-0.04, -0.02, [0.95, 0.55, 0.10, 1.0]),
        (0.03, 0.01, [0.15, 0.70, 0.90, 1.0]),
        (-0.01, 0.04, [0.85, 0.20, 0.30, 1.0]),
    ] {
        push_box(
            scene,
            Vec3::new(0.46 + offset_x, 0.27, 0.16 + offset_z),
            Vec3::new(0.035, 0.035, 0.035),
            color,
        );
    }

    // Baseboard and trim on the authored partition wall (object
    // `openarm_back_wall`: center (-0.10, 0.75, -0.85), half-extents
    // 0.45 x 0.40 x 0.025) so it reads as a low detailed partition rather
    // than a flat slab.
    const TRIM_COLOR: [f32; 4] = [0.42, 0.46, 0.52, 1.0];
    push_box(
        scene,
        Vec3::new(-0.10, 0.37, -0.822),
        Vec3::new(0.92, 0.05, 0.006),
        TRIM_COLOR,
    );
    push_box(
        scene,
        Vec3::new(-0.10, 0.62, -0.822),
        Vec3::new(0.92, 0.02, 0.006),
        TRIM_COLOR,
    );
    for x_m in [-0.42, 0.02, 0.26] {
        push_box(
            scene,
            Vec3::new(x_m, 0.75, -0.822),
            Vec3::new(0.012, 0.78, 0.004),
            TRIM_COLOR,
        );
    }
}

fn push_control_panel(
    scene: &mut RenderScene,
    fixed_delta_ticks: u64,
    telemetry: ControlFrameTelemetry,
) {
    // A clean console HUD: a bezel, a lit screen inset, and three telemetry
    // bars, all facing +Z toward the camera. Stood next to the pedestal (not
    // on the now-small partition wall, which sits too far back to stay
    // legible at this closer framing) so it reads as a mounted display
    // rather than a loose slab of boxes, and stays inside the frame.
    const PANEL_CENTER: Vec3 = Vec3::new(-0.36, 0.64, -0.05);
    const BEZEL_COLOR: [f32; 4] = [0.07, 0.08, 0.11, 1.0];
    const SCREEN_COLOR: [f32; 4] = [0.035, 0.055, 0.085, 1.0];
    const BAR_TRACK_COLOR: [f32; 4] = [0.08, 0.10, 0.14, 1.0];
    const BAR_BOTTOM_Y_M: f64 = PANEL_CENTER.y - 0.066;
    const BAR_MAX_HEIGHT_M: f64 = 0.133;

    push_box(
        scene,
        PANEL_CENTER,
        Vec3::new(0.32, 0.21, 0.021),
        BEZEL_COLOR,
    );
    push_box_material(
        scene,
        Vec3::new(PANEL_CENTER.x, PANEL_CENTER.y, PANEL_CENTER.z + 0.011),
        Vec3::new(0.28, 0.17, 0.004),
        Quat::IDENTITY,
        SCREEN_COLOR,
        PbrMaterial::new(SCREEN_COLOR, 0.15, 0.05, [0.01, 0.02, 0.03]),
    );

    let values = [
        if fixed_delta_ticks == 0 {
            0.0
        } else {
            telemetry.observation_age_ticks as f64 / fixed_delta_ticks as f64 / 2.0
        },
        telemetry.tracking_error_rad / 0.40,
        telemetry
            .effort_utilization
            .max(telemetry.saturated_fraction),
    ];
    let colors = [
        [0.10, 0.70, 0.95, 1.0],
        [1.00, 0.62, 0.08, 1.0],
        [0.92, 0.18, 0.16, 1.0],
    ];
    for (index, (value, color)) in values.into_iter().zip(colors).enumerate() {
        let x_m = PANEL_CENTER.x - 0.091 + index as f64 * 0.091;
        push_box(
            scene,
            Vec3::new(
                x_m,
                BAR_BOTTOM_Y_M + BAR_MAX_HEIGHT_M * 0.5,
                PANEL_CENTER.z + 0.017,
            ),
            Vec3::new(0.049, BAR_MAX_HEIGHT_M, 0.014),
            BAR_TRACK_COLOR,
        );
        let height_m = (BAR_MAX_HEIGHT_M * value.clamp(0.03, 1.0)).max(0.01);
        push_box(
            scene,
            Vec3::new(x_m, BAR_BOTTOM_Y_M + height_m * 0.5, PANEL_CENTER.z + 0.025),
            Vec3::new(0.035, height_m, 0.014),
            color,
        );
    }
}

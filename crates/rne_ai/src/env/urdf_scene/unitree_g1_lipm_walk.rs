//! LIPM + ZMP-preview walking for the official 23-DoF Unitree G1.
//!
//! This follows the model-based biped pipeline the open-source walking
//! controllers share (`jrl-umi3218/lipm_walking_controller`,
//! `BaselineWalkingController`, Kajita's ZMP preview control): plan footsteps,
//! generate a center-of-mass trajectory that tracks the ZMP reference, place the
//! swing foot on the next foothold with a lift bump, and convert the desired
//! center of mass and feet into joint targets with a planar two-link leg
//! inverse kinematics.
//!
//! The plant is the dynamic multibody G1, and the whole path is deterministic.

use super::{
    unitree_g1_dynamic_scene_path, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_assets::AssetError;
use rne_dynamics::{center_of_mass, ArticulatedModel};
use rne_ecs::World;
use rne_legged::{
    plan_walking_pattern, GaitSchedule, Horizontal, LimpParams, StraightWalkRequest,
    WalkingPattern, ZmpPreviewController,
};
use rne_math::Quat;
use rne_math::Vec3;
use rne_robot::FloatingBase;
use rne_wbc::{
    BaseAttitudeTask, ComTask, ContactPoint, PostureTask, WholeBodyConfig, WholeBodyController,
};
use rne_world::Transform3;
use std::path::PathBuf;

/// Thigh length of the G1 leg in meters (hip-yaw to knee).
pub const G1_THIGH_M: f64 = 0.331;
/// Shank length of the G1 leg in meters (knee to ankle).
#[allow(clippy::approx_constant)]
pub const G1_SHANK_M: f64 = 0.318;
/// Neutral hip-pitch angle in radians.
pub const G1_NEUTRAL_HIP_PITCH_RAD: f64 = -0.18;
/// Neutral knee angle in radians.
pub const G1_NEUTRAL_KNEE_RAD: f64 = 0.36;
/// Neutral hip-roll magnitude in radians (left positive, right negative).
pub const G1_NEUTRAL_HIP_ROLL_RAD: f64 = 0.05;
/// Lateral distance from the pelvis to the hip in meters.
pub const G1_HIP_LATERAL_M: f64 = 0.116;

/// Configuration for one deterministic LIPM walking run on the G1.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1LipmWalkConfig {
    /// Scene file to load.
    pub scene_path: PathBuf,
    /// Steps used to settle onto the stand before planning.
    pub settle_steps: usize,
    /// Simulation steps to run.
    pub rollout_steps: usize,
    /// Number of footsteps to plan.
    pub steps: usize,
    /// Nominal step length in meters.
    pub step_length_m: f64,
    /// Nominal step width in meters.
    pub step_width_m: f64,
    /// LIPM center-of-mass height in meters.
    pub com_height_m: f64,
    /// Single-support duration in seconds.
    pub single_support_s: f64,
    /// Double-support duration in seconds.
    pub double_support_s: f64,
    /// Initial weight-shift duration in seconds.
    pub weight_shift_s: f64,
    /// Peak swing-foot clearance in meters.
    pub swing_height_m: f64,
    /// Center-of-mass tracking feedback gain (fraction of the DCM error added
    /// to the reference).
    pub com_feedback_gain: f64,
    /// Scale on the DCM step adjustment (1.0 applies the full Khadiv landing
    /// correction; 0.0 disables it).
    pub dcm_foot_placement_gain: f64,
    /// Maximum landing-point adjustment in meters.
    pub max_foot_mod_m: f64,
    /// Servo stiffness for every joint.
    pub position_stiffness: f64,
    /// Servo damping for every joint.
    pub position_damping: f64,
    /// Actuator torque ceiling in newton-meters.
    pub torque_limit_nm: f64,
    /// Prints the per-step state when true.
    pub trace: bool,
    /// Center-of-mass task position gain for the whole-body solve.
    pub wbc_com_gain: f64,
    /// Center-of-mass task weight for the whole-body solve.
    pub wbc_com_weight: f64,
    /// Base-attitude task weight for the whole-body solve.
    pub wbc_angular_weight: f64,
    /// Posture task weight for the whole-body solve.
    pub wbc_posture_weight: f64,
    /// Holds the standing reference instead of tracking the walking pattern,
    /// which isolates the whole-body balance layer from the gait.
    pub stand_only: bool,
}

impl Default for UnitreeG1LipmWalkConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_dynamic_scene_path(),
            settle_steps: 240,
            rollout_steps: 900,
            steps: 8,
            step_length_m: 0.18,
            step_width_m: 0.20,
            com_height_m: 0.75,
            single_support_s: 0.5,
            double_support_s: 0.1,
            weight_shift_s: 0.4,
            swing_height_m: 0.05,
            com_feedback_gain: 0.3,
            dcm_foot_placement_gain: 1.0,
            max_foot_mod_m: 0.12,
            position_stiffness: 220.0,
            position_damping: 24.0,
            torque_limit_nm: 88.0,
            trace: false,
            wbc_com_gain: 100.0,
            wbc_com_weight: 1.0e5,
            wbc_angular_weight: 1.0e3,
            wbc_posture_weight: 1.0,
            stand_only: false,
        }
    }
}

/// Metrics from one LIPM walking run.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1LipmWalkOutcome {
    /// Planned pattern duration in seconds.
    pub pattern_duration_s: f64,
    /// Final planned center-of-mass horizontal position.
    pub planned_com_m: Horizontal,
    /// Final measured pelvis horizontal position.
    pub measured_pelvis_m: Horizontal,
    /// Forward (world +z) distance covered by the pelvis in meters.
    pub forward_distance_m: f64,
    /// Minimum pelvis height over the run in meters.
    pub min_height_m: f64,
    /// Maximum pelvis tilt from upright over the run in radians.
    pub max_tilt_rad: f64,
    /// Maximum horizontal distance between the measured pelvis and the planned
    /// center of mass over the run, in meters.
    pub max_tracking_error_m: f64,
    /// Largest ratio of an applied joint torque to its URDF limit over the run.
    pub max_torque_ratio: f64,
    /// Whether the whole-body solve reported a saturated actuator box.
    pub torque_saturated: bool,
    /// Whether the pelvis dropped below half its start height.
    pub fell: bool,
    /// Deterministic digest of the run.
    pub digest: u64,
}

/// Solves the planar two-link leg inverse kinematics.
///
/// `forward_m` is the ankle offset ahead of the hip and `down_m` the ankle drop
/// below the hip. Returns `(hip_pitch, knee, ankle_pitch)` for a flat foot.
pub fn g1_leg_ik(forward_m: f64, down_m: f64) -> (f64, f64, f64) {
    let clamp_unit = |value: f64| {
        if value.is_nan() {
            -1.0
        } else {
            value.clamp(-1.0, 1.0)
        }
    };
    let a = G1_THIGH_M;
    let b = G1_SHANK_M;
    let raw = (forward_m * forward_m + down_m * down_m).sqrt();
    if !raw.is_finite() {
        return (
            G1_NEUTRAL_HIP_PITCH_RAD,
            G1_NEUTRAL_KNEE_RAD,
            -G1_NEUTRAL_HIP_PITCH_RAD - G1_NEUTRAL_KNEE_RAD,
        );
    }
    let distance = raw.clamp((a - b).abs() + 1.0e-4, a + b - 1.0e-4);
    let knee = clamp_unit((distance * distance - a * a - b * b) / (2.0 * a * b)).acos();
    let direction = forward_m.atan2(down_m);
    let shoulder = clamp_unit((a * a + distance * distance - b * b) / (2.0 * a * distance)).acos();
    let hip_pitch = direction - shoulder;
    (hip_pitch, knee, -(hip_pitch + knee))
}

/// The single-support state of the current step.
#[derive(Clone, Copy, Debug, PartialEq)]
struct StepPhase {
    /// 1-based step index.
    index: usize,
    /// Elapsed single-support time in seconds.
    elapsed_s: f64,
    /// Single-support duration in seconds.
    duration_s: f64,
    /// Whether the left foot is the swing foot.
    swing_left: bool,
}

/// Locates the single-support phase of the step containing `time_s`.
fn step_phase(config: &UnitreeG1LipmWalkConfig, time_s: f64) -> Option<StepPhase> {
    if time_s < config.weight_shift_s {
        return None;
    }
    let mut remaining = time_s - config.weight_shift_s;
    for index in 1..=config.steps {
        if remaining < config.single_support_s {
            return Some(StepPhase {
                index,
                elapsed_s: remaining,
                duration_s: config.single_support_s,
                swing_left: index % 2 == 0,
            });
        }
        remaining -= config.single_support_s;
        if remaining < config.double_support_s {
            return None;
        }
        remaining -= config.double_support_s;
    }
    None
}

/// The eight proximal G1 leg links driven by whole-body torque. The ankles stay
/// position-servoed: their small inertia makes the 60 Hz torque path unstable.
const G1_TORQUE_LINKS: [&str; 8] = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
];

/// The twelve actuated G1 leg links, in the order the WBC torques are applied.
const G1_LEG_LINKS: [&str; 12] = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "left_ankle_pitch_link",
    "left_ankle_roll_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
    "right_ankle_pitch_link",
    "right_ankle_roll_link",
];

/// The G1's URDF actuator effort limit for an actuated link, in newton-meters.
fn g1_effort_limit_nm(link: &str) -> f64 {
    if link.ends_with("knee_link") {
        139.0
    } else if link.ends_with("ankle_pitch_link") || link.ends_with("ankle_roll_link") {
        35.0
    } else if link.contains("shoulder") || link.ends_with("elbow_link") || link.contains("wrist") {
        25.0
    } else {
        88.0
    }
}

/// Heel and toe contact points in the G1 foot link frame (URDF z-down frame).
const G1_FOOT_CONTACTS_LOCAL_M: [Vec3; 2] =
    [Vec3::new(-0.05, 0.0, -0.03), Vec3::new(0.12, 0.0, -0.03)];

/// Builds the G1 articulated model from the shipped URDF.
fn g1_model() -> ArticulatedModel {
    const URDF: &str = include_str!("../../../../../assets/robots/g1_description/g1_23dof.urdf");
    let document = rne_urdf_import::parse_urdf_document(URDF).expect("parse G1 URDF");
    let mut world = World::new();
    let config = rne_urdf_import::UrdfSpawnConfig {
        attach_colliders: false,
        attach_mesh_colliders: false,
        self_collisions: false,
        use_declared_inertial_masses: true,
        ..rne_urdf_import::UrdfSpawnConfig::default()
    };
    let spawned = rne_urdf_import::spawn_urdf_document_with_config(&mut world, &document, config)
        .expect("spawn G1");
    world.entity_mut(spawned.base_link).insert((
        Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_rotation_x(-std::f64::consts::FRAC_PI_2),
        ),
        FloatingBase,
    ));
    ArticulatedModel::from_robot(&world, spawned.robot).expect("G1 model")
}

/// Reads the floating-base state from the scene in the model's dof order.
fn read_g1_state(sim: &UrdfSceneSim, model: &ArticulatedModel, names: &[String]) -> Vec<f64> {
    let nv = model.nv();
    let base = sim.named_transform("pelvis").expect("pelvis pose");
    let floating = base.rotation * Quat::from_rotation_x(std::f64::consts::FRAC_PI_2);
    let (yaw, pitch, roll) = floating.to_euler(glam::EulerRot::ZYX);
    let mut state = vec![0.0; 2 * nv];
    state[0] = base.translation.x;
    state[1] = base.translation.y;
    state[2] = base.translation.z;
    state[3] = roll;
    state[4] = pitch;
    state[5] = yaw;
    for (dof, name) in names.iter().enumerate() {
        state[6 + dof] = sim.named_joint_position(name).unwrap_or(0.0);
        state[nv + 6 + dof] = sim.named_joint_velocity(name).unwrap_or(0.0);
    }
    state
}

/// The neutral ankle offset ahead of and below the hip, from the neutral pose.
fn neutral_ankle_offset() -> (f64, f64) {
    let a = G1_THIGH_M;
    let b = G1_SHANK_M;
    let h = G1_NEUTRAL_HIP_PITCH_RAD;
    let k = G1_NEUTRAL_KNEE_RAD;
    (
        a * h.sin() + b * (h + k).sin(),
        a * h.cos() + b * (h + k).cos(),
    )
}

/// Samples the planned feet at `time_s`.
///
/// Returns the world `(x, y, z)` of the left and right foot. The feet are
/// planted unless their step is in single support.
fn planned_feet(
    pattern: &WalkingPattern,
    config: &UnitreeG1LipmWalkConfig,
    time_s: f64,
    swing_mod: Horizontal,
) -> (Vec3, Vec3) {
    let plan = &pattern.plan;
    let mut left = plan.footsteps[0].position_m;
    let mut right = plan.footsteps[1].position_m;
    let mut left_lift = 0.0_f64;
    let mut right_lift = 0.0_f64;
    let single = config.single_support_s;
    let double = config.double_support_s;
    if time_s > config.weight_shift_s {
        let mut phase_time = time_s - config.weight_shift_s;
        for index in 1..=config.steps {
            let swing_left = index % 2 == 0;
            let target = plan
                .footsteps
                .get(index + 1)
                .map(|foot| foot.position_m + swing_mod);
            if phase_time < single {
                if let Some(target) = target {
                    let u = (phase_time / single).clamp(0.0, 1.0);
                    let smooth = u * u * (3.0 - 2.0 * u);
                    let lift = config.swing_height_m * (std::f64::consts::PI * u).sin();
                    if swing_left {
                        left = left.lerp(target, smooth);
                        left_lift = lift;
                    } else {
                        right = right.lerp(target, smooth);
                        right_lift = lift;
                    }
                }
                break;
            }
            if target.is_some() {
                if swing_left {
                    left = plan.footsteps[index + 1].position_m;
                } else {
                    right = plan.footsteps[index + 1].position_m;
                }
            }
            phase_time -= single;
            if phase_time < double {
                break;
            }
            phase_time -= double;
        }
    }
    (
        Vec3::new(left.x_m, left_lift, left.z_m),
        Vec3::new(right.x_m, right_lift, right.z_m),
    )
}

/// Runs one deterministic LIPM walking rollout on the dynamic G1.
pub fn run_unitree_g1_lipm_walk(
    config: UnitreeG1LipmWalkConfig,
) -> Result<UnitreeG1LipmWalkOutcome, AssetError> {
    let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
    sim.configure_position_motors(
        config.position_stiffness,
        config.position_damping,
        config.torque_limit_nm,
    );
    let stand = super::unitree_g1_gait_targets(
        0,
        super::UnitreeG1GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: 120,
        },
    );
    for _ in 0..config.settle_steps {
        sim.step_joint_position_targets(&stand);
    }

    let model = g1_model();
    let nv = model.nv();
    let names: Vec<String> = model
        .kinematic()
        .movable_joint_entities()
        .iter()
        .map(|joint| {
            let child = model.kinematic().joint_child_link(*joint).expect("child");
            let index = model.kinematic().link_index(child).expect("index");
            model
                .kinematic()
                .link_name(index)
                .expect("name")
                .to_string()
        })
        .collect();
    let leg_dofs: Vec<usize> = G1_LEG_LINKS
        .iter()
        .map(|name| {
            names
                .iter()
                .position(|candidate| candidate == name)
                .expect("leg dof")
        })
        .collect();
    let control_dim = nv - 6;
    let torque_limits_nm: Vec<f64> = names.iter().map(|name| g1_effort_limit_nm(name)).collect();
    let wbc = WholeBodyController::new(WholeBodyConfig {
        com_weight: config.wbc_com_weight,
        posture_weight: config.wbc_posture_weight,
        angular_weight: config.wbc_angular_weight,
        force_regularization: 1.0e-6,
        torque_limits_nm: Some(torque_limits_nm.clone()),
        enforce_torque_limits: false,
        ..WholeBodyConfig::default()
    });

    let sample_time_s = super::UNITREE_G1_SIM_DT_S;
    let params = LimpParams::new(config.com_height_m, 9.806_65);
    let controller =
        ZmpPreviewController::new(params, sample_time_s, 1.0, 1.0e-6, 80).map_err(|error| {
            AssetError::Invalid {
                path: config.scene_path.display().to_string(),
                message: format!("preview controller: {error}"),
            }
        })?;
    let pelvis_start = sim
        .named_transform("pelvis")
        .ok_or_else(|| AssetError::Invalid {
            path: config.scene_path.display().to_string(),
            message: "missing pelvis".into(),
        })?;
    // The plan's initial footholds must land on the measured feet, not on the
    // pelvis projection, or the first targets command a large leg jump.
    let start_horizontal = {
        let left = sim
            .named_transform("left_ankle_roll_link")
            .expect("left foot")
            .translation;
        let right = sim
            .named_transform("right_ankle_roll_link")
            .expect("right foot")
            .translation;
        Horizontal::new(0.5 * (left.x + right.x), 0.5 * (left.z + right.z))
    };
    let start_height = pelvis_start.translation.y;
    // The G1's local +Z is up, so its fore-aft axis is world X and its lateral
    // axis is world Z. The plan's `x_m` maps to world X and `z_m` to world Z.
    let request = StraightWalkRequest {
        direction: Horizontal::new(1.0, 0.0),
        start_com_m: start_horizontal,
        steps: config.steps,
        step_length_m: config.step_length_m,
        step_width_m: config.step_width_m,
        schedule: GaitSchedule {
            single_support_s: config.single_support_s,
            double_support_s: config.double_support_s,
            weight_shift_s: config.weight_shift_s,
            settle_s: 0.4,
        },
    };
    let pattern = plan_walking_pattern(&params, &controller, &request).map_err(|error| {
        AssetError::Invalid {
            path: config.scene_path.display().to_string(),
            message: format!("walking pattern: {error}"),
        }
    })?;

    // Anchor the LIPM trajectory on the measured center of mass: the plan's
    // absolute horizontal origin is a foot midpoint, which is not where the
    // model's CoM sits.
    let com_origin = {
        let q_origin = read_g1_state(&sim, &model, &names);
        center_of_mass(&model, &q_origin[..nv]).expect("start CoM")
    };
    let plan_origin = pattern.com_m[0];
    let (neutral_forward, neutral_down) = neutral_ankle_offset();
    let neutral_relative = {
        let pelvis = sim.named_transform("pelvis").expect("pelvis").translation;
        [
            sim.named_transform("left_ankle_roll_link")
                .expect("left foot")
                .translation
                - pelvis,
            sim.named_transform("right_ankle_roll_link")
                .expect("right foot")
                .translation
                - pelvis,
        ]
    };
    let up_reference = {
        let pose = sim.named_transform("pelvis").expect("pelvis");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };

    let mut min_height_m = start_height;
    let mut max_tilt_rad: f64 = 0.0;
    let mut max_tracking_error_m: f64 = 0.0;
    let mut digest = 0xcbf2_9ce4_8422_2325_u64;
    let mut fell = false;
    let mut max_torque_ratio: f64 = 0.0;
    let mut torque_saturated = false;

    for step in 0..config.rollout_steps {
        let time_s = step as f64 * sample_time_s;
        let motion_time_s = if config.stand_only { 0.0 } else { time_s };
        let index =
            ((motion_time_s / sample_time_s).floor() as usize).min(pattern.sample_count() - 1);
        let pattern_index = if index >= pattern.sample_count() {
            pattern.sample_count() - 1
        } else {
            index
        };
        let com_reference = Horizontal::new(
            com_origin.x + (pattern.com_m[pattern_index].x_m - plan_origin.x_m),
            com_origin.z + (pattern.com_m[pattern_index].z_m - plan_origin.z_m),
        );
        let com_velocity = pattern.com_velocity_m_s[pattern_index];
        let omega = params.omega_rad_s();

        let pelvis_now = sim.named_transform("pelvis").expect("pelvis");
        let observation = sim.observe();
        let measured_velocity = Vec3::new(
            observation.base_linear_velocity_x_m_s,
            0.0,
            observation.base_linear_velocity_z_m_s,
        );
        // DCM feedback: steer the reference back toward the plan.
        let reference_dcm = Vec3::new(
            com_reference.x_m + com_velocity.x_m / omega,
            0.0,
            com_reference.z_m + com_velocity.z_m / omega,
        );
        let measured_dcm = Vec3::new(
            pelvis_now.translation.x + measured_velocity.x / omega,
            0.0,
            pelvis_now.translation.z + measured_velocity.z / omega,
        );
        let correction = (reference_dcm - measured_dcm) * config.com_feedback_gain;
        let pelvis_target = Vec3::new(
            com_reference.x_m + correction.x,
            start_height,
            com_reference.z_m + correction.z,
        );

        // Khadiv et al. step adjustment: estimate where the divergent component
        // of motion will be at the end of the current step and pick the landing
        // point that brings it to the planned DCM one step later.
        let (planned_left, planned_right) =
            planned_feet(&pattern, &config, motion_time_s, Horizontal::ZERO);
        let swing_mod = match step_phase(&config, motion_time_s) {
            Some(phase) if config.dcm_foot_placement_gain != 0.0 => {
                let stance = if phase.swing_left {
                    planned_right
                } else {
                    planned_left
                };
                let u = Horizontal::new(stance.x, stance.z);
                let dcm_measured = Horizontal::new(
                    pelvis_now.translation.x + measured_velocity.x / omega,
                    pelvis_now.translation.z + measured_velocity.z / omega,
                );
                let remaining = (phase.duration_s - phase.elapsed_s).max(0.0);
                let decay = (omega * remaining).exp();
                let dcm_end = (dcm_measured - u) * decay + u;
                let future = (time_s + phase.duration_s).min(pattern.duration_s());
                let future_index =
                    ((future / sample_time_s).floor() as usize).min(pattern.sample_count() - 1);
                let dcm_reference = Horizontal::new(
                    pattern.com_m[future_index].x_m
                        + pattern.com_velocity_m_s[future_index].x_m / omega,
                    pattern.com_m[future_index].z_m
                        + pattern.com_velocity_m_s[future_index].z_m / omega,
                );
                let cycle_decay = (omega * phase.duration_s).exp();
                let u_estimated =
                    (dcm_reference - dcm_end * cycle_decay) * (1.0 / (1.0 - cycle_decay));
                let nominal = pattern
                    .plan
                    .footsteps
                    .get(phase.index + 1)
                    .map(|foot| foot.position_m)
                    .unwrap_or(u_estimated);
                let raw = (u_estimated - nominal) * config.dcm_foot_placement_gain;
                let magnitude = raw.norm();
                if magnitude > config.max_foot_mod_m {
                    raw * (config.max_foot_mod_m / magnitude)
                } else {
                    raw
                }
            }
            _ => Horizontal::ZERO,
        };
        let (left_foot, right_foot) = planned_feet(&pattern, &config, motion_time_s, swing_mod);
        // `planned_feet` returns the lift above the neutral foot height, not an
        // absolute world height.
        let left_foot = Vec3::new(
            left_foot.x,
            neutral_relative[0].y + pelvis_start.translation.y + left_foot.y,
            left_foot.z,
        );
        let right_foot = Vec3::new(
            right_foot.x,
            neutral_relative[1].y + pelvis_start.translation.y + right_foot.y,
            right_foot.z,
        );
        let mut targets = stand;
        for (side, foot, sign) in [
            ("left", left_foot, 1.0_f64),
            ("right", right_foot, -1.0_f64),
        ] {
            let delta =
                (foot - pelvis_target) - neutral_relative[if side == "left" { 0 } else { 1 }];
            let (hip_pitch, knee, ankle_pitch) =
                g1_leg_ik(neutral_forward + delta.z, neutral_down - delta.y);
            let hip_roll =
                sign * G1_NEUTRAL_HIP_ROLL_RAD + sign * delta.x.atan2(neutral_down + 0.30);
            let names = if side == "left" {
                [
                    "left_hip_pitch_link",
                    "left_hip_roll_link",
                    "left_hip_yaw_link",
                    "left_knee_link",
                    "left_ankle_pitch_link",
                    "left_ankle_roll_link",
                ]
            } else {
                [
                    "right_hip_pitch_link",
                    "right_hip_roll_link",
                    "right_hip_yaw_link",
                    "right_knee_link",
                    "right_ankle_pitch_link",
                    "right_ankle_roll_link",
                ]
            };
            let values = [hip_pitch, hip_roll, 0.0, knee, ankle_pitch, -hip_roll];
            for (name, value) in names.iter().zip(values) {
                if let Some(target) = targets.iter_mut().find(|target| target.link_name == *name) {
                    target.position = value;
                }
            }
        }
        // Whole-body balance layer: solve the leg torques that hold the center
        // of mass on the LIPM reference over the measured contacts, and let the
        // arms and waist stay position-servoed.
        let q_full = read_g1_state(&sim, &model, &names);
        let (q_model, qd_model) = q_full.split_at(nv);
        if let Some((index, value)) = q_full
            .iter()
            .enumerate()
            .find(|(_, value)| !value.is_finite())
        {
            eprintln!(
                "non-finite q[{index}]={value} names_len={} nv={nv}",
                names.len()
            );
            eprintln!("names={names:?}");
            break;
        }
        // The support set comes from the plan: the stance foot during single
        // support, both feet during the weight shift, double support, and the
        // settle tail.
        let support: Vec<&str> = match if config.stand_only {
            None
        } else {
            step_phase(&config, motion_time_s)
        } {
            Some(phase) => {
                if phase.swing_left {
                    vec!["right_ankle_roll_link"]
                } else {
                    vec!["left_ankle_roll_link"]
                }
            }
            None => vec!["left_ankle_roll_link", "right_ankle_roll_link"],
        };
        let mut contacts = Vec::new();
        for link_name in support {
            if let Some(link) = model.kinematic().link_entity_by_name(link_name) {
                for point in G1_FOOT_CONTACTS_LOCAL_M {
                    contacts.push(ContactPoint::new(link, point, 0.7));
                }
            }
        }
        let mut desired = vec![0.0; control_dim];
        for (dof, name) in names.iter().enumerate() {
            desired[dof] = targets
                .iter()
                .find(|target| target.link_name == *name)
                .map(|target| target.position)
                .unwrap_or(0.0);
        }
        let com_task = ComTask {
            desired_position_m: Vec3::new(com_reference.x_m, com_origin.y, com_reference.z_m),
            desired_velocity_m_s: Vec3::new(com_velocity.x_m, 0.0, com_velocity.z_m),
            desired_acceleration_m_s2: Vec3::ZERO,
            position_gain_s_inv2: config.wbc_com_gain,
            velocity_gain_s_inv: config.wbc_com_gain.sqrt(),
        };
        let pose = sim.named_transform("pelvis").expect("pelvis");
        let up_world = (pose.rotation * Vec3::Z).normalize_or_zero();
        let tilt_axis = Vec3::Y.cross(up_world);
        let sin_angle = tilt_axis.length();
        let tilt_angle = up_world.y.clamp(-1.0, 1.0).acos();
        let axis_body = if sin_angle > 1.0e-6 {
            pose.rotation.inverse() * (tilt_axis / sin_angle)
        } else {
            Vec3::ZERO
        };
        let observation = sim.observe();
        let omega_body = pose.rotation.inverse()
            * Vec3::new(
                observation.base_angular_velocity_x_rad_s,
                observation.base_angular_velocity_y_rad_s,
                observation.base_angular_velocity_z_rad_s,
            );
        let attitude = BaseAttitudeTask {
            desired_angular_acceleration_rad_s2: axis_body * (200.0 * tilt_angle)
                - omega_body * 16.0,
        };
        let posture = PostureTask {
            desired_joint_positions: desired,
            desired_joint_velocities: None,
            desired_joint_accelerations: None,
            position_gain_s_inv2: 25.0,
            velocity_gain_s_inv: 5.0,
        };
        let solution = wbc
            .solve(
                &model,
                q_model,
                qd_model,
                &contacts,
                Some(&com_task),
                Some(&attitude),
                Some(&posture),
            )
            .expect("G1 whole-body solve");
        torque_saturated |= solution.torque_saturated;
        for (name, dof) in G1_TORQUE_LINKS.iter().map(|name| {
            (
                name,
                leg_dofs[G1_LEG_LINKS.iter().position(|n| n == name).unwrap_or(0)],
            )
        }) {
            let ratio = solution.joint_torque_nm[dof].abs() / g1_effort_limit_nm(name);
            max_torque_ratio = max_torque_ratio.max(ratio);
        }
        let servo: Vec<UrdfJointPositionTarget<'_>> = targets
            .iter()
            .filter(|target| !G1_TORQUE_LINKS.contains(&target.link_name))
            .copied()
            .collect();
        sim.set_joint_position_targets(&servo);
        let torques: Vec<UrdfJointTorqueTarget<'_>> = G1_TORQUE_LINKS
            .iter()
            .map(|link_name| {
                let leg_index = G1_LEG_LINKS
                    .iter()
                    .position(|name| name == link_name)
                    .unwrap_or(0);
                let dof = leg_dofs[leg_index];
                let limit = g1_effort_limit_nm(link_name);
                UrdfJointTorqueTarget {
                    link_name,
                    torque_nm: solution.joint_torque_nm[dof].clamp(-limit, limit),
                    max_velocity_rad_s: 30.0,
                }
            })
            .collect();
        sim.step_joint_torques(&torques);
        let pelvis = sim.named_transform("pelvis").expect("pelvis");
        let up = (pelvis.rotation * up_reference).normalize_or_zero();
        let tilt = up.y.clamp(-1.0, 1.0).acos();
        max_tilt_rad = max_tilt_rad.max(tilt);
        min_height_m = min_height_m.min(pelvis.translation.y);
        let error = ((pelvis.translation.x - pelvis_target.x).powi(2)
            + (pelvis.translation.z - pelvis_target.z).powi(2))
        .sqrt();
        max_tracking_error_m = max_tracking_error_m.max(error);
        if pelvis.translation.y < 0.5 * start_height {
            fell = true;
        }
        if config.trace {
            println!(
                "  t={:5.2} pelx={:+.3} tgtx={:+.3} comrefx={:+.3} planLx={:+.3} actLx={:+.3} tgtz={:+.3} tilt={:.3}",
                time_s,
                pelvis.translation.x,
                pelvis_target.x,
                com_reference.x_m,
                left_foot.x,
                sim.named_transform("left_ankle_roll_link").expect("l").translation.x,
                pelvis_target.z,
                tilt,
            );
        }
        digest = digest.wrapping_mul(0x100_0000_01b3).wrapping_add(
            pelvis
                .translation
                .to_array()
                .iter()
                .fold(0_u64, |acc, value| {
                    acc.wrapping_mul(31)
                        .wrapping_add((value * 1.0e6) as i64 as u64)
                }),
        );
    }

    let pelvis_end = sim.named_transform("pelvis").expect("pelvis");
    let measured_pelvis_m = Horizontal::new(pelvis_end.translation.x, pelvis_end.translation.z);
    let planned_com_m = pattern.com_m.last().copied().unwrap_or(Horizontal::ZERO);
    Ok(UnitreeG1LipmWalkOutcome {
        pattern_duration_s: pattern.duration_s(),
        planned_com_m,
        measured_pelvis_m,
        forward_distance_m: pelvis_end.translation.x - start_horizontal.x_m,
        min_height_m,
        max_tilt_rad,
        max_tracking_error_m,
        max_torque_ratio,
        torque_saturated,
        fell,
        digest,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leg_ik_reproduces_the_neutral_pose() {
        let (forward, down) = neutral_ankle_offset();
        let (hip_pitch, knee, ankle_pitch) = g1_leg_ik(forward, down);
        assert!(
            (hip_pitch - G1_NEUTRAL_HIP_PITCH_RAD).abs() < 1.0e-9,
            "{hip_pitch}"
        );
        assert!((knee - G1_NEUTRAL_KNEE_RAD).abs() < 1.0e-9, "{knee}");
        assert!(
            (ankle_pitch + G1_NEUTRAL_HIP_PITCH_RAD + G1_NEUTRAL_KNEE_RAD).abs() < 1.0e-9,
            "{ankle_pitch}"
        );
    }

    #[test]
    fn leg_ik_shortens_the_leg_as_the_foot_rises() {
        let (forward, down) = neutral_ankle_offset();
        let (_, straight_knee, _) = g1_leg_ik(forward, down + 0.10);
        let (_, bent_knee, _) = g1_leg_ik(forward, down - 0.10);
        assert!(straight_knee < G1_NEUTRAL_KNEE_RAD, "{straight_knee}");
        assert!(bent_knee > G1_NEUTRAL_KNEE_RAD, "{bent_knee}");
    }

    #[test]
    fn leg_ik_is_finite_for_degenerate_inputs() {
        let (hip, knee, ankle) = g1_leg_ik(f64::NAN, f64::NAN);
        assert!(hip.is_finite() && knee.is_finite() && ankle.is_finite());
        let (hip, knee, ankle) = g1_leg_ik(10.0, 10.0);
        assert!(hip.is_finite() && knee.is_finite() && ankle.is_finite());
    }
}

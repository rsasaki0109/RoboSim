//! Headless evaluation harness for the command-conditioned official G1 gait.

use super::{
    unitree_g1_dynamic_scene_path, unitree_g1_gait_targets_for_velocity_with_yaw_stride_phase,
    UnitreeG1CommandedTorquePolicy, UnitreeG1GaitCommand, UnitreeG1VelocityCommand,
    UnitreeG1VelocityPolicyInput, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_assets::AssetError;
use rne_math::Vec3;
use std::path::PathBuf;

/// The eight G1 proximal links controlled by the hybrid torque policy.
pub const UNITREE_G1_TORQUE_LINKS: [&str; 8] = [
    "left_hip_pitch_link",
    "left_hip_roll_link",
    "left_hip_yaw_link",
    "left_knee_link",
    "right_hip_pitch_link",
    "right_hip_roll_link",
    "right_hip_yaw_link",
    "right_knee_link",
];

/// G1 servo-held-joint stiffness used by the validated hybrid gait.
pub const UNITREE_G1_POSITION_STIFFNESS: f64 = 220.0;
/// G1 servo-held-joint damping used by the validated hybrid gait.
pub const UNITREE_G1_POSITION_DAMPING: f64 = 24.0;
/// G1 proximal torque-PD stiffness used by the validated hybrid gait.
pub const UNITREE_G1_TORQUE_PD_STIFFNESS: f64 = 300.0;
/// G1 proximal torque-PD damping used by the validated hybrid gait.
pub const UNITREE_G1_TORQUE_PD_DAMPING: f64 = 10.0;
/// Conservative G1 actuator torque ceiling for the evaluation harness.
pub const UNITREE_G1_TORQUE_LIMIT_NM: f64 = 88.0;
/// G1 joint speed ceiling used while converting torque commands to motor targets.
pub const UNITREE_G1_SPEED_LIMIT_RAD_S: f64 = 30.0;
/// Fixed simulation interval used by the commanded G1 evaluation in seconds.
pub const UNITREE_G1_SIM_DT_S: f64 = 1.0 / 60.0;

/// Configuration for one deterministic, headless command-conditioned G1 run.
#[derive(Clone, Debug, PartialEq)]
pub struct UnitreeG1CommandedGaitConfig {
    /// Dynamic multibody G1 scene to load.
    pub scene_path: PathBuf,
    /// Nominal scripted gait whose stride and lift are scaled by the velocity command.
    pub base_command: UnitreeG1GaitCommand,
    /// Differential left/right stride scale per commanded yaw rate in seconds.
    pub yaw_stride_scale_per_rad_s: f64,
    /// Right-leg phase offset per commanded yaw rate in seconds.
    pub yaw_phase_offset_s_per_rad_s: f64,
    /// Torso-yaw target per commanded yaw rate in rad/(rad/s).
    pub yaw_torso_target_rad_per_rad_s: f64,
    /// Differential hip-roll target per commanded yaw rate in rad/(rad/s).
    pub yaw_hip_roll_target_rad_per_rad_s: f64,
    /// Additional differential hip-yaw target per commanded yaw rate in
    /// rad/(rad/s), applied to the swing/stance leg targets.
    pub yaw_hip_yaw_target_rad_per_rad_s: f64,
    /// Differential hip-yaw target per commanded yaw rate applied only while
    /// the corresponding foot is in swing, in rad/(rad/s).
    pub yaw_swing_hip_yaw_target_rad_per_rad_s: f64,
    /// Differential hip-roll target per commanded yaw rate applied only to
    /// the swing leg, in rad/(rad/s).
    pub yaw_swing_hip_roll_target_rad_per_rad_s: f64,
    /// Right hip-yaw target sign; `-1` is differential and `+1` is common-mode.
    pub yaw_hip_yaw_right_sign: f64,
    /// Mirrors the nominal gait for negative yaw commands.
    pub mirror_negative_yaw: bool,
    /// Constant velocity command used during the rollout.
    pub command: UnitreeG1VelocityCommand,
    /// Position-controlled settling ticks before locomotion begins.
    pub settle_steps: u64,
    /// Number of 60 Hz locomotion ticks to evaluate.
    pub rollout_steps: u64,
    /// Optional absolute accumulated-heading target clamp in radians. A
    /// positive finite value creates a bounded heading hold; non-finite values
    /// retain the integrated yaw-rate reference.
    pub heading_target_clamp_rad: f64,
    /// Position-motor stiffness for the hybrid gait's servo-held joints.
    pub position_stiffness: f64,
    /// Position-motor damping for the hybrid gait's servo-held joints.
    pub position_damping: f64,
    /// Proximal torque-PD stiffness in N·m/rad.
    pub torque_pd_stiffness: f64,
    /// Proximal torque-PD damping in N·m/(rad/s).
    pub torque_pd_damping: f64,
    /// Absolute actuator torque limit in N·m.
    pub torque_limit_nm: f64,
    /// Joint speed limit in rad/s used by the torque motor adapter.
    pub speed_limit_rad_s: f64,
    /// Base height below which the run is marked fallen.
    pub fall_height_m: f64,
    /// Upright tilt above which the run is marked fallen.
    pub fall_tilt_rad: f64,
    /// Optional locomotion tick at which to apply a root-body tilt disturbance.
    pub disturbance_step: Option<u64>,
    /// World-frame axis-angle disturbance applied at `disturbance_step`.
    pub disturbance_axis_angle_rad: [f64; 3],
}

impl Default for UnitreeG1CommandedGaitConfig {
    fn default() -> Self {
        Self {
            scene_path: unitree_g1_dynamic_scene_path(),
            base_command: UnitreeG1GaitCommand {
                stride_rad: 0.065,
                foot_lift_rad: 0.12,
                cycle_steps: 100,
            },
            yaw_stride_scale_per_rad_s: 0.0,
            yaw_phase_offset_s_per_rad_s: 0.0,
            yaw_torso_target_rad_per_rad_s: 0.0,
            yaw_hip_roll_target_rad_per_rad_s: 0.0,
            yaw_hip_yaw_target_rad_per_rad_s: 0.0,
            yaw_swing_hip_yaw_target_rad_per_rad_s: 0.0,
            yaw_swing_hip_roll_target_rad_per_rad_s: 0.0,
            yaw_hip_yaw_right_sign: -1.0,
            mirror_negative_yaw: true,
            command: UnitreeG1VelocityCommand {
                forward_m_s: 0.0276,
                yaw_rate_rad_s: 0.0,
            },
            settle_steps: 240,
            rollout_steps: 1440,
            heading_target_clamp_rad: f64::INFINITY,
            position_stiffness: UNITREE_G1_POSITION_STIFFNESS,
            position_damping: UNITREE_G1_POSITION_DAMPING,
            torque_pd_stiffness: UNITREE_G1_TORQUE_PD_STIFFNESS,
            torque_pd_damping: UNITREE_G1_TORQUE_PD_DAMPING,
            torque_limit_nm: UNITREE_G1_TORQUE_LIMIT_NM,
            speed_limit_rad_s: UNITREE_G1_SPEED_LIMIT_RAD_S,
            fall_height_m: 0.60,
            fall_tilt_rad: 0.50,
            disturbance_step: None,
            disturbance_axis_angle_rad: [0.0, 0.0, 0.04],
        }
    }
}

/// Measured metrics from one command-conditioned G1 replay.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeG1CommandedGaitOutcome {
    /// Command used after v0.1 envelope clamping.
    pub command: UnitreeG1VelocityCommand,
    /// Number of locomotion ticks completed.
    pub steps: u64,
    /// World X displacement in meters.
    pub base_x_displacement_m: f64,
    /// World Z displacement in meters.
    pub base_z_displacement_m: f64,
    /// Straight-line displacement in the ground plane in meters.
    pub total_displacement_m: f64,
    /// Mean body-forward velocity over the rollout in m/s.
    pub mean_forward_velocity_m_s: f64,
    /// Mean body yaw rate over the rollout in rad/s.
    pub mean_yaw_rate_rad_s: f64,
    /// Unwrapped accumulated body yaw in radians.
    pub total_yaw_rad: f64,
    /// Target accumulated body heading at the end of the rollout in radians.
    pub target_heading_rad: f64,
    /// Final heading error `target_heading_rad - total_yaw_rad` in radians.
    pub heading_error_rad: f64,
    /// Mean absolute accumulated-heading error in radians.
    pub mean_abs_heading_error_rad: f64,
    /// Mean absolute yaw-rate tracking error in rad/s.
    pub mean_abs_yaw_rate_error_rad_s: f64,
    /// Estimated turn radius from mean forward speed and absolute yaw rate.
    /// `None` denotes a near-zero measured yaw rate.
    pub turn_radius_m: Option<f64>,
    /// Lowest observed pelvis height in meters.
    pub min_height_m: f64,
    /// Largest true body-up tilt from the settled reference in radians.
    pub max_tilt_rad: f64,
    /// Largest absolute commanded proximal-joint torque in N·m.
    pub max_command_nm: f64,
    /// Whether the configured fall predicate was reached.
    pub fell: bool,
    /// Whether the requested root-body disturbance was successfully injected.
    pub disturbance_applied: bool,
    /// Stable digest of the observed replay trajectory.
    pub replay_digest: u64,
}

/// Runs the default command-conditioned G1 policy on the dynamic scene.
pub fn run_unitree_g1_commanded_gait(
    config: UnitreeG1CommandedGaitConfig,
) -> Result<UnitreeG1CommandedGaitOutcome, AssetError> {
    run_unitree_g1_commanded_gait_with_policy(config, UnitreeG1CommandedTorquePolicy::default())
}

/// Runs one command-conditioned G1 replay with an explicit policy candidate.
///
/// This is the evaluation boundary used by the deterministic CEM example. It
/// keeps scene loading, settling, contact observation, hybrid motor assembly,
/// disturbance injection, and metrics identical for every candidate.
pub fn run_unitree_g1_commanded_gait_with_policy(
    config: UnitreeG1CommandedGaitConfig,
    policy: UnitreeG1CommandedTorquePolicy,
) -> Result<UnitreeG1CommandedGaitOutcome, AssetError> {
    let mut sim = UrdfSceneSim::from_scene_path(&config.scene_path)?;
    sim.configure_position_motors(
        config.position_stiffness,
        config.position_damping,
        config.torque_limit_nm,
    );
    let stand = unitree_g1_gait_targets_for_velocity_with_yaw_stride_phase(
        0,
        config.base_command,
        UnitreeG1VelocityCommand::default(),
        0.0,
        0.0,
    );
    for _ in 0..config.settle_steps {
        sim.step_joint_position_targets(&stand);
    }

    let up_body_reference = {
        let pose = sim.named_transform("pelvis").expect("G1 pelvis pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    let start = sim.observe();
    let mut previous_yaw = start.base_relative_yaw_rad;
    let mut total_yaw_rad = 0.0;
    let mut heading_error_sum_rad = 0.0;
    let mut yaw_rate_error_sum_rad_s = 0.0;
    let mut abs_yaw_rate_sum_rad_s = 0.0;
    let mut forward_velocity_sum = 0.0;
    let mut yaw_rate_sum = 0.0;
    let mut min_height_m = start.base_y_m;
    let mut max_tilt_rad: f64 = 0.0;
    let mut max_command_nm: f64 = 0.0;
    let mut fell = false;
    let mut disturbance_applied = false;
    let mut replay_digest = 0xcbf29ce484222325;
    let command = config.command.clamped();

    for step in 0..config.rollout_steps {
        if config.disturbance_step == Some(step) {
            disturbance_applied =
                sim.tilt_named_body_rad("pelvis", config.disturbance_axis_angle_rad);
        }

        let stance = [
            sim.link_contact_impulse_ns("left_ankle_roll_link") > 0.0,
            sim.link_contact_impulse_ns("right_ankle_roll_link") > 0.0,
        ];
        let mut targets = unitree_g1_gait_targets_for_velocity_with_yaw_stride_phase(
            step,
            config.base_command,
            command,
            config.yaw_stride_scale_per_rad_s,
            config.yaw_phase_offset_s_per_rad_s,
        );
        if config.yaw_torso_target_rad_per_rad_s.is_finite() {
            targets[12].position += config.yaw_torso_target_rad_per_rad_s * command.yaw_rate_rad_s;
        }
        if config.yaw_hip_roll_target_rad_per_rad_s.is_finite() {
            let hip_roll = config.yaw_hip_roll_target_rad_per_rad_s * command.yaw_rate_rad_s;
            targets[1].position += hip_roll;
            targets[7].position -= hip_roll;
        }
        if config.yaw_hip_yaw_target_rad_per_rad_s.is_finite() {
            let hip_yaw = (config.yaw_hip_yaw_target_rad_per_rad_s * command.yaw_rate_rad_s)
                .clamp(-0.35, 0.35);
            targets[2].position += hip_yaw;
            targets[8].position -= hip_yaw;
        }
        if config.yaw_swing_hip_yaw_target_rad_per_rad_s.is_finite() {
            let swing_hip_yaw = (config.yaw_swing_hip_yaw_target_rad_per_rad_s
                * command.yaw_rate_rad_s)
                .clamp(-0.35, 0.35);
            if !stance[0] {
                targets[2].position += swing_hip_yaw;
            }
            if !stance[1] {
                targets[8].position -= swing_hip_yaw;
            }
        }
        if config.yaw_swing_hip_roll_target_rad_per_rad_s.is_finite() {
            let swing_hip_roll = (config.yaw_swing_hip_roll_target_rad_per_rad_s
                * command.yaw_rate_rad_s)
                .clamp(-0.30, 0.30);
            if !stance[0] {
                targets[1].position += swing_hip_roll;
            }
            if !stance[1] {
                targets[7].position -= swing_hip_roll;
            }
        }
        if config.yaw_hip_yaw_right_sign.is_finite() {
            let target_gain = 0.45 * command.yaw_rate_rad_s;
            let right_sign = config.yaw_hip_yaw_right_sign.clamp(-1.0, 1.0);
            targets[8].position += (right_sign + 1.0) * target_gain;
        }
        if config.mirror_negative_yaw && command.yaw_rate_rad_s < 0.0 {
            mirror_targets_sagittally(&mut targets);
        }
        let servo: Vec<UrdfJointPositionTarget<'_>> = targets
            .iter()
            .filter(|target| !UNITREE_G1_TORQUE_LINKS.contains(&target.link_name))
            .copied()
            .collect();
        sim.set_joint_position_targets(&servo);

        let observation = sim.observe();
        let world_velocity = Vec3::new(
            observation.base_linear_velocity_x_m_s,
            observation.base_linear_velocity_y_m_s,
            observation.base_linear_velocity_z_m_s,
        );
        let body_rotation = sim
            .named_transform("pelvis")
            .expect("G1 pelvis pose")
            .rotation;
        let measured_forward_velocity_m_s = (body_rotation.inverse() * world_velocity).z;
        let measured_yaw_rate_rad_s = observation.base_angular_velocity_y_rad_s;
        let target_heading_rad = bounded_heading_target(
            command.yaw_rate_rad_s * (step + 1) as f64 * UNITREE_G1_SIM_DT_S,
            config.heading_target_clamp_rad,
        );
        let input = UnitreeG1VelocityPolicyInput {
            two_cycle_phase: two_cycle_phase(step, config.base_command.cycle_steps),
            stance,
            command,
            measured_forward_velocity_m_s,
            measured_yaw_rate_rad_s,
            target_heading_rad,
            measured_heading_rad: total_yaw_rad,
            heading_error_rad: target_heading_rad - total_yaw_rad,
            yaw_rate_error_rad_s: command.yaw_rate_rad_s - measured_yaw_rate_rad_s,
        };
        let feed_forward = policy.torques_nm_for_command(input, config.torque_limit_nm);
        max_command_nm = max_command_nm.max(
            feed_forward
                .iter()
                .map(|torque| torque.abs())
                .fold(0.0, f64::max),
        );
        let torques: Vec<UrdfJointTorqueTarget<'_>> = UNITREE_G1_TORQUE_LINKS
            .iter()
            .enumerate()
            .map(|(index, link_name)| {
                let target_position = targets
                    .iter()
                    .find(|target| target.link_name == *link_name)
                    .expect("G1 torque link in gait targets")
                    .position;
                let q = sim
                    .named_joint_position(link_name)
                    .expect("G1 joint position");
                let qd = sim
                    .named_joint_velocity(link_name)
                    .expect("G1 joint velocity");
                let torque_nm = (config.torque_pd_stiffness * (target_position - q)
                    - config.torque_pd_damping * qd
                    + feed_forward[index])
                    .clamp(-config.torque_limit_nm, config.torque_limit_nm);
                UrdfJointTorqueTarget {
                    link_name,
                    torque_nm,
                    max_velocity_rad_s: config.speed_limit_rad_s,
                }
            })
            .collect();
        sim.step_joint_torques(&torques);

        let observed = sim.observe();
        let tilt_rad = true_tilt_rad(&sim, up_body_reference);
        let mut yaw_delta = observed.base_relative_yaw_rad - previous_yaw;
        while yaw_delta > std::f64::consts::PI {
            yaw_delta -= 2.0 * std::f64::consts::PI;
        }
        while yaw_delta < -std::f64::consts::PI {
            yaw_delta += 2.0 * std::f64::consts::PI;
        }
        total_yaw_rad += yaw_delta;
        previous_yaw = observed.base_relative_yaw_rad;
        let post_world_velocity = Vec3::new(
            observed.base_linear_velocity_x_m_s,
            observed.base_linear_velocity_y_m_s,
            observed.base_linear_velocity_z_m_s,
        );
        let post_rotation = sim
            .named_transform("pelvis")
            .expect("G1 pelvis pose")
            .rotation;
        let post_forward_velocity_m_s = (post_rotation.inverse() * post_world_velocity).z;
        forward_velocity_sum += post_forward_velocity_m_s;
        yaw_rate_sum += observed.base_angular_velocity_y_rad_s;
        heading_error_sum_rad += (target_heading_rad - total_yaw_rad).abs();
        yaw_rate_error_sum_rad_s +=
            (command.yaw_rate_rad_s - observed.base_angular_velocity_y_rad_s).abs();
        abs_yaw_rate_sum_rad_s += observed.base_angular_velocity_y_rad_s.abs();
        min_height_m = min_height_m.min(observed.base_y_m);
        max_tilt_rad = max_tilt_rad.max(tilt_rad);
        fell |= observed.base_y_m < config.fall_height_m || tilt_rad > config.fall_tilt_rad;
        digest_mix(&mut replay_digest, step as f64);
        digest_mix(&mut replay_digest, observed.base_x_m);
        digest_mix(&mut replay_digest, observed.base_y_m);
        digest_mix(&mut replay_digest, observed.base_z_m);
        digest_mix(&mut replay_digest, observed.base_relative_yaw_rad);
        digest_mix(&mut replay_digest, post_forward_velocity_m_s);
        digest_mix(
            &mut replay_digest,
            (sim.link_contact_impulse_ns("left_ankle_roll_link") > 0.0) as u8 as f64,
        );
        digest_mix(
            &mut replay_digest,
            (sim.link_contact_impulse_ns("right_ankle_roll_link") > 0.0) as u8 as f64,
        );
    }

    let end = sim.observe();
    let completed_steps = config.rollout_steps.max(1) as f64;
    let mean_forward_velocity_m_s = forward_velocity_sum / completed_steps;
    let mean_abs_yaw_rate_rad_s = abs_yaw_rate_sum_rad_s / completed_steps;
    Ok(UnitreeG1CommandedGaitOutcome {
        command,
        steps: config.rollout_steps,
        base_x_displacement_m: end.base_x_m - start.base_x_m,
        base_z_displacement_m: end.base_z_m - start.base_z_m,
        total_displacement_m: (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m),
        mean_forward_velocity_m_s,
        mean_yaw_rate_rad_s: yaw_rate_sum / completed_steps,
        total_yaw_rad,
        target_heading_rad: bounded_heading_target(
            command.yaw_rate_rad_s * config.rollout_steps as f64 * UNITREE_G1_SIM_DT_S,
            config.heading_target_clamp_rad,
        ),
        heading_error_rad: bounded_heading_target(
            command.yaw_rate_rad_s * config.rollout_steps as f64 * UNITREE_G1_SIM_DT_S,
            config.heading_target_clamp_rad,
        ) - total_yaw_rad,
        mean_abs_heading_error_rad: heading_error_sum_rad / completed_steps,
        mean_abs_yaw_rate_error_rad_s: yaw_rate_error_sum_rad_s / completed_steps,
        turn_radius_m: (mean_abs_yaw_rate_rad_s > 1.0e-9)
            .then_some(mean_forward_velocity_m_s.abs() / mean_abs_yaw_rate_rad_s),
        min_height_m,
        max_tilt_rad,
        max_command_nm,
        fell,
        disturbance_applied,
        replay_digest,
    })
}

fn bounded_heading_target(integrated_heading_rad: f64, clamp_rad: f64) -> f64 {
    if clamp_rad.is_finite() && clamp_rad > 0.0 {
        integrated_heading_rad.clamp(-clamp_rad, clamp_rad)
    } else {
        integrated_heading_rad
    }
}

fn two_cycle_phase(step: u64, cycle_steps: u64) -> f64 {
    let cycle_steps = cycle_steps.clamp(40, 180);
    (step % (2 * cycle_steps)) as f64 / (2 * cycle_steps) as f64
}

fn true_tilt_rad(sim: &UrdfSceneSim, up_body_reference: Vec3) -> f64 {
    let pose = sim.named_transform("pelvis").expect("G1 pelvis pose");
    let up = (pose.rotation * up_body_reference).normalize_or_zero();
    up.y.clamp(-1.0, 1.0).acos()
}

fn mirror_targets_sagittally(targets: &mut [UrdfJointPositionTarget<'static>; 23]) {
    const PAIRS: [(usize, usize, f64); 10] = [
        (0, 6, 1.0),
        (1, 7, -1.0),
        (2, 8, -1.0),
        (3, 9, 1.0),
        (4, 10, 1.0),
        (5, 11, -1.0),
        (13, 18, 1.0),
        (14, 19, -1.0),
        (15, 20, -1.0),
        (16, 21, 1.0),
    ];
    for (left, right, sign) in PAIRS {
        let left_position = targets[left].position;
        let right_position = targets[right].position;
        targets[left].position = sign * right_position;
        targets[right].position = sign * left_position;
    }
    let left_wrist = targets[17].position;
    targets[17].position = targets[22].position;
    targets[22].position = left_wrist;
}

fn digest_mix(digest: &mut u64, value: f64) {
    *digest ^= value.to_bits();
    *digest = digest.wrapping_mul(0x00000100000001b3);
}

/// Feed-forward scale applied to [`crate::UnitreeG1TorqueOverlay::LEARNED_STRIDE`] in
/// scripted walk segments that are not driven by a velocity command policy.
pub const UNITREE_G1_LEARNED_STRIDE_OVERLAY_SCALE: f64 = 0.66;

/// Advances one 60 Hz tick using the validated G1 hybrid control split.
///
/// Ankles and arms track `targets` through position motors updated without
/// stepping; hips and knees track the same targets through torque PD with
/// optional proximal feed-forward torques in [`UNITREE_G1_TORQUE_LINKS`] order.
pub fn step_unitree_g1_hybrid_joint_targets(
    sim: &mut UrdfSceneSim,
    targets: &[UrdfJointPositionTarget<'_>],
    feed_forward_nm: [f64; 8],
) {
    step_unitree_g1_hybrid_joint_targets_with_limits(
        sim,
        targets,
        feed_forward_nm,
        UNITREE_G1_TORQUE_PD_STIFFNESS,
        UNITREE_G1_TORQUE_PD_DAMPING,
        UNITREE_G1_TORQUE_LIMIT_NM,
        UNITREE_G1_SPEED_LIMIT_RAD_S,
    );
}

/// Hybrid G1 tick with explicit PD and actuator limits.
pub fn step_unitree_g1_hybrid_joint_targets_with_limits(
    sim: &mut UrdfSceneSim,
    targets: &[UrdfJointPositionTarget<'_>],
    feed_forward_nm: [f64; 8],
    kp: f64,
    kd: f64,
    torque_limit_nm: f64,
    speed_limit_rad_s: f64,
) {
    let servo: Vec<_> = targets
        .iter()
        .filter(|target| !UNITREE_G1_TORQUE_LINKS.contains(&target.link_name))
        .copied()
        .collect();
    sim.set_joint_position_targets(&servo);
    let torques: Vec<UrdfJointTorqueTarget<'_>> = UNITREE_G1_TORQUE_LINKS
        .iter()
        .enumerate()
        .map(|(index, link_name)| {
            let target_position = targets
                .iter()
                .find(|target| target.link_name == *link_name)
                .expect("G1 torque link in joint targets")
                .position;
            let q = sim
                .named_joint_position(link_name)
                .expect("G1 joint position");
            let qd = sim
                .named_joint_velocity(link_name)
                .expect("G1 joint velocity");
            UrdfJointTorqueTarget {
                link_name,
                torque_nm: (kp * (target_position - q) - kd * qd + feed_forward_nm[index])
                    .clamp(-torque_limit_nm, torque_limit_nm),
                max_velocity_rad_s: speed_limit_rad_s,
            }
        })
        .collect();
    sim.step_joint_torques(&torques);
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn phase_is_two_cycle_and_clamped() {
        assert_eq!(two_cycle_phase(0, 100), 0.0);
        assert_eq!(two_cycle_phase(100, 100), 0.5);
        assert_eq!(two_cycle_phase(200, 100), 0.0);
        assert_eq!(two_cycle_phase(0, 0), 0.0);
    }

    #[test]
    fn default_commanded_gait_config_is_finite_and_validated() {
        let config = UnitreeG1CommandedGaitConfig::default();
        assert!(config.scene_path.is_file());
        assert!(config.rollout_steps > 0);
        assert!(config.fall_height_m > 0.0);
        assert!(config.fall_tilt_rad > 0.0);
        assert!(config
            .disturbance_axis_angle_rad
            .iter()
            .all(|value| value.is_finite()));
    }

    #[test]
    fn commanded_gait_replay_digest_is_deterministic() {
        let config = UnitreeG1CommandedGaitConfig {
            settle_steps: 60,
            rollout_steps: 120,
            ..UnitreeG1CommandedGaitConfig::default()
        };

        let first = run_unitree_g1_commanded_gait(config.clone()).expect("first G1 replay");
        let second = run_unitree_g1_commanded_gait(config).expect("second G1 replay");

        assert!(!first.fell);
        assert_eq!(first, second);
    }

    #[test]
    fn commanded_steering_envelope_stays_upright_and_changes_path_sign() {
        let config = UnitreeG1CommandedGaitConfig {
            settle_steps: 60,
            rollout_steps: 240,
            ..UnitreeG1CommandedGaitConfig::default()
        };
        let forward = UnitreeG1VelocityCommand {
            forward_m_s: 0.0276,
            yaw_rate_rad_s: 0.0,
        };
        let left = UnitreeG1VelocityCommand {
            forward_m_s: 0.0276,
            yaw_rate_rad_s: 0.05,
        };
        let right = UnitreeG1VelocityCommand {
            forward_m_s: 0.0276,
            yaw_rate_rad_s: -0.05,
        };
        let forward_outcome = run_unitree_g1_commanded_gait_with_policy(
            UnitreeG1CommandedGaitConfig {
                command: forward,
                ..config.clone()
            },
            UnitreeG1CommandedTorquePolicy::default(),
        )
        .expect("forward G1 envelope");
        let left_outcome = run_unitree_g1_commanded_gait_with_policy(
            UnitreeG1CommandedGaitConfig {
                command: left,
                ..config.clone()
            },
            UnitreeG1CommandedTorquePolicy::default(),
        )
        .expect("left G1 envelope");
        let right_outcome = run_unitree_g1_commanded_gait_with_policy(
            UnitreeG1CommandedGaitConfig {
                command: right,
                ..config
            },
            UnitreeG1CommandedTorquePolicy::default(),
        )
        .expect("right G1 envelope");

        for outcome in [forward_outcome, left_outcome, right_outcome] {
            assert!(!outcome.fell);
            assert!(outcome.min_height_m > 0.70);
            assert!(outcome.max_tilt_rad < 0.50);
        }
        assert!(forward_outcome.total_displacement_m > 0.02);
        assert!(left_outcome.base_z_displacement_m > 0.0);
        assert!(right_outcome.base_z_displacement_m < 0.0);
    }

    #[test]
    fn light_pelvis_disturbance_is_applied_without_falling() {
        let config = UnitreeG1CommandedGaitConfig {
            settle_steps: 60,
            rollout_steps: 240,
            disturbance_step: Some(120),
            disturbance_axis_angle_rad: [0.0, 0.0, 0.02],
            ..UnitreeG1CommandedGaitConfig::default()
        };
        let outcome = run_unitree_g1_commanded_gait(config).expect("disturbed G1 envelope");
        assert!(outcome.disturbance_applied);
        assert!(!outcome.fell);
        assert!(outcome.min_height_m > 0.70);
        assert!(outcome.max_tilt_rad < 0.50);
    }

    #[test]
    fn v02_heading_candidate_flips_body_yaw_sign_and_reports_turn_metrics() {
        let candidate = UnitreeG1CommandedTorquePolicy {
            yaw_rate_kp_nm_per_rad_s: 32.0,
            max_yaw_torque_nm: 16.0,
            negative_yaw_rate_gain_scale: 0.5,
            mirror_yaw_overlay_negative: false,
            ..UnitreeG1CommandedTorquePolicy::default()
        };
        let base = UnitreeG1CommandedGaitConfig {
            settle_steps: 60,
            rollout_steps: 240,
            mirror_negative_yaw: false,
            yaw_hip_yaw_right_sign: -1.0,
            yaw_hip_yaw_target_rad_per_rad_s: 0.0,
            heading_target_clamp_rad: 0.08,
            ..UnitreeG1CommandedGaitConfig::default()
        };
        let left = run_unitree_g1_commanded_gait_with_policy(
            UnitreeG1CommandedGaitConfig {
                command: UnitreeG1VelocityCommand {
                    forward_m_s: 0.0276,
                    yaw_rate_rad_s: 0.05,
                },
                ..base.clone()
            },
            candidate,
        )
        .expect("left heading replay");
        let right = run_unitree_g1_commanded_gait_with_policy(
            UnitreeG1CommandedGaitConfig {
                command: UnitreeG1VelocityCommand {
                    forward_m_s: 0.0276,
                    yaw_rate_rad_s: -0.05,
                },
                ..base.clone()
            },
            candidate,
        )
        .expect("right heading replay");

        for outcome in [left, right] {
            assert!(!outcome.fell);
            assert!(outcome.min_height_m > 0.75);
            assert!(outcome.max_command_nm <= 88.0);
            assert!(outcome.target_heading_rad.is_finite());
            assert!(outcome.heading_error_rad.is_finite());
            assert!(outcome.mean_abs_yaw_rate_error_rad_s.is_finite());
            assert!(outcome.turn_radius_m.is_some());
        }
        assert!(
            left.total_yaw_rad > 0.01,
            "left body yaw {:+.4}",
            left.total_yaw_rad
        );
        assert!(
            right.total_yaw_rad < -0.001,
            "right body yaw {:+.4}",
            right.total_yaw_rad
        );
        assert!(left.target_heading_rad > 0.0);
        assert!(right.target_heading_rad < 0.0);

        let replay = run_unitree_g1_commanded_gait_with_policy(
            UnitreeG1CommandedGaitConfig {
                command: UnitreeG1VelocityCommand {
                    forward_m_s: 0.0276,
                    yaw_rate_rad_s: 0.05,
                },
                ..base
            },
            candidate,
        )
        .expect("deterministic heading replay");
        assert_eq!(left, replay);
    }
}

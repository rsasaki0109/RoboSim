//! Model-based trot for the welded Unitree Go2 (`unitree_go2_jump`), on joint torques.
//!
//! This is the controller of example 126, as Pinocchio-based quadruped stacks run it:
//!
//! * **Kinematics.** Each leg's joint origins and axes come from the simulated link
//!   frames; the foot's linear Jacobian column for a revolute joint is
//!   `axis x (foot - joint)`.
//! * **Stance.** Each planted foot takes its share of the weight plus a virtual
//!   spring-damper on its own hip height, which levels pitch and roll leg by leg, and
//!   horizontal force toward the commanded velocity and yaw rate; joints apply
//!   `tau = -J^T f`.
//! * **Swing.** The foot lands where the Raibert heuristic says, along a lifted arc
//!   tracked by Cartesian PD, `tau = J^T (Kp e - Kd v)`.
//!
//! Torques are clamped to the URDF effort limits (23.7 N·m hip and thigh, 45.43 N·m
//! calf); joint speed is limited to 30.1 rad/s. The controller runs at
//! [`UNITREE_GO2_MODEL_TROT_CONTROL_HZ`]; [`UnitreeGo2ModelTrot::stand_up`] sets the
//! scene's step to match.

use super::{unitree_go2_trot_targets, UnitreeGo2GaitCommand, UrdfJointTorqueTarget, UrdfSceneSim};
use rne_core::SimDuration;
use rne_math::{Hertz, Vec3};

/// Control rate of the model-based trot, in hertz.
pub const UNITREE_GO2_MODEL_TROT_CONTROL_HZ: f64 = 500.0;

const GRAVITY_M_S2: f64 = 9.81;
/// Declared total mass of the welded Go2.
const MASS_KG: f64 = 16.1;
/// URDF effort limits: hip, thigh, calf.
const EFFORT_NM: [f64; 3] = [23.7, 23.7, 45.43];
const MAX_JOINT_SPEED_RAD_S: f64 = 30.1;
const LEGS: [[&str; 4]; 4] = [
    ["FL_hip", "FL_thigh", "FL_calf", "FL_foot"],
    ["FR_hip", "FR_thigh", "FR_calf", "FR_foot"],
    ["RL_hip", "RL_thigh", "RL_calf", "RL_foot"],
    ["RR_hip", "RR_thigh", "RR_calf", "RR_foot"],
];
/// Trot: diagonal pairs in phase.
const PHASE_OFFSETS: [f64; 4] = [0.0, 0.5, 0.5, 0.0];
const GAIT_PERIOD_S: f64 = 0.4;
const DUTY: f64 = 0.5;
const STANCE_HEIGHT_M: f64 = 0.30;
const STEP_HEIGHT_M: f64 = 0.06;
/// Foot sphere centre height when resting on the floor.
const FOOT_GROUND_Y_M: f64 = 0.022;
const HEIGHT_STIFFNESS_N_PER_M: f64 = 600.0;
const HEIGHT_DAMPING_N_S_PER_M: f64 = 40.0;
const VELOCITY_GAIN_N_S_PER_M: f64 = 300.0;
const YAW_RATE_GAIN_N_S: f64 = 30.0;
const SWING_STIFFNESS_N_PER_M: f64 = 400.0;
const SWING_DAMPING_N_S_PER_M: f64 = 12.0;
const RAIBERT_GAIN_S: f64 = 0.1;

/// Walking command for [`UnitreeGo2ModelTrot`].
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UnitreeGo2TrotCommand {
    /// Speed along the body's current facing, in m/s. Example 126 walks at 0.25.
    pub forward_speed_m_s: f64,
    /// Yaw rate about the world up axis, in rad/s; positive turns left.
    pub yaw_rate_rad_s: f64,
}

/// State of the model-based Go2 trot.
#[derive(Clone, Debug)]
pub struct UnitreeGo2ModelTrot {
    step: u64,
    liftoff: [Vec3; 4],
    in_stance: [bool; 4],
    previous_yaw_rad: f64,
    heading_rad: f64,
}

/// One leg's kinematic state, read from the simulated link frames.
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

impl UnitreeGo2ModelTrot {
    /// Stands the Go2 up on position servos at the scene's own rate, then switches the
    /// scene to [`UNITREE_GO2_MODEL_TROT_CONTROL_HZ`] for torque control.
    pub fn stand_up(sim: &mut UrdfSceneSim) -> Self {
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
        sim.set_fixed_delta(SimDuration::from_hertz(Hertz::new(
            UNITREE_GO2_MODEL_TROT_CONTROL_HZ,
        )));
        let previous_yaw_rad = sim.observe().base_relative_yaw_rad;
        Self {
            step: 0,
            liftoff: [Vec3::ZERO; 4],
            in_stance: [true; 4],
            previous_yaw_rad,
            heading_rad: 0.0,
        }
    }

    /// Controller steps taken since [`Self::stand_up`].
    pub fn steps(&self) -> u64 {
        self.step
    }

    /// Time since [`Self::stand_up`], in seconds.
    pub fn time_s(&self) -> f64 {
        self.step as f64 / UNITREE_GO2_MODEL_TROT_CONTROL_HZ
    }

    /// Unwrapped heading change since [`Self::stand_up`], in radians; positive is left.
    pub fn heading_rad(&self) -> f64 {
        self.heading_rad
    }

    /// Computes leg torques for `command`, steps the scene once, and tracks heading.
    pub fn step(&mut self, sim: &mut UrdfSceneSim, command: UnitreeGo2TrotCommand) {
        let t_s = self.time_s();
        let observed = sim.observe();
        let base = sim.named_transform("base").expect("base pose");
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
        let yaw_rate_cmd = command.yaw_rate_rad_s;
        let velocity_cmd = flat_facing * command.forward_speed_m_s;

        let phases: [f64; 4] =
            std::array::from_fn(|i| (t_s / GAIT_PERIOD_S + PHASE_OFFSETS[i]) % 1.0);
        let stance_count = phases.iter().filter(|p| **p < DUTY).count().max(1) as f64;
        let stance_s = GAIT_PERIOD_S * DUTY;
        let mut torques: Vec<UrdfJointTorqueTarget<'_>> = Vec::with_capacity(12);
        for (index, names) in LEGS.iter().enumerate() {
            let leg = Leg::read(sim, *names);
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
                    max_velocity_rad_s: MAX_JOINT_SPEED_RAD_S,
                });
            }
        }
        sim.step_joint_torques(&torques);

        let after = sim.observe();
        let mut delta = after.base_relative_yaw_rad - self.previous_yaw_rad;
        while delta > std::f64::consts::PI {
            delta -= std::f64::consts::TAU;
        }
        while delta < -std::f64::consts::PI {
            delta += std::f64::consts::TAU;
        }
        self.heading_rad += delta;
        self.previous_yaw_rad = after.base_relative_yaw_rad;
        self.step += 1;
    }
}

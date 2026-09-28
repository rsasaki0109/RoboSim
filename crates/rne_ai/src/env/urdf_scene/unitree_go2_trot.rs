//! A model-based trot for the official Go2 with its feet attached.
//!
//! This is the controller structure Pinocchio-based quadruped stacks run,
//! closed on joint torques at [`UNITREE_GO2_TROT_CONTROL_HZ`]:
//!
//! * foot linear Jacobians from the simulated link frames, one column
//!   `axis x (foot - joint)` per revolute joint;
//! * stance legs take their share of the weight plus a spring-damper on their
//!   own hip height -- which levels pitch and roll leg by leg -- and push
//!   toward the commanded forward speed and yaw rate: `tau = -J^T f`;
//! * swing feet land where the Raibert heuristic puts them (under the hip,
//!   plus half a stance of body velocity, plus velocity-error and yaw terms)
//!   along a lifted arc tracked by Cartesian PD, `tau = J^T (Kp e - Kd v)`.
//!
//! It is written for `unitree_go2_jump` ([`unitree_go2_jump_scene_path`]): the
//! same URDF as the walking asset with its fixed children welded and its
//! declared masses. The walking asset leaves the feet on the floor, which this
//! controller's kinematics would read as feet that never move.
//!
//! [`unitree_go2_jump_scene_path`]: super::unitree_go2_jump_scene_path

use rne_core::SimDuration;
use rne_math::{Hertz, Vec3};

use super::{unitree_go2_trot_targets, UnitreeGo2GaitCommand, UrdfJointTorqueTarget, UrdfSceneSim};

/// Rate the trot closes its torque loop at, in hertz.
pub const UNITREE_GO2_TROT_CONTROL_HZ: f64 = 500.0;

const GRAVITY_M_S2: f64 = 9.81;
/// Effort limits from the URDF: hip, thigh, calf.
const EFFORT_NM: [f64; 3] = [23.7, 23.7, 45.43];
/// URDF speed limit shared by the joints, in rad/s.
const SPEED_LIMIT_RAD_S: f64 = 30.1;
const LEGS: [[&str; 4]; 4] = [
    ["FL_hip", "FL_thigh", "FL_calf", "FL_foot"],
    ["FR_hip", "FR_thigh", "FR_calf", "FR_foot"],
    ["RL_hip", "RL_thigh", "RL_calf", "RL_foot"],
    ["RR_hip", "RR_thigh", "RR_calf", "RR_foot"],
];
/// Trot phase offsets: diagonal pairs FL/RR and FR/RL half a cycle apart.
const PHASE_OFFSETS: [f64; 4] = [0.0, 0.5, 0.5, 0.0];

/// What the trot is asked to do this step.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct UnitreeGo2TrotCommand {
    /// Speed along the body's facing, in m/s.
    pub forward_m_s: f64,
    /// Yaw rate about the world vertical, in rad/s.
    pub yaw_rate_rad_s: f64,
}

/// Gait timing and gains of [`UnitreeGo2Trot`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct UnitreeGo2TrotGains {
    /// Model mass the stance legs share, in kilograms.
    pub mass_kg: f64,
    /// Full trot cycle, in seconds.
    pub period_s: f64,
    /// Stance fraction of the cycle.
    pub duty: f64,
    /// Hip height above its planted foot the stance springs hold, in meters.
    pub stance_height_m: f64,
    /// Swing foot apex above the floor, in meters.
    pub step_height_m: f64,
    /// Height of a planted foot's centre above the floor, in meters.
    pub foot_ground_y_m: f64,
    /// Stance hip-height spring, in N/m.
    pub height_stiffness_n_per_m: f64,
    /// Stance hip-height damper, in N·s/m.
    pub height_damping_n_s_per_m: f64,
    /// Stance force per m/s of velocity error, shared by the stance legs.
    pub velocity_gain_n_s_per_m: f64,
    /// Tangential stance force per rad/s of yaw-rate error.
    pub yaw_rate_gain_n_s: f64,
    /// Swing Cartesian stiffness, in N/m.
    pub swing_stiffness_n_per_m: f64,
    /// Swing Cartesian damping, in N·s/m.
    pub swing_damping_n_s_per_m: f64,
    /// Raibert velocity-error correction, in seconds.
    pub raibert_gain_s: f64,
}

impl Default for UnitreeGo2TrotGains {
    fn default() -> Self {
        Self {
            mass_kg: 16.1,
            period_s: 0.4,
            duty: 0.5,
            stance_height_m: 0.30,
            step_height_m: 0.06,
            foot_ground_y_m: 0.022,
            height_stiffness_n_per_m: 600.0,
            height_damping_n_s_per_m: 40.0,
            velocity_gain_n_s_per_m: 300.0,
            yaw_rate_gain_n_s: 30.0,
            swing_stiffness_n_per_m: 400.0,
            swing_damping_n_s_per_m: 12.0,
            raibert_gain_s: 0.1,
        }
    }
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
    fn read(sim: &UrdfSceneSim, names: [&'static str; 4]) -> Option<Self> {
        // URDF joint axes: hip abduction about x, thigh and calf about y.
        let local_axes = [Vec3::X, Vec3::Y, Vec3::Y];
        let mut origins = [Vec3::ZERO; 3];
        let mut axes = [Vec3::ZERO; 3];
        let mut joint_rates = [0.0; 3];
        for i in 0..3 {
            let pose = sim.named_transform(names[i])?;
            origins[i] = pose.translation;
            axes[i] = (pose.rotation * local_axes[i]).normalize();
            joint_rates[i] = sim.named_joint_velocity(names[i])?;
        }
        let foot = sim.named_transform(names[3])?.translation;
        Some(Self {
            joints: [names[0], names[1], names[2]],
            origins,
            axes,
            foot,
            joint_rates,
        })
    }

    fn jacobian(&self) -> [Vec3; 3] {
        std::array::from_fn(|i| self.axes[i].cross(self.foot - self.origins[i]))
    }

    fn foot_velocity_rel(&self) -> Vec3 {
        let jacobian = self.jacobian();
        (0..3).fold(Vec3::ZERO, |sum, i| sum + jacobian[i] * self.joint_rates[i])
    }

    fn torques_for(&self, force: Vec3) -> [f64; 3] {
        let jacobian = self.jacobian();
        std::array::from_fn(|i| jacobian[i].dot(force).clamp(-EFFORT_NM[i], EFFORT_NM[i]))
    }
}

/// Model-based trot controller state.
#[derive(Clone, Debug)]
pub struct UnitreeGo2Trot {
    gains: UnitreeGo2TrotGains,
    step: u64,
    liftoff: [Vec3; 4],
    in_stance: [bool; 4],
}

impl UnitreeGo2Trot {
    /// Creates a trot that starts at phase zero.
    pub fn new(gains: UnitreeGo2TrotGains) -> Self {
        Self {
            gains,
            step: 0,
            liftoff: [Vec3::ZERO; 4],
            in_stance: [true; 4],
        }
    }

    /// The gains this trot runs with.
    pub fn gains(&self) -> UnitreeGo2TrotGains {
        self.gains
    }

    /// Stands the robot up on its position servos at the scene's own step,
    /// then switches the scene to [`UNITREE_GO2_TROT_CONTROL_HZ`] for torque
    /// control.
    pub fn stand_up(sim: &mut UrdfSceneSim) {
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
            UNITREE_GO2_TROT_CONTROL_HZ,
        )));
    }

    /// Seconds since the trot started.
    pub fn time_s(&self) -> f64 {
        self.step as f64 / UNITREE_GO2_TROT_CONTROL_HZ
    }

    /// Computes this step's joint torques for `command` and steps `sim` once.
    ///
    /// Returns `false`, without stepping, if a leg link is missing -- the
    /// scene is not a Go2.
    pub fn step(&mut self, sim: &mut UrdfSceneSim, command: UnitreeGo2TrotCommand) -> bool {
        let gains = self.gains;
        let t_s = self.time_s();
        let observed = sim.observe();
        let Some(base) = sim.named_transform("base") else {
            return false;
        };
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
        let velocity_cmd = flat_facing * command.forward_m_s;

        let phases: [f64; 4] =
            std::array::from_fn(|i| (t_s / gains.period_s + PHASE_OFFSETS[i]) % 1.0);
        let stance_count = phases.iter().filter(|p| **p < gains.duty).count().max(1) as f64;
        let stance_s = gains.period_s * gains.duty;
        let mut torques: Vec<UrdfJointTorqueTarget<'_>> = Vec::with_capacity(12);
        for (index, names) in LEGS.iter().enumerate() {
            let Some(leg) = Leg::read(sim, *names) else {
                return false;
            };
            let hip = leg.origins[1];
            let force = if phases[index] < gains.duty {
                self.in_stance[index] = true;
                // Ground reaction wanted at this foot.
                let hip_height = hip.y - leg.foot.y;
                let hip_rise_rate = -leg.foot_velocity_rel().y;
                let lift = gains.mass_kg * GRAVITY_M_S2 / stance_count
                    + gains.height_stiffness_n_per_m * (gains.stance_height_m - hip_height)
                    - gains.height_damping_n_s_per_m * hip_rise_rate;
                let velocity_error = velocity_cmd - flat_velocity;
                let lever = Vec3::new(leg.foot.x, 0.0, leg.foot.z) - body_center;
                let tangent = Vec3::Y.cross(lever).normalize_or_zero();
                let push = velocity_error * (gains.velocity_gain_n_s_per_m / stance_count)
                    + tangent * (gains.yaw_rate_gain_n_s * (command.yaw_rate_rad_s - yaw_rate));
                // The foot pushes the ground the other way: tau = -J^T f.
                -Vec3::new(push.x, lift.max(0.0), push.z)
            } else {
                if self.in_stance[index] {
                    self.liftoff[index] = leg.foot;
                    self.in_stance[index] = false;
                }
                let swing = ((phases[index] - gains.duty) / (1.0 - gains.duty)).clamp(0.0, 1.0);
                let hip_ground = Vec3::new(hip.x, 0.0, hip.z);
                let from_center = hip_ground - body_center;
                let touchdown = hip_ground
                    + flat_velocity * (stance_s * 0.5)
                    + (flat_velocity - velocity_cmd) * gains.raibert_gain_s
                    + Vec3::Y.cross(from_center) * (command.yaw_rate_rad_s * stance_s * 0.5);
                let blend = 0.5 - 0.5 * (std::f64::consts::PI * swing).cos();
                let liftoff = self.liftoff[index];
                let target = Vec3::new(
                    liftoff.x + (touchdown.x - liftoff.x) * blend,
                    gains.foot_ground_y_m
                        + gains.step_height_m * (std::f64::consts::PI * swing).sin(),
                    liftoff.z + (touchdown.z - liftoff.z) * blend,
                );
                let foot_velocity = leg.foot_velocity_rel() + body_velocity;
                (target - leg.foot) * gains.swing_stiffness_n_per_m
                    - foot_velocity * gains.swing_damping_n_s_per_m
            };
            let tau = leg.torques_for(force);
            for (joint, torque) in leg.joints.iter().zip(tau) {
                torques.push(UrdfJointTorqueTarget {
                    link_name: joint,
                    torque_nm: torque,
                    max_velocity_rad_s: SPEED_LIMIT_RAD_S,
                });
            }
        }
        sim.step_joint_torques(&torques);
        self.step += 1;
        true
    }
}

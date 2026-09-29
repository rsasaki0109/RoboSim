//! Go2 push recovery: passive stance vs. active balance vs. balance with
//! stepping, under the same measured shove, on the footed Go2.

use rne_ai::{
    unitree_go2_trot_targets, UnitreeGo2GaitCommand, UrdfJointPositionTarget,
    UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_core::SimDuration;
use rne_math::{Hertz, Vec3};
use std::path::PathBuf;

const CONTROL_HZ: f64 = 500.0;
const GRAVITY_M_S2: f64 = 9.81;
const MASS_KG: f64 = 16.1;
const EFFORT_NM: [f64; 3] = [23.7, 23.7, 45.43];
const LEGS: [[&str; 4]; 4] = [
    ["FL_hip", "FL_thigh", "FL_calf", "FL_foot"],
    ["FR_hip", "FR_thigh", "FR_calf", "FR_foot"],
    ["RL_hip", "RL_thigh", "RL_calf", "RL_foot"],
    ["RR_hip", "RR_thigh", "RR_calf", "RR_foot"],
];
const PHASE_OFFSETS: [f64; 4] = [0.0, 0.5, 0.5, 0.0];
fn gait_period_s() -> f64 { std::env::var("GAIT_PERIOD_S").ok().and_then(|v| v.parse().ok()).unwrap_or(0.3) }
const DUTY: f64 = 0.5;
fn step_height_m() -> f64 { std::env::var("STEP_HEIGHT_M").ok().and_then(|v| v.parse().ok()).unwrap_or(0.06) }
const FOOT_GROUND_Y_M: f64 = 0.022;
fn swing_stiffness_n_per_m() -> f64 { std::env::var("SWING_STIFFNESS_N_PER_M").ok().and_then(|v| v.parse().ok()).unwrap_or(400.0) }
fn swing_damping_n_s_per_m() -> f64 { std::env::var("SWING_DAMPING_N_S_PER_M").ok().and_then(|v| v.parse().ok()).unwrap_or(12.0) }
fn capture_gain() -> f64 { std::env::var("CAPTURE_GAIN").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0) }
fn max_reach_m() -> f64 { std::env::var("MAX_REACH_M").ok().and_then(|v| v.parse().ok()).unwrap_or(10.0) }
fn capture_margin_m() -> f64 { std::env::var("CAPTURE_MARGIN_M").ok().and_then(|v| v.parse().ok()).unwrap_or(0.02) }
fn height_integral_gain() -> f64 { std::env::var("HEIGHT_KI").ok().and_then(|v| v.parse().ok()).unwrap_or(0.0) }
fn liftoff_load_n() -> f64 { std::env::var("LIFTOFF_LOAD_N").ok().and_then(|v| v.parse().ok()).unwrap_or(10.0) }
fn horizontal_weight() -> f64 { std::env::var("HORIZONTAL_WEIGHT").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0) }
fn moment_weight() -> f64 { std::env::var("MOMENT_WEIGHT").ok().and_then(|v| v.parse().ok()).unwrap_or(1.0) }
fn raibert_gain_s() -> f64 { std::env::var("RAIBERT_GAIN_S").ok().and_then(|v| v.parse().ok()).unwrap_or(0.1) }
fn friction_limit() -> f64 { std::env::var("FRICTION_LIMIT").ok().and_then(|v| v.parse().ok()).unwrap_or(0.6) }
fn home_stiffness_n_per_m() -> f64 { std::env::var("HOME_STIFFNESS_N_PER_M").ok().and_then(|v| v.parse().ok()).unwrap_or(400.0) }
fn home_damping_n_s_per_m() -> f64 { std::env::var("HOME_DAMPING_N_S_PER_M").ok().and_then(|v| v.parse().ok()).unwrap_or(150.0) }
const BODY_HEIGHT_STIFFNESS_N_PER_M: f64 = 1000.0;
const BODY_HEIGHT_DAMPING_N_S_PER_M: f64 = 120.0;
fn tilt_stiffness_nm_per_rad() -> f64 { std::env::var("TILT_STIFFNESS_NM_PER_RAD").ok().and_then(|v| v.parse().ok()).unwrap_or(120.0) }
fn tilt_damping_nm_s_per_rad() -> f64 { std::env::var("TILT_DAMPING_NM_S_PER_RAD").ok().and_then(|v| v.parse().ok()).unwrap_or(10.0) }
const YAW_STIFFNESS_NM_PER_RAD: f64 = 40.0;
const YAW_DAMPING_NM_S_PER_RAD: f64 = 4.0;
const FORCE_REGULARIZATION: f64 = 1.0e-4;

const PUSH_START_S: f64 = 1.0;
fn push_duration_s() -> f64 { std::env::var("PUSH_DURATION_S").ok().and_then(|v| v.parse().ok()).unwrap_or(0.1) }
const OBSERVE_S: f64 = 5.0;
fn step_trigger_speed_m_s() -> f64 { std::env::var("STEP_TRIGGER_SPEED_M_S").ok().and_then(|v| v.parse().ok()).unwrap_or(0.25) }
const STEP_RELEASE_SPEED_M_S: f64 = 0.08;
const STEP_RELEASE_HOLD_S: f64 = 0.6;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Mode {
    Passive,
    Balance,
    Step,
}

fn scene_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/unitree_go2_jump.rne.scene.toml")
}

struct Leg {
    joints: [&'static str; 3],
    origins: [Vec3; 3],
    axes: [Vec3; 3],
    foot: Vec3,
    joint_rates: [f64; 3],
}

impl Leg {
    fn read(sim: &UrdfSceneSim, names: [&'static str; 4]) -> Self {
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
        let foot = sim.named_transform(names[3]).expect("foot pose").translation;
        Self {
            joints: [names[0], names[1], names[2]],
            origins,
            axes,
            foot,
            joint_rates,
        }
    }

    fn jacobian(&self) -> [Vec3; 3] {
        std::array::from_fn(|i| self.axes[i].cross(self.foot - self.origins[i]))
    }

    fn foot_velocity_rel(&self) -> Vec3 {
        let jacobian = self.jacobian();
        (0..3).fold(Vec3::ZERO, |sum, i| sum + jacobian[i] * self.joint_rates[i])
    }

    fn torques_for(&self, f: Vec3) -> [f64; 3] {
        let jacobian = self.jacobian();
        std::array::from_fn(|i| jacobian[i].dot(f).clamp(-EFFORT_NM[i], EFFORT_NM[i]))
    }
}

struct Robot {
    sim: UrdfSceneSim,
    mode: Mode,
    stand: [UrdfJointPositionTarget<'static>; 12],
    step: u64,
    gait_start_s: Option<f64>,
    calm_s: f64,
    liftoff: [Vec3; 4],
    in_stance: [bool; 4],
    steps_taken: u32,
    /// Ground point the center of mass is held over.
    home: Vec3,
    height_m: f64,
    yaw_home_rad: f64,
    /// The trunk axis that points up in the settled stance. The URDF trunk
    /// frame is z-up inside the y-up world, so this is read, not assumed.
    up_local: Vec3,
    height_integral: f64,
    /// Whether each swing leg has left the ground in its current swing.
    lifted: [bool; 4],
}

impl Robot {
    fn new(mode: Mode) -> Self {
        let mut sim = UrdfSceneSim::from_scene_path(&scene_path()).expect("load welded Go2");
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
        let (com, _) = sim.dynamic_center_of_mass_m().expect("Go2 mass");
        let yaw_home_rad = sim.observe().base_relative_yaw_rad;
        let up_local = sim.named_transform("base").expect("base pose").rotation.inverse() * Vec3::Y;
        Self {
            sim,
            mode,
            stand,
            step: 0,
            gait_start_s: None,
            calm_s: 0.0,
            liftoff: [Vec3::ZERO; 4],
            in_stance: [true; 4],
            steps_taken: 0,
            home: Vec3::new(com.x, 0.0, com.z),
            height_m: com.y,
            yaw_home_rad,
            up_local,
            height_integral: 0.0,
            lifted: [false; 4],
        }
    }

    fn t_s(&self) -> f64 {
        self.step as f64 / CONTROL_HZ
    }

    fn step_once(&mut self, push: Vec3) {
        let t_s = self.t_s();
        if push != Vec3::ZERO {
            let trunk = self.sim.named_link_position_m("base").expect("trunk");
            assert!(self.sim.apply_named_link_wrench("base", trunk, push, Vec3::ZERO));
        }
        if self.mode == Mode::Passive {
            self.sim.step_joint_position_targets(&self.stand);
            self.step += 1;
            return;
        }
        let observed = self.sim.observe();
        let (com, _) = self.sim.dynamic_center_of_mass_m().expect("Go2 mass");
        let body_velocity = Vec3::new(
            observed.base_linear_velocity_x_m_s,
            observed.base_linear_velocity_y_m_s,
            observed.base_linear_velocity_z_m_s,
        );
        let angular_velocity = Vec3::new(
            observed.base_angular_velocity_x_rad_s,
            observed.base_angular_velocity_y_rad_s,
            observed.base_angular_velocity_z_rad_s,
        );
        let flat_velocity = Vec3::new(body_velocity.x, 0.0, body_velocity.z);
        let speed = flat_velocity.length();

        if self.mode == Mode::Step {
            match self.gait_start_s {
                None if self.needs_step(com, flat_velocity) => self.gait_start_s = Some(t_s),
                Some(start) => {
                    self.calm_s = if speed < STEP_RELEASE_SPEED_M_S {
                        self.calm_s + 1.0 / CONTROL_HZ
                    } else {
                        0.0
                    };
                    // Release only at a pair change, when every foot is down.
                    let cycles = (t_s - start) / (gait_period_s() * DUTY);
                    let at_boundary = cycles.fract() < 1.0 / (CONTROL_HZ * gait_period_s() * DUTY);
                    if self.calm_s >= STEP_RELEASE_HOLD_S && at_boundary && cycles >= 1.0 {
                        self.gait_start_s = None;
                        self.calm_s = 0.0;
                    }
                }
                None => {}
            }
            // While stepping the robot stops where it can; it does not
            // walk back to where it was shoved from.
            if self.gait_start_s.is_some() {
                self.home = Vec3::new(com.x, 0.0, com.z);
            }
        }
        let phases: [Option<f64>; 4] = std::array::from_fn(|i| {
            self.gait_start_s
                .map(|start| ((t_s - start) / gait_period_s() + PHASE_OFFSETS[i]) % 1.0)
        });
        let legs: Vec<Leg> = LEGS.iter().map(|names| Leg::read(&self.sim, *names)).collect();
        // Measured load under each foot. A leg scheduled to swing that still
        // carries the trunk stays in stance until it is unloaded (late
        // liftoff), and a swinging foot that meets the ground in the second
        // half of its swing is planted (early touchdown).
        let loads = self.foot_loads_n(&legs);
        let contact_aware = std::env::var("NO_CONTACT_AWARE").is_err();
        let stance: Vec<usize> = (0..4)
            .filter(|&i| match phases[i] {
                None => true,
                Some(p) if p < DUTY => true,
                Some(p) => {
                    let swing = (p - DUTY) / (1.0 - DUTY);
                    contact_aware
                        && ((swing < 0.5 && !self.lifted[i] && loads[i] > liftoff_load_n())
                            || (swing >= 0.5 && loads[i] > liftoff_load_n()))
                }
            })
            .collect();
        for i in 0..4 {
            match phases[i] {
                Some(p) if p >= DUTY && !stance.contains(&i) => self.lifted[i] = true,
                _ if stance.contains(&i) && phases[i].is_none_or(|p| p < DUTY) => {
                    self.lifted[i] = false;
                }
                _ => {}
            }
        }

        // Whole-body wrench about the center of mass: weight, a spring to the
        // stance height and home, and a spring-damper that levels the trunk
        // and holds its heading.
        let base = self.sim.named_transform("base").expect("base pose");
        let up = base.rotation * self.up_local;
        // Integral action on height takes up what the model leaves out: the
        // legs' own weight and the swing legs' pull on the trunk.
        self.height_integral = (self.height_integral
            + height_integral_gain() * (self.height_m - com.y) / CONTROL_HZ)
            .clamp(-60.0, 60.0);
        let flat_com = Vec3::new(com.x, 0.0, com.z);
        // The center of mass is held over the feet that carry it, so a push
        // is passed to the ground instead of leaning the trunk off its feet.
        let support = if std::env::var("WORLD_HOME").is_ok() {
            self.home
        } else {
            stance.iter().fold(Vec3::ZERO, |sum, &i| sum + legs[i].foot) / stance.len().max(1) as f64
        };
        let support = Vec3::new(support.x, 0.0, support.z);
        let force = Vec3::new(0.0, MASS_KG * GRAVITY_M_S2, 0.0)
            + (support - flat_com) * home_stiffness_n_per_m()
            - flat_velocity * home_damping_n_s_per_m()
            + Vec3::Y
                * (BODY_HEIGHT_STIFFNESS_N_PER_M * (self.height_m - com.y)
                    - BODY_HEIGHT_DAMPING_N_S_PER_M * body_velocity.y
                    + self.height_integral);
        let yaw_error = wrap(self.yaw_home_rad - observed.base_relative_yaw_rad);
        let flat_angular = Vec3::new(angular_velocity.x, 0.0, angular_velocity.z);
        let moment = up.cross(Vec3::Y) * tilt_stiffness_nm_per_rad()
            - flat_angular * tilt_damping_nm_s_per_rad()
            + Vec3::Y
                * (YAW_STIFFNESS_NM_PER_RAD * yaw_error
                    - YAW_DAMPING_NM_S_PER_RAD * angular_velocity.y);
        let levers: Vec<Vec3> = stance.iter().map(|&i| legs[i].foot - com).collect();
        let reactions = distribute_wrench(&levers, force, moment);
        if std::env::var("TRACE").is_ok() && self.step % 10 == 0 && self.step < 200 {
            let sum = reactions.iter().fold(Vec3::ZERO, |a, f| a + *f);
            eprintln!(
                "t={:.3} com=({:.3},{:.3},{:.3}) up=({:.3},{:.3},{:.3}) w=({:.2},{:.2},{:.2}) F=({:.1},{:.1},{:.1}) M=({:.2},{:.2},{:.2}) sumf=({:.1},{:.1},{:.1}) f0=({:.1},{:.1},{:.1}) roll={:.3} pitch={:.3}",
                t_s, com.x, com.y, com.z, up.x, up.y, up.z, angular_velocity.x, angular_velocity.y, angular_velocity.z,
                force.x, force.y, force.z, moment.x, moment.y, moment.z, sum.x, sum.y, sum.z,
                reactions[0].x, reactions[0].y, reactions[0].z,
                observed.base_relative_roll_rad, observed.base_relative_pitch_rad
            );
        }

        let stance_s = gait_period_s() * DUTY;
        let mut torques: Vec<UrdfJointTorqueTarget<'_>> = Vec::with_capacity(12);
        for (index, leg) in legs.iter().enumerate() {
            let hip = leg.origins[1];
            let foot_force = if let Some(slot) = stance.iter().position(|&i| i == index) {
                if !self.in_stance[index] {
                    self.steps_taken += 1;
                }
                self.in_stance[index] = true;
                // Ground reaction on the robot, kept inside the friction cone.
                let mut reaction = reactions[slot];
                reaction.y = reaction.y.max(0.0);
                let horizontal = Vec3::new(reaction.x, 0.0, reaction.z);
                let limit = friction_limit() * reaction.y;
                if horizontal.length() > limit {
                    let scaled = horizontal.normalize_or_zero() * limit;
                    reaction = Vec3::new(scaled.x, reaction.y, scaled.z);
                }
                // The foot pushes the ground the other way: tau = -J^T f.
                -reaction
            } else {
                let phase = phases[index].expect("swing leg has a phase");
                if self.in_stance[index] {
                    self.liftoff[index] = leg.foot;
                    self.in_stance[index] = false;
                }
                let swing = ((phase - DUTY) / (1.0 - DUTY)).clamp(0.0, 1.0);
                let hip_ground = Vec3::new(hip.x, 0.0, hip.z);
                let offset = flat_velocity * (stance_s * 0.5)
                    + flat_velocity * raibert_gain_s()
                    + flat_velocity * (capture_gain() * (self.height_m / GRAVITY_M_S2).sqrt());
                // Keep the touchdown inside the leg's reach.
                let reach = max_reach_m();
                let offset = if offset.length() > reach {
                    offset.normalize_or_zero() * reach
                } else {
                    offset
                };
                let touchdown = hip_ground + offset;
                let blend = 0.5 - 0.5 * (std::f64::consts::PI * swing).cos();
                let liftoff = self.liftoff[index];
                let target = Vec3::new(
                    liftoff.x + (touchdown.x - liftoff.x) * blend,
                    FOOT_GROUND_Y_M + step_height_m() * (std::f64::consts::PI * swing).sin(),
                    liftoff.z + (touchdown.z - liftoff.z) * blend,
                );
                let foot_velocity = leg.foot_velocity_rel() + body_velocity;
                (target - leg.foot) * swing_stiffness_n_per_m()
                    - foot_velocity * swing_damping_n_s_per_m()
            };
            for (joint, torque) in leg.joints.iter().zip(leg.torques_for(foot_force)) {
                torques.push(UrdfJointTorqueTarget {
                    link_name: joint,
                    torque_nm: torque,
                    max_velocity_rad_s: 30.1,
                });
            }
        }
        self.sim.step_joint_torques(&torques);
        self.step += 1;
    }

    /// Normal load under each foot, from the solved ground contacts nearest
    /// to it.
    fn foot_loads_n(&self, legs: &[Leg]) -> [f64; 4] {
        let mut loads = [0.0; 4];
        for (point, normal_n) in self.sim.named_body_contact_loads("ground") {
            let (nearest, distance) = legs
                .iter()
                .enumerate()
                .map(|(i, leg)| (i, (point - leg.foot).length()))
                .min_by(|a, b| a.1.total_cmp(&b.1))
                .expect("four legs");
            if distance < 0.05 {
                loads[nearest] += normal_n;
            }
        }
        loads
    }

    /// Whether the instantaneous capture point `com + v / omega0` has left
    /// the polygon of the four feet, shrunk by a margin: past that point no
    /// stance force can stop the body, and only a step can.
    fn needs_step(&self, com: Vec3, flat_velocity: Vec3) -> bool {
        if std::env::var("SPEED_TRIGGER").is_ok() {
            return flat_velocity.length() > step_trigger_speed_m_s();
        }
        let omega0 = (GRAVITY_M_S2 / self.height_m).sqrt();
        let capture = Vec3::new(com.x, 0.0, com.z) + flat_velocity / omega0;
        // FL, FR, RR, RL is a loop around the support polygon.
        let corners: Vec<Vec3> = [0, 1, 3, 2]
            .iter()
            .map(|&i| {
                let foot = self.sim.named_transform(LEGS[i][3]).expect("foot").translation;
                Vec3::new(foot.x, 0.0, foot.z)
            })
            .collect();
        let center = corners.iter().fold(Vec3::ZERO, |sum, c| sum + *c) / 4.0;
        (0..4).any(|k| {
            let (a, b) = (corners[k], corners[(k + 1) % 4]);
            let edge = (b - a).normalize_or_zero();
            let mut outward = Vec3::new(edge.z, 0.0, -edge.x);
            if outward.dot(a - center) < 0.0 {
                outward = -outward;
            }
            (capture - a).dot(outward) > -capture_margin_m()
        })
    }

    /// Ground contacts that are not at a foot: a knee, thigh, or the trunk.
    fn body_on_ground(&self) -> bool {
        let feet: Vec<Vec3> = LEGS
            .iter()
            .map(|names| self.sim.named_transform(names[3]).expect("foot").translation)
            .collect();
        self.sim
            .named_body_contact_points_m("ground")
            .into_iter()
            .any(|point| feet.iter().all(|foot| (point - *foot).length() > 0.05))
    }
}

fn wrap(angle: f64) -> f64 {
    (angle + std::f64::consts::PI).rem_euclid(std::f64::consts::TAU) - std::f64::consts::PI
}

/// Ground reactions at the stance feet that best produce `force` and
/// `moment` about the center of mass, as a weighted least-squares problem
/// with unilateral contact.
///
/// The residual weights put the vertical force and the moment ahead of the
/// horizontal force: when the stance cannot do both, the trunk keeps level
/// and the feet give up horizontal grip, which is what keeps a shove from
/// tipping the robot. A foot asked to pull on the ground is dropped from the
/// support and the problem is solved again.
fn distribute_wrench(levers: &[Vec3], force: Vec3, moment: Vec3) -> Vec<Vec3> {
    let weights = [
        horizontal_weight(),
        1.0,
        horizontal_weight(),
        moment_weight(),
        moment_weight(),
        moment_weight(),
    ];
    let target = [force.x, force.y, force.z, moment.x, moment.y, moment.z];
    let mut active: Vec<bool> = vec![true; levers.len()];
    loop {
        let feet: Vec<usize> = (0..levers.len()).filter(|&i| active[i]).collect();
        let mut reactions = vec![Vec3::ZERO; levers.len()];
        if feet.is_empty() {
            return reactions;
        }
        // Columns of A for foot i and axis k: [e_k ; r_i x e_k].
        let columns: Vec<[f64; 6]> = feet
            .iter()
            .flat_map(|&i| {
                [Vec3::X, Vec3::Y, Vec3::Z].map(|e| {
                    let m = levers[i].cross(e);
                    [e.x, e.y, e.z, m.x, m.y, m.z]
                })
            })
            .collect();
        let n = columns.len();
        let mut normal = vec![vec![0.0; n]; n];
        let mut rhs = vec![0.0; n];
        for p in 0..n {
            for q in 0..n {
                normal[p][q] = (0..6)
                    .map(|row| weights[row] * columns[p][row] * columns[q][row])
                    .sum();
            }
            normal[p][p] += FORCE_REGULARIZATION;
            rhs[p] = (0..6).map(|row| weights[row] * columns[p][row] * target[row]).sum();
        }
        let solution = solve(normal, rhs);
        let mut dropped = false;
        for (slot, &i) in feet.iter().enumerate() {
            let reaction = Vec3::new(solution[3 * slot], solution[3 * slot + 1], solution[3 * slot + 2]);
            if reaction.y < 0.0 {
                active[i] = false;
                dropped = true;
            }
            reactions[i] = reaction;
        }
        if !dropped || std::env::var("NO_ACTIVE_SET").is_ok() {
            return reactions;
        }
    }
}

/// Solves a square system by Gaussian elimination with partial pivoting.
fn solve(mut a: Vec<Vec<f64>>, mut b: Vec<f64>) -> Vec<f64> {
    let n = b.len();
    for col in 0..n {
        let pivot = (col..n)
            .max_by(|&i, &j| a[i][col].abs().total_cmp(&a[j][col].abs()))
            .expect("pivot row");
        a.swap(col, pivot);
        b.swap(col, pivot);
        for row in col + 1..n {
            let factor = a[row][col] / a[col][col];
            for k in col..n {
                a[row][k] -= factor * a[col][k];
            }
            b[row] -= factor * b[col];
        }
    }
    let mut x = vec![0.0; n];
    for row in (0..n).rev() {
        let tail: f64 = (row + 1..n).map(|k| a[row][k] * x[k]).sum();
        x[row] = (b[row] - tail) / a[row][row];
    }
    x
}

#[derive(Debug)]
struct Outcome {
    fell: bool,
    peak_t_s: f64,
    gait_on_end: bool,
    peak_tilt_rad: f64,
    min_height_m: f64,
    displacement_m: f64,
    final_speed_m_s: f64,
    steps: u32,
}

fn run(mode: Mode, force: Vec3) -> Outcome {
    let mut robot = Robot::new(mode);
    let start = robot.sim.observe();
    let mut fell = false;
    let mut peak_tilt_rad: f64 = 0.0;
    let mut min_height_m = f64::MAX;
    let mut peak_t_s = 0.0;
    while robot.t_s() < PUSH_START_S + OBSERVE_S {
        let t_s = robot.t_s();
        let pushing = (PUSH_START_S..PUSH_START_S + push_duration_s()).contains(&t_s);
        robot.step_once(if pushing { force } else { Vec3::ZERO });
        let o = robot.sim.observe();
        let tilt = o.base_relative_roll_rad.abs().max(o.base_relative_pitch_rad.abs());
        if tilt > peak_tilt_rad { peak_t_s = t_s; }
        peak_tilt_rad = peak_tilt_rad.max(tilt);
        min_height_m = min_height_m.min(o.base_y_m);
        if std::env::var("VTRACE").is_ok() && robot.step % 5 == 0 && t_s < PUSH_START_S + 0.3 && t_s > PUSH_START_S - 0.05 {
            let fz: Vec<String> = LEGS.iter().map(|n| { let p = robot.sim.named_transform(n[3]).unwrap().translation; format!("{:.3}/{:.3}", p.z, p.y) }).collect();
            eprintln!("t={:.3} vz={:.3} z={:.4} y={:.3} roll={:.3} pitch={:.3} feet_on={} feet z/y={}", t_s, o.base_linear_velocity_z_m_s, o.base_z_m, o.base_y_m, o.base_relative_roll_rad, o.base_relative_pitch_rad, robot.sim.named_body_contact_points_m("ground").len(), fz.join(" "));
        }
        if robot.body_on_ground() || o.base_y_m < 0.15 || tilt > 0.8 {
            if std::env::var("WHY").is_ok() {
                let feet: Vec<Vec3> = LEGS.iter().map(|n| robot.sim.named_transform(n[3]).unwrap().translation).collect();
                let stray: Vec<String> = robot.sim.named_body_contact_points_m("ground").into_iter()
                    .filter(|p| feet.iter().all(|f| (*p - *f).length() > 0.05))
                    .map(|p| format!("({:.3},{:.3},{:.3}) nearest_foot={:.3}", p.x, p.y, p.z, feet.iter().map(|f| (p - *f).length()).fold(f64::MAX, f64::min))).collect();
                eprintln!("fell at t={t_s:.3} h={:.3} tilt={tilt:.3} stray={stray:?}", o.base_y_m);
            }
            fell = true;
            break;
        }
    }
    let end = robot.sim.observe();
    Outcome {
        fell,
        peak_t_s,
        gait_on_end: robot.gait_start_s.is_some(),
        peak_tilt_rad,
        min_height_m,
        displacement_m: Vec3::new(end.base_x_m - start.base_x_m, 0.0, end.base_z_m - start.base_z_m)
            .length(),
        final_speed_m_s: Vec3::new(end.base_linear_velocity_x_m_s, 0.0, end.base_linear_velocity_z_m_s)
            .length(),
        steps: robot.steps_taken,
    }
}

fn mode_from_env() -> Mode {
    match std::env::var("MODE").as_deref() {
        Ok("passive") => Mode::Passive,
        Ok("balance") => Mode::Balance,
        _ => Mode::Step,
    }
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let axis = if args.iter().any(|a| a == "--forward") { Vec3::X } else { Vec3::Z };
    if let Ok(list) = std::env::var("SWEEP") {
        let mut best = 0.0;
        for f in list.split(',').map(|v| v.parse::<f64>().unwrap()) {
            let o = run(mode_from_env(), axis * f);
            if o.fell { break; }
            best = f;
        }
        println!("{:?} recovers up to {best} N", mode_from_env());
        return;
    }
    if let Ok(force) = std::env::var("ONE") {
        let mode = match std::env::var("MODE").as_deref() {
            Ok("passive") => Mode::Passive,
            Ok("step") => Mode::Step,
            _ => Mode::Balance,
        };
        let o = run(mode, axis * force.parse::<f64>().unwrap());
        println!("{o:?}");
        return;
    }
    for force_n in [0.0, 50.0, 100.0, 150.0, 200.0, 250.0, 300.0, 400.0] {
        for mode in [Mode::Passive, Mode::Balance, Mode::Step] {
            let o = run(mode, axis * force_n);
            println!(
                "{force_n:5.0} N {mode:?}: fell={} peak_tilt={:.3} min_h={:.3} disp={:.3} v_end={:.3} steps={}",
                o.fell, o.peak_tilt_rad, o.min_height_m, o.displacement_m, o.final_speed_m_s, o.steps
            );
        }
    }
}

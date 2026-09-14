//! Closed-loop whole-body jump for the floating-base Unitree Go2.
//!
//! State machine: crouch -> whole-body push-off -> ballistic flight -> landing.
//! During the push-off `rne_wbc` solves for the joint accelerations, contact
//! wrenches, and joint torques that realize a target center-of-mass
//! acceleration while keeping the four feet fixed, using a floating-base model
//! built from the simulator's own world. Flight uses position control.
//!
//! Run with `cargo run -p go2_hop --example 107_go2_hop`.

use glam::EulerRot;
use rne_ai::{
    unitree_go2_dynamic_scene_path, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_dynamics::{center_of_mass, ArticulatedModel};
use rne_math::Vec3;
use rne_robot::{FloatingBase, KinematicModel, Robot, Transform3};
use rne_wbc::{ComTask, ContactPoint, PostureTask, WholeBodyConfig, WholeBodyController};

const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.02);
const LEG_PREFIXES: [&str; 4] = ["FL", "FR", "RL", "RR"];

const SETTLE_STEPS: u64 = 240;
const CROUCH_STEPS: u64 = 45;
const PUSH_STEPS_MAX: u64 = 60;
const FLIGHT_STEPS_MAX: u64 = 240;
const LAND_STEPS: u64 = 150;

const POSITION_STIFFNESS: f64 = 420.0;
const POSITION_DAMPING: f64 = 28.0;
const TORQUE_LIMIT_NM: f64 = 23.7;

const STAND_THIGH_RAD: f64 = 0.8;
const STAND_CALF_RAD: f64 = -1.5;
const CROUCH_THIGH_RAD: f64 = 1.15;
const CROUCH_CALF_RAD: f64 = -2.1;
const TUCK_THIGH_RAD: f64 = 0.95;
const TUCK_CALF_RAD: f64 = -1.7;

const PUSH_ACCEL_M_S2: f64 = 20.0;
const AIRBORNE_FOOT_HEIGHT_M: f64 = 0.04;

/// The floating-base model and the joint link names in degree-of-freedom order.
struct HopModel {
    model: ArticulatedModel,
    joint_links: Vec<String>,
    foot_names: Vec<String>,
    torque_limits: Vec<f64>,
}

impl HopModel {
    fn build(sim: &mut UrdfSceneSim) -> Self {
        let world = sim.world_mut();
        let robot = world
            .iter_entities()
            .find_map(|entity| entity.get::<Robot>().map(|_| entity.id()))
            .expect("robot entity");
        let base = world.get::<Robot>(robot).expect("robot").base_link;
        let saved = world.get::<Transform3>(base).copied().unwrap_or_default();
        world
            .entity_mut(base)
            .insert((FloatingBase, Transform3::IDENTITY));
        let model = ArticulatedModel::from_robot(&*world, robot).expect("floating model");
        world.entity_mut(base).insert(saved);

        let kinematics = KinematicModel::from_robot(world, robot).expect("kinematic model");
        let joint_links: Vec<String> = kinematics
            .movable_joint_entities()
            .iter()
            .map(|joint| {
                let child = kinematics.joint_child_link(*joint).expect("child");
                let index = kinematics.link_index(child).expect("index");
                kinematics.link_name(index).expect("name").to_string()
            })
            .collect();
        let foot_names = LEG_PREFIXES
            .iter()
            .map(|prefix| format!("{prefix}_foot"))
            .collect();
        let torque_limits = vec![TORQUE_LIMIT_NM; joint_links.len()];
        Self {
            model,
            joint_links,
            foot_names,
            torque_limits,
        }
    }

    /// Builds the generalized configuration from the simulator state.
    fn q(&self, sim: &UrdfSceneSim) -> Vec<f64> {
        let base = sim.named_transform("base").expect("base pose");
        let (yaw, pitch, roll) = base.rotation.to_euler(EulerRot::ZYX);
        let mut q = vec![0.0; self.model.nv()];
        q[0] = base.translation.x;
        q[1] = base.translation.y;
        q[2] = base.translation.z;
        q[3] = roll;
        q[4] = pitch;
        q[5] = yaw;
        for (dof, link) in self.joint_links.iter().enumerate() {
            q[6 + dof] = sim.named_joint_position(link).unwrap_or(0.0);
        }
        q
    }

    fn qd(&self, sim: &UrdfSceneSim) -> Vec<f64> {
        let mut qd = vec![0.0; self.model.nv()];
        for (dof, link) in self.joint_links.iter().enumerate() {
            qd[6 + dof] = sim.named_joint_velocity(link).unwrap_or(0.0);
        }
        qd
    }
}

fn targets(thigh_rad: f64, calf_rad: f64) -> [UrdfJointPositionTarget<'static>; 12] {
    let mut out = [UrdfJointPositionTarget {
        link_name: "",
        position: 0.0,
    }; 12];
    for (leg, prefix) in LEG_PREFIXES.iter().enumerate() {
        let base = leg * 3;
        out[base] = UrdfJointPositionTarget {
            link_name: hip_name(prefix),
            position: 0.0,
        };
        out[base + 1] = UrdfJointPositionTarget {
            link_name: thigh_name(prefix),
            position: thigh_rad,
        };
        out[base + 2] = UrdfJointPositionTarget {
            link_name: calf_name(prefix),
            position: calf_rad,
        };
    }
    out
}

fn hip_name(prefix: &str) -> &'static str {
    match prefix {
        "FL" => "FL_hip",
        "FR" => "FR_hip",
        "RL" => "RL_hip",
        _ => "RR_hip",
    }
}

fn thigh_name(prefix: &str) -> &'static str {
    match prefix {
        "FL" => "FL_thigh",
        "FR" => "FR_thigh",
        "RL" => "RL_thigh",
        _ => "RR_thigh",
    }
}

fn calf_name(prefix: &str) -> &'static str {
    match prefix {
        "FL" => "FL_calf",
        "FR" => "FR_calf",
        "RL" => "RL_calf",
        _ => "RR_calf",
    }
}

fn tilt_rad(sim: &UrdfSceneSim, up_reference: Vec3) -> f64 {
    let pose = sim.named_transform("base").expect("base pose");
    let up = (pose.rotation * up_reference).normalize_or_zero();
    up.y.clamp(-1.0, 1.0).acos()
}

fn min_foot_height_m(sim: &UrdfSceneSim) -> f64 {
    LEG_PREFIXES
        .iter()
        .filter_map(|prefix| sim.named_transform(&format!("{prefix}_foot")))
        .map(|transform| transform.translation.y)
        .fold(f64::INFINITY, f64::min)
}

fn main() {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    let hop = HopModel::build(&mut sim);

    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let stand = targets(STAND_THIGH_RAD, STAND_CALF_RAD);
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
    let baseline_y_m = sim.observe().base_y_m;
    let up_reference = {
        let pose = sim.named_transform("base").expect("base pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    println!("baseline_y={baseline_y_m:.3} m");

    let crouch = targets(CROUCH_THIGH_RAD, CROUCH_CALF_RAD);
    for _ in 0..CROUCH_STEPS {
        sim.step_joint_position_targets(&crouch);
    }
    let crouch_y_m = sim.observe().base_y_m;

    let controller = WholeBodyController::new(WholeBodyConfig {
        torque_limits_nm: Some(hop.torque_limits.clone()),
        ..WholeBodyConfig::default()
    });

    let mut apex_y_m = f64::MIN;
    let mut max_tilt_rad = 0.0_f64;
    let mut airborne_steps = 0_u64;
    let mut landed = false;
    let mut last_com_accel = Vec3::ZERO;

    for _ in 0..PUSH_STEPS_MAX {
        let q = hop.q(&sim);
        let qd = hop.qd(&sim);
        let com = center_of_mass(&hop.model, &q).expect("com");
        let contacts: Vec<ContactPoint> = hop
            .foot_names
            .iter()
            .filter_map(|name| hop.model.kinematic().link_entity_by_name(name))
            .map(|link| ContactPoint::new(link, SOLE_OFFSET_LOCAL_M, 0.6))
            .collect();
        let mut com_task = ComTask::hold(com);
        com_task.desired_acceleration_m_s2 = Vec3::new(0.0, PUSH_ACCEL_M_S2, 0.0);
        com_task.position_gain_s_inv2 = 0.0;
        com_task.velocity_gain_s_inv = 4.0;
        let posture = PostureTask {
            desired_joint_positions: q[6..].to_vec(),
            position_gain_s_inv2: 4.0,
            velocity_gain_s_inv: 1.0,
        };
        let solution = controller
            .solve(
                &hop.model,
                &q,
                &qd,
                &contacts,
                Some(&com_task),
                Some(&posture),
            )
            .expect("wbc solve");
        last_com_accel = solution.com_acceleration_m_s2;
        let torques: Vec<UrdfJointTorqueTarget<'_>> = hop
            .joint_links
            .iter()
            .zip(&solution.joint_torque_nm)
            .map(|(link, torque)| UrdfJointTorqueTarget {
                link_name: link.as_str(),
                torque_nm: *torque,
                max_velocity_rad_s: 30.1,
            })
            .collect();
        sim.step_joint_torques(&torques);
        apex_y_m = apex_y_m.max(sim.observe().base_y_m);
        max_tilt_rad = max_tilt_rad.max(tilt_rad(&sim, up_reference));
        if min_foot_height_m(&sim) > AIRBORNE_FOOT_HEIGHT_M {
            airborne_steps += 1;
            break;
        }
    }

    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let tuck = targets(TUCK_THIGH_RAD, TUCK_CALF_RAD);
    for _ in 0..FLIGHT_STEPS_MAX {
        sim.step_joint_position_targets(&tuck);
        apex_y_m = apex_y_m.max(sim.observe().base_y_m);
        max_tilt_rad = max_tilt_rad.max(tilt_rad(&sim, up_reference));
        if min_foot_height_m(&sim) > AIRBORNE_FOOT_HEIGHT_M {
            airborne_steps += 1;
        } else if airborne_steps >= 3 {
            landed = true;
            break;
        }
    }

    let stand_again = targets(STAND_THIGH_RAD, STAND_CALF_RAD);
    for _ in 0..LAND_STEPS {
        sim.step_joint_position_targets(&stand_again);
    }
    let final_observation = sim.observe();

    let jump_height_m = apex_y_m - baseline_y_m;
    println!(
        "crouch_y={crouch_y_m:.3} apex_y={apex_y_m:.3} jump_height={jump_height_m:.3} airborne_steps={airborne_steps} landed={landed} final_y={:.3} max_tilt={max_tilt_rad:.3} com_accel=({:.2},{:.2},{:.2})",
        final_observation.base_y_m, last_com_accel.x, last_com_accel.y, last_com_accel.z,
    );
    if airborne_steps == 0 {
        println!("no liftoff yet");
    } else {
        println!("liftoff: {airborne_steps} airborne steps");
    }
}

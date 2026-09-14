//! Experimental closed-loop force control for a Unitree Go2 hop.
//!
//! State machine: crouch -> force-controlled push-off -> ballistic flight ->
//! landing. During the push-off the controller treats the body as one mass,
//! commands a per-foot vertical force from the desired center-of-mass
//! acceleration, and maps that force to joint torques through the leg Jacobian
//! taken from the simulator's own kinematic model.
//!
//! Status: does **not** achieve liftoff. Independent per-leg Jacobian-transpose
//! force control ignores the floating-base coupling: the body pitches (up to
//! ~1.9 rad) instead of rising, and once the joint torques saturate there is no
//! attitude authority left. A correct hop needs the whole-body inverse dynamics
//! (`rne_wbc`) run against a floating-base model built from the simulator's own
//! world; `UrdfSceneSim` currently exposes `world()` but not `world_mut()`, so
//! that model cannot be constructed in place yet. Kept as the measured
//! starting point for the next attempt.
//!
//! Run with `cargo run -p go2_hop --example 107_go2_hop`.

use rne_ai::{
    unitree_go2_dynamic_scene_path, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_ecs::{Entity, World};
use rne_math::Vec3;
use rne_physics::RigidBody;
use rne_robot::{KinematicModel, Robot};

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
const SPEED_LIMIT_RAD_S: f64 = 30.1;

const STAND_THIGH_RAD: f64 = 0.8;
const STAND_CALF_RAD: f64 = -1.5;
const CROUCH_THIGH_RAD: f64 = 1.15;
const CROUCH_CALF_RAD: f64 = -2.1;
const TUCK_THIGH_RAD: f64 = 0.95;
const TUCK_CALF_RAD: f64 = -1.7;

const GRAVITY_M_S2: f64 = 9.81;
const PUSH_ACCEL_M_S2: f64 = 30.0;
const PITCH_GAIN: f64 = -8.0;
const TORQUE_SIGN: f64 = 1.0;
const AIRBORNE_FOOT_HEIGHT_M: f64 = 0.04;

/// Static mapping from degree-of-freedom index to actuated child-link name.
struct LegMap {
    robot: Entity,
    joint_links: Vec<String>,
    leg_dofs: Vec<[usize; 3]>,
}

impl LegMap {
    fn build(world: &World) -> Self {
        let robot = world
            .iter_entities()
            .find_map(|entity| entity.get::<Robot>().map(|_| entity.id()))
            .expect("robot entity");
        let kinematic = KinematicModel::from_robot(world, robot).expect("kinematic model");
        let joints = kinematic.movable_joint_entities();
        let joint_links: Vec<String> = joints
            .iter()
            .map(|joint| {
                let child = kinematic.joint_child_link(*joint).expect("child link");
                let index = kinematic.link_index(child).expect("link index");
                kinematic.link_name(index).expect("link name").to_string()
            })
            .collect();
        let leg_dofs = LEG_PREFIXES
            .iter()
            .map(|prefix| {
                let mut dofs = [0_usize; 3];
                for (slot, suffix) in ["hip", "thigh", "calf"].iter().enumerate() {
                    let name = format!("{prefix}_{suffix}");
                    dofs[slot] = joint_links
                        .iter()
                        .position(|candidate| candidate == &name)
                        .unwrap_or_else(|| panic!("missing joint link {name}"));
                }
                dofs
            })
            .collect();
        Self {
            robot,
            joint_links,
            leg_dofs,
        }
    }

    fn mass_kg(&self, world: &World) -> f64 {
        let kinematic = KinematicModel::from_robot(world, self.robot).expect("kinematic model");
        (0..kinematic.link_count())
            .filter_map(|index| kinematic.link_entity(index))
            .filter_map(|entity| world.get::<RigidBody>(entity).map(|body| body.mass_kg))
            .sum()
    }

    fn joint_positions(&self, sim: &UrdfSceneSim) -> Vec<f64> {
        self.joint_links
            .iter()
            .map(|link| sim.named_joint_position(link).unwrap_or(0.0))
            .collect()
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

fn pitch_rad(sim: &UrdfSceneSim) -> f64 {
    let pose = sim.named_transform("base").expect("base pose");
    let forward = pose.rotation * Vec3::X;
    forward.y.atan2(forward.x)
}

fn min_foot_height_m(sim: &UrdfSceneSim) -> f64 {
    LEG_PREFIXES
        .iter()
        .filter_map(|prefix| sim.named_transform(&format!("{prefix}_foot")))
        .map(|transform| transform.translation.y)
        .fold(f64::INFINITY, f64::min)
}

/// One tick of force control. Returns the total commanded vertical force, in N.
fn push_tick(
    map: &LegMap,
    mass_kg: f64,
    sim: &mut UrdfSceneSim,
    accel_y_m_s2: f64,
    pitch_rad: f64,
) -> f64 {
    let per_foot_force = mass_kg * (accel_y_m_s2 + GRAVITY_M_S2) / 4.0;
    let torques: Vec<f64> = {
        let kinematic =
            KinematicModel::from_robot(sim.world(), map.robot).expect("kinematic model");
        let q = map.joint_positions(sim);
        let mut torques = vec![0.0; map.joint_links.len()];
        for (leg, prefix) in LEG_PREFIXES.iter().enumerate() {
            let foot = kinematic
                .link_entity_by_name(&format!("{prefix}_foot"))
                .expect("foot link");
            let jacobian = kinematic
                .jacobian(&q, foot, SOLE_OFFSET_LOCAL_M)
                .expect("jacobian");
            let scale = 1.0 + PITCH_GAIN * pitch_rad * if leg < 2 { 1.0 } else { -1.0 };
            let force = Vec3::new(0.0, per_foot_force * scale, 0.0);
            for &dof in &map.leg_dofs[leg] {
                let torque = TORQUE_SIGN
                    * -(jacobian.get(0, dof) * force.x
                        + jacobian.get(1, dof) * force.y
                        + jacobian.get(2, dof) * force.z);
                torques[dof] = torque.clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM);
            }
        }
        torques
    };

    let torque_targets: Vec<UrdfJointTorqueTarget<'_>> = map
        .joint_links
        .iter()
        .zip(&torques)
        .map(|(link, torque)| UrdfJointTorqueTarget {
            link_name: link.as_str(),
            torque_nm: *torque,
            max_velocity_rad_s: SPEED_LIMIT_RAD_S,
        })
        .collect();
    sim.step_joint_torques(&torque_targets);
    per_foot_force * 4.0
}

fn main() {
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
    let map = LegMap::build(sim.world());
    let mass_kg = map.mass_kg(sim.world());
    println!(
        "go2 hop: joints={} mass={mass_kg:.2} kg",
        map.joint_links.len()
    );

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
    println!(
        "baseline_y={baseline_y_m:.3} m pitch0={:.3}",
        pitch_rad(&sim)
    );

    let crouch = targets(CROUCH_THIGH_RAD, CROUCH_CALF_RAD);
    for _ in 0..CROUCH_STEPS {
        sim.step_joint_position_targets(&crouch);
    }
    let crouch_y_m = sim.observe().base_y_m;

    let mut apex_y_m = f64::MIN;
    let mut max_tilt_rad = 0.0_f64;
    let mut airborne_steps = 0_u64;
    let mut landed = false;

    for _ in 0..PUSH_STEPS_MAX {
        let pitch = pitch_rad(&sim);
        push_tick(&map, mass_kg, &mut sim, PUSH_ACCEL_M_S2, pitch);
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
        "crouch_y={crouch_y_m:.3} apex_y={apex_y_m:.3} jump_height={jump_height_m:.3} airborne_steps={airborne_steps} landed={landed} final_y={:.3} max_tilt={max_tilt_rad:.3}",
        final_observation.base_y_m,
    );
    if airborne_steps == 0 {
        println!("no liftoff yet");
    } else {
        println!("liftoff: {airborne_steps} airborne steps");
    }
}

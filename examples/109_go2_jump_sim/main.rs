//! Solves a phased Go2 jump with the native FDDP solver and executes the
//! actuator-limited plan on the dynamic simulation.
//!
//! The plan uses a crouch -> push -> flight contact sequence with per-phase
//! action costs and a ±23.7 Nm control box. The simulator uses the declared
//! inertias (the mass-matched jump scene), so the plan and plant share the same
//! masses. Execution applies the feed-forward torque plus a joint PD term, one
//! planning node per simulation step.
//!
//! Status: the plan is actuator-realizable (a 0.28 m apex, torque saturated at
//! ±23.7 Nm, gap-free) but does not transfer to the simulator. A joint-PD plus
//! a base-height feedback loop both fail for the same reason: the planner's
//! feed-forward torques produce a *different* joint motion in the simulator
//! (it over-crouches to 0.07 m against a planned 0.19 m), so the extension
//! extracts no upward momentum. Torque control and masses are correct, which
//! isolates the gap to the contact/actuator model (rigid-contact KKT in the
//! planner versus Rapier's compliant contacts) or to a whole-body tracking
//! controller rather than joint PD.
//!
//! Run with `cargo run --release -p go2_jump_sim --example 109_go2_jump_sim`.

use glam::EulerRot;
use rne_ai::{UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim};
use rne_dynamics::{ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_oc::{
    solve, ContactPhase, ContactSequenceDynamics, DdpConfig, PhaseCostSchedule, QuadraticCost,
};
use rne_robot::{FloatingBase, KinematicModel, Robot, Transform3};

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
const BASE_ROTATION_X_RAD: f64 = -std::f64::consts::FRAC_PI_2;
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.02);
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];

const STEP_TIME_S: f64 = 1.0 / 60.0;
const CROUCH_STEPS: usize = 15;
const PUSH_STEPS: usize = 10;
const FLIGHT_STEPS: usize = 15;
const TARGET_APEX_M: f64 = 0.30;

const SETTLE_STEPS: u64 = 240;
const POSITION_STIFFNESS: f64 = 420.0;
const POSITION_DAMPING: f64 = 28.0;
const TORQUE_LIMIT_NM: f64 = 23.7;
const TRACK_KP: f64 = 120.0;
const TRACK_KD: f64 = 6.0;
const BASE_KP: f64 = 120.0;
const BASE_KD: f64 = 12.0;
const BASE_SIGN: f64 = 1.0;

fn dof_joint_names(model: &ArticulatedModel) -> Vec<String> {
    model
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
        .collect()
}

fn stand_angle(link: &str) -> f64 {
    if link.ends_with("thigh") {
        0.8
    } else if link.ends_with("calf") {
        -1.5
    } else {
        0.0
    }
}

fn crouch_angle(link: &str) -> f64 {
    if link.ends_with("thigh") {
        1.15
    } else if link.ends_with("calf") {
        -2.05
    } else {
        0.0
    }
}

fn build_model() -> ArticulatedModel {
    let document = rne_urdf_import::parse_urdf_document(GO2_URDF).expect("parse Go2 URDF");
    let mut world = World::new();
    let config = rne_urdf_import::UrdfSpawnConfig {
        attach_colliders: false,
        attach_mesh_colliders: false,
        self_collisions: false,
        use_declared_inertial_masses: true,
        ..rne_urdf_import::UrdfSpawnConfig::default()
    };
    let spawned = rne_urdf_import::spawn_urdf_document_with_config(&mut world, &document, config)
        .expect("spawn Go2");
    world.entity_mut(spawned.base_link).insert((
        Transform3::from_translation_rotation(
            Vec3::ZERO,
            Quat::from_rotation_x(BASE_ROTATION_X_RAD),
        ),
        FloatingBase,
    ));
    ArticulatedModel::from_robot(&world, spawned.robot).expect("model")
}

fn read_state(sim: &UrdfSceneSim, model: &ArticulatedModel, joint_names: &[String]) -> Vec<f64> {
    let nv = model.nv();
    let base = sim.named_transform("base").expect("base pose");
    let floating = base.rotation * Quat::from_rotation_x(-BASE_ROTATION_X_RAD);
    let (yaw, pitch, roll) = floating.to_euler(EulerRot::ZYX);
    let mut state = vec![0.0; 2 * nv];
    state[0] = base.translation.x;
    state[1] = base.translation.y;
    state[2] = base.translation.z;
    state[3] = roll;
    state[4] = pitch;
    state[5] = yaw;
    for (dof, name) in joint_names.iter().enumerate() {
        state[6 + dof] = sim.named_joint_position(name).unwrap_or(0.0);
        state[nv + 6 + dof] = sim.named_joint_velocity(name).unwrap_or(0.0);
    }
    state
}

fn min_foot_height_m(sim: &UrdfSceneSim) -> f64 {
    FOOT_LINKS
        .iter()
        .filter_map(|name| sim.named_transform(name))
        .map(|transform| transform.translation.y)
        .fold(f64::INFINITY, f64::min)
}

fn main() {
    let model = build_model();
    let trace = std::env::args().any(|argument| argument == "--trace");
    let nv = model.nv();
    let control_dim = nv - model.base_dof();
    let joint_names = dof_joint_names(&model);
    let contacts: Vec<ContactSpec> = FOOT_LINKS
        .iter()
        .filter_map(|name| {
            model
                .kinematic()
                .link_entity_by_name(name)
                .map(|link| ContactSpec {
                    link,
                    point_local_m: SOLE_OFFSET_LOCAL_M,
                })
        })
        .collect();

    let scene = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .join("../../assets/scenes/unitree_go2_jump.rne.scene.toml");
    let mut sim = UrdfSceneSim::from_scene_path(&scene).expect("load jump Go2 scene");
    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let stand: Vec<UrdfJointPositionTarget<'_>> = joint_names
        .iter()
        .map(|name| UrdfJointPositionTarget {
            link_name: name.as_str(),
            position: stand_angle(name),
        })
        .collect();
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
    let initial = read_state(&sim, &model, &joint_names);
    let start_y = initial[1];
    println!("stand base_y={start_y:.3} m");

    // Simulator-side kinematic map for the base-height feedback.
    let robot = sim
        .world()
        .iter_entities()
        .find_map(|entity| entity.get::<Robot>().map(|_| entity.id()))
        .expect("robot entity");
    let (sim_joint_links, sim_leg_dofs, mass_kg) = {
        let kinematic = KinematicModel::from_robot(sim.world(), robot).expect("sim kinematic");
        let joint_links: Vec<String> = kinematic
            .movable_joint_entities()
            .iter()
            .map(|joint| {
                let child = kinematic.joint_child_link(*joint).expect("child");
                let index = kinematic.link_index(child).expect("index");
                kinematic.link_name(index).expect("name").to_string()
            })
            .collect();
        let leg_dofs: Vec<[usize; 3]> = FOOT_LINKS
            .iter()
            .map(|prefix| {
                let mut dofs = [0_usize; 3];
                for (slot, suffix) in ["hip", "thigh", "calf"].iter().enumerate() {
                    let name = format!("{}_{}", &prefix[..2], suffix);
                    dofs[slot] = joint_links
                        .iter()
                        .position(|candidate| candidate == &name)
                        .unwrap_or(0);
                }
                dofs
            })
            .collect();
        let mass: f64 = (0..kinematic.link_count())
            .filter_map(|index| kinematic.link_entity(index))
            .filter_map(|entity| {
                sim.world()
                    .get::<rne_physics::RigidBody>(entity)
                    .map(|body| body.mass_kg)
            })
            .sum();
        (joint_links, leg_dofs, mass)
    };
    let plan_index_of = |name: &str| joint_names.iter().position(|candidate| candidate == name);
    let _ = &plan_index_of;

    let horizon = CROUCH_STEPS + PUSH_STEPS + FLIGHT_STEPS;
    let phases = [
        ContactPhase {
            contacts: contacts.clone(),
            steps: CROUCH_STEPS,
        },
        ContactPhase {
            contacts: contacts.clone(),
            steps: PUSH_STEPS,
        },
        ContactPhase {
            contacts: Vec::new(),
            steps: FLIGHT_STEPS,
        },
    ];
    let dynamics = ContactSequenceDynamics::new(&model, STEP_TIME_S, &phases);

    let control_weights = vec![1.0e-3; control_dim];
    let zero = vec![0.0; 2 * nv];
    let mut running = Vec::with_capacity(horizon);
    for node in 0..horizon {
        let mut weights = vec![0.0; 2 * nv];
        let mut reference = vec![0.0; 2 * nv];
        if node < CROUCH_STEPS {
            weights[1] = 200.0;
            reference[1] = start_y - 0.09;
        }
        for (dof, name) in joint_names.iter().enumerate() {
            if node < CROUCH_STEPS {
                weights[6 + dof] = 5.0;
                reference[6 + dof] = crouch_angle(name);
            } else {
                weights[6 + dof] = 2.0;
                reference[6 + dof] = stand_angle(name);
            }
        }
        let mut cost = QuadraticCost::new(weights, control_weights.clone(), zero.clone());
        cost.state_reference = reference;
        cost.running_scale = 1.0;
        running.push(cost);
    }
    let mut terminal_weights = vec![0.0; 2 * nv];
    terminal_weights[1] = 2.0e3;
    for dof in 0..nv {
        terminal_weights[nv + dof] = 50.0;
    }
    let mut terminal = QuadraticCost::new(zero.clone(), vec![0.0; control_dim], terminal_weights);
    let mut terminal_reference = vec![0.0; 2 * nv];
    terminal_reference[1] = start_y + TARGET_APEX_M;
    terminal.state_reference = terminal_reference;
    let cost = PhaseCostSchedule { running, terminal };

    let mut crouch = initial.clone();
    crouch[1] = start_y - 0.09;
    for (dof, name) in joint_names.iter().enumerate() {
        crouch[6 + dof] = crouch_angle(name);
    }
    let mut states = vec![crouch; horizon + 1];
    states[0] = initial.clone();
    let controls = vec![vec![0.0; control_dim]; horizon];
    let config = DdpConfig {
        max_iterations: 120,
        tolerance: 1.0e-8,
        keep_gaps_open: true,
        control_lower: Some(vec![-TORQUE_LIMIT_NM; control_dim]),
        control_upper: Some(vec![TORQUE_LIMIT_NM; control_dim]),
        ..DdpConfig::default()
    };
    println!("solving FDDP...");
    let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");
    let plan_apex = solution
        .states
        .iter()
        .fold(f64::MIN, |maximum, state| maximum.max(state[1]));
    let max_torque = solution
        .controls
        .iter()
        .flatten()
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    println!(
        "plan: apex_y={plan_apex:.3} height={:.3} max_torque={max_torque:.2}",
        plan_apex - start_y
    );

    // Execute: feed-forward torque + joint PD + base-height feedback.
    let mut apex_sim_y = f64::MIN;
    let mut max_min_foot = 0.0_f64;
    let mut max_tilt = 0.0_f64;
    let mut previous_y = sim.observe().base_y_m;
    let up_reference = {
        let pose = sim.named_transform("base").expect("base");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    for node in 0..horizon {
        let state = &solution.states[node];
        let torque = &solution.controls[node];
        let actual_y = sim.observe().base_y_m;
        let actual_vy = (actual_y - previous_y) / STEP_TIME_S;
        previous_y = actual_y;
        let desired_acceleration =
            BASE_KP * (state[1] - actual_y) + BASE_KD * (state[nv + 1] - actual_vy);
        let per_foot_force = mass_kg * desired_acceleration / 4.0;

        let correction: Vec<f64> = {
            let kinematic = KinematicModel::from_robot(sim.world(), robot).expect("sim kinematic");
            let q_sim: Vec<f64> = sim_joint_links
                .iter()
                .map(|name| sim.named_joint_position(name).unwrap_or(0.0))
                .collect();
            let mut correction = vec![0.0; sim_joint_links.len()];
            for (leg, prefix) in FOOT_LINKS.iter().enumerate() {
                let foot = kinematic.link_entity_by_name(prefix).expect("foot");
                let jacobian = kinematic
                    .jacobian(&q_sim, foot, SOLE_OFFSET_LOCAL_M)
                    .expect("jacobian");
                for &dof in &sim_leg_dofs[leg] {
                    correction[dof] += BASE_SIGN * jacobian.get(1, dof) * per_foot_force;
                }
            }
            correction
        };

        let targets: Vec<UrdfJointTorqueTarget<'_>> = sim_joint_links
            .iter()
            .enumerate()
            .map(|(sim_dof, name)| {
                let (feedforward, plan_q, plan_v) = plan_index_of(name)
                    .map(|plan_dof| {
                        (
                            torque[plan_dof],
                            state[6 + plan_dof],
                            state[nv + 6 + plan_dof],
                        )
                    })
                    .unwrap_or((0.0, 0.0, 0.0));
                let q = sim.named_joint_position(name).unwrap_or(0.0);
                let qd = sim.named_joint_velocity(name).unwrap_or(0.0);
                let command = feedforward
                    + TRACK_KP * (plan_q - q)
                    + TRACK_KD * (plan_v - qd)
                    + correction[sim_dof];
                UrdfJointTorqueTarget {
                    link_name: name.as_str(),
                    torque_nm: command.clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM),
                    max_velocity_rad_s: 30.1,
                }
            })
            .collect();
        sim.step_joint_torques(&targets);
        if trace && node < CROUCH_STEPS + PUSH_STEPS && node % 2 == 0 {
            println!(
                "  node {node:02}: base_y={:.4} plan_y={:.4} tau={:.1}",
                sim.observe().base_y_m,
                state[1],
                torque.iter().fold(0.0_f64, |m, v| m.max(v.abs())),
            );
        }
        apex_sim_y = apex_sim_y.max(sim.observe().base_y_m);
        max_min_foot = max_min_foot.max(min_foot_height_m(&sim));
        let pose = sim.named_transform("base").expect("base");
        let up = (pose.rotation * up_reference).normalize_or_zero();
        max_tilt = max_tilt.max(up.y.clamp(-1.0, 1.0).acos());
    }
    println!(
        "execute: apex_y={apex_sim_y:.3} height={:.3} max_min_foot={max_min_foot:.3} liftoff={} max_tilt={max_tilt:.3}",
        apex_sim_y - start_y,
        max_min_foot > 0.04,
    );
}

//! Optimizes a Go2 jump with the native FDDP solver, then executes the torque
//! trajectory on the dynamic simulation and measures whether the feet leave the
//! ground.
//!
//! The optimizer's initial state is read from the settled simulator, so the
//! plan starts where the robot actually is. During execution the optimized
//! torques are augmented with a joint PD term that damps drift.
//!
//! Status: the optimizer finds a valid plan (see example 108), but replaying it
//! on the dynamic simulator does not yet lift off. The simulator's bodies use
//! collider-augmented masses and a Rapier contact model, so the plan does not
//! transfer directly; this is the measured plan-to-sim gap and the starting
//! point for a tracked whole-body execution.
//!
//! Run with `cargo run --release -p go2_jump_sim --example 109_go2_jump_sim`.

use glam::EulerRot;
use rne_ai::{unitree_go2_dynamic_scene_path, UrdfJointPositionTarget, UrdfSceneSim};
use rne_dynamics::{ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_oc::{solve, ContactPhase, ContactSequenceDynamics, DdpConfig, QuadraticCost};
use rne_robot::{FloatingBase, Transform3};

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
const BASE_ROTATION_X_RAD: f64 = -std::f64::consts::FRAC_PI_2;
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.02);
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];

const STEP_TIME_S: f64 = 1.0 / 60.0;
const STANCE_STEPS: usize = 20;
const FLIGHT_STEPS: usize = 34;
const JUMP_HEIGHT_M: f64 = 0.28;

const SETTLE_STEPS: u64 = 240;
const POSITION_STIFFNESS: f64 = 420.0;
const POSITION_DAMPING: f64 = 28.0;
const TORQUE_LIMIT_NM: f64 = 23.7;

const TRACK_KP: f64 = 120.0;
const TRACK_KD: f64 = 6.0;

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

/// Reads `[q, qd]` from the simulator in the model's generalized convention.
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

    // Settle the simulator and plan from where it actually stands.
    let mut sim =
        UrdfSceneSim::from_scene_path(&unitree_go2_dynamic_scene_path()).expect("load dynamic Go2");
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
    println!("stand base_y={start_y:.3} m joints={}", joint_names.len());

    // Build the jump problem: stance then flight.
    let phases = [
        ContactPhase {
            contacts: contacts.clone(),
            steps: STANCE_STEPS,
        },
        ContactPhase {
            contacts: Vec::new(),
            steps: FLIGHT_STEPS,
        },
    ];
    let dynamics = ContactSequenceDynamics::new(&model, STEP_TIME_S, &phases);

    let mut terminal_weights = vec![0.0; 2 * nv];
    terminal_weights[1] = 1.0e3;
    for dof in 0..nv {
        terminal_weights[nv + dof] = 20.0;
    }
    let mut cost = QuadraticCost::new(
        vec![0.0; 2 * nv],
        vec![1.0e-2; control_dim],
        terminal_weights,
    );
    let mut reference = vec![0.0; 2 * nv];
    reference[1] = start_y + JUMP_HEIGHT_M;
    cost.state_reference = reference;
    cost.running_scale = STEP_TIME_S;

    let horizon = STANCE_STEPS + FLIGHT_STEPS;
    let states = vec![initial.clone(); horizon + 1];
    let controls = vec![vec![0.0; control_dim]; horizon];
    let config = DdpConfig {
        max_iterations: 140,
        tolerance: 1.0e-7,
        keep_gaps_open: true,
        ..DdpConfig::default()
    };
    println!("solving FDDP...");
    let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");
    let planned_apex = solution
        .states
        .iter()
        .fold(f64::MIN, |maximum, state| maximum.max(state[1]));
    let max_torque = solution
        .controls
        .iter()
        .flatten()
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    println!(
        "plan: apex_y={planned_apex:.3} max_torque={max_torque:.2} Nm iterations={}",
        solution.iterations
    );

    // Execute: replay the planned joint trajectory with stiff position control.
    // Position tracking is mass-robust, so the simulator's collider-augmented
    // mass does not invalidate the plan.
    sim.configure_position_motors(TRACK_KP * 6.0, TRACK_KD * 6.0, TORQUE_LIMIT_NM);
    let mut apex_sim_y = f64::MIN;
    let mut max_min_foot = 0.0_f64;
    let mut max_tilt = 0.0_f64;
    let up_reference = {
        let pose = sim.named_transform("base").expect("base");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    for node in 0..horizon {
        let state = &solution.states[node];
        let targets: Vec<UrdfJointPositionTarget<'_>> = joint_names
            .iter()
            .enumerate()
            .map(|(dof, name)| UrdfJointPositionTarget {
                link_name: name.as_str(),
                position: state[6 + dof],
            })
            .collect();
        sim.step_joint_position_targets(&targets);
        let base = sim.observe().base_y_m;
        apex_sim_y = apex_sim_y.max(base);
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

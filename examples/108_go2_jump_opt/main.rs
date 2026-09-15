//! Optimizes a Unitree Go2 jump over a fixed contact sequence with the native
//! FDDP solver and Crocoddyl-style per-phase action costs.
//!
//! The sequence is crouch (four feet) -> push (four feet) -> flight (no
//! contacts). Each phase has its own quadratic reference: the crouch phase
//! loads the legs, the push phase extends them, and the terminal cost asks for
//! a target apex with small velocity.
//!
//! Run with `cargo run --release -p go2_jump_opt --example 108_go2_jump_opt`.

use rne_dynamics::{ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_oc::{
    solve, ContactPhase, ContactSequenceDynamics, DdpConfig, PhaseCostSchedule, QuadraticCost,
    ShootingDynamics,
};
use rne_robot::{FloatingBase, Transform3};

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
const BASE_ROTATION_X_RAD: f64 = -std::f64::consts::FRAC_PI_2;
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.02);
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];

const STEP_TIME_S: f64 = 0.02;
const CROUCH_STEPS: usize = 12;
const PUSH_STEPS: usize = 8;
const FLIGHT_STEPS: usize = 25;
const BASE_START_Y_M: f64 = 0.25;
const TARGET_APEX_Y_M: f64 = 0.40;

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

fn main() {
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
    let model = ArticulatedModel::from_robot(&world, spawned.robot).expect("model");
    let nv = model.nv();
    let control_dim = nv - model.base_dof();
    let joint_names = dof_joint_names(&model);

    let mut initial = vec![0.0; 2 * nv];
    initial[1] = BASE_START_Y_M;
    for (dof, name) in joint_names.iter().enumerate() {
        initial[6 + dof] = stand_angle(name);
    }

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
    let horizon = CROUCH_STEPS + PUSH_STEPS + FLIGHT_STEPS;
    println!("go2 jump: nv={nv} control_dim={control_dim} horizon={horizon}");

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
            reference[1] = BASE_START_Y_M - 0.09;
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
    terminal_reference[1] = TARGET_APEX_Y_M;
    terminal.state_reference = terminal_reference;
    let cost = PhaseCostSchedule { running, terminal };

    let mut crouch = initial.clone();
    crouch[1] = BASE_START_Y_M - 0.09;
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
        ..DdpConfig::default()
    };
    println!("solving FDDP...");
    let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");

    let apex_y = solution
        .states
        .iter()
        .fold(f64::MIN, |maximum, state| maximum.max(state[1]));
    let final_y = solution.states.last().expect("states")[1];
    let mut max_gap = 0.0_f64;
    for node in 0..horizon {
        let predicted = dynamics
            .step_at(node, &solution.states[node], &solution.controls[node])
            .expect("step");
        for (a, b) in solution.states[node + 1].iter().zip(&predicted) {
            max_gap = max_gap.max((a - b).abs());
        }
    }
    let max_torque = solution
        .controls
        .iter()
        .flatten()
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    let mut joint_min = f64::MAX;
    let mut joint_max = f64::MIN;
    for state in &solution.states {
        for dof in 0..control_dim {
            joint_min = joint_min.min(state[6 + dof]);
            joint_max = joint_max.max(state[6 + dof]);
        }
    }
    println!(
        "cost={:.4} apex_y={apex_y:.3} final_y={final_y:.3} jump={:.3} max_gap={max_gap:.2e} max_torque={max_torque:.2} joints=[{joint_min:.2},{joint_max:.2}] iterations={}",
        solution.cost,
        apex_y - BASE_START_Y_M,
        solution.iterations,
    );
}

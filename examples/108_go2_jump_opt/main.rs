//! Optimizes a Unitree Go2 jump over a fixed contact sequence with the native
//! FDDP solver.
//!
//! The sequence is stance (four feet) followed by flight (no contacts); the
//! terminal cost asks for a high base with small velocity, so the optimizer
//! must find a push-off that launches the floating base to the target apex.
//! Contact dynamics come from `rne_dynamics::constrained_forward_dynamics` and
//! the solve uses the FDDP warm start in `rne_oc`.
//!
//! Run with `cargo run -p go2_jump_opt --example 108_go2_jump_opt`.

use rne_dynamics::{ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::Vec3;
use rne_oc::{
    solve, ContactPhase, ContactSequenceDynamics, DdpConfig, QuadraticCost, ShootingDynamics,
};
use rne_robot::{FloatingBase, Transform3};

const GO2_URDF: &str = include_str!("../../assets/robots/go2_description/go2_description.rne.urdf");
const BASE_ROTATION_X_RAD: f64 = -std::f64::consts::FRAC_PI_2;
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.02);
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];

const STEP_TIME_S: f64 = 0.02;
const STANCE_STEPS: usize = 15;
const FLIGHT_STEPS: usize = 25;
const BASE_START_Y_M: f64 = 0.25;
const TARGET_APEX_Y_M: f64 = 0.55;

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
            rne_math::Quat::from_rotation_x(BASE_ROTATION_X_RAD),
        ),
        FloatingBase,
    ));
    let model = ArticulatedModel::from_robot(&world, spawned.robot).expect("model");
    let nv = model.nv();
    let control_dim = nv - model.base_dof();
    let joint_names = dof_joint_names(&model);

    // Initial standing configuration.
    let mut initial = vec![0.0; 2 * nv];
    initial[1] = BASE_START_Y_M;
    for (dof, name) in joint_names.iter().enumerate() {
        initial[6 + dof] = stand_angle(name);
    }
    println!(
        "go2 jump: nv={nv} control_dim={control_dim} horizon={}",
        STANCE_STEPS + FLIGHT_STEPS
    );

    // Fixed contact sequence: stance, then flight.
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
    println!("contacts={}", contacts.len());
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

    // Cost: small control effort, terminal base height and velocity.
    let state_weights = vec![0.0; 2 * nv];
    let control_weights = vec![1.0e-2; control_dim];
    let mut terminal_weights = vec![0.0; 2 * nv];
    terminal_weights[1] = 1.0e3;
    for dof in 0..nv {
        terminal_weights[nv + dof] = 20.0;
    }
    let mut cost = QuadraticCost::new(state_weights.clone(), control_weights, terminal_weights);
    let mut reference = vec![0.0; 2 * nv];
    reference[1] = TARGET_APEX_Y_M;
    cost.state_reference = reference;
    cost.running_scale = STEP_TIME_S;

    // Infeasible warm start: hold the initial state with zero controls.
    let states = vec![initial.clone(); STANCE_STEPS + FLIGHT_STEPS + 1];
    let controls = vec![vec![0.0; control_dim]; STANCE_STEPS + FLIGHT_STEPS];
    let config = DdpConfig {
        max_iterations: 120,
        tolerance: 1.0e-7,
        keep_gaps_open: true,
        ..DdpConfig::default()
    };
    println!("solving FDDP...");
    let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");

    let mut apex_y = f64::MIN;
    for state in &solution.states {
        apex_y = apex_y.max(state[1]);
    }
    let final_y = solution.states.last().expect("states")[1];
    // Feasibility: the dynamics gaps must be closed.
    let mut max_gap = 0.0_f64;
    for node in 0..STANCE_STEPS + FLIGHT_STEPS {
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

    println!(
        "cost={:.4} apex_y={apex_y:.3} final_y={final_y:.3} max_gap={max_gap:.2e} max_torque={max_torque:.2} Nm iterations={}",
        solution.cost, solution.iterations
    );
    println!(
        "jump_height_above_start={:.3} m (target apex {TARGET_APEX_Y_M:.2})",
        apex_y - BASE_START_Y_M
    );
}

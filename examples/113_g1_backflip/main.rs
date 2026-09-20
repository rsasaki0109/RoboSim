//! Optimizes a Unitree G1 backflip over a fixed contact sequence with the native
//! FDDP solver.
//!
//! The sequence is crouch (both feet) -> push (both feet) -> flight (no
//! contacts). The flight cost drives the base yaw coordinate through a full
//! `-2*pi` rotation while tracking a ballistic height arc, and the terminal cost
//! asks for a landed pose at the start height.
//!
//! The floating base composes as `base_world = R_euler * R_x(-90)`, so a
//! rotation about the world lateral (Z) axis — a backflip — is a continuous
//! change of the base **yaw** coordinate and does not enter the Euler
//! singularity (`rne_dynamics::base_velocity_map` inverts the Euler-rate map,
//! which is regular for a yaw rotation). The native `rne_oc` integrator now
//! routes the body twist through that map (`rne_dynamics::integrate_configuration`),
//! so the plan and the dynamics share one chart. See example 114 for a
//! render-only kinematic reference animation.
//!
//! Run with `cargo run --release -p g1_backflip --example 113_g1_backflip`.

use rne_dynamics::{ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_oc::{
    solve, ContactPhase, ContactSequenceDynamics, DdpConfig, PhaseCostSchedule, QuadraticCost,
    ShootingDynamics,
};
use rne_robot::{FloatingBase, Transform3};

const G1_URDF: &str = include_str!("../../assets/robots/g1_description/g1_23dof.urdf");
const BASE_ROTATION_X_RAD: f64 = -std::f64::consts::FRAC_PI_2;
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.035);
const FOOT_LINKS: [&str; 2] = ["left_ankle_roll_link", "right_ankle_roll_link"];

const STEP_TIME_S: f64 = 0.03;
const CROUCH_STEPS: usize = 8;
const PUSH_STEPS: usize = 6;
const FLIGHT_STEPS: usize = 22;
const BASE_START_Y_M: f64 = 0.82;
const CROUCH_Y_M: f64 = 0.66;
const TARGET_APEX_Y_M: f64 = 1.05;

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

fn side_sign(name: &str) -> f64 {
    if name.contains("left") {
        1.0
    } else if name.contains("right") {
        -1.0
    } else {
        0.0
    }
}

fn stand_angle(name: &str) -> f64 {
    if name.contains("hip_pitch") {
        -0.18
    } else if name.contains("hip_roll") {
        0.05 * side_sign(name)
    } else if name.contains("knee") {
        0.36
    } else if name.contains("ankle_pitch") {
        -0.18
    } else if name.contains("ankle_roll") {
        -0.03 * side_sign(name)
    } else if name.contains("shoulder_roll") {
        0.20 * side_sign(name)
    } else if name.contains("elbow") {
        0.42
    } else {
        0.0
    }
}

fn crouch_angle(name: &str) -> f64 {
    if name.contains("hip_pitch") {
        -0.55
    } else if name.contains("knee") {
        1.05
    } else if name.contains("ankle_pitch") {
        -0.50
    } else {
        stand_angle(name)
    }
}

fn tuck_angle(name: &str) -> f64 {
    if name.contains("hip_pitch") {
        -1.05
    } else if name.contains("knee") {
        1.75
    } else if name.contains("ankle_pitch") {
        -0.55
    } else if name.contains("shoulder_pitch") {
        -0.60
    } else if name.contains("elbow") {
        0.90
    } else {
        stand_angle(name)
    }
}

fn torque_limit(name: &str) -> f64 {
    if name.contains("knee") {
        139.0
    } else if name.contains("ankle") {
        35.0
    } else if name.contains("shoulder") || name.contains("elbow") || name.contains("wrist") {
        25.0
    } else {
        88.0
    }
}

fn main() {
    let document = rne_urdf_import::parse_urdf_document(G1_URDF).expect("parse G1 URDF");
    let mut world = World::new();
    let config = rne_urdf_import::UrdfSpawnConfig {
        attach_colliders: false,
        attach_mesh_colliders: false,
        self_collisions: false,
        use_declared_inertial_masses: true,
        ..rne_urdf_import::UrdfSpawnConfig::default()
    };
    let spawned = rne_urdf_import::spawn_urdf_document_with_config(&mut world, &document, config)
        .expect("spawn G1");
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
    assert_eq!(control_dim, 23, "expected 23 actuated G1 joints");

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
    assert_eq!(contacts.len(), 2, "expected two foot contacts");
    let horizon = CROUCH_STEPS + PUSH_STEPS + FLIGHT_STEPS;
    println!("g1 backflip FDDP probe: nv={nv} control_dim={control_dim} horizon={horizon}");

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

    let no_flip = std::env::var("G1_NO_FLIP").is_ok();
    let control_weights = vec![2.0e-4; control_dim];
    let zero = vec![0.0; 2 * nv];
    let mut running = Vec::with_capacity(horizon);
    for node in 0..horizon {
        let mut weights = vec![0.0; 2 * nv];
        let mut reference = vec![0.0; 2 * nv];
        if node < CROUCH_STEPS {
            weights[1] = 300.0;
            reference[1] = CROUCH_Y_M;
        }
        for (dof, name) in joint_names.iter().enumerate() {
            if node < CROUCH_STEPS {
                weights[6 + dof] = 8.0;
                reference[6 + dof] = crouch_angle(name);
            } else if node < CROUCH_STEPS + PUSH_STEPS {
                weights[6 + dof] = 3.0;
                reference[6 + dof] = stand_angle(name);
            } else {
                weights[6 + dof] = 2.0;
                reference[6 + dof] = tuck_angle(name);
            }
        }
        if node >= CROUCH_STEPS + PUSH_STEPS {
            let flight = (node - CROUCH_STEPS - PUSH_STEPS) as f64;
            let t = flight / FLIGHT_STEPS as f64;
            // Ballistic height arc and a full backward rotation of the base yaw.
            let arc = (std::f64::consts::PI * t).sin();
            weights[1] = 400.0;
            reference[1] = BASE_START_Y_M + (TARGET_APEX_Y_M - BASE_START_Y_M) * arc;
            if !no_flip {
                weights[5] = 400.0;
                reference[5] = -2.0 * std::f64::consts::PI * t;
            }
        }
        let mut cost = QuadraticCost::new(weights, control_weights.clone(), zero.clone());
        cost.state_reference = reference;
        cost.running_scale = 1.0;
        running.push(cost);
    }

    let mut terminal_weights = vec![0.0; 2 * nv];
    terminal_weights[1] = 4.0e3;
    if !no_flip {
        terminal_weights[5] = 1.0e3;
    }
    for dof in 0..nv {
        terminal_weights[nv + dof] = 40.0;
    }
    let mut terminal = QuadraticCost::new(zero.clone(), vec![0.0; control_dim], terminal_weights);
    let mut terminal_reference = vec![0.0; 2 * nv];
    terminal_reference[1] = BASE_START_Y_M;
    terminal_reference[5] = -2.0 * std::f64::consts::PI;
    for (dof, name) in joint_names.iter().enumerate() {
        terminal_reference[6 + dof] = stand_angle(name);
    }
    terminal.state_reference = terminal_reference;
    let cost = PhaseCostSchedule { running, terminal };

    // Warm start: a crouch pose held, then a tucked spin through flight.
    let mut crouch = initial.clone();
    crouch[1] = CROUCH_Y_M;
    for (dof, name) in joint_names.iter().enumerate() {
        crouch[6 + dof] = crouch_angle(name);
    }
    let mut states = Vec::with_capacity(horizon + 1);
    for node in 0..=horizon {
        let mut state = if node < CROUCH_STEPS {
            crouch.clone()
        } else if node < CROUCH_STEPS + PUSH_STEPS {
            let mut extended = initial.clone();
            extended[1] = BASE_START_Y_M + 0.05;
            extended
        } else {
            let flight = (node - CROUCH_STEPS - PUSH_STEPS) as f64;
            let t = flight / FLIGHT_STEPS as f64;
            let mut tucked = initial.clone();
            tucked[1] = BASE_START_Y_M
                + (TARGET_APEX_Y_M - BASE_START_Y_M) * (std::f64::consts::PI * t).sin();
            tucked[5] = -2.0 * std::f64::consts::PI * t;
            tucked[nv + 5] = -2.0 * std::f64::consts::PI / (FLIGHT_STEPS as f64 * STEP_TIME_S);
            for (dof, name) in joint_names.iter().enumerate() {
                tucked[6 + dof] = tuck_angle(name);
            }
            tucked
        };
        state[nv + 1] = 0.0;
        states.push(state);
    }
    states[0] = initial.clone();

    let controls = vec![vec![0.0; control_dim]; horizon];
    let limits: Vec<f64> = joint_names.iter().map(|name| torque_limit(name)).collect();
    // FDDP keeps the warm-start gaps open on the first pass, so the kinematic
    // reference can seed a solve that the analytic Jacobians then close.
    let config = DdpConfig {
        max_iterations: 300,
        tolerance: 1.0e-7,
        keep_gaps_open: true,
        control_lower: Some(limits.iter().map(|limit| -limit).collect()),
        control_upper: Some(limits.clone()),
        ..DdpConfig::default()
    };
    println!("solving FDDP...");
    let solution = solve(&dynamics, &cost, &states, &controls, &config).expect("solve");

    let apex_y = solution
        .states
        .iter()
        .fold(f64::MIN, |maximum, state| maximum.max(state[1]));
    let final_y = solution.states.last().expect("states")[1];
    let min_yaw = solution
        .states
        .iter()
        .fold(f64::MAX, |minimum, state| minimum.min(state[5]));
    let max_yaw = solution
        .states
        .iter()
        .fold(f64::MIN, |maximum, state| maximum.max(state[5]));
    let mut max_gap = 0.0_f64;
    let mut failed_nodes = 0_usize;
    for node in 0..horizon {
        match dynamics.step_at(node, &solution.states[node], &solution.controls[node]) {
            Ok(predicted) => {
                for (a, b) in solution.states[node + 1].iter().zip(&predicted) {
                    max_gap = max_gap.max((a - b).abs());
                }
            }
            Err(_) => failed_nodes += 1,
        }
    }
    let feasible = max_gap < 1.0e-4 && failed_nodes == 0;
    println!(
        "converged={} feasible={} cost_finite={}",
        solution.converged,
        feasible,
        solution.cost.is_finite(),
    );
    let max_torque = solution
        .controls
        .iter()
        .flatten()
        .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
    println!(
        "cost={:.4} apex_y={apex_y:.3} final_y={final_y:.3} jump={:.3} yaw=[{min_yaw:.2},{max_yaw:.2}] span={:.2} rad max_gap={max_gap:.2e} max_torque={max_torque:.1} iterations={}",
        solution.cost,
        apex_y - BASE_START_Y_M,
        max_yaw - min_yaw,
        solution.iterations,
    );
}

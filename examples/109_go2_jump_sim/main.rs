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
//! planner versus Rapier's compliant contacts).
//!
//! `--position-stance` instead tracks the planned joint trajectory with position
//! motors during stance (optionally phase-leading with `--lookahead`, gains via
//! `--kp`/`--kd`). This bypasses the contact mismatch and follows the plan much
//! more closely: the base rises 0.227 m (apex 0.449 m) versus 0.07 m under
//! torque control. It still does not lift off cleanly — the feet never leave the
//! ground and the base pitches ~38 degrees — because the position loop lags the
//! plan and the rise comes from leg extension plus a pitch rather than a
//! ballistic launch. Closing that gap needs a whole-body tracking controller
//! (`rne_wbc`) at the simulation rate, not gain tuning.
//!
//! `--wbc-stance` runs `rne_wbc` during stance, tracking the plan's center of
//! mass (position/velocity/acceleration) and holding the base level while the
//! four feet stay fixed (gains via `--wbc-kp`/`--wbc-kd`). It does inject the
//! planned energy — with a velocity gain of 20 the base reaches the planned
//! apex (0.525 m versus a planned 0.503 m) — but the base then pitches over
//! (about 2 rad) and the feet never leave the ground. The blocker was a model
//! mismatch: the floating model built from the simulator's own world places the
//! link frames differently from the plan model (the standing CoM reads 0.247 m
//! versus the plan's 0.135 m even though the link inertias and total mass are
//! identical), so the controller over-injected. Solving the WBC on the plan's
//! own model with the simulator state now tracks the planned CoM closely, but
//! the base still does not launch. Tracking the planned joint trajectory with
//! position motors during flight keeps the base much more level (a 0.65 rad
//! lean instead of a flip), so the remaining gap is purely the push: the
//! simulator's stance never reaches the planned takeoff velocity.
//!
//! `--vel-weight` wraps the planner cost in `rne_oc::ActuatorLimitCost`, a
//! hinge penalty that keeps joint speeds near their URDF limits. The default
//! plan peaks at 44 rad/s — well beyond the Go2 thigh limit of 15.7 rad/s — so
//! the unconstrained optimum is not even physically realizable. The penalty
//! brings the peak to 31 rad/s at a small apex cost, but the transfer still
//! fails, which locates the blocker in the planner/plant model mismatch (the
//! landed model never reaches the planned takeoff velocity) rather than in
//! actuator bandwidth.
//!
//! Run with `cargo run --release -p go2_jump_sim --example 109_go2_jump_sim`.

use glam::EulerRot;
use rne_ai::{UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim};
use rne_dynamics::{center_of_mass, ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_oc::{
    solve, ActuatorLimitCost, ContactPhase, ContactSequenceDynamics, DdpConfig, PhaseCostSchedule,
    QuadraticCost,
};
use rne_robot::{FloatingBase, KinematicModel, Robot, Transform3};
use rne_wbc::{
    BaseAttitudeTask, ComTask, ContactPoint, PostureTask, WholeBodyConfig, WholeBodyController,
};

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
const WBC_COM_KP: f64 = 120.0;
const WBC_COM_KD: f64 = 20.0;
const WBC_ATTITUDE_KP: f64 = -40.0;
const WBC_ATTITUDE_KD: f64 = 8.0;
const ACTUATOR_VELOCITY_WEIGHT: f64 = 5.0;

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

#[allow(clippy::needless_range_loop)]
fn main() {
    let model = build_model();
    let trace = std::env::args().any(|argument| argument == "--trace");
    let position_stance = std::env::args().any(|argument| argument == "--position-stance");
    let wbc_stance = std::env::args().any(|argument| argument == "--wbc-stance");
    let stance_kp = argument_value("--kp")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(320.0);
    let stance_kd = argument_value("--kd")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(14.0);
    let wbc_kp = argument_value("--wbc-kp")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(WBC_COM_KP);
    let wbc_kd = argument_value("--wbc-kd")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(WBC_COM_KD);
    let velocity_weight = argument_value("--vel-weight")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(ACTUATOR_VELOCITY_WEIGHT);
    let lookahead = argument_value("--lookahead")
        .and_then(|value| value.parse::<usize>().ok())
        .unwrap_or(0);
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
    let (sim_joint_links, sim_leg_dofs, mass_kg, sim_kinematic) = {
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
        (joint_links, leg_dofs, mass, kinematic)
    };
    let plan_index_of = |name: &str| joint_names.iter().position(|candidate| candidate == name);
    let _ = &plan_index_of;

    if position_stance {
        sim.configure_position_motors(stance_kp, stance_kd, TORQUE_LIMIT_NM);
    }
    if wbc_stance {
        sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    }
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
    let velocity_limits: Vec<f64> = model
        .kinematic()
        .joint_limits()
        .iter()
        .map(|limits| limits.max_velocity)
        .collect();
    let cost = ActuatorLimitCost::new(
        PhaseCostSchedule { running, terminal },
        velocity_limits,
        velocity_weight,
        nv,
        0,
    );

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
    let takeoff = CROUCH_STEPS + PUSH_STEPS;
    let takeoff_base_vel =
        (solution.states[takeoff + 1][1] - solution.states[takeoff][1]) / STEP_TIME_S;
    let mut max_joint_speed = 0.0_f64;
    for state in &solution.states {
        for dof in 0..control_dim {
            max_joint_speed = max_joint_speed.max(state[nv + 6 + dof].abs());
        }
    }
    println!(
        "plan: apex_y={plan_apex:.3} height={:.3} max_torque={max_torque:.2} takeoff_base_vel={takeoff_base_vel:.2} max_joint_speed={max_joint_speed:.2}",
        plan_apex - start_y
    );

    let plan_com: Vec<Vec3> = solution
        .states
        .iter()
        .map(|state| center_of_mass(&model, &state[..nv]).expect("plan com"))
        .collect();
    let plan_com_velocity = |index: usize| -> Vec3 {
        let next = (index + 1).min(solution.states.len() - 1);
        let previous = index;
        (plan_com[next] - plan_com[previous]) / STEP_TIME_S
    };
    let plan_com_acceleration = |index: usize| -> Vec3 {
        let next = (index + 1).min(solution.states.len() - 1);
        let previous = index.saturating_sub(1);
        (plan_com[next] - plan_com[index] * 2.0 + plan_com[previous]) / (STEP_TIME_S * STEP_TIME_S)
    };
    let wbc_controller = WholeBodyController::new(WholeBodyConfig {
        com_weight: 1.0e5,
        torque_limits_nm: Some(vec![TORQUE_LIMIT_NM; control_dim]),
        ..WholeBodyConfig::default()
    });

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

        if wbc_stance && node >= CROUCH_STEPS + PUSH_STEPS {
            let position_targets: Vec<UrdfJointPositionTarget<'_>> = joint_names
                .iter()
                .map(|name| UrdfJointPositionTarget {
                    link_name: name.as_str(),
                    position: state[6 + joint_names.iter().position(|n| n == name).unwrap_or(0)],
                })
                .collect();
            sim.step_joint_position_targets(&position_targets);
            apex_sim_y = apex_sim_y.max(sim.observe().base_y_m);
            max_min_foot = max_min_foot.max(min_foot_height_m(&sim));
            let pose = sim.named_transform("base").expect("base");
            let up = (pose.rotation * up_reference).normalize_or_zero();
            let tilt = up.y.clamp(-1.0, 1.0).acos();
            max_tilt = max_tilt.max(tilt);
            if trace {
                println!(
                    "  node {node:02}: base_y={:.4} plan_y={:.4} min_foot={:.4} tilt={:.3} (flight position)",
                    sim.observe().base_y_m,
                    state[1],
                    min_foot_height_m(&sim),
                    tilt,
                );
            }
            continue;
        }
        if wbc_stance && node < CROUCH_STEPS + PUSH_STEPS {
            let state_now = read_state(&sim, &model, &joint_names);
            let q = &state_now[..nv];
            let qd = vec![0.0; nv];
            let foot_contacts: Vec<ContactPoint> = FOOT_LINKS
                .iter()
                .filter_map(|name| model.kinematic().link_entity_by_name(name))
                .map(|link| ContactPoint::new(link, SOLE_OFFSET_LOCAL_M, 0.6))
                .collect();
            let com_task = ComTask {
                desired_position_m: plan_com[node],
                desired_velocity_m_s: plan_com_velocity(node),
                desired_acceleration_m_s2: plan_com_acceleration(node),
                position_gain_s_inv2: wbc_kp,
                velocity_gain_s_inv: wbc_kd,
            };
            let base_pose = sim.named_transform("base").expect("base pose");
            let floating_rotation =
                base_pose.rotation * Quat::from_rotation_x(-BASE_ROTATION_X_RAD);
            let up_world = (base_pose.rotation * Vec3::Z).normalize_or_zero();
            let tilt_axis_world = Vec3::Y.cross(up_world);
            let sin_angle = tilt_axis_world.length();
            let tilt_angle = up_world.y.clamp(-1.0, 1.0).acos();
            let axis_body = if sin_angle > 1.0e-6 {
                floating_rotation.inverse() * (tilt_axis_world / sin_angle)
            } else {
                Vec3::ZERO
            };
            let observation = sim.observe();
            let omega_body = floating_rotation.inverse()
                * Vec3::new(
                    observation.base_angular_velocity_x_rad_s,
                    observation.base_angular_velocity_y_rad_s,
                    observation.base_angular_velocity_z_rad_s,
                );
            let attitude = BaseAttitudeTask {
                desired_angular_acceleration_rad_s2: axis_body * (WBC_ATTITUDE_KP * tilt_angle)
                    - omega_body * WBC_ATTITUDE_KD,
            };
            let posture = PostureTask {
                desired_joint_positions: q[6..].to_vec(),
                position_gain_s_inv2: 4.0,
                velocity_gain_s_inv: 1.0,
            };
            let wbc_solution = wbc_controller
                .solve(
                    &model,
                    q,
                    &qd,
                    &foot_contacts,
                    Some(&com_task),
                    Some(&attitude),
                    Some(&posture),
                )
                .expect("wbc solve");
            let wbc_targets: Vec<UrdfJointTorqueTarget<'_>> = joint_names
                .iter()
                .zip(&wbc_solution.joint_torque_nm)
                .map(|(link, torque)| UrdfJointTorqueTarget {
                    link_name: link.as_str(),
                    torque_nm: torque.clamp(-TORQUE_LIMIT_NM, TORQUE_LIMIT_NM),
                    max_velocity_rad_s: 30.1,
                })
                .collect();
            sim.step_joint_torques(&wbc_targets);
            if trace {
                println!(
                    "  node {node:02}: base_y={:.4} sim_com={:.4} plan_com_y={:.4} min_foot={:.4} (wbc)",
                    sim.observe().base_y_m,
                    center_of_mass(&model, q).expect("com").y,
                    plan_com[node].y,
                    min_foot_height_m(&sim),
                );
            }
            apex_sim_y = apex_sim_y.max(sim.observe().base_y_m);
            max_min_foot = max_min_foot.max(min_foot_height_m(&sim));
            let pose = sim.named_transform("base").expect("base");
            let up = (pose.rotation * up_reference).normalize_or_zero();
            max_tilt = max_tilt.max(up.y.clamp(-1.0, 1.0).acos());
            continue;
        }

        let correction: Vec<f64> = {
            let kinematic = &sim_kinematic;
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

        if position_stance && node < CROUCH_STEPS + PUSH_STEPS {
            let position_targets: Vec<UrdfJointPositionTarget<'_>> = sim_joint_links
                .iter()
                .map(|name| {
                    let reference = solution
                        .states
                        .get((node + lookahead).min(horizon - 1))
                        .unwrap_or(state);
                    let position = plan_index_of(name)
                        .map(|plan_dof| reference[6 + plan_dof])
                        .unwrap_or_else(|| sim.named_joint_position(name).unwrap_or(0.0));
                    UrdfJointPositionTarget {
                        link_name: name.as_str(),
                        position,
                    }
                })
                .collect();
            sim.step_joint_position_targets(&position_targets);
            if trace {
                println!(
                    "  node {node:02}: base_y={:.4} plan_y={:.4} min_foot={:.4} (position)",
                    sim.observe().base_y_m,
                    state[1],
                    min_foot_height_m(&sim),
                );
            }
            apex_sim_y = apex_sim_y.max(sim.observe().base_y_m);
            max_min_foot = max_min_foot.max(min_foot_height_m(&sim));
            let pose = sim.named_transform("base").expect("base");
            let up = (pose.rotation * up_reference).normalize_or_zero();
            max_tilt = max_tilt.max(up.y.clamp(-1.0, 1.0).acos());
            continue;
        }
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
        if trace {
            let pose = sim.named_transform("base").expect("base");
            let up = (pose.rotation * up_reference).normalize_or_zero();
            println!(
                "  node {node:02}: base_y={:.4} plan_y={:.4} min_foot={:.4} tilt={:.3} (torque)",
                sim.observe().base_y_m,
                state[1],
                min_foot_height_m(&sim),
                up.y.clamp(-1.0, 1.0).acos(),
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

fn argument_value(flag: &str) -> Option<String> {
    let arguments: Vec<String> = std::env::args().collect();
    arguments
        .iter()
        .position(|argument| argument == flag)
        .and_then(|index| arguments.get(index + 1).cloned())
}

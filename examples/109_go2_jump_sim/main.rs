//! Solves a phased Go2 jump with the native FDDP solver and executes the
//! actuator-limited plan on the dynamic simulation.
//!
//! The plan uses a crouch -> push -> flight contact sequence with per-phase
//! action costs and a ±23.7 Nm control box. The simulator uses the declared
//! inertias (the mass-matched jump scene), so the plan and plant share the same
//! masses. Execution applies the feed-forward torque plus a joint PD term, one
//! planning node per simulation step.
//!
//! Status: the plan is actuator-realizable (a 0.25 m apex, torque saturated at
//! ±23.7 Nm, gap-free). It originally failed to transfer, and the cause was
//! not the controller: the plant's Go2 **foot link** was a free rigid body. The
//! URDF multibody excluded links reachable only through fixed joints, so the
//! foot (welded to the calf) fell away and the plant's foot frame never matched
//! the planner's. `--debug-fk` shows the error — the base and the chain through
//! the calf match the URDF forward kinematics, but the foot sits 0.138 m from
//! the calf instead of 0.213 m. Attaching it with the jump robot's
//! `weld_fixed_children = true` option makes the optimized jump lift off:
//!
//! - torque + PD: apex 0.324 m, tilt 0.45 rad, feet skim but do not lift off
//! - `--position-stance` (`--lookahead`): apex 0.402 m, jump 0.070 m, tilt 0.67
//! - `--wbc-stance`: apex 0.438 m, jump 0.106 m (plan 0.250 m), tilt 0.73 rad
//!
//! **The jump is not clean yet: the base pitches forward about 0.7 rad and the
//! robot lands nose-first.** The pitch is not from the pose — a static crouch
//! and a position-controlled push both stay level (tilt < 0.02 rad) — and the
//! plan itself stays upright (`base_x` within 5 cm, `base_pitch` within 0.17
//! rad). It is the whole-body center-of-mass task: while the four feet are
//! planted it satisfies the planned center of mass with a pitched base, and
//! `BaseAttitudeTask` (which only commands the base angular acceleration) has
//! almost no authority against it — raising its weight from 1e4 to 1e8 changes
//! the peak lean by less than 0.01 rad. Shrinking the jump hides the landing
//! but not the pitch, so the taller plan is kept and the attitude formulation
//! is the open problem.
//!
//! The example still owns the **landing**: the flight plan ends near the apex,
//! so it catches the touchdown with stiff position motors and settles into the
//! stand (`landed=true`, end height 0.332 m, settled tilt 0.006 rad). The catch
//! is open-loop and only holds a small jump, so the pitched 0.30 plan tumbles
//! on touchdown. A closed-loop landing controller and a base-attitude task with
//! real authority are the next steps.
//!
//! The whole-body stance feed has one further subtlety: it must pass the
//! *measured* joint velocities to the solver (feeding zeros over-drives the
//! center of mass and inflates the apex while the base pitches far more).
//!
//! `--vel-weight` wraps the planner cost in `rne_oc::ActuatorLimitCost`, a hinge
//! penalty that keeps joint speeds near their URDF limits. The default plan
//! peaks at 44 rad/s, beyond the Go2 thigh limit of 15.7 rad/s; the penalty
//! brings it to 31 rad/s.
//!
//! Tuning knobs: `--wbc-kp`/`--wbc-kd` (center-of-mass gains), `--com-ff`
//! (center-of-mass acceleration feed-forward), `--att-kp`/`--att-kd`/
//! `--att-weight` (base attitude), `--apex` (plan target), `--land-kp`/
//! `--land-kd` (touchdown catch), `--vel-weight`, and `--kp`/`--kd`/
//! `--lookahead` (position stance).
//!
//! `--gif` renders the live jump to `docs/media/go2-jump.gif`: the frames come
//! from the same headless simulation the metrics report, so the GIF shows the
//! real jump (requires a GPU; set `RNE_SKIP_GPU` to skip the capture).
//!
//! Run with `cargo run --release -p go2_jump_sim --example 109_go2_jump_sim`.

use glam::EulerRot;
use rne_ai::{
    build_visual_render_scene, UrdfJointPositionTarget, UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_dynamics::{center_of_mass, ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_oc::{
    solve, ActuatorLimitCost, ContactPhase, ContactSequenceDynamics, DdpConfig, PhaseCostSchedule,
    QuadraticCost,
};
use rne_render::{
    Camera, MeshRenderCache, RenderBackend, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_wgpu::{CameraOrbit, WgpuRenderBackend};
use rne_robot::{FloatingBase, KinematicModel, Robot, Transform3};
use rne_wbc::{
    BaseAttitudeTask, ComTask, ContactPoint, PostureTask, WholeBodyConfig, WholeBodyController,
};
use std::path::{Path, PathBuf};

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
const WBC_COM_KD: f64 = 40.0;
const WBC_COM_FF: f64 = 2.0;
const WBC_ATTITUDE_KP: f64 = -40.0;
const WBC_ATTITUDE_KD: f64 = 8.0;
const ACTUATOR_VELOCITY_WEIGHT: f64 = 5.0;

/// GIF capture frame size in pixels.
const GIF_WIDTH: u32 = 560;
const GIF_HEIGHT: u32 = 600;
/// Background color for captured GIF frames.
const GIF_CLEAR_COLOR: [f32; 4] = [0.035, 0.05, 0.08, 1.0];
/// Settle frames captured as a lead-in.
const GIF_LEAD_IN_FRAMES: u64 = 8;
/// Simulation steps of touchdown catch and settling after the flight plan.
const LANDING_STEPS: u64 = 300;
/// Position-motor gains for the touchdown catch.
const LANDING_STIFFNESS: f64 = 900.0;
const LANDING_DAMPING: f64 = 40.0;

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
    let gif = std::env::args().any(|argument| argument == "--gif");
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
    let attitude_kp = argument_value("--att-kp")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(WBC_ATTITUDE_KP);
    let attitude_kd = argument_value("--att-kd")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(WBC_ATTITUDE_KD);
    let target_apex = argument_value("--apex")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(TARGET_APEX_M);
    let land_kp = argument_value("--land-kp")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(LANDING_STIFFNESS);
    let land_kd = argument_value("--land-kd")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(LANDING_DAMPING);
    let com_ff = argument_value("--com-ff")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(WBC_COM_FF);
    let posture_weight = argument_value("--posture-weight")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(WholeBodyConfig::default().posture_weight);
    let angular_weight = argument_value("--att-weight")
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(WholeBodyConfig::default().angular_weight);
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
    let mut capture = if gif {
        if std::env::var("RNE_SKIP_GPU").is_ok() {
            None
        } else {
            Some(GifCapture::new(&sim))
        }
    } else {
        None
    };
    for step in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
        if let Some(capture) = capture.as_mut() {
            if step + GIF_LEAD_IN_FRAMES >= SETTLE_STEPS {
                capture.capture(&sim);
            }
        }
    }
    macro_rules! capture_frame {
        () => {
            if let Some(capture) = capture.as_mut() {
                capture.capture(&sim);
            }
        };
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
    // Diagnostic: the planner uses the URDF forward kinematics while the plant
    // is a Rapier articulation. Compare the two link frames at one state.
    if std::env::args().any(|a| a == "--debug-fk") {
        let q_full = read_state(&sim, &model, &joint_names);
        let fk = model
            .kinematic()
            .forward_kinematics(&q_full[..model.nv()])
            .expect("forward kinematics");
        let transforms = fk.transforms();
        for name in ["base", "FL_hip", "FL_thigh", "FL_calf", "FL_foot"] {
            let entity = model.kinematic().link_entity_by_name(name).expect(name);
            let index = model.kinematic().link_index(entity).expect("index");
            let plan = transforms[index].translation;
            let observed = sim.named_transform(name).map(|t| t.translation);
            println!(
                "  {name:9} plan=({:+.4},{:+.4},{:+.4}) sim={observed:?}",
                plan.x, plan.y, plan.z
            );
        }
        for name in ["FL_calf", "FL_foot"] {
            let entity = sim
                .world()
                .iter_entities()
                .find(|e| {
                    sim.world()
                        .get::<rne_ecs::Name>(e.id())
                        .is_some_and(|n| n.0 == name)
                })
                .map(|e| e.id());
            if let Some(e) = entity {
                let body = sim.world().get::<rne_physics::RigidBody>(e);
                println!(
                    "  {name:9} fixed_joint={} revolute_joint={} multibody={} body={:?}",
                    sim.world().get::<rne_physics::FixedJointDesc>(e).is_some(),
                    sim.world()
                        .get::<rne_physics::RevoluteJointDesc>(e)
                        .is_some(),
                    sim.world().get::<rne_physics::MultibodyLink>(e).is_some(),
                    body.map(|b| b.body_type),
                );
            }
        }
        return;
    }

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
    terminal_reference[1] = start_y + target_apex;
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
        posture_weight,
        angular_weight,
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
            capture_frame!();
            continue;
        }
        if wbc_stance && node < CROUCH_STEPS + PUSH_STEPS {
            let state_now = read_state(&sim, &model, &joint_names);
            let q = &state_now[..nv];
            let qd = state_now[nv..].to_vec();
            let foot_contacts: Vec<ContactPoint> = FOOT_LINKS
                .iter()
                .filter_map(|name| model.kinematic().link_entity_by_name(name))
                .map(|link| ContactPoint::new(link, SOLE_OFFSET_LOCAL_M, 0.6))
                .collect();
            let com_task = ComTask {
                desired_position_m: plan_com[node],
                desired_velocity_m_s: plan_com_velocity(node),
                desired_acceleration_m_s2: plan_com_acceleration(node) * com_ff,
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
                desired_angular_acceleration_rad_s2: axis_body * (attitude_kp * tilt_angle)
                    - omega_body * attitude_kd,
            };
            let posture = PostureTask {
                desired_joint_positions: state[6..nv].to_vec(),
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
            let pose_now = sim.named_transform("base").expect("base");
            let up_now = (pose_now.rotation * up_reference).normalize_or_zero();
            let stance_tilt = up_now.y.clamp(-1.0, 1.0).acos();
            if trace {
                println!(
                    "  node {node:02}: base_y={:.4} sim_com={:.4} plan_com_y={:.4} min_foot={:.4} tilt={stance_tilt:.3} (wbc)",
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
            capture_frame!();
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
            capture_frame!();
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
        capture_frame!();
    }
    // Landing: catch the touchdown and settle back into the stand. The flight
    // plan ends near the apex, so the example owns the descent: stiff position
    // motors drive the stand pose, absorb the impact, and hold it.
    sim.configure_position_motors(land_kp, land_kd, TORQUE_LIMIT_NM);
    let mut settled_tilt = 0.0_f64;
    let mut touchdown_y = f64::MAX;
    let landing_targets: Vec<UrdfJointPositionTarget<'_>> = joint_names
        .iter()
        .map(|name| UrdfJointPositionTarget {
            link_name: name.as_str(),
            position: stand_angle(name),
        })
        .collect();
    for step in 0..LANDING_STEPS {
        sim.step_joint_position_targets(&landing_targets);
        touchdown_y = touchdown_y.min(min_foot_height_m(&sim));
        let pose = sim.named_transform("base").expect("base");
        let up = (pose.rotation * up_reference).normalize_or_zero();
        let tilt = up.y.clamp(-1.0, 1.0).acos();
        if step + 90 >= LANDING_STEPS {
            settled_tilt = settled_tilt.max(tilt);
        }
        if trace && step % 30 == 0 {
            println!(
                "  land {step:03}: base_y={:.3} tilt={:.3} min_foot={:.3}",
                sim.observe().base_y_m,
                tilt,
                min_foot_height_m(&sim),
            );
        }
        // Subsample the slow settle so the GIF stays short.
        if step % 5 == 0 || step + 1 == LANDING_STEPS {
            capture_frame!();
        }
    }
    let landed_height = sim.observe().base_y_m;
    let landed = settled_tilt < 0.6 && landed_height > 0.15;
    println!(
        "landing: end_height={landed_height:.3} settled_tilt={settled_tilt:.3} touchdown_min_foot={touchdown_y:.3} landed={landed}"
    );
    if let Some(capture) = capture.as_mut() {
        capture.encode();
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

/// Renders the live jump simulation to `docs/media/go2-jump.gif`.
///
/// The frames come from the same headless simulation the metrics report, so the
/// GIF shows the real jump and not a replayed or scripted sequence.
struct GifCapture {
    backend: WgpuRenderBackend,
    camera: Camera,
    mesh_cache: MeshRenderCache,
    mesh_roots: Vec<PathBuf>,
    frames_dir: PathBuf,
    frame: usize,
}

impl GifCapture {
    fn new(sim: &UrdfSceneSim) -> Self {
        let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let frames_dir = repo_root.join("docs/media/go2-jump-frames");
        let _ = std::fs::remove_dir_all(&frames_dir);
        std::fs::create_dir_all(&frames_dir).expect("create jump frame directory");
        let backend = WgpuRenderBackend::new().expect("initialize wgpu");
        let camera = Camera::new(GIF_WIDTH, GIF_HEIGHT, std::f64::consts::FRAC_PI_4);
        Self {
            backend,
            camera,
            mesh_cache: MeshRenderCache::new(),
            mesh_roots: sim.mesh_package_roots().to_vec(),
            frames_dir,
            frame: 0,
        }
    }

    fn capture(&mut self, sim: &UrdfSceneSim) {
        let mesh_refs: Vec<&Path> = self.mesh_roots.iter().map(PathBuf::as_path).collect();
        let mut scene = build_visual_render_scene(sim.world());
        scene
            .items
            .retain(|item| !matches!(item.shape, VisualShape::Box { .. }));
        let observed = sim.observe();
        append_checker_floor(&mut scene, observed.base_x_m, observed.base_z_m, 0.18);
        self.mesh_cache
            .resolve_scene(&mut scene, &mesh_refs)
            .expect("resolve official Go2 meshes");
        let orbit = CameraOrbit {
            focus: Vec3::new(
                observed.base_x_m,
                (observed.base_y_m - 0.08).max(0.05),
                observed.base_z_m,
            ),
            yaw_rad: -1.25,
            pitch_rad: 1.02,
            distance_m: 2.3,
        };
        let output = self
            .backend
            .render_scene_camera(
                &self.camera,
                &orbit.camera_transform(),
                &scene,
                GIF_CLEAR_COLOR,
            )
            .expect("render jump frame");
        write_png(
            &self.frames_dir.join(format!("frame-{:03}.png", self.frame)),
            &output.color.rgba8,
            GIF_WIDTH,
            GIF_HEIGHT,
        )
        .expect("write jump frame");
        self.frame += 1;
    }

    fn encode(&self) {
        let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let gif_path = repo_root.join("docs/media/go2-jump.gif");
        build_gif(&self.frames_dir, &gif_path).expect("encode jump gif");
        let _ = std::fs::remove_dir_all(&self.frames_dir);
        println!(
            "rendered jump media to {} ({} frames)",
            gif_path.display(),
            self.frame
        );
    }
}

fn append_checker_floor(scene: &mut RenderScene, center_x_m: f64, center_z_m: f64, tile_m: f64) {
    let snap = |value: f64| (value / (2.0 * tile_m)).floor() * 2.0 * tile_m;
    for row in -10..=10 {
        for column in -10..=10 {
            let color = if (row + column) & 1 == 0 {
                [0.11, 0.15, 0.21, 1.0]
            } else {
                [0.055, 0.075, 0.11, 1.0]
            };
            scene.items.push(RenderSceneItem {
                transform: rne_math::Transform3 {
                    translation: Vec3::new(
                        snap(center_x_m) + column as f64 * tile_m,
                        -0.008,
                        snap(center_z_m) + row as f64 * tile_m,
                    ),
                    rotation: Quat::IDENTITY,
                    scale: Vec3::new(tile_m * 0.96, 0.008, tile_m * 0.96),
                },
                shape: VisualShape::Box { size_m: Vec3::ONE },
                color_rgba: color,
                mesh: None,
                base_color_texture: None,
                material: Default::default(),
            });
        }
    }
}

fn build_gif(frames_dir: &Path, gif_path: &Path) -> std::io::Result<()> {
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-framerate",
            "12",
            "-i",
            &frames_dir.join("frame-%03d.png").to_string_lossy(),
            "-vf",
            "fps=12,scale=560:-1:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors=160[p];[s1][p]paletteuse=dither=bayer:bayer_scale=3",
            &gif_path.to_string_lossy(),
        ])
        .status()?;
    status
        .success()
        .then_some(())
        .ok_or_else(|| std::io::Error::other("ffmpeg jump gif encode failed"))
}

fn write_png(path: &Path, rgba: &[u8], width: u32, height: u32) -> std::io::Result<()> {
    use png::{BitDepth, ColorType, Encoder};
    let file = std::fs::File::create(path)?;
    let mut encoder = Encoder::new(file, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut writer = encoder.write_header()?;
    writer.write_image_data(rgba).map_err(std::io::Error::other)
}

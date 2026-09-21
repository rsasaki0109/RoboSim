//! Optimizes a Unitree G1 backflip with a compliant contact-implicit model and
//! the direct multiple-shooting solver.
//!
//! Unlike example 113, the contact set is fixed (both toes are always candidate
//! contacts) and a smooth penalty law decides when they push. That removes the
//! contact-set change at the push-to-flight boundary, which is the diagnosed
//! blocker of the hard-contact sequence, and makes the dynamics differentiable
//! everywhere. The penalty stiffness is tunable so the resulting dynamics can be
//! made smooth enough for the Gauss-Seidel sweep.
//!
//! Measured result: at the 30 ms planning step the penalty law cannot represent
//! a backflip without a large unphysical penetration. To hold the robot up the
//! compliant force must reach a few hundred newtons, and with `F = k p^2` that
//! needs `p = sqrt(F/k)`. The physically motivated stiffness (`k` in the 1e5-1e6
//! range) is unstable at 30 ms, and a stable `k` (~1e3) implies 0.15-0.25 m of
//! penetration. The fixed-chart multiple-shooting gap settles around 0.4 and the
//! single-shooting DDP rollout diverges once the feet penetrate, so this probe
//! does not beat the hard-contact multiple shooting (~0.18 gap, no penetration).
//! It also exposed a real bug: `ContactImplicitArticulatedDynamics` integrated
//! the base with the raw body twist instead of through the floating-base chart
//! (now fixed, with a regression test in `rne_oc`).
//!
//! Run with `cargo run --release -p g1_backflip_compliant --example 115_g1_backflip_compliant [--stiffness N] [--damping N] [--exponent N] [--ground M] [--solver ddp|ms] [--gaps] [--gap-weight N] [--substeps N] [--iterations N]`.

use rne_dynamics::{centroidal_momentum, link_motions, ArticulatedModel, ContactSpec};
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_oc::{
    solve, solve_multiple_shooting, ComplementarityContactDynamics, CompliantContactModel,
    ContactImplicitArticulatedDynamics, CostDerivatives, CostModel, DdpConfig, DiscreteDynamics,
    MultipleShootingConfig, PhaseCostSchedule, QuadraticCost, TerminalDerivatives,
};
use rne_robot::{FloatingBase, Transform3};

const G1_URDF: &str = include_str!("../../assets/robots/g1_description/g1_23dof.urdf");
const BASE_ROTATION_X_RAD: f64 = -std::f64::consts::FRAC_PI_2;
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.035);
/// Toe contact, forward of the sole in the foot frame. The push phase uses it
/// instead of the full sole so the ankle can plantarflex and the heel can lift.
const TOE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.09, 0.0, -0.035);
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

/// Wraps a phase cost schedule and adds a Boston Dynamics-style centroidal
/// angular-momentum term during flight.
///
/// During a flight phase the whole-body angular momentum about the center of
/// mass is (to first order) conserved and set by the takeoff impulse. Tracking a
/// desired value keeps the flip generated by internal leg/arm motion instead of
/// an uncontrolled base reaction, which is what makes the aerial phase
/// well-conditioned. The schedule keeps its analytic derivatives; the momentum
/// term contributes a numerical state gradient and no control gradient (the
/// momentum does not depend on the control at a given state).
struct CentroidalMomentumCost<'a> {
    model: &'a ArticulatedModel,
    schedule: PhaseCostSchedule,
    flight_start: usize,
    weight: f64,
    desired_angular_momentum_z: f64,
}

impl CentroidalMomentumCost<'_> {
    fn angular_momentum_z(&self, state: &[f64]) -> f64 {
        let nv = state.len() / 2;
        let (q, qd) = state.split_at(nv);
        match centroidal_momentum(self.model, q, qd) {
            Ok(momentum) => momentum[5],
            Err(_) => 0.0,
        }
    }

    fn momentum_term(&self, node: usize, state: &[f64]) -> f64 {
        if node < self.flight_start {
            return 0.0;
        }
        let error = self.angular_momentum_z(state) - self.desired_angular_momentum_z;
        0.5 * self.weight * error * error
    }
}

impl CostModel for CentroidalMomentumCost<'_> {
    fn running(&self, node: usize, state: &[f64], control: &[f64]) -> f64 {
        self.schedule.running(node, state, control) + self.momentum_term(node, state)
    }

    fn terminal(&self, state: &[f64]) -> f64 {
        self.schedule.terminal(state)
    }

    fn running_derivatives(&self, node: usize, state: &[f64], control: &[f64]) -> CostDerivatives {
        let mut derivatives = self.schedule.running_derivatives(node, state, control);
        if node < self.flight_start {
            return derivatives;
        }
        let nv = state.len() / 2;
        let epsilon = 1.0e-6;
        for index in 0..nv {
            let mut plus = state.to_vec();
            plus[index] += epsilon;
            let mut minus = state.to_vec();
            minus[index] -= epsilon;
            let slope = (self.momentum_term(node, &plus) - self.momentum_term(node, &minus))
                / (2.0 * epsilon);
            derivatives.lx[index] += slope;
        }
        derivatives
    }

    fn terminal_derivatives(&self, state: &[f64]) -> TerminalDerivatives {
        self.schedule.terminal_derivatives(state)
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
    // The push rolls onto the toes so the ankle can extend and launch.
    let toe_contacts: Vec<ContactSpec> = FOOT_LINKS
        .iter()
        .filter_map(|name| {
            model
                .kinematic()
                .link_entity_by_name(name)
                .map(|link| ContactSpec {
                    link,
                    point_local_m: TOE_OFFSET_LOCAL_M,
                })
        })
        .collect();
    assert_eq!(toe_contacts.len(), 2, "expected two toe contacts");
    println!("g1 backflip compliant-contact probe: nv={nv} control_dim={control_dim}");

    let mut contact_model = CompliantContactModel::default();
    let mut horizon = CROUCH_STEPS + PUSH_STEPS + FLIGHT_STEPS;
    let mut max_iterations = 300_usize;
    let mut use_multiple_shooting = true;
    let mut keep_gaps_open = false;
    let mut gap_weight = 0.0_f64;
    let mut substeps = 1_usize;
    let mut use_complementarity = false;
    let mut args = std::env::args().skip(1);
    while let Some(arg) = args.next() {
        let mut value = || args.next().and_then(|v| v.parse::<f64>().ok());
        match arg.as_str() {
            "--stiffness" => {
                contact_model.stiffness_n_m = value().unwrap_or(contact_model.stiffness_n_m)
            }
            "--damping" => {
                contact_model.damping_n_s_m = value().unwrap_or(contact_model.damping_n_s_m)
            }
            "--exponent" => contact_model.penetration_exponent = value().unwrap_or(1.0),
            "--ground" => contact_model.ground_height_m = value().unwrap_or(0.0),
            "--friction" => contact_model.friction = value().unwrap_or(contact_model.friction),
            "--horizon" => horizon = value().map(|v| v as usize).unwrap_or(horizon),
            "--iterations" => {
                max_iterations = value().map(|v| v as usize).unwrap_or(max_iterations)
            }
            "--solver" => use_multiple_shooting = args.next().map(|v| v != "ddp").unwrap_or(true),
            "--gaps" => keep_gaps_open = true,
            "--gap-weight" => gap_weight = value().unwrap_or(gap_weight),
            "--substeps" => substeps = value().map(|v| v as usize).unwrap_or(substeps),
            "--model" => {
                use_complementarity = args.next().map(|v| v == "complementarity").unwrap_or(false)
            }
            _ => {}
        }
    }
    println!(
        "compliant contact: k={} c={} n={} ground={} mu={} horizon={horizon} substeps={substeps}",
        contact_model.stiffness_n_m,
        contact_model.damping_n_s_m,
        contact_model.penetration_exponent,
        contact_model.ground_height_m,
        contact_model.friction,
    );
    enum Model<'a> {
        Compliant(ContactImplicitArticulatedDynamics<'a>),
        Complementarity(ComplementarityContactDynamics<'a>),
    }

    impl DiscreteDynamics for Model<'_> {
        fn state_dim(&self) -> usize {
            match self {
                Model::Compliant(model) => DiscreteDynamics::state_dim(model),
                Model::Complementarity(model) => DiscreteDynamics::state_dim(model),
            }
        }
        fn control_dim(&self) -> usize {
            match self {
                Model::Compliant(model) => DiscreteDynamics::control_dim(model),
                Model::Complementarity(model) => DiscreteDynamics::control_dim(model),
            }
        }
        fn step(&self, state: &[f64], control: &[f64]) -> Result<Vec<f64>, rne_oc::OcError> {
            match self {
                Model::Compliant(model) => DiscreteDynamics::step(model, state, control),
                Model::Complementarity(model) => DiscreteDynamics::step(model, state, control),
            }
        }
    }

    let dynamics = if use_complementarity {
        println!("using hard complementarity contact");
        Model::Complementarity(ComplementarityContactDynamics::new(
            &model,
            STEP_TIME_S,
            toe_contacts.clone(),
            contact_model.friction,
        ))
    } else {
        Model::Compliant(
            ContactImplicitArticulatedDynamics::new(
                &model,
                STEP_TIME_S,
                toe_contacts.clone(),
                contact_model,
            )
            .with_substeps(substeps),
        )
    };

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
    for index in 0..6 {
        terminal_weights[nv + index] = 40.0;
    }
    for (dof, name) in joint_names.iter().enumerate() {
        // The wrist joints have low torque authority, so a hard zero terminal
        // velocity is not achievable in the last step and leaves a residual
        // gap. Ask for a soft stop there and a firm stop on the body.
        terminal_weights[nv + 6 + dof] = if name.contains("wrist") { 5.0 } else { 40.0 };
    }
    let mut terminal = QuadraticCost::new(zero.clone(), vec![0.0; control_dim], terminal_weights);
    let mut terminal_reference = vec![0.0; 2 * nv];
    terminal_reference[1] = BASE_START_Y_M;
    terminal_reference[5] = -2.0 * std::f64::consts::PI;
    for (dof, name) in joint_names.iter().enumerate() {
        terminal_reference[6 + dof] = stand_angle(name);
    }
    terminal.state_reference = terminal_reference;
    let schedule = PhaseCostSchedule { running, terminal };

    // Warm start: a crouch pose held, then a tucked spin through flight.
    let mut crouch = initial.clone();
    crouch[1] = CROUCH_Y_M;
    for (dof, name) in joint_names.iter().enumerate() {
        crouch[6 + dof] = crouch_angle(name);
    }
    // Launch velocity implied by the ballistic arc: the vertical speed needed
    // at takeoff to reach the apex over the flight time.
    let flight_time_s = FLIGHT_STEPS as f64 * STEP_TIME_S;
    let launch_velocity_m_s = 2.0 * (TARGET_APEX_Y_M - BASE_START_Y_M) / flight_time_s;
    let mut states = Vec::with_capacity(horizon + 1);
    for node in 0..=horizon {
        let mut state = if node < CROUCH_STEPS {
            // Ramp from the standing initial pose into the crouch so node 0 is
            // the exact initial state and no single step has to absorb the whole
            // 0.16 m drop (which the dynamics cannot produce).
            let linear = node as f64 / CROUCH_STEPS as f64;
            let t = linear * linear * (3.0 - 2.0 * linear);
            let mut ramp = initial.clone();
            ramp[1] = BASE_START_Y_M + (CROUCH_Y_M - BASE_START_Y_M) * t;
            for (dof, name) in joint_names.iter().enumerate() {
                ramp[6 + dof] = stand_angle(name) + (crouch_angle(name) - stand_angle(name)) * t;
            }
            ramp
        } else if node < CROUCH_STEPS + PUSH_STEPS {
            let push = (node - CROUCH_STEPS) as f64;
            let linear = (push + 1.0) / PUSH_STEPS as f64;
            // Ease-out so the joint velocity reaches zero at the push-to-flight
            // boundary, matching the flight ease-in and keeping the transition
            // continuous.
            let t = linear * linear * (3.0 - 2.0 * linear);
            let mut extended = initial.clone();
            extended[1] = BASE_START_Y_M + 0.05 * linear.sin();
            extended[nv + 1] = launch_velocity_m_s;
            for (dof, name) in joint_names.iter().enumerate() {
                extended[6 + dof] =
                    stand_angle(name) + (crouch_angle(name) - stand_angle(name)) * (1.0 - t);
            }
            extended
        } else {
            let flight = (node - CROUCH_STEPS - PUSH_STEPS) as f64;
            let t = flight / FLIGHT_STEPS as f64;
            let mut tucked = initial.clone();
            tucked[1] = BASE_START_Y_M
                + (TARGET_APEX_Y_M - BASE_START_Y_M) * (std::f64::consts::PI * t).sin();
            tucked[5] = -2.0 * std::f64::consts::PI * t;
            // Match the ballistic arc: the vertical velocity is the derivative
            // of the height profile, so the flight reference is dynamically
            // consistent instead of starting from rest under a rising arc.
            tucked[nv + 1] = (TARGET_APEX_Y_M - BASE_START_Y_M) * std::f64::consts::PI
                / (FLIGHT_STEPS as f64 * STEP_TIME_S)
                * (std::f64::consts::PI * t).cos();
            // The spin is built during flight; do not impose the full rate at
            // the push boundary, where the planted feet cannot supply it.
            tucked[nv + 5] = -2.0 * std::f64::consts::PI * t / (FLIGHT_STEPS as f64 * STEP_TIME_S);
            // Blend the legs from the push extension into the tuck over the
            // first third of flight so the push-to-flight joint velocity is
            // continuous instead of a step.
            let ramp = (t / 0.33).min(1.0);
            // Ease-in so the joint velocity starts at zero and matches the push
            // ease-out at the boundary.
            let blend = ramp * ramp * (3.0 - 2.0 * ramp);
            for (dof, name) in joint_names.iter().enumerate() {
                let start = stand_angle(name);
                tucked[6 + dof] = start + (tuck_angle(name) - start) * blend;
            }
            tucked
        };
        if node < CROUCH_STEPS {
            state[nv + 1] = 0.0;
        }
        states.push(state);
    }
    states[0] = initial.clone();

    if std::env::var("G1_DEBUG_TOE").is_ok() {
        let mut probe = states[0].clone();
        for k in 0..horizon {
            match dynamics.step(&probe, &vec![0.0; control_dim]) {
                Ok(next) => probe = next,
                Err(error) => {
                    println!("DEBUG rollout step {k} error {error}");
                    break;
                }
            }
            if k < 6 || k + 1 == horizon {
                let maximum = probe.iter().fold(0.0_f64, |m, v| m.max(v.abs()));
                println!(
                    "DEBUG rollout k={k} base_y={:.4} max_comp={maximum:.3e} finite={}",
                    probe[1],
                    probe.iter().all(|v| v.is_finite())
                );
            }
        }
    }
    println!("solving multiple shooting...");
    // Target the centroidal angular momentum the warm-start spin already
    // produces at mid-flight, so the aerial phase is shaped toward that flip.
    let flight_axis_momentum = {
        let mid = &states[CROUCH_STEPS + PUSH_STEPS + FLIGHT_STEPS / 2];
        let nv = mid.len() / 2;
        centroidal_momentum(&model, &mid[..nv], &mid[nv..])
            .map(|momentum| momentum[5])
            .unwrap_or(0.0)
    };
    let momentum_weight = std::env::var("G1_MOMENTUM_WEIGHT")
        .ok()
        .and_then(|value| value.parse().ok())
        .unwrap_or(0.0);
    let cost = CentroidalMomentumCost {
        model: &model,
        schedule,
        flight_start: CROUCH_STEPS + PUSH_STEPS,
        weight: momentum_weight,
        desired_angular_momentum_z: flight_axis_momentum,
    };
    println!("momentum: desired L_z={flight_axis_momentum:.3} kg m^2/s weight={momentum_weight}");
    let controls = vec![vec![0.0; control_dim]; horizon];
    let limits: Vec<f64> = joint_names.iter().map(|name| torque_limit(name)).collect();
    let solution = if use_multiple_shooting {
        let config = MultipleShootingConfig {
            max_iterations,
            tolerance: 1.0e-4,
            control_lower: Some(limits.iter().map(|limit| -limit).collect()),
            control_upper: Some(limits.clone()),
            ..MultipleShootingConfig::default()
        };
        solve_multiple_shooting(&dynamics, &cost, &states, &controls, &config).expect("solve")
    } else {
        let config = DdpConfig {
            max_iterations,
            tolerance: 1.0e-7,
            keep_gaps_open,
            gap_weight,
            control_lower: Some(limits.iter().map(|limit| -limit).collect()),
            control_upper: Some(limits.clone()),
            ..DdpConfig::default()
        };
        solve(&dynamics, &cost, &states, &controls, &config).expect("solve")
    };

    let apex_y = solution
        .states
        .iter()
        .fold(f64::MIN, |maximum, state| maximum.max(state[1]));
    let min_y = solution
        .states
        .iter()
        .fold(f64::MAX, |minimum, state| minimum.min(state[1]));
    if std::env::var("G1_DEBUG_TOE").is_ok() {
        for (node, state) in solution.states.iter().enumerate() {
            let (q, qd) = state.split_at(nv);
            if let Ok(motions) = link_motions(&model, q, qd) {
                let mut deepest = f64::MAX;
                for spec in &toe_contacts {
                    if let Some(index) = model.kinematic().link_index(spec.link) {
                        let motion = motions[index];
                        let offset = motion.world_transform.rotation * spec.point_local_m;
                        deepest = deepest.min(motion.world_transform.translation.y + offset.y);
                    }
                }
                println!(
                    "DEBUG node={node} base_y={:.4} toe_y_min={deepest:.5}",
                    state[1]
                );
            }
        }
    }
    // Deepest toe penetration below the ground plane over the whole plan. The
    // compliant law realizes the required contact force by this penetration, so
    // it is the physicality check for the penalty model.
    let mut max_penetration = 0.0_f64;
    for state in &solution.states {
        let (q, qd) = state.split_at(nv);
        if let Ok(motions) = link_motions(&model, q, qd) {
            for spec in &toe_contacts {
                if let Some(index) = model.kinematic().link_index(spec.link) {
                    let motion = motions[index];
                    let offset = motion.world_transform.rotation * spec.point_local_m;
                    let y = motion.world_transform.translation.y + offset.y;
                    max_penetration = max_penetration.max(-y);
                }
            }
        }
    }
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
    let mut worst_node = 0_usize;
    let mut worst_gap = 0.0_f64;
    let mut worst_component = 0_usize;
    for node in 0..horizon {
        match dynamics.step(&solution.states[node], &solution.controls[node]) {
            Ok(predicted) => {
                for (a, b) in solution.states[node + 1].iter().zip(&predicted) {
                    max_gap = max_gap.max((a - b).abs());
                }
                let mut node_gap = 0.0_f64;
                let mut node_gap_index = 0_usize;
                for (index, (a, b)) in solution.states[node + 1].iter().zip(&predicted).enumerate()
                {
                    if (a - b).abs() > node_gap {
                        node_gap = (a - b).abs();
                        node_gap_index = index;
                    }
                }
                if node_gap > worst_gap {
                    worst_gap = node_gap;
                    worst_node = node;
                    worst_component = node_gap_index;
                }
            }
            Err(_) => failed_nodes += 1,
        }
    }
    let worst_label = if worst_component < nv {
        format!("q[{}]", worst_component)
    } else if worst_component < nv + 6 {
        format!("base_qd[{}]", worst_component - nv)
    } else {
        let joint = worst_component - nv - 6;
        format!(
            "qd[{}] ({})",
            joint,
            joint_names.get(joint).map(String::as_str).unwrap_or("?")
        )
    };
    println!(
        "worst gap {worst_gap:.3e} at node {worst_node} component {worst_component} {worst_label} (crouch<{CROUCH_STEPS}, push<{})",
        CROUCH_STEPS + PUSH_STEPS
    );
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
        "cost={:.4} apex_y={apex_y:.3} min_y={min_y:.3} final_y={final_y:.3} jump={:.3} toe_penetration={max_penetration:.3} yaw=[{min_yaw:.2},{max_yaw:.2}] span={:.2} rad max_gap={max_gap:.2e} max_torque={max_torque:.1} iterations={}",
        solution.cost,
        apex_y - BASE_START_Y_M,
        max_yaw - min_yaw,
        solution.iterations,
    );
}

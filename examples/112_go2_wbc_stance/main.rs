//! WBC stance/walk diagnostic for the Unitree Go2.
//!
//! This isolates why whole-body torque control of the Go2 previously diverged.
//! It loads a declared-inertial-mass scene (ruling out scene/URDF mass mismatch)
//! and builds the WBC model from the live simulator world, then reports stance
//! and trot tracking at a configurable plant rate.
//!
//! Measured results (see `docs/PLAN_LEGGED_LOCOMOTION_FRONTIER.md`):
//! - At 60 Hz every task combination collapses, including pure gravity hold.
//! - `rne_dynamics::link_motions` had a missing `omega_body x v_body` term in
//!   the world bias acceleration; fixed and pinned by a finite-difference test.
//!   The earlier "stable 240 Hz posture-only stance" was an artifact of that
//!   bug, so with the corrected bias the WBC stance must be re-derived.
//! - Feeding the measured base velocity still destabilizes the otherwise
//!   zero-base stance, and the CoM/attitude tasks destabilize it too.
//!
//! Knobs: `RNE_WBC_HZ` (plant rate, default 240), `RNE_QD_MODE` (0 zero base,
//! 1 world twist, 2 body twist, 3/4 their negations), `RNE_QD_SCALE`
//! (fraction of the base velocity fed to the WBC), `RNE_WALK_STEPS`,
//! `RNE_STRIDE`, `RNE_LIFT`, `RNE_POSTURE_KP`/`RNE_POSTURE_KD`,
//! `RNE_COM_KP`/`RNE_COM_KD`, `RNE_ATT_KP`/`RNE_ATT_KD`,
//! `RNE_CONTACT_WEIGHT`, `RNE_CONTACT_COMPLIANCE`, `RNE_TORQUE_FILTER`;
//! flags `--walk`,
//! `--stance-contacts`, `--force-com`, `--force-att`, `--hybrid`.
//!
//! Run with `cargo run -p go2_wbc_stance --example 112_go2_wbc_stance`.

use glam::EulerRot;
use rne_ai::{unitree_go2_trot_targets, UnitreeGo2GaitCommand, UrdfSceneSim};
use rne_dynamics::{center_of_mass, ArticulatedModel};
use rne_math::{Hertz, Vec3};
use rne_robot::{FloatingBase, Transform3};
use rne_wbc::{
    BaseAttitudeTask, ComTask, ContactPoint, PostureTask, WholeBodyConfig, WholeBodyController,
};

const SCENE: &str = "../../assets/scenes/unitree_go2_declared.rne.scene.toml";
const FOOT_LINKS: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
const SOLE_OFFSET_LOCAL_M: Vec3 = Vec3::new(0.0, 0.0, -0.02);
const POSITION_STIFFNESS: f64 = 180.0;
const POSITION_DAMPING: f64 = 18.0;
const TORQUE_LIMIT_NM: f64 = 23.7;
const SETTLE_STEPS: u64 = 240;
const RUN_STEPS: u64 = 240;
const WALK_STEPS: u64 = 1200;
const ATTITUDE_GAIN: f64 = -200.0;
const ATTITUDE_RATE_GAIN: f64 = 16.0;

const STAND_THIGH_RAD: f64 = 0.8;
const STAND_CALF_RAD: f64 = -1.5;

struct Model {
    articulated: ArticulatedModel,
    joint_links: Vec<String>,
}

impl Model {
    fn build(sim: &mut UrdfSceneSim) -> Self {
        let world = sim.world_mut();
        let robot = world
            .iter_entities()
            .find_map(|entity| entity.get::<rne_robot::Robot>().map(|_| entity.id()))
            .expect("robot");
        let base = world
            .get::<rne_robot::Robot>(robot)
            .expect("robot")
            .base_link;
        let saved = world.get::<Transform3>(base).copied().unwrap_or_default();
        world
            .entity_mut(base)
            .insert((FloatingBase, Transform3::IDENTITY));
        let articulated =
            ArticulatedModel::from_robot(&*world, robot).expect("floating articulated model");
        world.entity_mut(base).insert(saved);

        let kinematic = articulated.kinematic();
        let joint_links = kinematic
            .movable_joint_entities()
            .iter()
            .map(|joint| {
                let child = kinematic.joint_child_link(*joint).expect("child");
                let index = kinematic.link_index(child).expect("index");
                kinematic.link_name(index).expect("name").to_string()
            })
            .collect();
        Self {
            articulated,
            joint_links,
        }
    }

    fn q(&self, sim: &UrdfSceneSim) -> Vec<f64> {
        let base = sim.named_transform("base").expect("base");
        let (yaw, pitch, roll) = base.rotation.to_euler(EulerRot::ZYX);
        let mut q = vec![0.0; self.articulated.nv()];
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
        // The floating-base generalized velocity convention is selected for the
        // experiment: 0 = zero base, 1 = world-frame twist, 2 = body-frame twist.
        let mode = std::env::var("RNE_QD_MODE")
            .ok()
            .and_then(|value| value.parse::<u32>().ok())
            .unwrap_or(0);
        let observation = sim.observe();
        let base_rotation = sim.named_transform("base").expect("base").rotation;
        let linear_world = Vec3::new(
            observation.base_linear_velocity_x_m_s,
            observation.base_linear_velocity_y_m_s,
            observation.base_linear_velocity_z_m_s,
        );
        let angular_world = Vec3::new(
            observation.base_angular_velocity_x_rad_s,
            observation.base_angular_velocity_y_rad_s,
            observation.base_angular_velocity_z_rad_s,
        );
        let (linear, angular) = match mode {
            1 => (linear_world, angular_world),
            2 => (
                base_rotation.inverse() * linear_world,
                base_rotation.inverse() * angular_world,
            ),
            3 => (
                -(base_rotation.inverse() * linear_world),
                -(base_rotation.inverse() * angular_world),
            ),
            4 => (-linear_world, -angular_world),
            _ => (Vec3::ZERO, Vec3::ZERO),
        };
        let scale = std::env::var("RNE_QD_SCALE")
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .unwrap_or(1.0);
        let mut qd = vec![0.0; self.articulated.nv()];
        qd[0] = scale * linear.x;
        qd[1] = scale * linear.y;
        qd[2] = scale * linear.z;
        qd[3] = scale * angular.x;
        qd[4] = scale * angular.y;
        qd[5] = scale * angular.z;
        for (dof, link) in self.joint_links.iter().enumerate() {
            qd[6 + dof] = sim.named_joint_velocity(link).unwrap_or(0.0);
        }
        qd
    }
}

fn targets() -> [(String, f64); 12] {
    let legs = ["FL", "FR", "RL", "RR"];
    let mut out = Vec::new();
    for leg in legs {
        out.push((format!("{leg}_hip"), 0.0));
        out.push((format!("{leg}_thigh"), STAND_THIGH_RAD));
        out.push((format!("{leg}_calf"), STAND_CALF_RAD));
    }
    out.try_into().expect("twelve targets")
}

fn settle(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let stand = targets();
    for _ in 0..SETTLE_STEPS {
        let targets: Vec<rne_ai::UrdfJointPositionTarget<'_>> = stand
            .iter()
            .map(|(name, position)| rne_ai::UrdfJointPositionTarget {
                link_name: name.as_str(),
                position: *position,
            })
            .collect();
        sim.step_joint_position_targets(&targets);
    }
}

fn run(
    sim: &mut UrdfSceneSim,
    model: &Model,
    with_tasks: bool,
    posture_weight: f64,
    com_weight: f64,
) -> (f64, f64, f64, f64) {
    let hybrid = std::env::args().any(|argument| argument == "--hybrid");
    let hybrid_pd = std::env::args().any(|argument| argument == "--hybrid-pd");
    let walk = std::env::args().any(|argument| argument == "--walk");
    let force_com = std::env::args().any(|argument| argument == "--force-com");
    let force_att = std::env::args().any(|argument| argument == "--force-att");
    let use_com =
        with_tasks && force_com && !std::env::args().any(|argument| argument == "--no-com");
    let use_attitude =
        with_tasks && force_att && !std::env::args().any(|argument| argument == "--no-att");
    let env_f64 = |name: &str, default: f64| {
        std::env::var(name)
            .ok()
            .and_then(|value| value.parse::<f64>().ok())
            .unwrap_or(default)
    };
    let posture_kp = env_f64("RNE_POSTURE_KP", 4.0);
    let posture_kd = env_f64("RNE_POSTURE_KD", 1.0);
    let stance_contacts = std::env::args().any(|argument| argument == "--stance-contacts");
    let walk_command = UnitreeGo2GaitCommand {
        stride_rad: env_f64("RNE_STRIDE", UnitreeGo2GaitCommand::default().stride_rad),
        foot_lift_rad: env_f64("RNE_LIFT", UnitreeGo2GaitCommand::default().foot_lift_rad),
        ..UnitreeGo2GaitCommand::default()
    };
    let com_kp = env_f64("RNE_COM_KP", 6.0);
    let com_kd = env_f64("RNE_COM_KD", 2.0);
    let att_kp = env_f64("RNE_ATT_KP", ATTITUDE_GAIN);
    let att_kd = env_f64("RNE_ATT_KD", ATTITUDE_RATE_GAIN);
    let controller = WholeBodyController::new(WholeBodyConfig {
        com_weight,
        posture_weight,
        angular_weight: 1.0e4,
        contact_weight: env_f64("RNE_CONTACT_WEIGHT", 1.0e6),
        contact_compliance: env_f64("RNE_CONTACT_COMPLIANCE", 0.0),
        dynamics_weight: env_f64("RNE_DYNAMICS_WEIGHT", 1.0e6),
        force_regularization: env_f64("RNE_FORCE_REG", 1.0e-4),
        acceleration_regularization: env_f64("RNE_ACCEL_REG", 1.0e-4),
        solver_regularization: env_f64("RNE_SOLVER_REG", 1.0e-9),
        torque_limits_nm: Some(vec![TORQUE_LIMIT_NM; model.joint_links.len()]),
        ..WholeBodyConfig::default()
    });
    let nominal_com = {
        let q = model.q(sim);
        center_of_mass(&model.articulated, &q).expect("com")
    };
    let up_reference = {
        let pose = sim.named_transform("base").expect("base");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };

    let start_x = sim.observe().base_x_m;
    let mut min_height = f64::MAX;
    let mut max_tilt: f64 = 0.0;
    let iterations = if walk {
        env_f64("RNE_WALK_STEPS", WALK_STEPS as f64) as u64
    } else {
        RUN_STEPS
    };
    let torque_filter = env_f64("RNE_TORQUE_FILTER", 0.0).clamp(0.0, 0.999_999);
    let mut held_torques = vec![0.0; model.joint_links.len()];
    for step in 0..iterations {
        let q = model.q(sim);
        let qd = model.qd(sim);
        let com = center_of_mass(&model.articulated, &q).expect("com");
        if hybrid {
            let position_targets: Vec<rne_ai::UrdfJointPositionTarget<'_>> = model
                .joint_links
                .iter()
                .filter_map(|link| {
                    if link.ends_with("_hip") {
                        Some(rne_ai::UrdfJointPositionTarget {
                            link_name: link.as_str(),
                            position: 0.0,
                        })
                    } else if link.ends_with("_calf") {
                        Some(rne_ai::UrdfJointPositionTarget {
                            link_name: link.as_str(),
                            position: STAND_CALF_RAD,
                        })
                    } else {
                        None
                    }
                })
                .collect();
            sim.set_joint_position_targets(&position_targets);
        }
        let phase = if walk { (step % 90) as f64 / 90.0 } else { 0.0 };
        let contacts: Vec<ContactPoint> = FOOT_LINKS
            .iter()
            .zip([0.0_f64, 0.5, 0.5, 0.0])
            .filter(|(_, offset)| !stance_contacts || (phase + offset).fract() < 0.7)
            .filter_map(|(foot, _)| {
                model
                    .articulated
                    .kinematic()
                    .link_entity_by_name(foot)
                    .map(|link| ContactPoint::new(link, SOLE_OFFSET_LOCAL_M, 0.6))
            })
            .collect();
        let com_task = use_com.then(|| {
            let mut task = ComTask::hold(com);
            task.desired_position_m = nominal_com;
            task.position_gain_s_inv2 = com_kp;
            task.velocity_gain_s_inv = com_kd;
            task
        });
        let posture_desired = if walk {
            let targets = unitree_go2_trot_targets(step, walk_command);
            model
                .joint_links
                .iter()
                .map(|link| {
                    targets
                        .iter()
                        .find(|target| target.link_name == link)
                        .map(|target| target.position)
                        .unwrap_or(0.0)
                })
                .collect()
        } else {
            q[6..].to_vec()
        };
        let posture = PostureTask {
            desired_joint_positions: posture_desired,
            desired_joint_velocities: None,
            desired_joint_accelerations: None,
            position_gain_s_inv2: if with_tasks { posture_kp } else { 0.0 },
            velocity_gain_s_inv: if with_tasks { posture_kd } else { 0.0 },
        };
        let attitude = use_attitude.then(|| {
            let pose = sim.named_transform("base").expect("base");
            let body_up = (pose.rotation * up_reference).normalize_or_zero();
            let axis_world = Vec3::Y.cross(body_up);
            let sin_angle = axis_world.length();
            let tilt = body_up.y.clamp(-1.0, 1.0).acos();
            let axis_body = if sin_angle > 1.0e-6 {
                pose.rotation.inverse() * (axis_world / sin_angle)
            } else {
                Vec3::ZERO
            };
            let observation = sim.observe();
            let omega_body = pose.rotation.inverse()
                * Vec3::new(
                    observation.base_angular_velocity_x_rad_s,
                    observation.base_angular_velocity_y_rad_s,
                    observation.base_angular_velocity_z_rad_s,
                );
            BaseAttitudeTask {
                desired_angular_acceleration_rad_s2: axis_body * (att_kp * tilt)
                    - omega_body * att_kd,
            }
        });

        let solution = controller
            .solve(
                &model.articulated,
                &q,
                &qd,
                &contacts,
                com_task.as_ref(),
                attitude.as_ref(),
                Some(&posture),
            )
            .expect("wbc solve");
        for (index, torque) in solution.joint_torque_nm.iter().enumerate() {
            held_torques[index] =
                torque_filter * held_torques[index] + (1.0 - torque_filter) * torque;
        }
        let torques: Vec<rne_ai::UrdfJointTorqueTarget<'_>> = model
            .joint_links
            .iter()
            .enumerate()
            .filter(|(_, link)| !hybrid || link.ends_with("_thigh"))
            .map(|(index, link)| {
                let supplement = if hybrid_pd && link.ends_with("_thigh") {
                    let dof = model
                        .joint_links
                        .iter()
                        .position(|candidate| candidate == link)
                        .expect("thigh dof");
                    120.0 * (STAND_THIGH_RAD - q[6 + dof]) - 12.0 * qd[6 + dof]
                } else {
                    0.0
                };
                rne_ai::UrdfJointTorqueTarget {
                    link_name: link.as_str(),
                    torque_nm: held_torques[index] + supplement,
                    max_velocity_rad_s: 30.1,
                }
            })
            .collect();
        if std::env::var("RNE_WBC_TRACE").is_ok() && step < 40 {
            let max_torque = solution
                .joint_torque_nm
                .iter()
                .fold(0.0_f64, |value, torque| value.max(torque.abs()));
            let com_accel = solution.com_acceleration_m_s2;
            println!(
                "  step {step:02}: y={:.4} com_az=({:+.4},{:+.4},{:+.4}) v_twist=({:+.4},{:+.4},{:+.4}) tau={max_torque:.2}",
                sim.observe().base_y_m,
                com_accel.x,
                com_accel.y,
                com_accel.z,
                qd[0],
                qd[1],
                qd[2],
            );
        }
        sim.step_joint_torques(&torques);

        let observation = sim.observe();
        min_height = min_height.min(observation.base_y_m);
        let body_up = (sim.named_transform("base").expect("base").rotation * up_reference)
            .normalize_or_zero();
        max_tilt = max_tilt.max(body_up.y.clamp(-1.0, 1.0).acos());
    }
    (
        min_height,
        max_tilt,
        sim.observe().base_y_m,
        sim.observe().base_x_m - start_x,
    )
}

fn make_sim(path: &std::path::Path, hz: f64) -> UrdfSceneSim {
    if (hz - 60.0).abs() < f64::EPSILON {
        UrdfSceneSim::from_scene_path(path).expect("load declared-mass Go2 scene")
    } else {
        UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
            path,
            0,
            rne_core::SimDuration::from_hertz(Hertz::new(hz)),
        )
        .expect("load declared-mass Go2 scene at fixed delta")
    }
}

fn main() {
    let path = std::path::PathBuf::from(env!("CARGO_MANIFEST_DIR")).join(SCENE);
    let hz = std::env::var("RNE_WBC_HZ")
        .ok()
        .and_then(|value| value.parse::<f64>().ok())
        .unwrap_or(240.0);
    let mut sim = make_sim(&path, hz);
    println!("plant rate = {hz:.0} Hz");
    let model = Model::build(&mut sim);
    let mass: f64 = (0..model.articulated.link_count())
        .filter_map(|index| model.articulated.link_inertia(index))
        .map(|inertia| inertia.mass_kg)
        .sum();
    settle(&mut sim);
    println!("declared-scene model mass = {mass:.3} kg");
    let after_settle = sim.observe();
    println!(
        "settled base v=({:.4},{:.4},{:.4}) m/s omega=({:.4},{:.4},{:.4}) rad/s y={:.3}",
        after_settle.base_linear_velocity_x_m_s,
        after_settle.base_linear_velocity_y_m_s,
        after_settle.base_linear_velocity_z_m_s,
        after_settle.base_angular_velocity_x_rad_s,
        after_settle.base_angular_velocity_y_rad_s,
        after_settle.base_angular_velocity_z_rad_s,
        after_settle.base_y_m,
    );

    if std::env::args().any(|argument| argument == "--walk") {
        settle(&mut sim);
        let (min_h, max_tilt, final_y, forward) = run(&mut sim, &model, true, 1.0, 1.0e5);
        println!(
            "walk : forward={forward:.3} minH={min_h:.3} maxTilt={max_tilt:.3} finalY={final_y:.3}"
        );
        return;
    }

    let (min_h, max_tilt, final_y, _forward) = run(&mut sim, &model, true, 1.0, 1.0e5);
    println!("posture: minH={min_h:.3} maxTilt={max_tilt:.3} finalY={final_y:.3}");

    let mut hold_sim = make_sim(&path, hz);
    settle(&mut hold_sim);
    let (min_h, max_tilt, final_y, _forward) = run(&mut hold_sim, &model, false, 1.0, 1.0e5);
    println!("hold : minH={min_h:.3} maxTilt={max_tilt:.3} finalY={final_y:.3}");
}

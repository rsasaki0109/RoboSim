//! Verifies Go2 velocity-command tracking and contact-derived terrain input.
//!
//! The controller starts with the same position-held stand as the pure-torque
//! example. Locomotion then uses one policy input containing the commanded
//! body-forward speed, measured body velocity, and the latest foot-contact
//! elevation observation. The flat and fixed-ramp scenes share the exact same
//! torque policy and actuator limits, making the terrain rollout a headless
//! regression rather than a renderer-only demonstration.

use rne_ai::{
    unitree_go2_dynamic_scene_path, unitree_go2_terrain_scene_path, unitree_go2_trot_targets,
    UnitreeGo2GaitCommand, UnitreeGo2PureTorquePolicy, UnitreeGo2TerrainObservation,
    UnitreeGo2VelocityCommand, UnitreeGo2VelocityPolicyConfig, UnitreeGo2VelocityPolicyInput,
    UrdfJointTorqueTarget, UrdfSceneSim,
};
use rne_math::Vec3;
use std::path::Path;

const JOINTS: [&str; 12] = [
    "FL_hip", "FL_thigh", "FL_calf", "FR_hip", "FR_thigh", "FR_calf", "RL_hip", "RL_thigh",
    "RL_calf", "RR_hip", "RR_thigh", "RR_calf",
];
const FEET: [&str; 4] = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"];
const CYCLE_STEPS: u64 = 45;
const SETTLE_STEPS: u64 = 240;
const ROLLOUT_STEPS: u64 = 1440;
const POSITION_STIFFNESS: f64 = 180.0;
const POSITION_DAMPING: f64 = 18.0;
const TORQUE_LIMIT_NM: f64 = 23.7;
const SPEED_LIMIT_RAD_S: f64 = 30.1;
const COMMAND_FORWARD_M_S: f64 = 0.14;

#[derive(Clone, Copy, Debug)]
struct RolloutOutcome {
    mean_forward_velocity_m_s: f64,
    mean_velocity_error_m_s: f64,
    base_x_displacement_m: f64,
    total_displacement_m: f64,
    min_height_m: f64,
    max_tilt_rad: f64,
    max_terrain_roughness_m: f64,
    max_terrain_slope_rad: f64,
    max_command_nm: f64,
}

fn startup_stand(sim: &mut UrdfSceneSim) {
    sim.configure_position_motors(POSITION_STIFFNESS, POSITION_DAMPING, TORQUE_LIMIT_NM);
    let stand = unitree_go2_trot_targets(
        0,
        UnitreeGo2GaitCommand {
            stride_rad: 0.0,
            foot_lift_rad: 0.0,
            cycle_steps: CYCLE_STEPS,
            ..UnitreeGo2GaitCommand::default()
        },
    );
    for _ in 0..SETTLE_STEPS {
        sim.step_joint_position_targets(&stand);
    }
}

fn joint_state(sim: &UrdfSceneSim) -> ([f64; 12], [f64; 12]) {
    let positions = JOINTS.map(|link| sim.named_joint_position(link).expect("Go2 joint position"));
    let velocities = JOINTS.map(|link| sim.named_joint_velocity(link).expect("Go2 joint velocity"));
    (positions, velocities)
}

fn measured_forward_velocity_m_s(sim: &UrdfSceneSim) -> f64 {
    let observation = sim.observe();
    let world_velocity = Vec3::new(
        observation.base_linear_velocity_x_m_s,
        observation.base_linear_velocity_y_m_s,
        observation.base_linear_velocity_z_m_s,
    );
    let base_rotation = sim.named_transform("base").expect("Go2 base pose").rotation;
    // The vendored Go2 URDF is authored with its learned forward gait along
    // the negative local-X axis after the scene's fixed frame conversion.
    -(base_rotation.inverse() * world_velocity).x
}

fn terrain_observation(
    sim: &UrdfSceneSim,
    previous: UnitreeGo2TerrainObservation,
) -> UnitreeGo2TerrainObservation {
    let mut elevations = [0.0; 4];
    let mut contacts = [false; 4];
    for (index, foot) in FEET.iter().enumerate() {
        contacts[index] = sim.link_contact_impulse_ns(foot) > 0.0;
        elevations[index] = sim
            .named_translation_m(foot)
            .map(|translation| translation.1)
            .unwrap_or(0.0);
    }

    let mut contact_elevations = [0.0; 4];
    let mut contact_count = 0;
    for (elevation, in_contact) in elevations.into_iter().zip(contacts) {
        if in_contact {
            contact_elevations[contact_count] = elevation;
            contact_count += 1;
        }
    }
    if contact_count == 0 {
        return previous;
    }

    let front = mean_contact_elevation(&elevations, &contacts, [0, 1])
        .unwrap_or(previous.front_elevation_m);
    let rear =
        mean_contact_elevation(&elevations, &contacts, [2, 3]).unwrap_or(previous.rear_elevation_m);
    let roughness_m = if contact_count > 1 {
        let active = &contact_elevations[..contact_count];
        active
            .iter()
            .copied()
            .fold(
                (f64::INFINITY, f64::NEG_INFINITY),
                |(minimum, maximum), value| (minimum.min(value), maximum.max(value)),
            )
            .1
            - active.iter().copied().fold(f64::INFINITY, f64::min)
    } else {
        previous.roughness_m
    };
    let slope_rad = if contacts[0] || contacts[1] {
        if contacts[2] || contacts[3] {
            ((front - rear) / 0.45).atan()
        } else {
            previous.slope_rad
        }
    } else {
        previous.slope_rad
    };
    UnitreeGo2TerrainObservation {
        front_elevation_m: front,
        rear_elevation_m: rear,
        roughness_m: roughness_m.max(0.0),
        slope_rad,
    }
}

fn mean_contact_elevation(
    elevations: &[f64; 4],
    contacts: &[bool; 4],
    indices: [usize; 2],
) -> Option<f64> {
    let mut sum = 0.0;
    let mut count = 0;
    for index in indices {
        if contacts[index] {
            sum += elevations[index];
            count += 1;
        }
    }
    (count > 0).then_some(sum / count as f64)
}

fn true_tilt_rad(sim: &UrdfSceneSim, up_body_reference: Vec3) -> f64 {
    let pose = sim.named_transform("base").expect("Go2 base pose");
    let up = (pose.rotation * up_body_reference).normalize_or_zero();
    up.y.clamp(-1.0, 1.0).acos()
}

fn rollout(scene_path: &Path, steps: u64) -> RolloutOutcome {
    let mut sim = UrdfSceneSim::from_scene_path(scene_path).expect("load Go2 scene");
    startup_stand(&mut sim);
    let up_body_reference = {
        let pose = sim.named_transform("base").expect("Go2 base pose");
        (pose.rotation.inverse() * Vec3::Y).normalize_or_zero()
    };
    let start = sim.observe();
    let policy = UnitreeGo2PureTorquePolicy::LEARNED_WALK;
    let config = UnitreeGo2VelocityPolicyConfig::default();
    let command = UnitreeGo2VelocityCommand {
        forward_m_s: COMMAND_FORWARD_M_S,
    };
    let mut terrain = UnitreeGo2TerrainObservation::default();
    let mut velocity_sum = 0.0;
    let mut velocity_error_sum = 0.0;
    let mut velocity_samples = 0_u64;
    let mut min_height_m = f64::MAX;
    let mut max_tilt_rad: f64 = 0.0;
    let mut max_terrain_roughness_m: f64 = 0.0;
    let mut max_terrain_slope_rad: f64 = 0.0;
    let mut max_command_nm: f64 = 0.0;

    for step in 0..steps {
        let (positions, velocities) = joint_state(&sim);
        let measured_velocity_m_s = measured_forward_velocity_m_s(&sim);
        let input = UnitreeGo2VelocityPolicyInput {
            phase: (step % CYCLE_STEPS) as f64 / CYCLE_STEPS as f64,
            joint_positions_rad: positions,
            joint_velocities_rad_s: velocities,
            command,
            measured_forward_velocity_m_s: measured_velocity_m_s,
            terrain,
            config,
        };
        let torques = policy.torques_nm_for_velocity_command(input, TORQUE_LIMIT_NM);
        max_command_nm = max_command_nm.max(
            torques
                .iter()
                .map(|torque| torque.abs())
                .fold(0.0_f64, f64::max),
        );
        let torque_targets: Vec<UrdfJointTorqueTarget<'_>> = JOINTS
            .iter()
            .zip(torques.iter())
            .map(|(link_name, torque_nm)| UrdfJointTorqueTarget {
                link_name,
                torque_nm: *torque_nm,
                max_velocity_rad_s: SPEED_LIMIT_RAD_S,
            })
            .collect();
        sim.step_joint_torques(&torque_targets);

        let observed_velocity_m_s = measured_forward_velocity_m_s(&sim);
        if step >= steps / 3 {
            velocity_sum += observed_velocity_m_s;
            velocity_error_sum += (COMMAND_FORWARD_M_S - observed_velocity_m_s).abs();
            velocity_samples += 1;
        }
        terrain = terrain_observation(&sim, terrain);
        max_terrain_roughness_m = max_terrain_roughness_m.max(terrain.roughness_m);
        max_terrain_slope_rad = max_terrain_slope_rad.max(terrain.slope_rad.abs());
        let observed = sim.observe();
        min_height_m = min_height_m.min(observed.base_y_m);
        max_tilt_rad = max_tilt_rad.max(true_tilt_rad(&sim, up_body_reference));
    }

    let end = sim.observe();
    RolloutOutcome {
        mean_forward_velocity_m_s: velocity_sum / velocity_samples.max(1) as f64,
        mean_velocity_error_m_s: velocity_error_sum / velocity_samples.max(1) as f64,
        base_x_displacement_m: end.base_x_m - start.base_x_m,
        total_displacement_m: (end.base_x_m - start.base_x_m).hypot(end.base_z_m - start.base_z_m),
        min_height_m,
        max_tilt_rad,
        max_terrain_roughness_m,
        max_terrain_slope_rad,
        max_command_nm,
    }
}

fn main() {
    let smoke = std::env::args().any(|argument| argument == "--smoke");
    let steps = if smoke { 720 } else { ROLLOUT_STEPS };
    let flat = rollout(&unitree_go2_dynamic_scene_path(), steps);
    let terrain = rollout(&unitree_go2_terrain_scene_path(), steps);

    for (label, outcome) in [("flat", flat), ("terrain", terrain)] {
        assert!(
            outcome.max_command_nm <= TORQUE_LIMIT_NM + 1.0e-9,
            "{label} command exceeded actuator limit: {:.3} N*m",
            outcome.max_command_nm
        );
        assert!(
            outcome.min_height_m > 0.13 && outcome.max_tilt_rad < 1.5,
            "{label} rollout must stay upright: minH {:.3} tilt {:.3}",
            outcome.min_height_m,
            outcome.max_tilt_rad
        );
    }
    println!(
        "Go2 velocity/terrain rollout: flat v {:.3} dx {:+.3} distance {:.3} rough {:.3}; terrain v {:.3} dx {:+.3} distance {:.3} rough {:.3} slope {:.3} minH {:.3}",
        flat.mean_forward_velocity_m_s,
        flat.base_x_displacement_m,
        flat.total_displacement_m,
        flat.max_terrain_roughness_m,
        terrain.mean_forward_velocity_m_s,
        terrain.base_x_displacement_m,
        terrain.total_displacement_m,
        terrain.max_terrain_roughness_m,
        terrain.max_terrain_slope_rad,
        terrain.min_height_m
    );
    if smoke {
        assert!(
            terrain.max_terrain_roughness_m > 0.01 || terrain.max_terrain_slope_rad > 0.03,
            "terrain smoke must observe a contact terrain change: rough {:.3} m slope {:.3} rad",
            terrain.max_terrain_roughness_m,
            terrain.max_terrain_slope_rad
        );
        println!(
            "Go2 velocity/terrain smoke ok: flat v {:.3} terrain v {:.3} rough {:.3} minH {:.3}",
            flat.mean_forward_velocity_m_s,
            terrain.mean_forward_velocity_m_s,
            terrain.max_terrain_roughness_m,
            terrain.min_height_m
        );
        return;
    }

    assert!(
        flat.mean_forward_velocity_m_s > 0.05
            && flat.mean_velocity_error_m_s < 0.12
            && flat.total_displacement_m > 1.0,
        "flat command must track and transport: v {:.3} err {:.3} distance {:.3}",
        flat.mean_forward_velocity_m_s,
        flat.mean_velocity_error_m_s,
        flat.total_displacement_m
    );
    assert!(
        (terrain.max_terrain_roughness_m > 0.02 || terrain.max_terrain_slope_rad > 0.03)
            && terrain.mean_forward_velocity_m_s > 0.03,
        "terrain command must observe terrain and keep moving: rough {:.3} slope {:.3} v {:.3}",
        terrain.max_terrain_roughness_m,
        terrain.max_terrain_slope_rad,
        terrain.mean_forward_velocity_m_s
    );
    println!(
        "Go2 velocity/terrain policy verified: flat v {:.3} err {:.3} distance {:.3}; terrain v {:.3} rough {:.3} minH {:.3} tilt {:.3}",
        flat.mean_forward_velocity_m_s,
        flat.mean_velocity_error_m_s,
        flat.total_displacement_m,
        terrain.mean_forward_velocity_m_s,
        terrain.max_terrain_roughness_m,
        terrain.min_height_m,
        terrain.max_tilt_rad
    );
}

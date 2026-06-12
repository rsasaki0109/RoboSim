//! Two shared-world agents controlling separate diff-drive robots in one ECS world.

use rne_ai::{
    attach_shared_diff_drive_policy, spawn_shared_diff_drive_agent_for_robot,
    step_shared_diff_drive_agents, AgentKind, ConstantVelocityPolicy, DiffDriveSim,
};
use rne_math::Vec3;
use rne_robot::{DiffDriveConfig, DiffDriveDriveMode};

fn main() {
    let mut sim = DiffDriveSim::with_robot_configs(&[
        DiffDriveConfig {
            model_name: "robot_a".into(),
            initial_translation_m: Vec3::new(0.0, 0.25, -1.0),
            drive_mode: DiffDriveDriveMode::JointDriven,
            ..DiffDriveConfig::default()
        },
        DiffDriveConfig {
            model_name: "robot_b".into(),
            initial_translation_m: Vec3::new(0.0, 0.25, 1.0),
            drive_mode: DiffDriveDriveMode::JointDriven,
            ..DiffDriveConfig::default()
        },
    ]);

    let robot_a = sim.robots()[0].robot;
    let robot_b = sim.robots()[1].robot;

    let agent_a = spawn_shared_diff_drive_agent_for_robot(
        &mut sim,
        "agent_a",
        AgentKind::Policy,
        robot_a,
        Some(1.5),
    );
    let agent_b = spawn_shared_diff_drive_agent_for_robot(
        &mut sim,
        "agent_b",
        AgentKind::Policy,
        robot_b,
        Some(1.0),
    );
    attach_shared_diff_drive_policy(&mut sim, agent_a, ConstantVelocityPolicy::new(6.0));
    attach_shared_diff_drive_policy(&mut sim, agent_b, ConstantVelocityPolicy::new(4.0));

    let mut final_a = 0.0;
    let mut final_b = 0.0;
    for _ in 0..300 {
        step_shared_diff_drive_agents(&mut sim);
        final_a = sim
            .world()
            .get::<rne_ai::SharedDiffDriveAgentState>(agent_a)
            .expect("agent_a")
            .last_observation()
            .base_x_m;
        final_b = sim
            .world()
            .get::<rne_ai::SharedDiffDriveAgentState>(agent_b)
            .expect("agent_b")
            .last_observation()
            .base_x_m;
    }

    println!(
        "multi-robot rollout: steps={}, robot_a_x={:.2} m, robot_b_x={:.2} m",
        sim.step_count(),
        final_a,
        final_b
    );

    if final_a < 0.5 || final_b < 0.5 {
        std::process::exit(1);
    }
}

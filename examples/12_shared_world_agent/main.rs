//! Agent entity in the same ECS world as the diff-drive simulation.

use rne_ai::{
    attach_shared_diff_drive_policy, spawn_shared_diff_drive_agent, step_shared_diff_drive_agent,
    AgentKind, ConstantVelocityPolicy, DiffDriveSim,
};

fn main() {
    let mut sim = DiffDriveSim::new();
    let agent = spawn_shared_diff_drive_agent(&mut sim, "forward_agent", AgentKind::Policy);
    attach_shared_diff_drive_policy(&mut sim, agent, ConstantVelocityPolicy::new(6.0));

    let initial = sim
        .world()
        .get::<rne_ai::SharedDiffDriveAgentState>(agent)
        .expect("agent state")
        .last_observation();
    println!(
        "spawned agent in sim world: robot={:?}, base_x={:.2} m",
        sim.world()
            .get::<rne_ai::AgentTarget>(agent)
            .and_then(|target| target.robot),
        initial.base_x_m
    );

    let mut final_x = initial.base_x_m;
    for _ in 0..300 {
        let obs = step_shared_diff_drive_agent(&mut sim, agent);
        final_x = obs.base_x_m;
    }

    println!(
        "shared-world rollout: steps={}, final_base_x={:.2} m, agent_entity={}",
        sim.step_count(),
        final_x,
        agent.index()
    );

    if final_x < 0.8 {
        std::process::exit(1);
    }
}

//! Agent entity with attachable policy controlling a diff-drive episode.

use rne_ai::{
    attach_diff_drive_policy, reset_diff_drive_agent, spawn_diff_drive_agent,
    step_diff_drive_agent, AgentKind, ConstantVelocityPolicy, DiffDriveEpisodeConfig,
};

fn main() {
    let mut world = rne_ecs::World::new();
    let agent = spawn_diff_drive_agent(
        &mut world,
        "forward_agent",
        DiffDriveEpisodeConfig {
            goal_x_m: 1.5,
            max_steps: 300,
            record_log: true,
            ..DiffDriveEpisodeConfig::default()
        },
        AgentKind::Policy,
    );
    attach_diff_drive_policy(&mut world, agent, ConstantVelocityPolicy::new(6.0));

    let mut step = reset_diff_drive_agent(&mut world, agent);
    println!(
        "reset: base_x={:.2} m, reward={:.3}",
        step.observation.base_x_m, step.reward
    );

    while !step.is_done() {
        step = step_diff_drive_agent(&mut world, agent);
    }

    let state = world
        .get::<rne_ai::DiffDriveAgentState>(agent)
        .expect("agent state");
    println!(
        "done: base_x={:.2} m, reward={:.3}, terminated={}, truncated={}, total_reward={:.3}",
        step.observation.base_x_m,
        step.reward,
        step.terminated,
        step.truncated,
        state.episode().total_reward()
    );
    println!(
        "recorded commands = {}",
        state.episode().log().records().len()
    );

    if !step.terminated {
        std::process::exit(1);
    }
}

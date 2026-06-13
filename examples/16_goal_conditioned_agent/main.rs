//! Goal-conditioned agent with curriculum and multi-task goal sampling.

use rne_ai::{
    attach_goal_conditioned_policy, attach_shared_goal_conditioned_policy, reset_diff_drive_agent,
    spawn_diff_drive_agent, spawn_shared_diff_drive_agent_for_robot, step_diff_drive_agent,
    step_shared_diff_drive_agent, AgentKind, DiffDriveEpisodeConfig, DiffDriveSim,
    GoalCurriculumConfig, GoalSeekingPolicy, GoalTaskSet,
};

fn main() {
    run_episode_agent();
    run_shared_world_agent();
}

fn run_episode_agent() {
    let mut world = rne_ecs::World::new();
    let agent = spawn_diff_drive_agent(
        &mut world,
        "curriculum_agent",
        DiffDriveEpisodeConfig {
            goal_curriculum: Some(GoalCurriculumConfig::easy_to_hard()),
            goal_tasks: Some(GoalTaskSet::forward_training()),
            max_steps: 400,
            rng_seed: 7,
            record_log: true,
            ..DiffDriveEpisodeConfig::default()
        },
        AgentKind::Policy,
    );
    attach_goal_conditioned_policy(&mut world, agent, GoalSeekingPolicy::new(6.0, 0.05));

    let mut successes = 0;
    let mut last_stage = 0;
    for episode in 0..6 {
        let mut step = reset_diff_drive_agent(&mut world, agent);
        let goal_x_m = world
            .get::<rne_ai::DiffDriveAgentState>(agent)
            .expect("agent state")
            .episode()
            .goal_x_m();
        let stage = world
            .get::<rne_ai::DiffDriveAgentState>(agent)
            .expect("agent state")
            .episode()
            .curriculum_stage_index()
            .unwrap_or(0);
        last_stage = stage;

        println!(
            "episode {episode}: goal_x={goal_x_m:.2} m stage={stage} delta={:.2} m",
            step.observation.goal_delta_x_m.unwrap_or(f64::NAN)
        );

        while !step.is_done() {
            step = step_diff_drive_agent(&mut world, agent);
        }

        if step.terminated {
            successes += 1;
        }
    }

    let state = world
        .get::<rne_ai::DiffDriveAgentState>(agent)
        .expect("agent state");
    println!(
        "episode agent: successes={successes}/6 final_stage={last_stage} total_reward={:.3} log_records={}",
        state.episode().total_reward(),
        state.episode().log().records().len()
    );

    if successes == 0 {
        std::process::exit(1);
    }
}

fn run_shared_world_agent() {
    let mut sim = DiffDriveSim::new();
    let robot = sim.robot().robot;
    let agent = spawn_shared_diff_drive_agent_for_robot(
        &mut sim,
        "shared_goal_agent",
        AgentKind::Policy,
        robot,
        Some(1.2),
    );
    attach_shared_goal_conditioned_policy(&mut sim, agent, GoalSeekingPolicy::new(6.0, 0.05));

    let mut final_x = 0.0;
    for _ in 0..300 {
        let obs = step_shared_diff_drive_agent(&mut sim, agent);
        final_x = obs.base_x_m;
        if obs.goal_delta_x_m.is_some_and(|delta| delta.abs() <= 0.05) {
            break;
        }
    }

    println!("shared-world agent: final_x={final_x:.2} m");
    if final_x < 1.15 {
        std::process::exit(1);
    }
}

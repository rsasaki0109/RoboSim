//! Differential drive episode with reward, termination, and optional log recording.

use rne_ai::{ConstantVelocityPolicy, DiffDriveEpisode, DiffDriveEpisodeConfig, Episode, Policy};

fn main() {
    let mut env = DiffDriveEpisode::new(DiffDriveEpisodeConfig {
        goal_x_m: 2.0,
        max_steps: 300,
        record_log: true,
        ..DiffDriveEpisodeConfig::default()
    });
    let mut policy = ConstantVelocityPolicy::new(6.0);

    let mut step = env.reset();
    println!(
        "reset: base_x={:.2} m, reward={:.3}",
        step.observation.base_x_m, step.reward
    );

    while !step.is_done() {
        let action = policy.act(&step.observation);
        step = env.step(action);
    }

    println!(
        "done: base_x={:.2} m, reward={:.3}, terminated={}, truncated={}, total_reward={:.3}",
        step.observation.base_x_m,
        step.reward,
        step.terminated,
        step.truncated,
        env.total_reward()
    );
    println!("recorded commands = {}", env.log().records().len());

    if !step.terminated {
        std::process::exit(1);
    }
}

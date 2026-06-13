//! Parallel diff-drive episodes with domain randomization.

use rne_ai::{
    DiffDriveAction, DiffDriveDomainRandomization, DiffDriveEpisodeConfig,
    VectorizedDiffDriveConfig, VectorizedDiffDriveEnv,
};

fn main() {
    let mut env = VectorizedDiffDriveEnv::new(VectorizedDiffDriveConfig {
        episode: DiffDriveEpisodeConfig {
            max_steps: 400,
            domain_randomization: Some(DiffDriveDomainRandomization {
                initial_x_m: Some((-0.2, 0.2)),
                initial_y_m: None,
                goal_x_m: Some((1.0, 1.8)),
            }),
            rng_seed: 42,
            ..DiffDriveEpisodeConfig::default()
        },
        num_envs: 8,
        auto_reset: true,
    });

    env.reset();
    let goals: Vec<_> = (0..env.num_envs())
        .map(|index| env.episode(index).goal_x_m())
        .collect();
    println!("reset goals = {goals:?}");

    let actions = vec![DiffDriveAction::forward(6.0); env.num_envs()];
    let mut total_reward = 0.0;
    let mut total_successes = 0;
    let mut max_base_x_m = 0.0_f64;

    for _ in 0..500 {
        let step = env.step(&actions);
        total_reward += step.rewards.iter().sum::<f64>();
        total_successes += step.success_count();
        max_base_x_m = max_base_x_m.max(
            step.observations
                .iter()
                .map(|observation| observation.base_x_m)
                .fold(0.0_f64, f64::max),
        );
    }

    println!(
        "vectorized rollout: envs={}, total_reward={:.2}, successes={}, max_base_x={:.2} m",
        env.num_envs(),
        total_reward,
        total_successes,
        max_base_x_m
    );

    if total_successes == 0 && max_base_x_m < 0.3 {
        std::process::exit(1);
    }
}

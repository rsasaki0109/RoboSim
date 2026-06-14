//! Batched mobile manipulator reach episodes for population-based RL rollouts.
//!
//! Evaluates a population of constant shoulder-velocity policies in lock-step with
//! [`VectorizedMobileManipulatorEnv`] and reports how many solve the reach task. This
//! mirrors the diff-drive `10_vectorized_episode` example for the manipulator.

use rne_ai::{
    MobileManipulatorAction, MobileManipulatorEpisodeConfig, VectorizedMobileManipulatorConfig,
    VectorizedMobileManipulatorEnv,
};

/// Candidate constant shoulder velocities (rad/s) — one policy per environment.
const SHOULDER_VELOCITIES_RAD_S: [f64; 8] = [-5.0, -4.0, -3.0, -1.0, 0.0, 1.0, 3.0, 5.0];
const MAX_STEPS: usize = 300;

fn run_population() -> (usize, Vec<f64>) {
    let num_envs = SHOULDER_VELOCITIES_RAD_S.len();
    let mut env = VectorizedMobileManipulatorEnv::new(VectorizedMobileManipulatorConfig::new(
        MobileManipulatorEpisodeConfig::reach(),
        num_envs,
    ));
    env.reset();

    let actions: Vec<MobileManipulatorAction> = SHOULDER_VELOCITIES_RAD_S
        .iter()
        .map(|&shoulder_velocity_rad_s| MobileManipulatorAction {
            shoulder_velocity_rad_s,
            ..MobileManipulatorAction::default()
        })
        .collect();

    let mut solved = vec![false; num_envs];
    for _ in 0..MAX_STEPS {
        let step = env.step(&actions);
        for (index, terminated) in step.terminated.iter().enumerate() {
            solved[index] |= *terminated;
        }
        if step.all_done() {
            break;
        }
    }

    let rewards: Vec<f64> = (0..num_envs)
        .map(|index| env.episode(index).total_reward())
        .collect();
    (solved.iter().filter(|s| **s).count(), rewards)
}

fn main() {
    let smoke = std::env::args().any(|arg| arg == "--smoke");
    let (success_count, rewards) = run_population();

    if smoke {
        // Some policies must solve the reach and some must fail — i.e. the batch
        // actually discriminates between policies.
        let solved_any = success_count > 0;
        let failed_any = success_count < SHOULDER_VELOCITIES_RAD_S.len();
        if solved_any && failed_any {
            println!(
                "vectorized smoke ok: {}/{} reach policies solved",
                success_count,
                SHOULDER_VELOCITIES_RAD_S.len()
            );
            return;
        }
        eprintln!(
            "smoke failed: expected a mix of solved/failed policies, got {success_count} solved"
        );
        std::process::exit(1);
    }

    println!(
        "vectorized reach: {}/{} solved",
        success_count,
        SHOULDER_VELOCITIES_RAD_S.len()
    );
    for (velocity, reward) in SHOULDER_VELOCITIES_RAD_S.iter().zip(&rewards) {
        println!("  shoulder={velocity:+.1} rad/s -> reward={reward:.3}");
    }
}

//! Run the frozen v1 protocol; stdout is the complete JSON evidence artifact.

#[cfg(not(feature = "mujoco"))]
fn main() -> anyhow::Result<()> {
    anyhow::bail!("the frozen protocol requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn main() -> anyhow::Result<()> {
    experiment::run()
}

#[cfg(feature = "mujoco")]
mod experiment {
    use anyhow::{ensure, Result};
    use rne_ai::derive_episode_seed;
    use rne_mobility_benchmark::observed_fixed_batch::{SensorFixedBatch, SensorTableLearner};
    use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
    use rne_physics_rapier::RapierBackend;
    use serde::Serialize;
    use std::{collections::BTreeSet, time::Instant};

    #[derive(Debug, Serialize)]
    struct LaneResult {
        lane_id: usize,
        physical_seed: u64,
        noise_seed: u64,
        completed_steps: usize,
        horizon_completed: bool,
        return_value: f64,
        tracking_error_integral_m: f64,
        execution_error: Option<String>,
    }

    fn rapier() -> Result<(RapierBackend, PhysicsBackendManifest)> {
        Ok((RapierBackend::new(), RapierBackend::manifest()))
    }

    fn evaluate<B, F>(factory: F, learner: &SensorTableLearner, pi: bool) -> Result<Vec<LaneResult>>
    where
        B: PhysicsBackend,
        F: Fn() -> Result<(B, PhysicsBackendManifest)>,
    {
        let mut batch = SensorFixedBatch::new_with_noise_root(factory, 4_310_001, 5_310_001, 8, 2)?;
        let mut integral_m = [0.0; 8];
        let mut results: Vec<_> = (0..8)
            .map(|lane| LaneResult {
                lane_id: lane,
                physical_seed: derive_episode_seed(4_310_001, lane as u64, 0),
                noise_seed: derive_episode_seed(5_310_001, lane as u64, 0),
                completed_steps: 0,
                horizon_completed: false,
                return_value: 0.0,
                tracking_error_integral_m: 0.0,
                execution_error: None,
            })
            .collect();
        for decision in 0..330 {
            let actions = batch
                .observations()
                .into_iter()
                .enumerate()
                .map(|(lane, observation)| {
                    let observation = observation?;
                    if !pi {
                        return learner.action(&observation.actor_tensors(), lane, decision, false);
                    }
                    let Some(latest) = observation.latest else {
                        return Ok(0.0);
                    };
                    if latest.target_velocity_m_s == 0.0 {
                        integral_m[lane] = 0.0;
                        return Ok(0.0);
                    }
                    let error =
                        latest.target_velocity_m_s - latest.estimate.estimated_linear_velocity_m_s;
                    let candidate = (integral_m[lane] + error * 0.01).clamp(-2.0, 2.0);
                    let voltage = 15.0 * error + 40.0 * candidate;
                    let clamped = voltage.clamp(-24.0, 24.0);
                    if clamped == voltage || error.signum() != voltage.signum() {
                        integral_m[lane] = candidate;
                    }
                    Ok(clamped)
                })
                .collect::<Result<Vec<_>>>()?;
            let output = batch.step_learning(&actions)?;
            for transition in output.transitions {
                let lane = &mut results[transition.lane_id];
                lane.completed_steps += 1;
                lane.horizon_completed = transition.truncated;
                lane.return_value += transition.reward;
                lane.tracking_error_integral_m += -transition.reward - 0.001;
            }
            let failed = !output.failures.is_empty();
            for failure in output.failures {
                results[failure.lane_id].execution_error = Some(failure.error);
            }
            if failed {
                break;
            }
        }
        Ok(results)
    }

    pub fn run() -> Result<()> {
        let revision = std::env::var("RNE_EXPERIMENT_REVISION")?;
        ensure!(
            !revision.trim().is_empty(),
            "record the actual code revision"
        );
        // Check actual generated seeds rather than assuming distinct roots suffice.
        let mut training = BTreeSet::new();
        for root in [1_310_001, 2_310_001] {
            for episode in 0..32 {
                for lane in 0..4 {
                    ensure!(
                        training.insert(derive_episode_seed(root, lane, episode)),
                        "training seed collision"
                    );
                }
            }
        }
        let mut held_out = BTreeSet::new();
        for root in [4_310_001, 5_310_001] {
            for lane in 0..8 {
                let seed = derive_episode_seed(root, lane, 0);
                ensure!(
                    !training.contains(&seed) && held_out.insert(seed),
                    "held-out seed collision"
                );
            }
        }
        let start = Instant::now();
        let mut batch = SensorFixedBatch::new_with_noise_root(rapier, 1_310_001, 2_310_001, 4, 2)?;
        let mut learner = SensorTableLearner::new(3_310_001);
        let mut training_returns = Vec::new();
        for episode in 0..32 {
            if episode != 0 {
                batch.reset_lanes(&(0..4).map(|lane| (lane, episode)).collect::<Vec<_>>())?;
            }
            let mut returns = [0.0; 4];
            for step in 0..330 {
                let output = learner.train_step(&mut batch, episode * 330 + step)?;
                ensure!(
                    output.failures.is_empty(),
                    "training aborted, no held-out evaluation: {:?}",
                    output.failures
                );
                for transition in output.transitions {
                    returns[transition.lane_id] += transition.reward;
                }
            }
            training_returns.push(returns);
        }
        let training_wall_s = start.elapsed().as_secs_f64();
        ensure!(learner.updates() == 42_240, "incomplete training");
        let frozen = learner.clone();
        let untrained = SensorTableLearner::new(3_310_001);
        let mujoco = || {
            Ok((
                rne_physics_mujoco::MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                    1_000_000,
                ))?,
                rne_physics_mujoco::MuJoCoBackend::manifest(),
            ))
        };
        let evaluations = serde_json::json!({
            "rapier": {
                "trained": evaluate(rapier, &learner, false)?,
                "untrained": evaluate(rapier, &untrained, false)?,
                "pi_15_40": evaluate(rapier, &untrained, true)?,
            },
            "mujoco": {
                "trained": evaluate(mujoco, &learner, false)?,
                "untrained": evaluate(mujoco, &untrained, false)?,
                "pi_15_40": evaluate(mujoco, &untrained, true)?,
            },
        });
        ensure!(
            learner == frozen && untrained.updates() == 0,
            "evaluation mutated policy"
        );
        println!(
            "{}",
            serde_json::to_string_pretty(&serde_json::json!({
                "schema_version": 1, "protocol": "MOBILITY_REFERENCE_LEARNING_EXPERIMENT.md/v1",
                "revision": revision, "training_backend": "rapier", "training_returns": training_returns,
                "training_physical_root": 1_310_001, "training_noise_root": 2_310_001,
                "exploration_root": 3_310_001, "held_out_physical_root": 4_310_001,
                "held_out_noise_root": 5_310_001, "seed_disjointness_checked": true,
                "training_updates": learner.updates(), "training_wall_s": training_wall_s,
                "training_updates_per_s": learner.updates() as f64 / training_wall_s,
                "training_timing_includes_construction_and_resets": true,
                "evaluations": evaluations,
            }))?
        );
        Ok(())
    }
}

//! Privileged SI-unit evaluation of fixed-period voltage histories and policies.
//!
//! History comparison supplies identical open-loop commands to both backends.
//! Policy evaluation instead queries sensor-only observations before every step.
//! Neither path establishes physical calibration or attested policy identity.

use super::*;

/// Complete fixed-horizon evaluator result, never an actor observation.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FixedVoltageEvaluation {
    /// Backend identity used for the physical rollout.
    pub backend: PhysicsBackendManifest,
    /// Explicit physical/sensor reset seed (sensor noise remains seed zero).
    pub episode_seed: u64,
    /// Exact applied reset contract, not nominal estimator calibration.
    pub reset_contract: SensorObservedContract,
    /// Fixed-period task whose horizon and reward were evaluated.
    pub task_spec: TaskSpec,
    /// All 330 commands, including the settling interval, in temporal order.
    pub actions_v: Vec<f64>,
    /// Completed physical time, in nanosecond ticks.
    pub time_ticks: u64,
    /// Sum of ten-tick right-end speed-error integrals over the entire episode.
    pub tracking_error_integral_m: f64,
    /// True speed at the physical horizon, not the last sensor capture.
    pub final_velocity_m_s: f64,
    /// Inclusive final-speed acceptance against 1 m/s, using the existing 0.1 m/s gate.
    pub final_velocity_accepted: bool,
    /// Completed rigid-body/joint hash, useful only for within-backend replay.
    pub privileged_final_physics_hash_v2: u64,
}

/// Evaluates exactly 330 finite, bounded commands from a fresh seeded world.
/// All commands are checked before backend world creation. Solver/sensor failures
/// return errors; horizon truncation does not imply final-speed acceptance.
pub fn evaluate_fixed_voltage_history<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    episode_seed: u64,
    actions_v: &[f64],
) -> Result<FixedVoltageEvaluation> {
    ensure!(
        actions_v.len() == 330,
        "fixed evaluation requires 330 commands"
    );
    ensure!(
        actions_v.iter().all(|action| action.is_finite()
            && (-MAXIMUM_VOLTAGE_V..=MAXIMUM_VOLTAGE_V).contains(action)),
        "fixed evaluation command outside voltage bounds"
    );
    let mut index = 0;
    evaluate_fixed_sensor_policy(backend, manifest, episode_seed, |_| {
        let action = actions_v[index];
        index += 1;
        Ok(action)
    })
}

/// Runs a fresh world with 330 fallible sensor-only decisions at 10 ms intervals.
/// The callback receives reset observation at time zero, then each prior transition's
/// exact actor snapshot (including freshness), never reward or physical truth.
/// Missing/stale observation handling and policy state are caller-owned. Callback
/// errors or invalid voltages abort execution without a fabricated success report.
/// Captured actions can be passed to [`evaluate_fixed_voltage_history`] for replay.
/// This boundary cannot attest that a callback ignores external privileged state.
pub fn evaluate_fixed_sensor_policy<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    episode_seed: u64,
    mut policy: impl FnMut(&SensorFixedObservation) -> Result<f64>,
) -> Result<FixedVoltageEvaluation> {
    let mut environment = SensorFixedEnvironment::new(backend, manifest.clone(), episode_seed)?;
    let mut observation = environment.observation()?;
    let mut actions_v = Vec::with_capacity(330);
    let mut integral_m = 0.0;
    for index in 0..330 {
        let action = policy(&observation)
            .with_context(|| format!("fixed sensor policy decision {index}"))?;
        let transition = environment.step(action)?;
        ensure!(
            transition.truncated == (index == 329),
            "fixed horizon mismatch"
        );
        integral_m += transition.privileged_tracking_error_integral_m;
        actions_v.push(action);
        observation = transition.observation;
    }
    let velocity_m_s = environment
        .runtime
        .world
        .get::<RigidBody>(environment.runtime.vehicle)
        .context("fixed evaluation final body missing")?
        .linear_velocity_m_s
        .x;
    ensure!(
        integral_m.is_finite() && velocity_m_s.is_finite(),
        "non-finite fixed evaluation"
    );
    Ok(FixedVoltageEvaluation {
        backend: manifest,
        episode_seed,
        reset_contract: environment.reset_contract().clone(),
        task_spec: sensor_fixed_task_spec(),
        actions_v,
        time_ticks: environment.observation()?.time_ticks,
        tracking_error_integral_m: integral_m,
        final_velocity_m_s: velocity_m_s,
        final_velocity_accepted: (velocity_m_s - TARGET_VELOCITY_M_S).abs() <= 0.1,
        privileged_final_physics_hash_v2: environment.privileged_physics_hash_v2()?,
    })
}

/// Reference sensor-only PI rollout with the existing nominal gains (15 V s/m,
/// 20 V/m), anti-windup and +/-24 V limits. A fresh controller is created per call.
/// Missing estimates command zero and clear integral state; stale estimates are
/// held and integrated over the actual 10 ms control period. Target comes from
/// the current boundary clock, not a retained estimate's older target timestamp.
/// This is a baseline, not a safety controller or a guarantee of task acceptance.
pub fn evaluate_fixed_reference_policy<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    episode_seed: u64,
) -> Result<FixedVoltageEvaluation> {
    let mut controller = VelocityController::new(&SensorObservedContract::nominal());
    evaluate_fixed_sensor_policy(backend, manifest, episode_seed, |observation| {
        let Some(latest) = &observation.latest else {
            return Ok(controller.update(0.0, 0.0, 0.01));
        };
        Ok(controller.update(
            latest.estimate.estimated_linear_velocity_m_s,
            fixed_target_velocity(observation.time_ticks),
            0.01,
        ))
    })
}

/// Freshly executed comparison; matching backend outputs do not imply task success.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FixedVoltageComparison {
    /// First backend's complete evaluator evidence.
    pub first: FixedVoltageEvaluation,
    /// Second backend's complete evaluator evidence.
    pub second: FixedVoltageEvaluation,
    /// Absolute gaps with explicit engineering regression tolerances.
    pub gaps: Vec<MobilityBenchmarkMetric>,
    /// All gaps passed, independently of final-speed acceptance.
    pub within_tolerance: bool,
    /// Both final-speed gates passed; not a calibration or complete-task verdict.
    pub both_final_velocities_accepted: bool,
}

/// Executes both backends under the same reset and command history. No caller-
/// supplied verdict or deserialized report is trusted as execution evidence.
/// Tolerances (0.10 m accumulated error and 0.10 m/s final speed) are regression
/// budgets, not empirically identified real-vehicle accuracy bounds.
pub fn compare_fixed_voltage_history<A: PhysicsBackend, B: PhysicsBackend>(
    first: (A, PhysicsBackendManifest),
    second: (B, PhysicsBackendManifest),
    episode_seed: u64,
    actions_v: &[f64],
) -> Result<FixedVoltageComparison> {
    let first = evaluate_fixed_voltage_history(first.0, first.1, episode_seed, actions_v)?;
    let second = evaluate_fixed_voltage_history(second.0, second.1, episode_seed, actions_v)?;
    compare_executed_evaluations(first, second)
}

/// Executes a fresh copy of the reference PI policy on each backend. Feedback
/// commands may differ because observations differ; this is not open-loop input
/// matching. Uses the same SI gap budgets as voltage-history comparison and
/// retains final-speed failures independently of agreement between backends.
pub fn compare_fixed_reference_policy<A: PhysicsBackend, B: PhysicsBackend>(
    first: (A, PhysicsBackendManifest),
    second: (B, PhysicsBackendManifest),
    episode_seed: u64,
) -> Result<FixedVoltageComparison> {
    compare_executed_evaluations(
        evaluate_fixed_reference_policy(first.0, first.1, episode_seed)?,
        evaluate_fixed_reference_policy(second.0, second.1, episode_seed)?,
    )
}

fn compare_executed_evaluations(
    first: FixedVoltageEvaluation,
    second: FixedVoltageEvaluation,
) -> Result<FixedVoltageComparison> {
    ensure!(
        first.reset_contract == second.reset_contract,
        "reset contract mismatch"
    );
    let gaps = vec![
        metric(
            "tracking_error_integral_gap_m",
            "m",
            (first.tracking_error_integral_m - second.tracking_error_integral_m).abs(),
            0.0,
            0.10,
        ),
        metric(
            "final_velocity_gap_m_s",
            "m/s",
            (first.final_velocity_m_s - second.final_velocity_m_s).abs(),
            0.0,
            0.10,
        ),
    ];
    Ok(FixedVoltageComparison {
        within_tolerance: gaps.iter().all(|gap| gap.passed),
        both_final_velocities_accepted: first.final_velocity_accepted
            && second.final_velocity_accepted,
        first,
        second,
        gaps,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_physics_rapier::RapierBackend;

    fn rapier() -> (RapierBackend, PhysicsBackendManifest) {
        (RapierBackend::new(), RapierBackend::manifest())
    }

    #[test]
    fn fixed_policy_receives_causal_snapshots_and_replays_exactly() {
        let mut decisions = 0_u64;
        let mut fresh = 0;
        let mut stale = 0;
        let (backend, manifest) = rapier();
        let report = evaluate_fixed_sensor_policy(backend, manifest, 42, |observation| {
            assert_eq!(observation.time_ticks, decisions * 10_000_000);
            if decisions == 0 {
                assert!(observation.latest.is_none());
                assert!(!observation.fresh_estimate);
            } else {
                let latest = observation.latest.as_ref().unwrap();
                assert!(observation.capture_ticks.unwrap() <= latest.decision_ticks);
                assert!(latest.decision_ticks <= observation.time_ticks);
                if observation.fresh_estimate {
                    fresh += 1;
                } else {
                    stale += 1;
                }
            }
            decisions += 1;
            Ok(if observation.time_ticks < 300_000_000 {
                0.0
            } else {
                3.0
            })
        })
        .unwrap();
        assert_eq!(decisions, 330);
        assert!(fresh > 0 && stale > 0);
        let (backend, manifest) = rapier();
        assert_eq!(
            report,
            evaluate_fixed_voltage_history(backend, manifest, 42, &report.actions_v).unwrap()
        );
    }

    #[test]
    fn fixed_policy_errors_and_invalid_actions_stop_at_the_failing_decision() {
        for invalid in [f64::NAN, f64::INFINITY, -24.1, 24.1] {
            let mut calls = 0;
            let (backend, manifest) = rapier();
            let result = evaluate_fixed_sensor_policy(backend, manifest, 42, |_| {
                calls += 1;
                Ok(if calls == 4 { invalid } else { 0.0 })
            });
            assert!(result.is_err());
            assert_eq!(calls, 4);
        }
        let (backend, manifest) = rapier();
        let error = evaluate_fixed_sensor_policy(backend, manifest, 42, |_| {
            anyhow::bail!("policy failure")
        })
        .unwrap_err();
        assert!(format!("{error:#}").contains("policy failure"));
    }

    #[test]
    fn fixed_reference_policy_has_reset_state_and_exact_voltage_replay() {
        let (backend, manifest) = rapier();
        let report = evaluate_fixed_reference_policy(backend, manifest, 42).unwrap();
        assert!(report.actions_v[..30].iter().all(|action| *action == 0.0));
        let (backend, manifest) = rapier();
        assert_eq!(
            report,
            evaluate_fixed_reference_policy(backend, manifest, 42).unwrap()
        );
        let (backend, manifest) = rapier();
        assert_eq!(
            report,
            evaluate_fixed_voltage_history(backend, manifest, 42, &report.actions_v).unwrap()
        );
        println!(
            "fixed reference {}",
            serde_json::to_string(&report).unwrap()
        );
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn fixed_reference_policy_mujoco_replays_its_own_feedback_commands() {
        use rne_physics_mujoco::MuJoCoBackend;
        for seed in [42, 43, 44] {
            let backend =
                MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))
                    .unwrap();
            let report =
                evaluate_fixed_reference_policy(backend, MuJoCoBackend::manifest(), seed).unwrap();
            let backend =
                MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))
                    .unwrap();
            assert_eq!(
                report,
                evaluate_fixed_voltage_history(
                    backend,
                    MuJoCoBackend::manifest(),
                    seed,
                    &report.actions_v
                )
                .unwrap()
            );
            println!(
                "fixed reference {}",
                serde_json::to_string(&report).unwrap()
            );
        }
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn fixed_reference_policy_cross_backend_comparison_retains_failures() {
        use rne_physics_mujoco::MuJoCoBackend;
        let mut failures = 0;
        for seed in [42, 43, 44] {
            let backend =
                MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))
                    .unwrap();
            let comparison = compare_fixed_reference_policy(
                rapier(),
                (backend, MuJoCoBackend::manifest()),
                seed,
            )
            .unwrap();
            assert!(comparison.within_tolerance, "{:?}", comparison.gaps);
            failures += usize::from(!comparison.both_final_velocities_accepted);
            println!(
                "fixed feedback comparison {}",
                serde_json::to_string(&comparison).unwrap()
            );
        }
        assert!(
            failures > 0,
            "baseline failures must not disappear behind backend agreement"
        );
    }

    #[test]
    fn fixed_evaluation_retains_failed_speed_gate_and_exact_replay() {
        let comparison =
            compare_fixed_voltage_history(rapier(), rapier(), 42, &[0.0; 330]).unwrap();
        assert_eq!(comparison.first, comparison.second);
        assert_eq!(comparison.first.time_ticks, 3_300_000_000);
        assert!(comparison.within_tolerance);
        assert!(!comparison.both_final_velocities_accepted);
        assert!(comparison.first.tracking_error_integral_m > 1.0);
        let json = serde_json::to_vec(&comparison).unwrap();
        assert_eq!(
            serde_json::from_slice::<FixedVoltageComparison>(&json).unwrap(),
            comparison
        );
    }

    #[test]
    fn fixed_evaluation_rejects_incomplete_and_invalid_command_histories() {
        for actions in [
            vec![0.0; 329],
            vec![0.0; 331],
            vec![f64::NAN; 330],
            vec![24.1; 330],
        ] {
            let (backend, manifest) = rapier();
            assert!(evaluate_fixed_voltage_history(backend, manifest, 42, &actions).is_err());
        }
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn fixed_evaluation_cross_backend_si_regression() {
        use rne_physics_mujoco::MuJoCoBackend;
        let mut actions = vec![0.0; 30];
        actions.extend(vec![3.0; 300]);
        for seed in [42, 43, 44] {
            let comparison = compare_fixed_voltage_history(
                rapier(),
                (
                    MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))
                        .unwrap(),
                    MuJoCoBackend::manifest(),
                ),
                seed,
                &actions,
            )
            .unwrap();
            println!("{}", serde_json::to_string(&comparison).unwrap());
            assert!(comparison.within_tolerance, "{:?}", comparison.gaps);
            // A constant voltage is a plant comparison, not a tuned speed controller.
            assert_eq!(
                comparison.both_final_velocities_accepted,
                comparison.first.final_velocity_accepted
                    && comparison.second.final_velocity_accepted
            );
        }
    }
}

//! Privileged SI-unit evaluation of a complete fixed-period voltage history.
//!
//! This is an open-loop plant comparison, not a claim of closed-loop policy parity
//! or physical calibration. Both backends receive exactly the same voltage history.

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
    let mut environment = SensorFixedEnvironment::new(backend, manifest.clone(), episode_seed)?;
    let mut integral_m = 0.0;
    for (index, action) in actions_v.iter().enumerate() {
        let transition = environment.step(*action)?;
        ensure!(
            transition.truncated == (index == 329),
            "fixed horizon mismatch"
        );
        integral_m += transition.privileged_tracking_error_integral_m;
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
        actions_v: actions_v.to_vec(),
        time_ticks: environment.observation()?.time_ticks,
        tracking_error_integral_m: integral_m,
        final_velocity_m_s: velocity_m_s,
        final_velocity_accepted: (velocity_m_s - TARGET_VELOCITY_M_S).abs() <= 0.1,
        privileged_final_physics_hash_v2: environment.privileged_physics_hash_v2()?,
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

//! Privileged SI-unit evaluation of fixed-period voltage histories and policies.
//!
//! History comparison supplies identical open-loop commands to both backends.
//! Policy evaluation instead queries sensor-only observations before every step.
//! Neither path establishes physical calibration or attested policy identity.

use super::*;

/// Bounded PI parameter selection with disjoint training and evaluation seeds.
pub mod tuning;

/// Evaluator-only snapshot at a completed 10 ms boundary. Wheel velocity is the
/// completed drive state, while the estimate may describe older sensor captures.
/// Differences are diagnostic accounting terms, not identified causal effects.
#[derive(Clone, Debug, PartialEq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct FixedVelocityDiagnostic {
    /// Completed simulation boundary in nanosecond ticks.
    pub time_ticks: u64,
    /// Voltage held during the interval that just ended (not the next action).
    pub interval_command_voltage_v: f64,
    /// Target at this boundary; the reward uses interval-start targets instead.
    pub target_velocity_m_s: f64,
    /// Latest sensor-derived speed, missing when no estimate has been accepted.
    pub estimated_velocity_m_s: Option<f64>,
    /// Original decision timestamp of the retained sensor estimate.
    pub estimate_decision_ticks: Option<u64>,
    /// Oldest input capture used by the retained estimate.
    pub capture_ticks: Option<u64>,
    /// Age of those inputs at this boundary, not at their original decision.
    pub input_age_ticks: Option<u64>,
    /// True carrier forward speed at this boundary.
    pub true_velocity_m_s: f64,
    /// True carrier speed change over the completed 10 ms interval, divided by 0.01 s.
    pub mean_acceleration_m_s2: f64,
    /// Completed wheel angular speed multiplied by the physical reset radius.
    pub physical_wheel_surface_velocity_m_s: f64,
    /// Same angular speed multiplied by the fixed nominal estimator radius.
    pub nominal_wheel_surface_velocity_m_s: f64,
}

/// Algebraic decomposition of target-minus-true speed at one boundary.
/// Terms sum to the total within floating-point tolerance. The measurement term
/// includes latency, quantization, filtering and calibration effects; the wheel
/// term is peripheral-minus-carrier speed, not a normalized tire slip ratio.
#[derive(Clone, Debug, PartialEq)]
pub struct FixedVelocityErrorBudget {
    /// Target minus retained sensor estimate: controller-visible tracking error.
    pub controller_error_m_s: f64,
    /// Retained estimate minus current nominal-radius wheel surface speed.
    pub measurement_history_residual_m_s: f64,
    /// Nominal-radius minus physical-radius wheel surface speed.
    pub radius_scale_difference_m_s: f64,
    /// Physical wheel surface speed minus carrier speed.
    pub wheel_carrier_difference_m_s: f64,
}

impl FixedVelocityDiagnostic {
    /// Decomposes speed error without inserting physical truth into actor inputs.
    /// Returns `None` when the sensor estimate is missing, never a fabricated zero.
    pub fn error_budget(&self) -> Option<FixedVelocityErrorBudget> {
        let estimate = self.estimated_velocity_m_s?;
        Some(FixedVelocityErrorBudget {
            controller_error_m_s: self.target_velocity_m_s - estimate,
            measurement_history_residual_m_s: estimate - self.nominal_wheel_surface_velocity_m_s,
            radius_scale_difference_m_s: self.nominal_wheel_surface_velocity_m_s
                - self.physical_wheel_surface_velocity_m_s,
            wheel_carrier_difference_m_s: self.physical_wheel_surface_velocity_m_s
                - self.true_velocity_m_s,
        })
    }
}

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
    /// All 330 completed-boundary diagnostics, separate from callback observations.
    pub privileged_velocity_diagnostics: Vec<FixedVelocityDiagnostic>,
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
    let mut diagnostics = Vec::with_capacity(330);
    let mut integral_m = 0.0;
    for index in 0..330 {
        let prior_velocity_m_s = environment
            .runtime
            .world
            .get::<RigidBody>(environment.runtime.vehicle)
            .context("fixed diagnostic body missing")?
            .linear_velocity_m_s
            .x;
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
        let velocity_m_s = environment
            .runtime
            .world
            .get::<RigidBody>(environment.runtime.vehicle)
            .context("fixed diagnostic body missing")?
            .linear_velocity_m_s
            .x;
        let wheel_velocity_rad_s = environment.runtime.drive_state.wheel_velocity_rad_s;
        let diagnostic = FixedVelocityDiagnostic {
            time_ticks: observation.time_ticks,
            interval_command_voltage_v: action,
            target_velocity_m_s: fixed_target_velocity(observation.time_ticks),
            estimated_velocity_m_s: observation
                .latest
                .as_ref()
                .map(|latest| latest.estimate.estimated_linear_velocity_m_s),
            estimate_decision_ticks: observation
                .latest
                .as_ref()
                .map(|latest| latest.decision_ticks),
            capture_ticks: observation.capture_ticks,
            input_age_ticks: observation.input_age_ticks,
            true_velocity_m_s: velocity_m_s,
            mean_acceleration_m_s2: (velocity_m_s - prior_velocity_m_s) / 0.01,
            physical_wheel_surface_velocity_m_s: wheel_velocity_rad_s
                * environment.runtime.plant.wheel.radius_m,
            nominal_wheel_surface_velocity_m_s: wheel_velocity_rad_s * plant_spec().wheel.radius_m,
        };
        ensure!(
            [
                diagnostic.true_velocity_m_s,
                diagnostic.mean_acceleration_m_s2,
                diagnostic.physical_wheel_surface_velocity_m_s,
                diagnostic.nominal_wheel_surface_velocity_m_s,
            ]
            .iter()
            .all(|value| value.is_finite())
                && diagnostic.estimated_velocity_m_s.is_none_or(f64::is_finite),
            "non-finite fixed diagnostic"
        );
        diagnostics.push(diagnostic);
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
        privileged_velocity_diagnostics: diagnostics,
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
    evaluate_fixed_pi_policy(backend, manifest, episode_seed, 15.0, 20.0)
}

/// Evaluates finite PI gains in [0, 100] with unchanged nominal calibration,
/// anti-windup, integral bounds, voltage limits, missing/stale handling and horizon.
/// Gains are policy parameters, never changes to the physical reset contract.
pub fn evaluate_fixed_pi_policy<B: PhysicsBackend>(
    backend: B,
    manifest: PhysicsBackendManifest,
    episode_seed: u64,
    kp_v_s_m: f64,
    ki_v_m: f64,
) -> Result<FixedVoltageEvaluation> {
    tuning::PiGains { kp_v_s_m, ki_v_m }.validate()?;
    let mut controller = VelocityController {
        kp_v_s_m,
        ki_v_m,
        integral_error_m: 0.0,
    };
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
    fn fixed_diagnostics_preserve_timing_and_account_for_total_speed_error() {
        let (backend, manifest) = rapier();
        let report = evaluate_fixed_reference_policy(backend, manifest, 42).unwrap();
        assert_eq!(report.privileged_velocity_diagnostics.len(), 330);
        let mut previous_velocity = 0.0;
        for (index, diagnostic) in report.privileged_velocity_diagnostics.iter().enumerate() {
            assert_eq!(diagnostic.time_ticks, (index as u64 + 1) * 10_000_000);
            assert_eq!(
                diagnostic.interval_command_voltage_v,
                report.actions_v[index]
            );
            assert!(
                diagnostic.capture_ticks.unwrap() <= diagnostic.estimate_decision_ticks.unwrap()
            );
            assert!(diagnostic.estimate_decision_ticks.unwrap() <= diagnostic.time_ticks);
            assert_eq!(
                diagnostic.input_age_ticks,
                Some(diagnostic.time_ticks - diagnostic.capture_ticks.unwrap())
            );
            assert!(
                (diagnostic.mean_acceleration_m_s2 * 0.01
                    - (diagnostic.true_velocity_m_s - previous_velocity))
                    .abs()
                    < 1e-12
            );
            previous_velocity = diagnostic.true_velocity_m_s;
            let budget = diagnostic.error_budget().unwrap();
            let sum = budget.controller_error_m_s
                + budget.measurement_history_residual_m_s
                + budget.radius_scale_difference_m_s
                + budget.wheel_carrier_difference_m_s;
            assert!(
                (sum - (diagnostic.target_velocity_m_s - diagnostic.true_velocity_m_s)).abs()
                    < 1e-12
            );
        }
        let terminal = report.privileged_velocity_diagnostics.last().unwrap();
        assert_eq!(terminal.true_velocity_m_s, report.final_velocity_m_s);
        let mut missing = terminal.clone();
        missing.estimated_velocity_m_s = None;
        assert!(missing.error_budget().is_none());
        // Current boundary target changes at 0.3 s, independently of old sensor targets.
        assert_eq!(
            report.privileged_velocity_diagnostics[28].target_velocity_m_s,
            0.0
        );
        assert_eq!(
            report.privileged_velocity_diagnostics[29].target_velocity_m_s,
            1.0
        );
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

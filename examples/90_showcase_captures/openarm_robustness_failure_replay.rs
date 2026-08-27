//! Converts the first OpenArm robustness-boundary failure into a portable replay.

use anyhow::{bail, Context, Result};
use rne_ai::{
    BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction, BehaviorReplayArtifact,
    BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation,
};
use serde::Deserialize;
use serde_json::{json, Value};
use sha2::{Digest, Sha256};
use std::fs;
use std::path::PathBuf;

const JOINT_INDEX: usize = 4;

#[derive(Debug, Deserialize)]
struct RapierTrace {
    kind: String,
    schema_version: u32,
    backend_id: String,
    controller_id: String,
    fixed_delta_ticks: u64,
    initial_state_digest: u64,
    observations: Vec<TraceObservation>,
}

#[derive(Debug, Deserialize)]
struct TraceObservation {
    step: u64,
    sim_time_ticks: u64,
    joint_position_rad: Vec<f64>,
    joint_velocity_rad_s: Vec<f64>,
    joint_reference_position_rad: Vec<f64>,
    joint_controller_observation_position_rad: Vec<f64>,
    joint_measurement_bias_rad: Vec<f64>,
    joint_controller_target_rad: Vec<f64>,
    joint_actuator_disturbance_rad: Vec<f64>,
    joint_position_target_rad: Vec<f64>,
    #[serde(default)]
    sensor_sample_published: bool,
    #[serde(default)]
    controller_observation_sequence: Option<u64>,
    #[serde(default)]
    controller_observation_age_ticks: Option<u64>,
    #[serde(default)]
    controller_rejected: bool,
    #[serde(default)]
    controller_rejection_reason: Option<String>,
    #[serde(default)]
    fail_safe_hold_active: bool,
    #[serde(default)]
    controller_state_frozen: bool,
    #[serde(default)]
    controller_recovered: bool,
    physics_hash: u64,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("OpenArm robustness failure replay failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let mut report_path = None;
    let mut trace_path = None;
    let mut output_path = None;
    let mut arguments = std::env::args().skip(1);
    while let Some(argument) = arguments.next() {
        match argument.as_str() {
            "--report" => report_path = Some(required_path(&mut arguments, "--report")?),
            "--trace" => trace_path = Some(required_path(&mut arguments, "--trace")?),
            "--output" => output_path = Some(required_path(&mut arguments, "--output")?),
            other => bail!("unknown argument {other:?}"),
        }
    }
    let report_path = report_path.context("--report is required")?;
    let trace_path = trace_path.context("--trace is required")?;
    let output_path = output_path.context("--output is required")?;
    let report_bytes =
        fs::read(&report_path).with_context(|| format!("read {}", report_path.display()))?;
    let trace_bytes =
        fs::read(&trace_path).with_context(|| format!("read {}", trace_path.display()))?;
    let report: Value = serde_json::from_slice(&report_bytes)
        .with_context(|| format!("parse {}", report_path.display()))?;
    let trace: RapierTrace = serde_json::from_slice(&trace_bytes)
        .with_context(|| format!("parse {}", trace_path.display()))?;
    validate_inputs(&report, &trace, &sha256(&trace_bytes))?;
    let failure = &report["first_failure"];
    let failure_step = required_u64(failure, "step")?;
    let failure_index = usize::try_from(failure_step - 1)?;
    let failure_observation = trace
        .observations
        .get(failure_index)
        .context("robustness failure step exceeds the Rapier trace")?;
    let requirement_id = required_str(failure, "requirement_id")?;
    let descriptor = BehaviorContractDescriptor {
        name: requirement_id.to_string(),
        kind: BehaviorContractKind::Always,
        entities: vec!["rne_rapier".to_string(), "openarm_right_joint5".to_string()],
    };
    let report_sha256 = sha256(&report_bytes);
    let trace_sha256 = sha256(&trace_bytes);
    let dimension_id = required_str(&report, "dimension_id")?;
    let availability_failure = dimension_id == "joint_feedback_publication_dropout";
    let latency_failure = dimension_id == "joint_feedback_controller_ingress_latency";
    let jitter_failure = dimension_id == "joint_feedback_controller_ingress_jitter";
    let stale_age_failure = dimension_id == "joint_feedback_controller_stale_age";
    let recovery_failure = dimension_id == "joint_feedback_dropout_recovery";
    let rearm_failure = dimension_id == "joint_feedback_repeated_dropout_rearm";
    let quantization_failure = dimension_id == "joint_position_measurement_quantization";
    let saturation_failure = dimension_id == "joint_position_measurement_saturation";
    let command_delay_failure = dimension_id == "actuator_command_delay";
    let command_rate_limit_failure = dimension_id == "actuator_command_rate_limit";
    let command_deadband_failure = dimension_id == "actuator_command_deadband";
    let disturbance_start_step = match dimension_id {
        "actuator_target_bias"
        | "actuator_command_delay"
        | "actuator_command_rate_limit"
        | "actuator_command_deadband" => required_u64(&report["dimension"], "start_step")?,
        "joint_position_measurement_bias" => {
            required_u64(&report["dimension"], "start_controller_step")?
        }
        "joint_position_measurement_quantization" | "joint_position_measurement_saturation" => {
            required_u64(&report["dimension"], "start_controller_step")?
        }
        "joint_feedback_publication_dropout"
        | "joint_feedback_dropout_recovery"
        | "joint_feedback_repeated_dropout_rearm" => {
            required_u64(&report["dimension"], "start_capture_sequence")?
        }
        "joint_feedback_controller_ingress_latency" => 1,
        "joint_feedback_controller_ingress_jitter" => {
            required_u64(&report["dimension"], "start_capture_sequence")?
        }
        "joint_feedback_controller_stale_age" => {
            required_u64(&report["dimension"], "start_controller_step")?
        }
        _ => bail!("unsupported robustness dimension"),
    };
    let requirement_limit = required_f64(
        failure,
        if command_rate_limit_failure || rearm_failure || saturation_failure {
            "minimum"
        } else {
            "maximum"
        },
    )?;
    let sample_period_s = trace.fixed_delta_ticks as f64 / 1_000_000_000.0;
    let mut cumulative_iae_rad_s = 0.0;
    let mut frames = Vec::with_capacity(failure_index + 2);
    frames.push(BehaviorReplayFrame {
        step: 0,
        sim_time_ticks: 0,
        action: BehaviorReplayAction::InitialObservation,
        observation: json!({
            "backend_id": trace.backend_id,
            "controller_id": trace.controller_id,
            "report_sha256": report_sha256,
            "trace_sha256": trace_sha256,
            "case_id": required_str(failure, "case_id")?,
            "dimension_value": dimension_value(failure)?,
            "dimension_unit": required_str(&report["dimension"], "unit")?,
            "requirement_id": requirement_id,
            "classification": format!("smallest_failed_{dimension_id}_grid_case"),
            "contract_status": "initial"
        }),
        state_digest: trace.initial_state_digest,
    });
    let mut consecutive_dropout_frames = 0_u64;
    let mut recovery_decisions = 0_u64;
    let mut recovery_started = false;
    for (observation_index, observation) in trace.observations[..=failure_index].iter().enumerate()
    {
        if availability_failure {
            if observation.step >= disturbance_start_step && !observation.sensor_sample_published {
                consecutive_dropout_frames += 1;
            } else if observation.step >= disturbance_start_step {
                consecutive_dropout_frames = 0;
            }
        } else if recovery_failure {
            if observation.controller_rejection_reason
                == Some("maximum_observation_age_ticks".to_string())
            {
                recovery_started = true;
                recovery_decisions = 0;
            } else if recovery_started {
                recovery_decisions += 1;
            }
        } else if !latency_failure
            && !jitter_failure
            && !stale_age_failure
            && !recovery_failure
            && !rearm_failure
            && !quantization_failure
            && !saturation_failure
            && !command_delay_failure
            && !command_rate_limit_failure
            && !command_deadband_failure
            && observation.step >= disturbance_start_step
        {
            cumulative_iae_rad_s += (observation.joint_position_rad[JOINT_INDEX]
                - observation.joint_reference_position_rad[JOINT_INDEX])
                .abs()
                * sample_period_s;
        }
        let failed = observation.step == failure_step;
        let delay_steps = command_delay_failure
            .then(|| dimension_value(failure))
            .transpose()?;
        let source_step = delay_steps
            .map(|delay| observation.step.saturating_sub(delay as u64))
            .filter(|_| observation.step >= disturbance_start_step);
        let expected_source_target_rad = source_step
            .and_then(|step| step.checked_sub(1))
            .and_then(|index| usize::try_from(index).ok())
            .and_then(|index| trace.observations.get(index))
            .map(|source| source.joint_controller_target_rad[JOINT_INDEX]);
        let source_relationship_delta_rad = expected_source_target_rad
            .map(|expected| (observation.joint_position_target_rad[JOINT_INDEX] - expected).abs());
        let previous_applied_target_rad = (command_rate_limit_failure || command_deadband_failure)
            .then(|| observation_index.checked_sub(1))
            .flatten()
            .and_then(|index| trace.observations.get(index))
            .map(|previous| previous.joint_position_target_rad[JOINT_INDEX])
            .filter(|_| observation.step >= disturbance_start_step);
        let maximum_delta_rad = command_rate_limit_failure
            .then(|| dimension_value(failure).map(|rate| rate * sample_period_s))
            .transpose()?;
        let expected_rate_limited_target_rad = previous_applied_target_rad
            .zip(maximum_delta_rad)
            .map(|(previous, maximum_delta)| {
                observation.joint_controller_target_rad[JOINT_INDEX]
                    .clamp(previous - maximum_delta, previous + maximum_delta)
            });
        let rate_relationship_delta_rad = expected_rate_limited_target_rad
            .map(|expected| (observation.joint_position_target_rad[JOINT_INDEX] - expected).abs());
        let deadband_rad = command_deadband_failure
            .then(|| dimension_value(failure))
            .transpose()?;
        let expected_deadband_target_rad =
            previous_applied_target_rad
                .zip(deadband_rad)
                .map(|(previous, deadband)| {
                    let commanded = observation.joint_controller_target_rad[JOINT_INDEX];
                    if (commanded - previous).abs() <= deadband {
                        previous
                    } else {
                        commanded
                    }
                });
        let deadband_relationship_delta_rad = expected_deadband_target_rad
            .map(|expected| (observation.joint_position_target_rad[JOINT_INDEX] - expected).abs());
        frames.push(BehaviorReplayFrame {
            step: observation.step,
            sim_time_ticks: observation.sim_time_ticks,
            action: BehaviorReplayAction::Advance,
            observation: json!({
                "joint5_reference_rad": observation.joint_reference_position_rad[JOINT_INDEX],
                "joint5_position_rad": observation.joint_position_rad[JOINT_INDEX],
                "joint5_velocity_rad_s": observation.joint_velocity_rad_s[JOINT_INDEX],
                "joint5_controller_observation_rad": observation.joint_controller_observation_position_rad.get(JOINT_INDEX).copied(),
                "joint5_measurement_bias_rad": observation.joint_measurement_bias_rad[JOINT_INDEX],
                "joint5_controller_target_rad": observation.joint_controller_target_rad[JOINT_INDEX],
                "joint5_disturbance_rad": observation.joint_actuator_disturbance_rad[JOINT_INDEX],
                "joint5_applied_target_rad": observation.joint_position_target_rad[JOINT_INDEX],
                "sensor_sample_published": observation.sensor_sample_published,
                "controller_observation_sequence": observation.controller_observation_sequence,
                "controller_observation_age_ticks": observation.controller_observation_age_ticks,
                "controller_rejected": observation.controller_rejected,
                "controller_rejection_reason": observation.controller_rejection_reason,
                "fail_safe_hold_active": observation.fail_safe_hold_active,
                "controller_state_frozen": observation.controller_state_frozen,
                "controller_recovered": observation.controller_recovered,
                "cumulative_iae_rad_s": (!availability_failure && !latency_failure && !jitter_failure && !stale_age_failure && !recovery_failure && !rearm_failure && !quantization_failure && !saturation_failure && !command_delay_failure && !command_rate_limit_failure && !command_deadband_failure).then_some(cumulative_iae_rad_s),
                "consecutive_dropout_frames": availability_failure.then_some(consecutive_dropout_frames),
                "controller_ingress_delay_frames": latency_failure.then(|| dimension_value(failure)).transpose()?,
                "controller_ingress_jitter_frames": jitter_failure.then(|| observation.controller_observation_age_ticks.map(|age| age / trace.fixed_delta_ticks - 1)).flatten(),
                "controller_selected_stale_frames": stale_age_failure.then(|| observation.controller_observation_age_ticks.map(|age| age / trace.fixed_delta_ticks - 1)).flatten(),
                "sensor_recovery_decisions": recovery_failure.then_some(recovery_decisions),
                "interburst_fresh_frames": rearm_failure.then(|| dimension_value(failure)).transpose()?,
                "sensor_quantization_step_rad": quantization_failure.then(|| dimension_value(failure)).transpose()?,
                "sensor_saturation_limit_abs_rad": saturation_failure.then(|| dimension_value(failure)).transpose()?,
                "actuator_delay_steps": delay_steps,
                "actuator_source_step": source_step,
                "expected_source_controller_target_rad": expected_source_target_rad,
                "source_relationship_delta_rad": source_relationship_delta_rad,
                "actuator_maximum_rate_rad_s": command_rate_limit_failure.then(|| dimension_value(failure)).transpose()?,
                "previous_applied_target_rad": previous_applied_target_rad,
                "maximum_delta_rad": maximum_delta_rad,
                "expected_rate_limited_target_rad": expected_rate_limited_target_rad,
                "rate_relationship_delta_rad": rate_relationship_delta_rad,
                "actuator_deadband_rad": deadband_rad,
                "expected_deadband_target_rad": expected_deadband_target_rad,
                "deadband_relationship_delta_rad": deadband_relationship_delta_rad,
                "maximum": (!command_rate_limit_failure && !rearm_failure && !saturation_failure).then_some(requirement_limit),
                "minimum": (command_rate_limit_failure || rearm_failure || saturation_failure).then_some(requirement_limit),
                "requirement_id": requirement_id,
                "contract_status": if failed { "failed" } else { "pending" }
            }),
            state_digest: observation.physics_hash,
        });
    }
    let observed = required_f64(failure, "observed")?;
    let message = if availability_failure {
        anyhow::ensure!(
            consecutive_dropout_frames as f64 == observed,
            "replayed consecutive dropout count differs from the report"
        );
        format!(
            "OpenArm joint-feedback publication reached {observed:.0} consecutive dropped frames at step {failure_step}, exceeding the fixed {requirement_limit:.0}-frame requirement"
        )
    } else if latency_failure {
        let delay_frames = observed as u64;
        let expected_sequence = failure_step
            .checked_sub(delay_frames + 2)
            .context("latency failure has no delayed source observation")?;
        let expected_age_ticks = (delay_frames + 1) * trace.fixed_delta_ticks;
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && failure_observation.controller_observation_sequence == Some(expected_sequence)
                && failure_observation.controller_observation_age_ticks == Some(expected_age_ticks),
            "replayed controller-ingress latency differs from the report"
        );
        format!(
            "OpenArm joint-feedback controller ingress reached {observed:.0} additional control period at step {failure_step}, preserving capture sequence {expected_sequence} and exceeding the fixed {requirement_limit:.0}-period requirement"
        )
    } else if jitter_failure {
        let expected_sequence = required_u64(failure, "controller_observation_sequence")?;
        let expected_age_ticks = (observed as u64 + 1) * trace.fixed_delta_ticks;
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && failure_observation.controller_observation_sequence == Some(expected_sequence)
                && failure_observation.controller_observation_age_ticks == Some(expected_age_ticks),
            "replayed controller-ingress jitter differs from the report"
        );
        format!(
            "OpenArm joint-feedback controller ingress jitter reached {observed:.0} control periods at step {failure_step}, preserving capture sequence {expected_sequence} and exceeding the fixed {requirement_limit:.0}-period requirement"
        )
    } else if stale_age_failure {
        let expected_sequence = required_u64(failure, "controller_observation_sequence")?;
        let observed_age_ticks = observed as u64;
        let selected_stale_frames = observed_age_ticks / trace.fixed_delta_ticks - 1;
        anyhow::ensure!(
            selected_stale_frames as f64 == dimension_value(failure)?
                && failure_observation.controller_observation_sequence == Some(expected_sequence)
                && failure_observation.controller_observation_age_ticks == Some(observed_age_ticks),
            "replayed controller stale-age selection differs from the report"
        );
        format!(
            "OpenArm controller selected capture sequence {expected_sequence} with age {observed_age_ticks} ticks at step {failure_step}, exceeding the fixed {requirement_limit:.0}-tick stale-observation limit"
        )
    } else if quantization_failure {
        let sequence = failure_observation
            .controller_observation_sequence
            .context("quantization failure has no controller observation")?;
        let raw =
            trace.observations[usize::try_from(sequence - 1)?].joint_position_rad[JOINT_INDEX];
        let expected = (raw / observed).round() * observed;
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && failure_step == disturbance_start_step
                && (failure_observation.joint_controller_observation_position_rad[JOINT_INDEX]
                    - expected)
                    .abs()
                    <= 1e-12,
            "replayed measurement quantization differs from the report"
        );
        format!(
            "OpenArm joint 5 position quantization reached {observed:.6} rad at step {failure_step}, exceeding the fixed {requirement_limit:.6} rad resolution requirement"
        )
    } else if saturation_failure {
        let sequence = failure_observation
            .controller_observation_sequence
            .context("saturation failure has no controller observation")?;
        let raw =
            trace.observations[usize::try_from(sequence - 1)?].joint_position_rad[JOINT_INDEX];
        let expected = raw.clamp(-observed, observed);
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && sequence == required_u64(failure, "controller_observation_sequence")?
                && (raw - required_f64(failure, "raw_position_rad")?).abs() <= 1e-12
                && (expected - required_f64(failure, "saturated_position_rad")?).abs() <= 1e-12
                && (failure_observation.joint_controller_observation_position_rad[JOINT_INDEX]
                    - expected)
                    .abs()
                    <= 1e-12,
            "replayed measurement saturation differs from the report"
        );
        format!(
            "OpenArm joint 5 position saturation limit reached {observed:.6} rad at step {failure_step}, below the fixed {requirement_limit:.6} rad minimum measurement range"
        )
    } else if rearm_failure {
        let burst_length = required_u64(&report["dimension"], "burst_length_frames")?;
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && observed == 0.0
                && failure_step == disturbance_start_step + burst_length
                && !failure_observation.sensor_sample_published,
            "replayed repeated-dropout re-arm boundary differs from the report"
        );
        format!(
            "OpenArm repeated joint-feedback dropout provided {observed:.0} fresh frames between bursts at step {failure_step}, below the fixed {requirement_limit:.0}-frame re-arm minimum"
        )
    } else if recovery_failure {
        anyhow::ensure!(
            recovery_decisions as f64 == observed
                && failure_observation.controller_recovered
                && failure_observation.controller_rejection_reason.is_none(),
            "replayed controller recovery timing differs from the report"
        );
        format!(
            "OpenArm joint-feedback controller recovered after {observed:.0} decisions at step {failure_step}, exceeding the fixed {requirement_limit:.0}-decision recovery requirement"
        )
    } else if command_delay_failure {
        let source_step = required_u64(failure, "source_step")?;
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && source_step == failure_step - observed as u64
                && failure_observation.joint_position_target_rad[JOINT_INDEX]
                    == trace.observations[usize::try_from(source_step - 1)?]
                        .joint_controller_target_rad[JOINT_INDEX],
            "replayed actuator command source differs from the report"
        );
        format!(
            "OpenArm joint 5 command transport reached {observed:.0} control periods at step {failure_step}, selecting source step {source_step} and exceeding the fixed {requirement_limit:.0}-period requirement"
        )
    } else if command_rate_limit_failure {
        let previous = trace
            .observations
            .get(
                failure_index
                    .checked_sub(1)
                    .context("rate-limit failure has no predecessor")?,
            )
            .context("rate-limit predecessor is absent")?
            .joint_position_target_rad[JOINT_INDEX];
        let maximum_delta_rad = observed * sample_period_s;
        let expected = failure_observation.joint_controller_target_rad[JOINT_INDEX]
            .clamp(previous - maximum_delta_rad, previous + maximum_delta_rad);
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && (failure_observation.joint_position_target_rad[JOINT_INDEX] - expected).abs()
                    <= 1e-14,
            "replayed actuator command rate-limit relationship differs from the report"
        );
        format!(
            "OpenArm joint 5 command slew was limited to {observed:.6} rad/s at step {failure_step}, below the fixed {requirement_limit:.6} rad/s minimum"
        )
    } else if command_deadband_failure {
        let previous = trace
            .observations
            .get(
                failure_index
                    .checked_sub(1)
                    .context("deadband failure has no predecessor")?,
            )
            .context("deadband predecessor is absent")?
            .joint_position_target_rad[JOINT_INDEX];
        let commanded = failure_observation.joint_controller_target_rad[JOINT_INDEX];
        let expected = if (commanded - previous).abs() <= observed {
            previous
        } else {
            commanded
        };
        anyhow::ensure!(
            observed == dimension_value(failure)?
                && (failure_observation.joint_position_target_rad[JOINT_INDEX] - expected).abs()
                    <= 1e-14,
            "replayed actuator command deadband relationship differs from the report"
        );
        format!(
            "OpenArm joint 5 command deadband reached {observed:.6} rad at step {failure_step}, exceeding the fixed {requirement_limit:.6} rad maximum"
        )
    } else {
        anyhow::ensure!(
            (cumulative_iae_rad_s - observed).abs() <= 1e-12,
            "replayed cumulative IAE differs from the report"
        );
        format!(
            "OpenArm joint 5 cumulative disturbance IAE reached {observed:.9} rad*s at step {failure_step}, exceeding the fixed {requirement_limit:.9} rad*s requirement under a {:.6} rad {dimension_id}",
            dimension_value(failure)?
        )
    };
    let scenario = required_str(&report, "suite_id")?.to_string();
    let replay = BehaviorReplayArtifact::new(
        scenario,
        digest_u64(&report_bytes),
        20260824,
        trace.fixed_delta_ticks,
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor.clone(),
            violation: BehaviorViolation {
                step: failure_step,
                sim_time_ticks: failure_observation.sim_time_ticks,
                state_digest: failure_observation.physics_hash,
                entities: descriptor.entities.clone(),
                message,
            },
        },
    )?;
    replay.write_json(&output_path)?;
    println!(
        "OpenArm robustness failure replay: requirement={requirement_id} step={failure_step} report_sha256={report_sha256}"
    );
    Ok(())
}

fn validate_inputs(report: &Value, trace: &RapierTrace, trace_sha256: &str) -> Result<()> {
    anyhow::ensure!(
        report["kind"] == "rne_openarm_robustness_report"
            && report["schema_version"] == 1
            && report["status"] == "passed",
        "report is not a supported robustness report"
    );
    let failure = &report["first_failure"];
    anyhow::ensure!(
        matches!(
            required_str(report, "dimension_id")?,
            "actuator_target_bias"
                | "actuator_command_delay"
                | "actuator_command_rate_limit"
                | "actuator_command_deadband"
                | "joint_position_measurement_bias"
                | "joint_feedback_publication_dropout"
                | "joint_feedback_controller_ingress_latency"
                | "joint_feedback_controller_ingress_jitter"
                | "joint_feedback_controller_stale_age"
                | "joint_feedback_dropout_recovery"
                | "joint_feedback_repeated_dropout_rearm"
                | "joint_position_measurement_quantization"
                | "joint_position_measurement_saturation"
        ),
        "unsupported robustness dimension"
    );
    let dimension_id = required_str(report, "dimension_id")?;
    let expected_requirement = match dimension_id {
        "joint_feedback_publication_dropout" => {
            "controller.sensor.maximum_consecutive_dropout_frames"
        }
        "joint_feedback_controller_ingress_latency" => {
            "controller.sensor.maximum_controller_ingress_delay_frames"
        }
        "joint_feedback_controller_ingress_jitter" => {
            "controller.sensor.maximum_controller_ingress_jitter_frames"
        }
        "joint_feedback_controller_stale_age" => "controller.sensor.maximum_observation_age_ticks",
        "joint_feedback_dropout_recovery" => "controller.sensor.maximum_recovery_decisions",
        "joint_feedback_repeated_dropout_rearm" => {
            "controller.sensor.minimum_interburst_fresh_frames"
        }
        "joint_position_measurement_quantization" => {
            "controller.sensor.maximum_position_quantization_step_rad"
        }
        "joint_position_measurement_saturation" => {
            "controller.sensor.minimum_position_saturation_limit_abs_rad"
        }
        "actuator_command_delay" => "controller.actuator.maximum_command_transport_delay_steps",
        "actuator_command_rate_limit" => "controller.actuator.minimum_command_slew_rate_rad_s",
        "actuator_command_deadband" => "controller.actuator.maximum_command_deadband_rad",
        _ => "controller.state.maximum_disturbance_iae_rad_s",
    };
    anyhow::ensure!(
        required_str(failure, "backend_id")? == "rne_rapier"
            && required_str(failure, "requirement_id")? == expected_requirement
            && if matches!(
                dimension_id,
                "actuator_command_rate_limit"
                    | "joint_feedback_repeated_dropout_rearm"
                    | "joint_position_measurement_saturation"
            ) {
                required_f64(failure, "observed")? < required_f64(failure, "minimum")?
            } else {
                required_f64(failure, "observed")? > required_f64(failure, "maximum")?
            }
            && dimension_value(failure)? == first_failing_value(&report["boundary"])?,
        "robustness report has no valid first boundary failure"
    );
    anyhow::ensure!(
        trace.kind == "rne_openarm_backend_trace"
            && trace.schema_version == 1
            && trace.backend_id == "rne_rapier",
        "trace is not a supported Rapier robustness trace"
    );
    let primary = report["primary_backend_results"]
        .as_array()
        .context("robustness report has no primary results")?
        .iter()
        .find(|item| item["case_id"] == failure["case_id"])
        .context("robustness report has no matching failed Rapier case")?;
    anyhow::ensure!(
        required_str(primary, "trace_sha256")? == trace_sha256,
        "failed Rapier trace digest differs from the robustness report"
    );
    anyhow::ensure!(
        !trace.observations.is_empty()
            && trace
                .observations
                .iter()
                .enumerate()
                .all(|(index, observation)| {
                    observation.step == index as u64 + 1
                        && observation.sim_time_ticks == observation.step * trace.fixed_delta_ticks
                        && [
                            observation.joint_position_rad.len(),
                            observation.joint_velocity_rad_s.len(),
                            observation.joint_reference_position_rad.len(),
                            observation.joint_measurement_bias_rad.len(),
                            observation.joint_controller_target_rad.len(),
                            observation.joint_actuator_disturbance_rad.len(),
                            observation.joint_position_target_rad.len(),
                        ]
                        .iter()
                        .all(|width| *width == 9)
                        && matches!(
                            observation.joint_controller_observation_position_rad.len(),
                            0 | 9
                        )
                }),
        "Rapier robustness observations are not contiguous nine-joint evidence"
    );
    Ok(())
}

fn required_path(arguments: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    arguments
        .next()
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .with_context(|| format!("{option} requires a path"))
}

fn required_str<'a>(value: &'a Value, field: &str) -> Result<&'a str> {
    value[field]
        .as_str()
        .with_context(|| format!("missing string field {field}"))
}

fn required_u64(value: &Value, field: &str) -> Result<u64> {
    value[field]
        .as_u64()
        .with_context(|| format!("missing integer field {field}"))
}

fn required_f64(value: &Value, field: &str) -> Result<f64> {
    value[field]
        .as_f64()
        .filter(|number| number.is_finite())
        .with_context(|| format!("missing finite field {field}"))
}

fn dimension_value(value: &Value) -> Result<f64> {
    required_f64(value, "dimension_value").or_else(|_| required_f64(value, "offset_rad"))
}

fn first_failing_value(value: &Value) -> Result<f64> {
    required_f64(value, "first_failing_value")
        .or_else(|_| required_f64(value, "first_failing_offset_rad"))
}

fn digest_u64(bytes: &[u8]) -> u64 {
    u64::from_le_bytes(Sha256::digest(bytes)[..8].try_into().expect("eight bytes"))
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

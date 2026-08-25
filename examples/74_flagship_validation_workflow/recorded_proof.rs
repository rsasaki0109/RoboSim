//! Installed recorded-playback and shadow proof for the flagship task.

use super::{write_pretty_json, FlagshipObservation, FlagshipRecordedStep, CONTROLLER_ID, TASK_ID};
use anyhow::{Context, Result};
use rne_ai::TaskSpec;
use rne_hardware_gateway::recorded::{
    evaluate_recorded_shadow_session, CalibrationBinding, RecordedArtifactBinding,
    RecordedShadowFrame, RecordedShadowReport, RecordedShadowSession, RecordedStreamContract,
    RecordedTensorUnit, RECORDED_SHADOW_SCHEMA_VERSION, RECORDED_SHADOW_SESSION_KIND,
};
use rne_hardware_gateway::shadow::ShadowTensorTolerance;
use rne_hardware_gateway::HardwareMode;
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::Path;

const PROOF_KIND: &str = "rne_installed_recorded_shadow_proof";
const PROOF_SCHEMA_VERSION: u32 = 1;
const EXPERIMENT_ID: &str = "rne.flagship.mobile_lift.recorded_shadow.v1";
const MAX_RECORDED_SAMPLES: usize = 512;
const DISCONNECT_SEQUENCE: u64 = 128;

pub(super) const PROOF_ARTIFACTS: [&str; 12] = [
    "recorded-shadow-calibration.json",
    "recorded-shadow-controller.json",
    "recorded-shadow-disconnect.report.json",
    "recorded-shadow-disconnect.session.json",
    "recorded-shadow-mujoco.trace.json",
    "recorded-shadow-playback.report.json",
    "recorded-shadow-playback.session.json",
    "recorded-shadow-proof.json",
    "recorded-shadow-rapier.trace.json",
    "recorded-shadow-requirements.json",
    "recorded-shadow-shadow.report.json",
    "recorded-shadow-shadow.session.json",
];

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct RetainedTrace<'a> {
    kind: &'static str,
    schema_version: u32,
    task_id: &'static str,
    controller_id: &'static str,
    backend: &'a str,
    samples: &'a [FlagshipRecordedStep],
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct InstalledRecordedShadowCase {
    id: &'static str,
    mode: HardwareMode,
    expected_status: &'static str,
    observed_status: String,
    accepted_samples: usize,
    violating_elements: usize,
    first_divergence_tensor: Option<String>,
    suppressed_actions: usize,
    actuator_writes_emitted: bool,
    session: &'static str,
    report: &'static str,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct InstalledRecordedShadowProof {
    kind: &'static str,
    schema_version: u32,
    status: &'static str,
    task_id: &'static str,
    controller_id: &'static str,
    clock_source: &'static str,
    cases: Vec<InstalledRecordedShadowCase>,
}

pub(super) fn write_recorded_shadow_proof(
    output: &Path,
    task: &TaskSpec,
    rapier: &[FlagshipRecordedStep],
    mujoco: &[FlagshipRecordedStep],
) -> Result<()> {
    let sample_count = rapier.len().min(mujoco.len()).min(MAX_RECORDED_SAMPLES);
    anyhow::ensure!(
        sample_count >= DISCONNECT_SEQUENCE as usize,
        "flagship traces need at least {DISCONNECT_SEQUENCE} paired samples"
    );
    let rapier = &rapier[..sample_count];
    let mujoco = &mujoco[..sample_count];

    let task_sha256 = sha256_file(&output.join("flagship.task.json"))?;
    write_pretty_json(
        &output.join("recorded-shadow-controller.json"),
        &serde_json::json!({
            "kind": "rne_controller_contract",
            "schema_version": 1,
            "controller_id": CONTROLLER_ID,
            "policy": "IkMobileLiftPickPlacePolicy",
            "normalization": "missing_joint_targets_hold_pre_step_observation_v1",
            "action_order": [
                "left_wheel_velocity_rad_s", "right_wheel_velocity_rad_s",
                "shoulder_target_rad", "elbow_target_rad", "wrist_yaw_target_rad",
                "lift_target_m", "gripper_velocity_m_s"
            ]
        }),
    )?;
    write_pretty_json(
        &output.join("recorded-shadow-calibration.json"),
        &serde_json::json!({
            "kind": "rne_identity_si_calibration",
            "schema_version": 1,
            "task_id": TASK_ID,
            "timestamp_behavior": "capture_and_availability_ticks_retained",
            "latency_behavior": "one_fixed_control_period",
            "noise_behavior": "retained_backend_observation_no_added_noise",
            "scale": 1.0,
            "offset": 0.0
        }),
    )?;
    write_pretty_json(
        &output.join("recorded-shadow-requirements.json"),
        &serde_json::json!({
            "kind": "rne_recorded_shadow_requirements",
            "schema_version": 1,
            "experiment_id": EXPERIMENT_ID,
            "sample_count": sample_count,
            "disconnect_sequence": DISCONNECT_SEQUENCE,
            "expected": {
                "playback": "passed",
                "shadow": "failed",
                "disconnect": "failed_as_expected",
                "maximum_actuator_writes": 0,
                "maximum_dropped_observations": 0
            },
            "comparison": "absolute_per_tensor_v1"
        }),
    )?;
    write_pretty_json(
        &output.join("recorded-shadow-rapier.trace.json"),
        &RetainedTrace {
            kind: "rne_flagship_recorded_trace",
            schema_version: 1,
            task_id: TASK_ID,
            controller_id: CONTROLLER_ID,
            backend: "rapier_native",
            samples: rapier,
        },
    )?;
    write_pretty_json(
        &output.join("recorded-shadow-mujoco.trace.json"),
        &RetainedTrace {
            kind: "rne_flagship_recorded_trace",
            schema_version: 1,
            task_id: TASK_ID,
            controller_id: CONTROLLER_ID,
            backend: "mujoco_native",
            samples: mujoco,
        },
    )?;

    let controller_sha256 = sha256_file(&output.join("recorded-shadow-controller.json"))?;
    let calibration_sha256 = sha256_file(&output.join("recorded-shadow-calibration.json"))?;
    let requirements_sha256 = sha256_file(&output.join("recorded-shadow-requirements.json"))?;
    let rapier_sha256 = sha256_file(&output.join("recorded-shadow-rapier.trace.json"))?;
    let mujoco_sha256 = sha256_file(&output.join("recorded-shadow-mujoco.trace.json"))?;

    let playback = run_case(
        output,
        task,
        rapier,
        rapier,
        "playback",
        HardwareMode::Playback,
        None,
        &task_sha256,
        &controller_sha256,
        &calibration_sha256,
        &requirements_sha256,
        &rapier_sha256,
        &rapier_sha256,
        "recorded-shadow-rapier.trace.json",
        "recorded-shadow-rapier.trace.json",
    )?;
    let shadow = run_case(
        output,
        task,
        rapier,
        mujoco,
        "shadow",
        HardwareMode::Shadow,
        None,
        &task_sha256,
        &controller_sha256,
        &calibration_sha256,
        &requirements_sha256,
        &rapier_sha256,
        &mujoco_sha256,
        "recorded-shadow-rapier.trace.json",
        "recorded-shadow-mujoco.trace.json",
    )?;
    let disconnect = run_case(
        output,
        task,
        rapier,
        rapier,
        "disconnect",
        HardwareMode::Shadow,
        Some(DISCONNECT_SEQUENCE),
        &task_sha256,
        &controller_sha256,
        &calibration_sha256,
        &requirements_sha256,
        &rapier_sha256,
        &rapier_sha256,
        "recorded-shadow-rapier.trace.json",
        "recorded-shadow-rapier.trace.json",
    )?;

    anyhow::ensure!(
        playback.summary.status == "passed"
            && shadow.summary.status == "failed"
            && shadow
                .comparison
                .samples
                .iter()
                .any(|sample| sample.first_violation.is_some())
            && disconnect.summary.status == "failed_as_expected",
        "installed recorded/shadow cases did not match their predeclared outcomes"
    );
    anyhow::ensure!(
        [&playback, &shadow, &disconnect]
            .iter()
            .all(|report| !report.summary.actuator_writes_emitted),
        "installed recorded/shadow proof emitted an actuator write"
    );
    let proof = InstalledRecordedShadowProof {
        kind: PROOF_KIND,
        schema_version: PROOF_SCHEMA_VERSION,
        status: "passed",
        task_id: TASK_ID,
        controller_id: CONTROLLER_ID,
        clock_source: "sim_clock_fixed_step",
        cases: vec![
            case("playback", HardwareMode::Playback, "passed", &playback),
            case("shadow", HardwareMode::Shadow, "failed", &shadow),
            case(
                "disconnect",
                HardwareMode::Shadow,
                "failed_as_expected",
                &disconnect,
            ),
        ],
    };
    write_pretty_json(&output.join("recorded-shadow-proof.json"), &proof)
}

#[allow(clippy::too_many_arguments)]
fn run_case(
    output: &Path,
    task: &TaskSpec,
    recorded: &[FlagshipRecordedStep],
    simulation: &[FlagshipRecordedStep],
    id: &str,
    mode: HardwareMode,
    disconnect: Option<u64>,
    task_sha256: &str,
    controller_sha256: &str,
    calibration_sha256: &str,
    requirements_sha256: &str,
    recorded_sha256: &str,
    simulation_sha256: &str,
    recorded_file: &str,
    simulation_file: &str,
) -> Result<RecordedShadowReport> {
    let fixed_delta_ticks = (task.control_step_s * 1_000_000_000.0).round() as u64;
    let frames = recorded
        .iter()
        .zip(simulation)
        .enumerate()
        .map(|(index, (recorded, simulation))| {
            let sequence = u64::try_from(index + 1).unwrap_or(u64::MAX);
            let captured_at_ticks = sequence.saturating_mul(fixed_delta_ticks);
            RecordedShadowFrame {
                observation_sequence: sequence,
                dropped_sequences_before: 0,
                captured_at_ticks,
                available_at_ticks: captured_at_ticks.saturating_add(fixed_delta_ticks),
                simulation_step: sequence,
                simulation_time_ticks: captured_at_ticks,
                recorded_values: flatten_observation(&recorded.observation),
                simulation_values: flatten_observation(&simulation.observation),
                action_sequence: sequence,
                action_submitted_at_ticks: captured_at_ticks.saturating_add(fixed_delta_ticks),
                action_values: recorded.action_values.clone(),
            }
        })
        .collect::<Vec<_>>();
    let session = RecordedShadowSession {
        kind: RECORDED_SHADOW_SESSION_KIND.to_string(),
        schema_version: RECORDED_SHADOW_SCHEMA_VERSION,
        experiment_id: EXPERIMENT_ID.to_string(),
        requirements_sha256: requirements_sha256.to_string(),
        task_id: task.task_id.clone(),
        task_sha256: task_sha256.to_string(),
        controller_id: CONTROLLER_ID.to_string(),
        controller_sha256: controller_sha256.to_string(),
        sources: vec![
            RecordedArtifactBinding {
                role: "recorded_trace".to_string(),
                kind: "rne_flagship_recorded_trace".to_string(),
                file_name: recorded_file.to_string(),
                sha256: recorded_sha256.to_string(),
            },
            RecordedArtifactBinding {
                role: "simulation_trace".to_string(),
                kind: "rne_flagship_recorded_trace".to_string(),
                file_name: simulation_file.to_string(),
                sha256: simulation_sha256.to_string(),
            },
        ],
        bootstrap_action_count: 1,
        stream: RecordedStreamContract {
            clock_source: "sim_clock_fixed_step".to_string(),
            tick_period_ns: 1,
            nominal_latency_ticks: fixed_delta_ticks,
            maximum_latency_ticks: fixed_delta_ticks,
            drop_policy: "explicit_sequence_gap_v1".to_string(),
            sample_capacity: frames.len(),
            tensor_units: task
                .observation
                .tensors
                .iter()
                .map(|tensor| RecordedTensorUnit {
                    tensor_name: tensor.name.clone(),
                    unit: tensor.unit.clone(),
                })
                .collect(),
            calibrations: vec![CalibrationBinding {
                role: "flagship_observation".to_string(),
                kind: "identity_si_v1".to_string(),
                sha256: calibration_sha256.to_string(),
            }],
        },
        tolerances: tolerances(task),
        frames,
        disconnect_after_observation_sequence: disconnect,
    };
    let session_name = format!("recorded-shadow-{id}.session.json");
    let report_name = format!("recorded-shadow-{id}.report.json");
    write_pretty_json(&output.join(&session_name), &session)?;
    let session_sha256 = sha256_file(&output.join(&session_name))?;
    let report = evaluate_recorded_shadow_session(task.clone(), session, session_sha256, mode)
        .with_context(|| format!("evaluate installed {id} recorded/shadow case"))?;
    write_pretty_json(&output.join(report_name), &report)?;
    Ok(report)
}

fn case(
    id: &'static str,
    mode: HardwareMode,
    expected_status: &'static str,
    report: &RecordedShadowReport,
) -> InstalledRecordedShadowCase {
    InstalledRecordedShadowCase {
        id,
        mode,
        expected_status,
        observed_status: report.summary.status.clone(),
        accepted_samples: report.summary.accepted_samples,
        violating_elements: report.comparison.summary.violating_elements,
        first_divergence_tensor: report
            .comparison
            .samples
            .iter()
            .find_map(|sample| sample.first_violation.as_ref())
            .map(|divergence| divergence.tensor_name.clone()),
        suppressed_actions: report.summary.suppressed_actions,
        actuator_writes_emitted: report.summary.actuator_writes_emitted,
        session: match id {
            "playback" => "recorded-shadow-playback.session.json",
            "shadow" => "recorded-shadow-shadow.session.json",
            _ => "recorded-shadow-disconnect.session.json",
        },
        report: match id {
            "playback" => "recorded-shadow-playback.report.json",
            "shadow" => "recorded-shadow-shadow.report.json",
            _ => "recorded-shadow-disconnect.report.json",
        },
    }
}

fn flatten_observation(observation: &FlagshipObservation) -> Vec<f64> {
    vec![
        observation.base_x_m,
        observation.base_z_m,
        observation.shoulder_position_rad,
        observation.elbow_position_rad,
        observation.wrist_yaw_position_rad,
        observation.lift_position_m,
        observation.gripper_position_m,
        observation.payload_x_m,
        observation.payload_y_m,
        observation.payload_z_m,
        observation.wrist_camera_pixels as f64,
        observation.wrist_depth_min_m,
        observation.traffic_actor_x_m,
        observation.traffic_actor_y_m,
        observation.traffic_actor_z_m,
        if observation.traffic_signal_green {
            1.0
        } else {
            0.0
        },
        if observation.traffic_clear { 1.0 } else { 0.0 },
        if observation.grasped_once { 1.0 } else { 0.0 },
        f64::from(observation.policy_phase_index),
    ]
}

fn tolerances(task: &TaskSpec) -> Vec<ShadowTensorTolerance> {
    task.observation
        .tensors
        .iter()
        .map(|tensor| ShadowTensorTolerance {
            tensor_name: tensor.name.clone(),
            absolute_tolerance: match tensor.name.as_str() {
                "base_position_m" => 0.40,
                "arm_joint_position_rad" => 0.20,
                "lift_position_m" | "gripper_position_m" => 0.04,
                "payload_position_m" => 0.07,
                "wrist_depth_min_m" => 0.02,
                _ => 0.0,
            },
        })
        .collect()
}

fn sha256_file(path: &Path) -> Result<String> {
    let bytes = fs::read(path).with_context(|| format!("read {}", path.display()))?;
    Ok(format!("{:x}", Sha256::digest(bytes)))
}

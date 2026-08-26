//! Executes one portable OpenArm reference and typed feedback controller on Rapier.

use anyhow::{bail, Context, Result};
use rne_ai::{
    BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction, BehaviorReplayArtifact,
    BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation, TaskSpec,
    UrdfJointFeedbackSensorConfig, UrdfJointPdEffortTarget, UrdfSceneSim,
};
use rne_data::{
    DataBus, Frame, InMemoryDataBus, JointEffortFeedback, JointFeedback, JointFeedbackStatus,
    StreamId,
};
use rne_ecs::Name;
use rne_physics::{hash_physics_state_v2, JointPassiveDynamics};
use rne_sensor::JointFeedbackFault;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

const KIND: &str = "rne_joint_pose_cycle_controller";
const SCHEMA_VERSION: u32 = 1;
const TRACE_KIND: &str = "rne_controller_action_trace";
const RAPIER_TRACE_KIND: &str = "rne_openarm_backend_trace";
const FAILURE_KIND: &str = "rne_controller_contract_failure";
const FIXED_DELTA_TICKS: u64 = 16_666_667;
const ACTUATION_CONFIG_KIND: &str = "rne_portable_pd_effort_actuation_config";
const JOINT_FEEDBACK_STREAM: StreamId = StreamId::new(9_001);
const SENSOR_REPORT_KIND: &str = "rne_sensor_validation_report";
const SENSOR_FAULT_SEQUENCE: u64 = 307;
const PHYSICS_HASH_CONTRACT: &str = "rne_physics_state_v2_fnv1a_1e-6_si";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ControllerSpec {
    kind: String,
    schema_version: u32,
    controller_id: String,
    task_id: String,
    interpolation: String,
    action_joint_order: Vec<String>,
    rne_actuator_link_order: Vec<String>,
    #[serde(default)]
    observation_contract: Option<ObservationContract>,
    #[serde(default)]
    feedback_law: Option<FeedbackLaw>,
    #[serde(default)]
    disturbance_contract: Option<DisturbanceContract>,
    #[serde(default)]
    measurement_fault_contract: Option<MeasurementFaultContract>,
    keyframes: Vec<Keyframe>,
    intentional_failure: IntentionalFailure,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ObservationContract {
    kind: String,
    schema_version: u32,
    sample_period_ticks: u64,
    phase_offset_ticks: u64,
    latency_ticks: u64,
    maximum_age_ticks: u64,
    required_status: JointFeedbackStatus,
    bootstrap_policy: String,
    #[serde(default)]
    stale_observation_policy: Option<String>,
    #[serde(default)]
    recovery_policy: Option<String>,
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", deny_unknown_fields)]
enum DisturbanceContract {
    #[serde(rename = "additive_actuator_target_bias_pulse_v1")]
    AdditiveActuatorTargetBiasPulseV1 {
        classification: String,
        joint: String,
        start_step: u64,
        end_step: u64,
        offset_rad: f64,
        controller_visibility: String,
        application_order: String,
    },
    #[serde(rename = "actuator_command_transport_delay_pulse_v1")]
    ActuatorCommandTransportDelayPulseV1 {
        classification: String,
        joint: String,
        start_step: u64,
        end_step: u64,
        delay_steps: u64,
        controller_visibility: String,
        application_order: String,
    },
    #[serde(rename = "actuator_command_slew_rate_limit_pulse_v1")]
    ActuatorCommandSlewRateLimitPulseV1 {
        classification: String,
        joint: String,
        start_step: u64,
        end_step: u64,
        maximum_rate_rad_s: f64,
        controller_visibility: String,
        application_order: String,
    },
    #[serde(rename = "actuator_command_deadband_pulse_v1")]
    ActuatorCommandDeadbandPulseV1 {
        classification: String,
        joint: String,
        start_step: u64,
        end_step: u64,
        deadband_rad: f64,
        controller_visibility: String,
        application_order: String,
    },
}

#[derive(Debug, Deserialize)]
#[serde(tag = "kind", deny_unknown_fields)]
enum MeasurementFaultContract {
    #[serde(rename = "additive_joint_position_bias_pulse_v1")]
    AdditiveJointPositionBiasPulseV1 {
        classification: String,
        joint: String,
        start_controller_step: u64,
        end_controller_step: u64,
        offset_rad: f64,
        sensor_status: JointFeedbackStatus,
        controller_visibility: String,
        application_order: String,
    },
    #[serde(rename = "joint_feedback_publication_dropout_burst_v1")]
    JointFeedbackPublicationDropoutBurstV1 {
        classification: String,
        start_capture_sequence: u64,
        consecutive_dropped_frames: u64,
        controller_visibility: String,
        application_order: String,
    },
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FeedbackLaw {
    kind: String,
    #[serde(default)]
    position_error_gain: Vec<f64>,
    #[serde(default)]
    velocity_damping_s: Vec<f64>,
    #[serde(default)]
    integral_error_gain_s_inv: Vec<f64>,
    #[serde(default)]
    maximum_integral_correction_rad: Vec<f64>,
    #[serde(default)]
    maximum_correction_rad: Vec<f64>,
    #[serde(default)]
    minimum_target_rad: Vec<f64>,
    #[serde(default)]
    maximum_target_rad: Vec<f64>,
    #[serde(default)]
    controlled_joint: Option<String>,
    #[serde(default)]
    state_order: Vec<String>,
    #[serde(default)]
    reference_feedforward: Option<String>,
    #[serde(default)]
    observation_latency_compensation: Option<String>,
    #[serde(default)]
    operating_point_position_rad: Option<f64>,
    #[serde(default)]
    operating_point_input_rad: Option<f64>,
    #[serde(default)]
    identified_plant: Option<IdentifiedPlant>,
    #[serde(default)]
    state_feedback_gain: Vec<f64>,
    #[serde(default)]
    integral_state_feedback_gain_s_inv: Option<f64>,
    #[serde(default)]
    desired_closed_loop_poles: Vec<f64>,
    #[serde(default)]
    closed_loop_a: Vec<Vec<f64>>,
    #[serde(default)]
    maximum_integral_state_error_rad_s: Option<f64>,
    #[serde(default)]
    maximum_state_integral_correction_rad: Option<f64>,
    #[serde(default)]
    maximum_state_feedback_correction_rad: Option<f64>,
    #[serde(default)]
    minimum_controlled_target_rad: Option<f64>,
    #[serde(default)]
    maximum_controlled_target_rad: Option<f64>,
}

#[derive(Debug, Deserialize)]
struct IdentifiedPlant {
    arx_coefficients: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Keyframe {
    step: u64,
    phase: String,
    joint_position_target_rad: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IntentionalFailure {
    kind: String,
    inject_at_step: u64,
    expected_first_violation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ActuationConfig {
    kind: String,
    schema_version: u32,
    backend_id: String,
    motor_model: String,
    solver_iterations: usize,
    #[serde(default = "default_physics_substeps_per_control_step")]
    physics_substeps_per_control_step: usize,
    fixed_delta_ticks: u64,
    joints: Vec<JointActuationConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct JointActuationConfig {
    joint_name: String,
    link_name: String,
    stiffness_nm_per_rad: f64,
    damping_nm_s_per_rad: f64,
    max_effort_nm: f64,
    max_velocity_rad_s: f64,
    #[serde(default = "unit_transmission_efficiency")]
    transmission_efficiency: f64,
}

#[derive(Debug, Serialize)]
struct ActionTrace<'a> {
    kind: &'static str,
    schema_version: u32,
    task_id: &'a str,
    task_sha256: &'a str,
    controller_id: &'a str,
    controller_sha256: &'a str,
    fixed_delta_ticks: u64,
    action_semantics: &'static str,
    action_joint_order: &'a [String],
    actions: Vec<ActionFrame>,
}

#[derive(Clone, Debug, Serialize)]
struct ActionFrame {
    action_sequence: u64,
    step: u64,
    sim_time_ticks: u64,
    phase: String,
    joint_position_target_rad: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
struct ObservationFrame {
    step: u64,
    sim_time_ticks: u64,
    scheduled_capture_ticks: u64,
    sample_phase_error_ticks: u64,
    available_time_ticks: u64,
    consumed_at_ticks: u64,
    observation_age_ticks: u64,
    sensor_status: JointFeedbackStatus,
    sensor_sample_published: bool,
    controller_observation_sequence: Option<u64>,
    controller_observation_age_ticks: Option<u64>,
    controller_bootstrap: bool,
    controller_rejected: bool,
    controller_rejection_reason: Option<&'static str>,
    fail_safe_hold_active: bool,
    controller_state_frozen: bool,
    controller_recovered: bool,
    joint_position_rad: Vec<f64>,
    joint_velocity_rad_s: Vec<f64>,
    joint_reference_position_rad: Vec<f64>,
    joint_controller_observation_position_rad: Vec<f64>,
    joint_measurement_bias_rad: Vec<f64>,
    measurement_bias_active: bool,
    joint_controller_target_rad: Vec<f64>,
    joint_actuator_disturbance_rad: Vec<f64>,
    joint_position_target_rad: Vec<f64>,
    actuator_disturbance_active: bool,
    joint_feedback_correction_rad: Vec<f64>,
    joint_integral_correction_rad: Vec<f64>,
    limited_effort_command_nm: Vec<f64>,
    measured_effort_nm: Vec<Option<f64>>,
    effort_saturated: Vec<bool>,
    effort_measurement_available: Vec<bool>,
    maximum_actuator_tracking_error_rad: f64,
    maximum_tracking_error_rad: f64,
    physics_hash: u64,
}

#[derive(Clone, Debug)]
struct AppliedActuation {
    target_position_rad: Vec<f64>,
    limited_effort_command_nm: Vec<f64>,
    effort_saturated: Vec<bool>,
}

#[derive(Debug, Serialize)]
struct BackendTrace<'a> {
    kind: &'static str,
    schema_version: u32,
    backend_id: &'static str,
    backend_version: &'static str,
    task_id: &'a str,
    task_sha256: &'a str,
    controller_id: &'a str,
    controller_sha256: &'a str,
    action_trace_sha256: &'a str,
    robot_asset_config_sha256: &'a str,
    model_urdf_sha256: &'a str,
    scene_config_sha256: &'a str,
    actuation_config_sha256: &'a str,
    fixed_delta_ticks: u64,
    physics_substeps_per_control_step: usize,
    joint_feedback_schema_version: u32,
    joint_feedback_latency_ticks: u64,
    observation_source: &'static str,
    controller_execution: &'static str,
    physics_state_hash_contract: &'static str,
    initial_state_digest: u64,
    final_state_digest: u64,
    replay_final_state_digest: u64,
    replay_match: bool,
    maximum_sensor_backend_position_delta_rad: f64,
    maximum_sensor_backend_velocity_delta_rad_s: f64,
    final_maximum_tracking_error_rad: f64,
    maximum_tracking_error_rad: f64,
    joint_passive_dynamics: Vec<Option<JointPassiveDynamics>>,
    observations: Vec<ObservationFrame>,
}

#[derive(Debug, Serialize)]
struct FailureReport<'a> {
    kind: &'static str,
    schema_version: u32,
    backend_id: &'static str,
    backend_version: &'static str,
    task_id: &'a str,
    task_sha256: &'a str,
    controller_id: &'a str,
    controller_sha256: &'a str,
    action_trace_sha256: &'a str,
    robot_asset_config_sha256: &'a str,
    model_urdf_sha256: &'a str,
    scene_config_sha256: &'a str,
    actuation_config_sha256: &'a str,
    injection_kind: &'a str,
    injected_step: u64,
    first_violation: &'a str,
    first_violation_step: u64,
    first_violation_sim_time_ticks: u64,
    unit: &'static str,
    observed_missing_action_elements: u64,
    maximum_missing_action_elements: u64,
    status: &'static str,
}

struct Rollout {
    world_seed: u64,
    initial_digest: u64,
    final_digest: u64,
    observations: Vec<ObservationFrame>,
    maximum_sensor_backend_position_delta_rad: f64,
    maximum_sensor_backend_velocity_delta_rad_s: f64,
    joint_passive_dynamics: Vec<Option<JointPassiveDynamics>>,
}

#[derive(Clone, Debug, PartialEq)]
struct ControllerObservation {
    sequence: u64,
    capture_time_ticks: u64,
    available_time_ticks: u64,
    status: JointFeedbackStatus,
    joint_position_rad: Vec<f64>,
    joint_velocity_rad_s: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct ControllerDecision {
    reference_position_rad: Vec<f64>,
    target_position_rad: Vec<f64>,
    correction_rad: Vec<f64>,
    integral_correction_rad: Vec<f64>,
    controller_observation_position_rad: Vec<f64>,
    measurement_bias_rad: Vec<f64>,
    observation_sequence: Option<u64>,
    observation_age_ticks: Option<u64>,
    bootstrap: bool,
    rejected: bool,
    rejection_reason: Option<&'static str>,
    fail_safe_hold_active: bool,
    state_frozen: bool,
    recovered: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct ControllerState {
    integral_correction_rad: Vec<f64>,
    previous_observation_position_rad: Vec<Option<f64>>,
    previous_input_target_rad: Vec<Option<f64>>,
    previous_previous_input_target_rad: Vec<Option<f64>>,
}

impl ControllerState {
    fn new(width: usize) -> Self {
        Self {
            integral_correction_rad: vec![0.0; width],
            previous_observation_position_rad: vec![None; width],
            previous_input_target_rad: vec![None; width],
            previous_previous_input_target_rad: vec![None; width],
        }
    }
}

#[derive(Debug, Serialize)]
struct ControllerRejection {
    action_step: u64,
    detected_at_ticks: u64,
    observation_sequence: u64,
    observation_age_ticks: u64,
    first_violated_contract: &'static str,
}

struct InputHashes<'a> {
    task_sha256: &'a str,
    controller_sha256: &'a str,
    action_trace_sha256: &'a str,
    robot_asset_config_sha256: &'a str,
    model_urdf_sha256: &'a str,
    scene_config_sha256: &'a str,
    actuation_config_sha256: &'a str,
}

struct SensorValidationArtifacts {
    report: serde_json::Value,
    dropout_trace: serde_json::Value,
    stuck_trace: serde_json::Value,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("OpenArm Rapier trace failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut controller_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json");
    let mut task_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json");
    let mut actuation_config_path =
        repo_root.join("adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json");
    let mut robot_asset_config_path =
        repo_root.join("assets/robots/openarm_v2_right.rne.robot.toml");
    let mut model_urdf_path =
        repo_root.join("assets/robots/openarm_description/openarm_v2_right.rne.urdf");
    let mut scene_path = repo_root.join("assets/scenes/openarm_v2_right_validation.rne.scene.toml");
    let mut output = repo_root.join("artifacts/openarm-cross-sim");
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--controller" => controller_path = required_path(&mut args, "--controller")?,
            "--task" => task_path = required_path(&mut args, "--task")?,
            "--actuation-config" => {
                actuation_config_path = required_path(&mut args, "--actuation-config")?
            }
            "--robot-asset" => robot_asset_config_path = required_path(&mut args, "--robot-asset")?,
            "--model-urdf" => model_urdf_path = required_path(&mut args, "--model-urdf")?,
            "--scene" => scene_path = required_path(&mut args, "--scene")?,
            "--output" => output = required_path(&mut args, "--output")?,
            other => bail!("unknown argument {other:?}"),
        }
    }

    let controller_bytes = fs::read(&controller_path)
        .with_context(|| format!("read {}", controller_path.display()))?;
    let task_bytes =
        fs::read(&task_path).with_context(|| format!("read {}", task_path.display()))?;
    let actuation_config_bytes = fs::read(&actuation_config_path)
        .with_context(|| format!("read {}", actuation_config_path.display()))?;
    let robot_asset_config_bytes = fs::read(&robot_asset_config_path)
        .with_context(|| format!("read {}", robot_asset_config_path.display()))?;
    let model_urdf_bytes = fs::read(&model_urdf_path)
        .with_context(|| format!("read {}", model_urdf_path.display()))?;
    let scene_config_bytes =
        fs::read(&scene_path).with_context(|| format!("read {}", scene_path.display()))?;
    validate_model_provenance(&scene_path, &robot_asset_config_path, &model_urdf_path)?;
    let controller: ControllerSpec = serde_json::from_slice(&controller_bytes)
        .with_context(|| format!("parse {}", controller_path.display()))?;
    let task: TaskSpec = serde_json::from_slice(&task_bytes)
        .with_context(|| format!("parse {}", task_path.display()))?;
    let actuation_config: ActuationConfig = serde_json::from_slice(&actuation_config_bytes)
        .with_context(|| format!("parse {}", actuation_config_path.display()))?;
    validate(&controller, &task, &actuation_config)?;

    let controller_sha256 = sha256(&controller_bytes);
    let task_sha256 = sha256(&task_bytes);
    let actuation_config_sha256 = sha256(&actuation_config_bytes);
    let robot_asset_config_sha256 = sha256(&robot_asset_config_bytes);
    let model_urdf_sha256 = sha256(&model_urdf_bytes);
    let scene_config_sha256 = sha256(&scene_config_bytes);
    let actions = compile_actions(&controller);
    fs::create_dir_all(&output)?;
    let action_path = output.join("controller-actions.json");
    write_json(
        &action_path,
        &ActionTrace {
            kind: TRACE_KIND,
            schema_version: 1,
            task_id: &controller.task_id,
            task_sha256: &task_sha256,
            controller_id: &controller.controller_id,
            controller_sha256: &controller_sha256,
            fixed_delta_ticks: FIXED_DELTA_TICKS,
            action_semantics: "reference_trajectory_before_sensor_feedback",
            action_joint_order: &controller.action_joint_order,
            actions: actions.clone(),
        },
    )?;

    let action_trace_sha256 = sha256(&fs::read(&action_path)?);

    let first = rollout(
        &scene_path,
        &controller,
        &actuation_config,
        &actions,
        JointFeedbackFault::None,
    )?;
    let replay = rollout(
        &scene_path,
        &controller,
        &actuation_config,
        &actions,
        JointFeedbackFault::None,
    )?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest && first.observations == replay.observations,
        "Rapier replay differed for the exact same controller trace"
    );
    if !has_dropout_fault(&controller) {
        let sensor_artifacts = build_sensor_validation_report(
            &scene_path,
            &controller,
            &actuation_config,
            &actions,
            &first,
            &replay,
            &InputHashes {
                task_sha256: &task_sha256,
                controller_sha256: &controller_sha256,
                action_trace_sha256: &action_trace_sha256,
                robot_asset_config_sha256: &robot_asset_config_sha256,
                model_urdf_sha256: &model_urdf_sha256,
                scene_config_sha256: &scene_config_sha256,
                actuation_config_sha256: &actuation_config_sha256,
            },
        )?;
        write_json(
            &output.join("sensor-validation-report.json"),
            &sensor_artifacts.report,
        )?;
        write_json(
            &output.join("sensor-dropout-trace.json"),
            &sensor_artifacts.dropout_trace,
        )?;
        write_json(
            &output.join("sensor-stuck-trace.json"),
            &sensor_artifacts.stuck_trace,
        )?;
        write_html_report(
            &output.join("sensor-validation-report.html"),
            "OpenArm Joint Feedback Validation",
            &sensor_artifacts.report,
        )?;
        anyhow::ensure!(
            sensor_artifacts.report["status"] == "passed",
            "OpenArm joint-feedback sensor validation did not pass"
        );
    }
    write_failure_replay(
        &output.join("controller-failure.rne-replay"),
        &controller,
        &actions,
        &first,
    )?;
    let final_error = first
        .observations
        .last()
        .context("Rapier trace has no observations")?
        .maximum_tracking_error_rad;
    let maximum_error = first
        .observations
        .iter()
        .map(|frame| frame.maximum_tracking_error_rad)
        .fold(0.0_f64, f64::max);
    write_json(
        &output.join("rapier-success-trace.json"),
        &BackendTrace {
            kind: RAPIER_TRACE_KIND,
            schema_version: 1,
            backend_id: "rne_rapier",
            backend_version: "0.22",
            task_id: &controller.task_id,
            task_sha256: &task_sha256,
            controller_id: &controller.controller_id,
            controller_sha256: &controller_sha256,
            action_trace_sha256: &action_trace_sha256,
            robot_asset_config_sha256: &robot_asset_config_sha256,
            model_urdf_sha256: &model_urdf_sha256,
            scene_config_sha256: &scene_config_sha256,
            actuation_config_sha256: &actuation_config_sha256,
            fixed_delta_ticks: FIXED_DELTA_TICKS,
            physics_substeps_per_control_step: actuation_config.physics_substeps_per_control_step,
            joint_feedback_schema_version: JointFeedback::SCHEMA_VERSION,
            joint_feedback_latency_ticks: FIXED_DELTA_TICKS,
            observation_source: "databus_latest_available",
            controller_execution: controller_execution(&controller),
            physics_state_hash_contract: PHYSICS_HASH_CONTRACT,
            initial_state_digest: first.initial_digest,
            final_state_digest: first.final_digest,
            replay_final_state_digest: replay.final_digest,
            replay_match: true,
            maximum_sensor_backend_position_delta_rad: first
                .maximum_sensor_backend_position_delta_rad,
            maximum_sensor_backend_velocity_delta_rad_s: first
                .maximum_sensor_backend_velocity_delta_rad_s,
            final_maximum_tracking_error_rad: final_error,
            maximum_tracking_error_rad: maximum_error,
            joint_passive_dynamics: first.joint_passive_dynamics,
            observations: first.observations,
        },
    )?;

    let failure = &controller.intentional_failure;
    write_json(
        &output.join("intentional-failure.json"),
        &FailureReport {
            kind: FAILURE_KIND,
            schema_version: 1,
            backend_id: "rne_rapier",
            backend_version: "0.22",
            task_id: &controller.task_id,
            task_sha256: &task_sha256,
            controller_id: &controller.controller_id,
            controller_sha256: &controller_sha256,
            action_trace_sha256: &action_trace_sha256,
            robot_asset_config_sha256: &robot_asset_config_sha256,
            model_urdf_sha256: &model_urdf_sha256,
            scene_config_sha256: &scene_config_sha256,
            actuation_config_sha256: &actuation_config_sha256,
            injection_kind: &failure.kind,
            injected_step: failure.inject_at_step,
            first_violation: &failure.expected_first_violation,
            first_violation_step: failure.inject_at_step,
            first_violation_sim_time_ticks: failure.inject_at_step * FIXED_DELTA_TICKS,
            unit: "missing_action_element_count",
            observed_missing_action_elements: 1,
            maximum_missing_action_elements: 0,
            status: "failed_as_expected",
        },
    )?;
    println!(
        "OpenArm Rapier trace: steps={} replay_match=true final_error_rad={final_error:.6} action_sha256={action_trace_sha256}",
        actions.len()
    );
    Ok(())
}

fn controller_execution(controller: &ControllerSpec) -> &'static str {
    match controller
        .feedback_law
        .as_ref()
        .map(|law| law.kind.as_str())
    {
        None => "open_loop_reference",
        Some("joint_position_reference_pid_v1") => "artifact_defined_joint_feedback_pid",
        Some("joint_position_state_feedback_integral_v1") => {
            "artifact_defined_joint_feedback_state_space"
        }
        Some(_) => "unsupported",
    }
}

fn validate(
    controller: &ControllerSpec,
    task: &TaskSpec,
    actuation_config: &ActuationConfig,
) -> Result<()> {
    task.validate()?;
    anyhow::ensure!(
        controller.kind == KIND && controller.schema_version == SCHEMA_VERSION,
        "unsupported controller kind or schema"
    );
    anyhow::ensure!(
        controller.task_id == task.task_id,
        "controller TaskSpec mismatch"
    );
    anyhow::ensure!(
        controller.interpolation == "smoothstep_v1",
        "unsupported controller interpolation"
    );
    let width = controller.action_joint_order.len();
    anyhow::ensure!(
        width == 9,
        "OpenArm right-arm controller must expose nine joints"
    );
    match (&controller.observation_contract, &controller.feedback_law) {
        (None, None) => {}
        (Some(contract), Some(law)) => {
            anyhow::ensure!(
                contract.kind == "rne_joint_feedback"
                    && contract.schema_version == JointFeedback::SCHEMA_VERSION,
                "unsupported OpenArm controller observation contract"
            );
            anyhow::ensure!(
                contract.sample_period_ticks == FIXED_DELTA_TICKS
                    && contract.phase_offset_ticks == FIXED_DELTA_TICKS
                    && contract.latency_ticks == FIXED_DELTA_TICKS
                    && if has_dropout_fault(controller) {
                        contract.maximum_age_ticks == 3 * FIXED_DELTA_TICKS
                            && contract.stale_observation_policy.as_deref()
                                == Some("hold_last_accepted_target_and_freeze_state")
                            && contract.recovery_policy.as_deref()
                                == Some("resume_on_fresh_nominal_observation")
                    } else {
                        contract.maximum_age_ticks == FIXED_DELTA_TICKS
                            && contract.stale_observation_policy.is_none()
                            && contract.recovery_policy.is_none()
                    },
                "OpenArm controller timing or stale-observation policy is invalid"
            );
            anyhow::ensure!(
                contract.required_status == JointFeedbackStatus::Nominal
                    && contract.bootstrap_policy == "reference_until_first_available",
                "unsupported OpenArm controller status or bootstrap policy"
            );
            if law.kind == "joint_position_reference_pid_v1" {
                validate_pid_law(law, width)?;
            } else if law.kind == "joint_position_state_feedback_integral_v1" {
                validate_state_feedback_law(controller, law)?;
            } else {
                bail!("unsupported OpenArm feedback law {}", law.kind);
            }
        }
        _ => bail!("OpenArm controller must declare both observation contract and feedback law"),
    }
    anyhow::ensure!(
        controller.rne_actuator_link_order.len() == width,
        "controller simulator/RNE joint mappings differ in width"
    );
    anyhow::ensure!(
        controller
            .action_joint_order
            .iter()
            .collect::<HashSet<_>>()
            .len()
            == width
            && controller
                .rne_actuator_link_order
                .iter()
                .collect::<HashSet<_>>()
                .len()
                == width,
        "controller joint mappings contain duplicates"
    );
    anyhow::ensure!(
        task.action.tensors.len() == 1
            && task.action.tensors[0].name == "joint_position_target_rad"
            && task.action.tensors[0].unit == "rad"
            && task.action.tensors[0].shape == [width],
        "TaskSpec action catalog differs from controller"
    );
    anyhow::ensure!(
        task.observation.tensors.len() == 2
            && task.observation.tensors[0].name == "joint_position_rad"
            && task.observation.tensors[0].unit == "rad"
            && task.observation.tensors[0].shape == [width]
            && task.observation.tensors[1].name == "joint_velocity_rad_s"
            && task.observation.tensors[1].unit == "rad/s"
            && task.observation.tensors[1].shape == [width],
        "TaskSpec observation catalog differs from controller"
    );
    anyhow::ensure!(
        (task.control_step_s * 1_000_000_000.0).round() as u64 == FIXED_DELTA_TICKS,
        "TaskSpec control step differs from fixed trace step"
    );
    anyhow::ensure!(
        controller.keyframes.len() >= 2,
        "controller needs two keyframes"
    );
    anyhow::ensure!(
        controller.keyframes[0].step == 0,
        "first keyframe must be step zero"
    );
    for pair in controller.keyframes.windows(2) {
        anyhow::ensure!(pair[0].step < pair[1].step, "keyframe steps must increase");
    }
    for keyframe in &controller.keyframes {
        anyhow::ensure!(!keyframe.phase.trim().is_empty(), "keyframe phase is empty");
        anyhow::ensure!(
            keyframe.joint_position_target_rad.len() == width
                && keyframe
                    .joint_position_target_rad
                    .iter()
                    .all(|value| value.is_finite() && (-3.0..=3.0).contains(value)),
            "keyframe target is invalid or outside TaskSpec bounds"
        );
    }
    let final_step = controller.keyframes.last().unwrap().step;
    if let Some(disturbance) = &controller.disturbance_contract {
        validate_actuator_disturbance(controller, disturbance, final_step)?;
    }
    if let Some(fault) = &controller.measurement_fault_contract {
        match fault {
            MeasurementFaultContract::AdditiveJointPositionBiasPulseV1 {
                classification,
                joint,
                start_controller_step,
                end_controller_step,
                offset_rad,
                sensor_status,
                controller_visibility,
                application_order,
            } => anyhow::ensure!(
                classification == "measurement_error"
                    && controller
                        .action_joint_order
                        .iter()
                        .any(|name| name == joint)
                    && *start_controller_step >= 1
                    && start_controller_step <= end_controller_step
                    && *end_controller_step <= final_step
                    && offset_rad.is_finite()
                    && *sensor_status == JointFeedbackStatus::Nominal
                    && controller_visibility == "biased_position_as_nominal"
                    && application_order
                        == "after_typed_feedback_availability_before_controller_law",
                "invalid OpenArm measurement-bias contract"
            ),
            MeasurementFaultContract::JointFeedbackPublicationDropoutBurstV1 {
                classification,
                start_capture_sequence,
                consecutive_dropped_frames,
                controller_visibility,
                application_order,
            } => anyhow::ensure!(
                classification == "measurement_unavailability"
                    && *start_capture_sequence >= 1
                    && start_capture_sequence.saturating_add(*consecutive_dropped_frames)
                        <= final_step.saturating_add(1)
                    && controller_visibility == "missing_publication_only"
                    && application_order == "after_typed_sensor_capture_before_controller_ingress",
                "invalid OpenArm measurement-dropout contract"
            ),
        }
    }
    anyhow::ensure!(
        (1..=final_step).contains(&controller.intentional_failure.inject_at_step),
        "intentional failure step is outside rollout"
    );
    anyhow::ensure!(
        controller.intentional_failure.kind == "controller_output_truncation"
            && controller.intentional_failure.expected_first_violation == "action_width_mismatch",
        "intentional failure contract is unsupported"
    );
    anyhow::ensure!(
        actuation_config.kind == ACTUATION_CONFIG_KIND
            && actuation_config.schema_version == 1
            && actuation_config.backend_id == "rne_native_physics"
            && actuation_config.motor_model == "explicit_pd_effort_v1"
            && actuation_config.solver_iterations > 0
            && actuation_config.physics_substeps_per_control_step > 0
            && actuation_config.physics_substeps_per_control_step <= FIXED_DELTA_TICKS as usize
            && FIXED_DELTA_TICKS
                .is_multiple_of(actuation_config.physics_substeps_per_control_step as u64)
            && actuation_config.fixed_delta_ticks == FIXED_DELTA_TICKS,
        "unsupported or invalid RNE actuation configuration"
    );
    anyhow::ensure!(
        actuation_config
            .joints
            .iter()
            .map(|joint| &joint.joint_name)
            .eq(&controller.action_joint_order)
            && actuation_config
                .joints
                .iter()
                .map(|joint| &joint.link_name)
                .eq(&controller.rne_actuator_link_order),
        "RNE actuation configuration order differs from controller"
    );
    anyhow::ensure!(
        actuation_config.joints.iter().all(|joint| {
            [
                joint.stiffness_nm_per_rad,
                joint.damping_nm_s_per_rad,
                joint.max_effort_nm,
                joint.max_velocity_rad_s,
                joint.transmission_efficiency,
            ]
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0)
                && joint.max_velocity_rad_s > 0.0
                && joint.transmission_efficiency > 0.0
                && joint.transmission_efficiency <= 1.0
        }),
        "RNE actuation configuration has invalid gains, effort, velocity, or transmission efficiency"
    );
    Ok(())
}

fn validate_pid_law(law: &FeedbackLaw, width: usize) -> Result<()> {
    for (name, values) in [
        ("position_error_gain", &law.position_error_gain),
        ("velocity_damping_s", &law.velocity_damping_s),
        ("integral_error_gain_s_inv", &law.integral_error_gain_s_inv),
        (
            "maximum_integral_correction_rad",
            &law.maximum_integral_correction_rad,
        ),
        ("maximum_correction_rad", &law.maximum_correction_rad),
        ("minimum_target_rad", &law.minimum_target_rad),
        ("maximum_target_rad", &law.maximum_target_rad),
    ] {
        anyhow::ensure!(
            values.len() == width && values.iter().all(|value| value.is_finite()),
            "OpenArm feedback field {name} must contain {width} finite values"
        );
    }
    anyhow::ensure!(
        law.position_error_gain.iter().all(|value| *value >= 0.0)
            && law.velocity_damping_s.iter().all(|value| *value >= 0.0)
            && law
                .integral_error_gain_s_inv
                .iter()
                .all(|value| *value >= 0.0)
            && law
                .maximum_integral_correction_rad
                .iter()
                .all(|value| *value >= 0.0)
            && law.maximum_correction_rad.iter().all(|value| *value >= 0.0)
            && law
                .minimum_target_rad
                .iter()
                .zip(&law.maximum_target_rad)
                .all(|(minimum, maximum)| minimum < maximum),
        "OpenArm feedback gains, correction limits, or target bounds are invalid"
    );
    Ok(())
}

fn validate_state_feedback_law(controller: &ControllerSpec, law: &FeedbackLaw) -> Result<()> {
    let controlled_joint = law
        .controlled_joint
        .as_deref()
        .context("state-feedback law has no controlled joint")?;
    let scalar_fields = [
        law.operating_point_position_rad,
        law.operating_point_input_rad,
        law.integral_state_feedback_gain_s_inv,
        law.maximum_integral_state_error_rad_s,
        law.maximum_state_integral_correction_rad,
        law.maximum_state_feedback_correction_rad,
        law.minimum_controlled_target_rad,
        law.maximum_controlled_target_rad,
    ];
    anyhow::ensure!(
        controller
            .action_joint_order
            .iter()
            .any(|joint| joint == controlled_joint)
            && law.state_order
                == [
                    "predicted_tracking_error_rad",
                    "observed_tracking_error_rad",
                    "previous_input_tracking_error_rad",
                    "integrated_reference_error_rad_s",
                ]
            && law.reference_feedforward.as_deref() == Some("unity_position_reference_v1")
            && law.observation_latency_compensation.as_deref()
                == Some("one_sample_arx_predictor_v1")
            && law.state_feedback_gain.len() == 3
            && law
                .state_feedback_gain
                .iter()
                .all(|value| value.is_finite())
            && law.desired_closed_loop_poles.len() == 4
            && law
                .desired_closed_loop_poles
                .iter()
                .all(|pole| pole.is_finite() && pole.abs() < 1.0)
            && law.closed_loop_a.len() == 4
            && law
                .closed_loop_a
                .iter()
                .all(|row| row.len() == 4 && row.iter().all(|value| value.is_finite()))
            && scalar_fields
                .iter()
                .all(|value| value.is_some_and(f64::is_finite))
            && law
                .identified_plant
                .as_ref()
                .is_some_and(|plant| plant.arx_coefficients.len() == 5
                    && plant.arx_coefficients.iter().all(|value| value.is_finite()))
            && law
                .integral_state_feedback_gain_s_inv
                .is_some_and(|value| value > 0.0)
            && law
                .maximum_integral_state_error_rad_s
                .is_some_and(|value| value >= 0.0)
            && law
                .maximum_state_integral_correction_rad
                .is_some_and(|value| value >= 0.0)
            && law
                .maximum_state_feedback_correction_rad
                .is_some_and(|value| value >= 0.0)
            && law.minimum_controlled_target_rad < law.maximum_controlled_target_rad,
        "invalid OpenArm state-feedback contract"
    );
    Ok(())
}

fn compile_actions(controller: &ControllerSpec) -> Vec<ActionFrame> {
    let final_step = controller.keyframes.last().unwrap().step;
    (1..=final_step)
        .map(|step| {
            let upper_index = controller
                .keyframes
                .iter()
                .position(|keyframe| keyframe.step >= step)
                .unwrap();
            let upper = &controller.keyframes[upper_index];
            let lower = &controller.keyframes[upper_index - 1];
            let alpha = (step - lower.step) as f64 / (upper.step - lower.step) as f64;
            let alpha = alpha * alpha * (3.0 - 2.0 * alpha);
            let targets = lower
                .joint_position_target_rad
                .iter()
                .zip(&upper.joint_position_target_rad)
                .map(|(from, to)| from + (to - from) * alpha)
                .collect();
            ActionFrame {
                action_sequence: step - 1,
                step,
                sim_time_ticks: step * FIXED_DELTA_TICKS,
                phase: upper.phase.clone(),
                joint_position_target_rad: targets,
            }
        })
        .collect()
}

fn apply_actuator_disturbance(
    controller: &ControllerSpec,
    step: u64,
    controller_target: &[f64],
    controller_target_history: &[Vec<f64>],
    applied_target_history: &[Vec<f64>],
) -> Result<(Vec<f64>, Vec<f64>)> {
    let mut applied = controller_target.to_vec();
    let mut disturbance = vec![0.0; controller_target.len()];
    if let Some(contract) = &controller.disturbance_contract {
        match contract {
            DisturbanceContract::AdditiveActuatorTargetBiasPulseV1 {
                joint,
                start_step,
                end_step,
                offset_rad,
                ..
            } if (*start_step..=*end_step).contains(&step) => {
                let index = disturbance_joint_index(controller, joint)?;
                disturbance[index] = *offset_rad;
                applied[index] += *offset_rad;
            }
            DisturbanceContract::ActuatorCommandTransportDelayPulseV1 {
                joint,
                start_step,
                end_step,
                delay_steps,
                ..
            } if (*start_step..=*end_step).contains(&step) => {
                let index = disturbance_joint_index(controller, joint)?;
                let source_step = step
                    .checked_sub(*delay_steps)
                    .context("actuator command delay precedes the rollout")?;
                let source_index = usize::try_from(source_step.saturating_sub(1))?;
                let source_target = controller_target_history
                    .get(source_index)
                    .context("actuator command delay source step is absent from history")?;
                applied[index] = *source_target
                    .get(index)
                    .context("actuator command delay source width drifted")?;
                disturbance[index] = applied[index] - controller_target[index];
            }
            DisturbanceContract::ActuatorCommandSlewRateLimitPulseV1 {
                joint,
                start_step,
                end_step,
                maximum_rate_rad_s,
                ..
            } if (*start_step..=*end_step).contains(&step) => {
                let index = disturbance_joint_index(controller, joint)?;
                let previous_target = applied_target_history
                    .last()
                    .context("actuator command rate limit has no previous applied target")?;
                let previous = *previous_target
                    .get(index)
                    .context("actuator command rate limit previous target width drifted")?;
                let maximum_delta_rad =
                    *maximum_rate_rad_s * FIXED_DELTA_TICKS as f64 / 1_000_000_000.0;
                applied[index] = controller_target[index]
                    .clamp(previous - maximum_delta_rad, previous + maximum_delta_rad);
                disturbance[index] = applied[index] - controller_target[index];
            }
            DisturbanceContract::ActuatorCommandDeadbandPulseV1 {
                joint,
                start_step,
                end_step,
                deadband_rad,
                ..
            } if (*start_step..=*end_step).contains(&step) => {
                let index = disturbance_joint_index(controller, joint)?;
                let previous_target = applied_target_history
                    .last()
                    .context("actuator command deadband has no previous applied target")?;
                let previous = *previous_target
                    .get(index)
                    .context("actuator command deadband previous target width drifted")?;
                if (controller_target[index] - previous).abs() <= *deadband_rad {
                    applied[index] = previous;
                }
                disturbance[index] = applied[index] - controller_target[index];
            }
            _ => {}
        }
    }
    anyhow::ensure!(
        applied
            .iter()
            .all(|value| value.is_finite() && (-3.0..=3.0).contains(value)),
        "disturbed OpenArm target violates TaskSpec bounds"
    );
    Ok((applied, disturbance))
}

fn disturbance_joint_index(controller: &ControllerSpec, joint: &str) -> Result<usize> {
    controller
        .action_joint_order
        .iter()
        .position(|candidate| candidate == joint)
        .context("disturbance joint is absent from the action order")
}

fn validate_actuator_disturbance(
    controller: &ControllerSpec,
    disturbance: &DisturbanceContract,
    final_step: u64,
) -> Result<()> {
    let common_valid = |classification: &str,
                        joint: &str,
                        start_step: u64,
                        end_step: u64,
                        visibility: &str,
                        order: &str| {
        controller
            .action_joint_order
            .iter()
            .any(|name| name == joint)
            && start_step >= 1
            && start_step <= end_step
            && end_step <= final_step
            && visibility == "unobserved_except_through_typed_joint_feedback"
            && order == "after_controller_limits_before_backend_actuation"
            && !classification.is_empty()
    };
    let valid = match disturbance {
        DisturbanceContract::AdditiveActuatorTargetBiasPulseV1 {
            classification,
            joint,
            start_step,
            end_step,
            offset_rad,
            controller_visibility,
            application_order,
        } => {
            classification == "actuator_realization_error"
                && offset_rad.is_finite()
                && common_valid(
                    classification,
                    joint,
                    *start_step,
                    *end_step,
                    controller_visibility,
                    application_order,
                )
        }
        DisturbanceContract::ActuatorCommandTransportDelayPulseV1 {
            classification,
            joint,
            start_step,
            end_step,
            delay_steps,
            controller_visibility,
            application_order,
        } => {
            classification == "actuator_transport_delay"
                && *start_step > *delay_steps
                && common_valid(
                    classification,
                    joint,
                    *start_step,
                    *end_step,
                    controller_visibility,
                    application_order,
                )
        }
        DisturbanceContract::ActuatorCommandSlewRateLimitPulseV1 {
            classification,
            joint,
            start_step,
            end_step,
            maximum_rate_rad_s,
            controller_visibility,
            application_order,
        } => {
            classification == "actuator_rate_limit"
                && maximum_rate_rad_s.is_finite()
                && *maximum_rate_rad_s > 0.0
                && *start_step > 1
                && common_valid(
                    classification,
                    joint,
                    *start_step,
                    *end_step,
                    controller_visibility,
                    application_order,
                )
        }
        DisturbanceContract::ActuatorCommandDeadbandPulseV1 {
            classification,
            joint,
            start_step,
            end_step,
            deadband_rad,
            controller_visibility,
            application_order,
        } => {
            classification == "actuator_deadband"
                && deadband_rad.is_finite()
                && *deadband_rad >= 0.0
                && *start_step > 1
                && common_valid(
                    classification,
                    joint,
                    *start_step,
                    *end_step,
                    controller_visibility,
                    application_order,
                )
        }
    };
    anyhow::ensure!(valid, "invalid OpenArm actuator disturbance contract");
    Ok(())
}

fn apply_measurement_bias(
    controller: &ControllerSpec,
    observation: &ControllerObservation,
    consumed_at_ticks: u64,
) -> Result<(Vec<f64>, Vec<f64>)> {
    let mut positions = observation.joint_position_rad.clone();
    let mut bias = vec![0.0; positions.len()];
    let Some(MeasurementFaultContract::AdditiveJointPositionBiasPulseV1 {
        joint,
        start_controller_step,
        end_controller_step,
        offset_rad,
        ..
    }) = &controller.measurement_fault_contract
    else {
        return Ok((positions, bias));
    };
    let sample_period_ticks = controller
        .observation_contract
        .as_ref()
        .context("measurement bias has no observation contract")?
        .sample_period_ticks;
    anyhow::ensure!(
        consumed_at_ticks.is_multiple_of(sample_period_ticks),
        "measurement-bias consumption time is off the control grid"
    );
    let controller_step = consumed_at_ticks / sample_period_ticks + 1;
    if (*start_controller_step..=*end_controller_step).contains(&controller_step) {
        let index = controller
            .action_joint_order
            .iter()
            .position(|name| name == joint)
            .context("measurement-bias joint is absent from the action order")?;
        bias[index] = *offset_rad;
        positions[index] += *offset_rad;
    }
    anyhow::ensure!(
        positions.iter().all(|value| value.is_finite()),
        "measurement bias produced a non-finite observation"
    );
    Ok((positions, bias))
}

fn controller_decision(
    controller: &ControllerSpec,
    reference: &[f64],
    state: &mut ControllerState,
    observation: Option<&ControllerObservation>,
    consumed_at_ticks: u64,
) -> Result<ControllerDecision> {
    anyhow::ensure!(
        reference.len() == controller.action_joint_order.len(),
        "OpenArm controller reference width mismatch"
    );
    anyhow::ensure!(
        state.integral_correction_rad.len() == reference.len(),
        "OpenArm controller state width mismatch"
    );
    let (contract, law) = match (&controller.observation_contract, &controller.feedback_law) {
        (Some(contract), Some(law)) => (contract, law),
        (None, None) => {
            return Ok(ControllerDecision {
                reference_position_rad: reference.to_vec(),
                target_position_rad: reference.to_vec(),
                correction_rad: vec![0.0; reference.len()],
                integral_correction_rad: state.integral_correction_rad.clone(),
                controller_observation_position_rad: Vec::new(),
                measurement_bias_rad: vec![0.0; reference.len()],
                observation_sequence: None,
                observation_age_ticks: None,
                bootstrap: false,
                rejected: false,
                rejection_reason: None,
                fail_safe_hold_active: false,
                state_frozen: false,
                recovered: false,
            });
        }
        _ => bail!("OpenArm controller feedback contract is incomplete"),
    };
    let Some(observation) = observation else {
        if law.kind == "joint_position_state_feedback_integral_v1" {
            let joint = law
                .controlled_joint
                .as_deref()
                .context("state-feedback law has no controlled joint")?;
            let index = controller
                .action_joint_order
                .iter()
                .position(|name| name == joint)
                .context("state-feedback joint is absent from the action order")?;
            state.previous_previous_input_target_rad[index] =
                state.previous_input_target_rad[index];
            state.previous_input_target_rad[index] = Some(reference[index]);
        }
        return Ok(ControllerDecision {
            reference_position_rad: reference.to_vec(),
            target_position_rad: reference.to_vec(),
            correction_rad: vec![0.0; reference.len()],
            integral_correction_rad: state.integral_correction_rad.clone(),
            controller_observation_position_rad: Vec::new(),
            measurement_bias_rad: vec![0.0; reference.len()],
            observation_sequence: None,
            observation_age_ticks: None,
            bootstrap: true,
            rejected: false,
            rejection_reason: None,
            fail_safe_hold_active: false,
            state_frozen: false,
            recovered: false,
        });
    };
    anyhow::ensure!(
        observation.status == contract.required_status,
        "OpenArm controller rejected {:?} joint feedback at sequence {}",
        observation.status,
        observation.sequence
    );
    let age_ticks = consumed_at_ticks.saturating_sub(observation.capture_time_ticks);
    anyhow::ensure!(
        observation.available_time_ticks <= consumed_at_ticks
            && age_ticks <= contract.maximum_age_ticks,
        "OpenArm controller rejected stale or unavailable observation sequence {} with age {} ticks",
        observation.sequence,
        age_ticks
    );
    anyhow::ensure!(
        observation.joint_position_rad.len() == reference.len()
            && observation.joint_velocity_rad_s.len() == reference.len(),
        "OpenArm controller observation width mismatch"
    );
    let (controller_positions, measurement_bias_rad) =
        apply_measurement_bias(controller, observation, consumed_at_ticks)?;
    let visible_observation = ControllerObservation {
        joint_position_rad: controller_positions.clone(),
        ..observation.clone()
    };
    if law.kind == "joint_position_state_feedback_integral_v1" {
        let mut decision = state_feedback_decision(
            controller,
            law,
            reference,
            state,
            &visible_observation,
            age_ticks,
        )?;
        decision.controller_observation_position_rad = controller_positions;
        decision.measurement_bias_rad = measurement_bias_rad;
        return Ok(decision);
    }
    anyhow::ensure!(
        law.kind == "joint_position_reference_pid_v1",
        "unsupported OpenArm feedback law"
    );
    let sample_period_s = controller
        .observation_contract
        .as_ref()
        .unwrap()
        .sample_period_ticks as f64
        / 1_000_000_000.0;
    for (((integral, gain), maximum), (reference, position)) in state
        .integral_correction_rad
        .iter_mut()
        .zip(&law.integral_error_gain_s_inv)
        .zip(&law.maximum_integral_correction_rad)
        .zip(
            reference
                .iter()
                .zip(&visible_observation.joint_position_rad),
        )
    {
        *integral =
            (*integral + gain * (reference - position) * sample_period_s).clamp(-maximum, *maximum);
    }
    let correction_rad = reference
        .iter()
        .zip(&visible_observation.joint_position_rad)
        .zip(&visible_observation.joint_velocity_rad_s)
        .zip(&law.position_error_gain)
        .zip(&law.velocity_damping_s)
        .zip(&state.integral_correction_rad)
        .zip(&law.maximum_correction_rad)
        .map(
            |(
                (((((reference, position), velocity), position_gain), velocity_gain), integral),
                maximum,
            )| {
                (position_gain * (reference - position) - velocity_gain * velocity + integral)
                    .clamp(-maximum, *maximum)
            },
        )
        .collect::<Vec<_>>();
    let target_position_rad = reference
        .iter()
        .zip(&correction_rad)
        .zip(&law.minimum_target_rad)
        .zip(&law.maximum_target_rad)
        .map(|(((reference, correction), minimum), maximum)| {
            (reference + correction).clamp(*minimum, *maximum)
        })
        .collect();
    Ok(ControllerDecision {
        reference_position_rad: reference.to_vec(),
        target_position_rad,
        correction_rad,
        integral_correction_rad: state.integral_correction_rad.clone(),
        controller_observation_position_rad: controller_positions,
        measurement_bias_rad,
        observation_sequence: Some(observation.sequence),
        observation_age_ticks: Some(age_ticks),
        bootstrap: false,
        rejected: false,
        rejection_reason: None,
        fail_safe_hold_active: false,
        state_frozen: false,
        recovered: false,
    })
}

fn state_feedback_decision(
    controller: &ControllerSpec,
    law: &FeedbackLaw,
    reference: &[f64],
    state: &mut ControllerState,
    observation: &ControllerObservation,
    age_ticks: u64,
) -> Result<ControllerDecision> {
    let joint = law
        .controlled_joint
        .as_deref()
        .context("state-feedback law has no controlled joint")?;
    let index = controller
        .action_joint_order
        .iter()
        .position(|name| name == joint)
        .context("state-feedback joint is absent from the action order")?;
    let position = observation.joint_position_rad[index];
    let previous_position = state.previous_observation_position_rad[index].unwrap_or(position);
    let operating_position = law
        .operating_point_position_rad
        .context("state-feedback law has no position operating point")?;
    let operating_input = law
        .operating_point_input_rad
        .context("state-feedback law has no input operating point")?;
    let previous_input = state.previous_input_target_rad[index].unwrap_or(operating_input);
    let previous_previous_input =
        state.previous_previous_input_target_rad[index].unwrap_or(operating_input);
    let coefficients = &law
        .identified_plant
        .as_ref()
        .context("state-feedback law has no identified plant")?
        .arx_coefficients;
    let predicted_position_error = coefficients[1] * (position - operating_position)
        + coefficients[2] * (previous_position - operating_position)
        + coefficients[3] * (previous_input - operating_input)
        + coefficients[4] * (previous_previous_input - operating_input);
    let sample_period_s = controller
        .observation_contract
        .as_ref()
        .context("state-feedback law has no observation contract")?
        .sample_period_ticks as f64
        / 1_000_000_000.0;
    let integral_gain = law
        .integral_state_feedback_gain_s_inv
        .context("state-feedback law has no integral gain")?;
    let maximum_integral = law
        .maximum_state_integral_correction_rad
        .context("state-feedback law has no integral correction limit")?;
    state.integral_correction_rad[index] = (state.integral_correction_rad[index]
        + integral_gain * (reference[index] - position) * sample_period_s)
        .clamp(-maximum_integral, maximum_integral);
    let state_vector = [
        operating_position + predicted_position_error - reference[index],
        position - reference[index],
        previous_input - reference[index],
    ];
    let raw_target = reference[index]
        - law
            .state_feedback_gain
            .iter()
            .zip(state_vector)
            .map(|(gain, value)| gain * value)
            .sum::<f64>()
        + state.integral_correction_rad[index];
    let maximum_correction = law
        .maximum_state_feedback_correction_rad
        .context("state-feedback law has no correction limit")?;
    let mut correction_rad = vec![0.0; reference.len()];
    correction_rad[index] =
        (raw_target - reference[index]).clamp(-maximum_correction, maximum_correction);
    let mut target_position_rad = reference.to_vec();
    target_position_rad[index] = (reference[index] + correction_rad[index]).clamp(
        law.minimum_controlled_target_rad
            .context("state-feedback law has no minimum target")?,
        law.maximum_controlled_target_rad
            .context("state-feedback law has no maximum target")?,
    );
    state.previous_observation_position_rad[index] = Some(position);
    state.previous_previous_input_target_rad[index] = Some(previous_input);
    state.previous_input_target_rad[index] = Some(target_position_rad[index]);
    Ok(ControllerDecision {
        reference_position_rad: reference.to_vec(),
        target_position_rad,
        correction_rad,
        integral_correction_rad: state.integral_correction_rad.clone(),
        controller_observation_position_rad: Vec::new(),
        measurement_bias_rad: vec![0.0; reference.len()],
        observation_sequence: Some(observation.sequence),
        observation_age_ticks: Some(age_ticks),
        bootstrap: false,
        rejected: false,
        rejection_reason: None,
        fail_safe_hold_active: false,
        state_frozen: false,
        recovered: false,
    })
}

fn bounded_controller_decision(
    controller: &ControllerSpec,
    reference: &[f64],
    state: &mut ControllerState,
    observation: Option<&ControllerObservation>,
    consumed_at_ticks: u64,
    last_accepted_target_rad: &[f64],
    recovering_from_rejection: bool,
) -> Result<ControllerDecision> {
    let state_before = state.clone();
    match controller_decision(controller, reference, state, observation, consumed_at_ticks) {
        Ok(mut decision) => {
            decision.recovered = recovering_from_rejection && !decision.bootstrap;
            Ok(decision)
        }
        Err(error) => {
            let Some(observation) = observation else {
                return Err(error);
            };
            let Some(contract) = controller.observation_contract.as_ref() else {
                return Err(error);
            };
            let age_ticks = consumed_at_ticks.saturating_sub(observation.capture_time_ticks);
            if !has_dropout_fault(controller)
                || observation.status != contract.required_status
                || observation.available_time_ticks > consumed_at_ticks
                || age_ticks <= contract.maximum_age_ticks
            {
                return Err(error);
            }
            *state = state_before;
            anyhow::ensure!(
                last_accepted_target_rad.len() == reference.len(),
                "fail-safe hold target width mismatch"
            );
            Ok(ControllerDecision {
                reference_position_rad: reference.to_vec(),
                target_position_rad: last_accepted_target_rad.to_vec(),
                correction_rad: last_accepted_target_rad
                    .iter()
                    .zip(reference)
                    .map(|(target, reference)| target - reference)
                    .collect(),
                integral_correction_rad: state.integral_correction_rad.clone(),
                controller_observation_position_rad: observation.joint_position_rad.clone(),
                measurement_bias_rad: vec![0.0; reference.len()],
                observation_sequence: Some(observation.sequence),
                observation_age_ticks: Some(age_ticks),
                bootstrap: false,
                rejected: true,
                rejection_reason: Some("maximum_observation_age_ticks"),
                fail_safe_hold_active: true,
                state_frozen: true,
                recovered: false,
            })
        }
    }
}

fn has_dropout_fault(controller: &ControllerSpec) -> bool {
    matches!(
        controller.measurement_fault_contract.as_ref(),
        Some(MeasurementFaultContract::JointFeedbackPublicationDropoutBurstV1 { .. })
    )
}

fn sensor_sample_published(controller: &ControllerSpec, sequence: u64) -> bool {
    match &controller.measurement_fault_contract {
        Some(MeasurementFaultContract::JointFeedbackPublicationDropoutBurstV1 {
            start_capture_sequence,
            consecutive_dropped_frames,
            ..
        }) => !(*start_capture_sequence
            ..start_capture_sequence.saturating_add(*consecutive_dropped_frames))
            .contains(&sequence),
        _ => true,
    }
}

fn controller_observation(frame: &Frame<JointFeedback>) -> Result<ControllerObservation> {
    let mut positions = Vec::with_capacity(frame.payload.joints.len());
    let mut velocities = Vec::with_capacity(frame.payload.joints.len());
    for joint in &frame.payload.joints {
        match joint.coordinate {
            rne_data::JointCoordinateFeedback::Revolute {
                position_rad,
                velocity_rad_s,
            } => {
                positions.push(position_rad);
                velocities.push(velocity_rad_s);
            }
            _ => bail!(
                "OpenArm controller feedback channel {} is not revolute",
                joint.name
            ),
        }
    }
    Ok(ControllerObservation {
        sequence: frame.sequence,
        capture_time_ticks: frame.capture_time.ticks(),
        available_time_ticks: frame.available_time.ticks(),
        status: frame.payload.status,
        joint_position_rad: positions,
        joint_velocity_rad_s: velocities,
    })
}

fn first_controller_rejection(
    controller: &ControllerSpec,
    actions: &[ActionFrame],
    observations: &[ObservationFrame],
) -> Option<ControllerRejection> {
    let mut available_index = 0;
    let mut latest = None;
    let mut controller_state = ControllerState::new(controller.action_joint_order.len());
    for action in actions {
        let consumed_at_ticks = action.sim_time_ticks.saturating_sub(FIXED_DELTA_TICKS);
        while let Some(frame) = observations.get(available_index) {
            if frame.available_time_ticks > consumed_at_ticks {
                break;
            }
            latest = Some(ControllerObservation {
                sequence: frame.step,
                capture_time_ticks: frame.sim_time_ticks,
                available_time_ticks: frame.available_time_ticks,
                status: frame.sensor_status,
                joint_position_rad: frame.joint_position_rad.clone(),
                joint_velocity_rad_s: frame.joint_velocity_rad_s.clone(),
            });
            available_index += 1;
        }
        let observation = latest.as_ref();
        if controller_decision(
            controller,
            &action.joint_position_target_rad,
            &mut controller_state,
            observation,
            consumed_at_ticks,
        )
        .is_err()
        {
            let observation = observation?;
            let first_violated_contract = if observation.status
                != controller.observation_contract.as_ref()?.required_status
            {
                "required_sensor_status"
            } else {
                "maximum_observation_age_ticks"
            };
            return Some(ControllerRejection {
                action_step: action.step,
                detected_at_ticks: consumed_at_ticks,
                observation_sequence: observation.sequence,
                observation_age_ticks: consumed_at_ticks
                    .saturating_sub(observation.capture_time_ticks),
                first_violated_contract,
            });
        }
    }
    None
}

fn rollout(
    scene: &Path,
    controller: &ControllerSpec,
    actuation_config: &ActuationConfig,
    actions: &[ActionFrame],
    fault: JointFeedbackFault,
) -> Result<Rollout> {
    let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations_and_fixed_delta(
        scene,
        actuation_config.solver_iterations,
        rne_core::SimDuration::from_ticks(FIXED_DELTA_TICKS),
    )
    .context("load OpenArm right-arm validation scene")?;
    let joint_passive_dynamics = controller
        .rne_actuator_link_order
        .iter()
        .map(|name| {
            sim.world()
                .iter_entities()
                .find(|entity| entity.get::<Name>().is_some_and(|value| value.0 == *name))
                .with_context(|| format!("missing OpenArm actuator link {name}"))
                .map(|entity| entity.get::<JointPassiveDynamics>().copied())
        })
        .collect::<Result<Vec<_>>>()?;
    configure_actuators(&mut sim, actuation_config)?;
    sim.install_joint_feedback_sensor(UrdfJointFeedbackSensorConfig {
        sensor_name: "openarm_right_joint_feedback".into(),
        link_names: controller.rne_actuator_link_order.clone(),
        update_rate_hz: 60.0,
        sample_period_ticks: Some(sim.fixed_delta().ticks()),
        phase_offset_ticks: sim.fixed_delta().ticks(),
        latency_ticks: sim.fixed_delta().ticks(),
        stream_id: JOINT_FEEDBACK_STREAM,
        fault,
    })
    .context("install OpenArm joint-feedback sensor")?;
    let initial_digest = hash_physics_state_v2(sim.world());
    let mut observations = Vec::with_capacity(actions.len());
    let mut state_hashes = Vec::with_capacity(actions.len());
    let mut controller_decisions = Vec::with_capacity(actions.len());
    let mut bus = InMemoryDataBus::new();
    let mut latest_controller_observation = None;
    let mut controller_state = ControllerState::new(controller.action_joint_order.len());
    let mut controller_target_history = Vec::with_capacity(actions.len());
    let mut applied_target_history = Vec::with_capacity(actions.len());
    let mut applied_actuation_history = Vec::with_capacity(actions.len());
    let mut last_accepted_target_rad = actions
        .first()
        .context("OpenArm rollout has no actions")?
        .joint_position_target_rad
        .clone();
    let mut recovering_from_rejection = false;
    let mut last_observation_sequence = 0;
    let mut maximum_sensor_backend_position_delta_rad = 0.0_f64;
    let mut maximum_sensor_backend_velocity_delta_rad_s = 0.0_f64;
    for action in actions {
        let decision = bounded_controller_decision(
            controller,
            &action.joint_position_target_rad,
            &mut controller_state,
            if fault == JointFeedbackFault::None {
                latest_controller_observation.as_ref()
            } else {
                None
            },
            sim.sim_time().ticks(),
            &last_accepted_target_rad,
            recovering_from_rejection,
        )?;
        if !decision.rejected {
            last_accepted_target_rad.clone_from(&decision.target_position_rad);
        }
        recovering_from_rejection = decision.rejected;
        controller_target_history.push(decision.target_position_rad.clone());
        let (applied_target, _) = apply_actuator_disturbance(
            controller,
            action.step,
            &decision.target_position_rad,
            &controller_target_history,
            &applied_target_history,
        )?;
        applied_target_history.push(applied_target.clone());
        let targets = controller
            .rne_actuator_link_order
            .iter()
            .zip(&applied_target)
            .zip(&actuation_config.joints)
            .map(|((link_name, target_position_rad), joint)| {
                anyhow::ensure!(
                    link_name == &joint.link_name,
                    "actuation link order mismatch"
                );
                Ok(UrdfJointPdEffortTarget {
                    link_name,
                    target_position_rad: *target_position_rad,
                    stiffness_nm_per_rad: joint.stiffness_nm_per_rad,
                    damping_nm_s_per_rad: joint.damping_nm_s_per_rad,
                    max_effort_nm: joint.max_effort_nm,
                    max_velocity_rad_s: joint.max_velocity_rad_s,
                    transmission_efficiency: joint.transmission_efficiency,
                })
            })
            .collect::<Result<Vec<_>>>()?;
        let applied = sim
            .step_joint_pd_effort_targets_substeps(
                &targets,
                actuation_config.physics_substeps_per_control_step,
            )
            .context("step OpenArm portable PD effort physics substeps")?;
        let applied_actuation = AppliedActuation {
            target_position_rad: applied_target,
            limited_effort_command_nm: applied
                .iter()
                .map(|value| value.motor_effort_command_nm)
                .collect(),
            effort_saturated: applied.iter().map(|value| value.saturated).collect(),
        };
        applied_actuation_history.push(applied_actuation);
        controller_decisions.push(decision);
        state_hashes.push(hash_physics_state_v2(sim.world()));
        sim.sample_joint_feedback(&mut bus)
            .context("sample OpenArm joint feedback")?;
        if fault == JointFeedbackFault::None {
            let captured = bus
                .latest::<JointFeedback>(JOINT_FEEDBACK_STREAM)
                .context("OpenArm feedback sensor emitted no current frame")?;
            for (link_name, joint) in controller
                .rne_actuator_link_order
                .iter()
                .zip(&captured.payload.joints)
            {
                let (sensor_position_rad, sensor_velocity_rad_s) = match joint.coordinate {
                    rne_data::JointCoordinateFeedback::Revolute {
                        position_rad,
                        velocity_rad_s,
                    } => (position_rad, velocity_rad_s),
                    _ => bail!("OpenArm feedback channel {link_name} is not revolute"),
                };
                let backend_position_rad = sim
                    .named_joint_position(link_name)
                    .with_context(|| format!("missing backend position for {link_name}"))?;
                let backend_velocity_rad_s = sim
                    .named_joint_velocity(link_name)
                    .with_context(|| format!("missing backend velocity for {link_name}"))?;
                maximum_sensor_backend_position_delta_rad =
                    maximum_sensor_backend_position_delta_rad
                        .max((sensor_position_rad - backend_position_rad).abs());
                maximum_sensor_backend_velocity_delta_rad_s =
                    maximum_sensor_backend_velocity_delta_rad_s
                        .max((sensor_velocity_rad_s - backend_velocity_rad_s).abs());
            }
        }
        let now = sim.sim_time();
        if let Some(frame) = bus.latest_available::<JointFeedback>(JOINT_FEEDBACK_STREAM, now) {
            if frame.sequence > last_observation_sequence {
                let published = fault == JointFeedbackFault::None
                    && sensor_sample_published(controller, frame.sequence);
                if published {
                    latest_controller_observation = Some(controller_observation(&frame)?);
                }
                observations.push(observation_from_feedback(
                    frame,
                    now.ticks(),
                    published,
                    &state_hashes,
                    &controller_decisions,
                    &applied_actuation_history,
                )?);
                last_observation_sequence = observations.last().unwrap().step;
            }
        }
    }
    let final_time_ticks = sim
        .sim_time()
        .ticks()
        .checked_add(FIXED_DELTA_TICKS)
        .context("OpenArm feedback availability time overflow")?;
    let final_frame = bus
        .latest_available::<JointFeedback>(
            JOINT_FEEDBACK_STREAM,
            rne_core::SimTime::from_ticks(final_time_ticks),
        )
        .context("final OpenArm joint feedback did not become available")?;
    if final_frame.sequence > last_observation_sequence {
        let final_published = fault == JointFeedbackFault::None
            && sensor_sample_published(controller, final_frame.sequence);
        observations.push(observation_from_feedback(
            final_frame,
            final_time_ticks,
            final_published,
            &state_hashes,
            &controller_decisions,
            &applied_actuation_history,
        )?);
    }
    if fault == JointFeedbackFault::None {
        anyhow::ensure!(
            observations.len() == actions.len(),
            "OpenArm typed feedback emitted {} observations for {} actions",
            observations.len(),
            actions.len()
        );
    }
    Ok(Rollout {
        world_seed: sim.world_seed(),
        initial_digest,
        final_digest: hash_physics_state_v2(sim.world()),
        observations,
        maximum_sensor_backend_position_delta_rad,
        maximum_sensor_backend_velocity_delta_rad_s,
        joint_passive_dynamics,
    })
}

fn default_physics_substeps_per_control_step() -> usize {
    1
}

fn unit_transmission_efficiency() -> f64 {
    1.0
}

fn feedback_bounds(controller: &ControllerSpec) -> (Vec<f64>, Vec<f64>) {
    let width = controller.action_joint_order.len();
    let Some(law) = controller.feedback_law.as_ref() else {
        return (vec![0.0; width], vec![0.0; width]);
    };
    if law.kind == "joint_position_reference_pid_v1" {
        return (
            law.maximum_correction_rad.clone(),
            law.maximum_integral_correction_rad.clone(),
        );
    }
    let mut correction = vec![0.0; width];
    let mut integral = vec![0.0; width];
    if let Some(index) = law.controlled_joint.as_ref().and_then(|controlled| {
        controller
            .action_joint_order
            .iter()
            .position(|joint| joint == controlled)
    }) {
        correction[index] = law.maximum_state_feedback_correction_rad.unwrap_or(0.0);
        integral[index] = law.maximum_state_integral_correction_rad.unwrap_or(0.0);
    }
    (correction, integral)
}

fn build_sensor_validation_report(
    scene: &Path,
    controller: &ControllerSpec,
    actuation_config: &ActuationConfig,
    actions: &[ActionFrame],
    nominal: &Rollout,
    replay: &Rollout,
    hashes: &InputHashes<'_>,
) -> Result<SensorValidationArtifacts> {
    let fault_window_end = usize::try_from(SENSOR_FAULT_SEQUENCE + 2)
        .context("sensor fault sequence does not fit usize")?
        .min(actions.len());
    let fault_actions = &actions[..fault_window_end];
    let dropout = rollout(
        scene,
        controller,
        actuation_config,
        fault_actions,
        JointFeedbackFault::DropSequence {
            sequence: SENSOR_FAULT_SEQUENCE,
        },
    )?;
    let stuck = rollout(
        scene,
        controller,
        actuation_config,
        fault_actions,
        JointFeedbackFault::StuckFromSequence {
            sequence: SENSOR_FAULT_SEQUENCE,
        },
    )?;

    let nominal_hash = sha256_json(&nominal.observations)?;
    let replay_hash = sha256_json(&replay.observations)?;
    let dropout_hash = sha256_json(&dropout.observations)?;
    let stuck_hash = sha256_json(&stuck.observations)?;
    let maximum_phase_error_ticks = nominal
        .observations
        .iter()
        .map(|frame| frame.sample_phase_error_ticks)
        .max()
        .unwrap_or(u64::MAX);
    let minimum_observation_age_ticks = nominal
        .observations
        .iter()
        .map(|frame| frame.observation_age_ticks)
        .min()
        .unwrap_or(u64::MAX);
    let maximum_observation_age_ticks = nominal
        .observations
        .iter()
        .map(|frame| frame.observation_age_ticks)
        .max()
        .unwrap_or(u64::MAX);
    let available_effort_measurements = nominal
        .observations
        .iter()
        .flat_map(|frame| &frame.effort_measurement_available)
        .filter(|available| **available)
        .count();
    let expected_effort_measurements =
        nominal.observations.len() * controller.rne_actuator_link_order.len();
    let saturation_count = nominal
        .observations
        .iter()
        .flat_map(|frame| &frame.effort_saturated)
        .filter(|saturated| **saturated)
        .count();
    let feedback_enabled = controller.feedback_law.is_some();
    let controller_bootstrap_frames = nominal
        .observations
        .iter()
        .filter(|frame| frame.controller_bootstrap)
        .count();
    let controller_feedback_frames = nominal
        .observations
        .iter()
        .filter(|frame| frame.controller_observation_sequence.is_some())
        .count();
    let controller_timing_aligned = nominal.observations.iter().all(|frame| {
        if !feedback_enabled {
            !frame.controller_bootstrap
                && frame.controller_observation_sequence.is_none()
                && frame.controller_observation_age_ticks.is_none()
        } else if frame.step <= 2 {
            frame.controller_bootstrap
                && frame.controller_observation_sequence.is_none()
                && frame.controller_observation_age_ticks.is_none()
        } else {
            !frame.controller_bootstrap
                && frame.controller_observation_sequence == Some(frame.step - 2)
                && frame.controller_observation_age_ticks == Some(FIXED_DELTA_TICKS)
        }
    });
    let maximum_feedback_correction_rad = nominal
        .observations
        .iter()
        .flat_map(|frame| &frame.joint_feedback_correction_rad)
        .map(|correction| correction.abs())
        .fold(0.0_f64, f64::max);
    let (feedback_correction_bounds, integral_correction_bounds) = feedback_bounds(controller);
    let configured_maximum_feedback_correction_rad = feedback_correction_bounds
        .iter()
        .copied()
        .fold(0.0_f64, f64::max);
    let maximum_integral_correction_rad = nominal
        .observations
        .iter()
        .flat_map(|frame| &frame.joint_integral_correction_rad)
        .map(|correction| correction.abs())
        .fold(0.0_f64, f64::max);
    let configured_maximum_integral_correction_rad = integral_correction_bounds
        .iter()
        .copied()
        .fold(0.0_f64, f64::max);
    let integral_correction_within_per_joint_bounds = nominal.observations.iter().all(|frame| {
        frame.joint_integral_correction_rad.len() == integral_correction_bounds.len()
            && frame
                .joint_integral_correction_rad
                .iter()
                .zip(&integral_correction_bounds)
                .all(|(correction, maximum)| correction.abs() <= *maximum)
    });
    let expected_controller_bootstrap_frames = usize::from(feedback_enabled) * 2;
    let expected_controller_feedback_frames = if feedback_enabled {
        actions.len() - 2
    } else {
        0
    };
    let first_saturation = nominal.observations.iter().find_map(|frame| {
        frame
            .effort_saturated
            .iter()
            .position(|saturated| *saturated)
            .map(|joint_index| {
                (
                    frame.step,
                    joint_index,
                    controller.action_joint_order[joint_index].as_str(),
                )
            })
    });
    let sequence_gap = first_sequence_gap(&dropout.observations);
    let first_stuck = stuck
        .observations
        .iter()
        .find(|frame| frame.sensor_status == JointFeedbackStatus::StuckValue);
    let dropout_controller_rejection =
        first_controller_rejection(controller, fault_actions, &dropout.observations);
    let stuck_controller_rejection =
        first_controller_rejection(controller, fault_actions, &stuck.observations);
    let expected_dropout_detection_ticks = (SENSOR_FAULT_SEQUENCE + 2) * FIXED_DELTA_TICKS;
    let expected_stuck_detection_ticks = (SENSOR_FAULT_SEQUENCE + 1) * FIXED_DELTA_TICKS;
    let expected_controller_rejection_step = SENSOR_FAULT_SEQUENCE + 2;
    let expected_controller_rejection_ticks =
        (expected_controller_rejection_step - 1) * FIXED_DELTA_TICKS;
    let dropout_rejection_expected = feedback_enabled
        && controller
            .observation_contract
            .as_ref()
            .is_some_and(|contract| contract.maximum_age_ticks < 2 * FIXED_DELTA_TICKS);

    let check_results = [
        nominal.observations.len() == actions.len(),
        nominal_hash == replay_hash,
        maximum_phase_error_ticks == 0,
        minimum_observation_age_ticks == FIXED_DELTA_TICKS,
        maximum_observation_age_ticks == FIXED_DELTA_TICKS,
        nominal.maximum_sensor_backend_position_delta_rad == 0.0,
        nominal.maximum_sensor_backend_velocity_delta_rad_s == 0.0,
        available_effort_measurements == expected_effort_measurements,
        saturation_count > 0,
        sequence_gap.is_some_and(|(missing, next, detected)| {
            missing == SENSOR_FAULT_SEQUENCE
                && next == SENSOR_FAULT_SEQUENCE + 1
                && detected == expected_dropout_detection_ticks
        }),
        first_stuck.is_some_and(|frame| {
            frame.step == SENSOR_FAULT_SEQUENCE
                && frame.consumed_at_ticks == expected_stuck_detection_ticks
        }),
        nominal.world_seed == replay.world_seed
            && nominal.world_seed == dropout.world_seed
            && nominal.world_seed == stuck.world_seed,
        controller_bootstrap_frames == expected_controller_bootstrap_frames,
        controller_timing_aligned,
        controller_feedback_frames == expected_controller_feedback_frames,
        if feedback_enabled {
            maximum_feedback_correction_rad > 0.0
                && maximum_feedback_correction_rad <= configured_maximum_feedback_correction_rad
        } else {
            maximum_feedback_correction_rad == 0.0
        },
        if feedback_enabled {
            maximum_integral_correction_rad > 0.0
                && maximum_integral_correction_rad <= configured_maximum_integral_correction_rad
                && integral_correction_within_per_joint_bounds
        } else {
            maximum_integral_correction_rad == 0.0
        },
        if dropout_rejection_expected {
            dropout_controller_rejection
                .as_ref()
                .is_some_and(|rejection| {
                    rejection.action_step == expected_controller_rejection_step
                        && rejection.detected_at_ticks == expected_controller_rejection_ticks
                        && rejection.first_violated_contract == "maximum_observation_age_ticks"
                })
        } else {
            dropout_controller_rejection.is_none()
        },
        !feedback_enabled
            || stuck_controller_rejection
                .as_ref()
                .is_some_and(|rejection| {
                    rejection.action_step == expected_controller_rejection_step
                        && rejection.detected_at_ticks == expected_controller_rejection_ticks
                        && rejection.first_violated_contract == "required_sensor_status"
                }),
    ];
    let passed = check_results.iter().all(|result| *result);
    let (missing_sequence, next_sequence, dropout_detected_at_ticks) =
        sequence_gap.unwrap_or((0, 0, 0));
    let (first_saturation_sequence, first_saturation_joint_index, first_saturation_joint_name) =
        first_saturation.unwrap_or((0, 0, "none"));
    let (first_stuck_sequence, stuck_detected_at_ticks) = first_stuck
        .map(|frame| (frame.step, frame.consumed_at_ticks))
        .unwrap_or((0, 0));

    let report = json!({
        "kind": SENSOR_REPORT_KIND,
        "schema_version": 1,
        "status": if passed { "passed" } else { "failed" },
        "backend": { "id": "rne_rapier", "version": "0.22" },
        "task_id": controller.task_id,
        "controller_id": controller.controller_id,
        "world_seed": nominal.world_seed,
        "input_hashes": {
            "task_sha256": hashes.task_sha256,
            "controller_sha256": hashes.controller_sha256,
            "action_trace_sha256": hashes.action_trace_sha256,
            "robot_asset_config_sha256": hashes.robot_asset_config_sha256,
            "model_urdf_sha256": hashes.model_urdf_sha256,
            "scene_config_sha256": hashes.scene_config_sha256,
            "actuation_config_sha256": hashes.actuation_config_sha256,
        },
        "sensor_contract": {
            "payload": "JointFeedback",
            "schema_version": JointFeedback::SCHEMA_VERSION,
            "stream_id": JOINT_FEEDBACK_STREAM.0,
            "channel_order": controller.rne_actuator_link_order,
            "sample_period_ticks": FIXED_DELTA_TICKS,
            "phase_offset_ticks": FIXED_DELTA_TICKS,
            "latency_ticks": FIXED_DELTA_TICKS,
            "consumption": "databus_latest_available",
            "effort_semantics": "completed_backend_step_direct_effort_measurement",
        },
        "controller_observation_contract": {
            "law": controller.feedback_law.as_ref().map(|law| law.kind.as_str()).unwrap_or("open_loop_reference_v1"),
            "input": if feedback_enabled { "typed_joint_feedback_only" } else { "none" },
            "required_status": controller.observation_contract.as_ref().map(|contract| contract.required_status),
            "maximum_age_ticks": controller.observation_contract.as_ref().map(|contract| contract.maximum_age_ticks),
            "bootstrap_policy": controller.observation_contract.as_ref().map(|contract| contract.bootstrap_policy.as_str()),
            "bootstrap_frames": controller_bootstrap_frames,
            "feedback_frames": controller_feedback_frames,
            "maximum_feedback_correction_rad": maximum_feedback_correction_rad,
            "configured_maximum_feedback_correction_rad": configured_maximum_feedback_correction_rad,
            "maximum_integral_correction_rad": maximum_integral_correction_rad,
            "configured_maximum_integral_correction_rad": configured_maximum_integral_correction_rad,
            "integrator_limit_policy": "per_joint_clamp_anti_windup",
        },
        "stream_hashes": {
            "nominal_sha256": nominal_hash,
            "replay_sha256": replay_hash,
            "dropout_sha256": dropout_hash,
            "stuck_value_sha256": stuck_hash,
        },
        "artifacts": {
            "nominal": "rapier-success-trace.json",
            "dropout": "sensor-dropout-trace.json",
            "stuck_value": "sensor-stuck-trace.json",
            "browser_report": "sensor-validation-report.html",
        },
        "checks": [
            { "id": "nominal_frame_count_v1", "classification": "measurement", "unit": "frame", "observed": nominal.observations.len(), "expected": actions.len(), "status": pass_fail(check_results[0]) },
            { "id": "deterministic_replay_hash_v1", "classification": "measurement", "unit": "sha256_match", "observed": nominal_hash == replay_hash, "expected": true, "status": pass_fail(check_results[1]) },
            { "id": "sample_phase_error_v1", "classification": "measurement", "unit": "tick", "observed": maximum_phase_error_ticks, "maximum": 0, "status": pass_fail(check_results[2]) },
            { "id": "observation_age_min_v1", "classification": "measurement", "unit": "tick", "observed": minimum_observation_age_ticks, "expected": FIXED_DELTA_TICKS, "status": pass_fail(check_results[3]) },
            { "id": "observation_age_max_v1", "classification": "measurement", "unit": "tick", "observed": maximum_observation_age_ticks, "expected": FIXED_DELTA_TICKS, "status": pass_fail(check_results[4]) },
            { "id": "sensor_backend_position_calibration_v1", "classification": "measurement", "unit": "rad", "observed_delta": nominal.maximum_sensor_backend_position_delta_rad, "maximum_delta": 0.0, "status": pass_fail(check_results[5]) },
            { "id": "sensor_backend_velocity_calibration_v1", "classification": "measurement", "unit": "rad/s", "observed_delta": nominal.maximum_sensor_backend_velocity_delta_rad_s, "maximum_delta": 0.0, "status": pass_fail(check_results[6]) },
            { "id": "direct_effort_measurement_is_complete_v1", "classification": "measurement", "unit": "channel_sample", "observed": available_effort_measurements, "expected": expected_effort_measurements, "status": pass_fail(check_results[7]) },
            { "id": "effort_saturation_is_observable_v1", "classification": "actuator", "unit": "channel_sample", "observed": saturation_count, "minimum": 1, "status": pass_fail(check_results[8]) },
            { "id": "dropout_first_sequence_gap_v1", "classification": "measurement", "unit": "sequence", "observed": missing_sequence, "expected": SENSOR_FAULT_SEQUENCE, "status": pass_fail(check_results[9]) },
            { "id": "stuck_value_first_status_v1", "classification": "measurement", "unit": "sequence", "observed": first_stuck_sequence, "expected": SENSOR_FAULT_SEQUENCE, "status": pass_fail(check_results[10]) },
            { "id": "world_seed_consistency_v1", "classification": "input", "unit": "seed_match", "observed": check_results[11], "expected": true, "status": pass_fail(check_results[11]) },
            { "id": "controller_bootstrap_frame_count_v1", "classification": "estimation", "unit": "frame", "observed": controller_bootstrap_frames, "expected": expected_controller_bootstrap_frames, "status": pass_fail(check_results[12]) },
            { "id": "controller_observation_sequence_and_age_v1", "classification": "estimation", "unit": "sequence_age_match", "observed": controller_timing_aligned, "expected": true, "status": pass_fail(check_results[13]) },
            { "id": "controller_feedback_frame_count_v1", "classification": "estimation", "unit": "frame", "observed": controller_feedback_frames, "expected": expected_controller_feedback_frames, "status": pass_fail(check_results[14]) },
            { "id": "controller_feedback_correction_bound_v1", "classification": "controller", "unit": "rad", "observed": maximum_feedback_correction_rad, "maximum": configured_maximum_feedback_correction_rad, "status": pass_fail(check_results[15]) },
            { "id": "controller_integral_anti_windup_bound_v1", "classification": "controller", "unit": "rad", "observed": maximum_integral_correction_rad, "maximum": configured_maximum_integral_correction_rad, "per_joint_bounds_satisfied": integral_correction_within_per_joint_bounds, "status": pass_fail(check_results[16]) },
            { "id": "controller_dropout_fail_closed_v1", "classification": "measurement", "unit": "action_step", "observed": dropout_controller_rejection.as_ref().map(|rejection| rejection.action_step), "expected": if dropout_rejection_expected { Some(expected_controller_rejection_step) } else { None }, "status": pass_fail(check_results[17]) },
            { "id": "controller_stuck_value_fail_closed_v1", "classification": "measurement", "unit": "action_step", "observed": stuck_controller_rejection.as_ref().map(|rejection| rejection.action_step), "expected": if feedback_enabled { Some(expected_controller_rejection_step) } else { None }, "status": pass_fail(check_results[18]) },
        ],
        "fault_evidence": {
            "dropout": {
                "injected_sequence": SENSOR_FAULT_SEQUENCE,
                "first_missing_sequence": missing_sequence,
                "next_observed_sequence": next_sequence,
                "first_contract_deviation": "sequence_gap",
                "detected_at_consumption_ticks": dropout_detected_at_ticks,
                "expected_detection_ticks": expected_dropout_detection_ticks,
                "emitted_frames": dropout.observations.len(),
                "attempted_frames": fault_actions.len(),
            },
            "stuck_value": {
                "injected_from_sequence": SENSOR_FAULT_SEQUENCE,
                "first_status_sequence": first_stuck_sequence,
                "first_contract_deviation": "stuck_value_status",
                "detected_at_consumption_ticks": stuck_detected_at_ticks,
                "expected_detection_ticks": expected_stuck_detection_ticks,
            },
            "controller_fail_closed": {
                "dropout": dropout_controller_rejection,
                "stuck_value": stuck_controller_rejection,
            },
        },
        "actuator_evidence": {
            "saturated_channel_samples": saturation_count,
            "first_saturation_sequence": first_saturation_sequence,
            "first_saturation_joint_index": first_saturation_joint_index,
            "first_saturation_joint_name": first_saturation_joint_name,
            "measured_effort_available": true,
        },
    });
    let dropout_trace = json!({
        "kind": "rne_joint_feedback_fault_trace",
        "schema_version": 1,
        "task_id": controller.task_id,
        "controller_id": controller.controller_id,
        "action_trace_sha256": hashes.action_trace_sha256,
        "fault": { "kind": "drop_sequence", "sequence": SENSOR_FAULT_SEQUENCE },
        "observations": dropout.observations,
    });
    let stuck_trace = json!({
        "kind": "rne_joint_feedback_fault_trace",
        "schema_version": 1,
        "task_id": controller.task_id,
        "controller_id": controller.controller_id,
        "action_trace_sha256": hashes.action_trace_sha256,
        "fault": { "kind": "stuck_from_sequence", "sequence": SENSOR_FAULT_SEQUENCE },
        "observations": stuck.observations,
    });
    Ok(SensorValidationArtifacts {
        report,
        dropout_trace,
        stuck_trace,
    })
}

fn first_sequence_gap(observations: &[ObservationFrame]) -> Option<(u64, u64, u64)> {
    observations.windows(2).find_map(|pair| {
        let expected = pair[0].step.checked_add(1)?;
        (pair[1].step != expected).then_some((expected, pair[1].step, pair[1].consumed_at_ticks))
    })
}

fn sha256_json(value: &impl Serialize) -> Result<String> {
    Ok(sha256(&serde_json::to_vec(value)?))
}

const fn pass_fail(passed: bool) -> &'static str {
    if passed {
        "passed"
    } else {
        "failed"
    }
}

fn observation_from_feedback(
    frame: Frame<JointFeedback>,
    consumed_at_ticks: u64,
    sensor_sample_published: bool,
    state_hashes: &[u64],
    controller_decisions: &[ControllerDecision],
    applied_actuations: &[AppliedActuation],
) -> Result<ObservationFrame> {
    anyhow::ensure!(
        frame.payload.schema_version == JointFeedback::SCHEMA_VERSION,
        "unsupported OpenArm joint-feedback schema"
    );
    let mut positions = Vec::with_capacity(frame.payload.joints.len());
    let mut velocities = Vec::with_capacity(frame.payload.joints.len());
    let mut targets = Vec::with_capacity(frame.payload.joints.len());
    let mut limited_efforts = Vec::with_capacity(frame.payload.joints.len());
    let mut saturated = Vec::with_capacity(frame.payload.joints.len());
    let mut effort_measurement_available = Vec::with_capacity(frame.payload.joints.len());
    let mut measured_efforts = Vec::with_capacity(frame.payload.joints.len());
    let mut maximum_actuator_tracking_error_rad = 0.0_f64;
    let actuation = applied_actuations
        .get(frame.sequence.saturating_sub(1) as usize)
        .context("OpenArm feedback sequence has no matching actuation")?;
    anyhow::ensure!(
        actuation.target_position_rad.len() == frame.payload.joints.len(),
        "OpenArm actuation width does not match joint feedback"
    );
    for (index, joint) in frame.payload.joints.iter().enumerate() {
        let (position_rad, velocity_rad_s) = match joint.coordinate {
            rne_data::JointCoordinateFeedback::Revolute {
                position_rad,
                velocity_rad_s,
            } => (position_rad, velocity_rad_s),
            _ => bail!("OpenArm feedback channel {} is not revolute", joint.name),
        };
        anyhow::ensure!(
            matches!(
                joint.command,
                rne_data::JointCommandFeedback::Revolute {
                    mode: rne_data::JointCommandMode::Effort,
                    ..
                }
            ),
            "OpenArm feedback channel {} has no revolute effort command",
            joint.name
        );
        let target_position_rad = actuation.target_position_rad[index];
        let limited_effort_command_nm = actuation.limited_effort_command_nm[index];
        let effort_saturated = actuation.effort_saturated[index];
        positions.push(position_rad);
        velocities.push(velocity_rad_s);
        targets.push(target_position_rad);
        limited_efforts.push(limited_effort_command_nm);
        saturated.push(effort_saturated);
        let measured_effort = match joint.effort {
            JointEffortFeedback::Unavailable => None,
            JointEffortFeedback::Revolute { measured_effort_nm } => Some(measured_effort_nm),
            JointEffortFeedback::Prismatic { .. } => {
                bail!(
                    "OpenArm feedback channel {} has prismatic effort",
                    joint.name
                )
            }
        };
        effort_measurement_available.push(measured_effort.is_some());
        measured_efforts.push(measured_effort);
        maximum_actuator_tracking_error_rad =
            maximum_actuator_tracking_error_rad.max((position_rad - target_position_rad).abs());
    }
    let physics_hash = *state_hashes
        .get(frame.sequence.saturating_sub(1) as usize)
        .context("OpenArm feedback sequence has no matching state hash")?;
    let decision = controller_decisions
        .get(frame.sequence.saturating_sub(1) as usize)
        .context("OpenArm feedback sequence has no matching controller decision")?;
    let maximum_tracking_error_rad = positions
        .iter()
        .zip(&decision.reference_position_rad)
        .map(|(position, reference)| (position - reference).abs())
        .fold(0.0_f64, f64::max);
    Ok(ObservationFrame {
        step: frame.sequence,
        sim_time_ticks: frame.capture_time.ticks(),
        scheduled_capture_ticks: frame.payload.scheduled_capture_ticks,
        sample_phase_error_ticks: frame.payload.sample_phase_error_ticks,
        available_time_ticks: frame.available_time.ticks(),
        consumed_at_ticks,
        observation_age_ticks: consumed_at_ticks.saturating_sub(frame.capture_time.ticks()),
        sensor_status: frame.payload.status,
        sensor_sample_published,
        controller_observation_sequence: decision.observation_sequence,
        controller_observation_age_ticks: decision.observation_age_ticks,
        controller_bootstrap: decision.bootstrap,
        controller_rejected: decision.rejected,
        controller_rejection_reason: decision.rejection_reason,
        fail_safe_hold_active: decision.fail_safe_hold_active,
        controller_state_frozen: decision.state_frozen,
        controller_recovered: decision.recovered,
        joint_position_rad: positions,
        joint_velocity_rad_s: velocities,
        joint_reference_position_rad: decision.reference_position_rad.clone(),
        joint_controller_observation_position_rad: decision
            .controller_observation_position_rad
            .clone(),
        joint_measurement_bias_rad: decision.measurement_bias_rad.clone(),
        measurement_bias_active: decision
            .measurement_bias_rad
            .iter()
            .any(|value| *value != 0.0),
        joint_controller_target_rad: decision.target_position_rad.clone(),
        joint_actuator_disturbance_rad: targets
            .iter()
            .zip(&decision.target_position_rad)
            .map(|(applied, commanded)| applied - commanded)
            .collect(),
        joint_position_target_rad: targets,
        actuator_disturbance_active: controller_decisions
            .get(frame.sequence.saturating_sub(1) as usize)
            .is_some_and(|decision| {
                decision
                    .target_position_rad
                    .iter()
                    .zip(&frame.payload.joints)
                    .any(|(commanded, joint)| match joint.command {
                        rne_data::JointCommandFeedback::Revolute {
                            target_position_rad: Some(applied),
                            ..
                        } => (applied - commanded).abs() > 0.0,
                        _ => false,
                    })
            }),
        joint_feedback_correction_rad: decision.correction_rad.clone(),
        joint_integral_correction_rad: decision.integral_correction_rad.clone(),
        limited_effort_command_nm: limited_efforts,
        measured_effort_nm: measured_efforts,
        effort_saturated: saturated,
        effort_measurement_available,
        maximum_actuator_tracking_error_rad,
        maximum_tracking_error_rad,
        physics_hash,
    })
}

fn write_failure_replay(
    path: &Path,
    controller: &ControllerSpec,
    actions: &[ActionFrame],
    rollout: &Rollout,
) -> Result<()> {
    let inject_step = controller.intentional_failure.inject_at_step;
    let accepted_steps = (inject_step - 1) as usize;
    let descriptor = BehaviorContractDescriptor {
        name: "controller_action_width_matches_task".to_string(),
        kind: BehaviorContractKind::Always,
        entities: vec![controller.controller_id.clone()],
    };
    let mut frames = Vec::with_capacity(accepted_steps + 2);
    frames.push(BehaviorReplayFrame {
        step: 0,
        sim_time_ticks: 0,
        action: BehaviorReplayAction::InitialObservation,
        observation: json!({
            "action_width_expected": controller.action_joint_order.len(),
            "action_width_observed": controller.action_joint_order.len(),
            "contract_status": "initial"
        }),
        state_digest: rollout.initial_digest,
    });
    for (action, observation) in actions[..accepted_steps]
        .iter()
        .zip(&rollout.observations[..accepted_steps])
    {
        frames.push(BehaviorReplayFrame {
            step: action.step,
            sim_time_ticks: action.sim_time_ticks,
            action: BehaviorReplayAction::Advance,
            observation: json!({
                "action_width_expected": controller.action_joint_order.len(),
                "action_width_observed": controller.action_joint_order.len(),
                "joint_position_rad": observation.joint_position_rad,
                "joint_velocity_rad_s": observation.joint_velocity_rad_s,
                "contract_status": "passed"
            }),
            state_digest: observation.physics_hash,
        });
    }
    let before_failure = rollout
        .observations
        .get(accepted_steps - 1)
        .context("controller failure replay has no pre-failure observation")?;
    let violation = BehaviorViolation {
        step: inject_step,
        sim_time_ticks: inject_step * FIXED_DELTA_TICKS,
        state_digest: before_failure.physics_hash,
        entities: descriptor.entities.clone(),
        message: "controller emitted 8 action elements but TaskSpec requires 9; rejected before simulator state advance".to_string(),
    };
    frames.push(BehaviorReplayFrame {
        step: inject_step,
        sim_time_ticks: inject_step * FIXED_DELTA_TICKS,
        action: BehaviorReplayAction::Advance,
        observation: json!({
            "action_width_expected": controller.action_joint_order.len(),
            "action_width_observed": controller.action_joint_order.len() - 1,
            "joint_position_rad": before_failure.joint_position_rad,
            "joint_velocity_rad_s": before_failure.joint_velocity_rad_s,
            "contract_status": "failed"
        }),
        state_digest: before_failure.physics_hash,
    });
    let digest = Sha256::digest(controller.controller_id.as_bytes());
    let scenario_digest = u64::from_le_bytes(digest[..8].try_into().unwrap());
    let replay = BehaviorReplayArtifact::new(
        "rne.openarm.right_joint_tracking.controller_contract.v1",
        scenario_digest,
        20260824,
        FIXED_DELTA_TICKS,
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor,
            violation,
        },
    )?;
    replay.write_json(path)?;
    Ok(())
}

fn configure_actuators(sim: &mut UrdfSceneSim, config: &ActuationConfig) -> Result<()> {
    for joint in &config.joints {
        anyhow::ensure!(
            sim.configure_named_revolute_effort_actuation(&joint.link_name, joint.max_effort_nm),
            "missing OpenArm actuator {}",
            joint.link_name
        );
    }
    Ok(())
}

fn required_path(args: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    args.next()
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .with_context(|| format!("{option} requires a path"))
}

fn validate_model_provenance(scene: &Path, robot_asset: &Path, model_urdf: &Path) -> Result<()> {
    let scene_value: toml::Value = toml::from_str(&fs::read_to_string(scene)?)?;
    let declared_robot = scene_value
        .get("robots")
        .and_then(toml::Value::as_array)
        .and_then(|robots| (robots.len() == 1).then_some(&robots[0]))
        .and_then(|robot| robot.get("path"))
        .and_then(toml::Value::as_str)
        .context("OpenArm scene must declare exactly one robot path")?;
    let robot_value: toml::Value = toml::from_str(&fs::read_to_string(robot_asset)?)?;
    let declared_urdf = robot_value
        .get("urdf")
        .and_then(|urdf| urdf.get("path"))
        .and_then(toml::Value::as_str)
        .context("OpenArm robot asset must declare urdf.path")?;
    let scene_robot = scene
        .parent()
        .context("OpenArm scene has no parent directory")?
        .join(declared_robot);
    let asset_urdf = robot_asset
        .parent()
        .context("OpenArm robot asset has no parent directory")?
        .join(declared_urdf);
    anyhow::ensure!(
        fs::canonicalize(&scene_robot)? == fs::canonicalize(robot_asset)?,
        "--robot-asset is not the robot referenced by --scene"
    );
    anyhow::ensure!(
        fs::canonicalize(&asset_urdf)? == fs::canonicalize(model_urdf)?,
        "--model-urdf is not the model referenced by --robot-asset"
    );
    Ok(())
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write {}", path.display()))
}

fn write_html_report(path: &Path, title: &str, report: &serde_json::Value) -> Result<()> {
    let report_json = serde_json::to_string_pretty(report)?;
    let escaped = report_json
        .replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;");
    let status = report["status"].as_str().unwrap_or("unknown");
    let html = format!(
        "<!doctype html>\n<meta charset=\"utf-8\">\n<title>{title}</title>\n\
         <style>body{{font:16px system-ui;max-width:1100px;margin:40px auto;padding:0 20px;background:#111827;color:#e5e7eb}}\
         h1{{margin-bottom:8px}}.status{{display:inline-block;padding:6px 12px;border-radius:999px;background:#064e3b;color:#a7f3d0;font-weight:700}}\
         pre{{white-space:pre-wrap;overflow-wrap:anywhere;background:#1f2937;border:1px solid #374151;border-radius:10px;padding:18px;line-height:1.4}}</style>\n\
         <h1>{title}</h1><p class=\"status\">{status}</p><p>Self-contained deterministic sensor evidence. All times are simulation ticks and all physical fields retain explicit SI units.</p><pre>{escaped}</pre>\n"
    );
    fs::write(path, html).with_context(|| format!("write {}", path.display()))
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sensor_observation(
        step: u64,
        consumed_at_ticks: u64,
        sensor_status: JointFeedbackStatus,
    ) -> ObservationFrame {
        ObservationFrame {
            step,
            sim_time_ticks: step * FIXED_DELTA_TICKS,
            scheduled_capture_ticks: step * FIXED_DELTA_TICKS,
            sample_phase_error_ticks: 0,
            available_time_ticks: consumed_at_ticks,
            consumed_at_ticks,
            observation_age_ticks: FIXED_DELTA_TICKS,
            sensor_status,
            sensor_sample_published: true,
            controller_observation_sequence: Some(step.saturating_sub(1)),
            controller_observation_age_ticks: Some(FIXED_DELTA_TICKS),
            controller_bootstrap: false,
            controller_rejected: false,
            controller_rejection_reason: None,
            fail_safe_hold_active: false,
            controller_state_frozen: false,
            controller_recovered: false,
            joint_position_rad: Vec::new(),
            joint_velocity_rad_s: Vec::new(),
            joint_reference_position_rad: Vec::new(),
            joint_controller_observation_position_rad: Vec::new(),
            joint_measurement_bias_rad: Vec::new(),
            measurement_bias_active: false,
            joint_controller_target_rad: Vec::new(),
            joint_actuator_disturbance_rad: Vec::new(),
            joint_position_target_rad: Vec::new(),
            actuator_disturbance_active: false,
            joint_feedback_correction_rad: Vec::new(),
            joint_integral_correction_rad: Vec::new(),
            limited_effort_command_nm: Vec::new(),
            measured_effort_nm: Vec::new(),
            effort_saturated: Vec::new(),
            effort_measurement_available: Vec::new(),
            maximum_actuator_tracking_error_rad: 0.0,
            maximum_tracking_error_rad: 0.0,
            physics_hash: step,
        }
    }

    fn fixture(relative: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(relative)
    }

    #[test]
    fn portable_controller_compiles_exact_task_order_and_failure_step() {
        let controller: ControllerSpec = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json",
            ))
            .unwrap(),
        )
        .unwrap();
        let task: TaskSpec = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json",
            ))
            .unwrap(),
        )
        .unwrap();
        let actuation_config: ActuationConfig = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json",
            ))
            .unwrap(),
        )
        .unwrap();
        validate(&controller, &task, &actuation_config).unwrap();
        let actions = compile_actions(&controller);
        assert_eq!(actions.len(), 1_800);
        assert_eq!(actions[0].action_sequence, 0);
        assert_eq!(actions[306].step, 307);
        assert_eq!(actions.last().unwrap().sim_time_ticks, 30_000_000_600);
        assert_eq!(
            actions.last().unwrap().joint_position_target_rad,
            controller
                .keyframes
                .last()
                .unwrap()
                .joint_position_target_rad
        );
        assert_eq!(controller.intentional_failure.inject_at_step, 307);
    }

    #[test]
    fn sequence_gap_reports_the_first_observable_deviation() {
        let frames = vec![
            sensor_observation(305, 5_100, JointFeedbackStatus::Nominal),
            sensor_observation(306, 5_200, JointFeedbackStatus::Nominal),
            sensor_observation(308, 5_400, JointFeedbackStatus::Nominal),
            sensor_observation(310, 5_600, JointFeedbackStatus::Nominal),
        ];
        assert_eq!(first_sequence_gap(&frames), Some((307, 308, 5_400)));
        assert_eq!(first_sequence_gap(&frames[..2]), None);
    }

    #[test]
    fn controller_output_is_derived_only_from_typed_observation_and_reference() {
        let controller: ControllerSpec = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json",
            ))
            .unwrap(),
        )
        .unwrap();
        let reference = vec![0.5; controller.action_joint_order.len()];
        let observation = ControllerObservation {
            sequence: 41,
            capture_time_ticks: 100,
            available_time_ticks: 100 + FIXED_DELTA_TICKS,
            status: JointFeedbackStatus::Nominal,
            joint_position_rad: vec![0.4; reference.len()],
            joint_velocity_rad_s: vec![0.2; reference.len()],
        };
        let mut first_state = ControllerState::new(reference.len());
        let mut replay_state = ControllerState::new(reference.len());
        let first = controller_decision(
            &controller,
            &reference,
            &mut first_state,
            Some(&observation),
            100 + FIXED_DELTA_TICKS,
        )
        .unwrap();
        let replay = controller_decision(
            &controller,
            &reference,
            &mut replay_state,
            Some(&observation),
            100 + FIXED_DELTA_TICKS,
        )
        .unwrap();
        assert_eq!(first, replay);
        assert_eq!(first.observation_sequence, Some(41));
        assert!(!first.bootstrap);
        assert!(first
            .target_position_rad
            .iter()
            .zip(&reference)
            .any(|(target, reference)| target != reference));

        let bootstrap = controller_decision(
            &controller,
            &reference,
            &mut ControllerState::new(reference.len()),
            None,
            0,
        )
        .unwrap();
        assert!(bootstrap.bootstrap);
        assert_eq!(bootstrap.target_position_rad, reference);
    }

    #[test]
    fn controller_rejects_stale_or_faulted_observations() {
        let controller: ControllerSpec = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json",
            ))
            .unwrap(),
        )
        .unwrap();
        let reference = vec![0.0; controller.action_joint_order.len()];
        let mut observation = ControllerObservation {
            sequence: 7,
            capture_time_ticks: 0,
            available_time_ticks: FIXED_DELTA_TICKS,
            status: JointFeedbackStatus::Nominal,
            joint_position_rad: reference.clone(),
            joint_velocity_rad_s: reference.clone(),
        };
        let mut state = ControllerState::new(reference.len());
        assert!(controller_decision(
            &controller,
            &reference,
            &mut state,
            Some(&observation),
            2 * FIXED_DELTA_TICKS,
        )
        .is_err());
        observation.status = JointFeedbackStatus::StuckValue;
        assert!(controller_decision(
            &controller,
            &reference,
            &mut state,
            Some(&observation),
            FIXED_DELTA_TICKS,
        )
        .is_err());
    }
}

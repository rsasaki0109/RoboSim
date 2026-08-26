//! Executes the portable OpenArm typed-feedback controller on native MuJoCo.

use anyhow::{bail, Context, Result};
use rne_ai::TaskSpec;
use rne_assets::load_and_spawn_scene;
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, Frame, InMemoryDataBus, JointEffortFeedback, JointFeedback, JointFeedbackStatus,
    StreamId,
};
use rne_ecs::{spawn_named, Entity, Name, World};
use rne_physics::{
    hash_physics_state_v2, JointActuation, JointPassiveDynamics, JointState, PhysicsBackend,
    PhysicsWorldDesc, PhysicsWorldId, RevoluteJointDesc,
};
use rne_physics_mujoco::MuJoCoBackend;
use rne_sensor::{
    sample_joint_feedback_sensors, JointFeedbackChannelSpec, JointFeedbackFault,
    JointFeedbackSensor, JointFeedbackSensorState,
};
use rne_world::WorldEntity;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::fs;
use std::path::{Path, PathBuf};

const FIXED_DELTA_TICKS: u64 = 16_666_667;
const JOINT_FEEDBACK_STREAM: StreamId = StreamId::new(9_001);
const BACKEND_ID: &str = "mujoco_native";
const BACKEND_VERSION: &str = "3.9.0";
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
    keyframes: Vec<serde_json::Value>,
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
struct IntentionalFailure {
    kind: String,
    inject_at_step: u64,
    expected_first_violation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ActionTrace {
    kind: String,
    schema_version: u32,
    task_id: String,
    task_sha256: String,
    controller_id: String,
    controller_sha256: String,
    fixed_delta_ticks: u64,
    action_semantics: String,
    action_joint_order: Vec<String>,
    actions: Vec<ActionFrame>,
}

#[derive(Clone, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ActionFrame {
    action_sequence: u64,
    step: u64,
    sim_time_ticks: u64,
    phase: String,
    joint_position_target_rad: Vec<f64>,
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

#[derive(Clone, Debug)]
struct ControllerObservation {
    sequence: u64,
    capture_time_ticks: u64,
    available_time_ticks: u64,
    status: JointFeedbackStatus,
    joint_position_rad: Vec<f64>,
    joint_velocity_rad_s: Vec<f64>,
}

#[derive(Clone, Debug)]
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

#[derive(Clone)]
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

struct Rollout {
    world_seed: u64,
    initial_digest: u64,
    final_digest: u64,
    observations: Vec<ObservationFrame>,
    maximum_sensor_backend_position_delta_rad: f64,
    maximum_sensor_backend_velocity_delta_rad_s: f64,
    joint_passive_dynamics: Vec<Option<JointPassiveDynamics>>,
}

struct MujocoOpenArmSim {
    world: World,
    backend: MuJoCoBackend,
    physics_world: PhysicsWorldId,
    sim_time: SimTime,
    fixed_delta: SimDuration,
    physics_substeps_per_control_step: usize,
    world_seed: u64,
}

impl MujocoOpenArmSim {
    fn new(
        scene_path: &Path,
        solver_iterations: usize,
        physics_substeps_per_control_step: usize,
    ) -> Result<Self> {
        let fixed_delta = SimDuration::from_ticks(FIXED_DELTA_TICKS);
        let physics_substeps = std::num::NonZeroUsize::new(physics_substeps_per_control_step)
            .context("physics substeps must be nonzero")?;
        let substep_deltas = fixed_delta
            .split_evenly(physics_substeps)
            .context("control period is too short for requested physics substeps")?;
        anyhow::ensure!(
            substep_deltas
                .iter()
                .all(|delta| *delta == substep_deltas[0]),
            "MuJoCo requires exact equal-tick physics substeps"
        );
        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, scene_path)
            .with_context(|| format!("load {}", scene_path.display()))?;
        let world_seed = world
            .get::<WorldEntity>(spawned.world)
            .map(|world| world.seed)
            .unwrap_or(0);
        let mut backend = MuJoCoBackend::new(substep_deltas[0]).context("create MuJoCo backend")?;
        let physics_world = backend
            .create_world(PhysicsWorldDesc {
                solver_iterations,
                ..PhysicsWorldDesc::default()
            })
            .context("create MuJoCo physics world")?;
        backend
            .sync_from_ecs(&mut world, physics_world)
            .context("compile OpenArm ECS into MuJoCo")?;
        backend
            .sync_to_ecs(&mut world, physics_world)
            .context("read initial OpenArm MuJoCo state")?;
        Ok(Self {
            world,
            backend,
            physics_world,
            sim_time: SimTime::default(),
            fixed_delta,
            physics_substeps_per_control_step,
            world_seed,
        })
    }

    fn find_named(&self, name: &str) -> Option<Entity> {
        self.world.iter_entities().find_map(|entity| {
            entity
                .get::<Name>()
                .is_some_and(|candidate| candidate.0 == name)
                .then_some(entity.id())
        })
    }

    fn configure_actuators(&mut self, config: &ActuationConfig) -> Result<()> {
        for joint in &config.joints {
            let entity = self
                .find_named(&joint.link_name)
                .with_context(|| format!("missing OpenArm actuator {}", joint.link_name))?;
            anyhow::ensure!(
                self.world.get::<RevoluteJointDesc>(entity).is_some(),
                "OpenArm actuator {} is not revolute",
                joint.link_name
            );
            self.world
                .entity_mut(entity)
                .insert(JointActuation::RevoluteEffort {
                    effort_nm: 0.0,
                    max_effort_nm: joint.max_effort_nm,
                });
        }
        Ok(())
    }

    fn install_sensor(
        &mut self,
        controller: &ControllerSpec,
        fault: JointFeedbackFault,
    ) -> Result<()> {
        let channels = controller
            .rne_actuator_link_order
            .iter()
            .map(|name| {
                self.find_named(name)
                    .map(|joint_entity| JointFeedbackChannelSpec {
                        name: name.clone(),
                        joint_entity,
                    })
                    .with_context(|| format!("missing joint-feedback link {name}"))
            })
            .collect::<Result<Vec<_>>>()?;
        let sensor = JointFeedbackSensor {
            update_rate_hz: 60.0,
            sample_period_ticks: Some(FIXED_DELTA_TICKS),
            phase_offset_ticks: FIXED_DELTA_TICKS,
            latency_ticks: FIXED_DELTA_TICKS,
            enabled: true,
            stream_id: JOINT_FEEDBACK_STREAM,
            channels,
            fault,
        };
        anyhow::ensure!(sensor.is_valid(), "invalid OpenArm joint-feedback sensor");
        let sensor_entity = spawn_named(&mut self.world, "openarm_right_joint_feedback");
        self.world
            .entity_mut(sensor_entity)
            .insert((sensor, JointFeedbackSensorState::default()));
        Ok(())
    }

    fn step_targets(
        &mut self,
        config: &ActuationConfig,
        link_names: &[String],
        targets: &[f64],
    ) -> Result<AppliedActuation> {
        anyhow::ensure!(link_names.len() == targets.len(), "action width mismatch");
        anyhow::ensure!(
            link_names.len() == config.joints.len(),
            "actuation width mismatch"
        );
        let mut efforts = Vec::with_capacity(link_names.len());
        let mut saturated = Vec::with_capacity(link_names.len());
        let deltas = self
            .fixed_delta
            .split_evenly(
                std::num::NonZeroUsize::new(self.physics_substeps_per_control_step)
                    .context("physics substeps must be nonzero")?,
            )
            .context("control period is too short for requested physics substeps")?;
        for delta in deltas {
            efforts.clear();
            saturated.clear();
            for ((name, target), joint) in link_names.iter().zip(targets).zip(&config.joints) {
                anyhow::ensure!(name == &joint.link_name, "actuation link order mismatch");
                let entity = self
                    .find_named(name)
                    .with_context(|| format!("missing actuator {name}"))?;
                let Some(JointActuation::RevoluteEffort { max_effort_nm, .. }) =
                    self.world.get::<JointActuation>(entity).copied()
                else {
                    bail!("actuator {name} is not in revolute effort mode");
                };
                let (position_rad, velocity_rad_s) = self.named_state(name)?;
                let raw_effort_nm = joint.stiffness_nm_per_rad * (target - position_rad)
                    - joint.damping_nm_s_per_rad * velocity_rad_s;
                let clamped_effort_nm = raw_effort_nm.clamp(-max_effort_nm, max_effort_nm);
                let motor_effort_command_nm = if clamped_effort_nm * velocity_rad_s > 0.0 {
                    let drive_fraction =
                        (1.0 - velocity_rad_s.abs() / joint.max_velocity_rad_s).clamp(0.0, 1.0);
                    clamped_effort_nm * drive_fraction
                } else {
                    clamped_effort_nm
                };
                let joint_effort_nm = motor_effort_command_nm * joint.transmission_efficiency;
                self.world
                    .entity_mut(entity)
                    .insert(JointActuation::RevoluteEffort {
                        effort_nm: joint_effort_nm,
                        max_effort_nm,
                    });
                efforts.push(motor_effort_command_nm);
                saturated.push(raw_effort_nm != motor_effort_command_nm);
            }
            self.backend
                .sync_from_ecs(&mut self.world, self.physics_world)
                .context("upload OpenArm command to MuJoCo")?;
            self.backend
                .step(self.physics_world, delta)
                .context("step OpenArm MuJoCo world")?;
            self.sim_time = self.sim_time + delta;
            self.backend
                .sync_to_ecs(&mut self.world, self.physics_world)
                .context("download OpenArm MuJoCo substep state")?;
        }
        Ok(AppliedActuation {
            target_position_rad: targets.to_vec(),
            limited_effort_command_nm: efforts,
            effort_saturated: saturated,
        })
    }

    fn sample(&mut self, bus: &mut InMemoryDataBus) -> Result<usize> {
        sample_joint_feedback_sensors(&mut self.world, self.sim_time, bus)
            .context("sample MuJoCo OpenArm joint feedback")
    }

    fn named_state(&self, name: &str) -> Result<(f64, f64)> {
        let entity = self
            .find_named(name)
            .with_context(|| format!("missing MuJoCo joint {name}"))?;
        match self.world.get::<JointState>(entity).copied() {
            Some(JointState::Revolute {
                position_rad,
                velocity_rad_s,
            }) => Ok((position_rad, velocity_rad_s)),
            _ => bail!("MuJoCo joint {name} has no revolute state"),
        }
    }
}

fn main() {
    if let Err(error) = run() {
        eprintln!("OpenArm MuJoCo trace failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut controller_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json");
    let task_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json");
    let mut actuation_path =
        repo_root.join("adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json");
    let mut robot_asset_path = repo_root.join("assets/robots/openarm_v2_right.rne.robot.toml");
    let mut model_urdf_path =
        repo_root.join("assets/robots/openarm_description/openarm_v2_right.rne.urdf");
    let mut scene = repo_root.join("assets/scenes/openarm_v2_right_validation.rne.scene.toml");
    let actions_path_default =
        repo_root.join("artifacts/openarm-cross-sim/controller-actions.json");
    let mut actions_path = actions_path_default;
    let mut output = repo_root.join("artifacts/openarm-cross-sim");
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--controller" => controller_path = required_path(&mut args, "--controller")?,
            "--actions" => actions_path = required_path(&mut args, "--actions")?,
            "--actuation-config" => {
                actuation_path = required_path(&mut args, "--actuation-config")?
            }
            "--robot-asset" => robot_asset_path = required_path(&mut args, "--robot-asset")?,
            "--model-urdf" => model_urdf_path = required_path(&mut args, "--model-urdf")?,
            "--scene" => scene = required_path(&mut args, "--scene")?,
            "--output" => output = required_path(&mut args, "--output")?,
            other => bail!("unknown argument {other:?}"),
        }
    }
    let controller_bytes = fs::read(&controller_path)?;
    let task_bytes = fs::read(&task_path)?;
    let actuation_bytes = fs::read(&actuation_path)?;
    let robot_asset_bytes = fs::read(&robot_asset_path)?;
    let model_urdf_bytes = fs::read(&model_urdf_path)?;
    let scene_bytes = fs::read(&scene)?;
    validate_model_provenance(&scene, &robot_asset_path, &model_urdf_path)?;
    let controller: ControllerSpec = serde_json::from_slice(&controller_bytes)?;
    let task: TaskSpec = serde_json::from_slice(&task_bytes)?;
    let actuation: ActuationConfig = serde_json::from_slice(&actuation_bytes)?;
    let actions_bytes = fs::read(&actions_path)
        .with_context(|| format!("read shared reference {}", actions_path.display()))?;
    let actions: ActionTrace = serde_json::from_slice(&actions_bytes)?;
    validate(
        &controller,
        &task,
        &actuation,
        &actions,
        &task_bytes,
        &controller_bytes,
    )?;
    let first = rollout(&scene, &controller, &actuation, &actions.actions)?;
    let replay = rollout(&scene, &controller, &actuation, &actions.actions)?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest && first.observations == replay.observations,
        "MuJoCo replay differed for identical inputs"
    );
    let final_error = first
        .observations
        .last()
        .context("MuJoCo trace has no observations")?
        .maximum_tracking_error_rad;
    let maximum_error = first
        .observations
        .iter()
        .map(|frame| frame.maximum_tracking_error_rad)
        .fold(0.0_f64, f64::max);
    let controller_sha256 = sha256(&controller_bytes);
    let task_sha256 = sha256(&task_bytes);
    let action_trace_sha256 = sha256(&actions_bytes);
    let actuation_sha256 = sha256(&actuation_bytes);
    let robot_asset_sha256 = sha256(&robot_asset_bytes);
    let model_urdf_sha256 = sha256(&model_urdf_bytes);
    let scene_config_sha256 = sha256(&scene_bytes);
    fs::create_dir_all(&output)?;
    write_json(
        &output.join("mujoco-success-trace.json"),
        &json!({
            "kind": "rne_openarm_backend_trace",
            "schema_version": 1,
            "backend_id": BACKEND_ID,
            "backend_version": BACKEND_VERSION,
            "task_id": controller.task_id,
            "task_sha256": task_sha256,
            "controller_id": controller.controller_id,
            "controller_sha256": controller_sha256,
            "action_trace_sha256": action_trace_sha256,
            "robot_asset_config_sha256": robot_asset_sha256,
            "model_urdf_sha256": model_urdf_sha256,
            "scene_config_sha256": scene_config_sha256,
            "actuation_config_sha256": actuation_sha256,
            "fixed_delta_ticks": FIXED_DELTA_TICKS,
            "physics_substeps_per_control_step": actuation.physics_substeps_per_control_step,
            "joint_feedback_schema_version": JointFeedback::SCHEMA_VERSION,
            "joint_feedback_latency_ticks": FIXED_DELTA_TICKS,
            "observation_source": "databus_latest_available",
            "controller_execution": controller_execution(&controller),
            "physics_state_hash_contract": PHYSICS_HASH_CONTRACT,
            "world_seed": first.world_seed,
            "initial_state_digest": first.initial_digest,
            "final_state_digest": first.final_digest,
            "replay_final_state_digest": replay.final_digest,
            "replay_match": true,
            "maximum_sensor_backend_position_delta_rad": first.maximum_sensor_backend_position_delta_rad,
            "maximum_sensor_backend_velocity_delta_rad_s": first.maximum_sensor_backend_velocity_delta_rad_s,
            "final_maximum_tracking_error_rad": final_error,
            "maximum_tracking_error_rad": maximum_error,
            "joint_passive_dynamics": first.joint_passive_dynamics,
            "observations": first.observations,
        }),
    )?;
    let failure = intentional_failure(
        &scene,
        &controller,
        &actuation,
        &actions.actions,
        &replay.observations,
    )?;
    write_json(
        &output.join("mujoco-intentional-failure.json"),
        &json!({
            "kind": "rne_controller_contract_failure",
            "schema_version": 1,
            "backend_id": BACKEND_ID,
            "backend_version": BACKEND_VERSION,
            "task_id": controller.task_id,
            "task_sha256": task_sha256,
            "controller_id": controller.controller_id,
            "controller_sha256": controller_sha256,
            "action_trace_sha256": action_trace_sha256,
            "robot_asset_config_sha256": robot_asset_sha256,
            "model_urdf_sha256": model_urdf_sha256,
            "scene_config_sha256": scene_config_sha256,
            "actuation_config_sha256": actuation_sha256,
            "injection_kind": controller.intentional_failure.kind,
            "injected_step": controller.intentional_failure.inject_at_step,
            "first_violation": controller.intentional_failure.expected_first_violation,
            "first_violation_step": controller.intentional_failure.inject_at_step,
            "first_violation_sim_time_ticks": controller.intentional_failure.inject_at_step * FIXED_DELTA_TICKS,
            "unit": "missing_action_element_count",
            "observed_missing_action_elements": 1,
            "maximum_missing_action_elements": 0,
            "rejection_code": "width_mismatch",
            "rejected_step_changed_state": failure,
            "status": "failed_as_expected",
        }),
    )?;
    println!(
        "OpenArm MuJoCo trace: steps={} replay_match=true final_error_rad={final_error:.6}",
        actions.actions.len()
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
    actuation: &ActuationConfig,
    actions: &ActionTrace,
    task_bytes: &[u8],
    controller_bytes: &[u8],
) -> Result<()> {
    task.validate()?;
    let width = controller.action_joint_order.len();
    anyhow::ensure!(
        controller.kind == "rne_joint_pose_cycle_controller"
            && controller.schema_version == 1
            && controller.interpolation == "smoothstep_v1"
            && !controller.keyframes.is_empty(),
        "unsupported controller artifact"
    );
    anyhow::ensure!(
        controller.task_id == task.task_id
            && width == 9
            && controller.rne_actuator_link_order.len() == width,
        "controller TaskSpec or joint order mismatch"
    );
    if let Some(disturbance) = &controller.disturbance_contract {
        validate_actuator_disturbance(controller, disturbance, actions.actions.len() as u64)?;
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
                    && *end_controller_step <= actions.actions.len() as u64
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
                        <= (actions.actions.len() as u64).saturating_add(1)
                    && controller_visibility == "missing_publication_only"
                    && application_order == "after_typed_sensor_capture_before_controller_ingress",
                "invalid OpenArm measurement-dropout contract"
            ),
        }
    }
    match (&controller.observation_contract, &controller.feedback_law) {
        (Some(contract), Some(law)) => {
            anyhow::ensure!(
                contract.kind == "rne_joint_feedback"
                    && contract.schema_version == JointFeedback::SCHEMA_VERSION
                    && contract.sample_period_ticks == FIXED_DELTA_TICKS
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
                    }
                    && contract.required_status == JointFeedbackStatus::Nominal
                    && contract.bootstrap_policy == "reference_until_first_available",
                "unsupported controller observation contract"
            );
            if law.kind == "joint_position_reference_pid_v1" {
                validate_pid_law(law, width)?;
            } else if law.kind == "joint_position_state_feedback_integral_v1" {
                validate_state_feedback_law(controller, law)?;
            } else {
                bail!("unsupported OpenArm feedback law {}", law.kind);
            }
        }
        (None, None) => {}
        _ => bail!("controller feedback contract is incomplete"),
    }
    anyhow::ensure!(
        actuation.kind == "rne_portable_pd_effort_actuation_config"
            && actuation.schema_version == 1
            && actuation.backend_id == "rne_native_physics"
            && actuation.motor_model == "explicit_pd_effort_v1"
            && actuation.physics_substeps_per_control_step > 0
            && actuation.physics_substeps_per_control_step <= FIXED_DELTA_TICKS as usize
            && FIXED_DELTA_TICKS.is_multiple_of(actuation.physics_substeps_per_control_step as u64)
            && actuation.fixed_delta_ticks == FIXED_DELTA_TICKS
            && actuation.joints.len() == width,
        "invalid native actuation contract"
    );
    anyhow::ensure!(
        actuation
            .joints
            .iter()
            .map(|joint| &joint.joint_name)
            .eq(&controller.action_joint_order)
            && actuation
                .joints
                .iter()
                .map(|joint| &joint.link_name)
                .eq(&controller.rne_actuator_link_order),
        "actuation order differs from controller"
    );
    anyhow::ensure!(
        actuation.joints.iter().all(|joint| {
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
        "native actuation configuration has invalid gains, effort, velocity, or transmission efficiency"
    );
    anyhow::ensure!(
        actions.kind == "rne_controller_action_trace"
            && actions.schema_version == 1
            && actions.task_id == task.task_id
            && actions.task_sha256 == sha256(task_bytes)
            && actions.controller_id == controller.controller_id
            && actions.controller_sha256 == sha256(controller_bytes)
            && actions.fixed_delta_ticks == FIXED_DELTA_TICKS
            && actions.action_semantics == "reference_trajectory_before_sensor_feedback"
            && actions.action_joint_order == controller.action_joint_order
            && (1..=10_000).contains(&actions.actions.len()),
        "shared reference artifact identity mismatch"
    );
    for (index, action) in actions.actions.iter().enumerate() {
        anyhow::ensure!(
            action.action_sequence == index as u64
                && action.step == index as u64 + 1
                && action.sim_time_ticks == action.step * FIXED_DELTA_TICKS
                && !action.phase.is_empty()
                && action.joint_position_target_rad.len() == width
                && action
                    .joint_position_target_rad
                    .iter()
                    .all(|value| value.is_finite()),
            "invalid reference action at step {}",
            index + 1
        );
    }
    Ok(())
}

fn validate_pid_law(law: &FeedbackLaw, width: usize) -> Result<()> {
    anyhow::ensure!(
        [
            &law.position_error_gain,
            &law.velocity_damping_s,
            &law.integral_error_gain_s_inv,
            &law.maximum_integral_correction_rad,
            &law.maximum_correction_rad,
            &law.minimum_target_rad,
            &law.maximum_target_rad,
        ]
        .iter()
        .all(|values| values.len() == width && values.iter().all(|value| value.is_finite())),
        "invalid PID feedback law"
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

fn rollout(
    scene: &Path,
    controller: &ControllerSpec,
    actuation: &ActuationConfig,
    actions: &[ActionFrame],
) -> Result<Rollout> {
    let mut sim = MujocoOpenArmSim::new(
        scene,
        actuation.solver_iterations,
        actuation.physics_substeps_per_control_step,
    )?;
    let joint_passive_dynamics = controller
        .rne_actuator_link_order
        .iter()
        .map(|name| {
            sim.find_named(name)
                .with_context(|| format!("missing MuJoCo actuator link {name}"))
                .map(|entity| sim.world.get::<JointPassiveDynamics>(entity).copied())
        })
        .collect::<Result<Vec<_>>>()?;
    sim.configure_actuators(actuation)?;
    sim.install_sensor(controller, JointFeedbackFault::None)?;
    let initial_digest = hash_physics_state_v2(&sim.world);
    let mut bus = InMemoryDataBus::new();
    let mut observations = Vec::with_capacity(actions.len());
    let mut state_hashes = Vec::with_capacity(actions.len());
    let mut decisions = Vec::with_capacity(actions.len());
    let mut latest_controller_observation = None;
    let mut controller_state = ControllerState::new(controller.action_joint_order.len());
    let mut controller_target_history = Vec::with_capacity(actions.len());
    let mut applied_target_history = Vec::with_capacity(actions.len());
    let mut applied_actuation_history = Vec::with_capacity(actions.len());
    let mut last_accepted_target_rad = actions
        .first()
        .context("MuJoCo rollout has no actions")?
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
            latest_controller_observation.as_ref(),
            sim.sim_time.ticks(),
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
        let applied_actuation = sim.step_targets(
            actuation,
            &controller.rne_actuator_link_order,
            &applied_target,
        )?;
        applied_actuation_history.push(applied_actuation);
        decisions.push(decision);
        state_hashes.push(hash_physics_state_v2(&sim.world));
        sim.sample(&mut bus)?;
        let captured = bus
            .latest::<JointFeedback>(JOINT_FEEDBACK_STREAM)
            .context("MuJoCo feedback sensor emitted no current frame")?;
        for (name, joint) in controller
            .rne_actuator_link_order
            .iter()
            .zip(&captured.payload.joints)
        {
            let (sensor_position_rad, sensor_velocity_rad_s) = coordinate(joint)?;
            let (backend_position_rad, backend_velocity_rad_s) = sim.named_state(name)?;
            maximum_sensor_backend_position_delta_rad = maximum_sensor_backend_position_delta_rad
                .max((sensor_position_rad - backend_position_rad).abs());
            maximum_sensor_backend_velocity_delta_rad_s =
                maximum_sensor_backend_velocity_delta_rad_s
                    .max((sensor_velocity_rad_s - backend_velocity_rad_s).abs());
        }
        let now = sim.sim_time;
        if let Some(frame) = bus.latest_available::<JointFeedback>(JOINT_FEEDBACK_STREAM, now) {
            if frame.sequence > last_observation_sequence {
                let published = sensor_sample_published(controller, frame.sequence);
                if published {
                    latest_controller_observation = Some(controller_observation(&frame)?);
                }
                observations.push(observation_frame(
                    frame,
                    now.ticks(),
                    published,
                    &state_hashes,
                    &decisions,
                    &applied_actuation_history,
                )?);
                last_observation_sequence = observations.last().unwrap().step;
            }
        }
    }
    let final_time = SimTime::from_ticks(sim.sim_time.ticks() + FIXED_DELTA_TICKS);
    let final_frame = bus
        .latest_available::<JointFeedback>(JOINT_FEEDBACK_STREAM, final_time)
        .context("final MuJoCo joint feedback did not become available")?;
    if final_frame.sequence > last_observation_sequence {
        let final_published = sensor_sample_published(controller, final_frame.sequence);
        observations.push(observation_frame(
            final_frame,
            final_time.ticks(),
            final_published,
            &state_hashes,
            &decisions,
            &applied_actuation_history,
        )?);
    }
    anyhow::ensure!(
        observations.len() == actions.len(),
        "MuJoCo emitted {} observations for {} actions",
        observations.len(),
        actions.len()
    );
    Ok(Rollout {
        world_seed: sim.world_seed,
        initial_digest,
        final_digest: hash_physics_state_v2(&sim.world),
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

fn intentional_failure(
    scene: &Path,
    controller: &ControllerSpec,
    actuation: &ActuationConfig,
    actions: &[ActionFrame],
    clean: &[ObservationFrame],
) -> Result<bool> {
    let inject_index = usize::try_from(controller.intentional_failure.inject_at_step - 1)?;
    let mut sim = MujocoOpenArmSim::new(
        scene,
        actuation.solver_iterations,
        actuation.physics_substeps_per_control_step,
    )?;
    sim.configure_actuators(actuation)?;
    for observation in &clean[..inject_index] {
        sim.step_targets(
            actuation,
            &controller.rne_actuator_link_order,
            &observation.joint_position_target_rad,
        )?;
    }
    let before = hash_physics_state_v2(&sim.world);
    let truncated =
        &clean[inject_index].joint_position_target_rad[..controller.action_joint_order.len() - 1];
    let rejected = sim
        .step_targets(actuation, &controller.rne_actuator_link_order, truncated)
        .is_err();
    let after_rejection = hash_physics_state_v2(&sim.world);
    anyhow::ensure!(rejected, "MuJoCo runner accepted a truncated action");
    anyhow::ensure!(
        before == after_rejection,
        "rejected MuJoCo action changed ECS state"
    );
    sim.step_targets(
        actuation,
        &controller.rne_actuator_link_order,
        &clean[inject_index].joint_position_target_rad,
    )?;
    anyhow::ensure!(
        sim.sim_time.ticks() == actions[inject_index].sim_time_ticks,
        "rejected MuJoCo action advanced simulation time"
    );
    Ok(before != after_rejection)
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
        _ => bail!("controller feedback contract is incomplete"),
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
        "controller rejected sensor status"
    );
    let age_ticks = consumed_at_ticks.saturating_sub(observation.capture_time_ticks);
    anyhow::ensure!(
        observation.available_time_ticks <= consumed_at_ticks
            && age_ticks <= contract.maximum_age_ticks,
        "controller rejected observation age"
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
    let sample_period_s = contract.sample_period_ticks as f64 / 1_000_000_000.0;
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
            |((((((reference, position), velocity), gain), damping), integral), maximum)| {
                (gain * (reference - position) - damping * velocity + integral)
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
        let (position, velocity) = coordinate(joint)?;
        positions.push(position);
        velocities.push(velocity);
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

fn coordinate(joint: &rne_data::JointFeedbackChannel) -> Result<(f64, f64)> {
    match joint.coordinate {
        rne_data::JointCoordinateFeedback::Revolute {
            position_rad,
            velocity_rad_s,
        } => Ok((position_rad, velocity_rad_s)),
        _ => bail!("feedback channel {} is not revolute", joint.name),
    }
}

fn observation_frame(
    frame: Frame<JointFeedback>,
    consumed_at_ticks: u64,
    sensor_sample_published: bool,
    state_hashes: &[u64],
    decisions: &[ControllerDecision],
    applied_actuations: &[AppliedActuation],
) -> Result<ObservationFrame> {
    let index = usize::try_from(frame.sequence.saturating_sub(1))?;
    let decision = decisions
        .get(index)
        .context("missing controller decision")?;
    let physics_hash = *state_hashes.get(index).context("missing physics hash")?;
    let actuation = applied_actuations
        .get(index)
        .context("missing applied actuation")?;
    let mut positions = Vec::new();
    let mut velocities = Vec::new();
    let mut targets = Vec::new();
    let mut efforts = Vec::new();
    let mut saturated = Vec::new();
    let mut effort_available = Vec::new();
    let mut measured_efforts = Vec::new();
    let mut maximum_actuator_tracking_error_rad = 0.0_f64;
    anyhow::ensure!(
        actuation.target_position_rad.len() == frame.payload.joints.len(),
        "actuation width does not match joint feedback"
    );
    for (joint_index, joint) in frame.payload.joints.iter().enumerate() {
        let (position, velocity) = coordinate(joint)?;
        anyhow::ensure!(
            matches!(
                joint.command,
                rne_data::JointCommandFeedback::Revolute {
                    mode: rne_data::JointCommandMode::Effort,
                    ..
                }
            ),
            "feedback channel {} has no effort command",
            joint.name
        );
        let target = actuation.target_position_rad[joint_index];
        let effort = actuation.limited_effort_command_nm[joint_index];
        let is_saturated = actuation.effort_saturated[joint_index];
        positions.push(position);
        velocities.push(velocity);
        targets.push(target);
        efforts.push(effort);
        saturated.push(is_saturated);
        let measured_effort = match joint.effort {
            JointEffortFeedback::Unavailable => None,
            JointEffortFeedback::Revolute { measured_effort_nm } => Some(measured_effort_nm),
            JointEffortFeedback::Prismatic { .. } => {
                bail!("feedback channel {} has prismatic effort", joint.name)
            }
        };
        effort_available.push(measured_effort.is_some());
        measured_efforts.push(measured_effort);
        maximum_actuator_tracking_error_rad =
            maximum_actuator_tracking_error_rad.max((position - target).abs());
    }
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
        actuator_disturbance_active: frame
            .payload
            .joints
            .iter()
            .zip(&decision.target_position_rad)
            .zip(&actuation.target_position_rad)
            .any(|((_joint, commanded), applied)| (applied - commanded).abs() > 0.0),
        joint_feedback_correction_rad: decision.correction_rad.clone(),
        joint_integral_correction_rad: decision.integral_correction_rad.clone(),
        limited_effort_command_nm: efforts,
        measured_effort_nm: measured_efforts,
        effort_saturated: saturated,
        effort_measurement_available: effort_available,
        maximum_actuator_tracking_error_rad,
        maximum_tracking_error_rad,
        physics_hash,
    })
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

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write {}", path.display()))
}

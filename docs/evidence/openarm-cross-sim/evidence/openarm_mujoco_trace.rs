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
    hash_physics_state_v2, JointActuation, JointMotorGainModel, JointState, PhysicsBackend,
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
    observation_contract: ObservationContract,
    feedback_law: FeedbackLaw,
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
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FeedbackLaw {
    kind: String,
    position_error_gain: Vec<f64>,
    velocity_damping_s: Vec<f64>,
    integral_error_gain_s_inv: Vec<f64>,
    maximum_integral_correction_rad: Vec<f64>,
    maximum_correction_rad: Vec<f64>,
    minimum_target_rad: Vec<f64>,
    maximum_target_rad: Vec<f64>,
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
    controller_observation_sequence: Option<u64>,
    controller_observation_age_ticks: Option<u64>,
    controller_bootstrap: bool,
    joint_position_rad: Vec<f64>,
    joint_velocity_rad_s: Vec<f64>,
    joint_reference_position_rad: Vec<f64>,
    joint_position_target_rad: Vec<f64>,
    joint_feedback_correction_rad: Vec<f64>,
    joint_integral_correction_rad: Vec<f64>,
    limited_effort_command_nm: Vec<f64>,
    effort_saturated: Vec<bool>,
    effort_measurement_available: Vec<bool>,
    maximum_actuator_tracking_error_rad: f64,
    maximum_tracking_error_rad: f64,
    physics_hash: u64,
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
    observation_sequence: Option<u64>,
    observation_age_ticks: Option<u64>,
    bootstrap: bool,
}

struct ControllerState {
    integral_correction_rad: Vec<f64>,
}

impl ControllerState {
    fn new(width: usize) -> Self {
        Self {
            integral_correction_rad: vec![0.0; width],
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
}

struct MujocoOpenArmSim {
    world: World,
    backend: MuJoCoBackend,
    physics_world: PhysicsWorldId,
    sim_time: SimTime,
    fixed_delta: SimDuration,
    world_seed: u64,
}

impl MujocoOpenArmSim {
    fn new(scene_path: &Path, solver_iterations: usize) -> Result<Self> {
        let fixed_delta = SimDuration::from_ticks(FIXED_DELTA_TICKS);
        let mut world = World::new();
        let spawned = load_and_spawn_scene(&mut world, scene_path)
            .with_context(|| format!("load {}", scene_path.display()))?;
        let world_seed = world
            .get::<WorldEntity>(spawned.world)
            .map(|world| world.seed)
            .unwrap_or(0);
        let mut backend = MuJoCoBackend::new(fixed_delta).context("create MuJoCo backend")?;
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
            let position_rad = self
                .world
                .get::<JointState>(entity)
                .copied()
                .and_then(JointState::position_rad)
                .with_context(|| format!("missing initial state for {}", joint.link_name))?;
            self.world.entity_mut(entity).insert((
                JointActuation::RevolutePosition {
                    target_position_rad: position_rad,
                    stiffness_nm_per_rad: joint.stiffness_nm_per_rad,
                    damping_nm_s_per_rad: joint.damping_nm_s_per_rad,
                    max_effort_nm: joint.max_effort_nm,
                },
                JointMotorGainModel::ForceBased,
            ));
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

    fn step_targets(&mut self, link_names: &[String], targets: &[f64]) -> Result<()> {
        anyhow::ensure!(link_names.len() == targets.len(), "action width mismatch");
        for (name, target) in link_names.iter().zip(targets) {
            let entity = self
                .find_named(name)
                .with_context(|| format!("missing actuator {name}"))?;
            let Some(JointActuation::RevolutePosition {
                stiffness_nm_per_rad,
                damping_nm_s_per_rad,
                max_effort_nm,
                ..
            }) = self.world.get::<JointActuation>(entity).copied()
            else {
                bail!("actuator {name} is not in revolute position mode");
            };
            self.world
                .entity_mut(entity)
                .insert(JointActuation::RevolutePosition {
                    target_position_rad: *target,
                    stiffness_nm_per_rad,
                    damping_nm_s_per_rad,
                    max_effort_nm,
                });
        }
        self.backend
            .sync_from_ecs(&mut self.world, self.physics_world)
            .context("upload OpenArm command to MuJoCo")?;
        self.backend
            .step(self.physics_world, self.fixed_delta)
            .context("step OpenArm MuJoCo world")?;
        self.backend
            .sync_to_ecs(&mut self.world, self.physics_world)
            .context("download OpenArm MuJoCo state")?;
        self.sim_time = self.sim_time + self.fixed_delta;
        Ok(())
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
    let controller_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json");
    let task_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json");
    let actuation_path =
        repo_root.join("adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json");
    let robot_asset_path = repo_root.join("assets/robots/openarm_v2_right.rne.robot.toml");
    let actions_path_default =
        repo_root.join("artifacts/openarm-cross-sim/controller-actions.json");
    let mut actions_path = actions_path_default;
    let mut output = repo_root.join("artifacts/openarm-cross-sim");
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--actions" => actions_path = required_path(&mut args, "--actions")?,
            "--output" => output = required_path(&mut args, "--output")?,
            other => bail!("unknown argument {other:?}"),
        }
    }
    let controller_bytes = fs::read(&controller_path)?;
    let task_bytes = fs::read(&task_path)?;
    let actuation_bytes = fs::read(&actuation_path)?;
    let robot_asset_bytes = fs::read(&robot_asset_path)?;
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
    let scene = repo_root.join("assets/scenes/openarm_v2_right_validation.rne.scene.toml");
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
            "actuation_config_sha256": actuation_sha256,
            "fixed_delta_ticks": FIXED_DELTA_TICKS,
            "joint_feedback_schema_version": JointFeedback::SCHEMA_VERSION,
            "joint_feedback_latency_ticks": FIXED_DELTA_TICKS,
            "observation_source": "databus_latest_available",
            "controller_execution": "artifact_defined_joint_feedback_pid",
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
    let contract = &controller.observation_contract;
    anyhow::ensure!(
        contract.kind == "rne_joint_feedback"
            && contract.schema_version == JointFeedback::SCHEMA_VERSION
            && contract.sample_period_ticks == FIXED_DELTA_TICKS
            && contract.phase_offset_ticks == FIXED_DELTA_TICKS
            && contract.latency_ticks == FIXED_DELTA_TICKS
            && contract.maximum_age_ticks == FIXED_DELTA_TICKS
            && contract.required_status == JointFeedbackStatus::Nominal
            && contract.bootstrap_policy == "reference_until_first_available",
        "unsupported controller observation contract"
    );
    let law = &controller.feedback_law;
    anyhow::ensure!(
        law.kind == "joint_position_reference_pid_v1"
            && [
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
        "invalid controller feedback law"
    );
    anyhow::ensure!(
        actuation.kind == "rne_revolute_position_actuation_config"
            && actuation.schema_version == 1
            && actuation.backend_id == "rne_native_physics"
            && actuation.motor_model == "force_based_v1"
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
        actions.kind == "rne_controller_action_trace"
            && actions.schema_version == 1
            && actions.task_id == task.task_id
            && actions.task_sha256 == sha256(task_bytes)
            && actions.controller_id == controller.controller_id
            && actions.controller_sha256 == sha256(controller_bytes)
            && actions.fixed_delta_ticks == FIXED_DELTA_TICKS
            && actions.action_semantics == "reference_trajectory_before_sensor_feedback"
            && actions.action_joint_order == controller.action_joint_order
            && actions.actions.len() == 1_800,
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

fn rollout(
    scene: &Path,
    controller: &ControllerSpec,
    actuation: &ActuationConfig,
    actions: &[ActionFrame],
) -> Result<Rollout> {
    let mut sim = MujocoOpenArmSim::new(scene, actuation.solver_iterations)?;
    sim.configure_actuators(actuation)?;
    sim.install_sensor(controller, JointFeedbackFault::None)?;
    let initial_digest = hash_physics_state_v2(&sim.world);
    let mut bus = InMemoryDataBus::new();
    let mut observations = Vec::with_capacity(actions.len());
    let mut state_hashes = Vec::with_capacity(actions.len());
    let mut decisions = Vec::with_capacity(actions.len());
    let mut latest_controller_observation = None;
    let mut controller_state = ControllerState::new(controller.action_joint_order.len());
    let mut last_observation_sequence = 0;
    let mut maximum_sensor_backend_position_delta_rad = 0.0_f64;
    let mut maximum_sensor_backend_velocity_delta_rad_s = 0.0_f64;
    for action in actions {
        let decision = controller_decision(
            controller,
            &action.joint_position_target_rad,
            &mut controller_state,
            latest_controller_observation.as_ref(),
            sim.sim_time.ticks(),
        )?;
        sim.step_targets(
            &controller.rne_actuator_link_order,
            &decision.target_position_rad,
        )?;
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
                latest_controller_observation = Some(controller_observation(&frame)?);
                observations.push(observation_frame(
                    frame,
                    now.ticks(),
                    &state_hashes,
                    &decisions,
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
        observations.push(observation_frame(
            final_frame,
            final_time.ticks(),
            &state_hashes,
            &decisions,
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
    })
}

fn intentional_failure(
    scene: &Path,
    controller: &ControllerSpec,
    actuation: &ActuationConfig,
    actions: &[ActionFrame],
    clean: &[ObservationFrame],
) -> Result<bool> {
    let inject_index = usize::try_from(controller.intentional_failure.inject_at_step - 1)?;
    let mut sim = MujocoOpenArmSim::new(scene, actuation.solver_iterations)?;
    sim.configure_actuators(actuation)?;
    for observation in &clean[..inject_index] {
        sim.step_targets(
            &controller.rne_actuator_link_order,
            &observation.joint_position_target_rad,
        )?;
    }
    let before = hash_physics_state_v2(&sim.world);
    let truncated =
        &clean[inject_index].joint_position_target_rad[..controller.action_joint_order.len() - 1];
    let rejected = sim
        .step_targets(&controller.rne_actuator_link_order, truncated)
        .is_err();
    let after_rejection = hash_physics_state_v2(&sim.world);
    anyhow::ensure!(rejected, "MuJoCo runner accepted a truncated action");
    anyhow::ensure!(
        before == after_rejection,
        "rejected MuJoCo action changed ECS state"
    );
    sim.step_targets(
        &controller.rne_actuator_link_order,
        &clean[inject_index].joint_position_target_rad,
    )?;
    anyhow::ensure!(
        sim.sim_time.ticks() == actions[inject_index].sim_time_ticks,
        "rejected MuJoCo action advanced simulation time"
    );
    Ok(before != after_rejection)
}

fn controller_decision(
    controller: &ControllerSpec,
    reference: &[f64],
    state: &mut ControllerState,
    observation: Option<&ControllerObservation>,
    consumed_at_ticks: u64,
) -> Result<ControllerDecision> {
    let Some(observation) = observation else {
        return Ok(ControllerDecision {
            reference_position_rad: reference.to_vec(),
            target_position_rad: reference.to_vec(),
            correction_rad: vec![0.0; reference.len()],
            integral_correction_rad: state.integral_correction_rad.clone(),
            observation_sequence: None,
            observation_age_ticks: None,
            bootstrap: true,
        });
    };
    let contract = &controller.observation_contract;
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
    let law = &controller.feedback_law;
    let sample_period_s =
        controller.observation_contract.sample_period_ticks as f64 / 1_000_000_000.0;
    for (((integral, gain), maximum), (reference, position)) in state
        .integral_correction_rad
        .iter_mut()
        .zip(&law.integral_error_gain_s_inv)
        .zip(&law.maximum_integral_correction_rad)
        .zip(reference.iter().zip(&observation.joint_position_rad))
    {
        *integral =
            (*integral + gain * (reference - position) * sample_period_s).clamp(-maximum, *maximum);
    }
    let correction_rad = reference
        .iter()
        .zip(&observation.joint_position_rad)
        .zip(&observation.joint_velocity_rad_s)
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
        observation_sequence: Some(observation.sequence),
        observation_age_ticks: Some(age_ticks),
        bootstrap: false,
    })
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
    state_hashes: &[u64],
    decisions: &[ControllerDecision],
) -> Result<ObservationFrame> {
    let index = usize::try_from(frame.sequence.saturating_sub(1))?;
    let decision = decisions
        .get(index)
        .context("missing controller decision")?;
    let physics_hash = *state_hashes.get(index).context("missing physics hash")?;
    let mut positions = Vec::new();
    let mut velocities = Vec::new();
    let mut targets = Vec::new();
    let mut efforts = Vec::new();
    let mut saturated = Vec::new();
    let mut effort_available = Vec::new();
    let mut maximum_actuator_tracking_error_rad = 0.0_f64;
    for joint in &frame.payload.joints {
        let (position, velocity) = coordinate(joint)?;
        let (target, effort, is_saturated) = match joint.command {
            rne_data::JointCommandFeedback::Revolute {
                target_position_rad: Some(target),
                limited_effort_command_nm,
                saturated,
                ..
            } => (target, limited_effort_command_nm, saturated),
            _ => bail!("feedback channel {} has no position command", joint.name),
        };
        positions.push(position);
        velocities.push(velocity);
        targets.push(target);
        efforts.push(effort);
        saturated.push(is_saturated);
        effort_available.push(!matches!(joint.effort, JointEffortFeedback::Unavailable));
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
        controller_observation_sequence: decision.observation_sequence,
        controller_observation_age_ticks: decision.observation_age_ticks,
        controller_bootstrap: decision.bootstrap,
        joint_position_rad: positions,
        joint_velocity_rad_s: velocities,
        joint_reference_position_rad: decision.reference_position_rad.clone(),
        joint_position_target_rad: targets,
        joint_feedback_correction_rad: decision.correction_rad.clone(),
        joint_integral_correction_rad: decision.integral_correction_rad.clone(),
        limited_effort_command_nm: efforts,
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

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write {}", path.display()))
}

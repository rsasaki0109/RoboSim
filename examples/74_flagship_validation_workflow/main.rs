//! One headless v0.7 workflow: inspect, yield to traffic, navigate, and place.
//!
//! The workflow intentionally keeps orchestration at the example boundary. The
//! robot episode and traffic runtime remain independently testable, while typed
//! behavior contracts observe their shared, fixed-step task state.

use anyhow::{bail, Context, Result};
use bevy_ecs::prelude::{Entity, World};
#[cfg(feature = "mujoco")]
use rne_ai::MobileManipulatorPhysicsFactory;
use rne_ai::{
    minimize_behavior_failure, run_behavior_scenarios_with_replays, stable_behavior_digest,
    verify_behavior_replay, ActionSpec, BehaviorContract, BehaviorContractError, BehaviorDimension,
    BehaviorDimensionValue, BehaviorFailureCase, BehaviorReport, BehaviorRun, BehaviorScenario,
    BehaviorScenarioStep, Episode, GraspMode, IkMobileLiftPickPlacePolicy, MobileLiftFailureClass,
    MobileLiftPickPlacePhase, MobileManipulatorAction, MobileManipulatorEpisode,
    MobileManipulatorEpisodeConfig, MobileManipulatorObservation, ObservationSpec, Policy,
    RandomDistributionSpec, RandomizationParameterSpec, RandomizationSpec, ResetSpec, RewardSpec,
    RewardTermSpec, TaskSpec, TensorBounds, TensorDType, TensorSpec, TerminationConditionSpec,
    TerminationKind, TerminationSpec,
};
use rne_assets::{load_scene_bundle, scene_dependency_paths};
use rne_core::{SimDuration, SimTime};
use rne_ecs::EntityUuid;
use rne_physics::hash_physics_state;
#[cfg(feature = "mujoco")]
use rne_physics_mujoco::MuJoCoBackend;
use rne_traffic::{
    advance_controlled_kinematic_traffic, KinematicTrafficConfig, SignalAspect, TrafficActor,
    TrafficDeparture, TrafficId, TrafficPose, TrafficRoute, TrafficRouteCatalog,
    TrafficRouteFollower, TrafficRuntime, TrafficSignalControl, TrafficSignalControls,
};
use serde::Serialize;
use std::ffi::OsStr;
use std::fs;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};
use uuid::Uuid;

const SCENARIO: &str = "mobile_lift_shared_aisle_inspection_pick_place";
const REPORT_KIND: &str = "rne_flagship_workflow_report";
const REPORT_SCHEMA_VERSION: u32 = 1;
#[cfg(feature = "mujoco")]
const CROSS_BACKEND_REPORT_KIND: &str = "rne_flagship_cross_backend_report";
#[cfg(feature = "mujoco")]
const CROSS_BACKEND_REPORT_SCHEMA_VERSION: u32 = 1;
const TASK_ID: &str = "rne.flagship.mobile_lift_shared_aisle.v1";
const SEED: u64 = 7;
const SIGNAL_RELEASE_STEP: u64 = 60;
const REQUIRED_INSPECTION_FRAMES: u32 = 3;
const MAX_WORKFLOW_STEPS: u64 = 8_000;
const BLACKOUT_DIMENSION: &str = "perception_blackout";
const DEPARTURE_DIMENSION: &str = "traffic_departure_delay_s";
const SPEED_DIMENSION: &str = "traffic_speed_delta_m_s";
const EXPECTED_FAILURE_CONTRACT: &str = "perception_stream_alive";
const ROBOT_NAME: &str = "mm_mobile_lift";
const PAYLOAD_NAME: &str = "mobile_lift_cube";
const TRAFFIC_NAME: &str = "aisle_vehicle_1";
const SIGNAL_NAME: &str = "aisle_signal";
#[cfg(feature = "mujoco")]
const COMPLETION_STEP_DELTA_MAX: f64 = 500.0;
#[cfg(feature = "mujoco")]
const BASE_PLANAR_DELTA_MAX_M: f64 = 0.40;
#[cfg(feature = "mujoco")]
const PAYLOAD_POSITION_DELTA_MAX_M: f64 = 0.06;
#[cfg(feature = "mujoco")]
const PAYLOAD_APEX_DELTA_MAX_M: f64 = 0.07;
#[cfg(feature = "mujoco")]
const ARM_JOINT_DELTA_MAX_RAD: f64 = 0.20;
#[cfg(feature = "mujoco")]
const LIFT_DELTA_MAX_M: f64 = 0.04;
#[cfg(feature = "mujoco")]
const GRIPPER_DELTA_MAX_M: f64 = 0.04;
#[cfg(feature = "mujoco")]
const WRIST_DEPTH_DELTA_MAX_M: f64 = 0.02;
#[cfg(feature = "mujoco")]
const REWARD_DELTA_MAX: f64 = 0.75;

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
enum FlagshipPhysicsBackend {
    #[default]
    Rapier,
    #[cfg(feature = "mujoco")]
    Mujoco,
}

impl FlagshipPhysicsBackend {
    const fn as_str(self) -> &'static str {
        match self {
            Self::Rapier => "rapier_native",
            #[cfg(feature = "mujoco")]
            Self::Mujoco => "mujoco_native",
        }
    }
}

#[cfg(feature = "mujoco")]
fn mujoco_physics_factory() -> MobileManipulatorPhysicsFactory<MuJoCoBackend> {
    MobileManipulatorPhysicsFactory::new("mujoco_native", |fixed_delta| {
        MuJoCoBackend::new(fixed_delta).map_err(|error| error.to_string())
    })
    .with_preflight(|backend, world| {
        backend
            .preflight_world(world)
            .map_err(|error| error.to_string())
    })
}

#[derive(Clone, Debug, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
struct FlagshipObservation {
    workflow_step: u64,
    policy_phase: String,
    policy_phase_index: i32,
    policy_failure: String,
    inspection_complete: bool,
    perception_valid: bool,
    wrist_camera_pixels: usize,
    wrist_depth_min_m: f64,
    traffic_signal_green: bool,
    traffic_clear: bool,
    traffic_actor_x_m: f64,
    traffic_actor_y_m: f64,
    traffic_actor_z_m: f64,
    traffic_stable_hash: u64,
    traffic_collision_count: usize,
    traffic_signal_violation_count: usize,
    deterministic_event_count: u32,
    motion_commanded: bool,
    base_x_m: f64,
    base_z_m: f64,
    shoulder_position_rad: f64,
    elbow_position_rad: f64,
    wrist_yaw_position_rad: f64,
    lift_position_m: f64,
    gripper_position_m: f64,
    payload_x_m: f64,
    payload_y_m: f64,
    payload_z_m: f64,
    maximum_payload_y_m: f64,
    grasped_once: bool,
    total_reward: f64,
    task_completed: bool,
    robot_terminated: bool,
    robot_truncated: bool,
    fault_injected: bool,
    fail_closed_abort: bool,
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ScenarioOverrides {
    perception_blackout: bool,
    traffic_departure_delay_s: f64,
    traffic_speed_delta_m_s: f64,
}

struct FlagshipScenario {
    episode: MobileManipulatorEpisode,
    physics_backend: FlagshipPhysicsBackend,
    policy: IkMobileLiftPickPlacePolicy,
    robot_observation: MobileManipulatorObservation,
    robot_terminated: bool,
    robot_truncated: bool,
    traffic_world: World,
    traffic_actor: Entity,
    traffic_routes: TrafficRouteCatalog,
    traffic_controls: TrafficSignalControls,
    traffic_runtime: TrafficRuntime,
    traffic_config: KinematicTrafficConfig,
    traffic_signal_green: bool,
    traffic_clear: bool,
    traffic_stable_hash: u64,
    traffic_collision_count: usize,
    traffic_signal_violation_count: usize,
    fixed_delta: SimDuration,
    workflow_step: u64,
    deterministic_event_count: u32,
    inspection_valid_streak: u32,
    inspection_complete: bool,
    grasped_once: bool,
    payload_resting_y_m: f64,
    maximum_payload_y_m: f64,
    task_completed: bool,
    fail_closed_abort: bool,
    fault_injected: bool,
    fault_step: u64,
    dimensions: Vec<BehaviorDimension>,
    scene_input_digest: u64,
    trace: Option<Arc<Mutex<Vec<FlagshipObservation>>>>,
}

impl FlagshipScenario {
    fn clean_with_physics(seed: u64, physics_backend: FlagshipPhysicsBackend) -> Result<Self> {
        Self::from_dimensions_with_physics(seed, &seeded_dimensions(seed, false)?, physics_backend)
    }

    fn fault_fixture(seed: u64) -> Result<Self> {
        Self::from_dimensions(seed, &seeded_dimensions(seed, true)?)
    }

    fn from_dimensions(seed: u64, dimensions: &[BehaviorDimension]) -> Result<Self> {
        Self::from_dimensions_with_physics(seed, dimensions, FlagshipPhysicsBackend::Rapier)
    }

    fn from_dimensions_with_physics(
        seed: u64,
        dimensions: &[BehaviorDimension],
        physics_backend: FlagshipPhysicsBackend,
    ) -> Result<Self> {
        let overrides = decode_dimensions(dimensions)?;
        let mut episode_config = MobileManipulatorEpisodeConfig::mobile_lift_pick_place();
        episode_config.max_steps = MAX_WORKFLOW_STEPS;
        episode_config.rng_seed = seed;
        let scene_input_digest = digest_scene_inputs(&episode_config.scene_path)?.0;
        let mut episode = match physics_backend {
            FlagshipPhysicsBackend::Rapier => MobileManipulatorEpisode::try_new(episode_config)?,
            #[cfg(feature = "mujoco")]
            FlagshipPhysicsBackend::Mujoco => MobileManipulatorEpisode::try_new_with_physics(
                episode_config,
                mujoco_physics_factory(),
            )?,
        };
        let initial = episode.reset();
        episode.set_grasp_mode(GraspMode::Friction);
        let payload_resting_y_m = episode
            .simulation()
            .named_translation_m(PAYLOAD_NAME)
            .context("flagship payload is missing")?
            .1;
        let fixed_delta = episode.simulation().fixed_delta();
        let (traffic_world, traffic_actor, traffic_routes, traffic_controls) =
            build_traffic(overrides)?;
        Ok(Self {
            episode,
            physics_backend,
            policy: IkMobileLiftPickPlacePolicy::new(),
            robot_observation: initial.observation,
            robot_terminated: initial.terminated,
            robot_truncated: initial.truncated,
            traffic_world,
            traffic_actor,
            traffic_routes,
            traffic_controls,
            traffic_runtime: TrafficRuntime::default(),
            traffic_config: KinematicTrafficConfig::default(),
            traffic_signal_green: false,
            traffic_clear: false,
            traffic_stable_hash: 0,
            traffic_collision_count: 0,
            traffic_signal_violation_count: 0,
            fixed_delta,
            workflow_step: 0,
            deterministic_event_count: 0,
            inspection_valid_streak: 0,
            inspection_complete: false,
            grasped_once: false,
            payload_resting_y_m,
            maximum_payload_y_m: payload_resting_y_m,
            task_completed: false,
            fail_closed_abort: false,
            fault_injected: false,
            fault_step: 300 + seed % 31,
            dimensions: dimensions.to_vec(),
            scene_input_digest,
            trace: None,
        })
    }

    fn with_trace(mut self, trace: Arc<Mutex<Vec<FlagshipObservation>>>) -> Self {
        self.trace = Some(trace);
        self
    }

    fn traffic_actor_x_m(&self) -> f64 {
        self.traffic_actor_pose().position_m[0]
    }

    fn traffic_actor_pose(&self) -> TrafficPose {
        *self
            .traffic_world
            .get::<TrafficPose>(self.traffic_actor)
            .expect("flagship traffic actor keeps its pose")
    }

    fn perception_valid(observation: &MobileManipulatorObservation) -> bool {
        observation.wrist_camera_pixels > 0
            && observation.wrist_depth_min_m.is_finite()
            && observation.wrist_depth_min_m > 0.0
    }

    fn observation(&self, motion_commanded: bool) -> FlagshipObservation {
        let payload = self
            .episode
            .simulation()
            .named_translation_m(PAYLOAD_NAME)
            .expect("flagship payload keeps its stable name");
        let perception_valid =
            !self.fault_injected && Self::perception_valid(&self.robot_observation);
        let traffic_pose = self.traffic_actor_pose();
        FlagshipObservation {
            workflow_step: self.workflow_step,
            policy_phase: phase_name(self.policy.phase()).to_string(),
            policy_phase_index: phase_index(self.policy.phase()),
            policy_failure: failure_name(self.policy.failure_class(&self.robot_observation))
                .to_string(),
            inspection_complete: self.inspection_complete,
            perception_valid,
            wrist_camera_pixels: if perception_valid {
                self.robot_observation.wrist_camera_pixels
            } else {
                0
            },
            wrist_depth_min_m: if perception_valid {
                self.robot_observation.wrist_depth_min_m
            } else {
                0.0
            },
            traffic_signal_green: self.traffic_signal_green,
            traffic_clear: self.traffic_clear,
            traffic_actor_x_m: traffic_pose.position_m[0],
            traffic_actor_y_m: traffic_pose.position_m[1],
            traffic_actor_z_m: traffic_pose.position_m[2],
            traffic_stable_hash: self.traffic_stable_hash,
            traffic_collision_count: self.traffic_collision_count,
            traffic_signal_violation_count: self.traffic_signal_violation_count,
            deterministic_event_count: self.deterministic_event_count,
            motion_commanded,
            base_x_m: self.robot_observation.base_x_m,
            base_z_m: self.robot_observation.base_z_m,
            shoulder_position_rad: self.robot_observation.shoulder_position_rad,
            elbow_position_rad: self.robot_observation.elbow_position_rad,
            wrist_yaw_position_rad: self.robot_observation.wrist_yaw_position_rad,
            lift_position_m: self.robot_observation.lift_position_m,
            gripper_position_m: self.robot_observation.gripper_position_m,
            payload_x_m: payload.0,
            payload_y_m: payload.1,
            payload_z_m: payload.2,
            maximum_payload_y_m: self.maximum_payload_y_m,
            grasped_once: self.grasped_once,
            total_reward: self.episode.total_reward(),
            task_completed: self.task_completed,
            robot_terminated: self.robot_terminated,
            robot_truncated: self.robot_truncated,
            fault_injected: self.fault_injected,
            fail_closed_abort: self.fail_closed_abort,
        }
    }

    fn record_trace(&self, observation: &FlagshipObservation) {
        if let Some(trace) = &self.trace {
            let mut trace = trace.lock().expect("flagship trace mutex is not poisoned");
            if trace
                .last()
                .is_none_or(|previous| previous.workflow_step != observation.workflow_step)
            {
                trace.push(observation.clone());
            }
        }
    }
}

impl BehaviorScenario for FlagshipScenario {
    type Observation = FlagshipObservation;

    fn fixed_delta(&self) -> SimDuration {
        self.fixed_delta
    }

    fn initial_observation(&self) -> Self::Observation {
        let observation = self.observation(false);
        self.record_trace(&observation);
        observation
    }

    fn state_digest(&self, observation: &Self::Observation) -> u64 {
        let mut bytes = b"rne_flagship_workflow_state_v1".to_vec();
        bytes.extend_from_slice(
            &hash_physics_state(self.episode.simulation().world()).to_le_bytes(),
        );
        bytes.extend_from_slice(&self.traffic_stable_hash.to_le_bytes());
        bytes.extend_from_slice(
            &serde_json::to_vec(observation).expect("flagship observation serializes"),
        );
        stable_behavior_digest(&bytes)
    }

    fn scenario_digest(&self) -> u64 {
        let mut bytes = b"rne_flagship_workflow_inputs_v1".to_vec();
        bytes.extend_from_slice(&self.scene_input_digest.to_le_bytes());
        bytes.extend_from_slice(&self.fixed_delta.ticks().to_le_bytes());
        bytes.extend_from_slice(&SIGNAL_RELEASE_STEP.to_le_bytes());
        bytes.extend_from_slice(&self.fault_step.to_le_bytes());
        bytes.extend_from_slice(self.physics_backend.as_str().as_bytes());
        bytes.extend_from_slice(
            &serde_json::to_vec(&flagship_task_spec(self.fixed_delta.ticks()))
                .expect("flagship TaskSpec serializes"),
        );
        bytes.extend_from_slice(
            &serde_json::to_vec(&self.dimensions).expect("flagship dimensions serialize"),
        );
        stable_behavior_digest(&bytes)
    }

    fn behavior_dimensions(&self) -> Vec<BehaviorDimension> {
        self.dimensions.clone()
    }

    fn contracts(&self) -> Result<Vec<BehaviorContract<Self::Observation>>, BehaviorContractError> {
        let traffic_deadline = SimDuration::from_ticks(self.fixed_delta.ticks() * 600);
        let task_deadline = SimDuration::from_ticks(self.fixed_delta.ticks() * MAX_WORKFLOW_STEPS);
        Ok(vec![
            BehaviorContract::always("finite_observation", observation_is_finite)?
                .with_entities([ROBOT_NAME, PAYLOAD_NAME, TRAFFIC_NAME])?,
            BehaviorContract::always(
                EXPECTED_FAILURE_CONTRACT,
                |observation: &FlagshipObservation| {
                    !observation.inspection_complete || observation.perception_valid
                },
            )?
            .with_entities([ROBOT_NAME, "wrist_rgbd"])?,
            BehaviorContract::always(
                "collision_free_traffic",
                |observation: &FlagshipObservation| observation.traffic_collision_count == 0,
            )?
            .with_entities([TRAFFIC_NAME])?,
            BehaviorContract::always("traffic_interlock", |observation: &FlagshipObservation| {
                !observation.motion_commanded
                    || (observation.inspection_complete && observation.traffic_clear)
            })?
            .with_entities([ROBOT_NAME, TRAFFIC_NAME, SIGNAL_NAME])?,
            BehaviorContract::eventually(
                "inspection_completed",
                traffic_deadline,
                |observation: &FlagshipObservation| observation.inspection_complete,
            )?
            .with_entities([ROBOT_NAME, "wrist_rgbd"])?,
            BehaviorContract::eventually(
                "traffic_cleared_shared_aisle",
                traffic_deadline,
                |observation: &FlagshipObservation| observation.traffic_clear,
            )?
            .with_entities([TRAFFIC_NAME, SIGNAL_NAME])?,
            BehaviorContract::eventually(
                "inspection_pick_place_completed",
                task_deadline,
                |observation: &FlagshipObservation| observation.task_completed,
            )?
            .with_entities([ROBOT_NAME, PAYLOAD_NAME])?,
        ])
    }

    fn advance(&mut self) -> BehaviorScenarioStep<Self::Observation> {
        self.workflow_step += 1;
        if self.workflow_step == SIGNAL_RELEASE_STEP {
            self.traffic_controls
                .set_aspect(&traffic_id(SIGNAL_NAME), SignalAspect::Green)
                .expect("flagship signal exists");
            self.traffic_signal_green = true;
            self.deterministic_event_count += 1;
        }
        let traffic_step = advance_controlled_kinematic_traffic(
            &mut self.traffic_world,
            &self.traffic_routes,
            &self.traffic_controls,
            &mut self.traffic_runtime,
            SimTime::from_ticks(self.workflow_step * self.fixed_delta.ticks()),
            self.fixed_delta,
            self.traffic_config,
        )
        .expect("validated flagship traffic step");
        self.traffic_stable_hash = traffic_step.stable_state_hash;
        self.traffic_collision_count += traffic_step.collision_count;
        self.traffic_signal_violation_count += traffic_step.signal_violation_count;
        self.traffic_clear = self.traffic_actor_x_m() >= 0.8;

        let raw_perception_valid = Self::perception_valid(&self.robot_observation);
        self.inspection_valid_streak = if raw_perception_valid {
            self.inspection_valid_streak.saturating_add(1)
        } else {
            0
        };
        self.inspection_complete |= self.inspection_valid_streak >= REQUIRED_INSPECTION_FRAMES;

        let blackout_enabled = dimension_boolean(&self.dimensions, BLACKOUT_DIMENSION);
        if blackout_enabled && self.workflow_step == self.fault_step {
            self.fault_injected = true;
            self.fail_closed_abort = true;
            self.deterministic_event_count += 1;
        }

        let permitted = self.inspection_complete
            && self.traffic_clear
            && raw_perception_valid
            && !self.fail_closed_abort;
        let action = if permitted {
            self.policy.act(&self.robot_observation)
        } else {
            MobileManipulatorAction::default()
        };
        let motion_commanded = action_commands_motion(&action);
        let robot_step = self.episode.step(action);
        self.robot_observation = robot_step.observation;
        self.robot_terminated = robot_step.terminated;
        self.robot_truncated = robot_step.truncated;
        self.grasped_once |= self.episode.simulation().is_grasping();
        let payload_y_m = self
            .episode
            .simulation()
            .named_translation_m(PAYLOAD_NAME)
            .expect("flagship payload keeps its stable name")
            .1;
        self.maximum_payload_y_m = self.maximum_payload_y_m.max(payload_y_m);
        self.task_completed = self.robot_terminated
            && self.grasped_once
            && self.maximum_payload_y_m > self.payload_resting_y_m + 0.12;

        let observation = self.observation(motion_commanded);
        self.record_trace(&observation);
        BehaviorScenarioStep {
            observation,
            done: self.fail_closed_abort || robot_step.is_done(),
        }
    }
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct WorkflowRunSummary {
    status: &'static str,
    steps: u64,
    sim_time_ticks: u64,
    final_state_digest: u64,
    contract_count: usize,
    behavior_report: String,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct WorkflowFailureSummary {
    expected_contract: &'static str,
    injected_step: u64,
    original_replay: String,
    minimized_replay: String,
    minimized_case: String,
    active_dimensions_before: usize,
    active_dimensions_after: usize,
    minimization_attempts: u32,
    matched_replay_frames: usize,
}

#[derive(Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct FlagshipWorkflowReport {
    schema_version: u32,
    kind: &'static str,
    scenario: &'static str,
    seed: u64,
    task_id: &'static str,
    fixed_delta_ticks: u64,
    imported_asset_digest: u64,
    imported_assets: Vec<String>,
    task_spec: String,
    physics_execution_paths: Vec<&'static str>,
    deterministic_events: Vec<&'static str>,
    success: WorkflowRunSummary,
    intentional_failure: WorkflowFailureSummary,
    browser_inspector: String,
    failure_capsule: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    cross_backend_report: Option<String>,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct CrossBackendOutcome {
    backend_id: &'static str,
    status: &'static str,
    steps: u64,
    sim_time_ticks: u64,
    final_state_digest: u64,
    behavior_report: String,
    final_observation: FlagshipObservation,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct CrossBackendCheck {
    id: &'static str,
    quantity: &'static str,
    unit: &'static str,
    observed_delta: f64,
    maximum_delta: f64,
    status: &'static str,
}

#[derive(Clone, Debug, Serialize)]
#[serde(deny_unknown_fields)]
struct CrossBackendReport {
    schema_version: u32,
    kind: &'static str,
    status: &'static str,
    scenario: &'static str,
    seed: u64,
    task_id: &'static str,
    task_spec: &'static str,
    task_spec_digest: u64,
    fixed_delta_ticks: u64,
    comparison_contract: &'static str,
    exact_outcomes: Vec<&'static str>,
    state_digest_contract: &'static str,
    backends: Vec<CrossBackendOutcome>,
    tolerance_checks: Vec<CrossBackendCheck>,
}

#[derive(Clone, Debug)]
struct Cli {
    output: PathBuf,
    cross_backend: bool,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("flagship validation failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let cli = parse_cli()?;
    let output = cli.output;
    fs::create_dir_all(&output)
        .with_context(|| format!("could not create {}", output.display()))?;

    let (clean_run, success_trace) = run_clean_flagship(FlagshipPhysicsBackend::Rapier)?;

    #[cfg(feature = "mujoco")]
    let cross_backend_evidence = if cli.cross_backend {
        let (mujoco_run, mujoco_trace) = run_clean_flagship(FlagshipPhysicsBackend::Mujoco)?;
        Some(build_cross_backend_report(
            &clean_run.report,
            &success_trace,
            &mujoco_run.report,
            &mujoco_trace,
        )?)
    } else {
        None
    };
    #[cfg(not(feature = "mujoco"))]
    let cross_backend_evidence: Option<(CrossBackendReport, BehaviorReport)> = {
        if cli.cross_backend {
            bail!("--cross-backend requires --features mujoco and a MuJoCo 3.9 runtime");
        }
        None
    };

    let mut failure_run = run_behavior_scenarios_with_replays(SCENARIO, [SEED], |seed| {
        FlagshipScenario::fault_fixture(seed)
    })?;
    if failure_run.report.passed() || failure_run.failure_replays.len() != 1 {
        bail!("intentional flagship failure did not emit exactly one replay");
    }
    let original = failure_run
        .failure_replays
        .pop()
        .expect("failure replay count checked");
    if original.failure.contract.name != EXPECTED_FAILURE_CONTRACT {
        bail!(
            "expected {EXPECTED_FAILURE_CONTRACT}, got {}",
            original.failure.contract.name
        );
    }
    let minimized = minimize_behavior_failure(&original, |dimensions| {
        let candidate = run_behavior_scenarios_with_replays(SCENARIO, [SEED], |seed| {
            FlagshipScenario::from_dimensions(seed, dimensions)
        })?;
        Ok::<_, rne_ai::BehaviorReplayError>(candidate.failure_replays.into_iter().next())
    })?;
    if minimized.active_dimensions_before != 3 || minimized.active_dimensions_after != 1 {
        bail!(
            "flagship minimization retained {}/{} active dimensions; expected 1/3",
            minimized.active_dimensions_after,
            minimized.active_dimensions_before
        );
    }
    let verification = verify_behavior_replay(&minimized.artifact, |seed, dimensions| {
        FlagshipScenario::from_dimensions(seed, dimensions)
    })?;

    let success_report_path = output.join("success.behavior-report.json");
    let failure_report_path = output.join("failure.behavior-report.json");
    let original_replay_path = output.join("failure.rne-replay");
    let minimized_replay_path = output.join("failure-minimized.rne-replay");
    let minimized_case_path = output.join("failure-minimized.behavior-case.json");
    let browser_path = output.join("replay-inspector.html");
    let task_spec_path = output.join("flagship.task.json");
    let summary_path = output.join("workflow-report.json");
    let cross_backend_path = output.join("cross-backend-report.json");
    let mujoco_success_path = output.join("mujoco-success.behavior-report.json");

    original.write_json(&original_replay_path)?;
    minimized.artifact.write_json(&minimized_replay_path)?;
    BehaviorFailureCase::from_replay(&minimized.artifact).write_json(&minimized_case_path)?;
    failure_run.report.set_failure_artifacts(
        SEED,
        Some("failure.rne-replay".to_string()),
        Some("failure-minimized.rne-replay".to_string()),
        Some("failure-minimized.behavior-case.json".to_string()),
    );
    write_pretty_json(&success_report_path, &clean_run.report)?;
    write_pretty_json(&failure_report_path, &failure_run.report)?;
    if let Some((cross_backend_report, mujoco_report)) = &cross_backend_evidence {
        write_pretty_json(&cross_backend_path, cross_backend_report)?;
        write_pretty_json(&mujoco_success_path, mujoco_report)?;
    }
    write_browser_inspector(&browser_path, &success_trace, &minimized.artifact)?;
    let task_spec = flagship_task_spec(minimized.artifact.fixed_delta_ticks);
    task_spec
        .validate()
        .context("generated flagship TaskSpec is invalid")?;
    write_pretty_json(&task_spec_path, &task_spec)?;

    let (imported_asset_digest, imported_assets) =
        digest_scene_inputs(&rne_ai::mm_mobile_lift_pick_place_scene_path())?;
    let success_seed = only_seed(&clean_run.report)?;
    let mut physics_execution_paths = vec!["rapier_native"];
    if cross_backend_evidence.is_some() {
        physics_execution_paths.push("mujoco_native");
    }
    let report = FlagshipWorkflowReport {
        schema_version: REPORT_SCHEMA_VERSION,
        kind: REPORT_KIND,
        scenario: SCENARIO,
        seed: SEED,
        task_id: TASK_ID,
        fixed_delta_ticks: minimized.artifact.fixed_delta_ticks,
        imported_asset_digest,
        imported_assets,
        task_spec: "flagship.task.json".to_string(),
        physics_execution_paths,
        deterministic_events: vec!["traffic_signal_green", "seeded_perception_blackout"],
        success: WorkflowRunSummary {
            status: "passed",
            steps: success_seed.steps,
            sim_time_ticks: success_seed.sim_time_ticks,
            final_state_digest: success_seed.final_state_digest,
            contract_count: success_seed.contracts.len(),
            behavior_report: "success.behavior-report.json".to_string(),
        },
        intentional_failure: WorkflowFailureSummary {
            expected_contract: EXPECTED_FAILURE_CONTRACT,
            injected_step: original.failure.violation.step,
            original_replay: "failure.rne-replay".to_string(),
            minimized_replay: "failure-minimized.rne-replay".to_string(),
            minimized_case: "failure-minimized.behavior-case.json".to_string(),
            active_dimensions_before: minimized.active_dimensions_before,
            active_dimensions_after: minimized.active_dimensions_after,
            minimization_attempts: minimized.attempts,
            matched_replay_frames: verification.matched_frames,
        },
        browser_inspector: "replay-inspector.html".to_string(),
        failure_capsule: "failure-capsule/capsule.json".to_string(),
        cross_backend_report: cross_backend_evidence
            .as_ref()
            .map(|_| "cross-backend-report.json".to_string()),
    };
    write_pretty_json(&summary_path, &report)?;

    if cross_backend_evidence
        .as_ref()
        .is_some_and(|(report, _)| report.status != "passed")
    {
        bail!("Rapier/MuJoCo flagship comparison exceeded its registered contract");
    }

    println!(
        "flagship workflow passed: success_steps={} failure_step={} minimized_dimensions={}/{}\nartifacts: {}",
        success_seed.steps,
        original.failure.violation.step,
        minimized.active_dimensions_after,
        minimized.active_dimensions_before,
        output.display()
    );
    Ok(())
}

fn parse_cli() -> Result<Cli> {
    let mut output = None;
    let mut cross_backend = false;
    for argument in std::env::args().skip(1) {
        if argument == "--cross-backend" {
            cross_backend = true;
        } else if argument.starts_with('-') {
            bail!("unknown flagship argument `{argument}`");
        } else if output.replace(PathBuf::from(argument)).is_some() {
            bail!("expected at most one output-directory argument");
        }
    }
    Ok(Cli {
        output: output.unwrap_or_else(|| PathBuf::from("artifacts/flagship-validation")),
        cross_backend,
    })
}

fn run_clean_flagship(
    physics_backend: FlagshipPhysicsBackend,
) -> Result<(BehaviorRun, Vec<FlagshipObservation>)> {
    let trace = Arc::new(Mutex::new(Vec::new()));
    let captured = Arc::clone(&trace);
    let run = run_behavior_scenarios_with_replays(SCENARIO, [SEED], |seed| {
        FlagshipScenario::clean_with_physics(seed, physics_backend)
            .map(|scenario| scenario.with_trace(Arc::clone(&captured)))
    })?;
    if !run.report.passed() || !run.failure_replays.is_empty() {
        bail!(
            "{} clean flagship run did not satisfy every behavior contract",
            physics_backend.as_str()
        );
    }
    let trace = trace
        .lock()
        .expect("flagship trace mutex is not poisoned")
        .clone();
    if !trace.last().is_some_and(semantic_outcome_passed) {
        bail!(
            "{} clean flagship trace did not end in the required semantic outcome",
            physics_backend.as_str()
        );
    }
    Ok((run, trace))
}

#[cfg(feature = "mujoco")]
fn build_cross_backend_report(
    rapier_report: &BehaviorReport,
    rapier_trace: &[FlagshipObservation],
    mujoco_report: &BehaviorReport,
    mujoco_trace: &[FlagshipObservation],
) -> Result<(CrossBackendReport, BehaviorReport)> {
    let rapier_seed = only_seed(rapier_report)?;
    let mujoco_seed = only_seed(mujoco_report)?;
    let rapier_final = rapier_trace
        .last()
        .context("Rapier flagship trace is empty")?;
    let mujoco_final = mujoco_trace
        .last()
        .context("MuJoCo flagship trace is empty")?;
    let fixed_delta_ticks = rapier_seed
        .sim_time_ticks
        .checked_div(rapier_seed.steps)
        .context("Rapier flagship completed in zero steps")?;
    let mujoco_fixed_delta_ticks = mujoco_seed
        .sim_time_ticks
        .checked_div(mujoco_seed.steps)
        .context("MuJoCo flagship completed in zero steps")?;
    if fixed_delta_ticks != mujoco_fixed_delta_ticks {
        bail!(
            "cross-backend fixed step differs: Rapier={fixed_delta_ticks}, MuJoCo={mujoco_fixed_delta_ticks}"
        );
    }

    let checks = vec![
        comparison_check(
            "completion_step_delta",
            "completion step",
            "step",
            rapier_seed.steps.abs_diff(mujoco_seed.steps) as f64,
            COMPLETION_STEP_DELTA_MAX,
        ),
        comparison_check(
            "base_planar_position_delta",
            "base planar position",
            "m",
            ((rapier_final.base_x_m - mujoco_final.base_x_m).powi(2)
                + (rapier_final.base_z_m - mujoco_final.base_z_m).powi(2))
            .sqrt(),
            BASE_PLANAR_DELTA_MAX_M,
        ),
        comparison_check(
            "payload_position_delta",
            "payload position",
            "m",
            ((rapier_final.payload_x_m - mujoco_final.payload_x_m).powi(2)
                + (rapier_final.payload_y_m - mujoco_final.payload_y_m).powi(2)
                + (rapier_final.payload_z_m - mujoco_final.payload_z_m).powi(2))
            .sqrt(),
            PAYLOAD_POSITION_DELTA_MAX_M,
        ),
        comparison_check(
            "payload_apex_delta",
            "maximum payload height",
            "m",
            (rapier_final.maximum_payload_y_m - mujoco_final.maximum_payload_y_m).abs(),
            PAYLOAD_APEX_DELTA_MAX_M,
        ),
        comparison_check(
            "arm_joint_position_delta",
            "maximum arm joint position",
            "rad",
            [
                (rapier_final.shoulder_position_rad - mujoco_final.shoulder_position_rad).abs(),
                (rapier_final.elbow_position_rad - mujoco_final.elbow_position_rad).abs(),
                (rapier_final.wrist_yaw_position_rad - mujoco_final.wrist_yaw_position_rad).abs(),
            ]
            .into_iter()
            .fold(0.0_f64, f64::max),
            ARM_JOINT_DELTA_MAX_RAD,
        ),
        comparison_check(
            "lift_position_delta",
            "lift position",
            "m",
            (rapier_final.lift_position_m - mujoco_final.lift_position_m).abs(),
            LIFT_DELTA_MAX_M,
        ),
        comparison_check(
            "gripper_position_delta",
            "gripper position",
            "m",
            (rapier_final.gripper_position_m - mujoco_final.gripper_position_m).abs(),
            GRIPPER_DELTA_MAX_M,
        ),
        comparison_check(
            "wrist_depth_delta",
            "wrist minimum depth",
            "m",
            (rapier_final.wrist_depth_min_m - mujoco_final.wrist_depth_min_m).abs(),
            WRIST_DEPTH_DELTA_MAX_M,
        ),
        comparison_check(
            "total_reward_delta",
            "episode total reward",
            "reward",
            (rapier_final.total_reward - mujoco_final.total_reward).abs(),
            REWARD_DELTA_MAX,
        ),
    ];
    let outcomes = vec![
        CrossBackendOutcome {
            backend_id: "rapier_native",
            status: if rapier_report.passed() && semantic_outcome_passed(rapier_final) {
                "passed"
            } else {
                "failed"
            },
            steps: rapier_seed.steps,
            sim_time_ticks: rapier_seed.sim_time_ticks,
            final_state_digest: rapier_seed.final_state_digest,
            behavior_report: "success.behavior-report.json".to_string(),
            final_observation: rapier_final.clone(),
        },
        CrossBackendOutcome {
            backend_id: "mujoco_native",
            status: if mujoco_report.passed() && semantic_outcome_passed(mujoco_final) {
                "passed"
            } else {
                "failed"
            },
            steps: mujoco_seed.steps,
            sim_time_ticks: mujoco_seed.sim_time_ticks,
            final_state_digest: mujoco_seed.final_state_digest,
            behavior_report: "mujoco-success.behavior-report.json".to_string(),
            final_observation: mujoco_final.clone(),
        },
    ];
    let passed = outcomes.iter().all(|outcome| outcome.status == "passed")
        && checks.iter().all(|check| check.status == "passed");
    let task_spec = flagship_task_spec(fixed_delta_ticks);
    let task_spec_digest = stable_behavior_digest(&serde_json::to_vec(&task_spec)?);
    Ok((
        CrossBackendReport {
            schema_version: CROSS_BACKEND_REPORT_SCHEMA_VERSION,
            kind: CROSS_BACKEND_REPORT_KIND,
            status: if passed { "passed" } else { "failed" },
            scenario: SCENARIO,
            seed: SEED,
            task_id: TASK_ID,
            task_spec: "flagship.task.json",
            task_spec_digest,
            fixed_delta_ticks,
            comparison_contract: "semantic_outcome_and_named_si_tolerances",
            exact_outcomes: vec![
                "all_behavior_contracts_passed",
                "inspection_completed",
                "traffic_cleared_without_collision_or_signal_violation",
                "payload_grasped_once",
                "pick_place_completed",
                "terminated_without_truncation_or_fail_closed_abort",
            ],
            state_digest_contract: "backend_specific_not_compared",
            backends: outcomes,
            tolerance_checks: checks,
        },
        mujoco_report.clone(),
    ))
}

#[cfg(feature = "mujoco")]
fn comparison_check(
    id: &'static str,
    quantity: &'static str,
    unit: &'static str,
    observed_delta: f64,
    maximum_delta: f64,
) -> CrossBackendCheck {
    CrossBackendCheck {
        id,
        quantity,
        unit,
        observed_delta,
        maximum_delta,
        status: if observed_delta <= maximum_delta {
            "passed"
        } else {
            "failed"
        },
    }
}

fn semantic_outcome_passed(observation: &FlagshipObservation) -> bool {
    observation.inspection_complete
        && observation.perception_valid
        && observation.traffic_clear
        && observation.traffic_collision_count == 0
        && observation.traffic_signal_violation_count == 0
        && observation.grasped_once
        && observation.task_completed
        && observation.robot_terminated
        && !observation.robot_truncated
        && observation.policy_phase == "release"
        && observation.policy_failure == "none"
        && !observation.fault_injected
        && !observation.fail_closed_abort
}

fn seeded_dimensions(seed: u64, perception_blackout: bool) -> Result<Vec<BehaviorDimension>> {
    Ok(vec![
        BehaviorDimension::boolean(BLACKOUT_DIMENSION, perception_blackout, false)?,
        BehaviorDimension::number(DEPARTURE_DIMENSION, 0.15 + (seed % 3) as f64 * 0.05, 0.0)?,
        BehaviorDimension::number(SPEED_DIMENSION, 0.10 + ((seed / 3) % 4) as f64 * 0.05, 0.0)?,
    ])
}

fn flagship_task_spec(fixed_delta_ticks: u64) -> TaskSpec {
    TaskSpec::new(
        TASK_ID,
        fixed_delta_ticks as f64 / 1_000_000_000.0,
        ObservationSpec::new(vec![
            TensorSpec::new("base_position_m", TensorDType::F64, vec![2], "m"),
            TensorSpec::new("arm_joint_position_rad", TensorDType::F64, vec![3], "rad"),
            TensorSpec::new("lift_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("gripper_position_m", TensorDType::F64, vec![], "m"),
            TensorSpec::new("payload_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("wrist_camera_pixel_count", TensorDType::I64, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, i64::MAX as f64)),
            TensorSpec::new("wrist_depth_min_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(0.0, f64::MAX)),
            TensorSpec::new("traffic_actor_position_m", TensorDType::F64, vec![3], "m"),
            TensorSpec::new("traffic_signal_green", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("traffic_clear", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("grasped", TensorDType::Bool, vec![], "1"),
            TensorSpec::new("policy_phase", TensorDType::I32, vec![], "1")
                .with_bounds(TensorBounds::broadcast(0.0, 9.0)),
        ]),
        ActionSpec::new(vec![
            TensorSpec::new("wheel_velocity_rad_s", TensorDType::F64, vec![2], "rad/s")
                .with_bounds(TensorBounds::broadcast(-10.0, 10.0)),
            TensorSpec::new("arm_joint_target_rad", TensorDType::F64, vec![3], "rad").with_bounds(
                TensorBounds::broadcast(-std::f64::consts::PI, std::f64::consts::PI),
            ),
            TensorSpec::new("lift_target_m", TensorDType::F64, vec![], "m")
                .with_bounds(TensorBounds::broadcast(-0.5, 0.5)),
            TensorSpec::new("gripper_velocity_m_s", TensorDType::F64, vec![], "m/s")
                .with_bounds(TensorBounds::broadcast(-0.1, 0.1)),
        ]),
        RewardSpec::weighted_sum(vec![
            RewardTermSpec::new("task_progress_m", 1.0, "m"),
            RewardTermSpec::new("step", -0.001, "1"),
            RewardTermSpec::new("task_completed", 10.0, "1"),
        ]),
        TerminationSpec::new(
            vec![
                TerminationConditionSpec::new(
                    "inspection_pick_place_completed",
                    TerminationKind::Success,
                ),
                TerminationConditionSpec::new("perception_stream_lost", TerminationKind::Failure),
            ],
            Some(MAX_WORKFLOW_STEPS),
        ),
        ResetSpec::splitmix64(false),
    )
    .with_randomization(RandomizationSpec::new(vec![
        RandomizationParameterSpec::new(
            DEPARTURE_DIMENSION,
            "s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
        RandomizationParameterSpec::new(
            SPEED_DIMENSION,
            "m/s",
            RandomDistributionSpec::Uniform {
                minimum: 0.0,
                maximum: 0.25,
            },
        ),
    ]))
}

fn decode_dimensions(dimensions: &[BehaviorDimension]) -> Result<ScenarioOverrides> {
    let mut blackout = None;
    let mut departure = None;
    let mut speed = None;
    for dimension in dimensions {
        match dimension.name.as_str() {
            BLACKOUT_DIMENSION => {
                let (
                    BehaviorDimensionValue::Boolean(value),
                    BehaviorDimensionValue::Boolean(false),
                ) = (&dimension.value, &dimension.baseline)
                else {
                    bail!("{BLACKOUT_DIMENSION} must be boolean with false baseline");
                };
                blackout = Some(*value);
            }
            DEPARTURE_DIMENSION | SPEED_DIMENSION => {
                let (
                    BehaviorDimensionValue::Number(value),
                    BehaviorDimensionValue::Number(baseline),
                ) = (&dimension.value, &dimension.baseline)
                else {
                    bail!("{} must be numeric", dimension.name);
                };
                if !value.is_finite() || *baseline != 0.0 || *value < 0.0 {
                    bail!(
                        "{} must be finite, non-negative, and use zero baseline",
                        dimension.name
                    );
                }
                if dimension.name == DEPARTURE_DIMENSION {
                    departure = Some(*value);
                } else {
                    speed = Some(*value);
                }
            }
            other => bail!("unknown flagship behavior dimension `{other}`"),
        }
    }
    if dimensions.len() != 3 {
        bail!("flagship replay requires exactly three behavior dimensions");
    }
    Ok(ScenarioOverrides {
        perception_blackout: blackout.context("missing perception blackout dimension")?,
        traffic_departure_delay_s: departure.context("missing traffic departure dimension")?,
        traffic_speed_delta_m_s: speed.context("missing traffic speed dimension")?,
    })
}

fn build_traffic(
    overrides: ScenarioOverrides,
) -> Result<(World, Entity, TrafficRouteCatalog, TrafficSignalControls)> {
    let route_id = traffic_id("route:shared-aisle");
    let route = TrafficRoute::new(
        route_id.clone(),
        vec![[-2.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        false,
    )?;
    let initial = route.sample(0.0);
    let mut routes = TrafficRouteCatalog::default();
    routes.insert(route)?;
    let mut controls = TrafficSignalControls::default();
    controls.insert(TrafficSignalControl {
        id: traffic_id(SIGNAL_NAME),
        route_id: route_id.clone(),
        stop_distance_m: 1.4,
        aspect: SignalAspect::Red,
    })?;
    let mut world = World::new();
    let actor = world
        .spawn((
            TrafficActor::motor_vehicle(),
            EntityUuid(Uuid::from_u128(0x7401)),
            TrafficRouteFollower {
                route_id,
                distance_m: 0.0,
                speed_m_s: 0.0,
                desired_speed_m_s: 1.2 + overrides.traffic_speed_delta_m_s,
                length_m: 0.4,
            },
            TrafficDeparture {
                departure_time_s: overrides.traffic_departure_delay_s,
            },
            TrafficPose {
                position_m: initial.position_m,
                yaw_rad: initial.yaw_rad,
            },
        ))
        .id();
    Ok((world, actor, routes, controls))
}

fn action_commands_motion(action: &MobileManipulatorAction) -> bool {
    action.left_wheel_velocity_rad_s != 0.0
        || action.right_wheel_velocity_rad_s != 0.0
        || action.shoulder_velocity_rad_s != 0.0
        || action.elbow_velocity_rad_s != 0.0
        || action.gripper_velocity_rad_s != 0.0
        || action.gripper_velocity_m_s != 0.0
        || action.lift_velocity_m_s != 0.0
        || action.lift_joint_target.is_some()
        || action.wrist_yaw_target_rad.is_some()
}

fn observation_is_finite(observation: &FlagshipObservation) -> bool {
    [
        observation.wrist_depth_min_m,
        observation.traffic_actor_x_m,
        observation.traffic_actor_y_m,
        observation.traffic_actor_z_m,
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
        observation.maximum_payload_y_m,
        observation.total_reward,
    ]
    .into_iter()
    .all(f64::is_finite)
}

fn dimension_boolean(dimensions: &[BehaviorDimension], name: &str) -> bool {
    dimensions
        .iter()
        .find(|dimension| dimension.name == name)
        .and_then(|dimension| match dimension.value {
            BehaviorDimensionValue::Boolean(value) => Some(value),
            _ => None,
        })
        .unwrap_or(false)
}

fn phase_name(phase: MobileLiftPickPlacePhase) -> &'static str {
    match phase {
        MobileLiftPickPlacePhase::Settle => "settle",
        MobileLiftPickPlacePhase::Navigate => "navigate",
        MobileLiftPickPlacePhase::Approach => "approach",
        MobileLiftPickPlacePhase::LowerToPick => "lower_to_pick",
        MobileLiftPickPlacePhase::Grasp => "grasp",
        MobileLiftPickPlacePhase::Lift => "lift",
        MobileLiftPickPlacePhase::Transport => "transport",
        MobileLiftPickPlacePhase::Lower => "lower",
        MobileLiftPickPlacePhase::Release => "release",
        MobileLiftPickPlacePhase::Done => "done",
    }
}

fn phase_index(phase: MobileLiftPickPlacePhase) -> i32 {
    match phase {
        MobileLiftPickPlacePhase::Settle => 0,
        MobileLiftPickPlacePhase::Navigate => 1,
        MobileLiftPickPlacePhase::Approach => 2,
        MobileLiftPickPlacePhase::LowerToPick => 3,
        MobileLiftPickPlacePhase::Grasp => 4,
        MobileLiftPickPlacePhase::Lift => 5,
        MobileLiftPickPlacePhase::Transport => 6,
        MobileLiftPickPlacePhase::Lower => 7,
        MobileLiftPickPlacePhase::Release => 8,
        MobileLiftPickPlacePhase::Done => 9,
    }
}

fn failure_name(failure: MobileLiftFailureClass) -> &'static str {
    match failure {
        MobileLiftFailureClass::None => "none",
        MobileLiftFailureClass::NavigateTimeout => "navigate_timeout",
        MobileLiftFailureClass::ApproachTimeout => "approach_timeout",
        MobileLiftFailureClass::PickupAlignmentTimeout => "pickup_alignment_timeout",
        MobileLiftFailureClass::GraspTimeout => "grasp_timeout",
        MobileLiftFailureClass::GraspSlip => "grasp_slip",
        MobileLiftFailureClass::LiftClearanceTimeout => "lift_clearance_timeout",
        MobileLiftFailureClass::TransportTimeout => "transport_timeout",
        MobileLiftFailureClass::LowerTimeout => "lower_timeout",
        MobileLiftFailureClass::ReleaseTimeout => "release_timeout",
    }
}

fn traffic_id(value: &str) -> TrafficId {
    TrafficId::new(value).expect("flagship traffic IDs are static and valid")
}

fn digest_scene_inputs(scene_path: &Path) -> Result<(u64, Vec<String>)> {
    let bundle = load_scene_bundle(scene_path)?;
    let mut bytes = b"rne_flagship_scene_inputs_v1".to_vec();
    let mut names = Vec::new();
    for path in scene_dependency_paths(&bundle) {
        let contents = fs::read(&path)
            .with_context(|| format!("could not read imported asset {}", path.display()))?;
        bytes.extend_from_slice(&(contents.len() as u64).to_le_bytes());
        bytes.extend_from_slice(&contents);
        let resolved = path
            .canonicalize()
            .with_context(|| format!("could not resolve imported asset {}", path.display()))?;
        names.push(portable_asset_path(&resolved));
    }
    Ok((stable_behavior_digest(&bytes), names))
}

fn portable_asset_path(path: &Path) -> String {
    let components = path.components().collect::<Vec<_>>();
    if let Some(index) = components
        .iter()
        .position(|component| component.as_os_str() == OsStr::new("assets"))
    {
        return components[index..]
            .iter()
            .map(|component| component.as_os_str().to_string_lossy())
            .collect::<Vec<_>>()
            .join("/");
    }
    path.file_name()
        .unwrap_or_else(|| OsStr::new("unknown-asset"))
        .to_string_lossy()
        .into_owned()
}

fn only_seed(report: &BehaviorReport) -> Result<&rne_ai::BehaviorSeedReport> {
    if report.seeds.len() != 1 {
        bail!("flagship report must contain exactly one seed");
    }
    Ok(&report.seeds[0])
}

fn write_pretty_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let bytes = serde_json::to_vec_pretty(value)?;
    fs::write(path, [bytes.as_slice(), b"\n"].concat())
        .with_context(|| format!("could not write {}", path.display()))
}

fn write_browser_inspector(
    path: &Path,
    success: &[FlagshipObservation],
    failure: &rne_ai::BehaviorReplayArtifact,
) -> Result<()> {
    let data = serde_json::json!({
        "success": success,
        "failure": failure.frames.iter().map(|frame| &frame.observation).collect::<Vec<_>>(),
    });
    let data = serde_json::to_string(&data)?.replace('<', "\\u003c");
    let html = r#"<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>RNE flagship replay inspector</title>
<style>
:root{color-scheme:dark;font:15px system-ui;background:#0c1117;color:#e6edf3}body{max-width:980px;margin:0 auto;padding:24px}h1{font-size:22px}main{display:grid;grid-template-columns:2fr 1fr;gap:18px}section{background:#151b23;border:1px solid #30363d;border-radius:10px;padding:16px}canvas{width:100%;background:#090d12;border-radius:8px}label{display:block;margin:8px 0}input[type=range]{width:100%}select{padding:5px;background:#21262d;color:inherit}dl{display:grid;grid-template-columns:1fr 1fr;gap:8px}dt{color:#8b949e}dd{margin:0;text-align:right}.ok{color:#3fb950}.bad{color:#f85149}@media(max-width:720px){main{grid-template-columns:1fr}}
</style></head><body><h1>RNE shared-aisle flagship replay</h1><p>Headless fixed-step evidence; switch between the successful run and minimized perception failure.</p>
<main><section><canvas id="world" width="700" height="440"></canvas><label>frame <input id="frame" type="range" min="0" value="0"></label></section>
<section><label>run <select id="run"><option value="success">success</option><option value="failure">minimized failure</option></select></label><dl id="facts"></dl></section></main>
<script id="replay-data" type="application/json">__DATA__</script><script>
const all=JSON.parse(document.querySelector('#replay-data').textContent), run=document.querySelector('#run'), slider=document.querySelector('#frame'), facts=document.querySelector('#facts'), canvas=document.querySelector('#world'), ctx=canvas.getContext('2d');
function frames(){return all[run.value]} function draw(){const xs=frames(), i=+slider.value, o=xs[i];slider.max=Math.max(0,xs.length-1);if(i>=xs.length){slider.value=xs.length-1;return draw()}ctx.clearRect(0,0,700,440);ctx.strokeStyle='#6e7681';ctx.lineWidth=8;ctx.beginPath();ctx.moveTo(80,220);ctx.lineTo(620,220);ctx.stroke();ctx.fillStyle=o.traffic_signal_green?'#3fb950':'#f85149';ctx.beginPath();ctx.arc(350,175,10,0,Math.PI*2);ctx.fill();const px=x=>350+x*115,pz=z=>220-z*115;ctx.fillStyle='#58a6ff';ctx.fillRect(px(o.base_x_m)-16,pz(o.base_z_m)-12,32,24);ctx.fillStyle='#d29922';ctx.beginPath();ctx.arc(px(o.payload_x_m),pz(o.payload_z_m),8,0,Math.PI*2);ctx.fill();ctx.fillStyle='#a371f7';ctx.fillRect(px(o.traffic_actor_x_m)-14,208,28,24);ctx.fillStyle='#e6edf3';ctx.fillText('robot',px(o.base_x_m)-18,pz(o.base_z_m)-18);ctx.fillText('payload',px(o.payload_x_m)-22,pz(o.payload_z_m)-12);facts.innerHTML=Object.entries({frame:i,step:o.workflow_step,phase:o.policy_phase,inspection:o.inspection_complete,perception:o.perception_valid,traffic_clear:o.traffic_clear,grasped:o.grasped_once,task_complete:o.task_completed,fail_closed:o.fail_closed_abort}).map(([k,v])=>`<dt>${k}</dt><dd class="${v===false&&(k==='perception'||k==='task_complete')?'bad':'ok'}">${v}</dd>`).join('')}
run.onchange=()=>{slider.value=0;draw()};slider.oninput=draw;draw();
</script></body></html>"#.replace("__DATA__", &data);
    fs::write(path, html).with_context(|| format!("could not write {}", path.display()))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn seeded_dimensions_round_trip_and_reject_unknown_names() {
        let dimensions = seeded_dimensions(SEED, true).expect("seeded dimensions");
        let decoded = decode_dimensions(&dimensions).expect("decode dimensions");
        assert!(decoded.perception_blackout);
        assert!(decoded.traffic_departure_delay_s > 0.0);
        assert!(decoded.traffic_speed_delta_m_s > 0.0);

        let mut unknown = dimensions;
        unknown[0].name = "unknown".to_string();
        assert!(decode_dimensions(&unknown)
            .expect_err("unknown dimension")
            .to_string()
            .contains("unknown flagship"));
    }

    #[test]
    fn flagship_task_contract_is_portable_and_valid() {
        let task = flagship_task_spec(16_666_666);
        task.validate().expect("flagship TaskSpec");
        assert_eq!(task.task_id, TASK_ID);
        assert_eq!(task.observation.tensors.len(), 12);
        assert_eq!(task.action.tensors.len(), 4);
        assert_eq!(task.termination.conditions.len(), 2);
    }

    #[cfg(feature = "mujoco")]
    #[test]
    fn mujoco_executes_the_clean_flagship_contracts() {
        let trace = Arc::new(Mutex::new(Vec::new()));
        let captured = Arc::clone(&trace);
        let run = run_behavior_scenarios_with_replays(SCENARIO, [SEED], |seed| {
            FlagshipScenario::clean_with_physics(seed, FlagshipPhysicsBackend::Mujoco)
                .map(|scenario| scenario.with_trace(Arc::clone(&captured)))
        })
        .expect("MuJoCo flagship run");
        let trace = trace.lock().expect("MuJoCo trace mutex");
        let last = trace.last().cloned();
        assert!(
            run.report.passed(),
            "MuJoCo final observation: {last:#?}\nreport: {:#?}",
            run.report,
        );
        assert!(run.failure_replays.is_empty());
    }

    #[test]
    fn seeded_blackout_replays_the_same_fail_closed_violation() {
        let run = run_behavior_scenarios_with_replays(SCENARIO, [SEED], |seed| {
            FlagshipScenario::fault_fixture(seed)
        })
        .expect("fault run");
        assert!(!run.report.passed());
        let replay = run.failure_replays.first().expect("failure replay");
        assert_eq!(replay.failure.contract.name, EXPECTED_FAILURE_CONTRACT);
        assert!(
            replay.frames.last().is_some_and(|frame| {
                frame
                    .observation
                    .get("fail_closed_abort")
                    .and_then(serde_json::Value::as_bool)
                    == Some(true)
            }),
            "last replay frame: {:?}",
            replay.frames.last()
        );
        verify_behavior_replay(replay, |seed, dimensions| {
            FlagshipScenario::from_dimensions(seed, dimensions)
        })
        .expect("deterministic fault replay");
    }
}

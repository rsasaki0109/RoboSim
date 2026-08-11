//! Command-line tools for RNE scene and robot assets.

use anyhow::{Context as _, Result};
use clap::{Parser, Subcommand};
use rne_assets::{
    inspect_asset, load_run_manifest, load_scene_bundle, smoke_spawn_scene, spawn_scene_bundle,
    validate_asset, AssetHotReloader, RunControllerKind, RunJointTrajectory, RunPhysicsBackend,
    RunPhysicsCapability, RunScenario, RunSensorKind, RunSensorSubscription, RunTrajectoryWaypoint,
    SpawnSceneOptions, ValidatedAsset,
};
use rne_core::control::{ControlCommand, EpisodeOutcome, RunControl, RunnerControl};
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, ImageDepth, ImageRgb8, ImuSample, InMemoryDataBus, PointCloud, WheelEncoderSample,
};
use rne_ecs::{Name, World};
use rne_log::{
    ReplayAction, ReplayArtifact, ReplayClock, ReplayContact,
    ReplayControllerKind as ArtifactControllerKind, ReplayFailureKind, ReplayFinalReport,
    ReplayFrame, ReplayJointPosition, ReplayJointState, ReplayObservation,
    ReplayRobotJointVelocity, ReplaySensorPayload, ReplaySensorPayloadData, ReplaySensorStream,
};
use rne_math::{yaw_rad, Hertz};
use rne_openscenario::{
    execute_scenario, execute_scenario_with_control, parse_openscenario_xml_file,
    stable_replay_input_digest, ScenarioDocument, ScenarioReplayArtifact, ScenarioReplayInputs,
    ScenarioRunOptions, SCENARIO_REPLAY_KIND,
};
use rne_physics::{
    hash_physics_state, require_capabilities, ContactEvent, PhysicsBackend, PhysicsCapability,
    PhysicsError, PhysicsWorldDesc, PhysicsWorldId,
};
use rne_physics_analytic::AnalyticBackend;
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_plugin::{
    ControllerActionFrame, ControllerJointObservation, ControllerObservationFrame,
    ControllerPlugin, ControllerResetContext, ControllerRobotObservation, ControllerScheduler,
    VelocityServoController, RNE_PLUGIN_ABI_VERSION, RNE_PLUGIN_MIN_ABI_VERSION,
};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, sync_all_joint_motors_from_actuators,
    Actuator, ActuatorCommand, ActuatorCommandBuffer, DiffDriveComponent, DifferentialDrive, Joint,
    JointKind, Robot,
};
use rne_sensor::{
    sample_sensors, Sensor, SensorKind, SensorSampleContext, SensorState,
    CAMERA_DEPTH_STREAM_OFFSET,
};
use rne_sumo::import_sumo_net_file;
use rne_traci::{CoSimulation, TraciClient};
use rne_traffic::{load_traffic_asset, save_traffic_asset, TrafficId};
use rne_world::{world_transform_of, Transform3, WorldEntity};
use serde::Serialize;
use std::collections::VecDeque;
use std::io::{BufRead, Write};
use std::path::{Path, PathBuf};
use std::sync::{mpsc, Arc, Mutex};
use std::thread;
use std::time::Duration;

const REPLAY_FLOAT_EPSILON: f64 = 1.0e-12;
const RUNNER_CONTROL_PROTOCOL_VERSION: u32 = 1;
const RUNNER_CONTROL_MAX_SNAPSHOT_BYTES: usize = 32 * 1024 * 1024;
const RUNNER_CONTROL_WRITE_TIMEOUT_MS: u64 = 500;
const LIVE_CAMERA_MAX_WIDTH: u32 = 160;
const LIVE_CAMERA_MAX_HEIGHT: u32 = 120;
const LIVE_CAMERA_TRANSPORT_MAX_WIDTH: u32 = 1920;
const LIVE_CAMERA_TRANSPORT_MAX_HEIGHT: u32 = 1080;
const LIVE_LIDAR_MAX_POINTS: usize = 256;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct LiveSnapshotOptions {
    camera_max_width: Option<u32>,
    camera_max_height: Option<u32>,
    include_full_depth: bool,
}

impl Default for LiveSnapshotOptions {
    fn default() -> Self {
        Self {
            camera_max_width: Some(LIVE_CAMERA_MAX_WIDTH),
            camera_max_height: Some(LIVE_CAMERA_MAX_HEIGHT),
            include_full_depth: false,
        }
    }
}

impl LiveSnapshotOptions {
    fn full_resolution() -> Self {
        Self {
            camera_max_width: None,
            camera_max_height: None,
            include_full_depth: true,
        }
    }
}

fn parse_run_sensor_kind(text: &str) -> Result<RunSensorKind, String> {
    match text {
        "imu" => Ok(RunSensorKind::Imu),
        "lidar" => Ok(RunSensorKind::Lidar),
        "camera" => Ok(RunSensorKind::Camera),
        "wheel_encoder" => Ok(RunSensorKind::WheelEncoder),
        other => Err(format!(
            "unknown sensor kind `{other}` (expected imu, lidar, camera, or wheel_encoder)"
        )),
    }
}

#[derive(Parser)]
#[command(
    name = "rne-asset",
    about = "Validate, simulate, and watch RNE asset files"
)]
struct Cli {
    #[command(subcommand)]
    command: Commands,
}

#[derive(Subcommand)]
enum Commands {
    /// Parse and validate a `.rne.scene.toml` or `.rne.robot.toml` file.
    Validate {
        /// Scene or robot asset path.
        path: PathBuf,
        /// Also spawn the scene into an ECS world as a smoke check.
        #[arg(long)]
        spawn: bool,
    },
    /// Print a human-readable asset summary.
    Inspect {
        /// Scene or robot asset path.
        path: PathBuf,
    },
    /// Run a scene headlessly at a fixed simulation rate.
    Simulate {
        /// Scene asset path.
        path: PathBuf,
        /// Number of fixed simulation steps.
        #[arg(long, default_value_t = 600)]
        steps: u64,
        /// Fixed simulation rate in hertz.
        #[arg(long, default_value_t = 60.0)]
        hz: f64,
        /// Wheel velocity applied to every differential-drive robot.
        #[arg(long, default_value_t = 0.0)]
        wheel_velocity_rad_s: f64,
        /// Named URDF / ECS joint to command.
        #[arg(long)]
        joint: Option<String>,
        /// Velocity command for `--joint`, in radians per second.
        #[arg(long)]
        joint_velocity_rad_s: Option<f64>,
        /// Effort command for `--joint`, in newton-meters.
        #[arg(long)]
        joint_effort_nm: Option<f64>,
        /// Run the same scene and inputs twice and compare the final report.
        #[arg(long)]
        determinism_check: bool,
        /// Record full payloads for sensors with this entity name (repeatable).
        #[arg(long)]
        sensor_name: Vec<String>,
        /// Record full payloads for sensors of this kind (repeatable).
        #[arg(long, value_parser = parse_run_sensor_kind)]
        sensor_kind: Vec<RunSensorKind>,
        /// Write a versioned `.rne-replay` artifact to this path.
        #[arg(long, value_name = "PATH")]
        replay_out: Option<PathBuf>,
    },
    /// Execute a versioned `.rne.run.toml` manifest headlessly.
    Run {
        /// Run manifest path.
        path: PathBuf,
        /// Accept runner control commands on stdin: `pause`, `resume`,
        /// `step N`, `reset`, and `quit`. Determinism re-checks are skipped in
        /// interactive mode.
        #[arg(long, conflicts_with = "control_port")]
        control_stdin: bool,
        /// Serve runner control commands over a local TCP port for a frontend:
        /// `pause`, `resume`, `step N`, `reset`, and `quit`, with live per-step
        /// status replies. Determinism re-checks are skipped in interactive mode.
        #[arg(long, value_name = "PORT", conflicts_with = "control_stdin")]
        control_port: Option<u16>,
        /// Stream camera and depth payloads at source resolution over TCP, up
        /// to the transport safety cap of 1920x1080 pixels per payload.
        /// This is opt-in because each status line can become substantially larger.
        #[arg(long, requires = "control_port")]
        control_camera_full_resolution: bool,
        /// Override the manifest's replay output path.
        #[arg(long, value_name = "PATH")]
        replay_out: Option<PathBuf>,
    },
    /// Replay a recorded `.rne-replay` artifact and verify every frame.
    Replay {
        /// Replay artifact path.
        path: PathBuf,
    },
    /// Convert a SUMO `.net.xml` road network into a `.rne.traffic.json` asset.
    SumoNet {
        /// SUMO `.net.xml` path.
        path: PathBuf,
        /// Output `.rne.traffic.json` path.
        #[arg(short, long)]
        out: PathBuf,
        /// Stable network id for the derived asset.
        #[arg(long, default_value = "sumo")]
        network_id: String,
    },
    /// Author and inspect controller plugins.
    Plugin {
        #[command(subcommand)]
        command: PluginCommand,
    },
    /// Run a headless SUMO co-simulation and report the mirrored vehicles.
    CoSim {
        /// SUMO `.net.xml` path.
        path: PathBuf,
        /// SUMO `.rou.xml` route file.
        #[arg(long)]
        routes: PathBuf,
        /// Number of co-simulation steps.
        #[arg(long, default_value_t = 10)]
        steps: u64,
        /// Run the same co-simulation twice and compare the stable hash.
        #[arg(long)]
        determinism_check: bool,
    },
    /// Poll a scene asset graph and reload when dependencies change.
    Watch {
        /// Scene asset path.
        path: PathBuf,
        /// Poll interval in milliseconds.
        #[arg(long, default_value_t = 500)]
        interval_ms: u64,
    },
}

#[derive(Subcommand)]
enum PluginCommand {
    /// Scaffold a new controller-plugin crate implementing the C ABI.
    New {
        /// Plugin and crate name (ASCII letters, digits, and underscores).
        name: String,
        /// Parent directory receiving the plugin crate.
        #[arg(long, value_name = "DIR", default_value = ".")]
        dir: PathBuf,
    },
    /// List built-in and discoverable controller plugins.
    List {
        /// Directories searched for plugin shared libraries (repeatable).
        #[arg(long, value_name = "DIR")]
        path: Vec<PathBuf>,
    },
}

fn main() -> Result<()> {
    match Cli::parse().command {
        Commands::Validate { path, spawn } => validate_command(&path, spawn),
        Commands::Inspect { path } => inspect_command(&path),
        Commands::Simulate {
            path,
            steps,
            hz,
            wheel_velocity_rad_s,
            joint,
            joint_velocity_rad_s,
            joint_effort_nm,
            determinism_check,
            sensor_name,
            sensor_kind,
            replay_out,
        } => {
            let mut sensor_subscriptions = sensor_name
                .into_iter()
                .map(|name| RunSensorSubscription {
                    name: Some(name),
                    kind: None,
                })
                .collect::<Vec<_>>();
            sensor_subscriptions.extend(sensor_kind.into_iter().map(|kind| {
                RunSensorSubscription {
                    name: None,
                    kind: Some(kind),
                }
            }));
            simulate_command(
                &path,
                steps,
                hz,
                DirectActionOptions {
                    wheel_velocity_rad_s,
                    joint: joint.as_deref(),
                    joint_velocity_rad_s,
                    joint_effort_nm,
                },
                determinism_check,
                sensor_subscriptions,
                replay_out.as_deref(),
            )
        }
        Commands::Run {
            path,
            control_stdin,
            control_port,
            control_camera_full_resolution,
            replay_out,
        } => run_manifest_command(
            &path,
            control_stdin,
            control_port,
            control_camera_full_resolution,
            replay_out.as_deref(),
        ),
        Commands::Replay { path } => replay_command(&path),
        Commands::SumoNet {
            path,
            out,
            network_id,
        } => sumo_net_command(&path, &out, &network_id),
        Commands::Plugin { command } => plugin_command(command),
        Commands::CoSim {
            path,
            routes,
            steps,
            determinism_check,
        } => co_sim_command(&path, &routes, steps, determinism_check),
        Commands::Watch { path, interval_ms } => watch_command(&path, interval_ms),
    }
}

fn validate_command(path: &Path, spawn: bool) -> Result<()> {
    let validated = validate_asset(path).with_context(|| format!("validate {}", path.display()))?;
    match &validated {
        ValidatedAsset::Scene(bundle) => {
            println!(
                "valid scene: robots={} seed={}",
                bundle.robots.len(),
                bundle.scene.world.seed
            );
            if spawn {
                let robot_count =
                    smoke_spawn_scene(path).with_context(|| format!("spawn {}", path.display()))?;
                println!("spawn ok: robots={robot_count}");
            }
        }
        ValidatedAsset::Robot { asset, .. } => {
            println!(
                "valid robot: kind={:?} model={}",
                asset.kind, asset.model_name
            );
        }
    }
    Ok(())
}

fn inspect_command(path: &Path) -> Result<()> {
    let report = inspect_asset(path).with_context(|| format!("inspect {}", path.display()))?;
    println!("{report}");
    Ok(())
}

#[derive(Clone, Debug, PartialEq)]
struct SimulationReport {
    steps: u64,
    sim_time_s: f64,
    seed: u64,
    robot_count: usize,
    differential_drive_count: usize,
    physics_hash: u64,
    first_base_translation_m: Option<[f64; 3]>,
    contact_pairs_max: u64,
    contact_impulse_max_ns: f32,
    min_base_height_m: Option<f64>,
    failure: Option<rne_log::ReplayFailureKind>,
}

#[derive(Clone, Debug)]
struct SimulationOptions<'a> {
    steps: u64,
    hz: f64,
    action: ReplayAction,
    trajectories: Vec<RunJointTrajectory>,
    plugin: Option<PluginControllerConfig>,
    seed_override: Option<u64>,
    determinism_check: bool,
    replay_out: Option<&'a Path>,
    replay_controller: ArtifactControllerKind,
    sensor_subscriptions: Vec<RunSensorSubscription>,
    physics_backend: RunPhysicsBackend,
    live_snapshot_options: LiveSnapshotOptions,
}

/// Parameters for a [`VelocityServoController`], a dynamically loaded, or a
/// discovered controller plugin selected by a run manifest.
#[derive(Clone, Debug, PartialEq)]
struct PluginControllerConfig {
    joint: String,
    target_rad: f64,
    gain: f64,
    max_velocity_rad_s: f64,
    /// Shared-library path to load through the controller-plugin C ABI.
    library: Option<PathBuf>,
    /// Directories searched for a library whose plugin name matches.
    plugin_paths: Vec<PathBuf>,
}

impl PluginControllerConfig {
    /// Builds the concrete controller plugin (the policy callback boundary).
    ///
    /// With [`Self::library`] set, the plugin is loaded directly from the
    /// shared library. Otherwise, if [`Self::plugin_paths`] is non-empty the
    /// plugin is discovered by name in those directories. Otherwise the
    /// built-in [`VelocityServoController`] is used.
    fn build(&self) -> Result<Box<dyn ControllerPlugin>> {
        if let Some(library) = &self.library {
            rne_plugin::load_controller_library(
                library,
                &self.joint,
                self.target_rad,
                self.gain,
                self.max_velocity_rad_s,
            )
            .map_err(|error| anyhow::anyhow!("{error}"))
        } else if !self.plugin_paths.is_empty() {
            let paths: Vec<&Path> = self.plugin_paths.iter().map(PathBuf::as_path).collect();
            rne_plugin::discover_controller_plugin(
                "velocity_servo",
                &paths,
                &self.joint,
                self.target_rad,
                self.gain,
                self.max_velocity_rad_s,
            )
            .map_err(|error| anyhow::anyhow!("{error}"))
        } else {
            Ok(Box::new(
                VelocityServoController::new(
                    "velocity_servo",
                    &self.joint,
                    self.target_rad,
                    self.gain,
                    self.max_velocity_rad_s,
                )
                .map_err(|error| anyhow::anyhow!("{error}"))?,
            ))
        }
    }
}

struct DirectActionOptions<'a> {
    wheel_velocity_rad_s: f64,
    joint: Option<&'a str>,
    joint_velocity_rad_s: Option<f64>,
    joint_effort_nm: Option<f64>,
}

fn simulate_command(
    path: &Path,
    steps: u64,
    hz: f64,
    direct_options: DirectActionOptions<'_>,
    determinism_check: bool,
    sensor_subscriptions: Vec<RunSensorSubscription>,
    replay_out: Option<&Path>,
) -> Result<()> {
    let action = direct_action(
        direct_options.wheel_velocity_rad_s,
        direct_options.joint,
        direct_options.joint_velocity_rad_s,
        direct_options.joint_effort_nm,
    )?;
    let replay_controller = action.controller_kind();
    run_simulation(
        path,
        SimulationOptions {
            steps,
            hz,
            action,
            seed_override: None,
            determinism_check,
            replay_out,
            replay_controller,
            sensor_subscriptions,
            trajectories: Vec::new(),
            plugin: None,
            physics_backend: RunPhysicsBackend::Rapier,
            live_snapshot_options: LiveSnapshotOptions::default(),
        },
        None,
    )
}

fn run_manifest_command(
    path: &Path,
    control_stdin: bool,
    control_port: Option<u16>,
    control_camera_full_resolution: bool,
    replay_out_override: Option<&Path>,
) -> Result<()> {
    let manifest =
        load_run_manifest(path).with_context(|| format!("load run manifest {}", path.display()))?;
    if let Some(scenario) = &manifest.scenario {
        if control_stdin {
            let mut transport = StdinRunnerControl::start()?;
            let mut control = RunControl::paused(&mut transport);
            println!(
                "control: scenario runner paused; commands on stdin: pause, resume, step N, reset, quit"
            );
            return run_scenario_manifest(
                path,
                &manifest,
                scenario,
                Some(&mut control),
                replay_out_override,
            );
        }
        if let Some(port) = control_port {
            let (mut transport, bound_port) = TcpRunnerControl::start(port)?;
            println!(
                "control: scenario runner listening on 127.0.0.1:{bound_port}; commands: pause, resume, step N, reset, quit"
            );
            let mut control = RunControl::paused(&mut transport);
            return run_scenario_manifest(
                path,
                &manifest,
                scenario,
                Some(&mut control),
                replay_out_override,
            );
        }
        return run_scenario_manifest(path, &manifest, scenario, None, replay_out_override);
    }
    let scene_path = manifest.resolve_scene_path(path);
    let (action, replay_controller) = match manifest.controller.kind {
        RunControllerKind::None => (
            ReplayAction::differential_drive(0.0),
            ArtifactControllerKind::None,
        ),
        RunControllerKind::DifferentialDrive => (
            ReplayAction::differential_drive(manifest.controller.wheel_velocity_rad_s),
            ArtifactControllerKind::DifferentialDrive,
        ),
        RunControllerKind::JointVelocity => (
            ReplayAction::joint_velocity(
                manifest.controller.joint.clone(),
                manifest.controller.velocity_rad_s,
            ),
            ArtifactControllerKind::JointVelocity,
        ),
        RunControllerKind::JointEffort => (
            ReplayAction::joint_effort(
                manifest.controller.joint.clone(),
                manifest.controller.effort_nm,
            ),
            ArtifactControllerKind::JointEffort,
        ),
        RunControllerKind::JointTrajectory => (
            ReplayAction::differential_drive(0.0),
            ArtifactControllerKind::JointTrajectory,
        ),
        RunControllerKind::Plugin => (
            ReplayAction::differential_drive(0.0),
            ArtifactControllerKind::Plugin,
        ),
    };
    let trajectories = if manifest.controller.kind == RunControllerKind::JointTrajectory {
        manifest.controller.joint_trajectories.clone()
    } else {
        Vec::new()
    };
    let plugin = if manifest.controller.kind == RunControllerKind::Plugin {
        Some(PluginControllerConfig {
            joint: manifest.controller.joint.clone(),
            target_rad: manifest.controller.target_rad,
            gain: manifest.controller.gain,
            max_velocity_rad_s: manifest.controller.max_velocity_rad_s,
            library: manifest
                .controller
                .library
                .as_deref()
                .map(|library| manifest.resolve_output_path(path, library)),
            plugin_paths: manifest
                .controller
                .plugin_paths
                .iter()
                .map(|search_path| manifest.resolve_output_path(path, search_path))
                .collect(),
        })
    } else {
        None
    };
    let replay_out = match replay_out_override {
        Some(override_path) => Some(override_path.to_path_buf()),
        None => manifest
            .output
            .replay_path
            .as_deref()
            .map(|output_path| manifest.resolve_output_path(path, output_path)),
    };
    if !manifest.physics.required_capabilities.is_empty() {
        verify_physics_requirements(
            manifest.physics.backend,
            &manifest.physics.required_capabilities,
        )
        .with_context(|| format!("verify physics requirements for {}", path.display()))?;
        let names = manifest
            .physics
            .required_capabilities
            .iter()
            .map(|capability| format!("{capability:?}"))
            .collect::<Vec<_>>()
            .join(", ");
        println!(
            "physics: backend={:?} required capabilities [{}]",
            manifest.physics.backend, names
        );
    }
    println!(
        "run manifest={} scene={} controller={:?}",
        path.display(),
        scene_path.display(),
        manifest.controller.kind
    );
    let options = SimulationOptions {
        steps: manifest.clock.steps,
        hz: manifest.clock.hz,
        action,
        trajectories,
        plugin,
        seed_override: manifest.seed,
        determinism_check: manifest.output.determinism_check,
        replay_out: replay_out.as_deref(),
        replay_controller,
        sensor_subscriptions: manifest.sensors.clone(),
        physics_backend: manifest.physics.backend,
        live_snapshot_options: if control_camera_full_resolution {
            LiveSnapshotOptions::full_resolution()
        } else {
            LiveSnapshotOptions::default()
        },
    };
    if control_stdin {
        let mut transport = StdinRunnerControl::start()?;
        let mut control = RunControl::paused(&mut transport);
        println!("control: runner paused; commands on stdin: pause, resume, step N, reset, quit");
        run_simulation(&scene_path, options, Some(&mut control))
    } else if let Some(port) = control_port {
        let (mut transport, bound_port) = TcpRunnerControl::start(port)?;
        println!(
            "control: listening on 127.0.0.1:{bound_port}; commands: pause, resume, step N, reset, quit"
        );
        let mut control = RunControl::paused(&mut transport);
        run_simulation(&scene_path, options, Some(&mut control))
    } else {
        run_simulation(&scene_path, options, None)
    }
}

fn verify_physics_requirements(
    backend_kind: RunPhysicsBackend,
    required: &[RunPhysicsCapability],
) -> Result<()> {
    if required.is_empty() {
        return Ok(());
    }
    let backend = RunnerBackend::new(backend_kind);
    let required_capabilities = required
        .iter()
        .map(|capability| match capability {
            RunPhysicsCapability::RigidBody => PhysicsCapability::RigidBody,
            RunPhysicsCapability::Articulation => PhysicsCapability::Articulation,
            RunPhysicsCapability::GpuRigidBody => PhysicsCapability::GpuRigidBody,
            RunPhysicsCapability::DeterministicStep => PhysicsCapability::DeterministicStep,
            RunPhysicsCapability::SoftBody => PhysicsCapability::SoftBody,
            RunPhysicsCapability::ContactForce => PhysicsCapability::ContactForce,
            RunPhysicsCapability::RaycastBatch => PhysicsCapability::RaycastBatch,
        })
        .collect::<Vec<_>>();
    require_capabilities(backend.capabilities(), &required_capabilities)
        .map_err(|error| anyhow::anyhow!("{error}"))
}

/// Physics backend selected by a run manifest, dispatching the trait surface.
enum RunnerBackend {
    /// Rapier: full contacts, articulation, and contact force.
    Rapier(RapierBackend),
    /// Deterministic collision-free analytic dynamics.
    Analytic(AnalyticBackend),
}

impl RunnerBackend {
    /// Creates the backend selected by a manifest.
    fn new(kind: RunPhysicsBackend) -> Self {
        match kind {
            RunPhysicsBackend::Rapier => Self::Rapier(RapierBackend::new()),
            RunPhysicsBackend::Analytic => Self::Analytic(AnalyticBackend::new()),
        }
    }

    fn create_world(&mut self, desc: PhysicsWorldDesc) -> Result<PhysicsWorldId, PhysicsError> {
        match self {
            Self::Rapier(backend) => backend.create_world(desc),
            Self::Analytic(backend) => backend.create_world(desc),
        }
    }

    fn sync_from_ecs(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
    ) -> Result<(), PhysicsError> {
        match self {
            Self::Rapier(backend) => backend.sync_from_ecs(world, physics_world),
            Self::Analytic(backend) => backend.sync_from_ecs(world, physics_world),
        }
    }

    fn step(
        &mut self,
        world: &mut World,
        physics_world: PhysicsWorldId,
        dt: SimDuration,
    ) -> Result<(), PhysicsError> {
        match self {
            Self::Rapier(backend) => step_physics(backend, world, physics_world, dt),
            Self::Analytic(backend) => {
                rne_physics_analytic::step_physics(backend, world, physics_world, dt)
            }
        }
    }

    fn contacts(&self, physics_world: PhysicsWorldId) -> Result<&[ContactEvent], PhysicsError> {
        match self {
            Self::Rapier(backend) => backend.contacts(physics_world),
            Self::Analytic(backend) => backend.contacts(physics_world),
        }
    }

    fn capabilities(&self) -> &[PhysicsCapability] {
        match self {
            Self::Rapier(backend) => backend.capabilities(),
            Self::Analytic(backend) => backend.capabilities(),
        }
    }
}

/// Samples sensors with the concrete backend behind a [`RunnerBackend`].
fn sample_sensors_for(
    backend: &RunnerBackend,
    world: &mut World,
    sim_time: SimTime,
    physics_world: PhysicsWorldId,
    bus: &mut InMemoryDataBus,
) {
    match backend {
        RunnerBackend::Rapier(physics) => {
            let mut context = SensorSampleContext {
                world,
                sim_time,
                physics,
                physics_world,
                render: None,
                scene: None,
            };
            sample_sensors(&mut context, bus);
        }
        RunnerBackend::Analytic(physics) => {
            let mut context = SensorSampleContext {
                world,
                sim_time,
                physics,
                physics_world,
                render: None,
                scene: None,
            };
            sample_sensors(&mut context, bus);
        }
    }
}

fn run_scenario_manifest(
    manifest_path: &Path,
    manifest: &rne_assets::RunManifest,
    scenario: &RunScenario,
    mut control: Option<&mut RunControl<'_>>,
    replay_out_override: Option<&Path>,
) -> Result<()> {
    let xosc_path = scenario.resolve_xosc_path(manifest_path);
    println!(
        "scenario manifest={} xosc={} steps={} hz={}",
        manifest_path.display(),
        xosc_path.display(),
        manifest.clock.steps,
        manifest.clock.hz
    );
    let (document, network, network_path) = load_scenario_inputs(&xosc_path, None)?;
    let options = ScenarioRunOptions {
        steps: manifest.clock.steps,
        hz: manifest.clock.hz,
    };
    let controlled = control.is_some();
    let first =
        execute_scenario_with_control(&document, &network, &options, control.as_deref_mut())
            .with_context(|| format!("execute scenario {}", xosc_path.display()))?;
    let control_commands = control
        .as_deref()
        .map(|control| control.recorded_commands().to_vec())
        .unwrap_or_default();
    print_scenario_report(&xosc_path, &first);
    if manifest.output.determinism_check && !controlled {
        let replay = execute_scenario(&document, &network, &options)
            .with_context(|| format!("re-execute scenario {}", xosc_path.display()))?;
        anyhow::ensure!(
            first == replay,
            "scenario determinism check failed: first={first:?} replay={replay:?}"
        );
        println!("determinism: identical scenario outcome");
    } else if controlled && manifest.output.determinism_check {
        println!("determinism: skipped in interactive mode");
    }
    let replay_out = replay_out_override.map(PathBuf::from).or_else(|| {
        manifest
            .output
            .replay_path
            .as_deref()
            .map(|path| manifest.resolve_output_path(manifest_path, path))
    });
    if let Some(replay_out) = replay_out {
        let scenario_digest = replay_input_digest(&xosc_path, "OpenSCENARIO")?;
        let network_digest = replay_input_digest(&network_path, "traffic network")?;
        let artifact = ScenarioReplayArtifact::new(
            ScenarioReplayInputs::new(
                xosc_path.display().to_string(),
                scenario_digest,
                network_path.display().to_string(),
                network_digest,
            ),
            options,
            first.steps,
            control_commands,
            first.clone(),
        );
        artifact
            .write_json(&replay_out)
            .with_context(|| format!("write scenario replay artifact {}", replay_out.display()))?;
        println!(
            "scenario replay: wrote {} (version={} steps={} replayable={})",
            replay_out.display(),
            artifact.schema_version,
            artifact.executed_steps,
            artifact.replayable
        );
        if controlled {
            println!(
                "scenario replay: recorded {} control commands",
                artifact.control_commands.len()
            );
        }
    }
    Ok(())
}

fn replay_input_digest(path: &Path, input_kind: &str) -> Result<u64> {
    let bytes = std::fs::read(path)
        .with_context(|| format!("read {input_kind} replay input {}", path.display()))?;
    Ok(stable_replay_input_digest(&bytes))
}

/// Loads an OpenSCENARIO document and its referenced traffic network.
///
/// `network_override` is used by replay verification so the artifact records
/// the exact network input rather than re-deriving it from a changed XOSC.
fn load_scenario_inputs(
    xosc_path: &Path,
    network_override: Option<&Path>,
) -> Result<(ScenarioDocument, rne_traffic::TrafficNetwork, PathBuf)> {
    let document = parse_openscenario_xml_file(xosc_path)
        .with_context(|| format!("parse OpenSCENARIO {}", xosc_path.display()))?;
    let network_path = network_override.map(PathBuf::from).unwrap_or_else(|| {
        let logic_file = Path::new(&document.road_network_logic_file);
        if logic_file.is_absolute() {
            logic_file.to_path_buf()
        } else {
            xosc_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(logic_file)
        }
    });
    let is_sumo_net = network_path
        .file_name()
        .map(|name| name.to_string_lossy().ends_with(".net.xml"))
        .unwrap_or(false);
    let network = if is_sumo_net {
        let network_id =
            TrafficId::new("sumo:scenario-network").map_err(|error| anyhow::anyhow!("{error}"))?;
        import_sumo_net_file(&network_id, &network_path)
            .map_err(|error| {
                anyhow::anyhow!("import SUMO network {}: {error}", network_path.display())
            })?
            .network
    } else {
        load_traffic_asset(&network_path)
            .map_err(|error| {
                anyhow::anyhow!("load traffic network {}: {error}", network_path.display())
            })?
            .network
    };
    Ok((document, network, network_path))
}

fn print_scenario_report(path: &Path, result: &rne_openscenario::ScenarioRunResult) {
    println!(
        "scenario {} route_length={:.3} m final_positions={} collisions={} signal_violations={} average_speed={:.3} m/s stable_hash={:#018x}",
        path.display(),
        result.route_length_m,
        result.final_positions_m.len(),
        result.collisions,
        result.signal_violations,
        result.average_speed_m_s,
        result.stable_hash
    );
}

fn run_simulation(
    path: &Path,
    options: SimulationOptions<'_>,
    mut control: Option<&mut RunControl<'_>>,
) -> Result<()> {
    let SimulationOptions {
        steps,
        hz,
        action,
        trajectories,
        plugin,
        seed_override,
        determinism_check,
        replay_out,
        replay_controller,
        sensor_subscriptions,
        physics_backend,
        live_snapshot_options,
    } = options;
    anyhow::ensure!(
        hz.is_finite() && hz > 0.0,
        "--hz must be finite and positive"
    );
    ensure_action_is_finite(&action)?;

    let run = simulate_scene_with_snapshot_options(
        path,
        steps,
        hz,
        action.clone(),
        seed_override,
        None,
        &trajectories,
        plugin
            .as_ref()
            .map(PluginControllerConfig::build)
            .transpose()?,
        &sensor_subscriptions,
        physics_backend,
        control.as_deref_mut(),
        live_snapshot_options,
    )?;
    print_simulation_report(path, &run.report, determinism_check);
    if control.is_none() && determinism_check {
        let replay = simulate_scene_with_snapshot_options(
            path,
            steps,
            hz,
            action,
            seed_override,
            None,
            &trajectories,
            plugin
                .as_ref()
                .map(PluginControllerConfig::build)
                .transpose()?,
            &sensor_subscriptions,
            physics_backend,
            None,
            live_snapshot_options,
        )?;
        anyhow::ensure!(
            run.report == replay.report,
            "determinism check failed: first={:?} replay={:?}",
            run.report,
            replay.report
        );
        ensure_replay_frames(&run.frames, &replay.frames)?;
        println!("determinism: identical final report");
    }
    if let Some(replay_out) = replay_out {
        let artifact = ReplayArtifact::new(
            path.display().to_string(),
            run.report.seed,
            ReplayClock::new(run.report.steps, hz),
            replay_controller,
            run.sensor_payload_streams.clone(),
            run.frames.clone(),
            replay_final_report(&run.report),
        );
        artifact
            .write_json(replay_out)
            .with_context(|| format!("write replay artifact {}", replay_out.display()))?;
        println!(
            "replay: wrote {} (version={} frames={} payload_streams={})",
            replay_out.display(),
            artifact.version,
            artifact.frames.len(),
            artifact.sensor_payload_streams.len()
        );
    }
    Ok(())
}

/// A [`RunnerControl`] transport backed by stdin lines.
///
/// A reader thread turns each stdin line into a control command. When stdin
/// closes the transport reports [`ControlCommand::Quit`], so a piped script
/// ends the run after its last line.
struct StdinRunnerControl {
    receiver: mpsc::Receiver<ControlCommand>,
}

impl StdinRunnerControl {
    /// Spawns the stdin reader thread and returns its transport.
    fn start() -> Result<Self> {
        let (sender, receiver) = mpsc::channel();
        thread::Builder::new()
            .name("rne-control-stdin".into())
            .spawn(move || {
                for line in std::io::stdin().lock().lines() {
                    match line {
                        Ok(line) => {
                            if let Some(command) = parse_control_line(&line) {
                                println!("[control] {command:?}");
                                if sender.send(command).is_err() {
                                    return;
                                }
                            }
                        }
                        Err(_) => return,
                    }
                }
                let _ = sender.send(ControlCommand::Quit);
            })
            .map_err(|error| anyhow::anyhow!("spawn stdin control reader: {error}"))?;
        Ok(Self { receiver })
    }
}

impl RunnerControl for StdinRunnerControl {
    fn try_poll(&mut self) -> Option<ControlCommand> {
        self.receiver.try_recv().ok()
    }

    fn wait_command(&mut self) -> ControlCommand {
        self.receiver.recv().unwrap_or(ControlCommand::Quit)
    }
}

/// A [`RunnerControl`] transport served over a local TCP connection.
///
/// A reader thread accepts one client, sends `ready paused protocol=1`, then turns each
/// line into a control command (acknowledging `ok <state>`). The main thread
/// streams a `status step=<n> t=<t> state=<state> snapshot=<json>` line after
/// every completed step, so a GUI or renderer frontend can both drive and
/// observe the run. An acknowledgement means the command was accepted by the
/// runner-control queue; the subsequent status is the applied-state boundary.
/// The snapshot contains bounded RGB camera previews,
/// deterministic LiDAR point samples, and latest IMU/wheel values when those
/// typed sensor streams are present. The `run` command can opt into source-
/// resolution RGB plus little-endian f32 depth payloads for TCP control, with
/// absolute per-image and per-status safety limits.
struct TcpRunnerControl {
    receiver: mpsc::Receiver<ControlCommand>,
    writer: Arc<Mutex<Option<std::io::BufWriter<std::net::TcpStream>>>>,
    paused: bool,
}

/// A deterministic in-memory runner-control transport used by scenario replay.
struct ScriptedRunnerControl {
    commands: VecDeque<ControlCommand>,
}

impl ScriptedRunnerControl {
    fn new(commands: Vec<ControlCommand>) -> Self {
        Self {
            commands: commands.into(),
        }
    }
}

impl RunnerControl for ScriptedRunnerControl {
    fn try_poll(&mut self) -> Option<ControlCommand> {
        self.commands.pop_front()
    }

    fn wait_command(&mut self) -> ControlCommand {
        self.commands.pop_front().unwrap_or(ControlCommand::Quit)
    }
}

impl TcpRunnerControl {
    /// Binds the control listener on `127.0.0.1:port` (port 0 picks an
    /// ephemeral port) and spawns the client reader thread. Returns the
    /// transport and the bound port.
    fn start(port: u16) -> Result<(Self, u16)> {
        let listener = std::net::TcpListener::bind(("127.0.0.1", port))
            .map_err(|error| anyhow::anyhow!("bind control listener on port {port}: {error}"))?;
        let bound_port = listener
            .local_addr()
            .map_err(|error| anyhow::anyhow!("query control listener address: {error}"))?
            .port();
        let writer = Arc::new(Mutex::new(None::<std::io::BufWriter<std::net::TcpStream>>));
        let thread_writer = writer.clone();
        let (sender, receiver) = mpsc::channel();
        thread::Builder::new()
            .name("rne-control-tcp".into())
            .spawn(move || {
                let Ok((stream, _peer)) = listener.accept() else {
                    let _ = sender.send(ControlCommand::Quit);
                    return;
                };
                let _ = stream.set_write_timeout(Some(Duration::from_millis(
                    RUNNER_CONTROL_WRITE_TIMEOUT_MS,
                )));
                let read_stream = stream.try_clone().ok();
                if let Ok(mut slot) = thread_writer.lock() {
                    *slot = Some(std::io::BufWriter::new(stream));
                }
                let write = |line: &str| {
                    if let Ok(mut slot) = thread_writer.lock() {
                        let failed = slot.as_mut().is_some_and(|writer| {
                            writer
                                .write_all(line.as_bytes())
                                .and_then(|()| writer.flush())
                                .is_err()
                        });
                        if failed {
                            *slot = None;
                        }
                    }
                };
                write(&format!(
                    "ready paused protocol={RUNNER_CONTROL_PROTOCOL_VERSION}\n"
                ));
                let Some(read_stream) = read_stream else {
                    let _ = sender.send(ControlCommand::Quit);
                    return;
                };
                let mut paused = true;
                let reader = std::io::BufReader::new(read_stream);
                for line in reader.lines() {
                    let Ok(line) = line else { break };
                    let Some(command) = parse_control_line(&line) else {
                        continue;
                    };
                    let quit = matches!(command, ControlCommand::Quit);
                    paused = match command {
                        ControlCommand::Pause
                        | ControlCommand::Step { .. }
                        | ControlCommand::Reset => true,
                        ControlCommand::Resume => false,
                        ControlCommand::Quit => paused,
                    };
                    if sender.send(command).is_err() {
                        break;
                    }
                    write(&format!(
                        "ok {}\n",
                        if paused { "paused" } else { "running" }
                    ));
                    if quit {
                        break;
                    }
                }
                let _ = sender.send(ControlCommand::Quit);
            })
            .map_err(|error| anyhow::anyhow!("spawn tcp control reader: {error}"))?;
        Ok((
            Self {
                receiver,
                writer,
                paused: true,
            },
            bound_port,
        ))
    }

    fn update_state(&mut self, command: ControlCommand) {
        self.paused = match command {
            ControlCommand::Pause | ControlCommand::Step { .. } | ControlCommand::Reset => true,
            ControlCommand::Resume => false,
            ControlCommand::Quit => self.paused,
        };
    }
}

impl RunnerControl for TcpRunnerControl {
    fn try_poll(&mut self) -> Option<ControlCommand> {
        let command = self.receiver.try_recv().ok();
        if let Some(command) = command {
            self.update_state(command);
        }
        command
    }

    fn wait_command(&mut self) -> ControlCommand {
        let command = self.receiver.recv().unwrap_or(ControlCommand::Quit);
        self.update_state(command);
        command
    }

    fn report_status(&mut self, step: u64, sim_time_s: f64, snapshot: &[u8]) {
        let state = if self.paused { "paused" } else { "running" };
        let line = runner_status_line(step, sim_time_s, state, snapshot);
        if let Ok(mut slot) = self.writer.lock() {
            let failed = slot.as_mut().is_some_and(|writer| {
                writer
                    .write_all(line.as_bytes())
                    .and_then(|()| writer.flush())
                    .is_err()
            });
            if failed {
                *slot = None;
            }
        }
    }
}

fn runner_status_line(step: u64, sim_time_s: f64, state: &str, snapshot: &[u8]) -> String {
    runner_status_line_with_limit(
        step,
        sim_time_s,
        state,
        snapshot,
        RUNNER_CONTROL_MAX_SNAPSHOT_BYTES,
    )
}

fn runner_status_line_with_limit(
    step: u64,
    sim_time_s: f64,
    state: &str,
    snapshot: &[u8],
    max_snapshot_bytes: usize,
) -> String {
    let sim_time = format_f64_status(sim_time_s);
    if snapshot.len() <= max_snapshot_bytes {
        return format!(
            "status step={step} t={sim_time} state={state} snapshot={}\n",
            String::from_utf8_lossy(snapshot)
        );
    }
    format!(
        "status step={step} t={sim_time} state={state} snapshot={{\"error\":\"snapshot_limit_exceeded\",\"snapshot_bytes\":{},\"limit_bytes\":{max_snapshot_bytes}}}\n",
        snapshot.len(),
    )
}

/// Compact per-step observation streamed to a live frontend.
///
/// Camera images are deterministic, nearest-neighbour previews capped by
/// [`LIVE_CAMERA_MAX_WIDTH`] and [`LIVE_CAMERA_MAX_HEIGHT`]. LiDAR points are
/// capped by [`LIVE_LIDAR_MAX_POINTS`]; replay artifacts remain the path for
/// full sensor payloads.
#[derive(serde::Serialize)]
struct LiveSnapshot<'a> {
    /// First differential-drive base translation, when present.
    base: Option<[f64; 3]>,
    /// First differential-drive base yaw, in radians, when present.
    base_yaw_rad: Option<f64>,
    /// Named joint positions, when the scene has articulated joints.
    joints: Option<LiveJointState<'a>>,
    /// Per-sensor stream summaries.
    sensors: Vec<LiveSensorStream>,
}

/// Named joint positions in a [`LiveSnapshot`].
#[derive(serde::Serialize)]
struct LiveJointState<'a> {
    /// Joint names matching the position array.
    names: &'a [String],
    /// Joint positions in radians, in name order.
    positions_rad: &'a [f64],
}

/// Sensor stream summary in a [`LiveSnapshot`].
#[derive(serde::Serialize)]
struct LiveSensorStream {
    /// DataBus stream identifier.
    stream_id: u64,
    /// Stable sensor kind label.
    kind: String,
    /// Last emitted sequence number.
    sequence: u64,
    /// Stable digest of the latest typed payload.
    payload_hash: u64,
    /// Camera preview, when this is a camera stream.
    #[serde(skip_serializing_if = "Option::is_none")]
    camera: Option<LiveCameraPreview>,
    /// Bounded world-frame LiDAR preview, when this is a LiDAR stream.
    #[serde(skip_serializing_if = "Option::is_none")]
    lidar: Option<LiveLidarPreview>,
    /// Latest IMU sample, when this is an IMU stream.
    #[serde(skip_serializing_if = "Option::is_none")]
    imu: Option<LiveImuSample>,
    /// Latest wheel encoder sample, when this is a wheel-encoder stream.
    #[serde(skip_serializing_if = "Option::is_none")]
    wheel_encoder: Option<LiveWheelEncoderSample>,
}

/// Bounded RGB-D camera preview in the runner status protocol.
#[derive(serde::Serialize)]
struct LiveCameraPreview {
    /// Source RGB width before transport downsampling.
    source_width: u32,
    /// Source RGB height before transport downsampling.
    source_height: u32,
    /// Preview width in pixels.
    width: u32,
    /// Preview height in pixels.
    height: u32,
    /// Row-major RGBA8 bytes encoded as base64 to keep the line protocol safe.
    rgba8_base64: String,
    /// Center-pixel depth in metres, when a paired depth frame is available.
    #[serde(skip_serializing_if = "Option::is_none")]
    depth_center_m: Option<f32>,
    /// Stable hash of the full paired depth frame.
    #[serde(skip_serializing_if = "Option::is_none")]
    depth_hash: Option<u64>,
    /// Source depth width when depth streaming is enabled.
    #[serde(skip_serializing_if = "Option::is_none")]
    depth_source_width: Option<u32>,
    /// Source depth height when depth streaming is enabled.
    #[serde(skip_serializing_if = "Option::is_none")]
    depth_source_height: Option<u32>,
    /// Transport depth width when full-resolution depth streaming is enabled.
    #[serde(skip_serializing_if = "Option::is_none")]
    depth_width: Option<u32>,
    /// Transport depth height when full-resolution depth streaming is enabled.
    #[serde(skip_serializing_if = "Option::is_none")]
    depth_height: Option<u32>,
    /// Little-endian f32 depth metres encoded as base64 when enabled.
    #[serde(skip_serializing_if = "Option::is_none")]
    depth_f32_le_base64: Option<String>,
}

/// Bounded world-frame LiDAR preview in the runner status protocol.
#[derive(serde::Serialize)]
struct LiveLidarPreview {
    /// Number of points in the full latest cloud.
    point_count: usize,
    /// Deterministically sampled points in metres.
    points_m: Vec<[f64; 3]>,
}

/// Latest IMU sample in a JSON-friendly fixed-size representation.
#[derive(serde::Serialize)]
struct LiveImuSample {
    /// Angular velocity in radians per second.
    angular_velocity_rad_s: [f64; 3],
    /// Linear acceleration in metres per second squared.
    linear_acceleration_m_s2: [f64; 3],
}

/// Latest wheel encoder sample in the runner status protocol.
#[derive(serde::Serialize)]
struct LiveWheelEncoderSample {
    /// Wheel position in radians.
    position_rad: f64,
    /// Wheel velocity in radians per second.
    velocity_rad_s: f64,
}

/// Serializes a [`ReplayObservation`] and base orientation into a compact
/// single-line JSON snapshot.
fn build_live_snapshot(
    observation: &ReplayObservation,
    base_yaw_rad: Option<f64>,
    bus: &InMemoryDataBus,
    options: LiveSnapshotOptions,
) -> String {
    let snapshot = LiveSnapshot {
        base: observation.base_translation_m,
        base_yaw_rad,
        joints: observation
            .joint_state
            .as_ref()
            .map(|state| LiveJointState {
                names: &state.names,
                positions_rad: &state.positions_rad,
            }),
        sensors: observation
            .sensor_streams
            .iter()
            .map(|stream| build_live_sensor_stream(stream, bus, options))
            .collect(),
    };
    serde_json::to_string(&snapshot).unwrap_or_else(|_| "{}".to_string())
}

fn build_live_sensor_stream(
    summary: &ReplaySensorStream,
    bus: &InMemoryDataBus,
    options: LiveSnapshotOptions,
) -> LiveSensorStream {
    let camera = if summary.kind == "camera" {
        let rgb = bus.latest::<ImageRgb8>(rne_data::StreamId::new(summary.stream_id));
        let depth = bus.latest::<ImageDepth>(rne_data::StreamId::new(
            summary.stream_id + rne_sensor::CAMERA_DEPTH_STREAM_OFFSET,
        ));
        rgb.as_ref().and_then(|rgb| {
            build_live_camera_preview(
                &rgb.payload,
                depth.as_ref().map(|frame| &frame.payload),
                options,
            )
        })
    } else {
        None
    };
    let lidar = if summary.kind == "lidar" {
        bus.latest::<PointCloud>(rne_data::StreamId::new(summary.stream_id))
            .map(|frame| LiveLidarPreview {
                point_count: frame.payload.points_m.len(),
                points_m: downsample_lidar_points(&frame.payload.points_m),
            })
    } else {
        None
    };
    let imu = if summary.kind == "imu" {
        bus.latest::<ImuSample>(rne_data::StreamId::new(summary.stream_id))
            .map(|frame| LiveImuSample {
                angular_velocity_rad_s: finite_vec3(frame.payload.angular_velocity_rad_s),
                linear_acceleration_m_s2: finite_vec3(frame.payload.linear_acceleration_m_s2),
            })
    } else {
        None
    };
    let wheel_encoder = if summary.kind == "wheel_encoder" {
        bus.latest::<WheelEncoderSample>(rne_data::StreamId::new(summary.stream_id))
            .map(|frame| LiveWheelEncoderSample {
                position_rad: finite_f64(frame.payload.position_rad),
                velocity_rad_s: finite_f64(frame.payload.velocity_rad_s),
            })
    } else {
        None
    };
    LiveSensorStream {
        stream_id: summary.stream_id,
        kind: summary.kind.clone(),
        sequence: summary.last_sequence,
        payload_hash: summary.payload_hash,
        camera,
        lidar,
        imu,
        wheel_encoder,
    }
}

fn build_live_camera_preview(
    rgb: &ImageRgb8,
    depth: Option<&ImageDepth>,
    options: LiveSnapshotOptions,
) -> Option<LiveCameraPreview> {
    let (width, height, rgba8) = downsample_rgba8(rgb, options)?;
    let (depth_source_width, depth_source_height, depth_width, depth_height, depth_f32_le_base64) =
        if options.include_full_depth {
            depth
                .and_then(|depth| encode_bounded_depth(depth, options))
                .map_or(
                    (None, None, None, None, None),
                    |(width, height, payload)| {
                        (
                            depth.map(|value| value.width),
                            depth.map(|value| value.height),
                            Some(width),
                            Some(height),
                            Some(payload),
                        )
                    },
                )
        } else {
            (None, None, None, None, None)
        };
    Some(LiveCameraPreview {
        source_width: rgb.width,
        source_height: rgb.height,
        width,
        height,
        rgba8_base64: base64::encode(rgba8),
        depth_center_m: depth
            .map(ImageDepth::center_depth_m)
            .filter(|value| value.is_finite()),
        depth_hash: depth.map(ImageDepth::hash_depth),
        depth_source_width,
        depth_source_height,
        depth_width,
        depth_height,
        depth_f32_le_base64,
    })
}

fn encode_bounded_depth(
    depth: &ImageDepth,
    options: LiveSnapshotOptions,
) -> Option<(u32, u32, String)> {
    let expected_len = (depth.width as usize).checked_mul(depth.height as usize)?;
    if depth.width == 0 || depth.height == 0 || depth.depth_m.len() < expected_len {
        return None;
    }
    let (width, height) = bounded_image_dimensions(depth.width, depth.height, options)?;
    let output_len = (width as usize).checked_mul(height as usize)?;
    let byte_len = output_len.checked_mul(std::mem::size_of::<f32>())?;
    let mut bytes = Vec::with_capacity(byte_len);
    for y in 0..height {
        let source_y = ((u64::from(y) * u64::from(depth.height)) / u64::from(height)) as usize;
        for x in 0..width {
            let source_x = ((u64::from(x) * u64::from(depth.width)) / u64::from(width)) as usize;
            let value = depth.depth_m[source_y * depth.width as usize + source_x];
            bytes.extend_from_slice(&value.to_le_bytes());
        }
    }
    Some((width, height, base64::encode(bytes)))
}

fn downsample_rgba8(
    image: &ImageRgb8,
    options: LiveSnapshotOptions,
) -> Option<(u32, u32, Vec<u8>)> {
    let expected_len = (image.width as usize)
        .checked_mul(image.height as usize)?
        .checked_mul(4)?;
    if image.width == 0 || image.height == 0 || image.rgba8.len() < expected_len {
        return None;
    }
    let (width, height) = bounded_image_dimensions(image.width, image.height, options)?;
    let output_len = (width as usize)
        .checked_mul(height as usize)?
        .checked_mul(4)?;
    let mut rgba8 = Vec::with_capacity(output_len);
    for y in 0..height {
        let source_y = ((u64::from(y) * u64::from(image.height)) / u64::from(height)) as usize;
        for x in 0..width {
            let source_x = ((u64::from(x) * u64::from(image.width)) / u64::from(width)) as usize;
            let source = (source_y * image.width as usize + source_x) * 4;
            rgba8.extend_from_slice(&image.rgba8[source..source + 4]);
        }
    }
    Some((width, height, rgba8))
}

fn bounded_image_dimensions(
    source_width: u32,
    source_height: u32,
    options: LiveSnapshotOptions,
) -> Option<(u32, u32)> {
    if source_width == 0 || source_height == 0 {
        return None;
    }
    let max_width = options
        .camera_max_width
        .unwrap_or(LIVE_CAMERA_TRANSPORT_MAX_WIDTH)
        .clamp(1, LIVE_CAMERA_TRANSPORT_MAX_WIDTH);
    let max_height = options
        .camera_max_height
        .unwrap_or(LIVE_CAMERA_TRANSPORT_MAX_HEIGHT)
        .clamp(1, LIVE_CAMERA_TRANSPORT_MAX_HEIGHT);
    let scale = (max_width as f64 / source_width as f64)
        .min(max_height as f64 / source_height as f64)
        .min(1.0);
    Some((
        ((source_width as f64 * scale).round() as u32).max(1),
        ((source_height as f64 * scale).round() as u32).max(1),
    ))
}

fn downsample_lidar_points(points: &[rne_math::Vec3]) -> Vec<[f64; 3]> {
    let finite_points = points
        .iter()
        .filter(|point| point.x.is_finite() && point.y.is_finite() && point.z.is_finite())
        .collect::<Vec<_>>();
    let limit = finite_points.len().min(LIVE_LIDAR_MAX_POINTS);
    if limit == 0 {
        return Vec::new();
    }
    (0..limit)
        .map(|index| {
            let source_index = if limit == 1 {
                0
            } else {
                index * (finite_points.len() - 1) / (limit - 1)
            };
            let point = finite_points[source_index];
            [point.x, point.y, point.z]
        })
        .collect()
}

fn finite_vec3(value: rne_math::Vec3) -> [f64; 3] {
    [
        finite_f64(value.x),
        finite_f64(value.y),
        finite_f64(value.z),
    ]
}

fn finite_f64(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

/// Formats a status f64 compactly for the control wire protocol.
fn format_f64_status(value: f64) -> String {
    format!("{value:.6}")
}

/// Parses one line of runner control input into a command.
fn parse_control_line(line: &str) -> Option<ControlCommand> {
    let mut parts = line.split_whitespace();
    match parts.next()? {
        "pause" => Some(ControlCommand::Pause),
        "resume" => Some(ControlCommand::Resume),
        "step" => {
            let frames = parts.next()?.parse::<u64>().ok()?;
            Some(ControlCommand::Step { frames })
        }
        "reset" => Some(ControlCommand::Reset),
        "quit" | "exit" => Some(ControlCommand::Quit),
        _ => None,
    }
}

#[cfg(test)]
fn simulate_scene(
    path: &Path,
    steps: u64,
    hz: f64,
    wheel_velocity_rad_s: f64,
) -> Result<SimulationReport> {
    Ok(simulate_scene_with_seed(path, steps, hz, wheel_velocity_rad_s, None)?.report)
}

#[derive(Debug, PartialEq)]
struct SimulationRun {
    report: SimulationReport,
    frames: Vec<ReplayFrame>,
    /// Streams whose full payloads were captured, sorted and unique.
    sensor_payload_streams: Vec<u64>,
}

#[cfg(test)]
fn simulate_scene_with_seed(
    path: &Path,
    steps: u64,
    hz: f64,
    wheel_velocity_rad_s: f64,
    seed_override: Option<u64>,
) -> Result<SimulationRun> {
    simulate_scene_with_action_schedule(
        path,
        steps,
        hz,
        ReplayAction::differential_drive(wheel_velocity_rad_s),
        seed_override,
        None,
        &[],
        None,
        &[],
        RunPhysicsBackend::Rapier,
        None,
    )
}

/// The per-episode world state the runner rebuilds on `reset`.
struct EpisodeSetup {
    world: World,
    backend: RunnerBackend,
    physics_world: PhysicsWorldId,
    drives: Vec<DifferentialDrive>,
    controller_robots: Vec<(String, rne_ecs::Entity)>,
    seed: u64,
    robot_count: usize,
}

/// Loads, spawns, and synchronizes the episode's initial world state.
fn build_episode_setup(
    path: &Path,
    seed_override: Option<u64>,
    physics_backend: RunPhysicsBackend,
) -> Result<EpisodeSetup> {
    let mut world = World::new();
    let mut bundle = load_scene_bundle(path)
        .map_err(|error| anyhow::anyhow!("load scene {}: {error}", path.display()))?;
    if let Some(seed) = seed_override {
        bundle.scene.world.seed = seed;
    }
    let spawned = spawn_scene_bundle(&mut world, &bundle, None, SpawnSceneOptions::default())
        .map_err(|error| anyhow::anyhow!("spawn scene {}: {error}", path.display()))?;
    let drives: Vec<_> = spawned
        .robots
        .iter()
        .filter_map(|(_, robot)| {
            world
                .get::<DiffDriveComponent>(robot.robot)
                .map(|drive| drive.0)
        })
        .collect();
    let controller_robots = spawned
        .robots
        .iter()
        .map(|(model_name, robot)| (model_name.clone(), robot.robot))
        .collect::<Vec<_>>();
    let controller_robots = canonicalize_controller_robots(&mut world, controller_robots)?;
    let seed = world
        .get::<WorldEntity>(spawned.world)
        .map(|entity| entity.seed)
        .unwrap_or_default();
    let mut backend = RunnerBackend::new(physics_backend);
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .map_err(|error| anyhow::anyhow!("create physics world: {error}"))?;
    backend
        .sync_from_ecs(&mut world, physics_world)
        .map_err(|error| anyhow::anyhow!("sync scene into physics: {error}"))?;
    Ok(EpisodeSetup {
        world,
        backend,
        physics_world,
        drives,
        controller_robots,
        seed,
        robot_count: spawned.robots.len(),
    })
}

fn canonicalize_controller_robots(
    world: &mut World,
    mut controller_robots: Vec<(String, rne_ecs::Entity)>,
) -> Result<Vec<(String, rne_ecs::Entity)>> {
    controller_robots.sort_by(|left, right| left.0.cmp(&right.0));
    anyhow::ensure!(
        controller_robots
            .windows(2)
            .all(|window| window[0].0 != window[1].0),
        "scene robot model names must be unique for controller scheduling"
    );
    for (robot_id, entity) in &controller_robots {
        anyhow::ensure!(
            !robot_id.trim().is_empty() && !robot_id.contains('\0'),
            "controller robot model names must be non-empty and NUL-free"
        );
        let mut robot = world.get_mut::<Robot>(*entity).ok_or_else(|| {
            anyhow::anyhow!("controller robot `{robot_id}` has no Robot component")
        })?;
        robot.model_name.clone_from(robot_id);
    }
    Ok(controller_robots)
}

#[allow(clippy::too_many_arguments)]
fn simulate_scene_with_action_schedule(
    path: &Path,
    steps: u64,
    hz: f64,
    action: ReplayAction,
    seed_override: Option<u64>,
    replay_frames: Option<&[ReplayFrame]>,
    trajectories: &[RunJointTrajectory],
    plugin: Option<Box<dyn ControllerPlugin>>,
    sensor_subscriptions: &[RunSensorSubscription],
    physics_backend: RunPhysicsBackend,
    control: Option<&mut RunControl<'_>>,
) -> Result<SimulationRun> {
    simulate_scene_with_snapshot_options(
        path,
        steps,
        hz,
        action,
        seed_override,
        replay_frames,
        trajectories,
        plugin,
        sensor_subscriptions,
        physics_backend,
        control,
        LiveSnapshotOptions::default(),
    )
}

#[allow(clippy::too_many_arguments)]
fn simulate_scene_with_snapshot_options(
    path: &Path,
    steps: u64,
    hz: f64,
    action: ReplayAction,
    seed_override: Option<u64>,
    replay_frames: Option<&[ReplayFrame]>,
    trajectories: &[RunJointTrajectory],
    plugin: Option<Box<dyn ControllerPlugin>>,
    sensor_subscriptions: &[RunSensorSubscription],
    physics_backend: RunPhysicsBackend,
    mut control: Option<&mut RunControl<'_>>,
    live_snapshot_options: LiveSnapshotOptions,
) -> Result<SimulationRun> {
    if let Some(replay_frames) = replay_frames {
        anyhow::ensure!(
            replay_frames.len() as u64 == steps,
            "replay contains {} frames but {} steps were requested",
            replay_frames.len(),
            steps
        );
    }

    let mut setup = build_episode_setup(path, seed_override, physics_backend)?;
    let controller_robot_ids = setup
        .controller_robots
        .iter()
        .map(|(robot_id, _)| robot_id.clone())
        .collect::<Vec<_>>();
    let mut scheduler = plugin
        .map(|plugin| {
            let mut scheduler = ControllerScheduler::new();
            scheduler.register("manifest_controller", plugin, controller_robot_ids.clone())?;
            Ok::<_, rne_plugin::ControllerScheduleError>(scheduler)
        })
        .transpose()
        .context("register controller plugin")?;
    if let Some(scheduler) = scheduler.as_mut() {
        let initialize = scheduler.configure().and_then(|()| {
            scheduler.activate(ControllerResetContext {
                episode: 0,
                seed: setup.seed,
                step: 0,
                sim_time_ticks: 0,
            })
        });
        if let Err(error) = initialize {
            let _ = scheduler.shutdown();
            return Err(error).context("initialize controller plugin");
        }
    }
    let mut episode = 0_u64;
    let run_result = (|| -> Result<SimulationRun> {
        'episode: loop {
            let dt = SimDuration::from_hertz(Hertz::new(hz));
            let mut sim_time = SimTime::ZERO;
            let mut command_buffer = ActuatorCommandBuffer::new();
            let mut data_bus = InMemoryDataBus::new();
            let sensor_payload_streams =
                resolve_sensor_subscriptions(&setup.world, sensor_subscriptions)?;
            let mut frames = Vec::new();
            let mut contact_pairs_max = 0_u64;
            let mut contact_impulse_max_ns = 0.0_f32;
            let mut min_base_height_m: Option<f64> = None;
            let mut step = 0_u64;
            while step < steps {
                if let Some(control) = control.as_deref_mut() {
                    match control.checkpoint() {
                        EpisodeOutcome::Advance => {}
                        EpisodeOutcome::Reset => {
                            let replacement =
                                build_episode_setup(path, seed_override, physics_backend)?;
                            let replacement_robot_ids = replacement
                                .controller_robots
                                .iter()
                                .map(|(robot_id, _)| robot_id.clone())
                                .collect::<Vec<_>>();
                            anyhow::ensure!(
                                replacement_robot_ids == controller_robot_ids,
                                "controller robot IDs changed while resetting the episode"
                            );
                            setup = replacement;
                            episode = episode.checked_add(1).ok_or_else(|| {
                                anyhow::anyhow!("controller episode index overflow")
                            })?;
                            if let Some(scheduler) = scheduler.as_mut() {
                                scheduler
                                    .reset(ControllerResetContext {
                                        episode,
                                        seed: setup.seed,
                                        step: 0,
                                        sim_time_ticks: 0,
                                    })
                                    .context("reset controller plugin")?;
                            }
                            continue 'episode;
                        }
                        EpisodeOutcome::Quit => break,
                    }
                }
                let frame_action = if let Some(replay_frames) = replay_frames {
                    let step_index = usize::try_from(step).map_err(|_| {
                        anyhow::anyhow!("replay step index {step} does not fit usize")
                    })?;
                    replay_frames[step_index].action.clone()
                } else if !trajectories.is_empty() {
                    interpolate_joint_positions(trajectories, sim_time.as_seconds().value())
                } else if let Some(scheduler) = scheduler.as_mut() {
                    plugin_controller_action(scheduler, &setup, step, sim_time)?
                } else {
                    action.clone()
                };
                apply_replay_action(
                    &setup.world,
                    &mut command_buffer,
                    &setup.drives,
                    &frame_action,
                    sim_time,
                )?;
                apply_actuator_commands(&mut setup.world, &mut command_buffer);
                sync_all_joint_motors_from_actuators(&mut setup.world);
                differential_drive_kinematics(&mut setup.world, &setup.drives, dt);
                setup
                    .backend
                    .step(&mut setup.world, setup.physics_world, dt)
                    .map_err(|error| anyhow::anyhow!("physics step: {error}"))?;
                sim_time = sim_time + dt;
                sample_sensors_for(
                    &setup.backend,
                    &mut setup.world,
                    sim_time,
                    setup.physics_world,
                    &mut data_bus,
                );

                let base_translation_m = setup.drives.first().and_then(|drive| {
                    setup
                        .world
                        .get::<Transform3>(drive.base_link)
                        .map(|transform| {
                            [
                                transform.translation.x,
                                transform.translation.y,
                                transform.translation.z,
                            ]
                        })
                });
                let contact = setup
                    .backend
                    .contacts(setup.physics_world)
                    .map_err(|error| anyhow::anyhow!("query contacts: {error}"))?;
                let contact = summarize_contacts(contact);
                contact_pairs_max = contact_pairs_max.max(contact.pair_count);
                contact_impulse_max_ns = contact_impulse_max_ns.max(contact.total_impulse_ns);
                if let Some(base_translation_m) = base_translation_m {
                    let height_m = base_translation_m[1];
                    min_base_height_m = Some(
                        min_base_height_m.map_or(height_m, |minimum: f64| minimum.min(height_m)),
                    );
                }
                let observation = ReplayObservation::new(base_translation_m)
                    .with_joint_state(capture_joint_state(&setup.world))
                    .with_sensor_streams(capture_sensor_streams(&setup.world, &data_bus))
                    .with_sensor_payloads(capture_sensor_payloads(
                        &setup.world,
                        &data_bus,
                        &sensor_payload_streams,
                    ))
                    .with_contact(Some(contact));
                let base_yaw_rad = setup.drives.first().map(|drive| {
                    yaw_rad(world_transform_of(&setup.world, drive.base_link).rotation)
                });
                let snapshot = build_live_snapshot(
                    &observation,
                    base_yaw_rad,
                    &data_bus,
                    live_snapshot_options,
                );
                frames.push(ReplayFrame::new(
                    step,
                    sim_time.ticks(),
                    frame_action,
                    observation,
                    hash_physics_state(&setup.world),
                ));
                step += 1;
                if let Some(control) = control.as_deref_mut() {
                    control.report_status(step, sim_time.as_seconds().value(), snapshot.as_bytes());
                }
            }

            let first_base_translation_m = setup.drives.first().and_then(|drive| {
                setup
                    .world
                    .get::<Transform3>(drive.base_link)
                    .map(|transform| {
                        [
                            transform.translation.x,
                            transform.translation.y,
                            transform.translation.z,
                        ]
                    })
            });
            let initial_base_height_m = frames
                .first()
                .and_then(|frame| frame.observation.base_translation_m)
                .map(|translation| translation[1]);
            let failure = classify_failure(initial_base_height_m, min_base_height_m);
            let steps_run = frames.len() as u64;
            return Ok(SimulationRun {
                report: SimulationReport {
                    steps: steps_run,
                    sim_time_s: sim_time.as_seconds().value(),
                    seed: setup.seed,
                    robot_count: setup.robot_count,
                    differential_drive_count: setup.drives.len(),
                    physics_hash: hash_physics_state(&setup.world),
                    first_base_translation_m,
                    contact_pairs_max,
                    contact_impulse_max_ns,
                    min_base_height_m,
                    failure,
                },
                frames,
                sensor_payload_streams,
            });
        }
    })();
    let shutdown_result = scheduler
        .as_mut()
        .map(|scheduler| scheduler.shutdown().context("shutdown controller plugin"))
        .transpose();
    match (run_result, shutdown_result) {
        (Ok(run), Ok(_)) => Ok(run),
        (Err(run_error), _) => Err(run_error),
        (Ok(_), Err(shutdown_error)) => Err(shutdown_error),
    }
}

fn direct_action(
    wheel_velocity_rad_s: f64,
    joint: Option<&str>,
    joint_velocity_rad_s: Option<f64>,
    joint_effort_nm: Option<f64>,
) -> Result<ReplayAction> {
    anyhow::ensure!(
        !(joint_velocity_rad_s.is_some() && joint_effort_nm.is_some()),
        "choose only one of --joint-velocity-rad-s and --joint-effort-nm"
    );
    if let Some(joint) = joint {
        anyhow::ensure!(
            wheel_velocity_rad_s == 0.0,
            "--wheel-velocity-rad-s cannot be combined with --joint"
        );
        anyhow::ensure!(!joint.trim().is_empty(), "--joint must not be empty");
        if let Some(velocity_rad_s) = joint_velocity_rad_s {
            return Ok(ReplayAction::joint_velocity(joint, velocity_rad_s));
        }
        if let Some(effort_nm) = joint_effort_nm {
            return Ok(ReplayAction::joint_effort(joint, effort_nm));
        }
        anyhow::bail!("--joint requires --joint-velocity-rad-s or --joint-effort-nm");
    }
    anyhow::ensure!(
        joint_velocity_rad_s.is_none() && joint_effort_nm.is_none(),
        "--joint is required for a named joint command"
    );
    Ok(ReplayAction::differential_drive(wheel_velocity_rad_s))
}

fn ensure_action_is_finite(action: &ReplayAction) -> Result<()> {
    match action {
        ReplayAction::DifferentialDrive {
            wheel_velocity_rad_s,
        } => anyhow::ensure!(
            wheel_velocity_rad_s.is_finite(),
            "wheel velocity command must be finite"
        ),
        ReplayAction::JointVelocity {
            joint,
            velocity_rad_s,
        } => {
            anyhow::ensure!(
                !joint.trim().is_empty(),
                "joint command name must not be empty"
            );
            anyhow::ensure!(
                velocity_rad_s.is_finite(),
                "joint velocity command must be finite"
            );
        }
        ReplayAction::JointEffort { joint, effort_nm } => {
            anyhow::ensure!(
                !joint.trim().is_empty(),
                "joint command name must not be empty"
            );
            anyhow::ensure!(effort_nm.is_finite(), "joint effort command must be finite");
        }
        ReplayAction::JointPositions { samples } => {
            for sample in samples {
                anyhow::ensure!(
                    !sample.joint.trim().is_empty(),
                    "joint positions command name must not be empty"
                );
                anyhow::ensure!(
                    sample.position_rad.is_finite(),
                    "joint positions command must be finite"
                );
            }
        }
        ReplayAction::JointVelocities { samples } => {
            for sample in samples {
                anyhow::ensure!(
                    !sample.joint.trim().is_empty(),
                    "joint velocities command name must not be empty"
                );
                anyhow::ensure!(
                    sample.velocity_rad_s.is_finite(),
                    "joint velocities command must be finite"
                );
            }
        }
        ReplayAction::RobotJointVelocities { samples } => {
            for sample in samples {
                anyhow::ensure!(
                    !sample.robot_id.trim().is_empty() && !sample.robot_id.contains('\0'),
                    "robot joint velocities robot ID must be non-empty and NUL-free"
                );
                anyhow::ensure!(
                    !sample.joint.trim().is_empty() && !sample.joint.contains('\0'),
                    "robot joint velocities command name must be non-empty and NUL-free"
                );
                anyhow::ensure!(
                    sample.velocity_rad_s.is_finite(),
                    "robot joint velocities command must be finite"
                );
            }
        }
    }
    Ok(())
}

fn interpolate_joint_positions(trajectories: &[RunJointTrajectory], t_s: f64) -> ReplayAction {
    let samples = trajectories
        .iter()
        .map(|trajectory| ReplayJointPosition {
            joint: trajectory.joint.clone(),
            position_rad: sample_trajectory(&trajectory.waypoints, t_s),
        })
        .collect();
    ReplayAction::JointPositions { samples }
}

fn sample_trajectory(waypoints: &[RunTrajectoryWaypoint], t_s: f64) -> f64 {
    if t_s <= waypoints[0].t_s {
        return waypoints[0].position_rad;
    }
    let last = waypoints.last().expect("validated waypoints");
    if t_s >= last.t_s {
        return last.position_rad;
    }
    for window in waypoints.windows(2) {
        if t_s >= window[0].t_s && t_s <= window[1].t_s {
            let span_s = window[1].t_s - window[0].t_s;
            let fraction = (t_s - window[0].t_s) / span_s;
            return window[0].position_rad
                + fraction * (window[1].position_rad - window[0].position_rad);
        }
    }
    unreachable!("t_s is clamped between the first and last waypoints")
}

/// Invokes the controller scheduler on a canonical robot-scoped observation.
fn plugin_controller_action(
    scheduler: &mut ControllerScheduler,
    setup: &EpisodeSetup,
    step: u64,
    sim_time: SimTime,
) -> Result<ReplayAction> {
    let observation =
        capture_controller_observation(&setup.world, &setup.controller_robots, step, sim_time)?;
    let action = scheduler
        .step(&observation)
        .context("step controller plugin")?;
    controller_replay_action(action)
}

fn controller_replay_action(action: ControllerActionFrame) -> Result<ReplayAction> {
    let samples = action
        .robots
        .into_iter()
        .flat_map(|robot| {
            robot
                .joint_velocities
                .into_iter()
                .map(move |command| ReplayRobotJointVelocity {
                    robot_id: robot.robot_id.clone(),
                    joint: command.name,
                    velocity_rad_s: command.velocity_rad_s,
                })
        })
        .collect::<Vec<_>>();
    anyhow::ensure!(
        !samples.is_empty(),
        "controller plugin produced no joint velocity commands"
    );
    Ok(ReplayAction::RobotJointVelocities { samples })
}

fn capture_controller_observation(
    world: &World,
    controller_robots: &[(String, rne_ecs::Entity)],
    step: u64,
    sim_time: SimTime,
) -> Result<ControllerObservationFrame> {
    let mut robots = Vec::with_capacity(controller_robots.len());
    let mut articulated_joint_count = 0_usize;
    for (robot_id, robot_entity) in controller_robots {
        let robot = world.get::<Robot>(*robot_entity).ok_or_else(|| {
            anyhow::anyhow!("controller robot `{robot_id}` has no Robot component")
        })?;
        anyhow::ensure!(
            robot.model_name == *robot_id,
            "controller robot identity mismatch: expected `{robot_id}`, found `{}`",
            robot.model_name
        );
        let joints = world
            .iter_entities()
            .filter_map(|entity_ref| {
                let entity = entity_ref.id();
                let joint = world.get::<Joint>(entity)?;
                if joint.robot != *robot_entity || joint.kind == JointKind::Fixed {
                    return None;
                }
                let name = world.get::<Name>(entity)?;
                Some(ControllerJointObservation::position_velocity(
                    name.0.clone(),
                    joint.position,
                    joint.velocity,
                ))
            })
            .collect::<Vec<_>>();
        articulated_joint_count += joints.len();
        robots.push(ControllerRobotObservation::new(robot_id.clone(), joints)?);
    }
    anyhow::ensure!(
        articulated_joint_count > 0,
        "plugin controller requires articulated joints in the scene"
    );
    Ok(ControllerObservationFrame::new(
        step,
        sim_time.ticks(),
        robots,
    )?)
}

fn apply_replay_action(
    world: &World,
    command_buffer: &mut ActuatorCommandBuffer,
    drives: &[rne_robot::DifferentialDrive],
    action: &ReplayAction,
    sim_time: SimTime,
) -> Result<()> {
    match action {
        ReplayAction::DifferentialDrive {
            wheel_velocity_rad_s,
        } => {
            for drive in drives {
                command_buffer.push(
                    ActuatorCommand::WheelVelocity {
                        wheel: drive.left_actuator,
                        velocity_rad_s: *wheel_velocity_rad_s,
                    },
                    sim_time,
                );
                command_buffer.push(
                    ActuatorCommand::WheelVelocity {
                        wheel: drive.right_actuator,
                        velocity_rad_s: *wheel_velocity_rad_s,
                    },
                    sim_time,
                );
            }
        }
        ReplayAction::JointVelocity {
            joint,
            velocity_rad_s,
        } => {
            let joints = named_joint_entities(world, joint)?;
            for joint in joints {
                command_buffer.push(
                    ActuatorCommand::JointVelocity {
                        joint,
                        velocity_rad_s: *velocity_rad_s,
                    },
                    sim_time,
                );
            }
        }
        ReplayAction::JointEffort { joint, effort_nm } => {
            let joints = named_joint_entities(world, joint)?;
            for joint in joints {
                command_buffer.push(
                    ActuatorCommand::JointEffort {
                        joint,
                        effort_nm: *effort_nm,
                    },
                    sim_time,
                );
            }
        }
        ReplayAction::JointPositions { samples } => {
            for sample in samples {
                let joints = named_joint_entities(world, &sample.joint)?;
                for joint in joints {
                    command_buffer.push(
                        ActuatorCommand::JointPosition {
                            joint,
                            position_rad: sample.position_rad,
                        },
                        sim_time,
                    );
                }
            }
        }
        ReplayAction::JointVelocities { samples } => {
            for sample in samples {
                let joints = named_joint_entities(world, &sample.joint)?;
                for joint in joints {
                    command_buffer.push(
                        ActuatorCommand::JointVelocity {
                            joint,
                            velocity_rad_s: sample.velocity_rad_s,
                        },
                        sim_time,
                    );
                }
            }
        }
        ReplayAction::RobotJointVelocities { samples } => {
            for sample in samples {
                let joint = robot_named_joint_entity(world, &sample.robot_id, &sample.joint)?;
                command_buffer.push(
                    ActuatorCommand::JointVelocity {
                        joint,
                        velocity_rad_s: sample.velocity_rad_s,
                    },
                    sim_time,
                );
            }
        }
    }
    Ok(())
}

fn robot_named_joint_entity(
    world: &World,
    robot_id: &str,
    joint_name: &str,
) -> Result<rne_ecs::Entity> {
    let mut robots = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let robot = world.get::<Robot>(entity)?;
            (robot.model_name == robot_id).then_some(entity)
        })
        .collect::<Vec<_>>();
    robots.sort_unstable();
    anyhow::ensure!(
        robots.len() == 1,
        "expected one controller robot named `{robot_id}`, found {}",
        robots.len()
    );
    let robot = robots[0];
    let mut joints = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let name = world.get::<Name>(entity)?;
            let joint = world.get::<Joint>(entity)?;
            (joint.robot == robot && name.0 == joint_name).then_some(entity)
        })
        .collect::<Vec<_>>();
    joints.sort_unstable();
    anyhow::ensure!(
        joints.len() == 1,
        "expected one joint named `{joint_name}` on controller robot `{robot_id}`, found {}",
        joints.len()
    );
    anyhow::ensure!(
        world.get::<Actuator>(joints[0]).is_some(),
        "joint `{joint_name}` on robot `{robot_id}` has no actuator; enable URDF articulation for this scene"
    );
    Ok(joints[0])
}

fn named_joint_entities(world: &World, name: &str) -> Result<Vec<rne_ecs::Entity>> {
    let mut joints = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let entity_name = world.get::<Name>(entity)?;
            if entity_name.0 != name || world.get::<Joint>(entity).is_none() {
                return None;
            }
            Some(entity)
        })
        .collect::<Vec<_>>();
    joints.sort_unstable();
    anyhow::ensure!(
        !joints.is_empty(),
        "no ECS joint named `{name}` exists in the scene"
    );
    for joint in &joints {
        anyhow::ensure!(
            world.get::<Actuator>(*joint).is_some(),
            "joint `{name}` has no actuator; enable URDF articulation for this scene"
        );
    }
    Ok(joints)
}

fn capture_joint_state(world: &World) -> Option<ReplayJointState> {
    let mut joints = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let name = world.get::<Name>(entity)?;
            let joint = world.get::<Joint>(entity)?;
            if joint.kind == JointKind::Fixed {
                return None;
            }
            Some((name.0.clone(), joint.position, joint.velocity, entity))
        })
        .collect::<Vec<_>>();
    joints.sort_by(|left, right| left.0.cmp(&right.0).then_with(|| left.3.cmp(&right.3)));
    if joints.is_empty() {
        return None;
    }
    Some(ReplayJointState {
        names: joints.iter().map(|joint| joint.0.clone()).collect(),
        positions_rad: joints.iter().map(|joint| joint.1).collect(),
        velocities_rad_s: joints.iter().map(|joint| joint.2).collect(),
    })
}

fn capture_sensor_streams(world: &World, bus: &InMemoryDataBus) -> Vec<ReplaySensorStream> {
    let mut sensors = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            world.get::<Sensor>(entity).map(|_| entity)
        })
        .collect::<Vec<_>>();
    sensors.sort_unstable_by_key(|entity| {
        world
            .get::<Sensor>(*entity)
            .map(|sensor| (sensor.stream_id.0, *entity))
    });

    sensors
        .into_iter()
        .filter_map(|entity| {
            let sensor = world.get::<Sensor>(entity)?;
            let state = world
                .get::<SensorState>(entity)
                .cloned()
                .unwrap_or_default();
            let (kind, payload_hash) = match &sensor.kind {
                SensorKind::Imu(_) => (
                    "imu",
                    latest_payload_hash::<ImuSample>(bus, sensor.stream_id),
                ),
                SensorKind::Lidar(_) => (
                    "lidar",
                    latest_payload_hash::<PointCloud>(bus, sensor.stream_id),
                ),
                SensorKind::Camera(_) => (
                    "camera",
                    combine_hashes(
                        latest_payload_hash::<ImageRgb8>(bus, sensor.stream_id),
                        latest_payload_hash::<ImageDepth>(
                            bus,
                            rne_data::StreamId::new(
                                sensor.stream_id.0 + rne_sensor::CAMERA_DEPTH_STREAM_OFFSET,
                            ),
                        ),
                    ),
                ),
                SensorKind::WheelEncoder(_) => (
                    "wheel_encoder",
                    latest_payload_hash::<WheelEncoderSample>(bus, sensor.stream_id),
                ),
            };
            Some(ReplaySensorStream {
                stream_id: sensor.stream_id.0,
                kind: kind.to_string(),
                frame_count: state.frame_count,
                last_sequence: state.last_sequence,
                payload_hash,
            })
        })
        .collect()
}

fn latest_payload_hash<T>(bus: &InMemoryDataBus, stream: rne_data::StreamId) -> u64
where
    T: rne_data::FramePayload + Serialize,
{
    bus.latest::<T>(stream)
        .map(|frame| stable_payload_hash(&frame.payload))
        .unwrap_or(0)
}

fn stable_payload_hash<T: Serialize>(payload: &T) -> u64 {
    let bytes = serde_json::to_vec(payload).unwrap_or_default();
    let mut hash = 0xcbf29ce484222325_u64;
    for byte in bytes {
        hash ^= u64::from(byte);
        hash = hash.wrapping_mul(0x100000001b3);
    }
    hash
}

fn combine_hashes(first: u64, second: u64) -> u64 {
    first.wrapping_mul(0x9e3779b185ebca87).rotate_left(17) ^ second
}

fn resolve_sensor_subscriptions(
    world: &World,
    subscriptions: &[RunSensorSubscription],
) -> Result<Vec<u64>> {
    if subscriptions.is_empty() {
        return Ok(Vec::new());
    }
    let mut sensors = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let sensor = world.get::<Sensor>(entity)?;
            let name = world.get::<Name>(entity).map(|name| name.0.clone());
            Some((sensor.stream_id.0, name, sensor.kind.clone()))
        })
        .collect::<Vec<_>>();
    sensors.sort_unstable_by_key(|(stream_id, _, _)| *stream_id);

    let mut matched = Vec::new();
    for subscription in subscriptions {
        let matched_before = matched.len();
        for (stream_id, name, kind) in &sensors {
            let matches_name = subscription
                .name
                .as_deref()
                .map(|wanted| name.as_deref() == Some(wanted))
                .unwrap_or(false);
            let matches_kind = subscription
                .kind
                .map(|wanted| run_sensor_kind_matches(kind, wanted))
                .unwrap_or(false);
            if (matches_name || matches_kind) && !matched.contains(stream_id) {
                matched.push(*stream_id);
            }
        }
        if matched.len() == matched_before {
            let selector = match (&subscription.name, &subscription.kind) {
                (Some(name), _) => format!("name `{name}`"),
                (_, Some(kind)) => format!("kind `{kind:?}`"),
                (None, None) => "without a selector".to_string(),
            };
            anyhow::bail!("sensor subscription by {selector} matched no sensor in the scene");
        }
    }
    matched.sort_unstable();
    Ok(matched)
}

fn run_sensor_kind_matches(kind: &SensorKind, wanted: RunSensorKind) -> bool {
    matches!(
        (kind, wanted),
        (SensorKind::Imu(_), RunSensorKind::Imu)
            | (SensorKind::Lidar(_), RunSensorKind::Lidar)
            | (SensorKind::Camera(_), RunSensorKind::Camera)
            | (SensorKind::WheelEncoder(_), RunSensorKind::WheelEncoder)
    )
}

/// Collapses the physics backend's per-step contact events into a replay summary.
fn summarize_contacts(contacts: &[ContactEvent]) -> ReplayContact {
    let pair_count = contacts.len() as u64;
    let mut total_impulse_ns = 0.0_f32;
    let mut max_impulse_ns = 0.0_f32;
    for contact in contacts {
        let impulse = contact.impulse.max(0.0);
        total_impulse_ns += impulse;
        max_impulse_ns = max_impulse_ns.max(impulse);
    }
    ReplayContact {
        pair_count,
        total_impulse_ns,
        max_impulse_ns,
    }
}

/// Classifies a run failure from the settle-height threshold.
///
/// A robot "fell" when its base height dropped below half of its initial base
/// height. The threshold is deliberately qualitative: the runner does not know
/// each robot's intended working height, so it flags any drop that cannot be a
/// normal standing pose. Runs without a tracked differential-drive base never
/// fail this check.
fn classify_failure(
    initial_base_height_m: Option<f64>,
    min_base_height_m: Option<f64>,
) -> Option<ReplayFailureKind> {
    let (initial_height_m, min_height_m) = match (initial_base_height_m, min_base_height_m) {
        (Some(initial), Some(minimum)) => (initial, minimum),
        _ => return None,
    };
    if !initial_height_m.is_finite() || initial_height_m <= 0.0 {
        return None;
    }
    if min_height_m < 0.5 * initial_height_m {
        Some(ReplayFailureKind::Fell)
    } else {
        None
    }
}

/// Converts a SUMO `.net.xml` road network into a `.rne.traffic.json` asset.
fn sumo_net_command(path: &Path, out: &Path, network_id: &str) -> Result<()> {
    let network_id = TrafficId::new(network_id)
        .map_err(|error| anyhow::anyhow!("invalid network id `{network_id}`: {error}"))?;
    let asset = import_sumo_net_file(&network_id, path)
        .with_context(|| format!("import SUMO network {}", path.display()))?;
    println!(
        "sumo: imported {} lanes={} junctions={} connections={}",
        path.display(),
        asset.network.lanes.len(),
        asset.network.junctions.len(),
        asset.network.connections.len()
    );
    if let Some(parent) = out.parent() {
        if !parent.as_os_str().is_empty() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("create output directory {}", parent.display()))?;
        }
    }
    save_traffic_asset(out, &asset)
        .with_context(|| format!("write traffic asset {}", out.display()))?;
    println!(
        "traffic: wrote {} (network={} schema_version={})",
        out.display(),
        asset.network.id,
        asset.schema_version
    );
    Ok(())
}

/// Result of one headless SUMO co-simulation run.
#[derive(Clone, Debug, PartialEq)]
struct CoSimReport {
    /// Number of co-simulation steps executed.
    steps: u64,
    /// Number of mirrored vehicles after the last step.
    final_actor_count: usize,
    /// Deterministic hash over every step's sorted vehicle states.
    stable_hash: u64,
}

/// Folds per-step sorted `(vehicle id, RNE position)` states into a stable hash.
fn stable_co_sim_hash(step_states: &[Vec<(String, [f64; 3])>]) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0100_0000_01b3;
    let mut hash = OFFSET;
    for state in step_states {
        for (id, position) in state {
            for byte in id.as_bytes() {
                hash = hash.wrapping_mul(PRIME).wrapping_add(u64::from(*byte));
            }
            for value in position {
                for byte in value.to_le_bytes() {
                    hash = hash.wrapping_mul(PRIME).wrapping_add(u64::from(byte));
                }
            }
        }
        hash = hash.wrapping_add(0x9e37_79b9_7f4a_7c15);
    }
    hash
}

/// Spawns SUMO with a net and route file and connects a TraCI client.
fn spawn_sumo_and_connect(net: &Path, routes: &Path) -> Result<(std::process::Child, TraciClient)> {
    let listener = std::net::TcpListener::bind("127.0.0.1:0")
        .map_err(|error| anyhow::anyhow!("bind co-sim port probe: {error}"))?;
    let port = listener
        .local_addr()
        .map_err(|error| anyhow::anyhow!("co-sim port probe: {error}"))?
        .port();
    drop(listener);
    let stderr_log = std::env::temp_dir().join(format!("rne-co-sim-sumo-{port}.log"));
    let stderr_file = std::fs::File::create(&stderr_log)
        .map_err(|error| anyhow::anyhow!("create sumo stderr log: {error}"))?;
    let mut child = std::process::Command::new("sumo")
        .args([
            "--net-file",
            net.to_str().expect("net path"),
            "--route-files",
            routes.to_str().expect("route path"),
            "--remote-port",
            &port.to_string(),
            "--start",
            "--no-warnings",
            "--no-step-log",
        ])
        .stdout(std::process::Stdio::null())
        .stderr(std::process::Stdio::from(stderr_file))
        .spawn()
        .map_err(|error| anyhow::anyhow!("spawn sumo: {error}"))?;
    let mut client = None;
    for _ in 0..100 {
        match TraciClient::connect("127.0.0.1", port) {
            Ok(connected) => {
                client = Some(connected);
                break;
            }
            Err(_) => std::thread::sleep(Duration::from_millis(100)),
        }
    }
    if let Some(client) = client {
        Ok((child, client))
    } else {
        let _ = child.kill();
        let _ = child.wait();
        let log = std::fs::read_to_string(&stderr_log).unwrap_or_default();
        anyhow::bail!("could not connect to SUMO on port {port}; stderr:\n{log}");
    }
}

/// Runs one headless SUMO co-simulation and returns its report.
fn run_co_simulation(net: &Path, routes: &Path, steps: u64) -> Result<CoSimReport> {
    let (mut child, client) = spawn_sumo_and_connect(net, routes)?;
    let mut co_sim = CoSimulation::from_client(client);
    let mut world = rne_ecs::World::new();
    let mut step_states: Vec<Vec<(String, [f64; 3])>> = Vec::new();
    for _ in 0..steps {
        co_sim
            .step(&mut world)
            .map_err(|error| anyhow::anyhow!("co-simulation step: {error}"))?;
        let mut state = co_sim
            .actors()
            .iter()
            .map(|(id, entity)| {
                let position = world
                    .get::<rne_traffic::TrafficPose>(*entity)
                    .map(|pose| pose.position_m)
                    .unwrap_or([f64::NAN; 3]);
                (id.clone(), position)
            })
            .collect::<Vec<_>>();
        state.sort_by(|left, right| left.0.cmp(&right.0));
        step_states.push(state);
    }
    let final_actor_count = co_sim.actors().len();
    let stable_hash = stable_co_sim_hash(&step_states);
    let _ = co_sim.close();
    let status = child
        .wait()
        .map_err(|error| anyhow::anyhow!("wait for sumo: {error}"))?;
    if !status.success() {
        anyhow::bail!("sumo exited with status {status}");
    }
    Ok(CoSimReport {
        steps,
        final_actor_count,
        stable_hash,
    })
}

/// Runs a headless SUMO co-simulation and prints the report.
fn co_sim_command(path: &Path, routes: &Path, steps: u64, determinism_check: bool) -> Result<()> {
    let report = run_co_simulation(path, routes, steps)
        .with_context(|| format!("co-simulate SUMO network {}", path.display()))?;
    println!(
        "co-sim: net={} routes={} steps={} final_actors={} stable_hash={:#018x}",
        path.display(),
        routes.display(),
        report.steps,
        report.final_actor_count,
        report.stable_hash
    );
    if determinism_check {
        let replay = run_co_simulation(path, routes, steps)
            .with_context(|| format!("re-run co-simulation for {}", path.display()))?;
        anyhow::ensure!(
            report.stable_hash == replay.stable_hash,
            "co-simulation determinism check failed: first={:#x} replay={:#x}",
            report.stable_hash,
            replay.stable_hash
        );
        println!("determinism: identical co-simulation outcome");
    }
    Ok(())
}

/// Dispatches the `plugin` subcommands.
fn plugin_command(command: PluginCommand) -> Result<()> {
    match command {
        PluginCommand::New { name, dir } => {
            let crate_dir = rne_plugin::scaffold_controller_plugin(&name, &dir)
                .map_err(|error| anyhow::anyhow!("{error}"))?;
            println!("plugin: scaffolded `{name}` at {}", crate_dir.display());
            println!(
                "  cargo build --manifest-path {}/Cargo.toml",
                crate_dir.display()
            );
            println!(
                "  plugin name `{name}`, controller ABI versions {RNE_PLUGIN_MIN_ABI_VERSION}..={RNE_PLUGIN_ABI_VERSION}"
            );
            Ok(())
        }
        PluginCommand::List { path } => {
            let paths: Vec<&Path> = path.iter().map(PathBuf::as_path).collect();
            println!("built-in: velocity_servo");
            let discovered = rne_plugin::discover_plugin_names(&paths)
                .map_err(|error| anyhow::anyhow!("{error}"))?;
            for (name, library_path) in discovered {
                println!("discovered: {name} ({})", library_path.display());
            }
            Ok(())
        }
    }
}

fn capture_sensor_payloads(
    world: &World,
    bus: &InMemoryDataBus,
    sensor_payload_streams: &[u64],
) -> Vec<ReplaySensorPayload> {
    if sensor_payload_streams.is_empty() {
        return Vec::new();
    }
    let mut sensors = world
        .iter_entities()
        .filter_map(|entity_ref| {
            let entity = entity_ref.id();
            let sensor = world.get::<Sensor>(entity)?;
            if !sensor_payload_streams.contains(&sensor.stream_id.0) {
                return None;
            }
            Some(sensor.clone())
        })
        .collect::<Vec<_>>();
    sensors.sort_unstable_by_key(|sensor| sensor.stream_id.0);
    sensors
        .into_iter()
        .filter_map(|sensor| {
            let (kind, sequence, data) = match &sensor.kind {
                SensorKind::Imu(_) => bus.latest::<ImuSample>(sensor.stream_id).map(|frame| {
                    (
                        "imu",
                        frame.sequence,
                        ReplaySensorPayloadData::Imu(frame.payload),
                    )
                })?,
                SensorKind::Lidar(_) => {
                    bus.latest::<PointCloud>(sensor.stream_id).map(|frame| {
                        (
                            "lidar",
                            frame.sequence,
                            ReplaySensorPayloadData::Lidar(frame.payload),
                        )
                    })?
                }
                SensorKind::Camera(_) => {
                    let rgb = bus.latest::<ImageRgb8>(sensor.stream_id);
                    let depth = bus.latest::<ImageDepth>(rne_data::StreamId::new(
                        sensor.stream_id.0 + CAMERA_DEPTH_STREAM_OFFSET,
                    ));
                    let (rgb, depth) = (rgb?, depth?);
                    (
                        "camera",
                        rgb.sequence,
                        ReplaySensorPayloadData::Camera {
                            rgb: rgb.payload,
                            depth: depth.payload,
                        },
                    )
                }
                SensorKind::WheelEncoder(_) => bus
                    .latest::<WheelEncoderSample>(sensor.stream_id)
                    .map(|frame| {
                    (
                        "wheel_encoder",
                        frame.sequence,
                        ReplaySensorPayloadData::WheelEncoder(frame.payload),
                    )
                })?,
            };
            Some(ReplaySensorPayload {
                stream_id: sensor.stream_id.0,
                kind: kind.to_string(),
                sequence,
                data,
            })
        })
        .collect()
}

fn replay_command(path: &Path) -> Result<()> {
    let text = std::fs::read_to_string(path)
        .with_context(|| format!("load replay artifact {}", path.display()))?;
    let is_scenario_replay = serde_json::from_str::<serde_json::Value>(&text)
        .ok()
        .and_then(|value| {
            value
                .get("kind")
                .and_then(serde_json::Value::as_str)
                .map(|kind| kind == SCENARIO_REPLAY_KIND)
        })
        .unwrap_or(false);
    if is_scenario_replay {
        let artifact = ScenarioReplayArtifact::from_json(&text)
            .with_context(|| format!("parse scenario replay artifact {}", path.display()))?;
        return replay_scenario_command(path, &artifact);
    }
    let artifact = ReplayArtifact::from_json(&text)
        .with_context(|| format!("parse replay artifact {}", path.display()))?;
    let scene_path = PathBuf::from(&artifact.scene);
    let run = simulate_scene_with_action_schedule(
        &scene_path,
        artifact.clock.steps,
        artifact.clock.hz,
        ReplayAction::differential_drive(0.0),
        Some(artifact.seed),
        Some(&artifact.frames),
        &[],
        None,
        &[],
        RunPhysicsBackend::Rapier,
        None,
    )
    .with_context(|| format!("replay scene {}", scene_path.display()))?;

    ensure_replay_frames(&artifact.frames, &run.frames)?;
    let actual_report = replay_final_report(&run.report);
    ensure_replay_reports(&artifact.final_report, &actual_report)?;
    println!(
        "replay verified artifact={} scene={} steps={} physics_hash={:#018x}",
        path.display(),
        scene_path.display(),
        artifact.clock.steps,
        actual_report.physics_hash
    );
    Ok(())
}

/// Re-executes and verifies a deterministic OpenSCENARIO replay artifact.
fn replay_scenario_command(path: &Path, artifact: &ScenarioReplayArtifact) -> Result<()> {
    anyhow::ensure!(
        artifact.replayable,
        "scenario replay artifact {} is marked non-replayable",
        path.display()
    );
    anyhow::ensure!(
        artifact.engine_version == env!("CARGO_PKG_VERSION"),
        "scenario replay engine version mismatch: artifact={} runtime={}; replay with the producing RNE version",
        artifact.engine_version,
        env!("CARGO_PKG_VERSION")
    );
    let xosc_path = PathBuf::from(&artifact.scenario_path);
    let network_path = PathBuf::from(&artifact.network_path);
    let actual_scenario_digest = replay_input_digest(&xosc_path, "OpenSCENARIO")?;
    anyhow::ensure!(
        actual_scenario_digest == artifact.scenario_digest,
        "scenario replay OpenSCENARIO input changed: expected={:#018x} actual={:#018x} path={}",
        artifact.scenario_digest,
        actual_scenario_digest,
        xosc_path.display()
    );
    let actual_network_digest = replay_input_digest(&network_path, "traffic network")?;
    anyhow::ensure!(
        actual_network_digest == artifact.network_digest,
        "scenario replay traffic-network input changed: expected={:#018x} actual={:#018x} path={}",
        artifact.network_digest,
        actual_network_digest,
        network_path.display()
    );
    let (document, network, _) = load_scenario_inputs(&xosc_path, Some(&network_path))?;
    let actual = if artifact.control_commands.is_empty() {
        execute_scenario(&document, &network, &artifact.options)
            .with_context(|| format!("replay scenario {}", xosc_path.display()))?
    } else {
        let mut transport = ScriptedRunnerControl::new(artifact.control_commands.clone());
        let mut control = RunControl::paused(&mut transport);
        execute_scenario_with_control(&document, &network, &artifact.options, Some(&mut control))
            .with_context(|| format!("replay controlled scenario {}", xosc_path.display()))?
    };
    anyhow::ensure!(
        actual.steps == artifact.executed_steps,
        "scenario replay step count mismatch: expected={} actual={}",
        artifact.executed_steps,
        actual.steps
    );
    anyhow::ensure!(
        actual == artifact.result,
        "scenario replay result mismatch: expected={:?} actual={:?}",
        artifact.result,
        actual
    );
    println!(
        "scenario replay verified artifact={} xosc={} steps={} stable_hash={:#018x}",
        path.display(),
        xosc_path.display(),
        actual.steps,
        actual.stable_hash
    );
    Ok(())
}

fn ensure_replay_frames(expected: &[ReplayFrame], actual: &[ReplayFrame]) -> Result<()> {
    anyhow::ensure!(
        expected.len() == actual.len(),
        "replay frame count mismatch: expected={} actual={}",
        expected.len(),
        actual.len()
    );
    for (expected_frame, actual_frame) in expected.iter().zip(actual) {
        anyhow::ensure!(
            expected_frame.step == actual_frame.step,
            "replay frame index mismatch: expected={} actual={}",
            expected_frame.step,
            actual_frame.step
        );
        anyhow::ensure!(
            expected_frame.sim_ticks == actual_frame.sim_ticks,
            "replay sim_ticks mismatch at step {}: expected={} actual={}",
            expected_frame.step,
            expected_frame.sim_ticks,
            actual_frame.sim_ticks
        );
        anyhow::ensure!(
            replay_action_matches(&expected_frame.action, &actual_frame.action),
            "replay action mismatch at step {}: expected={:?} actual={:?}",
            expected_frame.step,
            expected_frame.action,
            actual_frame.action
        );
        anyhow::ensure!(
            replay_translation_matches(
                expected_frame.observation.base_translation_m,
                actual_frame.observation.base_translation_m
            ),
            "replay observation mismatch at step {}: expected={:?} actual={:?}",
            expected_frame.step,
            expected_frame.observation.base_translation_m,
            actual_frame.observation.base_translation_m
        );
        anyhow::ensure!(
            replay_joint_state_matches(
                expected_frame.observation.joint_state.as_ref(),
                actual_frame.observation.joint_state.as_ref()
            ),
            "replay joint state mismatch at step {}: expected={:?} actual={:?}",
            expected_frame.step,
            expected_frame.observation.joint_state,
            actual_frame.observation.joint_state
        );
        anyhow::ensure!(
            expected_frame.observation.sensor_streams == actual_frame.observation.sensor_streams,
            "replay sensor stream mismatch at step {}: expected={:?} actual={:?}",
            expected_frame.step,
            expected_frame.observation.sensor_streams,
            actual_frame.observation.sensor_streams
        );
        anyhow::ensure!(
            replay_contact_matches(
                expected_frame.observation.contact,
                actual_frame.observation.contact
            ),
            "replay contact mismatch at step {}: expected={:?} actual={:?}",
            expected_frame.step,
            expected_frame.observation.contact,
            actual_frame.observation.contact
        );
        anyhow::ensure!(
            expected_frame.physics_hash == actual_frame.physics_hash,
            "replay frame mismatch at step {}: expected={expected_frame:?} actual={actual_frame:?}",
            expected_frame.step
        );
    }
    Ok(())
}

fn replay_action_matches(expected: &ReplayAction, actual: &ReplayAction) -> bool {
    match (expected, actual) {
        (
            ReplayAction::DifferentialDrive {
                wheel_velocity_rad_s: expected,
            },
            ReplayAction::DifferentialDrive {
                wheel_velocity_rad_s: actual,
            },
        ) => replay_float_matches(*expected, *actual),
        (
            ReplayAction::JointVelocity {
                joint: expected_joint,
                velocity_rad_s: expected_velocity,
            },
            ReplayAction::JointVelocity {
                joint: actual_joint,
                velocity_rad_s: actual_velocity,
            },
        ) => {
            expected_joint == actual_joint
                && replay_float_matches(*expected_velocity, *actual_velocity)
        }
        (
            ReplayAction::JointEffort {
                joint: expected_joint,
                effort_nm: expected_effort,
            },
            ReplayAction::JointEffort {
                joint: actual_joint,
                effort_nm: actual_effort,
            },
        ) => {
            expected_joint == actual_joint && replay_float_matches(*expected_effort, *actual_effort)
        }
        (
            ReplayAction::JointPositions {
                samples: expected_samples,
            },
            ReplayAction::JointPositions {
                samples: actual_samples,
            },
        ) => {
            expected_samples.len() == actual_samples.len()
                && expected_samples
                    .iter()
                    .zip(actual_samples)
                    .all(|(expected, actual)| {
                        expected.joint == actual.joint
                            && replay_float_matches(expected.position_rad, actual.position_rad)
                    })
        }
        (
            ReplayAction::JointVelocities {
                samples: expected_samples,
            },
            ReplayAction::JointVelocities {
                samples: actual_samples,
            },
        ) => {
            expected_samples.len() == actual_samples.len()
                && expected_samples
                    .iter()
                    .zip(actual_samples)
                    .all(|(expected, actual)| {
                        expected.joint == actual.joint
                            && replay_float_matches(expected.velocity_rad_s, actual.velocity_rad_s)
                    })
        }
        (
            ReplayAction::RobotJointVelocities {
                samples: expected_samples,
            },
            ReplayAction::RobotJointVelocities {
                samples: actual_samples,
            },
        ) => {
            expected_samples.len() == actual_samples.len()
                && expected_samples
                    .iter()
                    .zip(actual_samples)
                    .all(|(expected, actual)| {
                        expected.robot_id == actual.robot_id
                            && expected.joint == actual.joint
                            && replay_float_matches(expected.velocity_rad_s, actual.velocity_rad_s)
                    })
        }
        _ => false,
    }
}

fn replay_joint_state_matches(
    expected: Option<&ReplayJointState>,
    actual: Option<&ReplayJointState>,
) -> bool {
    match (expected, actual) {
        (None, None) => true,
        (Some(expected), Some(actual)) => {
            expected.names == actual.names
                && expected.positions_rad.len() == actual.positions_rad.len()
                && expected.velocities_rad_s.len() == actual.velocities_rad_s.len()
                && expected
                    .positions_rad
                    .iter()
                    .zip(&actual.positions_rad)
                    .all(|(expected, actual)| replay_float_matches(*expected, *actual))
                && expected
                    .velocities_rad_s
                    .iter()
                    .zip(&actual.velocities_rad_s)
                    .all(|(expected, actual)| replay_float_matches(*expected, *actual))
        }
        _ => false,
    }
}

fn ensure_replay_reports(expected: &ReplayFinalReport, actual: &ReplayFinalReport) -> Result<()> {
    anyhow::ensure!(
        expected.steps == actual.steps
            && expected.seed == actual.seed
            && expected.robot_count == actual.robot_count
            && expected.differential_drive_count == actual.differential_drive_count
            && expected.physics_hash == actual.physics_hash
            && expected.contact_pairs_max == actual.contact_pairs_max
            && replay_float_matches(
                f64::from(expected.contact_impulse_max_ns),
                f64::from(actual.contact_impulse_max_ns)
            )
            && replay_optional_float_matches(expected.min_base_height_m, actual.min_base_height_m)
            && expected.failure == actual.failure,
        "replay final report mismatch: expected={expected:?} actual={actual:?}"
    );
    anyhow::ensure!(
        replay_float_matches(expected.sim_time_s, actual.sim_time_s),
        "replay final sim_time_s mismatch: expected={} actual={}",
        expected.sim_time_s,
        actual.sim_time_s
    );
    anyhow::ensure!(
        replay_translation_matches(
            expected.first_base_translation_m,
            actual.first_base_translation_m
        ),
        "replay final base translation mismatch: expected={:?} actual={:?}",
        expected.first_base_translation_m,
        actual.first_base_translation_m
    );
    Ok(())
}

fn replay_translation_matches(expected: Option<[f64; 3]>, actual: Option<[f64; 3]>) -> bool {
    match (expected, actual) {
        (None, None) => true,
        (Some(expected), Some(actual)) => expected
            .into_iter()
            .zip(actual)
            .all(|(expected, actual)| replay_float_matches(expected, actual)),
        _ => false,
    }
}

fn replay_optional_float_matches(expected: Option<f64>, actual: Option<f64>) -> bool {
    match (expected, actual) {
        (None, None) => true,
        (Some(expected), Some(actual)) => replay_float_matches(expected, actual),
        _ => false,
    }
}

fn replay_contact_matches(expected: Option<ReplayContact>, actual: Option<ReplayContact>) -> bool {
    match (expected, actual) {
        (None, None) => true,
        (Some(expected), Some(actual)) => {
            expected.pair_count == actual.pair_count
                && replay_float_matches(
                    f64::from(expected.total_impulse_ns),
                    f64::from(actual.total_impulse_ns),
                )
                && replay_float_matches(
                    f64::from(expected.max_impulse_ns),
                    f64::from(actual.max_impulse_ns),
                )
        }
        _ => false,
    }
}

fn replay_float_matches(expected: f64, actual: f64) -> bool {
    (expected - actual).abs() <= REPLAY_FLOAT_EPSILON * expected.abs().max(actual.abs()).max(1.0)
}

fn replay_final_report(report: &SimulationReport) -> ReplayFinalReport {
    ReplayFinalReport::new(
        report.steps,
        report.sim_time_s,
        report.seed,
        report.robot_count,
        report.differential_drive_count,
        report.physics_hash,
        report.first_base_translation_m,
        report.contact_pairs_max,
        report.contact_impulse_max_ns,
        report.min_base_height_m,
        report.failure,
    )
}

fn print_simulation_report(path: &Path, report: &SimulationReport, determinism_check: bool) {
    let base = report.first_base_translation_m.map_or_else(
        || "none".to_string(),
        |translation| {
            format!(
                "[{:+.3}, {:+.3}, {:+.3}] m",
                translation[0], translation[1], translation[2]
            )
        },
    );
    let min_height = report
        .min_base_height_m
        .map_or_else(|| "none".to_string(), |height| format!("{height:.3} m"));
    let failure = report.failure.map_or_else(
        || "ok".to_string(),
        |failure| format!("FAILED: {failure:?}"),
    );
    println!(
        "simulate scene={} steps={} time={:.6} s seed={} robots={} diff_drive={} base={} physics_hash={:#018x} contacts_pairs_max={} contact_impulse_max={:.4} Ns min_height={} failure={}{}",
        path.display(),
        report.steps,
        report.sim_time_s,
        report.seed,
        report.robot_count,
        report.differential_drive_count,
        base,
        report.physics_hash,
        report.contact_pairs_max,
        report.contact_impulse_max_ns,
        min_height,
        failure,
        if determinism_check { " (checking replay)" } else { "" },
    );
}

fn watch_command(path: &Path, interval_ms: u64) -> Result<()> {
    let mut reloader =
        AssetHotReloader::load(path).with_context(|| format!("watch {}", path.display()))?;
    print_reload_summary(reloader.bundle());

    loop {
        if reloader.poll()? {
            println!("--- reload ---");
            print_reload_summary(reloader.bundle());
        }
        thread::sleep(Duration::from_millis(interval_ms));
    }
}

fn print_reload_summary(bundle: &rne_assets::SceneAssetBundle) {
    println!(
        "scene={} seed={} robots={}",
        bundle.scene_path.display(),
        bundle.scene.world.seed,
        bundle.robots.len()
    );
    for (robot_path, robot) in &bundle.robots {
        println!("  robot {} ({:?})", robot_path.display(), robot.kind);
    }
}

#[cfg(test)]
mod tests {
    use super::{
        apply_replay_action, bounded_image_dimensions, build_live_snapshot,
        canonicalize_controller_robots, capture_controller_observation, classify_failure,
        controller_replay_action, encode_bounded_depth, ensure_replay_frames,
        interpolate_joint_positions, parse_control_line, plugin_command, replay_command,
        replay_input_digest, replay_scenario_command, run_manifest_command,
        runner_status_line_with_limit, simulate_scene, simulate_scene_with_action_schedule,
        stable_co_sim_hash, sumo_net_command, verify_physics_requirements, LiveSnapshotOptions,
        PluginCommand, PluginControllerConfig, ScenarioReplayArtifact,
    };
    use rne_assets::{
        load_scene_bundle, spawn_scene_bundle, RunPhysicsBackend, RunPhysicsCapability,
        RunSensorKind, RunSensorSubscription, SpawnSceneOptions,
    };
    use rne_core::control::{ControlCommand, RunControl, RunnerControl};
    use rne_core::SimTime;
    use rne_data::{DataBus, Frame, ImageDepth, ImageRgb8, InMemoryDataBus, StreamId};
    use rne_ecs::{Name, World};
    use rne_log::{ReplayAction, ReplayObservation, ReplaySensorStream};
    use rne_plugin::{
        ControllerNegotiation, ControllerPlugin, ControllerPluginError, ControllerResetContext,
        ControllerScheduler, VelocityServoController,
    };
    use rne_robot::{
        apply_actuator_commands, Actuator, ActuatorCommandBuffer, Joint, JointKind, Robot,
    };
    use std::collections::BTreeMap;
    use std::collections::VecDeque;
    use std::path::{Path, PathBuf};
    use std::sync::{Arc, Mutex};

    #[test]
    fn mesh_diff_drive_simulation_replays_identically() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mesh_diff_drive.rne.scene.toml");
        let first = simulate_scene(&scene, 30, 60.0, 6.0).expect("first simulation");
        let replay = simulate_scene(&scene, 30, 60.0, 6.0).expect("replay simulation");
        assert_eq!(first, replay);
        assert_eq!(first.steps, 30);
        assert_eq!(first.differential_drive_count, 1);
        assert!(first.first_base_translation_m.expect("base pose")[0] > 0.0);
        assert!(first.min_base_height_m.is_some());
        assert_eq!(first.failure, None);
    }

    #[test]
    fn example_run_manifest_executes() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/mesh_diff_drive.rne.run.toml");
        run_manifest_command(&manifest, false, None, false, None).expect("run manifest");
        let replay = manifest
            .parent()
            .expect("manifest parent")
            .join("../../target/runs/mesh_diff_drive.rne-replay");
        replay_command(&replay).expect("replay manifest output");
    }

    #[test]
    fn live_snapshot_contains_pose_orientation_and_streams() {
        let observation = ReplayObservation::new(Some([1.0, 0.25, 2.0]));
        let bus = InMemoryDataBus::new();
        let json = build_live_snapshot(
            &observation,
            Some(0.5),
            &bus,
            LiveSnapshotOptions::default(),
        );
        let value: serde_json::Value = serde_json::from_str(&json).expect("live snapshot JSON");
        assert_eq!(value["base"], serde_json::json!([1.0, 0.25, 2.0]));
        assert_eq!(value["base_yaw_rad"], serde_json::json!(0.5));
        assert!(value["sensors"]
            .as_array()
            .expect("sensor array")
            .is_empty());
    }

    #[test]
    fn live_snapshot_contains_bounded_camera_preview() {
        let stream_id = StreamId::new(5);
        let mut bus = InMemoryDataBus::new();
        let rgb = ImageRgb8::from_rgba8(2, 1, vec![1, 2, 3, 255, 5, 6, 7, 255]);
        let depth = ImageDepth::new(2, 1, vec![1.0, 2.0]);
        bus.publish(Frame::new(
            stream_id,
            rne_ecs::Entity::from_raw(1),
            7,
            SimTime::from_ticks(1),
            rgb,
        ));
        bus.publish(Frame::new(
            StreamId::new(5 + rne_sensor::CAMERA_DEPTH_STREAM_OFFSET),
            rne_ecs::Entity::from_raw(1),
            7,
            SimTime::from_ticks(1),
            depth,
        ));
        let observation =
            ReplayObservation::new(None).with_sensor_streams(vec![ReplaySensorStream {
                stream_id: 5,
                kind: "camera".to_string(),
                frame_count: 1,
                last_sequence: 7,
                payload_hash: 99,
            }]);

        let json = build_live_snapshot(&observation, None, &bus, LiveSnapshotOptions::default());
        let value: serde_json::Value = serde_json::from_str(&json).expect("snapshot JSON");
        let camera = &value["sensors"][0]["camera"];
        assert_eq!(camera["source_width"], serde_json::json!(2));
        assert_eq!(camera["source_height"], serde_json::json!(1));
        assert_eq!(camera["width"], serde_json::json!(2));
        assert_eq!(camera["height"], serde_json::json!(1));
        assert_eq!(camera["depth_center_m"], serde_json::json!(2.0));
        assert_eq!(
            camera["depth_hash"],
            serde_json::json!(ImageDepth::new(2, 1, vec![1.0, 2.0]).hash_depth())
        );
        assert_eq!(
            base64::decode(camera["rgba8_base64"].as_str().expect("base64"))
                .expect("decode preview"),
            vec![1, 2, 3, 255, 5, 6, 7, 255]
        );

        let full_json = build_live_snapshot(
            &observation,
            None,
            &bus,
            LiveSnapshotOptions::full_resolution(),
        );
        let full_value: serde_json::Value =
            serde_json::from_str(&full_json).expect("full snapshot JSON");
        let full_camera = &full_value["sensors"][0]["camera"];
        assert_eq!(full_camera["depth_source_width"], serde_json::json!(2));
        assert_eq!(full_camera["depth_source_height"], serde_json::json!(1));
        assert_eq!(full_camera["depth_width"], serde_json::json!(2));
        assert_eq!(full_camera["depth_height"], serde_json::json!(1));
        assert_eq!(
            base64::decode(
                full_camera["depth_f32_le_base64"]
                    .as_str()
                    .expect("depth base64")
            )
            .expect("decode depth"),
            vec![0, 0, 128, 63, 0, 0, 0, 64]
        );
    }

    #[test]
    fn full_resolution_camera_transport_has_absolute_rgbd_caps() {
        let options = LiveSnapshotOptions::full_resolution();
        assert_eq!(
            bounded_image_dimensions(3840, 2160, options),
            Some((1920, 1080))
        );
        assert_eq!(
            bounded_image_dimensions(2160, 3840, options),
            Some((608, 1080))
        );

        let depth = ImageDepth::new(4, 2, vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0]);
        let bounded_options = LiveSnapshotOptions {
            camera_max_width: Some(2),
            camera_max_height: Some(1),
            include_full_depth: true,
        };
        let (width, height, encoded) =
            encode_bounded_depth(&depth, bounded_options).expect("bounded depth payload");
        assert_eq!((width, height), (2, 1));
        assert_eq!(
            base64::decode(encoded).expect("decode bounded depth"),
            vec![0, 0, 128, 63, 0, 0, 64, 64]
        );
    }

    #[test]
    fn runner_status_replaces_snapshots_above_the_transport_limit() {
        let within_limit = runner_status_line_with_limit(7, 0.5, "paused", b"{}", 2);
        assert!(within_limit.ends_with("snapshot={}\n"));

        let over_limit = runner_status_line_with_limit(7, 0.5, "paused", b"{}", 1);
        assert!(over_limit.contains("\"error\":\"snapshot_limit_exceeded\""));
        assert!(over_limit.contains("\"snapshot_bytes\":2"));
        assert!(over_limit.contains("\"limit_bytes\":1"));
    }

    #[test]
    fn lidar_payload_run_manifest_records_and_replays() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/mesh_diff_drive_lidar_payload.rne.run.toml");
        run_manifest_command(&manifest, false, None, false, None)
            .expect("run lidar payload manifest");
        let replay_path = manifest
            .parent()
            .expect("manifest parent")
            .join("../../target/runs/mesh_diff_drive_lidar_payload.rne-replay");
        let artifact = rne_log::ReplayArtifact::read_json(&replay_path).expect("read artifact");
        assert!(!artifact.sensor_payload_streams.is_empty());
        let last_frame = artifact.frames.last().expect("last frame");
        let payloads = &last_frame.observation.sensor_payloads;
        assert!(
            !payloads.is_empty(),
            "subscribed lidar payloads must be recorded"
        );
        assert_eq!(payloads[0].kind, "lidar");
        replay_command(&replay_path).expect("replay lidar payload artifact");
    }

    #[test]
    fn scenario_run_manifest_executes_deterministically() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/scenario_speed.rne.run.toml");
        run_manifest_command(&manifest, false, None, false, None).expect("run scenario manifest");
        let replay = manifest
            .parent()
            .expect("manifest parent")
            .join("../../target/runs/scenario_speed.rne-replay");
        let artifact = ScenarioReplayArtifact::read_json(&replay).expect("read scenario artifact");
        assert!(artifact.replayable);
        assert_eq!(artifact.executed_steps, 300);
        assert_eq!(artifact.engine_version, env!("CARGO_PKG_VERSION"));
        assert_eq!(
            artifact.scenario_digest,
            replay_input_digest(Path::new(&artifact.scenario_path), "OpenSCENARIO")
                .expect("digest scenario input")
        );
        assert_eq!(
            artifact.network_digest,
            replay_input_digest(Path::new(&artifact.network_path), "traffic network")
                .expect("digest network input")
        );
        replay_command(&replay).expect("replay scenario artifact");

        let mut mismatched = artifact;
        mismatched.scenario_digest ^= 1;
        let error = replay_scenario_command(&replay, &mismatched)
            .expect_err("changed scenario digest must be rejected");
        assert!(error.to_string().contains("OpenSCENARIO input changed"));
    }

    #[test]
    fn trajectory_run_manifest_negotiates_physics_and_replays() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/mm_minimal_joint_trajectory.rne.run.toml");
        run_manifest_command(&manifest, false, None, false, None)
            .expect("run trajectory manifest with physics checks");
        let replay = manifest
            .parent()
            .expect("manifest parent")
            .join("../../target/runs/mm_minimal_joint_trajectory.rne-replay");
        replay_command(&replay).expect("replay trajectory artifact");
    }

    #[test]
    fn analytic_backend_run_manifest_executes() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/cart_analytic.rne.run.toml");
        run_manifest_command(&manifest, false, None, false, None)
            .expect("run analytic backend manifest");
    }

    #[test]
    fn plugin_controller_run_manifest_executes_deterministically() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/mm_minimal_velocity_servo.rne.run.toml");
        run_manifest_command(&manifest, false, None, false, None)
            .expect("run plugin controller manifest");
    }

    #[test]
    fn missing_physics_capability_is_rejected() {
        let error = verify_physics_requirements(
            RunPhysicsBackend::Rapier,
            &[RunPhysicsCapability::GpuRigidBody],
        )
        .expect_err("gpu_rigid_body must be rejected");
        assert!(error.to_string().contains("lacks required capabilities"));
        verify_physics_requirements(
            RunPhysicsBackend::Rapier,
            &[RunPhysicsCapability::Articulation],
        )
        .expect("articulation is supported");
    }

    #[test]
    fn named_joint_velocity_records_sensor_streams_and_replays() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let action = ReplayAction::joint_velocity("shoulder_joint", 0.5);
        let first = simulate_scene_with_action_schedule(
            &scene,
            6,
            60.0,
            action,
            None,
            None,
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("joint simulation");

        let first_frame = first.frames.first().expect("first frame");
        assert_eq!(
            first_frame.action,
            ReplayAction::joint_velocity("shoulder_joint", 0.5)
        );
        let joint_state = first_frame
            .observation
            .joint_state
            .as_ref()
            .expect("joint state");
        assert!(joint_state
            .names
            .iter()
            .any(|name| name == "shoulder_joint"));
        assert_eq!(first_frame.observation.sensor_streams.len(), 1);
        assert_eq!(first_frame.observation.sensor_streams[0].kind, "camera");
        assert_eq!(first_frame.observation.sensor_streams[0].frame_count, 1);

        let replay = simulate_scene_with_action_schedule(
            &scene,
            6,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            Some(&first.frames),
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("joint replay");
        assert_eq!(first, replay);
    }

    #[test]
    fn camera_payload_subscription_records_full_payloads() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let subscription = RunSensorSubscription {
            name: None,
            kind: Some(RunSensorKind::Camera),
        };
        let run = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            None,
            &[subscription],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("camera payload simulation");

        assert_eq!(run.sensor_payload_streams.len(), 1);
        let last_frame = run.frames.last().expect("last frame");
        let stream_summary = last_frame
            .observation
            .sensor_streams
            .first()
            .expect("stream summary");
        assert_eq!(stream_summary.kind, "camera");
        let payloads = &last_frame.observation.sensor_payloads;
        assert_eq!(payloads.len(), 1, "camera payloads must be captured");
        assert_eq!(payloads[0].stream_id, run.sensor_payload_streams[0]);
        assert_eq!(payloads[0].kind, "camera");
        assert!(payloads[0].sequence >= 1);
        let camera_data = match &payloads[0].data {
            rne_log::ReplaySensorPayloadData::Camera { rgb, depth } => (rgb, depth),
            other => panic!("expected camera payload, got {other:?}"),
        };
        assert!(camera_data.0.width > 0 && camera_data.0.height > 0);
        assert_eq!(camera_data.0.width, camera_data.1.width);
        assert_eq!(camera_data.0.height, camera_data.1.height);

        let replay = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            Some(&run.frames),
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("camera replay");
        assert_eq!(replay.report, run.report);
        ensure_replay_frames(&run.frames, &replay.frames).expect("camera replay frames match");
    }

    #[test]
    fn sensor_subscription_without_match_is_rejected() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let subscription = RunSensorSubscription {
            name: Some("nonexistent_sensor".to_string()),
            kind: None,
        };
        let error = simulate_scene_with_action_schedule(
            &scene,
            6,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            None,
            &[subscription],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect_err("unknown sensor must be rejected");
        assert!(error.to_string().contains("matched no sensor"));
    }

    #[test]
    fn contact_annotations_are_captured_and_replay() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/cart_minimal.rne.scene.toml");
        let run = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("contact simulation");

        let settled_frame = run
            .frames
            .last()
            .expect("last frame")
            .observation
            .contact
            .expect("contact summary");
        assert!(
            settled_frame.pair_count >= 1,
            "settled cart wheels must rest on the ground"
        );
        assert!(settled_frame.total_impulse_ns > 0.0);
        assert!(settled_frame.max_impulse_ns > 0.0);
        assert!(run.report.contact_pairs_max >= 1);
        assert!(run.report.contact_impulse_max_ns > 0.0);
        assert!(run.report.min_base_height_m.is_none());
        assert_eq!(run.report.failure, None);

        let replay = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            Some(&run.frames),
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("contact replay");
        assert_eq!(replay.report, run.report);
        ensure_replay_frames(&run.frames, &replay.frames).expect("contact frames match");
    }

    #[test]
    fn joint_trajectory_interpolates_and_replays() {
        use rne_assets::{RunJointTrajectory, RunTrajectoryWaypoint};

        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let trajectories = vec![RunJointTrajectory {
            joint: "shoulder_joint".to_string(),
            waypoints: vec![
                RunTrajectoryWaypoint {
                    t_s: 0.0,
                    position_rad: 0.0,
                },
                RunTrajectoryWaypoint {
                    t_s: 0.5,
                    position_rad: 1.0,
                },
                RunTrajectoryWaypoint {
                    t_s: 1.0,
                    position_rad: 0.0,
                },
            ],
        }];
        let run = simulate_scene_with_action_schedule(
            &scene,
            60,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &trajectories,
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("trajectory simulation");

        assert_eq!(
            run.frames[0].action,
            interpolate_joint_positions(&trajectories, 0.0)
        );
        let mid = &run.frames[30].action;
        let ReplayAction::JointPositions { samples } = mid else {
            panic!("expected joint positions action, got {mid:?}");
        };
        assert_eq!(samples.len(), 1);
        assert_eq!(samples[0].joint, "shoulder_joint");
        assert!(
            (samples[0].position_rad - 1.0).abs() < 1e-6,
            "mid trajectory should approach the peak (got {})",
            samples[0].position_rad
        );

        let replay = simulate_scene_with_action_schedule(
            &scene,
            60,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            Some(&run.frames),
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("trajectory replay");
        assert_eq!(replay.report, run.report);
        ensure_replay_frames(&run.frames, &replay.frames).expect("trajectory frames match");
    }

    #[test]
    fn failure_classification_flags_a_fall() {
        use rne_log::ReplayFailureKind;
        assert_eq!(classify_failure(Some(0.25), Some(0.24)), None);
        assert_eq!(
            classify_failure(Some(0.25), Some(0.12)),
            Some(ReplayFailureKind::Fell)
        );
        assert_eq!(classify_failure(Some(0.25), None), None);
        assert_eq!(classify_failure(None, Some(0.1)), None);
        assert_eq!(classify_failure(Some(0.0), Some(0.0)), None);
    }

    #[test]
    fn stable_co_sim_hash_is_deterministic_and_sensitive() {
        let states = vec![
            vec![("v0".to_string(), [10.0, 0.0, 20.0])],
            vec![("v0".to_string(), [20.0, 0.0, 30.0])],
        ];
        assert_eq!(stable_co_sim_hash(&states), stable_co_sim_hash(&states));
        let moved = vec![
            vec![("v0".to_string(), [11.0, 0.0, 20.0])],
            vec![("v0".to_string(), [20.0, 0.0, 30.0])],
        ];
        assert_ne!(
            stable_co_sim_hash(&states),
            stable_co_sim_hash(&moved),
            "a different position must change the hash"
        );
    }

    #[test]
    fn sumo_net_command_imports_a_cross_intersection() {
        let fixture = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/networks/minimal_cross.net.xml");
        let out = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../target/runs/sumo_minimal_cross.rne.traffic.json");
        sumo_net_command(&fixture, &out, "sumo:minimal_cross").expect("import SUMO network");

        let asset = rne_traffic::load_traffic_asset(&out).expect("load imported asset");
        assert_eq!(asset.network.lanes.len(), 8);
        assert!(
            !asset.network.junctions.is_empty(),
            "lane endpoints must cluster into junctions"
        );
        assert!(
            asset.network.connections.len() >= 4,
            "approaches must connect through the junction"
        );
    }

    #[test]
    fn plugin_new_scaffolds_a_plugin_crate() {
        let parent =
            PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target/runs/plugin-new-cli-test");
        let _ = std::fs::remove_dir_all(&parent);
        plugin_command(PluginCommand::New {
            name: "cli_plugin".to_string(),
            dir: parent.clone(),
        })
        .expect("scaffold plugin");

        let crate_dir = parent.join("cli_plugin");
        assert!(crate_dir.join("Cargo.toml").exists());
        assert!(crate_dir.join("src/lib.rs").exists());
        assert!(crate_dir.join("rne-plugin.json").exists());
        let lib = std::fs::read_to_string(crate_dir.join("src/lib.rs")).expect("read lib.rs");
        assert!(lib.contains("rne_plugin_abi_version"));
        assert!(lib.contains("rne_plugin_name"));
        let _ = std::fs::remove_dir_all(&parent);
    }

    #[test]
    fn sumo_cross_scenario_drives_through_the_imported_network() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let xosc = root.join("assets/scenarios/sumo_cross.xosc");
        let document = rne_openscenario::parse_openscenario_xml_file(&xosc)
            .expect("parse SUMO cross scenario");
        let network_path = root.join("assets/networks/minimal_cross.net.xml");
        let network_id = rne_traffic::TrafficId::new("sumo:test").expect("network id");
        let asset = rne_sumo::import_sumo_net_file(&network_id, &network_path)
            .expect("import SUMO cross network");
        let result = rne_openscenario::execute_scenario(
            &document,
            &asset.network,
            &rne_openscenario::ScenarioRunOptions {
                steps: 600,
                hz: 60.0,
            },
        )
        .expect("execute SUMO cross scenario");
        assert!(
            result.route_length_m > 100.0,
            "the route must span the eastbound approach and beyond, got {}",
            result.route_length_m
        );
        assert_eq!(result.final_positions_m.len(), 1);
        assert_eq!(result.collisions, 0);
    }

    #[test]
    fn sumo_cross_run_manifest_executes_deterministically() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/sumo_cross.rne.run.toml");
        run_manifest_command(&manifest, false, None, false, None).expect("run SUMO cross manifest");
    }

    #[test]
    fn signalized_sumo_network_holds_the_actor_at_red() {
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
        let xosc = root.join("assets/scenarios/sumo_cross.xosc");
        let document = rne_openscenario::parse_openscenario_xml_file(&xosc)
            .expect("parse SUMO cross scenario");
        let network_path = root.join("assets/networks/signalized_cross.net.xml");
        let network_id = rne_traffic::TrafficId::new("sumo:signalized").expect("network id");
        let asset = rne_sumo::import_sumo_net_file(&network_id, &network_path)
            .expect("import signalized SUMO network");
        assert_eq!(asset.network.signals.len(), 1);

        let result = rne_openscenario::execute_scenario(
            &document,
            &asset.network,
            &rne_openscenario::ScenarioRunOptions {
                steps: 900,
                hz: 60.0,
            },
        )
        .expect("execute signalized SUMO scenario");
        assert!(result.route_length_m > 100.0);
        assert_eq!(result.final_positions_m.len(), 1);
        assert_eq!(result.collisions, 0);
        assert_eq!(result.signal_violations, 0);
        assert!(
            result.average_speed_m_s < 0.5,
            "the eastbound movement is red for 20 s, so the actor must be held at the stop line"
        );
        let final_position = result.final_positions_m[0];
        assert!(
            final_position[0] < 260.0,
            "the actor must have driven down the eastbound approach before stopping, got {final_position:?}"
        );
        assert!(
            final_position[0] > 190.0,
            "the actor must not cross the red stop line, got {final_position:?}"
        );
    }

    /// A queue-driven transport for deterministic runner-control tests.
    struct ScriptedControl {
        commands: VecDeque<ControlCommand>,
    }

    impl RunnerControl for ScriptedControl {
        fn try_poll(&mut self) -> Option<ControlCommand> {
            self.commands.pop_front()
        }

        fn wait_command(&mut self) -> ControlCommand {
            self.commands.pop_front().unwrap_or(ControlCommand::Quit)
        }
    }

    fn scripted(
        commands: Vec<ControlCommand>,
        scene: &Path,
        steps: u64,
    ) -> super::Result<super::SimulationRun> {
        let mut transport = ScriptedControl {
            commands: commands.into(),
        };
        let mut control = RunControl::new(&mut transport);
        simulate_scene_with_action_schedule(
            scene,
            steps,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            Some(&mut control),
        )
    }

    /// Locates the example controller-plugin shared library, building it on
    /// demand. A plain workspace build places it under `target/debug/`; cargo
    /// nextest does not emit cdylib artifacts for workspace members, so the
    /// helper falls back to building the crate itself.
    fn find_example_plugin_library() -> PathBuf {
        let target = std::env::var_os("CARGO_TARGET_DIR")
            .map(PathBuf::from)
            .unwrap_or_else(|| PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../../target"));
        let debug = target.join("debug");
        let file_name = if cfg!(target_os = "windows") {
            "rne_plugin_example_velocity_servo.dll"
        } else if cfg!(target_os = "macos") {
            "librne_plugin_example_velocity_servo.dylib"
        } else {
            "librne_plugin_example_velocity_servo.so"
        };
        let prefix = if cfg!(target_os = "windows") {
            "rne_plugin_example_velocity_servo-"
        } else {
            "librne_plugin_example_velocity_servo-"
        };
        let extension = Path::new(file_name)
            .extension()
            .map(|extension| format!(".{}", extension.to_string_lossy()))
            .expect("library extension");
        let find_in_deps = || -> Option<PathBuf> {
            let entries = std::fs::read_dir(debug.join("deps")).ok()?;
            for entry in entries.flatten() {
                let name = entry.file_name().to_string_lossy().into_owned();
                if name.starts_with(prefix) && name.ends_with(extension.as_str()) {
                    return Some(entry.path());
                }
            }
            None
        };
        let direct = debug.join(file_name);
        if direct.exists() {
            return direct;
        }
        if let Some(found) = find_in_deps() {
            return found;
        }
        let status = std::process::Command::new("cargo")
            .arg("build")
            .arg("-p")
            .arg("rne_plugin_example_velocity_servo")
            .current_dir(PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../.."))
            .status()
            .expect("run cargo to build the example plugin");
        assert!(
            status.success(),
            "cargo build -p rne_plugin_example_velocity_servo failed"
        );
        if direct.exists() {
            return direct;
        }
        if let Some(found) = find_in_deps() {
            return found;
        }
        panic!(
            "example plugin library still missing after a build; target dir: {}",
            debug.display()
        );
    }

    #[test]
    fn loaded_plugin_library_drives_the_same_policy_as_the_built_in() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let params = PluginControllerConfig {
            joint: "shoulder_joint".to_string(),
            target_rad: 1.0,
            gain: 2.0,
            max_velocity_rad_s: 5.0,
            library: Some(find_example_plugin_library()),
            plugin_paths: Vec::new(),
        };
        let loaded = params.build().expect("load example plugin");
        let built_in = PluginControllerConfig {
            library: None,
            ..params
        }
        .build()
        .expect("build-in plugin");

        let run_loaded = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            Some(loaded),
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("loaded plugin run");
        let run_built_in = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            Some(built_in),
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("built-in run");
        assert_eq!(
            run_loaded.report, run_built_in.report,
            "loaded and built-in plugins must drive identical runs"
        );
        ensure_replay_frames(&run_built_in.frames, &run_loaded.frames)
            .expect("loaded and built-in frames match");
    }

    #[test]
    fn discovered_plugin_drives_the_same_policy_as_the_built_in() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let search_path = find_example_plugin_library()
            .parent()
            .expect("library parent directory")
            .to_path_buf();
        let discovered = PluginControllerConfig {
            joint: "shoulder_joint".to_string(),
            target_rad: 1.0,
            gain: 2.0,
            max_velocity_rad_s: 5.0,
            library: None,
            plugin_paths: vec![search_path],
        }
        .build()
        .expect("discover velocity_servo");
        let built_in =
            VelocityServoController::new("velocity_servo", "shoulder_joint", 1.0, 2.0, 5.0)
                .expect("built-in controller");

        let run_discovered = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            Some(discovered),
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("discovered plugin run");
        let run_built_in = simulate_scene_with_action_schedule(
            &scene,
            30,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            Some(Box::new(built_in)),
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("built-in run");
        assert_eq!(
            run_discovered.report, run_built_in.report,
            "discovered and built-in plugins must drive identical runs"
        );
        ensure_replay_frames(&run_built_in.frames, &run_discovered.frames)
            .expect("discovered and built-in frames match");
    }

    #[derive(Debug)]
    struct LifecycleRecordingController {
        events: Arc<Mutex<Vec<String>>>,
    }

    impl LifecycleRecordingController {
        fn record(&self, event: impl Into<String>) {
            self.events.lock().expect("event lock").push(event.into());
        }
    }

    impl ControllerPlugin for LifecycleRecordingController {
        fn name(&self) -> &str {
            "lifecycle_recorder"
        }

        fn on_configure(
            &mut self,
            _negotiation: &ControllerNegotiation,
        ) -> Result<(), ControllerPluginError> {
            self.record("configure");
            Ok(())
        }

        fn on_reset(
            &mut self,
            context: ControllerResetContext,
        ) -> Result<(), ControllerPluginError> {
            self.record(format!("reset:{}", context.episode));
            Ok(())
        }

        fn on_shutdown(&mut self) -> Result<(), ControllerPluginError> {
            self.record("shutdown");
            Ok(())
        }

        fn joint_velocity_commands(
            &self,
            joint_names: &[&str],
            _positions_rad: &[f64],
        ) -> Vec<(String, f64)> {
            self.record("step");
            joint_names
                .iter()
                .find(|name| **name == "shoulder_joint")
                .map(|name| vec![(name.to_string(), 0.25)])
                .unwrap_or_default()
        }
    }

    #[test]
    fn runner_owns_controller_lifecycle_across_episode_reset() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let events = Arc::new(Mutex::new(Vec::new()));
        let controller = LifecycleRecordingController {
            events: Arc::clone(&events),
        };
        let mut transport = ScriptedControl {
            commands: vec![
                ControlCommand::Step { frames: 1 },
                ControlCommand::Reset,
                ControlCommand::Step { frames: 1 },
                ControlCommand::Quit,
            ]
            .into(),
        };
        let mut control = RunControl::new(&mut transport);
        let run = simulate_scene_with_action_schedule(
            &scene,
            20,
            60.0,
            ReplayAction::differential_drive(0.0),
            None,
            None,
            &[],
            Some(Box::new(controller)),
            &[],
            RunPhysicsBackend::Rapier,
            Some(&mut control),
        )
        .expect("controller lifecycle run");

        assert_eq!(run.frames.len(), 1, "only the final episode is reported");
        assert_eq!(
            *events.lock().expect("event lock"),
            [
                "configure",
                "reset:0",
                "step",
                "reset:1",
                "step",
                "shutdown"
            ]
        );
    }

    fn dual_robot_controller_result(
        reverse_spawn_order: bool,
    ) -> (String, BTreeMap<(String, String), f64>) {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_dual_controller.rne.scene.toml");
        let mut bundle = load_scene_bundle(&scene).expect("load dual-robot scene");
        if reverse_spawn_order {
            bundle.scene.robots.reverse();
            bundle.robots.reverse();
        }
        let mut world = World::new();
        let spawned = spawn_scene_bundle(&mut world, &bundle, None, SpawnSceneOptions::default())
            .expect("spawn dual-robot scene");
        let controller_robots = spawned
            .robots
            .iter()
            .map(|(robot_id, robot)| (robot_id.clone(), robot.robot))
            .collect::<Vec<_>>();
        let controller_robots = canonicalize_controller_robots(&mut world, controller_robots)
            .expect("canonical controller robots");

        let robot_b = controller_robots
            .iter()
            .find(|(robot_id, _)| robot_id == "mm_minimal_b")
            .expect("robot b")
            .1;
        let robot_b_joints = world
            .iter_entities()
            .filter_map(|entity_ref| {
                let entity = entity_ref.id();
                let joint = world.get::<Joint>(entity)?;
                (joint.robot == robot_b && joint.kind != JointKind::Fixed).then_some(entity)
            })
            .collect::<Vec<_>>();
        for joint in robot_b_joints {
            world.get_mut::<Joint>(joint).expect("joint").position = 0.5;
        }

        let observation =
            capture_controller_observation(&world, &controller_robots, 0, SimTime::ZERO)
                .expect("capture controller observation");
        let mut scheduler = ControllerScheduler::new();
        scheduler
            .register(
                "servo",
                Box::new(
                    VelocityServoController::new("velocity_servo", "shoulder_joint", 1.0, 2.0, 5.0)
                        .expect("servo"),
                ),
                controller_robots
                    .iter()
                    .map(|(robot_id, _)| robot_id.clone()),
            )
            .expect("register scheduler");
        scheduler.configure().expect("configure scheduler");
        scheduler
            .activate(ControllerResetContext {
                episode: 0,
                seed: 44,
                step: 0,
                sim_time_ticks: 0,
            })
            .expect("activate scheduler");
        let action = scheduler.step(&observation).expect("step scheduler");
        let action_json = action.to_json_pretty().expect("action JSON");
        let replay_action = controller_replay_action(action).expect("controller replay action");
        let mut command_buffer = ActuatorCommandBuffer::new();
        apply_replay_action(
            &world,
            &mut command_buffer,
            &[],
            &replay_action,
            SimTime::ZERO,
        )
        .expect("apply robot-scoped commands");
        apply_actuator_commands(&mut world, &mut command_buffer);
        let named_targets = world
            .iter_entities()
            .filter_map(|entity_ref| {
                let entity = entity_ref.id();
                let name = world.get::<Name>(entity)?;
                let joint = world.get::<Joint>(entity)?;
                let actuator = world.get::<Actuator>(entity)?;
                let robot = world.get::<Robot>(joint.robot)?;
                Some((
                    (robot.model_name.clone(), name.0.clone()),
                    actuator.target.velocity_rad_s,
                ))
            })
            .collect::<BTreeMap<_, _>>();
        scheduler.shutdown().expect("shutdown scheduler");
        (action_json, named_targets)
    }

    #[test]
    fn dual_robot_controller_is_independent_of_ecs_spawn_order() {
        let forward = dual_robot_controller_result(false);
        let reversed = dual_robot_controller_result(true);
        assert_eq!(forward, reversed);
        assert_eq!(
            forward
                .1
                .get(&("mm_minimal_a".to_string(), "shoulder_joint".to_string())),
            Some(&2.0)
        );
        assert_eq!(
            forward
                .1
                .get(&("mm_minimal_b".to_string(), "shoulder_joint".to_string())),
            Some(&1.0)
        );
    }

    #[test]
    fn parse_control_line_accepts_the_documented_vocabulary() {
        use rne_core::control::ControlCommand;
        assert_eq!(parse_control_line("pause"), Some(ControlCommand::Pause));
        assert_eq!(parse_control_line("resume"), Some(ControlCommand::Resume));
        assert_eq!(
            parse_control_line("step 5"),
            Some(ControlCommand::Step { frames: 5 })
        );
        assert_eq!(
            parse_control_line("step 5 # comment"),
            Some(ControlCommand::Step { frames: 5 })
        );
        assert_eq!(parse_control_line("reset"), Some(ControlCommand::Reset));
        assert_eq!(parse_control_line("quit"), Some(ControlCommand::Quit));
        assert_eq!(parse_control_line("exit"), Some(ControlCommand::Quit));
        assert_eq!(parse_control_line(""), None);
        assert_eq!(parse_control_line("  \t\n"), None);
        assert_eq!(parse_control_line("step"), None);
        assert_eq!(parse_control_line("step x"), None);
        assert_eq!(parse_control_line("unknown"), None);
    }

    #[test]
    fn step_command_pauses_after_the_requested_frames() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let run = scripted(
            vec![ControlCommand::Step { frames: 2 }, ControlCommand::Quit],
            &scene,
            200,
        )
        .expect("scripted step run");
        assert_eq!(run.report.steps, 2, "exactly the stepped frames are run");
        assert_eq!(run.frames.len(), 2);
    }

    #[test]
    fn quit_ends_the_episode_early() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mm_minimal.rne.scene.toml");
        let run = scripted(
            vec![
                ControlCommand::Step { frames: 3 },
                ControlCommand::Reset,
                ControlCommand::Step { frames: 1 },
                ControlCommand::Quit,
            ],
            &scene,
            200,
        )
        .expect("scripted reset run");
        assert_eq!(run.report.steps, 1, "the final episode runs one frame");
        assert_eq!(run.frames.len(), 1);
    }

    #[test]
    fn reset_restarts_the_episode_from_initial_conditions() {
        let scene = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/scenes/mesh_diff_drive.rne.scene.toml");
        let mut transport = ScriptedControl {
            commands: vec![
                ControlCommand::Step { frames: 10 },
                ControlCommand::Reset,
                ControlCommand::Step { frames: 10 },
                ControlCommand::Quit,
            ]
            .into(),
        };
        let mut control = RunControl::new(&mut transport);
        let run = simulate_scene_with_action_schedule(
            &scene,
            200,
            60.0,
            ReplayAction::differential_drive(6.0),
            None,
            None,
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            Some(&mut control),
        )
        .expect("scripted reset run");
        assert_eq!(
            run.frames.len(),
            10,
            "only the post-reset episode is reported"
        );

        let full = simulate_scene_with_action_schedule(
            &scene,
            10,
            60.0,
            ReplayAction::differential_drive(6.0),
            None,
            None,
            &[],
            None,
            &[],
            RunPhysicsBackend::Rapier,
            None,
        )
        .expect("baseline 10-step run");
        assert_eq!(
            run.report, full.report,
            "reset reproduces the initial episode"
        );
        ensure_replay_frames(&full.frames, &run.frames).expect("reset frames match the baseline");
    }
}

//! Command-line tools for RNE scene and robot assets.

use anyhow::{Context as _, Result};
use clap::{Parser, Subcommand};
use rne_assets::{
    inspect_asset, load_run_manifest, load_scene_bundle, smoke_spawn_scene, spawn_scene_bundle,
    validate_asset, AssetHotReloader, RunControllerKind, RunJointTrajectory, RunPhysicsBackend,
    RunPhysicsCapability, RunScenario, RunSensorKind, RunSensorSubscription, RunTrajectoryWaypoint,
    SpawnSceneOptions, ValidatedAsset,
};
use rne_core::{SimDuration, SimTime};
use rne_data::{
    DataBus, ImageDepth, ImageRgb8, ImuSample, InMemoryDataBus, PointCloud, WheelEncoderSample,
};
use rne_ecs::{Name, World};
use rne_log::{
    ReplayAction, ReplayArtifact, ReplayClock, ReplayContact,
    ReplayControllerKind as ArtifactControllerKind, ReplayFailureKind, ReplayFinalReport,
    ReplayFrame, ReplayJointPosition, ReplayJointState, ReplayObservation, ReplaySensorPayload,
    ReplaySensorPayloadData, ReplaySensorStream,
};
use rne_math::Hertz;
use rne_openscenario::{execute_scenario, parse_openscenario_xml_file, ScenarioRunOptions};
use rne_physics::{
    hash_physics_state, require_capabilities, ContactEvent, PhysicsBackend, PhysicsCapability,
    PhysicsError, PhysicsWorldDesc, PhysicsWorldId,
};
use rne_physics_analytic::AnalyticBackend;
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, sync_all_joint_motors_from_actuators,
    Actuator, ActuatorCommand, ActuatorCommandBuffer, DiffDriveComponent, Joint, JointKind,
};
use rne_sensor::{
    sample_sensors, Sensor, SensorKind, SensorSampleContext, SensorState,
    CAMERA_DEPTH_STREAM_OFFSET,
};
use rne_traffic::load_traffic_asset;
use rne_world::{Transform3, WorldEntity};
use serde::Serialize;
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

const REPLAY_FLOAT_EPSILON: f64 = 1.0e-12;

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
    },
    /// Replay a recorded `.rne-replay` artifact and verify every frame.
    Replay {
        /// Replay artifact path.
        path: PathBuf,
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
        Commands::Run { path } => run_manifest_command(&path),
        Commands::Replay { path } => replay_command(&path),
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
    seed_override: Option<u64>,
    determinism_check: bool,
    replay_out: Option<&'a Path>,
    replay_controller: ArtifactControllerKind,
    sensor_subscriptions: Vec<RunSensorSubscription>,
    physics_backend: RunPhysicsBackend,
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
            physics_backend: RunPhysicsBackend::Rapier,
        },
    )
}

fn run_manifest_command(path: &Path) -> Result<()> {
    let manifest =
        load_run_manifest(path).with_context(|| format!("load run manifest {}", path.display()))?;
    if let Some(scenario) = &manifest.scenario {
        return run_scenario_manifest(path, &manifest, scenario);
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
    };
    let trajectories = if manifest.controller.kind == RunControllerKind::JointTrajectory {
        manifest.controller.joint_trajectories.clone()
    } else {
        Vec::new()
    };
    let replay_out = manifest
        .output
        .replay_path
        .as_deref()
        .map(|output_path| manifest.resolve_output_path(path, output_path));
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
    run_simulation(
        &scene_path,
        SimulationOptions {
            steps: manifest.clock.steps,
            hz: manifest.clock.hz,
            action,
            trajectories,
            seed_override: manifest.seed,
            determinism_check: manifest.output.determinism_check,
            replay_out: replay_out.as_deref(),
            replay_controller,
            sensor_subscriptions: manifest.sensors.clone(),
            physics_backend: manifest.physics.backend,
        },
    )
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
) -> Result<()> {
    let xosc_path = scenario.resolve_xosc_path(manifest_path);
    println!(
        "scenario manifest={} xosc={} steps={} hz={}",
        manifest_path.display(),
        xosc_path.display(),
        manifest.clock.steps,
        manifest.clock.hz
    );
    let document = parse_openscenario_xml_file(&xosc_path)
        .with_context(|| format!("parse OpenSCENARIO {}", xosc_path.display()))?;
    let network_path = {
        let logic_file = Path::new(&document.road_network_logic_file);
        if logic_file.is_absolute() {
            logic_file.to_path_buf()
        } else {
            xosc_path
                .parent()
                .unwrap_or_else(|| Path::new("."))
                .join(logic_file)
        }
    };
    let asset = load_traffic_asset(&network_path).map_err(|error| {
        anyhow::anyhow!("load traffic network {}: {error}", network_path.display())
    })?;
    let options = ScenarioRunOptions {
        steps: manifest.clock.steps,
        hz: manifest.clock.hz,
    };
    let first = execute_scenario(&document, &asset.network, &options)
        .with_context(|| format!("execute scenario {}", xosc_path.display()))?;
    print_scenario_report(&xosc_path, &first);
    if manifest.output.determinism_check {
        let replay = execute_scenario(&document, &asset.network, &options)
            .with_context(|| format!("re-execute scenario {}", xosc_path.display()))?;
        anyhow::ensure!(
            first == replay,
            "scenario determinism check failed: first={first:?} replay={replay:?}"
        );
        println!("determinism: identical scenario outcome");
    }
    Ok(())
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

fn run_simulation(path: &Path, options: SimulationOptions<'_>) -> Result<()> {
    let SimulationOptions {
        steps,
        hz,
        action,
        trajectories,
        seed_override,
        determinism_check,
        replay_out,
        replay_controller,
        sensor_subscriptions,
        physics_backend,
    } = options;
    anyhow::ensure!(
        hz.is_finite() && hz > 0.0,
        "--hz must be finite and positive"
    );
    ensure_action_is_finite(&action)?;

    let run = simulate_scene_with_action_schedule(
        path,
        steps,
        hz,
        action.clone(),
        seed_override,
        None,
        &trajectories,
        &sensor_subscriptions,
        physics_backend,
    )?;
    print_simulation_report(path, &run.report, determinism_check);
    if determinism_check {
        let replay = simulate_scene_with_action_schedule(
            path,
            steps,
            hz,
            action,
            seed_override,
            None,
            &trajectories,
            &sensor_subscriptions,
            physics_backend,
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
            ReplayClock::new(steps, hz),
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
        &[],
        RunPhysicsBackend::Rapier,
    )
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
    sensor_subscriptions: &[RunSensorSubscription],
    physics_backend: RunPhysicsBackend,
) -> Result<SimulationRun> {
    if let Some(replay_frames) = replay_frames {
        anyhow::ensure!(
            replay_frames.len() as u64 == steps,
            "replay contains {} frames but {} steps were requested",
            replay_frames.len(),
            steps
        );
    }

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

    let dt = SimDuration::from_hertz(Hertz::new(hz));
    let mut sim_time = SimTime::ZERO;
    let mut command_buffer = ActuatorCommandBuffer::new();
    let mut data_bus = InMemoryDataBus::new();
    let sensor_payload_streams = resolve_sensor_subscriptions(&world, sensor_subscriptions)?;
    let mut frames = Vec::new();
    let mut contact_pairs_max = 0_u64;
    let mut contact_impulse_max_ns = 0.0_f32;
    let mut min_base_height_m: Option<f64> = None;
    for step in 0..steps {
        let frame_action = if let Some(replay_frames) = replay_frames {
            let step_index = usize::try_from(step)
                .map_err(|_| anyhow::anyhow!("replay step index {step} does not fit usize"))?;
            replay_frames[step_index].action.clone()
        } else if !trajectories.is_empty() {
            interpolate_joint_positions(trajectories, sim_time.as_seconds().value())
        } else {
            action.clone()
        };
        apply_replay_action(
            &world,
            &mut command_buffer,
            &drives,
            &frame_action,
            sim_time,
        )?;
        apply_actuator_commands(&mut world, &mut command_buffer);
        sync_all_joint_motors_from_actuators(&mut world);
        differential_drive_kinematics(&mut world, &drives, dt);
        backend
            .step(&mut world, physics_world, dt)
            .map_err(|error| anyhow::anyhow!("physics step: {error}"))?;
        sim_time = sim_time + dt;
        sample_sensors_for(&backend, &mut world, sim_time, physics_world, &mut data_bus);

        let base_translation_m = drives.first().and_then(|drive| {
            world.get::<Transform3>(drive.base_link).map(|transform| {
                [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ]
            })
        });
        let contact = backend
            .contacts(physics_world)
            .map_err(|error| anyhow::anyhow!("query contacts: {error}"))?;
        let contact = summarize_contacts(contact);
        contact_pairs_max = contact_pairs_max.max(contact.pair_count);
        contact_impulse_max_ns = contact_impulse_max_ns.max(contact.total_impulse_ns);
        if let Some(base_translation_m) = base_translation_m {
            let height_m = base_translation_m[1];
            min_base_height_m =
                Some(min_base_height_m.map_or(height_m, |minimum: f64| minimum.min(height_m)));
        }
        let observation = ReplayObservation::new(base_translation_m)
            .with_joint_state(capture_joint_state(&world))
            .with_sensor_streams(capture_sensor_streams(&world, &data_bus))
            .with_sensor_payloads(capture_sensor_payloads(
                &world,
                &data_bus,
                &sensor_payload_streams,
            ))
            .with_contact(Some(contact));
        frames.push(ReplayFrame::new(
            step,
            sim_time.ticks(),
            frame_action,
            observation,
            hash_physics_state(&world),
        ));
    }

    let first_base_translation_m = drives.first().and_then(|drive| {
        world.get::<Transform3>(drive.base_link).map(|transform| {
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
    Ok(SimulationRun {
        report: SimulationReport {
            steps,
            sim_time_s: sim_time.as_seconds().value(),
            seed,
            robot_count: spawned.robots.len(),
            differential_drive_count: drives.len(),
            physics_hash: hash_physics_state(&world),
            first_base_translation_m,
            contact_pairs_max,
            contact_impulse_max_ns,
            min_base_height_m,
            failure,
        },
        frames,
        sensor_payload_streams,
    })
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
    }
    Ok(())
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
    let artifact = ReplayArtifact::read_json(path)
        .with_context(|| format!("load replay artifact {}", path.display()))?;
    let scene_path = PathBuf::from(&artifact.scene);
    let run = simulate_scene_with_action_schedule(
        &scene_path,
        artifact.clock.steps,
        artifact.clock.hz,
        ReplayAction::differential_drive(0.0),
        Some(artifact.seed),
        Some(&artifact.frames),
        &[],
        &[],
        RunPhysicsBackend::Rapier,
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
        classify_failure, ensure_replay_frames, interpolate_joint_positions, replay_command,
        run_manifest_command, simulate_scene, simulate_scene_with_action_schedule,
        verify_physics_requirements,
    };
    use rne_assets::{
        RunPhysicsBackend, RunPhysicsCapability, RunSensorKind, RunSensorSubscription,
    };
    use rne_log::ReplayAction;
    use std::path::PathBuf;

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
        run_manifest_command(&manifest).expect("run manifest");
        let replay = manifest
            .parent()
            .expect("manifest parent")
            .join("../../target/runs/mesh_diff_drive.rne-replay");
        replay_command(&replay).expect("replay manifest output");
    }

    #[test]
    fn lidar_payload_run_manifest_records_and_replays() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/mesh_diff_drive_lidar_payload.rne.run.toml");
        run_manifest_command(&manifest).expect("run lidar payload manifest");
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
        run_manifest_command(&manifest).expect("run scenario manifest");
    }

    #[test]
    fn trajectory_run_manifest_negotiates_physics_and_replays() {
        let manifest = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../../assets/runs/mm_minimal_joint_trajectory.rne.run.toml");
        run_manifest_command(&manifest).expect("run trajectory manifest with physics checks");
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
        run_manifest_command(&manifest).expect("run analytic backend manifest");
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
            &[],
            RunPhysicsBackend::Rapier,
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
            &[],
            RunPhysicsBackend::Rapier,
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
            &[subscription],
            RunPhysicsBackend::Rapier,
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
            &[],
            RunPhysicsBackend::Rapier,
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
            &[subscription],
            RunPhysicsBackend::Rapier,
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
            &[],
            RunPhysicsBackend::Rapier,
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
            &[],
            RunPhysicsBackend::Rapier,
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
            &[],
            RunPhysicsBackend::Rapier,
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
            &[],
            RunPhysicsBackend::Rapier,
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
}

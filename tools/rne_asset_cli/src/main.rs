//! Command-line tools for RNE scene and robot assets.

use anyhow::{Context as _, Result};
use clap::{Parser, Subcommand};
use rne_assets::{
    inspect_asset, load_run_manifest, load_scene_bundle, smoke_spawn_scene, spawn_scene_bundle,
    validate_asset, AssetHotReloader, RunControllerKind, SpawnSceneOptions, ValidatedAsset,
};
use rne_core::{SimDuration, SimTime};
use rne_ecs::World;
use rne_log::{
    ReplayAction, ReplayArtifact, ReplayClock, ReplayControllerKind as ArtifactControllerKind,
    ReplayFinalReport, ReplayFrame, ReplayObservation,
};
use rne_math::Hertz;
use rne_physics::{hash_physics_state, PhysicsBackend, PhysicsWorldDesc};
use rne_physics_rapier::{step_physics, RapierBackend};
use rne_robot::{
    apply_actuator_commands, differential_drive_kinematics, ActuatorCommand, ActuatorCommandBuffer,
    DiffDriveComponent,
};
use rne_world::{Transform3, WorldEntity};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

const REPLAY_FLOAT_EPSILON: f64 = 1.0e-12;

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
        /// Run the same scene and inputs twice and compare the final report.
        #[arg(long)]
        determinism_check: bool,
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
            determinism_check,
            replay_out,
        } => simulate_command(
            &path,
            steps,
            hz,
            wheel_velocity_rad_s,
            determinism_check,
            replay_out.as_deref(),
        ),
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
}

#[derive(Clone, Copy, Debug)]
struct SimulationOptions<'a> {
    steps: u64,
    hz: f64,
    wheel_velocity_rad_s: f64,
    seed_override: Option<u64>,
    determinism_check: bool,
    replay_out: Option<&'a Path>,
    replay_controller: ArtifactControllerKind,
}

fn simulate_command(
    path: &Path,
    steps: u64,
    hz: f64,
    wheel_velocity_rad_s: f64,
    determinism_check: bool,
    replay_out: Option<&Path>,
) -> Result<()> {
    run_simulation(
        path,
        SimulationOptions {
            steps,
            hz,
            wheel_velocity_rad_s,
            seed_override: None,
            determinism_check,
            replay_out,
            replay_controller: ArtifactControllerKind::DifferentialDrive,
        },
    )
}

fn run_manifest_command(path: &Path) -> Result<()> {
    let manifest =
        load_run_manifest(path).with_context(|| format!("load run manifest {}", path.display()))?;
    let scene_path = manifest.resolve_scene_path(path);
    let wheel_velocity_rad_s = match manifest.controller.kind {
        RunControllerKind::None => 0.0,
        RunControllerKind::DifferentialDrive => manifest.controller.wheel_velocity_rad_s,
    };
    let replay_controller = match manifest.controller.kind {
        RunControllerKind::None => ArtifactControllerKind::None,
        RunControllerKind::DifferentialDrive => ArtifactControllerKind::DifferentialDrive,
    };
    let replay_out = manifest
        .output
        .replay_path
        .as_deref()
        .map(|output_path| manifest.resolve_output_path(path, output_path));
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
            wheel_velocity_rad_s,
            seed_override: manifest.seed,
            determinism_check: manifest.output.determinism_check,
            replay_out: replay_out.as_deref(),
            replay_controller,
        },
    )
}

fn run_simulation(path: &Path, options: SimulationOptions<'_>) -> Result<()> {
    let SimulationOptions {
        steps,
        hz,
        wheel_velocity_rad_s,
        seed_override,
        determinism_check,
        replay_out,
        replay_controller,
    } = options;
    anyhow::ensure!(
        hz.is_finite() && hz > 0.0,
        "--hz must be finite and positive"
    );
    anyhow::ensure!(
        wheel_velocity_rad_s.is_finite(),
        "--wheel-velocity-rad-s must be finite"
    );

    let run = simulate_scene_with_seed(path, steps, hz, wheel_velocity_rad_s, seed_override)?;
    print_simulation_report(path, &run.report, determinism_check);
    if determinism_check {
        let replay =
            simulate_scene_with_seed(path, steps, hz, wheel_velocity_rad_s, seed_override)?;
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
            run.frames.clone(),
            replay_final_report(&run.report),
        );
        artifact
            .write_json(replay_out)
            .with_context(|| format!("write replay artifact {}", replay_out.display()))?;
        println!(
            "replay: wrote {} (version={} frames={})",
            replay_out.display(),
            artifact.version,
            artifact.frames.len()
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
}

fn simulate_scene_with_seed(
    path: &Path,
    steps: u64,
    hz: f64,
    wheel_velocity_rad_s: f64,
    seed_override: Option<u64>,
) -> Result<SimulationRun> {
    simulate_scene_with_action_schedule(path, steps, hz, wheel_velocity_rad_s, seed_override, None)
}

fn simulate_scene_with_action_schedule(
    path: &Path,
    steps: u64,
    hz: f64,
    wheel_velocity_rad_s: f64,
    seed_override: Option<u64>,
    replay_frames: Option<&[ReplayFrame]>,
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
    let mut backend = RapierBackend::new();
    let physics_world = backend
        .create_world(PhysicsWorldDesc::default())
        .map_err(|error| anyhow::anyhow!("create physics world: {error}"))?;
    backend
        .sync_from_ecs(&mut world, physics_world)
        .map_err(|error| anyhow::anyhow!("sync scene into physics: {error}"))?;

    let dt = SimDuration::from_hertz(Hertz::new(hz));
    let mut sim_time = SimTime::ZERO;
    let mut command_buffer = ActuatorCommandBuffer::new();
    let mut frames = Vec::new();
    for step in 0..steps {
        let wheel_velocity_rad_s = if let Some(replay_frames) = replay_frames {
            let step_index = usize::try_from(step)
                .map_err(|_| anyhow::anyhow!("replay step index {step} does not fit usize"))?;
            replay_frames[step_index].action.wheel_velocity_rad_s
        } else {
            wheel_velocity_rad_s
        };
        for drive in &drives {
            command_buffer.push(
                ActuatorCommand::WheelVelocity {
                    wheel: drive.left_actuator,
                    velocity_rad_s: wheel_velocity_rad_s,
                },
                sim_time,
            );
            command_buffer.push(
                ActuatorCommand::WheelVelocity {
                    wheel: drive.right_actuator,
                    velocity_rad_s: wheel_velocity_rad_s,
                },
                sim_time,
            );
        }
        apply_actuator_commands(&mut world, &mut command_buffer);
        differential_drive_kinematics(&mut world, &drives, dt);
        step_physics(&mut backend, &mut world, physics_world, dt)
            .map_err(|error| anyhow::anyhow!("physics step: {error}"))?;
        sim_time = sim_time + dt;

        let base_translation_m = drives.first().and_then(|drive| {
            world.get::<Transform3>(drive.base_link).map(|transform| {
                [
                    transform.translation.x,
                    transform.translation.y,
                    transform.translation.z,
                ]
            })
        });
        frames.push(ReplayFrame::new(
            step,
            sim_time.ticks(),
            ReplayAction::differential_drive(wheel_velocity_rad_s),
            ReplayObservation::new(base_translation_m),
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
    Ok(SimulationRun {
        report: SimulationReport {
            steps,
            sim_time_s: sim_time.as_seconds().value(),
            seed,
            robot_count: spawned.robots.len(),
            differential_drive_count: drives.len(),
            physics_hash: hash_physics_state(&world),
            first_base_translation_m,
        },
        frames,
    })
}

fn replay_command(path: &Path) -> Result<()> {
    let artifact = ReplayArtifact::read_json(path)
        .with_context(|| format!("load replay artifact {}", path.display()))?;
    let scene_path = PathBuf::from(&artifact.scene);
    let run = simulate_scene_with_action_schedule(
        &scene_path,
        artifact.clock.steps,
        artifact.clock.hz,
        0.0,
        Some(artifact.seed),
        Some(&artifact.frames),
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
            replay_float_matches(
                expected_frame.action.wheel_velocity_rad_s,
                actual_frame.action.wheel_velocity_rad_s
            ),
            "replay action mismatch at step {}: expected={} actual={}",
            expected_frame.step,
            expected_frame.action.wheel_velocity_rad_s,
            actual_frame.action.wheel_velocity_rad_s
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
            expected_frame.physics_hash == actual_frame.physics_hash,
            "replay frame mismatch at step {}: expected={expected_frame:?} actual={actual_frame:?}",
            expected_frame.step
        );
    }
    Ok(())
}

fn ensure_replay_reports(expected: &ReplayFinalReport, actual: &ReplayFinalReport) -> Result<()> {
    anyhow::ensure!(
        expected.steps == actual.steps
            && expected.seed == actual.seed
            && expected.robot_count == actual.robot_count
            && expected.differential_drive_count == actual.differential_drive_count
            && expected.physics_hash == actual.physics_hash,
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
    println!(
        "simulate scene={} steps={} time={:.6} s seed={} robots={} diff_drive={} base={} physics_hash={:#018x}{}",
        path.display(),
        report.steps,
        report.sim_time_s,
        report.seed,
        report.robot_count,
        report.differential_drive_count,
        base,
        report.physics_hash,
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
    use super::{replay_command, run_manifest_command, simulate_scene};
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
}

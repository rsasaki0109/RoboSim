use anyhow::{bail, ensure, Context, Result};
use rne_mobility_benchmark::ackermann_observed::{
    run_ackermann_observed_failure_capsule, run_ackermann_observed_trace, AckermannObservedFault,
};
use rne_mobility_benchmark::ackermann_suspension::run_ackermann_suspension_trace;
use rne_mobility_benchmark::backend::run_backend_mobility_trace;
use rne_mobility_benchmark::diff_caster::run_differential_caster_trace;
use rne_mobility_benchmark::observed::run_sensor_observed_trace;
use rne_mobility_benchmark::per_wheel::run_per_wheel_skid_trace;
use rne_mobility_benchmark::per_wheel_observed::{
    run_per_wheel_observed_failure_capsule, run_per_wheel_observed_trace, PerWheelObservedFault,
};
use rne_mobility_benchmark::road_excitation::run_road_excitation_trace;
use rne_mobility_benchmark::run_mobility_benchmark;
use rne_physics_rapier::RapierBackend;
use std::path::PathBuf;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut output = None;
    let mut failure_replay = None;
    let mut fault = None;
    let mut backend = "analytic".to_string();
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--backend" => {
                backend = args.next().context("--backend requires a value")?;
            }
            "--output" => {
                output = Some(PathBuf::from(
                    args.next().context("--output requires a path")?,
                ));
            }
            "--failure-replay" => {
                failure_replay = Some(PathBuf::from(
                    args.next().context("--failure-replay requires a path")?,
                ));
            }
            "--fault" => {
                fault = Some(args.next().context("--fault requires a value")?);
            }
            other => bail!("unknown argument: {other}"),
        }
    }
    let (json, label) = match backend.as_str() {
        "analytic" => {
            let report = run_mobility_benchmark()?;
            ensure!(report.passed, "mobility benchmark verdict failed");
            (serde_json::to_string_pretty(&report)? + "\n", "analytic")
        }
        "rapier" => {
            let trace =
                run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier mobility benchmark verdict failed");
            (serde_json::to_string_pretty(&trace)? + "\n", "rapier")
        }
        "sensor-rapier" => {
            let trace = run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier sensor-observed verdict failed");
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "sensor-rapier",
            )
        }
        "skid-rapier" => {
            let trace = run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier per-wheel skid verdict failed");
            (serde_json::to_string_pretty(&trace)? + "\n", "skid-rapier")
        }
        "skid-sensor-rapier" => {
            let trace =
                run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier per-wheel sensor verdict failed");
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "skid-sensor-rapier",
            )
        }
        "skid-sensor-failure-rapier" => {
            let fault = parse_fatal_sensor_fault(
                fault
                    .as_deref()
                    .context("--backend skid-sensor-failure-rapier requires --fault")?,
            )?;
            let capsule = run_per_wheel_observed_failure_capsule(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )?;
            capsule.validate()?;
            (
                serde_json::to_string_pretty(&capsule)? + "\n",
                "skid-sensor-failure-rapier",
            )
        }
        "mujoco" => run_mujoco()?,
        "compare" => run_comparison(failure_replay.as_deref())?,
        "sensor-mujoco" => run_sensor_mujoco()?,
        "sensor-compare" => run_sensor_comparison()?,
        "skid-mujoco" => run_skid_mujoco()?,
        "skid-compare" => run_skid_comparison()?,
        "skid-sensor-mujoco" => run_skid_sensor_mujoco()?,
        "skid-sensor-compare" => run_skid_sensor_comparison()?,
        "diff-caster-rapier" => {
            let trace =
                run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(trace.passed, "Rapier differential-caster verdict failed");
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "diff-caster-rapier",
            )
        }
        "diff-caster-mujoco" => run_diff_caster_mujoco()?,
        "diff-caster-compare" => run_diff_caster_comparison()?,
        "ackermann-suspension-rapier" => {
            let trace =
                run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(
                trace.passed,
                "Rapier Ackermann-suspension verdict failed: {:#?}",
                trace.metrics
            );
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "ackermann-suspension-rapier",
            )
        }
        "ackermann-suspension-mujoco" => run_ackermann_suspension_mujoco()?,
        "ackermann-suspension-compare" => run_ackermann_suspension_comparison()?,
        "ackermann-sensor-rapier" => {
            let trace =
                run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(
                trace.passed,
                "Rapier sensor-only Ackermann verdict failed: {:#?}",
                trace.metrics
            );
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "ackermann-sensor-rapier",
            )
        }
        "ackermann-sensor-failure-rapier" => {
            let fault = parse_ackermann_fatal_sensor_fault(
                fault
                    .as_deref()
                    .context("--backend ackermann-sensor-failure-rapier requires --fault")?,
            )?;
            let capsule = run_ackermann_observed_failure_capsule(
                RapierBackend::new(),
                RapierBackend::manifest(),
                fault,
            )?;
            capsule.validate()?;
            (
                serde_json::to_string_pretty(&capsule)? + "\n",
                "ackermann-sensor-failure-rapier",
            )
        }
        "ackermann-sensor-mujoco" => run_ackermann_observed_mujoco()?,
        "ackermann-sensor-failure-mujoco" => {
            run_ackermann_observed_failure_mujoco(fault.as_deref())?
        }
        "ackermann-sensor-compare" => run_ackermann_observed_comparison()?,
        "road-excitation-rapier" => {
            let trace = run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest())?;
            ensure!(
                trace.passed,
                "Rapier road-excitation verdict failed: {:#?}",
                trace.metrics
            );
            (
                serde_json::to_string_pretty(&trace)? + "\n",
                "road-excitation-rapier",
            )
        }
        "road-excitation-mujoco" => run_road_excitation_mujoco()?,
        "road-excitation-compare" => run_road_excitation_comparison()?,
        other => bail!("unknown backend: {other}"),
    };
    ensure!(
        backend == "compare" || failure_replay.is_none(),
        "--failure-replay is valid only with --backend compare"
    );
    ensure!(
        matches!(
            backend.as_str(),
            "skid-sensor-failure-rapier"
                | "ackermann-sensor-failure-rapier"
                | "ackermann-sensor-failure-mujoco"
        ) || fault.is_none(),
        "--fault is valid only with a sensor-failure backend"
    );
    if let Some(path) = output {
        if let Some(parent) = path.parent() {
            std::fs::create_dir_all(parent)
                .with_context(|| format!("create {}", parent.display()))?;
        }
        std::fs::write(&path, json).with_context(|| format!("write {}", path.display()))?;
        println!("{label} mobility benchmark passed: {}", path.display());
    } else {
        print!("{json}");
    }
    Ok(())
}

#[cfg(feature = "mujoco")]
fn run_ackermann_observed_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_observed::ACKERMANN_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_ackermann_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(
        trace.passed,
        "MuJoCo sensor-only Ackermann verdict failed: {:#?}",
        trace.metrics
    );
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "ackermann-sensor-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_ackermann_observed_failure_mujoco(fault: Option<&str>) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_observed::ACKERMANN_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let fault = parse_ackermann_fatal_sensor_fault(
        fault.context("--backend ackermann-sensor-failure-mujoco requires --fault")?,
    )?;
    let capsule = run_ackermann_observed_failure_capsule(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
        fault,
    )?;
    capsule.validate()?;
    Ok((
        serde_json::to_string_pretty(&capsule)? + "\n",
        "ackermann-sensor-failure-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_ackermann_observed_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_observed::{
        compare_ackermann_observed_traces, ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_ackermann_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_ackermann_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_ackermann_observed_traces(rapier, mujoco)?;
    ensure!(
        comparison.passed,
        "sensor-only Ackermann comparison failed: {:#?}",
        comparison.metrics
    );
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "ackermann-sensor-rapier-vs-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_road_excitation_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::road_excitation::ROAD_EXCITATION_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_road_excitation_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(
        trace.passed,
        "MuJoCo road-excitation verdict failed: {:#?}",
        trace.metrics
    );
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "road-excitation-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_road_excitation_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::road_excitation::{
        compare_road_excitation_traces, ROAD_EXCITATION_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_road_excitation_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_road_excitation_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(ROAD_EXCITATION_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_road_excitation_traces(rapier, mujoco)?;
    ensure!(
        comparison.passed,
        "road-excitation comparison failed: {:#?}",
        comparison.metrics
    );
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "road-excitation-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_observed_mujoco() -> Result<(String, &'static str)> {
    bail!("sensor-only Ackermann mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_observed_failure_mujoco(_fault: Option<&str>) -> Result<(String, &'static str)> {
    bail!("sensor-only Ackermann failure mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_observed_comparison() -> Result<(String, &'static str)> {
    bail!("sensor-only Ackermann comparison requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_road_excitation_mujoco() -> Result<(String, &'static str)> {
    bail!("road-excitation mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_road_excitation_comparison() -> Result<(String, &'static str)> {
    bail!("road-excitation comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_ackermann_suspension_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_suspension::ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_ackermann_suspension_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(
        trace.passed,
        "MuJoCo Ackermann-suspension verdict failed: {:#?}",
        trace.metrics
    );
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "ackermann-suspension-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_ackermann_suspension_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::ackermann_suspension::{
        compare_ackermann_suspension_traces, ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_ackermann_suspension_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_ackermann_suspension_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            ACKERMANN_SUSPENSION_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_ackermann_suspension_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "Ackermann-suspension comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "ackermann-suspension-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_suspension_mujoco() -> Result<(String, &'static str)> {
    bail!("Ackermann-suspension mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_ackermann_suspension_comparison() -> Result<(String, &'static str)> {
    bail!("Ackermann-suspension comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_diff_caster_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::diff_caster::DIFF_CASTER_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_differential_caster_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(DIFF_CASTER_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(trace.passed, "MuJoCo differential-caster verdict failed");
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "diff-caster-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_diff_caster_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::diff_caster::{
        compare_differential_caster_traces, DIFF_CASTER_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_differential_caster_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_differential_caster_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(DIFF_CASTER_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_differential_caster_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "differential-caster comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "diff-caster-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_diff_caster_mujoco() -> Result<(String, &'static str)> {
    bail!("differential-caster mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_diff_caster_comparison() -> Result<(String, &'static str)> {
    bail!("differential-caster comparison requires --features mujoco")
}

fn parse_fatal_sensor_fault(value: &str) -> Result<PerWheelObservedFault> {
    match value {
        "encoder-stuck" => Ok(PerWheelObservedFault::FrontLeftEncoderStuck { sequence: 30 }),
        "encoder-saturated" => Ok(PerWheelObservedFault::FrontLeftEncoderSaturate {
            counter_bits: 4,
        }),
        "imu-stuck" => Ok(PerWheelObservedFault::ImuStuck { sequence: 30 }),
        other => bail!(
            "unknown fatal sensor fault {other}; expected encoder-stuck, encoder-saturated, or imu-stuck"
        ),
    }
}

fn parse_ackermann_fatal_sensor_fault(value: &str) -> Result<AckermannObservedFault> {
    match value {
        "wheel-stuck" => Ok(AckermannObservedFault::FrontLeftWheelStuck { sequence: 200 }),
        "wheel-saturated" => Ok(AckermannObservedFault::FrontLeftWheelSaturate {
            counter_bits: 4,
        }),
        "steering-stuck" => Ok(AckermannObservedFault::FrontRightSteeringStuck {
            sequence: 200,
        }),
        "imu-stuck" => Ok(AckermannObservedFault::ImuStuck { sequence: 200 }),
        other => bail!(
            "unknown fatal Ackermann sensor fault {other}; expected wheel-stuck, wheel-saturated, steering-stuck, or imu-stuck"
        ),
    }
}

#[cfg(feature = "mujoco")]
fn run_skid_sensor_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel_observed::PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let trace = run_per_wheel_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    ensure!(trace.passed, "MuJoCo per-wheel sensor verdict failed");
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "skid-sensor-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_skid_sensor_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel_observed::{
        compare_per_wheel_observed_traces, PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_per_wheel_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_per_wheel_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(
            PER_WHEEL_OBSERVED_FIXED_DELTA_TICKS,
        ))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_per_wheel_observed_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "per-wheel sensor comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "skid-sensor-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_sensor_mujoco() -> Result<(String, &'static str)> {
    bail!("per-wheel sensor mujoco requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_sensor_comparison() -> Result<(String, &'static str)> {
    bail!("per-wheel sensor comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_skid_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel::PER_WHEEL_SKID_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let backend = MuJoCoBackend::new(SimDuration::from_ticks(PER_WHEEL_SKID_FIXED_DELTA_TICKS))?;
    let trace = run_per_wheel_skid_trace(backend, MuJoCoBackend::manifest())?;
    ensure!(trace.passed, "MuJoCo per-wheel skid verdict failed");
    Ok((serde_json::to_string_pretty(&trace)? + "\n", "skid-mujoco"))
}

#[cfg(feature = "mujoco")]
fn run_skid_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::per_wheel::{
        compare_per_wheel_skid_traces, PER_WHEEL_SKID_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_per_wheel_skid_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_per_wheel_skid_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(PER_WHEEL_SKID_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_per_wheel_skid_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "per-wheel skid comparison failed");
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "skid-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_mujoco() -> Result<(String, &'static str)> {
    bail!("per-wheel skid mujoco backend requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_skid_comparison() -> Result<(String, &'static str)> {
    bail!("per-wheel skid comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_sensor_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::observed::SENSOR_OBSERVED_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let backend = MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))?;
    let trace = run_sensor_observed_trace(backend, MuJoCoBackend::manifest())?;
    ensure!(trace.passed, "MuJoCo sensor-observed verdict failed");
    Ok((
        serde_json::to_string_pretty(&trace)? + "\n",
        "sensor-mujoco",
    ))
}

#[cfg(feature = "mujoco")]
fn run_sensor_comparison() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::observed::{
        compare_sensor_observed_traces, SENSOR_OBSERVED_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_sensor_observed_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_sensor_observed_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(SENSOR_OBSERVED_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_sensor_observed_traces(rapier, mujoco)?;
    ensure!(
        comparison.passed,
        "sensor-observed cross-backend verdict failed"
    );
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "sensor-rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_sensor_mujoco() -> Result<(String, &'static str)> {
    bail!("sensor-observed mujoco backend requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_sensor_comparison() -> Result<(String, &'static str)> {
    bail!("sensor-observed backend comparison requires --features mujoco")
}

#[cfg(feature = "mujoco")]
fn run_mujoco() -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::backend::BACKEND_MOBILITY_FIXED_DELTA_TICKS;
    use rne_physics_mujoco::MuJoCoBackend;

    let backend = MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))?;
    let trace = run_backend_mobility_trace(backend, MuJoCoBackend::manifest())?;
    ensure!(trace.passed, "MuJoCo mobility benchmark verdict failed");
    Ok((serde_json::to_string_pretty(&trace)? + "\n", "mujoco"))
}

#[cfg(feature = "mujoco")]
fn run_comparison(failure_replay: Option<&std::path::Path>) -> Result<(String, &'static str)> {
    use rne_core::SimDuration;
    use rne_mobility_benchmark::backend::{
        backend_mobility_divergence_replay, compare_backend_mobility_traces,
        BACKEND_MOBILITY_FIXED_DELTA_TICKS,
    };
    use rne_physics_mujoco::MuJoCoBackend;

    let rapier = run_backend_mobility_trace(RapierBackend::new(), RapierBackend::manifest())?;
    let mujoco = run_backend_mobility_trace(
        MuJoCoBackend::new(SimDuration::from_ticks(BACKEND_MOBILITY_FIXED_DELTA_TICKS))?,
        MuJoCoBackend::manifest(),
    )?;
    let comparison = compare_backend_mobility_traces(rapier, mujoco)?;
    ensure!(comparison.passed, "cross-backend mobility verdict failed");
    if let Some(path) = failure_replay {
        let replay =
            backend_mobility_divergence_replay(&comparison.first, &comparison.second, 0.001)?;
        replay.write_json(path)?;
    }
    Ok((
        serde_json::to_string_pretty(&comparison)? + "\n",
        "rapier-vs-mujoco",
    ))
}

#[cfg(not(feature = "mujoco"))]
fn run_mujoco() -> Result<(String, &'static str)> {
    bail!("mujoco backend requires --features mujoco")
}

#[cfg(not(feature = "mujoco"))]
fn run_comparison(_failure_replay: Option<&std::path::Path>) -> Result<(String, &'static str)> {
    bail!("backend comparison requires --features mujoco")
}

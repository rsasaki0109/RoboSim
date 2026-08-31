use anyhow::{bail, ensure, Context, Result};
use rne_mobility_benchmark::backend::run_backend_mobility_trace;
use rne_mobility_benchmark::run_mobility_benchmark;
use rne_physics_rapier::RapierBackend;
use std::path::PathBuf;

fn main() -> Result<()> {
    let mut args = std::env::args().skip(1);
    let mut output = None;
    let mut failure_replay = None;
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
        "mujoco" => run_mujoco()?,
        "compare" => run_comparison(failure_replay.as_deref())?,
        other => bail!("unknown backend: {other}"),
    };
    ensure!(
        backend == "compare" || failure_replay.is_none(),
        "--failure-replay is valid only with --backend compare"
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

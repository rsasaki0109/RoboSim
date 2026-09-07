//! Separate-process checkpoint smoke. Data paths are explicit; never overwrite.

use anyhow::{ensure, Context, Result};
use rne_mobility_benchmark::observed_fixed_batch::SensorLearningSession;
use rne_physics::{PhysicsBackend, PhysicsBackendManifest};
use sha2::{Digest, Sha256};
use std::io::{Read, Write};

fn run<B, F>(factory: F, mode: &str, path: &std::path::Path) -> Result<()>
where
    B: PhysicsBackend,
    F: Fn() -> Result<(B, PhysicsBackendManifest)>,
{
    let mut session = match mode {
        "record" => {
            let mut session = SensorLearningSession::new(factory, 620, 621, 622, 2, 2)?;
            for index in 0..30 {
                if index == 15 {
                    session.reset_lanes(&[(1, 3)])?;
                }
                ensure!(
                    session.step()?.failures.is_empty(),
                    "recording solver failure"
                );
            }
            let bytes = session.checkpoint()?;
            let mut file = std::fs::OpenOptions::new()
                .write(true)
                .create_new(true)
                .open(path)
                .context("create checkpoint without overwriting")?;
            file.write_all(&bytes)?;
            file.sync_all()?;
            session
        }
        "resume" => {
            let file = std::fs::File::open(path)?;
            ensure!(
                file.metadata()?.len() <= 8 * 1024 * 1024,
                "checkpoint exceeds 8 MiB"
            );
            let mut bytes = Vec::new();
            file.take(8 * 1024 * 1024 + 1).read_to_end(&mut bytes)?;
            // A different worker count and fresh process must reproduce history.
            SensorLearningSession::from_checkpoint(factory, 1, &bytes)?
        }
        _ => anyhow::bail!("mode must be record or resume"),
    };
    for _ in 0..30 {
        ensure!(
            session.step()?.failures.is_empty(),
            "continuation solver failure"
        );
    }
    ensure!(
        session.learner().updates() == 120,
        "unexpected update count"
    );
    println!(
        "{}",
        serde_json::json!({
            "updates": session.learner().updates(),
            "continued_session_sha256": format!("{:x}", Sha256::digest(session.checkpoint()?)),
        })
    );
    Ok(())
}

fn main() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    ensure!(args.len() == 3, "usage: learning_session_checkpoint <rapier|mujoco> <record|resume> <external-checkpoint-path>");
    let path = std::path::Path::new(&args[2]);
    match args[0].as_str() {
        "rapier" => run(
            || {
                Ok((
                    rne_physics_rapier::RapierBackend::new(),
                    rne_physics_rapier::RapierBackend::manifest(),
                ))
            },
            &args[1],
            path,
        ),
        #[cfg(feature = "mujoco")]
        "mujoco" => run(
            || {
                Ok((
                    rne_physics_mujoco::MuJoCoBackend::new(rne_core::SimDuration::from_ticks(
                        1_000_000,
                    ))?,
                    rne_physics_mujoco::MuJoCoBackend::manifest(),
                ))
            },
            &args[1],
            path,
        ),
        _ => anyhow::bail!("unsupported backend; MuJoCo requires --features mujoco"),
    }
}

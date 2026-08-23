//! Reproducible README showcase captures for three independent RNE scenarios.
//!
//! Headless evidence:
//!
//! ```text
//! cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment all
//! ```
//!
//! GPU capture (writes 960x540 GIF/poster/metadata for the selected source):
//!
//! ```text
//! cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment all
//! ```

mod factory;
mod media;
mod office;
mod openarm;
mod ssl;
mod tsukuba;

use anyhow::{bail, Result};
use std::path::PathBuf;

fn main() {
    if let Err(error) = run() {
        eprintln!("showcase capture failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let smoke = args.iter().any(|argument| argument == "--smoke");
    let capture = args.iter().any(|argument| argument == "--capture");
    let environment = args
        .windows(2)
        .find(|window| window[0] == "--environment")
        .map(|window| window[1].as_str())
        .unwrap_or("all");
    if !smoke && !capture {
        bail!("choose --smoke or --capture");
    }
    let selected = parse_environment(environment)?;
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    for environment in selected {
        let metadata = match environment {
            Environment::Tsukuba => tsukuba::run(&repo_root, capture)?,
            Environment::OpenArm => openarm::run(&repo_root, capture)?,
            Environment::Factory => factory::run(&repo_root, capture)?,
            Environment::Office => office::run(&repo_root, capture)?,
            Environment::Ssl => ssl::run(&repo_root, capture)?,
        };
        println!(
            "showcase {}: steps={} final_digest={:#018x} replay_match={} captured={}",
            metadata.environment_id,
            metadata.simulation.steps,
            metadata.simulation.final_state_digest,
            metadata.simulation.replay_match,
            metadata.capture.is_some(),
        );
    }
    Ok(())
}

#[derive(Clone, Copy, Debug)]
enum Environment {
    Tsukuba,
    OpenArm,
    Factory,
    Office,
    Ssl,
}

fn parse_environment(value: &str) -> Result<Vec<Environment>> {
    match value {
        "all" => Ok(vec![
            Environment::OpenArm,
            Environment::Factory,
            Environment::Office,
        ]),
        "tsukuba" => Ok(vec![Environment::Tsukuba]),
        "openarm" => Ok(vec![Environment::OpenArm]),
        "factory" => Ok(vec![Environment::Factory]),
        "office" => Ok(vec![Environment::Office]),
        "ssl" => Ok(vec![Environment::Ssl]),
        other => bail!(
            "unknown --environment {other:?}; expected all|openarm|tsukuba|factory|office|ssl"
        ),
    }
}

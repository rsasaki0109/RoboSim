//! Command-line tools for RNE scene and robot assets.

use anyhow::{Context as _, Result};
use clap::{Parser, Subcommand};
use rne_assets::{
    inspect_asset, smoke_spawn_scene, validate_asset, AssetHotReloader, ValidatedAsset,
};
use std::path::{Path, PathBuf};
use std::thread;
use std::time::Duration;

#[derive(Parser)]
#[command(name = "rne-asset", about = "Validate and watch RNE asset files")]
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

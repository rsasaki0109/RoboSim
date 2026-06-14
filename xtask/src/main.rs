//! Workspace automation tasks for Robot Native Engine.

use std::process::{Command, ExitCode};
use std::{env, path::PathBuf};

fn main() -> ExitCode {
    match run() {
        Ok(()) => ExitCode::SUCCESS,
        Err(error) => {
            eprintln!("xtask error: {error:#}");
            ExitCode::FAILURE
        }
    }
}

fn run() -> anyhow::Result<()> {
    let mut args = env::args().skip(1);
    let command = args.next().unwrap_or_else(|| "ci".to_string());

    match command.as_str() {
        "ci" => ci(),
        "ci-ros2" => ci_ros2(),
        "ci-ros2-bridge" => ci_ros2_bridge(),
        "asset" => asset_command(&mut args),
        "lint-boundaries" => lint_boundaries(),
        other => anyhow::bail!("unknown xtask command: {other}"),
    }
}

fn ci() -> anyhow::Result<()> {
    run_step("cargo fmt --all -- --check")?;
    lint_boundaries()?;
    run_step("cargo clippy --workspace --all-targets -- -D warnings")?;
    run_step("cargo test --workspace")?;
    validate_repo_assets()?;
    run_example_smokes()?;
    Ok(())
}

fn run_example_smokes() -> anyhow::Result<()> {
    run_step("cargo run -p mobile_manipulator_arm --example 20_mobile_manipulator_arm -- --smoke")?;
    run_step(
        "cargo run -p mobile_manipulator_reach --example 21_mobile_manipulator_reach -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_grasp --example 22_mobile_manipulator_grasp -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_transport --example 23_mobile_manipulator_transport -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_wrist_cam --example 24_mobile_manipulator_wrist_cam -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_episode --example 25_mobile_manipulator_episode -- --smoke",
    )?;
    run_step(
        "cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator",
    )?;
    run_step(
        "cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator-mobile",
    )?;
    Ok(())
}

fn validate_repo_assets() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let scenes = [
        root.join("assets/scenes/episode_diff_drive.rne.scene.toml"),
        root.join("assets/scenes/mm_mobile.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal_grasp.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal_transport.rne.scene.toml"),
    ];
    let robots = [
        root.join("assets/robots/diff_drive.rne.robot.toml"),
        root.join("assets/robots/diff_drive_urdf.rne.robot.toml"),
        root.join("assets/robots/mm_minimal.rne.robot.toml"),
        root.join("assets/robots/mm_mobile.rne.robot.toml"),
    ];

    for scene in scenes {
        rne_assets::validate_asset(&scene).map_err(|error| {
            anyhow::anyhow!("asset validation failed for {}: {error}", scene.display())
        })?;
        let robot_count = rne_assets::smoke_spawn_scene(&scene).map_err(|error| {
            anyhow::anyhow!("asset spawn smoke failed for {}: {error}", scene.display())
        })?;
        println!("validated scene {} (robots={robot_count})", scene.display());
    }

    for robot in robots {
        rne_assets::validate_asset(&robot).map_err(|error| {
            anyhow::anyhow!("asset validation failed for {}: {error}", robot.display())
        })?;
        println!("validated robot {}", robot.display());
    }

    Ok(())
}

fn asset_command(args: &mut impl Iterator<Item = String>) -> anyhow::Result<()> {
    let subcommand = args.next().unwrap_or_else(|| "validate".to_string());
    let path = args.next().map(PathBuf::from).unwrap_or_else(|| {
        workspace_root()
            .expect("workspace root")
            .join("assets/scenes/episode_diff_drive.rne.scene.toml")
    });

    match subcommand.as_str() {
        "validate" => {
            let validated = rne_assets::validate_asset(&path)?;
            match validated {
                rne_assets::ValidatedAsset::Scene(bundle) => {
                    println!(
                        "valid scene: robots={} seed={}",
                        bundle.robots.len(),
                        bundle.scene.world.seed
                    );
                    let robot_count = rne_assets::smoke_spawn_scene(&path)?;
                    println!("spawn ok: robots={robot_count}");
                }
                rne_assets::ValidatedAsset::Robot { asset, .. } => {
                    println!(
                        "valid robot: kind={:?} model={}",
                        asset.kind, asset.model_name
                    );
                }
            }
        }
        "inspect" => {
            println!("{}", rne_assets::inspect_asset(&path)?);
        }
        other => anyhow::bail!("unknown asset subcommand: {other}"),
    }

    Ok(())
}

fn ci_ros2() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let script = root.join("adapters/ros2/rne_ros2_node/smoke_test.sh");
    if !script.is_file() {
        anyhow::bail!("missing ROS 2 smoke script at {}", script.display());
    }
    if !ros_setup_available() {
        println!("ROS 2 setup.bash not found under /opt/ros; skipping ci-ros2");
        return Ok(());
    }
    run_step(&format!("bash {}", script.display()))?;
    Ok(())
}

fn ci_ros2_bridge() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let script = root.join("adapters/ros2/rne_ros2_bridge/smoke_test.sh");
    if !script.is_file() {
        anyhow::bail!("missing ROS 2 bridge smoke script at {}", script.display());
    }
    if !ros_setup_available() {
        println!("ROS 2 setup.bash not found under /opt/ros; skipping ci-ros2-bridge");
        return Ok(());
    }
    run_step(&format!("bash {}", script.display()))?;
    Ok(())
}

fn ros_setup_available() -> bool {
    PathBuf::from("/opt/ros/jazzy/setup.bash").is_file()
        || PathBuf::from("/opt/ros/humble/setup.bash").is_file()
}

fn lint_boundaries() -> anyhow::Result<()> {
    let workspace_root = workspace_root()?;
    let forbidden = ["rcl", "rclrs", "rclcpp", "ros2", "adapters/", "../adapters"];

    for manifest in find_cargo_tomls(&workspace_root.join("crates"))? {
        let content = std::fs::read_to_string(&manifest)?;
        for line in content.lines() {
            let trimmed = line.trim();
            if !trimmed.starts_with('"') && !trimmed.contains(" = ") {
                continue;
            }
            for pattern in forbidden {
                if trimmed.contains(pattern) {
                    anyhow::bail!(
                        "forbidden dependency in core crate {}: {}",
                        manifest.display(),
                        trimmed
                    );
                }
            }
        }
    }

    println!("dependency boundary check passed");
    Ok(())
}

fn run_step(command: &str) -> anyhow::Result<()> {
    println!("$ {command}");
    let status = if cfg!(windows) {
        Command::new("cmd").args(["/C", command]).status()?
    } else {
        Command::new("sh").arg("-c").arg(command).status()?
    };

    if status.success() {
        Ok(())
    } else {
        anyhow::bail!("command failed with status {status}");
    }
}

fn workspace_root() -> anyhow::Result<PathBuf> {
    let output = Command::new("cargo")
        .args(["metadata", "--format-version", "1", "--no-deps"])
        .output()?;

    if !output.status.success() {
        anyhow::bail!("cargo metadata failed");
    }

    let metadata: serde_json::Value = serde_json::from_slice(&output.stdout)?;
    let root = metadata["workspace_root"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("missing workspace_root in cargo metadata"))?;

    Ok(PathBuf::from(root))
}

fn find_cargo_tomls(dir: &std::path::Path) -> anyhow::Result<Vec<PathBuf>> {
    let mut manifests = Vec::new();
    if !dir.exists() {
        return Ok(manifests);
    }

    for entry in std::fs::read_dir(dir)? {
        let entry = entry?;
        let path = entry.path();
        if path.is_dir() {
            manifests.extend(find_cargo_tomls(&path)?);
        } else if path.file_name().is_some_and(|name| name == "Cargo.toml") {
            manifests.push(path);
        }
    }

    Ok(manifests)
}

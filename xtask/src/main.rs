//! Workspace automation tasks for Robot Native Engine.

use image::AnimationDecoder;
use std::io::BufReader;
use std::process::{Command, ExitCode, Stdio};
use std::{
    env, fs,
    path::{Path, PathBuf},
};

const HERO_CONTACT_SHEET_FRAMES: [usize; 9] = [0, 6, 12, 18, 24, 30, 36, 42, 47];

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
        "house-gif-demo" => house_gif_demo(),
        "hero-media-check" => hero_media_check(),
        "hero-contact-sheet" => hero_contact_sheet(),
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
    house_gif_demo()?;
    hero_media_check()?;
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
        "cargo run -p mobile_manipulator_place --example 26_mobile_manipulator_place -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_vectorized --example 28_mobile_manipulator_vectorized -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_curriculum --example 29_mobile_manipulator_curriculum -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_lift --example 30_mobile_manipulator_lift -- --smoke",
    )?;
    run_step(
        "cargo run -p mobile_manipulator_lift_pick_place --example 31_mobile_manipulator_lift_pick_place -- --smoke",
    )?;
    run_step("cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero -- --smoke")?;
    run_step(
        "cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator",
    )?;
    run_step(
        "cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator-mobile",
    )?;
    run_step(
        "cargo run -p interactive_viewer --example 14_interactive_viewer -- --smoke --manipulator-lift",
    )?;
    Ok(())
}

fn house_gif_demo() -> anyhow::Result<()> {
    let python = python_command()?;
    run_step(&format!(
        "{python} examples/27_mobile_manipulator_rl/house_gif_demo.py --check"
    ))?;
    Ok(())
}

fn hero_media_check() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let readme_path = root.join("README.md");
    let gif_path = root.join("docs/media/rne-hero.gif");
    let png_path = root.join("docs/media/rne-hero.png");
    let metadata_path = root.join("docs/media/rne-hero.json");
    let readme = fs::read_to_string(&readme_path)?;
    anyhow::ensure!(
        readme.contains("srcset=\"docs/media/rne-hero.png\""),
        "README hero reduced-motion poster does not point at docs/media/rne-hero.png"
    );
    anyhow::ensure!(
        readme.contains("<img src=\"docs/media/rne-hero.gif\""),
        "README first hero image does not point at docs/media/rne-hero.gif"
    );
    anyhow::ensure!(
        readme.contains(
            "3D RNE mobile manipulator simulation navigating a house-like room while reaching with its arm"
        ),
        "README hero alt text does not describe the 3D mobile manipulator simulation"
    );
    anyhow::ensure!(
        readme.contains("examples/32_lift_pick_place_hero")
            && readme.contains("docs/media/rne-hero.json"),
        "README hero caption does not link the 3D generator and metadata"
    );

    let gif = fs::read(&gif_path)?;
    anyhow::ensure!(gif.starts_with(b"GIF8"), "README hero GIF header mismatch");
    anyhow::ensure!(gif.ends_with(b";"), "README hero GIF trailer missing");
    anyhow::ensure!(
        gif.len() > 100_000,
        "README hero GIF is unexpectedly small: {} bytes",
        gif.len()
    );
    anyhow::ensure!(png_path.is_file(), "README hero PNG is missing");
    let metadata: serde_json::Value = serde_json::from_str(&fs::read_to_string(&metadata_path)?)?;
    let gif_progression = inspect_gif_frame_progression(&gif_path)?;
    anyhow::ensure!(
        metadata["artifact"].as_str() == Some("rne_3d_mobile_manipulator_navigation_reach_hero"),
        "README hero metadata does not describe the 3D navigation/reach hero"
    );
    let metadata_width = metadata["width"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing width"))?;
    let metadata_height = metadata["height"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing height"))?;
    let metadata_frame_count = metadata["frame_count"]
        .as_u64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing frame_count"))?;
    anyhow::ensure!(
        gif_progression.width == u32::try_from(metadata_width)?,
        "README hero metadata width does not match GIF: metadata={metadata_width}, gif={}",
        gif_progression.width
    );
    anyhow::ensure!(
        gif_progression.height == u32::try_from(metadata_height)?,
        "README hero metadata height does not match GIF: metadata={metadata_height}, gif={}",
        gif_progression.height
    );
    anyhow::ensure!(
        u64::try_from(gif_progression.frame_count)? == metadata_frame_count,
        "README hero metadata frame_count does not match GIF: metadata={metadata_frame_count}, gif={}",
        gif_progression.frame_count
    );
    anyhow::ensure!(
        metadata["source"]["kind"].as_str() == Some("wgpu_simulation")
            && metadata["source"]["generator"].as_str() == Some("examples/32_lift_pick_place_hero")
            && metadata["source"]["scene"].as_str()
                == Some("assets/scenes/mm_mobile.rne.scene.toml")
            && metadata["source"]["policy"].as_str() == Some("MobileReachHeroPolicy")
            && metadata["source"]["physics"].as_str() == Some("MobileManipulatorSim/Rapier"),
        "README hero metadata source is not wgpu_simulation"
    );
    let overlays = metadata["overlays"]
        .as_array()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata overlays must be an array"))?;
    anyhow::ensure!(
        overlays
            .iter()
            .any(|overlay| overlay.as_str() == Some("house_context"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("base_path"))
            && overlays
                .iter()
                .any(|overlay| overlay.as_str() == Some("reach_target")),
        "README hero metadata is missing expected 3D overlays"
    );
    let base_travel_m = metadata["simulation"]["base_travel_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing base_travel_m"))?;
    let ee_travel_m = metadata["simulation"]["ee_travel_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing ee_travel_m"))?;
    let final_ee_target_error_m = metadata["simulation"]["final_ee_target_error_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing final_ee_target_error_m"))?;
    let max_final_ee_target_error_m = metadata["simulation"]["max_final_ee_target_error_m"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing max_final_ee_target_error_m")
        })?;
    let min_consecutive_frame_delta_ratio = metadata["simulation"]
        ["min_consecutive_frame_delta_ratio"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing min_consecutive_frame_delta_ratio")
        })?;
    let first_last_frame_delta_ratio = metadata["simulation"]["first_last_frame_delta_ratio"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!("README hero metadata missing first_last_frame_delta_ratio")
        })?;
    let min_consecutive_frame_delta_ratio_threshold = metadata["simulation"]
        ["min_consecutive_frame_delta_ratio_threshold"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!(
                "README hero metadata missing min_consecutive_frame_delta_ratio_threshold"
            )
        })?;
    let min_first_last_frame_delta_ratio_threshold = metadata["simulation"]
        ["min_first_last_frame_delta_ratio_threshold"]
        .as_f64()
        .ok_or_else(|| {
            anyhow::anyhow!(
                "README hero metadata missing min_first_last_frame_delta_ratio_threshold"
            )
        })?;
    let max_base_height_error_m = metadata["simulation"]["max_base_height_error_m"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing max_base_height_error_m"))?;
    let min_base_yaw_only_dot = metadata["simulation"]["min_base_yaw_only_dot"]
        .as_f64()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing min_base_yaw_only_dot"))?;
    let trajectory_digest = metadata["simulation"]["trajectory_digest"]
        .as_str()
        .ok_or_else(|| anyhow::anyhow!("README hero metadata missing trajectory_digest"))?;
    anyhow::ensure!(
        base_travel_m > 0.20,
        "README hero simulation base travel is too small: {base_travel_m:.2} m"
    );
    anyhow::ensure!(
        ee_travel_m > 0.15,
        "README hero simulation end-effector travel is too small: {ee_travel_m:.2} m"
    );
    anyhow::ensure!(
        max_final_ee_target_error_m <= 0.05,
        "README hero reach target threshold is too loose: {max_final_ee_target_error_m:.3} m"
    );
    anyhow::ensure!(
        final_ee_target_error_m <= max_final_ee_target_error_m,
        "README hero manipulator does not reach the target: final_ee_target_error={final_ee_target_error_m:.3} m"
    );
    anyhow::ensure!(
        min_consecutive_frame_delta_ratio_threshold >= 0.01,
        "README hero frame-delta threshold is too loose: {min_consecutive_frame_delta_ratio_threshold:.4}"
    );
    anyhow::ensure!(
        min_first_last_frame_delta_ratio_threshold >= 0.08,
        "README hero first/last frame-delta threshold is too loose: {min_first_last_frame_delta_ratio_threshold:.4}"
    );
    anyhow::ensure!(
        min_consecutive_frame_delta_ratio >= min_consecutive_frame_delta_ratio_threshold,
        "README hero GIF has nearly frozen adjacent frames: min_consecutive_frame_delta_ratio={min_consecutive_frame_delta_ratio:.4}"
    );
    anyhow::ensure!(
        first_last_frame_delta_ratio >= min_first_last_frame_delta_ratio_threshold,
        "README hero GIF lacks visible progression: first_last_frame_delta_ratio={first_last_frame_delta_ratio:.4}"
    );
    anyhow::ensure!(
        gif_progression.min_consecutive_frame_delta_ratio
            >= min_consecutive_frame_delta_ratio_threshold,
        "README hero GIF bytes have nearly frozen adjacent frames: min_consecutive_frame_delta_ratio={:.4}",
        gif_progression.min_consecutive_frame_delta_ratio
    );
    anyhow::ensure!(
        gif_progression.first_last_frame_delta_ratio >= min_first_last_frame_delta_ratio_threshold,
        "README hero GIF bytes lack visible progression: first_last_frame_delta_ratio={:.4}",
        gif_progression.first_last_frame_delta_ratio
    );
    anyhow::ensure!(
        max_base_height_error_m <= 0.01,
        "README hero mobile base leaves the ground plane: max_base_height_error={max_base_height_error_m:.4} m"
    );
    anyhow::ensure!(
        min_base_yaw_only_dot >= 0.999_999,
        "README hero mobile base is not upright: min_base_yaw_only_dot={min_base_yaw_only_dot:.9}"
    );
    anyhow::ensure!(
        trajectory_digest.len() == 18
            && trajectory_digest.starts_with("0x")
            && trajectory_digest[2..]
                .chars()
                .all(|character| character.is_ascii_hexdigit()),
        "README hero trajectory_digest must be a 64-bit hex string"
    );
    let live_trajectory_digest = hero_simulation_smoke_digest()?;
    anyhow::ensure!(
        live_trajectory_digest == trajectory_digest,
        "README hero trajectory digest is stale: metadata={trajectory_digest}, live={live_trajectory_digest}"
    );
    anyhow::ensure!(
        metadata["simulation"]["final_base_m"]
            .as_array()
            .is_some_and(|items| items.len() == 3)
            && metadata["simulation"]["final_ee_m"]
                .as_array()
                .is_some_and(|items| items.len() == 3),
        "README hero metadata final simulation positions must be 3D vectors"
    );
    anyhow::ensure!(
        metadata["byte_size"].as_u64() == Some(u64::try_from(gif.len())?),
        "README hero metadata byte_size does not match GIF bytes"
    );
    println!(
        "README 3D hero media ok: gif={} bytes metadata={}",
        gif.len(),
        metadata_path.display()
    );
    Ok(())
}

fn hero_simulation_smoke_digest() -> anyhow::Result<String> {
    let command = "cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero -- --smoke";
    let output = run_step_output(command)?;
    extract_hero_digest(&output)
        .ok_or_else(|| anyhow::anyhow!("hero smoke output did not include trajectory digest"))
}

fn hero_contact_sheet() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let gif_path = root.join("docs/media/rne-hero.gif");
    anyhow::ensure!(
        gif_path.is_file(),
        "README hero GIF is missing at {}",
        gif_path.display()
    );

    let output_dir = root.join("target/hero-debug");
    fs::create_dir_all(&output_dir)?;
    let output_path = output_dir.join("contact.png");

    let filter = hero_contact_sheet_filter();
    let status = Command::new("ffmpeg")
        .arg("-y")
        .arg("-v")
        .arg("error")
        .arg("-i")
        .arg(&gif_path)
        .arg("-vf")
        .arg(filter)
        .arg(&output_path)
        .status()
        .map_err(|error| anyhow::anyhow!("ffmpeg is required for hero-contact-sheet: {error}"))?;

    if !status.success() {
        anyhow::bail!("ffmpeg failed while generating {}", output_path.display());
    }

    println!(
        "wrote README hero contact sheet to {}",
        output_path.display()
    );
    Ok(())
}

fn hero_contact_sheet_filter() -> String {
    let select_frames = HERO_CONTACT_SHEET_FRAMES
        .iter()
        .map(|frame| format!("eq(n,{frame})"))
        .collect::<Vec<_>>()
        .join("+");
    format!("select='{select_frames}',scale=320:-1,tile=3x3")
}

#[derive(Clone, Copy, Debug)]
struct GifFrameProgression {
    width: u32,
    height: u32,
    frame_count: usize,
    min_consecutive_frame_delta_ratio: f64,
    first_last_frame_delta_ratio: f64,
}

fn inspect_gif_frame_progression(path: &Path) -> anyhow::Result<GifFrameProgression> {
    let file = fs::File::open(path)?;
    let decoder = image::codecs::gif::GifDecoder::new(BufReader::new(file))?;
    let frames = decoder.into_frames();

    let mut width = 0;
    let mut height = 0;
    let mut frame_count = 0usize;
    let mut first_frame_rgba8 = Vec::new();
    let mut previous_frame_rgba8 = Vec::new();
    let mut min_consecutive_frame_delta_ratio = 1.0_f64;
    let mut first_last_frame_delta_ratio = 0.0_f64;

    for frame in frames {
        let frame = frame?;
        let buffer = frame.into_buffer();
        let (frame_width, frame_height) = buffer.dimensions();
        let rgba8 = buffer.into_raw();

        if frame_count == 0 {
            width = frame_width;
            height = frame_height;
            first_frame_rgba8.clone_from(&rgba8);
        } else {
            anyhow::ensure!(
                frame_width == width && frame_height == height,
                "README hero GIF frame dimensions changed at frame {frame_count}: expected {}x{}, got {}x{}",
                width,
                height,
                frame_width,
                frame_height
            );
            let delta_ratio = frame_delta_ratio(&previous_frame_rgba8, &rgba8)?;
            min_consecutive_frame_delta_ratio = min_consecutive_frame_delta_ratio.min(delta_ratio);
            first_last_frame_delta_ratio = frame_delta_ratio(&first_frame_rgba8, &rgba8)?;
        }

        previous_frame_rgba8 = rgba8;
        frame_count += 1;
    }

    anyhow::ensure!(frame_count > 0, "README hero GIF has no decoded frames");
    if frame_count == 1 {
        min_consecutive_frame_delta_ratio = 0.0;
    }

    Ok(GifFrameProgression {
        width,
        height,
        frame_count,
        min_consecutive_frame_delta_ratio,
        first_last_frame_delta_ratio,
    })
}

fn frame_delta_ratio(previous_rgba8: &[u8], current_rgba8: &[u8]) -> anyhow::Result<f64> {
    anyhow::ensure!(
        previous_rgba8.len() == current_rgba8.len(),
        "hero frame buffers must have identical byte lengths"
    );
    anyhow::ensure!(
        previous_rgba8.len().is_multiple_of(4),
        "hero frame buffer length must be RGBA8-aligned"
    );
    let pixel_count = previous_rgba8.len() / 4;
    if pixel_count == 0 {
        return Ok(0.0);
    }
    let changed_pixels = previous_rgba8
        .chunks_exact(4)
        .zip(current_rgba8.chunks_exact(4))
        .filter(|(previous, current)| previous != current)
        .count();
    Ok(changed_pixels as f64 / pixel_count as f64)
}

fn extract_hero_digest(output: &str) -> Option<String> {
    let marker = "digest=";
    let start = output.find(marker)? + marker.len();
    let digest = output[start..]
        .split(|character: char| !character.is_ascii_hexdigit() && character != 'x')
        .next()?;
    if digest.len() == 18
        && digest.starts_with("0x")
        && digest[2..]
            .chars()
            .all(|character| character.is_ascii_hexdigit())
    {
        Some(digest.to_string())
    } else {
        None
    }
}

fn python_command() -> anyhow::Result<&'static str> {
    for candidate in ["python", "python3"] {
        if let Ok(status) = Command::new(candidate)
            .arg("--version")
            .stdout(Stdio::null())
            .stderr(Stdio::null())
            .status()
        {
            if status.success() {
                return Ok(candidate);
            }
        }
    }
    anyhow::bail!("python or python3 is required for house-gif-demo")
}

fn validate_repo_assets() -> anyhow::Result<()> {
    let root = workspace_root()?;
    let scenes = [
        root.join("assets/scenes/episode_diff_drive.rne.scene.toml"),
        root.join("assets/scenes/mm_mobile.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal_grasp.rne.scene.toml"),
        root.join("assets/scenes/mm_minimal_transport.rne.scene.toml"),
        root.join("assets/scenes/mm_lift.rne.scene.toml"),
        root.join("assets/scenes/mm_lift_pick.rne.scene.toml"),
    ];
    let robots = [
        root.join("assets/robots/diff_drive.rne.robot.toml"),
        root.join("assets/robots/diff_drive_urdf.rne.robot.toml"),
        root.join("assets/robots/mm_minimal.rne.robot.toml"),
        root.join("assets/robots/mm_mobile.rne.robot.toml"),
        root.join("assets/robots/mm_lift.rne.robot.toml"),
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

fn run_step_output(command: &str) -> anyhow::Result<String> {
    println!("$ {command}");
    let output = if cfg!(windows) {
        Command::new("cmd").args(["/C", command]).output()?
    } else {
        Command::new("sh").arg("-c").arg(command).output()?
    };

    let stdout = String::from_utf8_lossy(&output.stdout);
    let stderr = String::from_utf8_lossy(&output.stderr);
    print!("{stdout}");
    eprint!("{stderr}");

    if output.status.success() {
        Ok(format!("{stdout}\n{stderr}"))
    } else {
        anyhow::bail!("command failed with status {}", output.status);
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

#[cfg(test)]
mod tests {
    use super::{extract_hero_digest, frame_delta_ratio, hero_contact_sheet_filter};

    #[test]
    fn extracts_hero_digest_from_smoke_output() {
        let output = "3D hero simulation smoke ok: digest=0xd85cd8fbdbce1cb9, base_travel=4.51 m";
        assert_eq!(
            extract_hero_digest(output).as_deref(),
            Some("0xd85cd8fbdbce1cb9")
        );
    }

    #[test]
    fn rejects_missing_or_malformed_hero_digest() {
        assert_eq!(extract_hero_digest("3D hero simulation smoke ok"), None);
        assert_eq!(extract_hero_digest("digest=d85cd8fbdbce1cb9"), None);
        assert_eq!(extract_hero_digest("digest=0xd85cd8fbdbce1cb"), None);
    }

    #[test]
    fn builds_hero_contact_sheet_filter() {
        assert_eq!(
            hero_contact_sheet_filter(),
            "select='eq(n,0)+eq(n,6)+eq(n,12)+eq(n,18)+eq(n,24)+eq(n,30)+eq(n,36)+eq(n,42)+eq(n,47)',scale=320:-1,tile=3x3"
        );
    }

    #[test]
    fn computes_frame_delta_ratio_per_pixel() {
        let previous = [0, 0, 0, 255, 10, 10, 10, 255];
        let current = [0, 0, 0, 255, 11, 10, 10, 255];

        assert_eq!(frame_delta_ratio(&previous, &current).unwrap(), 0.5);
        assert_eq!(frame_delta_ratio(&previous, &previous).unwrap(), 0.0);
    }

    #[test]
    fn rejects_frame_delta_length_mismatch() {
        assert!(frame_delta_ratio(&[0, 0, 0, 255], &[0, 0, 0]).is_err());
    }
}

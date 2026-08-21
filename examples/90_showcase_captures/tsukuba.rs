//! Tsukuba Challenge full-run showcase source and capture.

use super::media::{
    capture_frames, push_box, push_cylinder, push_sphere, CameraEvidence, CaptureFrame,
    ShowcaseMetadata, SimulationEvidence, FRAME_COUNT,
};
use anyhow::{Context, Result};
use rne_ai::{build_visual_render_scene, BehaviorScenario, TsukubaFullRunScenario};
use rne_assets::load_and_spawn_scene;
use rne_ecs::World;
use rne_math::{Quat, Vec3};
use rne_physics::hash_physics_state;
use rne_plateau::{import_citygml_file, CoordinateMode, ImportOptions};
use rne_render::RenderScene;
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "tsukuba";
const SUBJECT: &str = "Tsukuba Challenge sidewalk robot";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: -0.68,
    pitch_rad: 1.05,
    distance_m: 7.5,
};

/// Run the Tsukuba headless source twice and return evidence plus an optional
/// GPU capture. The 79 full-run scenario remains the authoritative state.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let plateau_backdrop = load_plateau_backdrop(repo_root)?;
    let first = rollout(false, None, None)?;
    let replay = rollout(false, None, None)?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "Tsukuba replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    let evidence = SimulationEvidence {
        scenario: "TsukubaFullRunScenario::success (79_tsukuba_full_run)",
        steps: first.steps,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: "full_run_complete=true; three_stop_lines=true; signal_waits=true".into(),
    };
    let capture_evidence = if capture {
        let captured = rollout(true, Some(first.steps), Some(&plateau_backdrop))?;
        let orbit = CameraOrbit {
            focus: Vec3::new(5.15, 0.28, -0.15),
            yaw_rad: -0.68,
            pitch_rad: 1.05,
            distance_m: 7.5,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &captured.frames,
            orbit,
            [0.055, 0.075, 0.105, 1.0],
            FRAME_COUNT / 2,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "The blue diff-drive actor proxy and signal state are rebuilt from TsukubaFullRunScenario post-step observation; PLATEAU remains visual-only.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/tsukuba_full_run.rne.scene.toml",
            "assets/robots/tsukuba_confirmation.rne.robot.toml",
            "assets/environments/tsukuba_plateau_backdrop.rne.env.toml",
            "crates/rne_plateau/tests/fixtures/plateau_lod1_minimal.gml",
            "crates/rne_ai/src/env/tsukuba_full_run.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment tsukuba",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment tsukuba",
    };
    if capture {
        write_metadata(repo_root, &metadata)?;
    }
    Ok(metadata)
}

struct Rollout {
    steps: u64,
    initial_digest: u64,
    final_digest: u64,
    frames: Vec<CaptureFrame>,
}

fn rollout(
    capture: bool,
    expected_steps: Option<u64>,
    plateau_backdrop: Option<&RenderScene>,
) -> Result<Rollout> {
    let mut scenario = TsukubaFullRunScenario::success(1).context("load Tsukuba full-run")?;
    let initial_digest = hash_physics_state(scenario.simulation().world());
    let mut frames = Vec::new();
    let mut sample_steps = Vec::new();
    if capture {
        let total = expected_steps.context("capture needs headless Tsukuba step count")?;
        sample_steps = (1..=FRAME_COUNT)
            .map(|index| ((index as u64 * total).div_ceil(FRAME_COUNT as u64)).max(1))
            .collect();
    }
    let mut sample_index = 0;
    loop {
        let step = scenario.advance();
        let observation = step.observation;
        if capture
            && sample_index < sample_steps.len()
            && observation.step >= sample_steps[sample_index]
        {
            frames.push(CaptureFrame {
                step: observation.step,
                phase: if observation.signal_green {
                    "green-crossing".into()
                } else if observation.stopped {
                    "stop-line-wait".into()
                } else {
                    "sidewalk-cruise".into()
                },
                scene: render_scene(&scenario, observation, plateau_backdrop),
            });
            sample_index += 1;
        }
        if step.done {
            break;
        }
    }
    anyhow::ensure!(
        !capture || frames.len() == FRAME_COUNT,
        "Tsukuba capture sampled {} of {} frames",
        frames.len(),
        FRAME_COUNT
    );
    let final_observation = scenario.current_observation();
    anyhow::ensure!(
        final_observation.full_run_complete,
        "Tsukuba full run did not complete: {final_observation:?}"
    );
    Ok(Rollout {
        steps: final_observation.step,
        initial_digest,
        final_digest: hash_physics_state(scenario.simulation().world()),
        frames,
    })
}

fn render_scene(
    scenario: &TsukubaFullRunScenario,
    observation: rne_ai::TsukubaFullRunObservation,
    plateau_backdrop: Option<&RenderScene>,
) -> RenderScene {
    let mut scene = plateau_backdrop.cloned().unwrap_or_default();
    let mut foreground = build_visual_render_scene(scenario.simulation().world());
    scene.items.append(&mut foreground.items);
    // The built-in diff-drive visual is intentionally supplemented with a
    // high-contrast body proxy so the moving actor remains identifiable in a
    // city-scale 960x540 wide shot.
    push_box(
        &mut scene,
        Vec3::new(
            observation.base_x_m,
            observation.base_y_m,
            observation.base_z_m,
        ),
        Vec3::new(0.52, 0.34, 0.42),
        [0.08, 0.42, 0.82, 1.0],
    );
    push_box(
        &mut scene,
        Vec3::new(
            observation.base_x_m + 0.18 * observation.base_yaw_rad.cos(),
            observation.base_y_m + 0.21,
            observation.base_z_m + 0.18 * observation.base_yaw_rad.sin(),
        ),
        Vec3::new(0.24, 0.18, 0.24),
        [0.10, 0.72, 0.90, 1.0],
    );
    // Lane edge, crossing stripes, and a goal arch make the official geometry
    // readable even in a small poster while leaving physics untouched.
    for line_x_m in [2.5, 5.0, 7.5] {
        push_box(
            &mut scene,
            Vec3::new(line_x_m, 0.035, 0.0),
            Vec3::new(0.045, 0.02, 2.0),
            [0.98, 0.98, 0.94, 1.0],
        );
        push_cylinder(
            &mut scene,
            Vec3::new(line_x_m, 0.74, -0.88),
            0.035,
            1.45,
            Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
            [0.13, 0.15, 0.18, 1.0],
        );
        push_sphere(
            &mut scene,
            Vec3::new(line_x_m, 1.45, -0.88),
            0.09,
            if observation.signal_green {
                [0.10, 0.90, 0.35, 1.0]
            } else {
                [0.90, 0.12, 0.08, 1.0]
            },
        );
        // Orange traffic cones at each crossing are a visual-only overlay;
        // their deterministic positions make the stop-line action obvious.
        push_cylinder(
            &mut scene,
            Vec3::new(line_x_m - 0.34, 0.11, 0.76),
            0.10,
            0.22,
            Quat::IDENTITY,
            [0.95, 0.32, 0.05, 1.0],
        );
    }
    push_box(
        &mut scene,
        Vec3::new(10.0, 0.55, 0.0),
        Vec3::new(0.08, 1.1, 2.0),
        [0.12, 0.78, 0.38, 1.0],
    );
    scene
}

fn load_plateau_backdrop(repo_root: &Path) -> Result<RenderScene> {
    let manifest_path = repo_root.join("assets/environments/tsukuba_plateau_backdrop.rne.env.toml");
    let manifest_text = fs::read_to_string(&manifest_path)
        .with_context(|| format!("read {}", manifest_path.display()))?;
    let manifest: toml::Value =
        toml::from_str(&manifest_text).context("parse Tsukuba backdrop manifest")?;
    let fixture_relative = manifest
        .get("plateau_fixture_gml")
        .and_then(toml::Value::as_str)
        .context("backdrop manifest plateau_fixture_gml")?;
    let translation = manifest
        .get("backdrop_translation_m")
        .and_then(toml::Value::as_array)
        .context("backdrop manifest translation")?
        .iter()
        .map(|value| value.as_float().context("backdrop translation value"))
        .collect::<Result<Vec<_>>>()?;
    anyhow::ensure!(
        translation.len() == 3,
        "backdrop translation must have three values"
    );
    let fixture = manifest_path
        .parent()
        .expect("backdrop manifest parent")
        .join(fixture_relative);
    anyhow::ensure!(
        fixture.is_file(),
        "missing PLATEAU fixture {}",
        fixture.display()
    );
    let import_dir = std::env::var_os("CARGO_TARGET_DIR")
        .map(std::path::PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
        .join("rne-showcase-tsukuba-plateau");
    if import_dir.exists() {
        fs::remove_dir_all(&import_dir).context("remove stale Tsukuba PLATEAU import")?;
    }
    fs::create_dir_all(&import_dir).context("create Tsukuba PLATEAU import")?;
    let imported = import_citygml_file(
        &fixture,
        &import_dir,
        &ImportOptions {
            tile_name: "showcase-tsukuba-plateau".into(),
            coordinate_mode: CoordinateMode::GeographicDegrees,
            world_seed: 90,
            ..ImportOptions::default()
        },
    )
    .context("import PLATEAU Tsukuba fixture")?;
    anyhow::ensure!(
        imported.building_count >= 1,
        "PLATEAU backdrop has no buildings"
    );
    anyhow::ensure!(imported.road_count >= 1, "PLATEAU backdrop has no roads");
    let mut world = World::new();
    load_and_spawn_scene(&mut world, &imported.scene_path).context("spawn PLATEAU backdrop")?;
    let mut scene = build_visual_render_scene(&world);
    let translation = Vec3::new(translation[0], translation[1], translation[2]);
    for item in &mut scene.items {
        item.transform.translation = translation + item.transform.translation;
    }
    println!(
        "Tsukuba PLATEAU backdrop: buildings={} roads={} triangles={} items={}",
        imported.building_count,
        imported.road_count,
        imported.triangle_count,
        scene.items.len()
    );
    Ok(scene)
}

fn write_metadata(repo_root: &Path, metadata: &ShowcaseMetadata) -> Result<()> {
    let path = repo_root.join("docs/media/showcase-tsukuba.json");
    fs::write(&path, to_vec_pretty(metadata)?)
        .with_context(|| format!("write {}", path.display()))?;
    Ok(())
}

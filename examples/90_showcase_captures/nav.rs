//! Office AGV navigation showcase: the real office environment with a planned
//! route, the driven trajectory, and the docking goal drawn as render-only
//! overlays on top of the shared office scene.

use super::media::{
    capture_frames, push_box, push_sphere, CameraEvidence, CaptureFrame, ShowcaseMetadata,
    SimulationEvidence, FRAME_COUNT,
};
use super::office;
use anyhow::{Context, Result};
use rne_ai::{BehaviorScenario, OfficeAgvDeskPlaceScenario};
use rne_math::Vec3;
use rne_physics::hash_physics_state;
use rne_render::RenderScene;
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "nav";
const SUBJECT: &str = "office AGV navigation: planned route, driven trajectory, and docking goal";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.32,
    pitch_rad: 0.78,
    distance_m: 6.4,
};

/// Runs the dock-to-desk office mission and captures the AGV in the office with
/// navigation overlays.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(false, None)?;
    let replay = rollout(false, Some(first.steps))?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "Nav replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    let evidence = SimulationEvidence {
        scenario: "OfficeAgvDeskPlaceScenario::success (86_office_agv_desk_place)",
        steps: first.steps,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: "mission_complete=true; route planned; goal docked; replay deterministic".into(),
    };
    let capture_evidence = if capture {
        let captured = rollout(true, Some(first.steps))?;
        let orbit = CameraOrbit {
            focus: Vec3::new(4.65, 0.55, 0.0),
            yaw_rad: CAMERA.yaw_rad,
            pitch_rad: CAMERA.pitch_rad,
            distance_m: CAMERA.distance_m,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &captured.frames,
            orbit,
            [0.32, 0.36, 0.42, 1.0],
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
        visual_state_sync: "Office environment and AGV come from the shared office scene; the route, trajectory, and goal are render-only navigation overlays.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/office_agv_delivery.rne.scene.toml",
            "crates/rne_ai/src/env/office_agv_desk_place.rs",
            "examples/90_showcase_captures/nav.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment nav",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment nav",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-nav.json");
        fs::write(&path, to_vec_pretty(&metadata)?)
            .with_context(|| format!("write {}", path.display()))?;
    }
    Ok(metadata)
}

struct Rollout {
    steps: u64,
    initial_digest: u64,
    final_digest: u64,
    frames: Vec<CaptureFrame>,
}

fn rollout(capture: bool, expected_steps: Option<u64>) -> Result<Rollout> {
    let mut scenario = OfficeAgvDeskPlaceScenario::success(1).context("load office desk-place")?;
    let initial_digest = hash_physics_state(scenario.simulation().world());
    let mut frames = Vec::new();
    let mut trajectory: Vec<(f64, f64)> = Vec::new();
    let mut sample_steps = Vec::new();
    if capture {
        let total = expected_steps.unwrap_or(1);
        anyhow::ensure!(
            expected_steps.is_some(),
            "nav capture needs discovered step count"
        );
        sample_steps = (1..=FRAME_COUNT)
            .map(|index| ((index as u64 * total).div_ceil(FRAME_COUNT as u64)).max(1))
            .collect();
    }
    let mut sample_index = 0;
    loop {
        let step = scenario.advance();
        let observation = step.observation;
        trajectory.push((observation.base_x_m, observation.base_z_m));
        if capture
            && sample_index < sample_steps.len()
            && observation.step >= sample_steps[sample_index]
        {
            let mut scene = office::render_scene(&scenario, observation);
            append_navigation_overlays(&mut scene, &trajectory, observation.base_yaw_rad);
            frames.push(CaptureFrame {
                step: observation.step,
                phase: office::phase_name(observation),
                scene,
            });
            sample_index += 1;
        }
        if step.done {
            break;
        }
    }
    let final_observation = scenario.current_observation();
    anyhow::ensure!(
        final_observation.mission_complete,
        "office desk-place mission did not complete: {final_observation:?}"
    );
    anyhow::ensure!(
        !capture || frames.len() == FRAME_COUNT,
        "nav capture sampled {} of {} frames",
        frames.len(),
        FRAME_COUNT
    );
    Ok(Rollout {
        steps: final_observation.step,
        initial_digest,
        final_digest: hash_physics_state(scenario.simulation().world()),
        frames,
    })
}

/// Adds a planned route, the driven trajectory, a heading arrow, and the
/// docking goal to the office scene.
fn append_navigation_overlays(
    scene: &mut RenderScene,
    trajectory: &[(f64, f64)],
    base_yaw_rad: f64,
) {
    const ROUTE: [f32; 4] = [0.98, 0.78, 0.10, 1.0];
    const TRAIL: [f32; 4] = [0.10, 0.85, 0.95, 1.0];
    const GOAL: [f32; 4] = [0.15, 0.95, 0.45, 1.0];
    const ARROW: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

    // Planned route: a line of markers along the aisle from the dock to the desk.
    let mut x = 2.0;
    while x <= 7.2 {
        push_sphere(scene, Vec3::new(x, 0.06, 0.0), 0.05, ROUTE);
        x += 0.35;
    }
    // Docking goal ring at the desk.
    for (dx, dz) in [(-0.45, 0.0), (0.45, 0.0), (0.0, -0.45), (0.0, 0.45)] {
        push_sphere(scene, Vec3::new(6.5 + dx, 0.09, dz), 0.06, GOAL);
    }
    // Driven trajectory history.
    for (index, (x, z)) in trajectory.iter().enumerate() {
        if index % 6 == 0 {
            push_sphere(scene, Vec3::new(*x, 0.34, *z), 0.05, TRAIL);
        }
    }
    // Heading arrow in front of the AGV.
    if let Some((x, z)) = trajectory.last() {
        let forward = Vec3::new(base_yaw_rad.cos(), 0.0, base_yaw_rad.sin());
        let tip = Vec3::new(*x, 0.34, *z) + forward * 0.55;
        push_box(scene, tip, Vec3::new(0.18, 0.06, 0.08), ARROW);
    }
}

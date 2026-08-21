//! Office AGV desk-place showcase source and capture.

use super::media::{
    capture_frames, push_box, push_box_material, CameraEvidence, CaptureFrame, ShowcaseMetadata,
    SimulationEvidence, FRAME_COUNT,
};
use anyhow::{Context, Result};
use rne_ai::{
    build_visual_render_scene, BehaviorScenario, OfficeAgvDeskPlaceObservation,
    OfficeAgvDeskPlaceScenario,
};
use rne_math::{Quat, Vec3};
use rne_physics::hash_physics_state;
use rne_render::{PbrMaterial, RenderScene, VisualShape};
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "office";
const SUBJECT: &str = "office AGV shared-aisle desk place";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.0,
    pitch_rad: 1.30,
    distance_m: 5.8,
};

/// Run the 86 desk-place scenario and capture the actual ego, oncoming AGV,
/// and cargo-proxy state after each selected fixed-step observation.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(false, None)?;
    let replay = rollout(false, Some(first.steps))?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "Office replay digest mismatch: {:#x} != {:#x}",
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
        outcome: "mission_complete=true; yielded=true; dock_pickup=true; desk_place=true".into(),
    };
    let capture_evidence = if capture {
        let captured = rollout(true, Some(first.steps))?;
        let orbit = CameraOrbit {
            focus: Vec3::new(4.65, 0.38, 0.0),
            yaw_rad: CAMERA.yaw_rad,
            pitch_rad: CAMERA.pitch_rad,
            distance_m: CAMERA.distance_m,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &captured.frames,
            orbit,
            [0.075, 0.085, 0.105, 1.0],
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
        visual_state_sync: "Ego AGV meshes, oncoming AGV proxy, and cargo proxy are rebuilt from OfficeAgvDeskPlaceScenario post-step observation/state.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/office_agv_delivery.rne.scene.toml",
            "assets/robots/office_agv_delivery.rne.robot.toml",
            "crates/rne_ai/src/env/office_agv_desk_place.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment office",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment office",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-office.json");
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
    let mut sample_steps = Vec::new();
    if capture {
        let total = expected_steps.unwrap_or(1);
        // A first pass is used by `run` to discover the actual fixed-step
        // completion point; capture itself is called with that exact value.
        anyhow::ensure!(
            expected_steps.is_some(),
            "office capture needs discovered step count"
        );
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
                phase: phase_name(observation),
                scene: render_scene(&scenario, observation),
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
        "office capture sampled {} of {} frames",
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

fn phase_name(observation: OfficeAgvDeskPlaceObservation) -> String {
    if observation.desk_place_complete {
        "desk-place-complete".into()
    } else if observation.cargo_loaded && observation.desk_delivery_complete {
        "unload-at-desk".into()
    } else if observation.yielded_for_shared_aisle {
        "aisle-cleared-delivery".into()
    } else if observation.shared_aisle_occupied {
        "yield-for-oncoming-agv".into()
    } else {
        "drive-to-pickup-dock".into()
    }
}

fn render_scene(
    scenario: &OfficeAgvDeskPlaceScenario,
    observation: OfficeAgvDeskPlaceObservation,
) -> RenderScene {
    let mut scene = build_visual_render_scene(scenario.simulation().world());
    // The authored walls are collision boundaries, but a low eye-level
    // showcase camera would hide every actor behind them. Keep the floor and
    // desk while opening the render-only aisle for a readable wide shot.
    scene.items.retain(|item| {
        !matches!(
            item.shape,
            VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z < 0.2
        )
    });
    // Extend the authored corridor floor toward the close camera. The source
    // scene deliberately stops at the south wall; the extension keeps the
    // lower half of the poster an office aisle instead of a background void.
    push_box(
        &mut scene,
        Vec3::new(4.65, -0.035, 2.25),
        Vec3::new(6.0, 0.05, 4.5),
        [0.76, 0.74, 0.70, 1.0],
    );
    let (cargo_x_m, cargo_z_m) = scenario.cargo_translation_m();
    push_box_material(
        &mut scene,
        Vec3::new(observation.base_x_m, 0.24, observation.base_z_m),
        Vec3::new(0.52, 0.34, 0.42),
        Quat::from_rotation_y(observation.base_yaw_rad),
        [0.92, 0.30, 0.08, 1.0],
        PbrMaterial::new([0.92, 0.30, 0.08, 1.0], 0.34, 0.55, [0.0; 3]),
    );
    // The oncoming AGV and cargo are intentionally render-only proxies whose
    // transforms are copied from the scenario observation on every frame.
    push_box_material(
        &mut scene,
        Vec3::new(observation.other_agv_x_m, 0.24, 0.0),
        Vec3::new(0.50, 0.32, 0.40),
        Quat::IDENTITY,
        [0.10, 0.34, 0.70, 1.0],
        PbrMaterial::new([0.10, 0.34, 0.70, 1.0], 0.34, 0.55, [0.0; 3]),
    );
    push_box(
        &mut scene,
        Vec3::new(cargo_x_m, 0.37, cargo_z_m),
        Vec3::new(0.20, 0.20, 0.20),
        if observation.cargo_loaded {
            [0.95, 0.50, 0.06, 1.0]
        } else {
            [0.20, 0.74, 0.82, 1.0]
        },
    );
    // Desk shelving, aisle dividers, and a destination halo make the task
    // legible from the single fixed camera without adding physics entities.
    for x_m in [3.4, 4.7, 5.8] {
        push_box(
            &mut scene,
            Vec3::new(x_m, 0.65, -0.92),
            Vec3::new(0.72, 1.3, 0.10),
            [0.30, 0.36, 0.43, 1.0],
        );
    }
    // Far-side office wall and ceiling fixtures remove the empty sky band in
    // the poster while keeping the driving aisle open to the camera.
    push_box(
        &mut scene,
        Vec3::new(4.6, 1.80, -1.08),
        Vec3::new(5.9, 3.60, 0.08),
        [0.34, 0.40, 0.48, 1.0],
    );
    for x_m in [2.9, 4.4, 5.9, 7.2] {
        push_box(
            &mut scene,
            Vec3::new(x_m, 1.58, -1.02),
            Vec3::new(0.72, 0.10, 0.04),
            [0.88, 0.92, 0.86, 1.0],
        );
        push_box(
            &mut scene,
            Vec3::new(x_m, 1.28, -1.035),
            Vec3::new(0.56, 0.34, 0.025),
            [0.08, 0.34, 0.48, 1.0],
        );
    }
    for x_m in [2.0, 2.8, 3.6, 4.4, 5.2, 6.0] {
        push_box(
            &mut scene,
            Vec3::new(x_m, 0.028, 0.0),
            Vec3::new(0.42, 0.012, 0.035),
            [0.92, 0.69, 0.10, 1.0],
        );
    }
    // Yield line at the shared aisle and a desk-top/monitor silhouette at the
    // destination make the mission semantics readable without text labels.
    push_box(
        &mut scene,
        Vec3::new(3.45, 0.035, 0.0),
        Vec3::new(0.06, 0.025, 1.60),
        [0.96, 0.70, 0.08, 1.0],
    );
    push_box(
        &mut scene,
        Vec3::new(7.40, 0.86, 0.0),
        Vec3::new(0.95, 0.08, 1.35),
        [0.40, 0.25, 0.15, 1.0],
    );
    push_box(
        &mut scene,
        Vec3::new(7.28, 1.18, 0.0),
        Vec3::new(0.06, 0.42, 0.42),
        [0.08, 0.22, 0.30, 1.0],
    );
    push_box_material(
        &mut scene,
        Vec3::new(6.5, 0.08, 0.0),
        Vec3::new(0.75, 0.03, 0.75),
        Quat::IDENTITY,
        [0.10, 0.82, 0.42, 0.92],
        PbrMaterial::new([0.10, 0.82, 0.42, 0.92], 0.42, 0.10, [0.0; 3]),
    );
    scene
}

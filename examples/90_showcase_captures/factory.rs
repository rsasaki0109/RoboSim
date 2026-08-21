//! Unitree G1 factory-inspection showcase source and capture.

use super::media::{
    capture_frames, push_box, push_box_material, push_cylinder, CameraEvidence, CaptureFrame,
    ShowcaseMetadata, SimulationEvidence,
};
use anyhow::{Context, Result};
use rne_ai::{
    build_visual_render_scene, Episode, UnitreeG1InspectionAction, UnitreeG1InspectionEpisode,
    UnitreeG1InspectionEpisodeConfig,
};
use rne_math::{Quat, Vec3};
use rne_physics::hash_physics_state;
use rne_render::{MeshRenderCache, PbrMaterial, RenderScene};
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "factory";
const SUBJECT: &str = "Unitree G1 inspection route";
const CAPTURE_STEPS: u64 = 270;
const CAPTURE_FRAME_COUNT: usize = 54;
const CAPTURE_STRIDE: u64 = CAPTURE_STEPS / CAPTURE_FRAME_COUNT as u64;
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: -0.72,
    pitch_rad: 1.18,
    distance_m: 2.75,
};

/// Run the real UnitreeG1InspectionEpisode for the complete three-marker route
/// (270 fixed steps), then optionally render evenly sampled post-step states
/// with wgpu.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(capture)?;
    let replay = rollout(false)?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "Factory replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    anyhow::ensure!(
        first.steps == CAPTURE_STEPS,
        "factory source must run exactly {CAPTURE_STEPS} steps"
    );
    anyhow::ensure!(
        first.terminated && !first.truncated,
        "factory source must terminate successfully without truncation"
    );
    anyhow::ensure!(
        first.completed_markers == 3 && first.marker_count == 3,
        "factory source must complete 3/3 inspection markers"
    );
    if capture {
        anyhow::ensure!(
            first.frames.len() == CAPTURE_FRAME_COUNT,
            "factory capture must contain exactly {CAPTURE_FRAME_COUNT} sampled frames"
        );
    }
    let evidence = SimulationEvidence {
        scenario: "UnitreeG1InspectionEpisode (270 fixed steps; 3 markers)",
        steps: first.steps,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: format!(
            "terminated={}; completed_markers={}/{}; official_g1_meshes={}",
            first.terminated, first.completed_markers, first.marker_count, first.mesh_items
        ),
    };
    let capture_evidence = if capture {
        let orbit = CameraOrbit {
            focus: Vec3::new(0.10, 0.80, -0.25),
            yaw_rad: CAMERA.yaw_rad,
            pitch_rad: CAMERA.pitch_rad,
            distance_m: CAMERA.distance_m,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &first.frames,
            orbit,
            [0.040, 0.055, 0.075, 1.0],
            first.frames.len() / 2,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "Official G1 link meshes are rebuilt from UnitreeG1InspectionEpisode::simulation().world() after each fixed step.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/unitree_g1_factory.rne.scene.toml",
            "assets/robots/unitree_g1_dynamic.rne.robot.toml",
            "crates/rne_ai/src/env/urdf_scene/unitree_g1_inspection_episode.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment factory",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment factory",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-factory.json");
        fs::write(&path, to_vec_pretty(&metadata)?)
            .with_context(|| format!("write {}", path.display()))?;
    }
    Ok(metadata)
}

struct Rollout {
    steps: u64,
    initial_digest: u64,
    final_digest: u64,
    completed_markers: usize,
    marker_count: usize,
    mesh_items: usize,
    terminated: bool,
    truncated: bool,
    frames: Vec<CaptureFrame>,
}

fn rollout(capture: bool) -> Result<Rollout> {
    let mut episode = UnitreeG1InspectionEpisode::new(UnitreeG1InspectionEpisodeConfig::default())
        .context("load Unitree G1 factory inspection episode")?;
    let initial_digest = hash_physics_state(episode.simulation().world());
    let mut frames = Vec::new();
    let mut mesh_cache = MeshRenderCache::new();
    let mut mesh_items = 0;
    let mut terminal_step = None;
    for step_index in 1..=CAPTURE_STEPS {
        let step = episode.step(UnitreeG1InspectionAction { advance: true });
        if capture && (step_index % CAPTURE_STRIDE == 0 || step.terminated || step.truncated) {
            let (scene, current_mesh_items) = render_scene(&episode, &mut mesh_cache)?;
            mesh_items = mesh_items.max(current_mesh_items);
            frames.push(CaptureFrame {
                step: step_index,
                phase: capture_phase(step_index, step.observation, step.terminated),
                scene,
            });
        }
        if step.terminated || step.truncated {
            terminal_step = Some(step);
            break;
        }
    }
    let terminal_step = terminal_step.context("factory episode did not reach a terminal step")?;
    let final_observation = terminal_step.observation;
    anyhow::ensure!(
        terminal_step.terminated && !terminal_step.truncated,
        "factory terminal EpisodeStep must be terminated=true and truncated=false"
    );
    anyhow::ensure!(
        final_observation.completed_markers == 3 && final_observation.marker_count == 3,
        "factory terminal observation must report completed_markers=3 and marker_count=3"
    );
    if !capture {
        let (_, current_mesh_items) = render_scene(&episode, &mut mesh_cache)?;
        mesh_items = current_mesh_items;
    }
    anyhow::ensure!(
        mesh_items >= 20,
        "factory render resolved only {mesh_items} G1 mesh items"
    );
    Ok(Rollout {
        steps: episode.step_in_episode(),
        initial_digest,
        final_digest: hash_physics_state(episode.simulation().world()),
        completed_markers: final_observation.completed_markers,
        marker_count: final_observation.marker_count,
        mesh_items,
        terminated: terminal_step.terminated,
        truncated: terminal_step.truncated,
        frames,
    })
}

fn capture_phase(
    step_index: u64,
    observation: rne_ai::UnitreeG1InspectionObservation,
    terminated: bool,
) -> String {
    let marker_count = observation.marker_count.max(1) as u64;
    let marker_span = (CAPTURE_STEPS / marker_count).max(1);
    let marker_number = ((step_index.saturating_sub(1) / marker_span) + 1).min(marker_count);
    let step_in_marker = ((step_index.saturating_sub(1) % marker_span) + 1).min(marker_span);
    let phase = if terminated || step_in_marker == marker_span {
        "complete"
    } else if observation.gesture_progress > 0.01 {
        "inspect"
    } else {
        "approach"
    };
    format!("marker-{marker_number}-{phase}")
}

fn render_scene(
    episode: &UnitreeG1InspectionEpisode,
    cache: &mut MeshRenderCache,
) -> Result<(RenderScene, usize)> {
    let simulation = episode.simulation();
    let mut scene = build_visual_render_scene(simulation.world());
    let roots = simulation.mesh_package_roots().to_vec();
    let root_refs = roots
        .iter()
        .map(std::path::PathBuf::as_path)
        .collect::<Vec<_>>();
    cache
        .resolve_scene(&mut scene, &root_refs)
        .map_err(|error| anyhow::anyhow!("resolve official G1/factory meshes: {error}"))?;
    let mesh_items = scene
        .items
        .iter()
        .filter(|item| item.mesh.is_some())
        .count();
    // Render-only factory dressing keeps the G1 silhouette separated from the
    // dark rack and station without changing collision or task state.
    push_box(
        &mut scene,
        Vec3::new(0.0, 1.55, -1.17),
        Vec3::new(3.3, 0.08, 0.08),
        [0.20, 0.28, 0.34, 1.0],
    );
    for x_m in [-1.25, 0.0, 1.25] {
        push_cylinder(
            &mut scene,
            Vec3::new(x_m, 1.35, -1.08),
            0.025,
            2.6,
            Quat::IDENTITY,
            [0.26, 0.32, 0.38, 1.0],
        );
    }
    push_box_material(
        &mut scene,
        Vec3::new(0.2, 0.02, 0.42),
        Vec3::new(2.8, 0.035, 0.035),
        Quat::IDENTITY,
        [0.95, 0.62, 0.06, 1.0],
        PbrMaterial::new([0.95, 0.62, 0.06, 1.0], 0.36, 0.5, [0.0; 3]),
    );
    Ok((scene, mesh_items))
}

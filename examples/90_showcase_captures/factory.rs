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
const CAPTURE_STEPS: u64 = 40;
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: -0.72,
    pitch_rad: 1.18,
    distance_m: 2.75,
};

/// Run the real UnitreeG1InspectionEpisode for the requested forty fixed
/// steps, then optionally render those exact post-step states with wgpu.
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
        "factory source must run exactly 40 steps"
    );
    let evidence = SimulationEvidence {
        scenario: "UnitreeG1InspectionEpisode (40 fixed steps)",
        steps: first.steps,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: format!(
            "inspection_progress={:.3}; marker={}/{}; official_g1_meshes={}",
            first.gesture_progress, first.completed_markers, first.marker_count, first.mesh_items
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
    gesture_progress: f64,
    completed_markers: usize,
    marker_count: usize,
    mesh_items: usize,
    frames: Vec<CaptureFrame>,
}

fn rollout(capture: bool) -> Result<Rollout> {
    let mut episode = UnitreeG1InspectionEpisode::new(UnitreeG1InspectionEpisodeConfig::default())
        .context("load Unitree G1 factory inspection episode")?;
    let initial_digest = hash_physics_state(episode.simulation().world());
    let mut frames = Vec::new();
    let mut mesh_cache = MeshRenderCache::new();
    let mut mesh_items = 0;
    for step_index in 1..=CAPTURE_STEPS {
        let step = episode.step(UnitreeG1InspectionAction { advance: true });
        if capture {
            let (scene, current_mesh_items) = render_scene(&episode, &mut mesh_cache)?;
            mesh_items = mesh_items.max(current_mesh_items);
            frames.push(CaptureFrame {
                step: step_index,
                phase: if step.observation.gesture_progress > 0.01 {
                    "point-and-confirm".into()
                } else {
                    "factory-approach".into()
                },
                scene,
            });
        }
    }
    let final_observation = episode.current_observation();
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
        gesture_progress: final_observation.gesture_progress,
        completed_markers: final_observation.completed_markers,
        marker_count: final_observation.marker_count,
        mesh_items,
        frames,
    })
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

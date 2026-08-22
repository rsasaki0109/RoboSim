//! Real-capture Dr Johnson interior 3DGS hero for the authored mobile manipulator.
//!
//! The physics rollout is the same fixed-step, friction-grasp episode used by
//! the headless examples.  The render-only foreground is rebuilt from the
//! post-physics world transform of each of the ten URDF links, then resolved
//! through the visual-only manifest and PBR-aware [`rne_render::MeshRenderCache`].
//! The Dr Johnson scan is calibrated into the same Y-up metric frame as the
//! physics rollout. The measured floor and invisible pickup support share that frame;
//! 3DGS remains the appearance layer.
//!
//! Headless evidence (no GPU required):
//!
//! ```text
//! cargo run --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --smoke
//! ```
//!
//! GPU capture (writes 45 960x540 frames per camera, posters, GIFs, and metadata):
//!
//! ```text
//! cargo run --release --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --capture
//! ```

use anyhow::{Context, Result};
use png::{BitDepth, ColorType, Encoder};
use rne_ai::{
    Episode, GraspMode, IkMobileLiftPickPlacePolicy, MobileManipulatorEpisode,
    MobileManipulatorEpisodeConfig, Policy,
};
use rne_assets::{load_visual_manifest, VisualManifest};
use rne_math::{Quat, Transform3 as MathTransform3, Vec3};
use rne_physics::hash_physics_state;
use rne_render::{
    hash_rgba8, validate_gaussian_splat_manifest_with_override, Camera, HybridRenderScene,
    MeshRenderCache, PbrMaterial, RenderScene, RenderSceneItem, VisualShape,
};
use rne_render_3dgs::{load_gaussian_splat_background, render_hybrid_scene_camera};
use rne_render_wgpu::WgpuRenderBackend;
use rne_world::{world_transform_of, Transform3 as WorldTransform3};
use serde::Serialize;
use sha2::{Digest, Sha256};
use std::collections::BTreeSet;
use std::fs::{self, File};
use std::io::BufWriter;
use std::path::{Path, PathBuf};

const WIDTH: u32 = 960;
const HEIGHT: u32 = 540;
const FRAME_COUNT: usize = 45;
const FOV_Y_RAD: f64 = 0.800_689_935_801_928_9;
const CLEAR_COLOR: [f32; 4] = [0.055, 0.070, 0.085, 1.0];
const PAYLOAD_NAME: &str = "mobile_lift_cube";
const TARGET_X_M: f64 = -1.70;
const TARGET_Y_M: f64 = 0.035;
const TARGET_Z_M: f64 = -3.30;
const DRJOHNSON_COLLISION_PROXIES: [&str; 1] = ["mobile_lift_pick_support"];
const LINK_NAMES: [&str; 10] = [
    "base_link",
    "left_wheel",
    "right_wheel",
    "torso_link",
    "upper_arm_link",
    "forearm_link",
    "wrist_link",
    "gripper_base_link",
    "left_finger_link",
    "right_finger_link",
];

#[derive(Clone, Debug)]
struct RolloutFrame {
    step: u64,
    phase: String,
    foreground: RenderScene,
    grasping: bool,
    base_x_m: f64,
    base_z_m: f64,
    base_yaw_rad: f64,
    payload_x_m: f64,
    payload_y_m: f64,
    payload_z_m: f64,
    start_base_x_m: f64,
    start_base_z_m: f64,
    pick_x_m: f64,
    pick_z_m: f64,
    base_trajectory: Vec<(f64, f64)>,
    wrist_camera_transform: MathTransform3,
    wrist_rgbd: WristRgbdFrame,
}

#[derive(Clone, Debug)]
struct WristRgbdFrame {
    width_px: u32,
    height_px: u32,
    rgba8: Vec<u8>,
    depth_m: Vec<f32>,
    target_u_px: u32,
    target_v_px: u32,
    target_depth_m: f64,
    center_depth_m: f64,
    min_depth_m: f64,
    offset_x_m: f64,
    offset_y_m: f64,
}

#[derive(Clone, Debug, Serialize)]
struct SimulationEvidence {
    config: &'static str,
    policy: &'static str,
    grasp_mode: &'static str,
    terminated: bool,
    truncated: bool,
    grasped: bool,
    lift_clearance_m: f64,
    transport_distance_m: f64,
    place_error_m: f64,
    final_phase: String,
    phases_seen: Vec<String>,
    deterministic_digest: u64,
    replay_match: bool,
    steps: u64,
    wrist_camera_enabled: bool,
    wrist_rgbd_observed: bool,
}

#[derive(Clone, Debug, Serialize)]
struct CaptureEvidence {
    gpu_rendered: bool,
    width_px: u32,
    height_px: u32,
    frame_count: usize,
    frame_pattern: String,
    gif_path: String,
    gif_bytes: u64,
    gif_sha256: String,
    poster_path: String,
    poster_bytes: u64,
    poster_sha256: String,
    poster_frame: usize,
    sampled_sim_steps: Vec<u64>,
    unique_render_hashes: usize,
    duplicate_adjacent_frames: usize,
    overlay: OverlayEvidence,
    wrist_rgbd: WristRgbdEvidence,
}

#[derive(Clone, Debug, Serialize)]
struct OverlayEvidence {
    enabled: bool,
    camera_label: &'static str,
    state_source: &'static str,
    sampled_state_count: usize,
    map_trajectory_points: usize,
    telemetry_fields: [&'static str; 4],
}

#[derive(Clone, Debug, Serialize)]
struct WristRgbdEvidence {
    enabled: bool,
    source: &'static str,
    rgb_frame_count: usize,
    depth_frame_count: usize,
    target_projection_count: usize,
    width_px: u32,
    height_px: u32,
    target_fields: [&'static str; 5],
}

#[derive(Clone, Debug, Serialize)]
struct HeroMetadata {
    kind: &'static str,
    schema_version: u32,
    environment_id: String,
    renderer_identity: String,
    house_ply_path: String,
    house_ply_bytes: u64,
    house_ply_sha256: String,
    visual_manifest_path: String,
    visual_link_count: usize,
    visual_manifest_validated: bool,
    link_transform_sync_max_error_m: f64,
    foreground_mesh_items: usize,
    foreground_material_items: usize,
    collision_proxy_names: Vec<&'static str>,
    simulation: SimulationEvidence,
    capture: Option<CaptureEvidence>,
    reproduce_smoke: &'static str,
    reproduce_capture: &'static str,
    provenance: [&'static str; 3],
}

#[derive(Clone, Debug)]
struct Rollout {
    evidence: SimulationEvidence,
    frames: Vec<RolloutFrame>,
    link_sync_error_m: f64,
    foreground_mesh_items: usize,
    foreground_material_items: usize,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("House mobile-lift hero failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let args = std::env::args().skip(1).collect::<Vec<_>>();
    let smoke = args.iter().any(|argument| argument == "--smoke");
    let probe = args.iter().any(|argument| argument == "--probe");
    let capture = args.iter().any(|argument| argument == "--capture") || (!smoke && !probe);
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let house_manifest_path = repo_root
        .join("assets/environments/voxel51_drjohnson_3dgs/voxel51_drjohnson.rne.splat.toml");
    let ply_override = args
        .windows(2)
        .find(|window| window[0] == "--ply")
        .map(|window| PathBuf::from(&window[1]));
    anyhow::ensure!(
        ply_override.is_none() || probe,
        "--ply is a local visual probe override and requires --probe"
    );
    let house = validate_gaussian_splat_manifest_with_override(
        &house_manifest_path,
        ply_override.as_deref(),
    )
    .context("validate real-capture Dr Johnson 3DGS manifest")?;
    let drjohnson_scene_path =
        repo_root.join("assets/scenes/mm_mobile_lift_drjohnson.rne.scene.toml");
    let visual_manifest_path =
        repo_root.join("assets/robots/mm_mobile_lift/mm_mobile_lift.visual.toml");
    let visual_manifest = load_visual_manifest(&visual_manifest_path)
        .context("validate mm_mobile_lift visual manifest")?;
    if visual_manifest.links.len() != LINK_NAMES.len()
        || !LINK_NAMES
            .iter()
            .all(|name| visual_manifest.links.iter().any(|link| link.name == *name))
    {
        anyhow::bail!("visual manifest does not cover all ten mobile-lift links");
    }

    let first = rollout(
        &repo_root,
        &drjohnson_scene_path,
        &visual_manifest,
        false,
        None,
    )?;
    assert_success(&first)?;
    if smoke {
        let replay = rollout(
            &repo_root,
            &drjohnson_scene_path,
            &visual_manifest,
            false,
            None,
        )?;
        assert_success(&replay)?;
        anyhow::ensure!(
            first.evidence.deterministic_digest == replay.evidence.deterministic_digest,
            "deterministic digest changed on replay: {:#x} != {:#x}",
            first.evidence.deterministic_digest,
            replay.evidence.deterministic_digest
        );
        anyhow::ensure!(
            (first.link_sync_error_m - replay.link_sync_error_m).abs() < 1.0e-12,
            "link transform replay is not deterministic"
        );
        println!(
            "house mobile-lift hero smoke ok: terminated={} grasped={} lift_clearance={:.3} m transport={:.3} m place_error={:.4} m digest={:#018x} links={} meshes={} pbr={}",
            first.evidence.terminated,
            first.evidence.grasped,
            first.evidence.lift_clearance_m,
            first.evidence.transport_distance_m,
            first.evidence.place_error_m,
            first.evidence.deterministic_digest,
            visual_manifest.links.len(),
            first.foreground_mesh_items,
            first.foreground_material_items,
        );
        return Ok(());
    }

    if probe {
        let captured = rollout(
            &repo_root,
            &drjohnson_scene_path,
            &visual_manifest,
            true,
            Some(first.evidence.steps),
        )?;
        assert_success(&captured)?;
        let probe_dir = target_dir(&repo_root).join("rne-house-mobile-lift-hero");
        fs::create_dir_all(&probe_dir).context("create House hero probe directory")?;
        let probe_path = probe_dir.join("probe.png");
        let probe_frame = &captured.frames[FRAME_COUNT / 2];
        render_probe(&house, probe_frame, FRAME_COUNT / 2, &probe_path)?;
        println!("House mobile-lift hero probe: {}", probe_path.display());
        return Ok(());
    }

    if !capture {
        return Ok(());
    }
    let media_dir = repo_root.join("docs/media");
    fs::create_dir_all(&media_dir).context("create docs/media")?;
    let capture_dir = target_dir(&repo_root).join("rne-house-mobile-lift-hero");
    let _ = fs::remove_dir_all(&capture_dir);
    fs::create_dir_all(&capture_dir).context("create hero frame directory")?;
    let captured = rollout(
        &repo_root,
        &drjohnson_scene_path,
        &visual_manifest,
        true,
        Some(first.evidence.steps),
    )?;
    assert_success(&captured)?;
    anyhow::ensure!(
        first.evidence.deterministic_digest == captured.evidence.deterministic_digest,
        "capture rollout digest differs from the headless evidence rollout"
    );
    let (capture_evidence, follow_capture_evidence, poster_frame) =
        render_capture(&house, &captured, &capture_dir, &media_dir)?;
    let metadata = HeroMetadata {
        kind: "rne_house_mobile_lift_hero_metadata",
        schema_version: 1,
        environment_id: house.environment_id.clone(),
        renderer_identity: house.renderer_identity.clone(),
        house_ply_path: relative_path(&repo_root, &house.ply_path),
        house_ply_bytes: fs::metadata(&house.ply_path)?.len(),
        house_ply_sha256: sha256_file(&house.ply_path),
        visual_manifest_path: relative_path(&repo_root, &visual_manifest_path),
        visual_link_count: visual_manifest.links.len(),
        visual_manifest_validated: true,
        link_transform_sync_max_error_m: captured.link_sync_error_m,
        foreground_mesh_items: captured.foreground_mesh_items,
        foreground_material_items: captured.foreground_material_items,
        collision_proxy_names: DRJOHNSON_COLLISION_PROXIES.to_vec(),
        simulation: captured.evidence,
        capture: Some(capture_evidence),
        reproduce_smoke: "cargo run --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --smoke",
        reproduce_capture: "cargo run --release --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --capture",
        provenance: [
            "assets/environments/voxel51_drjohnson_3dgs/PROVENANCE.md",
            "assets/environments/voxel51_drjohnson_3dgs/LICENSE.txt",
            "assets/robots/mm_mobile_lift/PROVENANCE.md",
        ],
    };
    let metadata_path = media_dir.join("house-mobile-manipulation.json");
    fs::write(&metadata_path, serde_json::to_vec_pretty(&metadata)?)
        .with_context(|| format!("write {}", metadata_path.display()))?;
    let mut follow_metadata = metadata.clone();
    follow_metadata.kind = "rne_real_indoor_3dgs_robot_motion_metadata";
    follow_metadata.capture = Some(follow_capture_evidence);
    let follow_metadata_path = media_dir.join("showcase-real-3dgs.json");
    fs::write(
        &follow_metadata_path,
        serde_json::to_vec_pretty(&follow_metadata)?,
    )
    .with_context(|| format!("write {}", follow_metadata_path.display()))?;
    println!(
        "captured House mobile-lift hero: frames={} poster_frame={} gif={} poster={} metadata={}",
        FRAME_COUNT,
        poster_frame,
        media_dir.join("house-mobile-manipulation.gif").display(),
        media_dir.join("house-mobile-manipulation.png").display(),
        metadata_path.display(),
    );
    Ok(())
}

fn rollout(
    repo_root: &Path,
    scene_path: &Path,
    visual_manifest: &VisualManifest,
    capture: bool,
    expected_steps: Option<u64>,
) -> Result<Rollout> {
    let mut policy = IkMobileLiftPickPlacePolicy::new();
    let mut config = MobileManipulatorEpisodeConfig::mobile_lift_pick_place();
    config.max_steps = policy.total_steps();
    config.scene_path = scene_path.to_path_buf();
    config.task = rne_ai::MobileManipulatorTask::Place {
        object_name: PAYLOAD_NAME.into(),
        target: rne_ai::ReachTarget::new(TARGET_X_M, TARGET_Y_M, TARGET_Z_M),
        place_tolerance_m: 0.12,
    };
    let mut episode = MobileManipulatorEpisode::new(config);
    let mut step = episode.reset();
    episode.set_grasp_mode(GraspMode::Friction);
    for proxy in DRJOHNSON_COLLISION_PROXIES {
        anyhow::ensure!(
            episode.simulation().entity_named(proxy).is_some(),
            "Dr Johnson collision proxy is missing from the physics scene: {proxy}"
        );
    }
    let visual_root = repo_root.join("assets/robots/mm_mobile_lift");
    let package_root = visual_root.as_path();
    let package_roots = [package_root];
    let mut cache = MeshRenderCache::new();
    let mut frames = Vec::new();
    let total_policy_steps = policy.total_steps();
    let sample_targets = if capture {
        let sample_total = expected_steps.unwrap_or(total_policy_steps);
        (1..=FRAME_COUNT)
            .map(|frame| {
                (frame as u64 * sample_total)
                    .div_ceil(FRAME_COUNT as u64)
                    .max(1)
            })
            .collect::<Vec<_>>()
    } else {
        Vec::new()
    };
    let mut next_sample = 0_usize;
    let payload_initial = episode
        .simulation()
        .named_translation_m(PAYLOAD_NAME)
        .context("initial mobile-lift cube")?;
    let start_base_x_m = step.observation.base_x_m;
    let start_base_z_m = step.observation.base_z_m;
    let resting_y = payload_initial.1;
    let mut max_payload_y = resting_y;
    let mut grasped = false;
    let mut terminated = false;
    let mut truncated = false;
    let mut phases = BTreeSet::new();
    let mut max_sync_error_m = 0.0_f64;
    let mut mesh_items = 0_usize;
    let mut pbr_items = 0_usize;
    let mut base_trajectory = Vec::new();
    let wrist_camera_enabled = episode.simulation().wrist_camera_enabled();
    let mut wrist_rgbd_observed = false;

    anyhow::ensure!(
        wrist_camera_enabled,
        "Dr Johnson mobile-lift scene must configure a wrist camera"
    );

    for action_step in 0..total_policy_steps {
        let phase = policy.phase();
        phases.insert(format!("{phase:?}"));
        step = episode.step(policy.act(&step.observation));
        grasped |= episode.simulation().is_grasping();
        terminated |= step.terminated;
        truncated |= step.truncated;
        let payload = episode
            .simulation()
            .named_translation_m(PAYLOAD_NAME)
            .context("mobile-lift cube during rollout")?;
        max_payload_y = max_payload_y.max(payload.1);
        wrist_rgbd_observed |= episode.simulation().latest_wrist_camera().is_some()
            && episode.simulation().latest_wrist_depth().is_some()
            && episode.simulation().latest_wrist_rgbd_target().is_some();

        let should_sample = capture
            && next_sample < sample_targets.len()
            && action_step + 1 >= sample_targets[next_sample];
        if should_sample {
            let (scene, sync_error, scene_mesh_items, scene_pbr_items) = mobile_lift_foreground(
                episode.simulation(),
                visual_manifest,
                &mut cache,
                &package_roots,
            )?;
            max_sync_error_m = max_sync_error_m.max(sync_error);
            mesh_items = mesh_items.max(scene_mesh_items);
            pbr_items = pbr_items.max(scene_pbr_items);
            base_trajectory.push((step.observation.base_x_m, step.observation.base_z_m));
            let wrist_rgb = episode
                .simulation()
                .latest_wrist_camera()
                .context("sampled wrist RGB frame")?;
            let wrist_depth = episode
                .simulation()
                .latest_wrist_depth()
                .context("sampled wrist depth frame")?;
            let wrist_target = episode
                .simulation()
                .latest_wrist_rgbd_target()
                .context("sampled wrist RGB-D target estimate")?;
            anyhow::ensure!(
                wrist_rgb.width == wrist_depth.width
                    && wrist_rgb.height == wrist_depth.height
                    && wrist_rgb.rgba8.len()
                        == (wrist_rgb.width as usize)
                            .saturating_mul(wrist_rgb.height as usize)
                            .saturating_mul(4)
                    && wrist_depth.depth_m.len()
                        == (wrist_depth.width as usize).saturating_mul(wrist_depth.height as usize),
                "sampled wrist RGB-D payload dimensions are inconsistent"
            );
            let wrist_camera_world = episode
                .simulation()
                .wrist_camera_transform()
                .context("sampled wrist camera transform")?;
            frames.push(RolloutFrame {
                step: action_step + 1,
                phase: format!("{:?}", policy.phase()),
                foreground: scene,
                grasping: episode.simulation().is_grasping(),
                base_x_m: step.observation.base_x_m,
                base_z_m: step.observation.base_z_m,
                base_yaw_rad: step.observation.base_yaw_rad,
                payload_x_m: payload.0,
                payload_y_m: payload.1,
                payload_z_m: payload.2,
                start_base_x_m,
                start_base_z_m,
                pick_x_m: payload_initial.0,
                pick_z_m: payload_initial.2,
                base_trajectory: base_trajectory.clone(),
                wrist_camera_transform: MathTransform3::from_translation_rotation(
                    wrist_camera_world.translation,
                    wrist_camera_world.rotation,
                ),
                wrist_rgbd: WristRgbdFrame {
                    width_px: wrist_rgb.width,
                    height_px: wrist_rgb.height,
                    rgba8: wrist_rgb.rgba8,
                    depth_m: wrist_depth.depth_m,
                    target_u_px: wrist_target.pixel_u_px,
                    target_v_px: wrist_target.pixel_v_px,
                    target_depth_m: wrist_target.depth_m,
                    center_depth_m: wrist_target.center_depth_m,
                    min_depth_m: wrist_target.min_depth_m,
                    offset_x_m: wrist_target.offset_x_m,
                    offset_y_m: wrist_target.offset_y_m,
                },
            });
            next_sample += 1;
        }
        if step.is_done() {
            // The success termination is intentionally observed but the rest of
            // the fixed policy budget is not needed for a headless evidence run.
            break;
        }
    }
    if capture {
        anyhow::ensure!(
            frames.len() == FRAME_COUNT,
            "capture sampled {} frames for {} simulation steps; expected {}",
            frames.len(),
            episode.simulation().step_count(),
            FRAME_COUNT
        );
    }
    let payload_final = episode
        .simulation()
        .named_translation_m(PAYLOAD_NAME)
        .context("final mobile-lift cube")?;
    let transport_distance_m =
        (payload_initial.0 - payload_final.0).hypot(payload_initial.2 - payload_final.2);
    let place_error_m = (payload_final.0 - TARGET_X_M)
        .hypot(payload_final.1 - TARGET_Y_M)
        .hypot(payload_final.2 - TARGET_Z_M);
    // Resolve one final post-physics frame even in --smoke mode. This keeps
    // visual manifest validation tied to actual link-world synchronization,
    // rather than merely proving that the TOML parser accepted the manifest.
    let (_, final_sync_error, final_mesh_items, final_pbr_items) = mobile_lift_foreground(
        episode.simulation(),
        visual_manifest,
        &mut cache,
        &package_roots,
    )?;
    max_sync_error_m = max_sync_error_m.max(final_sync_error);
    mesh_items = mesh_items.max(final_mesh_items);
    pbr_items = pbr_items.max(final_pbr_items);
    let evidence = SimulationEvidence {
        config: "MobileManipulatorEpisodeConfig::mobile_lift_pick_place",
        policy: "IkMobileLiftPickPlacePolicy",
        grasp_mode: "GraspMode::Friction",
        terminated,
        truncated,
        grasped,
        lift_clearance_m: max_payload_y - resting_y,
        transport_distance_m,
        place_error_m,
        final_phase: format!("{:?}", policy.phase()),
        phases_seen: phases.into_iter().collect(),
        deterministic_digest: hash_physics_state(episode.simulation().world()),
        replay_match: true,
        steps: episode.simulation().step_count(),
        wrist_camera_enabled,
        wrist_rgbd_observed,
    };
    Ok(Rollout {
        evidence,
        frames,
        link_sync_error_m: max_sync_error_m,
        foreground_mesh_items: mesh_items,
        foreground_material_items: pbr_items,
    })
}

fn assert_success(rollout: &Rollout) -> Result<()> {
    let evidence = &rollout.evidence;
    anyhow::ensure!(
        evidence.terminated,
        "hero rollout did not terminate successfully: truncated={} grasped={} lift={:.3}m transport={:.3}m place_error={:.3}m final_phase={} steps={}",
        evidence.truncated,
        evidence.grasped,
        evidence.lift_clearance_m,
        evidence.transport_distance_m,
        evidence.place_error_m,
        evidence.final_phase,
        evidence.steps,
    );
    anyhow::ensure!(!evidence.truncated, "hero rollout was truncated");
    anyhow::ensure!(
        evidence.grasped,
        "hero rollout never established a friction grasp"
    );
    anyhow::ensure!(
        evidence.lift_clearance_m > 0.12,
        "hero payload did not lift clear of pickup: {:.3} m",
        evidence.lift_clearance_m
    );
    anyhow::ensure!(
        evidence.transport_distance_m > 1.5,
        "hero payload did not visibly transport: {:.3} m",
        evidence.transport_distance_m
    );
    anyhow::ensure!(
        evidence.place_error_m < 0.12,
        "hero payload was not placed: {:.3} m",
        evidence.place_error_m
    );
    anyhow::ensure!(
        evidence.wrist_camera_enabled && evidence.wrist_rgbd_observed,
        "hero rollout did not publish synchronized wrist RGB-D evidence"
    );
    anyhow::ensure!(
        rollout.foreground_mesh_items >= LINK_NAMES.len(),
        "hero foreground resolved {} mesh parts; expected at least {}",
        rollout.foreground_mesh_items,
        LINK_NAMES.len()
    );
    anyhow::ensure!(
        rollout.foreground_material_items >= LINK_NAMES.len(),
        "hero foreground resolved {} PBR parts; expected at least {}",
        rollout.foreground_material_items,
        LINK_NAMES.len()
    );
    anyhow::ensure!(
        rollout.link_sync_error_m <= 1.0e-9,
        "hero link transform synchronization error is {:.3e} m",
        rollout.link_sync_error_m
    );
    Ok(())
}

fn mobile_lift_foreground(
    sim: &rne_ai::MobileManipulatorSim,
    visual_manifest: &VisualManifest,
    cache: &mut MeshRenderCache,
    package_roots: &[&Path],
) -> Result<(RenderScene, f64, usize, usize)> {
    let mut scene = RenderScene::new();
    let mut max_sync_error_m = 0.0_f64;
    for name in LINK_NAMES {
        let link = visual_manifest
            .links
            .iter()
            .find(|link| link.name == name)
            .with_context(|| format!("missing visual manifest link {name}"))?;
        let entity = sim
            .entity_named(name)
            .with_context(|| format!("missing simulation link {name}"))?;
        let world = world_transform_of(sim.world(), entity);
        let render_world = WorldTransform3 {
            translation: world.translation,
            rotation: world.rotation,
            scale: world.scale,
        };
        let scale = Vec3::from_array(link.scale);
        scene.items.push(RenderScene::item_from_visual(
            render_world,
            VisualShape::Mesh {
                path: format!("package://mm_mobile_lift/{}", link.mesh),
                scale,
            },
            [1.0; 4],
            WorldTransform3::IDENTITY,
        ));
        // The render item is deliberately built from the same world transform;
        // this numerical check catches a stale link-pose cache before capture.
        let error = (scene
            .items
            .last()
            .expect("link render item")
            .transform
            .translation
            - render_world.translation)
            .length();
        max_sync_error_m = max_sync_error_m.max(error);
    }
    let payload = sim
        .named_translation_m(PAYLOAD_NAME)
        .context("missing mobile-lift cube visual")?;
    scene.items.push(box_item(
        Vec3::new(payload.0, payload.1, payload.2),
        Vec3::splat(0.07),
        [0.95, 0.20, 0.035, 1.0],
        PbrMaterial::new([0.95, 0.20, 0.035, 1.0], 0.28, 0.38, [0.03, 0.005, 0.0]),
    ));
    cache
        .resolve_scene(&mut scene, package_roots)
        .context("resolve mm_mobile_lift PBR links")?;
    let mesh_items = scene
        .items
        .iter()
        .filter(|item| item.mesh.is_some())
        .count();
    let pbr_items = scene
        .items
        .iter()
        .filter(|item| {
            item.mesh.is_some()
                && (item.material.normal_texture.is_some()
                    || item.material.metallic_roughness_texture.is_some()
                    || item.material.emissive_texture.is_some())
        })
        .count();
    Ok((scene, max_sync_error_m, mesh_items, pbr_items))
}

fn box_item(
    translation: Vec3,
    size: Vec3,
    color: [f32; 4],
    material: PbrMaterial,
) -> RenderSceneItem {
    RenderSceneItem {
        transform: MathTransform3 {
            translation,
            rotation: Quat::IDENTITY,
            scale: size,
        },
        shape: VisualShape::Box { size_m: Vec3::ONE },
        color_rgba: color,
        mesh: None,
        base_color_texture: None,
        material,
    }
}

fn render_capture(
    house: &rne_render::GaussianSplatEnvironment,
    rollout: &Rollout,
    capture_dir: &Path,
    media_dir: &Path,
) -> Result<(CaptureEvidence, CaptureEvidence, usize)> {
    anyhow::ensure!(
        rollout.frames.len() == FRAME_COUNT,
        "capture did not produce {FRAME_COUNT} frames"
    );
    let mut backend = WgpuRenderBackend::new().context("initialize wgpu for House hero")?;
    let mut background = load_gaussian_splat_background(backend.device(), house)
        .context("load House Gaussian background")?;
    let camera = Camera::new(WIDTH, HEIGHT, FOV_Y_RAD);
    let wrist_camera = Camera::new(160, 120, std::f64::consts::FRAC_PI_3);
    let mut render_hashes = Vec::with_capacity(FRAME_COUNT);
    let mut follow_render_hashes = Vec::with_capacity(FRAME_COUNT);
    let follow_dir = capture_dir.join("follow");
    fs::create_dir_all(&follow_dir).context("create Dr Johnson follow-camera frame directory")?;
    for (index, frame) in rollout.frames.iter().enumerate() {
        let hybrid = HybridRenderScene::new(house.clone(), frame.foreground.clone());
        let wrist_output = render_hybrid_scene_camera(
            &mut backend,
            &mut background,
            &wrist_camera,
            &frame.wrist_camera_transform,
            &hybrid,
            CLEAR_COLOR,
        )
        .with_context(|| format!("render wrist RGB-D frame {index}"))?;
        let wrist_rgbd = wrist_rgbd_from_render(frame, &wrist_camera, wrist_output);
        let output = render_hybrid_scene_camera(
            &mut backend,
            &mut background,
            &camera,
            &drjohnson_camera_transform(index, &house.transform),
            &hybrid,
            CLEAR_COLOR,
        )
        .with_context(|| format!("render House hero frame {index}"))?;
        anyhow::ensure!(
            output.color.width == WIDTH && output.color.height == HEIGHT,
            "unexpected GPU output dimensions {}x{}",
            output.color.width,
            output.color.height
        );
        let mut hero_rgba = output.color.rgba8;
        annotate_frame(&mut hero_rgba, frame, &wrist_rgbd, index);
        write_png(
            &capture_dir.join(format!("frame-{index:03}.png")),
            WIDTH,
            HEIGHT,
            &hero_rgba,
        )?;
        render_hashes.push(hash_rgba8(&hero_rgba));
        let follow_output = render_hybrid_scene_camera(
            &mut backend,
            &mut background,
            &camera,
            &drjohnson_follow_camera_transform(index),
            &hybrid,
            CLEAR_COLOR,
        )
        .with_context(|| format!("render Dr Johnson follow-camera frame {index}"))?;
        let mut follow_rgba = follow_output.color.rgba8;
        annotate_frame(&mut follow_rgba, frame, &wrist_rgbd, index);
        write_png(
            &follow_dir.join(format!("frame-{index:03}.png")),
            WIDTH,
            HEIGHT,
            &follow_rgba,
        )?;
        follow_render_hashes.push(hash_rgba8(&follow_rgba));
    }
    let unique_render_hashes = render_hashes.iter().copied().collect::<BTreeSet<_>>().len();
    let duplicate_adjacent_frames = render_hashes
        .windows(2)
        .filter(|pair| pair[0] == pair[1])
        .count();
    anyhow::ensure!(
        unique_render_hashes >= FRAME_COUNT.saturating_sub(5),
        "capture contains too many duplicate frames: {unique_render_hashes}/{FRAME_COUNT} unique"
    );
    anyhow::ensure!(
        duplicate_adjacent_frames <= 5,
        "capture contains {duplicate_adjacent_frames} adjacent duplicate frames"
    );
    let poster_frame = choose_poster_frame(rollout);
    fs::copy(
        capture_dir.join(format!("frame-{poster_frame:03}.png")),
        media_dir.join("house-mobile-manipulation.png"),
    )?;
    let gif_path = media_dir.join("house-mobile-manipulation.gif");
    build_gif(capture_dir, &gif_path, 16)?;
    let gif_bytes = fs::metadata(&gif_path)?.len();
    anyhow::ensure!(
        gif_bytes <= 5_000_000,
        "hero GIF exceeds 5 MB: {gif_bytes} bytes"
    );
    let poster_path = media_dir.join("house-mobile-manipulation.png");
    let evidence = CaptureEvidence {
        gpu_rendered: true,
        width_px: WIDTH,
        height_px: HEIGHT,
        frame_count: FRAME_COUNT,
        frame_pattern: relative_path(capture_dir, &capture_dir.join("frame-%03d.png")),
        gif_path: relative_path(media_dir, &gif_path),
        gif_bytes,
        gif_sha256: sha256_file(&gif_path),
        poster_path: relative_path(media_dir, &poster_path),
        poster_bytes: fs::metadata(&poster_path)?.len(),
        poster_sha256: sha256_file(&poster_path),
        poster_frame,
        sampled_sim_steps: rollout.frames.iter().map(|frame| frame.step).collect(),
        unique_render_hashes,
        duplicate_adjacent_frames,
        overlay: overlay_evidence(rollout),
        wrist_rgbd: wrist_rgbd_evidence(rollout),
    };
    let follow_unique_hashes = follow_render_hashes
        .iter()
        .copied()
        .collect::<BTreeSet<_>>()
        .len();
    let follow_adjacent_duplicates = follow_render_hashes
        .windows(2)
        .filter(|pair| pair[0] == pair[1])
        .count();
    anyhow::ensure!(
        follow_unique_hashes >= FRAME_COUNT.saturating_sub(5),
        "follow capture contains too many duplicate frames: {follow_unique_hashes}/{FRAME_COUNT} unique"
    );
    let follow_gif_path = media_dir.join("showcase-real-3dgs.gif");
    let follow_poster_path = media_dir.join("showcase-real-3dgs.png");
    fs::copy(
        follow_dir.join(format!("frame-{poster_frame:03}.png")),
        &follow_poster_path,
    )?;
    build_gif(&follow_dir, &follow_gif_path, 16)?;
    let follow_gif_bytes = fs::metadata(&follow_gif_path)?.len();
    anyhow::ensure!(
        follow_gif_bytes <= 5_000_000,
        "Dr Johnson follow GIF exceeds 5 MB: {follow_gif_bytes} bytes"
    );
    let follow_evidence = CaptureEvidence {
        gpu_rendered: true,
        width_px: WIDTH,
        height_px: HEIGHT,
        frame_count: FRAME_COUNT,
        frame_pattern: relative_path(capture_dir, &follow_dir.join("frame-%03d.png")),
        gif_path: relative_path(media_dir, &follow_gif_path),
        gif_bytes: follow_gif_bytes,
        gif_sha256: sha256_file(&follow_gif_path),
        poster_path: relative_path(media_dir, &follow_poster_path),
        poster_bytes: fs::metadata(&follow_poster_path)?.len(),
        poster_sha256: sha256_file(&follow_poster_path),
        poster_frame,
        sampled_sim_steps: rollout.frames.iter().map(|frame| frame.step).collect(),
        unique_render_hashes: follow_unique_hashes,
        duplicate_adjacent_frames: follow_adjacent_duplicates,
        overlay: overlay_evidence(rollout),
        wrist_rgbd: wrist_rgbd_evidence(rollout),
    };
    Ok((evidence, follow_evidence, poster_frame))
}

fn overlay_evidence(rollout: &Rollout) -> OverlayEvidence {
    OverlayEvidence {
        enabled: true,
        camera_label: "CAM 3DGS / REC",
        state_source: "post-physics rollout state sampled at each capture frame",
        sampled_state_count: rollout.frames.len(),
        map_trajectory_points: rollout
            .frames
            .last()
            .map_or(0, |frame| frame.base_trajectory.len()),
        telemetry_fields: ["phase", "grasp", "transport_m", "base_yaw_rad"],
    }
}

fn wrist_rgbd_evidence(rollout: &Rollout) -> WristRgbdEvidence {
    let first = rollout.frames.first().map(|frame| &frame.wrist_rgbd);
    WristRgbdEvidence {
        enabled: rollout.evidence.wrist_camera_enabled,
        source: "post-physics wrist pose rendering of real 3DGS plus robot, synchronized with DataBus RGB-D",
        rgb_frame_count: rollout.frames.len(),
        depth_frame_count: rollout.frames.len(),
        target_projection_count: rollout.frames.len(),
        width_px: if first.is_some() { 160 } else { 0 },
        height_px: if first.is_some() { 120 } else { 0 },
        target_fields: [
            "payload_pixel_uv",
            "optical_depth_m",
            "center_depth_m",
            "offset_x_m",
            "offset_y_m",
        ],
    }
}

fn wrist_rgbd_from_render(
    frame: &RolloutFrame,
    camera: &Camera,
    output: rne_render::CameraPassOutput,
) -> WristRgbdFrame {
    let local_target = frame.wrist_camera_transform.rotation.conjugate()
        * (Vec3::new(frame.payload_x_m, frame.payload_y_m, frame.payload_z_m)
            - frame.wrist_camera_transform.translation);
    let target_depth_m = (-local_target.z).max(camera.near_m);
    let focal_y_px = f64::from(camera.height) * 0.5 / (camera.fov_y_rad * 0.5).tan();
    let center_u_px = (f64::from(camera.width) - 1.0) * 0.5;
    let center_v_px = (f64::from(camera.height) - 1.0) * 0.5;
    let target_u_px = (center_u_px + local_target.x * focal_y_px / target_depth_m)
        .round()
        .clamp(0.0, f64::from(camera.width.saturating_sub(1))) as u32;
    let target_v_px = (center_v_px - local_target.y * focal_y_px / target_depth_m)
        .round()
        .clamp(0.0, f64::from(camera.height.saturating_sub(1))) as u32;
    let center_index =
        (output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize;
    let center_depth_m = output
        .depth
        .depth_m
        .get(center_index)
        .copied()
        .map_or(camera.far_m, f64::from);
    let min_depth_m = output
        .depth
        .depth_m
        .iter()
        .copied()
        .filter(|depth_m| depth_m.is_finite() && *depth_m > 0.0)
        .reduce(f32::min)
        .map_or(camera.far_m, f64::from);

    WristRgbdFrame {
        width_px: output.color.width,
        height_px: output.color.height,
        rgba8: output.color.rgba8,
        depth_m: output.depth.depth_m,
        target_u_px,
        target_v_px,
        target_depth_m,
        center_depth_m,
        min_depth_m,
        offset_x_m: local_target.x,
        offset_y_m: local_target.y,
    }
}

/// Draw the evidence UI directly into the rendered RGBA buffer.
///
/// The map and mission state come from the sampled post-physics [`RolloutFrame`],
/// while the wrist inset is the RGB-D pass rendered at that frame's wrist pose.
fn annotate_frame(
    rgba: &mut [u8],
    frame: &RolloutFrame,
    wrist_rgbd: &WristRgbdFrame,
    _frame_index: usize,
) {
    draw_camera_brackets(rgba, WIDTH as i32, HEIGHT as i32);
    panel(rgba, 18, 18, 300, 94, [8, 18, 28, 220]);
    text(rgba, 30, 28, "CAM 3DGS / REC", 2, [104, 235, 240, 255]);
    text(rgba, 30, 48, "PHASE", 1, [155, 173, 186, 255]);
    text(
        rgba,
        80,
        48,
        &short_phase(&frame.phase),
        2,
        [242, 183, 76, 255],
    );
    text(
        rgba,
        30,
        68,
        if is_navigation_phase(&frame.phase) {
            "NAVIGATION"
        } else {
            "MANIPULATION"
        },
        1,
        [201, 218, 226, 255],
    );
    text(
        rgba,
        30,
        86,
        &format!(
            "GRASP {}  TRN {:.2}M",
            if frame.grasping { "LOCK" } else { "OPEN" },
            (frame.payload_x_m - frame.pick_x_m).hypot(frame.payload_z_m - frame.pick_z_m)
        ),
        1,
        if frame.grasping {
            [116, 240, 155, 255]
        } else {
            [208, 218, 224, 255]
        },
    );

    draw_wrist_rgbd_pip(rgba, wrist_rgbd);
    draw_map(rgba, frame);
}

fn draw_wrist_rgbd_pip(rgba: &mut [u8], wrist_rgbd: &WristRgbdFrame) {
    const PANEL_X: i32 = 632;
    const PANEL_Y: i32 = 18;
    const PANEL_W: i32 = 310;
    const PANEL_H: i32 = 174;
    const VIEW_Y: i32 = 44;
    const VIEW_W: i32 = 140;
    const VIEW_H: i32 = 105;
    const RGB_X: i32 = 642;
    const DEPTH_X: i32 = 792;

    panel(rgba, PANEL_X, PANEL_Y, PANEL_W, PANEL_H, [8, 18, 28, 226]);
    text(
        rgba,
        PANEL_X + 10,
        PANEL_Y + 10,
        "WRIST RGB-D / LIVE",
        1,
        [104, 235, 240, 255],
    );
    blit_rgba_nearest(
        rgba,
        &wrist_rgbd.rgba8,
        wrist_rgbd.width_px,
        wrist_rgbd.height_px,
        RGB_X,
        VIEW_Y,
        VIEW_W,
        VIEW_H,
    );
    blit_depth_nearest(
        rgba,
        &wrist_rgbd.depth_m,
        wrist_rgbd.width_px,
        wrist_rgbd.height_px,
        DEPTH_X,
        VIEW_Y,
        VIEW_W,
        VIEW_H,
    );
    border(rgba, RGB_X, VIEW_Y, VIEW_W, VIEW_H, [201, 218, 226, 255]);
    border(rgba, DEPTH_X, VIEW_Y, VIEW_W, VIEW_H, [201, 218, 226, 255]);
    text(rgba, RGB_X + 4, VIEW_Y + 4, "RGB", 1, [255, 255, 255, 255]);
    text(
        rgba,
        DEPTH_X + 4,
        VIEW_Y + 4,
        "DEPTH",
        1,
        [255, 255, 255, 255],
    );
    draw_rgbd_target(rgba, RGB_X, VIEW_Y, VIEW_W, VIEW_H, wrist_rgbd);
    draw_rgbd_target(rgba, DEPTH_X, VIEW_Y, VIEW_W, VIEW_H, wrist_rgbd);
    text(
        rgba,
        PANEL_X + 10,
        PANEL_Y + 140,
        &format!(
            "TARGET {:.2}M  XY {:+.2} {:+.2}",
            wrist_rgbd.target_depth_m, wrist_rgbd.offset_x_m, wrist_rgbd.offset_y_m
        ),
        1,
        [242, 183, 76, 255],
    );
    text(
        rgba,
        PANEL_X + 10,
        PANEL_Y + 156,
        &format!(
            "CENTER {:.2}M  MIN {:.2}M",
            wrist_rgbd.center_depth_m, wrist_rgbd.min_depth_m
        ),
        1,
        [201, 218, 226, 255],
    );
}

#[allow(clippy::too_many_arguments)]
fn blit_rgba_nearest(
    destination: &mut [u8],
    source: &[u8],
    source_width: u32,
    source_height: u32,
    x: i32,
    y: i32,
    width: i32,
    height: i32,
) {
    if source_width == 0 || source_height == 0 || width <= 0 || height <= 0 {
        return;
    }
    for destination_y in 0..height {
        let source_y = (destination_y as u32 * source_height / height as u32)
            .min(source_height.saturating_sub(1));
        for destination_x in 0..width {
            let source_x = (destination_x as u32 * source_width / width as u32)
                .min(source_width.saturating_sub(1));
            let index = ((source_y * source_width + source_x) * 4) as usize;
            if let Some(pixel) = source.get(index..index + 4) {
                blend_pixel(
                    destination,
                    WIDTH as i32,
                    HEIGHT as i32,
                    x + destination_x,
                    y + destination_y,
                    [pixel[0], pixel[1], pixel[2], 255],
                );
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn blit_depth_nearest(
    destination: &mut [u8],
    source: &[f32],
    source_width: u32,
    source_height: u32,
    x: i32,
    y: i32,
    width: i32,
    height: i32,
) {
    if source_width == 0 || source_height == 0 || width <= 0 || height <= 0 {
        return;
    }
    let mut near_m = f32::INFINITY;
    let mut far_m = 0.0_f32;
    for depth_m in source
        .iter()
        .copied()
        .filter(|value| value.is_finite() && *value > 0.0)
    {
        near_m = near_m.min(depth_m);
        far_m = far_m.max(depth_m);
    }
    let span_m = (far_m - near_m).max(1.0e-4);
    for destination_y in 0..height {
        let source_y = (destination_y as u32 * source_height / height as u32)
            .min(source_height.saturating_sub(1));
        for destination_x in 0..width {
            let source_x = (destination_x as u32 * source_width / width as u32)
                .min(source_width.saturating_sub(1));
            let index = (source_y * source_width + source_x) as usize;
            let color = source
                .get(index)
                .copied()
                .filter(|depth_m| depth_m.is_finite() && *depth_m > 0.0)
                .map_or([4, 8, 14, 255], |depth_m| {
                    depth_color(((depth_m - near_m) / span_m).clamp(0.0, 1.0))
                });
            blend_pixel(
                destination,
                WIDTH as i32,
                HEIGHT as i32,
                x + destination_x,
                y + destination_y,
                color,
            );
        }
    }
}

fn depth_color(normalized: f32) -> [u8; 4] {
    let normalized = normalized.clamp(0.0, 1.0);
    let red = ((1.0 - normalized) * 255.0) as u8;
    let green = ((1.0 - (normalized - 0.5).abs() * 2.0) * 220.0) as u8;
    let blue = (normalized * 255.0) as u8;
    [red, green, blue, 255]
}

fn draw_rgbd_target(
    rgba: &mut [u8],
    x: i32,
    y: i32,
    width: i32,
    height: i32,
    frame: &WristRgbdFrame,
) {
    let target_x =
        x + (frame.target_u_px as i32 * width / frame.width_px.max(1) as i32).clamp(0, width - 1);
    let target_y = y
        + (frame.target_v_px as i32 * height / frame.height_px.max(1) as i32).clamp(0, height - 1);
    let color = [242, 183, 76, 255];
    let half_width = 12;
    let half_height = 10;
    border(
        rgba,
        target_x - half_width,
        target_y - half_height,
        half_width * 2,
        half_height * 2,
        color,
    );
    marker(rgba, target_x, target_y, color);
}

fn border(rgba: &mut [u8], x: i32, y: i32, width: i32, height: i32, color: [u8; 4]) {
    line(rgba, WIDTH as i32, HEIGHT as i32, x, y, x + width, y, color);
    line(
        rgba,
        WIDTH as i32,
        HEIGHT as i32,
        x,
        y,
        x,
        y + height,
        color,
    );
    line(
        rgba,
        WIDTH as i32,
        HEIGHT as i32,
        x + width,
        y,
        x + width,
        y + height,
        color,
    );
    line(
        rgba,
        WIDTH as i32,
        HEIGHT as i32,
        x,
        y + height,
        x + width,
        y + height,
        color,
    );
}

fn draw_camera_brackets(rgba: &mut [u8], width: i32, height: i32) {
    let color = [104, 235, 240, 255];
    let inset = 12;
    let length = 22;
    line(
        rgba,
        width,
        height,
        inset,
        inset,
        inset + length,
        inset,
        color,
    );
    line(
        rgba,
        width,
        height,
        inset,
        inset,
        inset,
        inset + length,
        color,
    );
    line(
        rgba,
        width,
        height,
        width - inset,
        inset,
        width - inset - length,
        inset,
        color,
    );
    line(
        rgba,
        width,
        height,
        width - inset,
        inset,
        width - inset,
        inset + length,
        color,
    );
    line(
        rgba,
        width,
        height,
        inset,
        height - inset,
        inset + length,
        height - inset,
        color,
    );
    line(
        rgba,
        width,
        height,
        inset,
        height - inset,
        inset,
        height - inset - length,
        color,
    );
    line(
        rgba,
        width,
        height,
        width - inset,
        height - inset,
        width - inset - length,
        height - inset,
        color,
    );
    line(
        rgba,
        width,
        height,
        width - inset,
        height - inset,
        width - inset,
        height - inset - length,
        color,
    );
}

fn draw_map(rgba: &mut [u8], frame: &RolloutFrame) {
    const X: i32 = 18;
    const Y: i32 = 374;
    const W: i32 = 282;
    const H: i32 = 148;
    panel(rgba, X, Y, W, H, [8, 18, 28, 224]);
    text(
        rgba,
        X + 12,
        Y + 10,
        "TOP-DOWN / LIVE PATH",
        1,
        [104, 235, 240, 255],
    );
    let min_x = frame.start_base_x_m.min(frame.pick_x_m).min(TARGET_X_M) - 0.35;
    let max_x = frame.start_base_x_m.max(frame.pick_x_m).max(TARGET_X_M) + 0.35;
    let min_z = frame.start_base_z_m.min(frame.pick_z_m).min(TARGET_Z_M) - 0.35;
    let max_z = frame.start_base_z_m.max(frame.pick_z_m).max(TARGET_Z_M) + 0.35;
    let map_x = X + 12;
    let map_y = Y + 30;
    let map_w = W - 24;
    let map_h = H - 42;
    for pair in frame.base_trajectory.windows(2) {
        let a = map_point(
            pair[0], min_x, max_x, min_z, max_z, map_x, map_y, map_w, map_h,
        );
        let b = map_point(
            pair[1], min_x, max_x, min_z, max_z, map_x, map_y, map_w, map_h,
        );
        line(
            rgba,
            WIDTH as i32,
            HEIGHT as i32,
            a.0,
            a.1,
            b.0,
            b.1,
            [242, 183, 76, 255],
        );
    }
    let start = map_point(
        (frame.start_base_x_m, frame.start_base_z_m),
        min_x,
        max_x,
        min_z,
        max_z,
        map_x,
        map_y,
        map_w,
        map_h,
    );
    let pick = map_point(
        (frame.pick_x_m, frame.pick_z_m),
        min_x,
        max_x,
        min_z,
        max_z,
        map_x,
        map_y,
        map_w,
        map_h,
    );
    let goal = map_point(
        (TARGET_X_M, TARGET_Z_M),
        min_x,
        max_x,
        min_z,
        max_z,
        map_x,
        map_y,
        map_w,
        map_h,
    );
    marker(rgba, start.0, start.1, [190, 200, 208, 255]);
    marker(rgba, pick.0, pick.1, [242, 183, 76, 255]);
    marker(rgba, goal.0, goal.1, [116, 240, 155, 255]);
    let robot = map_point(
        (frame.base_x_m, frame.base_z_m),
        min_x,
        max_x,
        min_z,
        max_z,
        map_x,
        map_y,
        map_w,
        map_h,
    );
    circle(rgba, robot.0, robot.1, 6, [104, 235, 240, 255]);
    let heading = (
        robot.0 + (frame.base_yaw_rad.cos() * 12.0) as i32,
        robot.1 - (frame.base_yaw_rad.sin() * 12.0) as i32,
    );
    line(
        rgba,
        WIDTH as i32,
        HEIGHT as i32,
        robot.0,
        robot.1,
        heading.0,
        heading.1,
        [104, 235, 240, 255],
    );
    text(
        rgba,
        X + 14,
        Y + H - 12,
        "START  PICK  GOAL",
        1,
        [190, 200, 208, 255],
    );
}

#[allow(clippy::too_many_arguments)]
fn map_point(
    point: (f64, f64),
    min_x: f64,
    max_x: f64,
    min_z: f64,
    max_z: f64,
    x: i32,
    y: i32,
    width: i32,
    height: i32,
) -> (i32, i32) {
    let px = ((point.0 - min_x) / (max_x - min_x)).clamp(0.0, 1.0);
    let pz = ((max_z - point.1) / (max_z - min_z)).clamp(0.0, 1.0);
    (
        x + (px * f64::from(width)) as i32,
        y + (pz * f64::from(height)) as i32,
    )
}

fn is_navigation_phase(phase: &str) -> bool {
    phase.contains("Navigate") || phase.contains("Approach")
}

fn short_phase(phase: &str) -> String {
    phase
        .replace("NavigateToPick", "NAV_PICK")
        .replace("NavigateToPlace", "NAV_PLACE")
        .to_ascii_uppercase()
}

fn panel(rgba: &mut [u8], x: i32, y: i32, width: i32, height: i32, color: [u8; 4]) {
    for py in y..y + height {
        for px in x..x + width {
            blend_pixel(rgba, WIDTH as i32, HEIGHT as i32, px, py, color);
        }
    }
}

fn marker(rgba: &mut [u8], x: i32, y: i32, color: [u8; 4]) {
    line(rgba, WIDTH as i32, HEIGHT as i32, x - 5, y, x + 5, y, color);
    line(rgba, WIDTH as i32, HEIGHT as i32, x, y - 5, x, y + 5, color);
}

fn circle(rgba: &mut [u8], cx: i32, cy: i32, radius: i32, color: [u8; 4]) {
    for y in -radius..=radius {
        for x in -radius..=radius {
            if x * x + y * y <= radius * radius {
                blend_pixel(rgba, WIDTH as i32, HEIGHT as i32, cx + x, cy + y, color);
            }
        }
    }
}

#[allow(clippy::too_many_arguments)]
fn line(
    rgba: &mut [u8],
    width: i32,
    height: i32,
    mut x0: i32,
    mut y0: i32,
    x1: i32,
    y1: i32,
    color: [u8; 4],
) {
    let dx = (x1 - x0).abs();
    let sx = if x0 < x1 { 1 } else { -1 };
    let dy = -(y1 - y0).abs();
    let sy = if y0 < y1 { 1 } else { -1 };
    let mut error = dx + dy;
    loop {
        blend_pixel(rgba, width, height, x0, y0, color);
        if x0 == x1 && y0 == y1 {
            break;
        }
        let twice = 2 * error;
        if twice >= dy {
            error += dy;
            x0 += sx;
        }
        if twice <= dx {
            error += dx;
            y0 += sy;
        }
    }
}

fn blend_pixel(rgba: &mut [u8], width: i32, height: i32, x: i32, y: i32, color: [u8; 4]) {
    if x < 0 || y < 0 || x >= width || y >= height {
        return;
    }
    let index = ((y * width + x) * 4) as usize;
    let alpha = u16::from(color[3]);
    let inverse = 255_u16 - alpha;
    for channel in 0..3 {
        rgba[index + channel] = ((u16::from(color[channel]) * alpha
            + u16::from(rgba[index + channel]) * inverse)
            / 255) as u8;
    }
    rgba[index + 3] = 255;
}

fn text(rgba: &mut [u8], x: i32, y: i32, value: &str, scale: i32, color: [u8; 4]) {
    let mut cursor = x;
    for character in value.chars() {
        let glyph = glyph_rows(character);
        for (row, bits) in glyph.iter().enumerate() {
            for column in 0..5 {
                if bits & (1 << (4 - column)) != 0 {
                    for sy in 0..scale {
                        for sx in 0..scale {
                            blend_pixel(
                                rgba,
                                WIDTH as i32,
                                HEIGHT as i32,
                                cursor + column * scale + sx,
                                y + row as i32 * scale + sy,
                                color,
                            );
                        }
                    }
                }
            }
        }
        cursor += 6 * scale;
    }
}

fn glyph_rows(character: char) -> [u8; 7] {
    match character {
        'A' => [0x0E, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'B' => [0x1E, 0x11, 0x11, 0x1E, 0x11, 0x11, 0x1E],
        'C' => [0x0F, 0x10, 0x10, 0x10, 0x10, 0x10, 0x0F],
        'D' => [0x1E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x1E],
        'E' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x1F],
        'F' => [0x1F, 0x10, 0x10, 0x1E, 0x10, 0x10, 0x10],
        'G' => [0x0F, 0x10, 0x10, 0x17, 0x11, 0x11, 0x0F],
        'H' => [0x11, 0x11, 0x11, 0x1F, 0x11, 0x11, 0x11],
        'I' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x1F],
        'J' => [0x01, 0x01, 0x01, 0x01, 0x11, 0x11, 0x0E],
        'K' => [0x11, 0x12, 0x14, 0x18, 0x14, 0x12, 0x11],
        'L' => [0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x1F],
        'M' => [0x11, 0x1B, 0x15, 0x15, 0x11, 0x11, 0x11],
        'N' => [0x11, 0x19, 0x15, 0x13, 0x11, 0x11, 0x11],
        'O' => [0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'P' => [0x1E, 0x11, 0x11, 0x1E, 0x10, 0x10, 0x10],
        'Q' => [0x0E, 0x11, 0x11, 0x11, 0x15, 0x12, 0x0D],
        'R' => [0x1E, 0x11, 0x11, 0x1E, 0x14, 0x12, 0x11],
        'S' => [0x0F, 0x10, 0x10, 0x0E, 0x01, 0x01, 0x1E],
        'T' => [0x1F, 0x04, 0x04, 0x04, 0x04, 0x04, 0x04],
        'U' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E],
        'V' => [0x11, 0x11, 0x11, 0x11, 0x11, 0x0A, 0x04],
        'W' => [0x11, 0x11, 0x11, 0x15, 0x15, 0x1B, 0x11],
        'X' => [0x11, 0x11, 0x0A, 0x04, 0x0A, 0x11, 0x11],
        'Y' => [0x11, 0x11, 0x0A, 0x04, 0x04, 0x04, 0x04],
        'Z' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x10, 0x1F],
        '0' => [0x0E, 0x11, 0x13, 0x15, 0x19, 0x11, 0x0E],
        '1' => [0x04, 0x0C, 0x04, 0x04, 0x04, 0x04, 0x0E],
        '2' => [0x0E, 0x11, 0x01, 0x02, 0x04, 0x08, 0x1F],
        '3' => [0x1E, 0x01, 0x01, 0x0E, 0x01, 0x01, 0x1E],
        '4' => [0x02, 0x06, 0x0A, 0x12, 0x1F, 0x02, 0x02],
        '5' => [0x1F, 0x10, 0x10, 0x1E, 0x01, 0x01, 0x1E],
        '6' => [0x0E, 0x10, 0x10, 0x1E, 0x11, 0x11, 0x0E],
        '7' => [0x1F, 0x01, 0x02, 0x04, 0x08, 0x08, 0x08],
        '8' => [0x0E, 0x11, 0x11, 0x0E, 0x11, 0x11, 0x0E],
        '9' => [0x0E, 0x11, 0x11, 0x0F, 0x01, 0x01, 0x0E],
        '/' => [0x01, 0x02, 0x02, 0x04, 0x08, 0x08, 0x10],
        '_' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1F],
        '.' => [0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0x0C],
        '-' => [0x00, 0x00, 0x00, 0x1F, 0x00, 0x00, 0x00],
        ':' => [0x00, 0x0C, 0x0C, 0x00, 0x0C, 0x0C, 0x00],
        _ => [0; 7],
    }
}

#[cfg(test)]
mod overlay_tests {
    use super::{blit_rgba_nearest, depth_color, map_point, short_phase, HEIGHT, WIDTH};

    #[test]
    fn map_point_keeps_world_markers_inside_inset() {
        assert_eq!(
            map_point((0.0, 0.0), -1.0, 1.0, -1.0, 1.0, 10, 20, 100, 80),
            (60, 60)
        );
        assert_eq!(
            map_point((4.0, -4.0), -1.0, 1.0, -1.0, 1.0, 10, 20, 100, 80),
            (110, 100)
        );
    }

    #[test]
    fn phase_label_preserves_real_rollout_phase() {
        assert_eq!(short_phase("NavigateToPick"), "NAV_PICK");
        assert_eq!(short_phase("Transport"), "TRANSPORT");
    }

    #[test]
    fn wrist_rgb_nearest_blit_preserves_sensor_pixels() {
        let source = [255, 0, 0, 255, 0, 0, 255, 255];
        let mut destination = vec![0; (WIDTH * HEIGHT * 4) as usize];
        blit_rgba_nearest(&mut destination, &source, 2, 1, 0, 0, 4, 2);
        assert_eq!(&destination[0..4], &[255, 0, 0, 255]);
        assert_eq!(&destination[12..16], &[0, 0, 255, 255]);
    }

    #[test]
    fn depth_palette_distinguishes_near_and_far_samples() {
        assert_eq!(depth_color(0.0), [255, 0, 0, 255]);
        assert_eq!(depth_color(1.0), [0, 0, 255, 255]);
        assert_ne!(depth_color(0.25), depth_color(0.75));
    }
}

fn render_probe(
    house: &rne_render::GaussianSplatEnvironment,
    frame: &RolloutFrame,
    frame_index: usize,
    path: &Path,
) -> Result<()> {
    let mut backend = WgpuRenderBackend::new().context("initialize wgpu for House hero probe")?;
    let mut background = load_gaussian_splat_background(backend.device(), house)
        .context("load House Gaussian background for probe")?;
    let camera = Camera::new(WIDTH, HEIGHT, FOV_Y_RAD);
    let hybrid = HybridRenderScene::new(house.clone(), frame.foreground.clone());
    let output = render_hybrid_scene_camera(
        &mut backend,
        &mut background,
        &camera,
        &drjohnson_camera_transform(frame_index, &house.transform),
        &hybrid,
        CLEAR_COLOR,
    )
    .context("render House hero probe")?;
    write_png(path, WIDTH, HEIGHT, &output.color.rgba8)?;
    let follow_output = render_hybrid_scene_camera(
        &mut backend,
        &mut background,
        &camera,
        &drjohnson_follow_camera_transform(frame_index),
        &hybrid,
        CLEAR_COLOR,
    )
    .context("render Dr Johnson robot-motion probe")?;
    write_png(
        &path.with_file_name("probe-follow.png"),
        WIDTH,
        HEIGHT,
        &follow_output.color.rgba8,
    )
}

fn drjohnson_camera_transform(_index: usize, _scene_transform: &MathTransform3) -> MathTransform3 {
    // Exact transformed COLMAP pose for Dr Johnson frame IMG_6293. Keeping the
    // capture on a measured camera ray preserves the real reconstruction's
    // geometry instead of inventing a free-view orbit through sparse splats.
    MathTransform3::from_translation_rotation(
        Vec3::new(-3.117_781_018, 1.421_316_325, -1.672_926_404),
        Quat::from_xyzw(0.0, -0.461_148_822_3, 0.0, 0.887_322_806_9) * Quat::from_rotation_x(-0.16),
    )
}

fn drjohnson_follow_camera_transform(_index: usize) -> MathTransform3 {
    // Exact transformed COLMAP pose for Dr Johnson frame IMG_6292.
    MathTransform3::from_translation_rotation(
        Vec3::new(-3.093_182_539, 1.852_322_531, -1.670_394_267),
        Quat::from_xyzw(
            -0.004_500_635_1,
            -0.468_533_511_7,
            -0.017_629_964_0,
            0.883_258_329_7,
        ) * Quat::from_rotation_x(-0.30),
    )
}

fn choose_poster_frame(rollout: &Rollout) -> usize {
    rollout
        .frames
        .iter()
        .position(|frame| frame.grasping && frame.phase == "Transport")
        .unwrap_or(FRAME_COUNT / 2)
}

fn build_gif(frames_dir: &Path, gif_path: &Path, max_colors: u8) -> Result<()> {
    let input = frames_dir.join("frame-%03d.png");
    let filter = format!(
        "fps=8,scale=960:540:flags=lanczos,split[s0][s1];[s0]palettegen=max_colors={max_colors}:stats_mode=diff[p];[s1][p]paletteuse=dither=none"
    );
    let status = std::process::Command::new("ffmpeg")
        .args([
            "-y",
            "-loglevel",
            "error",
            "-framerate",
            "8",
            "-i",
            &input.to_string_lossy(),
            "-frames:v",
            &FRAME_COUNT.to_string(),
            "-vf",
            &filter,
            &gif_path.to_string_lossy(),
        ])
        .status()
        .context("spawn ffmpeg")?;
    anyhow::ensure!(status.success(), "ffmpeg GIF encode failed");
    Ok(())
}

fn target_dir(repo_root: &Path) -> PathBuf {
    std::env::var_os("CARGO_TARGET_DIR")
        .map(PathBuf::from)
        .unwrap_or_else(|| repo_root.join("target"))
}

fn relative_path(root: &Path, path: &Path) -> String {
    path.strip_prefix(root)
        .unwrap_or(path)
        .to_string_lossy()
        .replace('\\', "/")
}

fn sha256_file(path: &Path) -> String {
    Sha256::digest(fs::read(path).expect("read hash input"))
        .iter()
        .map(|byte| format!("{byte:02x}"))
        .collect()
}

fn write_png(path: &Path, width: u32, height: u32, rgba8: &[u8]) -> Result<()> {
    let file = File::create(path).with_context(|| format!("create {}", path.display()))?;
    let writer = BufWriter::new(file);
    let mut encoder = Encoder::new(writer, width, height);
    encoder.set_color(ColorType::Rgba);
    encoder.set_depth(BitDepth::Eight);
    let mut png_writer = encoder.write_header().context("write PNG header")?;
    png_writer
        .write_image_data(rgba8)
        .context("write PNG pixels")?;
    Ok(())
}

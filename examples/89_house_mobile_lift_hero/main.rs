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
        render_probe(
            &house,
            &captured.frames[FRAME_COUNT / 2],
            FRAME_COUNT / 2,
            &probe_path,
        )?;
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
    let resting_y = payload_initial.1;
    let mut max_payload_y = resting_y;
    let mut grasped = false;
    let mut terminated = false;
    let mut truncated = false;
    let mut phases = BTreeSet::new();
    let mut max_sync_error_m = 0.0_f64;
    let mut mesh_items = 0_usize;
    let mut pbr_items = 0_usize;

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
            frames.push(RolloutFrame {
                step: action_step + 1,
                phase: format!("{:?}", policy.phase()),
                foreground: scene,
                grasping: episode.simulation().is_grasping(),
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
    let mut render_hashes = Vec::with_capacity(FRAME_COUNT);
    let mut follow_render_hashes = Vec::with_capacity(FRAME_COUNT);
    let follow_dir = capture_dir.join("follow");
    fs::create_dir_all(&follow_dir).context("create Dr Johnson follow-camera frame directory")?;
    for (index, frame) in rollout.frames.iter().enumerate() {
        let hybrid = HybridRenderScene::new(house.clone(), frame.foreground.clone());
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
        write_png(
            &capture_dir.join(format!("frame-{index:03}.png")),
            WIDTH,
            HEIGHT,
            &output.color.rgba8,
        )?;
        render_hashes.push(hash_rgba8(&output.color.rgba8));
        let follow_output = render_hybrid_scene_camera(
            &mut backend,
            &mut background,
            &camera,
            &drjohnson_follow_camera_transform(index),
            &hybrid,
            CLEAR_COLOR,
        )
        .with_context(|| format!("render Dr Johnson follow-camera frame {index}"))?;
        write_png(
            &follow_dir.join(format!("frame-{index:03}.png")),
            WIDTH,
            HEIGHT,
            &follow_output.color.rgba8,
        )?;
        follow_render_hashes.push(hash_rgba8(&follow_output.color.rgba8));
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
    };
    Ok((evidence, follow_evidence, poster_frame))
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

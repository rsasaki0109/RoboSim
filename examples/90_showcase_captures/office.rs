//! Office AGV desk-place showcase source and capture.

use super::media::{
    capture_frames, push_box, push_box_material, push_cylinder, push_sphere, CameraEvidence,
    CaptureFrame, ShowcaseMetadata, SimulationEvidence, FRAME_COUNT,
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
use std::f64::consts::FRAC_PI_2;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "office";
const SUBJECT: &str = "office AGV shared-aisle desk place";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.40,
    pitch_rad: 0.86,
    distance_m: 5.0,
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

pub(crate) fn phase_name(observation: OfficeAgvDeskPlaceObservation) -> String {
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

/// Adds a small-warehouse AGV assembly (chassis, four wheels, front bumper,
/// a LiDAR mast/puck, a status light strip, and a lift deck) following the
/// exact simulated base pose. Every part transform is derived from `center`
/// and `yaw`, so the render-only assembly tracks the same pose a single box
/// proxy would have used.
fn push_agv(
    scene: &mut RenderScene,
    center: Vec3,
    yaw: f64,
    body: [f32; 4],
    accent: [f32; 4],
    light: [f32; 4],
) {
    let rot = Quat::from_rotation_y(yaw);
    let at = |local: Vec3| center + rot * local;
    let vertical = (rot * Quat::from_rotation_x(FRAC_PI_2)).normalize();
    let axle = (rot * Quat::from_rotation_y(FRAC_PI_2)).normalize();

    // Lower chassis deck and a slightly inset upper deck give the silhouette
    // a bevelled, rounded-looking profile instead of a single flat box.
    push_box_material(
        scene,
        at(Vec3::new(0.0, -0.08, 0.0)),
        Vec3::new(0.56, 0.16, 0.44),
        rot,
        body,
        PbrMaterial::new(body, 0.42, 0.28, [0.0; 3]),
    );
    push_box_material(
        scene,
        at(Vec3::new(0.0, 0.045, 0.0)),
        Vec3::new(0.46, 0.10, 0.36),
        rot,
        body,
        PbrMaterial::new(body, 0.36, 0.24, [0.0; 3]),
    );
    push_box_material(
        scene,
        at(Vec3::new(0.0, 0.005, 0.0)),
        Vec3::new(0.58, 0.018, 0.46),
        rot,
        accent,
        PbrMaterial::new(accent, 0.55, 0.10, [0.0; 3]),
    );
    // Lift deck carrying the cargo tote.
    push_box_material(
        scene,
        at(Vec3::new(0.0, 0.108, 0.0)),
        Vec3::new(0.40, 0.020, 0.32),
        rot,
        accent,
        PbrMaterial::new(accent, 0.30, 0.55, [0.0; 3]),
    );
    // Front bumper.
    push_box_material(
        scene,
        at(Vec3::new(0.275, -0.03, 0.0)),
        Vec3::new(0.045, 0.11, 0.40),
        rot,
        accent,
        PbrMaterial::new(accent, 0.20, 0.55, [0.0; 3]),
    );
    // Status light strips along both long edges of the upper deck.
    for side in [-1.0, 1.0] {
        push_box_material(
            scene,
            at(Vec3::new(-0.02, 0.098, side * 0.187)),
            Vec3::new(0.40, 0.014, 0.014),
            rot,
            light,
            PbrMaterial::new(
                light,
                0.15,
                0.05,
                [light[0] * 1.3, light[1] * 1.3, light[2] * 1.3],
            ),
        );
    }
    // LiDAR mast and puck.
    push_cylinder(
        scene,
        at(Vec3::new(-0.13, 0.155, 0.0)),
        0.014,
        0.13,
        vertical,
        [0.12, 0.12, 0.14, 1.0],
    );
    push_cylinder(
        scene,
        at(Vec3::new(-0.13, 0.232, 0.0)),
        0.034,
        0.030,
        vertical,
        [0.05, 0.05, 0.06, 1.0],
    );
    push_sphere(scene, at(Vec3::new(-0.13, 0.232, 0.0)), 0.009, accent);
    // Four wheels/casters with an axle aligned across the chassis width.
    for (dx, dz) in [
        (-0.205, 0.19),
        (-0.205, -0.19),
        (0.205, 0.19),
        (0.205, -0.19),
    ] {
        push_cylinder(
            scene,
            at(Vec3::new(dx, -0.146, dz)),
            0.074,
            0.055,
            axle,
            [0.045, 0.045, 0.05, 1.0],
        );
        push_sphere(
            scene,
            at(Vec3::new(dx, -0.146, dz)),
            0.014,
            [0.55, 0.57, 0.60, 1.0],
        );
    }
}

/// Adds a floor with tiled patches of alternating albedo so the surface
/// reads as carpet/tile instead of one flat slab.
fn push_floor_tiles(
    scene: &mut RenderScene,
    center: Vec3,
    tile_size: (f64, f64),
    counts: (i32, i32),
    y_m: f64,
    tones: ([f32; 4], [f32; 4]),
) {
    let (tw, td) = tile_size;
    let (cols, rows) = counts;
    let origin_x = center.x - (cols as f64) * tw / 2.0 + tw / 2.0;
    let origin_z = center.z - (rows as f64) * td / 2.0 + td / 2.0;
    for row in 0..rows {
        for col in 0..cols {
            let color = if (row + col) % 2 == 0 {
                tones.0
            } else {
                tones.1
            };
            push_box_material(
                scene,
                Vec3::new(origin_x + col as f64 * tw, y_m, origin_z + row as f64 * td),
                Vec3::new(tw * 0.97, 0.006, td * 0.97),
                Quat::IDENTITY,
                color,
                PbrMaterial::new(color, 0.72, 0.03, [0.0; 3]),
            );
        }
    }
}

/// Adds a desk with legs, a monitor, and a keyboard at a fixed footprint,
/// plus a nearby chair. Static office furniture, so world-space coordinates
/// are authored directly rather than derived from a moving observation.
fn push_desk(scene: &mut RenderScene) {
    let wood: [f32; 4] = [0.66, 0.49, 0.32, 1.0];
    let metal: [f32; 4] = [0.16, 0.17, 0.19, 1.0];
    let screen: [f32; 4] = [0.05, 0.10, 0.15, 1.0];

    push_box_material(
        scene,
        Vec3::new(7.45, 0.78, 0.0),
        Vec3::new(0.76, 0.045, 1.46),
        Quat::IDENTITY,
        wood,
        PbrMaterial::new(wood, 0.42, 0.10, [0.0; 3]),
    );
    for (dx, dz) in [(-0.33, 0.62), (-0.33, -0.62), (0.33, 0.62), (0.33, -0.62)] {
        push_cylinder(
            scene,
            Vec3::new(7.45 + dx, 0.38, dz),
            0.025,
            0.74,
            Quat::IDENTITY,
            metal,
        );
    }
    // Monitor: stand + screen with a faint "on" glow.
    push_box_material(
        scene,
        Vec3::new(7.30, 0.845, 0.0),
        Vec3::new(0.03, 0.09, 0.03),
        Quat::IDENTITY,
        metal,
        PbrMaterial::new(metal, 0.30, 0.65, [0.0; 3]),
    );
    push_box_material(
        scene,
        Vec3::new(7.26, 1.02, 0.0),
        Vec3::new(0.025, 0.30, 0.44),
        Quat::IDENTITY,
        screen,
        PbrMaterial::new(screen, 0.20, 0.10, [0.03, 0.09, 0.16]),
    );
    // Keyboard.
    push_box(
        scene,
        Vec3::new(7.55, 0.815, 0.0),
        Vec3::new(0.30, 0.02, 0.14),
        [0.20, 0.21, 0.23, 1.0],
    );
    // Chair on the far side of the desk, tucked away from the AGV aisle.
    let chair: [f32; 4] = [0.14, 0.18, 0.26, 1.0];
    push_box(
        scene,
        Vec3::new(7.45, 0.46, 0.98),
        Vec3::new(0.42, 0.05, 0.42),
        chair,
    );
    push_box(
        scene,
        Vec3::new(7.45, 0.72, 1.16),
        Vec3::new(0.42, 0.46, 0.05),
        chair,
    );
    for (dx, dz) in [(-0.18, 0.80), (-0.18, 1.16), (0.18, 0.80), (0.18, 1.16)] {
        push_cylinder(
            scene,
            Vec3::new(7.45 + dx, 0.23, dz),
            0.018,
            0.46,
            Quat::IDENTITY,
            metal,
        );
    }
}

/// Adds a shelving unit with a potted plant near the pickup dock.
fn push_shelf_and_plant(scene: &mut RenderScene) {
    let shelf: [f32; 4] = [0.42, 0.46, 0.51, 1.0];
    push_box(
        scene,
        Vec3::new(1.55, 0.90, 1.00),
        Vec3::new(0.40, 1.65, 0.32),
        shelf,
    );
    for y in [0.30, 0.70, 1.10, 1.50] {
        push_box(
            scene,
            Vec3::new(1.55, y, 1.00),
            Vec3::new(0.42, 0.025, 0.34),
            [0.30, 0.33, 0.37, 1.0],
        );
    }
    for (idx, y) in [0.42, 0.82, 1.22].into_iter().enumerate() {
        let tone = if idx % 2 == 0 {
            [0.62, 0.30, 0.10, 1.0]
        } else {
            [0.10, 0.28, 0.46, 1.0]
        };
        push_box(
            scene,
            Vec3::new(1.42, y, 1.00),
            Vec3::new(0.14, 0.18, 0.20),
            tone,
        );
    }
    // Potted plant.
    let pot: [f32; 4] = [0.58, 0.32, 0.19, 1.0];
    push_cylinder(
        scene,
        Vec3::new(1.95, 0.14, 1.02),
        0.11,
        0.28,
        Quat::IDENTITY,
        pot,
    );
    let leaf_dark: [f32; 4] = [0.10, 0.40, 0.16, 1.0];
    let leaf_light: [f32; 4] = [0.20, 0.56, 0.24, 1.0];
    push_sphere(scene, Vec3::new(1.95, 0.34, 1.02), 0.13, leaf_dark);
    push_sphere(scene, Vec3::new(1.88, 0.46, 0.97), 0.10, leaf_light);
    push_sphere(scene, Vec3::new(2.02, 0.44, 1.08), 0.10, leaf_light);
    push_sphere(scene, Vec3::new(1.96, 0.52, 1.01), 0.09, leaf_dark);
}

/// Adds a low frosted-glass partition divider off the AGV's driving lane.
///
/// The renderer used for this showcase composites opaque geometry only (no
/// alpha blending), so a translucent-looking glass panel is approximated
/// with a pale, low, glossy opaque pane rather than a true alpha value —
/// a large near-white alpha-blended pane would otherwise render as a solid
/// opaque slab and block the shot.
fn push_glass_partition(scene: &mut RenderScene) {
    let glass: [f32; 4] = [0.80, 0.92, 0.95, 1.0];
    push_box_material(
        scene,
        Vec3::new(6.35, 0.42, 1.02),
        Vec3::new(0.62, 0.62, 0.025),
        Quat::IDENTITY,
        glass,
        PbrMaterial::new(glass, 0.04, 0.05, [0.0; 3]),
    );
    let frame: [f32; 4] = [0.30, 0.32, 0.35, 1.0];
    for dx in [-0.31, 0.31] {
        push_box(
            scene,
            Vec3::new(6.35 + dx, 0.42, 1.02),
            Vec3::new(0.025, 0.64, 0.04),
            frame,
        );
    }
}

/// Adds a painted dock outline and corner markers at the pickup dock.
fn push_dock_markings(scene: &mut RenderScene) {
    let paint: [f32; 4] = [0.96, 0.72, 0.10, 1.0];
    for x in [2.10, 2.90] {
        push_box(
            scene,
            Vec3::new(x, 0.029, 0.0),
            Vec3::new(0.03, 0.006, 1.05),
            paint,
        );
    }
    for z in [-0.52, 0.52] {
        push_box(
            scene,
            Vec3::new(2.5, 0.029, z),
            Vec3::new(0.83, 0.006, 0.03),
            paint,
        );
    }
}

pub(crate) fn render_scene(
    scenario: &OfficeAgvDeskPlaceScenario,
    observation: OfficeAgvDeskPlaceObservation,
) -> RenderScene {
    let mut scene = build_visual_render_scene(scenario.simulation().world());
    // The authored walls and the flat collision-box desk are collision
    // boundaries, but a low eye-level showcase camera would hide every
    // actor behind them, and a bare box desk reads poorly. Drop both render
    // items and rebuild richer render-only geometry from the same footprint.
    scene.items.retain(|item| {
        let is_wall = matches!(
            item.shape,
            VisualShape::Box { size_m } if size_m.x > 5.0 && size_m.z < 0.2
        );
        let is_flat_desk = matches!(
            item.shape,
            VisualShape::Box { size_m }
                if (size_m.x - 0.7).abs() < 0.01
                    && (size_m.y - 0.8).abs() < 0.01
                    && (size_m.z - 1.4).abs() < 0.01
        );
        !(is_wall || is_flat_desk)
    });
    // Extend the authored corridor floor toward the close camera. The source
    // scene deliberately stops at the south wall; the extension keeps the
    // lower half of the poster an office floor instead of a background void.
    push_box(
        &mut scene,
        Vec3::new(4.65, -0.035, 2.25),
        Vec3::new(6.0, 0.05, 4.5),
        [0.80, 0.78, 0.73, 1.0],
    );
    push_floor_tiles(
        &mut scene,
        Vec3::new(4.65, 0.022, 0.0),
        (0.9, 0.95),
        (7, 2),
        0.022,
        ([0.84, 0.82, 0.77, 1.0], [0.77, 0.75, 0.70, 1.0]),
    );
    push_floor_tiles(
        &mut scene,
        Vec3::new(4.65, -0.006, 2.1),
        (1.1, 1.05),
        (6, 3),
        -0.006,
        ([0.83, 0.81, 0.76, 1.0], [0.76, 0.74, 0.69, 1.0]),
    );
    let (cargo_x_m, cargo_z_m) = scenario.cargo_translation_m();
    push_agv(
        &mut scene,
        Vec3::new(observation.base_x_m, 0.24, observation.base_z_m),
        observation.base_yaw_rad,
        [0.94, 0.33, 0.07, 1.0],
        [0.14, 0.15, 0.17, 1.0],
        [1.0, 0.55, 0.06, 1.0],
    );
    // The oncoming AGV and cargo are intentionally render-only proxies whose
    // transforms are copied from the scenario observation on every frame.
    push_agv(
        &mut scene,
        Vec3::new(observation.other_agv_x_m, 0.24, 0.0),
        std::f64::consts::PI,
        [0.10, 0.32, 0.72, 1.0],
        [0.13, 0.14, 0.16, 1.0],
        [0.10, 0.55, 0.92, 1.0],
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
            [0.32, 0.38, 0.45, 1.0],
        );
    }
    // Far-side office wall and ceiling fixtures remove the empty sky band in
    // the poster while keeping the driving aisle open to the camera.
    push_box(
        &mut scene,
        Vec3::new(4.6, 1.80, -1.08),
        Vec3::new(5.9, 3.60, 0.08),
        [0.38, 0.44, 0.52, 1.0],
    );
    for x_m in [2.9, 4.4, 5.9, 7.2] {
        push_box_material(
            &mut scene,
            Vec3::new(x_m, 1.58, -1.02),
            Vec3::new(0.72, 0.10, 0.04),
            Quat::IDENTITY,
            [0.96, 0.98, 1.0, 1.0],
            PbrMaterial::new([0.96, 0.98, 1.0, 1.0], 0.30, 0.02, [0.55, 0.57, 0.60]),
        );
        push_box(
            &mut scene,
            Vec3::new(x_m, 1.28, -1.035),
            Vec3::new(0.56, 0.34, 0.025),
            [0.09, 0.36, 0.50, 1.0],
        );
    }
    push_dock_markings(&mut scene);
    // Yield line at the shared aisle and a desk-top/monitor silhouette at the
    // destination make the mission semantics readable without text labels.
    push_box(
        &mut scene,
        Vec3::new(3.45, 0.035, 0.0),
        Vec3::new(0.06, 0.025, 1.60),
        [0.96, 0.70, 0.08, 1.0],
    );
    push_desk(&mut scene);
    push_shelf_and_plant(&mut scene);
    push_glass_partition(&mut scene);
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

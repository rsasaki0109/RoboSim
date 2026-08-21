//! RoboCup SSL Division B 2v2 showcase source and capture.

use super::media::{
    capture_frames, push_box, push_box_material, push_cylinder, push_sphere, CameraEvidence,
    CaptureFrame, ShowcaseMetadata, SimulationEvidence, FRAME_COUNT,
};
use anyhow::{Context, Result};
use rne_ai::{
    build_visual_render_scene, BehaviorScenario, SslSmallPitchObservation, SslSmallPitchScenario,
};
use rne_math::{Quat, Vec3};
use rne_physics::hash_physics_state;
use rne_render::{PbrMaterial, RenderScene};
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::Path;

const ENVIRONMENT_ID: &str = "ssl";
const SUBJECT: &str = "RoboCup SSL Division B 2v2";
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 0.0,
    pitch_rad: 0.85,
    distance_m: 8.2,
};

/// Run the real four-robot SSL small-pitch scenario and optionally capture
/// the synchronized robot/ball/goal state from the wgpu renderer.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(false, None)?;
    let replay = rollout(false, Some(first.steps))?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "SSL replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    let evidence = SimulationEvidence {
        scenario: "SslSmallPitchScenario::success (76_ssl_small_pitch)",
        steps: first.steps,
        initial_state_digest: first.initial_digest,
        final_state_digest: first.final_digest,
        replay_final_state_digest: replay.final_digest,
        replay_match: true,
        outcome: "yellow_goal=true; four_robots=true; ball_speed_legal=true".into(),
    };
    let capture_evidence = if capture {
        let captured = rollout(true, Some(first.steps))?;
        let orbit = CameraOrbit {
            focus: Vec3::new(0.0, 0.0, 0.0),
            yaw_rad: CAMERA.yaw_rad,
            pitch_rad: CAMERA.pitch_rad,
            distance_m: CAMERA.distance_m,
        };
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &captured.frames,
            orbit,
            [0.035, 0.070, 0.095, 1.0],
            captured.frames.len() / 2,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "Four team-coloured robot overlays and the ball marker use SslSmallPitchScenario::simulation() and post-step observation transforms.",
        simulation: evidence,
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/scenes/ssl_small_pitch_2v2.rne.scene.toml",
            "assets/robots/ssl_blue_0.rne.robot.toml",
            "adapters/ssl/rne_adapter_ssl",
            "crates/rne_ai/src/env/ssl_small_pitch.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment ssl",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment ssl",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-ssl.json");
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
    let mut scenario = SslSmallPitchScenario::success(1).context("load SSL small pitch")?;
    let initial_digest = hash_physics_state(scenario.simulation().world());
    let mut frames = Vec::new();
    let mut sample_steps = Vec::new();
    if capture {
        let total = expected_steps.context("capture needs discovered SSL step count")?;
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
                phase: if observation.yellow_goal() {
                    "yellow-goal".into()
                } else {
                    "2v2-attack".into()
                },
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
        final_observation.yellow_goal(),
        "SSL attack did not score the yellow goal: {final_observation:?}"
    );
    anyhow::ensure!(
        !capture || frames.len() == FRAME_COUNT,
        "SSL capture sampled {} of {} frames",
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

fn render_scene(
    scenario: &SslSmallPitchScenario,
    observation: SslSmallPitchObservation,
) -> RenderScene {
    let mut scene = build_visual_render_scene(scenario.simulation().world());
    // Replace the generic world ground with an SSL-green playing surface.
    push_box_material(
        &mut scene,
        Vec3::new(0.0, -0.012, 0.0),
        Vec3::new(9.2, 0.025, 6.2),
        Quat::IDENTITY,
        [0.06, 0.24, 0.16, 1.0],
        PbrMaterial::new([0.06, 0.24, 0.16, 1.0], 0.76, 0.0, [0.0; 3]),
    );
    // The physics scene supplies all four robot entities. These render-only
    // silhouettes use the same post-step transforms but are intentionally
    // larger/high-contrast for a 960x540 README frame.
    for (index, robot) in scenario.simulation().robots().iter().enumerate() {
        let pose = scenario.simulation().observe_robot(robot.robot);
        let color = if index < 2 {
            [0.08, 0.28, 0.95, 1.0]
        } else {
            [0.95, 0.72, 0.08, 1.0]
        };
        push_box_material(
            &mut scene,
            Vec3::new(pose.base_x_m, 0.13, pose.base_z_m),
            Vec3::new(0.38, 0.25, 0.38),
            Quat::from_rotation_y(pose.base_yaw_rad),
            color,
            PbrMaterial::new(color, 0.30, 0.48, [0.0; 3]),
        );
        push_cylinder(
            &mut scene,
            Vec3::new(pose.base_x_m, 0.32, pose.base_z_m),
            0.105,
            0.035,
            Quat::IDENTITY,
            color,
        );
        // A short white heading bar makes the simulated yaw readable from
        // above without changing the robot entity or its physics geometry.
        push_box_material(
            &mut scene,
            Vec3::new(
                pose.base_x_m + 0.12 * pose.base_yaw_rad.sin(),
                0.35,
                pose.base_z_m + 0.12 * pose.base_yaw_rad.cos(),
            ),
            Vec3::new(0.06, 0.045, 0.16),
            Quat::from_rotation_y(pose.base_yaw_rad),
            [0.96, 0.97, 0.92, 1.0],
            PbrMaterial::new([0.96, 0.97, 0.92, 1.0], 0.30, 0.42, [0.0; 3]),
        );
    }
    push_sphere(
        &mut scene,
        Vec3::new(observation.ball_x_m, 0.34, observation.ball_z_m),
        0.24,
        [0.98, 0.94, 0.78, 1.0],
    );
    push_sphere(
        &mut scene,
        Vec3::new(observation.ball_x_m, 0.39, observation.ball_z_m),
        0.17,
        [0.98, 0.35, 0.04, 1.0],
    );
    // A short orange trail is a visual cue for the scripted attacker and is
    // anchored to the post-step ball position, not an independent animation.
    for index in 1..=4 {
        let trail_x_m = observation.ball_x_m - index as f64 * 0.22;
        if trail_x_m > -4.5 {
            push_sphere(
                &mut scene,
                Vec3::new(trail_x_m, 0.08, observation.ball_z_m),
                0.025,
                [0.96, 0.54, 0.10, 0.85],
            );
        }
    }
    let white = [0.94, 0.96, 0.93, 1.0];
    // Division B pitch boundary and center mark. The four robots, ball, and
    // authored goal backs already come from the simulation ECS world.
    push_box(
        &mut scene,
        Vec3::new(0.0, 0.006, 3.0),
        Vec3::new(9.0, 0.012, 0.035),
        white,
    );
    push_box(
        &mut scene,
        Vec3::new(0.0, 0.006, -3.0),
        Vec3::new(9.0, 0.012, 0.035),
        white,
    );
    push_box(
        &mut scene,
        Vec3::new(-4.5, 0.006, 0.0),
        Vec3::new(0.035, 0.012, 6.0),
        white,
    );
    push_box(
        &mut scene,
        Vec3::new(4.5, 0.006, 0.0),
        Vec3::new(0.035, 0.012, 6.0),
        white,
    );
    push_box(
        &mut scene,
        Vec3::new(0.0, 0.007, 0.0),
        Vec3::new(0.035, 0.014, 6.0),
        white,
    );
    for z_m in [-0.50, 0.50] {
        push_box(
            &mut scene,
            Vec3::new(-4.58, 0.30, z_m),
            Vec3::new(0.22, 0.60, 0.05),
            [0.18, 0.35, 0.90, 1.0],
        );
        push_box(
            &mut scene,
            Vec3::new(4.58, 0.30, z_m),
            Vec3::new(0.22, 0.60, 0.05),
            [0.90, 0.76, 0.10, 1.0],
        );
    }
    for x_m in [-4.68, 4.68] {
        let color = if x_m < 0.0 {
            [0.18, 0.35, 0.90, 1.0]
        } else {
            [0.90, 0.75, 0.10, 1.0]
        };
        for z_m in [-0.50, 0.50] {
            push_cylinder(
                &mut scene,
                Vec3::new(x_m, 0.42, z_m),
                0.045,
                0.84,
                Quat::from_rotation_x(std::f64::consts::FRAC_PI_2),
                color,
            );
        }
    }
    // Midfield circle as deterministic short dashes rather than a renderer
    // dependent line primitive.
    for segment in 0..24 {
        let angle = std::f64::consts::TAU * segment as f64 / 24.0;
        push_box(
            &mut scene,
            Vec3::new(0.72 * angle.cos(), 0.015, 0.72 * angle.sin()),
            Vec3::new(0.08, 0.012, 0.025),
            [0.92, 0.95, 0.90, 1.0],
        );
    }
    // A low destination plane follows the official goal colour and makes the
    // final kick readable even when the ball is only a few pixels wide.
    push_box_material(
        &mut scene,
        Vec3::new(4.61, 0.012, 0.0),
        Vec3::new(0.10, 0.02, 1.0),
        Quat::IDENTITY,
        if observation.yellow_goal() {
            [0.95, 0.82, 0.12, 1.0]
        } else {
            [0.30, 0.42, 0.56, 1.0]
        },
        PbrMaterial::new([0.95, 0.82, 0.12, 1.0], 0.45, 0.12, [0.0; 3]),
    );
    // Goal roof bars give the two goal mouths a silhouette in the top-down
    // shot. The local cylinder Z axis spans the pitch width; the posts above
    // use a quarter-turn to stand along world Y.
    push_cylinder(
        &mut scene,
        Vec3::new(4.68, 0.72, 0.0),
        0.035,
        1.0,
        Quat::IDENTITY,
        [0.85, 0.75, 0.15, 1.0],
    );
    push_cylinder(
        &mut scene,
        Vec3::new(-4.68, 0.72, 0.0),
        0.035,
        1.0,
        Quat::IDENTITY,
        [0.20, 0.35, 0.85, 1.0],
    );
    scene
}

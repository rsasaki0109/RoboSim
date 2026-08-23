//! OpenArm v2 bimanual-control showcase source and capture.

use super::media::{
    capture_frames, CameraEvidence, CaptureFrame, ShowcaseMetadata, SimulationEvidence,
};
use anyhow::{Context, Result};
use rne_ai::{build_visual_render_scene, UrdfJointPositionTarget, UrdfSceneSim};
use rne_math::Vec3;
use rne_physics::hash_physics_state;
use rne_render::{MeshRenderCache, RenderScene};
use rne_render_wgpu::CameraOrbit;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::{Path, PathBuf};

const ENVIRONMENT_ID: &str = "openarm";
const SUBJECT: &str = "Official OpenArm v2 bimanual control";
const CAPTURE_STEPS: u64 = 1_400;
const CAPTURE_FRAME_COUNT: usize = 46;
const CAPTURE_STRIDE: u64 = CAPTURE_STEPS / CAPTURE_FRAME_COUNT as u64;
const CAMERA: CameraEvidence = CameraEvidence {
    fov_y_rad: std::f64::consts::FRAC_PI_4,
    yaw_rad: 2.42,
    pitch_rad: 1.10,
    distance_m: 1.68,
};

const LEFT_LINKS: [&str; 9] = [
    "openarm_left_link1",
    "openarm_left_link2",
    "openarm_left_link3",
    "openarm_left_link4",
    "openarm_left_link5",
    "openarm_left_link6",
    "openarm_left_ee_base_link",
    "openarm_left_ee_link1",
    "openarm_left_ee_link2",
];
const RIGHT_LINKS: [&str; 9] = [
    "openarm_right_link1",
    "openarm_right_link2",
    "openarm_right_link3",
    "openarm_right_link4",
    "openarm_right_link5",
    "openarm_right_link6",
    "openarm_right_ee_base_link",
    "openarm_right_ee_link1",
    "openarm_right_ee_link2",
];

#[derive(Clone, Copy, Debug)]
struct BimanualPose {
    left: [f64; 9],
    right: [f64; 9],
}

const HOME: BimanualPose = BimanualPose {
    left: [
        0.0,
        0.0,
        0.0,
        std::f64::consts::FRAC_PI_2,
        0.0,
        0.0,
        0.0,
        0.55,
        0.55,
    ],
    right: [
        0.0,
        0.0,
        0.0,
        std::f64::consts::FRAC_PI_2,
        0.0,
        0.0,
        0.0,
        -0.55,
        -0.55,
    ],
};
const APPROACH: BimanualPose = BimanualPose {
    left: [-1.20, -1.10, 0.60, 2.00, -0.60, 0.50, 0.40, 0.55, 0.55],
    right: [1.20, 1.10, -0.60, 2.00, 0.60, -0.50, -0.40, -0.55, -0.55],
};
const CLOSE: BimanualPose = BimanualPose {
    left: [-1.20, -1.10, 0.60, 2.00, -0.60, 0.50, 0.40, 0.08, 0.08],
    right: [1.20, 1.10, -0.60, 2.00, 0.60, -0.50, -0.40, -0.08, -0.08],
};
const RAISE: BimanualPose = BimanualPose {
    left: [-1.60, -0.80, 0.50, 1.30, -0.50, 0.30, 0.40, 0.08, 0.08],
    right: [1.60, 0.80, -0.50, 1.30, 0.50, -0.30, -0.40, -0.08, -0.08],
};
const PRESENT: BimanualPose = BimanualPose {
    left: [-2.40, -1.40, 1.00, 1.00, -1.00, 0.60, 0.80, 0.08, 0.08],
    right: [2.40, 1.40, -1.00, 1.00, 1.00, -0.60, -0.80, -0.08, -0.08],
};

/// Runs a force-limited OpenArm bimanual joint-space cycle and optionally
/// renders evenly sampled post-step states using the same Rapier rollout.
pub fn run(repo_root: &Path, capture: bool) -> Result<ShowcaseMetadata> {
    let first = rollout(repo_root, capture)?;
    let replay = rollout(repo_root, false)?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest,
        "OpenArm replay digest mismatch: {:#x} != {:#x}",
        first.final_digest,
        replay.final_digest
    );
    anyhow::ensure!(
        first.actuated_joint_count == 18,
        "OpenArm must expose 18 actuators"
    );
    anyhow::ensure!(
        first.mesh_items >= 45,
        "OpenArm resolved only {} visual mesh parts",
        first.mesh_items
    );
    anyhow::ensure!(
        first.left_end_effector_travel_m >= 0.16 && first.right_end_effector_travel_m >= 0.16,
        "OpenArm end effectors did not visibly move: left={:.3} m right={:.3} m",
        first.left_end_effector_travel_m,
        first.right_end_effector_travel_m
    );
    anyhow::ensure!(
        first.max_final_proximal_joint_error_rad <= 0.13,
        "OpenArm final proximal-joint tracking error exceeded 0.13 rad: {:.4}",
        first.max_final_proximal_joint_error_rad
    );
    anyhow::ensure!(
        first.left_gripper_aperture_change_m >= 0.015
            && first.right_gripper_aperture_change_m >= 0.015,
        "OpenArm grippers did not visibly actuate: left={:.4} m right={:.4} m",
        first.left_gripper_aperture_change_m,
        first.right_gripper_aperture_change_m
    );
    if capture {
        anyhow::ensure!(
            first.frames.len() == CAPTURE_FRAME_COUNT,
            "OpenArm capture must contain {CAPTURE_FRAME_COUNT} frames"
        );
    }

    let capture_evidence = if capture {
        Some(capture_frames(
            repo_root,
            ENVIRONMENT_ID,
            &first.frames,
            CameraOrbit {
                focus: Vec3::new(0.0, 0.68, 0.06),
                yaw_rad: CAMERA.yaw_rad,
                pitch_rad: CAMERA.pitch_rad,
                distance_m: CAMERA.distance_m,
            },
            [0.025, 0.035, 0.055, 1.0],
            26,
        )?)
    } else {
        None
    };
    let metadata = ShowcaseMetadata {
        kind: "rne_showcase_environment_metadata",
        schema_version: 1,
        environment_id: ENVIRONMENT_ID,
        subject: SUBJECT,
        visual_state_sync: "Official OpenArm link visuals are rebuilt from the post-Rapier ECS world after every sampled fixed step.",
        simulation: SimulationEvidence {
            scenario: "OpenArm v2 bimanual force-limited position-control cycle",
            steps: CAPTURE_STEPS,
            initial_state_digest: first.initial_digest,
            final_state_digest: first.final_digest,
            replay_final_state_digest: replay.final_digest,
            replay_match: true,
            outcome: format!(
                "actuators={}; left_ee_travel_m={:.4}; right_ee_travel_m={:.4}; left_gripper_aperture_change_m={:.4}; right_gripper_aperture_change_m={:.4}; max_final_proximal_joint_error_rad={:.5}; visual_mesh_parts={}",
                first.actuated_joint_count,
                first.left_end_effector_travel_m,
                first.right_end_effector_travel_m,
                first.left_gripper_aperture_change_m,
                first.right_gripper_aperture_change_m,
                first.max_final_proximal_joint_error_rad,
                first.mesh_items
            ),
        },
        capture: capture_evidence,
        camera: CAMERA,
        provenance: vec![
            "assets/robots/openarm_description/PROVENANCE.md",
            "assets/robots/openarm_description/LICENSE.openarm_description",
            "assets/robots/openarm_description/openarm_v2.rne.urdf",
            "assets/robots/openarm_v2_left.rne.robot.toml",
            "assets/robots/openarm_v2_right.rne.robot.toml",
            "assets/scenes/openarm_v2_showcase.rne.scene.toml",
            "examples/90_showcase_captures/openarm.rs",
        ],
        reproduce_smoke: "cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment openarm",
        reproduce_capture: "cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment openarm",
    };
    if capture {
        let path = repo_root.join("docs/media/showcase-openarm.json");
        fs::write(&path, to_vec_pretty(&metadata)?)
            .with_context(|| format!("write {}", path.display()))?;
    }
    Ok(metadata)
}

struct Rollout {
    initial_digest: u64,
    final_digest: u64,
    actuated_joint_count: usize,
    left_end_effector_travel_m: f64,
    right_end_effector_travel_m: f64,
    left_gripper_aperture_change_m: f64,
    right_gripper_aperture_change_m: f64,
    max_final_proximal_joint_error_rad: f64,
    mesh_items: usize,
    frames: Vec<CaptureFrame>,
}

fn rollout(repo_root: &Path, capture: bool) -> Result<Rollout> {
    let scene_path = repo_root.join("assets/scenes/openarm_v2_showcase.rne.scene.toml");
    let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations(&scene_path, 16)
        .context("load official OpenArm v2 bimanual scene")?;
    configure_official_position_motors(&mut sim)?;
    let observation = sim.observe();
    let initial_digest = hash_physics_state(sim.world());
    let left_start = link_position(&sim, "openarm_left_ee_base_link")?;
    let right_start = link_position(&sim, "openarm_right_ee_base_link")?;
    let mut left_travel_m: f64 = 0.0;
    let mut right_travel_m: f64 = 0.0;
    let mut left_aperture_min_m = f64::INFINITY;
    let mut left_aperture_max_m = 0.0_f64;
    let mut right_aperture_min_m = f64::INFINITY;
    let mut right_aperture_max_m = 0.0_f64;
    let mut frames = Vec::new();
    let mut cache = MeshRenderCache::new();
    let mut mesh_items = 0;
    for step in 1..=CAPTURE_STEPS {
        let pose = commanded_pose(step);
        let targets = targets_for_pose(&pose);
        sim.step_joint_position_targets(&targets);
        left_travel_m = left_travel_m
            .max((link_position(&sim, "openarm_left_ee_base_link")? - left_start).length());
        right_travel_m = right_travel_m
            .max((link_position(&sim, "openarm_right_ee_base_link")? - right_start).length());
        let left_aperture_m =
            finger_aperture_m(&sim, "openarm_left_ee_link1", "openarm_left_ee_link2")?;
        let right_aperture_m =
            finger_aperture_m(&sim, "openarm_right_ee_link1", "openarm_right_ee_link2")?;
        left_aperture_min_m = left_aperture_min_m.min(left_aperture_m);
        left_aperture_max_m = left_aperture_max_m.max(left_aperture_m);
        right_aperture_min_m = right_aperture_min_m.min(right_aperture_m);
        right_aperture_max_m = right_aperture_max_m.max(right_aperture_m);
        if capture && step % CAPTURE_STRIDE == 0 {
            let (scene, current_mesh_items) = render_scene(&sim, &mut cache)?;
            mesh_items = mesh_items.max(current_mesh_items);
            frames.push(CaptureFrame {
                step,
                phase: phase_label(step).to_string(),
                scene,
            });
        }
    }
    if !capture {
        let (_, current_mesh_items) = render_scene(&sim, &mut cache)?;
        mesh_items = current_mesh_items;
    }
    let final_targets = targets_for_pose(&HOME);
    let max_final_proximal_joint_error_rad = final_targets
        .iter()
        .filter(|target| {
            !target.link_name.contains("ee_link") && !target.link_name.contains("ee_base_link")
        })
        .filter_map(|target| {
            sim.named_joint_position(target.link_name)
                .map(|position| (position - target.position).abs())
        })
        .fold(0.0_f64, f64::max);
    Ok(Rollout {
        initial_digest,
        final_digest: hash_physics_state(sim.world()),
        actuated_joint_count: observation.actuated_joint_count,
        left_end_effector_travel_m: left_travel_m,
        right_end_effector_travel_m: right_travel_m,
        left_gripper_aperture_change_m: left_aperture_max_m - left_aperture_min_m,
        right_gripper_aperture_change_m: right_aperture_max_m - right_aperture_min_m,
        max_final_proximal_joint_error_rad,
        mesh_items,
        frames,
    })
}

fn configure_official_position_motors(sim: &mut UrdfSceneSim) -> Result<()> {
    for (links, side) in [(&LEFT_LINKS, "left"), (&RIGHT_LINKS, "right")] {
        for (index, link) in links.iter().enumerate() {
            let (stiffness, damping, max_force_nm) = match index {
                0 | 1 => (230.0, 2.7, 40.0),
                2 | 3 => (190.0, 2.2, 27.0),
                4..=6 => (30.0, 1.5, 7.0),
                _ => (30.0, 0.2, 7.0),
            };
            anyhow::ensure!(
                sim.configure_named_position_motor(link, stiffness, damping, max_force_nm),
                "missing OpenArm {side} actuator link {link}"
            );
        }
    }
    Ok(())
}

fn commanded_pose(step: u64) -> BimanualPose {
    match step {
        0..=200 => interpolate(HOME, APPROACH, smooth(step as f64 / 200.0)),
        201..=360 => interpolate(APPROACH, CLOSE, smooth((step - 200) as f64 / 160.0)),
        361..=580 => interpolate(CLOSE, RAISE, smooth((step - 360) as f64 / 220.0)),
        581..=780 => interpolate(RAISE, PRESENT, smooth((step - 580) as f64 / 200.0)),
        _ => interpolate(PRESENT, HOME, smooth((step - 780) as f64 / 620.0)),
    }
}

fn interpolate(from: BimanualPose, to: BimanualPose, alpha: f64) -> BimanualPose {
    let blend = |a: [f64; 9], b: [f64; 9]| std::array::from_fn(|i| a[i] + (b[i] - a[i]) * alpha);
    BimanualPose {
        left: blend(from.left, to.left),
        right: blend(from.right, to.right),
    }
}

fn smooth(alpha: f64) -> f64 {
    let alpha = alpha.clamp(0.0, 1.0);
    alpha * alpha * (3.0 - 2.0 * alpha)
}

fn targets_for_pose(pose: &BimanualPose) -> Vec<UrdfJointPositionTarget<'static>> {
    LEFT_LINKS
        .iter()
        .zip(pose.left)
        .chain(RIGHT_LINKS.iter().zip(pose.right))
        .map(|(link_name, position)| UrdfJointPositionTarget {
            link_name,
            position,
        })
        .collect()
}

fn phase_label(step: u64) -> &'static str {
    match step {
        0..=200 => "approach",
        201..=360 => "close-grippers",
        361..=580 => "coordinated-raise",
        581..=780 => "present",
        _ => "return-home",
    }
}

fn link_position(sim: &UrdfSceneSim, link: &str) -> Result<Vec3> {
    let (x_m, y_m, z_m) = sim
        .link_translation_m(link)
        .with_context(|| format!("missing OpenArm link {link}"))?;
    Ok(Vec3::new(x_m, y_m, z_m))
}

fn finger_aperture_m(sim: &UrdfSceneSim, first_link: &str, second_link: &str) -> Result<f64> {
    let fingertip = |link: &str| -> Result<Vec3> {
        let transform = sim
            .named_transform(link)
            .with_context(|| format!("missing OpenArm finger link {link}"))?;
        Ok(transform.translation
            + transform.rotation * (Vec3::new(0.0, 0.0, -0.065) * transform.scale))
    };
    Ok((fingertip(first_link)? - fingertip(second_link)?).length())
}

fn render_scene(sim: &UrdfSceneSim, cache: &mut MeshRenderCache) -> Result<(RenderScene, usize)> {
    let mut scene = build_visual_render_scene(sim.world());
    let roots = sim.mesh_package_roots().to_vec();
    let root_refs = roots.iter().map(PathBuf::as_path).collect::<Vec<_>>();
    cache
        .resolve_scene(&mut scene, &root_refs)
        .map_err(|error| anyhow::anyhow!("resolve official OpenArm meshes: {error}"))?;
    let mesh_items = scene
        .items
        .iter()
        .filter(|item| item.mesh.is_some())
        .count();

    Ok((scene, mesh_items))
}

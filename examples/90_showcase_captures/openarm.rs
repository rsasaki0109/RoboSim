//! OpenArm v2 bimanual-control showcase source and capture.

use super::media::{
    capture_frames, push_box, CameraEvidence, CaptureFrame, ShowcaseMetadata, SimulationEvidence,
};
use anyhow::{Context, Result};
use rne_ai::{
    build_visual_render_scene, UrdfJointFeedbackSensorConfig, UrdfJointPdEffortTarget,
    UrdfJointPositionTarget, UrdfSceneSim,
};
use rne_data::{
    DataBus, Frame, InMemoryDataBus, JointCoordinateFeedback, JointFeedback, JointFeedbackStatus,
    StreamId,
};
use rne_math::Vec3;
use rne_physics::hash_physics_state;
use rne_render::{MeshRenderCache, RenderScene};
use rne_render_wgpu::CameraOrbit;
use rne_sensor::JointFeedbackFault;
use serde_json::to_vec_pretty;
use std::fs;
use std::path::{Path, PathBuf};

const ENVIRONMENT_ID: &str = "openarm";
const SUBJECT: &str = "Official OpenArm v2 bimanual control";
const CAPTURE_STEPS: u64 = 1_400;
const CAPTURE_FRAME_COUNT: usize = 46;
const CAPTURE_STRIDE: u64 = CAPTURE_STEPS / CAPTURE_FRAME_COUNT as u64;
const PHYSICS_SUBSTEPS_PER_CONTROL_STEP: usize = 19;
const JOINT_FEEDBACK_STREAM: StreamId = StreamId::new(9_090);
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
struct ActuatorProfile {
    stiffness_nm_per_rad: f64,
    damping_nm_s_per_rad: f64,
    max_effort_nm: f64,
    max_velocity_rad_s: f64,
}

#[derive(Clone, Copy, Debug, Default)]
struct ControlFrameTelemetry {
    observation_age_ticks: u64,
    tracking_error_rad: f64,
    effort_utilization: f64,
    saturated_fraction: f64,
}

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
    anyhow::ensure!(
        first.feedback_decisions >= CAPTURE_STEPS - 2,
        "OpenArm typed feedback controlled only {} of {} steps",
        first.feedback_decisions,
        CAPTURE_STEPS
    );
    anyhow::ensure!(
        first.sensor_samples == CAPTURE_STEPS,
        "OpenArm joint sensor emitted {} of {} samples",
        first.sensor_samples,
        CAPTURE_STEPS
    );
    anyhow::ensure!(
        first.max_observation_age_ticks == first.fixed_delta_ticks,
        "OpenArm feedback age must be exactly one control period: age={} period={}",
        first.max_observation_age_ticks,
        first.fixed_delta_ticks
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
        visual_state_sync: "Official OpenArm link visuals and the latency/error/effort control panel are rebuilt from the same post-Rapier fixed-step state.",
        simulation: SimulationEvidence {
            scenario: "OpenArm v2 bimanual delayed joint-feedback and portable PD-effort cycle",
            steps: CAPTURE_STEPS,
            initial_state_digest: first.initial_digest,
            final_state_digest: first.final_digest,
            replay_final_state_digest: replay.final_digest,
            replay_match: true,
            outcome: format!(
                "actuators={}; typed_feedback_decisions={}; sensor_samples={}; feedback_latency_ticks={}; max_tracking_error_rad={:.5}; saturated_channel_samples={}; left_ee_travel_m={:.4}; right_ee_travel_m={:.4}; left_gripper_aperture_change_m={:.4}; right_gripper_aperture_change_m={:.4}; max_final_proximal_joint_error_rad={:.5}; visual_mesh_parts={}",
                first.actuated_joint_count,
                first.feedback_decisions,
                first.sensor_samples,
                first.max_observation_age_ticks,
                first.max_tracking_error_rad,
                first.saturated_channel_samples,
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
    fixed_delta_ticks: u64,
    feedback_decisions: u64,
    sensor_samples: u64,
    max_observation_age_ticks: u64,
    max_tracking_error_rad: f64,
    saturated_channel_samples: u64,
    mesh_items: usize,
    frames: Vec<CaptureFrame>,
}

fn rollout(repo_root: &Path, capture: bool) -> Result<Rollout> {
    let scene_path = repo_root.join("assets/scenes/openarm_v2_showcase.rne.scene.toml");
    let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations(&scene_path, 16)
        .context("load official OpenArm v2 bimanual scene")?;
    configure_official_effort_actuators(&mut sim)?;
    let fixed_delta_ticks = sim.fixed_delta().ticks();
    let link_order = LEFT_LINKS
        .iter()
        .chain(RIGHT_LINKS.iter())
        .map(|link| (*link).to_string())
        .collect::<Vec<_>>();
    sim.install_joint_feedback_sensor(UrdfJointFeedbackSensorConfig {
        sensor_name: "openarm_bimanual_joint_feedback".into(),
        link_names: link_order.clone(),
        update_rate_hz: 60.0,
        sample_period_ticks: Some(fixed_delta_ticks),
        phase_offset_ticks: fixed_delta_ticks,
        latency_ticks: fixed_delta_ticks,
        stream_id: JOINT_FEEDBACK_STREAM,
        fault: JointFeedbackFault::None,
    })
    .context("install OpenArm bimanual joint-feedback sensor")?;
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
    let mut bus = InMemoryDataBus::new();
    let mut feedback_decisions = 0;
    let mut sensor_samples = 0;
    let mut max_observation_age_ticks = 0;
    let mut max_tracking_error_rad = 0.0_f64;
    let mut saturated_channel_samples = 0;
    let mut mesh_items = 0;
    for step in 1..=CAPTURE_STEPS {
        let pose = commanded_pose(step);
        let reference_targets = targets_for_pose(&pose);
        let visible_feedback =
            bus.latest_available::<JointFeedback>(JOINT_FEEDBACK_STREAM, sim.sim_time());
        let (controller_targets, observation_age_ticks) = feedback_adjusted_targets(
            &reference_targets,
            visible_feedback.as_ref(),
            sim.sim_time().ticks(),
            &link_order,
        )?;
        if visible_feedback.is_some() {
            feedback_decisions += 1;
            max_observation_age_ticks = max_observation_age_ticks.max(observation_age_ticks);
        }
        let effort_targets = controller_targets
            .iter()
            .enumerate()
            .map(|(index, target)| {
                let profile = actuator_profile(index);
                UrdfJointPdEffortTarget {
                    link_name: target.link_name,
                    target_position_rad: target.position,
                    stiffness_nm_per_rad: profile.stiffness_nm_per_rad,
                    damping_nm_s_per_rad: profile.damping_nm_s_per_rad,
                    max_effort_nm: profile.max_effort_nm,
                    max_velocity_rad_s: profile.max_velocity_rad_s,
                    transmission_efficiency: 1.0,
                }
            })
            .collect::<Vec<_>>();
        let applied = sim
            .step_joint_pd_effort_targets_substeps(
                &effort_targets,
                PHYSICS_SUBSTEPS_PER_CONTROL_STEP,
            )
            .context("step OpenArm portable PD-effort controller")?;
        sensor_samples += sim
            .sample_joint_feedback(&mut bus)
            .context("sample OpenArm bimanual joint feedback")? as u64;
        saturated_channel_samples += applied.iter().filter(|value| value.saturated).count() as u64;
        let tracking_error_rad = reference_targets
            .iter()
            .map(|target| {
                sim.named_joint_position(target.link_name)
                    .map(|position| (position - target.position).abs())
                    .with_context(|| format!("missing OpenArm joint {}", target.link_name))
            })
            .collect::<Result<Vec<_>>>()?
            .into_iter()
            .fold(0.0_f64, f64::max);
        max_tracking_error_rad = max_tracking_error_rad.max(tracking_error_rad);
        let effort_utilization = applied
            .iter()
            .enumerate()
            .map(|(index, value)| value.effort_nm.abs() / actuator_profile(index).max_effort_nm)
            .fold(0.0_f64, f64::max);
        let telemetry = ControlFrameTelemetry {
            observation_age_ticks,
            tracking_error_rad,
            effort_utilization,
            saturated_fraction: applied.iter().filter(|value| value.saturated).count() as f64
                / applied.len() as f64,
        };
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
            let (scene, current_mesh_items) = render_scene(&sim, &mut cache, telemetry)?;
            mesh_items = mesh_items.max(current_mesh_items);
            frames.push(CaptureFrame {
                step,
                phase: phase_label(step).to_string(),
                scene,
            });
        }
    }
    if !capture {
        let (_, current_mesh_items) =
            render_scene(&sim, &mut cache, ControlFrameTelemetry::default())?;
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
        fixed_delta_ticks,
        feedback_decisions,
        sensor_samples,
        max_observation_age_ticks,
        max_tracking_error_rad,
        saturated_channel_samples,
        mesh_items,
        frames,
    })
}

fn configure_official_effort_actuators(sim: &mut UrdfSceneSim) -> Result<()> {
    for (links, side) in [(&LEFT_LINKS, "left"), (&RIGHT_LINKS, "right")] {
        for (index, link) in links.iter().enumerate() {
            let profile = actuator_profile(index);
            anyhow::ensure!(
                sim.configure_named_revolute_effort_actuation(link, profile.max_effort_nm),
                "missing OpenArm {side} actuator link {link}"
            );
        }
    }
    Ok(())
}

fn actuator_profile(index: usize) -> ActuatorProfile {
    match index % LEFT_LINKS.len() {
        0 | 1 => ActuatorProfile {
            stiffness_nm_per_rad: 69.0,
            damping_nm_s_per_rad: 0.027,
            max_effort_nm: 40.0,
            max_velocity_rad_s: 16.755,
        },
        2 | 3 => ActuatorProfile {
            stiffness_nm_per_rad: 36.0,
            damping_nm_s_per_rad: 0.2,
            max_effort_nm: 27.0,
            max_velocity_rad_s: 5.4454,
        },
        4 => ActuatorProfile {
            stiffness_nm_per_rad: 36.0,
            damping_nm_s_per_rad: 0.2,
            max_effort_nm: 7.0,
            max_velocity_rad_s: 20.944,
        },
        5 | 6 => ActuatorProfile {
            stiffness_nm_per_rad: 12.0,
            damping_nm_s_per_rad: 0.06,
            max_effort_nm: 7.0,
            max_velocity_rad_s: 20.944,
        },
        _ => ActuatorProfile {
            stiffness_nm_per_rad: 12.0,
            damping_nm_s_per_rad: 0.03,
            max_effort_nm: 0.5,
            max_velocity_rad_s: 5.0,
        },
    }
}

fn feedback_adjusted_targets(
    references: &[UrdfJointPositionTarget<'static>],
    feedback: Option<&Frame<JointFeedback>>,
    consumed_at_ticks: u64,
    link_order: &[String],
) -> Result<(Vec<UrdfJointPositionTarget<'static>>, u64)> {
    let Some(feedback) = feedback else {
        return Ok((references.to_vec(), 0));
    };
    anyhow::ensure!(
        feedback.payload.schema_version == JointFeedback::SCHEMA_VERSION
            && feedback.payload.status == JointFeedbackStatus::Nominal,
        "OpenArm controller requires nominal joint feedback"
    );
    anyhow::ensure!(
        feedback.payload.joints.len() == references.len() && link_order.len() == references.len(),
        "OpenArm joint-feedback width mismatch"
    );
    let adjusted = references
        .iter()
        .zip(&feedback.payload.joints)
        .zip(link_order)
        .enumerate()
        .map(|(index, ((reference, joint), expected_name))| {
            anyhow::ensure!(
                joint.name == *expected_name && reference.link_name == expected_name,
                "OpenArm joint-feedback order mismatch at channel {index}"
            );
            let (position_rad, velocity_rad_s) = match joint.coordinate {
                JointCoordinateFeedback::Revolute {
                    position_rad,
                    velocity_rad_s,
                } => (position_rad, velocity_rad_s),
                _ => anyhow::bail!("OpenArm feedback channel {} is not revolute", joint.name),
            };
            let finger = index % LEFT_LINKS.len() >= 7;
            let position_gain = if finger { 0.15 } else { 0.30 };
            let velocity_damping_s = if finger { 0.001 } else { 0.002 };
            let maximum_correction_rad = if finger { 0.02 } else { 0.04 };
            let correction_rad = (position_gain * (reference.position - position_rad)
                - velocity_damping_s * velocity_rad_s)
                .clamp(-maximum_correction_rad, maximum_correction_rad);
            Ok(UrdfJointPositionTarget {
                link_name: reference.link_name,
                position: reference.position + correction_rad,
            })
        })
        .collect::<Result<Vec<_>>>()?;
    Ok((
        adjusted,
        consumed_at_ticks.saturating_sub(feedback.capture_time.ticks()),
    ))
}

fn commanded_pose(step: u64) -> BimanualPose {
    match step {
        0..=200 => interpolate(HOME, APPROACH, smooth(step as f64 / 200.0)),
        201..=360 => interpolate(APPROACH, CLOSE, smooth((step - 200) as f64 / 160.0)),
        361..=580 => interpolate(CLOSE, RAISE, smooth((step - 360) as f64 / 220.0)),
        581..=780 => interpolate(RAISE, PRESENT, smooth((step - 580) as f64 / 200.0)),
        781..=1180 => interpolate(PRESENT, HOME, smooth((step - 780) as f64 / 400.0)),
        _ => {
            let mut pose = HOME;
            let alpha = smooth((step - 1180) as f64 / 220.0);
            let finger_position = 0.55 + (0.18 - 0.55) * alpha;
            pose.left[7] = finger_position;
            pose.left[8] = finger_position;
            pose.right[7] = -finger_position;
            pose.right[8] = -finger_position;
            pose
        }
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

fn render_scene(
    sim: &UrdfSceneSim,
    cache: &mut MeshRenderCache,
    telemetry: ControlFrameTelemetry,
) -> Result<(RenderScene, usize)> {
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
    push_control_panel(&mut scene, sim.fixed_delta().ticks(), telemetry);

    Ok((scene, mesh_items))
}

fn push_control_panel(
    scene: &mut RenderScene,
    fixed_delta_ticks: u64,
    telemetry: ControlFrameTelemetry,
) {
    const PANEL_Z_M: f64 = -0.675;
    const BAR_BOTTOM_Y_M: f64 = 0.58;
    const BAR_MAX_HEIGHT_M: f64 = 0.34;
    push_box(
        scene,
        Vec3::new(-0.67, 0.77, PANEL_Z_M),
        Vec3::new(0.48, 0.48, 0.035),
        [0.025, 0.040, 0.065, 1.0],
    );
    let values = [
        if fixed_delta_ticks == 0 {
            0.0
        } else {
            telemetry.observation_age_ticks as f64 / fixed_delta_ticks as f64 / 2.0
        },
        telemetry.tracking_error_rad / 0.40,
        telemetry
            .effort_utilization
            .max(telemetry.saturated_fraction),
    ];
    let colors = [
        [0.10, 0.70, 0.95, 1.0],
        [1.00, 0.62, 0.08, 1.0],
        [0.92, 0.18, 0.16, 1.0],
    ];
    for (index, (value, color)) in values.into_iter().zip(colors).enumerate() {
        let x_m = -0.82 + index as f64 * 0.15;
        push_box(
            scene,
            Vec3::new(
                x_m,
                BAR_BOTTOM_Y_M + BAR_MAX_HEIGHT_M * 0.5,
                PANEL_Z_M + 0.025,
            ),
            Vec3::new(0.075, BAR_MAX_HEIGHT_M, 0.025),
            [0.08, 0.11, 0.16, 1.0],
        );
        let height_m = (BAR_MAX_HEIGHT_M * value.clamp(0.03, 1.0)).max(0.01);
        push_box(
            scene,
            Vec3::new(x_m, BAR_BOTTOM_Y_M + height_m * 0.5, PANEL_Z_M + 0.045),
            Vec3::new(0.055, height_m, 0.025),
            color,
        );
    }
}

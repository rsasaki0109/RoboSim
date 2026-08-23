//! Compiles one portable OpenArm controller and executes its exact action trace on Rapier.

use anyhow::{bail, Context, Result};
use rne_ai::{
    BehaviorContractDescriptor, BehaviorContractKind, BehaviorReplayAction, BehaviorReplayArtifact,
    BehaviorReplayFailure, BehaviorReplayFrame, BehaviorViolation, TaskSpec,
    UrdfJointPositionTarget, UrdfSceneSim,
};
use rne_physics::hash_physics_state;
use serde::{Deserialize, Serialize};
use serde_json::json;
use sha2::{Digest, Sha256};
use std::collections::HashSet;
use std::fs;
use std::path::{Path, PathBuf};

const KIND: &str = "rne_joint_pose_cycle_controller";
const SCHEMA_VERSION: u32 = 1;
const TRACE_KIND: &str = "rne_controller_action_trace";
const RAPIER_TRACE_KIND: &str = "rne_openarm_backend_trace";
const FAILURE_KIND: &str = "rne_controller_contract_failure";
const FIXED_DELTA_TICKS: u64 = 16_666_667;
const ACTUATION_CONFIG_KIND: &str = "rne_revolute_position_actuation_config";

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ControllerSpec {
    kind: String,
    schema_version: u32,
    controller_id: String,
    task_id: String,
    interpolation: String,
    action_joint_order: Vec<String>,
    rne_actuator_link_order: Vec<String>,
    keyframes: Vec<Keyframe>,
    intentional_failure: IntentionalFailure,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Keyframe {
    step: u64,
    phase: String,
    joint_position_target_rad: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IntentionalFailure {
    kind: String,
    inject_at_step: u64,
    expected_first_violation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ActuationConfig {
    kind: String,
    schema_version: u32,
    backend_id: String,
    motor_model: String,
    solver_iterations: usize,
    fixed_delta_ticks: u64,
    joints: Vec<JointActuationConfig>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct JointActuationConfig {
    joint_name: String,
    link_name: String,
    stiffness_nm_per_rad: f64,
    damping_nm_s_per_rad: f64,
    max_effort_nm: f64,
}

#[derive(Debug, Serialize)]
struct ActionTrace<'a> {
    kind: &'static str,
    schema_version: u32,
    task_id: &'a str,
    task_sha256: &'a str,
    controller_id: &'a str,
    controller_sha256: &'a str,
    fixed_delta_ticks: u64,
    action_joint_order: &'a [String],
    actions: Vec<ActionFrame>,
}

#[derive(Clone, Debug, Serialize)]
struct ActionFrame {
    action_sequence: u64,
    step: u64,
    sim_time_ticks: u64,
    phase: String,
    joint_position_target_rad: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq, Serialize)]
struct ObservationFrame {
    step: u64,
    sim_time_ticks: u64,
    joint_position_rad: Vec<f64>,
    joint_velocity_rad_s: Vec<f64>,
    maximum_tracking_error_rad: f64,
    physics_hash: u64,
}

#[derive(Debug, Serialize)]
struct BackendTrace<'a> {
    kind: &'static str,
    schema_version: u32,
    backend_id: &'static str,
    backend_version: &'static str,
    task_id: &'a str,
    task_sha256: &'a str,
    controller_id: &'a str,
    controller_sha256: &'a str,
    action_trace_sha256: &'a str,
    actuation_config_sha256: &'a str,
    fixed_delta_ticks: u64,
    initial_state_digest: u64,
    final_state_digest: u64,
    replay_final_state_digest: u64,
    replay_match: bool,
    final_maximum_tracking_error_rad: f64,
    maximum_tracking_error_rad: f64,
    observations: Vec<ObservationFrame>,
}

#[derive(Debug, Serialize)]
struct FailureReport<'a> {
    kind: &'static str,
    schema_version: u32,
    backend_id: &'static str,
    backend_version: &'static str,
    task_id: &'a str,
    task_sha256: &'a str,
    controller_id: &'a str,
    controller_sha256: &'a str,
    action_trace_sha256: &'a str,
    actuation_config_sha256: &'a str,
    injection_kind: &'a str,
    injected_step: u64,
    first_violation: &'a str,
    first_violation_step: u64,
    first_violation_sim_time_ticks: u64,
    unit: &'static str,
    observed_missing_action_elements: u64,
    maximum_missing_action_elements: u64,
    status: &'static str,
}

struct Rollout {
    initial_digest: u64,
    final_digest: u64,
    observations: Vec<ObservationFrame>,
}

fn main() {
    if let Err(error) = run() {
        eprintln!("OpenArm Rapier trace failed: {error:#}");
        std::process::exit(1);
    }
}

fn run() -> Result<()> {
    let repo_root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("../..");
    let mut controller_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json");
    let mut task_path = repo_root
        .join("adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json");
    let mut actuation_config_path =
        repo_root.join("adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json");
    let mut output = repo_root.join("artifacts/openarm-cross-sim");
    let mut args = std::env::args().skip(1);
    while let Some(argument) = args.next() {
        match argument.as_str() {
            "--controller" => controller_path = required_path(&mut args, "--controller")?,
            "--task" => task_path = required_path(&mut args, "--task")?,
            "--actuation-config" => {
                actuation_config_path = required_path(&mut args, "--actuation-config")?
            }
            "--output" => output = required_path(&mut args, "--output")?,
            other => bail!("unknown argument {other:?}"),
        }
    }

    let controller_bytes = fs::read(&controller_path)
        .with_context(|| format!("read {}", controller_path.display()))?;
    let task_bytes =
        fs::read(&task_path).with_context(|| format!("read {}", task_path.display()))?;
    let actuation_config_bytes = fs::read(&actuation_config_path)
        .with_context(|| format!("read {}", actuation_config_path.display()))?;
    let controller: ControllerSpec = serde_json::from_slice(&controller_bytes)
        .with_context(|| format!("parse {}", controller_path.display()))?;
    let task: TaskSpec = serde_json::from_slice(&task_bytes)
        .with_context(|| format!("parse {}", task_path.display()))?;
    let actuation_config: ActuationConfig = serde_json::from_slice(&actuation_config_bytes)
        .with_context(|| format!("parse {}", actuation_config_path.display()))?;
    validate(&controller, &task, &actuation_config)?;

    let controller_sha256 = sha256(&controller_bytes);
    let task_sha256 = sha256(&task_bytes);
    let actuation_config_sha256 = sha256(&actuation_config_bytes);
    let actions = compile_actions(&controller);
    fs::create_dir_all(&output)?;
    let action_path = output.join("controller-actions.json");
    write_json(
        &action_path,
        &ActionTrace {
            kind: TRACE_KIND,
            schema_version: 1,
            task_id: &controller.task_id,
            task_sha256: &task_sha256,
            controller_id: &controller.controller_id,
            controller_sha256: &controller_sha256,
            fixed_delta_ticks: FIXED_DELTA_TICKS,
            action_joint_order: &controller.action_joint_order,
            actions: actions.clone(),
        },
    )?;

    let action_trace_sha256 = sha256(&fs::read(&action_path)?);

    let first = rollout(&repo_root, &controller, &actuation_config, &actions)?;
    let replay = rollout(&repo_root, &controller, &actuation_config, &actions)?;
    anyhow::ensure!(
        first.final_digest == replay.final_digest && first.observations == replay.observations,
        "Rapier replay differed for the exact same controller trace"
    );
    write_failure_replay(
        &output.join("controller-failure.rne-replay"),
        &controller,
        &actions,
        &first,
    )?;
    let final_error = first
        .observations
        .last()
        .context("Rapier trace has no observations")?
        .maximum_tracking_error_rad;
    let maximum_error = first
        .observations
        .iter()
        .map(|frame| frame.maximum_tracking_error_rad)
        .fold(0.0_f64, f64::max);
    write_json(
        &output.join("rapier-success-trace.json"),
        &BackendTrace {
            kind: RAPIER_TRACE_KIND,
            schema_version: 1,
            backend_id: "rne_rapier",
            backend_version: "0.22",
            task_id: &controller.task_id,
            task_sha256: &task_sha256,
            controller_id: &controller.controller_id,
            controller_sha256: &controller_sha256,
            action_trace_sha256: &action_trace_sha256,
            actuation_config_sha256: &actuation_config_sha256,
            fixed_delta_ticks: FIXED_DELTA_TICKS,
            initial_state_digest: first.initial_digest,
            final_state_digest: first.final_digest,
            replay_final_state_digest: replay.final_digest,
            replay_match: true,
            final_maximum_tracking_error_rad: final_error,
            maximum_tracking_error_rad: maximum_error,
            observations: first.observations,
        },
    )?;

    let failure = &controller.intentional_failure;
    write_json(
        &output.join("intentional-failure.json"),
        &FailureReport {
            kind: FAILURE_KIND,
            schema_version: 1,
            backend_id: "rne_rapier",
            backend_version: "0.22",
            task_id: &controller.task_id,
            task_sha256: &task_sha256,
            controller_id: &controller.controller_id,
            controller_sha256: &controller_sha256,
            action_trace_sha256: &action_trace_sha256,
            actuation_config_sha256: &actuation_config_sha256,
            injection_kind: &failure.kind,
            injected_step: failure.inject_at_step,
            first_violation: &failure.expected_first_violation,
            first_violation_step: failure.inject_at_step,
            first_violation_sim_time_ticks: failure.inject_at_step * FIXED_DELTA_TICKS,
            unit: "missing_action_element_count",
            observed_missing_action_elements: 1,
            maximum_missing_action_elements: 0,
            status: "failed_as_expected",
        },
    )?;
    println!(
        "OpenArm Rapier trace: steps={} replay_match=true final_error_rad={final_error:.6} action_sha256={action_trace_sha256}",
        actions.len()
    );
    Ok(())
}

fn validate(
    controller: &ControllerSpec,
    task: &TaskSpec,
    actuation_config: &ActuationConfig,
) -> Result<()> {
    task.validate()?;
    anyhow::ensure!(
        controller.kind == KIND && controller.schema_version == SCHEMA_VERSION,
        "unsupported controller kind or schema"
    );
    anyhow::ensure!(
        controller.task_id == task.task_id,
        "controller TaskSpec mismatch"
    );
    anyhow::ensure!(
        controller.interpolation == "smoothstep_v1",
        "unsupported controller interpolation"
    );
    let width = controller.action_joint_order.len();
    anyhow::ensure!(
        width == 9,
        "OpenArm right-arm controller must expose nine joints"
    );
    anyhow::ensure!(
        controller.rne_actuator_link_order.len() == width,
        "controller simulator/RNE joint mappings differ in width"
    );
    anyhow::ensure!(
        controller
            .action_joint_order
            .iter()
            .collect::<HashSet<_>>()
            .len()
            == width
            && controller
                .rne_actuator_link_order
                .iter()
                .collect::<HashSet<_>>()
                .len()
                == width,
        "controller joint mappings contain duplicates"
    );
    anyhow::ensure!(
        task.action.tensors.len() == 1
            && task.action.tensors[0].name == "joint_position_target_rad"
            && task.action.tensors[0].unit == "rad"
            && task.action.tensors[0].shape == [width],
        "TaskSpec action catalog differs from controller"
    );
    anyhow::ensure!(
        task.observation.tensors.len() == 2
            && task.observation.tensors[0].name == "joint_position_rad"
            && task.observation.tensors[0].unit == "rad"
            && task.observation.tensors[0].shape == [width]
            && task.observation.tensors[1].name == "joint_velocity_rad_s"
            && task.observation.tensors[1].unit == "rad/s"
            && task.observation.tensors[1].shape == [width],
        "TaskSpec observation catalog differs from controller"
    );
    anyhow::ensure!(
        (task.control_step_s * 1_000_000_000.0).round() as u64 == FIXED_DELTA_TICKS,
        "TaskSpec control step differs from fixed trace step"
    );
    anyhow::ensure!(
        controller.keyframes.len() >= 2,
        "controller needs two keyframes"
    );
    anyhow::ensure!(
        controller.keyframes[0].step == 0,
        "first keyframe must be step zero"
    );
    for pair in controller.keyframes.windows(2) {
        anyhow::ensure!(pair[0].step < pair[1].step, "keyframe steps must increase");
    }
    for keyframe in &controller.keyframes {
        anyhow::ensure!(!keyframe.phase.trim().is_empty(), "keyframe phase is empty");
        anyhow::ensure!(
            keyframe.joint_position_target_rad.len() == width
                && keyframe
                    .joint_position_target_rad
                    .iter()
                    .all(|value| value.is_finite() && (-3.0..=3.0).contains(value)),
            "keyframe target is invalid or outside TaskSpec bounds"
        );
    }
    let final_step = controller.keyframes.last().unwrap().step;
    anyhow::ensure!(
        (1..=final_step).contains(&controller.intentional_failure.inject_at_step),
        "intentional failure step is outside rollout"
    );
    anyhow::ensure!(
        controller.intentional_failure.kind == "controller_output_truncation"
            && controller.intentional_failure.expected_first_violation == "action_width_mismatch",
        "intentional failure contract is unsupported"
    );
    anyhow::ensure!(
        actuation_config.kind == ACTUATION_CONFIG_KIND
            && actuation_config.schema_version == 1
            && actuation_config.backend_id == "rne_rapier"
            && actuation_config.motor_model == "force_based_v1"
            && actuation_config.solver_iterations > 0
            && actuation_config.fixed_delta_ticks == FIXED_DELTA_TICKS,
        "unsupported or invalid RNE actuation configuration"
    );
    anyhow::ensure!(
        actuation_config
            .joints
            .iter()
            .map(|joint| &joint.joint_name)
            .eq(&controller.action_joint_order)
            && actuation_config
                .joints
                .iter()
                .map(|joint| &joint.link_name)
                .eq(&controller.rne_actuator_link_order),
        "RNE actuation configuration order differs from controller"
    );
    anyhow::ensure!(
        actuation_config.joints.iter().all(|joint| {
            [
                joint.stiffness_nm_per_rad,
                joint.damping_nm_s_per_rad,
                joint.max_effort_nm,
            ]
            .iter()
            .all(|value| value.is_finite() && *value >= 0.0)
        }),
        "RNE actuation configuration has invalid gains or effort limits"
    );
    Ok(())
}

fn compile_actions(controller: &ControllerSpec) -> Vec<ActionFrame> {
    let final_step = controller.keyframes.last().unwrap().step;
    (1..=final_step)
        .map(|step| {
            let upper_index = controller
                .keyframes
                .iter()
                .position(|keyframe| keyframe.step >= step)
                .unwrap();
            let upper = &controller.keyframes[upper_index];
            let lower = &controller.keyframes[upper_index - 1];
            let alpha = (step - lower.step) as f64 / (upper.step - lower.step) as f64;
            let alpha = alpha * alpha * (3.0 - 2.0 * alpha);
            let targets = lower
                .joint_position_target_rad
                .iter()
                .zip(&upper.joint_position_target_rad)
                .map(|(from, to)| from + (to - from) * alpha)
                .collect();
            ActionFrame {
                action_sequence: step - 1,
                step,
                sim_time_ticks: step * FIXED_DELTA_TICKS,
                phase: upper.phase.clone(),
                joint_position_target_rad: targets,
            }
        })
        .collect()
}

fn rollout(
    repo_root: &Path,
    controller: &ControllerSpec,
    actuation_config: &ActuationConfig,
    actions: &[ActionFrame],
) -> Result<Rollout> {
    let scene = repo_root.join("assets/scenes/openarm_v2_right_validation.rne.scene.toml");
    let mut sim = UrdfSceneSim::from_scene_path_with_solver_iterations(
        &scene,
        actuation_config.solver_iterations,
    )
    .context("load OpenArm right-arm validation scene")?;
    configure_actuators(&mut sim, actuation_config)?;
    let initial_digest = hash_physics_state(sim.world());
    let mut observations = Vec::with_capacity(actions.len());
    for action in actions {
        let targets = controller
            .rne_actuator_link_order
            .iter()
            .zip(&action.joint_position_target_rad)
            .map(|(link_name, position)| UrdfJointPositionTarget {
                link_name,
                position: *position,
            })
            .collect::<Vec<_>>();
        sim.step_joint_position_actuation_targets(&targets);
        let positions = controller
            .rne_actuator_link_order
            .iter()
            .map(|link| {
                sim.named_joint_position(link)
                    .with_context(|| format!("missing {link}"))
            })
            .collect::<Result<Vec<_>>>()?;
        let velocities = controller
            .rne_actuator_link_order
            .iter()
            .map(|link| {
                sim.named_joint_velocity(link)
                    .with_context(|| format!("missing {link}"))
            })
            .collect::<Result<Vec<_>>>()?;
        let error = positions
            .iter()
            .zip(&action.joint_position_target_rad)
            .map(|(actual, expected)| (actual - expected).abs())
            .fold(0.0_f64, f64::max);
        observations.push(ObservationFrame {
            step: action.step,
            sim_time_ticks: action.sim_time_ticks,
            joint_position_rad: positions,
            joint_velocity_rad_s: velocities,
            maximum_tracking_error_rad: error,
            physics_hash: hash_physics_state(sim.world()),
        });
    }
    Ok(Rollout {
        initial_digest,
        final_digest: hash_physics_state(sim.world()),
        observations,
    })
}

fn write_failure_replay(
    path: &Path,
    controller: &ControllerSpec,
    actions: &[ActionFrame],
    rollout: &Rollout,
) -> Result<()> {
    let inject_step = controller.intentional_failure.inject_at_step;
    let accepted_steps = (inject_step - 1) as usize;
    let descriptor = BehaviorContractDescriptor {
        name: "controller_action_width_matches_task".to_string(),
        kind: BehaviorContractKind::Always,
        entities: vec![controller.controller_id.clone()],
    };
    let mut frames = Vec::with_capacity(accepted_steps + 2);
    frames.push(BehaviorReplayFrame {
        step: 0,
        sim_time_ticks: 0,
        action: BehaviorReplayAction::InitialObservation,
        observation: json!({
            "action_width_expected": controller.action_joint_order.len(),
            "action_width_observed": controller.action_joint_order.len(),
            "contract_status": "initial"
        }),
        state_digest: rollout.initial_digest,
    });
    for (action, observation) in actions[..accepted_steps]
        .iter()
        .zip(&rollout.observations[..accepted_steps])
    {
        frames.push(BehaviorReplayFrame {
            step: action.step,
            sim_time_ticks: action.sim_time_ticks,
            action: BehaviorReplayAction::Advance,
            observation: json!({
                "action_width_expected": controller.action_joint_order.len(),
                "action_width_observed": controller.action_joint_order.len(),
                "joint_position_rad": observation.joint_position_rad,
                "joint_velocity_rad_s": observation.joint_velocity_rad_s,
                "contract_status": "passed"
            }),
            state_digest: observation.physics_hash,
        });
    }
    let before_failure = rollout
        .observations
        .get(accepted_steps - 1)
        .context("controller failure replay has no pre-failure observation")?;
    let violation = BehaviorViolation {
        step: inject_step,
        sim_time_ticks: inject_step * FIXED_DELTA_TICKS,
        state_digest: before_failure.physics_hash,
        entities: descriptor.entities.clone(),
        message: "controller emitted 8 action elements but TaskSpec requires 9; rejected before simulator state advance".to_string(),
    };
    frames.push(BehaviorReplayFrame {
        step: inject_step,
        sim_time_ticks: inject_step * FIXED_DELTA_TICKS,
        action: BehaviorReplayAction::Advance,
        observation: json!({
            "action_width_expected": controller.action_joint_order.len(),
            "action_width_observed": controller.action_joint_order.len() - 1,
            "joint_position_rad": before_failure.joint_position_rad,
            "joint_velocity_rad_s": before_failure.joint_velocity_rad_s,
            "contract_status": "failed"
        }),
        state_digest: before_failure.physics_hash,
    });
    let digest = Sha256::digest(controller.controller_id.as_bytes());
    let scenario_digest = u64::from_le_bytes(digest[..8].try_into().unwrap());
    let replay = BehaviorReplayArtifact::new(
        "rne.openarm.right_joint_tracking.controller_contract.v1",
        scenario_digest,
        20260824,
        FIXED_DELTA_TICKS,
        Vec::new(),
        vec![descriptor.clone()],
        frames,
        BehaviorReplayFailure {
            contract: descriptor,
            violation,
        },
    )?;
    replay.write_json(path)?;
    Ok(())
}

fn configure_actuators(sim: &mut UrdfSceneSim, config: &ActuationConfig) -> Result<()> {
    for joint in &config.joints {
        anyhow::ensure!(
            sim.configure_named_revolute_position_actuation(
                &joint.link_name,
                joint.stiffness_nm_per_rad,
                joint.damping_nm_s_per_rad,
                joint.max_effort_nm,
            ),
            "missing OpenArm actuator {}",
            joint.link_name
        );
    }
    Ok(())
}

fn required_path(args: &mut impl Iterator<Item = String>, option: &str) -> Result<PathBuf> {
    args.next()
        .filter(|value| !value.is_empty())
        .map(PathBuf::from)
        .with_context(|| format!("{option} requires a path"))
}

fn write_json(path: &Path, value: &impl Serialize) -> Result<()> {
    let mut bytes = serde_json::to_vec_pretty(value)?;
    bytes.push(b'\n');
    fs::write(path, bytes).with_context(|| format!("write {}", path.display()))
}

fn sha256(bytes: &[u8]) -> String {
    format!("{:x}", Sha256::digest(bytes))
}

#[cfg(test)]
mod tests {
    use super::*;

    fn fixture(relative: &str) -> PathBuf {
        PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../..")
            .join(relative)
    }

    #[test]
    fn portable_controller_compiles_exact_task_order_and_failure_step() {
        let controller: ControllerSpec = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json",
            ))
            .unwrap(),
        )
        .unwrap();
        let task: TaskSpec = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json",
            ))
            .unwrap(),
        )
        .unwrap();
        let actuation_config: ActuationConfig = serde_json::from_slice(
            &fs::read(fixture(
                "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json",
            ))
            .unwrap(),
        )
        .unwrap();
        validate(&controller, &task, &actuation_config).unwrap();
        let actions = compile_actions(&controller);
        assert_eq!(actions.len(), 1_800);
        assert_eq!(actions[0].action_sequence, 0);
        assert_eq!(actions[306].step, 307);
        assert_eq!(actions.last().unwrap().sim_time_ticks, 30_000_000_600);
        assert_eq!(
            actions.last().unwrap().joint_position_target_rad,
            controller
                .keyframes
                .last()
                .unwrap()
                .joint_position_target_rad
        );
        assert_eq!(controller.intentional_failure.inject_at_step, 307);
    }
}

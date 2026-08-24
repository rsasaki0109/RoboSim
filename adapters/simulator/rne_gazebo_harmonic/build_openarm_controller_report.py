#!/usr/bin/env python3
"""Build the reproducible OpenArm PID/state-space controller comparison report."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any


ROLES = ("pid", "state_feedback")
BACKENDS = ("rne_rapier", "mujoco_native", "gazebo_sim")
TRACE_FILES = {
    "rne_rapier": "rapier-success-trace.json",
    "mujoco_native": "mujoco-success-trace.json",
    "gazebo_sim": "gazebo-success-trace.json",
}
FAILURE_FILES = {
    "rne_rapier": "intentional-failure.json",
    "mujoco_native": "mujoco-intentional-failure.json",
    "gazebo_sim": "gazebo-intentional-failure.json",
}
JOINT = "openarm_right_joint5"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--experiment-manifest",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_experiments.json",
    )
    parser.add_argument(
        "--requirements",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_controller_requirements.json",
    )
    return parser.parse_args()


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_module(name: str, path: Path) -> Any:
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def requirement_map(registry: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if (
        set(registry) != {"kind", "schema_version", "registry_id", "requirements"}
        or registry.get("kind") != "rne_openarm_controller_requirements_registry"
        or registry.get("schema_version") != 1
        or not isinstance(registry.get("requirements"), list)
    ):
        raise ValueError("unsupported controller requirements registry")
    result = {item["id"]: item for item in registry["requirements"]}
    if len(result) != len(registry["requirements"]):
        raise ValueError("controller requirements contain duplicate ids")
    for item in result.values():
        bound = "maximum" if "maximum" in item else "minimum"
        if set(item) != {"id", "gate", "unit", bound}:
            raise ValueError(f"invalid controller requirement {item.get('id')}")
    return result


def check(
    requirement: dict[str, Any], observed: float | None, suffix: str = "", **extra: Any
) -> dict[str, Any]:
    passed = observed is not None and math.isfinite(observed)
    if "maximum" in requirement:
        passed = passed and observed <= requirement["maximum"]
        bound = {"maximum": requirement["maximum"]}
    else:
        passed = passed and observed >= requirement["minimum"]
        bound = {"minimum": requirement["minimum"]}
    return {
        "id": requirement["id"] + suffix,
        "gate": requirement["gate"],
        "unit": requirement["unit"],
        "observed": observed,
        **bound,
        "status": "passed" if passed else "failed",
        **extra,
    }


def maximum_delta(left: list[float], right: list[float]) -> float:
    if len(left) != len(right):
        raise ValueError("controller reproduction vector width drifted")
    return max((abs(a - b) for a, b in zip(left, right)), default=0.0)


def reproduce_decisions(
    runner: Any,
    controller: dict[str, Any],
    actions: list[dict[str, Any]],
    observations: list[dict[str, Any]],
) -> dict[str, Any]:
    width = len(controller["action_joint_order"])
    integral = [0.0] * width
    previous_position: list[float | None] = [None] * width
    previous_input: list[float | None] = [None] * width
    previous_previous_input: list[float | None] = [None] * width
    controller_target_history: list[list[float]] = []
    applied_target_history: list[list[float]] = []
    maximum = 0.0
    first_mismatch = None
    for index, (action, actual) in enumerate(zip(actions, observations)):
        delayed = observations[index - 2] if index >= 2 else None
        decision = runner.controller_decision(
            controller,
            action["joint_position_target_rad"],
            integral,
            previous_position,
            previous_input,
            previous_previous_input,
            delayed,
            (action["step"] - 1) * controller["observation_contract"]["sample_period_ticks"],
        )
        controller_target_history.append(decision["target"].copy())
        applied_target, disturbance = runner.apply_actuator_disturbance(
            controller,
            action["step"],
            decision["target"],
            controller_target_history,
            applied_target_history,
        )
        applied_target_history.append(applied_target.copy())
        delta = max(
            maximum_delta(decision["target"], actual["joint_controller_target_rad"]),
            maximum_delta(applied_target, actual["joint_position_target_rad"]),
            maximum_delta(
                disturbance, actual["joint_actuator_disturbance_rad"]
            ),
            maximum_delta(
                decision["controller_observation_position_rad"],
                actual["joint_controller_observation_position_rad"],
            ),
            maximum_delta(
                decision["joint_measurement_bias_rad"],
                actual["joint_measurement_bias_rad"],
            ),
            maximum_delta(decision["correction"], actual["joint_feedback_correction_rad"]),
            maximum_delta(
                decision["integral_correction"], actual["joint_integral_correction_rad"]
            ),
        )
        maximum = max(maximum, delta)
        metadata_matches = (
            decision["observation_sequence"]
            == actual["controller_observation_sequence"]
            and decision["observation_age_ticks"]
            == actual["controller_observation_age_ticks"]
            and decision["bootstrap"] == actual["controller_bootstrap"]
            and actual["measurement_bias_active"]
            == any(value != 0.0 for value in decision["joint_measurement_bias_rad"])
        )
        if first_mismatch is None and (delta > 1e-12 or not metadata_matches):
            first_mismatch = {
                "step": action["step"],
                "maximum_numeric_delta_rad": delta,
                "metadata_matches": metadata_matches,
            }
    return {
        "maximum_numeric_delta_rad": maximum,
        "first_mismatch": first_mismatch,
        "decision_count": len(actions),
    }


def rms(values: list[float]) -> float:
    return math.sqrt(sum(value * value for value in values) / len(values))


def disturbance_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
    sample_rate_hz: float,
) -> dict[str, Any]:
    contract = controller["disturbance_contract"]
    start_step = contract["start_step"]
    end_step = contract["end_step"]
    recovery_band_rad = 0.005
    recovery_hold_samples = 30
    evaluation_end_step = min(len(observations), end_step + 120)
    first_realization_mismatch = None
    maximum_realization_delta_rad = 0.0
    realized_active_step_count = 0
    maximum_recomputed_applied_rate_rad_s = 0.0
    previous_expected_applied: list[float] | None = None
    for frame in observations:
        step = frame["step"]
        controller_target = frame["joint_controller_target_rad"]
        expected_applied = controller_target.copy()
        expected_source_step = step
        if start_step <= step <= end_step:
            if contract["kind"] == "additive_actuator_target_bias_pulse_v1":
                expected_applied[joint_index] += contract["offset_rad"]
            elif contract["kind"] == "actuator_command_transport_delay_pulse_v1":
                expected_source_step = step - contract["delay_steps"]
                expected_applied[joint_index] = observations[expected_source_step - 1][
                    "joint_controller_target_rad"
                ][joint_index]
            elif contract["kind"] == "actuator_command_slew_rate_limit_pulse_v1":
                if previous_expected_applied is None:
                    raise ValueError("rate limit has no previous expected applied target")
                maximum_delta_rad = contract["maximum_rate_rad_s"] / sample_rate_hz
                previous = previous_expected_applied[joint_index]
                expected_applied[joint_index] = min(
                    max(controller_target[joint_index], previous - maximum_delta_rad),
                    previous + maximum_delta_rad,
                )
                maximum_recomputed_applied_rate_rad_s = max(
                    maximum_recomputed_applied_rate_rad_s,
                    abs(expected_applied[joint_index] - previous) * sample_rate_hz,
                )
            else:
                raise ValueError("unsupported actuator disturbance contract")
        expected_disturbance = [
            applied - commanded
            for applied, commanded in zip(expected_applied, controller_target)
        ]
        actual_applied = frame["joint_position_target_rad"]
        actual_disturbance = frame["joint_actuator_disturbance_rad"]
        realization_delta = max(
            maximum_delta(expected_applied, actual_applied),
            maximum_delta(expected_disturbance, actual_disturbance),
        )
        expected_active = any(value != 0.0 for value in expected_disturbance)
        active = frame["actuator_disturbance_active"]
        maximum_realization_delta_rad = max(
            maximum_realization_delta_rad, realization_delta
        )
        if expected_active:
            realized_active_step_count += 1
        if first_realization_mismatch is None and (
            realization_delta > 1e-14 or active != expected_active
        ):
            first_realization_mismatch = {
                "step": step,
                "expected_source_step": expected_source_step,
                "expected_applied_target_rad": expected_applied[joint_index],
                "observed_applied_target_rad": actual_applied[joint_index],
                "realization_delta_rad": realization_delta,
                "expected_active": expected_active,
                "observed_active": active,
            }
        previous_expected_applied = expected_applied
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    pulse_errors = errors[start_step - 1 : end_step]
    evaluation_errors = errors[start_step - 1 : evaluation_end_step]
    recovery_step = next(
        (
            step
            for step in range(end_step + 1, evaluation_end_step - recovery_hold_samples + 2)
            if all(
                abs(errors[index]) <= recovery_band_rad
                for index in range(step - 1, step - 1 + recovery_hold_samples)
            )
        ),
        None,
    )
    recovery_time_s = (
        None if recovery_step is None else (recovery_step - end_step) / sample_rate_hz
    )
    recovery_check_value_s = (
        recovery_time_s
        if recovery_time_s is not None
        else (evaluation_end_step - end_step + 1) / sample_rate_hz
    )
    return {
        "contract": contract,
        "first_realization_mismatch": first_realization_mismatch,
        "realization_verification": {
            "relationship": (
                "applied_target_at_step_equals_controller_target_at_step_minus_delay_steps"
                if contract["kind"] == "actuator_command_transport_delay_pulse_v1"
                else (
                    "applied_target_delta_is_clamped_to_declared_rate_times_fixed_delta"
                    if contract["kind"] == "actuator_command_slew_rate_limit_pulse_v1"
                    else "applied_target_equals_controller_target_plus_declared_bias"
                )
            ),
            "maximum_delta_rad": maximum_realization_delta_rad,
            "realized_active_step_count": realized_active_step_count,
            "source_step_recomputed_from_trace": (
                contract["kind"] == "actuator_command_transport_delay_pulse_v1"
            ),
            "maximum_recomputed_applied_rate_rad_s": (
                maximum_recomputed_applied_rate_rad_s
                if contract["kind"] == "actuator_command_slew_rate_limit_pulse_v1"
                else None
            ),
            "previous_applied_target_recomputed_from_trace": (
                contract["kind"] == "actuator_command_slew_rate_limit_pulse_v1"
            ),
        },
        "peak_tracking_error_rad": max(abs(value) for value in pulse_errors),
        "iae_rad_s": sum(abs(value) for value in evaluation_errors) / sample_rate_hz,
        "recovery_band_rad": recovery_band_rad,
        "recovery_hold_samples": recovery_hold_samples,
        "evaluation_end_step": evaluation_end_step,
        "recovered": recovery_step is not None,
        "recovery_step": recovery_step,
        "recovery_time_s": recovery_time_s,
        "recovery_check_value_s": recovery_check_value_s,
    }


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    script_dir = Path(__file__).resolve().parent
    plant = load_module("rne_controller_report_plant", script_dir / "build_openarm_plant_report.py")
    runner = load_module("rne_controller_report_runner", script_dir / "run_openarm_trace.py")
    root = args.suite_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    suite_path = root / "openarm-controller-suite.json"
    suite = load(suite_path)
    manifest = load(args.experiment_manifest.resolve())
    registry = load(args.requirements.resolve())
    requirements = requirement_map(registry)
    if (
        suite.get("kind") != "rne_openarm_controller_suite"
        or suite.get("schema_version") != 1
        or suite.get("experiment_id") != manifest["experiment_id"]
        or suite.get("source_experiment_manifest_sha256")
        != sha256(args.experiment_manifest.resolve())
        or [item["role"] for item in suite.get("controllers", [])] != list(ROLES)
    ):
        raise ValueError("controller suite identity drifted")
    order = manifest["action_joint_order"]
    joint_index = order.index(JOINT)
    step_segment = plant.segment(manifest, "joint5_step_doublet")
    ramp_segment = plant.segment(manifest, "joint5_triangular_ramp")
    all_checks = []
    reports = []
    metrics: dict[str, dict[str, dict[str, Any]]] = {role: {} for role in ROLES}
    plot_data: dict[str, Any] = {"step": list(range(1, 3601)), "controllers": {}}
    controller_artifacts = {}
    for declaration in suite["controllers"]:
        role = declaration["role"]
        controller_path = root / declaration["path"]
        controller = load(controller_path)
        if (
            sha256(controller_path) != declaration["sha256"]
            or controller.get("controller_id") != declaration["controller_id"]
            or controller.get("action_joint_order") != order
        ):
            raise ValueError(f"{role} controller identity drifted")
        if controller.get("disturbance_contract") != suite.get(
            "shared_disturbance_contract"
        ):
            raise ValueError(f"{role} disturbance contract drifted")
        controller_artifacts[role] = controller
        role_root = root / role
        action_path = role_root / "controller-actions.json"
        action_artifact = load(action_path)
        actions = action_artifact["actions"]
        if (
            action_artifact.get("controller_sha256") != sha256(controller_path)
            or action_artifact.get("controller_id") != controller["controller_id"]
            or len(actions) != 3600
        ):
            raise ValueError(f"{role} action trace drifted")
        if len(controller.get("keyframes", [])) != len(actions) + 1:
            raise ValueError(f"{role} controller keyframe count drifted")
        for index, (action, keyframe) in enumerate(
            zip(actions, controller["keyframes"][1:]), 1
        ):
            if (
                action.get("action_sequence") != index - 1
                or action.get("step") != index
                or action.get("sim_time_ticks") != index * suite["fixed_delta_ticks"]
                or action.get("phase") != keyframe["phase"]
                or maximum_delta(
                    action.get("joint_position_target_rad", []),
                    keyframe["joint_position_target_rad"],
                )
                > 1e-14
            ):
                raise ValueError(f"{role} action differs at step {index}")
        plot_data["controllers"][role] = {"reference_rad": [
            action["joint_position_target_rad"][joint_index] for action in actions
        ], "backends": {}}
        for backend in BACKENDS:
            trace_path = role_root / TRACE_FILES[backend]
            failure_path = role_root / FAILURE_FILES[backend]
            trace = load(trace_path)
            failure = load(failure_path)
            observations = trace["observations"]
            expected_execution = (
                "artifact_defined_joint_feedback_pid"
                if role == "pid"
                else "artifact_defined_joint_feedback_state_space"
            )
            if (
                trace.get("backend_id") != backend
                or trace.get("controller_id") != controller["controller_id"]
                or trace.get("controller_sha256") != sha256(controller_path)
                or trace.get("action_trace_sha256") != sha256(action_path)
                or trace.get("controller_execution") != expected_execution
                or not trace.get("replay_match")
                or len(observations) != len(actions)
                or failure.get("controller_id") != controller["controller_id"]
                or failure.get("controller_sha256") != sha256(controller_path)
                or failure.get("action_trace_sha256") != sha256(action_path)
                or failure.get("first_violation") != "action_width_mismatch"
                or failure.get("first_violation_step") != 307
                or failure.get("status") != "failed_as_expected"
            ):
                raise ValueError(f"{role}/{backend} trace or failure identity drifted")
            reproduction = reproduce_decisions(runner, controller, actions, observations)
            disturbance = disturbance_metrics(
                controller, observations, joint_index, manifest["sample_rate_hz"]
            )
            step = plant.step_metrics(
                actions,
                observations,
                joint_index,
                step_segment,
                manifest["sample_rate_hz"],
                manifest["analysis"],
            )
            ramp_start = ramp_segment["start_step"] - 1
            ramp_end = ramp_segment["end_step"]
            ramp_rmse = rms([
                observations[index]["joint_position_rad"][joint_index]
                - actions[index]["joint_position_target_rad"][joint_index]
                for index in range(ramp_start, ramp_end)
            ])
            maximum_correction = max(
                abs(frame["joint_feedback_correction_rad"][joint_index])
                for frame in observations
            )
            maximum_integral = max(
                abs(frame["joint_integral_correction_rad"][joint_index])
                for frame in observations
            )
            saturation_values = [
                frame.get("effort_saturated", frame.get("actuator_command_saturated"))
                for frame in observations
            ]
            saturation_fraction = sum(
                sum(bool(value) for value in values) for values in saturation_values
            ) / (len(observations) * len(order))
            item = {
                "role": role,
                "backend_id": backend,
                "trace_sha256": sha256(trace_path),
                "replay_match": True,
                "step_response": step,
                "ramp_tracking_rmse_rad": ramp_rmse,
                "maximum_feedback_correction_rad": maximum_correction,
                "maximum_integral_correction_rad": maximum_integral,
                "actuator_saturated_sample_fraction": saturation_fraction,
                "controller_reproduction": reproduction,
                "disturbance_rejection": disturbance,
                "intentional_failure": {
                    "step": failure["first_violation_step"],
                    "kind": failure["first_violation"],
                },
            }
            reports.append(item)
            metrics[role][backend] = item
            plot_data["controllers"][role]["backends"][backend] = [
                frame["joint_position_rad"][joint_index] for frame in observations
            ]
            all_checks.extend(
                [
                    check(
                        requirements["controller.maximum_decision_reproduction_delta_rad"],
                        reproduction["maximum_numeric_delta_rad"],
                        f".{role}.{backend}",
                    ),
                    check(
                        requirements["controller.maximum_correction_rad"],
                        maximum_correction,
                        f".{role}.{backend}",
                    ),
                    check(
                        requirements["controller.maximum_integral_correction_rad"],
                        maximum_integral,
                        f".{role}.{backend}",
                    ),
                ]
            )
            if disturbance["first_realization_mismatch"] is not None:
                raise ValueError(f"{role}/{backend} disturbance realization drifted")
            if role == "state_feedback":
                all_checks.extend(
                    [
                        check(
                            requirements["controller.state.maximum_settling_time_s"],
                            step["settling_time_s"],
                            f".{backend}",
                        ),
                        check(
                            requirements["controller.state.maximum_overshoot_fraction"],
                            step["overshoot_fraction"],
                            f".{backend}",
                        ),
                        check(
                            requirements["controller.state.maximum_ramp_tracking_rmse_rad"],
                            ramp_rmse,
                            f".{backend}",
                        ),
                        check(
                            requirements[
                                "controller.state.maximum_disturbance_peak_error_rad"
                            ],
                            disturbance["peak_tracking_error_rad"],
                            f".{backend}",
                        ),
                        check(
                            requirements[
                                "controller.state.maximum_disturbance_recovery_time_s"
                            ],
                            disturbance["recovery_check_value_s"],
                            f".{backend}",
                            recovered=disturbance["recovered"],
                            recovery_step=disturbance["recovery_step"],
                        ),
                        check(
                            requirements[
                                "controller.state.maximum_disturbance_iae_rad_s"
                            ],
                            disturbance["iae_rad_s"],
                            f".{backend}",
                        ),
                    ]
                )
    state_law = controller_artifacts["state_feedback"]["feedback_law"]
    all_checks.extend(
        [
            check(
                requirements["controller.model.minimum_absolute_controllability_determinant"],
                abs(state_law["identified_plant"]["controllability_determinant"]),
            ),
            check(
                requirements["controller.model.minimum_absolute_observability_determinant"],
                abs(state_law["identified_plant"]["observability_determinant"]),
            ),
            check(
                requirements["controller.model.maximum_declared_pole_magnitude"],
                max(abs(value) for value in state_law["desired_closed_loop_poles"]),
            ),
        ]
    )
    state_settling = [
        metrics["state_feedback"][backend]["step_response"]["settling_time_s"]
        for backend in BACKENDS
    ]
    all_checks.append(
        check(
            requirements["controller.state.maximum_cross_backend_settling_delta_s"],
            max(state_settling) - min(state_settling),
        )
    )
    state_recovery = [
        metrics["state_feedback"][backend]["disturbance_rejection"][
            "recovery_check_value_s"
        ]
        for backend in BACKENDS
    ]
    all_checks.append(
        check(
            requirements[
                "controller.state.maximum_cross_backend_disturbance_recovery_delta_s"
            ],
            max(state_recovery) - min(state_recovery),
        )
    )
    pid_rapier_settling = metrics["pid"]["rne_rapier"]["step_response"]["settling_time_s"]
    state_rapier_settling = metrics["state_feedback"]["rne_rapier"]["step_response"]["settling_time_s"]
    all_checks.append(
        check(
            requirements["controller.state.minimum_rapier_settling_improvement_s"],
            pid_rapier_settling - state_rapier_settling,
        )
    )
    baseline_deadline_index = math.ceil(
        requirements["controller.pid.maximum_settling_time_s"]["maximum"]
        * manifest["sample_rate_hz"]
    )
    baseline_deadline_step = step_segment["positive_step"] + baseline_deadline_index
    baseline_band_rad = metrics["pid"]["rne_rapier"]["step_response"][
        "settling_band_rad"
    ]
    baseline_reference = plot_data["controllers"]["pid"]["reference_rad"]
    baseline_output = plot_data["controllers"]["pid"]["backends"]["rne_rapier"]
    baseline_first_violation_step = next(
        step
        for step in range(baseline_deadline_step, step_segment["negative_step"])
        if abs(baseline_output[step - 1] - baseline_reference[step - 1])
        > baseline_band_rad
    )
    baseline_failure = check(
        requirements["controller.pid.maximum_settling_time_s"],
        pid_rapier_settling,
        ".pid.rne_rapier.baseline_diagnostic",
        classification="non_gating_baseline",
        first_violation_step=baseline_first_violation_step,
        settling_deadline_step=baseline_deadline_step,
        settling_band_rad=baseline_band_rad,
        target_rad=baseline_reference[baseline_first_violation_step - 1],
        position_at_deadline_rad=baseline_output[baseline_deadline_step - 1],
        position_at_violation_rad=baseline_output[baseline_first_violation_step - 1],
    )
    status = "passed" if all(item["status"] == "passed" for item in all_checks) else "failed"
    report = {
        "kind": "rne_openarm_controller_comparison_report",
        "schema_version": 1,
        "status": status,
        "suite_id": suite["suite_id"],
        "task_id": suite["task_id"],
        "experiment_id": suite["experiment_id"],
        "inputs": {
            "suite_sha256": sha256(suite_path),
            "requirements_sha256": sha256(args.requirements.resolve()),
            "experiment_manifest_sha256": sha256(args.experiment_manifest.resolve()),
        },
        "model_design": state_law,
        "disturbance_contract": suite["shared_disturbance_contract"],
        "backend_results": reports,
        "baseline_first_failure": baseline_failure,
        "first_failed_requirement": next(
            (item for item in all_checks if item["status"] == "failed"), None
        ),
        "checks": all_checks,
        "plot_data": plot_data,
    }
    write_json(output / "openarm-controller-comparison-report.json", report)
    write_html(output / "openarm-controller-comparison-report.html", report)
    print(
        f"OpenArm controller comparison: status={status} "
        f"pid_rapier_settle_s={pid_rapier_settling:.6f} "
        f"state_rapier_settle_s={state_rapier_settling:.6f}"
    )
    return 0


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    document = r"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenArm controller comparison</title><style>
body{margin:0;background:#09121f;color:#eef5ff;font:14px system-ui,sans-serif}main{max-width:1240px;margin:auto;padding:28px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}.card{background:#132238;border:1px solid #2a4667;border-radius:10px;padding:14px}table{width:100%;border-collapse:collapse;margin:14px 0}th,td{border:1px solid #2a4667;padding:6px;text-align:right}th:first-child,td:first-child{text-align:left}canvas{width:100%;height:280px;background:#fff;border-radius:8px}.passed{color:#6ee7aa}.failed{color:#ff8b78}code{color:#b9ddff}</style></head><body><main><h1>OpenArm PID / state-space comparison</h1><div id="verdict"></div><div id="plots"></div><h2>Results</h2><div id="results" class="grid"></div><h2>Fixed checks</h2><div id="checks"></div><script>
const r=__REPORT__,f=x=>x==null?'n/a':Number(x).toFixed(6),colors={rne_rapier:'#1261a0',mujoco_native:'#c2410c',gazebo_sim:'#15803d'};document.querySelector('#verdict').innerHTML=`<section class=card><p>Status: <b class=${r.status}>${r.status}</b></p><p>PID baseline first failure: <code>${r.baseline_first_failure.id}</code> (${f(r.baseline_first_failure.observed)} s)</p><p>First gating failure: <code>${r.first_failed_requirement?r.first_failed_requirement.id:'none'}</code></p></section>`;
function plot(role){const d=r.plot_data.controllers[role],c=document.createElement('canvas');c.width=1160;c.height=280;const x=c.getContext('2d'),n=d.reference_rad.length,dc=r.disturbance_contract;x.fillStyle='#ef444422';x.fillRect((dc.start_step-1)/(n-1)*c.width,0,(dc.end_step-dc.start_step+1)/(n-1)*c.width,c.height);function line(v,color,w=1.3){x.beginPath();for(let i=0;i<n;i++){const px=i/(n-1)*c.width,py=c.height-(v[i]+.25)/.65*c.height;i?x.lineTo(px,py):x.moveTo(px,py)}x.strokeStyle=color;x.lineWidth=w;x.stroke()}line(d.reference_rad,'#111',1.7);Object.entries(d.backends).forEach(([k,v])=>line(v,colors[k]));const section=document.createElement('section');section.innerHTML=`<h2>${role}</h2><p>Red band: unobserved +${dc.offset_rad} rad actuator-target bias pulse.</p>`;section.appendChild(c);return section}const plots=document.querySelector('#plots');plots.appendChild(plot('pid'));plots.appendChild(plot('state_feedback'));
document.querySelector('#results').innerHTML=r.backend_results.map(b=>`<section class=card><h3>${b.role} / ${b.backend_id}</h3><p>settle: ${f(b.step_response.settling_time_s)} s</p><p>rise: ${f(b.step_response.rise_time_s)} s</p><p>overshoot: ${f(b.step_response.overshoot_fraction)}</p><p>ramp RMSE: ${f(b.ramp_tracking_rmse_rad)} rad</p><p>disturbance peak: ${f(b.disturbance_rejection.peak_tracking_error_rad)} rad</p><p>recovery: ${b.disturbance_rejection.recovery_time_s==null?'not recovered':f(b.disturbance_rejection.recovery_time_s)+' s'}</p><p>disturbance IAE: ${f(b.disturbance_rejection.iae_rad_s)} rad·s</p><p>decision delta: ${f(b.controller_reproduction.maximum_numeric_delta_rad)} rad</p></section>`).join('');document.querySelector('#checks').innerHTML=`<table><tr><th>requirement</th><th>gate</th><th>observed</th><th>limit</th><th>status</th></tr>${r.checks.map(q=>`<tr><td>${q.id}</td><td>${q.gate}</td><td>${f(q.observed)} ${q.unit}</td><td>${q.maximum!=null?'≤ '+f(q.maximum):'≥ '+f(q.minimum)} ${q.unit}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`;
</script></main></body></html>""".replace("__REPORT__", payload)
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm controller report failed: {error}")
        raise SystemExit(2)

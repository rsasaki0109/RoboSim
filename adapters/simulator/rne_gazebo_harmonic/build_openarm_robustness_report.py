#!/usr/bin/env python3
"""Build the browser-readable OpenArm robustness boundary report."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any


JOINT = "openarm_right_joint5"
TRACE_FILES = {
    "rne_rapier": "rapier-success-trace.json",
    "mujoco_native": "mujoco-success-trace.json",
    "gazebo_sim": "gazebo-success-trace.json",
}


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--suite-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
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


def controller_dimension_value(controller: dict[str, Any], dimension_id: str) -> Any:
    if dimension_id == "actuator_target_bias":
        return controller.get("disturbance_contract", {}).get("offset_rad")
    if dimension_id == "actuator_command_delay":
        return controller.get("disturbance_contract", {}).get("delay_steps")
    if dimension_id == "actuator_command_rate_limit":
        return controller.get("disturbance_contract", {}).get("maximum_rate_rad_s")
    if dimension_id == "joint_position_measurement_bias":
        return controller.get("measurement_fault_contract", {}).get("offset_rad")
    if dimension_id == "joint_feedback_publication_dropout":
        return controller.get("measurement_fault_contract", {}).get(
            "consecutive_dropped_frames"
        )
    raise ValueError(f"unsupported robustness dimension {dimension_id}")


def load_controller_report_module(script_dir: Path) -> Any:
    path = script_dir / "build_openarm_controller_report.py"
    spec = importlib.util.spec_from_file_location("rne_openarm_controller_report", path)
    if spec is None or spec.loader is None:
        raise ValueError("cannot load OpenArm controller report functions")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def first_requirement_violation(
    observations: list[dict[str, Any]],
    metrics: dict[str, Any],
    joint_index: int,
    sample_rate_hz: float,
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    contract = metrics["contract"]
    start_step = contract["start_step"]
    end_step = contract["end_step"]
    evaluation_end_step = metrics["evaluation_end_step"]
    candidates = []
    peak_requirement = requirements[
        "controller.state.maximum_disturbance_peak_error_rad"
    ]
    for frame in observations[start_step - 1 : end_step]:
        error = abs(
            frame["joint_position_rad"][joint_index]
            - frame["joint_reference_position_rad"][joint_index]
        )
        if error > peak_requirement["maximum"]:
            candidates.append(
                {
                    "requirement_id": peak_requirement["id"],
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "observed": error,
                    "maximum": peak_requirement["maximum"],
                    "unit": peak_requirement["unit"],
                }
            )
            break
    iae_requirement = requirements["controller.state.maximum_disturbance_iae_rad_s"]
    cumulative_iae = 0.0
    for frame in observations[start_step - 1 : evaluation_end_step]:
        cumulative_iae += abs(
            frame["joint_position_rad"][joint_index]
            - frame["joint_reference_position_rad"][joint_index]
        ) / sample_rate_hz
        if cumulative_iae > iae_requirement["maximum"]:
            candidates.append(
                {
                    "requirement_id": iae_requirement["id"],
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "observed": cumulative_iae,
                    "maximum": iae_requirement["maximum"],
                    "unit": iae_requirement["unit"],
                }
            )
            break
    recovery_requirement = requirements[
        "controller.state.maximum_disturbance_recovery_time_s"
    ]
    if metrics["recovery_check_value_s"] > recovery_requirement["maximum"]:
        deadline_step = end_step + math.ceil(
            recovery_requirement["maximum"] * sample_rate_hz
        )
        frame = observations[min(deadline_step, len(observations)) - 1]
        candidates.append(
            {
                "requirement_id": recovery_requirement["id"],
                "step": frame["step"],
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": metrics["recovery_check_value_s"],
                "maximum": recovery_requirement["maximum"],
                "unit": recovery_requirement["unit"],
            }
        )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def command_delay_violation(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    requirement: dict[str, Any],
) -> dict[str, Any] | None:
    contract = controller.get("disturbance_contract", {})
    if contract.get("kind") != "actuator_command_transport_delay_pulse_v1":
        return None
    delay_steps = contract["delay_steps"]
    if delay_steps <= requirement["maximum"]:
        return None
    step = contract["start_step"]
    frame = observations[step - 1]
    return {
        "requirement_id": requirement["id"],
        "step": step,
        "sim_time_ticks": frame["sim_time_ticks"],
        "observed": delay_steps,
        "maximum": requirement["maximum"],
        "unit": requirement["unit"],
        "source_step": step - delay_steps,
    }


def command_rate_limit_violation(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    requirement: dict[str, Any],
) -> dict[str, Any] | None:
    contract = controller.get("disturbance_contract", {})
    if contract.get("kind") != "actuator_command_slew_rate_limit_pulse_v1":
        return None
    maximum_rate_rad_s = contract["maximum_rate_rad_s"]
    if maximum_rate_rad_s >= requirement["minimum"]:
        return None
    step = contract["start_step"]
    frame = observations[step - 1]
    return {
        "requirement_id": requirement["id"],
        "step": step,
        "sim_time_ticks": frame["sim_time_ticks"],
        "observed": maximum_rate_rad_s,
        "minimum": requirement["minimum"],
        "unit": requirement["unit"],
    }


def measurement_bias_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") != "additive_joint_position_bias_pulse_v1":
        return None
    maximum_realization_delta_rad = 0.0
    first_realization_mismatch = None
    active_decision_count = 0
    for frame in observations:
        step = frame["step"]
        expected_bias = (
            contract["offset_rad"]
            if contract["start_controller_step"] <= step <= contract["end_controller_step"]
            else 0.0
        )
        actual_bias = frame["joint_measurement_bias_rad"][joint_index]
        active = frame["measurement_bias_active"]
        observation_sequence = frame["controller_observation_sequence"]
        controller_positions = frame["joint_controller_observation_position_rad"]
        realization_delta = abs(actual_bias - expected_bias)
        if observation_sequence is None:
            relationship_delta = 0.0 if not controller_positions else math.inf
        else:
            source = observations[observation_sequence - 1]
            relationship_delta = max(
                abs(
                    visible
                    - raw
                    - (expected_bias if index == joint_index else 0.0)
                )
                for index, (visible, raw) in enumerate(
                    zip(controller_positions, source["joint_position_rad"])
                )
            )
        delta = max(realization_delta, relationship_delta)
        maximum_realization_delta_rad = max(maximum_realization_delta_rad, delta)
        if expected_bias != 0.0:
            active_decision_count += 1
        metadata_matches = active == (expected_bias != 0.0)
        if first_realization_mismatch is None and (delta > 1e-12 or not metadata_matches):
            first_realization_mismatch = {
                "step": step,
                "expected_bias_rad": expected_bias,
                "actual_bias_rad": actual_bias,
                "relationship_delta_rad": relationship_delta,
                "metadata_matches": metadata_matches,
            }
    return {
        "contract": contract,
        "maximum_realization_delta_rad": maximum_realization_delta_rad,
        "first_realization_mismatch": first_realization_mismatch,
        "active_decision_count": active_decision_count,
    }


def availability_metrics(
    controller: dict[str, Any], observations: list[dict[str, Any]]
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") != "joint_feedback_publication_dropout_burst_v1":
        return None
    start = contract["start_capture_sequence"]
    count = contract["consecutive_dropped_frames"]
    expected_unpublished = set(range(start, start + count))
    actual_unpublished = {
        frame["step"] for frame in observations if not frame["sensor_sample_published"]
    }
    first_publication_mismatch = next(
        (
            {
                "step": frame["step"],
                "expected_published": frame["step"] not in expected_unpublished,
                "actual_published": frame["sensor_sample_published"],
            }
            for frame in observations
            if frame["sensor_sample_published"]
            != (frame["step"] not in expected_unpublished)
        ),
        None,
    )
    maximum_age_ticks = max(
        (
            frame["controller_observation_age_ticks"]
            for frame in observations
            if frame["controller_observation_age_ticks"] is not None
        ),
        default=0,
    )
    rejected = [frame for frame in observations if frame["controller_rejected"]]
    recovered = [frame for frame in observations if frame["controller_recovered"]]
    maximum_hold_target_delta_rad = 0.0
    maximum_frozen_integral_delta_rad = 0.0
    first_hold_mismatch = None
    for frame in rejected:
        index = frame["step"] - 1
        previous = observations[index - 1]
        target_delta = max(
            abs(actual - expected)
            for actual, expected in zip(
                frame["joint_controller_target_rad"],
                previous["joint_controller_target_rad"],
            )
        )
        integral_delta = max(
            abs(actual - expected)
            for actual, expected in zip(
                frame["joint_integral_correction_rad"],
                previous["joint_integral_correction_rad"],
            )
        )
        maximum_hold_target_delta_rad = max(maximum_hold_target_delta_rad, target_delta)
        maximum_frozen_integral_delta_rad = max(
            maximum_frozen_integral_delta_rad, integral_delta
        )
        metadata_matches = (
            frame["controller_rejection_reason"] == "maximum_observation_age_ticks"
            and frame["fail_safe_hold_active"]
            and frame["controller_state_frozen"]
        )
        if first_hold_mismatch is None and (
            target_delta > 1e-12 or integral_delta > 1e-12 or not metadata_matches
        ):
            first_hold_mismatch = {
                "step": frame["step"],
                "target_delta_rad": target_delta,
                "integral_delta_rad": integral_delta,
                "metadata_matches": metadata_matches,
            }
    recovery_decisions = (
        recovered[0]["step"] - rejected[-1]["step"] if rejected and recovered else 0
    )
    first_source_mismatch = None
    maximum_source_delta_rad = 0.0
    for frame in observations:
        sequence = frame["controller_observation_sequence"]
        visible = frame["joint_controller_observation_position_rad"]
        if sequence is None:
            delta = 0.0 if not visible else math.inf
            source_published = True
        else:
            source = observations[sequence - 1]
            source_published = source["sensor_sample_published"]
            delta = max(
                (abs(actual - raw) for actual, raw in zip(visible, source["joint_position_rad"])),
                default=0.0,
            )
        maximum_source_delta_rad = max(maximum_source_delta_rad, delta)
        if first_source_mismatch is None and (delta > 1e-12 or not source_published):
            first_source_mismatch = {
                "step": frame["step"],
                "controller_observation_sequence": sequence,
                "source_published": source_published,
                "position_delta_rad": delta,
            }
    return {
        "contract": contract,
        "expected_unpublished_sequences": sorted(expected_unpublished),
        "actual_unpublished_sequences": sorted(actual_unpublished),
        "publication_realization_matches": expected_unpublished == actual_unpublished,
        "first_publication_mismatch": first_publication_mismatch,
        "maximum_consecutive_dropout_frames": count,
        "maximum_controller_observation_age_ticks": maximum_age_ticks,
        "rejected_decision_count": len(rejected),
        "first_rejected_step": rejected[0]["step"] if rejected else None,
        "maximum_fail_safe_target_delta_rad": maximum_hold_target_delta_rad,
        "maximum_frozen_integral_delta_rad": maximum_frozen_integral_delta_rad,
        "first_hold_mismatch": first_hold_mismatch,
        "recovery_decision_count": recovery_decisions,
        "first_recovered_step": recovered[0]["step"] if recovered else None,
        "maximum_controller_source_delta_rad": maximum_source_delta_rad,
        "first_controller_source_mismatch": first_source_mismatch,
    }


def first_availability_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    dropout_requirement = requirements[
        "controller.sensor.maximum_consecutive_dropout_frames"
    ]
    if metrics["maximum_consecutive_dropout_frames"] > dropout_requirement["maximum"]:
        step = (
            metrics["contract"]["start_capture_sequence"]
            + int(dropout_requirement["maximum"])
        )
        frame = observations[step - 1]
        candidates.append(
            {
                "requirement_id": dropout_requirement["id"],
                "step": step,
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": int(dropout_requirement["maximum"]) + 1,
                "maximum": dropout_requirement["maximum"],
                "unit": dropout_requirement["unit"],
            }
        )
    age_requirement = requirements["controller.sensor.maximum_observation_age_ticks"]
    age_frame = next(
        (
            frame
            for frame in observations
            if frame["controller_observation_age_ticks"] is not None
            and frame["controller_observation_age_ticks"] > age_requirement["maximum"]
        ),
        None,
    )
    if age_frame is not None:
        candidates.append(
            {
                "requirement_id": age_requirement["id"],
                "step": age_frame["step"],
                "sim_time_ticks": age_frame["sim_time_ticks"],
                "observed": age_frame["controller_observation_age_ticks"],
                "maximum": age_requirement["maximum"],
                "unit": age_requirement["unit"],
            }
        )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def evaluate_trace(
    report_module: Any,
    controller: dict[str, Any],
    controller_path: Path,
    action_path: Path,
    trace_path: Path,
    backend_id: str,
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any]:
    action = load(action_path)
    trace = load(trace_path)
    observations = trace.get("observations", [])
    if (
        trace.get("kind") != "rne_openarm_backend_trace"
        or trace.get("schema_version") != 1
        or trace.get("backend_id") != backend_id
        or trace.get("controller_id") != controller["controller_id"]
        or trace.get("controller_sha256") != sha256(controller_path)
        or trace.get("action_trace_sha256") != sha256(action_path)
        or not trace.get("replay_match")
        or len(observations) != len(action.get("actions", []))
        or len(observations) != 3600
    ):
        raise ValueError(f"{backend_id} robustness trace identity drifted")
    joint_index = controller["action_joint_order"].index(JOINT)
    sample_rate_hz = 1_000_000_000.0 / trace["fixed_delta_ticks"]
    metrics = report_module.disturbance_metrics(
        controller, observations, joint_index, sample_rate_hz
    )
    sensor_metrics = measurement_bias_metrics(controller, observations, joint_index)
    availability = availability_metrics(controller, observations)
    if metrics["first_realization_mismatch"] is not None:
        raise ValueError(f"{backend_id} robustness disturbance realization drifted")
    if availability is not None:
        if (
            not availability["publication_realization_matches"]
            or availability["first_controller_source_mismatch"] is not None
            or availability["first_hold_mismatch"] is not None
        ):
            raise ValueError(f"{backend_id} measurement-dropout realization drifted")
        checks = [
            report_module.check(
                requirements["controller.sensor.maximum_consecutive_dropout_frames"],
                availability["maximum_consecutive_dropout_frames"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_observation_age_ticks"],
                availability["maximum_controller_observation_age_ticks"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_fail_safe_target_delta_rad"],
                availability["maximum_fail_safe_target_delta_rad"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_recovery_decisions"],
                availability["recovery_decision_count"],
            ),
        ]
        first_violation = first_availability_violation(
            availability, observations, requirements
        )
    else:
        checks = [
            report_module.check(
                requirements["controller.state.maximum_disturbance_peak_error_rad"],
                metrics["peak_tracking_error_rad"],
            ),
            report_module.check(
                requirements["controller.state.maximum_disturbance_recovery_time_s"],
                metrics["recovery_check_value_s"],
                recovered=metrics["recovered"],
            ),
            report_module.check(
                requirements["controller.state.maximum_disturbance_iae_rad_s"],
                metrics["iae_rad_s"],
            ),
        ]
        performance_violation = first_requirement_violation(
            observations, metrics, joint_index, sample_rate_hz, requirements
        )
        delay_contract = controller.get("disturbance_contract", {})
        if delay_contract.get("kind") == "actuator_command_transport_delay_pulse_v1":
            delay_requirement = requirements[
                "controller.actuator.maximum_command_transport_delay_steps"
            ]
            checks.append(
                report_module.check(delay_requirement, delay_contract["delay_steps"])
            )
            delay_violation = command_delay_violation(
                controller, observations, delay_requirement
            )
            candidates = [
                candidate
                for candidate in (performance_violation, delay_violation)
                if candidate is not None
            ]
            first_violation = (
                min(candidates, key=lambda item: (item["step"], item["requirement_id"]))
                if candidates
                else None
            )
        elif delay_contract.get("kind") == "actuator_command_slew_rate_limit_pulse_v1":
            rate_requirement = requirements[
                "controller.actuator.minimum_command_slew_rate_rad_s"
            ]
            checks.append(
                report_module.check(
                    rate_requirement, delay_contract["maximum_rate_rad_s"]
                )
            )
            rate_violation = command_rate_limit_violation(
                controller, observations, rate_requirement
            )
            candidates = [
                candidate
                for candidate in (performance_violation, rate_violation)
                if candidate is not None
            ]
            first_violation = (
                min(candidates, key=lambda item: (item["step"], item["requirement_id"]))
                if candidates
                else None
            )
        else:
            first_violation = performance_violation
    if sensor_metrics is not None:
        checks.append(
            report_module.check(
                requirements["controller.sensor.maximum_bias_realization_delta_rad"],
                sensor_metrics["maximum_realization_delta_rad"],
            )
        )
        if sensor_metrics["first_realization_mismatch"] is not None:
            raise ValueError(f"{backend_id} measurement-bias realization drifted")
    status = "passed" if all(check["status"] == "passed" for check in checks) else "failed"
    if (first_violation is None) != (status == "passed"):
        raise ValueError(f"{backend_id} first robustness violation disagrees with checks")
    return {
        "backend_id": backend_id,
        "status": status,
        "trace_sha256": sha256(trace_path),
        "action_trace_sha256": sha256(action_path),
        "replay_match": True,
        "metrics": metrics,
        "measurement_bias": sensor_metrics,
        "measurement_availability": availability,
        "checks": checks,
        "first_violation": first_violation,
        "plot": {
            "reference_rad": [
                frame["joint_reference_position_rad"][joint_index]
                for frame in observations
            ],
            "position_rad": [frame["joint_position_rad"][joint_index] for frame in observations],
            "controller_target_rad": [
                frame["joint_controller_target_rad"][joint_index]
                for frame in observations
            ],
            "applied_target_rad": [
                frame["joint_position_target_rad"][joint_index]
                for frame in observations
            ],
        },
    }


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    root = args.suite_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    suite_path = root / "openarm-robustness-suite.json"
    suite = load(suite_path)
    requirements_registry = load(args.requirements.resolve())
    report_module = load_controller_report_module(Path(__file__).resolve().parent)
    requirements = report_module.requirement_map(requirements_registry)
    if (
        suite.get("kind") != "rne_openarm_robustness_suite"
        or suite.get("schema_version") != 1
        or suite.get("dimension_id")
        not in {
            "actuator_target_bias",
            "actuator_command_delay",
            "actuator_command_rate_limit",
            "joint_position_measurement_bias",
            "joint_feedback_publication_dropout",
        }
        or suite.get("primary_sweep_backend") != "rne_rapier"
        or suite.get("inputs", {}).get("requirements_sha256")
        != sha256(args.requirements.resolve())
        or [case.get("dimension_value", case.get("offset_rad")) for case in suite.get("cases", [])]
        != suite.get("dimension", {}).get("values")
    ):
        raise ValueError("robustness suite identity drifted")
    primary_results = []
    controllers = {}
    for declaration in suite["cases"]:
        case_id = declaration["case_id"]
        dimension_value = declaration.get("dimension_value", declaration.get("offset_rad"))
        controller_path = root / declaration["controller_path"]
        controller = load(controller_path)
        if (
            sha256(controller_path) != declaration["controller_sha256"]
            or controller.get("controller_id") != declaration["controller_id"]
            or controller_dimension_value(controller, suite["dimension_id"])
            != dimension_value
        ):
            raise ValueError(f"{case_id} controller identity drifted")
        controllers[case_id] = (controller, controller_path)
        action_path = root / case_id / "rne_rapier/controller-actions.json"
        trace_path = root / case_id / "rne_rapier/rapier-success-trace.json"
        result = evaluate_trace(
            report_module,
            controller,
            controller_path,
            action_path,
            trace_path,
            "rne_rapier",
            requirements,
        )
        primary_results.append(
            {
                "case_id": case_id,
                "dimension_value": dimension_value,
                **result,
            }
        )
    first_failure_index = next(
        (index for index, result in enumerate(primary_results) if result["status"] == "failed"),
        None,
    )
    if first_failure_index is None or first_failure_index == 0:
        raise ValueError("fixed robustness grid did not bracket a passing/failing boundary")
    if any(result["status"] != "failed" for result in primary_results[first_failure_index:]):
        raise ValueError("robustness result is non-monotonic after the first failed grid point")
    last_passing = primary_results[first_failure_index - 1]
    first_failing = primary_results[first_failure_index]
    boundary_case_ids = [last_passing["case_id"], first_failing["case_id"]]
    cross_backend = []
    for case_id in boundary_case_ids:
        controller, controller_path = controllers[case_id]
        action_path = root / case_id / "rne_rapier/controller-actions.json"
        for backend_id in suite["backend_order"]:
            if backend_id == "rne_rapier":
                result = next(item for item in primary_results if item["case_id"] == case_id)
            else:
                trace_path = root / case_id / backend_id / TRACE_FILES[backend_id]
                result = evaluate_trace(
                    report_module,
                    controller,
                    controller_path,
                    action_path,
                    trace_path,
                    backend_id,
                    requirements,
                )
                result = {
                    "case_id": case_id,
                    "dimension_value": next(
                        item["dimension_value"]
                        for item in primary_results
                        if item["case_id"] == case_id
                    ),
                    **result,
                }
            cross_backend.append(result)
    passing_results = [item for item in cross_backend if item["case_id"] == last_passing["case_id"]]
    failing_results = [item for item in cross_backend if item["case_id"] == first_failing["case_id"]]
    if any(item["status"] != "passed" for item in passing_results):
        raise ValueError("boundary-predecessor case does not pass every backend")
    failure_ids = {
        item["first_violation"]["requirement_id"]
        for item in failing_results
        if item["first_violation"] is not None
    }
    if any(item["status"] != "failed" for item in failing_results) or len(failure_ids) != 1:
        raise ValueError("minimum failing case is not portable across backends")
    first_failure = first_failing["first_violation"] | {
        "case_id": first_failing["case_id"],
        "backend_id": "rne_rapier",
        "dimension_value": first_failing["dimension_value"],
        "final_observed": next(
            check["observed"]
            for check in first_failing["checks"]
            if check["id"] == first_failing["first_violation"]["requirement_id"]
        ),
    }
    report = {
        "kind": "rne_openarm_robustness_report",
        "schema_version": 1,
        "status": "passed",
        "suite_id": suite["suite_id"],
        "task_id": suite["task_id"],
        "controller_role": suite["controller_role"],
        "dimension_id": suite["dimension_id"],
        "dimension": suite["dimension"],
        "evaluation": suite["evaluation"],
        "inputs": {
            "suite_sha256": sha256(suite_path),
            "requirements_sha256": sha256(args.requirements.resolve()),
        },
        "primary_backend_results": primary_results,
        "boundary": {
            "last_passing_case_id": last_passing["case_id"],
            "last_passing_value": last_passing["dimension_value"],
            "first_failing_case_id": first_failing["case_id"],
            "first_failing_value": first_failing["dimension_value"],
            "portable_first_failed_requirement": next(iter(failure_ids)),
        },
        "cross_backend_boundary_results": cross_backend,
        "first_failure": first_failure,
    }
    if suite["dimension_id"] in {
        "actuator_target_bias",
        "joint_position_measurement_bias",
    }:
        for result in report["primary_backend_results"]:
            result["offset_rad"] = result["dimension_value"]
        for result in report["cross_backend_boundary_results"]:
            result["offset_rad"] = result["dimension_value"]
        report["boundary"]["last_passing_offset_rad"] = report["boundary"][
            "last_passing_value"
        ]
        report["boundary"]["first_failing_offset_rad"] = report["boundary"][
            "first_failing_value"
        ]
        report["first_failure"]["offset_rad"] = report["first_failure"][
            "dimension_value"
        ]
    stems = {
        "actuator_target_bias": "openarm-robustness-report",
        "actuator_command_delay": "openarm-command-delay-robustness-report",
        "actuator_command_rate_limit": "openarm-command-rate-limit-robustness-report",
        "joint_position_measurement_bias": "openarm-sensor-bias-robustness-report",
        "joint_feedback_publication_dropout": "openarm-sensor-dropout-robustness-report",
    }
    stem = stems[suite["dimension_id"]]
    write_json(output / f"{stem}.json", report)
    write_html(output / f"{stem}.html", report)
    print(
        "OpenArm robustness report: "
        f"last_pass={last_passing['dimension_value']} {suite['dimension']['unit']} "
        f"first_fail={first_failing['dimension_value']} {suite['dimension']['unit']} "
        f"requirement={next(iter(failure_ids))}"
    )
    return 0


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    document = r'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenArm robustness envelope</title><style>
body{margin:0;background:#09121f;color:#eef5ff;font:14px system-ui,sans-serif}main{max-width:1240px;margin:auto;padding:28px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(250px,1fr));gap:12px}.card{background:#132238;border:1px solid #2a4667;border-radius:10px;padding:14px}.passed{color:#6ee7aa}.failed{color:#ff8b78}table{width:100%;border-collapse:collapse}th,td{border:1px solid #2a4667;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}canvas{width:100%;height:260px;background:#fff;border-radius:8px}</style></head><body><main><h1>OpenArm actuator-bias robustness envelope</h1><div id="summary"></div><h2>Rapier sweep</h2><div id="sweep"></div><h2>Portable boundary</h2><div id="boundary" class="grid"></div><h2>Boundary traces</h2><div id="plots"></div><script>
const r=__REPORT__,f=x=>x==null?'n/a':Number(x).toFixed(6),colors={rne_rapier:'#1261a0',mujoco_native:'#c2410c',gazebo_sim:'#15803d'},availability=r.dimension_id==='joint_feedback_publication_dropout';document.querySelector('#summary').innerHTML=`<section class=card><p>Status: <b class=${r.status}>${r.status}</b></p><p>Last passing value: ${f(r.boundary.last_passing_value)} ${r.dimension.unit}</p><p>First failing value: ${f(r.boundary.first_failing_value)} ${r.dimension.unit}</p><p>Portable first failure: <code>${r.boundary.portable_first_failed_requirement}</code></p><p>First violation: step ${r.first_failure.step}, ${f(r.first_failure.observed)} ${r.first_failure.unit}</p></section>`;
document.querySelector('#sweep').innerHTML=availability?`<table><tr><th>case</th><th>dropped frames</th><th>max age ticks</th><th>rejections</th><th>recovery decisions</th><th>status</th></tr>${r.primary_backend_results.map(q=>`<tr><td>${q.case_id}</td><td>${q.dimension_value}</td><td>${q.measurement_availability.maximum_controller_observation_age_ticks}</td><td>${q.measurement_availability.rejected_decision_count}</td><td>${q.measurement_availability.recovery_decision_count}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`:`<table><tr><th>case</th><th>value</th><th>peak rad</th><th>recovery s</th><th>IAE rad·s</th><th>status</th></tr>${r.primary_backend_results.map(q=>`<tr><td>${q.case_id}</td><td>${f(q.dimension_value)}</td><td>${f(q.metrics.peak_tracking_error_rad)}</td><td>${f(q.metrics.recovery_time_s)}</td><td>${f(q.metrics.iae_rad_s)}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`;
document.querySelector('h1').textContent=`OpenArm ${r.dimension_id.replaceAll('_',' ')} robustness envelope`;document.querySelector('#boundary').innerHTML=r.cross_backend_boundary_results.map(q=>availability?`<section class=card><h3>${q.case_id} / ${q.backend_id}</h3><p class=${q.status}>${q.status}</p><p>max age ${q.measurement_availability.maximum_controller_observation_age_ticks} ticks</p><p>rejections ${q.measurement_availability.rejected_decision_count}</p><p>recovery ${q.measurement_availability.recovery_decision_count} decision(s)</p></section>`:`<section class=card><h3>${q.case_id} / ${q.backend_id}</h3><p class=${q.status}>${q.status}</p><p>peak ${f(q.metrics.peak_tracking_error_rad)} rad</p><p>recovery ${f(q.metrics.recovery_time_s)} s</p><p>IAE ${f(q.metrics.iae_rad_s)} rad·s</p></section>`).join('');function plot(caseId){const rows=r.cross_backend_boundary_results.filter(q=>q.case_id===caseId),c=document.createElement('canvas');c.width=1160;c.height=260;const x=c.getContext('2d'),n=rows[0].plot.reference_rad.length,start=(r.dimension.start_step??r.dimension.start_controller_step??r.dimension.start_capture_sequence)-1,end=r.dimension.end_step??r.dimension.end_controller_step??(r.dimension.start_capture_sequence+r.boundary.first_failing_value);x.fillStyle='#ef444422';x.fillRect(start/(n-1)*c.width,0,(end-start)/(n-1)*c.width,c.height);function line(v,color,w=1.4){x.beginPath();for(let i=0;i<n;i++){const px=i/(n-1)*c.width,py=c.height-(v[i]+.16)/.34*c.height;i?x.lineTo(px,py):x.moveTo(px,py)}x.strokeStyle=color;x.lineWidth=w;x.stroke()}line(rows[0].plot.reference_rad,'#111',1.8);rows.forEach(q=>line(q.plot.position_rad,colors[q.backend_id]));const s=document.createElement('section');s.innerHTML=`<h3>${caseId}</h3>`;s.appendChild(c);return s}const plots=document.querySelector('#plots');plots.appendChild(plot(r.boundary.last_passing_case_id));plots.appendChild(plot(r.boundary.first_failing_case_id));
</script></main></body></html>'''.replace("__REPORT__", payload)
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm robustness report failed: {error}")
        raise SystemExit(2)

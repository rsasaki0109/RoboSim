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
    if dimension_id == "actuator_command_deadband":
        return controller.get("disturbance_contract", {}).get("deadband_rad")
    if dimension_id == "joint_position_measurement_bias":
        return controller.get("measurement_fault_contract", {}).get("offset_rad")
    if dimension_id == "joint_feedback_publication_dropout":
        return controller.get("measurement_fault_contract", {}).get(
            "consecutive_dropped_frames"
        )
    if dimension_id == "joint_feedback_controller_ingress_latency":
        return controller.get("measurement_fault_contract", {}).get("delay_frames")
    if dimension_id == "joint_feedback_controller_ingress_jitter":
        return controller.get("measurement_fault_contract", {}).get(
            "maximum_jitter_frames"
        )
    if dimension_id == "joint_feedback_controller_stale_age":
        return controller.get("measurement_fault_contract", {}).get(
            "additional_stale_frames"
        )
    if dimension_id == "joint_feedback_dropout_recovery":
        return controller.get("measurement_fault_contract", {}).get(
            "additional_recovery_hold_decisions"
        )
    if dimension_id == "joint_feedback_repeated_dropout_rearm":
        return controller.get("measurement_fault_contract", {}).get(
            "interburst_fresh_frames"
        )
    if dimension_id == "joint_position_measurement_quantization":
        return controller.get("measurement_fault_contract", {}).get(
            "quantization_step_rad"
        )
    if dimension_id == "joint_position_measurement_saturation":
        return controller.get("measurement_fault_contract", {}).get(
            "saturation_limit_abs_rad"
        )
    if dimension_id == "joint_position_stuck_value":
        return controller.get("measurement_fault_contract", {}).get(
            "consecutive_stuck_frames"
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


def command_deadband_violation(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    requirement: dict[str, Any],
) -> dict[str, Any] | None:
    contract = controller.get("disturbance_contract", {})
    if contract.get("kind") != "actuator_command_deadband_pulse_v1":
        return None
    deadband_rad = contract["deadband_rad"]
    if deadband_rad <= requirement["maximum"]:
        return None
    step = contract["start_step"]
    frame = observations[step - 1]
    return {
        "requirement_id": requirement["id"],
        "step": step,
        "sim_time_ticks": frame["sim_time_ticks"],
        "observed": deadband_rad,
        "maximum": requirement["maximum"],
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


def quantization_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") != "joint_position_quantization_pulse_v1":
        return None
    step_rad = contract["quantization_step_rad"]
    maximum_realization_delta_rad = 0.0
    maximum_quantization_error_rad = 0.0
    first_realization_mismatch = None
    active_decision_count = 0
    for frame in observations:
        controller_step = frame["step"]
        sequence = frame["controller_observation_sequence"]
        visible = frame["joint_controller_observation_position_rad"]
        active = (
            sequence is not None
            and contract["start_controller_step"]
            <= controller_step
            <= contract["end_controller_step"]
        )
        if sequence is None:
            continue
        raw = observations[sequence - 1]["joint_position_rad"][joint_index]
        if active and step_rad > 0.0:
            expected = math.copysign(
                math.floor(abs(raw) / step_rad + 0.5) * step_rad, raw
            )
            active_decision_count += 1
        else:
            expected = raw
            active_decision_count += int(active)
        actual = visible[joint_index]
        realization_delta = abs(actual - expected)
        quantization_error = abs(actual - raw) if active else 0.0
        maximum_realization_delta_rad = max(
            maximum_realization_delta_rad, realization_delta
        )
        maximum_quantization_error_rad = max(
            maximum_quantization_error_rad, quantization_error
        )
        if first_realization_mismatch is None and realization_delta > 1e-12:
            first_realization_mismatch = {
                "step": controller_step,
                "observation_sequence": sequence,
                "raw_position_rad": raw,
                "expected_quantized_position_rad": expected,
                "actual_controller_position_rad": actual,
                "realization_delta_rad": realization_delta,
            }
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    maximum_age_ticks = max(
        (
            frame["controller_observation_age_ticks"]
            for frame in observations
            if frame["controller_observation_age_ticks"] is not None
        ),
        default=0,
    )
    return {
        "contract": contract,
        "quantization_step_rad": step_rad,
        "maximum_quantization_error_rad": maximum_quantization_error_rad,
        "maximum_realization_delta_rad": maximum_realization_delta_rad,
        "first_realization_mismatch": first_realization_mismatch,
        "active_decision_count": active_decision_count,
        "controlled_joint_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "controlled_joint_final_error_rad": abs(errors[-1]),
        "maximum_controller_observation_age_ticks": maximum_age_ticks,
        "rejected_decision_count": sum(
            bool(frame["controller_rejected"]) for frame in observations
        ),
        "recovery_decision_count": 0,
    }


def first_quantization_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    step_requirement = requirements[
        "controller.sensor.maximum_position_quantization_step_rad"
    ]
    if metrics["quantization_step_rad"] > step_requirement["maximum"]:
        step = metrics["contract"]["start_controller_step"]
        frame = observations[step - 1]
        candidates.append(
            {
                "requirement_id": step_requirement["id"],
                "step": step,
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": metrics["quantization_step_rad"],
                "maximum": step_requirement["maximum"],
                "unit": step_requirement["unit"],
            }
        )
    for requirement_id, metric_name in (
        (
            "controller.sensor.maximum_quantization_realization_delta_rad",
            "maximum_realization_delta_rad",
        ),
        (
            "controller.sensor_quantization.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_quantization.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_name] > requirement["maximum"]:
            frame = observations[-1]
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "observed": metrics[metric_name],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
                }
            )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def saturation_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") != "joint_position_saturation_pulse_v1":
        return None
    limit_rad = contract["saturation_limit_abs_rad"]
    maximum_realization_delta_rad = 0.0
    maximum_saturation_error_rad = 0.0
    first_realization_mismatch = None
    first_saturated_observation = None
    active_decision_count = 0
    saturated_decision_count = 0
    for frame in observations:
        controller_step = frame["step"]
        sequence = frame["controller_observation_sequence"]
        visible = frame["joint_controller_observation_position_rad"]
        active = (
            sequence is not None
            and contract["start_controller_step"]
            <= controller_step
            <= contract["end_controller_step"]
        )
        if sequence is None:
            continue
        raw = observations[sequence - 1]["joint_position_rad"][joint_index]
        expected = max(-limit_rad, min(limit_rad, raw)) if active else raw
        actual = visible[joint_index]
        realization_delta = abs(actual - expected)
        saturation_error = abs(actual - raw) if active else 0.0
        active_decision_count += int(active)
        saturated = active and abs(expected - raw) > 1e-15
        saturated_decision_count += int(saturated)
        if saturated and first_saturated_observation is None:
            first_saturated_observation = {
                "step": controller_step,
                "observation_sequence": sequence,
                "raw_position_rad": raw,
                "saturated_position_rad": expected,
            }
        maximum_realization_delta_rad = max(
            maximum_realization_delta_rad, realization_delta
        )
        maximum_saturation_error_rad = max(
            maximum_saturation_error_rad, saturation_error
        )
        metadata_matches = bool(frame["measurement_bias_active"]) == saturated
        if first_realization_mismatch is None and (
            realization_delta > 1e-12 or not metadata_matches
        ):
            first_realization_mismatch = {
                "step": controller_step,
                "observation_sequence": sequence,
                "raw_position_rad": raw,
                "expected_saturated_position_rad": expected,
                "actual_controller_position_rad": actual,
                "realization_delta_rad": realization_delta,
                "metadata_matches": metadata_matches,
            }
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    maximum_age_ticks = max(
        (
            frame["controller_observation_age_ticks"]
            for frame in observations
            if frame["controller_observation_age_ticks"] is not None
        ),
        default=0,
    )
    return {
        "contract": contract,
        "saturation_limit_abs_rad": limit_rad,
        "maximum_saturation_error_rad": maximum_saturation_error_rad,
        "maximum_realization_delta_rad": maximum_realization_delta_rad,
        "first_realization_mismatch": first_realization_mismatch,
        "first_saturated_observation": first_saturated_observation,
        "active_decision_count": active_decision_count,
        "saturated_decision_count": saturated_decision_count,
        "controlled_joint_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "controlled_joint_final_error_rad": abs(errors[-1]),
        "maximum_controller_observation_age_ticks": maximum_age_ticks,
        "rejected_decision_count": sum(
            bool(frame["controller_rejected"]) for frame in observations
        ),
        "recovery_decision_count": 0,
    }


def first_saturation_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    range_requirement = requirements[
        "controller.sensor.minimum_position_saturation_limit_abs_rad"
    ]
    if metrics["saturation_limit_abs_rad"] < range_requirement["minimum"]:
        first_saturated = metrics["first_saturated_observation"]
        if first_saturated is None:
            raise ValueError("unsupported measurement range did not exercise saturation")
        step = first_saturated["step"]
        frame = observations[step - 1]
        candidates.append(
            {
                "requirement_id": range_requirement["id"],
                "step": step,
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": metrics["saturation_limit_abs_rad"],
                "minimum": range_requirement["minimum"],
                "unit": range_requirement["unit"],
                "controller_observation_sequence": first_saturated[
                    "observation_sequence"
                ],
                "raw_position_rad": first_saturated["raw_position_rad"],
                "saturated_position_rad": first_saturated[
                    "saturated_position_rad"
                ],
            }
        )
    for requirement_id, metric_name in (
        (
            "controller.sensor.maximum_saturation_realization_delta_rad",
            "maximum_realization_delta_rad",
        ),
        (
            "controller.sensor_saturation.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_saturation.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_name] > requirement["maximum"]:
            frame = observations[-1]
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "observed": metrics[metric_name],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
                }
            )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def stuck_value_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") != "joint_position_stuck_value_burst_v1":
        return None
    start = contract["start_capture_sequence"]
    count = contract["consecutive_stuck_frames"]
    end = start + count
    held_position_rad = observations[start - 2]["joint_position_rad"][joint_index]
    expected_sequences = list(range(start, end))
    captured_stuck_sequences = [
        frame["step"]
        for frame in observations
        if frame["sensor_status"] == "stuck_value"
    ]
    status_realization_matches = captured_stuck_sequences == expected_sequences
    consumed = [
        frame
        for frame in observations
        if frame["controller_observation_sequence"] in expected_sequences
    ]
    maximum_realization_delta_rad = 0.0
    maximum_raw_source_divergence_rad = 0.0
    maximum_fail_safe_target_delta_rad = 0.0
    first_realization_mismatch = None
    first_policy_mismatch = None
    rejected_before = None
    for frame in consumed:
        sequence = frame["controller_observation_sequence"]
        actual = frame["joint_controller_observation_position_rad"][joint_index]
        realization_delta = abs(actual - held_position_rad)
        raw = observations[sequence - 1]["joint_position_rad"][joint_index]
        maximum_realization_delta_rad = max(
            maximum_realization_delta_rad, realization_delta
        )
        maximum_raw_source_divergence_rad = max(
            maximum_raw_source_divergence_rad, abs(raw - held_position_rad)
        )
        if first_realization_mismatch is None and realization_delta > 1e-12:
            first_realization_mismatch = {
                "step": frame["step"],
                "observation_sequence": sequence,
                "expected_held_position_rad": held_position_rad,
                "actual_controller_position_rad": actual,
                "realization_delta_rad": realization_delta,
            }
        policy_matches = (
            frame["controller_rejected"]
            and frame["controller_rejection_reason"] == "required_sensor_status"
            and frame["fail_safe_hold_active"]
            and frame["controller_state_frozen"]
        )
        if first_policy_mismatch is None and not policy_matches:
            first_policy_mismatch = {
                "step": frame["step"],
                "observation_sequence": sequence,
                "controller_rejected": frame["controller_rejected"],
                "controller_rejection_reason": frame[
                    "controller_rejection_reason"
                ],
                "fail_safe_hold_active": frame["fail_safe_hold_active"],
                "controller_state_frozen": frame["controller_state_frozen"],
            }
        if rejected_before is None:
            rejected_before = observations[frame["step"] - 2][
                "joint_controller_target_rad"
            ][joint_index]
        maximum_fail_safe_target_delta_rad = max(
            maximum_fail_safe_target_delta_rad,
            abs(frame["joint_controller_target_rad"][joint_index] - rejected_before),
        )
    last_rejected_step = consumed[-1]["step"] if consumed else None
    recovered = next(
        (
            frame
            for frame in observations
            if last_rejected_step is not None
            and frame["step"] > last_rejected_step
            and frame["controller_recovered"]
        ),
        None,
    )
    recovery_decision_count = (
        recovered["step"] - last_rejected_step
        if recovered is not None and last_rejected_step is not None
        else 0
    )
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    return {
        "contract": contract,
        "consecutive_stuck_frames": count,
        "held_source_sequence": start - 1,
        "held_position_rad": held_position_rad,
        "captured_stuck_sequences": captured_stuck_sequences,
        "status_realization_matches": status_realization_matches,
        "consumed_stuck_decision_count": len(consumed),
        "rejected_decision_count": sum(
            bool(frame["controller_rejected"]) for frame in consumed
        ),
        "maximum_realization_delta_rad": maximum_realization_delta_rad,
        "maximum_raw_source_divergence_rad": maximum_raw_source_divergence_rad,
        "maximum_fail_safe_target_delta_rad": maximum_fail_safe_target_delta_rad,
        "first_realization_mismatch": first_realization_mismatch,
        "first_policy_mismatch": first_policy_mismatch,
        "last_rejected_step": last_rejected_step,
        "recovery_decision_count": recovery_decision_count,
        "recovered_step": recovered["step"] if recovered is not None else None,
        "controlled_joint_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "controlled_joint_final_error_rad": abs(errors[-1]),
        "maximum_controller_observation_age_ticks": max(
            (
                frame["controller_observation_age_ticks"]
                for frame in observations
                if frame["controller_observation_age_ticks"] is not None
            ),
            default=0,
        ),
    }


def first_stuck_value_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    count_requirement = requirements[
        "controller.sensor.maximum_consecutive_stuck_value_frames"
    ]
    if metrics["consecutive_stuck_frames"] > count_requirement["maximum"]:
        sequence = (
            metrics["contract"]["start_capture_sequence"]
            + count_requirement["maximum"]
        )
        frame = next(
            item
            for item in observations
            if item["controller_observation_sequence"] == sequence
        )
        candidates.append(
            {
                "requirement_id": count_requirement["id"],
                "step": frame["step"],
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": metrics["consecutive_stuck_frames"],
                "maximum": count_requirement["maximum"],
                "unit": count_requirement["unit"],
                "controller_observation_sequence": sequence,
                "held_source_sequence": metrics["held_source_sequence"],
                "held_position_rad": metrics["held_position_rad"],
            }
        )
    for requirement_id, metric_name in (
        (
            "controller.sensor.maximum_stuck_value_realization_delta_rad",
            "maximum_realization_delta_rad",
        ),
        (
            "controller.sensor.maximum_fail_safe_target_delta_rad",
            "maximum_fail_safe_target_delta_rad",
        ),
        (
            "controller.sensor.maximum_recovery_decisions",
            "recovery_decision_count",
        ),
        (
            "controller.sensor_stuck.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_stuck.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_name] > requirement["maximum"]:
            frame = observations[-1]
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "observed": metrics[metric_name],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
                }
            )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def availability_metrics(
    controller: dict[str, Any], observations: list[dict[str, Any]]
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") not in {
        "joint_feedback_publication_dropout_burst_v1",
        "joint_feedback_dropout_recovery_hold_v1",
        "joint_feedback_repeated_dropout_bursts_v1",
    }:
        return None
    start = contract["start_capture_sequence"]
    if contract["kind"] == "joint_feedback_repeated_dropout_bursts_v1":
        burst_length = contract["burst_length_frames"]
        second_start = start + burst_length + contract["interburst_fresh_frames"]
        expected_unpublished = set(range(start, start + burst_length)) | set(
            range(second_start, second_start + burst_length)
        )
    else:
        count = contract["consecutive_dropped_frames"]
        expected_unpublished = set(range(start, start + count))
    actual_unpublished = {
        frame["step"] for frame in observations if not frame["sensor_sample_published"]
    }
    maximum_consecutive_dropout_frames = 0
    consecutive_dropout_frames = 0
    for frame in observations:
        if frame["step"] in actual_unpublished:
            consecutive_dropout_frames += 1
            maximum_consecutive_dropout_frames = max(
                maximum_consecutive_dropout_frames, consecutive_dropout_frames
            )
        else:
            consecutive_dropout_frames = 0
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
    stale_rejected = [
        frame
        for frame in rejected
        if frame["controller_rejection_reason"] == "maximum_observation_age_ticks"
    ]
    recovery_hold_rejected = [
        frame
        for frame in rejected
        if frame["controller_rejection_reason"] == "recovery_confirmation_pending"
    ]
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
        allowed_rejection_reasons = {"maximum_observation_age_ticks"}
        if contract.get("kind") == "joint_feedback_dropout_recovery_hold_v1":
            allowed_rejection_reasons.add("recovery_confirmation_pending")
        metadata_matches = (
            frame["controller_rejection_reason"] in allowed_rejection_reasons
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
        recovered[0]["step"] - stale_rejected[-1]["step"]
        if stale_rejected and recovered
        else 0
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
        "maximum_consecutive_dropout_frames": maximum_consecutive_dropout_frames,
        "maximum_controller_observation_age_ticks": maximum_age_ticks,
        "rejected_decision_count": len(rejected),
        "stale_rejected_decision_count": len(stale_rejected),
        "recovery_hold_rejected_decision_count": len(recovery_hold_rejected),
        "first_rejected_step": rejected[0]["step"] if rejected else None,
        "maximum_fail_safe_target_delta_rad": maximum_hold_target_delta_rad,
        "maximum_frozen_integral_delta_rad": maximum_frozen_integral_delta_rad,
        "first_hold_mismatch": first_hold_mismatch,
        "recovery_decision_count": recovery_decisions,
        "first_recovered_step": recovered[0]["step"] if recovered else None,
        "maximum_controller_source_delta_rad": maximum_source_delta_rad,
        "first_controller_source_mismatch": first_source_mismatch,
    }


def recovery_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") != "joint_feedback_dropout_recovery_hold_v1":
        return None
    metrics = availability_metrics(controller, observations)
    if metrics is None:
        raise ValueError("dropout-recovery availability metrics are missing")
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    metrics["additional_recovery_hold_decisions"] = contract[
        "additional_recovery_hold_decisions"
    ]
    metrics["controlled_joint_rmse_rad"] = math.sqrt(
        sum(error * error for error in errors) / len(errors)
    )
    metrics["controlled_joint_final_error_rad"] = abs(errors[-1])
    return metrics


def rearm_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if (
        contract is None
        or contract.get("kind") != "joint_feedback_repeated_dropout_bursts_v1"
    ):
        return None
    metrics = availability_metrics(controller, observations)
    if metrics is None:
        raise ValueError("repeated-dropout re-arm availability metrics are missing")
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    metrics["interburst_fresh_frames"] = contract["interburst_fresh_frames"]
    metrics["controlled_joint_rmse_rad"] = math.sqrt(
        sum(error * error for error in errors) / len(errors)
    )
    metrics["controlled_joint_final_error_rad"] = abs(errors[-1])
    return metrics


def first_rearm_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    spacing_requirement = requirements[
        "controller.sensor.minimum_interburst_fresh_frames"
    ]
    if metrics["interburst_fresh_frames"] < spacing_requirement["minimum"]:
        step = (
            metrics["contract"]["start_capture_sequence"]
            + metrics["contract"]["burst_length_frames"]
        )
        frame = observations[step - 1]
        candidates.append(
            {
                "requirement_id": spacing_requirement["id"],
                "step": step,
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": metrics["interburst_fresh_frames"],
                "minimum": spacing_requirement["minimum"],
                "unit": spacing_requirement["unit"],
            }
        )
    for requirement_id, metric_name in (
        (
            "controller.sensor.maximum_observation_age_ticks",
            "maximum_controller_observation_age_ticks",
        ),
        (
            "controller.sensor.maximum_fail_safe_target_delta_rad",
            "maximum_fail_safe_target_delta_rad",
        ),
        (
            "controller.sensor.maximum_recovery_decisions",
            "recovery_decision_count",
        ),
        (
            "controller.sensor_rearm.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_rearm.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_name] > requirement["maximum"]:
            frame = observations[-1]
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "observed": metrics[metric_name],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
                }
            )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def first_recovery_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    recovery_requirement = requirements["controller.sensor.maximum_recovery_decisions"]
    if metrics["recovery_decision_count"] > recovery_requirement["maximum"]:
        step = metrics["first_recovered_step"]
        frame = observations[step - 1]
        candidates.append(
            {
                "requirement_id": recovery_requirement["id"],
                "step": step,
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": metrics["recovery_decision_count"],
                "maximum": recovery_requirement["maximum"],
                "unit": recovery_requirement["unit"],
            }
        )
    for requirement_id, metric_name in (
        (
            "controller.sensor_recovery.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_recovery.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_name] > requirement["maximum"]:
            frame = observations[-1]
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "observed": metrics[metric_name],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
                }
            )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


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


def latency_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
    fixed_delta_ticks: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") != "joint_feedback_controller_ingress_delay_v1":
        return None
    delay_frames = contract["delay_frames"]
    expected_age_ticks = (delay_frames + 1) * fixed_delta_ticks
    first_realization_mismatch = None
    maximum_source_delta_rad = 0.0
    maximum_age_ticks = 0
    for frame in observations:
        sequence = frame["controller_observation_sequence"]
        visible = frame["joint_controller_observation_position_rad"]
        expected_sequence = frame["step"] - delay_frames - 2
        if expected_sequence < 1:
            expected_sequence = None
        if sequence is None:
            source_delta = 0.0 if not visible else math.inf
            age_matches = frame["controller_observation_age_ticks"] is None
        else:
            source = observations[sequence - 1]
            source_delta = max(
                (
                    abs(actual - raw)
                    for actual, raw in zip(visible, source["joint_position_rad"])
                ),
                default=0.0,
            )
            age = frame["controller_observation_age_ticks"]
            maximum_age_ticks = max(maximum_age_ticks, age)
            age_matches = age == expected_age_ticks
        maximum_source_delta_rad = max(maximum_source_delta_rad, source_delta)
        realization_matches = (
            frame["sensor_sample_published"]
            and sequence == expected_sequence
            and age_matches
            and source_delta <= 1e-12
        )
        if first_realization_mismatch is None and not realization_matches:
            first_realization_mismatch = {
                "step": frame["step"],
                "expected_observation_sequence": expected_sequence,
                "actual_observation_sequence": sequence,
                "expected_age_ticks": (
                    None if expected_sequence is None else expected_age_ticks
                ),
                "actual_age_ticks": frame["controller_observation_age_ticks"],
                "source_delta_rad": source_delta,
                "sensor_sample_published": frame["sensor_sample_published"],
            }
    rejected = [frame for frame in observations if frame["controller_rejected"]]
    maximum_hold_target_delta_rad = 0.0
    maximum_frozen_integral_delta_rad = 0.0
    first_hold_mismatch = None
    for frame in rejected:
        previous = observations[frame["step"] - 2]
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
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    return {
        "contract": contract,
        "base_sensor_latency_ticks": fixed_delta_ticks,
        "controller_ingress_delay_frames": delay_frames,
        "expected_controller_observation_age_ticks": expected_age_ticks,
        "maximum_controller_observation_age_ticks": maximum_age_ticks,
        "bootstrap_decision_count": sum(
            frame["controller_bootstrap"] for frame in observations
        ),
        "rejected_decision_count": len(rejected),
        "first_rejected_step": rejected[0]["step"] if rejected else None,
        "maximum_fail_safe_target_delta_rad": maximum_hold_target_delta_rad,
        "maximum_frozen_integral_delta_rad": maximum_frozen_integral_delta_rad,
        "maximum_controller_source_delta_rad": maximum_source_delta_rad,
        "first_realization_mismatch": first_realization_mismatch,
        "first_hold_mismatch": first_hold_mismatch,
        "controlled_joint_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "controlled_joint_final_error_rad": abs(errors[-1]),
    }


def first_latency_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    delay_requirement = requirements[
        "controller.sensor.maximum_controller_ingress_delay_frames"
    ]
    if metrics["controller_ingress_delay_frames"] > delay_requirement["maximum"]:
        step = metrics["bootstrap_decision_count"] + 1
        frame = observations[step - 1]
        candidates.append(
            {
                "requirement_id": delay_requirement["id"],
                "step": step,
                "sim_time_ticks": frame["sim_time_ticks"],
                "observed": metrics["controller_ingress_delay_frames"],
                "maximum": delay_requirement["maximum"],
                "unit": delay_requirement["unit"],
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
    final_step = observations[-1]
    for requirement_id, metric_key in (
        (
            "controller.sensor_latency.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_latency.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_key] > requirement["maximum"]:
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": final_step["step"],
                    "sim_time_ticks": final_step["sim_time_ticks"],
                    "observed": metrics[metric_key],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
                }
            )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def jitter_delay_frames(contract: dict[str, Any], capture_sequence: int) -> int:
    maximum = contract["maximum_jitter_frames"]
    if (
        maximum == 0
        or capture_sequence < contract["start_capture_sequence"]
        or capture_sequence > contract["end_capture_sequence"]
    ):
        return 0
    phase = (capture_sequence - contract["start_capture_sequence"]) % (maximum + 1)
    return maximum if phase < maximum else 0


def jitter_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
    fixed_delta_ticks: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if (
        contract is None
        or contract.get("kind")
        != "joint_feedback_controller_ingress_jitter_pulse_v1"
    ):
        return None
    first_realization_mismatch = None
    maximum_source_delta_rad = 0.0
    maximum_age_ticks = 0
    maximum_realized_jitter_frames = 0
    realized_jitter_by_step = []
    eligible_at_step: dict[int, list[int]] = {}
    for source in observations:
        eligible_step = (
            source["step"] + jitter_delay_frames(contract, source["step"]) + 2
        )
        eligible_at_step.setdefault(eligible_step, []).append(source["step"])
    latest_eligible_sequence = None
    for frame in observations:
        consumed_at_ticks = frame["sim_time_ticks"] - fixed_delta_ticks
        for sequence in eligible_at_step.get(frame["step"], []):
            if observations[sequence - 1]["sensor_sample_published"] and (
                latest_eligible_sequence is None or sequence > latest_eligible_sequence
            ):
                latest_eligible_sequence = sequence
        expected_sequence = latest_eligible_sequence
        sequence = frame["controller_observation_sequence"]
        visible = frame["joint_controller_observation_position_rad"]
        if expected_sequence is None:
            expected_age_ticks = None
            source_delta = 0.0 if not visible else math.inf
            realized_jitter_frames = 0
        else:
            source = observations[expected_sequence - 1]
            expected_age_ticks = consumed_at_ticks - source["sim_time_ticks"]
            source_delta = max(
                (
                    abs(actual - raw)
                    for actual, raw in zip(visible, source["joint_position_rad"])
                ),
                default=0.0,
            )
            realized_jitter_frames = max(
                expected_age_ticks // fixed_delta_ticks - 1, 0
            )
            maximum_age_ticks = max(maximum_age_ticks, expected_age_ticks)
            maximum_realized_jitter_frames = max(
                maximum_realized_jitter_frames, realized_jitter_frames
            )
        realized_jitter_by_step.append(realized_jitter_frames)
        maximum_source_delta_rad = max(maximum_source_delta_rad, source_delta)
        realization_matches = (
            frame["sensor_sample_published"]
            and sequence == expected_sequence
            and frame["controller_observation_age_ticks"] == expected_age_ticks
            and source_delta <= 1e-12
        )
        if first_realization_mismatch is None and not realization_matches:
            first_realization_mismatch = {
                "step": frame["step"],
                "expected_observation_sequence": expected_sequence,
                "actual_observation_sequence": sequence,
                "expected_age_ticks": expected_age_ticks,
                "actual_age_ticks": frame["controller_observation_age_ticks"],
                "source_delta_rad": source_delta,
                "sensor_sample_published": frame["sensor_sample_published"],
            }
    rejected = [frame for frame in observations if frame["controller_rejected"]]
    maximum_hold_target_delta_rad = 0.0
    maximum_frozen_integral_delta_rad = 0.0
    first_hold_mismatch = None
    for frame in rejected:
        previous = observations[frame["step"] - 2]
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
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    jittered_captures = [
        sequence
        for sequence in range(
            contract["start_capture_sequence"], contract["end_capture_sequence"] + 1
        )
        if jitter_delay_frames(contract, sequence) > 0
    ]
    return {
        "contract": contract,
        "base_sensor_latency_ticks": fixed_delta_ticks,
        "maximum_declared_jitter_frames": contract["maximum_jitter_frames"],
        "maximum_realized_jitter_frames": maximum_realized_jitter_frames,
        "maximum_controller_observation_age_ticks": maximum_age_ticks,
        "jittered_capture_count": len(jittered_captures),
        "nominal_capture_count_within_window": (
            contract["end_capture_sequence"]
            - contract["start_capture_sequence"]
            + 1
            - len(jittered_captures)
        ),
        "realized_jitter_frames_by_step": realized_jitter_by_step,
        "bootstrap_decision_count": sum(
            frame["controller_bootstrap"] for frame in observations
        ),
        "rejected_decision_count": len(rejected),
        "first_rejected_step": rejected[0]["step"] if rejected else None,
        "maximum_fail_safe_target_delta_rad": maximum_hold_target_delta_rad,
        "maximum_frozen_integral_delta_rad": maximum_frozen_integral_delta_rad,
        "maximum_controller_source_delta_rad": maximum_source_delta_rad,
        "first_realization_mismatch": first_realization_mismatch,
        "first_hold_mismatch": first_hold_mismatch,
        "controlled_joint_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "controlled_joint_final_error_rad": abs(errors[-1]),
    }


def first_jitter_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
    jitter_requirement = requirements[
        "controller.sensor.maximum_controller_ingress_jitter_frames"
    ]
    jitter_frame = next(
        (
            frame
            for frame, realized in zip(
                observations, metrics["realized_jitter_frames_by_step"]
            )
            if realized > jitter_requirement["maximum"]
        ),
        None,
    )
    if jitter_frame is not None:
        realized = metrics["realized_jitter_frames_by_step"][jitter_frame["step"] - 1]
        candidates.append(
            {
                "requirement_id": jitter_requirement["id"],
                "step": jitter_frame["step"],
                "sim_time_ticks": jitter_frame["sim_time_ticks"],
                "observed": realized,
                "maximum": jitter_requirement["maximum"],
                "unit": jitter_requirement["unit"],
                "controller_observation_sequence": jitter_frame[
                    "controller_observation_sequence"
                ],
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
    final_frame = observations[-1]
    for requirement_id, metric_key in (
        (
            "controller.sensor_jitter.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_jitter.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_key] > requirement["maximum"]:
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": final_frame["step"],
                    "sim_time_ticks": final_frame["sim_time_ticks"],
                    "observed": metrics[metric_key],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
                }
            )
    return min(candidates, key=lambda item: (item["step"], item["requirement_id"])) if candidates else None


def stale_age_metrics(
    controller: dict[str, Any],
    observations: list[dict[str, Any]],
    joint_index: int,
    fixed_delta_ticks: int,
) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if (
        contract is None
        or contract.get("kind") != "joint_feedback_controller_stale_age_pulse_v1"
    ):
        return None
    maximum_age_ticks = 0
    maximum_selected_stale_frames = 0
    selected_stale_frames_by_step = []
    maximum_source_delta_rad = 0.0
    first_realization_mismatch = None
    for frame in observations:
        step = frame["step"]
        offset = (
            contract["additional_stale_frames"]
            if contract["start_controller_step"]
            <= step
            <= contract["end_controller_step"]
            else 0
        )
        expected_sequence = step - 2 - offset
        if expected_sequence < 1:
            expected_sequence = None
        expected_age_ticks = (
            None if expected_sequence is None else (offset + 1) * fixed_delta_ticks
        )
        sequence = frame["controller_observation_sequence"]
        visible = frame["joint_controller_observation_position_rad"]
        if expected_sequence is None:
            source_delta = 0.0 if not visible else math.inf
            selected_stale_frames = 0
        else:
            source = observations[expected_sequence - 1]
            source_delta = max(
                (
                    abs(actual - raw)
                    for actual, raw in zip(visible, source["joint_position_rad"])
                ),
                default=0.0,
            )
            selected_stale_frames = offset
            maximum_age_ticks = max(maximum_age_ticks, expected_age_ticks)
            maximum_selected_stale_frames = max(
                maximum_selected_stale_frames, selected_stale_frames
            )
        selected_stale_frames_by_step.append(selected_stale_frames)
        maximum_source_delta_rad = max(maximum_source_delta_rad, source_delta)
        realization_matches = (
            frame["sensor_sample_published"]
            and sequence == expected_sequence
            and frame["controller_observation_age_ticks"] == expected_age_ticks
            and source_delta <= 1e-12
        )
        if first_realization_mismatch is None and not realization_matches:
            first_realization_mismatch = {
                "step": step,
                "expected_observation_sequence": expected_sequence,
                "actual_observation_sequence": sequence,
                "expected_age_ticks": expected_age_ticks,
                "actual_age_ticks": frame["controller_observation_age_ticks"],
                "source_delta_rad": source_delta,
                "sensor_sample_published": frame["sensor_sample_published"],
            }
    rejected = [frame for frame in observations if frame["controller_rejected"]]
    recovered = [frame for frame in observations if frame["controller_recovered"]]
    maximum_hold_target_delta_rad = 0.0
    maximum_frozen_integral_delta_rad = 0.0
    first_hold_mismatch = None
    for frame in rejected:
        previous = observations[frame["step"] - 2]
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
    errors = [
        frame["joint_position_rad"][joint_index]
        - frame["joint_reference_position_rad"][joint_index]
        for frame in observations
    ]
    return {
        "contract": contract,
        "base_sensor_latency_ticks": fixed_delta_ticks,
        "maximum_selected_stale_frames": maximum_selected_stale_frames,
        "maximum_controller_observation_age_ticks": maximum_age_ticks,
        "selected_stale_frames_by_step": selected_stale_frames_by_step,
        "rejected_decision_count": len(rejected),
        "first_rejected_step": rejected[0]["step"] if rejected else None,
        "maximum_fail_safe_target_delta_rad": maximum_hold_target_delta_rad,
        "maximum_frozen_integral_delta_rad": maximum_frozen_integral_delta_rad,
        "recovery_decision_count": recovery_decisions,
        "first_recovered_step": recovered[0]["step"] if recovered else None,
        "maximum_controller_source_delta_rad": maximum_source_delta_rad,
        "first_realization_mismatch": first_realization_mismatch,
        "first_hold_mismatch": first_hold_mismatch,
        "controlled_joint_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "controlled_joint_final_error_rad": abs(errors[-1]),
    }


def first_stale_age_violation(
    metrics: dict[str, Any],
    observations: list[dict[str, Any]],
    requirements: dict[str, dict[str, Any]],
) -> dict[str, Any] | None:
    candidates = []
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
                "controller_observation_sequence": age_frame[
                    "controller_observation_sequence"
                ],
            }
        )
    final_frame = observations[-1]
    for requirement_id, metric_key in (
        (
            "controller.sensor_stale.maximum_controlled_joint_rmse_rad",
            "controlled_joint_rmse_rad",
        ),
        (
            "controller.sensor_stale.maximum_controlled_joint_final_error_rad",
            "controlled_joint_final_error_rad",
        ),
    ):
        requirement = requirements[requirement_id]
        if metrics[metric_key] > requirement["maximum"]:
            candidates.append(
                {
                    "requirement_id": requirement["id"],
                    "step": final_frame["step"],
                    "sim_time_ticks": final_frame["sim_time_ticks"],
                    "observed": metrics[metric_key],
                    "maximum": requirement["maximum"],
                    "unit": requirement["unit"],
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
    quantization = quantization_metrics(controller, observations, joint_index)
    saturation = saturation_metrics(controller, observations, joint_index)
    stuck_value = stuck_value_metrics(controller, observations, joint_index)
    availability = availability_metrics(controller, observations)
    recovery = recovery_metrics(controller, observations, joint_index)
    rearm = rearm_metrics(controller, observations, joint_index)
    if recovery is not None or rearm is not None:
        availability = None
    latency = latency_metrics(
        controller, observations, joint_index, trace["fixed_delta_ticks"]
    )
    jitter = jitter_metrics(
        controller, observations, joint_index, trace["fixed_delta_ticks"]
    )
    stale_age = stale_age_metrics(
        controller, observations, joint_index, trace["fixed_delta_ticks"]
    )
    if metrics["first_realization_mismatch"] is not None:
        raise ValueError(f"{backend_id} robustness disturbance realization drifted")
    if stuck_value is not None:
        if (
            not stuck_value["status_realization_matches"]
            or stuck_value["first_realization_mismatch"] is not None
            or stuck_value["first_policy_mismatch"] is not None
            or stuck_value["consumed_stuck_decision_count"]
            != stuck_value["consecutive_stuck_frames"]
            or stuck_value["rejected_decision_count"]
            != stuck_value["consecutive_stuck_frames"]
            or (
                stuck_value["consecutive_stuck_frames"] > 0
                and stuck_value["recovery_decision_count"] == 0
            )
        ):
            raise ValueError(f"{backend_id} measurement-stuck-value realization drifted")
        checks = [
            report_module.check(
                requirements[
                    "controller.sensor.maximum_consecutive_stuck_value_frames"
                ],
                stuck_value["consecutive_stuck_frames"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor.maximum_stuck_value_realization_delta_rad"
                ],
                stuck_value["maximum_realization_delta_rad"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_fail_safe_target_delta_rad"],
                stuck_value["maximum_fail_safe_target_delta_rad"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_recovery_decisions"],
                stuck_value["recovery_decision_count"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_stuck.maximum_controlled_joint_rmse_rad"
                ],
                stuck_value["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_stuck.maximum_controlled_joint_final_error_rad"
                ],
                stuck_value["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_stuck_value_violation(
            stuck_value, observations, requirements
        )
    elif saturation is not None:
        if saturation["first_realization_mismatch"] is not None:
            raise ValueError(f"{backend_id} measurement-saturation realization drifted")
        checks = [
            report_module.check(
                requirements[
                    "controller.sensor.minimum_position_saturation_limit_abs_rad"
                ],
                saturation["saturation_limit_abs_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor.maximum_saturation_realization_delta_rad"
                ],
                saturation["maximum_realization_delta_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_saturation.maximum_controlled_joint_rmse_rad"
                ],
                saturation["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_saturation.maximum_controlled_joint_final_error_rad"
                ],
                saturation["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_saturation_violation(
            saturation, observations, requirements
        )
    elif quantization is not None:
        if quantization["first_realization_mismatch"] is not None:
            raise ValueError(f"{backend_id} measurement-quantization realization drifted")
        checks = [
            report_module.check(
                requirements["controller.sensor.maximum_position_quantization_step_rad"],
                quantization["quantization_step_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor.maximum_quantization_realization_delta_rad"
                ],
                quantization["maximum_realization_delta_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_quantization.maximum_controlled_joint_rmse_rad"
                ],
                quantization["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_quantization.maximum_controlled_joint_final_error_rad"
                ],
                quantization["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_quantization_violation(
            quantization, observations, requirements
        )
    elif rearm is not None:
        if (
            not rearm["publication_realization_matches"]
            or rearm["first_controller_source_mismatch"] is not None
            or rearm["first_hold_mismatch"] is not None
            or rearm["recovery_hold_rejected_decision_count"] != 0
        ):
            raise ValueError(f"{backend_id} repeated-dropout re-arm realization drifted")
        checks = [
            report_module.check(
                requirements["controller.sensor.minimum_interburst_fresh_frames"],
                rearm["interburst_fresh_frames"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_observation_age_ticks"],
                rearm["maximum_controller_observation_age_ticks"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_fail_safe_target_delta_rad"],
                rearm["maximum_fail_safe_target_delta_rad"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_recovery_decisions"],
                rearm["recovery_decision_count"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_rearm.maximum_controlled_joint_rmse_rad"
                ],
                rearm["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_rearm.maximum_controlled_joint_final_error_rad"
                ],
                rearm["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_rearm_violation(rearm, observations, requirements)
    elif recovery is not None:
        if (
            not recovery["publication_realization_matches"]
            or recovery["first_controller_source_mismatch"] is not None
            or recovery["first_hold_mismatch"] is not None
            or recovery["stale_rejected_decision_count"] != 1
            or recovery["recovery_hold_rejected_decision_count"]
            != recovery["additional_recovery_hold_decisions"]
        ):
            raise ValueError(f"{backend_id} dropout-recovery realization drifted")
        checks = [
            report_module.check(
                requirements["controller.sensor.maximum_fail_safe_target_delta_rad"],
                recovery["maximum_fail_safe_target_delta_rad"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_recovery_decisions"],
                recovery["recovery_decision_count"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_recovery.maximum_controlled_joint_rmse_rad"
                ],
                recovery["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_recovery.maximum_controlled_joint_final_error_rad"
                ],
                recovery["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_recovery_violation(
            recovery, observations, requirements
        )
    elif stale_age is not None:
        if (
            stale_age["first_realization_mismatch"] is not None
            or stale_age["first_hold_mismatch"] is not None
        ):
            raise ValueError(f"{backend_id} stale-age realization drifted")
        checks = [
            report_module.check(
                requirements["controller.sensor.maximum_observation_age_ticks"],
                stale_age["maximum_controller_observation_age_ticks"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_fail_safe_target_delta_rad"],
                stale_age["maximum_fail_safe_target_delta_rad"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_recovery_decisions"],
                stale_age["recovery_decision_count"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_stale.maximum_controlled_joint_rmse_rad"
                ],
                stale_age["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_stale.maximum_controlled_joint_final_error_rad"
                ],
                stale_age["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_stale_age_violation(
            stale_age, observations, requirements
        )
    elif jitter is not None:
        if (
            jitter["first_realization_mismatch"] is not None
            or jitter["first_hold_mismatch"] is not None
        ):
            raise ValueError(f"{backend_id} measurement-jitter realization drifted")
        checks = [
            report_module.check(
                requirements[
                    "controller.sensor.maximum_controller_ingress_jitter_frames"
                ],
                jitter["maximum_realized_jitter_frames"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_observation_age_ticks"],
                jitter["maximum_controller_observation_age_ticks"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_fail_safe_target_delta_rad"],
                jitter["maximum_fail_safe_target_delta_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_jitter.maximum_controlled_joint_rmse_rad"
                ],
                jitter["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_jitter.maximum_controlled_joint_final_error_rad"
                ],
                jitter["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_jitter_violation(jitter, observations, requirements)
    elif latency is not None:
        if (
            latency["first_realization_mismatch"] is not None
            or latency["first_hold_mismatch"] is not None
        ):
            raise ValueError(f"{backend_id} measurement-latency realization drifted")
        checks = [
            report_module.check(
                requirements[
                    "controller.sensor.maximum_controller_ingress_delay_frames"
                ],
                latency["controller_ingress_delay_frames"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_observation_age_ticks"],
                latency["maximum_controller_observation_age_ticks"],
            ),
            report_module.check(
                requirements["controller.sensor.maximum_fail_safe_target_delta_rad"],
                latency["maximum_fail_safe_target_delta_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_latency.maximum_controlled_joint_rmse_rad"
                ],
                latency["controlled_joint_rmse_rad"],
            ),
            report_module.check(
                requirements[
                    "controller.sensor_latency.maximum_controlled_joint_final_error_rad"
                ],
                latency["controlled_joint_final_error_rad"],
            ),
        ]
        first_violation = first_latency_violation(latency, observations, requirements)
    elif availability is not None:
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
        elif delay_contract.get("kind") == "actuator_command_deadband_pulse_v1":
            deadband_requirement = requirements[
                "controller.actuator.maximum_command_deadband_rad"
            ]
            checks.append(
                report_module.check(deadband_requirement, delay_contract["deadband_rad"])
            )
            deadband_violation = command_deadband_violation(
                controller, observations, deadband_requirement
            )
            candidates = [
                candidate
                for candidate in (performance_violation, deadband_violation)
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
        "measurement_latency": latency,
        "measurement_jitter": jitter,
        "measurement_stale_age": stale_age,
        "measurement_recovery": recovery,
        "measurement_rearm": rearm,
        "measurement_quantization": quantization,
        "measurement_saturation": saturation,
        "measurement_stuck_value": stuck_value,
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
            "actuator_command_deadband",
            "joint_position_measurement_bias",
            "joint_feedback_publication_dropout",
            "joint_feedback_controller_ingress_latency",
            "joint_feedback_controller_ingress_jitter",
            "joint_feedback_controller_stale_age",
            "joint_feedback_dropout_recovery",
            "joint_feedback_repeated_dropout_rearm",
            "joint_position_measurement_quantization",
            "joint_position_measurement_saturation",
            "joint_position_stuck_value",
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
        "actuator_command_deadband": "openarm-command-deadband-robustness-report",
        "joint_position_measurement_bias": "openarm-sensor-bias-robustness-report",
        "joint_feedback_publication_dropout": "openarm-sensor-dropout-robustness-report",
        "joint_feedback_controller_ingress_latency": "openarm-sensor-latency-robustness-report",
        "joint_feedback_controller_ingress_jitter": "openarm-sensor-jitter-robustness-report",
        "joint_feedback_controller_stale_age": "openarm-sensor-stale-age-robustness-report",
        "joint_feedback_dropout_recovery": "openarm-sensor-recovery-robustness-report",
        "joint_feedback_repeated_dropout_rearm": "openarm-sensor-rearm-robustness-report",
        "joint_position_measurement_quantization": "openarm-sensor-quantization-robustness-report",
        "joint_position_measurement_saturation": "openarm-sensor-saturation-robustness-report",
        "joint_position_stuck_value": "openarm-sensor-stuck-robustness-report",
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
const r=__REPORT__,f=x=>x==null?'n/a':Number(x).toFixed(6),colors={rne_rapier:'#1261a0',mujoco_native:'#c2410c',gazebo_sim:'#15803d'},availability=r.dimension_id==='joint_feedback_publication_dropout',latency=r.dimension_id==='joint_feedback_controller_ingress_latency',jitter=r.dimension_id==='joint_feedback_controller_ingress_jitter',stale=r.dimension_id==='joint_feedback_controller_stale_age',recovery=r.dimension_id==='joint_feedback_dropout_recovery',rearm=r.dimension_id==='joint_feedback_repeated_dropout_rearm',quantization=r.dimension_id==='joint_position_measurement_quantization',saturation=r.dimension_id==='joint_position_measurement_saturation',stuck=r.dimension_id==='joint_position_stuck_value',timingDimension=latency||jitter||stale||recovery||rearm||quantization||saturation||stuck,timing=q=>latency?q.measurement_latency:jitter?q.measurement_jitter:stale?q.measurement_stale_age:recovery?q.measurement_recovery:rearm?q.measurement_rearm:quantization?q.measurement_quantization:saturation?q.measurement_saturation:q.measurement_stuck_value,timingLabel=latency?'ingress delay':jitter?'peak jitter':stale?'additional stale age':recovery?'additional recovery hold':rearm?'interburst fresh frames':quantization?'quantization step':saturation?'saturation limit':'stuck frames';document.querySelector('#summary').innerHTML=`<section class=card><p>Status: <b class=${r.status}>${r.status}</b></p><p>Last passing value: ${f(r.boundary.last_passing_value)} ${r.dimension.unit}</p><p>First failing value: ${f(r.boundary.first_failing_value)} ${r.dimension.unit}</p><p>Portable first failure: <code>${r.boundary.portable_first_failed_requirement}</code></p><p>First violation: step ${r.first_failure.step}, ${f(r.first_failure.observed)} ${r.first_failure.unit}</p></section>`;
document.querySelector('#sweep').innerHTML=availability?`<table><tr><th>case</th><th>dropped frames</th><th>max age ticks</th><th>rejections</th><th>recovery decisions</th><th>status</th></tr>${r.primary_backend_results.map(q=>`<tr><td>${q.case_id}</td><td>${q.dimension_value}</td><td>${q.measurement_availability.maximum_controller_observation_age_ticks}</td><td>${q.measurement_availability.rejected_decision_count}</td><td>${q.measurement_availability.recovery_decision_count}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`:timingDimension?`<table><tr><th>case</th><th>${timingLabel}</th><th>max age ticks</th><th>joint RMSE rad</th><th>final error rad</th><th>rejections</th><th>recovery decisions</th><th>status</th></tr>${r.primary_backend_results.map(q=>`<tr><td>${q.case_id}</td><td>${q.dimension_value}</td><td>${timing(q).maximum_controller_observation_age_ticks}</td><td>${f(timing(q).controlled_joint_rmse_rad)}</td><td>${f(timing(q).controlled_joint_final_error_rad)}</td><td>${timing(q).rejected_decision_count}</td><td>${timing(q).recovery_decision_count}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`:`<table><tr><th>case</th><th>value</th><th>peak rad</th><th>recovery s</th><th>IAE rad·s</th><th>status</th></tr>${r.primary_backend_results.map(q=>`<tr><td>${q.case_id}</td><td>${f(q.dimension_value)}</td><td>${f(q.metrics.peak_tracking_error_rad)}</td><td>${f(q.metrics.recovery_time_s)}</td><td>${f(q.metrics.iae_rad_s)}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`;
document.querySelector('h1').textContent=`OpenArm ${r.dimension_id.replaceAll('_',' ')} robustness envelope`;document.querySelector('#boundary').innerHTML=r.cross_backend_boundary_results.map(q=>availability?`<section class=card><h3>${q.case_id} / ${q.backend_id}</h3><p class=${q.status}>${q.status}</p><p>max age ${q.measurement_availability.maximum_controller_observation_age_ticks} ticks</p><p>rejections ${q.measurement_availability.rejected_decision_count}</p><p>recovery ${q.measurement_availability.recovery_decision_count} decision(s)</p></section>`:timingDimension?`<section class=card><h3>${q.case_id} / ${q.backend_id}</h3><p class=${q.status}>${q.status}</p><p>max age ${timing(q).maximum_controller_observation_age_ticks} ticks</p><p>RMSE ${f(timing(q).controlled_joint_rmse_rad)} rad</p><p>final error ${f(timing(q).controlled_joint_final_error_rad)} rad</p><p>rejections ${timing(q).rejected_decision_count}</p></section>`:`<section class=card><h3>${q.case_id} / ${q.backend_id}</h3><p class=${q.status}>${q.status}</p><p>peak ${f(q.metrics.peak_tracking_error_rad)} rad</p><p>recovery ${f(q.metrics.recovery_time_s)} s</p><p>IAE ${f(q.metrics.iae_rad_s)} rad·s</p></section>`).join('');function plot(caseId){const rows=r.cross_backend_boundary_results.filter(q=>q.case_id===caseId),c=document.createElement('canvas');c.width=1160;c.height=260;const x=c.getContext('2d'),n=rows[0].plot.reference_rad.length,start=(r.dimension.start_step??r.dimension.start_controller_step??r.dimension.start_capture_sequence??1)-1,end=r.dimension.end_step??r.dimension.end_controller_step??r.dimension.end_capture_sequence??(r.dimension.start_capture_sequence?r.dimension.start_capture_sequence+r.boundary.first_failing_value:n);x.fillStyle='#ef444422';x.fillRect(start/(n-1)*c.width,0,(end-start)/(n-1)*c.width,c.height);function line(v,color,w=1.4){x.beginPath();for(let i=0;i<n;i++){const px=i/(n-1)*c.width,py=c.height-(v[i]+.16)/.34*c.height;i?x.lineTo(px,py):x.moveTo(px,py)}x.strokeStyle=color;x.lineWidth=w;x.stroke()}line(rows[0].plot.reference_rad,'#111',1.8);rows.forEach(q=>line(q.plot.position_rad,colors[q.backend_id]));const s=document.createElement('section');s.innerHTML=`<h3>${caseId}</h3>`;s.appendChild(c);return s}const plots=document.querySelector('#plots');plots.appendChild(plot(r.boundary.last_passing_case_id));plots.appendChild(plot(r.boundary.first_failing_case_id));
</script></main></body></html>'''.replace("__REPORT__", payload)
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm robustness report failed: {error}")
        raise SystemExit(2)

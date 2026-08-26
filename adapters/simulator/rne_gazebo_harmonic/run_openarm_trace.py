#!/usr/bin/env python3
"""Runs one compiled OpenArm controller trace through the Gazebo adapter."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import selectors
import subprocess
import sys
from typing import Any


HOST_KIND = "rne_simulator_host_frame"
ADAPTER_KIND = "rne_simulator_adapter_frame"
FIXED_DELTA_TICKS = 16_666_667
RESPONSE_TIMEOUT_S = 30.0


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--actions", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--adapter", type=Path, default=root / "rne_gazebo_harmonic_adapter.py"
    )
    parser.add_argument("--runtime-manifest", type=Path, default=root / "runtime.json")
    parser.add_argument(
        "--task", type=Path, default=root / "openarm_right_joint_tracking.task.json"
    )
    parser.add_argument(
        "--controller", type=Path, default=root / "openarm_right_pose_cycle.controller.json"
    )
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as source:
        value = json.load(source)
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def runtime_artifact_path(runtime_path: Path, role: str) -> Path:
    runtime = load_json(runtime_path)
    matches = [entry for entry in runtime["artifacts"] if entry["role"] == role]
    if len(matches) != 1:
        raise ValueError(f"runtime manifest must contain one {role} artifact")
    path = runtime_path.parent / matches[0]["file"]
    if path.stat().st_size != matches[0]["size_bytes"] or sha256(path) != matches[0]["sha256"]:
        raise ValueError(f"runtime artifact {path.name} differs from manifest")
    return path


class AdapterProcess:
    def __init__(
        self,
        adapter: Path,
        runtime: Path,
        task: Path,
        actuation_diagnostics_output: Path | None = None,
    ) -> None:
        command = [
            sys.executable,
            str(adapter),
            "--runtime-manifest",
            str(runtime),
            "--task",
            str(task),
        ]
        if actuation_diagnostics_output is not None:
            command.extend(
                ["--actuation-diagnostics-output", str(actuation_diagnostics_output)]
            )
        self.process = subprocess.Popen(
            command,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        if self.process.stdin is None or self.process.stdout is None:
            raise RuntimeError("adapter process did not expose pipes")
        self.selector = selectors.DefaultSelector()
        self.selector.register(self.process.stdout, selectors.EVENT_READ)

    def exchange(self, frame: dict[str, Any]) -> dict[str, Any]:
        assert self.process.stdin is not None
        assert self.process.stdout is not None
        self.process.stdin.write(json.dumps(frame, separators=(",", ":")) + "\n")
        self.process.stdin.flush()
        if not self.selector.select(RESPONSE_TIMEOUT_S):
            raise TimeoutError(
                f"Gazebo adapter response exceeded {RESPONSE_TIMEOUT_S:g} seconds"
            )
        line = self.process.stdout.readline()
        if not line:
            raise RuntimeError(f"Gazebo adapter exited with {self.process.poll()}")
        response = json.loads(line)
        if (
            response.get("kind") != ADAPTER_KIND
            or response.get("schema_version") != 1
            or response.get("session_id") != frame["session_id"]
            or response.get("request_sequence") != frame["sequence"]
        ):
            raise RuntimeError("Gazebo adapter response envelope drifted")
        return response

    def finish(self) -> None:
        self.selector.close()
        status = self.process.wait(timeout=RESPONSE_TIMEOUT_S)
        if status != 0:
            raise RuntimeError(f"Gazebo adapter exited with status {status}")


def host_frame(session: str, sequence: int, payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "kind": HOST_KIND,
        "schema_version": 1,
        "session_id": session,
        "sequence": sequence,
        "payload": payload,
    }


def open_and_reset(
    process: AdapterProcess,
    session: str,
    task: dict[str, Any],
    task_sha256: str,
    observation_width: int,
    action_width: int,
) -> None:
    ready = process.exchange(
        host_frame(
            session,
            1,
            {
                "type": "open",
                "task_id": task["task_id"],
                "task_sha256": task_sha256,
                "observation_width": observation_width,
                "action_width": action_width,
                "fixed_delta_ticks": FIXED_DELTA_TICKS,
            },
        )
    )["payload"]
    if ready.get("type") != "ready" or ready.get("adapter_id") != "rne.gazebo_harmonic.openarm_right.v1":
        raise RuntimeError("Gazebo adapter did not accept the exact task identity")
    reset = process.exchange(host_frame(session, 2, {"type": "reset", "seed": 20260824}))[
        "payload"
    ]
    if reset.get("type") != "reset_complete" or reset.get("seed") != 20260824:
        raise RuntimeError("Gazebo adapter reset contract drifted")


def controller_decision(
    controller: dict[str, Any],
    reference: list[float],
    integral_correction: list[float],
    previous_observation_position: list[float | None],
    previous_input_target: list[float | None],
    previous_previous_input_target: list[float | None],
    observation: dict[str, Any] | None,
    consumed_at_ticks: int,
) -> dict[str, Any]:
    """Evaluate the artifact-defined feedback law without simulator-private state."""
    if "observation_contract" not in controller and "feedback_law" not in controller:
        return {
            "target": reference.copy(),
            "correction": [0.0] * len(reference),
            "integral_correction": integral_correction.copy(),
            "controller_observation_position_rad": [],
            "joint_measurement_bias_rad": [0.0] * len(reference),
            "observation_sequence": None,
            "observation_age_ticks": None,
            "bootstrap": False,
        }
    if observation is None:
        law = controller["feedback_law"]
        if law["kind"] == "joint_position_state_feedback_integral_v1":
            index = controller["action_joint_order"].index(law["controlled_joint"])
            previous_previous_input_target[index] = previous_input_target[index]
            previous_input_target[index] = reference[index]
        return {
            "target": reference.copy(),
            "correction": [0.0] * len(reference),
            "integral_correction": integral_correction.copy(),
            "controller_observation_position_rad": [],
            "joint_measurement_bias_rad": [0.0] * len(reference),
            "observation_sequence": None,
            "observation_age_ticks": None,
            "bootstrap": True,
        }
    contract = controller["observation_contract"]
    if observation["sensor_status"] != contract["required_status"]:
        raise RuntimeError("OpenArm controller rejected non-nominal joint feedback")
    age_ticks = consumed_at_ticks - observation["sim_time_ticks"]
    if age_ticks < 0 or age_ticks > contract["maximum_age_ticks"]:
        raise RuntimeError("OpenArm controller rejected stale or future joint feedback")
    law = controller["feedback_law"]
    sample_period_s = contract["sample_period_ticks"] / 1_000_000_000.0
    controller_positions, measurement_bias = apply_measurement_bias(
        controller, observation, consumed_at_ticks
    )
    if law["kind"] == "joint_position_state_feedback_integral_v1":
        index = controller["action_joint_order"].index(law["controlled_joint"])
        position = controller_positions[index]
        previous_position = previous_observation_position[index]
        if previous_position is None:
            previous_position = position
        operating_position = law["operating_point_position_rad"]
        operating_input = law["operating_point_input_rad"]
        previous_input = previous_input_target[index]
        previous_previous_input = previous_previous_input_target[index]
        if previous_input is None:
            previous_input = operating_input
        if previous_previous_input is None:
            previous_previous_input = operating_input
        _, a1, a2, b1, b2 = law["identified_plant"]["arx_coefficients"]
        predicted_position_error = (
            a1 * (position - operating_position)
            + a2 * (previous_position - operating_position)
            + b1 * (previous_input - operating_input)
            + b2 * (previous_previous_input - operating_input)
        )
        integral_gain = law["integral_state_feedback_gain_s_inv"]
        maximum_integral = law["maximum_state_integral_correction_rad"]
        integral_correction[index] = max(
            -maximum_integral,
            min(
                maximum_integral,
                integral_correction[index]
                + integral_gain * (reference[index] - position) * sample_period_s,
            ),
        )
        state = [
            operating_position + predicted_position_error - reference[index],
            position - reference[index],
            previous_input - reference[index],
        ]
        raw_target = (
            reference[index]
            - sum(gain * value for gain, value in zip(law["state_feedback_gain"], state))
            + integral_correction[index]
        )
        maximum_correction = law["maximum_state_feedback_correction_rad"]
        correction = [0.0] * len(reference)
        correction[index] = max(
            -maximum_correction,
            min(maximum_correction, raw_target - reference[index]),
        )
        target = reference.copy()
        target[index] = max(
            law["minimum_controlled_target_rad"],
            min(
                law["maximum_controlled_target_rad"],
                reference[index] + correction[index],
            ),
        )
        previous_observation_position[index] = position
        previous_previous_input_target[index] = previous_input
        previous_input_target[index] = target[index]
        return {
            "target": target,
            "correction": correction,
            "integral_correction": integral_correction.copy(),
            "controller_observation_position_rad": controller_positions,
            "joint_measurement_bias_rad": measurement_bias,
            "observation_sequence": observation["step"],
            "observation_age_ticks": age_ticks,
            "bootstrap": False,
        }
    if law["kind"] != "joint_position_reference_pid_v1":
        raise RuntimeError("unsupported OpenArm feedback law")
    for index, (desired, position, gain, maximum) in enumerate(
        zip(
            reference,
            controller_positions,
            law["integral_error_gain_s_inv"],
            law["maximum_integral_correction_rad"],
        )
    ):
        integral_correction[index] = max(
            -maximum,
            min(
                maximum,
                integral_correction[index]
                + gain * (desired - position) * sample_period_s,
            ),
        )
    correction = [
        max(
            -limit,
            min(limit, gain * (desired - position) - damping * velocity + integral),
        )
        for desired, position, velocity, gain, damping, integral, limit in zip(
            reference,
            controller_positions,
            observation["joint_velocity_rad_s"],
            law["position_error_gain"],
            law["velocity_damping_s"],
            integral_correction,
            law["maximum_correction_rad"],
        )
    ]
    target = [
        max(minimum, min(maximum, desired + delta))
        for desired, delta, minimum, maximum in zip(
            reference,
            correction,
            law["minimum_target_rad"],
            law["maximum_target_rad"],
        )
    ]
    return {
        "target": target,
        "correction": correction,
        "integral_correction": integral_correction.copy(),
        "controller_observation_position_rad": controller_positions,
        "joint_measurement_bias_rad": measurement_bias,
        "observation_sequence": observation["step"],
        "observation_age_ticks": age_ticks,
        "bootstrap": False,
    }


def dropout_contract(controller: dict[str, Any]) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if isinstance(contract, dict) and contract.get("kind") == "joint_feedback_publication_dropout_burst_v1":
        return contract
    return None


def latency_contract(controller: dict[str, Any]) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if (
        isinstance(contract, dict)
        and contract.get("kind") == "joint_feedback_controller_ingress_delay_v1"
    ):
        return contract
    return None


def jitter_contract(controller: dict[str, Any]) -> dict[str, Any] | None:
    contract = controller.get("measurement_fault_contract")
    if (
        isinstance(contract, dict)
        and contract.get("kind")
        == "joint_feedback_controller_ingress_jitter_pulse_v1"
    ):
        return contract
    return None


def availability_contract(controller: dict[str, Any]) -> dict[str, Any] | None:
    return dropout_contract(controller) or latency_contract(controller) or jitter_contract(controller)


def controller_ingress_delay_frames(
    controller: dict[str, Any], capture_sequence: int | None = None
) -> int:
    contract = latency_contract(controller)
    if contract is not None:
        return contract["delay_frames"]
    contract = jitter_contract(controller)
    if contract is None or capture_sequence is None:
        return 0
    maximum = contract["maximum_jitter_frames"]
    if (
        maximum == 0
        or capture_sequence < contract["start_capture_sequence"]
        or capture_sequence > contract["end_capture_sequence"]
    ):
        return 0
    phase = (capture_sequence - contract["start_capture_sequence"]) % (maximum + 1)
    return maximum if phase < maximum else 0


def sensor_sample_published(controller: dict[str, Any], sequence: int) -> bool:
    contract = dropout_contract(controller)
    if contract is None:
        return True
    start = contract["start_capture_sequence"]
    return not start <= sequence < start + contract["consecutive_dropped_frames"]


def validate_measurement_fault(controller: dict[str, Any], action_count: int) -> None:
    contract = controller.get("measurement_fault_contract")
    if contract is None or contract.get("kind") == "additive_joint_position_bias_pulse_v1":
        return
    if contract.get("kind") == "joint_feedback_controller_ingress_delay_v1":
        expected_keys = {
            "kind",
            "classification",
            "delay_frames",
            "controller_visibility",
            "application_order",
        }
        delay = contract.get("delay_frames")
        if (
            set(contract) != expected_keys
            or contract.get("classification") != "measurement_transport_latency"
            or not isinstance(delay, int)
            or isinstance(delay, bool)
            or not 0 <= delay <= action_count
            or contract.get("controller_visibility")
            != "delayed_nominal_publication_with_original_capture_timestamp"
            or contract.get("application_order")
            != "after_typed_feedback_availability_before_controller_ingress"
        ):
            raise ValueError("invalid OpenArm controller-ingress latency contract")
        return
    if contract.get("kind") == "joint_feedback_controller_ingress_jitter_pulse_v1":
        expected_keys = {
            "kind",
            "classification",
            "maximum_jitter_frames",
            "start_capture_sequence",
            "end_capture_sequence",
            "schedule",
            "controller_visibility",
            "application_order",
        }
        maximum = contract.get("maximum_jitter_frames")
        start = contract.get("start_capture_sequence")
        end = contract.get("end_capture_sequence")
        if (
            set(contract) != expected_keys
            or contract.get("classification") != "measurement_transport_jitter"
            or not isinstance(maximum, int)
            or isinstance(maximum, bool)
            or not 0 <= maximum <= action_count
            or not isinstance(start, int)
            or isinstance(start, bool)
            or not isinstance(end, int)
            or isinstance(end, bool)
            or not 1 <= start <= end <= action_count
            or contract.get("schedule")
            != "maximum_delay_for_n_frames_then_nominal_v1"
            or contract.get("controller_visibility")
            != "jittered_nominal_publication_with_original_capture_timestamp"
            or contract.get("application_order")
            != "after_typed_feedback_availability_before_controller_ingress"
        ):
            raise ValueError("invalid OpenArm controller-ingress jitter contract")
        return
    expected_keys = {
        "kind",
        "classification",
        "start_capture_sequence",
        "consecutive_dropped_frames",
        "controller_visibility",
        "application_order",
    }
    start = contract.get("start_capture_sequence")
    count = contract.get("consecutive_dropped_frames")
    if (
        set(contract) != expected_keys
        or contract.get("kind") != "joint_feedback_publication_dropout_burst_v1"
        or contract.get("classification") != "measurement_unavailability"
        or not isinstance(start, int)
        or not isinstance(count, int)
        or not 1 <= start <= action_count
        or not 0 <= count
        or start + count > action_count + 1
        or contract.get("controller_visibility") != "missing_publication_only"
        or contract.get("application_order")
        != "after_typed_sensor_capture_before_controller_ingress"
    ):
        raise ValueError("invalid OpenArm measurement-dropout contract")


def bounded_controller_decision(
    controller: dict[str, Any],
    reference: list[float],
    integral_correction: list[float],
    previous_observation_position: list[float | None],
    previous_input_target: list[float | None],
    previous_previous_input_target: list[float | None],
    observation: dict[str, Any] | None,
    consumed_at_ticks: int,
    last_accepted_target: list[float],
    recovering_from_rejection: bool,
) -> dict[str, Any]:
    contract = controller.get("observation_contract")
    if observation is not None and availability_contract(controller) is not None:
        age_ticks = consumed_at_ticks - observation["sim_time_ticks"]
        if age_ticks > contract["maximum_age_ticks"]:
            if len(last_accepted_target) != len(reference):
                raise RuntimeError("fail-safe hold target width mismatch")
            return {
                "target": last_accepted_target.copy(),
                "correction": [
                    target - desired for target, desired in zip(last_accepted_target, reference)
                ],
                "integral_correction": integral_correction.copy(),
                "controller_observation_position_rad": observation["joint_position_rad"].copy(),
                "joint_measurement_bias_rad": [0.0] * len(reference),
                "observation_sequence": observation["step"],
                "observation_age_ticks": age_ticks,
                "bootstrap": False,
                "controller_rejected": True,
                "controller_rejection_reason": "maximum_observation_age_ticks",
                "fail_safe_hold_active": True,
                "controller_state_frozen": True,
                "controller_recovered": False,
            }
    decision = controller_decision(
        controller,
        reference,
        integral_correction,
        previous_observation_position,
        previous_input_target,
        previous_previous_input_target,
        observation,
        consumed_at_ticks,
    )
    decision.update(
        {
            "controller_rejected": False,
            "controller_rejection_reason": None,
            "fail_safe_hold_active": False,
            "controller_state_frozen": False,
            "controller_recovered": recovering_from_rejection and not decision["bootstrap"],
        }
    )
    return decision


def apply_measurement_bias(
    controller: dict[str, Any],
    observation: dict[str, Any],
    consumed_at_ticks: int,
) -> tuple[list[float], list[float]]:
    positions = observation["joint_position_rad"].copy()
    bias = [0.0] * len(positions)
    contract = controller.get("measurement_fault_contract")
    if contract is None:
        return positions, bias
    if contract.get("kind") in {
        "joint_feedback_publication_dropout_burst_v1",
        "joint_feedback_controller_ingress_delay_v1",
        "joint_feedback_controller_ingress_jitter_pulse_v1",
    }:
        return positions, bias
    expected_keys = {
        "kind",
        "classification",
        "joint",
        "start_controller_step",
        "end_controller_step",
        "offset_rad",
        "sensor_status",
        "controller_visibility",
        "application_order",
    }
    sample_period_ticks = controller["observation_contract"]["sample_period_ticks"]
    if consumed_at_ticks % sample_period_ticks != 0:
        raise RuntimeError("measurement-bias consumption time is off the control grid")
    controller_step = consumed_at_ticks // sample_period_ticks + 1
    if (
        set(contract) != expected_keys
        or contract["kind"] != "additive_joint_position_bias_pulse_v1"
        or contract["classification"] != "measurement_error"
        or contract["joint"] not in controller["action_joint_order"]
        or not 1 <= contract["start_controller_step"] <= contract["end_controller_step"]
        or not math.isfinite(contract["offset_rad"])
        or contract["sensor_status"] != "nominal"
        or contract["controller_visibility"] != "biased_position_as_nominal"
        or contract["application_order"]
        != "after_typed_feedback_availability_before_controller_law"
    ):
        raise RuntimeError("invalid OpenArm measurement-bias contract")
    if contract["start_controller_step"] <= controller_step <= contract["end_controller_step"]:
        index = controller["action_joint_order"].index(contract["joint"])
        bias[index] = contract["offset_rad"]
        positions[index] += bias[index]
    if not all(math.isfinite(value) for value in positions):
        raise RuntimeError("measurement bias produced a non-finite observation")
    return positions, bias


def apply_actuator_disturbance(
    controller: dict[str, Any],
    step: int,
    controller_target: list[float],
    controller_target_history: list[list[float]],
    applied_target_history: list[list[float]],
) -> tuple[list[float], list[float]]:
    """Apply a declared plant-input disturbance after the controller boundary."""
    disturbance = [0.0] * len(controller_target)
    applied = controller_target.copy()
    contract = controller.get("disturbance_contract")
    if contract is None:
        return applied, disturbance
    common_keys = {
        "kind",
        "classification",
        "joint",
        "start_step",
        "end_step",
        "controller_visibility",
        "application_order",
    }
    if (
        contract["joint"] not in controller["action_joint_order"]
        or not 1 <= contract["start_step"] <= contract["end_step"]
        or contract["controller_visibility"]
        != "unobserved_except_through_typed_joint_feedback"
        or contract["application_order"]
        != "after_controller_limits_before_backend_actuation"
    ):
        raise RuntimeError("invalid OpenArm actuator disturbance contract")
    kind = contract["kind"]
    if kind == "additive_actuator_target_bias_pulse_v1":
        valid_specific = (
            set(contract) == common_keys | {"offset_rad"}
            and contract["classification"] == "actuator_realization_error"
            and isinstance(contract["offset_rad"], (int, float))
            and math.isfinite(contract["offset_rad"])
        )
    elif kind == "actuator_command_transport_delay_pulse_v1":
        valid_specific = (
            set(contract) == common_keys | {"delay_steps"}
            and contract["classification"] == "actuator_transport_delay"
            and isinstance(contract["delay_steps"], int)
            and not isinstance(contract["delay_steps"], bool)
            and contract["delay_steps"] >= 0
            and contract["start_step"] > contract["delay_steps"]
        )
    elif kind == "actuator_command_slew_rate_limit_pulse_v1":
        valid_specific = (
            set(contract) == common_keys | {"maximum_rate_rad_s"}
            and contract["classification"] == "actuator_rate_limit"
            and isinstance(contract["maximum_rate_rad_s"], (int, float))
            and not isinstance(contract["maximum_rate_rad_s"], bool)
            and math.isfinite(contract["maximum_rate_rad_s"])
            and contract["maximum_rate_rad_s"] > 0.0
            and contract["start_step"] > 1
        )
    elif kind == "actuator_command_deadband_pulse_v1":
        valid_specific = (
            set(contract) == common_keys | {"deadband_rad"}
            and contract["classification"] == "actuator_deadband"
            and isinstance(contract["deadband_rad"], (int, float))
            and not isinstance(contract["deadband_rad"], bool)
            and math.isfinite(contract["deadband_rad"])
            and contract["deadband_rad"] >= 0.0
            and contract["start_step"] > 1
        )
    else:
        valid_specific = False
    if not valid_specific:
        raise RuntimeError("invalid OpenArm actuator disturbance contract")
    if contract["start_step"] <= step <= contract["end_step"]:
        index = controller["action_joint_order"].index(contract["joint"])
        if kind == "additive_actuator_target_bias_pulse_v1":
            disturbance[index] = contract["offset_rad"]
            applied[index] += disturbance[index]
        elif kind == "actuator_command_transport_delay_pulse_v1":
            source_step = step - contract["delay_steps"]
            try:
                applied[index] = controller_target_history[source_step - 1][index]
            except IndexError as error:
                raise RuntimeError(
                    "actuator command delay source step is absent from history"
                ) from error
            disturbance[index] = applied[index] - controller_target[index]
        elif kind == "actuator_command_slew_rate_limit_pulse_v1":
            if not applied_target_history:
                raise RuntimeError(
                    "actuator command rate limit has no previous applied target"
                )
            previous = applied_target_history[-1][index]
            maximum_delta_rad = (
                contract["maximum_rate_rad_s"] * FIXED_DELTA_TICKS / 1_000_000_000.0
            )
            applied[index] = min(
                max(controller_target[index], previous - maximum_delta_rad),
                previous + maximum_delta_rad,
            )
            disturbance[index] = applied[index] - controller_target[index]
        else:
            if not applied_target_history:
                raise RuntimeError(
                    "actuator command deadband has no previous applied target"
                )
            previous = applied_target_history[-1][index]
            if abs(controller_target[index] - previous) <= contract["deadband_rad"]:
                applied[index] = previous
            disturbance[index] = applied[index] - controller_target[index]
    if not all(math.isfinite(value) and -3.0 <= value <= 3.0 for value in applied):
        raise RuntimeError("disturbed OpenArm target violates TaskSpec bounds")
    return applied, disturbance


def run_success(
    args: argparse.Namespace,
    task: dict[str, Any],
    task_sha256: str,
    action_artifact: dict[str, Any],
    controller: dict[str, Any],
    session: str,
    actuation_diagnostics_output: Path,
) -> tuple[list[dict[str, Any]], int, dict[str, Any]]:
    process = AdapterProcess(
        args.adapter,
        args.runtime_manifest,
        args.task,
        actuation_diagnostics_output,
    )
    joint_count = len(action_artifact["action_joint_order"])
    open_and_reset(process, session, task, task_sha256, 2 * joint_count, joint_count)
    observations: list[dict[str, Any]] = []
    adapter_config = load_json(runtime_artifact_path(args.runtime_manifest, "adapter_config"))
    previous_positions = [0.0] * joint_count
    final_digest = 0
    integral_correction = [0.0] * joint_count
    previous_observation_position: list[float | None] = [None] * joint_count
    previous_input_target: list[float | None] = [None] * joint_count
    previous_previous_input_target: list[float | None] = [None] * joint_count
    controller_target_history: list[list[float]] = []
    applied_target_history: list[list[float]] = []
    last_accepted_target = action_artifact["actions"][0]["joint_position_target_rad"].copy()
    recovering_from_rejection = False
    for action in action_artifact["actions"]:
        consumed_at_ticks = (action["step"] - 1) * FIXED_DELTA_TICKS
        delayed_observation = next(
            (
                frame
                for frame in reversed(observations)
                if frame["sensor_sample_published"]
                and frame["sim_time_ticks"]
                + FIXED_DELTA_TICKS
                + controller_ingress_delay_frames(controller, frame["step"])
                * FIXED_DELTA_TICKS
                <= consumed_at_ticks
            ),
            None,
        )
        decision = bounded_controller_decision(
            controller,
            action["joint_position_target_rad"],
            integral_correction,
            previous_observation_position,
            previous_input_target,
            previous_previous_input_target,
            delayed_observation,
            consumed_at_ticks,
            last_accepted_target,
            recovering_from_rejection,
        )
        if not decision["controller_rejected"]:
            last_accepted_target = decision["target"].copy()
        recovering_from_rejection = decision["controller_rejected"]
        controller_target_history.append(decision["target"].copy())
        applied_target, disturbance = apply_actuator_disturbance(
            controller,
            action["step"],
            decision["target"],
            controller_target_history,
            applied_target_history,
        )
        applied_target_history.append(applied_target.copy())
        actuator_command_saturated = [
            abs(
                adapter_config["position_gain_s_inv"] * (target - position)
            )
            > adapter_config["maximum_velocity_rad_s"]
            for target, position in zip(applied_target, previous_positions)
        ]
        payload = process.exchange(
            host_frame(
                session,
                action["step"] + 2,
                {
                    "type": "step",
                    "action_sequence": action["action_sequence"],
                    "values": applied_target,
                },
            )
        )["payload"]
        if (
            payload.get("type") != "stepped"
            or payload.get("step") != action["step"]
            or payload.get("sim_time_ticks") != action["sim_time_ticks"]
        ):
            raise RuntimeError(f"Gazebo fixed-step response drifted at step {action['step']}")
        values = payload["values"]
        if len(values) != 2 * joint_count or not all(math.isfinite(value) for value in values):
            raise RuntimeError("Gazebo observation violated the TaskSpec")
        positions = values[:joint_count]
        velocities = values[joint_count:]
        maximum_error = max(
            abs(actual - expected)
            for actual, expected in zip(positions, action["joint_position_target_rad"])
        )
        maximum_actuator_error = max(
            abs(actual - expected)
            for actual, expected in zip(positions, applied_target)
        )
        observations.append(
            {
                "step": action["step"],
                "sim_time_ticks": action["sim_time_ticks"],
                "sensor_status": "nominal",
                "sensor_sample_published": sensor_sample_published(
                    controller, action["step"]
                ),
                "controller_observation_sequence": decision["observation_sequence"],
                "controller_observation_age_ticks": decision[
                    "observation_age_ticks"
                ],
                "controller_bootstrap": decision["bootstrap"],
                "controller_rejected": decision["controller_rejected"],
                "controller_rejection_reason": decision[
                    "controller_rejection_reason"
                ],
                "fail_safe_hold_active": decision["fail_safe_hold_active"],
                "controller_state_frozen": decision["controller_state_frozen"],
                "controller_recovered": decision["controller_recovered"],
                "joint_position_rad": positions,
                "joint_velocity_rad_s": velocities,
                "joint_reference_position_rad": action[
                    "joint_position_target_rad"
                ],
                "joint_controller_observation_position_rad": decision[
                    "controller_observation_position_rad"
                ],
                "joint_measurement_bias_rad": decision["joint_measurement_bias_rad"],
                "measurement_bias_active": any(
                    value != 0.0 for value in decision["joint_measurement_bias_rad"]
                ),
                "joint_controller_target_rad": decision["target"],
                "joint_actuator_disturbance_rad": disturbance,
                "joint_position_target_rad": applied_target,
                "actuator_disturbance_active": any(value != 0.0 for value in disturbance),
                "joint_feedback_correction_rad": decision["correction"],
                "joint_integral_correction_rad": decision["integral_correction"],
                "actuator_command_saturated": actuator_command_saturated,
                "actuator_command_semantics": "gazebo_position_velocity_limit_v1",
                "maximum_actuator_tracking_error_rad": maximum_actuator_error,
                "maximum_tracking_error_rad": maximum_error,
            }
        )
        previous_positions = positions
        final_digest = payload["state_digest"]
    closed = process.exchange(
        host_frame(session, len(action_artifact["actions"]) + 3, {"type": "close"})
    )["payload"]
    if closed.get("type") != "closed":
        raise RuntimeError("Gazebo adapter did not close cleanly")
    process.finish()
    diagnostics = load_json(actuation_diagnostics_output)
    if (
        diagnostics.get("kind") != "rne_gazebo_actuation_diagnostics"
        or diagnostics.get("joint_order") != action_artifact["action_joint_order"]
        or len(diagnostics.get("steps", [])) != len(observations)
    ):
        raise RuntimeError("Gazebo actuation diagnostic sidecar violated its contract")
    for observation, diagnostic in zip(observations, diagnostics["steps"]):
        if diagnostic.get("step") != observation["step"]:
            raise RuntimeError("Gazebo actuation diagnostic step order drifted")
        observation["actuator_realization"] = diagnostic
        observation["actuator_command_saturated"] = [
            count > 0 for count in diagnostic["joint_saturation_substep_count"]
        ]
        observation["actuator_command_semantics"] = (
            "gazebo_pre_update_realized_command_v1"
        )
    return observations, final_digest, diagnostics


def run_intentional_failure(
    args: argparse.Namespace,
    task: dict[str, Any],
    task_sha256: str,
    controller: dict[str, Any],
    action_artifact: dict[str, Any],
    clean_observations: list[dict[str, Any]],
    runtime_manifest_sha256: str,
    adapter_config_sha256: str,
) -> dict[str, Any]:
    injection = controller["intentional_failure"]
    inject_step = injection["inject_at_step"]
    process = AdapterProcess(args.adapter, args.runtime_manifest, args.task)
    session = "rne.openarm.gazebo.intentional-failure.v1"
    joint_count = len(action_artifact["action_joint_order"])
    open_and_reset(process, session, task, task_sha256, 2 * joint_count, joint_count)
    actions = action_artifact["actions"]
    for action, clean in zip(
        actions[: inject_step - 1], clean_observations[: inject_step - 1]
    ):
        payload = process.exchange(
            host_frame(
                session,
                action["step"] + 2,
                {
                    "type": "step",
                    "action_sequence": action["action_sequence"],
                    "values": clean["joint_position_target_rad"],
                },
            )
        )["payload"]
        if payload.get("type") != "stepped":
            raise RuntimeError("Gazebo failed before the injected controller fault")
    injected = actions[inject_step - 1]
    rejected = process.exchange(
        host_frame(
            session,
            inject_step + 2,
            {
                "type": "step",
                "action_sequence": injected["action_sequence"],
                "values": clean_observations[inject_step - 1][
                    "joint_position_target_rad"
                ][:-1],
            },
        )
    )["payload"]
    if rejected != {"type": "rejected", "code": "width_mismatch"}:
        raise RuntimeError("Gazebo did not reject the truncated controller output")
    accepted = process.exchange(
        host_frame(
            session,
            inject_step + 3,
            {
                "type": "step",
                "action_sequence": injected["action_sequence"],
                "values": clean_observations[inject_step - 1][
                    "joint_position_target_rad"
                ],
            },
        )
    )["payload"]
    expected = clean_observations[inject_step - 1]
    state_unchanged = (
        accepted.get("type") == "stepped"
        and accepted.get("step") == inject_step
        and accepted.get("sim_time_ticks") == inject_step * FIXED_DELTA_TICKS
        and accepted.get("values")
        == expected["joint_position_rad"] + expected["joint_velocity_rad_s"]
    )
    closed = process.exchange(host_frame(session, inject_step + 4, {"type": "close"}))[
        "payload"
    ]
    if closed.get("type") != "closed":
        raise RuntimeError("Gazebo failure session did not close cleanly")
    process.finish()
    if not state_unchanged:
        raise RuntimeError("rejected controller output advanced or changed Gazebo state")
    return {
        "kind": "rne_controller_contract_failure",
        "schema_version": 1,
        "backend_id": "gazebo_sim",
        "backend_version": "8.15.0",
        "task_id": task["task_id"],
        "task_sha256": task_sha256,
        "controller_id": controller["controller_id"],
        "controller_sha256": sha256(args.controller),
        "action_trace_sha256": sha256(args.actions),
        "runtime_manifest_sha256": runtime_manifest_sha256,
        "adapter_config_sha256": adapter_config_sha256,
        "injection_kind": injection["kind"],
        "injected_step": inject_step,
        "first_violation": injection["expected_first_violation"],
        "first_violation_step": inject_step,
        "first_violation_sim_time_ticks": inject_step * FIXED_DELTA_TICKS,
        "unit": "missing_action_element_count",
        "observed_missing_action_elements": 1,
        "maximum_missing_action_elements": 0,
        "rejection_code": "width_mismatch",
        "rejected_step_changed_state": False,
        "status": "failed_as_expected",
    }


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    task = load_json(args.task)
    controller = load_json(args.controller)
    actions = load_json(args.actions)
    task_sha256 = sha256(args.task)
    controller_sha256 = sha256(args.controller)
    runtime_manifest_sha256 = sha256(args.runtime_manifest)
    adapter_config_path = runtime_artifact_path(args.runtime_manifest, "adapter_config")
    robot_model_path = runtime_artifact_path(args.runtime_manifest, "robot_model")
    world_path = runtime_artifact_path(args.runtime_manifest, "world")
    adapter_config_sha256 = sha256(adapter_config_path)
    robot_model_sha256 = sha256(robot_model_path)
    world_sha256 = sha256(world_path)
    adapter_config = load_json(adapter_config_path)
    if (
        actions.get("kind") != "rne_controller_action_trace"
        or actions.get("task_id") != task.get("task_id")
        or actions.get("task_sha256") != task_sha256
        or actions.get("controller_id") != controller.get("controller_id")
        or actions.get("controller_sha256") != controller_sha256
        or actions.get("fixed_delta_ticks") != FIXED_DELTA_TICKS
        or actions.get("action_semantics")
        != "reference_trajectory_before_sensor_feedback"
        or actions.get("action_joint_order") != controller.get("action_joint_order")
    ):
        raise ValueError("compiled action trace is not bound to this TaskSpec/controller")
    joint_count = len(actions["action_joint_order"])
    contract = controller.get("observation_contract")
    law = controller.get("feedback_law")
    pid_vector_fields = (
        "position_error_gain",
        "velocity_damping_s",
        "integral_error_gain_s_inv",
        "maximum_integral_correction_rad",
        "maximum_correction_rad",
        "minimum_target_rad",
        "maximum_target_rad",
    )
    feedback_enabled = contract is not None or law is not None
    validate_measurement_fault(controller, len(actions["actions"]))
    availability_fault = availability_contract(controller)
    if feedback_enabled and (
        not isinstance(contract, dict)
        or not isinstance(law, dict)
        or contract.get("kind") != "rne_joint_feedback"
        or contract.get("schema_version") != 1
        or contract.get("sample_period_ticks") != FIXED_DELTA_TICKS
        or contract.get("phase_offset_ticks") != FIXED_DELTA_TICKS
        or contract.get("latency_ticks") != FIXED_DELTA_TICKS
        or (
            availability_fault is None
            and (
                contract.get("maximum_age_ticks") != FIXED_DELTA_TICKS
                or "stale_observation_policy" in contract
                or "recovery_policy" in contract
            )
        )
        or (
            availability_fault is not None
            and (
                contract.get("maximum_age_ticks") != 3 * FIXED_DELTA_TICKS
                or contract.get("stale_observation_policy")
                != "hold_last_accepted_target_and_freeze_state"
                or contract.get("recovery_policy")
                != "resume_on_fresh_nominal_observation"
            )
        )
        or contract.get("required_status") != "nominal"
        or contract.get("bootstrap_policy") != "reference_until_first_available"
    ):
        raise ValueError("unsupported OpenArm controller observation/feedback contract")
    if feedback_enabled:
        if law["kind"] == "joint_position_reference_pid_v1":
            if any(len(law.get(field, [])) != joint_count for field in pid_vector_fields):
                raise ValueError("PID controller vector width differs from the task")
        elif law["kind"] == "joint_position_state_feedback_integral_v1":
            numeric_fields = (
                "operating_point_position_rad",
                "operating_point_input_rad",
                "integral_state_feedback_gain_s_inv",
                "maximum_integral_state_error_rad_s",
                "maximum_state_integral_correction_rad",
                "maximum_state_feedback_correction_rad",
                "minimum_controlled_target_rad",
                "maximum_controlled_target_rad",
            )
            if (
                law.get("controlled_joint") not in actions["action_joint_order"]
                or law.get("state_order")
                != [
                    "predicted_tracking_error_rad",
                    "observed_tracking_error_rad",
                    "previous_input_tracking_error_rad",
                    "integrated_reference_error_rad_s",
                ]
                or law.get("reference_feedforward") != "unity_position_reference_v1"
                or law.get("observation_latency_compensation")
                != "one_sample_arx_predictor_v1"
                or len(law.get("state_feedback_gain", [])) != 3
                or len(law.get("desired_closed_loop_poles", [])) != 4
                or any(
                    not math.isfinite(value)
                    for field in numeric_fields
                    for value in [law.get(field, math.nan)]
                )
            ):
                raise ValueError("invalid state-feedback controller contract")
        else:
            raise ValueError("unsupported OpenArm feedback law")
    first_diagnostics_path = args.output / "gazebo-actuation-diagnostics-a.json"
    replay_diagnostics_path = args.output / "gazebo-actuation-diagnostics-b.json"
    first, first_digest, first_diagnostics = run_success(
        args,
        task,
        task_sha256,
        actions,
        controller,
        "rne.openarm.gazebo.success-a.v1",
        first_diagnostics_path,
    )
    replay, replay_digest, replay_diagnostics = run_success(
        args,
        task,
        task_sha256,
        actions,
        controller,
        "rne.openarm.gazebo.success-b.v1",
        replay_diagnostics_path,
    )
    if (
        first != replay
        or first_digest != replay_digest
        or first_diagnostics != replay_diagnostics
    ):
        raise RuntimeError("Gazebo replay differed for the exact same controller trace")
    failure = run_intentional_failure(
        args,
        task,
        task_sha256,
        controller,
        actions,
        first,
        runtime_manifest_sha256,
        adapter_config_sha256,
    )
    write_json(
        args.output / "gazebo-success-trace.json",
        {
            "kind": "rne_openarm_backend_trace",
            "schema_version": 1,
            "backend_id": "gazebo_sim",
            "backend_version": "8.15.0",
            "task_id": task["task_id"],
            "task_sha256": task_sha256,
            "controller_id": controller["controller_id"],
            "controller_sha256": controller_sha256,
            "action_trace_sha256": sha256(args.actions),
            "runtime_manifest_sha256": runtime_manifest_sha256,
            "adapter_config_sha256": adapter_config_sha256,
            "robot_model_sha256": robot_model_sha256,
            "world_sha256": world_sha256,
            "actuation_mode": adapter_config.get("actuation_mode", "velocity_servo"),
            "physics_substeps_per_control_step": adapter_config.get(
                "physics_substeps_per_control_step", 1
            ),
            "actuator_saturation_behavior": adapter_config.get(
                "saturation_behavior", "velocity_clamp"
            ),
            "actuator_failure_behavior": adapter_config.get(
                "failure_behavior", "reject_invalid_configuration_before_simulator_start"
            ),
            "actuation_diagnostics_kind": first_diagnostics["kind"],
            "actuation_diagnostics_sha256": sha256(first_diagnostics_path),
            "replay_actuation_diagnostics_sha256": sha256(replay_diagnostics_path),
            "fixed_delta_ticks": FIXED_DELTA_TICKS,
            "joint_feedback_schema_version": 1,
            "joint_feedback_latency_ticks": FIXED_DELTA_TICKS,
            "observation_source": "adapter_task_tensor_to_typed_joint_feedback",
            "controller_execution": (
                "open_loop_reference"
                if not feedback_enabled
                else (
                    "artifact_defined_joint_feedback_pid"
                    if law["kind"] == "joint_position_reference_pid_v1"
                    else "artifact_defined_joint_feedback_state_space"
                )
            ),
            "state_hash_contract": "rne_gazebo_adapter_state_v1_blake2b64_f64",
            "final_state_digest": first_digest,
            "replay_final_state_digest": replay_digest,
            "replay_match": True,
            "final_maximum_tracking_error_rad": first[-1]["maximum_tracking_error_rad"],
            "maximum_tracking_error_rad": max(
                frame["maximum_tracking_error_rad"] for frame in first
            ),
            "observations": first,
        },
    )
    write_json(args.output / "gazebo-intentional-failure.json", failure)
    print(
        "OpenArm Gazebo trace: "
        f"steps={len(first)} replay_match=true "
        f"final_error_rad={first[-1]['maximum_tracking_error_rad']:.6f}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm Gazebo trace failed: {error}", file=sys.stderr)
        raise SystemExit(2)

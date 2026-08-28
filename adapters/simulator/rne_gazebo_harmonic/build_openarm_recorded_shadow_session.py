#!/usr/bin/env python3
"""Compile retained OpenArm traces into a content-addressed shadow session."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--task", required=True, type=Path)
    parser.add_argument("--controller", required=True, type=Path)
    parser.add_argument("--actions", required=True, type=Path)
    parser.add_argument("--recorded-trace", required=True, type=Path)
    parser.add_argument("--simulation-trace", required=True, type=Path)
    parser.add_argument(
        "--calibration",
        type=Path,
        default=SCRIPT_DIR / "openarm_joint_state.calibration.json",
    )
    parser.add_argument(
        "--requirements",
        type=Path,
        default=SCRIPT_DIR / "openarm_recorded_shadow_requirements.json",
    )
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--disconnect-after-sequence", type=int)
    return parser.parse_args()


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def compile_session(
    task_path: Path,
    controller_path: Path,
    actions_path: Path,
    recorded_trace_path: Path,
    simulation_trace_path: Path,
    calibration_path: Path,
    requirements_path: Path,
    disconnect_after_sequence: int | None,
) -> dict[str, Any]:
    task = load(task_path)
    controller = load(controller_path)
    action_artifact = load(actions_path)
    recorded = load(recorded_trace_path)
    simulation = load(simulation_trace_path)
    requirements = load(requirements_path)
    actions = action_artifact.get("actions")
    recorded_observations = recorded.get("observations")
    simulation_observations = simulation.get("observations")
    task_hash = sha256(task_path)
    controller_hash = sha256(controller_path)
    action_hash = sha256(actions_path)
    if (
        task.get("kind") != "rne_task_spec"
        or requirements.get("kind") != "rne_openarm_recorded_shadow_requirements"
        or requirements.get("task_id") != task.get("task_id")
        or controller.get("task_id") != task.get("task_id")
        or action_artifact.get("task_sha256") != task_hash
        or action_artifact.get("controller_sha256") != controller_hash
        or not isinstance(actions, list)
        or not isinstance(recorded_observations, list)
        or not isinstance(simulation_observations, list)
        or len(actions) != len(recorded_observations)
        or len(actions) != len(simulation_observations)
    ):
        raise ValueError("TaskSpec, controller, actions, and traces are not one session")
    for trace in (recorded, simulation):
        if (
            trace.get("kind") != "rne_openarm_backend_trace"
            or trace.get("task_sha256") != task_hash
            or trace.get("controller_sha256") != controller_hash
            or trace.get("action_trace_sha256") != action_hash
            or trace.get("replay_match") is not True
        ):
            raise ValueError("backend trace identity or replay evidence differs")
    report_module = load_module(
        "rne_recorded_shadow_controller_report",
        SCRIPT_DIR / "build_openarm_controller_report.py",
    )
    runner_module = load_module(
        "rne_recorded_shadow_controller_runner", SCRIPT_DIR / "run_openarm_trace.py"
    )
    reproduction = report_module.reproduce_decisions(
        runner_module, controller, actions, recorded_observations
    )
    if (
        reproduction["first_mismatch"] is not None
        or reproduction["maximum_numeric_delta_rad"] > 1.0e-12
    ):
        raise ValueError("recorded controller decisions cannot be reproduced exactly")
    fixed_delta_ticks = recorded.get("fixed_delta_ticks")
    if (
        not isinstance(fixed_delta_ticks, int)
        or fixed_delta_ticks <= 0
        or simulation.get("fixed_delta_ticks") != fixed_delta_ticks
        or requirements.get("fixed_delta_ticks") != fixed_delta_ticks
    ):
        raise ValueError("backend fixed-step contracts differ")
    frames = []
    for decision_index in range(2, len(actions)):
        source = recorded_observations[decision_index - 2]
        simulated = simulation_observations[decision_index - 2]
        decision = recorded_observations[decision_index]
        sequence = source["step"]
        if (
            decision.get("controller_observation_sequence") != sequence
            or decision.get("controller_bootstrap") is not False
            or source.get("sensor_status") != "nominal"
            or source.get("sensor_sample_published") is not True
            or source.get("sample_phase_error_ticks") != 0
            or source.get("available_time_ticks") - source.get("scheduled_capture_ticks")
            != fixed_delta_ticks
            or simulated.get("step") != sequence
        ):
            raise ValueError(f"recorded observation ordering differs at decision {decision_index + 1}")
        frames.append(
            {
                "observation_sequence": sequence,
                "dropped_sequences_before": 0,
                "captured_at_ticks": source["scheduled_capture_ticks"],
                "available_at_ticks": source["available_time_ticks"],
                "simulation_step": simulated["step"],
                "simulation_time_ticks": simulated["sim_time_ticks"],
                "recorded_values": source["joint_position_rad"]
                + source["joint_velocity_rad_s"],
                "simulation_values": simulated["joint_position_rad"]
                + simulated["joint_velocity_rad_s"],
                "action_sequence": actions[decision_index]["action_sequence"],
                "action_submitted_at_ticks": source["available_time_ticks"],
                "action_values": decision["joint_controller_target_rad"],
            }
        )
    if disconnect_after_sequence is not None:
        if (
            disconnect_after_sequence
            != requirements.get("disconnect_after_observation_sequence")
            or not any(
                frame["observation_sequence"] == disconnect_after_sequence
                for frame in frames
            )
        ):
            raise ValueError("disconnect sequence differs from the declared fault")
    calibration = load(calibration_path)
    declared_tolerances = requirements.get("tolerances")
    task_tensors = task["observation"]["tensors"]
    if (
        calibration.get("kind") != "rne_joint_state_calibration"
        or calibration.get("task_id") != task["task_id"]
        or calibration.get("position_unit") != task["observation"]["tensors"][0]["unit"]
        or calibration.get("velocity_unit") != task["observation"]["tensors"][1]["unit"]
        or requirements.get("bootstrap_action_count") != 2
        or len(frames) < requirements.get("minimum_compared_samples", 0)
        or requirements.get("maximum_dropped_observations") != 0
        or requirements.get("requires_zero_actuator_writes") is not True
        or requirements.get("maximum_disconnect_response_ticks") != 0
        or not isinstance(declared_tolerances, list)
        or [item.get("tensor_name") for item in declared_tolerances]
        != [tensor["name"] for tensor in task_tensors]
        or [item.get("unit") for item in declared_tolerances]
        != [tensor["unit"] for tensor in task_tensors]
    ):
        raise ValueError("recorded/shadow requirements or calibration differ")
    return {
        "kind": "rne_recorded_shadow_session",
        "schema_version": 1,
        "experiment_id": requirements["experiment_id"],
        "requirements_sha256": sha256(requirements_path),
        "task_id": task["task_id"],
        "task_sha256": task_hash,
        "controller_id": controller["controller_id"],
        "controller_sha256": controller_hash,
        "sources": [
            {
                "role": "controller_actions",
                "kind": action_artifact["kind"],
                "file_name": actions_path.name,
                "sha256": action_hash,
            },
            {
                "role": "recorded_trace",
                "kind": recorded["backend_id"],
                "file_name": recorded_trace_path.name,
                "sha256": sha256(recorded_trace_path),
            },
            {
                "role": "simulation_trace",
                "kind": simulation["backend_id"],
                "file_name": simulation_trace_path.name,
                "sha256": sha256(simulation_trace_path),
            },
            {
                "role": "joint_state_calibration",
                "kind": calibration["kind"],
                "file_name": calibration_path.name,
                "sha256": sha256(calibration_path),
            },
        ],
        "bootstrap_action_count": requirements["bootstrap_action_count"],
        "stream": {
            "clock_source": "rne_sim_clock",
            "tick_period_ns": 1,
            "nominal_latency_ticks": requirements["nominal_latency_ticks"],
            "maximum_latency_ticks": requirements["maximum_latency_ticks"],
            "drop_policy": requirements["drop_policy"],
            "sample_capacity": len(frames),
            "tensor_units": [
                {"tensor_name": tensor["name"], "unit": tensor["unit"]}
                for tensor in task["observation"]["tensors"]
            ],
            "calibrations": [
                {
                    "role": "joint_state",
                    "kind": calibration["kind"],
                    "sha256": sha256(calibration_path),
                }
            ],
        },
        "tolerances": [
            {
                "tensor_name": tolerance["tensor_name"],
                "absolute_tolerance": tolerance["absolute_tolerance"],
            }
            for tolerance in requirements["tolerances"]
        ],
        "frames": frames,
        "disconnect_after_observation_sequence": disconnect_after_sequence,
    }


def main() -> int:
    args = parse_args()
    session = compile_session(
        args.task.resolve(),
        args.controller.resolve(),
        args.actions.resolve(),
        args.recorded_trace.resolve(),
        args.simulation_trace.resolve(),
        args.calibration.resolve(),
        args.requirements.resolve(),
        args.disconnect_after_sequence,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(session, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    print(
        f"OpenArm recorded/shadow session: samples={len(session['frames'])} "
        f"disconnect={session['disconnect_after_observation_sequence']} -> {args.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

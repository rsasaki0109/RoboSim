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
RESPONSE_TIMEOUT_S = 15.0


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


class AdapterProcess:
    def __init__(self, adapter: Path, runtime: Path, task: Path) -> None:
        self.process = subprocess.Popen(
            [
                sys.executable,
                str(adapter),
                "--runtime-manifest",
                str(runtime),
                "--task",
                str(task),
            ],
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
            raise TimeoutError("Gazebo adapter response exceeded 15 seconds")
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


def run_success(
    args: argparse.Namespace,
    task: dict[str, Any],
    task_sha256: str,
    action_artifact: dict[str, Any],
    session: str,
) -> tuple[list[dict[str, Any]], int]:
    process = AdapterProcess(args.adapter, args.runtime_manifest, args.task)
    joint_count = len(action_artifact["action_joint_order"])
    open_and_reset(process, session, task, task_sha256, 2 * joint_count, joint_count)
    observations: list[dict[str, Any]] = []
    final_digest = 0
    for action in action_artifact["actions"]:
        payload = process.exchange(
            host_frame(
                session,
                action["step"] + 2,
                {
                    "type": "step",
                    "action_sequence": action["action_sequence"],
                    "values": action["joint_position_target_rad"],
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
        observations.append(
            {
                "step": action["step"],
                "sim_time_ticks": action["sim_time_ticks"],
                "joint_position_rad": positions,
                "joint_velocity_rad_s": velocities,
                "maximum_tracking_error_rad": maximum_error,
            }
        )
        final_digest = payload["state_digest"]
    closed = process.exchange(
        host_frame(session, len(action_artifact["actions"]) + 3, {"type": "close"})
    )["payload"]
    if closed.get("type") != "closed":
        raise RuntimeError("Gazebo adapter did not close cleanly")
    process.finish()
    return observations, final_digest


def run_intentional_failure(
    args: argparse.Namespace,
    task: dict[str, Any],
    task_sha256: str,
    controller: dict[str, Any],
    action_artifact: dict[str, Any],
    clean_observations: list[dict[str, Any]],
) -> dict[str, Any]:
    injection = controller["intentional_failure"]
    inject_step = injection["inject_at_step"]
    process = AdapterProcess(args.adapter, args.runtime_manifest, args.task)
    session = "rne.openarm.gazebo.intentional-failure.v1"
    joint_count = len(action_artifact["action_joint_order"])
    open_and_reset(process, session, task, task_sha256, 2 * joint_count, joint_count)
    actions = action_artifact["actions"]
    for action in actions[: inject_step - 1]:
        payload = process.exchange(
            host_frame(
                session,
                action["step"] + 2,
                {
                    "type": "step",
                    "action_sequence": action["action_sequence"],
                    "values": action["joint_position_target_rad"],
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
                "values": injected["joint_position_target_rad"][:-1],
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
                "values": injected["joint_position_target_rad"],
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
    if (
        actions.get("kind") != "rne_controller_action_trace"
        or actions.get("task_id") != task.get("task_id")
        or actions.get("task_sha256") != task_sha256
        or actions.get("controller_id") != controller.get("controller_id")
        or actions.get("controller_sha256") != controller_sha256
        or actions.get("fixed_delta_ticks") != FIXED_DELTA_TICKS
        or actions.get("action_joint_order") != controller.get("action_joint_order")
    ):
        raise ValueError("compiled action trace is not bound to this TaskSpec/controller")
    first, first_digest = run_success(
        args, task, task_sha256, actions, "rne.openarm.gazebo.success-a.v1"
    )
    replay, replay_digest = run_success(
        args, task, task_sha256, actions, "rne.openarm.gazebo.success-b.v1"
    )
    if first != replay or first_digest != replay_digest:
        raise RuntimeError("Gazebo replay differed for the exact same controller trace")
    failure = run_intentional_failure(args, task, task_sha256, controller, actions, first)
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
            "fixed_delta_ticks": FIXED_DELTA_TICKS,
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

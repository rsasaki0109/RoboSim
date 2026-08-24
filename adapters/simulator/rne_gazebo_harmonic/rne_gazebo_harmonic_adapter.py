#!/usr/bin/env python3
"""Process-isolated Gazebo Harmonic adapter for OpenArm joint tracking."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path
import struct
import subprocess
import sys
from typing import Any

import gz.sim8 as gz_sim

from openarm_actuation import realize_joint_command, validate_actuation


HOST_KIND = "rne_simulator_host_frame"
ADAPTER_KIND = "rne_simulator_adapter_frame"
SCHEMA_VERSION = 1


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--runtime-manifest", required=True, type=Path)
    parser.add_argument("--task", required=True, type=Path)
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


def tensor_width(catalog: dict[str, Any]) -> int:
    width = 0
    for tensor in catalog["tensors"]:
        elements = 1
        for extent in tensor["shape"]:
            elements *= extent
        width += elements
    return width


def artifact_path(manifest_path: Path, manifest: dict[str, Any], role: str) -> Path:
    matches = [entry for entry in manifest["artifacts"] if entry["role"] == role]
    if len(matches) != 1:
        raise ValueError(f"runtime manifest must contain one {role} artifact")
    entry = matches[0]
    path = manifest_path.parent / entry["file"]
    if path.stat().st_size != entry["size_bytes"] or sha256(path) != entry["sha256"]:
        raise ValueError(f"runtime artifact {entry['file']} does not match its manifest")
    return path


class GazeboOpenArmAdapter:
    def __init__(self, runtime_path: Path, task_path: Path) -> None:
        self.runtime_path = runtime_path.resolve()
        self.task_path = task_path.resolve()
        self.runtime = load_json(self.runtime_path)
        self.task = load_json(self.task_path)
        self.world_path = artifact_path(self.runtime_path, self.runtime, "world")
        self.robot_path = artifact_path(self.runtime_path, self.runtime, "robot_model")
        self.config = load_json(
            artifact_path(self.runtime_path, self.runtime, "adapter_config")
        )
        self.task_sha256 = sha256(self.task_path)
        self.observation_width = tensor_width(self.task["observation"])
        self.action_width = tensor_width(self.task["action"])
        self.fixed_delta_ticks = round(float(self.task["control_step_s"]) * 1_000_000_000)
        self.joint_names = list(self.config["joint_order"])
        if self.observation_width != 2 * len(self.joint_names):
            raise ValueError("observation width must be joint positions followed by velocities")
        if self.action_width != len(self.joint_names):
            raise ValueError("action width must equal configured joint count")
        self.actuation_mode, self.physics_substeps, self.effort_joint_count = (
            validate_actuation(self.config, len(self.joint_names))
        )
        if self.runtime["fixed_delta_ticks"] != self.fixed_delta_ticks:
            raise ValueError("runtime and TaskSpec fixed delta differ")
        if self.runtime["simulator_version"] != gazebo_version():
            raise ValueError("installed Gazebo version differs from runtime manifest")
        self.session_id: str | None = None
        self.last_sequence: int | None = None
        self.next_action_sequence = 0
        self.reset_seed: int | None = None
        self.step_count = 0
        self.fixture: gz_sim.TestFixture | None = None
        self.server: gz_sim.Server | None = None
        self.targets = [0.0] * self.action_width
        self.observation = [0.0] * self.observation_width

        resource_paths = [str(self.world_path.parent), str(self.robot_path.parent)]
        repo_assets = Path(__file__).resolve().parents[3] / "assets" / "robots"
        resource_paths.append(str(repo_assets))
        existing = os.environ.get("GZ_SIM_RESOURCE_PATH")
        if existing:
            resource_paths.append(existing)
        os.environ["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join(resource_paths)

    def response(self, request: dict[str, Any], payload: dict[str, Any]) -> dict[str, Any]:
        return {
            "kind": ADAPTER_KIND,
            "schema_version": SCHEMA_VERSION,
            "session_id": request.get("session_id", "rne.simulator.invalid.v1"),
            "request_sequence": request.get("sequence", 0),
            "payload": payload,
        }

    def rejected(self, request: dict[str, Any], code: str) -> dict[str, Any]:
        return self.response(request, {"type": "rejected", "code": code})

    def handle(self, request: dict[str, Any]) -> dict[str, Any]:
        if set(request) != {"kind", "schema_version", "session_id", "sequence", "payload"}:
            raise ValueError("host frame has unknown or missing fields")
        if request["kind"] != HOST_KIND or request["schema_version"] != SCHEMA_VERSION:
            raise ValueError("unsupported host frame envelope")
        if not isinstance(request["payload"], dict):
            raise ValueError("payload must be an object")
        sequence = request["sequence"]
        if self.last_sequence is not None and sequence <= self.last_sequence:
            return self.rejected(request, "non_monotonic_sequence")
        if self.session_id is not None and request["session_id"] != self.session_id:
            return self.rejected(request, "session_mismatch")
        self.last_sequence = sequence
        payload_type = request["payload"].get("type")
        if payload_type == "open":
            return self.open(request)
        if payload_type == "reset":
            return self.reset(request)
        if payload_type == "step":
            return self.step(request)
        if payload_type == "close":
            return self.close(request)
        raise ValueError("unknown host payload type")

    def open(self, request: dict[str, Any]) -> dict[str, Any]:
        payload = request["payload"]
        if set(payload) != {
            "type",
            "task_id",
            "task_sha256",
            "observation_width",
            "action_width",
            "fixed_delta_ticks",
        }:
            raise ValueError("open payload has unknown or missing fields")
        if self.session_id is not None:
            return self.rejected(request, "already_open")
        if payload["task_id"] != self.task["task_id"] or payload["task_sha256"] != self.task_sha256:
            return self.rejected(request, "task_mismatch")
        if payload["observation_width"] != self.observation_width or payload["action_width"] != self.action_width:
            return self.rejected(request, "width_mismatch")
        if payload["fixed_delta_ticks"] != self.fixed_delta_ticks:
            return self.rejected(request, "fixed_delta_mismatch")
        self.session_id = request["session_id"]
        return self.response(
            request,
            {
                "type": "ready",
                "simulator_id": self.runtime["simulator_id"],
                "simulator_version": self.runtime["simulator_version"],
                "adapter_id": self.config["adapter_id"],
                "task_id": self.task["task_id"],
                "task_sha256": self.task_sha256,
                "observation_width": self.observation_width,
                "action_width": self.action_width,
                "fixed_delta_ticks": self.fixed_delta_ticks,
            },
        )

    def reset(self, request: dict[str, Any]) -> dict[str, Any]:
        payload = request["payload"]
        if set(payload) != {"type", "seed"}:
            raise ValueError("reset payload has unknown or missing fields")
        if self.session_id is None:
            return self.rejected(request, "not_open")
        self.reset_seed = payload["seed"]
        self.next_action_sequence = 0
        self.step_count = 0
        self.targets = [0.0] * self.action_width
        self.observation = [0.0] * self.observation_width
        self._create_fixture()
        return self.response(
            request,
            {
                "type": "reset_complete",
                "seed": self.reset_seed,
                "values": self.observation,
                "state_digest": state_digest(self.step_count, self.observation),
            },
        )

    def _create_fixture(self) -> None:
        self.fixture = gz_sim.TestFixture(str(self.world_path))
        self.fixture.on_pre_update(self._pre_update)
        self.fixture.on_post_update(self._post_update)
        self.fixture.finalize()
        self.server = self.fixture.server()

    def _joints(self, ecm: gz_sim.EntityComponentManager) -> list[gz_sim.Joint]:
        world = gz_sim.World(gz_sim.world_entity(ecm))
        model_entity = world.model_by_name(ecm, self.config["model_name"])
        if model_entity == gz_sim.K_NULL_ENTITY:
            raise RuntimeError("configured OpenArm model was not loaded")
        model = gz_sim.Model(model_entity)
        joints = [gz_sim.Joint(model.joint_by_name(ecm, name)) for name in self.joint_names]
        if any(not joint.valid(ecm) for joint in joints):
            raise RuntimeError("configured OpenArm joint was not loaded")
        return joints

    def _pre_update(self, _info: gz_sim.UpdateInfo, ecm: gz_sim.EntityComponentManager) -> None:
        for index, (joint, target) in enumerate(zip(self._joints(ecm), self.targets)):
            joint.enable_position_check(ecm, True)
            joint.enable_velocity_check(ecm, True)
            position = joint.position(ecm)
            current = position[0] if position else 0.0
            measured_velocity = joint.velocity(ecm)
            current_velocity = measured_velocity[0] if measured_velocity else 0.0
            command_kind, command = realize_joint_command(
                self.config,
                self.actuation_mode,
                self.effort_joint_count,
                index,
                target,
                current,
                current_velocity,
            )
            if command_kind == "effort_nm":
                joint.set_force(ecm, [command])
            else:
                joint.set_velocity(ecm, [command])

    def _post_update(self, _info: gz_sim.UpdateInfo, ecm: gz_sim.EntityComponentManager) -> None:
        positions: list[float] = []
        velocities: list[float] = []
        for joint in self._joints(ecm):
            position = joint.position(ecm)
            velocity = joint.velocity(ecm)
            positions.append(position[0] if position else 0.0)
            velocities.append(velocity[0] if velocity else 0.0)
        self.observation = positions + velocities

    def step(self, request: dict[str, Any]) -> dict[str, Any]:
        payload = request["payload"]
        if set(payload) != {"type", "action_sequence", "values"}:
            raise ValueError("step payload has unknown or missing fields")
        if self.session_id is None:
            return self.rejected(request, "not_open")
        if self.reset_seed is None or self.server is None:
            return self.rejected(request, "reset_required")
        values = payload["values"]
        if len(values) != self.action_width:
            return self.rejected(request, "width_mismatch")
        if not all(isinstance(value, (int, float)) and math.isfinite(value) for value in values):
            return self.rejected(request, "non_finite_value")
        if payload["action_sequence"] != self.next_action_sequence:
            return self.rejected(request, "action_sequence_mismatch")
        self.targets = [float(value) for value in values]
        if not self.server.run(True, self.physics_substeps, False):
            raise RuntimeError("Gazebo failed to advance one iteration")
        self.step_count += 1
        self.next_action_sequence += 1
        return self.response(
            request,
            {
                "type": "stepped",
                "action_sequence": payload["action_sequence"],
                "step": self.step_count,
                "sim_time_ticks": self.step_count * self.fixed_delta_ticks,
                "values": self.observation,
                "terminated": False,
                "truncated": False,
                "state_digest": state_digest(self.step_count, self.observation),
            },
        )

    def close(self, request: dict[str, Any]) -> dict[str, Any]:
        if set(request["payload"]) != {"type"}:
            raise ValueError("close payload has unknown fields")
        if self.session_id is None:
            return self.rejected(request, "not_open")
        self.fixture = None
        self.server = None
        return self.response(request, {"type": "closed"})


def gazebo_version() -> str:
    result = subprocess.run(
        ["gz", "sim", "--versions"],
        check=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    return result.stdout.strip().splitlines()[-1]


def state_digest(step: int, values: list[float]) -> int:
    digest = hashlib.blake2b(digest_size=8, person=b"RNE-GZ-H")
    digest.update(step.to_bytes(8, "little"))
    for value in values:
        digest.update(struct.pack("<d", value))
    return int.from_bytes(digest.digest(), "little")


def main() -> int:
    args = parse_args()
    adapter = GazeboOpenArmAdapter(args.runtime_manifest, args.task)
    for line in sys.stdin:
        request = json.loads(line)
        response = adapter.handle(request)
        sys.stdout.write(json.dumps(response, separators=(",", ":"), allow_nan=False) + "\n")
        sys.stdout.flush()
        if response["payload"]["type"] == "closed":
            return 0
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:  # The adapter must never contaminate stdout.
        print(f"rne Gazebo adapter failed: {error}", file=sys.stderr)
        raise SystemExit(2)

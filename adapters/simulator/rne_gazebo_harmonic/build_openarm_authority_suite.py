#!/usr/bin/env python3
"""Compiles content-addressed OpenArm joint-authority fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
from typing import Any


COPIED_FILES = (
    "openarm_payload.world.sdf",
    "openarm_v2_right.payload.urdf",
    "openarm_payload.rne.robot.toml",
    "openarm_payload.rne.scene.toml",
)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--baseline-fixture", required=True, type=Path)
    parser.add_argument(
        "--manifest", type=Path, default=root / "openarm_authority_experiments.json"
    )
    parser.add_argument(
        "--actuation-config",
        type=Path,
        default=root / "openarm_right.rne_actuation.json",
    )
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


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def case_id(scale: float) -> str:
    return f"authority-{round(scale * 1000):04d}permille"


def artifact(path: Path, role: str) -> dict[str, Any]:
    return {
        "role": role,
        "file": path.name,
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def compile_suite(
    manifest_path: Path,
    baseline_fixture: Path,
    actuation_config_path: Path,
    output: Path,
) -> dict[str, Any]:
    manifest = load(manifest_path)
    scales = manifest.get("authority_scale_grid")
    if (
        manifest.get("kind")
        != "rne_openarm_actuator_authority_experiment_manifest"
        or manifest.get("schema_version") != 1
        or not isinstance(scales, list)
        or len(scales) < 3
        or scales != sorted(set(scales), reverse=True)
        or scales[0] != 1.0
        or any(
            not isinstance(scale, (int, float))
            or not math.isfinite(scale)
            or not 0.0 < scale <= 1.0
            for scale in scales
        )
    ):
        raise ValueError("unsupported authority experiment manifest")
    baseline_runtime = load(baseline_fixture / "runtime.json")
    baseline_adapter = load(baseline_fixture / "openarm_right.adapter.json")
    actuation = load(actuation_config_path)
    joint = manifest["controlled_joint"]
    joint_order = baseline_adapter["joint_order"]
    if joint not in joint_order:
        raise ValueError("controlled joint is absent from Gazebo adapter order")
    joint_index = joint_order.index(joint)
    native_matches = [item for item in actuation["joints"] if item["joint_name"] == joint]
    if len(native_matches) != 1:
        raise ValueError("controlled joint is absent from native actuation config")
    native_effort_nm = native_matches[0]["max_effort_nm"]
    gazebo_effort_nm = baseline_adapter["maximum_effort_nm"][joint_index]
    if native_effort_nm != gazebo_effort_nm:
        raise ValueError("native and Gazebo baseline effort limits differ")
    for name in COPIED_FILES:
        if not (baseline_fixture / name).is_file():
            raise ValueError(f"baseline fixture is missing {name}")

    cases = []
    for scale in scales:
        identifier = case_id(float(scale))
        directory = output / identifier
        directory.mkdir(parents=True, exist_ok=True)
        for name in COPIED_FILES:
            shutil.copyfile(baseline_fixture / name, directory / name)

        case_actuation = json.loads(json.dumps(actuation))
        case_native = next(
            item for item in case_actuation["joints"] if item["joint_name"] == joint
        )
        realized_effort_nm = native_effort_nm * scale
        case_native["max_effort_nm"] = realized_effort_nm
        authority_degradation = {
            "kind": "controlled_joint_max_effort_scale_v1",
            "joint": joint,
            "scale": scale,
            "baseline_max_effort_nm": native_effort_nm,
            "realized_max_effort_nm": realized_effort_nm,
            "failure_behavior": "clamp_at_degraded_effort_limit",
        }
        actuation_path = directory / "openarm_right.rne_actuation.json"
        write_json(actuation_path, case_actuation)

        case_adapter = json.loads(json.dumps(baseline_adapter))
        case_adapter["maximum_effort_nm"][joint_index] = realized_effort_nm
        case_adapter["authority_degradation"] = authority_degradation
        adapter_path = directory / "openarm_right.adapter.json"
        write_json(adapter_path, case_adapter)

        runtime = json.loads(json.dumps(baseline_runtime))
        runtime["artifacts"] = [
            artifact(directory / "openarm_payload.world.sdf", "world"),
            artifact(directory / "openarm_v2_right.payload.urdf", "robot_model"),
            artifact(adapter_path, "adapter_config"),
        ]
        runtime_path = directory / "runtime.json"
        write_json(runtime_path, runtime)
        fixture = {
            "kind": "rne_openarm_actuator_authority_fixture",
            "schema_version": 1,
            "case_id": identifier,
            "controlled_joint": joint,
            "authority_scale": scale,
            "baseline_max_effort_nm": native_effort_nm,
            "realized_max_effort_nm": realized_effort_nm,
            "actuation_config_sha256": sha256(actuation_path),
            "adapter_config_sha256": sha256(adapter_path),
            "runtime_manifest_sha256": sha256(runtime_path),
            "model_urdf_sha256": sha256(directory / "openarm_v2_right.payload.urdf"),
            "robot_asset_config_sha256": sha256(
                directory / "openarm_payload.rne.robot.toml"
            ),
            "scene_config_sha256": sha256(
                directory / "openarm_payload.rne.scene.toml"
            ),
        }
        write_json(directory / "authority-fixture.json", fixture)
        cases.append(fixture)

    suite = {
        "kind": "rne_openarm_actuator_authority_suite",
        "schema_version": 1,
        "experiment_id": manifest["experiment_id"],
        "controlled_joint": manifest["controlled_joint"],
        "declared_minimum_supported_authority_scale": manifest[
            "declared_minimum_supported_authority_scale"
        ],
        "requirements": manifest["requirements"],
        "backend_order": ["rne_rapier", "mujoco_native", "gazebo_sim"],
        "inputs": {
            "experiment_manifest_sha256": sha256(manifest_path),
            "baseline_runtime_manifest_sha256": sha256(
                baseline_fixture / "runtime.json"
            ),
            "baseline_adapter_config_sha256": sha256(
                baseline_fixture / "openarm_right.adapter.json"
            ),
            "source_actuation_config_sha256": sha256(actuation_config_path),
        },
        "cases": cases,
    }
    write_json(output / "authority-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_suite(
        args.manifest.resolve(),
        args.baseline_fixture.resolve(),
        args.actuation_config.resolve(),
        args.output.resolve(),
    )
    print(f"OpenArm authority suite: {len(suite['cases'])} cases -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

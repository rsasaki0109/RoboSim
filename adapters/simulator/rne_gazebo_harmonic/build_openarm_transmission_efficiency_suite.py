#!/usr/bin/env python3
"""Compile OpenArm motor-to-joint transmission-efficiency fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
from typing import Any


PORTABLE_MODEL_FILE = "openarm_v2_right.coulomb.urdf"
GAZEBO_MODEL_FILE = "openarm_v2_right.payload.urdf"
WORLD_FILE = "openarm_payload.world.sdf"
ROBOT_FILE = "openarm_payload.rne.robot.toml"
SCENE_FILE = "openarm_payload.rne.scene.toml"
ACTUATION_FILE = "openarm_right.rne_actuation.json"
ADAPTER_FILE = "openarm_right.adapter.json"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-fixture", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root / "openarm_transmission_efficiency_experiments.json",
    )
    return parser.parse_args()


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as source:
        for chunk in iter(lambda: source.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def artifact(path: Path, role: str) -> dict[str, Any]:
    return {
        "role": role,
        "file": path.name,
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def case_id(efficiency: float) -> str:
    percent = round(efficiency * 100.0)
    if not math.isclose(efficiency, percent / 100.0, abs_tol=1e-12):
        raise ValueError("transmission efficiencies must resolve to whole percent")
    return f"joint5-efficiency-{percent:03d}pct"


def validate_manifest(manifest: dict[str, Any]) -> None:
    grid = manifest.get("transmission_efficiency_grid")
    supported = manifest.get("declared_minimum_supported_efficiency")
    if (
        manifest.get("kind")
        != "rne_openarm_transmission_efficiency_experiment_manifest"
        or manifest.get("schema_version") != 1
        or not isinstance(grid, list)
        or len(grid) < 3
        or grid != sorted(set(grid), reverse=True)
        or grid[0] != 1.0
        or any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or not 0.0 < value <= 1.0
            for value in grid
        )
        or supported not in grid
        or supported == grid[-1]
    ):
        raise ValueError("unsupported transmission-efficiency experiment manifest")


def actuation_config(
    source: dict[str, Any], joint_name: str, efficiency: float
) -> dict[str, Any]:
    config = json.loads(json.dumps(source))
    joints = config.get("joints")
    if not isinstance(joints, list) or [item.get("joint_name") for item in joints].count(
        joint_name
    ) != 1:
        raise ValueError("portable actuation does not contain the controlled joint exactly once")
    for joint in joints:
        joint["transmission_efficiency"] = efficiency if joint["joint_name"] == joint_name else 1.0
    return config


def adapter_config(
    source: dict[str, Any], joint_name: str, efficiency: float
) -> dict[str, Any]:
    config = json.loads(json.dumps(source))
    joints = config.get("joint_order")
    if not isinstance(joints, list) or joints.count(joint_name) != 1:
        raise ValueError("Gazebo adapter does not contain the controlled joint exactly once")
    values = [1.0] * len(joints)
    values[joints.index(joint_name)] = efficiency
    config["transmission_efficiency_by_joint"] = values
    return config


def compile_suite(manifest_path: Path, baseline: Path, output: Path) -> dict[str, Any]:
    manifest = load(manifest_path)
    validate_manifest(manifest)
    required = (
        PORTABLE_MODEL_FILE,
        GAZEBO_MODEL_FILE,
        WORLD_FILE,
        ROBOT_FILE,
        SCENE_FILE,
        ACTUATION_FILE,
        ADAPTER_FILE,
        "runtime.json",
        "coulomb-friction-fixture.json",
    )
    for name in required:
        if not (baseline / name).is_file():
            raise ValueError(f"baseline fixture is missing {name}")
    baseline_coulomb = load(baseline / "coulomb-friction-fixture.json")
    joint_name = manifest["controlled_joint"]
    if (
        baseline_coulomb.get("plant_coulomb_friction_nm") != 0.5
        or baseline_coulomb.get("controlled_joint") != joint_name
    ):
        raise ValueError("transmission suite requires the qualified 0.5 N*m fixture")
    source_actuation = load(baseline / ACTUATION_FILE)
    source_adapter = load(baseline / ADAPTER_FILE)
    baseline_runtime = load(baseline / "runtime.json")
    fixed_hashes = {
        name: sha256(baseline / name)
        for name in (PORTABLE_MODEL_FILE, GAZEBO_MODEL_FILE, WORLD_FILE, ROBOT_FILE, SCENE_FILE)
    }
    cases = []
    for raw_efficiency in manifest["transmission_efficiency_grid"]:
        efficiency = float(raw_efficiency)
        identifier = case_id(efficiency)
        directory = output / identifier
        directory.mkdir(parents=True, exist_ok=True)
        for name in (
            PORTABLE_MODEL_FILE,
            GAZEBO_MODEL_FILE,
            WORLD_FILE,
            ROBOT_FILE,
            SCENE_FILE,
        ):
            shutil.copyfile(baseline / name, directory / name)
        write_json(
            directory / ACTUATION_FILE,
            actuation_config(source_actuation, joint_name, efficiency),
        )
        write_json(
            directory / ADAPTER_FILE,
            adapter_config(source_adapter, joint_name, efficiency),
        )
        runtime = json.loads(json.dumps(baseline_runtime))
        runtime["artifacts"] = [
            artifact(directory / WORLD_FILE, "world"),
            artifact(directory / GAZEBO_MODEL_FILE, "robot_model"),
            artifact(directory / ADAPTER_FILE, "adapter_config"),
        ]
        write_json(directory / "runtime.json", runtime)
        fixture = {
            "kind": "rne_openarm_transmission_efficiency_fixture",
            "schema_version": 1,
            "case_id": identifier,
            "controlled_joint": joint_name,
            "transmission_efficiency": efficiency,
            "fixed_plant_coulomb_friction_nm": 0.5,
            "fixed_plant_coulomb_transition_velocity_rad_s": baseline_coulomb[
                "plant_coulomb_transition_velocity_rad_s"
            ],
            "transmission_loss_application_order": (
                "after_motor_effort_and_speed_limit_before_joint_effort_and_passive_loss"
            ),
            "portable_realized_efficiency": next(
                joint["transmission_efficiency"]
                for joint in load(directory / ACTUATION_FILE)["joints"]
                if joint["joint_name"] == joint_name
            ),
            "gazebo_realized_efficiency": load(directory / ADAPTER_FILE)[
                "transmission_efficiency_by_joint"
            ][source_adapter["joint_order"].index(joint_name)],
            "fixed_artifact_sha256": fixed_hashes,
            "portable_model_urdf_sha256": sha256(directory / PORTABLE_MODEL_FILE),
            "gazebo_runtime_model_urdf_sha256": sha256(directory / GAZEBO_MODEL_FILE),
            "world_sha256": sha256(directory / WORLD_FILE),
            "robot_asset_config_sha256": sha256(directory / ROBOT_FILE),
            "scene_config_sha256": sha256(directory / SCENE_FILE),
            "actuation_config_sha256": sha256(directory / ACTUATION_FILE),
            "adapter_config_sha256": sha256(directory / ADAPTER_FILE),
            "runtime_manifest_sha256": sha256(directory / "runtime.json"),
        }
        write_json(directory / "transmission-efficiency-fixture.json", fixture)
        cases.append(fixture)
    suite = {
        "kind": "rne_openarm_transmission_efficiency_suite",
        "schema_version": 1,
        "experiment_id": manifest["experiment_id"],
        "controlled_joint": joint_name,
        "backend_order": manifest["backend_order"],
        "declared_minimum_supported_efficiency": manifest[
            "declared_minimum_supported_efficiency"
        ],
        "requirements": manifest["requirements"],
        "parameter_semantics": manifest["parameter_semantics"],
        "boundary_rule": manifest["boundary_rule"],
        "inputs": {
            "experiment_manifest_sha256": sha256(manifest_path),
            "baseline_fixture_sha256": sha256(baseline / "coulomb-friction-fixture.json"),
            "source_actuation_config_sha256": sha256(baseline / ACTUATION_FILE),
            "source_adapter_config_sha256": sha256(baseline / ADAPTER_FILE),
        },
        "cases": cases,
    }
    write_json(output / "transmission-efficiency-suite.json", suite)
    return suite


def main() -> None:
    args = parse_args()
    suite = compile_suite(args.manifest, args.baseline_fixture, args.output)
    print(f"wrote {len(suite['cases'])} OpenArm transmission fixtures to {args.output}")


if __name__ == "__main__":
    main()

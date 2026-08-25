#!/usr/bin/env python3
"""Compile portable OpenArm regularized-Coulomb fixtures and Gazebo derivations."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
from typing import Any

from build_openarm_joint_loss_suite import build_urdf, joint_dynamics


SOURCE_MODEL_FILE = "openarm_v2_right.payload.urdf"
PORTABLE_MODEL_FILE = "openarm_v2_right.coulomb.urdf"
GAZEBO_MODEL_FILE = SOURCE_MODEL_FILE
WORLD_FILE = "openarm_payload.world.sdf"
ROBOT_FILE = "openarm_payload.rne.robot.toml"
SCENE_FILE = "openarm_payload.rne.scene.toml"
ACTUATION_FILE = "openarm_right.rne_actuation.json"
ADAPTER_FILE = "openarm_right.adapter.json"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--baseline-fixture", required=True, type=Path)
    parser.add_argument(
        "--manifest", type=Path, default=root / "openarm_coulomb_friction_experiments.json"
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


def case_id(friction_nm: float) -> str:
    milli_nm = round(friction_nm * 1000.0)
    return f"joint5-coulomb-{milli_nm:04d}mn"


def validate_manifest(manifest: dict[str, Any]) -> None:
    grid = manifest.get("coulomb_friction_grid_nm")
    transition = manifest.get("coulomb_transition_velocity_rad_s")
    damping = manifest.get("viscous_damping_nm_s_per_rad")
    supported = manifest.get("declared_supported_coulomb_friction_nm")
    if (
        manifest.get("kind") != "rne_openarm_coulomb_friction_experiment_manifest"
        or manifest.get("schema_version") != 1
        or not isinstance(grid, list)
        or len(grid) < 3
        or grid != sorted(set(grid))
        or grid[0] != 0.0
        or any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 0.0
            for value in grid
        )
        or not isinstance(transition, (int, float))
        or not math.isfinite(transition)
        or transition <= 0.0
        or not isinstance(damping, (int, float))
        or not math.isfinite(damping)
        or damping < 0.0
        or supported not in grid
        or supported == grid[-1]
    ):
        raise ValueError("unsupported Coulomb-friction experiment manifest")


def portable_robot_config(source: str) -> str:
    needle = f'path = "{SOURCE_MODEL_FILE}"'
    if source.count(needle) != 1:
        raise ValueError("baseline robot config does not name the source model exactly once")
    return source.replace(needle, f'path = "{PORTABLE_MODEL_FILE}"')


def adapter_config(
    source: dict[str, Any], joint_name: str, friction_nm: float, transition_rad_s: float
) -> dict[str, Any]:
    config = json.loads(json.dumps(source))
    joints = config.get("joint_order")
    effort_indices = config.get("effort_joint_indices")
    if (
        config.get("actuation_mode") != "effort_pd"
        or not isinstance(joints, list)
        or joints.count(joint_name) != 1
        or not isinstance(effort_indices, list)
    ):
        raise ValueError("baseline Gazebo adapter is not the qualified effort fixture")
    index = joints.index(joint_name)
    if index not in effort_indices:
        raise ValueError("controlled joint is not effort-controlled")
    config["plant_coulomb_friction_nm"] = [0.0] * len(joints)
    config["plant_coulomb_transition_velocity_rad_s"] = [0.0] * len(joints)
    config["plant_coulomb_friction_nm"][index] = friction_nm
    config["plant_coulomb_transition_velocity_rad_s"][index] = transition_rad_s
    config["plant_loss_application_order"] = (
        "actuator_effort_clamp_then_regularized_coulomb_then_gazebo_set_force"
    )
    return config


def compile_suite(
    manifest_path: Path, baseline: Path, actuation_config_path: Path, output: Path
) -> dict[str, Any]:
    manifest = load(manifest_path)
    validate_manifest(manifest)
    required = (
        SOURCE_MODEL_FILE,
        WORLD_FILE,
        ROBOT_FILE,
        SCENE_FILE,
        ADAPTER_FILE,
        "runtime.json",
    )
    for name in required:
        if not (baseline / name).is_file():
            raise ValueError(f"baseline fixture is missing {name}")
    if not actuation_config_path.is_file():
        raise ValueError("source actuation config is missing")
    source_model = baseline / SOURCE_MODEL_FILE
    source_bytes = source_model.read_bytes()
    joint_name = manifest["controlled_joint"]
    if joint_dynamics(source_bytes, joint_name) is not None:
        raise ValueError("baseline controlled joint must not declare plant dynamics")
    baseline_runtime = load(baseline / "runtime.json")
    baseline_adapter = load(baseline / ADAPTER_FILE)
    cases = []
    damping = float(manifest["viscous_damping_nm_s_per_rad"])
    transition = float(manifest["coulomb_transition_velocity_rad_s"])
    for raw_friction in manifest["coulomb_friction_grid_nm"]:
        friction = float(raw_friction)
        identifier = case_id(friction)
        directory = output / identifier
        directory.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(baseline / WORLD_FILE, directory / WORLD_FILE)
        shutil.copyfile(baseline / SCENE_FILE, directory / SCENE_FILE)
        shutil.copyfile(actuation_config_path, directory / ACTUATION_FILE)

        portable_path = directory / PORTABLE_MODEL_FILE
        portable_path.write_bytes(build_urdf(source_bytes, joint_name, damping, friction))
        gazebo_path = directory / GAZEBO_MODEL_FILE
        gazebo_path.write_bytes(build_urdf(source_bytes, joint_name, damping, 0.0))
        robot_text = portable_robot_config((baseline / ROBOT_FILE).read_text(encoding="utf-8"))
        (directory / ROBOT_FILE).write_text(robot_text, encoding="utf-8")
        write_json(directory / ADAPTER_FILE, adapter_config(baseline_adapter, joint_name, friction, transition))

        runtime = json.loads(json.dumps(baseline_runtime))
        runtime["artifacts"] = [
            artifact(directory / WORLD_FILE, "world"),
            artifact(gazebo_path, "robot_model"),
            artifact(directory / ADAPTER_FILE, "adapter_config"),
        ]
        write_json(directory / "runtime.json", runtime)
        portable_dynamics = joint_dynamics(portable_path.read_bytes(), joint_name)
        gazebo_dynamics = joint_dynamics(gazebo_path.read_bytes(), joint_name)
        fixture = {
            "kind": "rne_openarm_coulomb_friction_fixture",
            "schema_version": 1,
            "case_id": identifier,
            "controlled_joint": joint_name,
            "plant_viscous_damping_nm_s_per_rad": damping,
            "plant_coulomb_friction_nm": friction,
            "plant_coulomb_transition_velocity_rad_s": transition,
            "portable_model_realized_dynamics": list(portable_dynamics or (0.0, 0.0)),
            "gazebo_runtime_model_realized_dynamics": list(gazebo_dynamics or (0.0, 0.0)),
            "gazebo_adapter_realized_coulomb_friction_nm": friction,
            "gazebo_adapter_realized_transition_velocity_rad_s": transition,
            "gazebo_derivation": manifest["gazebo_derivation"],
            "source_model_sha256": sha256(source_model),
            "portable_model_urdf_sha256": sha256(portable_path),
            "gazebo_runtime_model_urdf_sha256": sha256(gazebo_path),
            "world_sha256": sha256(directory / WORLD_FILE),
            "robot_asset_config_sha256": sha256(directory / ROBOT_FILE),
            "scene_config_sha256": sha256(directory / SCENE_FILE),
            "actuation_config_sha256": sha256(directory / ACTUATION_FILE),
            "adapter_config_sha256": sha256(directory / ADAPTER_FILE),
            "runtime_manifest_sha256": sha256(directory / "runtime.json"),
        }
        write_json(directory / "coulomb-friction-fixture.json", fixture)
        cases.append(fixture)

    suite = {
        "kind": "rne_openarm_coulomb_friction_suite",
        "schema_version": 1,
        "experiment_id": manifest["experiment_id"],
        "controlled_joint": joint_name,
        "backend_order": manifest["backend_order"],
        "declared_supported_coulomb_friction_nm": manifest[
            "declared_supported_coulomb_friction_nm"
        ],
        "requirements": manifest["requirements"],
        "parameter_semantics": manifest["parameter_semantics"],
        "gazebo_derivation": manifest["gazebo_derivation"],
        "boundary_rule": manifest["boundary_rule"],
        "inputs": {
            "experiment_manifest_sha256": sha256(manifest_path),
            "baseline_runtime_manifest_sha256": sha256(baseline / "runtime.json"),
            "source_model_sha256": sha256(source_model),
            "source_actuation_config_sha256": sha256(actuation_config_path),
            "source_adapter_config_sha256": sha256(baseline / ADAPTER_FILE),
        },
        "cases": cases,
    }
    write_json(output / "coulomb-friction-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_suite(
        args.manifest.resolve(),
        args.baseline_fixture.resolve(),
        args.actuation_config.resolve(),
        args.output.resolve(),
    )
    print(f"OpenArm Coulomb-friction suite: {len(suite['cases'])} cases -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

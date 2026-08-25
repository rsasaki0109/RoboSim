#!/usr/bin/env python3
"""Compile content-addressed OpenArm plant joint-loss fixtures."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import shutil
from typing import Any
import xml.etree.ElementTree as ET


COPIED_FILES = (
    "openarm_payload.rne.robot.toml",
    "openarm_payload.rne.scene.toml",
    "openarm_right.adapter.json",
)
MODEL_FILE = "openarm_v2_right.payload.urdf"
WORLD_FILE = "openarm_payload.world.sdf"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--baseline-fixture", required=True, type=Path)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root / "openarm_joint_loss_experiments.json",
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


def case_id(damping_nm_s_per_rad: float) -> str:
    milli_units = round(damping_nm_s_per_rad * 1000.0)
    return f"joint5-damping-{milli_units:05d}mnms-per-rad"


def artifact(path: Path, role: str) -> dict[str, Any]:
    return {
        "role": role,
        "file": path.name,
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def joint_dynamics(urdf: bytes, joint_name: str) -> tuple[float, float] | None:
    root = ET.fromstring(urdf)
    matches = [joint for joint in root.findall("joint") if joint.get("name") == joint_name]
    if len(matches) != 1:
        raise ValueError(f"expected exactly one controlled joint named {joint_name}")
    dynamics = matches[0].find("dynamics")
    if dynamics is None:
        return None
    return (
        float(dynamics.get("damping", "0")),
        float(dynamics.get("friction", "0")),
    )


def build_urdf(
    base_urdf: bytes,
    joint_name: str,
    damping_nm_s_per_rad: float,
    friction_nm: float,
) -> bytes:
    existing = joint_dynamics(base_urdf, joint_name)
    if existing is not None:
        raise ValueError(f"controlled joint {joint_name} already declares dynamics")
    if damping_nm_s_per_rad == 0.0 and friction_nm == 0.0:
        return base_urdf

    text = base_urdf.decode("utf-8")
    escaped = re.escape(joint_name)
    pattern = re.compile(
        rf"(<joint\b[^>]*\bname=[\"']{escaped}[\"'][^>]*>)(.*?)(</joint>)",
        re.DOTALL,
    )
    matches = list(pattern.finditer(text))
    if len(matches) != 1:
        raise ValueError(f"could not isolate controlled joint {joint_name}")
    match = matches[0]
    indent_match = re.search(r"\n([ \t]+)<", match.group(2))
    indent = indent_match.group(1) if indent_match else "    "
    dynamics = (
        f'\n{indent}<dynamics damping="{damping_nm_s_per_rad:.12g}" '
        f'friction="{friction_nm:.12g}"/>'
    )
    body = match.group(2).rstrip() + dynamics + "\n"
    result = text[: match.start()] + match.group(1) + body + match.group(3) + text[match.end() :]
    encoded = result.encode("utf-8")
    if joint_dynamics(encoded, joint_name) != (damping_nm_s_per_rad, friction_nm):
        raise ValueError("generated joint dynamics do not match requested SI values")
    return encoded


def compile_suite(
    manifest_path: Path,
    baseline_fixture: Path,
    actuation_config_path: Path,
    output: Path,
) -> dict[str, Any]:
    manifest = load(manifest_path)
    grid = manifest.get("viscous_damping_grid_nm_s_per_rad")
    friction = manifest.get("coulomb_friction_nm")
    if (
        manifest.get("kind") != "rne_openarm_joint_loss_experiment_manifest"
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
        or not isinstance(friction, (int, float))
        or not math.isfinite(friction)
        or friction != 0.0
    ):
        raise ValueError("unsupported joint-loss experiment manifest")

    source_model = baseline_fixture / MODEL_FILE
    source_world = baseline_fixture / WORLD_FILE
    source_runtime = baseline_fixture / "runtime.json"
    for path in (source_model, source_world, source_runtime):
        if not path.is_file():
            raise ValueError(f"baseline fixture is missing {path.name}")
    for name in COPIED_FILES:
        if not (baseline_fixture / name).is_file():
            raise ValueError(f"baseline fixture is missing {name}")
    if not actuation_config_path.is_file():
        raise ValueError("source actuation config is missing")

    baseline_urdf = source_model.read_bytes()
    joint = manifest["controlled_joint"]
    if joint_dynamics(baseline_urdf, joint) is not None:
        raise ValueError("baseline controlled joint must not declare plant dynamics")
    baseline_runtime = load(source_runtime)
    cases = []
    for raw_damping in grid:
        damping = float(raw_damping)
        identifier = case_id(damping)
        directory = output / identifier
        directory.mkdir(parents=True, exist_ok=True)
        for name in COPIED_FILES:
            shutil.copyfile(baseline_fixture / name, directory / name)
        shutil.copyfile(
            actuation_config_path, directory / "openarm_right.rne_actuation.json"
        )
        shutil.copyfile(source_world, directory / WORLD_FILE)
        model_path = directory / MODEL_FILE
        model_path.write_bytes(build_urdf(baseline_urdf, joint, damping, float(friction)))

        runtime = json.loads(json.dumps(baseline_runtime))
        runtime["artifacts"] = [
            artifact(directory / WORLD_FILE, "world"),
            artifact(model_path, "robot_model"),
            artifact(directory / "openarm_right.adapter.json", "adapter_config"),
        ]
        runtime_path = directory / "runtime.json"
        write_json(runtime_path, runtime)
        realized = joint_dynamics(model_path.read_bytes(), joint)
        realized_damping, realized_friction = realized or (0.0, 0.0)
        fixture = {
            "kind": "rne_openarm_joint_loss_fixture",
            "schema_version": 1,
            "case_id": identifier,
            "controlled_joint": joint,
            "plant_viscous_damping_nm_s_per_rad": damping,
            "plant_coulomb_friction_nm": float(friction),
            "realized_viscous_damping_nm_s_per_rad": realized_damping,
            "realized_coulomb_friction_nm": realized_friction,
            "dynamics_element_present": realized is not None,
            "source_model_sha256": sha256(source_model),
            "model_urdf_sha256": sha256(model_path),
            "world_sha256": sha256(directory / WORLD_FILE),
            "robot_asset_config_sha256": sha256(directory / "openarm_payload.rne.robot.toml"),
            "scene_config_sha256": sha256(directory / "openarm_payload.rne.scene.toml"),
            "actuation_config_sha256": sha256(directory / "openarm_right.rne_actuation.json"),
            "adapter_config_sha256": sha256(directory / "openarm_right.adapter.json"),
            "runtime_manifest_sha256": sha256(runtime_path),
        }
        write_json(directory / "joint-loss-fixture.json", fixture)
        cases.append(fixture)

    suite = {
        "kind": "rne_openarm_joint_loss_suite",
        "schema_version": 1,
        "experiment_id": manifest["experiment_id"],
        "controlled_joint": joint,
        "backend_order": manifest["backend_order"],
        "declared_supported_viscous_damping_nm_s_per_rad": manifest[
            "declared_supported_viscous_damping_nm_s_per_rad"
        ],
        "requirements": manifest["requirements"],
        "parameter_semantics": manifest["parameter_semantics"],
        "inputs": {
            "experiment_manifest_sha256": sha256(manifest_path),
            "baseline_runtime_manifest_sha256": sha256(source_runtime),
            "source_model_sha256": sha256(source_model),
            "source_actuation_config_sha256": sha256(
                actuation_config_path
            ),
            "source_adapter_config_sha256": sha256(
                baseline_fixture / "openarm_right.adapter.json"
            ),
        },
        "cases": cases,
    }
    write_json(output / "joint-loss-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_suite(
        args.manifest.resolve(),
        args.baseline_fixture.resolve(),
        args.actuation_config.resolve(),
        args.output.resolve(),
    )
    print(f"OpenArm joint-loss suite: {len(suite['cases'])} cases -> {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

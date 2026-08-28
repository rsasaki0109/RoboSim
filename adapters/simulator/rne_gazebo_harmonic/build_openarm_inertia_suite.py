#!/usr/bin/env python3
"""Compile OpenArm link-inertia fixtures without changing mass or center of mass."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import shutil
from typing import Any
import xml.etree.ElementTree as ET


PORTABLE_SOURCE_FILE = "openarm_v2_right.coulomb.urdf"
PORTABLE_MODEL_FILE = "openarm_v2_right.inertia.urdf"
GAZEBO_MODEL_FILE = "openarm_v2_right.payload.urdf"
WORLD_FILE = "openarm_payload.world.sdf"
ROBOT_FILE = "openarm_payload.rne.robot.toml"
SCENE_FILE = "openarm_payload.rne.scene.toml"
ACTUATION_FILE = "openarm_right.rne_actuation.json"
ADAPTER_FILE = "openarm_right.adapter.json"
TENSOR_NAMES = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-fixture", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--manifest", type=Path, default=root / "openarm_inertia_experiments.json"
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


def case_id(scale: float) -> str:
    if not float(scale).is_integer() or scale < 1.0:
        raise ValueError("inertia scales must be positive whole-number multipliers")
    return f"joint5-inertia-{int(scale):02d}x"


def validate_manifest(manifest: dict[str, Any]) -> None:
    grid = manifest.get("inertia_scale_grid")
    supported = manifest.get("declared_supported_inertia_scale")
    if (
        manifest.get("kind") != "rne_openarm_inertia_experiment_manifest"
        or manifest.get("schema_version") != 1
        or not isinstance(grid, list)
        or len(grid) < 3
        or grid != sorted(set(grid))
        or grid[0] != 1.0
        or any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value < 1.0
            or not float(value).is_integer()
            for value in grid
        )
        or supported not in grid
        or supported == grid[-1]
        or not isinstance(manifest.get("controlled_joint"), str)
        or not isinstance(manifest.get("inertia_link"), str)
    ):
        raise ValueError("unsupported inertia experiment manifest")


def inertial(root: ET.Element, link_name: str) -> tuple[ET.Element, ET.Element, ET.Element]:
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"inertia link {link_name!r} is absent")
    node = link.find("inertial")
    if node is None:
        raise ValueError("inertia link has no inertial")
    origin = node.find("origin")
    mass = node.find("mass")
    tensor = node.find("inertia")
    if origin is None or mass is None or tensor is None:
        raise ValueError("inertia link has an incomplete inertial")
    return origin, mass, tensor


def tensor_values(tensor: ET.Element) -> dict[str, float]:
    try:
        values = {name: float(tensor.attrib[name]) for name in TENSOR_NAMES}
    except (KeyError, ValueError) as error:
        raise ValueError("inertia tensor is incomplete or non-numeric") from error
    if any(not math.isfinite(value) for value in values.values()):
        raise ValueError("inertia tensor must be finite")
    return values


def validate_physical_tensor(values: dict[str, float]) -> None:
    ixx, ixy, ixz = values["ixx"], values["ixy"], values["ixz"]
    iyy, iyz, izz = values["iyy"], values["iyz"], values["izz"]
    determinant_2 = ixx * iyy - ixy * ixy
    determinant_3 = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    tolerance = 1e-15
    if (
        ixx <= 0.0
        or determinant_2 <= 0.0
        or determinant_3 <= 0.0
        or ixx + iyy + tolerance < izz
        or ixx + izz + tolerance < iyy
        or iyy + izz + tolerance < ixx
    ):
        raise ValueError("inertia tensor must be positive definite and physically realizable")


def read_inertial(model: bytes, link_name: str) -> dict[str, Any]:
    root = ET.fromstring(model.decode("utf-8"))
    origin, mass, tensor = inertial(root, link_name)
    values = tensor_values(tensor)
    validate_physical_tensor(values)
    xyz = [float(item) for item in origin.attrib["xyz"].split()]
    if len(xyz) != 3 or any(not math.isfinite(value) for value in xyz):
        raise ValueError("inertial center of mass must be a finite three-vector")
    mass_kg = float(mass.attrib["value"])
    if not math.isfinite(mass_kg) or mass_kg <= 0.0:
        raise ValueError("inertial mass must be finite and positive")
    return {"mass_kg": mass_kg, "center_of_mass_m": xyz, "tensor_kg_m2": values}


def scale_inertia(model: bytes, link_name: str, scale: float) -> bytes:
    text = model.decode("utf-8").replace("\r\n", "\n")
    root = ET.fromstring(text)
    _, _, tensor = inertial(root, link_name)
    original = tensor_values(tensor)
    validate_physical_tensor(original)
    scaled = {name: value * scale for name, value in original.items()}
    validate_physical_tensor(scaled)
    tensor.attrib.update({name: format(value, ".17g") for name, value in scaled.items()})
    ET.indent(root, space="  ")
    return ("<?xml version=\"1.0\"?>\n" + ET.tostring(root, encoding="unicode") + "\n").encode(
        "utf-8"
    )


def portable_robot_config(source: str) -> str:
    needle = f'path = "{PORTABLE_SOURCE_FILE}"'
    if source.count(needle) != 1:
        raise ValueError("baseline robot config must name the Coulomb model exactly once")
    return source.replace(needle, f'path = "{PORTABLE_MODEL_FILE}"')


def compile_suite(manifest_path: Path, baseline: Path, output: Path) -> dict[str, Any]:
    manifest = load(manifest_path)
    validate_manifest(manifest)
    required = (
        PORTABLE_SOURCE_FILE,
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
    if (
        baseline_coulomb.get("plant_coulomb_friction_nm") != 0.5
        or baseline_coulomb.get("controlled_joint") != manifest["controlled_joint"]
    ):
        raise ValueError("inertia suite requires the qualified 0.5 N*m Coulomb fixture")

    link_name = manifest["inertia_link"]
    portable_source = (baseline / PORTABLE_SOURCE_FILE).read_bytes()
    gazebo_source = (baseline / GAZEBO_MODEL_FILE).read_bytes()
    portable_base = read_inertial(portable_source, link_name)
    gazebo_base = read_inertial(gazebo_source, link_name)
    if portable_base != gazebo_base:
        raise ValueError("portable and Gazebo baseline inertials differ")

    baseline_runtime = load(baseline / "runtime.json")
    cases = []
    for raw_scale in manifest["inertia_scale_grid"]:
        scale = float(raw_scale)
        identifier = case_id(scale)
        directory = output / identifier
        directory.mkdir(parents=True, exist_ok=True)
        for name in (WORLD_FILE, SCENE_FILE, ACTUATION_FILE, ADAPTER_FILE):
            shutil.copyfile(baseline / name, directory / name)

        portable_path = directory / PORTABLE_MODEL_FILE
        gazebo_path = directory / GAZEBO_MODEL_FILE
        portable_path.write_bytes(scale_inertia(portable_source, link_name, scale))
        gazebo_path.write_bytes(scale_inertia(gazebo_source, link_name, scale))
        (directory / ROBOT_FILE).write_text(
            portable_robot_config((baseline / ROBOT_FILE).read_text(encoding="utf-8")),
            encoding="utf-8",
        )
        runtime = json.loads(json.dumps(baseline_runtime))
        runtime["artifacts"] = [
            artifact(directory / WORLD_FILE, "world"),
            artifact(gazebo_path, "robot_model"),
            artifact(directory / ADAPTER_FILE, "adapter_config"),
        ]
        write_json(directory / "runtime.json", runtime)
        portable_realized = read_inertial(portable_path.read_bytes(), link_name)
        gazebo_realized = read_inertial(gazebo_path.read_bytes(), link_name)
        fixture = {
            "kind": "rne_openarm_inertia_fixture",
            "schema_version": 1,
            "case_id": identifier,
            "controlled_joint": manifest["controlled_joint"],
            "inertia_link": link_name,
            "inertia_scale": scale,
            "baseline_inertial": portable_base,
            "portable_realized_inertial": portable_realized,
            "gazebo_realized_inertial": gazebo_realized,
            "fixed_plant_coulomb_friction_nm": 0.5,
            "fixed_plant_coulomb_transition_velocity_rad_s": baseline_coulomb[
                "plant_coulomb_transition_velocity_rad_s"
            ],
            "source_portable_model_sha256": sha256(baseline / PORTABLE_SOURCE_FILE),
            "source_gazebo_model_sha256": sha256(baseline / GAZEBO_MODEL_FILE),
            "portable_model_urdf_sha256": sha256(portable_path),
            "gazebo_runtime_model_urdf_sha256": sha256(gazebo_path),
            "world_sha256": sha256(directory / WORLD_FILE),
            "robot_asset_config_sha256": sha256(directory / ROBOT_FILE),
            "scene_config_sha256": sha256(directory / SCENE_FILE),
            "actuation_config_sha256": sha256(directory / ACTUATION_FILE),
            "adapter_config_sha256": sha256(directory / ADAPTER_FILE),
            "runtime_manifest_sha256": sha256(directory / "runtime.json"),
        }
        write_json(directory / "inertia-fixture.json", fixture)
        cases.append(fixture)

    suite = {
        "kind": "rne_openarm_inertia_suite",
        "schema_version": 1,
        "experiment_id": manifest["experiment_id"],
        "controlled_joint": manifest["controlled_joint"],
        "inertia_link": link_name,
        "backend_order": manifest["backend_order"],
        "declared_supported_inertia_scale": manifest["declared_supported_inertia_scale"],
        "requirements": manifest["requirements"],
        "parameter_semantics": manifest["parameter_semantics"],
        "boundary_rule": manifest["boundary_rule"],
        "inputs": {
            "experiment_manifest_sha256": sha256(manifest_path),
            "baseline_fixture_sha256": sha256(baseline / "coulomb-friction-fixture.json"),
            "source_portable_model_sha256": sha256(baseline / PORTABLE_SOURCE_FILE),
            "source_gazebo_model_sha256": sha256(baseline / GAZEBO_MODEL_FILE),
            "source_actuation_config_sha256": sha256(baseline / ACTUATION_FILE),
            "source_adapter_config_sha256": sha256(baseline / ADAPTER_FILE),
        },
        "cases": cases,
    }
    write_json(output / "inertia-suite.json", suite)
    return suite


def main() -> None:
    args = parse_args()
    suite = compile_suite(args.manifest, args.baseline_fixture, args.output)
    print(f"wrote {len(suite['cases'])} OpenArm inertia fixtures to {args.output}")


if __name__ == "__main__":
    main()

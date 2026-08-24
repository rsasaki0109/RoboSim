#!/usr/bin/env python3
"""Build deterministic OpenArm payload URDF fixtures for all simulator backends."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


def load_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def case_id(mass_kg: float) -> str:
    grams = round(mass_kg * 1000.0)
    if not math.isclose(mass_kg, grams / 1000.0, abs_tol=1e-12):
        raise ValueError("payload masses must resolve to whole grams")
    return f"payload-{grams:04d}g"


def box_inertia(mass_kg: float, size_m: list[float]) -> dict[str, float]:
    x_m, y_m, z_m = size_m
    return {
        "ixx_kg_m2": mass_kg * (y_m * y_m + z_m * z_m) / 12.0,
        "iyy_kg_m2": mass_kg * (x_m * x_m + z_m * z_m) / 12.0,
        "izz_kg_m2": mass_kg * (x_m * x_m + y_m * y_m) / 12.0,
        "ixy_kg_m2": 0.0,
        "ixz_kg_m2": 0.0,
        "iyz_kg_m2": 0.0,
    }


def vector(values: list[float]) -> str:
    return " ".join(format(value, ".17g") for value in values)


def parse_vector(value: str) -> list[float]:
    result = [float(item) for item in value.split()]
    if len(result) != 3:
        raise ValueError("expected three-vector")
    return result


def parallel_axis(mass_kg: float, displacement_m: list[float]) -> list[list[float]]:
    squared_norm = sum(value * value for value in displacement_m)
    return [
        [
            mass_kg
            * (
                (squared_norm if row == column else 0.0)
                - displacement_m[row] * displacement_m[column]
            )
            for column in range(3)
        ]
        for row in range(3)
    ]


def add_matrix(*matrices: list[list[float]]) -> list[list[float]]:
    return [
        [sum(matrix[row][column] for matrix in matrices) for column in range(3)]
        for row in range(3)
    ]


def combined_inertial(
    base_mass_kg: float,
    base_com_m: list[float],
    base_inertia: dict[str, float],
    payload_mass_kg: float,
    payload_com_m: list[float],
    payload_size_m: list[float],
) -> tuple[float, list[float], dict[str, float]]:
    combined_mass_kg = base_mass_kg + payload_mass_kg
    combined_com_m = [
        (base_mass_kg * base_com_m[index] + payload_mass_kg * payload_com_m[index])
        / combined_mass_kg
        for index in range(3)
    ]
    base_matrix = [
        [base_inertia["ixx"], base_inertia["ixy"], base_inertia["ixz"]],
        [base_inertia["ixy"], base_inertia["iyy"], base_inertia["iyz"]],
        [base_inertia["ixz"], base_inertia["iyz"], base_inertia["izz"]],
    ]
    payload = box_inertia(payload_mass_kg, payload_size_m)
    payload_matrix = [
        [payload["ixx_kg_m2"], 0.0, 0.0],
        [0.0, payload["iyy_kg_m2"], 0.0],
        [0.0, 0.0, payload["izz_kg_m2"]],
    ]
    base_offset = [base_com_m[index] - combined_com_m[index] for index in range(3)]
    payload_offset = [payload_com_m[index] - combined_com_m[index] for index in range(3)]
    matrix = add_matrix(
        base_matrix,
        parallel_axis(base_mass_kg, base_offset),
        payload_matrix,
        parallel_axis(payload_mass_kg, payload_offset),
    )
    return combined_mass_kg, combined_com_m, {
        "ixx": matrix[0][0],
        "ixy": matrix[0][1],
        "ixz": matrix[0][2],
        "iyy": matrix[1][1],
        "iyz": matrix[1][2],
        "izz": matrix[2][2],
    }


def build_urdf(base_urdf: bytes, manifest: dict[str, Any], mass_kg: float) -> bytes:
    text = base_urdf.decode("utf-8").replace("\r\n", "\n")
    if "openarm_payload_visual" in text:
        raise ValueError("base URDF already contains the payload fixture")
    root = ET.fromstring(text)
    if mass_kg == 0.0:
        return text.encode("utf-8")
    link = root.find(f"./link[@name='{manifest['attachment_parent_link']}']")
    if link is None:
        raise ValueError("payload attachment parent link is absent")
    inertial = link.find("inertial")
    if inertial is None or inertial.find("origin") is None or inertial.find("mass") is None or inertial.find("inertia") is None:
        raise ValueError("payload attachment parent has no complete inertial")
    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    base_mass_kg = float(mass.attrib["value"])
    base_com_m = parse_vector(origin.attrib["xyz"])
    base_inertia = {name: float(inertia.attrib[name]) for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")}
    payload_com_m = [
        manifest["attachment_origin_xyz_m"][index]
        + manifest["payload_center_of_mass_local_m"][index]
        for index in range(3)
    ]
    combined_mass_kg, combined_com_m, combined_tensor = combined_inertial(
        base_mass_kg,
        base_com_m,
        base_inertia,
        mass_kg,
        payload_com_m,
        manifest["payload_box_size_m"],
    )
    origin.attrib.update({"xyz": vector(combined_com_m), "rpy": "0 0 0"})
    mass.attrib["value"] = format(combined_mass_kg, ".17g")
    inertia.attrib.update({name: format(value, ".17g") for name, value in combined_tensor.items()})
    visual = ET.Element("visual", {"name": "openarm_payload_visual"})
    ET.SubElement(visual, "origin", {"xyz": vector(payload_com_m), "rpy": "0 0 0"})
    geometry = ET.SubElement(visual, "geometry")
    ET.SubElement(geometry, "box", {"size": vector(manifest["payload_box_size_m"])})
    material = ET.SubElement(visual, "material", {"name": "openarm_payload_blue"})
    ET.SubElement(material, "color", {"rgba": "0.12 0.42 0.86 1"})
    inertial_index = list(link).index(inertial)
    link.insert(inertial_index, visual)
    ET.indent(root, space="  ")
    return ("<?xml version=\"1.0\"?>\n" + ET.tostring(root, encoding="unicode") + "\n").encode("utf-8")


def artifact(role: str, path: Path) -> dict[str, Any]:
    return {
        "role": role,
        "file": path.name,
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def write_text(path: Path, value: str) -> None:
    path.write_text(value, encoding="utf-8", newline="\n")


def compile_suite(
    manifest_path: Path,
    base_urdf_path: Path,
    adapter_config_path: Path,
    actuation_config_path: Path,
    output: Path,
) -> dict[str, Any]:
    manifest = load_json(manifest_path)
    masses = manifest.get("payload_mass_grid_kg")
    if (
        manifest.get("kind") != "rne_openarm_payload_experiment_manifest"
        or manifest.get("schema_version") != 1
        or not isinstance(masses, list)
        or len(masses) < 3
        or masses != sorted(set(masses))
        or masses[0] != 0.0
        or any(not isinstance(value, (int, float)) or value < 0 or not math.isfinite(value) for value in masses)
    ):
        raise ValueError("unsupported payload experiment manifest")
    for key in ("attachment_origin_xyz_m", "payload_box_size_m", "payload_center_of_mass_local_m"):
        values = manifest.get(key)
        if not isinstance(values, list) or len(values) != 3 or any(not math.isfinite(v) for v in values):
            raise ValueError(f"invalid {key}")
    if any(value <= 0 for value in manifest["payload_box_size_m"]):
        raise ValueError("payload box dimensions must be positive")
    base_urdf = base_urdf_path.read_bytes()
    adapter_config = load_json(adapter_config_path)
    actuation_config = load_json(actuation_config_path)
    joint_configs = {item["joint_name"]: item for item in actuation_config["joints"]}
    if set(joint_configs) != set(adapter_config["joint_order"]):
        raise ValueError("adapter and actuation joint orders differ")
    ordered_actuation = [joint_configs[name] for name in adapter_config["joint_order"]]
    substeps = manifest.get("gazebo_physics_substeps_per_control_step")
    if not isinstance(substeps, int) or substeps < 1:
        raise ValueError("invalid Gazebo physics substep count")
    stiffness_scale = manifest.get("gazebo_effort_stiffness_scale")
    damping_scale = manifest.get("gazebo_effort_damping_scale")
    if any(
        not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0.0
        for value in (stiffness_scale, damping_scale)
    ):
        raise ValueError("invalid Gazebo effort gain scale")
    adapter_config.update(
        {
            "actuation_mode": "effort_pd",
            "effort_joint_indices": list(range(7)),
            "physics_substeps_per_control_step": substeps,
            "stiffness_nm_per_rad": [
                item["stiffness_nm_per_rad"] * stiffness_scale
                for item in ordered_actuation
            ],
            "damping_nm_s_per_rad": [
                item["damping_nm_s_per_rad"] * damping_scale
                for item in ordered_actuation
            ],
            "maximum_effort_nm": [
                item["max_effort_nm"] for item in ordered_actuation
            ],
            "source_actuation_stiffness_scale": stiffness_scale,
            "source_actuation_damping_scale": damping_scale,
            "saturation_behavior": "clamp_each_joint_effort_before_pre_update",
            "failure_behavior": "reject_invalid_configuration_before_simulator_start",
        }
    )
    adapter_config_bytes = (json.dumps(adapter_config, indent=2) + "\n").encode("utf-8")
    cases = []
    for mass_kg in masses:
        identifier = case_id(float(mass_kg))
        case_dir = output / identifier
        case_dir.mkdir(parents=True, exist_ok=True)
        urdf_path = case_dir / "openarm_v2_right.payload.urdf"
        urdf_path.write_bytes(build_urdf(base_urdf, manifest, float(mass_kg)))
        robot_path = case_dir / "openarm_payload.rne.robot.toml"
        write_text(robot_path, '''kind = "urdf"
model_name = "openarm_v2_right"

[urdf]
path = "openarm_v2_right.payload.urdf"
base_body_type = "fixed"
initial_translation_m = [0.0, 0.698, 0.031]
initial_rotation_rpy = [-1.5707963267948966, 0.0, 0.0]
articulation = true
collisions = true
mesh_collisions = false
self_collisions = false
multibody = true
use_declared_inertial_masses = true
''')
        scene_path = case_dir / "openarm_payload.rne.scene.toml"
        write_text(scene_path, '''[world]
gravity_m_s2 = [0.0, -9.81, 0.0]
seed = 20260824

[ground]
enabled = false

[[robots]]
path = "openarm_payload.rne.robot.toml"
''')
        world_path = case_dir / "openarm_payload.world.sdf"
        write_text(world_path, '''<?xml version="1.0"?>
<sdf version="1.10">
  <world name="rne_openarm_payload">
    <physics name="fixed_step" type="ignored">
      <max_step_size>0.0016666667</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <include>
      <uri>openarm_v2_right.payload.urdf</uri>
      <name>openarm_v2_right</name>
    </include>
    <joint name="openarm_right_world_fixed_joint" type="fixed">
      <parent>world</parent>
      <child>openarm_v2_right::openarm_right_base_link</child>
    </joint>
  </world>
</sdf>
''')
        config_path = case_dir / "openarm_right.adapter.json"
        config_path.write_bytes(adapter_config_bytes)
        inertia = box_inertia(float(mass_kg), manifest["payload_box_size_m"]) if mass_kg else None
        fixture = {
            "kind": "rne_openarm_payload_fixture",
            "schema_version": 1,
            "case_id": identifier,
            "payload_present": mass_kg > 0.0,
            "payload_mass_kg": mass_kg,
            "payload_center_of_mass_local_m": manifest["payload_center_of_mass_local_m"] if mass_kg else None,
            "payload_inertia": inertia,
            "inertial_application": "lumped_into_openarm_right_ee_base_link",
            "gazebo_actuation_scope": manifest["gazebo_actuation_scope"],
            "attachment_parent_link": manifest["attachment_parent_link"],
            "attachment_origin_xyz_m": manifest["attachment_origin_xyz_m"],
            "source_model_sha256": sha256_bytes(base_urdf),
            "model_urdf_sha256": sha256(urdf_path),
            "robot_asset_config_sha256": sha256(robot_path),
            "scene_config_sha256": sha256(scene_path),
        }
        fixture_path = case_dir / "payload-fixture.json"
        write_text(fixture_path, json.dumps(fixture, indent=2) + "\n")
        runtime = {
            "kind": "rne_external_simulator_runtime_manifest",
            "schema_version": 1,
            "simulator_id": "gazebo_sim",
            "simulator_version": "8.15.0",
            "distribution": "harmonic_ubuntu_22.04_amd64",
            "fixed_delta_ticks": 16_666_667,
            "artifacts": [
                artifact("world", world_path),
                artifact("robot_model", urdf_path),
                artifact("adapter_config", config_path),
            ],
        }
        runtime_path = case_dir / "runtime.json"
        write_text(runtime_path, json.dumps(runtime, indent=2) + "\n")
        cases.append(
            {
                **fixture,
                "fixture_file": fixture_path.relative_to(output).as_posix(),
                "runtime_manifest_file": runtime_path.relative_to(output).as_posix(),
            }
        )
    suite = {
        "kind": "rne_openarm_payload_suite",
        "schema_version": 1,
        "experiment_id": manifest["experiment_id"],
        "backend_order": manifest["backend_order"],
        "controlled_joint": manifest["controlled_joint"],
        "requirements": manifest["requirements"],
        "declared_supported_payload_mass_kg": manifest["declared_supported_payload_mass_kg"],
        "inputs": {
            "experiment_manifest_sha256": sha256(manifest_path),
            "source_model_sha256": sha256(base_urdf_path),
            "adapter_config_sha256": sha256(adapter_config_path),
            "actuation_config_sha256": sha256(actuation_config_path),
        },
        "cases": cases,
    }
    output.mkdir(parents=True, exist_ok=True)
    write_text(output / "payload-suite.json", json.dumps(suite, indent=2) + "\n")
    return suite


def main() -> None:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_payload_experiments.json",
    )
    parser.add_argument(
        "--base-urdf",
        type=Path,
        default=root
        / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
    )
    parser.add_argument(
        "--adapter-config",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json",
    )
    parser.add_argument(
        "--actuation-config",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json",
    )
    args = parser.parse_args()
    suite = compile_suite(
        args.manifest,
        args.base_urdf,
        args.adapter_config,
        args.actuation_config,
        args.output,
    )
    print(f"OpenArm payload suite: {len(suite['cases'])} cases -> {args.output}")


if __name__ == "__main__":
    main()

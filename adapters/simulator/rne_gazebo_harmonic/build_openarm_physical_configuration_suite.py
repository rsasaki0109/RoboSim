#!/usr/bin/env python3
"""Compile the two official OpenArm right-arm product configurations."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any
import xml.etree.ElementTree as ET


TENSOR_NAMES = ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
MODEL_FILE = "openarm_v2_right.configuration.urdf"
ROBOT_FILE = "openarm_configuration.rne.robot.toml"
SCENE_FILE = "openarm_configuration.rne.scene.toml"
WORLD_FILE = "openarm_configuration.world.sdf"
ADAPTER_FILE = "openarm_right.adapter.json"
ACTUATION_FILE = "openarm_right.rne_actuation.json"
CONTROLLER_FILE = "openarm_physical_configuration.controller.json"
TASK_FILE = "openarm_physical_configuration.task.json"


def parse_args() -> argparse.Namespace:
    repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path(__file__).resolve().parent
        / "openarm_physical_configuration_experiments.json",
    )
    parser.add_argument("--repo-root", type=Path, default=repo)
    return parser.parse_args()


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def sha256(path: Path) -> str:
    return sha256_bytes(path.read_bytes())


def write_text(path: Path, value: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(value, encoding="utf-8", newline="\n")


def write_json(path: Path, value: dict[str, Any]) -> None:
    write_text(path, json.dumps(value, indent=2, allow_nan=False) + "\n")


def artifact(role: str, path: Path) -> dict[str, Any]:
    return {
        "role": role,
        "file": path.name,
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def tensor_values(node: ET.Element) -> dict[str, float]:
    try:
        values = {name: float(node.attrib[name]) for name in TENSOR_NAMES}
    except (KeyError, ValueError) as error:
        raise ValueError("inertia tensor is incomplete or non-numeric") from error
    if any(not math.isfinite(value) for value in values.values()):
        raise ValueError("inertia tensor must be finite")
    return values


def validate_physical_tensor(values: dict[str, float]) -> dict[str, float]:
    ixx, ixy, ixz = values["ixx"], values["ixy"], values["ixz"]
    iyy, iyz, izz = values["iyy"], values["iyz"], values["izz"]
    minor2 = ixx * iyy - ixy * ixy
    determinant = (
        ixx * (iyy * izz - iyz * iyz)
        - ixy * (ixy * izz - iyz * ixz)
        + ixz * (ixy * iyz - iyy * ixz)
    )
    tolerance = 1e-15
    if (
        ixx <= 0.0
        or minor2 <= 0.0
        or determinant <= 0.0
        or ixx + iyy + tolerance < izz
        or ixx + izz + tolerance < iyy
        or iyy + izz + tolerance < ixx
    ):
        raise ValueError("inertia tensor is not positive definite and realizable")
    return {"leading_principal_minor_2": minor2, "determinant": determinant}


def model_inertials(model: bytes) -> list[dict[str, Any]]:
    root = ET.fromstring(model.decode("utf-8"))
    result = []
    for link in root.findall("./link"):
        inertial = link.find("inertial")
        if inertial is None:
            continue
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        tensor = inertial.find("inertia")
        if origin is None or mass is None or tensor is None:
            raise ValueError(f"{link.attrib['name']} has incomplete inertial data")
        mass_kg = float(mass.attrib["value"])
        center = [float(value) for value in origin.attrib["xyz"].split()]
        if (
            not math.isfinite(mass_kg)
            or mass_kg <= 0.0
            or len(center) != 3
            or any(not math.isfinite(value) for value in center)
        ):
            raise ValueError(f"{link.attrib['name']} has invalid mass or center of mass")
        values = tensor_values(tensor)
        certificate = validate_physical_tensor(values)
        result.append(
            {
                "link": link.attrib["name"],
                "mass_kg": mass_kg,
                "center_of_mass_m": center,
                "tensor_kg_m2": values,
                "positive_definite_certificate": certificate,
            }
        )
    if not result:
        raise ValueError("model contains no inertial links")
    return result


def compile_model(source: bytes, configuration: dict[str, Any]) -> bytes:
    text = source.decode("utf-8").replace("\r\n", "\n")
    if not configuration["removed_links"] and not configuration["removed_joints"]:
        model_inertials(text.encode("utf-8"))
        return text.encode("utf-8")
    root = ET.fromstring(text)
    for joint_name in configuration["removed_joints"]:
        joint = root.find(f"./joint[@name='{joint_name}']")
        if joint is None:
            raise ValueError(f"source model is missing removable joint {joint_name}")
        root.remove(joint)
    for link_name in configuration["removed_links"]:
        link = root.find(f"./link[@name='{link_name}']")
        if link is None:
            raise ValueError(f"source model is missing removable link {link_name}")
        root.remove(link)
    for joint in root.findall("./joint"):
        if joint.find("parent").attrib["link"] in configuration["removed_links"] or joint.find(
            "child"
        ).attrib["link"] in configuration["removed_links"]:
            raise ValueError("compiled model retains a joint reference to a removed link")
    ET.indent(root, space="  ")
    compiled = (
        '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode") + "\n"
    ).encode("utf-8")
    model_inertials(compiled)
    return compiled


def validate_manifest(manifest: dict[str, Any], repo: Path) -> None:
    configurations = manifest.get("configurations")
    if (
        manifest.get("kind")
        != "rne_openarm_physical_configuration_experiment_manifest"
        or manifest.get("schema_version") != 1
        or manifest.get("configuration_order") != ["arm_only", "pinch_gripper"]
        or not isinstance(configurations, list)
        or [item.get("id") for item in configurations]
        != manifest["configuration_order"]
    ):
        raise ValueError("unsupported physical-configuration manifest")
    if manifest.get("controlled_joint_count") != 7:
        raise ValueError("physical-configuration experiment must control seven arm joints")
    for configuration in configurations:
        preset = repo / configuration["vendored_preset"]
        if not preset.is_file() or sha256(preset) != configuration["preset_sha256"]:
            raise ValueError(f"upstream preset hash drifted for {configuration['id']}")
        text = preset.read_text(encoding="utf-8")
        enabled = "primary_arm_end_effector:\n  type: end_effector\n  enabled: true" in text
        disabled = "primary_arm_end_effector:\n  type: end_effector\n  enabled: false" in text
        if enabled != configuration["primary_arm_end_effector_enabled"] or disabled == enabled:
            raise ValueError(f"upstream preset semantics drifted for {configuration['id']}")


def compile_suite(manifest_path: Path, repo: Path, output: Path) -> dict[str, Any]:
    manifest = load(manifest_path)
    validate_manifest(manifest, repo)
    source_model_path = repo / "assets/robots/openarm_description/openarm_v2_right.rne.urdf"
    adapter_source = repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json"
    controller_source = repo / manifest["source_controller"]
    task_source = repo / manifest["source_task_spec"]
    actuation_source = repo / manifest["source_actuation_config"]
    width = manifest["controlled_joint_count"]
    controller = load(controller_source)
    controller["controller_id"] = manifest["controller_id"]
    controller["task_id"] = manifest["task_id"]
    controller["action_joint_order"] = controller["action_joint_order"][:width]
    controller["rne_actuator_link_order"] = controller["rne_actuator_link_order"][:width]
    for keyframe in controller["keyframes"]:
        keyframe["joint_position_target_rad"] = keyframe[
            "joint_position_target_rad"
        ][:width]
    controller_path = output / CONTROLLER_FILE
    write_json(controller_path, controller)
    task = load(task_source)
    task["task_id"] = manifest["task_id"]
    for tensor in task["observation"]["tensors"]:
        tensor["shape"] = [width]
    for tensor in task["action"]["tensors"]:
        tensor["shape"] = [width]
    task["termination"]["max_episode_steps"] = controller["keyframes"][-1]["step"]
    task_path = output / TASK_FILE
    write_json(task_path, task)
    actuation = load(actuation_source)
    actuation["joints"] = actuation["joints"][:width]
    shared_actuation_path = output / ACTUATION_FILE
    write_json(shared_actuation_path, actuation)
    source = source_model_path.read_bytes()
    cases = []
    for configuration in manifest["configurations"]:
        case_dir = output / configuration["id"]
        case_dir.mkdir(parents=True, exist_ok=True)
        model_path = case_dir / MODEL_FILE
        model_path.write_bytes(compile_model(source, configuration))
        inertials = model_inertials(model_path.read_bytes())
        total_mass_kg = sum(item["mass_kg"] for item in inertials)
        expected_mass_kg = configuration["expected_articulated_mass_kg"]
        mass_delta_kg = abs(total_mass_kg - expected_mass_kg)
        if mass_delta_kg > 1e-12:
            raise ValueError(f"compiled mass drifted for {configuration['id']}")

        robot_path = case_dir / ROBOT_FILE
        write_text(
            robot_path,
            '''kind = "urdf"
model_name = "openarm_v2_right"

[urdf]
path = "openarm_v2_right.configuration.urdf"
base_body_type = "fixed"
initial_translation_m = [0.0, 0.698, 0.031]
initial_rotation_rpy = [-1.5707963267948966, 0.0, 0.0]
articulation = true
collisions = true
mesh_collisions = false
self_collisions = false
multibody = true
use_declared_inertial_masses = true
''',
        )
        scene_path = case_dir / SCENE_FILE
        write_text(
            scene_path,
            '''[world]
gravity_m_s2 = [0.0, -9.81, 0.0]
seed = 20260824

[ground]
enabled = false

[[robots]]
path = "openarm_configuration.rne.robot.toml"
''',
        )
        world_path = case_dir / WORLD_FILE
        write_text(
            world_path,
            '''<?xml version="1.0"?>
<sdf version="1.10">
  <world name="rne_openarm_physical_configuration">
    <physics name="fixed_step" type="ignored">
      <max_step_size>0.0016666667</max_step_size>
      <real_time_factor>0</real_time_factor>
    </physics>
    <gravity>0 0 -9.81</gravity>
    <include>
      <uri>openarm_v2_right.configuration.urdf</uri>
      <name>openarm_v2_right</name>
    </include>
    <joint name="openarm_right_world_fixed_joint" type="fixed">
      <parent>world</parent>
      <child>openarm_v2_right::openarm_right_base_link</child>
    </joint>
  </world>
</sdf>
''',
        )
        adapter = load(adapter_source)
        adapter["joint_order"] = adapter["joint_order"][:width]
        adapter_path = case_dir / ADAPTER_FILE
        write_json(adapter_path, adapter)
        actuation_path = case_dir / ACTUATION_FILE
        actuation_path.write_bytes(shared_actuation_path.read_bytes())
        fixture = {
            "kind": "rne_openarm_physical_configuration_fixture",
            "schema_version": 1,
            "case_id": configuration["id"],
            "upstream_repository": manifest["upstream_repository"],
            "upstream_commit": manifest["upstream_commit"],
            "upstream_preset": configuration["upstream_preset"],
            "vendored_preset_sha256": configuration["preset_sha256"],
            "primary_arm_end_effector_enabled": configuration[
                "primary_arm_end_effector_enabled"
            ],
            "removed_links": configuration["removed_links"],
            "removed_joints": configuration["removed_joints"],
            "inertials": inertials,
            "articulated_mass_kg": total_mass_kg,
            "expected_articulated_mass_kg": expected_mass_kg,
            "mass_realization_delta_kg": mass_delta_kg,
            "source_model_sha256": sha256(source_model_path),
            "model_urdf_sha256": sha256(model_path),
            "robot_asset_config_sha256": sha256(robot_path),
            "scene_config_sha256": sha256(scene_path),
            "actuation_config_sha256": sha256(actuation_path),
            "gazebo_adapter_config_sha256": sha256(adapter_path),
        }
        fixture_path = case_dir / "physical-configuration-fixture.json"
        write_json(fixture_path, fixture)
        runtime = {
            "kind": "rne_external_simulator_runtime_manifest",
            "schema_version": 1,
            "simulator_id": "gazebo_sim",
            "simulator_version": "8.15.0",
            "distribution": "harmonic_ubuntu_22.04_amd64",
            "fixed_delta_ticks": 16_666_667,
            "artifacts": [
                artifact("world", world_path),
                artifact("robot_model", model_path),
                artifact("adapter_config", adapter_path),
            ],
        }
        runtime_path = case_dir / "runtime.json"
        write_json(runtime_path, runtime)
        cases.append(
            {
                "case_id": configuration["id"],
                "fixture_file": fixture_path.relative_to(output).as_posix(),
                "runtime_manifest_file": runtime_path.relative_to(output).as_posix(),
                "model_urdf_sha256": fixture["model_urdf_sha256"],
                "articulated_mass_kg": total_mass_kg,
            }
        )
    mass_delta = cases[1]["articulated_mass_kg"] - cases[0]["articulated_mass_kg"]
    if abs(mass_delta - manifest["expected_gripper_mass_delta_kg"]) > 1e-12:
        raise ValueError("official configuration mass delta drifted")
    suite = {
        "kind": "rne_openarm_physical_configuration_suite",
        "schema_version": 1,
        "experiment_id": manifest["experiment_id"],
        "configuration_order": manifest["configuration_order"],
        "expected_gripper_mass_delta_kg": manifest["expected_gripper_mass_delta_kg"],
        "realized_gripper_mass_delta_kg": mass_delta,
        "inputs": {
            "manifest_sha256": sha256(manifest_path),
            "source_model_sha256": sha256(source_model_path),
            "source_controller_sha256": sha256(controller_source),
            "source_task_spec_sha256": sha256(task_source),
            "source_actuation_config_sha256": sha256(actuation_source),
            "controller_sha256": sha256(controller_path),
            "task_spec_sha256": sha256(task_path),
            "actuation_config_sha256": sha256(shared_actuation_path),
        },
        "cases": cases,
    }
    write_json(output / "physical-configuration-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_suite(args.manifest.resolve(), args.repo_root.resolve(), args.output.resolve())
    print(
        f"OpenArm physical configurations: cases={len(suite['cases'])} "
        f"gripper_mass_delta_kg={suite['realized_gripper_mass_delta_kg']:.12g}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

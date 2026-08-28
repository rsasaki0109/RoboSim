#!/usr/bin/env python3
"""Compile a fixed OpenArm actuator-bias robustness sweep."""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--dimension",
        choices=("actuator_target_bias", "joint_position_measurement_bias"),
        default="actuator_target_bias",
    )
    parser.add_argument(
        "--robustness-manifest",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_robustness_experiments.json",
    )
    parser.add_argument(
        "--plant-report",
        type=Path,
        default=root
        / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
    )
    parser.add_argument(
        "--plant-experiment-manifest",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_experiments.json",
    )
    parser.add_argument(
        "--limits-controller",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json",
    )
    parser.add_argument(
        "--requirements",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_controller_requirements.json",
    )
    return parser.parse_args()


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def load_controller_compiler(script_dir: Path) -> Any:
    path = script_dir / "build_openarm_controller_suite.py"
    spec = importlib.util.spec_from_file_location("rne_openarm_controller_compiler", path)
    if spec is None or spec.loader is None:
        raise ValueError("cannot load the OpenArm controller compiler")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def case_id(offset_rad: float) -> str:
    milliradians = round(offset_rad * 1000.0)
    if not math.isclose(offset_rad, milliradians / 1000.0, abs_tol=1e-12):
        raise ValueError("robustness offsets must resolve to whole milliradians")
    return f"bias-{milliradians:03d}mrad"


def sensor_case_id(offset_rad: float) -> str:
    return "sensor-" + case_id(offset_rad)


def compile_robustness_suite(
    compiler: Any,
    robustness_path: Path,
    plant_report_path: Path,
    plant_manifest_path: Path,
    limits_controller_path: Path,
    requirements_path: Path,
    dimension_id: str = "actuator_target_bias",
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    manifest = load(robustness_path)
    requirements = load(requirements_path)
    if (
        manifest.get("kind") != "rne_openarm_robustness_experiment_manifest"
        or manifest.get("schema_version") != 2
        or manifest.get("controller_role") != "state_feedback"
        or manifest.get("primary_sweep_backend") != "rne_rapier"
        or manifest.get("backend_order")
        != ["rne_rapier", "mujoco_native", "gazebo_sim"]
    ):
        raise ValueError("unsupported OpenArm robustness manifest")
    dimension = manifest.get("dimensions", {}).get(dimension_id, {})
    values = dimension.get("values")
    if (
        dimension_id not in {"actuator_target_bias", "joint_position_measurement_bias"}
        or not isinstance(values, list)
        or len(values) < 3
        or values != sorted(set(values))
        or values[0] != 0.0
        or not all(isinstance(value, float) and math.isfinite(value) for value in values)
    ):
        raise ValueError("invalid robustness grid")
    expected_kind = (
        "additive_actuator_target_bias_pulse_v1"
        if dimension_id == "actuator_target_bias"
        else "additive_joint_position_bias_pulse_v1"
    )
    expected_classification = (
        "actuator_realization_error"
        if dimension_id == "actuator_target_bias"
        else "measurement_error"
    )
    if (
        dimension.get("kind") != expected_kind
        or dimension.get("classification") != expected_classification
    ):
        raise ValueError("robustness dimension identity drifted")
    requirement_ids = {item["id"] for item in requirements.get("requirements", [])}
    if not set(manifest["evaluation"]["requirement_ids"]).issubset(requirement_ids):
        raise ValueError("robustness manifest names an unknown requirement")
    _, base_controllers = compiler.compile_suite(
        plant_report_path, plant_manifest_path, limits_controller_path
    )
    base = base_controllers["state_feedback"]
    controllers: dict[str, dict[str, Any]] = {}
    for offset_rad in values:
        identifier = (
            case_id(offset_rad)
            if dimension_id == "actuator_target_bias"
            else sensor_case_id(offset_rad)
        )
        controller = copy.deepcopy(base)
        controller["controller_id"] = (
            f"rne.controller.openarm_right.plant_state_feedback_integral.{identifier}.v1"
        )
        if dimension_id == "actuator_target_bias":
            contract = controller["disturbance_contract"]
            contract["offset_rad"] = offset_rad
            fields = (
                "kind",
                "classification",
                "joint",
                "start_step",
                "end_step",
                "application_order",
                "controller_visibility",
            )
        else:
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {key: value for key, value in dimension.items() if key not in {"unit", "values"}}
            contract["offset_rad"] = offset_rad
            controller["measurement_fault_contract"] = contract
            fields = (
                "kind",
                "classification",
                "joint",
                "start_controller_step",
                "end_controller_step",
                "sensor_status",
                "application_order",
                "controller_visibility",
            )
        for field in fields:
            if contract[field] != dimension[field]:
                raise ValueError(f"robustness contract field {field} drifted")
        controllers[identifier] = controller
    suite = {
        "kind": "rne_openarm_robustness_suite",
        "schema_version": 1,
        "suite_id": manifest["experiment_id"],
        "task_id": base["task_id"],
        "controller_role": manifest["controller_role"],
        "dimension_id": dimension_id,
        "primary_sweep_backend": manifest["primary_sweep_backend"],
        "backend_order": manifest["backend_order"],
        "dimension": dimension,
        "evaluation": manifest["evaluation"],
        "inputs": {
            "robustness_manifest_sha256": sha256(robustness_path),
            "plant_report_sha256": sha256(plant_report_path),
            "plant_experiment_manifest_sha256": sha256(plant_manifest_path),
            "limits_controller_sha256": sha256(limits_controller_path),
            "requirements_sha256": sha256(requirements_path),
        },
        "cases": [],
    }
    return suite, controllers


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    compiler = load_controller_compiler(Path(__file__).resolve().parent)
    suite, controllers = compile_robustness_suite(
        compiler,
        args.robustness_manifest.resolve(),
        args.plant_report.resolve(),
        args.plant_experiment_manifest.resolve(),
        args.limits_controller.resolve(),
        args.requirements.resolve(),
        args.dimension,
    )
    for identifier, controller in controllers.items():
        path = output / f"openarm-state-feedback-{identifier}.controller.json"
        write_json(path, controller)
        suite["cases"].append(
            {
                "case_id": identifier,
                "offset_rad": (
                    controller["disturbance_contract"]["offset_rad"]
                    if args.dimension == "actuator_target_bias"
                    else controller["measurement_fault_contract"]["offset_rad"]
                ),
                "controller_id": controller["controller_id"],
                "controller_path": path.name,
                "controller_sha256": sha256(path),
            }
        )
    write_json(output / "openarm-robustness-suite.json", suite)
    print(
        "OpenArm robustness suite: "
        f"cases={len(controllers)} primary_backend={suite['primary_sweep_backend']}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm robustness suite failed: {error}")
        raise SystemExit(2)

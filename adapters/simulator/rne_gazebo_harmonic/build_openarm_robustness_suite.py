#!/usr/bin/env python3
"""Compile a fixed OpenArm robustness sweep."""

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
        choices=(
            "actuator_target_bias",
            "actuator_command_delay",
            "actuator_command_rate_limit",
            "actuator_command_deadband",
            "joint_position_measurement_bias",
            "joint_feedback_publication_dropout",
            "joint_feedback_controller_ingress_latency",
            "joint_feedback_controller_ingress_jitter",
            "joint_feedback_controller_stale_age",
            "joint_feedback_dropout_recovery",
            "joint_feedback_repeated_dropout_rearm",
            "joint_position_measurement_quantization",
            "joint_position_measurement_saturation",
        ),
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


def dropout_case_id(consecutive_frames: int) -> str:
    return f"dropout-{consecutive_frames:03d}frames"


def sensor_latency_case_id(delay_frames: int) -> str:
    return f"sensor-latency-{delay_frames:03d}frames"


def sensor_jitter_case_id(jitter_frames: int) -> str:
    return f"sensor-jitter-{jitter_frames:03d}frames"


def sensor_stale_case_id(stale_frames: int) -> str:
    return f"sensor-stale-{stale_frames:03d}frames"


def sensor_recovery_case_id(additional_decisions: int) -> str:
    return f"sensor-recovery-{additional_decisions:03d}decisions"


def sensor_rearm_case_id(fresh_frames: int) -> str:
    return f"sensor-rearm-{fresh_frames:03d}fresh"


def sensor_quantization_case_id(step_rad: float) -> str:
    step_urad = round(step_rad * 1_000_000.0)
    if abs(step_urad / 1_000_000.0 - step_rad) > 1e-12:
        raise ValueError("sensor quantization grid must use whole microradians")
    return f"sensor-quantization-{step_urad:05d}urad"


def sensor_saturation_case_id(limit_rad: float) -> str:
    limit_mrad = round(limit_rad * 1000.0)
    if not math.isclose(limit_rad, limit_mrad / 1000.0, abs_tol=1e-12):
        raise ValueError("sensor saturation grid must use whole milliradians")
    return f"sensor-saturation-{limit_mrad:03d}mrad"


def delay_case_id(delay_steps: int) -> str:
    return f"delay-{delay_steps:03d}steps"


def rate_limit_case_id(maximum_rate_rad_s: float) -> str:
    milliradians_per_second = round(maximum_rate_rad_s * 1000.0)
    if not math.isclose(
        maximum_rate_rad_s, milliradians_per_second / 1000.0, abs_tol=1e-12
    ):
        raise ValueError("rate limits must resolve to whole milliradians per second")
    return f"rate-{milliradians_per_second:03d}mrad-s"


def deadband_case_id(deadband_rad: float) -> str:
    microradians = round(deadband_rad * 1_000_000.0)
    if not math.isclose(deadband_rad, microradians / 1_000_000.0, abs_tol=1e-12):
        raise ValueError("deadbands must resolve to whole microradians")
    return f"deadband-{microradians:04d}urad"


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
    supported_dimensions = {
        "actuator_target_bias",
        "actuator_command_delay",
        "actuator_command_rate_limit",
        "actuator_command_deadband",
        "joint_position_measurement_bias",
        "joint_feedback_publication_dropout",
        "joint_feedback_controller_ingress_latency",
        "joint_feedback_controller_ingress_jitter",
        "joint_feedback_controller_stale_age",
        "joint_feedback_dropout_recovery",
        "joint_feedback_repeated_dropout_rearm",
        "joint_position_measurement_quantization",
        "joint_position_measurement_saturation",
    }
    integer_dimension = dimension_id in {
        "actuator_command_delay",
        "joint_feedback_publication_dropout",
        "joint_feedback_controller_ingress_latency",
        "joint_feedback_controller_ingress_jitter",
        "joint_feedback_controller_stale_age",
        "joint_feedback_dropout_recovery",
        "joint_feedback_repeated_dropout_rearm",
    }
    if dimension_id in {
        "actuator_command_rate_limit",
        "joint_feedback_repeated_dropout_rearm",
        "joint_position_measurement_saturation",
    }:
        severity_order = {
            "actuator_command_rate_limit": "descending_maximum_rate_rad_s",
            "joint_feedback_repeated_dropout_rearm": "descending_interburst_fresh_frames",
            "joint_position_measurement_saturation": "descending_saturation_limit_abs_rad",
        }[dimension_id]
        grid_valid = (
            isinstance(values, list)
            and len(values) >= 3
            and values == sorted(set(values), reverse=True)
            and all(
                isinstance(value, (int, float))
                and not isinstance(value, bool)
                and math.isfinite(value)
                and (
                    value > 0.0
                    if dimension_id in {
                        "actuator_command_rate_limit",
                        "joint_position_measurement_saturation",
                    }
                    else isinstance(value, int) and value >= 0
                )
                for value in values
            )
            and dimension.get("severity_order") == severity_order
        )
    else:
        grid_valid = (
            isinstance(values, list)
            and len(values) >= 3
            and values == sorted(set(values))
            and values[0] == 0
            and all(
                isinstance(value, int) and value >= 0
                if integer_dimension
                else isinstance(value, float) and math.isfinite(value)
                for value in values
            )
        )
    if (
        dimension_id
        not in supported_dimensions
        or not grid_valid
    ):
        raise ValueError("invalid robustness grid")
    identities = {
        "actuator_target_bias": (
            "additive_actuator_target_bias_pulse_v1",
            "actuator_realization_error",
        ),
        "actuator_command_delay": (
            "actuator_command_transport_delay_pulse_v1",
            "actuator_transport_delay",
        ),
        "actuator_command_rate_limit": (
            "actuator_command_slew_rate_limit_pulse_v1",
            "actuator_rate_limit",
        ),
        "actuator_command_deadband": (
            "actuator_command_deadband_pulse_v1",
            "actuator_deadband",
        ),
        "joint_position_measurement_bias": (
            "additive_joint_position_bias_pulse_v1",
            "measurement_error",
        ),
        "joint_feedback_publication_dropout": (
            "joint_feedback_publication_dropout_burst_v1",
            "measurement_unavailability",
        ),
        "joint_feedback_controller_ingress_latency": (
            "joint_feedback_controller_ingress_delay_v1",
            "measurement_transport_latency",
        ),
        "joint_feedback_controller_ingress_jitter": (
            "joint_feedback_controller_ingress_jitter_pulse_v1",
            "measurement_transport_jitter",
        ),
        "joint_feedback_controller_stale_age": (
            "joint_feedback_controller_stale_age_pulse_v1",
            "measurement_selection_staleness",
        ),
        "joint_feedback_dropout_recovery": (
            "joint_feedback_dropout_recovery_hold_v1",
            "measurement_recovery_policy",
        ),
        "joint_feedback_repeated_dropout_rearm": (
            "joint_feedback_repeated_dropout_bursts_v1",
            "measurement_rearm_spacing",
        ),
        "joint_position_measurement_quantization": (
            "joint_position_quantization_pulse_v1",
            "measurement_resolution",
        ),
        "joint_position_measurement_saturation": (
            "joint_position_saturation_pulse_v1",
            "measurement_range",
        ),
    }
    expected_kind, expected_classification = identities[dimension_id]
    if (
        dimension.get("kind") != expected_kind
        or dimension.get("classification") != expected_classification
    ):
        raise ValueError("robustness dimension identity drifted")
    requirement_ids = {item["id"] for item in requirements.get("requirements", [])}
    evaluation_key = {
        "actuator_command_delay": "delay_evaluation",
        "actuator_command_rate_limit": "rate_limit_evaluation",
        "actuator_command_deadband": "deadband_evaluation",
        "joint_feedback_publication_dropout": "availability_evaluation",
        "joint_feedback_controller_ingress_latency": "latency_evaluation",
        "joint_feedback_controller_ingress_jitter": "jitter_evaluation",
        "joint_feedback_controller_stale_age": "stale_age_evaluation",
        "joint_feedback_dropout_recovery": "recovery_evaluation",
        "joint_feedback_repeated_dropout_rearm": "rearm_evaluation",
        "joint_position_measurement_quantization": "quantization_evaluation",
        "joint_position_measurement_saturation": "saturation_evaluation",
    }.get(dimension_id, "evaluation")
    evaluation = manifest[evaluation_key]
    if not set(evaluation["requirement_ids"]).issubset(requirement_ids):
        raise ValueError("robustness manifest names an unknown requirement")
    _, base_controllers = compiler.compile_suite(
        plant_report_path, plant_manifest_path, limits_controller_path
    )
    base = base_controllers["state_feedback"]
    controllers: dict[str, dict[str, Any]] = {}
    for value in values:
        if dimension_id == "actuator_target_bias":
            identifier = case_id(value)
        elif dimension_id == "actuator_command_delay":
            identifier = delay_case_id(value)
        elif dimension_id == "actuator_command_rate_limit":
            identifier = rate_limit_case_id(value)
        elif dimension_id == "actuator_command_deadband":
            identifier = deadband_case_id(value)
        elif dimension_id == "joint_position_measurement_bias":
            identifier = sensor_case_id(value)
        elif dimension_id == "joint_feedback_publication_dropout":
            identifier = dropout_case_id(value)
        elif dimension_id == "joint_feedback_controller_ingress_latency":
            identifier = sensor_latency_case_id(value)
        elif dimension_id == "joint_feedback_controller_ingress_jitter":
            identifier = sensor_jitter_case_id(value)
        elif dimension_id == "joint_feedback_controller_stale_age":
            identifier = sensor_stale_case_id(value)
        elif dimension_id == "joint_feedback_dropout_recovery":
            identifier = sensor_recovery_case_id(value)
        elif dimension_id == "joint_feedback_repeated_dropout_rearm":
            identifier = sensor_rearm_case_id(value)
        elif dimension_id == "joint_position_measurement_quantization":
            identifier = sensor_quantization_case_id(value)
        elif dimension_id == "joint_position_measurement_saturation":
            identifier = sensor_saturation_case_id(value)
        controller = copy.deepcopy(base)
        controller["controller_id"] = (
            f"rne.controller.openarm_right.plant_state_feedback_integral.{identifier}.v1"
        )
        if dimension_id == "actuator_target_bias":
            contract = controller["disturbance_contract"]
            contract["offset_rad"] = value
            fields = (
                "kind",
                "classification",
                "joint",
                "start_step",
                "end_step",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "actuator_command_delay":
            contract = {
                key: item
                for key, item in dimension.items()
                if key not in {"unit", "values"}
            }
            contract["delay_steps"] = value
            controller["disturbance_contract"] = contract
            fields = (
                "kind",
                "classification",
                "joint",
                "start_step",
                "end_step",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "actuator_command_rate_limit":
            contract = {
                key: item
                for key, item in dimension.items()
                if key not in {"unit", "values", "severity_order"}
            }
            contract["maximum_rate_rad_s"] = value
            controller["disturbance_contract"] = contract
            fields = (
                "kind",
                "classification",
                "joint",
                "start_step",
                "end_step",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "actuator_command_deadband":
            contract = {
                key: item for key, item in dimension.items() if key not in {"unit", "values"}
            }
            contract["deadband_rad"] = value
            controller["disturbance_contract"] = contract
            fields = (
                "kind",
                "classification",
                "joint",
                "start_step",
                "end_step",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "joint_position_measurement_bias":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {key: value for key, value in dimension.items() if key not in {"unit", "values"}}
            contract["offset_rad"] = value
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
        elif dimension_id == "joint_feedback_publication_dropout":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: value
                for key, value in dimension.items()
                if key
                not in {
                    "unit",
                    "values",
                    "maximum_age_ticks",
                    "stale_observation_policy",
                    "recovery_policy",
                }
            }
            contract["consecutive_dropped_frames"] = value
            controller["measurement_fault_contract"] = contract
            controller["observation_contract"]["maximum_age_ticks"] = dimension[
                "maximum_age_ticks"
            ]
            controller["observation_contract"]["stale_observation_policy"] = dimension[
                "stale_observation_policy"
            ]
            controller["observation_contract"]["recovery_policy"] = dimension[
                "recovery_policy"
            ]
            fields = (
                "kind",
                "classification",
                "start_capture_sequence",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "joint_feedback_controller_ingress_latency":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: item
                for key, item in dimension.items()
                if key
                not in {
                    "unit",
                    "values",
                    "base_sensor_latency_ticks",
                    "maximum_age_ticks",
                    "stale_observation_policy",
                    "recovery_policy",
                }
            }
            contract["delay_frames"] = value
            controller["measurement_fault_contract"] = contract
            controller["observation_contract"]["maximum_age_ticks"] = dimension[
                "maximum_age_ticks"
            ]
            controller["observation_contract"]["stale_observation_policy"] = dimension[
                "stale_observation_policy"
            ]
            controller["observation_contract"]["recovery_policy"] = dimension[
                "recovery_policy"
            ]
            fields = (
                "kind",
                "classification",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "joint_feedback_controller_ingress_jitter":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: item
                for key, item in dimension.items()
                if key
                not in {
                    "unit",
                    "values",
                    "base_sensor_latency_ticks",
                    "maximum_age_ticks",
                    "stale_observation_policy",
                    "recovery_policy",
                }
            }
            contract["maximum_jitter_frames"] = value
            controller["measurement_fault_contract"] = contract
            controller["observation_contract"]["maximum_age_ticks"] = dimension[
                "maximum_age_ticks"
            ]
            controller["observation_contract"]["stale_observation_policy"] = dimension[
                "stale_observation_policy"
            ]
            controller["observation_contract"]["recovery_policy"] = dimension[
                "recovery_policy"
            ]
            fields = (
                "kind",
                "classification",
                "start_capture_sequence",
                "end_capture_sequence",
                "schedule",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "joint_feedback_controller_stale_age":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: item
                for key, item in dimension.items()
                if key
                not in {
                    "unit",
                    "values",
                    "base_sensor_latency_ticks",
                    "maximum_age_ticks",
                    "stale_observation_policy",
                    "recovery_policy",
                }
            }
            contract["additional_stale_frames"] = value
            controller["measurement_fault_contract"] = contract
            controller["observation_contract"]["maximum_age_ticks"] = dimension[
                "maximum_age_ticks"
            ]
            controller["observation_contract"]["stale_observation_policy"] = dimension[
                "stale_observation_policy"
            ]
            controller["observation_contract"]["recovery_policy"] = dimension[
                "recovery_policy"
            ]
            fields = (
                "kind",
                "classification",
                "start_controller_step",
                "end_controller_step",
                "selection_policy",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "joint_feedback_dropout_recovery":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: item
                for key, item in dimension.items()
                if key
                not in {
                    "unit",
                    "values",
                    "maximum_age_ticks",
                    "stale_observation_policy",
                    "recovery_policy",
                }
            }
            contract["additional_recovery_hold_decisions"] = value
            controller["measurement_fault_contract"] = contract
            controller["observation_contract"]["maximum_age_ticks"] = dimension[
                "maximum_age_ticks"
            ]
            controller["observation_contract"]["stale_observation_policy"] = dimension[
                "stale_observation_policy"
            ]
            controller["observation_contract"]["recovery_policy"] = dimension[
                "recovery_policy"
            ]
            fields = (
                "kind",
                "classification",
                "start_capture_sequence",
                "consecutive_dropped_frames",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "joint_feedback_repeated_dropout_rearm":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: item
                for key, item in dimension.items()
                if key
                not in {
                    "unit",
                    "values",
                    "severity_order",
                    "maximum_age_ticks",
                    "stale_observation_policy",
                    "recovery_policy",
                }
            }
            contract["interburst_fresh_frames"] = value
            controller["measurement_fault_contract"] = contract
            controller["observation_contract"]["maximum_age_ticks"] = dimension[
                "maximum_age_ticks"
            ]
            controller["observation_contract"]["stale_observation_policy"] = dimension[
                "stale_observation_policy"
            ]
            controller["observation_contract"]["recovery_policy"] = dimension[
                "recovery_policy"
            ]
            fields = (
                "kind",
                "classification",
                "start_capture_sequence",
                "burst_length_frames",
                "burst_count",
                "schedule",
                "application_order",
                "controller_visibility",
            )
        elif dimension_id == "joint_position_measurement_quantization":
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: item
                for key, item in dimension.items()
                if key not in {"unit", "values"}
            }
            contract["quantization_step_rad"] = value
            controller["measurement_fault_contract"] = contract
            fields = (
                "kind",
                "classification",
                "joint",
                "start_controller_step",
                "end_controller_step",
                "quantization_rule",
                "sensor_status",
                "application_order",
                "controller_visibility",
            )
        else:
            controller["disturbance_contract"]["offset_rad"] = 0.0
            contract = {
                key: item
                for key, item in dimension.items()
                if key not in {"unit", "values", "severity_order"}
            }
            contract["saturation_limit_abs_rad"] = value
            controller["measurement_fault_contract"] = contract
            fields = (
                "kind",
                "classification",
                "joint",
                "start_controller_step",
                "end_controller_step",
                "saturation_rule",
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
        "evaluation": evaluation,
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
        if args.dimension == "actuator_target_bias":
            dimension_value = controller["disturbance_contract"]["offset_rad"]
        elif args.dimension == "actuator_command_delay":
            dimension_value = controller["disturbance_contract"]["delay_steps"]
        elif args.dimension == "actuator_command_rate_limit":
            dimension_value = controller["disturbance_contract"]["maximum_rate_rad_s"]
        elif args.dimension == "actuator_command_deadband":
            dimension_value = controller["disturbance_contract"]["deadband_rad"]
        elif args.dimension == "joint_position_measurement_bias":
            dimension_value = controller["measurement_fault_contract"]["offset_rad"]
        elif args.dimension == "joint_feedback_publication_dropout":
            dimension_value = controller["measurement_fault_contract"][
                "consecutive_dropped_frames"
            ]
        elif args.dimension == "joint_feedback_controller_ingress_latency":
            dimension_value = controller["measurement_fault_contract"]["delay_frames"]
        elif args.dimension == "joint_feedback_controller_ingress_jitter":
            dimension_value = controller["measurement_fault_contract"][
                "maximum_jitter_frames"
            ]
        elif args.dimension == "joint_feedback_dropout_recovery":
            dimension_value = controller["measurement_fault_contract"][
                "additional_recovery_hold_decisions"
            ]
        elif args.dimension == "joint_feedback_repeated_dropout_rearm":
            dimension_value = controller["measurement_fault_contract"][
                "interburst_fresh_frames"
            ]
        elif args.dimension == "joint_position_measurement_quantization":
            dimension_value = controller["measurement_fault_contract"][
                "quantization_step_rad"
            ]
        elif args.dimension == "joint_position_measurement_saturation":
            dimension_value = controller["measurement_fault_contract"][
                "saturation_limit_abs_rad"
            ]
        else:
            dimension_value = controller["measurement_fault_contract"][
                "additional_stale_frames"
            ]
        declaration = {
            "case_id": identifier,
            "dimension_value": dimension_value,
            "controller_id": controller["controller_id"],
            "controller_path": path.name,
            "controller_sha256": sha256(path),
        }
        if args.dimension == "joint_feedback_publication_dropout":
            declaration["consecutive_dropped_frames"] = dimension_value
        elif args.dimension == "joint_feedback_controller_ingress_latency":
            declaration["delay_frames"] = dimension_value
        elif args.dimension == "joint_feedback_controller_ingress_jitter":
            declaration["maximum_jitter_frames"] = dimension_value
        elif args.dimension == "joint_feedback_controller_stale_age":
            declaration["additional_stale_frames"] = dimension_value
        elif args.dimension == "joint_feedback_dropout_recovery":
            declaration["additional_recovery_hold_decisions"] = dimension_value
        elif args.dimension == "joint_feedback_repeated_dropout_rearm":
            declaration["interburst_fresh_frames"] = dimension_value
        elif args.dimension == "joint_position_measurement_quantization":
            declaration["quantization_step_rad"] = dimension_value
        elif args.dimension == "joint_position_measurement_saturation":
            declaration["saturation_limit_abs_rad"] = dimension_value
        elif args.dimension == "actuator_command_delay":
            declaration["delay_steps"] = dimension_value
        elif args.dimension == "actuator_command_rate_limit":
            declaration["maximum_rate_rad_s"] = dimension_value
        elif args.dimension == "actuator_command_deadband":
            declaration["deadband_rad"] = dimension_value
        else:
            declaration["offset_rad"] = dimension_value
        suite["cases"].append(declaration)
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

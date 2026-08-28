#!/usr/bin/env python3
"""Compile reproducible PID and state-feedback OpenArm controller artifacts."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any


JOINT = "openarm_right_joint5"
NOMINAL_BACKEND = "rne_rapier"
DESIRED_POLES = [0.5, 0.65, 0.75, 0.85]
MAXIMUM_CORRECTION_RAD = 0.04
MAXIMUM_INTEGRAL_CORRECTION_RAD = 0.015
DISTURBANCE_START_STEP = 3241
DISTURBANCE_END_STEP = 3300
DISTURBANCE_OFFSET_RAD = 0.03


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--plant-report",
        type=Path,
        default=root
        / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
    )
    parser.add_argument(
        "--experiment-manifest",
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
    return parser.parse_args()


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def identity(width: int) -> list[list[float]]:
    return [[float(row == column) for column in range(width)] for row in range(width)]


def matrix_add(left: list[list[float]], right: list[list[float]]) -> list[list[float]]:
    return [
        [a + b for a, b in zip(left_row, right_row)]
        for left_row, right_row in zip(left, right)
    ]


def matrix_scale(matrix: list[list[float]], scalar: float) -> list[list[float]]:
    return [[value * scalar for value in row] for row in matrix]


def matrix_multiply(
    left: list[list[float]], right: list[list[float]]
) -> list[list[float]]:
    columns = list(zip(*right))
    return [
        [sum(a * b for a, b in zip(row, column)) for column in columns]
        for row in left
    ]


def matrix_power(matrix: list[list[float]], exponent: int) -> list[list[float]]:
    result = identity(len(matrix))
    for _ in range(exponent):
        result = matrix_multiply(result, matrix)
    return result


def inverse(matrix: list[list[float]]) -> list[list[float]]:
    width = len(matrix)
    augmented = [row.copy() + unit for row, unit in zip(matrix, identity(width))]
    for column in range(width):
        pivot = max(range(column, width), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) <= 1e-14:
            raise ValueError("matrix is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(width):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                value - factor * pivot_value
                for value, pivot_value in zip(augmented[row], augmented[column])
            ]
    return [row[width:] for row in augmented]


def determinant(matrix: list[list[float]]) -> float:
    work = [row.copy() for row in matrix]
    value = 1.0
    for column in range(len(work)):
        pivot = max(range(column, len(work)), key=lambda row: abs(work[row][column]))
        if abs(work[pivot][column]) <= 1e-15:
            return 0.0
        if pivot != column:
            work[column], work[pivot] = work[pivot], work[column]
            value = -value
        pivot_value = work[column][column]
        value *= pivot_value
        for row in range(column + 1, len(work)):
            factor = work[row][column] / pivot_value
            for index in range(column + 1, len(work)):
                work[row][index] -= factor * work[column][index]
    return value


def polynomial_from_roots(roots: list[float]) -> list[float]:
    coefficients = [1.0]
    for root in roots:
        expanded = [0.0] * (len(coefficients) + 1)
        for index, coefficient in enumerate(coefficients):
            expanded[index] += coefficient
            expanded[index + 1] -= root * coefficient
        coefficients = expanded
    return coefficients


def pole_placement(
    augmented_a: list[list[float]], augmented_b: list[list[float]], poles: list[float]
) -> tuple[list[float], float, list[list[float]]]:
    width = len(augmented_a)
    controllability_columns = [
        matrix_multiply(matrix_power(augmented_a, exponent), augmented_b)
        for exponent in range(width)
    ]
    controllability = [
        [controllability_columns[column][row][0] for column in range(width)]
        for row in range(width)
    ]
    controllability_determinant = determinant(controllability)
    if abs(controllability_determinant) <= 1e-10:
        raise ValueError("augmented plant is not controllable")
    coefficients = polynomial_from_roots(poles)
    phi = [[0.0] * width for _ in range(width)]
    for exponent, coefficient in zip(range(width, -1, -1), coefficients):
        phi = matrix_add(phi, matrix_scale(matrix_power(augmented_a, exponent), coefficient))
    selector = [[0.0] * (width - 1) + [1.0]]
    gain = matrix_multiply(matrix_multiply(selector, inverse(controllability)), phi)[0]
    closed_loop = matrix_add(
        augmented_a,
        matrix_scale(matrix_multiply(augmented_b, [gain]), -1.0),
    )
    for pole in poles:
        characteristic = [
            [pole * float(row == column) - closed_loop[row][column] for column in range(width)]
            for row in range(width)
        ]
        if abs(determinant(characteristic)) > 1e-8:
            raise ValueError("placed closed-loop poles do not match the declaration")
    return gain, controllability_determinant, closed_loop


def compile_plant_controller(manifest_path: Path) -> dict[str, Any]:
    compiler_path = manifest_path.parent / "build_openarm_plant_controller.py"
    spec = importlib.util.spec_from_file_location("rne_openarm_plant_compiler", compiler_path)
    if spec is None or spec.loader is None:
        raise ValueError("cannot load the OpenArm plant controller compiler")
    compiler = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compiler)
    return compiler.compile_controller(compiler.load_manifest(manifest_path))


def nominal_arx(report: dict[str, Any]) -> list[float]:
    if (
        report.get("kind") != "rne_openarm_plant_lab_report"
        or report.get("schema_version") != 1
    ):
        raise ValueError("unsupported OpenArm plant report")
    backend = next(
        (item for item in report["backends"] if item["backend_id"] == NOMINAL_BACKEND),
        None,
    )
    if backend is None:
        raise ValueError("plant report has no nominal Rapier model")
    coefficients = backend["arx_model"]["coefficients"]
    if len(coefficients) != 5 or not all(math.isfinite(value) for value in coefficients):
        raise ValueError("nominal ARX coefficients are invalid")
    return coefficients


def observation_contract(fixed_delta_ticks: int) -> dict[str, Any]:
    return {
        "kind": "rne_joint_feedback",
        "schema_version": 1,
        "sample_period_ticks": fixed_delta_ticks,
        "phase_offset_ticks": fixed_delta_ticks,
        "latency_ticks": fixed_delta_ticks,
        "maximum_age_ticks": fixed_delta_ticks,
        "required_status": "nominal",
        "bootstrap_policy": "reference_until_first_available",
    }


def disturbance_contract() -> dict[str, Any]:
    return {
        "kind": "additive_actuator_target_bias_pulse_v1",
        "classification": "actuator_realization_error",
        "joint": JOINT,
        "start_step": DISTURBANCE_START_STEP,
        "end_step": DISTURBANCE_END_STEP,
        "offset_rad": DISTURBANCE_OFFSET_RAD,
        "controller_visibility": "unobserved_except_through_typed_joint_feedback",
        "application_order": "after_controller_limits_before_backend_actuation",
    }


def compile_suite(
    report_path: Path, manifest_path: Path, limits_controller_path: Path
) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    report = load(report_path)
    manifest = load(manifest_path)
    limits_controller = load(limits_controller_path)
    base = compile_plant_controller(manifest_path)
    manifest_input = next(
        item for item in report["inputs"] if item["role"] == "experiment_manifest"
    )
    if (
        report["experiment_id"] != manifest["experiment_id"]
        or manifest_input["sha256"] != sha256(manifest_path)
        or report["experiment_contract"]["fixed_delta_ticks"]
        != manifest["fixed_delta_ticks"]
    ):
        raise ValueError("plant report and experiment manifest identity drifted")
    order = manifest["action_joint_order"]
    joint_index = order.index(JOINT)
    minimum_targets = limits_controller["feedback_law"]["minimum_target_rad"]
    maximum_targets = limits_controller["feedback_law"]["maximum_target_rad"]
    if len(minimum_targets) != len(order) or len(maximum_targets) != len(order):
        raise ValueError("controller target limits do not match the plant joint order")
    _, a1, a2, b1, b2 = nominal_arx(report)
    plant_a = [[a1, a2, b2], [1.0, 0.0, 0.0], [0.0, 0.0, 0.0]]
    plant_b = [[b1], [0.0], [1.0]]
    sample_period_s = manifest["fixed_delta_ticks"] / 1_000_000_000.0
    augmented_a = [row + [0.0] for row in plant_a] + [
        [-sample_period_s, 0.0, 0.0, 1.0]
    ]
    augmented_b = plant_b + [[0.0]]
    plant_c = [[1.0, 0.0, 0.0]]
    dynamic_a = [[a1, a2], [1.0, 0.0]]
    dynamic_c = [[1.0, 0.0]]
    observability = [
        matrix_multiply(dynamic_c, matrix_power(dynamic_a, exponent))[0]
        for exponent in range(2)
    ]
    observability_determinant = determinant(observability)
    if abs(observability_determinant) <= 1e-10:
        raise ValueError("identified plant is not observable")
    gain, controllability_determinant, closed_loop = pole_placement(
        augmented_a, augmented_b, DESIRED_POLES
    )
    state_feedback_gain = gain[:3]
    integral_error_gain_s_inv = -gain[3]
    if integral_error_gain_s_inv <= 0.0:
        raise ValueError("state-feedback integral gain has the wrong sign")
    maximum_integral_error_rad_s = (
        MAXIMUM_INTEGRAL_CORRECTION_RAD / integral_error_gain_s_inv
    )
    zero = [0.0] * len(order)
    pid_position = zero.copy()
    pid_velocity = zero.copy()
    pid_integral = zero.copy()
    pid_maximum_integral = zero.copy()
    pid_maximum_correction = zero.copy()
    pid_position[joint_index] = 0.3
    pid_velocity[joint_index] = 0.002
    pid_integral[joint_index] = 0.3
    pid_maximum_integral[joint_index] = MAXIMUM_INTEGRAL_CORRECTION_RAD
    pid_maximum_correction[joint_index] = MAXIMUM_CORRECTION_RAD
    contract = observation_contract(manifest["fixed_delta_ticks"])
    pid = dict(base)
    pid["controller_id"] = "rne.controller.openarm_right.plant_pid.v1"
    pid["observation_contract"] = contract
    pid["disturbance_contract"] = disturbance_contract()
    pid["feedback_law"] = {
        "kind": "joint_position_reference_pid_v1",
        "position_error_gain": pid_position,
        "velocity_damping_s": pid_velocity,
        "integral_error_gain_s_inv": pid_integral,
        "maximum_integral_correction_rad": pid_maximum_integral,
        "maximum_correction_rad": pid_maximum_correction,
        "minimum_target_rad": minimum_targets,
        "maximum_target_rad": maximum_targets,
    }
    state_feedback = dict(base)
    state_feedback["controller_id"] = (
        "rne.controller.openarm_right.plant_state_feedback_integral.v1"
    )
    state_feedback["observation_contract"] = contract
    state_feedback["disturbance_contract"] = disturbance_contract()
    state_feedback["feedback_law"] = {
        "kind": "joint_position_state_feedback_integral_v1",
        "controlled_joint": JOINT,
        "state_order": [
            "predicted_tracking_error_rad",
            "observed_tracking_error_rad",
            "previous_input_tracking_error_rad",
            "integrated_reference_error_rad_s",
        ],
        "reference_feedforward": "unity_position_reference_v1",
        "observation_latency_compensation": "one_sample_arx_predictor_v1",
        "operating_point_position_rad": manifest["operating_point_rad"][joint_index],
        "operating_point_input_rad": manifest["operating_point_rad"][joint_index],
        "identified_plant": {
            "kind": "siso_arx_2_2_with_intercept",
            "source_backend_id": NOMINAL_BACKEND,
            "source_report_sha256": sha256(report_path),
            "arx_coefficients": nominal_arx(report),
            "discrete_a": plant_a,
            "discrete_b": [row[0] for row in plant_b],
            "discrete_c": plant_c[0],
            "augmented_a": augmented_a,
            "augmented_b": [row[0] for row in augmented_b],
            "controllability_determinant": controllability_determinant,
            "observability_scope": "dynamic_output_state_with_known_input_history_v1",
            "observability_determinant": observability_determinant,
        },
        "state_feedback_gain": state_feedback_gain,
        "integral_state_feedback_gain_s_inv": integral_error_gain_s_inv,
        "desired_closed_loop_poles": DESIRED_POLES,
        "closed_loop_a": closed_loop,
        "maximum_integral_state_error_rad_s": maximum_integral_error_rad_s,
        "maximum_state_integral_correction_rad": MAXIMUM_INTEGRAL_CORRECTION_RAD,
        "maximum_state_feedback_correction_rad": MAXIMUM_CORRECTION_RAD,
        "minimum_controlled_target_rad": minimum_targets[joint_index],
        "maximum_controlled_target_rad": maximum_targets[joint_index],
    }
    suite = {
        "kind": "rne_openarm_controller_suite",
        "schema_version": 1,
        "suite_id": "rne.openarm.right.plant_controller_comparison.v1",
        "task_id": manifest["task_id"],
        "experiment_id": manifest["experiment_id"],
        "controlled_joint": JOINT,
        "nominal_model_backend_id": NOMINAL_BACKEND,
        "source_plant_report_sha256": sha256(report_path),
        "source_experiment_manifest_sha256": sha256(manifest_path),
        "fixed_delta_ticks": manifest["fixed_delta_ticks"],
        "observation_latency_samples": 1,
        "shared_maximum_correction_rad": MAXIMUM_CORRECTION_RAD,
        "shared_maximum_integral_correction_rad": MAXIMUM_INTEGRAL_CORRECTION_RAD,
        "shared_disturbance_contract": disturbance_contract(),
        "controllers": [],
    }
    return suite, {"pid": pid, "state_feedback": state_feedback}


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    suite, controllers = compile_suite(
        args.plant_report.resolve(),
        args.experiment_manifest.resolve(),
        args.limits_controller.resolve(),
    )
    for name, controller in controllers.items():
        path = output / f"openarm-plant-{name.replace('_', '-')}.controller.json"
        write_json(path, controller)
        suite["controllers"].append(
            {
                "role": name,
                "controller_id": controller["controller_id"],
                "path": path.name,
                "sha256": sha256(path),
            }
        )
    write_json(output / "openarm-controller-suite.json", suite)
    print(
        "OpenArm controller suite: "
        f"controllers={len(controllers)} nominal_backend={NOMINAL_BACKEND}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm controller suite failed: {error}")
        raise SystemExit(2)

#!/usr/bin/env python3
"""Build a reproducible OpenArm time/frequency-domain plant report."""

from __future__ import annotations

import argparse
import cmath
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from typing import Any, Iterable
import xml.etree.ElementTree as ET


BACKENDS = ("rne_rapier", "mujoco_native", "gazebo_sim")
TRACE_FILES = {
    "rne_rapier": "rapier-success-trace.json",
    "mujoco_native": "mujoco-success-trace.json",
    "gazebo_sim": "gazebo-success-trace.json",
}
FAILURE_FILES = {
    "rne_rapier": "intentional-failure.json",
    "mujoco_native": "mujoco-intentional-failure.json",
    "gazebo_sim": "gazebo-intentional-failure.json",
}
JOINT5 = "openarm_right_joint5"
QUALITY_GATES = {
    "measurement_integrity",
    "estimation_validity",
    "plant_integrity",
    "identification_validity",
    "closed_loop_performance",
    "portability",
}


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=root)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_experiments.json",
    )
    parser.add_argument(
        "--requirements",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_requirements.json",
    )
    parser.add_argument("--controller", required=True, type=Path)
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


def json_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def segment(manifest: dict[str, Any], identifier: str) -> dict[str, Any]:
    return next(item for item in manifest["segments"] if item["id"] == identifier)


def requirement_map(registry: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if (
        set(registry) != {"kind", "schema_version", "registry_id", "requirements"}
        or
        registry.get("kind") != "rne_openarm_plant_requirements_registry"
        or registry.get("schema_version") != 1
    ):
        raise ValueError("unsupported OpenArm plant requirements registry")
    requirements = registry.get("requirements")
    if not isinstance(requirements, list):
        raise ValueError("requirements registry has no requirement list")
    by_id = {requirement["id"]: requirement for requirement in requirements}
    if len(by_id) != len(requirements):
        raise ValueError("requirements registry contains duplicate ids")
    for requirement in requirements:
        expected = (
            {"id", "gate", "unit", "maximum"}
            if "maximum" in requirement
            else {"id", "gate", "unit", "minimum"}
        )
        if set(requirement) != expected:
            raise ValueError(f"requirement {requirement.get('id')} has unknown or missing fields")
        if requirement["gate"] not in QUALITY_GATES:
            raise ValueError(
                f"requirement {requirement['id']} has unknown quality gate"
            )
    return by_id


def compile_declared_controller(manifest_path: Path) -> dict[str, Any]:
    compiler_path = manifest_path.parent / "build_openarm_plant_controller.py"
    spec = importlib.util.spec_from_file_location("rne_openarm_plant_compiler", compiler_path)
    if spec is None or spec.loader is None:
        raise ValueError("cannot load the OpenArm plant manifest compiler")
    compiler = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compiler)
    return compiler.compile_controller(compiler.load_manifest(manifest_path))


def upper_check(
    requirement: dict[str, Any],
    observed: float | None,
    suffix: str = "",
    **diagnostic: Any,
) -> dict[str, Any]:
    maximum = requirement["maximum"]
    return {
        "id": requirement["id"] + suffix,
        "gate": requirement["gate"],
        "unit": requirement["unit"],
        "observed": observed,
        "maximum": maximum,
        "status": "passed"
        if observed is not None and math.isfinite(observed) and observed <= maximum
        else "failed",
        **diagnostic,
    }


def lower_check(
    requirement: dict[str, Any], observed: float, suffix: str = ""
) -> dict[str, Any]:
    minimum = requirement["minimum"]
    return {
        "id": requirement["id"] + suffix,
        "gate": requirement["gate"],
        "unit": requirement["unit"],
        "observed": observed,
        "minimum": minimum,
        "status": "passed"
        if math.isfinite(observed) and observed >= minimum
        else "failed",
    }


def joint_limits(urdf: Path, order: list[str]) -> list[dict[str, float]]:
    parsed: dict[str, dict[str, float]] = {}
    for joint in ET.parse(urdf).getroot().findall("joint"):
        limit = joint.find("limit")
        if limit is not None and all(
            limit.get(field) is not None for field in ("lower", "upper", "velocity")
        ):
            parsed[joint.attrib["name"]] = {
                "minimum_position_rad": float(limit.attrib["lower"]),
                "maximum_position_rad": float(limit.attrib["upper"]),
                "maximum_velocity_rad_s": float(limit.attrib["velocity"]),
            }
    missing = [name for name in order if name not in parsed]
    if missing:
        raise ValueError(f"URDF limits missing for {missing}")
    return [parsed[name] for name in order]


def rms(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        raise ValueError("cannot calculate RMS of an empty sequence")
    return math.sqrt(sum(value * value for value in values) / len(values))


def first_crossing(values: list[float], threshold: float, increasing: bool) -> int | None:
    for index, value in enumerate(values):
        if (increasing and value >= threshold) or (not increasing and value <= threshold):
            return index
    return None


def step_metrics(
    actions: list[dict[str, Any]],
    observations: list[dict[str, Any]],
    joint_index: int,
    step_segment: dict[str, Any],
    sample_rate_hz: float,
    analysis: dict[str, Any],
) -> dict[str, Any]:
    start = step_segment["positive_step"] - 1
    end = step_segment["negative_step"] - 1
    baseline_start = max(step_segment["start_step"] - 1, start - 60)
    baseline = sum(
        frame["joint_position_rad"][joint_index]
        for frame in observations[baseline_start:start]
    ) / (start - baseline_start)
    response = [
        frame["joint_position_rad"][joint_index] for frame in observations[start:end]
    ]
    command = [
        frame["joint_position_target_rad"][joint_index] for frame in actions[start:end]
    ]
    steady = sum(response[-30:]) / 30
    achieved = steady - baseline
    commanded = step_segment["amplitude_rad"]
    increasing = achieved >= 0.0
    low = baseline + analysis["step_rise_low_fraction"] * achieved
    high = baseline + analysis["step_rise_high_fraction"] * achieved
    low_index = first_crossing(response, low, increasing)
    high_index = first_crossing(response, high, increasing)
    rise_time_s = (
        (high_index - low_index) / sample_rate_hz
        if low_index is not None and high_index is not None and high_index >= low_index
        else None
    )
    peak = max(response) if increasing else min(response)
    overshoot_fraction = max(
        0.0,
        ((peak - steady) if increasing else (steady - peak)) / max(abs(achieved), 1e-15),
    )
    band = analysis["step_settling_band_fraction"] * abs(commanded)
    settling_index = None
    for index in range(len(response)):
        if all(abs(value - command[-1]) <= band for value in response[index:]):
            settling_index = index
            break
    error = [measured - desired for measured, desired in zip(response, command)]
    return {
        "commanded_step_rad": commanded,
        "baseline_position_rad": baseline,
        "steady_position_rad": steady,
        "achieved_step_rad": achieved,
        "static_gain": achieved / commanded,
        "rise_time_s": rise_time_s,
        "settling_time_s": (
            settling_index / sample_rate_hz if settling_index is not None else None
        ),
        "settling_band_rad": band,
        "overshoot_fraction": overshoot_fraction,
        "steady_state_error_rad": steady - command[-1],
        "iae_rad_s": sum(abs(value) for value in error) / sample_rate_hz,
        "ise_rad2_s": sum(value * value for value in error) / sample_rate_hz,
    }


def complex_projection(
    values: list[float], sample_rate_hz: float, frequency_hz: float
) -> complex:
    count = len(values)
    if count < 3:
        raise ValueError("frequency projection window is too short")
    mean = sum(values) / count
    weighted = 0j
    weight_sum = 0.0
    for index, value in enumerate(values):
        weight = 0.5 - 0.5 * math.cos(2.0 * math.pi * index / (count - 1))
        weighted += weight * (value - mean) * cmath.exp(
            -2j * math.pi * frequency_hz * index / sample_rate_hz
        )
        weight_sum += weight
    return weighted / weight_sum


def chirp_response(
    manifest: dict[str, Any],
    actions: list[dict[str, Any]],
    observations: list[dict[str, Any]],
    joint_index: int,
) -> list[dict[str, Any]]:
    chirp = segment(manifest, "joint5_chirp_training")
    rate = manifest["sample_rate_hz"]
    count = chirp["end_step"] - chirp["start_step"] + 1
    half_window = manifest["analysis"]["chirp_window_samples"] // 2
    result = []
    for frequency in chirp["frequency_grid_hz"]:
        alpha = (frequency - chirp["start_frequency_hz"]) / (
            chirp["end_frequency_hz"] - chirp["start_frequency_hz"]
        )
        center = chirp["start_step"] - 1 + round(alpha * (count - 1))
        start = max(chirp["start_step"] - 1, center - half_window)
        end = min(chirp["end_step"], center + half_window)
        input_values = [
            frame["joint_position_target_rad"][joint_index]
            for frame in actions[start:end]
        ]
        output_values = [
            frame["joint_position_rad"][joint_index]
            for frame in observations[start:end]
        ]
        input_projection = complex_projection(input_values, rate, frequency)
        output_projection = complex_projection(output_values, rate, frequency)
        transfer = output_projection / input_projection
        result.append(
            {
                "frequency_hz": frequency,
                "window_start_step": start + 1,
                "window_end_step": end,
                "input_projection_amplitude_rad": 2.0 * abs(input_projection),
                "gain_rad_per_rad": abs(transfer),
                "phase_rad": math.atan2(transfer.imag, transfer.real),
            }
        )
    return result


def coupling_matrix(
    manifest: dict[str, Any],
    actions: list[dict[str, Any]],
    observations: list[dict[str, Any]],
) -> dict[str, Any]:
    coupling = segment(manifest, "multi_axis_coupling")
    start = coupling["start_step"] - 1
    end = coupling["end_step"]
    rate = manifest["sample_rate_hz"]
    order = manifest["action_joint_order"]
    columns = []
    for source in coupling["sources"]:
        source_index = order.index(source["joint"])
        input_values = [
            frame["joint_position_target_rad"][source_index]
            for frame in actions[start:end]
        ]
        input_projection = complex_projection(
            input_values, rate, source["frequency_hz"]
        )
        output_gains = []
        output_phases = []
        for output_index in range(len(order)):
            output_values = [
                frame["joint_position_rad"][output_index]
                for frame in observations[start:end]
            ]
            response = complex_projection(
                output_values, rate, source["frequency_hz"]
            ) / input_projection
            output_gains.append(abs(response))
            output_phases.append(math.atan2(response.imag, response.real))
        columns.append(
            {
                "source_joint": source["joint"],
                "frequency_hz": source["frequency_hz"],
                "input_projection_amplitude_rad": 2.0 * abs(input_projection),
                "output_gain_rad_per_rad": output_gains,
                "output_phase_rad": output_phases,
            }
        )
    return {"output_joint_order": order, "columns": columns}


def solve(matrix: list[list[float]], vector: list[float]) -> list[float]:
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    width = len(vector)
    for column in range(width):
        pivot = max(range(column, width), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-14:
            raise ValueError("ARX normal equation is singular")
        augmented[column], augmented[pivot] = augmented[pivot], augmented[column]
        scale = augmented[column][column]
        augmented[column] = [value / scale for value in augmented[column]]
        for row in range(width):
            if row == column:
                continue
            factor = augmented[row][column]
            augmented[row] = [
                left - factor * right
                for left, right in zip(augmented[row], augmented[column])
            ]
    return [augmented[row][-1] for row in range(width)]


def arx_rows(inputs: list[float], outputs: list[float]) -> tuple[list[list[float]], list[float]]:
    rows = []
    expected = []
    for index in range(2, len(outputs)):
        rows.append(
            [1.0, outputs[index - 1], outputs[index - 2], inputs[index - 1], inputs[index - 2]]
        )
        expected.append(outputs[index])
    return rows, expected


def fit_arx(rows: list[list[float]], outputs: list[float]) -> list[float]:
    width = len(rows[0])
    normal = [[0.0] * width for _ in range(width)]
    rhs = [0.0] * width
    for row, output in zip(rows, outputs):
        for left in range(width):
            rhs[left] += row[left] * output
            for right in range(width):
                normal[left][right] += row[left] * row[right]
    for index in range(width):
        normal[index][index] += 1e-10
    return solve(normal, rhs)


def predict(coefficients: list[float], rows: list[list[float]], expected: list[float]) -> dict[str, Any]:
    residuals = [
        sum(coefficient * value for coefficient, value in zip(coefficients, row)) - output
        for row, output in zip(rows, expected)
    ]
    return {
        "sample_count": len(residuals),
        "one_step_prediction_rmse_rad": rms(residuals),
        "maximum_absolute_residual_rad": max(abs(value) for value in residuals),
        "mean_residual_rad": sum(residuals) / len(residuals),
    }


def dataset(
    actions: list[dict[str, Any]],
    observations: list[dict[str, Any]],
    joint_index: int,
    window: dict[str, Any],
) -> list[dict[str, Any]]:
    start = window["start_step"] - 1
    end = window["end_step"]
    return [
        {
            "step": action["step"],
            "input_target_rad": action["joint_position_target_rad"][joint_index],
            "output_position_rad": observation["joint_position_rad"][joint_index],
            "output_velocity_rad_s": observation["joint_velocity_rad_s"][joint_index],
        }
        for action, observation in zip(actions[start:end], observations[start:end])
    ]


def per_joint_metrics(
    actions: list[dict[str, Any]], observations: list[dict[str, Any]], order: list[str], rate: float
) -> list[dict[str, Any]]:
    result = []
    for index, name in enumerate(order):
        errors = [
            observation["joint_position_rad"][index]
            - action["joint_position_target_rad"][index]
            for action, observation in zip(actions, observations)
        ]
        positions = [observation["joint_position_rad"][index] for observation in observations]
        velocities = [observation["joint_velocity_rad_s"][index] for observation in observations]
        result.append(
            {
                "joint": name,
                "tracking_rmse_rad": rms(errors),
                "iae_rad_s": sum(abs(value) for value in errors) / rate,
                "ise_rad2_s": sum(value * value for value in errors) / rate,
                "minimum_position_rad": min(positions),
                "maximum_position_rad": max(positions),
                "peak_absolute_velocity_rad_s": max(abs(value) for value in velocities),
            }
        )
    return result


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main() -> int:
    args = parse_args()
    repo = args.repo_root.resolve()
    trace_root = args.trace_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    manifest_path = args.manifest.resolve()
    requirements_path = args.requirements.resolve()
    controller_path = args.controller.resolve()
    manifest = load(manifest_path)
    registry = load(requirements_path)
    requirements = requirement_map(registry)
    controller = load(controller_path)
    if controller != compile_declared_controller(manifest_path):
        raise ValueError("generated controller does not exactly match the experiment manifest")
    actions_path = trace_root / "controller-actions.json"
    actions_artifact = load(actions_path)
    actions = actions_artifact["actions"]
    order = manifest["action_joint_order"]
    joint5_index = order.index(JOINT5)
    final_step = manifest["segments"][-1]["end_step"]
    if (
        manifest.get("kind") != "rne_openarm_plant_experiment_manifest"
        or manifest.get("schema_version") != 1
        or controller.get("controller_id") != manifest["controller_id"]
        or actions_artifact.get("controller_sha256") != sha256(controller_path)
        or actions_artifact.get("action_joint_order") != order
        or len(actions) != final_step
        or actions[-1]["step"] != final_step
    ):
        raise ValueError("plant manifest/controller/action identity drifted")
    for index, (action, keyframe) in enumerate(zip(actions, controller["keyframes"][1:]), 1):
        if (
            action.get("action_sequence") != index - 1
            or action.get("step") != index
            or action.get("sim_time_ticks") != index * manifest["fixed_delta_ticks"]
            or action.get("phase") != keyframe["phase"]
            or len(action.get("joint_position_target_rad", [])) != len(order)
            or any(
                abs(actual - declared) > 1e-14
                for actual, declared in zip(
                    action["joint_position_target_rad"],
                    keyframe["joint_position_target_rad"],
                )
            )
        ):
            raise ValueError(f"compiled action differs from the manifest at step {index}")

    task_path = repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json"
    urdf_path = repo / "assets/robots/openarm_description/openarm_v2_right.rne.urdf"
    actuation_path = repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json"
    gazebo_adapter_path = repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json"
    if actions_artifact["task_sha256"] != sha256(task_path):
        raise ValueError("plant action TaskSpec hash drifted")
    limits = joint_limits(urdf_path, order)
    rate = manifest["sample_rate_hz"]
    training_window = segment(manifest, manifest["analysis"]["training_segment"])
    validation_window = segment(manifest, manifest["analysis"]["validation_segment"])
    ramp_window = segment(manifest, "joint5_triangular_ramp")
    step_window = segment(manifest, "joint5_step_doublet")

    traces = {backend: load(trace_root / TRACE_FILES[backend]) for backend in BACKENDS}
    failures = {backend: load(trace_root / FAILURE_FILES[backend]) for backend in BACKENDS}
    backend_reports = []
    excitation_actions = actions[300:]
    maximum_input_offset = max(
        abs(value - center)
        for action in excitation_actions
        for value, center in zip(
            action["joint_position_target_rad"], manifest["operating_point_rad"]
        )
    )
    all_checks = [
        upper_check(
            requirements["plant.input.maximum_offset_rad"], maximum_input_offset
        )
    ]
    for excitation_id in (
        "joint5_step_doublet",
        "joint5_triangular_ramp",
        "joint5_chirp_training",
        "joint5_multisine_validation",
    ):
        excitation = segment(manifest, excitation_id)
        start = excitation["start_step"] - 1
        end = excitation["end_step"]
        excitation_rms = rms(
            actions[index]["joint_position_target_rad"][joint5_index]
            - manifest["operating_point_rad"][joint5_index]
            for index in range(start, end)
        )
        all_checks.append(
            lower_check(
                requirements["plant.input.minimum_excitation_rms_rad"],
                excitation_rms,
                f".{excitation_id}",
            )
        )
    hard_violations = []
    datasets = []
    plot_outputs: dict[str, list[float]] = {}
    tracking_rmse = {}
    for backend in BACKENDS:
        trace_path = trace_root / TRACE_FILES[backend]
        trace = traces[backend]
        observations = trace["observations"]
        if (
            trace.get("backend_id") != backend
            or trace.get("controller_sha256") != sha256(controller_path)
            or trace.get("action_trace_sha256") != sha256(actions_path)
            or trace.get("controller_execution") != "open_loop_reference"
            or not trace.get("replay_match")
            or len(observations) != len(actions)
        ):
            raise ValueError(f"{backend} plant trace identity or replay drifted")
        for index, observation in enumerate(observations, 1):
            if (
                observation.get("step") != index
                or observation.get("sim_time_ticks")
                != index * manifest["fixed_delta_ticks"]
                or observation.get("sensor_status") != "nominal"
                or len(observation.get("joint_position_rad", [])) != len(order)
                or len(observation.get("joint_velocity_rad_s", [])) != len(order)
            ):
                raise ValueError(f"{backend} observation contract drifted at step {index}")
        failure = failures[backend]
        if (
            failure.get("first_violation") != "action_width_mismatch"
            or failure.get("first_violation_step")
            != manifest["intentional_failure"]["inject_at_step"]
            or failure.get("status") != "failed_as_expected"
        ):
            raise ValueError(f"{backend} intentional failure drifted")

        step_result = step_metrics(actions, observations, joint5_index, step_window, rate, manifest["analysis"])
        ramp_start = ramp_window["start_step"] - 1
        ramp_end = ramp_window["end_step"]
        ramp_rmse = rms(
            observations[index]["joint_position_rad"][joint5_index]
            - actions[index]["joint_position_target_rad"][joint5_index]
            for index in range(ramp_start, ramp_end)
        )
        saturation_fields = [
            frame.get("effort_saturated", frame.get("actuator_command_saturated"))
            for frame in observations
        ]
        if any(values is None or len(values) != len(order) for values in saturation_fields):
            raise ValueError(f"{backend} trace has no complete actuator saturation evidence")
        saturation_count = sum(
            sum(bool(value) for value in values) for values in saturation_fields
        )
        saturation_semantics = (
            "gazebo_position_velocity_limit_v1"
            if backend == "gazebo_sim"
            else "bounded_force_effort_v1"
        )
        saturation_fraction = saturation_count / (len(observations) * len(order))
        frequency = chirp_response(manifest, actions, observations, joint5_index)
        coupling = coupling_matrix(manifest, actions, observations)
        training = dataset(actions, observations, joint5_index, training_window)
        validation = dataset(actions, observations, joint5_index, validation_window)
        training_rows, training_outputs = arx_rows(
            [row["input_target_rad"] for row in training],
            [row["output_position_rad"] for row in training],
        )
        validation_rows, validation_outputs = arx_rows(
            [row["input_target_rad"] for row in validation],
            [row["output_position_rad"] for row in validation],
        )
        coefficients = fit_arx(training_rows, training_outputs)
        training_metrics = predict(coefficients, training_rows, training_outputs)
        validation_metrics = predict(coefficients, validation_rows, validation_outputs)
        dataset_artifact = {
            "kind": "rne_openarm_plant_dataset",
            "schema_version": 1,
            "backend_id": backend,
            "experiment_id": manifest["experiment_id"],
            "joint": JOINT5,
            "training_segment": training_window["id"],
            "validation_segment": validation_window["id"],
            "training": training,
            "validation": validation,
        }
        dataset_path = output / f"{backend}-plant-dataset.json"
        write_json(dataset_path, dataset_artifact)
        datasets.append(
            {
                "backend_id": backend,
                "path": dataset_path.name,
                "sha256": sha256(dataset_path),
                "training_content_sha256": json_sha256(training),
                "validation_content_sha256": json_sha256(validation),
            }
        )

        settling_requirement = requirements["plant.step.maximum_settling_time_s"]
        step_response = [
            frame["joint_position_rad"][joint5_index]
            for frame in observations[
                step_window["positive_step"] - 1 : step_window["negative_step"] - 1
            ]
        ]
        settling_deadline_index = math.ceil(
            settling_requirement["maximum"] * rate
        )
        settling_deadline_index = min(settling_deadline_index, len(step_response) - 1)
        settling_deadline_step = step_window["positive_step"] + settling_deadline_index
        settling_target_rad = actions[step_window["positive_step"] - 1][
            "joint_position_target_rad"
        ][joint5_index]
        checks = [
            upper_check(requirements["plant.step.maximum_overshoot_fraction"], step_result["overshoot_fraction"], f".{backend}"),
            upper_check(
                settling_requirement,
                step_result["settling_time_s"],
                f".{backend}",
                first_violation_step=(
                    settling_deadline_step
                    if step_result["settling_time_s"] is None
                    or step_result["settling_time_s"] > settling_requirement["maximum"]
                    else None
                ),
                settling_deadline_step=settling_deadline_step,
                settling_band_rad=step_result["settling_band_rad"],
                target_rad=settling_target_rad,
                position_at_deadline_rad=step_response[settling_deadline_index],
            ),
            upper_check(requirements["plant.ramp.maximum_tracking_rmse_rad"], ramp_rmse, f".{backend}"),
            upper_check(
                requirements["plant.model.maximum_validation_rmse_rad"],
                validation_metrics["one_step_prediction_rmse_rad"],
                f".{backend}",
            ),
            upper_check(
                requirements["plant.actuator.maximum_saturated_sample_fraction"],
                saturation_fraction,
                f".{backend}",
            ),
        ]
        for point in frequency:
            checks.append(
                lower_check(
                    requirements["plant.frequency.minimum_projection_amplitude_rad"],
                    point["input_projection_amplitude_rad"],
                    f".{backend}.{point['frequency_hz']:.3f}hz",
                )
            )
        for joint_index, (joint, limit) in enumerate(zip(order, limits)):
            for frame in observations:
                position = frame["joint_position_rad"][joint_index]
                velocity = abs(frame["joint_velocity_rad_s"][joint_index])
                if (
                    position < limit["minimum_position_rad"] - requirements["plant.hard_position_limit_epsilon_rad"]["maximum"]
                    or position > limit["maximum_position_rad"] + requirements["plant.hard_position_limit_epsilon_rad"]["maximum"]
                    or velocity > limit["maximum_velocity_rad_s"] + requirements["plant.hard_velocity_limit_epsilon_rad_s"]["maximum"]
                ):
                    hard_violations.append(
                        {
                            "backend_id": backend,
                            "step": frame["step"],
                            "joint": joint,
                            "position_rad": position,
                            "absolute_velocity_rad_s": velocity,
                            **limit,
                        }
                    )
                    break
        joint_metrics = per_joint_metrics(actions, observations, order, rate)
        tracking_rmse[backend] = joint_metrics[joint5_index]["tracking_rmse_rad"]
        plot_outputs[backend] = [
            frame["joint_position_rad"][joint5_index] for frame in observations
        ]
        backend_reports.append(
            {
                "backend_id": backend,
                "backend_version": trace["backend_version"],
                "trace_sha256": sha256(trace_path),
                "state_hash_contract": trace.get(
                    "physics_state_hash_contract", trace.get("state_hash_contract")
                ),
                "initial_state_digest": trace.get("initial_state_digest"),
                "final_state_digest": trace["final_state_digest"],
                "replay_final_state_digest": trace["replay_final_state_digest"],
                "unique_step_state_digest_count": len(
                    {
                        frame["physics_hash"]
                        for frame in observations
                        if "physics_hash" in frame
                    }
                ),
                "per_joint_time_domain": joint_metrics,
                "joint5_step_response": step_result,
                "joint5_ramp_tracking_rmse_rad": ramp_rmse,
                "joint5_frequency_response": frequency,
                "coupling_matrix": coupling,
                "arx_model": {
                    "kind": "siso_arx_2_2_with_intercept",
                    "equation": "q[k]=c0+c1*q[k-1]+c2*q[k-2]+c3*u[k-1]+c4*u[k-2]",
                    "coefficients": coefficients,
                    "training": training_metrics,
                    "independent_validation": validation_metrics,
                },
                "saturated_channel_samples": saturation_count,
                "saturated_sample_fraction": saturation_fraction,
                "saturation_semantics": saturation_semantics,
                "checks": checks,
            }
        )
        all_checks.extend(checks)

    pairwise = []
    maximum_rmse_delta = 0.0
    for left_index, left in enumerate(BACKENDS):
        for right in BACKENDS[left_index + 1 :]:
            delta = abs(tracking_rmse[left] - tracking_rmse[right])
            maximum_rmse_delta = max(maximum_rmse_delta, delta)
            pairwise.append(
                {"left": left, "right": right, "joint5_tracking_rmse_delta_rad": delta}
            )
    cross_check = upper_check(
        requirements["plant.cross_backend.maximum_joint5_rmse_delta_rad"],
        maximum_rmse_delta,
    )
    all_checks.append(cross_check)
    hard_violations.sort(key=lambda item: (item["step"], item["backend_id"], item["joint"]))
    hard_check = {
        "id": "plant.hard_joint_limits",
        "gate": "plant_integrity",
        "unit": "violation_count",
        "observed": len(hard_violations),
        "maximum": 0,
        "status": "passed" if not hard_violations else "failed",
    }
    all_checks.append(hard_check)
    status = "passed" if all(check["status"] == "passed" for check in all_checks) else "needs_tuning"
    first_failed_requirement = next(
        (check for check in all_checks if check["status"] == "failed"), None
    )
    report = {
        "kind": "rne_openarm_plant_lab_report",
        "schema_version": 1,
        "status": status,
        "experiment_id": manifest["experiment_id"],
        "task_id": manifest["task_id"],
        "controller_id": manifest["controller_id"],
        "experiment_contract": {
            "clock": "simulation_time",
            "fixed_delta_ticks": manifest["fixed_delta_ticks"],
            "sample_rate_hz": rate,
            "input": manifest["input"],
            "output": manifest["output"],
            "operating_point_rad": manifest["operating_point_rad"],
            "joint_order": order,
            "segments": manifest["segments"],
            "analysis": manifest["analysis"],
        },
        "inputs": [
            {"role": "experiment_manifest", "sha256": sha256(manifest_path)},
            {"role": "requirements_registry", "sha256": sha256(requirements_path)},
            {"role": "generated_controller", "sha256": sha256(controller_path)},
            {"role": "action_trace", "sha256": sha256(actions_path)},
            {"role": "task_spec", "sha256": sha256(task_path)},
            {"role": "robot_model", "sha256": sha256(urdf_path)},
            {"role": "actuation_config", "sha256": sha256(actuation_path)},
            {"role": "gazebo_adapter_config", "sha256": sha256(gazebo_adapter_path)},
        ],
        "requirements_registry": registry,
        "datasets": datasets,
        "backends": backend_reports,
        "cross_backend": {
            "pairwise": pairwise,
            "maximum_joint5_tracking_rmse_delta_rad": maximum_rmse_delta,
            "check": cross_check,
        },
        "intentional_failures": [
            {
                "backend_id": backend,
                "first_violation": failures[backend]["first_violation"],
                "first_violation_step": failures[backend]["first_violation_step"],
                "failure_artifact_sha256": sha256(trace_root / FAILURE_FILES[backend]),
            }
            for backend in BACKENDS
        ],
        "first_hard_contract_divergence": hard_violations[0] if hard_violations else None,
        "first_failed_requirement": first_failed_requirement,
        "checks": all_checks,
        "plot_data": {
            "step": [action["step"] for action in actions],
            "joint5_input_target_rad": [
                action["joint_position_target_rad"][joint5_index] for action in actions
            ],
            "joint5_output_position_rad": plot_outputs,
        },
    }
    report_path = output / "openarm-plant-lab-report.json"
    write_json(report_path, report)
    write_html(output / "openarm-plant-lab-report.html", report)
    print(
        f"OpenArm plant lab: status={status} backends={len(BACKENDS)} "
        f"first_hard_divergence={hard_violations[0]['step'] if hard_violations else 'none'}"
    )
    return 0


def _write_html_legacy(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    document = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenArm plant lab</title><style>
body{{margin:0;background:#09121f;color:#eef5ff;font:14px system-ui,sans-serif}}main{{max-width:1240px;margin:auto;padding:28px}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}}.card{{background:#132238;border:1px solid #2a4667;border-radius:10px;padding:14px}}table{{width:100%;border-collapse:collapse}}th,td{{border:1px solid #2a4667;padding:6px;text-align:right}}th:first-child,td:first-child{{text-align:left}}canvas{{width:100%;height:300px;background:#fff;border-radius:8px}}.passed{{color:#6ee7aa}}.needs_tuning,.failed{{color:#ffb36b}}</style></head><body><main><h1>OpenArm plant lab</h1><p>Status: <b class="{report['status']}">{report['status']}</b></p><p>First failed requirement: <code>{report['first_failed_requirement']['id'] if report['first_failed_requirement'] else 'none'}</code></p><canvas id="trace" width="1160" height="300"></canvas><div id="summary" class="grid"></div><h2>Frequency response</h2><div id="frequency"></div><h2>Cross-axis coupling</h2><div id="coupling"></div><h2>Checks</h2><div id="checks"></div><script>const r={payload};const f=x=>x==null?'n/a':Number(x).toFixed(5);const c=document.querySelector('#trace'),x=c.getContext('2d'),d=r.plot_data,n=d.step.length,colors={{rne_rapier:'#1261a0',mujoco_native:'#c2410c',gazebo_sim:'#15803d'}};function line(v,color){{x.beginPath();for(let i=0;i<n;i++){{const px=i/(n-1)*c.width,py=c.height-(v[i]+.25)/.65*c.height;i?x.lineTo(px,py):x.moveTo(px,py)}}x.strokeStyle=color;x.lineWidth=1.3;x.stroke()}}line(d.joint5_input_target_rad,'#111');Object.entries(d.joint5_output_position_rad).forEach(([k,v])=>line(v,colors[k]));document.querySelector('#summary').innerHTML=r.backends.map(b=>`<section class=card><h3>${{b.backend_id}}</h3><p>step rise / settle: ${{f(b.joint5_step_response.rise_time_s)}} / ${{f(b.joint5_step_response.settling_time_s)}} s</p><p>overshoot: ${{f(b.joint5_step_response.overshoot_fraction)}}</p><p>ramp RMSE: ${{f(b.joint5_ramp_tracking_rmse_rad)}} rad</p><p>ARX train / validation: ${{f(b.arx_model.training.one_step_prediction_rmse_rad)}} / ${{f(b.arx_model.independent_validation.one_step_prediction_rmse_rad)}} rad</p></section>`).join('');document.querySelector('#frequency').innerHTML=r.backends.map(b=>`<h3>${{b.backend_id}}</h3><table><tr><th>Hz</th><th>input rad</th><th>gain</th><th>phase rad</th></tr>${{b.joint5_frequency_response.map(p=>`<tr><td>${{f(p.frequency_hz)}}</td><td>${{f(p.input_projection_amplitude_rad)}}</td><td>${{f(p.gain_rad_per_rad)}}</td><td>${{f(p.phase_rad)}}</td></tr>`).join('')}}</table>`).join('');document.querySelector('#coupling').innerHTML=r.backends.map(b=>`<h3>${{b.backend_id}}</h3><table><tr><th>output / source</th>${{b.coupling_matrix.columns.map(v=>`<th>${{v.source_joint}}<br>${{f(v.frequency_hz)}} Hz</th>`).join('')}}</tr>${{b.coupling_matrix.output_joint_order.map((name,i)=>`<tr><td>${{name}}</td>${{b.coupling_matrix.columns.map(v=>`<td>${{f(v.output_gain_rad_per_rad[i])}}</td>`).join('')}}</tr>`).join('')}}</table>`).join('');document.querySelector('#checks').innerHTML=`<table><tr><th>requirement</th><th>observed</th><th>status</th></tr>${{r.checks.map(q=>`<tr><td>${{q.id}}</td><td>${{f(q.observed)}}</td><td class=${{q.status}}>${{q.status}}</td></tr>`).join('')}}</table>`;</script></main></body></html>"""
    path.write_text(document, encoding="utf-8")


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    document = r"""<!doctype html>
<html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>OpenArm plant lab</title><style>
body{margin:0;background:#09121f;color:#eef5ff;font:14px system-ui,sans-serif}
main{max-width:1240px;margin:auto;padding:28px}.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}
.card{background:#132238;border:1px solid #2a4667;border-radius:10px;padding:14px}.failure{border-color:#e8873a}
table{width:100%;border-collapse:collapse;margin-bottom:20px}th,td{border:1px solid #2a4667;padding:6px;text-align:right}
th:first-child,td:first-child{text-align:left}canvas{width:100%;height:300px;background:#fff;border-radius:8px}
.passed{color:#6ee7aa}.needs_tuning,.failed{color:#ffb36b}code{color:#b9ddff}
</style></head><body><main>
<h1>OpenArm plant lab</h1><div id="verdict"></div>
<p>Joint 5 target and measured response. Black: input; blue: Rapier; orange: MuJoCo; green: Gazebo.</p>
<canvas id="trace" width="1160" height="300"></canvas><div id="summary" class="grid"></div>
<h2>Frequency response</h2><div id="frequency"></div>
<h2>Cross-axis coupling</h2><div id="coupling"></div>
<h2>Fixed requirement checks</h2><div id="checks"></div>
<script>
const r=__REPORT__, f=x=>x==null?'n/a':Number(x).toFixed(5), fail=r.first_failed_requirement;
document.querySelector('#verdict').innerHTML=`<section class="card ${fail?'failure':''}"><p>Status: <b class="${r.status}">${r.status}</b></p><p>First failed requirement: <code>${fail?fail.id:'none'}</code></p><p>Gate / first violation step: <code>${fail?fail.gate:'n/a'}</code> / <code>${fail&&fail.first_violation_step!=null?fail.first_violation_step:'n/a'}</code></p></section>`;
const c=document.querySelector('#trace'),x=c.getContext('2d'),d=r.plot_data,n=d.step.length,colors={rne_rapier:'#1261a0',mujoco_native:'#c2410c',gazebo_sim:'#15803d'};
function line(v,color){x.beginPath();for(let i=0;i<n;i++){const px=i/(n-1)*c.width,py=c.height-(v[i]+.25)/.65*c.height;i?x.lineTo(px,py):x.moveTo(px,py)}x.strokeStyle=color;x.lineWidth=1.3;x.stroke()}
line(d.joint5_input_target_rad,'#111');Object.entries(d.joint5_output_position_rad).forEach(([k,v])=>line(v,colors[k]));
document.querySelector('#summary').innerHTML=r.backends.map(b=>`<section class="card"><h3>${b.backend_id}</h3><p>step rise / settle: ${f(b.joint5_step_response.rise_time_s)} / ${f(b.joint5_step_response.settling_time_s)} s</p><p>overshoot: ${f(b.joint5_step_response.overshoot_fraction)}</p><p>ramp RMSE: ${f(b.joint5_ramp_tracking_rmse_rad)} rad</p><p>ARX train / validation: ${f(b.arx_model.training.one_step_prediction_rmse_rad)} / ${f(b.arx_model.independent_validation.one_step_prediction_rmse_rad)} rad</p></section>`).join('');
document.querySelector('#frequency').innerHTML=r.backends.map(b=>`<h3>${b.backend_id}</h3><table><tr><th>Hz</th><th>input rad</th><th>gain</th><th>phase rad</th></tr>${b.joint5_frequency_response.map(p=>`<tr><td>${f(p.frequency_hz)}</td><td>${f(p.input_projection_amplitude_rad)}</td><td>${f(p.gain_rad_per_rad)}</td><td>${f(p.phase_rad)}</td></tr>`).join('')}</table>`).join('');
document.querySelector('#coupling').innerHTML=r.backends.map(b=>`<h3>${b.backend_id}</h3><table><tr><th>output / source</th>${b.coupling_matrix.columns.map(v=>`<th>${v.source_joint}<br>${f(v.frequency_hz)} Hz</th>`).join('')}</tr>${b.coupling_matrix.output_joint_order.map((name,i)=>`<tr><td>${name}</td>${b.coupling_matrix.columns.map(v=>`<td>${f(v.output_gain_rad_per_rad[i])}</td>`).join('')}</tr>`).join('')}</table>`).join('');
document.querySelector('#checks').innerHTML=`<table><tr><th>requirement</th><th>gate</th><th>observed</th><th>limit</th><th>first step</th><th>status</th></tr>${r.checks.map(q=>`<tr><td>${q.id}</td><td>${q.gate}</td><td>${f(q.observed)} ${q.unit}</td><td>${q.maximum!=null?'≤ '+f(q.maximum):'≥ '+f(q.minimum)} ${q.unit}</td><td>${q.first_violation_step==null?'n/a':q.first_violation_step}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`;
</script></main></body></html>""".replace("__REPORT__", payload)
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm plant report failed: {error}", file=sys.stderr)
        raise SystemExit(2)

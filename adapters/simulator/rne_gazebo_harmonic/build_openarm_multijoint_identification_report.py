#!/usr/bin/env python3
"""Build held-out MIMO identification evidence for the OpenArm right arm."""

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
        / "adapters/simulator/rne_gazebo_harmonic/openarm_multijoint_identification_experiments.json",
    )
    parser.add_argument(
        "--requirements",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_multijoint_identification_requirements.json",
    )
    parser.add_argument(
        "--controller",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_multijoint_identification.controller.json",
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


def json_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), allow_nan=False
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def rms(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        raise ValueError("cannot calculate RMS of an empty sequence")
    return math.sqrt(sum(value * value for value in values) / len(values))


def segment(manifest: dict[str, Any], identifier: str) -> dict[str, Any]:
    try:
        return next(item for item in manifest["segments"] if item["id"] == identifier)
    except StopIteration as error:
        raise ValueError(f"missing experiment segment {identifier}") from error


def requirement_map(registry: dict[str, Any]) -> dict[str, dict[str, Any]]:
    if (
        set(registry) != {"kind", "schema_version", "registry_id", "requirements"}
        or registry.get("kind")
        != "rne_openarm_multijoint_identification_requirements_registry"
        or registry.get("schema_version") != 1
    ):
        raise ValueError("unsupported OpenArm multijoint requirements registry")
    requirements = registry.get("requirements")
    if not isinstance(requirements, list):
        raise ValueError("requirements registry has no requirement list")
    by_id = {item["id"]: item for item in requirements}
    if len(by_id) != len(requirements):
        raise ValueError("requirements registry contains duplicate ids")
    for item in requirements:
        expected = (
            {"id", "gate", "unit", "maximum"}
            if "maximum" in item
            else {"id", "gate", "unit", "minimum"}
        )
        if set(item) != expected or item["gate"] not in QUALITY_GATES:
            raise ValueError(f"invalid requirement {item.get('id')}")
    return by_id


def upper_check(
    requirement: dict[str, Any], observed: float, suffix: str = ""
) -> dict[str, Any]:
    return {
        "id": requirement["id"] + suffix,
        "gate": requirement["gate"],
        "unit": requirement["unit"],
        "observed": observed,
        "maximum": requirement["maximum"],
        "status": "passed"
        if math.isfinite(observed) and observed <= requirement["maximum"]
        else "failed",
    }


def lower_check(
    requirement: dict[str, Any], observed: float, suffix: str = ""
) -> dict[str, Any]:
    return {
        "id": requirement["id"] + suffix,
        "gate": requirement["gate"],
        "unit": requirement["unit"],
        "observed": observed,
        "minimum": requirement["minimum"],
        "status": "passed"
        if math.isfinite(observed) and observed >= requirement["minimum"]
        else "failed",
    }


def residual_autocorrelation_check(
    requirement: dict[str, Any],
    numerical_floor_requirement: dict[str, Any],
    metrics: dict[str, Any],
    suffix: str,
) -> dict[str, Any]:
    observed = metrics["maximum_absolute_residual_autocorrelation"]
    residual_rmse = metrics["one_step_prediction_rmse_rad"]
    autocorrelation_passed = math.isfinite(observed) and observed <= requirement["maximum"]
    numerical_exactness_passed = (
        math.isfinite(residual_rmse)
        and residual_rmse <= numerical_floor_requirement["maximum"]
    )
    return {
        "id": requirement["id"] + suffix,
        "gate": requirement["gate"],
        "unit": requirement["unit"],
        "observed": observed,
        "maximum": requirement["maximum"],
        "status": "passed"
        if autocorrelation_passed or numerical_exactness_passed
        else "failed",
        "acceptance_path": (
            "autocorrelation_bound"
            if autocorrelation_passed
            else "numerical_exactness_floor"
            if numerical_exactness_passed
            else "none"
        ),
        "residual_rmse_rad": residual_rmse,
        "numerical_exactness_requirement_id": numerical_floor_requirement["id"],
        "numerical_exactness_maximum_rad": numerical_floor_requirement["maximum"],
    }


def compile_declared_controller(manifest_path: Path) -> dict[str, Any]:
    compiler_path = manifest_path.parent / "build_openarm_plant_controller.py"
    spec = importlib.util.spec_from_file_location(
        "rne_openarm_multijoint_compiler", compiler_path
    )
    if spec is None or spec.loader is None:
        raise ValueError("cannot load the OpenArm plant compiler")
    compiler = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(compiler)
    return compiler.compile_controller(compiler.load_manifest(manifest_path))


def solve(matrix: list[list[float]], vector: list[float]) -> list[float]:
    augmented = [row[:] + [value] for row, value in zip(matrix, vector)]
    width = len(vector)
    for column in range(width):
        pivot = max(range(column, width), key=lambda row: abs(augmented[row][column]))
        if abs(augmented[pivot][column]) < 1e-18:
            raise ValueError("MIMO ARX normal equation is singular")
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


def inverse(matrix: list[list[float]]) -> list[list[float]]:
    width = len(matrix)
    columns = []
    for index in range(width):
        unit = [0.0] * width
        unit[index] = 1.0
        columns.append(solve(matrix, unit))
    return [[columns[column][row] for column in range(width)] for row in range(width)]


def matrix_rank(rows: list[list[float]]) -> int:
    work = [row[:] for row in rows]
    if not work:
        return 0
    height = len(work)
    width = len(work[0])
    for column in range(width):
        column_scale = max(abs(row[column]) for row in work)
        if column_scale > 0.0:
            for row in work:
                row[column] /= column_scale
    tolerance = 1e-10
    rank = 0
    for column in range(width):
        pivot = max(range(rank, height), key=lambda row: abs(work[row][column]))
        if abs(work[pivot][column]) <= tolerance:
            continue
        work[rank], work[pivot] = work[pivot], work[rank]
        divisor = work[rank][column]
        for value_index in range(column, width):
            work[rank][value_index] /= divisor
        for row in range(rank + 1, height):
            factor = work[row][column]
            if abs(factor) <= tolerance:
                continue
            for value_index in range(column, width):
                work[row][value_index] -= factor * work[rank][value_index]
        rank += 1
        if rank == height:
            break
    return rank


def window_values(
    actions: list[dict[str, Any]],
    observations: list[dict[str, Any]],
    window: dict[str, Any],
    indices: list[int],
    operating: list[float],
) -> tuple[list[list[float]], list[list[float]], list[list[float]]]:
    start = window["start_step"] - 1
    end = window["end_step"]
    inputs = [
        [action["joint_position_target_rad"][index] - operating[index] for index in indices]
        for action in actions[start:end]
    ]
    outputs = [
        [frame["joint_position_rad"][index] - operating[index] for index in indices]
        for frame in observations[start:end]
    ]
    velocities = [
        [frame["joint_velocity_rad_s"][index] for index in indices]
        for frame in observations[start:end]
    ]
    return inputs, outputs, velocities


def state_input_rows(
    inputs: list[list[float]],
    positions: list[list[float]],
    velocities: list[list[float]],
    output_index: int,
) -> tuple[list[list[float]], list[float]]:
    rows = []
    expected = []
    for sample in range(1, len(positions)):
        rows.append(
            [
                1.0,
                *positions[sample - 1],
                *velocities[sample - 1],
                *inputs[sample],
            ]
        )
        expected.append(positions[sample][output_index])
    return rows, expected


def fit_model(
    rows: list[list[float]], expected: list[float], ridge_lambda: float
) -> tuple[list[float], list[list[float]]]:
    width = len(rows[0])
    normal = [[0.0] * width for _ in range(width)]
    rhs = [0.0] * width
    for row, output in zip(rows, expected):
        for left in range(width):
            rhs[left] += row[left] * output
            for right in range(width):
                normal[left][right] += row[left] * row[right]
    for index in range(width):
        normal[index][index] += ridge_lambda
    return solve(normal, rhs), inverse(normal)


def quadratic_form(vector: list[float], matrix: list[list[float]]) -> float:
    return sum(
        vector[left] * matrix[left][right] * vector[right]
        for left in range(len(vector))
        for right in range(len(vector))
    )


def prediction_metrics(
    coefficients: list[float],
    covariance_basis: list[list[float]],
    training_rows: list[list[float]],
    training_expected: list[float],
    rows: list[list[float]],
    expected: list[float],
    maximum_lag: int,
    confidence_z: float,
) -> tuple[dict[str, Any], list[float]]:
    training_residuals = [
        sum(coefficient * value for coefficient, value in zip(coefficients, row))
        - output
        for row, output in zip(training_rows, training_expected)
    ]
    degrees = max(1, len(training_residuals) - len(coefficients))
    variance = sum(value * value for value in training_residuals) / degrees
    predictions = [
        sum(coefficient * value for coefficient, value in zip(coefficients, row))
        for row in rows
    ]
    residuals = [prediction - output for prediction, output in zip(predictions, expected)]
    mean = sum(residuals) / len(residuals)
    centered = [value - mean for value in residuals]
    denominator = sum(value * value for value in centered)
    autocorrelation = []
    for lag in range(1, maximum_lag + 1):
        value = (
            sum(centered[index] * centered[index - lag] for index in range(lag, len(centered)))
            / denominator
            if denominator > 0.0
            else 0.0
        )
        autocorrelation.append({"lag": lag, "value": value})
    half_widths = [
        confidence_z
        * math.sqrt(max(0.0, variance * (1.0 + quadratic_form(row, covariance_basis))))
        for row in rows
    ]
    return (
        {
            "sample_count": len(residuals),
            "one_step_prediction_rmse_rad": rms(residuals),
            "maximum_absolute_residual_rad": max(abs(value) for value in residuals),
            "mean_residual_rad": mean,
            "maximum_absolute_residual_autocorrelation": max(
                abs(item["value"]) for item in autocorrelation
            ),
            "residual_autocorrelation": autocorrelation,
            "residual_variance_rad2": variance,
            "maximum_95_prediction_half_width_rad": max(half_widths),
        },
        predictions,
    )


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


def coherence(
    inputs: list[float],
    outputs: list[float],
    sample_rate_hz: float,
    frequency_hz: float,
    window_samples: int,
    overlap_samples: int,
) -> float:
    stride = window_samples - overlap_samples
    if stride <= 0:
        raise ValueError("coherence window overlap must be smaller than its window")
    spectra = []
    for start in range(0, len(inputs) - window_samples + 1, stride):
        end = start + window_samples
        spectra.append(
            (
                complex_projection(inputs[start:end], sample_rate_hz, frequency_hz),
                complex_projection(outputs[start:end], sample_rate_hz, frequency_hz),
            )
        )
    if len(spectra) < 2:
        raise ValueError("coherence estimator requires at least two windows")
    cross = sum(left.conjugate() * right for left, right in spectra)
    input_power = sum(abs(left) ** 2 for left, _ in spectra)
    output_power = sum(abs(right) ** 2 for _, right in spectra)
    if input_power <= 0.0 or output_power <= 0.0:
        return 0.0
    return min(1.0, abs(cross) ** 2 / (input_power * output_power))


def coupling_matrix(
    manifest: dict[str, Any],
    inputs: list[list[float]],
    outputs: list[list[float]],
) -> dict[str, Any]:
    validation = segment(manifest, manifest["analysis"]["validation_segment"])
    rate = manifest["sample_rate_hz"]
    columns = []
    for source_index, source in enumerate(validation["sources"]):
        gains = [[] for _ in outputs[0]]
        for frequency in source["frequencies_hz"]:
            input_projection = complex_projection(
                [row[source_index] for row in inputs], rate, frequency
            )
            for output_index in range(len(outputs[0])):
                output_projection = complex_projection(
                    [row[output_index] for row in outputs], rate, frequency
                )
                gains[output_index].append(abs(output_projection / input_projection))
        columns.append(
            {
                "source_joint": source["joint"],
                "frequencies_hz": source["frequencies_hz"],
                "mean_output_gain_rad_per_rad": [
                    sum(values) / len(values) for values in gains
                ],
            }
        )
    return {
        "output_joint_order": manifest["analysis"]["identified_joint_order"],
        "columns": columns,
    }


def write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    document = r"""<!doctype html><html lang="en"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenArm multijoint identification</title><style>
body{margin:0;background:#09121f;color:#eef5ff;font:14px system-ui,sans-serif}main{max-width:1320px;margin:auto;padding:28px}
.card{background:#132238;border:1px solid #2a4667;border-radius:10px;padding:14px;margin-bottom:16px}.failed{color:#ffb36b}.passed{color:#6ee7aa}
table{width:100%;border-collapse:collapse;margin-bottom:24px}th,td{border:1px solid #2a4667;padding:6px;text-align:right}th:first-child,td:first-child{text-align:left}
code{color:#b9ddff}</style></head><body><main><h1>OpenArm seven-joint held-out MIMO identification</h1><div id="verdict"></div>
<h2>Per-joint held-out evidence</h2><div id="models"></div><h2>Coupled-mode gain matrices</h2><div id="coupling"></div>
<h2>Fixed requirements</h2><div id="checks"></div><script>
const r=__REPORT__,f=x=>Number(x).toFixed(6),fail=r.first_failed_requirement;
document.querySelector('#verdict').innerHTML=`<section class=card><p>Status: <b class=${r.status}>${r.status}</b></p><p>Training: seven isolated multisine regions; validation: one simultaneous seven-input region; validation refit: <code>false</code></p><p>First failed requirement: <code>${fail?fail.id:'none'}</code></p></section>`;
document.querySelector('#models').innerHTML=r.backends.map(b=>`<h3>${b.backend_id}</h3><table><tr><th>output</th><th>rank</th><th>min coherence</th><th>validation RMSE rad</th><th>|residual mean| rad</th><th>max residual ACF</th><th>95% half-width rad</th></tr>${b.models.map(m=>`<tr><td>${m.output_joint}</td><td>${m.training_regressor_rank}</td><td>${f(m.minimum_diagonal_coherence)}</td><td>${f(m.validation.one_step_prediction_rmse_rad)}</td><td>${f(Math.abs(m.validation.mean_residual_rad))}</td><td>${f(m.validation.maximum_absolute_residual_autocorrelation)}</td><td>${f(m.validation.maximum_95_prediction_half_width_rad)}</td></tr>`).join('')}</table>`).join('');
document.querySelector('#coupling').innerHTML=r.backends.map(b=>`<h3>${b.backend_id}</h3><table><tr><th>output / input</th>${b.coupling_matrix.columns.map(c=>`<th>${c.source_joint.replace('openarm_right_','')}</th>`).join('')}</tr>${b.coupling_matrix.output_joint_order.map((j,i)=>`<tr><td>${j.replace('openarm_right_','')}</td>${b.coupling_matrix.columns.map(c=>`<td>${f(c.mean_output_gain_rad_per_rad[i])}</td>`).join('')}</tr>`).join('')}</table>`).join('');
document.querySelector('#checks').innerHTML=`<table><tr><th>requirement</th><th>gate</th><th>observed</th><th>limit</th><th>status</th></tr>${r.checks.map(q=>`<tr><td>${q.id}</td><td>${q.gate}</td><td>${f(q.observed)} ${q.unit}</td><td>${q.maximum!=null?'≤ '+f(q.maximum):'≥ '+f(q.minimum)} ${q.unit}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`;
</script></main></body></html>""".replace("__REPORT__", payload)
    path.write_text(document, encoding="utf-8")


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
        raise ValueError("generated controller differs from the experiment manifest")
    actions_path = trace_root / "controller-actions.json"
    actions_artifact = load(actions_path)
    actions = actions_artifact["actions"]
    order = manifest["action_joint_order"]
    identified = manifest["analysis"]["identified_joint_order"]
    indices = [order.index(joint) for joint in identified]
    operating = manifest["operating_point_rad"]
    final_step = manifest["segments"][-1]["end_step"]
    task_path = repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json"
    urdf_path = repo / "assets/robots/openarm_description/openarm_v2_right.rne.urdf"
    actuation_path = repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json"
    gazebo_adapter_path = repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json"
    if (
        manifest.get("experiment_id") != "rne.openarm.right.multijoint_identification.v1"
        or identified != order[:7]
        or len(manifest["analysis"]["training_segments"]) != len(identified)
        or actions_artifact.get("controller_sha256") != sha256(controller_path)
        or actions_artifact.get("task_sha256") != sha256(task_path)
        or actions_artifact.get("action_joint_order") != order
        or len(actions) != final_step
        or actions[-1].get("step") != final_step
    ):
        raise ValueError("multijoint manifest/controller/action identity drifted")

    training_windows = [
        segment(manifest, identifier)
        for identifier in manifest["analysis"]["training_segments"]
    ]
    validation_window = segment(manifest, manifest["analysis"]["validation_segment"])
    maximum_offset = max(
        abs(action["joint_position_target_rad"][index] - operating[index])
        for action in actions[300:]
        for index in indices
    )
    all_checks = [
        upper_check(requirements["identification.input.maximum_offset_rad"], maximum_offset)
    ]
    training_rms: dict[str, float] = {}
    for joint_index, (joint, window) in enumerate(zip(identified, training_windows)):
        start = window["start_step"] - 1
        end = window["end_step"]
        value = rms(
            actions[sample]["joint_position_target_rad"][indices[joint_index]]
            - operating[indices[joint_index]]
            for sample in range(start, end)
        )
        training_rms[joint] = value
        all_checks.append(
            lower_check(
                requirements["identification.input.minimum_training_rms_rad"],
                value,
                f".{joint}",
            )
        )
    validation_start = validation_window["start_step"] - 1
    validation_end = validation_window["end_step"]
    validation_rms = {}
    for joint_index, joint in enumerate(identified):
        value = rms(
            actions[sample]["joint_position_target_rad"][indices[joint_index]]
            - operating[indices[joint_index]]
            for sample in range(validation_start, validation_end)
        )
        validation_rms[joint] = value
        all_checks.append(
            lower_check(
                requirements["identification.input.minimum_validation_rms_rad"],
                value,
                f".{joint}",
            )
        )

    plant_module_path = manifest_path.parent / "build_openarm_plant_report.py"
    plant_spec = importlib.util.spec_from_file_location("rne_openarm_plant_math", plant_module_path)
    if plant_spec is None or plant_spec.loader is None:
        raise ValueError("cannot load shared OpenArm plant math")
    plant_math = importlib.util.module_from_spec(plant_spec)
    plant_spec.loader.exec_module(plant_math)
    limits = plant_math.joint_limits(urdf_path, order)
    backend_reports = []
    validation_rmse_by_backend: dict[str, dict[str, float]] = {}
    hard_violations = []
    for backend in BACKENDS:
        trace_path = trace_root / TRACE_FILES[backend]
        failure_path = trace_root / FAILURE_FILES[backend]
        trace = load(trace_path)
        failure = load(failure_path)
        observations = trace["observations"]
        if (
            trace.get("backend_id") != backend
            or trace.get("controller_sha256") != sha256(controller_path)
            or trace.get("action_trace_sha256") != sha256(actions_path)
            or trace.get("controller_execution") != "open_loop_reference"
            or not trace.get("replay_match")
            or len(observations) != len(actions)
            or failure.get("status") != "failed_as_expected"
            or failure.get("first_violation") != "action_width_mismatch"
            or failure.get("first_violation_step")
            != manifest["intentional_failure"]["inject_at_step"]
        ):
            raise ValueError(f"{backend} trace or intentional failure drifted")
        for step, observation in enumerate(observations, 1):
            if (
                observation.get("step") != step
                or observation.get("sim_time_ticks")
                != step * manifest["fixed_delta_ticks"]
                or observation.get("sensor_status") != "nominal"
                or len(observation.get("joint_position_rad", [])) != len(order)
                or len(observation.get("joint_velocity_rad_s", [])) != len(order)
            ):
                raise ValueError(f"{backend} observation drifted at step {step}")

        training_data = [
            window_values(actions, observations, window, indices, operating)
            for window in training_windows
        ]
        validation_inputs, validation_outputs, validation_velocities = window_values(
            actions, observations, validation_window, indices, operating
        )
        models = []
        backend_checks = []
        validation_rmse_by_backend[backend] = {}
        for output_index, output_joint in enumerate(identified):
            training_rows: list[list[float]] = []
            training_expected: list[float] = []
            for inputs, outputs, velocities in training_data:
                rows, expected = state_input_rows(
                    inputs,
                    outputs,
                    velocities,
                    output_index,
                )
                training_rows.extend(rows)
                training_expected.extend(expected)
            validation_rows, validation_expected = state_input_rows(
                validation_inputs,
                validation_outputs,
                validation_velocities,
                output_index,
            )
            rank = matrix_rank(training_rows)
            coefficients, covariance_basis = fit_model(
                training_rows,
                training_expected,
                manifest["analysis"]["ridge_lambda"],
            )
            metrics, predictions = prediction_metrics(
                coefficients,
                covariance_basis,
                training_rows,
                training_expected,
                validation_rows,
                validation_expected,
                manifest["analysis"]["residual_autocorrelation_max_lag"],
                manifest["analysis"]["confidence_z"],
            )
            source_inputs, source_outputs, _ = training_data[output_index]
            source = training_windows[output_index]
            diagonal_coherence = [
                {
                    "frequency_hz": frequency,
                    "magnitude_squared_coherence": coherence(
                        [row[output_index] for row in source_inputs],
                        [row[output_index] for row in source_outputs],
                        manifest["sample_rate_hz"],
                        frequency,
                        manifest["analysis"]["coherence_window_samples"],
                        manifest["analysis"]["coherence_overlap_samples"],
                    ),
                }
                for frequency in source["frequencies_hz"]
            ]
            minimum_coherence = min(
                item["magnitude_squared_coherence"] for item in diagonal_coherence
            )
            validation_rmse_by_backend[backend][output_joint] = metrics[
                "one_step_prediction_rmse_rad"
            ]
            suffix = f".{backend}.{output_joint}"
            checks = [
                lower_check(
                    requirements["identification.design.minimum_regressor_rank"],
                    rank,
                    suffix,
                ),
                lower_check(
                    requirements["identification.frequency.minimum_diagonal_coherence"],
                    minimum_coherence,
                    suffix,
                ),
                upper_check(
                    requirements["identification.model.maximum_validation_rmse_rad"],
                    metrics["one_step_prediction_rmse_rad"],
                    suffix,
                ),
                upper_check(
                    requirements[
                        "identification.model.maximum_absolute_residual_mean_rad"
                    ],
                    abs(metrics["mean_residual_rad"]),
                    suffix,
                ),
                residual_autocorrelation_check(
                    requirements[
                        "identification.model.maximum_absolute_residual_autocorrelation"
                    ],
                    requirements[
                        "identification.model.maximum_residual_rmse_for_autocorrelation_relevance_rad"
                    ],
                    metrics,
                    suffix,
                ),
                upper_check(
                    requirements[
                        "identification.model.maximum_95_prediction_half_width_rad"
                    ],
                    metrics["maximum_95_prediction_half_width_rad"],
                    suffix,
                ),
            ]
            backend_checks.extend(checks)
            models.append(
                {
                    "output_joint": output_joint,
                    "kind": manifest["analysis"]["plant_model"],
                    "equation": "q_i[k]=c+sum_j(aqj*q_j[k-1]+avj*v_j[k-1]+bj*u_j[k])",
                    "coefficient_order": [
                        "intercept",
                        *[
                            f"{joint}.position_state" for joint in identified
                        ],
                        *[
                            f"{joint}.velocity_state" for joint in identified
                        ],
                        *[
                            f"{joint}.input_interval_k" for joint in identified
                        ],
                    ],
                    "coefficients": coefficients,
                    "training_regressor_rank": rank,
                    "training_sample_count": len(training_rows),
                    "validation_refit": False,
                    "diagonal_coherence": diagonal_coherence,
                    "minimum_diagonal_coherence": minimum_coherence,
                    "validation": metrics,
                    "validation_prediction_rad": predictions,
                }
            )
        all_checks.extend(backend_checks)
        for joint_index, (joint, limit) in enumerate(zip(order, limits)):
            for observation in observations:
                position = observation["joint_position_rad"][joint_index]
                velocity = abs(observation["joint_velocity_rad_s"][joint_index])
                if (
                    position
                    < limit["minimum_position_rad"]
                    - requirements["identification.hard_position_limit_epsilon_rad"][
                        "maximum"
                    ]
                    or position
                    > limit["maximum_position_rad"]
                    + requirements["identification.hard_position_limit_epsilon_rad"][
                        "maximum"
                    ]
                    or velocity
                    > limit["maximum_velocity_rad_s"]
                    + requirements[
                        "identification.hard_velocity_limit_epsilon_rad_s"
                    ]["maximum"]
                ):
                    hard_violations.append(
                        {
                            "backend_id": backend,
                            "step": observation["step"],
                            "joint": joint,
                            "position_rad": position,
                            "absolute_velocity_rad_s": velocity,
                            **limit,
                        }
                    )
                    break
        normalized_dataset = {
            "training": training_data,
            "validation_inputs": validation_inputs,
            "validation_outputs": validation_outputs,
            "validation_velocities": validation_velocities,
        }
        backend_reports.append(
            {
                "backend_id": backend,
                "backend_version": trace["backend_version"],
                "trace_sha256": sha256(trace_path),
                "dataset_content_sha256": json_sha256(normalized_dataset),
                "replay_match": trace["replay_match"],
                "initial_state_digest": trace.get("initial_state_digest"),
                "final_state_digest": trace["final_state_digest"],
                "replay_final_state_digest": trace["replay_final_state_digest"],
                "models": models,
                "coupling_matrix": coupling_matrix(
                    manifest, validation_inputs, validation_outputs
                ),
                "checks": backend_checks,
            }
        )

    cross_backend = []
    for joint in identified:
        values = [validation_rmse_by_backend[backend][joint] for backend in BACKENDS]
        delta = max(values) - min(values)
        check = upper_check(
            requirements[
                "identification.cross_backend.maximum_validation_rmse_delta_rad"
            ],
            delta,
            f".{joint}",
        )
        all_checks.append(check)
        cross_backend.append(
            {
                "joint": joint,
                "validation_rmse_rad": {
                    backend: validation_rmse_by_backend[backend][joint]
                    for backend in BACKENDS
                },
                "maximum_delta_rad": delta,
                "check": check,
            }
        )
    hard_violations.sort(
        key=lambda item: (item["step"], item["backend_id"], item["joint"])
    )
    hard_check = {
        "id": "identification.hard_joint_limits",
        "gate": "plant_integrity",
        "unit": "violation_count",
        "observed": len(hard_violations),
        "maximum": 0,
        "status": "passed" if not hard_violations else "failed",
    }
    all_checks.append(hard_check)
    status = (
        "passed"
        if all(check["status"] == "passed" for check in all_checks)
        else "needs_tuning"
    )
    first_failed = next(
        (check for check in all_checks if check["status"] == "failed"), None
    )
    report = {
        "kind": "rne_openarm_multijoint_identification_report",
        "schema_version": 1,
        "status": status,
        "experiment_id": manifest["experiment_id"],
        "task_id": manifest["task_id"],
        "controller_id": manifest["controller_id"],
        "contract": {
            "clock": "simulation_time",
            "fixed_delta_ticks": manifest["fixed_delta_ticks"],
            "sample_rate_hz": manifest["sample_rate_hz"],
            "identified_joint_order": identified,
            "training_segments": manifest["analysis"]["training_segments"],
            "validation_segment": manifest["analysis"]["validation_segment"],
            "validation_refit": False,
            "model": manifest["analysis"]["plant_model"],
            "state_fields": manifest["analysis"]["state_fields"],
            "state_lags": manifest["analysis"]["state_lags"],
            "input_delay_steps": manifest["analysis"]["input_delay_steps"],
            "input_alignment": manifest["analysis"]["input_alignment"],
            "uncertainty": {
                "confidence_level": manifest["analysis"]["confidence_level"],
                "method": "training_residual_variance_times_regularized_normal_inverse_v1",
            },
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
        "operating_region": {
            "operating_point_rad": [operating[index] for index in indices],
            "maximum_input_offset_rad": maximum_offset,
            "training_input_rms_rad": training_rms,
            "validation_input_rms_rad": validation_rms,
        },
        "backends": backend_reports,
        "cross_backend": cross_backend,
        "intentional_failures": [
            {
                "backend_id": backend,
                "first_violation": "action_width_mismatch",
                "first_violation_step": manifest["intentional_failure"][
                    "inject_at_step"
                ],
                "failure_artifact_sha256": sha256(trace_root / FAILURE_FILES[backend]),
            }
            for backend in BACKENDS
        ],
        "first_hard_contract_divergence": hard_violations[0]
        if hard_violations
        else None,
        "first_failed_requirement": first_failed,
        "checks": all_checks,
    }
    report_path = output / "openarm-multijoint-identification-report.json"
    write_json(report_path, report)
    write_html(output / "openarm-multijoint-identification-report.html", report)
    print(
        f"OpenArm multijoint identification: status={status} "
        f"backends={len(BACKENDS)} models={len(BACKENDS) * len(identified)}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm multijoint identification report failed: {error}", file=sys.stderr)
        raise SystemExit(2)

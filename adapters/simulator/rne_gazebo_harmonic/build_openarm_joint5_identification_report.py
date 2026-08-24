#!/usr/bin/env python3
"""Identifies OpenArm joint-5 self dynamics and coupled-motion residuals."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any
import xml.etree.ElementTree as ET


JOINT_NAME = "openarm_right_joint5"
ISOLATED_RMSE_LIMIT_RAD = 0.05
COUPLED_RMSE_LIMIT_RAD = 0.10
POSITION_EPSILON_RAD = 1e-6
TRAIN_START_STEP = 201
TRAIN_END_STEP = 1200
VALIDATION_START_STEP = 1201
VALIDATION_END_STEP = 1800


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=root)
    parser.add_argument(
        "--controller",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_joint5_identification.controller.json",
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


def arx_rows(
    positions: list[float], targets: list[float], start_step: int, end_step: int
) -> tuple[list[list[float]], list[float]]:
    rows: list[list[float]] = []
    outputs: list[float] = []
    for output_step in range(max(start_step, 3), end_step + 1):
        output_index = output_step - 1
        rows.append(
            [
                1.0,
                positions[output_index - 1],
                positions[output_index - 2],
                targets[output_index - 1],
                targets[output_index - 2],
            ]
        )
        outputs.append(positions[output_index])
    return rows, outputs


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


def prediction_metrics(
    coefficients: list[float], rows: list[list[float]], outputs: list[float]
) -> dict[str, float]:
    errors = [
        sum(coefficient * value for coefficient, value in zip(coefficients, row))
        - output
        for row, output in zip(rows, outputs)
    ]
    return {
        "sample_count": len(errors),
        "one_step_prediction_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "maximum_absolute_residual_rad": max(abs(error) for error in errors),
    }


def discrete_poles(coefficients: list[float], dt_s: float) -> list[dict[str, float]]:
    a1 = coefficients[1]
    a2 = coefficients[2]
    discriminant = complex(a1 * a1 + 4.0 * a2, 0.0) ** 0.5
    values = [(a1 + discriminant) / 2.0, (a1 - discriminant) / 2.0]
    poles: list[dict[str, float]] = []
    for value in values:
        magnitude = abs(value)
        angle = math.atan2(value.imag, value.real)
        sigma = math.log(max(magnitude, 1e-15)) / dt_s
        omega = angle / dt_s
        natural_frequency = math.hypot(sigma, omega)
        poles.append(
            {
                "real": value.real,
                "imaginary": value.imag,
                "magnitude": magnitude,
                "equivalent_natural_frequency_rad_s": natural_frequency,
                "equivalent_damping_ratio": (
                    -sigma / natural_frequency if natural_frequency > 0.0 else 1.0
                ),
            }
        )
    return poles


def check(identifier: str, observed: float, maximum: float) -> dict[str, Any]:
    return {
        "id": identifier,
        "unit": "rad",
        "observed": observed,
        "maximum": maximum,
        "status": "passed"
        if math.isfinite(observed) and observed <= maximum
        else "failed",
    }


def phase_metrics(
    actions: list[dict[str, Any]], observations: list[dict[str, Any]], joint_index: int
) -> list[dict[str, Any]]:
    ordered_phases: list[str] = []
    grouped: dict[str, list[float]] = {}
    for action, observation in zip(actions, observations):
        phase = action["phase"]
        if phase not in grouped:
            grouped[phase] = []
            ordered_phases.append(phase)
        grouped[phase].append(
            observation["joint_position_rad"][joint_index]
            - action["joint_position_target_rad"][joint_index]
        )
    return [
        {
            "phase": phase,
            "sample_count": len(grouped[phase]),
            "tracking_rmse_rad": math.sqrt(
                sum(error * error for error in grouped[phase]) / len(grouped[phase])
            ),
            "maximum_absolute_tracking_error_rad": max(
                abs(error) for error in grouped[phase]
            ),
        }
        for phase in ordered_phases
    ]


def main() -> int:
    args = parse_args()
    repo = args.repo_root.resolve()
    trace_root = args.trace_root.resolve()
    output = args.output.resolve()
    controller_path = args.controller.resolve()
    task_path = (
        repo
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json"
    )
    actuation_path = (
        repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json"
    )
    gazebo_runtime_path = repo / "adapters/simulator/rne_gazebo_harmonic/runtime.json"
    gazebo_config_path = (
        repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json"
    )
    model_path = repo / "assets/robots/openarm_description/openarm_v2_right.rne.urdf"
    actions_path = trace_root / "controller-actions.json"
    rapier_path = trace_root / "rapier-success-trace.json"
    gazebo_path = trace_root / "gazebo-success-trace.json"
    actions_artifact = load(actions_path)
    actions = actions_artifact["actions"]
    controller = load(controller_path)
    joint_order = actions_artifact["action_joint_order"]
    joint_index = joint_order.index(JOINT_NAME)
    dt_s = actions_artifact["fixed_delta_ticks"] / 1_000_000_000.0
    if (
        actions_artifact["task_sha256"] != sha256(task_path)
        or actions_artifact["controller_sha256"] != sha256(controller_path)
        or actions_artifact["controller_id"] != controller["controller_id"]
        or len(actions) != VALIDATION_END_STEP
    ):
        raise ValueError("identification action trace identity or length drifted")

    limit_node = next(
        joint.find("limit")
        for joint in ET.parse(model_path).getroot().findall("joint")
        if joint.attrib["name"] == JOINT_NAME
    )
    position_minimum = float(limit_node.attrib["lower"])
    position_maximum = float(limit_node.attrib["upper"])
    backend_reports: list[dict[str, Any]] = []
    first_violations: list[dict[str, Any]] = []
    for backend_id, trace_path in (
        ("rne_rapier", rapier_path),
        ("gazebo_sim", gazebo_path),
    ):
        trace = load(trace_path)
        if (
            trace["backend_id"] != backend_id
            or trace["task_sha256"] != sha256(task_path)
            or trace["controller_sha256"] != sha256(controller_path)
            or trace["action_trace_sha256"] != sha256(actions_path)
            or len(trace["observations"]) != len(actions)
        ):
            raise ValueError(f"{backend_id} identification trace identity drifted")
        observations = trace["observations"]
        positions = [frame["joint_position_rad"][joint_index] for frame in observations]
        targets = [
            action["joint_position_target_rad"][joint_index] for action in actions
        ]
        errors = [position - target for position, target in zip(positions, targets)]
        isolated = errors[TRAIN_START_STEP - 1 : TRAIN_END_STEP]
        coupled = errors[VALIDATION_START_STEP - 1 : VALIDATION_END_STEP]
        isolated_rmse = math.sqrt(
            sum(error * error for error in isolated) / len(isolated)
        )
        coupled_rmse = math.sqrt(sum(error * error for error in coupled) / len(coupled))
        train_rows, train_outputs = arx_rows(
            positions, targets, TRAIN_START_STEP, TRAIN_END_STEP
        )
        validation_rows, validation_outputs = arx_rows(
            positions, targets, VALIDATION_START_STEP, VALIDATION_END_STEP
        )
        coefficients = fit_arx(train_rows, train_outputs)
        violation = None
        for frame, position in zip(observations, positions):
            if (
                position < position_minimum - POSITION_EPSILON_RAD
                or position > position_maximum + POSITION_EPSILON_RAD
            ):
                violation = {
                    "backend_id": backend_id,
                    "contract": "urdf_joint_position_limit",
                    "joint": JOINT_NAME,
                    "step": frame["step"],
                    "sim_time_ticks": frame["sim_time_ticks"],
                    "unit": "rad",
                    "observed": position,
                    "minimum": position_minimum,
                    "maximum": position_maximum,
                }
                first_violations.append(violation)
                break
        checks = [
            check(
                f"{backend_id}.joint5_isolated_tracking_rmse_rad_v1",
                isolated_rmse,
                ISOLATED_RMSE_LIMIT_RAD,
            ),
            check(
                f"{backend_id}.joint5_coupled_tracking_rmse_rad_v1",
                coupled_rmse,
                COUPLED_RMSE_LIMIT_RAD,
            ),
            {
                "id": f"{backend_id}.joint5_urdf_position_range_rad_v1",
                "unit": "rad",
                "observed_minimum": min(positions),
                "observed_maximum": max(positions),
                "minimum": position_minimum,
                "maximum": position_maximum,
                "status": "passed" if violation is None else "failed",
            },
        ]
        backend_reports.append(
            {
                "backend_id": backend_id,
                "backend_version": trace["backend_version"],
                "joint": JOINT_NAME,
                "isolated_tracking_rmse_rad": isolated_rmse,
                "coupled_tracking_rmse_rad": coupled_rmse,
                "coupling_amplification_ratio": coupled_rmse
                / max(isolated_rmse, 1e-15),
                "phase_metrics": phase_metrics(actions, observations, joint_index),
                "arx_model": {
                    "kind": "siso_arx_2_2_with_intercept",
                    "equation": "q[k+1]=c0+c1*q[k]+c2*q[k-1]+c3*u[k]+c4*u[k-1]",
                    "coefficients": coefficients,
                    "discrete_poles": discrete_poles(coefficients, dt_s),
                    "training": {
                        "start_step": TRAIN_START_STEP,
                        "end_step": TRAIN_END_STEP,
                        **prediction_metrics(coefficients, train_rows, train_outputs),
                    },
                    "coupled_validation": {
                        "start_step": VALIDATION_START_STEP,
                        "end_step": VALIDATION_END_STEP,
                        **prediction_metrics(
                            coefficients, validation_rows, validation_outputs
                        ),
                    },
                },
                "checks": checks,
                "first_position_limit_violation": violation,
            }
        )

    rapier_report, gazebo_report = backend_reports
    reproduced = (
        rapier_report["checks"][0]["status"] == "passed"
        and gazebo_report["checks"][0]["status"] == "passed"
        and rapier_report["checks"][1]["status"] == "failed"
        and gazebo_report["checks"][1]["status"] == "passed"
        and rapier_report["first_position_limit_violation"] is not None
        and gazebo_report["first_position_limit_violation"] is None
    )
    corrected = (
        all(
            check_result["status"] == "passed"
            for backend_report in backend_reports
            for check_result in backend_report["checks"]
        )
        and not first_violations
    )
    first_violations.sort(key=lambda value: (value["step"], value["backend_id"]))
    report = {
        "kind": "rne_openarm_joint5_identification_report",
        "schema_version": 1,
        "status": (
            "coupled_response_passed"
            if corrected
            else (
                "expected_coupling_failure_reproduced"
                if reproduced
                else "unexpected_result"
            )
        ),
        "task_id": actions_artifact["task_id"],
        "controller_id": controller["controller_id"],
        "experiment_contract": {
            "clock": "simulation_time",
            "sample_period_s": dt_s,
            "input": "joint_position_target_rad",
            "output": "joint_position_rad",
            "training_window": [TRAIN_START_STEP, TRAIN_END_STEP],
            "coupled_validation_window": [
                VALIDATION_START_STEP,
                VALIDATION_END_STEP,
            ],
            "isolated_rmse_limit_rad": ISOLATED_RMSE_LIMIT_RAD,
            "coupled_rmse_limit_rad": COUPLED_RMSE_LIMIT_RAD,
            "position_limit_epsilon_rad": POSITION_EPSILON_RAD,
        },
        "inputs": [
            {"role": "task_spec", "sha256": sha256(task_path)},
            {"role": "experiment_controller", "sha256": sha256(controller_path)},
            {"role": "action_trace", "sha256": sha256(actions_path)},
            {"role": "rapier_trace", "sha256": sha256(rapier_path)},
            {"role": "gazebo_trace", "sha256": sha256(gazebo_path)},
            {"role": "robot_model", "sha256": sha256(model_path)},
            {"role": "rapier_actuation_config", "sha256": sha256(actuation_path)},
            {"role": "gazebo_runtime_manifest", "sha256": sha256(gazebo_runtime_path)},
            {"role": "gazebo_adapter_config", "sha256": sha256(gazebo_config_path)},
        ],
        "backends": backend_reports,
        "first_contract_divergence": first_violations[0] if first_violations else None,
        "diagnosis": (
            "Joint 5 meets the isolated SISO tracking contract on both backends. "
            "Only RNE/Rapier loses tracking and crosses the URDF hard position limit "
            "when the other arm joints move, localizing the failure to coupled "
            "articulation dynamics/constraint enforcement rather than the portable "
            "joint-5 reference trajectory."
        ),
    }
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "joint5-identification-report.json"
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    write_html(output / "joint5-identification-report.html", report)
    print(
        f"OpenArm joint5 identification: status={report['status']} "
        f"first_divergence_step={report['first_contract_divergence']['step'] if report['first_contract_divergence'] else 'none'}"
    )
    return 0 if reproduced or corrected else 1


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":")).replace("</", "<\\/")
    document = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenArm joint-5 identification</title><style>
body{{margin:0;background:#09121f;color:#eef5ff;font:14px system-ui,sans-serif}}main{{max-width:1180px;margin:auto;padding:28px}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(310px,1fr));gap:14px}}.card{{background:#132238;border:1px solid #2a4667;border-radius:10px;padding:14px}}table{{width:100%;border-collapse:collapse}}th,td{{border:1px solid #2a4667;padding:7px;text-align:right}}th:first-child,td:first-child{{text-align:left}}.pass{{color:#6ee7aa}}.fail{{color:#ff8585}}code{{word-break:break-all}}</style></head><body><main><h1>OpenArm joint-5 plant identification</h1><p>Status: <b>{report["status"]}</b></p><p>{report["diagnosis"]}</p><div id="summary" class="grid"></div><h2>Phase metrics</h2><div id="phases"></div><h2>First hard-contract divergence</h2><div id="violation" class="card"></div><script>const r={payload};const f=x=>Number(x).toFixed(6);document.querySelector('#summary').innerHTML=r.backends.map(b=>`<div class=card><h3>${{b.backend_id}}</h3><p>isolated RMSE: ${{f(b.isolated_tracking_rmse_rad)}} rad</p><p>coupled RMSE: ${{f(b.coupled_tracking_rmse_rad)}} rad</p><p>amplification: ${{f(b.coupling_amplification_ratio)}}×</p><p>ARX train / validation residual: ${{f(b.arx_model.training.one_step_prediction_rmse_rad)}} / ${{f(b.arx_model.coupled_validation.one_step_prediction_rmse_rad)}} rad</p></div>`).join('');document.querySelector('#phases').innerHTML=r.backends.map(b=>`<h3>${{b.backend_id}}</h3><table><tr><th>phase</th><th>samples</th><th>RMSE rad</th><th>max error rad</th></tr>${{b.phase_metrics.map(p=>`<tr><td>${{p.phase}}</td><td>${{p.sample_count}}</td><td>${{f(p.tracking_rmse_rad)}}</td><td>${{f(p.maximum_absolute_tracking_error_rad)}}</td></tr>`).join('')}}</table>`).join('');const v=r.first_contract_divergence;document.querySelector('#violation').innerHTML=v?`<b class=fail>${{v.backend_id}} · ${{v.joint}}</b><p>step ${{v.step}} (${{v.sim_time_ticks}} ns): ${{f(v.observed)}} rad outside ${{f(v.minimum)}} … ${{f(v.maximum)}} rad</p>`:'none';</script></main></body></html>"""
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm joint5 identification failed: {error}", file=sys.stderr)
        raise SystemExit(2)

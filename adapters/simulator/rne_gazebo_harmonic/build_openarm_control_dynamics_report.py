#!/usr/bin/env python3
"""Derives control-engineering metrics from retained OpenArm backend traces."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
import xml.etree.ElementTree as ET
from typing import Any


TRACKING_RMSE_LIMIT_RAD = 0.10
TERMINAL_BIAS_LIMIT_RAD = 0.01
TERMINAL_WINDOW_SAMPLES = 30
VELOCITY_LIMIT_EPSILON_RAD_S = 1e-6
POSITION_LIMIT_EPSILON_RAD = 1e-6


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=root)
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


def joint_limits(urdf: Path, joint_order: list[str]) -> list[dict[str, float]]:
    root = ET.parse(urdf).getroot()
    limits: dict[str, dict[str, float]] = {}
    for joint in root.findall("joint"):
        limit = joint.find("limit")
        if limit is not None and all(
            limit.get(field) is not None for field in ("lower", "upper", "velocity")
        ):
            limits[joint.attrib["name"]] = {
                "position_min": float(limit.attrib["lower"]),
                "position_max": float(limit.attrib["upper"]),
                "velocity_max": float(limit.attrib["velocity"]),
                "effort_max": float(limit.attrib["effort"]),
            }
    missing = [joint for joint in joint_order if joint not in limits]
    if missing:
        raise ValueError(f"URDF has incomplete position/velocity limits for {missing}")
    return [limits[joint] for joint in joint_order]


def metric_check(
    metric_id: str, unit: str, observed: float, maximum: float
) -> dict[str, Any]:
    status = "passed" if math.isfinite(observed) and observed <= maximum else "failed"
    return {
        "id": metric_id,
        "unit": unit,
        "observed": observed,
        "maximum": maximum,
        "status": status,
    }


def range_check(
    metric_id: str,
    unit: str,
    observed_minimum: float,
    observed_maximum: float,
    minimum: float,
    maximum: float,
) -> dict[str, Any]:
    status = (
        "passed"
        if all(math.isfinite(value) for value in (observed_minimum, observed_maximum))
        and observed_minimum >= minimum - POSITION_LIMIT_EPSILON_RAD
        and observed_maximum <= maximum + POSITION_LIMIT_EPSILON_RAD
        else "failed"
    )
    return {
        "id": metric_id,
        "unit": unit,
        "observed_minimum": observed_minimum,
        "observed_maximum": observed_maximum,
        "minimum": minimum,
        "maximum": maximum,
        "status": status,
    }


def main() -> int:
    args = parse_args()
    trace_root = args.trace_root.resolve()
    output = args.output.resolve()
    repo = args.repo_root.resolve()
    actions_path = trace_root / "controller-actions.json"
    rapier_path = trace_root / "rapier-success-trace.json"
    gazebo_path = trace_root / "gazebo-success-trace.json"
    actions_artifact = load(actions_path)
    actions = actions_artifact["actions"]
    controller_path = (
        repo
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json"
    )
    task_path = (
        repo
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json"
    )
    urdf_path = repo / "assets/robots/openarm_description/openarm_v2_right.rne.urdf"
    rapier_robot_asset_path = repo / "assets/robots/openarm_v2_right.rne.robot.toml"
    rapier_actuation_path = (
        repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json"
    )
    gazebo_runtime_path = repo / "adapters/simulator/rne_gazebo_harmonic/runtime.json"
    gazebo_adapter_config_path = (
        repo / "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json"
    )
    controller = load(controller_path)
    task = load(task_path)
    rapier_actuation = load(rapier_actuation_path)
    gazebo_adapter_config = load(gazebo_adapter_config_path)
    dt_s = actions_artifact["fixed_delta_ticks"] / 1_000_000_000.0
    joint_order = actions_artifact["action_joint_order"]
    limits = joint_limits(urdf_path, joint_order)
    if (
        actions_artifact["task_sha256"] != sha256(task_path)
        or actions_artifact["controller_sha256"] != sha256(controller_path)
        or actions_artifact["action_joint_order"] != controller["action_joint_order"]
    ):
        raise ValueError("action trace identity differs from TaskSpec/controller")
    if (
        rapier_actuation.get("backend_id") != "rne_rapier"
        or rapier_actuation.get("motor_model") != "force_based_v1"
        or rapier_actuation.get("fixed_delta_ticks")
        != actions_artifact["fixed_delta_ticks"]
        or [joint["joint_name"] for joint in rapier_actuation["joints"]] != joint_order
        or gazebo_adapter_config.get("joint_order") != joint_order
    ):
        raise ValueError("actuation configuration identity/order drifted")
    for joint, configured, limit in zip(
        joint_order, rapier_actuation["joints"], limits
    ):
        if configured["max_effort_nm"] > limit["effort_max"] + 1e-9:
            raise ValueError(f"{joint} configured effort exceeds the URDF limit")

    backend_reports: list[dict[str, Any]] = []
    violations: list[dict[str, Any]] = []
    traces = [("rne_rapier", rapier_path), ("gazebo_sim", gazebo_path)]
    loaded_traces: dict[str, dict[str, Any]] = {}
    for expected_backend, trace_path in traces:
        trace = load(trace_path)
        loaded_traces[expected_backend] = trace
        if (
            trace["backend_id"] != expected_backend
            or trace["task_sha256"] != sha256(task_path)
            or trace["controller_sha256"] != sha256(controller_path)
            or trace["action_trace_sha256"] != sha256(actions_path)
            or len(trace["observations"]) != len(actions)
        ):
            raise ValueError(f"{expected_backend} trace identity or length drifted")
        if expected_backend == "rne_rapier" and trace.get(
            "actuation_config_sha256"
        ) != sha256(rapier_actuation_path):
            raise ValueError("Rapier trace differs from its actuation configuration")
        if expected_backend == "rne_rapier" and trace.get(
            "robot_asset_config_sha256"
        ) != sha256(rapier_robot_asset_path):
            raise ValueError("Rapier trace differs from its robot asset configuration")
        if expected_backend == "gazebo_sim" and (
            trace.get("runtime_manifest_sha256") != sha256(gazebo_runtime_path)
            or trace.get("adapter_config_sha256") != sha256(gazebo_adapter_config_path)
        ):
            raise ValueError("Gazebo trace differs from its runtime/configuration")
        per_joint: list[dict[str, Any]] = []
        checks: list[dict[str, Any]] = []
        for index, (joint, limit) in enumerate(zip(joint_order, limits)):
            errors = [
                frame["joint_position_rad"][index]
                - action["joint_position_target_rad"][index]
                for frame, action in zip(trace["observations"], actions)
            ]
            velocities = [
                frame["joint_velocity_rad_s"][index] for frame in trace["observations"]
            ]
            positions = [
                frame["joint_position_rad"][index] for frame in trace["observations"]
            ]
            rmse = math.sqrt(sum(error * error for error in errors) / len(errors))
            iae = sum(abs(error) for error in errors) * dt_s
            ise = sum(error * error for error in errors) * dt_s
            terminal_bias = (
                sum(errors[-TERMINAL_WINDOW_SAMPLES:]) / TERMINAL_WINDOW_SAMPLES
            )
            peak_velocity = max(abs(value) for value in velocities)
            joint_checks = [
                metric_check(
                    f"{joint}.tracking_rmse_rad_v1",
                    "rad",
                    rmse,
                    TRACKING_RMSE_LIMIT_RAD,
                ),
                metric_check(
                    f"{joint}.terminal_bias_rad_v1",
                    "rad",
                    abs(terminal_bias),
                    TERMINAL_BIAS_LIMIT_RAD,
                ),
                metric_check(
                    f"{joint}.peak_velocity_rad_s_v1",
                    "rad/s",
                    peak_velocity,
                    limit["velocity_max"] + VELOCITY_LIMIT_EPSILON_RAD_S,
                ),
                range_check(
                    f"{joint}.position_range_rad_v1",
                    "rad",
                    min(positions),
                    max(positions),
                    limit["position_min"],
                    limit["position_max"],
                ),
            ]
            checks.extend(joint_checks)
            for frame, velocity in zip(trace["observations"], velocities):
                if abs(velocity) > limit["velocity_max"] + VELOCITY_LIMIT_EPSILON_RAD_S:
                    violations.append(
                        {
                            "backend_id": expected_backend,
                            "contract": "urdf_joint_velocity_limit",
                            "joint": joint,
                            "step": frame["step"],
                            "sim_time_ticks": frame["sim_time_ticks"],
                            "unit": "rad/s",
                            "observed": abs(velocity),
                            "maximum": limit["velocity_max"],
                        }
                    )
                    break
            for frame, position in zip(trace["observations"], positions):
                if position < limit["position_min"] - POSITION_LIMIT_EPSILON_RAD:
                    violations.append(
                        {
                            "backend_id": expected_backend,
                            "contract": "urdf_joint_position_limit",
                            "joint": joint,
                            "step": frame["step"],
                            "sim_time_ticks": frame["sim_time_ticks"],
                            "unit": "rad",
                            "observed": position,
                            "minimum": limit["position_min"],
                            "maximum": limit["position_max"],
                        }
                    )
                    break
                if position > limit["position_max"] + POSITION_LIMIT_EPSILON_RAD:
                    violations.append(
                        {
                            "backend_id": expected_backend,
                            "contract": "urdf_joint_position_limit",
                            "joint": joint,
                            "step": frame["step"],
                            "sim_time_ticks": frame["sim_time_ticks"],
                            "unit": "rad",
                            "observed": position,
                            "minimum": limit["position_min"],
                            "maximum": limit["position_max"],
                        }
                    )
                    break
            per_joint.append(
                {
                    "joint": joint,
                    "tracking_rmse_rad": rmse,
                    "integral_absolute_error_rad_s": iae,
                    "integral_squared_error_rad2_s": ise,
                    "maximum_absolute_tracking_error_rad": max(
                        abs(error) for error in errors
                    ),
                    "terminal_window_mean_error_rad": terminal_bias,
                    "terminal_window_samples": TERMINAL_WINDOW_SAMPLES,
                    "peak_absolute_velocity_rad_s": peak_velocity,
                    "minimum_position_rad": min(positions),
                    "maximum_position_rad": max(positions),
                    "urdf_position_minimum_rad": limit["position_min"],
                    "urdf_position_maximum_rad": limit["position_max"],
                    "urdf_velocity_limit_rad_s": limit["velocity_max"],
                    "urdf_effort_limit_nm": limit["effort_max"],
                    "checks": joint_checks,
                }
            )
        phases: list[dict[str, Any]] = []
        keyframes = controller["keyframes"]
        for lower, upper in zip(keyframes, keyframes[1:]):
            start = lower["step"] + 1
            end = upper["step"]
            phase_errors = []
            for index in range(start - 1, end):
                frame = trace["observations"][index]
                action = actions[index]
                phase_errors.extend(
                    actual - target
                    for actual, target in zip(
                        frame["joint_position_rad"], action["joint_position_target_rad"]
                    )
                )
            phases.append(
                {
                    "phase": upper["phase"],
                    "start_step": start,
                    "end_step": end,
                    "tracking_rmse_rad": math.sqrt(
                        sum(error * error for error in phase_errors) / len(phase_errors)
                    ),
                    "maximum_absolute_tracking_error_rad": max(
                        abs(error) for error in phase_errors
                    ),
                }
            )
        backend_reports.append(
            {
                "backend_id": expected_backend,
                "backend_version": trace["backend_version"],
                "sample_count": len(actions),
                "sample_period_s": dt_s,
                "joint_metrics": per_joint,
                "phase_metrics": phases,
                "checks": checks,
                "status": "passed"
                if all(item["status"] == "passed" for item in checks)
                else "needs_tuning",
            }
        )

    cross_joint: list[dict[str, Any]] = []
    rapier = loaded_traces["rne_rapier"]["observations"]
    gazebo = loaded_traces["gazebo_sim"]["observations"]
    for index, joint in enumerate(joint_order):
        position_deltas = [
            left["joint_position_rad"][index] - right["joint_position_rad"][index]
            for left, right in zip(rapier, gazebo)
        ]
        velocity_deltas = [
            left["joint_velocity_rad_s"][index] - right["joint_velocity_rad_s"][index]
            for left, right in zip(rapier, gazebo)
        ]
        maximum_position = max(abs(value) for value in position_deltas)
        maximum_step = max(
            range(len(position_deltas)), key=lambda step: abs(position_deltas[step])
        )
        cross_joint.append(
            {
                "joint": joint,
                "position_delta_rmse_rad": math.sqrt(
                    sum(value * value for value in position_deltas)
                    / len(position_deltas)
                ),
                "maximum_position_delta_rad": maximum_position,
                "maximum_position_delta_step": maximum_step + 1,
                "maximum_velocity_delta_rad_s": max(
                    abs(value) for value in velocity_deltas
                ),
                "final_position_delta_rad": abs(position_deltas[-1]),
            }
        )

    violations.sort(key=lambda item: (item["step"], item["backend_id"], item["joint"]))
    readiness_passed = all(report["status"] == "passed" for report in backend_reports)
    report = {
        "kind": "rne_control_dynamics_report",
        "schema_version": 1,
        "status": "passed" if readiness_passed else "needs_tuning",
        "task_id": task["task_id"],
        "controller_id": controller["controller_id"],
        "measurement_contract": {
            "clock": "simulation_time",
            "sample_phase": "post_update",
            "sample_period_s": dt_s,
            "position_unit": "rad",
            "velocity_unit": "rad/s",
            "action_unit": "rad",
            "action_semantics": "joint_position_target",
            "terminal_window_samples": TERMINAL_WINDOW_SAMPLES,
        },
        "actuation_contracts": [
            {
                "backend_id": "rne_rapier",
                "motor_model": rapier_actuation["motor_model"],
                "solver_iterations": rapier_actuation["solver_iterations"],
                "configuration_sha256": sha256(rapier_actuation_path),
                "robot_asset_config_sha256": sha256(rapier_robot_asset_path),
                "joint_count": len(rapier_actuation["joints"]),
            },
            {
                "backend_id": "gazebo_sim",
                "position_gain_s_inv": gazebo_adapter_config["position_gain_s_inv"],
                "maximum_velocity_rad_s": gazebo_adapter_config[
                    "maximum_velocity_rad_s"
                ],
                "runtime_manifest_sha256": sha256(gazebo_runtime_path),
                "configuration_sha256": sha256(gazebo_adapter_config_path),
                "joint_count": len(gazebo_adapter_config["joint_order"]),
            },
        ],
        "tolerance_registry": {
            "schema_version": 1,
            "entries": [
                {
                    "id": "openarm_tracking_rmse_rad_v1",
                    "unit": "rad",
                    "maximum": TRACKING_RMSE_LIMIT_RAD,
                    "rationale": "Flags trajectory tracking too weak for backend comparison even when the final pose eventually converges.",
                },
                {
                    "id": "openarm_terminal_bias_rad_v1",
                    "unit": "rad",
                    "maximum": TERMINAL_BIAS_LIMIT_RAD,
                    "rationale": "Bounds the mean signed error over the final 0.5 s controller window.",
                },
                {
                    "id": "openarm_urdf_position_limit_rad_v1",
                    "unit": "rad",
                    "absolute_epsilon": POSITION_LIMIT_EPSILON_RAD,
                    "rationale": "The measured joint position must remain inside the model-declared hard range throughout the trajectory.",
                },
                {
                    "id": "openarm_urdf_velocity_limit_rad_s_v1",
                    "unit": "rad/s",
                    "absolute_epsilon": VELOCITY_LIMIT_EPSILON_RAD_S,
                    "rationale": "The measured joint velocity must remain within the model-declared actuator limit.",
                },
            ],
        },
        "inputs": [
            {"role": "task_spec", "sha256": sha256(task_path)},
            {"role": "controller", "sha256": sha256(controller_path)},
            {"role": "action_trace", "sha256": sha256(actions_path)},
            {"role": "rapier_trace", "sha256": sha256(rapier_path)},
            {"role": "gazebo_trace", "sha256": sha256(gazebo_path)},
            {"role": "robot_model", "sha256": sha256(urdf_path)},
            {
                "role": "rapier_robot_asset_config",
                "sha256": sha256(rapier_robot_asset_path),
            },
            {
                "role": "rapier_actuation_config",
                "sha256": sha256(rapier_actuation_path),
            },
            {
                "role": "gazebo_runtime_manifest",
                "sha256": sha256(gazebo_runtime_path),
            },
            {
                "role": "gazebo_adapter_config",
                "sha256": sha256(gazebo_adapter_config_path),
            },
        ],
        "backends": backend_reports,
        "cross_backend_diagnostics": cross_joint,
        "first_contract_violation": violations[0] if violations else None,
        "contract_violations": violations,
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "control-dynamics-report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    write_html(output / "control-dynamics-report.html", report)
    first = report["first_contract_violation"]
    print(
        f"OpenArm control dynamics: status={report['status']} "
        + (
            f"first_violation={first['backend_id']}:{first['joint']}@{first['step']}"
            if first
            else "first_violation=none"
        )
    )
    return 0


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":")).replace("</", "<\\/")
    document = f"""<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenArm control dynamics</title><style>
body{{margin:0;background:#08111d;color:#edf5ff;font:14px system-ui,sans-serif}}main{{max-width:1250px;margin:auto;padding:28px}}h1{{font-size:28px}}.warn{{color:#ffbd66}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:14px}}.card{{background:#122033;border:1px solid #29415f;border-radius:10px;padding:14px}}table{{width:100%;border-collapse:collapse;margin-top:10px}}th,td{{border:1px solid #29415f;padding:7px;text-align:right}}th:first-child,td:first-child{{text-align:left}}.failed{{color:#ff7e7e}}.passed{{color:#6ee7aa}}code{{word-break:break-all;font-size:11px}}</style></head><body><main><h1>OpenArm control dynamics</h1><p>Status: <strong class="warn">{report["status"].upper()}</strong></p><div id="summary" class="grid"></div><h2>Per-joint metrics</h2><div id="metrics"></div><h2>First actuator-contract violation</h2><div id="violation" class="card"></div><h2>Content-addressed inputs</h2><div id="inputs" class="card"></div><script>const r={payload};const fmt=x=>Number(x).toFixed(6);document.querySelector('#summary').innerHTML=r.backends.map(b=>`<div class=card><b>${{b.backend_id}} ${{b.backend_version}}</b><p>samples: ${{b.sample_count}} @ ${{b.sample_period_s}} s</p><p class=${{b.status==='passed'?'passed':'failed'}}>${{b.status}}</p></div>`).join('');document.querySelector('#metrics').innerHTML=r.backends.map(b=>`<h3>${{b.backend_id}}</h3><table><tr><th>joint</th><th>RMSE rad</th><th>IAE rad·s</th><th>ISE rad²·s</th><th>terminal bias rad</th><th>position / URDF range rad</th><th>peak / limit rad/s</th></tr>${{b.joint_metrics.map(j=>`<tr><td>${{j.joint}}</td><td>${{fmt(j.tracking_rmse_rad)}}</td><td>${{fmt(j.integral_absolute_error_rad_s)}}</td><td>${{fmt(j.integral_squared_error_rad2_s)}}</td><td>${{fmt(j.terminal_window_mean_error_rad)}}</td><td>${{fmt(j.minimum_position_rad)}}…${{fmt(j.maximum_position_rad)}} / ${{fmt(j.urdf_position_minimum_rad)}}…${{fmt(j.urdf_position_maximum_rad)}}</td><td>${{fmt(j.peak_absolute_velocity_rad_s)}} / ${{fmt(j.urdf_velocity_limit_rad_s)}}</td></tr>`).join('')}}</table>`).join('');const v=r.first_contract_violation;document.querySelector('#violation').innerHTML=v?`<b class=failed>${{v.backend_id}} · ${{v.joint}}</b><p>step ${{v.step}} (${{v.sim_time_ticks}} ns): ${{fmt(v.observed)}} ${{v.unit}}; allowed ${{v.minimum===undefined?'|x| ≤ '+fmt(v.maximum):fmt(v.minimum)+' … '+fmt(v.maximum)}} ${{v.unit}}</p>`:'none';document.querySelector('#inputs').innerHTML=r.inputs.map(x=>`<p><b>${{x.role}}</b> <code>${{x.sha256}}</code></p>`).join('');</script></main></body></html>"""
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm control dynamics failed: {error}", file=sys.stderr)
        raise SystemExit(2)

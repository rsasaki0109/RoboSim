#!/usr/bin/env python3
"""Build a browser-readable OpenArm plant viscous-damping envelope report."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any
import xml.etree.ElementTree as ET


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_openarm_authority_report import (  # noqa: E402
    TRACE_FILES,
    check_maximum,
    load,
    metrics,
    sha256,
    validate_gazebo_sidecars,
    write_json,
)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-root", required=True, type=Path)
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--experiment",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_joint_loss_experiments.json",
    )
    parser.add_argument(
        "--controller",
        type=Path,
        default=root
        / "docs/evidence/openarm-controller-lab/evidence/openarm-plant-state-feedback.controller.json",
    )
    return parser.parse_args()


def urdf_joint_dynamics(path: Path, joint_name: str) -> tuple[float, float, bool]:
    root = ET.parse(path).getroot()
    joints = [joint for joint in root.findall("joint") if joint.get("name") == joint_name]
    if len(joints) != 1:
        raise ValueError(f"{path} must contain exactly one {joint_name}")
    dynamics = joints[0].find("dynamics")
    if dynamics is None:
        return 0.0, 0.0, False
    damping = float(dynamics.get("damping", "0"))
    friction = float(dynamics.get("friction", "0"))
    if not math.isfinite(damping) or not math.isfinite(friction):
        raise ValueError(f"{path} contains non-finite joint dynamics")
    return damping, friction, True


def actuator_damping(path: Path, joint_name: str) -> float:
    config = load(path)
    matches = [item for item in config["joints"] if item["joint_name"] == joint_name]
    if len(matches) != 1:
        raise ValueError(f"{path} must contain exactly one actuator for {joint_name}")
    value = matches[0]["damping_nm_s_per_rad"]
    if not isinstance(value, (int, float)) or not math.isfinite(value) or value < 0.0:
        raise ValueError(f"{path} contains invalid actuator damping")
    return float(value)


def outcome_status(checks: list[dict[str, Any]], supported: bool) -> str:
    capacity = next(
        item
        for item in checks
        if item["requirement_id"]
        == "joint_loss.maximum_supported_viscous_damping_nm_s_per_rad"
    )
    non_capacity_passed = all(
        item["status"] == "passed"
        for item in checks
        if item is not capacity
    )
    if supported and non_capacity_passed and capacity["status"] == "passed":
        return "passed"
    if not supported and capacity["status"] == "failed" and non_capacity_passed:
        return "outside_declared_envelope"
    return "failed"


def build_report(
    fixture_root: Path,
    trace_root: Path,
    experiment_path: Path,
    controller_path: Path,
) -> dict[str, Any]:
    suite_path = fixture_root / "joint-loss-suite.json"
    suite = load(suite_path)
    experiment = load(experiment_path)
    controller = load(controller_path)
    if (
        suite.get("kind") != "rne_openarm_joint_loss_suite"
        or suite.get("experiment_id") != experiment.get("experiment_id")
        or suite.get("requirements") != experiment.get("requirements")
        or suite["inputs"]["experiment_manifest_sha256"] != sha256(experiment_path)
    ):
        raise ValueError("joint-loss suite and experiment identity differ")
    requirements = {item["id"]: item for item in suite["requirements"]}
    joint = suite["controlled_joint"]
    joint_index = controller["action_joint_order"].index(joint)
    controller_sha = sha256(controller_path)
    supported_maximum = suite["declared_supported_viscous_damping_nm_s_per_rad"]
    outcomes = []
    missing_traces = []
    action_hashes = set()
    for case in suite["cases"]:
        case_dir = fixture_root / case["case_id"]
        fixture = load(case_dir / "joint-loss-fixture.json")
        if fixture != case:
            raise ValueError(f"{case['case_id']} fixture differs from suite")
        for filename, field in (
            ("openarm_v2_right.payload.urdf", "model_urdf_sha256"),
            ("openarm_payload.world.sdf", "world_sha256"),
            ("openarm_payload.rne.robot.toml", "robot_asset_config_sha256"),
            ("openarm_payload.rne.scene.toml", "scene_config_sha256"),
            ("openarm_right.rne_actuation.json", "actuation_config_sha256"),
            ("openarm_right.adapter.json", "adapter_config_sha256"),
            ("runtime.json", "runtime_manifest_sha256"),
        ):
            if sha256(case_dir / filename) != case[field]:
                raise ValueError(f"{case['case_id']} {field} differs")
        realized_damping, realized_friction, dynamics_present = urdf_joint_dynamics(
            case_dir / "openarm_v2_right.payload.urdf", joint
        )
        if (
            realized_damping != case["realized_viscous_damping_nm_s_per_rad"]
            or realized_friction != case["realized_coulomb_friction_nm"]
            or dynamics_present != case["dynamics_element_present"]
        ):
            raise ValueError(f"{case['case_id']} URDF dynamics differ from fixture")
        actuator_damping_value = actuator_damping(
            case_dir / "openarm_right.rne_actuation.json", joint
        )
        realization_delta = max(
            abs(realized_damping - case["plant_viscous_damping_nm_s_per_rad"]),
            abs(realized_friction - case["plant_coulomb_friction_nm"]),
        )
        supported = case["plant_viscous_damping_nm_s_per_rad"] <= supported_maximum
        for backend in suite["backend_order"]:
            directory, filename = TRACE_FILES[backend]
            trace_path = trace_root / directory / case["case_id"] / filename
            if not trace_path.exists():
                missing_traces.append(
                    {"backend_id": backend, "case_id": case["case_id"]}
                )
                continue
            trace = load(trace_path)
            if (
                trace.get("backend_id") != backend
                or trace.get("controller_sha256") != controller_sha
                or trace.get("replay_match") is not True
            ):
                raise ValueError(f"{backend} {case['case_id']} trace identity differs")
            if backend == "gazebo_sim":
                if (
                    trace.get("robot_model_sha256") != case["model_urdf_sha256"]
                    or trace.get("adapter_config_sha256")
                    != case["adapter_config_sha256"]
                    or trace.get("runtime_manifest_sha256")
                    != case["runtime_manifest_sha256"]
                ):
                    raise ValueError(f"Gazebo {case['case_id']} provenance differs")
            elif (
                trace.get("model_urdf_sha256") != case["model_urdf_sha256"]
                or trace.get("actuation_config_sha256")
                != case["actuation_config_sha256"]
                or trace.get("robot_asset_config_sha256")
                != case["robot_asset_config_sha256"]
                or trace.get("scene_config_sha256") != case["scene_config_sha256"]
            ):
                raise ValueError(f"{backend} {case['case_id']} provenance differs")
            action_hashes.add(trace["action_trace_sha256"])
            measured = metrics(trace, joint_index)
            if backend == "gazebo_sim":
                measured.update(validate_gazebo_sidecars(trace_path, trace, joint_index))
            checks = [
                check_maximum(
                    "joint_loss.maximum_model_parameter_realization_delta",
                    realization_delta,
                    requirements[
                        "joint_loss.maximum_model_parameter_realization_delta"
                    ],
                ),
                check_maximum(
                    "joint_loss.maximum_controlled_joint_rmse_rad",
                    measured["tracking_rmse_rad"],
                    requirements["joint_loss.maximum_controlled_joint_rmse_rad"],
                ),
                check_maximum(
                    "joint_loss.maximum_controlled_joint_final_error_rad",
                    measured["final_absolute_error_rad"],
                    requirements[
                        "joint_loss.maximum_controlled_joint_final_error_rad"
                    ],
                ),
                check_maximum(
                    "joint_loss.maximum_supported_viscous_damping_nm_s_per_rad",
                    case["plant_viscous_damping_nm_s_per_rad"],
                    requirements[
                        "joint_loss.maximum_supported_viscous_damping_nm_s_per_rad"
                    ],
                ),
                {
                    "requirement_id": "joint_loss.requires_exact_replay",
                    "unit": "bool",
                    "observed": trace["replay_match"],
                    "required": True,
                    "status": "passed" if trace["replay_match"] else "failed",
                },
            ]
            outcomes.append(
                {
                    "backend_id": backend,
                    "case_id": case["case_id"],
                    "plant_viscous_damping_nm_s_per_rad": case[
                        "plant_viscous_damping_nm_s_per_rad"
                    ],
                    "plant_coulomb_friction_nm": case["plant_coulomb_friction_nm"],
                    "actuator_servo_damping_nm_s_per_rad": actuator_damping_value,
                    "metrics": measured,
                    "checks": checks,
                    "status": outcome_status(checks, supported),
                    "trace_sha256": sha256(trace_path),
                }
            )
    expected = len(suite["cases"]) * len(suite["backend_order"])
    accepted = {"passed", "outside_declared_envelope"}
    status = (
        "incomplete"
        if missing_traces
        else (
            "passed"
            if len(outcomes) == expected
            and all(item["status"] in accepted for item in outcomes)
            and len(action_hashes) == 1
            else "needs_tuning"
        )
    )
    performance_ids = {
        "joint_loss.maximum_controlled_joint_rmse_rad",
        "joint_loss.maximum_controlled_joint_final_error_rad",
    }
    first_failures = []
    for backend in suite["backend_order"]:
        failure = next(
            (
                outcome
                for outcome in outcomes
                if outcome["backend_id"] == backend
                and any(
                    check["requirement_id"] in performance_ids
                    and check["status"] == "failed"
                    for check in outcome["checks"]
                )
            ),
            None,
        )
        first_failures.append(
            {
                "backend_id": backend,
                "first_failing_case_id": failure["case_id"] if failure else None,
                "plant_viscous_damping_nm_s_per_rad": (
                    failure["plant_viscous_damping_nm_s_per_rad"] if failure else None
                ),
                "first_failed_requirement": (
                    next(
                        check["requirement_id"]
                        for check in failure["checks"]
                        if check["requirement_id"] in performance_ids
                        and check["status"] == "failed"
                    )
                    if failure
                    else None
                ),
            }
        )
    return {
        "kind": "rne_openarm_joint_loss_report",
        "schema_version": 1,
        "status": status,
        "experiment_id": suite["experiment_id"],
        "controlled_joint": joint,
        "parameter_semantics": suite["parameter_semantics"],
        "inputs": {
            "suite_sha256": sha256(suite_path),
            "experiment_manifest_sha256": sha256(experiment_path),
            "controller_sha256": controller_sha,
            "action_trace_sha256": (
                next(iter(action_hashes)) if len(action_hashes) == 1 else None
            ),
        },
        "outcomes": outcomes,
        "first_performance_failures": first_failures,
        "missing_traces": missing_traces,
    }


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    html = """<!doctype html><html lang="en"><meta charset="utf-8"><title>OpenArm joint-loss envelope</title><style>
body{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}main{max-width:1220px;margin:auto;padding:28px}table{width:100%;border-collapse:collapse}th,td{border:1px solid #294563;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}.card{background:#112238;border:1px solid #294563;border-radius:10px;padding:14px;margin:12px 0}.passed,.outside_declared_envelope{color:#64e6a1}.failed,.needs_tuning{color:#ff9b85}.incomplete{color:#ffd477}</style><main><h1>OpenArm joint-5 plant viscous-damping envelope</h1><p>Status: <b id="status"></b></p><div id="summary" class="card"></div><table><thead><tr><th>backend / case</th><th>plant N·m·s/rad</th><th>servo N·m·s/rad</th><th>RMSE rad</th><th>final rad</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><h2>First performance failures</h2><div id="failures" class="card"></div><script>const r=__REPORT__,f=x=>x===undefined||x===null?'—':Number(x).toFixed(6),s=document.querySelector('#status');s.textContent=r.status;s.className=r.status;document.querySelector('#summary').textContent=`controlled joint: ${r.controlled_joint} · outcomes: ${r.outcomes.length} · missing: ${r.missing_traces.length} · ${r.parameter_semantics}`;document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${x.backend_id} / ${x.case_id}</td><td>${f(x.plant_viscous_damping_nm_s_per_rad)}</td><td>${f(x.actuator_servo_damping_nm_s_per_rad)}</td><td>${f(x.metrics.tracking_rmse_rad)}</td><td>${f(x.metrics.final_absolute_error_rad)}</td><td class=${x.status}>${x.status}</td></tr>`).join('');document.querySelector('#failures').innerHTML=r.first_performance_failures.map(x=>`<p><b>${x.backend_id}</b>: ${x.first_failing_case_id||'none'} ${x.first_failed_requirement||''}</p>`).join('');</script></main></html>""".replace(
        "__REPORT__", payload
    )
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(html, encoding="utf-8")


def main() -> int:
    args = parse_args()
    report = build_report(
        args.fixture_root.resolve(),
        args.trace_root.resolve(),
        args.experiment.resolve(),
        args.controller.resolve(),
    )
    write_json(args.output / "openarm-joint-loss-report.json", report)
    write_html(args.output / "openarm-joint-loss-report.html", report)
    print(
        f"OpenArm joint-loss report: status={report['status']} "
        f"outcomes={len(report['outcomes'])} missing={len(report['missing_traces'])}"
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

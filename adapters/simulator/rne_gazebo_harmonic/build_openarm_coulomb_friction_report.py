#!/usr/bin/env python3
"""Build the browser-readable OpenArm regularized-Coulomb envelope report."""

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


PERFORMANCE_IDS = {
    "coulomb.maximum_controlled_joint_rmse_rad",
    "coulomb.maximum_controlled_joint_final_error_rad",
}
STRUCTURAL_IDS = {
    "coulomb.maximum_model_parameter_realization_delta",
    "coulomb.maximum_transition_velocity_realization_delta",
    "coulomb.requires_exact_replay",
}
CAPACITY_ID = "coulomb.maximum_supported_friction_nm"


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
        / "adapters/simulator/rne_gazebo_harmonic/openarm_coulomb_friction_experiments.json",
    )
    parser.add_argument("--controller", required=True, type=Path)
    return parser.parse_args()


def urdf_dynamics(path: Path, joint_name: str) -> tuple[float, float]:
    root = ET.parse(path).getroot()
    matches = [joint for joint in root.findall("joint") if joint.get("name") == joint_name]
    if len(matches) != 1:
        raise ValueError(f"{path} must contain exactly one {joint_name}")
    dynamics = matches[0].find("dynamics")
    if dynamics is None:
        return 0.0, 0.0
    values = (float(dynamics.get("damping", "0")), float(dynamics.get("friction", "0")))
    if not all(math.isfinite(value) for value in values):
        raise ValueError(f"{path} contains non-finite joint dynamics")
    return values


def outcome_status(checks: list[dict[str, Any]], supported: bool) -> str:
    required = PERFORMANCE_IDS | STRUCTURAL_IDS | {CAPACITY_ID}
    if len(checks) != len(required) or {item.get("requirement_id") for item in checks} != required:
        return "failed"
    capacity = next(item for item in checks if item["requirement_id"] == CAPACITY_ID)
    performance = [item for item in checks if item["requirement_id"] in PERFORMANCE_IDS]
    structural = [item for item in checks if item["requirement_id"] in STRUCTURAL_IDS]
    non_capacity_passed = all(item["status"] == "passed" for item in performance + structural)
    if supported and capacity["status"] == "passed" and non_capacity_passed:
        return "passed"
    if not supported and capacity["status"] == "failed" and non_capacity_passed:
        return "outside_declared_envelope"
    if (
        not supported
        and capacity["status"] == "failed"
        and any(item["status"] == "failed" for item in performance)
        and all(item["status"] == "passed" for item in structural)
    ):
        return "expected_boundary_failure"
    return "failed"


def native_realization(trace: dict[str, Any], joint_index: int) -> tuple[float, float, float]:
    values = trace.get("joint_passive_dynamics")
    if not isinstance(values, list) or len(values) <= joint_index:
        raise ValueError("native trace does not retain joint passive dynamics")
    dynamics = values[joint_index]
    if not isinstance(dynamics, dict) or dynamics.get("kind") != "revolute":
        raise ValueError("native trace controlled-joint passive dynamics differ")
    return (
        float(dynamics["viscous_damping_nm_s_per_rad"]),
        float(dynamics["coulomb_friction_nm"]),
        float(dynamics["coulomb_transition_velocity_rad_s"]),
    )


def gazebo_realization(
    case_dir: Path, joint_name: str, joint_index: int
) -> tuple[float, float, float]:
    runtime_damping, native_friction = urdf_dynamics(
        case_dir / "openarm_v2_right.payload.urdf", joint_name
    )
    config = load(case_dir / "openarm_right.adapter.json")
    friction = float(config["plant_coulomb_friction_nm"][joint_index])
    transition = float(config["plant_coulomb_transition_velocity_rad_s"][joint_index])
    if native_friction != 0.0:
        raise ValueError("Gazebo runtime model retained backend-native friction")
    return runtime_damping, friction, transition


def build_report(
    fixture_root: Path,
    trace_root: Path,
    experiment_path: Path,
    controller_path: Path,
) -> dict[str, Any]:
    suite_path = fixture_root / "coulomb-friction-suite.json"
    suite = load(suite_path)
    experiment = load(experiment_path)
    controller = load(controller_path)
    if (
        suite.get("kind") != "rne_openarm_coulomb_friction_suite"
        or suite.get("schema_version") != 1
        or suite.get("experiment_id") != experiment.get("experiment_id")
        or suite.get("requirements") != experiment.get("requirements")
        or suite["inputs"]["experiment_manifest_sha256"] != sha256(experiment_path)
    ):
        raise ValueError("Coulomb suite and experiment identity differ")
    requirements = {item["id"]: item for item in suite["requirements"]}
    joint = suite["controlled_joint"]
    joint_index = controller["action_joint_order"].index(joint)
    controller_sha = sha256(controller_path)
    supported_maximum = suite["declared_supported_coulomb_friction_nm"]
    outcomes: list[dict[str, Any]] = []
    missing: list[dict[str, str]] = []
    action_hashes: set[str] = set()
    for case in suite["cases"]:
        case_dir = fixture_root / case["case_id"]
        if load(case_dir / "coulomb-friction-fixture.json") != case:
            raise ValueError(f"{case['case_id']} fixture differs from suite")
        for filename, field in (
            ("openarm_v2_right.coulomb.urdf", "portable_model_urdf_sha256"),
            ("openarm_v2_right.payload.urdf", "gazebo_runtime_model_urdf_sha256"),
            ("openarm_payload.world.sdf", "world_sha256"),
            ("openarm_payload.rne.robot.toml", "robot_asset_config_sha256"),
            ("openarm_payload.rne.scene.toml", "scene_config_sha256"),
            ("openarm_right.rne_actuation.json", "actuation_config_sha256"),
            ("openarm_right.adapter.json", "adapter_config_sha256"),
            ("runtime.json", "runtime_manifest_sha256"),
        ):
            if sha256(case_dir / filename) != case[field]:
                raise ValueError(f"{case['case_id']} {field} differs")
        portable = urdf_dynamics(case_dir / "openarm_v2_right.coulomb.urdf", joint)
        if list(portable) != case["portable_model_realized_dynamics"]:
            raise ValueError(f"{case['case_id']} portable URDF dynamics differ")
        supported = case["plant_coulomb_friction_nm"] <= supported_maximum
        for backend in suite["backend_order"]:
            directory, filename = TRACE_FILES[backend]
            trace_path = trace_root / directory / case["case_id"] / filename
            if not trace_path.exists():
                missing.append({"backend_id": backend, "case_id": case["case_id"]})
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
                    trace.get("robot_model_sha256")
                    != case["gazebo_runtime_model_urdf_sha256"]
                    or trace.get("adapter_config_sha256") != case["adapter_config_sha256"]
                    or trace.get("runtime_manifest_sha256") != case["runtime_manifest_sha256"]
                    or trace.get("world_sha256") != case["world_sha256"]
                ):
                    raise ValueError(f"Gazebo {case['case_id']} provenance differs")
                realized = gazebo_realization(case_dir, joint, joint_index)
            else:
                if (
                    trace.get("model_urdf_sha256") != case["portable_model_urdf_sha256"]
                    or trace.get("actuation_config_sha256") != case["actuation_config_sha256"]
                    or trace.get("robot_asset_config_sha256")
                    != case["robot_asset_config_sha256"]
                    or trace.get("scene_config_sha256") != case["scene_config_sha256"]
                ):
                    raise ValueError(f"{backend} {case['case_id']} provenance differs")
                realized = native_realization(trace, joint_index)
            expected = (
                case["plant_viscous_damping_nm_s_per_rad"],
                case["plant_coulomb_friction_nm"],
                case["plant_coulomb_transition_velocity_rad_s"],
            )
            model_delta = abs(realized[1] - expected[1])
            transition_delta = abs(realized[2] - expected[2])
            measured = metrics(trace, joint_index)
            if backend == "gazebo_sim":
                measured.update(validate_gazebo_sidecars(trace_path, trace, joint_index))
                diagnostics = load(trace_path.parent / "gazebo-actuation-diagnostics-a.json")
                passive_min = min(
                    step["joint_passive_coulomb_effort_min_nm"][joint_index]
                    for step in diagnostics["steps"]
                )
                passive_max = max(
                    step["joint_passive_coulomb_effort_max_nm"][joint_index]
                    for step in diagnostics["steps"]
                )
                measured["passive_coulomb_effort_min_nm"] = passive_min
                measured["passive_coulomb_effort_max_nm"] = passive_max
                if max(abs(passive_min), abs(passive_max)) > expected[1] + 1.0e-12:
                    raise ValueError("Gazebo passive effort exceeded declared magnitude")
            checks = [
                check_maximum(
                    "coulomb.maximum_model_parameter_realization_delta",
                    model_delta,
                    requirements["coulomb.maximum_model_parameter_realization_delta"],
                ),
                check_maximum(
                    "coulomb.maximum_transition_velocity_realization_delta",
                    transition_delta,
                    requirements["coulomb.maximum_transition_velocity_realization_delta"],
                ),
                check_maximum(
                    "coulomb.maximum_controlled_joint_rmse_rad",
                    measured["tracking_rmse_rad"],
                    requirements["coulomb.maximum_controlled_joint_rmse_rad"],
                ),
                check_maximum(
                    "coulomb.maximum_controlled_joint_final_error_rad",
                    measured["final_absolute_error_rad"],
                    requirements["coulomb.maximum_controlled_joint_final_error_rad"],
                ),
                check_maximum(CAPACITY_ID, expected[1], requirements[CAPACITY_ID]),
                {
                    "requirement_id": "coulomb.requires_exact_replay",
                    "unit": "bool",
                    "observed": trace["replay_match"],
                    "required": True,
                    "status": "passed" if trace["replay_match"] else "failed",
                },
            ]
            action_hashes.add(trace["action_trace_sha256"])
            outcomes.append(
                {
                    "backend_id": backend,
                    "case_id": case["case_id"],
                    "plant_viscous_damping_nm_s_per_rad": expected[0],
                    "plant_coulomb_friction_nm": expected[1],
                    "plant_coulomb_transition_velocity_rad_s": expected[2],
                    "realized_passive_dynamics": {
                        "viscous_damping_nm_s_per_rad": realized[0],
                        "coulomb_friction_nm": realized[1],
                        "coulomb_transition_velocity_rad_s": realized[2],
                    },
                    "metrics": measured,
                    "checks": checks,
                    "status": outcome_status(checks, supported),
                    "trace_sha256": sha256(trace_path),
                }
            )
    expected_count = len(suite["cases"]) * len(suite["backend_order"])
    accepted = {"passed", "outside_declared_envelope", "expected_boundary_failure"}
    status = (
        "incomplete"
        if missing
        else (
            "passed"
            if len(outcomes) == expected_count
            and all(item["status"] in accepted for item in outcomes)
            and len(action_hashes) == 1
            else "needs_tuning"
        )
    )
    first_failures = []
    for backend in suite["backend_order"]:
        failure = next(
            (
                outcome
                for outcome in outcomes
                if outcome["backend_id"] == backend
                and any(
                    check["requirement_id"] in PERFORMANCE_IDS
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
                "plant_coulomb_friction_nm": (
                    failure["plant_coulomb_friction_nm"] if failure else None
                ),
                "first_failed_requirement": (
                    next(
                        check["requirement_id"]
                        for check in failure["checks"]
                        if check["requirement_id"] in PERFORMANCE_IDS
                        and check["status"] == "failed"
                    )
                    if failure
                    else None
                ),
            }
        )
    return {
        "kind": "rne_openarm_coulomb_friction_report",
        "schema_version": 1,
        "status": status,
        "experiment_id": suite["experiment_id"],
        "controlled_joint": joint,
        "parameter_semantics": suite["parameter_semantics"],
        "gazebo_derivation": suite["gazebo_derivation"],
        "inputs": {
            "suite_sha256": sha256(suite_path),
            "experiment_manifest_sha256": sha256(experiment_path),
            "controller_sha256": controller_sha,
            "action_trace_sha256": next(iter(action_hashes)) if len(action_hashes) == 1 else None,
        },
        "outcomes": outcomes,
        "first_performance_failures": first_failures,
        "missing_traces": missing,
    }


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    html = """<!doctype html><html lang="en"><meta charset="utf-8"><title>OpenArm regularized-Coulomb envelope</title><style>
body{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}main{max-width:1250px;margin:auto;padding:28px}table{width:100%;border-collapse:collapse}th,td{border:1px solid #294563;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}.card{background:#112238;border:1px solid #294563;border-radius:10px;padding:14px;margin:12px 0}.passed,.outside_declared_envelope,.expected_boundary_failure{color:#64e6a1}.failed,.needs_tuning{color:#ff9b85}.incomplete{color:#ffd477}</style><main><h1>OpenArm joint-5 regularized-Coulomb envelope</h1><p>Status: <b id="status"></b></p><div id="summary" class="card"></div><table><thead><tr><th>backend / case</th><th>friction N·m</th><th>transition rad/s</th><th>RMSE rad</th><th>final rad</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><h2>First performance failures</h2><div id="failures" class="card"></div><script>const r=__REPORT__,f=x=>x===undefined||x===null?'—':Number(x).toFixed(6),s=document.querySelector('#status');s.textContent=r.status;s.className=r.status;document.querySelector('#summary').textContent=`controlled joint: ${r.controlled_joint} · outcomes: ${r.outcomes.length} · missing: ${r.missing_traces.length} · ${r.parameter_semantics}`;document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${x.backend_id} / ${x.case_id}</td><td>${f(x.plant_coulomb_friction_nm)}</td><td>${f(x.plant_coulomb_transition_velocity_rad_s)}</td><td>${f(x.metrics.tracking_rmse_rad)}</td><td>${f(x.metrics.final_absolute_error_rad)}</td><td class=${x.status}>${x.status}</td></tr>`).join('');document.querySelector('#failures').innerHTML=r.first_performance_failures.map(x=>`<p><b>${x.backend_id}</b>: ${x.first_failing_case_id||'none'} ${x.first_failed_requirement||''}</p>`).join('');</script></main></html>""".replace("__REPORT__", payload)
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
    write_json(args.output / "openarm-coulomb-friction-report.json", report)
    write_html(args.output / "openarm-coulomb-friction-report.html", report)
    print(
        f"OpenArm Coulomb-friction report: status={report['status']} "
        f"outcomes={len(report['outcomes'])} missing={len(report['missing_traces'])}"
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

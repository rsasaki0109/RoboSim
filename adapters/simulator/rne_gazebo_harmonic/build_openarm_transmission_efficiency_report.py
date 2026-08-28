#!/usr/bin/env python3
"""Build the browser-readable OpenArm transmission-efficiency envelope report."""

from __future__ import annotations

import argparse
import html
import json
import math
from pathlib import Path
import sys
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_openarm_authority_report import (  # noqa: E402
    TRACE_FILES,
    actuator_evidence,
    check_maximum,
    load,
    metrics,
    sha256,
    write_json,
)


PERFORMANCE_IDS = {
    "transmission.maximum_controlled_joint_rmse_rad",
    "transmission.maximum_controlled_joint_final_error_rad",
}
STRUCTURAL_IDS = {
    "transmission.maximum_efficiency_realization_delta",
    "transmission.maximum_fixed_model_hash_delta",
    "transmission.maximum_motor_effort_excess_nm",
    "transmission.maximum_joint_effort_excess_nm",
    "transmission.requires_exact_replay",
}
CAPACITY_ID = "transmission.minimum_supported_efficiency"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-root", required=True, type=Path)
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--controller", required=True, type=Path)
    parser.add_argument(
        "--experiment",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_transmission_efficiency_experiments.json",
    )
    return parser.parse_args()


def required_check(identifier: str, observed: bool) -> dict[str, Any]:
    return {
        "requirement_id": identifier,
        "unit": "bool",
        "observed": observed,
        "required": True,
        "status": "passed" if observed else "failed",
    }


def capacity_check(efficiency: float, minimum: float) -> dict[str, Any]:
    return {
        "requirement_id": CAPACITY_ID,
        "unit": "1",
        "observed": efficiency,
        "minimum": minimum,
        "status": "passed" if efficiency >= minimum else "failed",
    }


def outcome_status(checks: list[dict[str, Any]], supported: bool) -> str:
    required = PERFORMANCE_IDS | STRUCTURAL_IDS | {CAPACITY_ID}
    if {item.get("requirement_id") for item in checks} != required:
        return "failed"
    capacity = next(item for item in checks if item["requirement_id"] == CAPACITY_ID)
    performance = [item for item in checks if item["requirement_id"] in PERFORMANCE_IDS]
    structural = [item for item in checks if item["requirement_id"] in STRUCTURAL_IDS]
    if supported and all(item["status"] == "passed" for item in checks):
        return "passed"
    if not supported and capacity["status"] == "failed" and all(
        item["status"] == "passed" for item in structural
    ):
        if any(item["status"] == "failed" for item in performance):
            return "expected_boundary_failure"
        return "outside_declared_envelope"
    return "failed"


def actuation_joint(case_dir: Path, joint_name: str) -> dict[str, Any]:
    joints = load(case_dir / "openarm_right.rne_actuation.json").get("joints")
    matches = (
        [item for item in joints if item.get("joint_name") == joint_name]
        if isinstance(joints, list)
        else []
    )
    if len(matches) != 1:
        raise ValueError("portable actuation omits the controlled joint")
    return matches[0]


def gazebo_effort_evidence(trace_path: Path, joint_index: int) -> dict[str, float]:
    trace = load(trace_path)
    first_path = trace_path.parent / "gazebo-actuation-diagnostics-a.json"
    replay_path = trace_path.parent / "gazebo-actuation-diagnostics-b.json"
    if (
        sha256(first_path) != trace.get("actuation_diagnostics_sha256")
        or sha256(replay_path) != trace.get("replay_actuation_diagnostics_sha256")
        or sha256(first_path) != sha256(replay_path)
    ):
        raise ValueError("Gazebo transmission diagnostic replay hash differs")
    diagnostics = load(first_path)
    replay = load(replay_path)
    steps = diagnostics.get("steps")
    observations = trace.get("observations")
    if (
        diagnostics != replay
        or not isinstance(steps, list)
        or not steps
        or not isinstance(observations, list)
        or len(steps) != len(observations)
        or any(
            frame.get("actuator_realization") != step
            for frame, step in zip(observations, steps)
        )
    ):
        raise ValueError("Gazebo transmission diagnostics have no steps")

    def peak(minimum_key: str, maximum_key: str) -> float:
        values: list[float] = []
        for step in steps:
            minimum = step.get(minimum_key)
            maximum = step.get(maximum_key)
            if (
                not isinstance(minimum, list)
                or not isinstance(maximum, list)
                or len(minimum) <= joint_index
                or len(maximum) <= joint_index
            ):
                raise ValueError("Gazebo diagnostics omit transmission effort")
            values.extend((abs(float(minimum[joint_index])), abs(float(maximum[joint_index]))))
        if not all(math.isfinite(value) for value in values):
            raise ValueError("Gazebo transmission effort is non-finite")
        return max(values)

    return {
        "motor_effort_command_peak_abs_nm": peak(
            "joint_applied_command_min", "joint_applied_command_max"
        ),
        "joint_transmitted_effort_peak_abs_nm": peak(
            "joint_transmitted_effort_min_nm", "joint_transmitted_effort_max_nm"
        ),
    }


def trace_path(trace_root: Path, case_id: str, backend: str) -> Path:
    directory, filename = TRACE_FILES[backend]
    return trace_root / case_id / directory / filename


def validate_fixture_hashes(case_dir: Path, case: dict[str, Any]) -> None:
    files = (
        ("openarm_v2_right.coulomb.urdf", "portable_model_urdf_sha256"),
        ("openarm_v2_right.payload.urdf", "gazebo_runtime_model_urdf_sha256"),
        ("openarm_payload.world.sdf", "world_sha256"),
        ("openarm_payload.rne.robot.toml", "robot_asset_config_sha256"),
        ("openarm_payload.rne.scene.toml", "scene_config_sha256"),
        ("openarm_right.rne_actuation.json", "actuation_config_sha256"),
        ("openarm_right.adapter.json", "adapter_config_sha256"),
        ("runtime.json", "runtime_manifest_sha256"),
    )
    for filename, field in files:
        if sha256(case_dir / filename) != case[field]:
            raise ValueError(f"{case['case_id']} {field} differs")


def build_report(
    fixture_root: Path,
    trace_root: Path,
    experiment_path: Path,
    controller_path: Path,
) -> dict[str, Any]:
    suite_path = fixture_root / "transmission-efficiency-suite.json"
    suite = load(suite_path)
    experiment = load(experiment_path)
    controller = load(controller_path)
    if (
        suite.get("kind") != "rne_openarm_transmission_efficiency_suite"
        or suite.get("schema_version") != 1
        or suite.get("experiment_id") != experiment.get("experiment_id")
        or suite.get("requirements") != experiment.get("requirements")
        or suite["inputs"]["experiment_manifest_sha256"] != sha256(experiment_path)
    ):
        raise ValueError("transmission suite and experiment identity differ")
    requirements = {item["id"]: item for item in suite["requirements"]}
    joint_name = suite["controlled_joint"]
    joint_index = controller["action_joint_order"].index(joint_name)
    controller_sha = sha256(controller_path)
    minimum = float(suite["declared_minimum_supported_efficiency"])
    fixed_hashes = suite["cases"][0]["fixed_artifact_sha256"]
    outcomes: list[dict[str, Any]] = []
    action_hashes: set[str] = set()

    for case in suite["cases"]:
        case_dir = fixture_root / case["case_id"]
        if load(case_dir / "transmission-efficiency-fixture.json") != case:
            raise ValueError(f"{case['case_id']} fixture differs from suite")
        validate_fixture_hashes(case_dir, case)
        efficiency = float(case["transmission_efficiency"])
        joint_config = actuation_joint(case_dir, joint_name)
        realized_portable = float(joint_config["transmission_efficiency"])
        adapter = load(case_dir / "openarm_right.adapter.json")
        adapter_index = adapter["joint_order"].index(joint_name)
        realized_gazebo = float(adapter["transmission_efficiency_by_joint"][adapter_index])
        realization_delta = max(
            abs(realized_portable - efficiency), abs(realized_gazebo - efficiency)
        )
        model_hashes_fixed = case["fixed_artifact_sha256"] == fixed_hashes
        motor_limit_nm = float(joint_config["max_effort_nm"])
        supported = efficiency >= minimum

        for backend in suite["backend_order"]:
            path = trace_path(trace_root, case["case_id"], backend)
            trace = load(path)
            if (
                trace.get("backend_id") != backend
                or trace.get("controller_sha256") != controller_sha
            ):
                raise ValueError(f"{backend} {case['case_id']} trace identity differs")
            action_hashes.add(trace["action_trace_sha256"])
            if backend == "gazebo_sim":
                provenance_ok = (
                    trace.get("robot_model_sha256")
                    == case["gazebo_runtime_model_urdf_sha256"]
                    and trace.get("adapter_config_sha256") == case["adapter_config_sha256"]
                    and trace.get("runtime_manifest_sha256")
                    == case["runtime_manifest_sha256"]
                    and trace.get("world_sha256") == case["world_sha256"]
                )
                effort = gazebo_effort_evidence(path, joint_index)
                motor_peak = effort["motor_effort_command_peak_abs_nm"]
                joint_peak = effort["joint_transmitted_effort_peak_abs_nm"]
                measured_peak = None
            else:
                provenance_ok = (
                    trace.get("model_urdf_sha256") == case["portable_model_urdf_sha256"]
                    and trace.get("actuation_config_sha256") == case["actuation_config_sha256"]
                    and trace.get("robot_asset_config_sha256")
                    == case["robot_asset_config_sha256"]
                    and trace.get("scene_config_sha256") == case["scene_config_sha256"]
                )
                evidence = actuator_evidence(trace, joint_index)
                motor_peak = float(evidence["limited_effort_command_peak_abs_nm"])
                joint_peak = motor_peak * efficiency
                measured_peak = evidence.get("measured_effort_peak_abs_nm")
            trace_metrics = metrics(trace, joint_index)
            motor_excess = max(0.0, motor_peak - motor_limit_nm)
            joint_limit_nm = motor_limit_nm * efficiency
            joint_excess = max(0.0, joint_peak - joint_limit_nm)
            checks = [
                check_maximum(
                    "transmission.maximum_efficiency_realization_delta",
                    realization_delta,
                    requirements["transmission.maximum_efficiency_realization_delta"],
                ),
                required_check(
                    "transmission.maximum_fixed_model_hash_delta",
                    model_hashes_fixed and provenance_ok,
                ),
                check_maximum(
                    "transmission.maximum_motor_effort_excess_nm",
                    motor_excess,
                    requirements["transmission.maximum_motor_effort_excess_nm"],
                ),
                check_maximum(
                    "transmission.maximum_joint_effort_excess_nm",
                    joint_excess,
                    requirements["transmission.maximum_joint_effort_excess_nm"],
                ),
                check_maximum(
                    "transmission.maximum_controlled_joint_rmse_rad",
                    trace_metrics["tracking_rmse_rad"],
                    requirements["transmission.maximum_controlled_joint_rmse_rad"],
                ),
                check_maximum(
                    "transmission.maximum_controlled_joint_final_error_rad",
                    trace_metrics["final_absolute_error_rad"],
                    requirements[
                        "transmission.maximum_controlled_joint_final_error_rad"
                    ],
                ),
                capacity_check(efficiency, minimum),
                required_check(
                    "transmission.requires_exact_replay", trace.get("replay_match") is True
                ),
            ]
            outcomes.append(
                {
                    "case_id": case["case_id"],
                    "backend_id": backend,
                    "transmission_efficiency": efficiency,
                    "declared_supported": supported,
                    "status": outcome_status(checks, supported),
                    "tracking": trace_metrics,
                    "metrics": trace_metrics,
                    "effort_boundary": {
                        "motor_effort_limit_nm": motor_limit_nm,
                        "joint_effort_limit_nm": joint_limit_nm,
                        "motor_effort_command_peak_abs_nm": motor_peak,
                        "joint_transmitted_effort_peak_abs_nm": joint_peak,
                        "backend_measured_joint_effort_peak_abs_nm": measured_peak,
                        "measured_vs_transmitted_peak_delta_nm": (
                            abs(float(measured_peak) - joint_peak)
                            if measured_peak is not None
                            else None
                        ),
                    },
                    "checks": checks,
                    "trace": {
                        "file": str(path),
                        "sha256": sha256(path),
                        "action_trace_sha256": trace["action_trace_sha256"],
                    },
                    "trace_sha256": sha256(path),
                }
            )

    below = [case for case in suite["cases"] if case["transmission_efficiency"] < minimum]
    boundary_case_id = below[0]["case_id"] if below else None
    boundary_outcomes = [item for item in outcomes if item["case_id"] == boundary_case_id]
    supported_outcomes = [item for item in outcomes if item["declared_supported"]]
    failed = [item for item in outcomes if item["status"] == "failed"]
    boundary_demonstrated = bool(boundary_outcomes) and any(
        item["status"] == "expected_boundary_failure" for item in boundary_outcomes
    )
    report_status = (
        "passed"
        if not failed
        and supported_outcomes
        and all(item["status"] == "passed" for item in supported_outcomes)
        and boundary_demonstrated
        and len(action_hashes) == 1
        else "failed"
    )
    return {
        "kind": "rne_openarm_transmission_efficiency_report",
        "schema_version": 1,
        "status": report_status,
        "experiment_id": suite["experiment_id"],
        "controlled_joint": joint_name,
        "parameter_semantics": suite["parameter_semantics"],
        "declared_minimum_supported_efficiency": minimum,
        "first_outside_case_id": boundary_case_id,
        "boundary_demonstrated": boundary_demonstrated,
        "same_action_trace_across_all_runs": len(action_hashes) == 1,
        "action_trace_sha256": next(iter(action_hashes)) if len(action_hashes) == 1 else None,
        "inputs": {
            "suite_sha256": sha256(suite_path),
            "experiment_manifest_sha256": sha256(experiment_path),
            "controller_sha256": controller_sha,
            "fixed_artifact_sha256": fixed_hashes,
        },
        "outcomes": outcomes,
    }


def write_html(path: Path, report: dict[str, Any]) -> None:
    rows = []
    for item in report["outcomes"]:
        tracking = item["tracking"]
        effort = item["effort_boundary"]
        rows.append(
            "<tr>"
            f"<td>{html.escape(item['case_id'])}</td>"
            f"<td>{html.escape(item['backend_id'])}</td>"
            f"<td>{item['transmission_efficiency']:.0%}</td>"
            f"<td>{tracking['tracking_rmse_rad']:.6f}</td>"
            f"<td>{tracking['final_absolute_error_rad']:.6f}</td>"
            f"<td>{effort['motor_effort_command_peak_abs_nm']:.4f}</td>"
            f"<td>{effort['joint_transmitted_effort_peak_abs_nm']:.4f}</td>"
            f"<td>{html.escape(item['status'])}</td>"
            "</tr>"
        )
    payload = html.escape(json.dumps(report, indent=2, allow_nan=False))
    document = f"""<!doctype html>
<html lang="en"><meta charset="utf-8"><title>OpenArm transmission envelope</title>
<style>body{{font:14px system-ui;margin:2rem;max-width:1200px}}table{{border-collapse:collapse;width:100%}}th,td{{border:1px solid #bbb;padding:.45rem;text-align:right}}th:first-child,td:first-child,th:nth-child(2),td:nth-child(2){{text-align:left}}.passed{{color:#087830}}pre{{white-space:pre-wrap;background:#f5f5f5;padding:1rem}}</style>
<h1>OpenArm motor-to-joint transmission envelope</h1>
<p class="{report['status']}"><strong>Report status: {report['status']}</strong></p>
<p>Declared support: efficiency &ge; {report['declared_minimum_supported_efficiency']:.0%}. Motor-side PD is limited before efficiency is applied; passive Coulomb loss is applied on the joint side.</p>
<table><thead><tr><th>Case</th><th>Backend</th><th>Efficiency</th><th>RMSE rad</th><th>Final rad</th><th>Motor peak N·m</th><th>Joint peak N·m</th><th>Outcome</th></tr></thead><tbody>{''.join(rows)}</tbody></table>
<h2>Machine-readable evidence</h2><pre>{payload}</pre></html>"""
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(document, encoding="utf-8")


def main() -> int:
    args = parse_args()
    try:
        report = build_report(
            args.fixture_root,
            args.trace_root,
            args.experiment,
            args.controller,
        )
        args.output.mkdir(parents=True, exist_ok=True)
        write_json(args.output / "transmission-efficiency-report.json", report)
        write_html(args.output / "transmission-efficiency-report.html", report)
        print(f"OpenArm transmission report: {report['status']}")
        return 0 if report["status"] == "passed" else 1
    except Exception as error:  # noqa: BLE001
        print(f"OpenArm transmission report failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

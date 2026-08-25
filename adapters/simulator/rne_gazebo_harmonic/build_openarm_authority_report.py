#!/usr/bin/env python3
"""Builds a browser-readable cross-backend actuator-authority report."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


TRACE_FILES = {
    "rne_rapier": ("rapier", "rapier-success-trace.json"),
    "mujoco_native": ("mujoco", "mujoco-success-trace.json"),
    "gazebo_sim": ("gazebo", "gazebo-success-trace.json"),
}


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
        / "adapters/simulator/rne_gazebo_harmonic/openarm_authority_experiments.json",
    )
    parser.add_argument(
        "--controller",
        type=Path,
        default=root
        / "docs/evidence/openarm-controller-lab/evidence/openarm-plant-state-feedback.controller.json",
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


def metrics(trace: dict[str, Any], joint_index: int) -> dict[str, float]:
    observations = trace.get("observations")
    if not isinstance(observations, list) or not observations:
        raise ValueError("authority trace has no observations")
    errors = [
        frame["joint_reference_position_rad"][joint_index]
        - frame["joint_position_rad"][joint_index]
        for frame in observations
    ]
    return {
        "sample_count": len(errors),
        "tracking_rmse_rad": math.sqrt(
            sum(error * error for error in errors) / len(errors)
        ),
        "final_absolute_error_rad": abs(errors[-1]),
        "maximum_absolute_error_rad": max(abs(error) for error in errors),
        "integral_absolute_error_rad_s": sum(abs(error) for error in errors)
        * 0.016666667,
    }


def actuator_evidence(trace: dict[str, Any], joint_index: int) -> dict[str, Any]:
    """Classify effort evidence without confusing commands with measurements."""
    observations = trace.get("observations")
    if not isinstance(observations, list) or not observations:
        raise ValueError("actuator evidence trace has no observations")
    command_frames = [
        frame
        for frame in observations
        if isinstance(frame.get("limited_effort_command_nm"), list)
        and len(frame["limited_effort_command_nm"]) > joint_index
        and isinstance(frame.get("effort_saturated"), list)
        and len(frame["effort_saturated"]) > joint_index
    ]
    measurement_frames = [
        frame
        for frame in observations
        if isinstance(frame.get("effort_measurement_available"), list)
        and len(frame["effort_measurement_available"]) > joint_index
        and frame["effort_measurement_available"][joint_index] is True
    ]
    kind = (
        "measured_effort"
        if measurement_frames
        else "command_model_only" if len(command_frames) == len(observations) else "unavailable"
    )
    result: dict[str, Any] = {
        "kind": kind,
        "sample_count": len(observations),
        "command_sample_count": len(command_frames),
        "measured_effort_sample_count": len(measurement_frames),
        "measured_effort_fraction": len(measurement_frames) / len(observations),
    }
    if command_frames:
        result.update(
            {
                "command_saturation_sample_count": sum(
                    bool(frame["effort_saturated"][joint_index])
                    for frame in command_frames
                ),
                "command_saturation_fraction": sum(
                    bool(frame["effort_saturated"][joint_index])
                    for frame in command_frames
                )
                / len(command_frames),
                "limited_effort_command_peak_abs_nm": max(
                    abs(float(frame["limited_effort_command_nm"][joint_index]))
                    for frame in command_frames
                ),
            }
        )
    return result


def check_maximum(identifier: str, observed: float, requirement: dict[str, Any]) -> dict[str, Any]:
    return {
        "requirement_id": identifier,
        "unit": requirement["unit"],
        "observed": observed,
        "maximum": requirement["maximum"],
        "status": "passed" if observed <= requirement["maximum"] else "failed",
    }


def outcome_status(checks: list[dict[str, Any]], authority_supported: bool) -> str:
    authority_id = "authority.minimum_supported_scale"
    all_passed = all(
        item["status"] == "passed"
        for item in checks
    )
    authority = next(item for item in checks if item["requirement_id"] == authority_id)
    replay = next(
        item
        for item in checks
        if item["requirement_id"] == "authority.requires_exact_replay"
    )
    if authority_supported and all_passed:
        return "passed"
    if (
        not authority_supported
        and authority["status"] == "failed"
        and replay["status"] == "passed"
    ):
        return "failed_as_expected"
    return "failed"


def validate_gazebo_sidecars(trace_path: Path, trace: dict[str, Any], joint_index: int) -> dict[str, float]:
    first_path = trace_path.parent / "gazebo-actuation-diagnostics-a.json"
    replay_path = trace_path.parent / "gazebo-actuation-diagnostics-b.json"
    if (
        sha256(first_path) != trace.get("actuation_diagnostics_sha256")
        or sha256(replay_path) != trace.get("replay_actuation_diagnostics_sha256")
        or sha256(first_path) != sha256(replay_path)
    ):
        raise ValueError("Gazebo authority diagnostic replay hash differs")
    first = load(first_path)
    replay = load(replay_path)
    steps = first.get("steps")
    observations = trace["observations"]
    if (
        first != replay
        or not isinstance(steps, list)
        or len(steps) != len(observations)
        or any(
            frame.get("actuator_realization") != step
            for frame, step in zip(observations, steps)
        )
    ):
        raise ValueError("Gazebo authority diagnostics differ from embedded evidence")
    substeps = sum(step["substep_count"] for step in steps)
    return {
        "actuator_saturation_fraction": sum(
            step["joint_saturation_substep_count"][joint_index] for step in steps
        )
        / substeps,
        "actuator_raw_command_peak_abs": max(
            step["joint_raw_command_peak_abs"][joint_index] for step in steps
        ),
    }


def build_report(
    fixture_root: Path,
    trace_root: Path,
    experiment_path: Path,
    controller_path: Path,
) -> dict[str, Any]:
    suite_path = fixture_root / "authority-suite.json"
    suite = load(suite_path)
    experiment = load(experiment_path)
    controller = load(controller_path)
    if (
        suite.get("kind") != "rne_openarm_actuator_authority_suite"
        or suite.get("experiment_id") != experiment.get("experiment_id")
        or suite.get("requirements") != experiment.get("requirements")
        or suite["inputs"]["experiment_manifest_sha256"] != sha256(experiment_path)
    ):
        raise ValueError("authority suite and experiment identity differ")
    requirements = {item["id"]: item for item in suite["requirements"]}
    joint_index = controller["action_joint_order"].index(suite["controlled_joint"])
    controller_sha = sha256(controller_path)
    minimum_scale = suite["declared_minimum_supported_authority_scale"]
    outcomes = []
    missing_traces = []
    action_hashes = set()
    for case in suite["cases"]:
        case_dir = fixture_root / case["case_id"]
        fixture = load(case_dir / "authority-fixture.json")
        for filename, field in (
            ("openarm_right.rne_actuation.json", "actuation_config_sha256"),
            ("openarm_right.adapter.json", "adapter_config_sha256"),
            ("runtime.json", "runtime_manifest_sha256"),
            ("openarm_v2_right.payload.urdf", "model_urdf_sha256"),
            ("openarm_payload.rne.robot.toml", "robot_asset_config_sha256"),
            ("openarm_payload.rne.scene.toml", "scene_config_sha256"),
        ):
            if sha256(case_dir / filename) != fixture[field] or fixture[field] != case[field]:
                raise ValueError(f"{case['case_id']} {field} differs")
        for backend in suite["backend_order"]:
            directory, filename = TRACE_FILES[backend]
            trace_path = trace_root / directory / case["case_id"] / filename
            if not trace_path.exists():
                missing_traces.append({"backend_id": backend, "case_id": case["case_id"]})
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
                ):
                    raise ValueError("Gazebo authority config hash differs")
            elif (
                trace.get("model_urdf_sha256") != case["model_urdf_sha256"]
                or trace.get("actuation_config_sha256")
                != case["actuation_config_sha256"]
                or trace.get("robot_asset_config_sha256")
                != case["robot_asset_config_sha256"]
                or trace.get("scene_config_sha256") != case["scene_config_sha256"]
            ):
                raise ValueError(f"{backend} native authority provenance differs")
            action_hashes.add(trace["action_trace_sha256"])
            measured = metrics(trace, joint_index)
            if backend == "gazebo_sim":
                measured.update(validate_gazebo_sidecars(trace_path, trace, joint_index))
            checks = [
                check_maximum(
                    "authority.maximum_controlled_joint_rmse_rad",
                    measured["tracking_rmse_rad"],
                    requirements["authority.maximum_controlled_joint_rmse_rad"],
                ),
                check_maximum(
                    "authority.maximum_controlled_joint_final_error_rad",
                    measured["final_absolute_error_rad"],
                    requirements["authority.maximum_controlled_joint_final_error_rad"],
                ),
                {
                    "requirement_id": "authority.minimum_supported_scale",
                    "unit": "ratio",
                    "observed": case["authority_scale"],
                    "minimum": requirements["authority.minimum_supported_scale"]["minimum"],
                    "status": (
                        "passed" if case["authority_scale"] >= minimum_scale else "failed"
                    ),
                },
                {
                    "requirement_id": "authority.requires_exact_replay",
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
                    "authority_scale": case["authority_scale"],
                    "realized_max_effort_nm": case["realized_max_effort_nm"],
                    "metrics": measured,
                    "checks": checks,
                    "status": outcome_status(
                        checks, case["authority_scale"] >= minimum_scale
                    ),
                    "trace_sha256": sha256(trace_path),
                }
            )
    expected = len(suite["cases"]) * len(suite["backend_order"])
    status = (
        "incomplete"
        if missing_traces
        else (
            "passed"
            if len(outcomes) == expected
            and all(item["status"] in {"passed", "failed_as_expected"} for item in outcomes)
            else "needs_tuning"
        )
    )
    performance_ids = {
        "authority.maximum_controlled_joint_rmse_rad",
        "authority.maximum_controlled_joint_final_error_rad",
    }
    first_performance_failures = []
    for backend in suite["backend_order"]:
        failure = next(
            (
                outcome
                for outcome in outcomes
                if outcome["backend_id"] == backend
                and any(
                    item["requirement_id"] in performance_ids
                    and item["status"] == "failed"
                    for item in outcome["checks"]
                )
            ),
            None,
        )
        first_performance_failures.append(
            {
                "backend_id": backend,
                "first_failing_case_id": failure["case_id"] if failure else None,
                "authority_scale": failure["authority_scale"] if failure else None,
                "first_failed_requirement": (
                    next(
                        item["requirement_id"]
                        for item in failure["checks"]
                        if item["requirement_id"] in performance_ids
                        and item["status"] == "failed"
                    )
                    if failure
                    else None
                ),
            }
        )
    return {
        "kind": "rne_openarm_actuator_authority_report",
        "schema_version": 1,
        "status": status,
        "experiment_id": suite["experiment_id"],
        "controlled_joint": suite["controlled_joint"],
        "inputs": {
            "suite_sha256": sha256(suite_path),
            "experiment_manifest_sha256": sha256(experiment_path),
            "controller_sha256": controller_sha,
            "action_trace_sha256": next(iter(action_hashes)) if len(action_hashes) == 1 else None,
        },
        "outcomes": outcomes,
        "first_performance_failures": first_performance_failures,
        "missing_traces": missing_traces,
    }


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    html = """<!doctype html><html lang="en"><meta charset="utf-8"><title>OpenArm actuator authority</title><style>
body{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}main{max-width:1180px;margin:auto;padding:28px}table{width:100%;border-collapse:collapse}th,td{border:1px solid #294563;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}.card{background:#112238;border:1px solid #294563;border-radius:10px;padding:14px;margin:12px 0}.passed,.failed_as_expected{color:#64e6a1}.failed,.needs_tuning{color:#ff9b85}.incomplete{color:#ffd477}</style><main><h1>OpenArm actuator-authority envelope</h1><p>Status: <b id="status"></b></p><div id="summary" class="card"></div><table><thead><tr><th>backend / case</th><th>scale</th><th>effort N·m</th><th>RMSE rad</th><th>final rad</th><th>saturation</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><h2>First performance failures</h2><div id="failures" class="card"></div><script>const r=__REPORT__,f=x=>x===undefined?'—':Number(x).toFixed(6),s=document.querySelector('#status');s.textContent=r.status;s.className=r.status;document.querySelector('#summary').textContent=`controlled joint: ${r.controlled_joint} · outcomes: ${r.outcomes.length} · missing: ${r.missing_traces.length}`;document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${x.backend_id} / ${x.case_id}</td><td>${f(x.authority_scale)}</td><td>${f(x.realized_max_effort_nm)}</td><td>${f(x.metrics.tracking_rmse_rad)}</td><td>${f(x.metrics.final_absolute_error_rad)}</td><td>${f(x.metrics.actuator_saturation_fraction)}</td><td class=${x.status}>${x.status}</td></tr>`).join('');document.querySelector('#failures').innerHTML=r.first_performance_failures.map(x=>`<p><b>${x.backend_id}</b>: ${x.first_failing_case_id||'none'} ${x.first_failed_requirement||''}</p>`).join('');</script></main></html>""".replace(
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
    output = args.output.resolve()
    write_json(output / "openarm-authority-report.json", report)
    write_html(output / "openarm-authority-report.html", report)
    print(
        f"OpenArm authority report: status={report['status']} "
        f"outcomes={len(report['outcomes'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

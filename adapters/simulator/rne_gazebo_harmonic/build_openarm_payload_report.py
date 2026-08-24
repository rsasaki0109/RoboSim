#!/usr/bin/env python3
"""Build an independently recomputed OpenArm payload robustness report."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any
import xml.etree.ElementTree as ET


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_openarm_payload_suite import combined_inertial  # noqa: E402


TRACE_FILES = {
    "rne_rapier": ("rapier", "rapier-success-trace.json"),
    "mujoco_native": ("mujoco", "mujoco-success-trace.json"),
    "gazebo_sim": ("gazebo", "gazebo-success-trace.json"),
}


def load(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain one JSON object")
    return value


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def parse_vector(value: str) -> list[float]:
    result = [float(item) for item in value.split()]
    if len(result) != 3:
        raise ValueError("expected a three-vector")
    return result


def link_inertial(urdf_path: Path, link_name: str) -> tuple[float, list[float], dict[str, float]]:
    root = ET.parse(urdf_path).getroot()
    link = root.find(f"./link[@name='{link_name}']")
    if link is None:
        raise ValueError(f"{urdf_path} has no {link_name}")
    inertial = link.find("inertial")
    if inertial is None:
        raise ValueError(f"{link_name} has no inertial")
    origin = inertial.find("origin")
    mass = inertial.find("mass")
    inertia = inertial.find("inertia")
    if origin is None or mass is None or inertia is None:
        raise ValueError(f"{link_name} has an incomplete inertial")
    return (
        float(mass.attrib["value"]),
        parse_vector(origin.attrib["xyz"]),
        {
            name: float(inertia.attrib[name])
            for name in ("ixx", "ixy", "ixz", "iyy", "iyz", "izz")
        },
    )


def maximum_parameter_delta(
    actual: tuple[float, list[float], dict[str, float]],
    expected: tuple[float, list[float], dict[str, float]],
) -> float:
    return max(
        abs(actual[0] - expected[0]),
        *(abs(a - b) for a, b in zip(actual[1], expected[1])),
        *(abs(actual[2][name] - expected[2][name]) for name in expected[2]),
    )


def controlled_joint_metrics(
    trace: dict[str, Any], joint_index: int, fixed_delta_s: float
) -> dict[str, float]:
    frames = trace.get("observations")
    if not isinstance(frames, list) or not frames:
        raise ValueError("payload trace has no observations")
    errors = [
        frame["joint_reference_position_rad"][joint_index]
        - frame["joint_position_rad"][joint_index]
        for frame in frames
    ]
    return {
        "sample_count": len(errors),
        "final_absolute_error_rad": abs(errors[-1]),
        "tracking_rmse_rad": math.sqrt(sum(error * error for error in errors) / len(errors)),
        "integral_absolute_error_rad_s": sum(abs(error) for error in errors)
        * fixed_delta_s,
        "maximum_absolute_error_rad": max(abs(error) for error in errors),
    }


def requirement_map(suite: dict[str, Any]) -> dict[str, dict[str, Any]]:
    requirements = suite.get("requirements")
    if not isinstance(requirements, list):
        raise ValueError("payload suite has no requirements")
    result = {item["id"]: item for item in requirements}
    if len(result) != len(requirements):
        raise ValueError("payload requirement IDs are not unique")
    return result


def check(identifier: str, observed: Any, requirement: dict[str, Any]) -> dict[str, Any]:
    passed = (
        observed == requirement["required"]
        if "required" in requirement
        else observed <= requirement["maximum"]
    )
    return {
        "requirement_id": identifier,
        "unit": requirement["unit"],
        "observed": observed,
        "maximum": requirement.get("maximum"),
        "required": requirement.get("required"),
        "status": "passed" if passed else "failed",
    }


def outcome_status(checks: list[dict[str, Any]], payload_supported: bool) -> str:
    mass_id = "payload.maximum_supported_mass_kg"
    non_mass_passed = all(
        item["status"] == "passed" for item in checks if item["requirement_id"] != mass_id
    )
    mass_check = next(item for item in checks if item["requirement_id"] == mass_id)
    if payload_supported and non_mass_passed and mass_check["status"] == "passed":
        return "passed"
    if not payload_supported and non_mass_passed and mass_check["status"] == "failed":
        return "failed_as_expected"
    return "failed"


def build_report(
    fixture_root: Path,
    trace_root: Path,
    experiment_path: Path,
    base_urdf_path: Path,
    controller_path: Path,
) -> dict[str, Any]:
    suite_path = fixture_root / "payload-suite.json"
    suite = load(suite_path)
    experiment = load(experiment_path)
    controller = load(controller_path)
    if (
        suite.get("kind") != "rne_openarm_payload_suite"
        or suite.get("schema_version") != 1
        or suite.get("experiment_id") != experiment.get("experiment_id")
        or suite.get("controlled_joint") != experiment.get("controlled_joint")
        or suite.get("requirements") != experiment.get("requirements")
    ):
        raise ValueError("payload suite and experiment manifest differ")
    if suite["inputs"]["experiment_manifest_sha256"] != sha256(experiment_path):
        raise ValueError("payload suite experiment hash differs")
    if suite["inputs"]["source_model_sha256"] != sha256(base_urdf_path):
        raise ValueError("payload suite source model hash differs")
    joint_name = suite["controlled_joint"]
    joint_index = controller["action_joint_order"].index(joint_name)
    controller_sha256 = sha256(controller_path)
    requirements = requirement_map(suite)
    base_mass, base_com, base_inertia = link_inertial(
        base_urdf_path, experiment["attachment_parent_link"]
    )
    payload_com = [
        experiment["attachment_origin_xyz_m"][index]
        + experiment["payload_center_of_mass_local_m"][index]
        for index in range(3)
    ]
    model_cases = []
    outcomes = []
    missing_traces = []
    action_hashes = set()
    for case in suite["cases"]:
        case_dir = fixture_root / case["case_id"]
        fixture_path = case_dir / "payload-fixture.json"
        urdf_path = case_dir / "openarm_v2_right.payload.urdf"
        robot_path = case_dir / "openarm_payload.rne.robot.toml"
        scene_path = case_dir / "openarm_payload.rne.scene.toml"
        fixture = load(fixture_path)
        for path, field in (
            (urdf_path, "model_urdf_sha256"),
            (robot_path, "robot_asset_config_sha256"),
            (scene_path, "scene_config_sha256"),
        ):
            if sha256(path) != fixture[field] or fixture[field] != case[field]:
                raise ValueError(f"{case['case_id']} {field} differs")
        actual = link_inertial(urdf_path, experiment["attachment_parent_link"])
        expected = (
            (base_mass, base_com, base_inertia)
            if case["payload_mass_kg"] == 0.0
            else combined_inertial(
                base_mass,
                base_com,
                base_inertia,
                case["payload_mass_kg"],
                payload_com,
                experiment["payload_box_size_m"],
            )
        )
        realization_delta = maximum_parameter_delta(actual, expected)
        model_check = check(
            "payload.maximum_model_parameter_realization_delta",
            realization_delta,
            requirements["payload.maximum_model_parameter_realization_delta"],
        )
        model_cases.append(
            {
                "case_id": case["case_id"],
                "payload_mass_kg": case["payload_mass_kg"],
                "combined_mass_kg": actual[0],
                "combined_center_of_mass_m": actual[1],
                "combined_inertia_kg_m2": actual[2],
                "maximum_parameter_realization_delta": realization_delta,
                "check": model_check,
                "model_urdf_sha256": case["model_urdf_sha256"],
            }
        )
        for backend_id in suite["backend_order"]:
            directory, filename = TRACE_FILES[backend_id]
            trace_path = trace_root / directory / case["case_id"] / filename
            if not trace_path.exists():
                missing_traces.append(
                    {"backend_id": backend_id, "case_id": case["case_id"]}
                )
                continue
            trace = load(trace_path)
            if (
                trace.get("backend_id") != backend_id
                or trace.get("controller_sha256") != controller_sha256
                or trace.get("replay_match") is not True
            ):
                raise ValueError(f"{backend_id} {case['case_id']} trace identity differs")
            if backend_id == "gazebo_sim":
                if trace.get("robot_model_sha256") != case["model_urdf_sha256"]:
                    raise ValueError("Gazebo trace model hash differs")
            elif (
                trace.get("model_urdf_sha256") != case["model_urdf_sha256"]
                or trace.get("robot_asset_config_sha256")
                != case["robot_asset_config_sha256"]
                or trace.get("scene_config_sha256") != case["scene_config_sha256"]
            ):
                raise ValueError(f"{backend_id} native model provenance differs")
            action_hashes.add(trace["action_trace_sha256"])
            metrics = controlled_joint_metrics(trace, joint_index, 0.016666667)
            checks = [
                model_check,
                check(
                    "payload.maximum_controlled_joint_rmse_rad",
                    metrics["tracking_rmse_rad"],
                    requirements["payload.maximum_controlled_joint_rmse_rad"],
                ),
                check(
                    "payload.maximum_controlled_joint_final_error_rad",
                    metrics["final_absolute_error_rad"],
                    requirements["payload.maximum_controlled_joint_final_error_rad"],
                ),
                check(
                    "payload.maximum_supported_mass_kg",
                    case["payload_mass_kg"],
                    requirements["payload.maximum_supported_mass_kg"],
                ),
                check(
                    "payload.requires_exact_replay",
                    trace["replay_match"],
                    requirements["payload.requires_exact_replay"],
                ),
            ]
            supported = (
                case["payload_mass_kg"]
                <= requirements["payload.maximum_supported_mass_kg"]["maximum"]
            )
            outcomes.append(
                {
                    "backend_id": backend_id,
                    "case_id": case["case_id"],
                    "payload_mass_kg": case["payload_mass_kg"],
                    "trace_sha256": sha256(trace_path),
                    "action_trace_sha256": trace["action_trace_sha256"],
                    "metrics": metrics,
                    "checks": checks,
                    "expected_payload_supported": supported,
                    "status": outcome_status(checks, supported),
                }
            )
    if len(action_hashes) > 1:
        raise ValueError("payload traces do not use one compiled action artifact")
    first_failures = []
    for backend_id in suite["backend_order"]:
        backend_outcomes = [item for item in outcomes if item["backend_id"] == backend_id]
        failed = next(
            (
                item
                for item in backend_outcomes
                if item["status"] in {"failed", "failed_as_expected"}
            ),
            None,
        )
        first_failures.append(
            {
                "backend_id": backend_id,
                "first_failing_case_id": failed["case_id"] if failed else None,
                "first_failing_mass_kg": failed["payload_mass_kg"] if failed else None,
                "first_failed_requirement": next(
                    (
                        item["requirement_id"]
                        for item in failed["checks"]
                        if item["status"] == "failed"
                    ),
                    None,
                )
                if failed
                else None,
            }
        )
    complete = not missing_traces and len(outcomes) == len(suite["cases"]) * len(
        suite["backend_order"]
    )
    all_pass = complete and all(
        item["status"] in {"passed", "failed_as_expected"} for item in outcomes
    )
    return {
        "kind": "rne_openarm_payload_robustness_report",
        "schema_version": 1,
        "status": "passed" if all_pass else ("needs_tuning" if complete else "incomplete"),
        "experiment_id": suite["experiment_id"],
        "controlled_joint": joint_name,
        "fixed_delta_ticks": 16_666_667,
        "backend_order": suite["backend_order"],
        "requirements": suite["requirements"],
        "inputs": {
            "payload_suite_sha256": sha256(suite_path),
            "experiment_manifest_sha256": sha256(experiment_path),
            "source_model_sha256": sha256(base_urdf_path),
            "controller_sha256": controller_sha256,
            "action_trace_sha256": next(iter(action_hashes), None),
        },
        "model_cases": model_cases,
        "outcomes": outcomes,
        "missing_traces": missing_traces,
        "first_failures": first_failures,
    }


def write_outputs(output: Path, report: dict[str, Any]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    json_path = output / "openarm-payload-report.json"
    html_path = output / "openarm-payload-report.html"
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    html = '''<!doctype html><meta charset="utf-8"><title>OpenArm physical payload robustness</title><style>
body{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}main{max-width:1280px;margin:auto;padding:28px}table{width:100%;border-collapse:collapse}th,td{border:1px solid #294563;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}.card{background:#112238;border:1px solid #294563;border-radius:10px;padding:14px;margin:12px 0}.passed,.failed_as_expected{color:#64e6a1}.failed,.needs_tuning{color:#ff9b85}.incomplete{color:#ffd477}code{word-break:break-all}</style><main><h1>OpenArm physical payload robustness</h1><p>Status: <b id="status"></b></p><div id="summary" class="card"></div><h2>Backend outcomes</h2><table><thead><tr><th>backend / case</th><th>mass kg</th><th>RMSE rad</th><th>final rad</th><th>IAE rad·s</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><h2>First failures</h2><div id="failures" class="card"></div><script>const r=__REPORT__,f=x=>Number(x).toFixed(6),s=document.querySelector('#status');s.textContent=r.status;s.className=r.status;document.querySelector('#summary').innerHTML=`controlled joint: <b>${r.controlled_joint}</b> · model cases: ${r.model_cases.length} · traces: ${r.outcomes.length} · missing: ${r.missing_traces.length}<br>action <code>${r.inputs.action_trace_sha256||'not available'}</code>`;document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${x.backend_id} / ${x.case_id}</td><td>${x.payload_mass_kg.toFixed(3)}</td><td>${f(x.metrics.tracking_rmse_rad)}</td><td>${f(x.metrics.final_absolute_error_rad)}</td><td>${f(x.metrics.integral_absolute_error_rad_s)}</td><td class=${x.status}>${x.status}</td></tr>`).join('');document.querySelector('#failures').innerHTML=r.first_failures.map(x=>`<p><b>${x.backend_id}</b>: ${x.first_failing_case_id||'none'} ${x.first_failed_requirement||''}</p>`).join('');</script></main>'''.replace(
        "__REPORT__", payload
    )
    html_path.write_text(html + "\n", encoding="utf-8")


def main() -> None:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-root", required=True, type=Path)
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--experiment",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_payload_experiments.json",
    )
    parser.add_argument(
        "--base-urdf",
        type=Path,
        default=root
        / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
    )
    parser.add_argument(
        "--controller",
        type=Path,
        default=root
        / "docs/evidence/openarm-controller-lab/evidence/openarm-plant-state-feedback.controller.json",
    )
    args = parser.parse_args()
    report = build_report(
        args.fixture_root,
        args.trace_root,
        args.experiment,
        args.base_urdf,
        args.controller,
    )
    write_outputs(args.output, report)
    print(
        f"OpenArm payload report: status={report['status']} outcomes={len(report['outcomes'])}"
    )


if __name__ == "__main__":
    main()

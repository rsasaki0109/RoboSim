#!/usr/bin/env python3
"""Build a browser report for the OpenArm recorded/playback/shadow gate."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
from typing import Any


CASES = (
    ("recorded_playback", "openarm-recorded-playback", "playback", "passed"),
    ("cross_backend_shadow", "openarm-shadow-cross-backend", "shadow", "failed"),
    ("transport_disconnect", "openarm-shadow-disconnect", "shadow", "failed_as_expected"),
)


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--session-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--task", type=Path, default=root / "openarm_right_joint_tracking.task.json"
    )
    parser.add_argument(
        "--controller",
        type=Path,
        default=root / "openarm_right_pose_cycle.controller.json",
    )
    parser.add_argument(
        "--requirements",
        type=Path,
        default=root / "openarm_recorded_shadow_requirements.json",
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


def first_violation(report: dict[str, Any]) -> dict[str, Any] | None:
    for sample in report["comparison"]["samples"]:
        violation = sample.get("first_violation")
        if violation is not None:
            return {
                "observation_sequence": sample["hardware_sequence"],
                "simulation_step": sample["simulation_step"],
                "simulation_time_ticks": sample["simulation_time_ticks"],
                **violation,
            }
    return None


def build_report(
    session_root: Path,
    task_path: Path,
    controller_path: Path,
    requirements_path: Path,
) -> dict[str, Any]:
    task = load(task_path)
    controller = load(controller_path)
    requirements = load(requirements_path)
    task_hash = sha256(task_path)
    controller_hash = sha256(controller_path)
    requirements_hash = sha256(requirements_path)
    results = []
    checks = []
    for role, stem, expected_mode, expected_status in CASES:
        session_path = session_root / f"{stem}.session.json"
        report_path = session_root / f"{stem}.report.json"
        session = load(session_path)
        report = load(report_path)
        if (
            report.get("kind") != "rne_recorded_shadow_report"
            or report.get("schema_version") != 1
            or report.get("session_sha256") != sha256(session_path)
            or report.get("task_id") != task.get("task_id")
            or report.get("task_sha256") != task_hash
            or report.get("controller_id") != controller.get("controller_id")
            or report.get("controller_sha256") != controller_hash
            or report.get("experiment_id") != requirements.get("experiment_id")
            or report.get("requirements_sha256") != requirements_hash
            or report.get("mode") != expected_mode
            or report.get("stream") != session.get("stream")
            or report.get("sources") != session.get("sources")
            or report["gateway"].get("task_id") != task["task_id"]
            or report["gateway"].get("mode") != expected_mode
            or report["gateway"]["final_snapshot"].get("connection_state")
            != "disconnected"
        ):
            raise ValueError(f"{role} report provenance or terminal state differs")
        summary = report["summary"]
        comparison = report["comparison"]["summary"]
        role_checks = [
            ("status", summary["status"] == expected_status),
            (
                "sample_count",
                summary["accepted_samples"]
                >= (
                    requirements["disconnect_after_observation_sequence"]
                    if role == "transport_disconnect"
                    else requirements["minimum_compared_samples"]
                ),
            ),
            (
                "maximum_latency_ticks",
                summary["maximum_observed_latency_ticks"]
                <= requirements["maximum_latency_ticks"],
            ),
            (
                "maximum_dropped_observations",
                summary["dropped_observations"]
                <= requirements["maximum_dropped_observations"],
            ),
            (
                "zero_actuator_writes",
                summary["actuator_writes_emitted"] is False,
            ),
            (
                "all_actions_suppressed",
                summary["suppressed_actions"] == summary["accepted_samples"],
            ),
        ]
        if role == "recorded_playback":
            role_checks.append(("exact_recorded_replay", comparison["passed"] is True))
        elif role == "cross_backend_shadow":
            role_checks.append(
                ("first_cross_backend_deviation_retained", first_violation(report) is not None)
            )
        else:
            role_checks.extend(
                [
                    (
                        "transport_failure_observed",
                        summary["transport_failure_observed"] is True,
                    ),
                    ("comparison_isolated", comparison["passed"] is True),
                ]
            )
        checks.extend(
            {
                "id": f"recorded_shadow.{role}.{name}",
                "status": "passed" if passed else "failed",
            }
            for name, passed in role_checks
        )
        results.append(
            {
                "role": role,
                "mode": expected_mode,
                "status": summary["status"],
                "accepted_samples": summary["accepted_samples"],
                "suppressed_actions": summary["suppressed_actions"],
                "dropped_observations": summary["dropped_observations"],
                "maximum_observed_latency_ticks": summary[
                    "maximum_observed_latency_ticks"
                ],
                "actuator_writes_emitted": summary["actuator_writes_emitted"],
                "transport_failure_observed": summary[
                    "transport_failure_observed"
                ],
                "comparison": comparison,
                "first_violation": first_violation(report),
                "session_sha256": sha256(session_path),
                "report_sha256": sha256(report_path),
            }
        )
    status = "passed" if all(item["status"] == "passed" for item in checks) else "failed"
    return {
        "kind": "rne_openarm_recorded_shadow_gate_report",
        "schema_version": 1,
        "status": status,
        "experiment_id": requirements["experiment_id"],
        "task_id": task["task_id"],
        "controller_id": controller["controller_id"],
        "inputs": {
            "task_sha256": task_hash,
            "controller_sha256": controller_hash,
            "requirements_sha256": requirements_hash,
        },
        "clock_source": "rne_sim_clock",
        "tensor_units": [
            {"tensor_name": tensor["name"], "unit": tensor["unit"]}
            for tensor in task["observation"]["tensors"]
        ],
        "cases": results,
        "checks": checks,
    }


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    html = """<!doctype html><html lang="en"><meta charset="utf-8"><title>OpenArm recorded / shadow gate</title><style>
body{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}main{max-width:1250px;margin:auto;padding:28px}.card{background:#112238;border:1px solid #294563;border-radius:10px;padding:14px;margin:12px 0}table{width:100%;border-collapse:collapse}th,td{border:1px solid #294563;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}.passed,.failed_as_expected{color:#64e6a1}.failed{color:#ff9b85}code{word-break:break-all}</style><main><h1>OpenArm recorded playback / shadow gate</h1><p>Gate status: <b id="status"></b></p><div id="identity" class="card"></div><table><thead><tr><th>case</th><th>mode</th><th>samples</th><th>latency ticks</th><th>max delta</th><th>violations</th><th>actuator writes</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><h2>First cross-backend deviation</h2><div id="violation" class="card"></div><h2>Checks</h2><div id="checks" class="card"></div><script>const r=__REPORT__,f=x=>x==null?'—':Number(x).toFixed(6),s=document.querySelector('#status');s.textContent=r.status;s.className=r.status;document.querySelector('#identity').innerHTML=`TaskSpec <code>${r.inputs.task_sha256}</code><br>controller <code>${r.inputs.controller_sha256}</code><br>requirements <code>${r.inputs.requirements_sha256}</code>`;document.querySelector('#rows').innerHTML=r.cases.map(x=>`<tr><td>${x.role}</td><td>${x.mode}</td><td>${x.accepted_samples}</td><td>${x.maximum_observed_latency_ticks}</td><td>${f(x.comparison.max_absolute_error)}</td><td>${x.comparison.violating_elements}</td><td>${x.actuator_writes_emitted}</td><td class=${x.status}>${x.status}</td></tr>`).join('');const v=r.cases.find(x=>x.role==='cross_backend_shadow').first_violation;document.querySelector('#violation').innerHTML=v?`sequence <b>${v.observation_sequence}</b> · ${v.tensor_name}[${v.tensor_element}] · |${f(v.hardware_value)} − ${f(v.simulation_value)}| = <b>${f(v.absolute_error)} ${v.unit}</b> &gt; ${f(v.absolute_tolerance)} ${v.unit}`:'none';document.querySelector('#checks').innerHTML=r.checks.map(x=>`<div class=${x.status}>${x.status} · ${x.id}</div>`).join('');</script></main></html>""".replace(
        "__REPORT__", payload
    )
    path.write_text(html, encoding="utf-8")


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = build_report(
        args.session_root.resolve(),
        args.task.resolve(),
        args.controller.resolve(),
        args.requirements.resolve(),
    )
    json_path = output / "openarm-recorded-shadow-report.json"
    json_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    write_html(output / "openarm-recorded-shadow-report.html", report)
    print(
        f"OpenArm recorded/shadow gate: status={report['status']} "
        f"cases={len(report['cases'])} -> {output}"
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Select an OpenArm joint-loss controller from predeclared real traces."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import shutil
import sys
from typing import Any


SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_openarm_authority_report import load, metrics, sha256, write_json  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-root", required=True, type=Path)
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def limit_label(value: float) -> str:
    return f"{round(value * 1000.0):03d}mrad"


def select_outcome(outcomes: list[dict[str, Any]]) -> dict[str, Any] | None:
    passing = [item for item in outcomes if item["status"] == "passed"]
    if not passing:
        return None
    return min(
        passing,
        key=lambda item: (
            item["metrics"]["tracking_rmse_rad"],
            item["maximum_state_feedback_correction_rad"],
        ),
    )


def build_report(candidate_root: Path, trace_root: Path) -> dict[str, Any]:
    suite_path = candidate_root / "openarm-joint-loss-controller-tuning-suite.json"
    suite = load(suite_path)
    if (
        suite.get("kind") != "rne_openarm_joint_loss_controller_tuning_suite"
        or suite.get("schema_version") != 1
        or suite.get("selection_rule")
        != "minimum_rmse_among_passing_candidates_then_smallest_correction_limit"
    ):
        raise ValueError("unsupported joint-loss controller tuning suite")
    requirements = {item["id"]: item for item in suite["requirements"]}
    outcomes = []
    missing = []
    for candidate in suite["candidates"]:
        controller_path = candidate_root / candidate["file"]
        if sha256(controller_path) != candidate["sha256"]:
            raise ValueError(f"candidate hash differs for {candidate['file']}")
        controller = load(controller_path)
        joint_index = controller["action_joint_order"].index(suite["controlled_joint"])
        label = limit_label(candidate["maximum_state_feedback_correction_rad"])
        trace_path = trace_root / label / "mujoco" / "mujoco-success-trace.json"
        action_path = trace_root / label / "rapier" / "controller-actions.json"
        if not trace_path.is_file() or not action_path.is_file():
            missing.append(candidate["controller_id"])
            continue
        trace = load(trace_path)
        actions = load(action_path)
        if (
            trace.get("backend_id") != suite["tuning_backend_id"]
            or trace.get("controller_id") != candidate["controller_id"]
            or trace.get("controller_sha256") != candidate["sha256"]
            or actions.get("controller_sha256") != candidate["sha256"]
            or trace.get("action_trace_sha256") != sha256(action_path)
            or trace.get("replay_match") is not True
        ):
            raise ValueError(f"trace identity differs for {candidate['controller_id']}")
        measured = metrics(trace, joint_index)
        rmse_maximum = requirements[
            "joint_loss.maximum_controlled_joint_rmse_rad"
        ]["maximum"]
        final_maximum = requirements[
            "joint_loss.maximum_controlled_joint_final_error_rad"
        ]["maximum"]
        status = (
            "passed"
            if measured["tracking_rmse_rad"] <= rmse_maximum
            and measured["final_absolute_error_rad"] <= final_maximum
            else "failed"
        )
        outcomes.append(
            {
                **candidate,
                "metrics": measured,
                "status": status,
                "trace_sha256": sha256(trace_path),
                "action_trace_sha256": sha256(action_path),
            }
        )
    selected = select_outcome(outcomes)
    status = "incomplete" if missing else ("passed" if selected else "failed")
    return {
        "kind": "rne_openarm_joint_loss_controller_tuning_report",
        "schema_version": 1,
        "status": status,
        "tuning_id": suite["tuning_id"],
        "tuning_backend_id": suite["tuning_backend_id"],
        "tuning_case_id": suite["tuning_case_id"],
        "selection_rule": suite["selection_rule"],
        "validation_scope": suite["validation_scope"],
        "suite_sha256": sha256(suite_path),
        "outcomes": outcomes,
        "missing_controller_ids": missing,
        "selected": selected,
    }


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    document = """<!doctype html><html lang="en"><meta charset="utf-8"><title>OpenArm joint-loss controller tuning</title><style>
body{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}main{max-width:1050px;margin:auto;padding:28px}.card{background:#112238;border:1px solid #294563;border-radius:10px;padding:14px}table{width:100%;border-collapse:collapse;margin-top:14px}th,td{border:1px solid #294563;padding:8px;text-align:right}th:first-child,td:first-child{text-align:left}.passed{color:#64e6a1}.failed{color:#ff9b85}</style><main><h1>OpenArm joint-loss controller tuning</h1><div id="summary" class="card"></div><table><thead><tr><th>controller</th><th>limit rad</th><th>RMSE rad</th><th>final rad</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><script>const r=__REPORT__,f=x=>Number(x).toFixed(6),s=r.selected;document.querySelector('#summary').innerHTML=`status: <b class=${r.status}>${r.status}</b><br>selection: ${r.selection_rule}<br>selected: ${s?s.controller_id:'none'}`;document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${x.controller_id}</td><td>${f(x.maximum_state_feedback_correction_rad)}</td><td>${f(x.metrics.tracking_rmse_rad)}</td><td>${f(x.metrics.final_absolute_error_rad)}</td><td class=${x.status}>${x.status}</td></tr>`).join('');</script></main></html>""".replace(
        "__REPORT__", payload
    )
    path.write_text(document, encoding="utf-8")


def main() -> int:
    args = parse_args()
    candidate_root = args.candidate_root.resolve()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    report = build_report(candidate_root, args.trace_root.resolve())
    write_json(output / "openarm-joint-loss-controller-tuning-report.json", report)
    write_html(output / "openarm-joint-loss-controller-tuning-report.html", report)
    selected = report["selected"]
    if selected is not None:
        source = candidate_root / selected["file"]
        selected_path = output / "openarm-joint-loss-selected.controller.json"
        shutil.copyfile(source, selected_path)
        if sha256(selected_path) != selected["sha256"]:
            raise ValueError("selected controller copy differs from candidate")
    print(
        f"OpenArm joint-loss controller tuning report: status={report['status']} "
        f"selected={selected['controller_id'] if selected else 'none'}"
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm joint-loss controller tuning report failed: {error}")
        raise SystemExit(2)

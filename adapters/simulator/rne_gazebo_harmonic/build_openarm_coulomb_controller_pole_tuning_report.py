#!/usr/bin/env python3
"""Build the OpenArm Coulomb controller pole-placement tuning report."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_openarm_authority_report import load, metrics, sha256, write_json  # noqa: E402
from build_openarm_coulomb_friction_report import native_realization  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--candidate-root", required=True, type=Path)
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def select_candidate(outcomes: list[dict[str, Any]]) -> dict[str, Any] | None:
    passing = [outcome for outcome in outcomes if outcome["status"] == "passed"]
    return min(
        passing,
        key=lambda outcome: (outcome["metrics"]["tracking_rmse_rad"], outcome["maximum_pole"]),
        default=None,
    )


def build_report(candidate_root: Path, trace_root: Path) -> dict[str, Any]:
    suite_path = candidate_root / "openarm-coulomb-controller-pole-tuning-suite.json"
    suite = load(suite_path)
    if (
        suite.get("kind") != "rne_openarm_coulomb_controller_pole_tuning_suite"
        or suite.get("schema_version") != 1
        or suite.get("tuning_backend_id") != "rne_rapier"
    ):
        raise ValueError("unsupported Coulomb controller pole tuning suite")
    outcomes = []
    shared_trajectory = None
    for candidate in suite["candidates"]:
        controller_path = candidate_root / candidate["file"]
        if sha256(controller_path) != candidate["sha256"]:
            raise ValueError(f"{candidate['candidate_id']} controller differs")
        controller = load(controller_path)
        trajectory = {
            "task_id": controller["task_id"],
            "interpolation": controller["interpolation"],
            "action_joint_order": controller["action_joint_order"],
            "keyframes": controller["keyframes"],
            "observation_contract": controller["observation_contract"],
            "disturbance_contract": controller["disturbance_contract"],
        }
        if shared_trajectory is None:
            shared_trajectory = trajectory
        elif trajectory != shared_trajectory:
            raise ValueError("pole candidates changed the trajectory or external contract")
        trace_path = trace_root / candidate["candidate_id"] / "rapier-success-trace.json"
        trace = load(trace_path)
        joint_index = controller["action_joint_order"].index(suite["controlled_joint"])
        realized = native_realization(trace, joint_index)
        measured = metrics(trace, joint_index)
        checks = {
            "controller_identity": trace.get("controller_sha256") == candidate["sha256"],
            "plant_realization": realized[1:] == (
                suite["plant_coulomb_friction_nm"],
                suite["plant_coulomb_transition_velocity_rad_s"],
            ),
            "tracking_rmse": measured["tracking_rmse_rad"]
            <= suite["maximum_controlled_joint_rmse_rad"],
            "final_error": measured["final_absolute_error_rad"]
            <= suite["maximum_controlled_joint_final_error_rad"],
            "exact_replay": trace.get("replay_match") is suite["requires_exact_replay"],
        }
        outcomes.append(
            {
                **candidate,
                "metrics": measured,
                "checks": checks,
                "status": "passed" if all(checks.values()) else "failed",
                "trace_sha256": sha256(trace_path),
                "action_trace_sha256": trace["action_trace_sha256"],
            }
        )
    selected = select_candidate(outcomes)
    return {
        "kind": "rne_openarm_coulomb_controller_pole_tuning_report",
        "schema_version": 1,
        "tuning_id": suite["tuning_id"],
        "status": "passed" if selected else "needs_tuning",
        "controlled_joint": suite["controlled_joint"],
        "tuning_backend_id": suite["tuning_backend_id"],
        "selection_rule": suite["selection_rule"],
        "validation_rule": suite["validation_rule"],
        "shared_trajectory_sha256": sha256_bytes(shared_trajectory),
        "suite_sha256": sha256(suite_path),
        "outcomes": outcomes,
        "selected_candidate_id": selected["candidate_id"] if selected else None,
        "next_experiment": None if selected else "bounded model-based Coulomb feedforward",
    }


def sha256_bytes(value: Any) -> str:
    import hashlib

    return hashlib.sha256(
        json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
    ).hexdigest()


def render_html(report: dict[str, Any]) -> str:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    return f"""<!doctype html><html><meta charset="utf-8"><title>OpenArm Coulomb pole tuning</title><style>body{{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}}main{{max-width:1000px;margin:auto;padding:28px}}table{{width:100%;border-collapse:collapse}}th,td{{border:1px solid #294563;padding:7px;text-align:right}}th:first-child,td:first-child{{text-align:left}}.passed{{color:#64e6a1}}.failed,.needs_tuning{{color:#ff9b85}}</style><main><h1>OpenArm Coulomb controller pole tuning</h1><p>Status: <b>{report['status']}</b></p><table><thead><tr><th>candidate</th><th>maximum pole</th><th>RMSE rad</th><th>final rad</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><script>const r={payload},f=x=>Number(x).toFixed(6);document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${{x.candidate_id}}</td><td>${{f(x.maximum_pole)}}</td><td>${{f(x.metrics.tracking_rmse_rad)}}</td><td>${{f(x.metrics.final_absolute_error_rad)}}</td><td class="${{x.status}}">${{x.status}}</td></tr>`).join('');</script></main></html>"""


def main() -> int:
    args = parse_args()
    report = build_report(args.candidate_root.resolve(), args.trace_root.resolve())
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output / "openarm-coulomb-controller-pole-tuning-report.json", report)
    (args.output / "openarm-coulomb-controller-pole-tuning-report.html").write_text(
        render_html(report), encoding="utf-8"
    )
    print(
        f"OpenArm Coulomb pole tuning report: status={report['status']} "
        f"selected={report['selected_candidate_id']}"
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

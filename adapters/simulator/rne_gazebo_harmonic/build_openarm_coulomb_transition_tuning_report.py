#!/usr/bin/env python3
"""Build the browser-readable OpenArm Coulomb transition tuning report."""

from __future__ import annotations

import argparse
import html
import json
from pathlib import Path
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_openarm_authority_report import load, metrics, sha256, write_json
from build_openarm_coulomb_friction_report import native_realization


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fixture-root", required=True, type=Path)
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--controller", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def evaluate_candidate(
    candidate: dict[str, Any],
    requirements: dict[str, Any],
    trace: dict[str, Any],
    joint_index: int,
) -> dict[str, Any]:
    measured = metrics(trace, joint_index)
    realized = native_realization(trace, joint_index)
    checks = {
        "parameter_realization": abs(realized[2] - candidate["transition_velocity_rad_s"])
        <= 1.0e-12,
        "kinetic_fraction": candidate["kinetic_fraction_at_reference_velocity"]
        >= requirements["minimum_kinetic_fraction_at_reference_velocity"],
        "tracking_rmse": measured["tracking_rmse_rad"]
        <= requirements["maximum_controlled_joint_rmse_rad"],
        "final_error": measured["final_absolute_error_rad"]
        <= requirements["maximum_controlled_joint_final_error_rad"],
        "exact_replay": trace.get("replay_match")
        is requirements["requires_exact_replay"],
    }
    return {
        **candidate,
        "realized_transition_velocity_rad_s": realized[2],
        "metrics": measured,
        "checks": checks,
        "status": "passed" if all(checks.values()) else "failed",
    }


def select_candidate(outcomes: list[dict[str, Any]]) -> dict[str, Any] | None:
    passing = [outcome for outcome in outcomes if outcome["status"] == "passed"]
    return max(passing, key=lambda outcome: outcome["transition_velocity_rad_s"], default=None)


def build_report(
    fixture_root: Path, trace_root: Path, controller_path: Path
) -> dict[str, Any]:
    suite_path = fixture_root / "coulomb-transition-tuning-suite.json"
    suite = load(suite_path)
    if (
        suite.get("kind") != "rne_openarm_coulomb_transition_tuning_suite"
        or suite.get("schema_version") != 1
        or suite.get("tuning_backend_id") != "rne_rapier"
    ):
        raise ValueError("unsupported Coulomb transition tuning suite")
    controller = load(controller_path)
    joint_index = controller["action_joint_order"].index(suite["controlled_joint"])
    controller_sha = sha256(controller_path)
    outcomes = []
    action_hashes = set()
    for candidate in suite["candidates"]:
        candidate_root = fixture_root / candidate["candidate_id"]
        case_root = candidate_root / "fixtures" / candidate["case_id"]
        trace_path = trace_root / candidate["candidate_id"] / "rapier-success-trace.json"
        candidate_manifest = candidate_root / "coulomb-friction-experiment.json"
        suite_candidate = candidate_root / "fixtures" / "coulomb-friction-suite.json"
        fixture = case_root / "coulomb-friction-fixture.json"
        for path, expected in (
            (candidate_manifest, candidate["candidate_manifest_sha256"]),
            (suite_candidate, candidate["suite_sha256"]),
            (fixture, candidate["fixture_sha256"]),
            (
                case_root / "openarm_payload.rne.robot.toml",
                candidate["robot_asset_config_sha256"],
            ),
            (
                case_root / "openarm_v2_right.coulomb.urdf",
                candidate["portable_model_urdf_sha256"],
            ),
        ):
            if sha256(path) != expected:
                raise ValueError(f"{candidate['candidate_id']} provenance differs")
        trace = load(trace_path)
        if (
            trace.get("backend_id") != "rne_rapier"
            or trace.get("controller_sha256") != controller_sha
            or trace.get("model_urdf_sha256")
            != candidate["portable_model_urdf_sha256"]
            or trace.get("robot_asset_config_sha256")
            != candidate["robot_asset_config_sha256"]
        ):
            raise ValueError(f"{candidate['candidate_id']} trace identity differs")
        action_hashes.add(trace["action_trace_sha256"])
        outcome = evaluate_candidate(
            candidate, suite["requirements"], trace, joint_index
        )
        outcome["trace_sha256"] = sha256(trace_path)
        outcomes.append(outcome)
    if len(action_hashes) != 1:
        raise ValueError("transition candidates did not use one action trace")
    selected = select_candidate(outcomes)
    return {
        "kind": "rne_openarm_coulomb_transition_tuning_report",
        "schema_version": 1,
        "experiment_id": suite["experiment_id"],
        "status": "passed" if selected is not None else "needs_tuning",
        "controlled_joint": suite["controlled_joint"],
        "tuning_backend_id": suite["tuning_backend_id"],
        "plant_viscous_damping_nm_s_per_rad": suite[
            "plant_viscous_damping_nm_s_per_rad"
        ],
        "plant_coulomb_friction_nm": suite["plant_coulomb_friction_nm"],
        "selection_rule": suite["selection_rule"],
        "validation_rule": suite["validation_rule"],
        "requirements": suite["requirements"],
        "controller_sha256": controller_sha,
        "action_trace_sha256": next(iter(action_hashes)),
        "suite_sha256": sha256(suite_path),
        "outcomes": outcomes,
        "selected_transition_velocity_rad_s": (
            selected["transition_velocity_rad_s"] if selected else None
        ),
        "next_experiment": (
            None
            if selected
            else "predeclared physics-substep sensitivity with controller and plant held fixed"
        ),
    }


def render_html(report: dict[str, Any]) -> str:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    return f"""<!doctype html><html><meta charset="utf-8"><title>OpenArm Coulomb transition tuning</title><style>
body{{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}}main{{max-width:1050px;margin:auto;padding:28px}}table{{width:100%;border-collapse:collapse}}th,td{{border:1px solid #294563;padding:7px;text-align:right}}th:first-child,td:first-child{{text-align:left}}.passed{{color:#64e6a1}}.failed,.needs_tuning{{color:#ff9b85}}</style><main><h1>OpenArm Coulomb transition tuning</h1><p>Status: <b class="{html.escape(report['status'])}">{html.escape(report['status'])}</b></p><table><thead><tr><th>candidate</th><th>transition rad/s</th><th>kinetic fraction</th><th>RMSE rad</th><th>final rad</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><script>const r={payload},f=x=>Number(x).toFixed(6);document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${{x.candidate_id}}</td><td>${{f(x.transition_velocity_rad_s)}}</td><td>${{f(x.kinetic_fraction_at_reference_velocity)}}</td><td>${{f(x.metrics.tracking_rmse_rad)}}</td><td>${{f(x.metrics.final_absolute_error_rad)}}</td><td class="${{x.status}}">${{x.status}}</td></tr>`).join('');</script></main></html>"""


def main() -> int:
    args = parse_args()
    report = build_report(
        args.fixture_root.resolve(),
        args.trace_root.resolve(),
        args.controller.resolve(),
    )
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output / "openarm-coulomb-transition-tuning-report.json", report)
    (args.output / "openarm-coulomb-transition-tuning-report.html").write_text(
        render_html(report), encoding="utf-8"
    )
    print(
        f"OpenArm Coulomb transition tuning report: status={report['status']} "
        f"candidates={len(report['outcomes'])}"
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

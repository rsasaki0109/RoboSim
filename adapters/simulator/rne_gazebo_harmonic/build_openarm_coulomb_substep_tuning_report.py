#!/usr/bin/env python3
"""Build the OpenArm Rapier Coulomb physics-substep tuning report."""

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
    parser.add_argument("--fixture-root", required=True, type=Path)
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--controller", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def select_candidate(outcomes: list[dict[str, Any]]) -> dict[str, Any] | None:
    passing = [outcome for outcome in outcomes if outcome["status"] == "passed"]
    return min(
        passing,
        key=lambda outcome: outcome["physics_substeps_per_control_step"],
        default=None,
    )


def build_report(
    fixture_root: Path, trace_root: Path, controller_path: Path
) -> dict[str, Any]:
    suite_path = fixture_root / "coulomb-substep-tuning-suite.json"
    suite = load(suite_path)
    controller = load(controller_path)
    if (
        suite.get("kind") != "rne_openarm_coulomb_substep_tuning_suite"
        or suite.get("schema_version") != 1
        or suite.get("tuning_backend_id") != "rne_rapier"
    ):
        raise ValueError("unsupported Coulomb substep tuning suite")
    joint_index = controller["action_joint_order"].index(suite["controlled_joint"])
    controller_sha = sha256(controller_path)
    outcomes = []
    action_hashes = set()
    for candidate in suite["candidates"]:
        case_root = fixture_root / candidate["candidate_id"]
        trace_path = trace_root / candidate["candidate_id"] / "rapier-success-trace.json"
        fixture_path = case_root / "coulomb-substep-fixture.json"
        for path, expected in (
            (fixture_path, candidate["fixture_sha256"]),
            (
                case_root / "openarm_right.rne_actuation.json",
                candidate["actuation_config_sha256"],
            ),
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
            or trace.get("actuation_config_sha256")
            != candidate["actuation_config_sha256"]
            or trace.get("robot_asset_config_sha256")
            != candidate["robot_asset_config_sha256"]
            or trace.get("model_urdf_sha256")
            != candidate["portable_model_urdf_sha256"]
            or trace.get("physics_substeps_per_control_step")
            != candidate["physics_substeps_per_control_step"]
        ):
            raise ValueError(f"{candidate['candidate_id']} trace identity differs")
        action_hashes.add(trace["action_trace_sha256"])
        measured = metrics(trace, joint_index)
        realized = native_realization(trace, joint_index)
        checks = {
            "plant_realization": realized
            == (
                suite["plant_viscous_damping_nm_s_per_rad"],
                suite["plant_coulomb_friction_nm"],
                suite["plant_coulomb_transition_velocity_rad_s"],
            ),
            "exact_control_period": sum(candidate["exact_substep_tick_partition"])
            == suite["control_period_ticks"]
            and trace["fixed_delta_ticks"] == suite["control_period_ticks"],
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
            }
        )
    if len(action_hashes) != 1:
        raise ValueError("substep candidates did not use one action trace")
    selected = select_candidate(outcomes)
    return {
        "kind": "rne_openarm_coulomb_substep_tuning_report",
        "schema_version": 1,
        "experiment_id": suite["experiment_id"],
        "status": "passed" if selected else "needs_tuning",
        "controlled_joint": suite["controlled_joint"],
        "tuning_backend_id": suite["tuning_backend_id"],
        "selection_rule": suite["selection_rule"],
        "validation_rule": suite["validation_rule"],
        "controller_sha256": controller_sha,
        "action_trace_sha256": next(iter(action_hashes)),
        "suite_sha256": sha256(suite_path),
        "outcomes": outcomes,
        "selected_physics_substeps_per_control_step": (
            selected["physics_substeps_per_control_step"] if selected else None
        ),
    }


def render_html(report: dict[str, Any]) -> str:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace(
        "</", "<\\/"
    )
    return f"""<!doctype html><html><meta charset="utf-8"><title>OpenArm Coulomb substep tuning</title><style>body{{margin:0;background:#07111f;color:#eaf2ff;font:14px system-ui,sans-serif}}main{{max-width:1000px;margin:auto;padding:28px}}table{{width:100%;border-collapse:collapse}}th,td{{border:1px solid #294563;padding:7px;text-align:right}}th:first-child,td:first-child{{text-align:left}}.passed{{color:#64e6a1}}.failed,.needs_tuning{{color:#ff9b85}}</style><main><h1>OpenArm Coulomb physics-substep tuning</h1><p>Status: <b>{report['status']}</b></p><table><thead><tr><th>candidate</th><th>substeps</th><th>RMSE rad</th><th>final rad</th><th>status</th></tr></thead><tbody id="rows"></tbody></table><script>const r={payload},f=x=>Number(x).toFixed(6);document.querySelector('#rows').innerHTML=r.outcomes.map(x=>`<tr><td>${{x.candidate_id}}</td><td>${{x.physics_substeps_per_control_step}}</td><td>${{f(x.metrics.tracking_rmse_rad)}}</td><td>${{f(x.metrics.final_absolute_error_rad)}}</td><td class="${{x.status}}">${{x.status}}</td></tr>`).join('');</script></main></html>"""


def main() -> int:
    args = parse_args()
    report = build_report(
        args.fixture_root.resolve(), args.trace_root.resolve(), args.controller.resolve()
    )
    args.output.mkdir(parents=True, exist_ok=True)
    write_json(args.output / "openarm-coulomb-substep-tuning-report.json", report)
    (args.output / "openarm-coulomb-substep-tuning-report.html").write_text(
        render_html(report), encoding="utf-8"
    )
    print(
        f"OpenArm Coulomb substep tuning report: status={report['status']} "
        f"selected={report['selected_physics_substeps_per_control_step']}"
    )
    return 0 if report["status"] == "passed" else 1


if __name__ == "__main__":
    raise SystemExit(main())

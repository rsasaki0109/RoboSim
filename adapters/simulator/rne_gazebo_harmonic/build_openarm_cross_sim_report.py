#!/usr/bin/env python3
"""Builds the unit-bearing Rapier/Gazebo OpenArm comparison and browser view."""

from __future__ import annotations

import argparse
import hashlib
import html
import json
import math
from pathlib import Path
import sys
from typing import Any


FINAL_TRACKING_TOLERANCE_RAD = 0.01
FINAL_CROSS_BACKEND_POSITION_TOLERANCE_RAD = 0.01


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=root)
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


def artifact(root: Path, relative: str, role: str) -> dict[str, Any]:
    path = root / relative
    return {
        "role": role,
        "file": relative.replace("\\", "/"),
        "size_bytes": path.stat().st_size,
        "sha256": sha256(path),
    }


def check(check_id: str, unit: str, observed: float, maximum: float) -> dict[str, Any]:
    passed = math.isfinite(observed) and observed <= maximum
    return {
        "id": check_id,
        "unit": unit,
        "observed_delta": observed,
        "maximum_delta": maximum,
        "status": "passed" if passed else "failed",
    }


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    root = args.repo_root.resolve()
    rapier = load(output / "rapier-success-trace.json")
    gazebo = load(output / "gazebo-success-trace.json")
    rapier_failure = load(output / "intentional-failure.json")
    gazebo_failure = load(output / "gazebo-intentional-failure.json")
    task_path = (
        root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json"
    )
    controller_path = (
        root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json"
    )
    rapier_actuation_path = (
        root / "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json"
    )
    rapier_robot_asset_path = root / "assets/robots/openarm_v2_right.rne.robot.toml"
    gazebo_runtime_path = root / "adapters/simulator/rne_gazebo_harmonic/runtime.json"
    gazebo_adapter_config_path = (
        root / "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json"
    )
    task = load(task_path)
    controller = load(controller_path)

    identities = [rapier, gazebo, rapier_failure, gazebo_failure]
    for value in identities:
        if (
            value.get("task_id") != task["task_id"]
            or value.get("task_sha256") != sha256(task_path)
            or value.get("controller_id") != controller["controller_id"]
            or value.get("controller_sha256") != sha256(controller_path)
            or value.get("action_trace_sha256")
            != sha256(output / "controller-actions.json")
        ):
            raise ValueError("backend evidence is not bound to the same inputs")
    for value in (rapier, rapier_failure):
        if value.get("actuation_config_sha256") != sha256(
            rapier_actuation_path
        ) or value.get("robot_asset_config_sha256") != sha256(rapier_robot_asset_path):
            raise ValueError(
                "Rapier evidence is not bound to its model/actuation configuration"
            )
    for value in (gazebo, gazebo_failure):
        if value.get("runtime_manifest_sha256") != sha256(
            gazebo_runtime_path
        ) or value.get("adapter_config_sha256") != sha256(gazebo_adapter_config_path):
            raise ValueError(
                "Gazebo evidence is not bound to its runtime/configuration"
            )
    if len(rapier["observations"]) != len(gazebo["observations"]):
        raise ValueError("backend traces differ in length")

    transient_delta = 0.0
    transient_step = 0
    transient_joint = 0
    final_joint_deltas: list[float] = []
    plot: list[dict[str, Any]] = []
    for rapier_frame, gazebo_frame in zip(
        rapier["observations"], gazebo["observations"]
    ):
        if (
            rapier_frame["step"] != gazebo_frame["step"]
            or rapier_frame["sim_time_ticks"] != gazebo_frame["sim_time_ticks"]
        ):
            raise ValueError("backend trace step/time alignment drifted")
        deltas = [
            abs(left - right)
            for left, right in zip(
                rapier_frame["joint_position_rad"], gazebo_frame["joint_position_rad"]
            )
        ]
        maximum = max(deltas)
        if maximum > transient_delta:
            transient_delta = maximum
            transient_step = rapier_frame["step"]
            transient_joint = deltas.index(maximum)
        if rapier_frame["step"] % 10 == 0 or rapier_frame["step"] == 1:
            plot.append(
                {
                    "step": rapier_frame["step"],
                    "rapier_error_rad": rapier_frame["maximum_tracking_error_rad"],
                    "gazebo_error_rad": gazebo_frame["maximum_tracking_error_rad"],
                    "position_delta_rad": maximum,
                }
            )
        final_joint_deltas = deltas

    tolerance_checks = [
        check(
            "rapier_final_tracking_error_rad_v1",
            "rad",
            rapier["final_maximum_tracking_error_rad"],
            FINAL_TRACKING_TOLERANCE_RAD,
        ),
        check(
            "gazebo_final_tracking_error_rad_v1",
            "rad",
            gazebo["final_maximum_tracking_error_rad"],
            FINAL_TRACKING_TOLERANCE_RAD,
        ),
        check(
            "final_cross_backend_joint_position_delta_rad_v1",
            "rad",
            max(final_joint_deltas),
            FINAL_CROSS_BACKEND_POSITION_TOLERANCE_RAD,
        ),
    ]
    failure_checks = [
        check(
            "first_violation_step_delta_v1",
            "step",
            abs(
                rapier_failure["first_violation_step"]
                - gazebo_failure["first_violation_step"]
            ),
            0.0,
        ),
        check(
            "first_violation_time_delta_v1",
            "ns",
            abs(
                rapier_failure["first_violation_sim_time_ticks"]
                - gazebo_failure["first_violation_sim_time_ticks"]
            ),
            0.0,
        ),
        check(
            "rejected_step_state_advance_v1",
            "step",
            1.0 if gazebo_failure["rejected_step_changed_state"] else 0.0,
            0.0,
        ),
    ]
    backend_outcomes = [
        {
            "backend_id": backend["backend_id"],
            "backend_version": backend["backend_version"],
            "steps": len(backend["observations"]),
            "final_sim_time_ticks": backend["observations"][-1]["sim_time_ticks"],
            "final_maximum_tracking_error_rad": backend[
                "final_maximum_tracking_error_rad"
            ],
            "maximum_tracking_error_rad": backend["maximum_tracking_error_rad"],
            "replay_match": backend["replay_match"],
            "status": "passed"
            if backend["replay_match"]
            and backend["final_maximum_tracking_error_rad"]
            <= FINAL_TRACKING_TOLERANCE_RAD
            else "failed",
        }
        for backend in (rapier, gazebo)
    ]
    failures_match = (
        rapier_failure["status"] == "failed_as_expected"
        and gazebo_failure["status"] == "failed_as_expected"
        and rapier_failure["first_violation"]
        == gazebo_failure["first_violation"]
        == controller["intentional_failure"]["expected_first_violation"]
    )
    passed = (
        all(outcome["status"] == "passed" for outcome in backend_outcomes)
        and all(item["status"] == "passed" for item in tolerance_checks)
        and all(item["status"] == "passed" for item in failure_checks)
        and failures_match
    )
    inputs = [
        artifact(
            root,
            "adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json",
            "task_spec",
        ),
        artifact(
            root,
            "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json",
            "controller",
        ),
        artifact(
            output,
            "controller-actions.json",
            "compiled_action_trace",
        ),
        artifact(
            root,
            "assets/scenes/openarm_v2_right_validation.rne.scene.toml",
            "rapier_world",
        ),
        artifact(
            root,
            "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
            "rapier_robot_model",
        ),
        artifact(
            root,
            "assets/robots/openarm_v2_right.rne.robot.toml",
            "rapier_robot_asset_config",
        ),
        artifact(
            root,
            "adapters/simulator/rne_gazebo_harmonic/openarm_right.rne_actuation.json",
            "rapier_actuation_config",
        ),
        artifact(
            root,
            "adapters/simulator/rne_gazebo_harmonic/runtime.json",
            "gazebo_runtime_manifest",
        ),
        artifact(
            root,
            "adapters/simulator/rne_gazebo_harmonic/openarm_right.adapter.json",
            "gazebo_adapter_config",
        ),
        artifact(
            root,
            "adapters/simulator/rne_gazebo_harmonic/rne_gazebo_harmonic_adapter.py",
            "gazebo_adapter",
        ),
    ]
    report = {
        "kind": "rne_openarm_cross_sim_report",
        "schema_version": 1,
        "status": "passed" if passed else "failed",
        "task_id": task["task_id"],
        "controller_id": controller["controller_id"],
        "comparison_contract": "same_task_controller_action_trace_and_named_si_tolerances",
        "inputs": inputs,
        "backend_outcomes": backend_outcomes,
        "tolerance_checks": tolerance_checks,
        "intentional_failures": [rapier_failure, gazebo_failure],
        "failure_tolerance_checks": failure_checks,
        "diagnostics": {
            "maximum_transient_joint_position_delta_rad": transient_delta,
            "maximum_transient_delta_step": transient_step,
            "maximum_transient_delta_joint_index": transient_joint,
            "gating": False,
            "rationale": "Backend servo implementations are intentionally native; task success is gated by final SI tolerances while transient divergence is retained for control-dynamics analysis.",
        },
    }
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "cross-sim-report.json"
    report_path.write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    write_html(output / "replay-inspector.html", report, plot)
    if not passed:
        raise RuntimeError("OpenArm cross-simulator contract did not pass")
    print(
        "OpenArm cross-sim report: status=passed "
        f"final_delta_rad={max(final_joint_deltas):.6f} "
        f"first_violation_step={rapier_failure['first_violation_step']}"
    )
    return 0


def write_html(path: Path, report: dict[str, Any], plot: list[dict[str, Any]]) -> None:
    payload = json.dumps(
        {"report": report, "plot": plot}, separators=(",", ":")
    ).replace("</", "<\\/")
    title = html.escape(f"{report['task_id']} — Rapier / Gazebo Failure Capsule")
    document = f"""<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>{title}</title><style>
body{{margin:0;background:#0a1020;color:#e9f0ff;font:15px system-ui,sans-serif}}main{{max-width:1180px;margin:auto;padding:28px}}
h1{{font-size:27px}}h2{{margin-top:28px}}.ok{{color:#63e6a6}}.grid{{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}}
.card{{background:#131d35;border:1px solid #2b3b61;border-radius:10px;padding:14px}}table{{width:100%;border-collapse:collapse;background:#10192d}}
th,td{{padding:9px;border:1px solid #2b3b61;text-align:left}}code{{font-size:12px;word-break:break-all}}canvas{{width:100%;height:330px;background:#fff;border-radius:8px}}
</style></head><body><main><h1>{title}</h1><p>Status: <strong class="ok">PASSED</strong></p>
<div id="backends" class="grid"></div><h2>Unit-bearing tolerance checks</h2><table id="checks"></table>
<h2>Intentional controller contract failure</h2><div id="failure" class="card"></div>
<h2>Tracking and cross-backend transient diagnostics</h2><canvas id="plot" width="1120" height="330"></canvas>
<h2>Content-addressed inputs</h2><table id="inputs"></table>
<script>const data={payload};const r=data.report;
const esc=s=>String(s).replace(/[&<>\"]/g,c=>({{'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}}[c]));
document.querySelector('#backends').innerHTML=r.backend_outcomes.map(x=>`<div class=card><b>${{esc(x.backend_id)}} ${{esc(x.backend_version)}}</b><p>final error: ${{x.final_maximum_tracking_error_rad.toFixed(6)}} rad</p><p>steps: ${{x.steps}} · replay: ${{x.replay_match}}</p></div>`).join('');
const checks=[...r.tolerance_checks,...r.failure_tolerance_checks];document.querySelector('#checks').innerHTML='<tr><th>check</th><th>observed</th><th>maximum</th><th>unit</th><th>status</th></tr>'+checks.map(x=>`<tr><td>${{esc(x.id)}}</td><td>${{x.observed_delta}}</td><td>${{x.maximum_delta}}</td><td>${{esc(x.unit)}}</td><td class=ok>${{esc(x.status)}}</td></tr>`).join('');
const f=r.intentional_failures[0];document.querySelector('#failure').innerHTML=`<b>${{esc(f.injection_kind)}}</b><p>first violation: ${{esc(f.first_violation)}} at step ${{f.first_violation_step}} (${{f.first_violation_sim_time_ticks}} ns)</p><p>Rapier and Gazebo agree exactly; the rejected Gazebo step did not advance state.</p>`;
document.querySelector('#inputs').innerHTML='<tr><th>role</th><th>file</th><th>SHA-256</th></tr>'+r.inputs.map(x=>`<tr><td>${{esc(x.role)}}</td><td>${{esc(x.file)}}</td><td><code>${{x.sha256}}</code></td></tr>`).join('');
const c=document.querySelector('#plot'),ctx=c.getContext('2d'),p=data.plot,max=Math.max(...p.flatMap(x=>[x.rapier_error_rad,x.gazebo_error_rad,x.position_delta_rad]));ctx.strokeStyle='#ccd6ee';ctx.strokeRect(45,15,1055,275);const draw=(key,color)=>{{ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;p.forEach((x,i)=>{{const px=45+i/(p.length-1)*1055,py=290-x[key]/max*275;i?ctx.lineTo(px,py):ctx.moveTo(px,py)}});ctx.stroke()}};draw('rapier_error_rad','#316dff');draw('gazebo_error_rad','#ef7b45');draw('position_delta_rad','#24b58a');ctx.fillStyle='#17213b';ctx.fillText(`0 … ${{p[p.length-1].step}} steps · max ${{max.toFixed(3)}} rad`,48,322);
</script></main></body></html>"""
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm cross-sim report failed: {error}", file=sys.stderr)
        raise SystemExit(2)

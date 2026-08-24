#!/usr/bin/env python3
"""Builds the unit-bearing Rapier/MuJoCo/Gazebo OpenArm comparison."""

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
CONTROLLER_REPRODUCTION_TOLERANCE_RAD = 1.0e-12
PHYSICS_HASH_CONTRACT = "rne_physics_state_v2_fnv1a_1e-6_si"


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


def verify_controller_execution(
    trace: dict[str, Any], controller: dict[str, Any], actions: dict[str, Any]
) -> dict[str, Any]:
    """Independently reproduce every artifact-defined feedback decision."""
    observations = trace["observations"]
    references = actions["actions"]
    law = controller["feedback_law"]
    if trace.get("controller_execution") != "artifact_defined_joint_feedback_pid":
        raise ValueError(f"{trace['backend_id']} did not declare feedback execution")
    maximum_reference_delta = 0.0
    maximum_correction_delta = 0.0
    maximum_integral_delta = 0.0
    maximum_target_delta = 0.0
    timing_mismatches = 0
    integral_correction = [0.0] * len(controller["action_joint_order"])
    sample_period_s = (
        controller["observation_contract"]["sample_period_ticks"] / 1_000_000_000.0
    )
    for index, (frame, reference_frame) in enumerate(zip(observations, references)):
        reference = reference_frame["joint_position_target_rad"]
        maximum_reference_delta = max(
            maximum_reference_delta,
            *(
                abs(actual - expected)
                for actual, expected in zip(
                    frame["joint_reference_position_rad"], reference
                )
            ),
        )
        if index < 2:
            source = None
            expected_correction = [0.0] * len(reference)
            expected_sequence = None
            expected_age = None
            expected_bootstrap = True
        else:
            source = observations[index - 2]
            integral_correction = [
                max(
                    -maximum,
                    min(
                        maximum,
                        integral
                        + gain * (desired - position) * sample_period_s,
                    ),
                )
                for integral, gain, maximum, desired, position in zip(
                    integral_correction,
                    law["integral_error_gain_s_inv"],
                    law["maximum_integral_correction_rad"],
                    reference,
                    source["joint_position_rad"],
                )
            ]
            expected_correction = [
                max(
                    -limit,
                    min(
                        limit,
                        gain * (desired - position) - damping * velocity + integral,
                    ),
                )
                for desired, position, velocity, gain, damping, integral, limit in zip(
                    reference,
                    source["joint_position_rad"],
                    source["joint_velocity_rad_s"],
                    law["position_error_gain"],
                    law["velocity_damping_s"],
                    integral_correction,
                    law["maximum_correction_rad"],
                )
            ]
            expected_sequence = source["step"]
            expected_age = controller["observation_contract"]["latency_ticks"]
            expected_bootstrap = False
        expected_target = [
            max(minimum, min(maximum, desired + correction))
            for desired, correction, minimum, maximum in zip(
                reference,
                expected_correction,
                law["minimum_target_rad"],
                law["maximum_target_rad"],
            )
        ]
        maximum_correction_delta = max(
            maximum_correction_delta,
            *(abs(actual - expected) for actual, expected in zip(
                frame["joint_feedback_correction_rad"], expected_correction
            )),
        )
        maximum_integral_delta = max(
            maximum_integral_delta,
            *(abs(actual - expected) for actual, expected in zip(
                frame["joint_integral_correction_rad"], integral_correction
            )),
        )
        maximum_target_delta = max(
            maximum_target_delta,
            *(abs(actual - expected) for actual, expected in zip(
                frame["joint_position_target_rad"], expected_target
            )),
        )
        if (
            frame["controller_observation_sequence"] != expected_sequence
            or frame["controller_observation_age_ticks"] != expected_age
            or frame["controller_bootstrap"] != expected_bootstrap
            or frame.get("sensor_status") != "nominal"
        ):
            timing_mismatches += 1
    return {
        "backend_id": trace["backend_id"],
        "law": law["kind"],
        "input": "typed_joint_feedback_only",
        "evaluated_frames": len(observations),
        "feedback_frames": max(0, len(observations) - 2),
        "bootstrap_frames": min(2, len(observations)),
        "timing_mismatch_count": timing_mismatches,
        "maximum_reference_roundtrip_delta_rad": maximum_reference_delta,
        "maximum_recomputed_correction_delta_rad": maximum_correction_delta,
        "maximum_recomputed_integral_delta_rad": maximum_integral_delta,
        "maximum_recomputed_target_delta_rad": maximum_target_delta,
    }


def main() -> int:
    args = parse_args()
    output = args.output.resolve()
    root = args.repo_root.resolve()
    rapier = load(output / "rapier-success-trace.json")
    mujoco = load(output / "mujoco-success-trace.json")
    gazebo = load(output / "gazebo-success-trace.json")
    rapier_failure = load(output / "intentional-failure.json")
    mujoco_failure = load(output / "mujoco-intentional-failure.json")
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
    actions = load(output / "controller-actions.json")

    backends = [rapier, mujoco, gazebo]
    failures = [rapier_failure, mujoco_failure, gazebo_failure]
    identities = [*backends, *failures]
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
    for value in (rapier, mujoco, rapier_failure, mujoco_failure):
        if value.get("actuation_config_sha256") != sha256(
            rapier_actuation_path
        ) or value.get("robot_asset_config_sha256") != sha256(rapier_robot_asset_path):
            raise ValueError(
                "native physics evidence is not bound to its model/actuation configuration"
            )
    for value in (rapier, mujoco):
        if value.get("physics_state_hash_contract") != PHYSICS_HASH_CONTRACT:
            raise ValueError(
                f"{value['backend_id']} does not use the articulated state hash contract"
            )
    for value in (gazebo, gazebo_failure):
        if value.get("runtime_manifest_sha256") != sha256(
            gazebo_runtime_path
        ) or value.get("adapter_config_sha256") != sha256(gazebo_adapter_config_path):
            raise ValueError(
                "Gazebo evidence is not bound to its runtime/configuration"
            )
    if len({len(backend["observations"]) for backend in backends}) != 1:
        raise ValueError("backend traces differ in length")
    controller_evidence = [
        verify_controller_execution(backend, controller, actions)
        for backend in backends
    ]

    transient_delta = 0.0
    transient_step = 0
    transient_joint = 0
    final_joint_deltas: list[float] = []
    plot: list[dict[str, Any]] = []
    for frames in zip(*(backend["observations"] for backend in backends)):
        if len({frame["step"] for frame in frames}) != 1 or len(
            {frame["sim_time_ticks"] for frame in frames}
        ) != 1:
            raise ValueError("backend trace step/time alignment drifted")
        deltas = [
            max(values) - min(values)
            for values in zip(*(frame["joint_position_rad"] for frame in frames))
        ]
        maximum = max(deltas)
        if maximum > transient_delta:
            transient_delta = maximum
            transient_step = frames[0]["step"]
            transient_joint = deltas.index(maximum)
        if frames[0]["step"] % 10 == 0 or frames[0]["step"] == 1:
            plot.append(
                {
                    "step": frames[0]["step"],
                    "rapier_error_rad": frames[0]["maximum_tracking_error_rad"],
                    "mujoco_error_rad": frames[1]["maximum_tracking_error_rad"],
                    "gazebo_error_rad": frames[2]["maximum_tracking_error_rad"],
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
            "mujoco_final_tracking_error_rad_v1",
            "rad",
            mujoco["final_maximum_tracking_error_rad"],
            FINAL_TRACKING_TOLERANCE_RAD,
        ),
        check(
            "gazebo_final_tracking_error_rad_v1",
            "rad",
            gazebo["final_maximum_tracking_error_rad"],
            FINAL_TRACKING_TOLERANCE_RAD,
        ),
        check(
            "maximum_final_cross_backend_joint_position_delta_rad_v1",
            "rad",
            max(final_joint_deltas),
            FINAL_CROSS_BACKEND_POSITION_TOLERANCE_RAD,
        ),
    ]
    state_hash_evidence = []
    for backend in (rapier, mujoco):
        unique_hashes = len(
            {frame["physics_hash"] for frame in backend["observations"]}
        )
        progressed = (
            unique_hashes > 1
            and backend["initial_state_digest"] != backend["final_state_digest"]
        )
        replay_digest_match = (
            backend["final_state_digest"] == backend["replay_final_state_digest"]
        )
        state_hash_evidence.append(
            {
                "backend_id": backend["backend_id"],
                "contract": PHYSICS_HASH_CONTRACT,
                "unique_step_hash_count": unique_hashes,
                "initial_final_digest_differ": backend["initial_state_digest"]
                != backend["final_state_digest"],
                "replay_final_digest_match": replay_digest_match,
            }
        )
        tolerance_checks.extend(
            [
                check(
                    f"{backend['backend_id']}_articulated_state_hash_progression_v1",
                    "boolean_failure",
                    0.0 if progressed else 1.0,
                    0.0,
                ),
                check(
                    f"{backend['backend_id']}_articulated_replay_digest_v1",
                    "boolean_failure",
                    0.0 if replay_digest_match else 1.0,
                    0.0,
                ),
            ]
        )
    for evidence in controller_evidence:
        backend_id = evidence["backend_id"]
        tolerance_checks.extend(
            [
                check(
                    f"{backend_id}_controller_timing_mismatch_v1",
                    "frame",
                    evidence["timing_mismatch_count"],
                    0.0,
                ),
                check(
                    f"{backend_id}_controller_reference_roundtrip_v1",
                    "rad",
                    evidence["maximum_reference_roundtrip_delta_rad"],
                    CONTROLLER_REPRODUCTION_TOLERANCE_RAD,
                ),
                check(
                    f"{backend_id}_controller_correction_reproduction_v1",
                    "rad",
                    evidence["maximum_recomputed_correction_delta_rad"],
                    CONTROLLER_REPRODUCTION_TOLERANCE_RAD,
                ),
                check(
                    f"{backend_id}_controller_integral_reproduction_v1",
                    "rad",
                    evidence["maximum_recomputed_integral_delta_rad"],
                    CONTROLLER_REPRODUCTION_TOLERANCE_RAD,
                ),
                check(
                    f"{backend_id}_controller_target_reproduction_v1",
                    "rad",
                    evidence["maximum_recomputed_target_delta_rad"],
                    CONTROLLER_REPRODUCTION_TOLERANCE_RAD,
                ),
            ]
        )
    failure_checks = [
        check(
            "maximum_first_violation_step_delta_v1",
            "step",
            max(failure["first_violation_step"] for failure in failures)
            - min(failure["first_violation_step"] for failure in failures),
            0.0,
        ),
        check(
            "maximum_first_violation_time_delta_v1",
            "ns",
            max(failure["first_violation_sim_time_ticks"] for failure in failures)
            - min(failure["first_violation_sim_time_ticks"] for failure in failures),
            0.0,
        ),
        check(
            "mujoco_rejected_step_state_advance_v1",
            "step",
            1.0 if mujoco_failure["rejected_step_changed_state"] else 0.0,
            0.0,
        ),
        check(
            "gazebo_rejected_step_state_advance_v1",
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
            "physics_state_hash_contract": backend.get(
                "physics_state_hash_contract", "adapter_defined_state_digest"
            ),
            "status": "passed"
            if backend["replay_match"]
            and backend["final_maximum_tracking_error_rad"]
            <= FINAL_TRACKING_TOLERANCE_RAD
            else "failed",
        }
        for backend in backends
    ]
    failures_match = (
        all(failure["status"] == "failed_as_expected" for failure in failures)
        and len({failure["first_violation"] for failure in failures}) == 1
        and failures[0]["first_violation"]
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
            "native_physics_actuation_config",
        ),
        artifact(
            root,
            "examples/90_showcase_captures/openarm_mujoco_trace.rs",
            "native_mujoco_runner",
        ),
        artifact(
            root,
            "crates/rne_physics_mujoco/src/compiler.rs",
            "native_mujoco_model_compiler",
        ),
        artifact(
            root,
            "crates/rne_physics_mujoco/src/backend.rs",
            "native_mujoco_backend",
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
        "comparison_contract": "same_task_controller_reference_and_artifact_defined_sensor_feedback_across_rapier_native_mujoco_and_gazebo_with_named_si_tolerances",
        "inputs": inputs,
        "backend_outcomes": backend_outcomes,
        "controller_execution_evidence": controller_evidence,
        "state_hash_evidence": state_hash_evidence,
        "tolerance_checks": tolerance_checks,
        "intentional_failures": failures,
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
    title = html.escape(
        f"{report['task_id']} — Rapier / native MuJoCo / Gazebo Failure Capsule"
    )
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
const f=r.intentional_failures[0];document.querySelector('#failure').innerHTML=`<b>${{esc(f.injection_kind)}}</b><p>first violation: ${{esc(f.first_violation)}} at step ${{f.first_violation_step}} (${{f.first_violation_sim_time_ticks}} ns)</p><p>Rapier, native MuJoCo, and Gazebo agree exactly; rejected MuJoCo and Gazebo steps did not advance state.</p>`;
document.querySelector('#inputs').innerHTML='<tr><th>role</th><th>file</th><th>SHA-256</th></tr>'+r.inputs.map(x=>`<tr><td>${{esc(x.role)}}</td><td>${{esc(x.file)}}</td><td><code>${{x.sha256}}</code></td></tr>`).join('');
const c=document.querySelector('#plot'),ctx=c.getContext('2d'),p=data.plot,max=Math.max(...p.flatMap(x=>[x.rapier_error_rad,x.mujoco_error_rad,x.gazebo_error_rad,x.position_delta_rad]));ctx.strokeStyle='#ccd6ee';ctx.strokeRect(45,15,1055,275);const draw=(key,color)=>{{ctx.beginPath();ctx.strokeStyle=color;ctx.lineWidth=2;p.forEach((x,i)=>{{const px=45+i/(p.length-1)*1055,py=290-x[key]/max*275;i?ctx.lineTo(px,py):ctx.moveTo(px,py)}});ctx.stroke()}};draw('rapier_error_rad','#316dff');draw('mujoco_error_rad','#9b59d0');draw('gazebo_error_rad','#ef7b45');draw('position_delta_rad','#24b58a');ctx.fillStyle='#17213b';ctx.fillText(`Rapier · MuJoCo · Gazebo · pairwise delta | 0 … ${{p[p.length-1].step}} steps · max ${{max.toFixed(3)}} rad`,48,322);
</script></main></body></html>"""
    path.write_text(document, encoding="utf-8")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm cross-sim report failed: {error}", file=sys.stderr)
        raise SystemExit(2)

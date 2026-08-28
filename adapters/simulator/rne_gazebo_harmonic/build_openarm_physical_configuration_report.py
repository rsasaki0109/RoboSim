#!/usr/bin/env python3
"""Build browser evidence for the official OpenArm end-effector configurations."""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
from typing import Any, Iterable


BACKENDS = ("rne_rapier", "mujoco_native", "gazebo_sim")
TRACE_FILES = {
    "rne_rapier": "rapier-success-trace.json",
    "mujoco_native": "mujoco-success-trace.json",
    "gazebo_sim": "gazebo-success-trace.json",
}
FAILURE_FILES = {
    "rne_rapier": "intentional-failure.json",
    "mujoco_native": "mujoco-intentional-failure.json",
    "gazebo_sim": "gazebo-intentional-failure.json",
}
QUALITY_GATES = {"source_integrity", "plant_integrity", "identification_validity", "portability"}


def parse_args() -> argparse.Namespace:
    repo = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--trace-root", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--repo-root", type=Path, default=repo)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=Path(__file__).resolve().parent
        / "openarm_physical_configuration_experiments.json",
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


def rms(values: Iterable[float]) -> float:
    values = list(values)
    if not values:
        raise ValueError("cannot calculate RMS of an empty sequence")
    return math.sqrt(sum(value * value for value in values) / len(values))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise ValueError(f"cannot load {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def requirement_map(manifest: dict[str, Any]) -> dict[str, dict[str, Any]]:
    requirements = manifest.get("requirements")
    if not isinstance(requirements, list):
        raise ValueError("physical-configuration manifest has no requirements")
    result = {item["id"]: item for item in requirements}
    if len(result) != len(requirements):
        raise ValueError("physical-configuration requirements contain duplicate ids")
    for item in requirements:
        if item.get("gate") not in QUALITY_GATES or sum(
            key in item for key in ("minimum", "maximum", "required")
        ) != 1:
            raise ValueError(f"invalid physical-configuration requirement {item.get('id')}")
    return result


def check(requirement: dict[str, Any], observed: float | bool, suffix: str = "") -> dict[str, Any]:
    result = {
        "id": requirement["id"] + suffix,
        "gate": requirement["gate"],
        "unit": requirement["unit"],
        "observed": observed,
    }
    if "maximum" in requirement:
        result["maximum"] = requirement["maximum"]
        passed = float(observed) <= float(requirement["maximum"])
    elif "minimum" in requirement:
        result["minimum"] = requirement["minimum"]
        passed = float(observed) >= float(requirement["minimum"])
    else:
        result["required"] = requirement["required"]
        passed = observed is requirement["required"]
    result["status"] = "passed" if passed else "failed"
    return result


def maximum_numeric_delta(left: Any, right: Any) -> float:
    if isinstance(left, bool) or isinstance(right, bool):
        return 0.0 if left is right else math.inf
    if isinstance(left, (int, float)) and isinstance(right, (int, float)):
        return abs(float(left) - float(right))
    if isinstance(left, dict) and isinstance(right, dict) and set(left) == set(right):
        return max((maximum_numeric_delta(left[key], right[key]) for key in left), default=0.0)
    if isinstance(left, list) and isinstance(right, list) and len(left) == len(right):
        return max((maximum_numeric_delta(a, b) for a, b in zip(left, right)), default=0.0)
    return 0.0 if left == right else math.inf


def validate_runtime(runtime_path: Path) -> dict[str, str]:
    runtime = load(runtime_path)
    if (
        runtime.get("kind") != "rne_external_simulator_runtime_manifest"
        or runtime.get("schema_version") != 1
        or runtime.get("simulator_id") != "gazebo_sim"
    ):
        raise ValueError(f"unsupported runtime manifest {runtime_path}")
    result = {}
    for artifact in runtime["artifacts"]:
        path = runtime_path.parent / artifact["file"]
        if path.stat().st_size != artifact["size_bytes"] or sha256(path) != artifact["sha256"]:
            raise ValueError(f"runtime artifact {path.name} differs from manifest")
        result[artifact["role"]] = artifact["sha256"]
    if set(result) != {"world", "robot_model", "adapter_config"}:
        raise ValueError("runtime artifact roles drifted")
    return result


def matrix_values(matrix: dict[str, Any]) -> list[float]:
    return [
        float(value)
        for column in matrix["columns"]
        for value in column["mean_output_gain_rad_per_rad"]
    ]


def write_json(path: Path, value: Any) -> None:
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def write_html(path: Path, report: dict[str, Any]) -> None:
    payload = json.dumps(report, separators=(",", ":"), allow_nan=False).replace("</", "<\\/")
    document = r'''<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>OpenArm physical configuration evidence</title><style>
body{margin:0;background:#09121f;color:#eef5ff;font:14px system-ui,sans-serif}main{max-width:1240px;margin:auto;padding:28px}.card{background:#132238;border:1px solid #2a4667;border-radius:10px;padding:14px;margin-bottom:16px}.passed{color:#6ee7aa}.failed{color:#ffb36b}table{width:100%;border-collapse:collapse;margin-bottom:24px}th,td{border:1px solid #2a4667;padding:7px;text-align:right}th:first-child,td:first-child{text-align:left}code{color:#b9ddff}</style></head><body><main><h1>OpenArm official configuration coupled-mode evidence</h1><div id="summary"></div><h2>Physical source and realization</h2><div id="models"></div><h2>Coupled response separation</h2><div id="responses"></div><h2>Fixed requirements</h2><div id="checks"></div><script>
const r=__REPORT__,f=x=>typeof x==='boolean'?String(x):Number(x).toPrecision(7);document.querySelector('#summary').innerHTML=`<section class=card><p>Status: <b class=${r.status}>${r.status}</b></p><p>Source: <code>${r.source.upstream_repository}@${r.source.upstream_commit}</code></p><p>Same seven-axis TaskSpec, controller, actuation, and actions; only the official arm-only / pinch-gripper product configuration changes.</p><p>First failed requirement: <code>${r.first_failed_requirement?r.first_failed_requirement.id:'none'}</code></p></section>`;document.querySelector('#models').innerHTML=`<table><tr><th>configuration</th><th>end effector</th><th>mass kg</th><th>inertial links</th><th>model realization delta</th></tr>${r.configurations.map(c=>`<tr><td>${c.configuration_id}</td><td>${c.primary_arm_end_effector_enabled?'pinch gripper':'none'}</td><td>${f(c.articulated_mass_kg)}</td><td>${c.inertial_link_count}</td><td>${f(c.model_realization_delta)}</td></tr>`).join('')}</table>`;document.querySelector('#responses').innerHTML=`<table><tr><th>backend</th><th>response RMS delta rad</th><th>peak delta rad</th><th>coupling-matrix Frobenius delta</th></tr>${r.backend_comparisons.map(c=>`<tr><td>${c.backend_id}</td><td>${f(c.coupled_response_rms_delta_rad)}</td><td>${f(c.coupled_response_peak_delta_rad)}</td><td>${f(c.coupling_matrix_frobenius_delta)}</td></tr>`).join('')}</table>`;document.querySelector('#checks').innerHTML=`<table><tr><th>requirement</th><th>gate</th><th>observed</th><th>limit</th><th>status</th></tr>${r.checks.map(q=>`<tr><td>${q.id}</td><td>${q.gate}</td><td>${f(q.observed)} ${q.unit}</td><td>${q.maximum!=null?'≤ '+f(q.maximum):q.minimum!=null?'≥ '+f(q.minimum):'= '+q.required} ${q.unit}</td><td class=${q.status}>${q.status}</td></tr>`).join('')}</table>`;</script></main></body></html>'''.replace("__REPORT__", payload)
    path.write_text(document, encoding="utf-8")


def build_report(trace_root: Path, output: Path, repo: Path, manifest_path: Path) -> dict[str, Any]:
    manifest = load(manifest_path)
    requirements = requirement_map(manifest)
    suite_path = trace_root / "physical-configuration-suite.json"
    suite = load(suite_path)
    suite_module = load_module(
        "rne_openarm_physical_configuration_suite",
        manifest_path.parent / "build_openarm_physical_configuration_suite.py",
    )
    identification = load_module(
        "rne_openarm_multijoint_math",
        manifest_path.parent / "build_openarm_multijoint_identification_report.py",
    )
    identification_manifest = load(
        manifest_path.parent / "openarm_multijoint_identification_experiments.json"
    )
    plant_math = load_module(
        "rne_openarm_plant_math", manifest_path.parent / "build_openarm_plant_report.py"
    )
    controller_path = trace_root / suite_module.CONTROLLER_FILE
    task_path = trace_root / suite_module.TASK_FILE
    actuation_path = trace_root / suite_module.ACTUATION_FILE
    controller = load(controller_path)
    order = controller["action_joint_order"]
    if (
        suite.get("kind") != "rne_openarm_physical_configuration_suite"
        or suite.get("experiment_id") != manifest["experiment_id"]
        or suite.get("configuration_order") != manifest["configuration_order"]
        or controller.get("controller_id") != manifest["controller_id"]
        or controller.get("task_id") != manifest["task_id"]
        or len(order) != manifest["controlled_joint_count"]
        or suite["inputs"]["manifest_sha256"] != sha256(manifest_path)
        or suite["inputs"]["controller_sha256"] != sha256(controller_path)
        or suite["inputs"]["task_spec_sha256"] != sha256(task_path)
        or suite["inputs"]["actuation_config_sha256"] != sha256(actuation_path)
    ):
        raise ValueError("physical-configuration suite or shared contract drifted")

    source_mismatches = 0
    for configuration in manifest["configurations"]:
        preset = repo / configuration["vendored_preset"]
        source_mismatches += int(not preset.is_file() or sha256(preset) != configuration["preset_sha256"])
    checks = [
        check(
            requirements["physical_configuration.maximum_source_hash_mismatch_count"],
            source_mismatches,
        )
    ]
    configuration_reports = []
    traces: dict[str, dict[str, dict[str, Any]]] = {}
    actions_by_configuration: dict[str, tuple[Path, dict[str, Any]]] = {}
    limits_by_configuration = {}
    for case in suite["cases"]:
        identifier = case["case_id"]
        case_dir = trace_root / identifier
        fixture_path = case_dir / "physical-configuration-fixture.json"
        fixture = load(fixture_path)
        model_path = case_dir / suite_module.MODEL_FILE
        robot_path = case_dir / suite_module.ROBOT_FILE
        scene_path = case_dir / suite_module.SCENE_FILE
        case_actuation_path = case_dir / suite_module.ACTUATION_FILE
        adapter_path = case_dir / suite_module.ADAPTER_FILE
        runtime_path = case_dir / "runtime.json"
        runtime_hashes = validate_runtime(runtime_path)
        realized_inertials = suite_module.model_inertials(model_path.read_bytes())
        model_delta = maximum_numeric_delta(fixture["inertials"], realized_inertials)
        mass_delta = abs(
            sum(item["mass_kg"] for item in realized_inertials)
            - fixture["expected_articulated_mass_kg"]
        )
        if (
            fixture.get("kind") != "rne_openarm_physical_configuration_fixture"
            or fixture.get("case_id") != identifier
            or fixture.get("model_urdf_sha256") != sha256(model_path)
            or fixture.get("robot_asset_config_sha256") != sha256(robot_path)
            or fixture.get("scene_config_sha256") != sha256(scene_path)
            or fixture.get("actuation_config_sha256") != sha256(case_actuation_path)
            or fixture.get("gazebo_adapter_config_sha256") != sha256(adapter_path)
            or runtime_hashes["robot_model"] != sha256(model_path)
            or runtime_hashes["world"] != sha256(case_dir / suite_module.WORLD_FILE)
            or runtime_hashes["adapter_config"] != sha256(adapter_path)
        ):
            raise ValueError(f"physical model fixture drifted for {identifier}")
        checks.extend(
            [
                check(
                    requirements["physical_configuration.maximum_model_realization_delta"],
                    model_delta,
                    f".{identifier}",
                ),
                check(
                    requirements["physical_configuration.maximum_mass_delta_kg"],
                    mass_delta,
                    f".{identifier}",
                ),
            ]
        )
        configuration_reports.append(
            {
                "configuration_id": identifier,
                "upstream_preset": fixture["upstream_preset"],
                "vendored_preset_sha256": fixture["vendored_preset_sha256"],
                "primary_arm_end_effector_enabled": fixture[
                    "primary_arm_end_effector_enabled"
                ],
                "model_urdf_sha256": sha256(model_path),
                "articulated_mass_kg": sum(item["mass_kg"] for item in realized_inertials),
                "inertial_link_count": len(realized_inertials),
                "model_realization_delta": model_delta,
                "mass_realization_delta_kg": mass_delta,
                "positive_definite_inertial_links": [item["link"] for item in realized_inertials],
            }
        )
        actions_path = case_dir / "controller-actions.json"
        actions = load(actions_path)
        if (
            actions.get("controller_sha256") != sha256(controller_path)
            or actions.get("task_sha256") != sha256(task_path)
            or actions.get("action_joint_order") != order
            or len(actions.get("actions", [])) != controller["keyframes"][-1]["step"]
        ):
            raise ValueError(f"shared controller actions drifted for {identifier}")
        actions_by_configuration[identifier] = (actions_path, actions)
        limits_by_configuration[identifier] = plant_math.joint_limits(model_path, order)
        traces[identifier] = {}
        for backend in BACKENDS:
            trace_path = case_dir / TRACE_FILES[backend]
            failure_path = case_dir / FAILURE_FILES[backend]
            trace = load(trace_path)
            failure = load(failure_path)
            observations = trace.get("observations", [])
            if (
                trace.get("backend_id") != backend
                or trace.get("controller_sha256") != sha256(controller_path)
                or trace.get("task_sha256") != sha256(task_path)
                or trace.get("action_trace_sha256") != sha256(actions_path)
                or trace.get("controller_execution") != "open_loop_reference"
                or len(observations) != len(actions["actions"])
                or failure.get("status") != "failed_as_expected"
                or failure.get("first_violation") != "action_width_mismatch"
            ):
                raise ValueError(f"{identifier}/{backend} trace identity drifted")
            if backend == "gazebo_sim":
                diagnostics_path = case_dir / "gazebo-actuation-diagnostics-a.json"
                replay_diagnostics_path = (
                    case_dir / "gazebo-actuation-diagnostics-b.json"
                )
                identity_ok = (
                    trace.get("runtime_manifest_sha256") == sha256(runtime_path)
                    and trace.get("adapter_config_sha256") == sha256(adapter_path)
                    and trace.get("robot_model_sha256") == sha256(model_path)
                    and trace.get("world_sha256") == sha256(case_dir / suite_module.WORLD_FILE)
                    and trace.get("actuation_diagnostics_sha256")
                    == sha256(diagnostics_path)
                    and trace.get("replay_actuation_diagnostics_sha256")
                    == sha256(replay_diagnostics_path)
                    and sha256(diagnostics_path) == sha256(replay_diagnostics_path)
                )
            else:
                identity_ok = (
                    trace.get("model_urdf_sha256") == sha256(model_path)
                    and trace.get("robot_asset_config_sha256") == sha256(robot_path)
                    and trace.get("scene_config_sha256") == sha256(scene_path)
                    and trace.get("actuation_config_sha256") == sha256(case_actuation_path)
                )
            if not identity_ok:
                raise ValueError(f"{identifier}/{backend} model identity drifted")
            for step, observation in enumerate(observations, 1):
                if (
                    observation.get("step") != step
                    or observation.get("sim_time_ticks") != step * 16_666_667
                    or observation.get("sensor_status") != "nominal"
                    or len(observation.get("joint_position_rad", [])) != len(order)
                    or len(observation.get("joint_velocity_rad_s", [])) != len(order)
                ):
                    raise ValueError(f"{identifier}/{backend} observation drifted at step {step}")
            replay_exact = bool(trace.get("replay_match")) and trace.get(
                "final_state_digest"
            ) == trace.get("replay_final_state_digest")
            checks.append(
                check(
                    requirements["physical_configuration.requires_exact_replay"],
                    replay_exact,
                    f".{identifier}.{backend}",
                )
            )
            expected_failure_step = controller["intentional_failure"]["inject_at_step"]
            checks.append(
                check(
                    requirements[
                        "physical_configuration.maximum_intentional_failure_step_delta"
                    ],
                    abs(failure["first_violation_step"] - expected_failure_step),
                    f".{identifier}.{backend}",
                )
            )
            violations = 0
            for observation in observations:
                for joint_index, limit in enumerate(limits_by_configuration[identifier]):
                    if (
                        observation["joint_position_rad"][joint_index]
                        < limit["minimum_position_rad"] - 1e-12
                        or observation["joint_position_rad"][joint_index]
                        > limit["maximum_position_rad"] + 1e-12
                        or abs(observation["joint_velocity_rad_s"][joint_index])
                        > limit["maximum_velocity_rad_s"] + 1e-12
                    ):
                        violations += 1
            checks.append(
                check(
                    requirements[
                        "physical_configuration.maximum_hard_limit_violation_count"
                    ],
                    violations,
                    f".{identifier}.{backend}",
                )
            )
            traces[identifier][backend] = {
                "trace": trace,
                "trace_path": trace_path,
                "failure_path": failure_path,
                "replay_exact": replay_exact,
                "hard_limit_violation_count": violations,
            }

    action_hashes = {sha256(path) for path, _ in actions_by_configuration.values()}
    if len(action_hashes) != 1:
        raise ValueError("official configurations did not consume byte-identical actions")
    gripper_mass_delta = (
        configuration_reports[1]["articulated_mass_kg"]
        - configuration_reports[0]["articulated_mass_kg"]
    )
    checks.append(
        check(
            requirements["physical_configuration.minimum_gripper_mass_delta_kg"],
            gripper_mass_delta,
        )
    )
    validation = manifest["validation_segment"]
    start = validation["start_step"] - 1
    end = validation["end_step"]
    operating = identification_manifest["operating_point_rad"][: len(order)]
    backend_comparisons = []
    for backend in BACKENDS:
        arm_observations = traces["arm_only"][backend]["trace"]["observations"]
        gripper_observations = traces["pinch_gripper"][backend]["trace"]["observations"]
        deltas = [
            gripper_observations[sample]["joint_position_rad"][joint]
            - arm_observations[sample]["joint_position_rad"][joint]
            for sample in range(start, end)
            for joint in range(len(order))
        ]
        response_rms = rms(deltas)
        response_peak = max(abs(value) for value in deltas)
        matrices = {}
        for identifier in manifest["configuration_order"]:
            _, actions = actions_by_configuration[identifier]
            observations = traces[identifier][backend]["trace"]["observations"]
            window = {
                "start_step": validation["start_step"],
                "end_step": validation["end_step"],
            }
            inputs, outputs, _ = identification.window_values(
                actions["actions"], observations, window, list(range(len(order))), operating
            )
            matrices[identifier] = identification.coupling_matrix(
                identification_manifest, inputs, outputs
            )
        left = matrix_values(matrices["arm_only"])
        right = matrix_values(matrices["pinch_gripper"])
        matrix_delta = math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))
        response_check = check(
            requirements[
                "physical_configuration.minimum_coupled_response_rms_delta_rad"
            ],
            response_rms,
            f".{backend}",
        )
        matrix_check = check(
            requirements[
                "physical_configuration.minimum_coupling_matrix_frobenius_delta"
            ],
            matrix_delta,
            f".{backend}",
        )
        checks.extend((response_check, matrix_check))
        backend_comparisons.append(
            {
                "backend_id": backend,
                "coupled_response_rms_delta_rad": response_rms,
                "coupled_response_peak_delta_rad": response_peak,
                "per_joint_response_rms_delta_rad": {
                    joint: rms(
                        gripper_observations[sample]["joint_position_rad"][index]
                        - arm_observations[sample]["joint_position_rad"][index]
                        for sample in range(start, end)
                    )
                    for index, joint in enumerate(order)
                },
                "coupling_matrix_frobenius_delta": matrix_delta,
                "coupling_matrices": matrices,
                "checks": [response_check, matrix_check],
            }
        )
    first_failed = next((item for item in checks if item["status"] == "failed"), None)
    report = {
        "kind": "rne_openarm_physical_configuration_coupled_mode_report",
        "schema_version": 1,
        "status": "passed" if first_failed is None else "needs_tuning",
        "experiment_id": manifest["experiment_id"],
        "source": {
            "upstream_repository": manifest["upstream_repository"],
            "upstream_commit": manifest["upstream_commit"],
            "source_model": manifest["source_model"],
            "configuration_presets": [
                {
                    "configuration_id": item["id"],
                    "upstream_path": item["upstream_preset"],
                    "sha256": item["preset_sha256"],
                }
                for item in manifest["configurations"]
            ],
        },
        "contract": {
            "task_id": manifest["task_id"],
            "controller_id": manifest["controller_id"],
            "controlled_joint_order": order,
            "fixed_delta_ticks": 16_666_667,
            "validation_segment": validation,
            "same_task_controller_actuation_and_actions": True,
            "comparison_fields": ["joint_position_rad", "joint_velocity_rad_s"],
            "solver_private_state_used": False,
        },
        "inputs": {
            "manifest_sha256": sha256(manifest_path),
            "suite_sha256": sha256(suite_path),
            "controller_sha256": sha256(controller_path),
            "task_spec_sha256": sha256(task_path),
            "actuation_config_sha256": sha256(actuation_path),
            "action_trace_sha256": next(iter(action_hashes)),
        },
        "configurations": configuration_reports,
        "gripper_mass_delta_kg": gripper_mass_delta,
        "backend_comparisons": backend_comparisons,
        "trace_evidence": [
            {
                "configuration_id": identifier,
                "backend_id": backend,
                "trace_sha256": sha256(traces[identifier][backend]["trace_path"]),
                "intentional_failure_sha256": sha256(
                    traces[identifier][backend]["failure_path"]
                ),
                "replay_exact": traces[identifier][backend]["replay_exact"],
                "hard_limit_violation_count": traces[identifier][backend][
                    "hard_limit_violation_count"
                ],
            }
            for identifier in manifest["configuration_order"]
            for backend in BACKENDS
        ],
        "first_failed_requirement": first_failed,
        "checks": checks,
    }
    output.mkdir(parents=True, exist_ok=True)
    write_json(output / "openarm-physical-configuration-report.json", report)
    write_html(output / "openarm-physical-configuration-report.html", report)
    return report


def main() -> int:
    args = parse_args()
    report = build_report(
        args.trace_root.resolve(),
        args.output.resolve(),
        args.repo_root.resolve(),
        args.manifest.resolve(),
    )
    print(
        f"OpenArm physical configuration: status={report['status']} "
        f"backends={len(report['backend_comparisons'])}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

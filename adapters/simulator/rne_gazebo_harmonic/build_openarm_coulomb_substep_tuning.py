#!/usr/bin/env python3
"""Compile fixed-plant OpenArm physics-substep tuning fixtures."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import shutil
from typing import Any

from build_openarm_coulomb_friction_suite import load, sha256, write_json


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--source-case", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root / "openarm_coulomb_substep_tuning.json",
    )
    return parser.parse_args()


def validate_manifest(manifest: dict[str, Any]) -> None:
    grid = manifest.get("physics_substeps_per_control_step_grid")
    if (
        manifest.get("kind") != "rne_openarm_coulomb_substep_tuning_manifest"
        or manifest.get("schema_version") != 1
        or manifest.get("tuning_backend_id") != "rne_rapier"
        or not isinstance(grid, list)
        or len(grid) < 3
        or grid != sorted(set(grid))
        or grid[0] != 1
        or any(not isinstance(value, int) or value <= 0 for value in grid)
        or manifest.get("control_period_ticks", 0) < grid[-1]
        or manifest.get("selection_rule")
        != "smallest_physics_substep_count_passing_all_predeclared_requirements"
    ):
        raise ValueError("unsupported Coulomb substep tuning manifest")


def compile_tuning(
    manifest_path: Path, source_case: Path, output: Path
) -> dict[str, Any]:
    manifest = load(manifest_path)
    source_fixture = load(source_case / "coulomb-friction-fixture.json")
    source_actuation = load(source_case / "openarm_right.rne_actuation.json")
    validate_manifest(manifest)
    if (
        source_fixture.get("controlled_joint") != manifest["controlled_joint"]
        or source_fixture.get("plant_viscous_damping_nm_s_per_rad")
        != manifest["plant_viscous_damping_nm_s_per_rad"]
        or source_fixture.get("plant_coulomb_friction_nm")
        != manifest["plant_coulomb_friction_nm"]
        or source_fixture.get("plant_coulomb_transition_velocity_rad_s")
        != manifest["plant_coulomb_transition_velocity_rad_s"]
        or source_actuation.get("solver_iterations") != manifest["solver_iterations"]
        or source_actuation.get("fixed_delta_ticks") != manifest["control_period_ticks"]
    ):
        raise ValueError("source case differs from substep tuning manifest")

    candidates = []
    for substeps in manifest["physics_substeps_per_control_step_grid"]:
        identifier = f"substeps-{substeps:02d}"
        case_root = output / identifier
        shutil.copytree(source_case, case_root, dirs_exist_ok=True)
        actuation_path = case_root / "openarm_right.rne_actuation.json"
        actuation = json.loads(json.dumps(source_actuation))
        actuation["physics_substeps_per_control_step"] = substeps
        write_json(actuation_path, actuation)
        fixture = json.loads(json.dumps(source_fixture))
        fixture["case_id"] = identifier
        fixture["source_coulomb_case_id"] = source_fixture["case_id"]
        fixture["physics_substeps_per_control_step"] = substeps
        fixture["control_period_ticks"] = manifest["control_period_ticks"]
        fixture["exact_substep_tick_partition"] = [
            manifest["control_period_ticks"] // substeps
            + int(index < manifest["control_period_ticks"] % substeps)
            for index in range(substeps)
        ]
        fixture["actuation_config_sha256"] = sha256(actuation_path)
        fixture_path = case_root / "coulomb-substep-fixture.json"
        write_json(fixture_path, fixture)
        candidates.append(
            {
                "candidate_id": identifier,
                "physics_substeps_per_control_step": substeps,
                "control_period_ticks": manifest["control_period_ticks"],
                "exact_substep_tick_partition": fixture[
                    "exact_substep_tick_partition"
                ],
                "fixture_sha256": sha256(fixture_path),
                "actuation_config_sha256": fixture["actuation_config_sha256"],
                "robot_asset_config_sha256": fixture[
                    "robot_asset_config_sha256"
                ],
                "portable_model_urdf_sha256": fixture[
                    "portable_model_urdf_sha256"
                ],
                "scene_config_sha256": fixture["scene_config_sha256"],
            }
        )
    suite = {
        "kind": "rne_openarm_coulomb_substep_tuning_suite",
        "schema_version": 1,
        **{
            key: manifest[key]
            for key in (
                "experiment_id",
                "controlled_joint",
                "tuning_backend_id",
                "control_period_ticks",
                "solver_iterations",
                "plant_viscous_damping_nm_s_per_rad",
                "plant_coulomb_friction_nm",
                "plant_coulomb_transition_velocity_rad_s",
                "maximum_controlled_joint_rmse_rad",
                "maximum_controlled_joint_final_error_rad",
                "requires_exact_control_period_ticks",
                "requires_exact_replay",
                "selection_rule",
                "validation_rule",
            )
        },
        "inputs": {
            "manifest_sha256": sha256(manifest_path),
            "source_fixture_sha256": sha256(
                source_case / "coulomb-friction-fixture.json"
            ),
            "source_actuation_config_sha256": sha256(
                source_case / "openarm_right.rne_actuation.json"
            ),
        },
        "candidates": candidates,
    }
    write_json(output / "coulomb-substep-tuning-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_tuning(
        args.manifest.resolve(), args.source_case.resolve(), args.output.resolve()
    )
    print(f"OpenArm Coulomb substep tuning: {len(suite['candidates'])} candidates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

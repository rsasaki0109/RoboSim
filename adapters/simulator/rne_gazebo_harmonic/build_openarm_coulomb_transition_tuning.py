#!/usr/bin/env python3
"""Compile the predeclared OpenArm Coulomb transition-width tuning fixtures."""

from __future__ import annotations

import argparse
import copy
import json
import math
from pathlib import Path
from typing import Any

from build_openarm_coulomb_friction_suite import compile_suite, load, sha256, write_json


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parent
    parser = argparse.ArgumentParser()
    parser.add_argument("--baseline-fixture", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--tuning-manifest",
        type=Path,
        default=root / "openarm_coulomb_transition_tuning.json",
    )
    parser.add_argument(
        "--base-experiment",
        type=Path,
        default=root / "openarm_coulomb_friction_experiments.json",
    )
    parser.add_argument(
        "--actuation-config",
        type=Path,
        default=root / "openarm_right.rne_actuation.json",
    )
    return parser.parse_args()


def validate_manifest(manifest: dict[str, Any]) -> None:
    grid = manifest.get("transition_velocity_grid_rad_s")
    reference = manifest.get("kinetic_reference_velocity_rad_s")
    minimum_fraction = manifest.get("minimum_kinetic_fraction_at_reference_velocity")
    if (
        manifest.get("kind") != "rne_openarm_coulomb_transition_tuning_manifest"
        or manifest.get("schema_version") != 1
        or manifest.get("tuning_backend_id") != "rne_rapier"
        or not isinstance(grid, list)
        or len(grid) < 3
        or grid != sorted(set(grid))
        or any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0.0
            for value in grid
        )
        or not isinstance(reference, (int, float))
        or not math.isfinite(reference)
        or reference <= 0.0
        or not isinstance(minimum_fraction, (int, float))
        or not 0.0 < minimum_fraction < 1.0
        or any(math.tanh(reference / value) < minimum_fraction for value in grid)
        or manifest.get("selection_rule")
        != "largest_transition_velocity_passing_all_predeclared_requirements"
    ):
        raise ValueError("unsupported Coulomb transition tuning manifest")


def candidate_id(transition_rad_s: float) -> str:
    micro_rad_s = round(transition_rad_s * 1_000_000.0)
    return f"transition-{micro_rad_s:06d}urad-s"


def compile_tuning(
    tuning_manifest_path: Path,
    base_experiment_path: Path,
    baseline_fixture: Path,
    actuation_config_path: Path,
    output: Path,
) -> dict[str, Any]:
    tuning = load(tuning_manifest_path)
    base = load(base_experiment_path)
    validate_manifest(tuning)
    if (
        base.get("controlled_joint") != tuning["controlled_joint"]
        or base.get("viscous_damping_nm_s_per_rad")
        != tuning["plant_viscous_damping_nm_s_per_rad"]
        or tuning["plant_coulomb_friction_nm"]
        not in base.get("coulomb_friction_grid_nm", [])
    ):
        raise ValueError("base Coulomb experiment differs from transition tuning manifest")

    candidates = []
    for raw_transition in tuning["transition_velocity_grid_rad_s"]:
        transition = float(raw_transition)
        identifier = candidate_id(transition)
        candidate_root = output / identifier
        candidate_manifest_path = candidate_root / "coulomb-friction-experiment.json"
        candidate_manifest = copy.deepcopy(base)
        candidate_manifest["experiment_id"] = f"{tuning['experiment_id']}.{identifier}"
        candidate_manifest["coulomb_transition_velocity_rad_s"] = transition
        write_json(candidate_manifest_path, candidate_manifest)
        fixture_root = candidate_root / "fixtures"
        suite = compile_suite(
            candidate_manifest_path,
            baseline_fixture,
            actuation_config_path,
            fixture_root,
        )
        matching = [
            case
            for case in suite["cases"]
            if case["plant_coulomb_friction_nm"] == tuning["plant_coulomb_friction_nm"]
        ]
        if len(matching) != 1:
            raise ValueError(f"{identifier} has no unique tuning-friction fixture")
        case = matching[0]
        candidates.append(
            {
                "candidate_id": identifier,
                "transition_velocity_rad_s": transition,
                "kinetic_fraction_at_reference_velocity": math.tanh(
                    tuning["kinetic_reference_velocity_rad_s"] / transition
                ),
                "case_id": case["case_id"],
                "candidate_manifest_sha256": sha256(candidate_manifest_path),
                "suite_sha256": sha256(fixture_root / "coulomb-friction-suite.json"),
                "fixture_sha256": sha256(
                    fixture_root / case["case_id"] / "coulomb-friction-fixture.json"
                ),
                "robot_asset_config_sha256": case["robot_asset_config_sha256"],
                "portable_model_urdf_sha256": case["portable_model_urdf_sha256"],
            }
        )

    suite = {
        "kind": "rne_openarm_coulomb_transition_tuning_suite",
        "schema_version": 1,
        "experiment_id": tuning["experiment_id"],
        "controlled_joint": tuning["controlled_joint"],
        "tuning_backend_id": tuning["tuning_backend_id"],
        "plant_viscous_damping_nm_s_per_rad": tuning[
            "plant_viscous_damping_nm_s_per_rad"
        ],
        "plant_coulomb_friction_nm": tuning["plant_coulomb_friction_nm"],
        "requirements": {
            "kinetic_reference_velocity_rad_s": tuning[
                "kinetic_reference_velocity_rad_s"
            ],
            "minimum_kinetic_fraction_at_reference_velocity": tuning[
                "minimum_kinetic_fraction_at_reference_velocity"
            ],
            "maximum_controlled_joint_rmse_rad": tuning[
                "maximum_controlled_joint_rmse_rad"
            ],
            "maximum_controlled_joint_final_error_rad": tuning[
                "maximum_controlled_joint_final_error_rad"
            ],
            "requires_exact_replay": tuning["requires_exact_replay"],
        },
        "selection_rule": tuning["selection_rule"],
        "validation_rule": tuning["validation_rule"],
        "inputs": {
            "tuning_manifest_sha256": sha256(tuning_manifest_path),
            "base_experiment_sha256": sha256(base_experiment_path),
            "baseline_runtime_manifest_sha256": sha256(
                baseline_fixture / "runtime.json"
            ),
            "actuation_config_sha256": sha256(actuation_config_path),
        },
        "candidates": candidates,
    }
    write_json(output / "coulomb-transition-tuning-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_tuning(
        args.tuning_manifest.resolve(),
        args.base_experiment.resolve(),
        args.baseline_fixture.resolve(),
        args.actuation_config.resolve(),
        args.output.resolve(),
    )
    print(f"OpenArm Coulomb transition tuning: {len(suite['candidates'])} candidates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

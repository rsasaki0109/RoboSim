#!/usr/bin/env python3
"""Compile predeclared OpenArm Coulomb controller pole-placement candidates."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any

SCRIPT_DIR = Path(__file__).resolve().parent
sys.path.insert(0, str(SCRIPT_DIR))
from build_openarm_controller_suite import pole_placement  # noqa: E402
from build_openarm_coulomb_friction_suite import load, sha256, write_json  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-controller", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=SCRIPT_DIR / "openarm_coulomb_controller_pole_tuning.json",
    )
    return parser.parse_args()


def validate_manifest(manifest: dict[str, Any]) -> None:
    candidates = manifest.get("candidates")
    identifiers = [item.get("candidate_id") for item in candidates or []]
    if (
        manifest.get("kind")
        != "rne_openarm_coulomb_controller_pole_tuning_manifest"
        or manifest.get("schema_version") != 1
        or not isinstance(candidates, list)
        or len(candidates) < 3
        or len(set(identifiers)) != len(candidates)
        or any(
            not isinstance(item.get("desired_closed_loop_poles"), list)
            or len(item["desired_closed_loop_poles"]) != 4
            or item["desired_closed_loop_poles"]
            != sorted(set(item["desired_closed_loop_poles"]))
            or any(
                not isinstance(pole, (int, float))
                or not math.isfinite(pole)
                or not 0.0 < pole < 1.0
                for pole in item["desired_closed_loop_poles"]
            )
            for item in candidates
        )
        or manifest.get("selection_rule")
        != "minimum_rmse_among_passing_candidates_then_smallest_maximum_pole"
    ):
        raise ValueError("unsupported Coulomb controller pole tuning manifest")


def compile_candidates(
    manifest_path: Path, base_controller_path: Path, output: Path
) -> dict[str, Any]:
    manifest = load(manifest_path)
    base = load(base_controller_path)
    validate_manifest(manifest)
    law = base.get("feedback_law")
    if (
        base.get("kind") != "rne_joint_pose_cycle_controller"
        or base.get("schema_version") != 1
        or not isinstance(law, dict)
        or law.get("kind") != "joint_position_state_feedback_integral_v1"
        or law.get("controlled_joint") != manifest["controlled_joint"]
    ):
        raise ValueError("base controller is not the supported state-feedback controller")
    plant = law["identified_plant"]
    augmented_a = plant["augmented_a"]
    augmented_b = [[value] for value in plant["augmented_b"]]
    maximum_integral_correction = law["maximum_state_integral_correction_rad"]
    output.mkdir(parents=True, exist_ok=True)
    candidates = []
    for spec in manifest["candidates"]:
        poles = [float(value) for value in spec["desired_closed_loop_poles"]]
        gain, controllability_determinant, closed_loop = pole_placement(
            augmented_a, augmented_b, poles
        )
        integral_gain = -gain[3]
        if integral_gain <= 0.0:
            raise ValueError(f"{spec['candidate_id']} integral gain has the wrong sign")
        controller = json.loads(json.dumps(base))
        controller["controller_id"] = (
            "rne.controller.openarm_right.coulomb_poles_"
            f"{spec['candidate_id']}.v1"
        )
        tuned = controller["feedback_law"]
        tuned["state_feedback_gain"] = gain[:3]
        tuned["integral_state_feedback_gain_s_inv"] = integral_gain
        tuned["desired_closed_loop_poles"] = poles
        tuned["closed_loop_a"] = closed_loop
        tuned["maximum_integral_state_error_rad_s"] = (
            maximum_integral_correction / integral_gain
        )
        tuned["identified_plant"][
            "controllability_determinant"
        ] = controllability_determinant
        filename = f"openarm-coulomb-poles-{spec['candidate_id']}.controller.json"
        path = output / filename
        write_json(path, controller)
        candidates.append(
            {
                "candidate_id": spec["candidate_id"],
                "desired_closed_loop_poles": poles,
                "maximum_pole": max(poles),
                "state_feedback_gain": gain[:3],
                "integral_state_feedback_gain_s_inv": integral_gain,
                "controller_id": controller["controller_id"],
                "file": filename,
                "sha256": sha256(path),
            }
        )
    suite = {
        "kind": "rne_openarm_coulomb_controller_pole_tuning_suite",
        "schema_version": 1,
        **{
            key: manifest[key]
            for key in (
                "tuning_id",
                "controlled_joint",
                "tuning_backend_id",
                "plant_coulomb_friction_nm",
                "plant_coulomb_transition_velocity_rad_s",
                "maximum_controlled_joint_rmse_rad",
                "maximum_controlled_joint_final_error_rad",
                "requires_exact_replay",
                "selection_rule",
                "validation_rule",
            )
        },
        "manifest_sha256": sha256(manifest_path),
        "base_controller_sha256": sha256(base_controller_path),
        "candidates": candidates,
    }
    write_json(output / "openarm-coulomb-controller-pole-tuning-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_candidates(
        args.manifest.resolve(), args.base_controller.resolve(), args.output.resolve()
    )
    print(f"OpenArm Coulomb controller pole tuning: {len(suite['candidates'])} candidates")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

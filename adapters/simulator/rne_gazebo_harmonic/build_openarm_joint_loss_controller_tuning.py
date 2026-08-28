#!/usr/bin/env python3
"""Compile predeclared OpenArm joint-loss controller tuning candidates."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any


KIND = "rne_openarm_joint_loss_controller_tuning_manifest"
CONTROLLER_KIND = "rne_joint_pose_cycle_controller"
LAW_KIND = "joint_position_state_feedback_integral_v1"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_joint_loss_controller_tuning.json",
    )
    parser.add_argument(
        "--base-controller",
        type=Path,
        default=root
        / "docs/evidence/openarm-controller-lab/evidence/openarm-plant-state-feedback.controller.json",
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


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def candidate_name(maximum_correction_rad: float) -> str:
    milli_rad = round(maximum_correction_rad * 1000.0)
    return f"openarm-joint-loss-state-feedback-{milli_rad:03d}mrad.controller.json"


def validate_manifest(manifest: dict[str, Any]) -> list[float]:
    grid = manifest.get("maximum_state_feedback_correction_grid_rad")
    requirements = manifest.get("requirements")
    if (
        manifest.get("kind") != KIND
        or manifest.get("schema_version") != 1
        or not isinstance(grid, list)
        or len(grid) < 2
        or grid != sorted(set(grid))
        or any(
            not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0.0
            for value in grid
        )
        or not isinstance(requirements, list)
        or {item.get("id") for item in requirements if isinstance(item, dict)}
        != {
            "joint_loss.maximum_controlled_joint_rmse_rad",
            "joint_loss.maximum_controlled_joint_final_error_rad",
        }
        or manifest.get("selection_rule")
        != "minimum_rmse_among_passing_candidates_then_smallest_correction_limit"
    ):
        raise ValueError("unsupported joint-loss controller tuning manifest")
    return [float(value) for value in grid]


def compile_candidates(
    manifest_path: Path, base_controller_path: Path, output: Path
) -> dict[str, Any]:
    manifest = load(manifest_path)
    grid = validate_manifest(manifest)
    base = load(base_controller_path)
    law = base.get("feedback_law")
    if (
        base.get("kind") != CONTROLLER_KIND
        or base.get("schema_version") != 1
        or not isinstance(law, dict)
        or law.get("kind") != LAW_KIND
        or law.get("controlled_joint") != manifest.get("controlled_joint")
        or not isinstance(law.get("maximum_state_feedback_correction_rad"), (int, float))
    ):
        raise ValueError("base controller is not the supported state-feedback controller")

    output.mkdir(parents=True, exist_ok=True)
    candidates = []
    for maximum_correction_rad in grid:
        controller = json.loads(json.dumps(base))
        milli_rad = round(maximum_correction_rad * 1000.0)
        controller["controller_id"] = (
            "rne.controller.openarm_right.joint_loss_state_feedback_"
            f"{milli_rad:03d}mrad.v1"
        )
        controller["feedback_law"][
            "maximum_state_feedback_correction_rad"
        ] = maximum_correction_rad
        path = output / candidate_name(maximum_correction_rad)
        write_json(path, controller)
        candidates.append(
            {
                "maximum_state_feedback_correction_rad": maximum_correction_rad,
                "controller_id": controller["controller_id"],
                "file": path.name,
                "sha256": sha256(path),
            }
        )

    suite = {
        "kind": "rne_openarm_joint_loss_controller_tuning_suite",
        "schema_version": 1,
        "tuning_id": manifest["tuning_id"],
        "controlled_joint": manifest["controlled_joint"],
        "tuning_backend_id": manifest["tuning_backend_id"],
        "tuning_case_id": manifest["tuning_case_id"],
        "requirements": manifest["requirements"],
        "selection_rule": manifest["selection_rule"],
        "validation_scope": manifest["validation_scope"],
        "manifest_sha256": sha256(manifest_path),
        "base_controller_sha256": sha256(base_controller_path),
        "candidates": candidates,
    }
    write_json(output / "openarm-joint-loss-controller-tuning-suite.json", suite)
    return suite


def main() -> int:
    args = parse_args()
    suite = compile_candidates(
        args.manifest.resolve(), args.base_controller.resolve(), args.output.resolve()
    )
    print(
        "OpenArm joint-loss controller tuning suite: "
        f"candidates={len(suite['candidates'])} -> {args.output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm joint-loss controller tuning failed: {error}")
        raise SystemExit(2)

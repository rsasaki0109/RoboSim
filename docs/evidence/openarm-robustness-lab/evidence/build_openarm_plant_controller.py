#!/usr/bin/env python3
"""Compile the versioned OpenArm plant experiment into the shared controller schema."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
from typing import Any


MANIFEST_KIND = "rne_openarm_plant_experiment_manifest"
CONTROLLER_KIND = "rne_joint_pose_cycle_controller"


def parse_args() -> argparse.Namespace:
    root = Path(__file__).resolve().parents[3]
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--manifest",
        type=Path,
        default=root
        / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_experiments.json",
    )
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args()


def load_manifest(path: Path) -> dict[str, Any]:
    manifest = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(manifest, dict):
        raise ValueError("plant experiment manifest must be one JSON object")
    required = {
        "kind",
        "schema_version",
        "experiment_id",
        "task_id",
        "controller_id",
        "fixed_delta_ticks",
        "sample_rate_hz",
        "action_joint_order",
        "rne_actuator_link_order",
        "initial_reference_rad",
        "operating_point_rad",
        "input",
        "output",
        "segments",
        "analysis",
        "intentional_failure",
    }
    if set(manifest) != required:
        raise ValueError(
            f"plant experiment manifest fields differ: {sorted(set(manifest) ^ required)}"
        )
    width = len(manifest["action_joint_order"])
    if (
        manifest["kind"] != MANIFEST_KIND
        or manifest["schema_version"] != 1
        or width != 9
        or len(set(manifest["action_joint_order"])) != width
        or len(manifest["rne_actuator_link_order"]) != width
        or len(manifest["initial_reference_rad"]) != width
        or len(manifest["operating_point_rad"]) != width
        or manifest["fixed_delta_ticks"] <= 0
    ):
        raise ValueError("invalid plant experiment identity, joint order, or clock")
    expected_hz = 1_000_000_000.0 / manifest["fixed_delta_ticks"]
    if not math.isclose(manifest["sample_rate_hz"], expected_hz, abs_tol=1e-12):
        raise ValueError("sample rate and fixed simulation period differ")
    segments = manifest["segments"]
    if not segments or segments[0]["start_step"] != 1:
        raise ValueError("plant experiment must start at step one")
    for index, segment in enumerate(segments):
        if segment["start_step"] > segment["end_step"]:
            raise ValueError(f"invalid segment bounds for {segment['id']}")
        if index and segment["start_step"] != segments[index - 1]["end_step"] + 1:
            raise ValueError("plant experiment segments must be contiguous")
    return manifest


def segment_target(
    manifest: dict[str, Any], segment: dict[str, Any], step: int
) -> list[float]:
    target = list(manifest["operating_point_rad"])
    order = manifest["action_joint_order"]
    rate_hz = manifest["sample_rate_hz"]
    kind = segment["kind"]
    if kind == "operating_point_ramp":
        if step >= segment["ramp_end_step"]:
            return target
        alpha = (step - segment["start_step"] + 1) / (
            segment["ramp_end_step"] - segment["start_step"] + 1
        )
        smooth = alpha * alpha * (3.0 - 2.0 * alpha)
        return [
            initial + (operating - initial) * smooth
            for initial, operating in zip(
                manifest["initial_reference_rad"], manifest["operating_point_rad"]
            )
        ]
    if kind == "hold":
        return target
    if kind == "step_doublet":
        index = order.index(segment["joint"])
        if segment["positive_step"] <= step < segment["negative_step"]:
            target[index] += segment["amplitude_rad"]
        return target
    if kind == "triangular_ramp":
        index = order.index(segment["joint"])
        if step <= segment["peak_step"]:
            alpha = (step - segment["start_step"] + 1) / (
                segment["peak_step"] - segment["start_step"] + 1
            )
        else:
            alpha = (segment["end_step"] - step) / (
                segment["end_step"] - segment["peak_step"]
            )
        target[index] += segment["peak_amplitude_rad"] * max(0.0, alpha)
        return target
    elapsed_s = (step - segment["start_step"]) / rate_hz
    if kind == "linear_chirp":
        index = order.index(segment["joint"])
        duration_s = (segment["end_step"] - segment["start_step"]) / rate_hz
        sweep_hz_s = (
            segment["end_frequency_hz"] - segment["start_frequency_hz"]
        ) / duration_s
        phase = 2.0 * math.pi * (
            segment["start_frequency_hz"] * elapsed_s
            + 0.5 * sweep_hz_s * elapsed_s * elapsed_s
        )
        target[index] += segment["amplitude_rad"] * math.sin(phase)
        return target
    if kind == "multisine":
        index = order.index(segment["joint"])
        target[index] += sum(
            amplitude * math.sin(2.0 * math.pi * frequency * elapsed_s + phase)
            for frequency, amplitude, phase in zip(
                segment["frequencies_hz"],
                segment["amplitudes_rad"],
                segment["phases_rad"],
            )
        )
        return target
    if kind == "frequency_separated_coupling":
        for source in segment["sources"]:
            index = order.index(source["joint"])
            target[index] += source["amplitude_rad"] * math.sin(
                2.0 * math.pi * source["frequency_hz"] * elapsed_s
                + source["phase_rad"]
            )
        return target
    raise ValueError(f"unsupported plant experiment segment kind {kind!r}")


def compile_controller(manifest: dict[str, Any]) -> dict[str, Any]:
    final_step = manifest["segments"][-1]["end_step"]
    segment_index = 0
    keyframes = [
        {
            "step": 0,
            "phase": "initial_reference",
            "joint_position_target_rad": manifest["initial_reference_rad"],
        }
    ]
    for step in range(1, final_step + 1):
        while step > manifest["segments"][segment_index]["end_step"]:
            segment_index += 1
        segment = manifest["segments"][segment_index]
        keyframes.append(
            {
                "step": step,
                "phase": segment["id"],
                "joint_position_target_rad": segment_target(manifest, segment, step),
            }
        )
    return {
        "kind": CONTROLLER_KIND,
        "schema_version": 1,
        "controller_id": manifest["controller_id"],
        "task_id": manifest["task_id"],
        "interpolation": "smoothstep_v1",
        "action_joint_order": manifest["action_joint_order"],
        "rne_actuator_link_order": manifest["rne_actuator_link_order"],
        "keyframes": keyframes,
        "intentional_failure": manifest["intentional_failure"],
    }


def main() -> int:
    args = parse_args()
    manifest = load_manifest(args.manifest.resolve())
    controller = compile_controller(manifest)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(
        json.dumps(controller, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    print(
        f"OpenArm plant controller: steps={controller['keyframes'][-1]['step']} "
        f"output={args.output}"
    )
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"OpenArm plant controller failed: {error}", file=sys.stderr)
        raise SystemExit(2)

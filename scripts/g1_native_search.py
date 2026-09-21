#!/usr/bin/env python3
"""Deterministic bounded pattern search using the live native G1 physics probe.

This optimizes a diagnostic contact model. It never promotes an objective value
or a completed rollout to physical backflip qualification.
"""

import argparse
import copy
import hashlib
import json
import math
import shutil
import subprocess
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# index, bounds, initial coordinate step: launch, tuck, opening, landing.
AXES = [
    (0, 1.8, 2.7, 0.15),
    (3, 0.5, 2.0, 0.2),
    (6, 1.4, 2.7, 0.2),
    (7, 1.2, 2.7, 0.2),
    (11, 4.5, 6.1, 0.25),
    (10, -0.7, 0.3, 0.15),
]


def loss(result):
    """Score motion and measured limits; this is not a qualification predicate."""
    frames = result.get("frames", [])
    if not frames:
        return 1e6
    last = frames[-1]
    values = [
        result["signed_rotation_rad"],
        result["peak_joint_speed_ratio"],
        result["max_joint_position_excess_rad"],
        last["upright"],
        last["base_translation_m"][1],
    ]
    if not all(math.isfinite(x) for x in values):
        return 1e6
    rotation, speed, excess, upright, height = values
    duration = max(0.0, min(5.0, result["completed_maneuver_time_s"]))
    tail_speed = result.get("final_second_max_base_speed_m_s")
    return (
        50 * (5 - duration)
        + 20 * abs(rotation + math.tau)
        + 40 * (1 - upright)
        + 100 * max(0.0, 0.65 - height)
        + 500 * max(0.0, speed - 1.05) ** 2
        + 1000 * max(0.0, excess - 0.02) ** 2
        + (50 if result.get("takeoff_s") is None else 0)
        + (20 * min(tail_speed, 10) if tail_speed is not None else 0)
    )


def write_json(path, value):
    temporary = path.with_suffix(".tmp")
    temporary.write_text(json.dumps(value, indent=2, allow_nan=False) + "\n")
    temporary.replace(path)


def search(binary, scene, candidate, output, rounds, workers, axes=AXES):
    """Evaluate each coordinate batch concurrently, then select in fixed order."""
    binary, scene, output = (
        Path(binary).resolve(),
        Path(scene).resolve(),
        Path(output).resolve(),
    )
    if not binary.is_file() or not scene.is_file():
        raise ValueError("built native binary and prepared scene required")
    if rounds < 0 or workers not in range(1, 5):
        raise ValueError("nonnegative rounds and 1..4 workers required")
    if shutil.disk_usage(output.parent).free < 30 * 1024**3:
        raise RuntimeError("30 GiB disk reserve required")
    output.mkdir(exist_ok=False)
    rows = []
    next_index = 0
    binary_hash = hashlib.sha256(binary.read_bytes()).hexdigest()

    def evaluate(item):
        index, parameters = item
        if shutil.disk_usage(output).free < 30 * 1024**3:
            raise RuntimeError("30 GiB disk reserve required")
        prefix = output / f"candidate-{index:04d}"
        input_path = prefix.with_suffix(".json")
        rollout_path = prefix.with_suffix(".rollout.json")
        write_json(input_path, parameters)
        command = [
            str(binary),
            "--native-probe",
            "--native-declared",
            "--native-velocity-servo",
            "--native-scene",
            str(scene),
            "--native-dt-us",
            "500",
            "--native-stop-on-fall",
            "--native-candidate",
            str(input_path),
            "--native-output",
            str(rollout_path),
        ]
        with prefix.with_suffix(".log").open("w") as log:
            subprocess.run(
                command, stdout=log, stderr=subprocess.STDOUT, check=True, timeout=600
            )
        result = json.loads(rollout_path.read_text())
        if result["backend"] != "RoboSim/Rapier":
            raise ValueError("unexpected physics backend")
        return {
            "index": index,
            "loss": loss(result),
            "candidate": parameters,
            "qualified_backflip": False,
            "rollout_sha256": hashlib.sha256(rollout_path.read_bytes()).hexdigest(),
            "completed_maneuver_time_s": result["completed_maneuver_time_s"],
            "peak_joint_speed_ratio": result["peak_joint_speed_ratio"],
        }

    def save(best):
        write_json(
            output / "search.json",
            {
                "algorithm": "bounded_coordinate_pattern_search",
                "binary_sha256": binary_hash,
                "scene": str(scene),
                "workers": workers,
                "dt_s": 0.0005,
                "axes": axes,
                "qualified_backflip": False,
                "note": "Diagnostic model only; complete contact qualification remains open.",
                "evaluations": rows,
                "best": best,
            },
        )
        write_json(output / "best-candidate.json", best["candidate"])

    best = evaluate((next_index, candidate))
    rows.append(best)
    next_index += 1
    save(best)
    with ThreadPoolExecutor(max_workers=workers) as pool:
        for iteration in range(rounds):
            batch = []
            for index, lower, upper, step in axes:
                for sign in [-1, 1]:
                    variant = copy.deepcopy(best["candidate"])
                    variant["parameters"][index] = max(
                        lower,
                        min(
                            upper,
                            variant["parameters"][index] + sign * step / 2**iteration,
                        ),
                    )
                    batch.append((next_index, variant))
                    next_index += 1
            results = list(pool.map(evaluate, batch))
            rows.extend(results)
            best = min([best] + results, key=lambda row: (row["loss"], row["index"]))
            save(best)
            print(
                f"round {iteration + 1}: {len(rows)} evaluations, best loss {best['loss']:.6f}; qualification false",
                flush=True,
            )
    return best


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    for name in ["binary", "scene", "candidate", "output"]:
        parser.add_argument("--" + name, required=True, type=Path)
    parser.add_argument("--rounds", type=int, default=1)
    parser.add_argument("--workers", type=int, default=2)
    args = parser.parse_args()
    search(
        args.binary,
        args.scene,
        json.loads(args.candidate.read_text()),
        args.output,
        args.rounds,
        args.workers,
    )


if __name__ == "__main__":
    main()

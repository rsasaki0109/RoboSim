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

# A focused first stage after armature alignment: launch hip, tuck knee, opening.
LAUNCH_TUCK_OPEN_AXES = [
    (3, 0.5, 2.0, 0.2),
    (7, 1.2, 2.7, 0.4),
    (11, 4.5, 6.1, 0.25),
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
    contact_cost = 0.0
    if result.get("structural_contact_filter"):
        pairs = result.get("contact_pair_audit")
        if not isinstance(pairs, list):
            return 1e6
        for pair in pairs:
            impulse = pair.get("max_normal_impulse_ns")
            if not isinstance(impulse, (int, float)) or not math.isfinite(impulse) or impulse < 0:
                return 1e6
            names = {pair.get("link_a"), pair.get("link_b")}
            is_support = "environment" in names and bool(names & {"left_ankle_roll_link", "right_ankle_roll_link"})
            if not is_support:
                contact_cost += 250 * min(1.0, impulse)
    return (
        contact_cost
        + 50 * (5 - duration)
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


def search(
    binary,
    scene,
    candidate,
    output,
    rounds,
    workers,
    axes=AXES,
    *,
    dt_us=500,
    motor_mode="velocity",
    duration_s=5,
    structural_filter=False,
):
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
    if dt_us not in (62.5, 125, 250, 500, 1000) or motor_mode not in ("velocity", "effort"):
        raise ValueError("supported step and velocity/effort motor mode required")
    if not isinstance(duration_s, int) or duration_s not in range(5, 16):
        raise ValueError("integer maneuver duration in 5..15 required")
    if not isinstance(structural_filter, bool):
        raise ValueError("boolean structural_filter required")
    timeout_s = int((1200 if dt_us < 500 else 600) * max(1.0, 125 / dt_us) * duration_s / 5)
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
        if hashlib.sha256(binary.read_bytes()).hexdigest() != binary_hash:
            raise ValueError("native binary changed during campaign")
        prefix = output / f"candidate-{index:04d}"
        input_path = prefix.with_suffix(".json")
        rollout_path = prefix.with_suffix(".rollout.json")
        write_json(input_path, parameters)
        command = [
            str(binary),
            "--native-probe",
            "--native-declared",
            "--native-scene",
            str(scene),
            "--native-dt-us",
            str(dt_us),
            "--native-duration-s",
            str(duration_s),
            "--native-stop-on-fall",
            "--native-candidate",
            str(input_path),
            "--native-output",
            str(rollout_path),
        ]
        if structural_filter:
            command.append("--native-structural-filter")
        if motor_mode == "velocity":
            command.append("--native-velocity-servo")
        with prefix.with_suffix(".log").open("w") as log:
            subprocess.run(
                command,
                stdout=log,
                stderr=subprocess.STDOUT,
                check=True,
                timeout=timeout_s,
            )
        result = json.loads(rollout_path.read_text())
        if hashlib.sha256(binary.read_bytes()).hexdigest() != binary_hash:
            raise ValueError("native binary changed during campaign")
        if result["backend"] != "RoboSim/Rapier":
            raise ValueError("unexpected physics backend")
        if (
            result["dt_s"] != dt_us * 1e-6
            or result.get("maneuver_duration_s") != duration_s
            or result["velocity_servo"] != (motor_mode == "velocity")
            or result["implicit_position_motors"]
            or result["joint_armature_kg_m2"]
            != parameters.get("joint_armature_kg_m2", 0.0)
        ):
            raise ValueError(
                "rollout step, motor mode or armature differs from campaign"
            )
        if result.get("structural_contact_filter", False) != structural_filter:
            raise ValueError("rollout structural contact policy differs from campaign")
        if structural_filter and (
            result.get("convex_collider_count", 0) <= 0
            or not isinstance(result.get("contact_pair_audit"), list)
            or not isinstance(result.get("structural_excluded_link_pairs"), list)
        ):
            raise ValueError("full-contact geometry and pair audit required")
        for field in (
            "recovery_s",
            "landing_kp_nm_per_rad",
            "landing_kd_nm_s_per_rad",
            "landing_capture_gain_rad_per_m",
            "landing_pitch_rate_gain_s",
            "landing_early_com_velocity_gain_s_per_m",
        ):
            if field in parameters and result.get(field) != parameters[field]:
                raise ValueError(f"rollout {field} differs from campaign")
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
                "dt_s": dt_us * 1e-6,
                "motor_mode": motor_mode,
                "structural_filter": structural_filter,
                "timeout_s": timeout_s,
                "maneuver_duration_s": duration_s,
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
            # Candidates were all generated from the round's starting point.
            # Consume in index order so checkpoints and ties stay deterministic.
            for result in pool.map(evaluate, batch):
                rows.append(result)
                best = min([best, result], key=lambda row: (row["loss"], row["index"]))
                save(best)
                print(
                    f"candidate {result['index']}: loss {result['loss']:.6f}, "
                    f"completed {result['completed_maneuver_time_s']:.6f} s, "
                    f"speed/rating {result['peak_joint_speed_ratio']:.6f}",
                    flush=True,
                )
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
    parser.add_argument("--dt-us", type=float, choices=[62.5, 125, 250, 500, 1000], default=500)
    parser.add_argument(
        "--motor-mode", choices=["velocity", "effort"], default="velocity"
    )
    parser.add_argument("--axes", choices=["all", "launch-tuck-open"], default="all")
    parser.add_argument("--structural-filter", action="store_true")
    args = parser.parse_args()
    search(
        args.binary,
        args.scene,
        json.loads(args.candidate.read_text()),
        args.output,
        args.rounds,
        args.workers,
        AXES if args.axes == "all" else LAUNCH_TUCK_OPEN_AXES,
        dt_us=args.dt_us,
        motor_mode=args.motor_mode,
        structural_filter=args.structural_filter,
    )


if __name__ == "__main__":
    main()

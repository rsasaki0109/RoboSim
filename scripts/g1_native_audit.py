#!/usr/bin/env python3
"""Fail-closed audit of measured native G1 rollout gates (stdlib only).

This checks recorded metrics, not source/model provenance or hardware capability.
Keep the original recordings and their producer hashes alongside the report.
"""

import argparse
import gzip
import json
import math
from pathlib import Path


def finite(value):
    return type(value) in (int, float) and math.isfinite(value)


def audit(result):
    """Return individual gates; absent telemetry cannot count as passing."""
    gates = {}

    def bound(key, predicate):
        value = result.get(key)
        gates[key] = finite(value) and predicate(value)

    gates["native_direct_effort"] = (
        result.get("backend") == "RoboSim/Rapier"
        and result.get("declared_inertial_scene") is True
        and result.get("velocity_servo") is False
        and result.get("implicit_position_motors") is False
        and result.get("standing_only") is False
    )
    duration, completed, dt = (result.get(k) for k in
        ("maneuver_duration_s", "completed_maneuver_time_s", "dt_s"))
    timing_valid = all(finite(x) for x in (duration, completed, dt)) and dt > 0
    gates["completed"] = (
        timing_valid and duration >= 5 and abs(completed - duration) < dt / 2
        and "failure" in result and result["failure"] is None
    )
    bound("signed_rotation_rad", lambda x: abs(x + math.tau) < 0.25)
    bound("longest_air_s", lambda x: x > 0.25)
    takeoff, touchdown = result.get("takeoff_s"), result.get("touchdown_s")
    gates["flight_and_landing"] = (
        finite(takeoff) and finite(touchdown) and finite(completed)
        and 0 <= takeoff < touchdown < completed
    )
    bound("peak_joint_speed_ratio", lambda x: 0 <= x < 1.05)
    bound("max_joint_position_excess_rad", lambda x: 0 <= x < 0.02)
    bound("final_second_min_upright", lambda x: 0.99 < x <= 1.000001)
    bound("final_second_max_base_speed_m_s", lambda x: 0 <= x < 0.1)
    bound("final_second_min_base_height_m", lambda x: x > 0.65)
    bound("final_standing_error", lambda x: 0 <= x < 0.03)
    gates["continuous_support"] = result.get("final_second_continuous_foot_contact") is True
    names = result.get("joint_link_names")
    names_valid = (isinstance(names, list) and len(names) == 23
        and all(isinstance(n, str) for n in names) and len(set(names)) == 23)
    exclusions = result.get("structural_excluded_link_pairs")
    gates["full_contact_profile"] = (
        result.get("structural_contact_filter") is True
        and result.get("convex_collider_count") == 21
        and result.get("compound_part_counts") == [4, 4]
        and isinstance(exclusions, list) and len(exclusions) == 38
        and names_valid
    )
    efforts = result.get("joint_effort_audit")
    expected_steps = round((duration + 1) / dt) if timing_valid else -1
    gates["measured_effort_limits"] = (
        result.get("effort_measurements_valid") is True
        and names_valid and isinstance(efforts, list) and len(efforts) == len(names)
        and all(isinstance(row, dict) and row.get("link_name") == name
            and finite(row.get("limit_nm")) and row["limit_nm"] > 0
            and finite(row.get("peak_measured_effort_nm"))
            and 0 <= row["peak_measured_effort_nm"] <= row["limit_nm"]
            and row.get("measured_steps") == expected_steps
            for name, row in zip(names, efforts))
    )
    pairs, gaps = result.get("contact_pair_audit"), result.get("contact_separation_audit")
    contact_valid = isinstance(pairs, list) and bool(pairs) and isinstance(gaps, list) and bool(gaps)
    nonfoot_ground, overlapping_self = [], []
    if contact_valid:
        for row in pairs + gaps:
            if not isinstance(row, dict) or not all(isinstance(row.get(k), str) for k in ("link_a", "link_b")):
                contact_valid = False
                continue
            pair = (row["link_a"], row["link_b"])
            if "environment" in pair and not any(n in pair for n in ("left_ankle_roll_link", "right_ankle_roll_link")):
                nonfoot_ground.append(pair)
        for row in gaps:
            if not isinstance(row, dict) or not all(isinstance(row.get(k), str) for k in ("link_a", "link_b")):
                continue
            gap, count = row.get("min_solver_separation_m"), row.get("negative_separation_steps")
            reported = row.get("reported_steps")
            if (not finite(gap) or type(count) is not int or type(reported) is not int
                    or not 0 <= count <= reported or reported <= 0 or (gap < 0) != (count > 0)):
                contact_valid = False
            elif count and "environment" not in (row.get("link_a"), row.get("link_b")):
                overlapping_self.append((row.get("link_a"), row.get("link_b")))
        for row in pairs:
            if not isinstance(row, dict) or not finite(row.get("max_normal_impulse_ns")) or row["max_normal_impulse_ns"] < 0:
                contact_valid = False
        if contact_valid:
            pair_names = lambda row: tuple(sorted((row["link_a"], row["link_b"])))
            # Every retained impulse pair needs geometric evidence too; a
            # truncated self-gap table must not look collision-free.
            contact_valid = {pair_names(row) for row in pairs} <= {pair_names(row) for row in gaps}
    gates["contact_evidence"] = contact_valid
    gates["no_nonfoot_ground_contact"] = contact_valid and not nonfoot_ground
    gates["no_nonadjacent_self_overlap"] = contact_valid and not overlapping_self
    return {"passed_recorded_gates": all(gates.values()), "gates": gates,
        "failed_gates": [key for key, passed in gates.items() if not passed],
        "nonfoot_ground_pairs": sorted(set(nonfoot_ground)),
        "overlapping_self_pairs": sorted(set(overlapping_self))}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("recordings", type=Path, nargs="+")
    args = parser.parse_args()
    reports = []
    for path in args.recordings:
        data = gzip.decompress(path.read_bytes()) if path.suffix == ".gz" else path.read_bytes()
        reports.append({"recording": str(path), **audit(json.loads(data))})
    print(json.dumps(reports, indent=2, allow_nan=False))
    return 0 if all(row["passed_recorded_gates"] for row in reports) else 1


if __name__ == "__main__":
    raise SystemExit(main())

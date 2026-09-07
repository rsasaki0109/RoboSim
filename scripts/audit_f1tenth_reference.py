"""Offline pose/twist consistency diagnostic, not independent accuracy scoring.

No command alignment, clock offset correction or automatic angular scaling.
Finite differences are retrospective interval averages, never online observations.
"""

import argparse
import bisect
import hashlib
import json
import math
from pathlib import Path

from audit_f1tenth_source import SOURCE_SHA256, verify_source


def yaw(quaternion):
    """Planar heading of a verified approximately unit x/y/z/w quaternion."""
    if len(quaternion) != 4 or not all(map(math.isfinite, quaternion)):
        raise ValueError("invalid quaternion")
    x, y, z, w = quaternion
    if abs(sum(v * v for v in quaternion) - 1.0) > 1e-3:
        raise ValueError("non-unit quaternion")
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def validate_rows(poses, twists):
    """Reject unordered or nonfinite diagnostic inputs before calculating."""
    for rows in (poses, twists):
        if any(len(r) != 4 or not isinstance(r[0], int) or r[0] < 0
               or not all(math.isfinite(v) for v in r[1:]) for r in rows):
            raise ValueError("invalid reference row")
        if any(b[0] <= a[0] for a, b in zip(rows, rows[1:])):
            raise ValueError("source timestamps must strictly increase")


def compare(poses, twists):
    """Compare world x/y and planar heading differences to causal latest twist.

    Rows: pose=(header_ns,x,y,heading), twist=(header_ns,vx,vy,angular_z).
    Fixed diagnostic policy: intervals 1..50 ms, twist age 0..20 ms.
    """
    validate_rows(poses, twists)
    times = [r[0] for r in twists]
    pairs = 0
    skipped_interval = 0
    missing_twist = 0
    linear_error = 0.0
    heading_squared = 0.0
    angular_squared = 0.0
    cross = 0.0
    angular_error = 0.0
    for a, b in zip(poses, poses[1:]):
        ticks = b[0] - a[0]
        if not 1_000_000 <= ticks <= 50_000_000:
            skipped_interval += 1
            continue
        index = bisect.bisect_right(times, b[0]) - 1
        if index < 0 or b[0] - times[index] > 20_000_000:
            missing_twist += 1
            continue
        t = twists[index]
        dt = ticks * 1e-9
        vx, vy = (b[1] - a[1]) / dt, (b[2] - a[2]) / dt
        rate = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3])) / dt
        linear_error += (vx - t[1]) ** 2 + (vy - t[2]) ** 2
        heading_squared += rate ** 2
        angular_squared += t[3] ** 2
        cross += rate * t[3]
        angular_error += (rate - t[3]) ** 2
        pairs += 1
    return {
        "pose_samples": len(poses), "twist_samples": len(twists), "paired_intervals": pairs,
        "skipped_interval_count": skipped_interval, "missing_or_stale_twist_count": missing_twist,
        "pose_interval_bounds_ns": [1_000_000, 50_000_000], "max_twist_age_ns": 20_000_000,
        "planar_linear_difference_rms_m_s": math.sqrt(linear_error / pairs) if pairs else None,
        "pose_heading_rate_rms_rad_s": math.sqrt(heading_squared / pairs) if pairs else None,
        "recorded_angular_z_rms": math.sqrt(angular_squared / pairs) if pairs else None,
        "unscaled_angular_difference_rms": math.sqrt(angular_error / pairs) if pairs else None,
        "diagnostic_through_origin_angular_scale": cross / angular_squared if angular_squared else None,
        "scale_applied": False, "independent_accuracy_validated": False,
        "reference_qualified": False,
    }


def compare_integrals(poses, twists):
    """Integrate held world-frame twist over each complete pose interval.

    A newly arrived value applies only after its timestamp, never to the previous
    interval. Reject the entire pair if any positive-duration part lacks fresh
    twist. This is a source-time consistency check, not proof of clock alignment.
    """
    validate_rows(poses, twists)
    times = [r[0] for r in twists]
    pairs = skipped = missing = 0
    position_error = angle_error = pose_angle_squared = raw_angle_squared = cross = 0.0
    for a, b in zip(poses, poses[1:]):
        if not 1_000_000 <= b[0] - a[0] <= 50_000_000:
            skipped += 1
            continue
        cursor = a[0]
        index = bisect.bisect_right(times, cursor) - 1
        displacement = [0.0, 0.0, 0.0]
        while cursor < b[0]:
            if index < 0 or cursor >= times[index] + 20_000_000:
                break
            end = min(b[0], times[index] + 20_000_000)
            if index + 1 < len(times):
                end = min(end, times[index + 1])
            dt = (end - cursor) * 1e-9
            for axis in range(3):
                displacement[axis] += twists[index][axis + 1] * dt
            cursor = end
            if index + 1 < len(times) and times[index + 1] == cursor:
                index += 1
        if cursor != b[0]:
            missing += 1
            continue
        angle = math.atan2(math.sin(b[3] - a[3]), math.cos(b[3] - a[3]))
        position_error += sum((b[axis + 1] - a[axis + 1] - displacement[axis]) ** 2 for axis in range(2))
        angle_error += (angle - displacement[2]) ** 2
        pose_angle_squared += angle ** 2
        raw_angle_squared += displacement[2] ** 2
        cross += angle * displacement[2]
        pairs += 1
    return {
        "paired_intervals": pairs, "skipped_interval_count": skipped,
        "incompletely_observed_interval_count": missing,
        "pose_interval_bounds_ns": [1_000_000, 50_000_000], "max_twist_hold_ns": 20_000_000,
        "planar_displacement_difference_rms_m": math.sqrt(position_error / pairs) if pairs else None,
        "pose_heading_increment_rms_rad": math.sqrt(pose_angle_squared / pairs) if pairs else None,
        "integrated_recorded_angular_z_rms": math.sqrt(raw_angle_squared / pairs) if pairs else None,
        "unscaled_angular_increment_difference_rms": math.sqrt(angle_error / pairs) if pairs else None,
        "diagnostic_through_origin_angular_scale": cross / raw_angle_squared if raw_angle_squared else None,
        "scale_applied": False, "reference_qualified": False,
    }


def audit(path):
    verify_source(path)
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    poses, twists = [], []
    base = "/vrpn_client_node/Car_2_Tracking/"
    with Reader(path) as reader:
        connections = [c for c in reader.connections if c.topic in (base + "pose", base + "twist")]
        store = get_typestore(Stores.EMPTY)
        definitions = {}
        for c in connections:
            definitions.update(get_types_from_msg(c.msgdef.data, c.msgtype))
        store.register(definitions)
        for c, _, raw in reader.messages(connections=connections):
            msg = store.deserialize_ros1(raw, c.msgtype)
            if msg.header.frame_id != "world":
                raise ValueError("unexpected reference frame")
            stamp = msg.header.stamp
            if not 0 <= stamp.nanosec < 1_000_000_000:
                raise ValueError("invalid nanoseconds")
            ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            if c.topic == base + "pose":
                q = msg.pose.orientation
                poses.append((ns, msg.pose.position.x, msg.pose.position.y, yaw((q.x, q.y, q.z, q.w))))
            else:
                twists.append((ns, msg.twist.linear.x, msg.twist.linear.y, msg.twist.angular.z))
    if len(poses) != 13426 or len(twists) != 13402:
        raise ValueError("pinned capture count mismatch")
    report = compare(poses, twists)
    report["interval_integral_comparison"] = compare_integrals(poses, twists)
    report.update(kind="rne_f1tenth_reference_consistency", schema_version=2, source_sha256=SOURCE_SHA256)
    canonical = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
    report["audit_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    print(json.dumps(audit(parser.parse_args().bag), indent=2, sort_keys=True, allow_nan=False))

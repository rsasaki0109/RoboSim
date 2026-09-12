"""Audit clocks in exposed F1TENTH runs without synchronizing capture time.

The unstamped command can only be ordered on the rosbag recorder clock. Affine
header-to-bag fits are diagnostics of recorded clock behavior, never latency
measurements and never corrections applied to source data.
"""

import argparse
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path

from audit_f1tenth_controls import EXPOSED_RUNS
from audit_f1tenth_source import verify_source

CMD = "/cmd_vel"
POSE = "/vrpn_client_node/Car_2_Tracking/pose"
TWIST = "/vrpn_client_node/Car_2_Tracking/twist"
VESC = "/sensors/core"
TOPICS = {
    CMD: "geometry_msgs/msg/Twist",
    POSE: "geometry_msgs/msg/PoseStamped",
    TWIST: "geometry_msgs/msg/TwistStamped",
    VESC: "vesc_msgs/msg/VescStateStamped",
}


def percentile(values, probability):
    """Return a deterministic linearly interpolated percentile."""
    if not values or not 0.0 <= probability <= 1.0:
        raise ValueError("invalid percentile input")
    ordered = sorted(values)
    position = probability * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    fraction = position - lower
    return ordered[lower] * (1.0 - fraction) + ordered[upper] * fraction


def interval_report(times):
    """Describe source ordering without repairing reversals or duplicates."""
    if not times or any(not isinstance(value, int) or value < 0 for value in times):
        raise ValueError("invalid clock values")
    deltas = [b - a for a, b in zip(times, times[1:])]
    positive = [value for value in deltas if value > 0]
    return {
        "sample_count": len(times),
        "reversal_count": sum(value < 0 for value in deltas),
        "duplicate_count": sum(value == 0 for value in deltas),
        "minimum_positive_interval_ns": min(positive) if positive else None,
        "median_positive_interval_ns": percentile(positive, 0.5) if positive else None,
        "p99_positive_interval_ns": percentile(positive, 0.99) if positive else None,
        "maximum_positive_interval_ns": max(positive) if positive else None,
        "strictly_increasing": len(positive) == len(deltas),
    }


def affine_clock_report(rows):
    """Fit bag elapsed time from header elapsed time for diagnostics only."""
    if len(rows) < 3:
        raise ValueError("too few stamped clock rows")
    if any(
        not isinstance(bag, int) or not isinstance(header, int) or bag < 0 or header < 0
        for bag, header in rows
    ):
        raise ValueError("invalid stamped clock row")
    bag_origin, header_origin = rows[0]
    header_elapsed = [header - header_origin for _, header in rows]
    bag_elapsed = [bag - bag_origin for bag, _ in rows]
    x_mean = sum(header_elapsed) / len(rows)
    y_mean = sum(bag_elapsed) / len(rows)
    denominator = sum((value - x_mean) ** 2 for value in header_elapsed)
    if denominator == 0:
        raise ValueError("stamped clock has no elapsed-time excitation")
    scale = sum(
        (x - x_mean) * (y - y_mean) for x, y in zip(header_elapsed, bag_elapsed)
    ) / denominator
    intercept = y_mean - scale * x_mean
    residuals = [y - (scale * x + intercept) for x, y in zip(header_elapsed, bag_elapsed)]
    absolute = [abs(value) for value in residuals]
    raw_offsets = [bag - header for bag, header in rows]
    return {
        "sample_count": len(rows),
        "bag_origin_ns": bag_origin,
        "header_origin_ns": header_origin,
        "bag_elapsed_from_header_elapsed_scale": scale,
        "scale_error_ppm": (scale - 1.0) * 1_000_000.0,
        "elapsed_intercept_ns": intercept,
        "residual_rms_ns": math.sqrt(sum(value * value for value in residuals) / len(rows)),
        "absolute_residual_p50_ns": percentile(absolute, 0.5),
        "absolute_residual_p95_ns": percentile(absolute, 0.95),
        "absolute_residual_p99_ns": percentile(absolute, 0.99),
        "maximum_absolute_residual_ns": max(absolute),
        "minimum_raw_bag_minus_header_ns": min(raw_offsets),
        "maximum_raw_bag_minus_header_ns": max(raw_offsets),
        "mapping_applied": False,
        "physical_latency_qualified": False,
    }


def report_from_rows(rows):
    """Build the fail-closed clock contract from already decoded timestamps."""
    if set(rows) != set(TOPICS):
        raise ValueError("required F1TENTH clock channels missing")
    command_bag = [bag for bag, header in rows[CMD] if header is None]
    if len(command_bag) != len(rows[CMD]):
        raise ValueError("command unexpectedly contains a decoded header timestamp")
    channel_reports = {}
    for topic in sorted(TOPICS):
        bag_times = [bag for bag, _ in rows[topic]]
        headers = [header for _, header in rows[topic] if header is not None]
        channel = {"bag_time": interval_report(bag_times), "header_present": bool(headers)}
        if headers:
            if len(headers) != len(rows[topic]):
                raise ValueError("partially stamped clock channel")
            channel["header_time"] = interval_report(headers)
            channel["header_to_bag_diagnostic"] = affine_clock_report(rows[topic])
        channel_reports[topic] = channel
    recorder_order = all(
        channel["bag_time"]["strictly_increasing"] for channel in channel_reports.values()
    )
    return {
        "channels": channel_reports,
        "command_header_present": False,
        "recorder_time_ordering_qualified": recorder_order,
        "deterministic_bag_time_replay_qualified": recorder_order,
        "command_capture_clock_qualified": False,
        "input_to_reference_capture_clock_qualified": False,
        "physical_latency_qualified": False,
        "timestamp_adjustments_applied": False,
        "clock_mapping_applied": False,
    }


def audit(path):
    """Decode timestamps only from an immutable, development-exposed run."""
    if path.name not in EXPOSED_RUNS:
        raise ValueError("clock audit is forbidden for unexposed F1TENTH runs")
    source = verify_source(path)
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    rows = {topic: [] for topic in TOPICS}
    with Reader(path) as reader:
        connections = [connection for connection in reader.connections if connection.topic in TOPICS]
        if {connection.topic for connection in connections} != set(TOPICS):
            raise ValueError("required F1TENTH clock channels missing")
        expected = {topic: 0 for topic in TOPICS}
        definitions = {}
        for connection in connections:
            if connection.msgtype != TOPICS[connection.topic]:
                raise ValueError("unexpected F1TENTH clock message type")
            definitions.update(get_types_from_msg(connection.msgdef.data, connection.msgtype))
            expected[connection.topic] += connection.msgcount
        store = get_typestore(Stores.EMPTY)
        store.register(definitions)
        for connection, bag_ns, raw in reader.messages(connections=connections):
            header_ns = None
            if connection.topic != CMD:
                message = store.deserialize_ros1(raw, connection.msgtype)
                stamp = message.header.stamp
                if not 0 <= stamp.nanosec < 1_000_000_000:
                    raise ValueError("invalid header nanoseconds")
                header_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            rows[connection.topic].append((bag_ns, header_ns))
        if any(len(rows[topic]) != expected[topic] for topic in TOPICS):
            raise ValueError("decoded clock count differs from connection index")

    report = report_from_rows(rows)
    report.update(
        schema_version=1,
        kind="rne_f1tenth_exposed_clock_audit",
        source_file=path.name,
        source_sha256=source.sha256,
        reader_version=importlib.metadata.version("rosbags"),
    )
    canonical = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
    report["audit_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    print(json.dumps(audit(parser.parse_args().bag), indent=2, sort_keys=True, allow_nan=False))

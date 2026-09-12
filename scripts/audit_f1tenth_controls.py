"""Audit exposed F1TENTH command paths without qualifying clocks or steering feedback."""

import argparse
import bisect
import hashlib
import importlib.metadata
import json
import math
from pathlib import Path

from audit_f1tenth_source import verify_source

EXPOSED_RUNS = {
    "ex-hard-r1_2023-06-12-19-50-37.bag",
    "ex-hard-r2_2023-06-12-19-59-52.bag",
}
CMD = "/cmd_vel"
MOTOR = "/commands/motor/speed"
SERVO = "/commands/servo/position"
SERVO_ECHO = "/sensors/servo_position_command"
TOPICS = {
    CMD: "geometry_msgs/msg/Twist",
    MOTOR: "std_msgs/msg/Float64",
    SERVO: "std_msgs/msg/Float64",
    SERVO_ECHO: "std_msgs/msg/Float64",
}
PAIR_LIMIT_NS = 5_000_000


def finite(value):
    value = float(value)
    if not math.isfinite(value):
        raise ValueError("nonfinite F1TENTH command value")
    return value


def summarize(values):
    """Return bounded scalar range and exact nonzero count."""
    if not values:
        raise ValueError("empty F1TENTH command channel")
    return {
        "count": len(values),
        "minimum": min(values),
        "maximum": max(values),
        "nonzero_count": sum(value != 0.0 for value in values),
    }


def nearest_pairs(source, target, maximum_delta_ns=PAIR_LIMIT_NS):
    """Pair each source with its nearest target without altering either timestamp."""
    if any(b[0] <= a[0] for a, b in zip(source, source[1:])):
        raise ValueError("source command timestamps are not strictly increasing")
    if any(b[0] <= a[0] for a, b in zip(target, target[1:])):
        raise ValueError("target command timestamps are not strictly increasing")
    times = [row[0] for row in target]
    pairs = []
    for source_time, source_value in source:
        insertion = bisect.bisect_left(times, source_time)
        candidates = [index for index in (insertion - 1, insertion) if 0 <= index < len(target)]
        if not candidates:
            continue
        index = min(candidates, key=lambda candidate: (abs(times[candidate] - source_time), candidate))
        delta = times[index] - source_time
        if abs(delta) <= maximum_delta_ns:
            pairs.append((source_value, target[index][1], delta))
    return pairs


def affine_report(source, target):
    """Describe, but do not apply, an affine mapping over fixed nearest-time pairs."""
    pairs = nearest_pairs(source, target)
    if len(pairs) < 3:
        raise ValueError("too few paired F1TENTH command values")
    x_mean = sum(pair[0] for pair in pairs) / len(pairs)
    y_mean = sum(pair[1] for pair in pairs) / len(pairs)
    denominator = sum((pair[0] - x_mean) ** 2 for pair in pairs)
    if denominator == 0.0:
        raise ValueError("F1TENTH source command has no excitation")
    slope = sum((pair[0] - x_mean) * (pair[1] - y_mean) for pair in pairs) / denominator
    intercept = y_mean - slope * x_mean
    residuals = [pair[1] - (slope * pair[0] + intercept) for pair in pairs]
    return {
        "pair_count": len(pairs),
        "maximum_pair_delta_ns": PAIR_LIMIT_NS,
        "minimum_target_minus_source_ns": min(pair[2] for pair in pairs),
        "maximum_target_minus_source_ns": max(pair[2] for pair in pairs),
        "slope": slope,
        "intercept": intercept,
        "residual_rms": math.sqrt(sum(value * value for value in residuals) / len(residuals)),
        "maximum_absolute_residual": max(abs(value) for value in residuals),
        "mapping_applied": False,
        "calibration_qualified": False,
    }


def audit(path):
    """Decode only command paths from an already development-exposed run."""
    if path.name not in EXPOSED_RUNS:
        raise ValueError("control audit is forbidden for unexposed F1TENTH runs")
    source = verify_source(path)
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    rows = {topic: [] for topic in TOPICS}
    twist_components = {name: [] for name in ("linear_x", "linear_y", "linear_z", "angular_x", "angular_y", "angular_z")}
    with Reader(path) as reader:
        connections = [connection for connection in reader.connections if connection.topic in TOPICS]
        if {connection.topic for connection in connections} != set(TOPICS):
            raise ValueError("required F1TENTH command channels missing")
        definitions = {}
        expected = {topic: 0 for topic in TOPICS}
        for connection in connections:
            if connection.msgtype != TOPICS[connection.topic]:
                raise ValueError("unexpected F1TENTH command message type")
            definitions.update(get_types_from_msg(connection.msgdef.data, connection.msgtype))
            expected[connection.topic] += connection.msgcount
        store = get_typestore(Stores.EMPTY)
        store.register(definitions)
        for connection, bag_ns, raw in reader.messages(connections=connections):
            message = store.deserialize_ros1(raw, connection.msgtype)
            if connection.topic == CMD:
                values = {
                    "linear_x": finite(message.linear.x),
                    "linear_y": finite(message.linear.y),
                    "linear_z": finite(message.linear.z),
                    "angular_x": finite(message.angular.x),
                    "angular_y": finite(message.angular.y),
                    "angular_z": finite(message.angular.z),
                }
                for name, value in values.items():
                    twist_components[name].append(value)
                rows[CMD].append((bag_ns, values))
            else:
                rows[connection.topic].append((bag_ns, finite(message.data)))
        if any(len(rows[topic]) != expected[topic] for topic in TOPICS):
            raise ValueError("decoded F1TENTH command count differs from connection index")

    linear_x = [(time, values["linear_x"]) for time, values in rows[CMD]]
    angular_z = [(time, values["angular_z"]) for time, values in rows[CMD]]
    report = {
        "schema_version": 1,
        "kind": "rne_f1tenth_exposed_control_audit",
        "source_file": path.name,
        "source_sha256": source.sha256,
        "reader_version": importlib.metadata.version("rosbags"),
        "twist_components": {
            name: summarize(values) for name, values in sorted(twist_components.items())
        },
        "motor_command": summarize([value for _, value in rows[MOTOR]]),
        "servo_command": summarize([value for _, value in rows[SERVO]]),
        "servo_command_echo": summarize([value for _, value in rows[SERVO_ECHO]]),
        "linear_x_to_motor_command": affine_report(linear_x, rows[MOTOR]),
        "angular_z_to_servo_command": affine_report(angular_z, rows[SERVO]),
        "servo_command_to_echo": affine_report(rows[SERVO], rows[SERVO_ECHO]),
        "bag_time_used_for_pairing": True,
        "capture_clock_mapping_qualified": False,
        "steering_measurement_present": False,
        "physical_calibration_qualified": False,
        "mapping_applied": False,
    }
    canonical = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
    report["audit_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    print(json.dumps(audit(parser.parse_args().bag), indent=2, sort_keys=True, allow_nan=False))

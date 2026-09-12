"""Read-only audit of the pinned Zenodo F1TENTH run; never emits calibration.

Uses the independent rosbags ROS1 reader, outside all core crates. Run with -B
and external TEMP/TMP. Source bytes must match before parsing message definitions.
"""

import argparse
import hashlib
import importlib.metadata
import json
import math
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class SourceSpec:
    """Exact immutable identity for one admitted complete run."""

    bytes: int
    sha256: str


SOURCE_SPECS = {
    "ex-hard-r1_2023-06-12-19-50-37.bag": SourceSpec(
        92_374_784, "682fb7d5256bd2adf79c04eebbfae8e31abe559d930fcd18a260c6ee4daede81"
    ),
    "ex-hard-r2_2023-06-12-19-59-52.bag": SourceSpec(
        84_985_680, "3ba7b5c13da68227bf8af27370e7205f142dca4ca3340c8b1b51feba70cc22ac"
    ),
}
TOPICS = {
    "/cmd_vel": "geometry_msgs/msg/Twist",
    "/commands/motor/speed": "std_msgs/msg/Float64",
    "/sensors/core": "vesc_msgs/msg/VescStateStamped",
    "/vrpn_client_node/Car_2_Tracking/pose": "geometry_msgs/msg/PoseStamped",
    "/vrpn_client_node/Car_2_Tracking/twist": "geometry_msgs/msg/TwistStamped",
}


def verify_source(path, source_specs=SOURCE_SPECS):
    """Stream-hash exactly the selected immutable capture before opening a bag."""
    spec = source_specs.get(path.name)
    if spec is None:
        raise ValueError("F1TENTH run is not in the immutable allowlist")
    if not path.is_file() or path.stat().st_size != spec.bytes:
        raise ValueError("F1TENTH source byte count differs from its pinned run")
    digest = hashlib.sha256()
    count = 0
    with path.open("rb") as stream:
        while block := stream.read(1024 * 1024):
            count += len(block)
            if count > spec.bytes:
                raise ValueError("source grew beyond pinned size")
            digest.update(block)
    if count != spec.bytes or digest.hexdigest() != spec.sha256:
        raise ValueError("source hash mismatch; refusing unqualified input")
    return spec


class ChannelAudit:
    """Preserve clock inconsistencies instead of correcting or aligning them."""

    def __init__(self):
        self.count = 0
        self.last_bag = None
        self.last_header = None
        self.bag_reversals = 0
        self.header_reversals = 0
        self.header_duplicates = 0
        self.min_delta = None
        self.max_delta = None
        self.unstamped = 0
        self.zero_voltage = 0
        self.fault_outside_declared = 0

    def add(self, bag_ns, header_ns=None, voltage=None, fault=None):
        if not isinstance(bag_ns, int) or bag_ns < 0:
            raise ValueError("invalid bag timestamp")
        if header_ns is not None and (not isinstance(header_ns, int) or header_ns < 0):
            raise ValueError("invalid header timestamp")
        if voltage is not None and not math.isfinite(voltage):
            raise ValueError("nonfinite telemetry")
        self.count += 1
        if self.last_bag is not None:
            self.bag_reversals += bag_ns < self.last_bag
        self.last_bag = bag_ns
        if header_ns is None:
            self.unstamped += 1
        else:
            if self.last_header is not None:
                self.header_reversals += header_ns < self.last_header
                self.header_duplicates += header_ns == self.last_header
            self.last_header = header_ns
            delta = bag_ns - header_ns
            self.min_delta = delta if self.min_delta is None else min(self.min_delta, delta)
            self.max_delta = delta if self.max_delta is None else max(self.max_delta, delta)
        self.zero_voltage += voltage == 0 if voltage is not None else 0
        self.fault_outside_declared += fault not in range(7) if fault is not None else 0

    def report(self):
        return {
            "sample_count": self.count,
            "bag_time_reversals": self.bag_reversals,
            "header_time_reversals": self.header_reversals,
            "header_time_duplicates": self.header_duplicates,
            "unstamped_samples": self.unstamped,
            "min_bag_minus_header_ns": self.min_delta,
            "max_bag_minus_header_ns": self.max_delta,
            "zero_input_voltage_samples": self.zero_voltage,
            "fault_outside_declared_samples": self.fault_outside_declared,
        }


def audit(path):
    source = verify_source(path)
    # Delayed import keeps synthetic contract tests independent of ROS tooling.
    from rosbags.rosbag1 import Reader
    from rosbags.typesys import Stores, get_typestore, get_types_from_msg

    store = get_typestore(Stores.EMPTY)
    channels = {topic: ChannelAudit() for topic in TOPICS}
    with Reader(path) as reader:
        connections = [c for c in reader.connections if c.topic in TOPICS]
        if {c.topic for c in connections} != set(TOPICS):
            raise ValueError("required source channels missing")
        definitions = {}
        expected = {topic: 0 for topic in TOPICS}
        for connection in connections:
            if connection.msgtype != TOPICS[connection.topic]:
                raise ValueError("unexpected message type")
            definitions.update(get_types_from_msg(connection.msgdef.data, connection.msgtype))
            expected[connection.topic] += connection.msgcount
        store.register(definitions)
        for connection, bag_ns, raw in reader.messages(connections=connections):
            msg = store.deserialize_ros1(raw, connection.msgtype)
            header_ns = None
            if hasattr(msg, "header"):
                stamp = msg.header.stamp
                if not 0 <= stamp.nanosec < 1_000_000_000:
                    raise ValueError("invalid header nanoseconds")
                header_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            state = msg.state if connection.topic == "/sensors/core" else None
            channels[connection.topic].add(
                bag_ns, header_ns,
                state.voltage_input if state is not None else None,
                state.fault_code if state is not None else None,
            )
        if any(channels[t].count != expected[t] for t in TOPICS):
            raise ValueError("decoded counts differ from connection index")
    report = {
        "schema_version": 1,
        "kind": "rne_f1tenth_source_audit",
        "source_file": path.name,
        "source_sha256": source.sha256,
        "source_bytes": source.bytes,
        "reader_version": importlib.metadata.version("rosbags"),
        "channels": {t: channels[t].report() for t in sorted(channels)},
        "physical_accuracy_validated": False,
        "clock_mapping_qualified": False,
        "motor_electrical_identification_qualified": False,
        "angular_velocity_reference_qualified": False,
        "timestamp_adjustments_applied": False,
    }
    canonical = json.dumps(report, sort_keys=True, separators=(",", ":"), allow_nan=False)
    report["audit_sha256"] = hashlib.sha256(canonical.encode()).hexdigest()
    return report


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    args = parser.parse_args()
    print(json.dumps(audit(args.bag), indent=2, sort_keys=True, allow_nan=False))

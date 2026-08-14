#!/usr/bin/env python3
"""Bounded RNE wire-v1 device bridge for LeRobot LeKiwi v0.6.0.

The bridge runs beside the physical device, normally on the LeKiwi Raspberry
Pi. Standard output is reserved for one strict JSON response per host request;
diagnostics go to standard error. The numerical wire carries only the nine
reference-profile state values and three base commands. Cameras remain on the
RNE dataset path.
"""

from __future__ import annotations

import argparse
import importlib.metadata
import json
import math
import re
import sys
import threading
import time
from dataclasses import dataclass
from typing import Any, Protocol

WIRE_KIND_HOST = "rne_hardware_host_frame"
WIRE_KIND_DEVICE = "rne_hardware_device_frame"
WIRE_SCHEMA_VERSION = 1
BRIDGE_SCHEMA_VERSION = 1
TASK_ID = "rne.lekiwi_so101.base_shadow.v1"
DEVICE_ID = "rne.lekiwi_so101.bridge.v1"
OBSERVATION_WIDTH = 9
ACTION_WIDTH = 3
MAX_FRAME_BYTES = 64 * 1024
MAX_LINEAR_SPEED_M_S = 0.1
MAX_ANGULAR_SPEED_RAD_S = math.pi / 6.0
DEFAULT_WATCHDOG_TIMEOUT_MS = 500
ARM_KEYS = (
    "arm_shoulder_pan.pos",
    "arm_shoulder_lift.pos",
    "arm_elbow_flex.pos",
    "arm_wrist_flex.pos",
    "arm_wrist_roll.pos",
    "arm_gripper.pos",
)
STATE_KEYS = ARM_KEYS + ("x.vel", "y.vel", "theta.vel")
IDENTIFIER = re.compile(r"^[a-z0-9._/-]+$")


class RobotBackend(Protocol):
    """Small pinned surface consumed from LeRobot or the deterministic mock."""

    def connect(self) -> None: ...

    def get_observation(self) -> dict[str, Any]: ...

    def send_action(self, action: dict[str, float]) -> dict[str, float]: ...

    def stop_base(self) -> None: ...

    def disconnect(self) -> None: ...


class BridgeProtocolError(Exception):
    """A host request violated a stable wire invariant."""

    def __init__(self, code: str):
        super().__init__(code)
        self.code = code


class FatalSafetyError(Exception):
    """The process could not confirm a physical base stop."""


def _exact_keys(value: dict[str, Any], expected: set[str]) -> bool:
    return set(value) == expected


def _is_uint(value: Any) -> bool:
    return isinstance(value, int) and not isinstance(value, bool) and value >= 0


def _is_finite_number(value: Any) -> bool:
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def _validate_identifier(value: Any) -> bool:
    return isinstance(value, str) and IDENTIFIER.fullmatch(value) is not None


def _validate_request(frame: Any) -> dict[str, Any]:
    if not isinstance(frame, dict) or not _exact_keys(
        frame, {"kind", "schema_version", "session_id", "sequence", "payload"}
    ):
        raise BridgeProtocolError("protocol_violation")
    if frame["kind"] != WIRE_KIND_HOST or frame["schema_version"] != WIRE_SCHEMA_VERSION:
        raise BridgeProtocolError("protocol_violation")
    if not _validate_identifier(frame["session_id"]) or not _is_uint(frame["sequence"]):
        raise BridgeProtocolError("protocol_violation")
    payload = frame["payload"]
    if not isinstance(payload, dict) or not isinstance(payload.get("type"), str):
        raise BridgeProtocolError("protocol_violation")
    kind = payload["type"]
    if kind == "open":
        if not _exact_keys(
            payload,
            {"type", "task_id", "mode", "observation_width", "action_width"},
        ):
            raise BridgeProtocolError("protocol_violation")
        if (
            not _validate_identifier(payload["task_id"])
            or not isinstance(payload["mode"], str)
            or not _is_uint(payload["observation_width"])
            or not _is_uint(payload["action_width"])
        ):
            raise BridgeProtocolError("protocol_violation")
    elif kind in {"poll_observation", "close"}:
        if not _exact_keys(payload, {"type"}):
            raise BridgeProtocolError("protocol_violation")
    elif kind == "actuate":
        if not _exact_keys(payload, {"type", "frame"}) or not isinstance(payload["frame"], dict):
            raise BridgeProtocolError("protocol_violation")
        actuation = payload["frame"]
        if not _exact_keys(
            actuation,
            {
                "action_sequence",
                "queued_at_ms",
                "values",
                "safety_stop",
                "reason",
            },
        ):
            raise BridgeProtocolError("protocol_violation")
        sequence = actuation["action_sequence"]
        if sequence is not None and not _is_uint(sequence):
            raise BridgeProtocolError("protocol_violation")
        if (
            not _is_uint(actuation["queued_at_ms"])
            or not isinstance(actuation["values"], list)
            or not isinstance(actuation["safety_stop"], bool)
            or (actuation["reason"] is not None and not isinstance(actuation["reason"], str))
            or any(not _is_finite_number(value) for value in actuation["values"])
        ):
            raise BridgeProtocolError("protocol_violation")
        if actuation["safety_stop"]:
            if sequence is not None or actuation["reason"] is None:
                raise BridgeProtocolError("protocol_violation")
        elif sequence is None or actuation["reason"] is not None:
            raise BridgeProtocolError("protocol_violation")
    else:
        raise BridgeProtocolError("protocol_violation")
    return frame


def _response(frame: dict[str, Any], payload: dict[str, Any]) -> dict[str, Any]:
    return {
        "kind": WIRE_KIND_DEVICE,
        "schema_version": WIRE_SCHEMA_VERSION,
        "session_id": frame["session_id"],
        "request_sequence": frame["sequence"],
        "payload": payload,
    }


class DeviceWatchdog:
    """Independent real-time base-stop watchdog owned by the adapter process."""

    def __init__(
        self,
        robot: RobotBackend,
        robot_lock: threading.RLock,
        timeout_ms: int,
    ):
        if timeout_ms <= 0:
            raise ValueError("watchdog timeout must be positive")
        self._robot = robot
        self._robot_lock = robot_lock
        self._timeout_ns = timeout_ms * 1_000_000
        self._condition = threading.Condition()
        self._deadline_ns: int | None = None
        self._tripped = False
        self._failed = False
        self._stop = False
        self._thread = threading.Thread(target=self._run, name="rne-lekiwi-watchdog", daemon=True)
        self._thread.start()

    @property
    def tripped(self) -> bool:
        with self._condition:
            return self._tripped

    @property
    def failed(self) -> bool:
        with self._condition:
            return self._failed

    def arm(self) -> None:
        with self._condition:
            self._tripped = False
            self._deadline_ns = time.monotonic_ns() + self._timeout_ns
            self._condition.notify_all()

    def refresh(self) -> None:
        with self._condition:
            if self._deadline_ns is not None and not self._tripped:
                self._deadline_ns = time.monotonic_ns() + self._timeout_ns
                self._condition.notify_all()

    def disarm(self) -> None:
        with self._condition:
            self._deadline_ns = None
            self._condition.notify_all()

    def close(self) -> None:
        with self._condition:
            self._stop = True
            self._deadline_ns = None
            self._condition.notify_all()
        self._thread.join(timeout=2.0)

    def _run(self) -> None:
        while True:
            with self._condition:
                if self._stop:
                    return
                deadline = self._deadline_ns
                if deadline is None:
                    self._condition.wait()
                    continue
                remaining_ns = deadline - time.monotonic_ns()
                if remaining_ns > 0:
                    self._condition.wait(remaining_ns / 1_000_000_000)
                    continue
                self._deadline_ns = None
            try:
                with self._robot_lock:
                    self._robot.stop_base()
            except Exception as error:
                print(f"watchdog failed to stop LeKiwi base: {error}", file=sys.stderr)
                with self._condition:
                    self._failed = True
                    self._stop = True
                return
            with self._condition:
                self._tripped = True


@dataclass
class DeviceBridge:
    """One strict RNE session over a concrete robot backend."""

    robot: RobotBackend
    watchdog_timeout_ms: int = DEFAULT_WATCHDOG_TIMEOUT_MS

    def __post_init__(self) -> None:
        self._robot_lock = threading.RLock()
        self._watchdog = DeviceWatchdog(self.robot, self._robot_lock, self.watchdog_timeout_ms)
        self._open = False
        self._terminal = False
        self._connected = False
        self._session_id: str | None = None
        self._mode: str | None = None
        self._last_request_sequence: int | None = None
        self._observation_sequence = 0
        self._arm_hold_vendor: list[float] | None = None

    @property
    def terminal(self) -> bool:
        return self._terminal

    def handle(self, frame: dict[str, Any]) -> dict[str, Any]:
        if self._terminal:
            return _response(frame, {"type": "rejected", "code": "terminal_state"})
        if self._last_request_sequence is not None and frame["sequence"] <= self._last_request_sequence:
            return _response(frame, {"type": "rejected", "code": "non_monotonic_sequence"})
        if self._session_id is not None and frame["session_id"] != self._session_id:
            return _response(frame, {"type": "rejected", "code": "session_mismatch"})
        self._last_request_sequence = frame["sequence"]

        if self._watchdog.failed:
            raise FatalSafetyError("device watchdog could not confirm a base stop")
        if self._watchdog.tripped and frame["payload"]["type"] != "close":
            self._terminal = True
            return _response(
                frame,
                {
                    "type": "safety_signal",
                    "reason": "command_stale",
                    "safe_stop_applied": True,
                },
            )
        try:
            return self._handle_checked(frame)
        except BridgeProtocolError as error:
            return _response(frame, {"type": "rejected", "code": error.code})
        except Exception as error:
            print(f"LeKiwi transport fault: {error}", file=sys.stderr)
            if not self._safe_stop():
                raise FatalSafetyError("could not confirm base stop") from error
            self._terminal = True
            return _response(
                frame,
                {
                    "type": "disconnected",
                    "reason": "transport_fault",
                    "safe_stop_applied": True,
                },
            )

    def _handle_checked(self, frame: dict[str, Any]) -> dict[str, Any]:
        payload = frame["payload"]
        kind = payload["type"]
        if kind == "open":
            if self._open:
                raise BridgeProtocolError("already_open")
            if payload["mode"] not in {"shadow", "hil", "live"}:
                raise BridgeProtocolError("unsupported_mode")
            if (
                payload["task_id"] != TASK_ID
                or payload["observation_width"] != OBSERVATION_WIDTH
                or payload["action_width"] != ACTION_WIDTH
            ):
                raise BridgeProtocolError("width_mismatch")
            with self._robot_lock:
                self.robot.connect()
                self._connected = True
                self.robot.stop_base()
            self._session_id = frame["session_id"]
            self._mode = payload["mode"]
            self._open = True
            if self._mode in {"hil", "live"}:
                self._watchdog.arm()
            return _response(
                frame,
                {
                    "type": "ready",
                    "device_id": DEVICE_ID,
                    "task_id": TASK_ID,
                    "observation_width": OBSERVATION_WIDTH,
                    "action_width": ACTION_WIDTH,
                },
            )
        if not self._open:
            raise BridgeProtocolError("not_open")
        if kind == "poll_observation":
            return self._poll_observation(frame)
        if kind == "actuate":
            return self._actuate(frame, payload["frame"])
        if kind == "close":
            self._safe_close()
            self._terminal = True
            return _response(frame, {"type": "closed"})
        raise BridgeProtocolError("protocol_violation")

    def _poll_observation(self, frame: dict[str, Any]) -> dict[str, Any]:
        with self._robot_lock:
            observation = self.robot.get_observation()
        vendor_values = [observation[key] for key in STATE_KEYS]
        if any(not _is_finite_number(value) for value in vendor_values):
            raise ValueError("non-finite LeKiwi observation")
        if not 0.0 <= float(vendor_values[5]) <= 100.0:
            raise ValueError("LeKiwi gripper observation outside [0, 100]")
        self._arm_hold_vendor = [float(value) for value in vendor_values[:6]]
        values = [
            *(float(value) * math.pi / 180.0 for value in vendor_values[:5]),
            float(vendor_values[5]),
            float(vendor_values[6]),
            float(vendor_values[7]),
            float(vendor_values[8]) * math.pi / 180.0,
        ]
        self._observation_sequence += 1
        return _response(
            frame,
            {
                "type": "observation",
                "sequence": self._observation_sequence,
                "values": values,
            },
        )

    def _actuate(self, frame: dict[str, Any], actuation: dict[str, Any]) -> dict[str, Any]:
        if actuation["safety_stop"]:
            if len(actuation["values"]) != ACTION_WIDTH or any(
                float(value) != 0.0 for value in actuation["values"]
            ):
                raise BridgeProtocolError("width_mismatch")
            if not self._safe_stop():
                raise FatalSafetyError("could not confirm requested base stop")
            self._watchdog.disarm()
            return _response(
                frame,
                {
                    "type": "actuation_accepted",
                    "action_sequence": None,
                    "safety_stop": True,
                },
            )
        if self._mode == "shadow":
            raise BridgeProtocolError("authority_denied")
        values = actuation["values"]
        if len(values) != ACTION_WIDTH:
            raise BridgeProtocolError("width_mismatch")
        if self._arm_hold_vendor is None:
            raise BridgeProtocolError("terminal_state")
        x_m_s, y_m_s, yaw_rad_s = (float(value) for value in values)
        if (
            abs(x_m_s) > MAX_LINEAR_SPEED_M_S
            or abs(y_m_s) > MAX_LINEAR_SPEED_M_S
            or abs(yaw_rad_s) > MAX_ANGULAR_SPEED_RAD_S
        ):
            if not self._safe_stop():
                raise FatalSafetyError("could not stop after actuator limit")
            self._watchdog.disarm()
            self._terminal = True
            return _response(
                frame,
                {
                    "type": "safety_signal",
                    "reason": "actuator_limit",
                    "safe_stop_applied": True,
                },
            )
        action = dict(zip(ARM_KEYS, self._arm_hold_vendor, strict=True))
        action.update(
            {
                "x.vel": x_m_s,
                "y.vel": y_m_s,
                "theta.vel": yaw_rad_s * 180.0 / math.pi,
            }
        )
        with self._robot_lock:
            self.robot.send_action(action)
        self._watchdog.refresh()
        return _response(
            frame,
            {
                "type": "actuation_accepted",
                "action_sequence": actuation["action_sequence"],
                "safety_stop": False,
            },
        )

    def _safe_stop(self) -> bool:
        if not self._connected:
            return True
        try:
            with self._robot_lock:
                self.robot.stop_base()
            return True
        except Exception as error:
            print(f"LeKiwi base stop failed: {error}", file=sys.stderr)
            return False

    def _safe_close(self) -> None:
        self._watchdog.disarm()
        if not self._connected:
            return
        if not self._safe_stop():
            raise FatalSafetyError("could not stop base before disconnect")
        with self._robot_lock:
            self.robot.disconnect()
        self._connected = False
        self._open = False

    def close(self) -> None:
        self._watchdog.close()
        if self._connected:
            try:
                self._safe_close()
            except Exception as error:
                print(f"LeKiwi shutdown failed: {error}", file=sys.stderr)


class MockLeKiwi:
    """Dependency-free deterministic backend used by process conformance tests."""

    def __init__(self) -> None:
        self.connected = False
        self.actions: list[dict[str, float]] = []
        self.stop_count = 0

    def connect(self) -> None:
        self.connected = True

    def get_observation(self) -> dict[str, float]:
        if not self.connected:
            raise RuntimeError("mock is disconnected")
        return {
            "arm_shoulder_pan.pos": 10.0,
            "arm_shoulder_lift.pos": -20.0,
            "arm_elbow_flex.pos": 30.0,
            "arm_wrist_flex.pos": -40.0,
            "arm_wrist_roll.pos": 50.0,
            "arm_gripper.pos": 60.0,
            "x.vel": 0.0,
            "y.vel": 0.0,
            "theta.vel": 0.0,
        }

    def send_action(self, action: dict[str, float]) -> dict[str, float]:
        if not self.connected:
            raise RuntimeError("mock is disconnected")
        self.actions.append(dict(action))
        return dict(action)

    def stop_base(self) -> None:
        self.stop_count += 1

    def disconnect(self) -> None:
        self.connected = False


def _make_real_robot(args: argparse.Namespace) -> RobotBackend:
    installed = importlib.metadata.version("lerobot")
    if installed != "0.6.0":
        raise RuntimeError(f"reference bridge requires lerobot==0.6.0, found {installed}")
    from lerobot.robots.lekiwi.config_lekiwi import LeKiwiConfig
    from lerobot.robots.lekiwi.lekiwi import LeKiwi

    config = LeKiwiConfig(
        id=args.robot_id,
        port=args.port,
        cameras={},
        disable_torque_on_disconnect=True,
        max_relative_target=None,
        use_degrees=True,
    )
    return LeKiwi(config)


def _write_response(response: dict[str, Any]) -> None:
    encoded = json.dumps(
        response,
        ensure_ascii=True,
        allow_nan=False,
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"
    if len(encoded) > MAX_FRAME_BYTES:
        raise RuntimeError("device response exceeds wire bound")
    sys.stdout.buffer.write(encoded)
    sys.stdout.buffer.flush()


def _run(bridge: DeviceBridge) -> int:
    try:
        while True:
            line = sys.stdin.buffer.readline(MAX_FRAME_BYTES + 1)
            if not line:
                return 0
            if len(line) > MAX_FRAME_BYTES or not line.endswith(b"\n"):
                raise BridgeProtocolError("protocol_violation")
            if b"\n" in line[:-1] or b"\r" in line[:-1]:
                raise BridgeProtocolError("protocol_violation")
            try:
                raw = json.loads(line[:-1])
                frame = _validate_request(raw)
            except (json.JSONDecodeError, UnicodeDecodeError, BridgeProtocolError) as error:
                print(f"invalid RNE hardware frame: {error}", file=sys.stderr)
                return 2
            _write_response(bridge.handle(frame))
            if bridge.terminal:
                return 0
    except FatalSafetyError as error:
        print(f"fatal LeKiwi safety failure: {error}", file=sys.stderr)
        return 3
    finally:
        bridge.close()


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mock", action="store_true", help="use deterministic no-hardware backend")
    parser.add_argument("--robot-id", default="rne_lekiwi_reference")
    parser.add_argument("--port", default="/dev/ttyACM0")
    parser.add_argument(
        "--mock-watchdog-timeout-ms",
        type=int,
        default=DEFAULT_WATCHDOG_TIMEOUT_MS,
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()
    if args.mock_watchdog_timeout_ms <= 0:
        parser.error("--mock-watchdog-timeout-ms must be positive")
    return args


def main() -> int:
    """Runs one bounded device process until close, terminal safety, or EOF."""
    args = _parse_args()
    try:
        robot = MockLeKiwi() if args.mock else _make_real_robot(args)
    except Exception as error:
        print(f"failed to initialize LeKiwi backend: {error}", file=sys.stderr)
        return 1
    timeout_ms = (
        args.mock_watchdog_timeout_ms if args.mock else DEFAULT_WATCHDOG_TIMEOUT_MS
    )
    return _run(DeviceBridge(robot=robot, watchdog_timeout_ms=timeout_ms))


if __name__ == "__main__":
    raise SystemExit(main())

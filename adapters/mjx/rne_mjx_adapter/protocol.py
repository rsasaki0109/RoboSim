"""Versioned wire contract shared by the MJX-Warp server and its tests."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

ADAPTER_ID = "mjx_warp"
RUNTIME_ID = "mujoco_mjx_warp"
PROTOCOL_SCHEMA_VERSION = 1
CAPABILITY_REPORT_SCHEMA_VERSION = 1
CONFORMANCE_REPORT_SCHEMA_VERSION = 1
SCALE_REPORT_SCHEMA_VERSION = 1
TASK_SPEC_SCHEMA_VERSION = 1
BATCH_CHECKPOINT_SCHEMA_VERSION = 2
TASK_SPEC_KIND = "rne_task_spec"
REQUEST_KIND = "rne_accelerator_request"
RESPONSE_KIND = "rne_accelerator_response"
CAPABILITY_REPORT_KIND = "rne_accelerator_capability_report"
CONFORMANCE_REPORT_KIND = "rne_accelerator_conformance_report"
SCALE_REPORT_KIND = "rne_accelerator_scale_report"
SEED_STRATEGY = "split_mix64_lane_episode_v1"
SUPPORTED_BATCH_WIDTHS = (1, 16, 256, 4096)
MAX_REQUEST_BYTES = 16 * 1024 * 1024
MASK64 = (1 << 64) - 1


class ProtocolError(Exception):
    """Stable machine-readable protocol failure."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        details: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.message = message
        self.details = details or {}

    def as_object(self) -> dict[str, Any]:
        """Returns the stable JSON error payload."""

        return {
            "code": self.code,
            "message": self.message,
            "details": self.details,
        }


def canonical_json(value: Any) -> str:
    """Serializes a protocol value without NaN, whitespace, or key-order drift."""

    try:
        return json.dumps(
            value,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError) as error:
        raise ProtocolError(
            "non_canonical_json",
            "protocol value is not finite canonical JSON",
        ) from error


def parse_request_line(line: str) -> dict[str, Any]:
    """Parses one bounded JSONL request and validates its common envelope."""

    encoded_length = len(line.encode("utf-8"))
    if encoded_length == 0:
        raise ProtocolError("empty_request", "request line must not be empty")
    if encoded_length > MAX_REQUEST_BYTES:
        raise ProtocolError(
            "request_too_large",
            f"request exceeds {MAX_REQUEST_BYTES} bytes",
            details={"actual_bytes": encoded_length, "maximum_bytes": MAX_REQUEST_BYTES},
        )
    try:
        request = json.loads(line, parse_constant=_reject_json_constant)
    except (json.JSONDecodeError, ProtocolError) as error:
        if isinstance(error, ProtocolError):
            raise
        raise ProtocolError("invalid_json", "request is not valid JSON") from error
    if not isinstance(request, dict):
        raise ProtocolError("invalid_request", "request must be a JSON object")
    if request.get("kind") != REQUEST_KIND:
        raise ProtocolError(
            "request_kind_mismatch",
            f"request kind must be {REQUEST_KIND}",
        )
    if request.get("schema_version") != PROTOCOL_SCHEMA_VERSION:
        raise ProtocolError(
            "protocol_version_mismatch",
            f"protocol schema must be {PROTOCOL_SCHEMA_VERSION}",
            details={"supported_schema_version": PROTOCOL_SCHEMA_VERSION},
        )
    request_id = request.get("request_id")
    require_unsigned(request_id, "request_id")
    operation = request.get("operation")
    if not isinstance(operation, str) or not operation:
        raise ProtocolError("invalid_request", "operation must be a non-empty string")
    return request


def success_response(request_id: int, result: Any) -> dict[str, Any]:
    """Builds a successful protocol response."""

    return {
        "kind": RESPONSE_KIND,
        "schema_version": PROTOCOL_SCHEMA_VERSION,
        "request_id": request_id,
        "ok": True,
        "result": result,
    }


def error_response(request_id: int | None, error: ProtocolError) -> dict[str, Any]:
    """Builds a failed protocol response without exposing a Python traceback."""

    return {
        "kind": RESPONSE_KIND,
        "schema_version": PROTOCOL_SCHEMA_VERSION,
        "request_id": request_id,
        "ok": False,
        "error": error.as_object(),
    }


def require_unsigned(value: Any, field: str, maximum: int = MASK64) -> int:
    """Returns a bounded unsigned integer or raises a stable protocol error."""

    if isinstance(value, bool) or not isinstance(value, int) or not 0 <= value <= maximum:
        raise ProtocolError(
            "invalid_request",
            f"{field} must be an integer in 0..={maximum}",
        )
    return value


def require_exact_keys(
    value: dict[str, Any],
    *,
    required: set[str],
    optional: set[str] = frozenset(),
    context: str,
) -> None:
    """Rejects missing and unknown object fields."""

    missing = sorted(required - value.keys())
    unknown = sorted(value.keys() - required - optional)
    if missing or unknown:
        raise ProtocolError(
            "invalid_request",
            f"{context} fields do not match protocol",
            details={"missing": missing, "unknown": unknown},
        )


def validate_task_spec(task_spec: Any) -> dict[str, Any]:
    """Validates the portable fields required at the accelerator boundary."""

    if not isinstance(task_spec, dict):
        raise ProtocolError("task_spec_invalid", "task_spec must be an object")
    required = {
        "kind",
        "schema_version",
        "task_id",
        "control_step_s",
        "observation",
        "action",
        "reward",
        "termination",
        "reset",
        "curriculum",
        "randomization",
    }
    require_exact_keys(task_spec, required=required, context="task_spec")
    if task_spec["kind"] != TASK_SPEC_KIND:
        raise ProtocolError("task_spec_invalid", f"task_spec kind must be {TASK_SPEC_KIND}")
    if task_spec["schema_version"] != TASK_SPEC_SCHEMA_VERSION:
        raise ProtocolError(
            "task_spec_version_mismatch",
            f"TaskSpec schema must be {TASK_SPEC_SCHEMA_VERSION}",
        )
    if not isinstance(task_spec["task_id"], str) or not task_spec["task_id"]:
        raise ProtocolError("task_spec_invalid", "task_id must be a non-empty string")
    control_step_s = task_spec["control_step_s"]
    if (
        isinstance(control_step_s, bool)
        or not isinstance(control_step_s, (int, float))
        or not 0.0 < float(control_step_s) < float("inf")
    ):
        raise ProtocolError("task_spec_invalid", "control_step_s must be finite and positive")
    _validate_tensor_space(task_spec["observation"], "observation")
    _validate_tensor_space(task_spec["action"], "action")
    reset = task_spec["reset"]
    if not isinstance(reset, dict) or reset.get("seed_strategy") != SEED_STRATEGY:
        raise ProtocolError(
            "task_spec_invalid",
            f"reset.seed_strategy must be {SEED_STRATEGY}",
        )
    if not isinstance(reset.get("supports_partial_reset"), bool):
        raise ProtocolError(
            "task_spec_invalid",
            "reset.supports_partial_reset must be boolean",
        )
    canonical_json(task_spec)
    return task_spec


def validate_bound_task_spec(task_spec: Any, expected: dict[str, Any]) -> dict[str, Any]:
    """Requires byte-equivalent canonical TaskSpec semantics for a binding."""

    validated = validate_task_spec(task_spec)
    validate_task_spec(expected)
    if canonical_json(validated) != canonical_json(expected):
        raise ProtocolError(
            "task_spec_mismatch",
            "TaskSpec does not exactly match the selected task binding",
            details={
                "actual_task_id": validated["task_id"],
                "expected_task_id": expected["task_id"],
            },
        )
    return validated


def load_json_fixture(path: Path) -> dict[str, Any]:
    """Loads a repository-owned finite JSON fixture."""

    try:
        with path.open("r", encoding="utf-8") as source:
            value = json.load(source, parse_constant=_reject_json_constant)
    except (OSError, json.JSONDecodeError) as error:
        raise ProtocolError(
            "fixture_invalid",
            f"failed to load fixture {path.name}",
        ) from error
    if not isinstance(value, dict):
        raise ProtocolError("fixture_invalid", f"fixture {path.name} must be an object")
    return value


def derive_episode_seed(root_seed: int, lane_id: int, episode_index: int) -> int:
    """Matches RNE's domain-separated SplitMix64 lane/episode derivation."""

    require_unsigned(root_seed, "root_seed")
    require_unsigned(lane_id, "lane_id")
    require_unsigned(episode_index, "episode_index")
    lane = _splitmix64(lane_id ^ 0x524E_452D_4C41_4E45)
    episode = _splitmix64(episode_index ^ 0x524E_452D_4550_4953)
    return _splitmix64(root_seed ^ lane ^ episode)


def fnv1a64(value: Any) -> int:
    """Hashes canonical JSON for same-build diagnostic replay evidence."""

    digest = 0xCBF2_9CE4_8422_2325
    for byte in canonical_json(value).encode("utf-8"):
        digest ^= byte
        digest = (digest * 0x0000_0100_0000_01B3) & MASK64
    return digest


def _validate_tensor_space(space: Any, context: str) -> None:
    if not isinstance(space, dict) or set(space) != {"tensors"}:
        raise ProtocolError("task_spec_invalid", f"{context} must contain only tensors")
    tensors = space["tensors"]
    if not isinstance(tensors, list) or not tensors:
        raise ProtocolError("task_spec_invalid", f"{context}.tensors must be non-empty")
    names: set[str] = set()
    required = {"name", "dtype", "shape", "unit", "layout", "bounds"}
    for index, tensor in enumerate(tensors):
        if not isinstance(tensor, dict) or set(tensor) != required:
            raise ProtocolError(
                "task_spec_invalid",
                f"{context}.tensors[{index}] fields do not match schema",
            )
        name = tensor["name"]
        if not isinstance(name, str) or not name or name in names:
            raise ProtocolError(
                "task_spec_invalid",
                f"{context}.tensors[{index}].name must be unique and non-empty",
            )
        names.add(name)
        if tensor["dtype"] not in {"f32", "f64", "i32", "i64", "u8", "bool"}:
            raise ProtocolError("task_spec_invalid", f"unsupported dtype for tensor {name}")
        if tensor["layout"] != "row_major":
            raise ProtocolError("task_spec_invalid", f"unsupported layout for tensor {name}")
        if not isinstance(tensor["shape"], list) or any(
            isinstance(dimension, bool) or not isinstance(dimension, int) or dimension <= 0
            for dimension in tensor["shape"]
        ):
            raise ProtocolError("task_spec_invalid", f"invalid shape for tensor {name}")
        if not isinstance(tensor["unit"], str) or not tensor["unit"]:
            raise ProtocolError("task_spec_invalid", f"unit is required for tensor {name}")


def _splitmix64(value: int) -> int:
    value = (value + 0x9E37_79B9_7F4A_7C15) & MASK64
    value = ((value ^ (value >> 30)) * 0xBF58_476D_1CE4_E5B9) & MASK64
    value = ((value ^ (value >> 27)) * 0x94D0_49BB_1331_11EB) & MASK64
    return (value ^ (value >> 31)) & MASK64


def _reject_json_constant(value: str) -> None:
    raise ProtocolError("invalid_json", f"non-finite JSON number {value} is forbidden")

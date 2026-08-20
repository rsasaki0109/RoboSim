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


def validate_capability_report(report: Any) -> dict[str, Any]:
    """Validates capability identity, status semantics, and exact runtime pins."""

    if not isinstance(report, dict):
        raise ProtocolError("report_invalid", "capability report must be an object")
    required = {
        "kind",
        "schema_version",
        "adapter_id",
        "runtime_id",
        "status",
        "unavailable_reason_code",
        "execution_boundary",
        "precision",
        "protocol_schema",
        "task_spec_schema",
        "batch_checkpoint_schema",
        "conformance_report_schema",
        "scale_report_schema",
        "supported_task_ids",
        "supported_batch_widths",
        "requires_nvidia_gpu",
        "unsupported_features",
        "runtime",
        "runtime_contract",
        "runtime_contract_schema",
    }
    require_exact_keys(report, required=required, context="capability report")
    if (
        report["kind"] != CAPABILITY_REPORT_KIND
        or report["schema_version"] != CAPABILITY_REPORT_SCHEMA_VERSION
        or report["adapter_id"] != ADAPTER_ID
        or report["runtime_id"] != RUNTIME_ID
        or report["execution_boundary"] != "out_of_process_python"
        or report["precision"] != "f64"
        or report["protocol_schema"] != PROTOCOL_SCHEMA_VERSION
        or report["task_spec_schema"] != TASK_SPEC_SCHEMA_VERSION
        or report["batch_checkpoint_schema"] != BATCH_CHECKPOINT_SCHEMA_VERSION
        or report["conformance_report_schema"] != CONFORMANCE_REPORT_SCHEMA_VERSION
        or report["scale_report_schema"] != SCALE_REPORT_SCHEMA_VERSION
        or report["supported_task_ids"] != ["rne.physics.free_fall.mjx.v1"]
        or report["supported_batch_widths"] != list(SUPPORTED_BATCH_WIDTHS)
        or report["requires_nvidia_gpu"] is not True
    ):
        raise ProtocolError("report_invalid", "capability contract identity mismatch")
    unsupported = report["unsupported_features"]
    if (
        not isinstance(unsupported, list)
        or not unsupported
        or unsupported != sorted(set(unsupported))
        or any(not isinstance(feature, str) or not feature for feature in unsupported)
    ):
        raise ProtocolError("report_invalid", "unsupported features are not canonical")

    runtime_contract = report["runtime_contract"]
    runtime_contract_fields = {
        "schema_version",
        "operating_system",
        "architecture",
        "python",
        "cuda_major",
        "nvidia_driver_minimum",
        "packages",
        "official_sources",
    }
    if not isinstance(runtime_contract, dict):
        raise ProtocolError("report_invalid", "runtime contract must be an object")
    require_exact_keys(
        runtime_contract,
        required=runtime_contract_fields,
        context="runtime contract",
    )
    package_fields = {
        "jax",
        "jaxlib",
        "jax_cuda_plugin",
        "mujoco",
        "mujoco_mjx",
        "warp_lang",
    }
    packages = runtime_contract["packages"]
    if not isinstance(packages, dict):
        raise ProtocolError("report_invalid", "runtime packages must be an object")
    require_exact_keys(packages, required=package_fields, context="runtime packages")
    if (
        report["runtime_contract_schema"] != 1
        or runtime_contract["schema_version"] != 1
        or runtime_contract["operating_system"] != "linux"
        or runtime_contract["architecture"] != "x86_64"
        or runtime_contract["python"] != "3.12"
        or runtime_contract["cuda_major"] != 13
        or runtime_contract["nvidia_driver_minimum"] != 580
        or not isinstance(runtime_contract["official_sources"], list)
        or not runtime_contract["official_sources"]
        or any(
            not isinstance(source, str) or not source.startswith("https://")
            for source in runtime_contract["official_sources"]
        )
    ):
        raise ProtocolError("report_invalid", "runtime contract is invalid")

    runtime = report["runtime"]
    runtime_fields = {
        "python_version",
        "platform",
        "machine",
        "jax_version",
        "jaxlib_version",
        "jax_cuda_plugin_version",
        "mujoco_version",
        "mujoco_mjx_version",
        "warp_version",
        "jax_backend",
        "jax_devices",
        "nvidia_driver_version",
    }
    if not isinstance(runtime, dict):
        raise ProtocolError("report_invalid", "runtime probe must be an object")
    require_exact_keys(runtime, required=runtime_fields, context="runtime probe")
    for field in ("python_version", "platform", "machine"):
        if not isinstance(runtime[field], str) or not runtime[field]:
            raise ProtocolError("report_invalid", f"runtime {field} is invalid")
    if not isinstance(runtime["jax_devices"], list) or any(
        not isinstance(device, str) or not device for device in runtime["jax_devices"]
    ):
        raise ProtocolError("report_invalid", "runtime devices are invalid")

    status = report["status"]
    reason = report["unavailable_reason_code"]
    version_fields = {
        "jax_version": "jax",
        "jaxlib_version": "jaxlib",
        "jax_cuda_plugin_version": "jax_cuda_plugin",
        "mujoco_version": "mujoco",
        "mujoco_mjx_version": "mujoco_mjx",
        "warp_version": "warp_lang",
    }
    if status == "available":
        try:
            driver_major = int((runtime["nvidia_driver_version"] or "").split(".", maxsplit=1)[0])
        except (TypeError, ValueError):
            driver_major = -1
        if (
            reason is not None
            or runtime["platform"] != runtime_contract["operating_system"]
            or runtime["machine"] != runtime_contract["architecture"]
            or not (
                runtime["python_version"] == runtime_contract["python"]
                or runtime["python_version"].startswith(runtime_contract["python"] + ".")
            )
            or driver_major < runtime_contract["nvidia_driver_minimum"]
            or runtime["jax_backend"] != "gpu"
            or not runtime["jax_devices"]
            or any(runtime[field] != packages[package] for field, package in version_fields.items())
        ):
            raise ProtocolError("report_invalid", "available runtime claims do not match pins")
    elif status == "unavailable":
        if not isinstance(reason, str) or not reason or any(
            not (character.isalnum() or character in "_.-") for character in reason
        ):
            raise ProtocolError("report_invalid", "unavailable reason code is invalid")
    elif status == "test_only":
        if (
            reason is not None
            or runtime["jax_backend"] is not None
            or runtime["jax_devices"]
            or any(runtime[field] is not None for field in version_fields)
        ):
            raise ProtocolError("report_invalid", "test-only report claims accelerator runtime")
    else:
        raise ProtocolError("report_invalid", "unknown capability status")
    canonical_json(report)
    return report


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

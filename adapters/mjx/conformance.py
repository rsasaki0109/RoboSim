"""Generate bounded CPU-reference parity evidence for the MJX-Warp adapter."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
from copy import deepcopy
from pathlib import Path
from typing import Any

from rne_mjx_adapter.backend import (
    GRAVITY_M_S2,
    INITIAL_POSITION_Y_M,
    TASK_ID,
)
from rne_mjx_adapter.protocol import (
    ADAPTER_ID,
    CONFORMANCE_REPORT_KIND,
    CONFORMANCE_REPORT_SCHEMA_VERSION,
    ProtocolError,
    canonical_json,
    load_json_fixture,
)
from rne_mjx_adapter.server import create_backend

POSITION_TOLERANCE_M = 1e-9
VELOCITY_TOLERANCE_M_S = 1e-9
DEFAULT_ROOT_SEED = 42
DEFAULT_STEPS = 60


def build_report(
    adapter_root: Path,
    *,
    backend_name: str,
    allow_test_backend: bool,
    batch_width: int,
    steps: int = DEFAULT_STEPS,
    injected_position_bias_m: float = 0.0,
) -> dict[str, Any]:
    """Runs the binding and returns conformance-report v1."""

    if isinstance(steps, bool) or not isinstance(steps, int) or not 1 <= steps <= 1_000_000:
        raise ProtocolError("invalid_steps", "steps must be an integer in 1..=1000000")
    if not math.isfinite(injected_position_bias_m):
        raise ProtocolError("invalid_fault", "injected position bias must be finite")
    fixtures = adapter_root / "fixtures"
    task_path = fixtures / "free-fall-task-spec-v1.json"
    model_path = fixtures / "free-fall-v1.xml"
    task_spec = load_json_fixture(task_path)
    backend = create_backend(backend_name, adapter_root, allow_test_backend)
    capability = backend.capability_report()
    if capability["status"] == "unavailable":
        raise ProtocolError(
            "runtime_unavailable",
            "accelerator conformance runtime is unavailable",
            details={"reason_code": capability["unavailable_reason_code"]},
        )
    session = backend.create_session(
        task_spec,
        DEFAULT_ROOT_SEED,
        batch_width,
        False,
    )
    actions = [[0.0] for _ in range(batch_width)]
    result: dict[str, Any] | None = None
    for _ in range(steps):
        result = session.step(actions)
    assert result is not None

    dt_s = float(task_spec["control_step_s"])
    expected_velocity_y_m_s = GRAVITY_M_S2 * dt_s * steps
    expected_position_y_m = (
        INITIAL_POSITION_Y_M
        + GRAVITY_M_S2 * dt_s * dt_s * steps * (steps + 1) / 2.0
    )
    actual_position_y_m = result["observations"][0][0] + injected_position_bias_m
    actual_velocity_y_m_s = result["observations"][0][1]
    position_delta_m = abs(actual_position_y_m - expected_position_y_m)
    velocity_delta_m_s = abs(actual_velocity_y_m_s - expected_velocity_y_m_s)
    passed = (
        position_delta_m <= POSITION_TOLERANCE_M
        and velocity_delta_m_s <= VELOCITY_TOLERANCE_M_S
    )
    checkpoint = session.checkpoint()
    evidence_class = (
        "contract_test" if capability["status"] == "test_only" else "accelerator"
    )
    report: dict[str, Any] = {
        "kind": CONFORMANCE_REPORT_KIND,
        "schema_version": CONFORMANCE_REPORT_SCHEMA_VERSION,
        "adapter_id": ADAPTER_ID,
        "evidence_class": evidence_class,
        "backend_status": capability["status"],
        "precision": capability["precision"],
        "task_id": TASK_ID,
        "task_spec_schema": task_spec["schema_version"],
        "task_spec_sha256": _sha256_text(canonical_json(task_spec)),
        "model_sha256": _sha256_text(
            model_path.read_text(encoding="utf-8").replace("\r\n", "\n")
        ),
        "root_seed": DEFAULT_ROOT_SEED,
        "batch_width": batch_width,
        "steps": steps,
        "reference": {
            "backend_id": "mujoco_cpu",
            "case_id": "mujoco.rigid_body.free_fall",
            "integration": "f64_semi_implicit_euler",
            "position_y_m": expected_position_y_m,
            "velocity_y_m_s": expected_velocity_y_m_s,
        },
        "actual": {
            "position_y_m": actual_position_y_m,
            "velocity_y_m_s": actual_velocity_y_m_s,
            "lane_zero_episode_seed": result["episode_seeds"][0],
            "lane_zero_replay_digest": result["lane_replay_digests"][0],
        },
        "tolerances": {
            "position_delta_m": POSITION_TOLERANCE_M,
            "velocity_delta_m_s": VELOCITY_TOLERANCE_M_S,
        },
        "metrics": {
            "position_delta_m": position_delta_m,
            "velocity_delta_m_s": velocity_delta_m_s,
        },
        "fault_injection": {
            "position_bias_m": injected_position_bias_m,
        },
        "runtime_contract": deepcopy(capability["runtime_contract"]),
        "runtime": deepcopy(capability["runtime"]),
        "checkpoint_schema": checkpoint["schema_version"],
        "passed": passed,
    }
    report["content_sha256"] = _report_digest(report)
    validate_report(report)
    return report


def validate_report(report: Any) -> dict[str, Any]:
    """Validates the report envelope, finite payload, and content digest."""

    if not isinstance(report, dict):
        raise ProtocolError("report_invalid", "conformance report must be an object")
    required = {
        "kind",
        "schema_version",
        "adapter_id",
        "evidence_class",
        "backend_status",
        "precision",
        "task_id",
        "task_spec_schema",
        "task_spec_sha256",
        "model_sha256",
        "root_seed",
        "batch_width",
        "steps",
        "reference",
        "actual",
        "tolerances",
        "metrics",
        "fault_injection",
        "runtime_contract",
        "runtime",
        "checkpoint_schema",
        "passed",
        "content_sha256",
    }
    if set(report) != required:
        raise ProtocolError(
            "report_invalid",
            "conformance report fields do not match schema",
            details={
                "missing": sorted(required - report.keys()),
                "unknown": sorted(report.keys() - required),
            },
        )
    if report["kind"] != CONFORMANCE_REPORT_KIND:
        raise ProtocolError("report_invalid", "conformance report kind mismatch")
    if report["schema_version"] != CONFORMANCE_REPORT_SCHEMA_VERSION:
        raise ProtocolError("report_invalid", "conformance report schema mismatch")
    if (
        report["adapter_id"] != ADAPTER_ID
        or report["task_id"] != TASK_ID
        or report["task_spec_schema"] != 1
        or report["checkpoint_schema"] != 2
        or report["precision"] != "f64"
    ):
        raise ProtocolError("report_invalid", "conformance contract identity mismatch")
    if report["evidence_class"] not in {"contract_test", "accelerator"}:
        raise ProtocolError("report_invalid", "unknown evidence class")
    if report["evidence_class"] == "contract_test" and report["backend_status"] != "test_only":
        raise ProtocolError("report_invalid", "contract-test evidence must use test_only backend")
    nested_fields = {
        "reference": {
            "backend_id",
            "case_id",
            "integration",
            "position_y_m",
            "velocity_y_m_s",
        },
        "actual": {
            "position_y_m",
            "velocity_y_m_s",
            "lane_zero_episode_seed",
            "lane_zero_replay_digest",
        },
        "tolerances": {"position_delta_m", "velocity_delta_m_s"},
        "metrics": {"position_delta_m", "velocity_delta_m_s"},
        "fault_injection": {"position_bias_m"},
    }
    for field, fields in nested_fields.items():
        if not isinstance(report[field], dict) or set(report[field]) != fields:
            raise ProtocolError("report_invalid", f"{field} fields do not match schema")
    if (
        report["reference"]["backend_id"] != "mujoco_cpu"
        or report["reference"]["case_id"] != "mujoco.rigid_body.free_fall"
        or report["reference"]["integration"] != "f64_semi_implicit_euler"
        or report["tolerances"]["position_delta_m"] != POSITION_TOLERANCE_M
        or report["tolerances"]["velocity_delta_m_s"] != VELOCITY_TOLERANCE_M_S
    ):
        raise ProtocolError("report_invalid", "conformance reference or tolerance drifted")
    position_delta_m = abs(
        report["actual"]["position_y_m"] - report["reference"]["position_y_m"]
    )
    velocity_delta_m_s = abs(
        report["actual"]["velocity_y_m_s"]
        - report["reference"]["velocity_y_m_s"]
    )
    if (
        report["metrics"]["position_delta_m"] != position_delta_m
        or report["metrics"]["velocity_delta_m_s"] != velocity_delta_m_s
    ):
        raise ProtocolError("report_invalid", "conformance metrics were not recomputed")
    expected_passed = (
        position_delta_m <= POSITION_TOLERANCE_M
        and velocity_delta_m_s <= VELOCITY_TOLERANCE_M_S
    )
    if report["passed"] != expected_passed:
        raise ProtocolError("report_invalid", "conformance pass verdict mismatch")
    canonical_json(report)
    if report["content_sha256"] != _report_digest(report):
        raise ProtocolError("report_digest_mismatch", "conformance report digest mismatch")
    return report


def write_report(path: Path, report: dict[str, Any]) -> None:
    """Atomically writes a validated pretty JSON report."""

    validate_report(report)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(report, allow_nan=False, indent=2, sort_keys=True) + "\n"
    for attempt in range(100):
        temporary = path.with_name(f".{path.name}.tmp-{attempt}")
        try:
            destination = temporary.open("x", encoding="utf-8", newline="\n")
        except FileExistsError:
            continue
        try:
            with destination:
                destination.write(payload)
                destination.flush()
                os.fsync(destination.fileno())
            os.replace(temporary, path)
            return
        except Exception:
            temporary.unlink(missing_ok=True)
            raise
    raise ProtocolError(
        "temporary_file_exhausted",
        "could not reserve a bounded conformance-report temporary file",
    )


def main(argv: list[str] | None = None) -> int:
    """Runs conformance and optionally writes the report."""

    parser = argparse.ArgumentParser(description="RNE MJX-Warp conformance report")
    parser.add_argument("--backend", choices=("mjx_warp", "fake"), default="mjx_warp")
    parser.add_argument("--allow-test-backend", action="store_true")
    parser.add_argument("--batch-width", type=int, default=1)
    parser.add_argument("--steps", type=int, default=DEFAULT_STEPS)
    parser.add_argument("--inject-position-bias-m", type=float, default=0.0)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    adapter_root = Path(__file__).resolve().parent
    try:
        report = build_report(
            adapter_root,
            backend_name=args.backend,
            allow_test_backend=args.allow_test_backend,
            batch_width=args.batch_width,
            steps=args.steps,
            injected_position_bias_m=args.inject_position_bias_m,
        )
        if args.output is not None:
            write_report(args.output, report)
        print(canonical_json(report))
        return 0 if report["passed"] else 1
    except (OSError, ProtocolError) as error:
        if isinstance(error, ProtocolError):
            payload = error.as_object()
        else:
            payload = {
                "code": "io_error",
                "message": "conformance report I/O failed",
                "details": {},
            }
        print(canonical_json({"kind": "rne_accelerator_conformance_error", "error": payload}))
        return 2


def _report_digest(report: dict[str, Any]) -> str:
    without_digest = {key: value for key, value in report.items() if key != "content_sha256"}
    return _sha256_text(canonical_json(without_digest))


def _sha256_text(value: str) -> str:
    return _sha256_bytes(value.encode("utf-8"))


def _sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())

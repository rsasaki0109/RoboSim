"""Measure selected accelerator batch widths without timing simulation logic."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import time
from copy import deepcopy
from pathlib import Path
from typing import Any

from rne_mjx_adapter.backend import TASK_ID
from rne_mjx_adapter.protocol import (
    ADAPTER_ID,
    SCALE_REPORT_KIND,
    SCALE_REPORT_SCHEMA_VERSION,
    SUPPORTED_BATCH_WIDTHS,
    ProtocolError,
    canonical_json,
    load_json_fixture,
)
from rne_mjx_adapter.server import create_backend

DEFAULT_ROOT_SEED = 42
DEFAULT_WARMUP_STEPS = 32
DEFAULT_MEASURED_STEPS = 256


def build_scale_report(
    adapter_root: Path,
    *,
    backend_name: str,
    allow_test_backend: bool,
    widths: list[int],
    warmup_steps: int,
    measured_steps: int,
) -> dict[str, Any]:
    """Measures fixed-step batches and proves width-independent lane zero."""

    _validate_positive_steps(warmup_steps, "warmup_steps", allow_zero=True)
    _validate_positive_steps(measured_steps, "measured_steps", allow_zero=False)
    if not widths or widths != sorted(set(widths)):
        raise ProtocolError(
            "invalid_widths",
            "widths must be a non-empty, strictly increasing unique list",
        )
    if any(width not in SUPPORTED_BATCH_WIDTHS for width in widths):
        raise ProtocolError(
            "unsupported_batch_width",
            f"widths must be selected from {list(SUPPORTED_BATCH_WIDTHS)}",
        )

    fixtures = adapter_root / "fixtures"
    task_spec = load_json_fixture(fixtures / "free-fall-task-spec-v1.json")
    model_text = (fixtures / "free-fall-v1.xml").read_text(encoding="utf-8")
    backend = create_backend(backend_name, adapter_root, allow_test_backend)
    capability = backend.capability_report()
    if capability["status"] == "unavailable":
        raise ProtocolError(
            "runtime_unavailable",
            "accelerator scale runtime is unavailable",
            details={"reason_code": capability["unavailable_reason_code"]},
        )

    runs: list[dict[str, Any]] = []
    lane_zero_digests: list[int] = []
    for width in widths:
        session = backend.create_session(
            task_spec,
            DEFAULT_ROOT_SEED,
            width,
            True,
        )
        actions = [[0.0] for _ in range(width)]
        result: dict[str, Any] | None = None
        for _ in range(warmup_steps):
            result = session.step(actions)
        started_ns = time.perf_counter_ns()
        for _ in range(measured_steps):
            result = session.step(actions)
        elapsed_ns = max(1, time.perf_counter_ns() - started_ns)
        assert result is not None
        transitions = width * measured_steps
        throughput_transitions_s = transitions * 1_000_000_000.0 / elapsed_ns
        if not math.isfinite(throughput_transitions_s):
            raise ProtocolError("non_finite_timing", "scale throughput is not finite")
        lane_zero_digest = result["lane_replay_digests"][0]
        lane_zero_digests.append(lane_zero_digest)
        runs.append(
            {
                "batch_width": width,
                "transitions": transitions,
                "elapsed_ns": elapsed_ns,
                "throughput_transitions_s": throughput_transitions_s,
                "lane_zero_replay_digest": lane_zero_digest,
                "lane_zero_episode_index": result["episode_indices"][0],
                "lane_zero_episode_seed": result["episode_seeds"][0],
            }
        )

    lane_zero_digest_consistent = len(set(lane_zero_digests)) == 1
    promotion_widths_complete = widths == list(SUPPORTED_BATCH_WIDTHS)
    evidence_class = (
        "contract_test" if capability["status"] == "test_only" else "accelerator"
    )
    passed = lane_zero_digest_consistent and (
        evidence_class == "contract_test" or promotion_widths_complete
    )
    report: dict[str, Any] = {
        "kind": SCALE_REPORT_KIND,
        "schema_version": SCALE_REPORT_SCHEMA_VERSION,
        "adapter_id": ADAPTER_ID,
        "evidence_class": evidence_class,
        "backend_status": capability["status"],
        "precision": capability["precision"],
        "measurement_boundary": "python_session_api",
        "task_id": TASK_ID,
        "task_spec_schema": task_spec["schema_version"],
        "task_spec_sha256": _sha256_text(canonical_json(task_spec)),
        "model_sha256": _sha256_text(model_text.replace("\r\n", "\n")),
        "root_seed": DEFAULT_ROOT_SEED,
        "warmup_steps": warmup_steps,
        "measured_steps": measured_steps,
        "requested_widths": list(widths),
        "promotion_widths_complete": promotion_widths_complete,
        "lane_zero_digest_consistent": lane_zero_digest_consistent,
        "runs": runs,
        "runtime_contract": deepcopy(capability["runtime_contract"]),
        "runtime": deepcopy(capability["runtime"]),
        "passed": passed,
    }
    report["content_sha256"] = _report_digest(report)
    validate_scale_report(report)
    return report


def validate_scale_report(report: Any) -> dict[str, Any]:
    """Validates the scale-report envelope, finite timings, and digest."""

    if not isinstance(report, dict):
        raise ProtocolError("report_invalid", "scale report must be an object")
    required = {
        "kind",
        "schema_version",
        "adapter_id",
        "evidence_class",
        "backend_status",
        "precision",
        "measurement_boundary",
        "task_id",
        "task_spec_schema",
        "task_spec_sha256",
        "model_sha256",
        "root_seed",
        "warmup_steps",
        "measured_steps",
        "requested_widths",
        "promotion_widths_complete",
        "lane_zero_digest_consistent",
        "runs",
        "runtime_contract",
        "runtime",
        "passed",
        "content_sha256",
    }
    if set(report) != required:
        raise ProtocolError("report_invalid", "scale report fields do not match schema")
    if report["kind"] != SCALE_REPORT_KIND or report["schema_version"] != SCALE_REPORT_SCHEMA_VERSION:
        raise ProtocolError("report_invalid", "scale report kind or schema mismatch")
    if (
        report["adapter_id"] != ADAPTER_ID
        or report["task_id"] != TASK_ID
        or report["task_spec_schema"] != 1
        or report["precision"] != "f64"
    ):
        raise ProtocolError("report_invalid", "scale contract identity mismatch")
    if report["evidence_class"] not in {"contract_test", "accelerator"}:
        raise ProtocolError("report_invalid", "unknown scale evidence class")
    if report["evidence_class"] == "contract_test" and report["backend_status"] != "test_only":
        raise ProtocolError("report_invalid", "contract-test scale must use test_only backend")
    if report["measurement_boundary"] != "python_session_api":
        raise ProtocolError("report_invalid", "scale measurement boundary mismatch")
    if not isinstance(report["runs"], list) or not report["runs"]:
        raise ProtocolError("report_invalid", "scale report requires at least one run")
    if (
        not isinstance(report["requested_widths"], list)
        or len(report["runs"]) != len(report["requested_widths"])
        or report["requested_widths"] != sorted(set(report["requested_widths"]))
        or any(width not in SUPPORTED_BATCH_WIDTHS for width in report["requested_widths"])
    ):
        raise ProtocolError("report_invalid", "scale run count differs from requested widths")
    _validate_positive_steps(report["warmup_steps"], "warmup_steps", allow_zero=True)
    _validate_positive_steps(report["measured_steps"], "measured_steps", allow_zero=False)
    run_fields = {
        "batch_width",
        "transitions",
        "elapsed_ns",
        "throughput_transitions_s",
        "lane_zero_replay_digest",
        "lane_zero_episode_index",
        "lane_zero_episode_seed",
    }
    for index, run in enumerate(report["runs"]):
        if (
            not isinstance(run, dict)
            or set(run) != run_fields
            or run.get("elapsed_ns", 0) <= 0
            or not math.isfinite(run.get("throughput_transitions_s", float("nan")))
            or run.get("transitions") != run.get("batch_width", 0) * report["measured_steps"]
        ):
            raise ProtocolError("report_invalid", "scale run timing is invalid")
        if run["batch_width"] != report["requested_widths"][index]:
            raise ProtocolError("report_invalid", "scale run order differs from requested widths")
        expected_throughput = run["transitions"] * 1_000_000_000.0 / run["elapsed_ns"]
        if run["throughput_transitions_s"] != expected_throughput:
            raise ProtocolError("report_invalid", "scale throughput was not recomputed")
    digest_consistent = len(
        {run["lane_zero_replay_digest"] for run in report["runs"]}
    ) == 1
    if report["lane_zero_digest_consistent"] != digest_consistent:
        raise ProtocolError("report_invalid", "lane-zero consistency verdict mismatch")
    promotion_complete = report["requested_widths"] == list(SUPPORTED_BATCH_WIDTHS)
    if report["promotion_widths_complete"] != promotion_complete:
        raise ProtocolError("report_invalid", "promotion-width verdict mismatch")
    expected_passed = digest_consistent and (
        report["evidence_class"] == "contract_test" or promotion_complete
    )
    if report["passed"] != expected_passed:
        raise ProtocolError("report_invalid", "scale pass verdict mismatch")
    canonical_json(report)
    if report["content_sha256"] != _report_digest(report):
        raise ProtocolError("report_digest_mismatch", "scale report digest mismatch")
    return report


def write_scale_report(path: Path, report: dict[str, Any]) -> None:
    """Atomically writes a validated scale report."""

    validate_scale_report(report)
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
        "could not reserve a bounded scale-report temporary file",
    )


def main(argv: list[str] | None = None) -> int:
    """Runs all selected widths and optionally writes the report."""

    parser = argparse.ArgumentParser(description="RNE MJX-Warp scale report")
    parser.add_argument("--backend", choices=("mjx_warp", "fake"), default="mjx_warp")
    parser.add_argument("--allow-test-backend", action="store_true")
    parser.add_argument("--widths", default=",".join(str(width) for width in SUPPORTED_BATCH_WIDTHS))
    parser.add_argument("--warmup-steps", type=int, default=DEFAULT_WARMUP_STEPS)
    parser.add_argument("--measured-steps", type=int, default=DEFAULT_MEASURED_STEPS)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    try:
        widths = [int(value) for value in args.widths.split(",")]
        report = build_scale_report(
            Path(__file__).resolve().parent,
            backend_name=args.backend,
            allow_test_backend=args.allow_test_backend,
            widths=widths,
            warmup_steps=args.warmup_steps,
            measured_steps=args.measured_steps,
        )
        if args.output is not None:
            write_scale_report(args.output, report)
        print(canonical_json(report))
        return 0 if report["passed"] else 1
    except (OSError, ProtocolError, ValueError) as error:
        if isinstance(error, ProtocolError):
            payload = error.as_object()
        else:
            payload = {
                "code": "invalid_scale_request",
                "message": "scale request or I/O failed",
                "details": {},
            }
        print(canonical_json({"kind": "rne_accelerator_scale_error", "error": payload}))
        return 2


def _validate_positive_steps(value: int, field: str, *, allow_zero: bool) -> None:
    minimum = 0 if allow_zero else 1
    if isinstance(value, bool) or not isinstance(value, int) or not minimum <= value <= 1_000_000:
        raise ProtocolError(
            "invalid_steps",
            f"{field} must be an integer in {minimum}..=1000000",
        )


def _report_digest(report: dict[str, Any]) -> str:
    without_digest = {key: value for key, value in report.items() if key != "content_sha256"}
    return _sha256_text(canonical_json(without_digest))


def _sha256_text(value: str) -> str:
    return hashlib.sha256(value.encode("utf-8")).hexdigest()


if __name__ == "__main__":
    raise SystemExit(main())

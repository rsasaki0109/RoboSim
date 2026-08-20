"""Generate the portable accelerator process-conformance compatibility vector."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

from protocol_fixture import build_transcript, validate_transcript
from rne_mjx_adapter.protocol import canonical_json

REPORT_KIND = "rne_accelerator_process_conformance_report"
REPORT_SCHEMA_VERSION = 1
CHECKS = (
    ("spawn", "fresh adapter process launched without a shell"),
    ("probe", "capability report is bound to selected contracts"),
    (
        "create",
        "session created with exact TaskSpec, seed, width, and reset mode",
    ),
    ("reset_lanes", "lane zero reset through its next deterministic episode seed"),
    ("step", "one exact action produced a finite correlated step"),
    ("checkpoint", "portable checkpoint returned after reset and step"),
    ("restore", "exact checkpoint restored the checkpointed lane state"),
    ("close", "session closed explicitly"),
    (
        "unsupported_operation",
        "unsupported operation failed with a stable protocol error",
    ),
    ("shutdown", "adapter acknowledged shutdown and exited successfully"),
    (
        "transcript_binding",
        "all exchanges bind to the manifest, runtime contract, TaskSpec, and checkpoint",
    ),
)


def _repo_text_bytes(path: Path) -> bytes:
    """Reads subject bytes as committed LF text, ignoring checkout CRLF."""

    data = path.read_bytes()
    # Windows checkouts may materialize CRLF even when git stores LF
    # (`eol=lf`). Subject digests must stay platform-neutral.
    if b"\r\n" in data:
        data = data.replace(b"\r\n", b"\n")
    return data


def build_process_conformance_fixture(adapter_root: Path) -> dict[str, Any]:
    """Builds a platform-neutral typed-reader fixture from the frozen lifecycle."""

    transcript = build_transcript(adapter_root)
    subject_path = adapter_root / "rne_mjx_adapter" / "server.py"
    manifest_path = adapter_root / "accelerator.toml"
    runtime_path = adapter_root / "runtime.toml"
    task_path = adapter_root / "fixtures" / "free-fall-task-spec-v1.json"
    subject = _repo_text_bytes(subject_path)
    manifest = _repo_text_bytes(manifest_path)
    runtime = _repo_text_bytes(runtime_path)
    task = _repo_text_bytes(task_path)
    arguments = [
        "-m",
        "rne_mjx_adapter",
        "--backend",
        "fake",
        "--allow-test-backend",
    ]
    report = {
        "kind": REPORT_KIND,
        "schema_version": REPORT_SCHEMA_VERSION,
        "status": "passed",
        "subject": {
            "adapter_file": subject_path.name,
            "adapter_sha256": hashlib.sha256(subject).hexdigest(),
            "adapter_size_bytes": len(subject),
            "launcher_file": "python",
            "arguments_sha256": hashlib.sha256(
                canonical_json(arguments).encode("utf-8")
            ).hexdigest(),
            "argument_count": len(arguments),
            "manifest_file": manifest_path.name,
            "manifest_sha256": hashlib.sha256(manifest).hexdigest(),
            "runtime_file": runtime_path.name,
            "runtime_sha256": hashlib.sha256(runtime).hexdigest(),
            "task_file": task_path.name,
            "task_sha256": hashlib.sha256(task).hexdigest(),
        },
        "adapter_id": transcript["adapter_id"],
        "task_id": transcript["task_id"],
        "task_spec_schema": transcript["task_spec_schema"],
        "task_spec_sha256": transcript["task_spec_sha256"],
        "protocol_schema": transcript["protocol_schema"],
        "root_seed": transcript["root_seed"],
        "batch_width": transcript["batch_width"],
        "checks": [
            {"id": check_id, "status": "passed", "detail": detail}
            for check_id, detail in CHECKS
        ],
        "frames": transcript["frames"],
    }
    validate_process_conformance_fixture(report)
    return report


def validate_process_conformance_fixture(report: Any) -> dict[str, Any]:
    """Checks the portable report vector before it reaches the Rust reader."""

    fields = {
        "kind",
        "schema_version",
        "status",
        "subject",
        "adapter_id",
        "task_id",
        "task_spec_schema",
        "task_spec_sha256",
        "protocol_schema",
        "root_seed",
        "batch_width",
        "checks",
        "frames",
    }
    if not isinstance(report, dict) or set(report) != fields:
        raise ValueError("process-conformance report fields do not match schema")
    if (
        report["kind"] != REPORT_KIND
        or report["schema_version"] != REPORT_SCHEMA_VERSION
        or report["status"] != "passed"
        or report["root_seed"] != 42
        or report["batch_width"] != 1
    ):
        raise ValueError("process-conformance report identity mismatch")
    if [check["id"] for check in report["checks"]] != [item[0] for item in CHECKS]:
        raise ValueError("process-conformance check registry mismatch")
    if any(
        check.get("status") != "passed" or check.get("detail") != detail
        for check, (_, detail) in zip(report["checks"], CHECKS)
    ):
        raise ValueError("process-conformance verdict mismatch")
    validate_transcript(
        {
            "kind": "rne_accelerator_protocol_transcript",
            "schema_version": 1,
            "protocol_schema": report["protocol_schema"],
            "adapter_id": report["adapter_id"],
            "task_id": report["task_id"],
            "task_spec_schema": report["task_spec_schema"],
            "task_spec_sha256": report["task_spec_sha256"],
            "root_seed": report["root_seed"],
            "batch_width": report["batch_width"],
            "frames": report["frames"],
        }
    )
    canonical_json(report)
    return report


def write_fixture(path: Path, report: dict[str, Any]) -> None:
    """Atomically writes a validated pretty JSON report vector."""

    validate_process_conformance_fixture(report)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(report, allow_nan=False, indent=2, sort_keys=True) + "\n"
    temporary = path.with_name(f".{path.name}.tmp")
    try:
        with temporary.open("x", encoding="utf-8", newline="\n") as destination:
            destination.write(payload)
            destination.flush()
            os.fsync(destination.fileno())
        os.replace(temporary, path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    adapter_root = Path(__file__).resolve().parent
    report = build_process_conformance_fixture(adapter_root)
    write_fixture(args.output, report)
    print(canonical_json(report))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

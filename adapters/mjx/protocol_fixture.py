"""Generate the deterministic accelerator protocol-v1 compatibility transcript."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
from typing import Any

from rne_mjx_adapter.backend import FakeBackend
from rne_mjx_adapter.protocol import (
    ADAPTER_ID,
    PROTOCOL_SCHEMA_VERSION,
    ProtocolError,
    canonical_json,
    error_response,
    load_json_fixture,
    parse_request_line,
    success_response,
)
from rne_mjx_adapter.server import AcceleratorServer

TRANSCRIPT_KIND = "rne_accelerator_protocol_transcript"
TRANSCRIPT_SCHEMA_VERSION = 1
TASK_ID = "rne.physics.free_fall.mjx.v1"
ROOT_SEED = 42
BATCH_WIDTH = 1
OPERATIONS = (
    "probe",
    "create",
    "reset_lanes",
    "step",
    "checkpoint",
    "restore",
    "close",
    "unsupported_v1_fixture",
    "shutdown",
)


def build_transcript(adapter_root: Path) -> dict[str, Any]:
    """Runs every lifecycle family against the deterministic fake backend."""

    fixtures = adapter_root / "fixtures"
    task_spec = load_json_fixture(fixtures / "free-fall-task-spec-v1.json")
    server = AcceleratorServer(FakeBackend(fixtures))
    frames: list[dict[str, Any]] = []

    probe = _exchange(server, _request(0, "probe"))
    runtime = probe["response"]["result"]["runtime"]
    runtime["python_version"] = "<runtime>"
    runtime["platform"] = "<runtime>"
    runtime["machine"] = "<runtime>"
    frames.append(probe)

    frames.append(
        _exchange(
            server,
            _request(
                1,
                "create",
                session_id="contract",
                task_spec=task_spec,
                root_seed=ROOT_SEED,
                batch_width=BATCH_WIDTH,
                auto_reset=False,
            ),
        )
    )
    frames.append(
        _exchange(
            server,
            _request(2, "reset_lanes", session_id="contract", lane_ids=[0]),
        )
    )
    frames.append(
        _exchange(
            server,
            _request(3, "step", session_id="contract", actions=[[0.0]]),
        )
    )
    checkpoint_frame = _exchange(
        server,
        _request(4, "checkpoint", session_id="contract"),
    )
    frames.append(checkpoint_frame)
    frames.append(
        _exchange(
            server,
            _request(
                5,
                "restore",
                session_id="contract",
                checkpoint=checkpoint_frame["response"]["result"],
            ),
        )
    )
    frames.append(
        _exchange(server, _request(6, "close", session_id="contract"))
    )
    frames.append(_exchange(server, _request(7, "unsupported_v1_fixture")))
    frames.append(_exchange(server, _request(8, "shutdown")))

    transcript = {
        "kind": TRANSCRIPT_KIND,
        "schema_version": TRANSCRIPT_SCHEMA_VERSION,
        "protocol_schema": PROTOCOL_SCHEMA_VERSION,
        "adapter_id": ADAPTER_ID,
        "task_id": TASK_ID,
        "task_spec_schema": task_spec["schema_version"],
        "task_spec_sha256": hashlib.sha256(
            canonical_json(task_spec).encode("utf-8")
        ).hexdigest(),
        "root_seed": ROOT_SEED,
        "batch_width": BATCH_WIDTH,
        "frames": frames,
    }
    validate_transcript(transcript)
    return transcript


def validate_transcript(transcript: Any) -> dict[str, Any]:
    """Checks the frozen transcript envelope and request/response correlation."""

    fields = {
        "kind",
        "schema_version",
        "protocol_schema",
        "adapter_id",
        "task_id",
        "task_spec_schema",
        "task_spec_sha256",
        "root_seed",
        "batch_width",
        "frames",
    }
    if not isinstance(transcript, dict) or set(transcript) != fields:
        raise ProtocolError("transcript_invalid", "transcript fields do not match schema")
    if (
        transcript["kind"] != TRANSCRIPT_KIND
        or transcript["schema_version"] != TRANSCRIPT_SCHEMA_VERSION
        or transcript["protocol_schema"] != PROTOCOL_SCHEMA_VERSION
        or transcript["adapter_id"] != ADAPTER_ID
        or transcript["task_id"] != TASK_ID
        or transcript["task_spec_schema"] != 1
        or transcript["root_seed"] != ROOT_SEED
        or transcript["batch_width"] != BATCH_WIDTH
        or not isinstance(transcript["task_spec_sha256"], str)
        or len(transcript["task_spec_sha256"]) != 64
    ):
        raise ProtocolError("transcript_invalid", "transcript identity mismatch")
    frames = transcript["frames"]
    if not isinstance(frames, list) or len(frames) != len(OPERATIONS):
        raise ProtocolError("transcript_invalid", "transcript frame count mismatch")
    for request_id, (frame, operation) in enumerate(zip(frames, OPERATIONS)):
        if not isinstance(frame, dict) or set(frame) != {"request", "response"}:
            raise ProtocolError("transcript_invalid", "transcript frame fields mismatch")
        request = frame["request"]
        response = frame["response"]
        if (
            not isinstance(request, dict)
            or request.get("request_id") != request_id
            or request.get("operation") != operation
            or not isinstance(response, dict)
            or response.get("request_id") != request_id
            or response.get("kind") != "rne_accelerator_response"
            or response.get("schema_version") != PROTOCOL_SCHEMA_VERSION
        ):
            raise ProtocolError("transcript_invalid", "transcript correlation mismatch")
        parse_request_line(canonical_json(request))
    canonical_json(transcript)
    return transcript


def write_transcript(path: Path, transcript: dict[str, Any]) -> None:
    """Atomically writes a validated pretty JSON transcript."""

    validate_transcript(transcript)
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = json.dumps(transcript, allow_nan=False, indent=2, sort_keys=True) + "\n"
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


def _request(request_id: int, operation: str, **fields: Any) -> dict[str, Any]:
    return {
        "kind": "rne_accelerator_request",
        "schema_version": PROTOCOL_SCHEMA_VERSION,
        "request_id": request_id,
        "operation": operation,
        **fields,
    }


def _exchange(server: AcceleratorServer, request: dict[str, Any]) -> dict[str, Any]:
    validated = parse_request_line(canonical_json(request))
    try:
        response = success_response(request["request_id"], server.dispatch(validated))
    except ProtocolError as error:
        response = error_response(request["request_id"], error)
    return {"request": request, "response": response}


def main() -> int:
    """Writes or prints the committed transcript."""

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    transcript = build_transcript(Path(__file__).resolve().parent)
    if args.output is not None:
        write_transcript(args.output, transcript)
    print(canonical_json(transcript))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

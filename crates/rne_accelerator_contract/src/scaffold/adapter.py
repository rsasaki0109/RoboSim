#!/usr/bin/env python3
"""Dependency-free RNE accelerator protocol-v1 transport scaffold.

The generated fixture responder proves framing and installed conformance only.
Replace ``dispatch`` with an independently implemented accelerator backend
before submitting external evidence.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Any

MAX_LINE_BYTES = 16 * 1024 * 1024


def canonical_json(value: Any) -> str:
    return json.dumps(value, allow_nan=False, separators=(",", ":"), sort_keys=True)


def load_frames() -> list[dict[str, Any]]:
    path = Path(__file__).with_name("protocol-fixture.json")
    value = json.loads(path.read_text(encoding="utf-8"))
    frames = value.get("frames") if isinstance(value, dict) else None
    if not isinstance(frames, list) or len(frames) != 9:
        raise ValueError("protocol fixture must contain nine frames")
    return frames


def read_request() -> dict[str, Any]:
    line = sys.stdin.buffer.readline(MAX_LINE_BYTES + 2)
    if not line or len(line) > MAX_LINE_BYTES + 1 or not line.endswith(b"\n"):
        raise ValueError("request must be one bounded newline-terminated JSON object")
    value = json.loads(line)
    if not isinstance(value, dict):
        raise ValueError("request must be a JSON object")
    return value


def dispatch(request: dict[str, Any], frame: dict[str, Any]) -> dict[str, Any]:
    """Fixture handler to replace with real runtime/session state."""

    if request != frame.get("request"):
        raise ValueError("request differs from the scaffold conformance fixture")
    response = frame.get("response")
    if not isinstance(response, dict):
        raise ValueError("fixture response must be a JSON object")
    return response


def main() -> int:
    for frame in load_frames():
        response = dispatch(read_request(), frame)
        sys.stdout.write(canonical_json(response) + "\n")
        sys.stdout.flush()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Bounded JSONL server for the out-of-process accelerator boundary."""

from __future__ import annotations

import re
import sys
import traceback
from pathlib import Path
from typing import Any, TextIO

from .backend import AcceleratorBackend, FakeBackend, FreeFallSession, MjxWarpBackend
from .protocol import (
    ProtocolError,
    canonical_json,
    error_response,
    parse_request_line,
    require_exact_keys,
    require_unsigned,
    success_response,
)

SESSION_ID_PATTERN = re.compile(r"^[A-Za-z0-9_.-]{1,64}$")
COMMON_KEYS = {"kind", "schema_version", "request_id", "operation"}
MAX_SESSIONS = 8


class AcceleratorServer:
    """Owns isolated sessions and dispatches one request at a time."""

    def __init__(self, backend: AcceleratorBackend) -> None:
        self._backend = backend
        self._sessions: dict[str, FreeFallSession] = {}
        self.shutdown_requested = False

    def dispatch(self, request: dict[str, Any]) -> Any:
        """Dispatches a validated request envelope."""

        operation = request["operation"]
        if operation == "probe":
            require_exact_keys(request, required=COMMON_KEYS, context="probe request")
            return self._backend.capability_report()
        if operation == "create":
            require_exact_keys(
                request,
                required=COMMON_KEYS
                | {"session_id", "task_spec", "root_seed", "batch_width", "auto_reset"},
                context="create request",
            )
            session_id = _session_id(request["session_id"])
            if session_id in self._sessions:
                raise ProtocolError(
                    "session_exists",
                    f"session {session_id!r} already exists",
                )
            if len(self._sessions) >= MAX_SESSIONS:
                raise ProtocolError(
                    "session_limit_reached",
                    f"adapter permits at most {MAX_SESSIONS} concurrent sessions",
                )
            session = self._backend.create_session(
                request["task_spec"],
                require_unsigned(request["root_seed"], "root_seed"),
                require_unsigned(request["batch_width"], "batch_width", 1 << 20),
                request["auto_reset"],
            )
            self._sessions[session_id] = session
            return {"session_id": session_id, **session.initial_state()}
        if operation == "reset_lanes":
            require_exact_keys(
                request,
                required=COMMON_KEYS | {"session_id", "lane_ids"},
                context="reset_lanes request",
            )
            return self._session(request).reset_lanes(request["lane_ids"])
        if operation == "step":
            require_exact_keys(
                request,
                required=COMMON_KEYS | {"session_id", "actions"},
                context="step request",
            )
            return self._session(request).step(request["actions"])
        if operation == "checkpoint":
            require_exact_keys(
                request,
                required=COMMON_KEYS | {"session_id"},
                context="checkpoint request",
            )
            return self._session(request).checkpoint()
        if operation == "restore":
            require_exact_keys(
                request,
                required=COMMON_KEYS | {"session_id", "checkpoint"},
                context="restore request",
            )
            return self._session(request).restore_checkpoint(request["checkpoint"])
        if operation == "close":
            require_exact_keys(
                request,
                required=COMMON_KEYS | {"session_id"},
                context="close request",
            )
            session_id = _session_id(request["session_id"])
            if session_id not in self._sessions:
                raise ProtocolError("session_not_found", f"session {session_id!r} does not exist")
            del self._sessions[session_id]
            return {"session_id": session_id, "closed": True}
        if operation == "shutdown":
            require_exact_keys(request, required=COMMON_KEYS, context="shutdown request")
            self._sessions.clear()
            self.shutdown_requested = True
            return {"shutdown": True}
        raise ProtocolError(
            "unsupported_operation",
            f"unsupported operation {operation!r}",
        )

    def _session(self, request: dict[str, Any]) -> FreeFallSession:
        session_id = _session_id(request["session_id"])
        try:
            return self._sessions[session_id]
        except KeyError as error:
            raise ProtocolError(
                "session_not_found",
                f"session {session_id!r} does not exist",
            ) from error


def serve(
    backend: AcceleratorBackend,
    source: TextIO = sys.stdin,
    sink: TextIO = sys.stdout,
) -> int:
    """Serves bounded requests until EOF or an explicit shutdown."""

    server = AcceleratorServer(backend)
    for line in source:
        request_id: int | None = None
        try:
            request = parse_request_line(line.rstrip("\r\n"))
            request_id = request["request_id"]
            response = success_response(request_id, server.dispatch(request))
        except ProtocolError as error:
            response = error_response(request_id, error)
        except Exception:
            traceback.print_exc(file=sys.stderr)
            response = error_response(
                request_id,
                ProtocolError(
                    "internal_error",
                    "adapter failed without exposing process internals",
                ),
            )
        sink.write(canonical_json(response) + "\n")
        sink.flush()
        if server.shutdown_requested:
            return 0
    return 0


def create_backend(name: str, adapter_root: Path, allow_test_backend: bool) -> AcceleratorBackend:
    """Creates the selected backend while protecting the fake from production use."""

    fixtures = adapter_root / "fixtures"
    if name == "mjx_warp":
        return MjxWarpBackend(fixtures)
    if name == "fake" and allow_test_backend:
        return FakeBackend(fixtures)
    if name == "fake":
        raise ProtocolError(
            "test_backend_forbidden",
            "fake backend requires the explicit --allow-test-backend flag",
        )
    raise ProtocolError("invalid_backend", f"unknown backend {name!r}")


def _session_id(value: Any) -> str:
    if not isinstance(value, str) or SESSION_ID_PATTERN.fullmatch(value) is None:
        raise ProtocolError(
            "invalid_session_id",
            "session_id must match [A-Za-z0-9_.-]{1,64}",
        )
    return value

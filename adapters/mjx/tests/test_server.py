"""Subprocess tests for the actual stdin/stdout boundary."""

from __future__ import annotations

import json
import subprocess
import sys
import unittest
from pathlib import Path
from typing import Any

ADAPTER_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = ADAPTER_ROOT.parents[1]
SERVE = ADAPTER_ROOT / "serve.py"


class ServerProcess:
    """Small synchronous client that ensures stdout remains JSONL-only."""

    def __init__(self) -> None:
        self.process = subprocess.Popen(
            [
                sys.executable,
                str(SERVE),
                "--backend",
                "fake",
                "--allow-test-backend",
            ],
            cwd=REPOSITORY_ROOT,
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            encoding="utf-8",
        )
        assert self.process.stdin is not None
        assert self.process.stdout is not None

    def request(self, request: dict[str, Any]) -> dict[str, Any]:
        assert self.process.stdin is not None
        assert self.process.stdout is not None
        self.process.stdin.write(json.dumps(request, allow_nan=False) + "\n")
        self.process.stdin.flush()
        line = self.process.stdout.readline()
        if not line:
            stderr = self.process.stderr.read() if self.process.stderr is not None else ""
            raise AssertionError(f"server exited without a response: {stderr}")
        return json.loads(line)

    def close(self) -> None:
        if self.process.poll() is None:
            self.request(_request(999, "shutdown"))
        self.process.communicate(timeout=10)
        if self.process.returncode != 0:
            stderr = self.process.stderr.read() if self.process.stderr is not None else ""
            raise AssertionError(f"server returned {self.process.returncode}: {stderr}")


def _request(request_id: int, operation: str, **fields: Any) -> dict[str, Any]:
    return {
        "kind": "rne_accelerator_request",
        "schema_version": 1,
        "request_id": request_id,
        "operation": operation,
        **fields,
    }


class ServerTests(unittest.TestCase):
    """Exercises create, step, checkpoint, restore, close, and errors."""

    def setUp(self) -> None:
        self.server = ServerProcess()
        with (ADAPTER_ROOT / "fixtures" / "free-fall-task-spec-v1.json").open(
            "r", encoding="utf-8"
        ) as source:
            self.task_spec = json.load(source)

    def tearDown(self) -> None:
        self.server.close()

    def test_full_session_round_trip(self) -> None:
        probe = self.server.request(_request(0, "probe"))
        self.assertTrue(probe["ok"])
        self.assertEqual(probe["result"]["status"], "test_only")

        created = self.server.request(
            _request(
                1,
                "create",
                session_id="contract",
                task_spec=self.task_spec,
                root_seed=42,
                batch_width=16,
                auto_reset=True,
            )
        )
        self.assertTrue(created["ok"])
        self.assertEqual(created["result"]["episode_seeds"][0], 1298720818104676741)

        actions = [[0.0] for _ in range(16)]
        stepped = self.server.request(
            _request(2, "step", session_id="contract", actions=actions)
        )
        self.assertTrue(stepped["ok"])
        checkpoint = self.server.request(
            _request(3, "checkpoint", session_id="contract")
        )["result"]

        restored = self.server.request(
            _request(
                4,
                "restore",
                session_id="contract",
                checkpoint=checkpoint,
            )
        )
        self.assertTrue(restored["ok"])
        closed = self.server.request(_request(5, "close", session_id="contract"))
        self.assertEqual(closed["result"], {"closed": True, "session_id": "contract"})

    def test_unknown_fields_and_noncanonical_lane_ids_are_rejected(self) -> None:
        invalid = _request(10, "probe", extra=True)
        response = self.server.request(invalid)
        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "invalid_request")

        self.server.request(
            _request(
                11,
                "create",
                session_id="lanes",
                task_spec=self.task_spec,
                root_seed=42,
                batch_width=16,
                auto_reset=False,
            )
        )
        response = self.server.request(
            _request(12, "reset_lanes", session_id="lanes", lane_ids=[2, 1])
        )
        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "non_canonical_lane_ids")

    def test_session_count_is_bounded(self) -> None:
        for index in range(8):
            response = self.server.request(
                _request(
                    20 + index,
                    "create",
                    session_id=f"bounded-{index}",
                    task_spec=self.task_spec,
                    root_seed=index,
                    batch_width=1,
                    auto_reset=False,
                )
            )
            self.assertTrue(response["ok"])
        response = self.server.request(
            _request(
                28,
                "create",
                session_id="bounded-overflow",
                task_spec=self.task_spec,
                root_seed=9,
                batch_width=1,
                auto_reset=False,
            )
        )
        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "session_limit_reached")


class LauncherTests(unittest.TestCase):
    """Ensures the deterministic fake cannot be selected accidentally."""

    def test_fake_backend_requires_explicit_test_flag(self) -> None:
        result = subprocess.run(
            [sys.executable, str(SERVE), "--backend", "fake"],
            cwd=REPOSITORY_ROOT,
            check=False,
            capture_output=True,
            text=True,
            encoding="utf-8",
            timeout=10,
        )
        self.assertEqual(result.returncode, 2)
        response = json.loads(result.stdout)
        self.assertFalse(response["ok"])
        self.assertEqual(response["error"]["code"], "test_backend_forbidden")


if __name__ == "__main__":
    unittest.main()

"""Dependency-free tests for the portable accelerator contract."""

from __future__ import annotations

import json
import sys
import tempfile
import unittest
from copy import deepcopy
from pathlib import Path
from unittest.mock import patch

ADAPTER_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = ADAPTER_ROOT.parents[1]
sys.path.insert(0, str(ADAPTER_ROOT))

from rne_mjx_adapter.backend import FakeBackend, MjxWarpBackend  # noqa: E402
from rne_mjx_adapter.protocol import (  # noqa: E402
    CAPABILITY_REPORT_KIND,
    ProtocolError,
    canonical_json,
    derive_episode_seed,
    load_json_fixture,
    validate_bound_task_spec,
    validate_task_spec,
)
from conformance import (  # noqa: E402
    _report_digest,
    build_report,
    validate_report,
    write_report,
)
from scale import (  # noqa: E402
    _report_digest as scale_report_digest,
    build_scale_report,
    validate_scale_report,
)


class ProtocolTests(unittest.TestCase):
    """Checks cross-language schema, seed, and replay invariants."""

    def setUp(self) -> None:
        self.fixtures = ADAPTER_ROOT / "fixtures"
        self.task_spec = load_json_fixture(self.fixtures / "free-fall-task-spec-v1.json")

    def test_seed_vectors_match_rust_task_contract(self) -> None:
        self.assertEqual(
            [derive_episode_seed(42, 0, episode) for episode in range(4)],
            [
                1298720818104676741,
                6147948423359611076,
                17925233603215598159,
                2375635680555833453,
            ],
        )

    def test_task_binding_rejects_semantic_drift(self) -> None:
        validate_task_spec(self.task_spec)
        validate_bound_task_spec(self.task_spec, self.task_spec)
        changed = deepcopy(self.task_spec)
        changed["control_step_s"] = 0.02
        with self.assertRaisesRegex(ProtocolError, "exactly match"):
            validate_bound_task_spec(changed, self.task_spec)

    def test_partial_reset_does_not_perturb_lane_zero(self) -> None:
        backend = FakeBackend(self.fixtures)
        control = backend.create_session(self.task_spec, 42, 16, False)
        reset = backend.create_session(self.task_spec, 42, 16, False)
        actions = [[0.0] for _ in range(16)]
        for _ in range(3):
            control_step = control.step(actions)
            reset_step = reset.step(actions)
        reset.reset_lanes([2])
        control_step = control.step(actions)
        reset_step = reset.step(actions)
        self.assertEqual(control_step["observations"][0], reset_step["observations"][0])
        self.assertEqual(
            control_step["lane_replay_digests"][0],
            reset_step["lane_replay_digests"][0],
        )
        self.assertNotEqual(
            control_step["lane_replay_digests"][2],
            reset_step["lane_replay_digests"][2],
        )

    def test_checkpoint_replay_restores_the_next_step(self) -> None:
        backend = FakeBackend(self.fixtures)
        original = backend.create_session(self.task_spec, 7, 16, True)
        actions = [[0.0] for _ in range(16)]
        for _ in range(4):
            original.step(actions)
        original.reset_lanes([2, 7])
        checkpoint = original.checkpoint()

        restored = backend.create_session(self.task_spec, 7, 16, True)
        restored.restore_checkpoint(json.loads(canonical_json(checkpoint)))
        self.assertEqual(original.checkpoint(), restored.checkpoint())
        self.assertEqual(original.step(actions), restored.step(actions))

    def test_replay_log_has_a_hard_limit(self) -> None:
        session = FakeBackend(self.fixtures).create_session(self.task_spec, 7, 1, False)
        with patch("rne_mjx_adapter.backend.MAX_REPLAY_OPERATIONS", 2):
            session.step([[0.0]])
            session.step([[0.0]])
            with self.assertRaisesRegex(ProtocolError, "operation limit"):
                session.step([[0.0]])

    def test_auto_reset_is_deferred_until_after_terminal_result(self) -> None:
        session = FakeBackend(self.fixtures).create_session(self.task_spec, 9, 1, True)
        terminal = None
        for _ in range(60):
            terminal = session.step([[0.0]])
        assert terminal is not None
        self.assertEqual(terminal["truncated"], [True])
        self.assertEqual(terminal["reset"], [False])
        terminal_position = terminal["observations"][0][0]

        after_reset = session.step([[0.0]])
        self.assertEqual(after_reset["reset"], [True])
        self.assertGreater(after_reset["observations"][0][0], terminal_position)
        self.assertEqual(after_reset["episode_indices"], [1])

    def test_runtime_probe_is_versioned_and_fail_closed(self) -> None:
        report = MjxWarpBackend(self.fixtures).capability_report()
        self.assertEqual(report["kind"], CAPABILITY_REPORT_KIND)
        self.assertIn(report["status"], {"available", "unavailable"})
        if report["status"] == "unavailable":
            self.assertIsNotNone(report["unavailable_reason_code"])

    def test_conformance_report_passes_and_fault_injection_fails(self) -> None:
        report = build_report(
            ADAPTER_ROOT,
            backend_name="fake",
            allow_test_backend=True,
            batch_width=1,
        )
        self.assertTrue(report["passed"])
        self.assertEqual(report["evidence_class"], "contract_test")
        validate_report(report)

        divergent = build_report(
            ADAPTER_ROOT,
            backend_name="fake",
            allow_test_backend=True,
            batch_width=1,
            injected_position_bias_m=0.01,
        )
        self.assertFalse(divergent["passed"])
        self.assertGreater(divergent["metrics"]["position_delta_m"], 0.009)
        self.assertNotEqual(report["content_sha256"], divergent["content_sha256"])

        tampered = deepcopy(divergent)
        tampered["passed"] = True
        tampered["content_sha256"] = _report_digest(tampered)
        with self.assertRaisesRegex(ProtocolError, "verdict"):
            validate_report(tampered)

    def test_conformance_report_atomic_write_retries_stale_temp(self) -> None:
        report = build_report(
            ADAPTER_ROOT,
            backend_name="fake",
            allow_test_backend=True,
            batch_width=1,
        )
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            (Path(directory) / ".report.json.tmp-0").write_text(
                "stale", encoding="utf-8"
            )
            write_report(path, report)
            with path.open("r", encoding="utf-8") as source:
                restored = json.load(source)
            self.assertEqual(restored, report)
            validate_report(restored)

    def test_normalized_conformance_report_matches_golden(self) -> None:
        report = build_report(
            ADAPTER_ROOT,
            backend_name="fake",
            allow_test_backend=True,
            batch_width=1,
        )
        report["runtime"]["python_version"] = "<runtime>"
        report["runtime"]["platform"] = "<runtime>"
        report["runtime"]["machine"] = "<runtime>"
        report["content_sha256"] = _report_digest(report)
        golden_path = (
            REPOSITORY_ROOT
            / "tests"
            / "golden"
            / "accelerators"
            / "conformance-report-v1.json"
        )
        with golden_path.open("r", encoding="utf-8") as source:
            golden = json.load(source)
        self.assertEqual(report, golden)
        validate_report(golden)

    def test_scale_report_keeps_lane_zero_identical_across_widths(self) -> None:
        report = build_scale_report(
            ADAPTER_ROOT,
            backend_name="fake",
            allow_test_backend=True,
            widths=[1, 16],
            warmup_steps=1,
            measured_steps=3,
        )
        self.assertTrue(report["passed"])
        self.assertTrue(report["lane_zero_digest_consistent"])
        self.assertFalse(report["promotion_widths_complete"])
        self.assertEqual(report["evidence_class"], "contract_test")
        self.assertEqual(
            report["runs"][0]["lane_zero_replay_digest"],
            report["runs"][1]["lane_zero_replay_digest"],
        )
        validate_scale_report(report)

        tampered = deepcopy(report)
        tampered["runs"][0]["throughput_transitions_s"] = 1.0
        tampered["content_sha256"] = scale_report_digest(tampered)
        with self.assertRaisesRegex(ProtocolError, "throughput"):
            validate_scale_report(tampered)

    def test_normalized_scale_report_matches_golden(self) -> None:
        report = build_scale_report(
            ADAPTER_ROOT,
            backend_name="fake",
            allow_test_backend=True,
            widths=[1, 16],
            warmup_steps=1,
            measured_steps=3,
        )
        report["runtime"]["python_version"] = "<runtime>"
        report["runtime"]["platform"] = "<runtime>"
        report["runtime"]["machine"] = "<runtime>"
        for run in report["runs"]:
            run["elapsed_ns"] = 1
            run["throughput_transitions_s"] = run["transitions"] * 1_000_000_000.0
        report["content_sha256"] = scale_report_digest(report)
        golden_path = (
            REPOSITORY_ROOT
            / "tests"
            / "golden"
            / "accelerators"
            / "scale-report-v1.json"
        )
        with golden_path.open("r", encoding="utf-8") as source:
            golden = json.load(source)
        self.assertEqual(report, golden)
        validate_scale_report(golden)

    def test_normalized_capability_report_matches_golden(self) -> None:
        report = FakeBackend(self.fixtures).capability_report()
        report["runtime"]["python_version"] = "<runtime>"
        report["runtime"]["platform"] = "<runtime>"
        report["runtime"]["machine"] = "<runtime>"
        golden_path = (
            REPOSITORY_ROOT
            / "tests"
            / "golden"
            / "accelerators"
            / "capability-report-v1.json"
        )
        with golden_path.open("r", encoding="utf-8") as source:
            golden = json.load(source)
        self.assertEqual(report, golden)


if __name__ == "__main__":
    unittest.main()

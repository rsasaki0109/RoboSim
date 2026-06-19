"""Regression tests for the mobile-manipulator report and sweep tools.

These tests use only the Python standard library and synthetic report bundles,
so they can run without rne_py or training a policy.
"""

import copy
import importlib.util
import json
import subprocess
import sys
import tempfile
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_DIR = Path(__file__).resolve().parent
if str(EXAMPLE_DIR) not in sys.path:
    sys.path.insert(0, str(EXAMPLE_DIR))

from policy_schema import (
    POLICY_ACTION_LIMIT_RAD_S,
    POLICY_ACTION_SCHEMA_HASH,
    POLICY_ARTIFACT_ALGORITHM,
    POLICY_ARTIFACT_VERSION,
    POLICY_OBSERVATION_SCHEMA_HASH,
    POLICY_PARAM_DIM,
    policy_metadata_payload,
)

COMPARE = EXAMPLE_DIR / "compare_reports.py"
SWEEP = EXAMPLE_DIR / "sweep.py"


def _write_json(path, payload):
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_report(
    root,
    name,
    *,
    error=0.1,
    reward=10.0,
    policy_mutator=None,
    manifest_mutator=None,
):
    report_dir = root / name
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "index.html").write_text("<html></html>\n", encoding="utf-8")
    (report_dir / "rollout.csv").write_text(
        "step,total_reward,done\n0,0,false\n", encoding="utf-8"
    )

    policy = {
        **policy_metadata_payload(),
        "param_dim": POLICY_PARAM_DIM,
        "action_limit_rad_s": POLICY_ACTION_LIMIT_RAD_S,
        "params": [0.0, 1.0, 0.0, 1.0],
        "best_reward": reward,
        "training_iterations": 3,
    }
    if policy_mutator is not None:
        policy_mutator(policy)
    _write_json(report_dir / "policy.json", policy)

    manifest = {
        "schema_version": 1,
        "policy_algorithm": POLICY_ARTIFACT_ALGORITHM,
        "policy_schema_version": POLICY_ARTIFACT_VERSION,
        "observation_schema_hash": POLICY_OBSERVATION_SCHEMA_HASH,
        "action_schema_hash": POLICY_ACTION_SCHEMA_HASH,
        "best_reward": reward,
        "training_iterations": 3,
        "rollout_rows": 1,
        "final_total_reward": reward,
        "final_target_error": error,
        "artifacts": {
            "index": "index.html",
            "policy": "policy.json",
            "rollout_csv": "rollout.csv",
        },
    }
    if manifest_mutator is not None:
        manifest_mutator(manifest)
    _write_json(report_dir / "manifest.json", manifest)
    return report_dir


def _run(args, *, expect_success=True):
    result = subprocess.run(args, cwd=ROOT, text=True, capture_output=True)
    if expect_success and result.returncode != 0:
        raise AssertionError(
            "expected command success\n"
            f"command={args}\nstdout={result.stdout}\nstderr={result.stderr}"
        )
    if not expect_success and result.returncode == 0:
        raise AssertionError(
            "expected command failure\n"
            f"command={args}\nstdout={result.stdout}\nstderr={result.stderr}"
        )
    return result


def _load_train_module():
    module_name = "_rne_mobile_manipulator_train_test"
    previous_rne_py = sys.modules.get("rne_py")
    sys.modules["rne_py"] = types.SimpleNamespace()
    try:
        spec = importlib.util.spec_from_file_location(module_name, EXAMPLE_DIR / "train.py")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module
    finally:
        if previous_rne_py is None:
            sys.modules.pop("rne_py", None)
        else:
            sys.modules["rne_py"] = previous_rne_py


class ReportToolTests(unittest.TestCase):
    def test_train_policy_payload_loader_enforces_schema_envelope(self):
        train = _load_train_module()
        payload = train._policy_payload(
            [0.0, 1.0, 0.0, 1.0],
            best_reward=7.0,
            training_iterations=3,
        )
        self.assertEqual(payload["schema_version"], 2)
        self.assertEqual(payload["observation_schema_hash"], train.POLICY_OBSERVATION_SCHEMA_HASH)
        self.assertEqual(payload["action_schema_hash"], train.POLICY_ACTION_SCHEMA_HASH)

        with tempfile.TemporaryDirectory(prefix="rne_train_policy_test_") as temp:
            path = Path(temp) / "policy.json"
            _write_json(path, payload)
            loaded = train._load_policy(path)
            self.assertEqual(loaded["params"], [0.0, 1.0, 0.0, 1.0])

            broken_hash = copy.deepcopy(payload)
            broken_hash["observation_schema_hash"] = "sha256:wrong"
            _write_json(path, broken_hash)
            with self.assertRaisesRegex(ValueError, "observation schema hash mismatch"):
                train._load_policy(path)

            broken_embedded_schema = copy.deepcopy(payload)
            broken_embedded_schema["observation_schema"]["id"] = "changed"
            _write_json(path, broken_embedded_schema)
            with self.assertRaisesRegex(
                ValueError, "embedded observation schema does not match"
            ):
                train._load_policy(path)

    def test_compare_reports_writes_ranked_outputs_and_copies_best_policy(self):
        with tempfile.TemporaryDirectory(prefix="rne_compare_test_") as temp:
            root = Path(temp)
            reports = root / "reports"
            _write_report(reports, "seed_0000", error=0.2, reward=5.0)
            _write_report(reports, "seed_0001", error=0.1, reward=7.0)

            leaderboard_json = root / "leaderboard.json"
            leaderboard_csv = root / "leaderboard.csv"
            best_policy = root / "best_policy.json"
            best_report = root / "best_report.json"

            _run(
                [
                    sys.executable,
                    str(COMPARE),
                    str(reports),
                    "--json",
                    str(leaderboard_json),
                    "--csv",
                    str(leaderboard_csv),
                    "--best-policy-out",
                    str(best_policy),
                    "--best-report-out",
                    str(best_report),
                    "--require-reports-at-least",
                    "2",
                    "--require-final-error-at-most",
                    "0.11",
                ]
            )

            leaderboard = json.loads(leaderboard_json.read_text(encoding="utf-8"))
            self.assertEqual(leaderboard["schema_version"], 1)
            self.assertEqual(leaderboard["reports_considered"], 2)
            self.assertEqual(leaderboard["reports"][0]["report"], "seed_0001")
            self.assertTrue(leaderboard_csv.is_file())

            copied_policy = json.loads(best_policy.read_text(encoding="utf-8"))
            self.assertEqual(copied_policy["algorithm"], POLICY_ARTIFACT_ALGORITHM)

            summary = json.loads(best_report.read_text(encoding="utf-8"))
            self.assertEqual(summary["schema_version"], 1)
            self.assertEqual(summary["best"]["report"], "seed_0001")

    def test_compare_reports_rejects_invalid_policy_artifacts(self):
        cases = [
            (
                "missing params",
                lambda policy: policy.pop("params"),
                "policy artifact missing fields: params",
            ),
            (
                "schema mismatch",
                lambda policy: policy.__setitem__("schema_version", 1),
                "policy schema_version mismatch",
            ),
            (
                "algorithm mismatch",
                lambda policy: policy.__setitem__("algorithm", "other"),
                "policy algorithm mismatch",
            ),
            (
                "observation schema hash mismatch",
                lambda policy: policy.__setitem__(
                    "observation_schema_hash", "sha256:wrong"
                ),
                "policy observation schema hash mismatch",
            ),
            (
                "embedded schema hash mismatch",
                lambda policy: policy["observation_schema"].__setitem__(
                    "id", "changed"
                ),
                "policy embedded observation schema does not match its hash",
            ),
            (
                "params length mismatch",
                lambda policy: policy.__setitem__("params", [0.0, 1.0, 0.0]),
                "policy params length mismatch",
            ),
        ]
        for name, policy_mutator, expected in cases:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory(prefix="rne_policy_test_") as temp:
                    report = _write_report(
                        Path(temp), "seed_0000", policy_mutator=policy_mutator
                    )
                    result = _run(
                        [sys.executable, str(COMPARE), str(report)],
                        expect_success=False,
                    )
                    self.assertIn(expected, result.stderr)

    def test_compare_reports_rejects_unsafe_artifact_paths(self):
        cases = [
            (
                "absolute",
                lambda manifest, temp: manifest["artifacts"].__setitem__(
                    "policy", str(Path(temp) / "policy.json")
                ),
                "must be relative",
            ),
            (
                "escape",
                lambda manifest, _temp: manifest["artifacts"].__setitem__(
                    "policy", "../policy.json"
                ),
                "escapes report directory",
            ),
            (
                "missing",
                lambda manifest, _temp: manifest["artifacts"].__setitem__(
                    "policy", "missing_policy.json"
                ),
                "does not exist",
            ),
        ]
        for name, manifest_mutator, expected in cases:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory(prefix="rne_artifact_test_") as temp:
                    report = _write_report(
                        Path(temp),
                        "seed_0000",
                        manifest_mutator=lambda manifest: manifest_mutator(manifest, temp),
                    )
                    result = _run(
                        [sys.executable, str(COMPARE), str(report)],
                        expect_success=False,
                    )
                    self.assertIn(expected, result.stderr)

    def test_sweep_skip_complete_uses_expected_manifests_and_writes_manifest(self):
        with tempfile.TemporaryDirectory(prefix="rne_sweep_test_") as temp:
            out = Path(temp) / "sweep"
            _write_report(out, "seed_0000", error=0.2, reward=5.0)
            _write_report(out, "seed_0001", error=0.1, reward=7.0)
            _write_report(out, "stale_better_report", error=0.0, reward=99.0)

            _run(
                [
                    sys.executable,
                    str(SWEEP),
                    "--out",
                    str(out),
                    "--runs",
                    "2",
                    "--skip-complete",
                    "--iterations",
                    "1",
                    "--population",
                    "2",
                    "--elite",
                    "1",
                    "--rollout-steps",
                    "1",
                    "--require-final-error-at-most",
                    "0.11",
                ]
            )

            sweep_manifest = json.loads(
                (out / "sweep_manifest.json").read_text(encoding="utf-8")
            )
            self.assertEqual(sweep_manifest["schema_version"], 1)
            self.assertEqual(
                sweep_manifest["expected_manifests"],
                ["seed_0000/manifest.json", "seed_0001/manifest.json"],
            )

            leaderboard = json.loads((out / "leaderboard.json").read_text(encoding="utf-8"))
            self.assertEqual(leaderboard["reports_considered"], 2)
            self.assertEqual(leaderboard["reports"][0]["report"], "seed_0001")

            best_policy = json.loads((out / "best_policy.json").read_text(encoding="utf-8"))
            self.assertEqual(best_policy["best_reward"], 7.0)

    def test_sweep_dry_run_does_not_write_outputs(self):
        with tempfile.TemporaryDirectory(prefix="rne_sweep_dry_run_test_") as temp:
            out = Path(temp) / "sweep"
            result = _run(
                [
                    sys.executable,
                    str(SWEEP),
                    "--out",
                    str(out),
                    "--runs",
                    "2",
                    "--iterations",
                    "1",
                    "--population",
                    "2",
                    "--elite",
                    "1",
                    "--dry-run",
                ]
            )

            self.assertIn("compare_reports.py", result.stdout)
            self.assertFalse((out / "sweep_manifest.json").exists())
            self.assertFalse((out / "leaderboard.json").exists())


if __name__ == "__main__":
    unittest.main()

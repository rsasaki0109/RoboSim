"""Regression tests for the mobile-manipulator report and sweep tools.

These tests use only the Python standard library and synthetic report bundles,
so they can run without rne_py or training a policy.
"""

import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
EXAMPLE_DIR = Path(__file__).resolve().parent
COMPARE = EXAMPLE_DIR / "compare_reports.py"
SWEEP = EXAMPLE_DIR / "sweep.py"
POLICY_ALGORITHM = "rne_mobile_manipulator_linear_reach_policy_v1"


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
        "schema_version": 1,
        "algorithm": POLICY_ALGORITHM,
        "param_dim": 4,
        "action_limit_rad_s": 6.0,
        "params": [0.0, 1.0, 0.0, 1.0],
        "best_reward": reward,
        "training_iterations": 3,
    }
    if policy_mutator is not None:
        policy_mutator(policy)
    _write_json(report_dir / "policy.json", policy)

    manifest = {
        "schema_version": 1,
        "policy_algorithm": POLICY_ALGORITHM,
        "policy_schema_version": 1,
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


class ReportToolTests(unittest.TestCase):
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
            self.assertEqual(copied_policy["algorithm"], POLICY_ALGORITHM)

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
                lambda policy: policy.__setitem__("schema_version", 2),
                "policy schema_version mismatch",
            ),
            (
                "algorithm mismatch",
                lambda policy: policy.__setitem__("algorithm", "other"),
                "policy algorithm mismatch",
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

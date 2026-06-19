"""Regression tests for the mobile-manipulator report and sweep tools.

These tests use only the Python standard library and synthetic report bundles,
so they can run without rne_py or training a policy.
"""

import copy
import contextlib
import hashlib
import importlib.util
import io
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
from rollout_schema import (
    ROLLOUT_CSV_HEADER,
    ROLLOUT_CSV_SCHEMA_HASH,
    ROLLOUT_CSV_SCHEMA_VERSION,
)
import render_house_gif

COMPARE = EXAMPLE_DIR / "compare_reports.py"
HOUSE_GIF_DEMO = EXAMPLE_DIR / "house_gif_demo.py"
README_HERO_METADATA = ROOT / "docs" / "media" / "rne-hero.json"
RENDER_HOUSE_GIF = EXAMPLE_DIR / "render_house_gif.py"
SWEEP = EXAMPLE_DIR / "sweep.py"
HOUSE_GIF_DEMO_SMALL_GIF_SHA256 = (
    "sha256:4658e5ec1b30ff3e6f453c62363aa3330be3949e8b8386da51d848beddb5c639"
)
HOUSE_GIF_DEMO_SMALL_CSV_SHA256 = (
    "sha256:17df3644c1e76746b974d3e3b438504495443605da4ba13ce4d48c58ee9d369f"
)
HOUSE_GIF_DEMO_SMALL_METADATA_SHA256 = (
    "sha256:b084836e35e662248e759150de5bca73b531553a6abbcfb820223225ad767e4d"
)


def _write_json(path, payload):
    path.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def _write_rollout(path, rows):
    path.write_text(ROLLOUT_CSV_HEADER + "\n" + "\n".join(rows) + "\n", encoding="utf-8")


def _file_sha256(path):
    return f"sha256:{hashlib.sha256(path.read_bytes()).hexdigest()}"


def _write_report(
    root,
    name,
    *,
    error=0.1,
    reward=10.0,
    house_gif=False,
    policy_mutator=None,
    manifest_mutator=None,
):
    report_dir = root / name
    report_dir.mkdir(parents=True, exist_ok=True)
    (report_dir / "index.html").write_text("<html></html>\n", encoding="utf-8")
    _write_rollout(
        report_dir / "rollout.csv",
        [f"0,0,0,0,0,0,0,{error},0,0,{reward / 10.0},0,{reward},{reward},false"],
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
        "schema_version": 2,
        "policy_algorithm": POLICY_ARTIFACT_ALGORITHM,
        "policy_schema_version": POLICY_ARTIFACT_VERSION,
        "observation_schema_hash": POLICY_OBSERVATION_SCHEMA_HASH,
        "action_schema_hash": POLICY_ACTION_SCHEMA_HASH,
        "rollout_schema_version": ROLLOUT_CSV_SCHEMA_VERSION,
        "rollout_schema_hash": ROLLOUT_CSV_SCHEMA_HASH,
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
    if house_gif:
        gif_path = report_dir / "rollout_house.gif"
        render_house_gif.write_gif(
            gif_path,
            [[0] * 4],
            width=2,
            height=2,
            fps=1,
        )
        manifest["artifacts"]["rollout_house_gif"] = "rollout_house.gif"
        manifest["artifacts"]["rollout_house_gif_metadata"] = "rollout_house.json"
        manifest["rollout_house_gif"] = {
            "width": 2,
            "height": 2,
            "max_frames": 1,
            "frame_count": 1,
            "sample_count": 1,
            "fps": 1.0,
            "byte_size": gif_path.stat().st_size,
            "sha256": _file_sha256(gif_path),
        }
        _write_json(
            report_dir / "rollout_house.json",
            {
                "schema_version": 1,
                "artifact": "rne_mobile_manipulator_house_gif",
                "gif_path": "rollout_house.gif",
                "source": {"kind": "rollout_csv", "path": "rollout.csv"},
                **manifest["rollout_house_gif"],
            },
        )
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

    def test_train_report_dir_can_include_house_gif_artifact(self):
        train = _load_train_module()
        train.rollout_policy = lambda _params, max_steps: [
            {
                "step": 0,
                "base_x": -0.7,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_x": -0.35,
                "ee_y": 0.0,
                "ee_z": 0.35,
                "target_dx": 0.9,
                "target_dy": 0.0,
                "target_dz": 0.4,
                "shoulder_action": 0.5,
                "elbow_action": 0.1,
                "reward": 0.2,
                "total_reward": 0.2,
                "done": False,
            },
            {
                "step": 1,
                "base_x": -0.3,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_x": 0.05,
                "ee_y": 0.0,
                "ee_z": 0.65,
                "target_dx": 0.5,
                "target_dy": 0.0,
                "target_dz": 0.1,
                "shoulder_action": 0.2,
                "elbow_action": -0.1,
                "reward": 0.4,
                "total_reward": 0.6,
                "done": False,
            },
            {
                "step": 2,
                "base_x": 0.1,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_x": 0.45,
                "ee_y": 0.0,
                "ee_z": 0.8,
                "target_dx": 0.1,
                "target_dy": 0.0,
                "target_dz": 0.0,
                "shoulder_action": 0.0,
                "elbow_action": -0.2,
                "reward": 0.6,
                "total_reward": 1.2,
                "done": True,
            },
        ][:max_steps]

        with tempfile.TemporaryDirectory(prefix="rne_train_report_gif_test_") as temp:
            report_dir = Path(temp) / "report"
            report = train._write_report_dir(
                str(report_dir),
                [0.0, 1.0, 0.0, 1.0],
                best_reward=1.2,
                training_iterations=3,
                rollout_steps=3,
                house_gif=True,
                house_gif_width=120,
                house_gif_height=80,
                house_gif_max_frames=3,
                house_gif_fps=8.0,
            )

            manifest = json.loads((report_dir / "manifest.json").read_text(encoding="utf-8"))
            self.assertEqual(
                manifest["artifacts"]["rollout_house_gif"], "rollout_house.gif"
            )
            self.assertEqual(
                manifest["artifacts"]["rollout_house_gif_metadata"],
                "rollout_house.json",
            )
            self.assertEqual(
                {key: manifest["rollout_house_gif"][key] for key in ("width", "height", "max_frames", "frame_count", "fps")},
                {"width": 120, "height": 80, "max_frames": 3, "frame_count": 3, "fps": 8.0},
            )
            self.assertEqual(manifest["rollout_house_gif"]["sample_count"], 3)
            gif_metadata = json.loads(
                (report_dir / "rollout_house.json").read_text(encoding="utf-8")
            )
            self.assertEqual(gif_metadata["gif_path"], "rollout_house.gif")
            self.assertEqual(
                gif_metadata["source"], {"kind": "rollout_csv", "path": "rollout.csv"}
            )
            self.assertEqual(gif_metadata["sample_count"], 3)
            index_html = (report_dir / "index.html").read_text()
            self.assertIn('src="rollout_house.gif"', index_html)
            self.assertIn('href="rollout_house.json"', index_html)
            self.assertIn("Mobile manipulator moving through a house scene", index_html)
            self.assertEqual(Path(report["gif"]), report_dir / "rollout_house.gif")
            self.assertEqual(
                Path(report["gif_metadata"]), report_dir / "rollout_house.json"
            )

            data = (report_dir / "rollout_house.gif").read_bytes()
            self.assertTrue(data.startswith(b"GIF89a"))
            self.assertEqual(data[-1:], b";")
            self.assertGreater(len(data), 1024)
            self.assertEqual(manifest["rollout_house_gif"]["byte_size"], len(data))
            self.assertEqual(
                manifest["rollout_house_gif"]["sha256"],
                f"sha256:{hashlib.sha256(data).hexdigest()}",
            )

    def test_train_eval_can_write_rollout_csv_and_house_gif_together(self):
        train = _load_train_module()
        train.evaluate_policy = lambda _params, _episodes: 1.2
        train.rollout_policy = lambda _params, max_steps: [
            {
                "step": 0,
                "base_x": -0.4,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_x": -0.2,
                "ee_y": 0.0,
                "ee_z": 0.4,
                "target_dx": 0.6,
                "target_dy": 0.0,
                "target_dz": 0.2,
                "shoulder_action": 0.2,
                "elbow_action": 0.1,
                "reward": 0.3,
                "total_reward": 0.3,
                "done": False,
            },
            {
                "step": 1,
                "base_x": 0.0,
                "base_y": 0.0,
                "base_yaw": 0.0,
                "ee_x": 0.3,
                "ee_y": 0.0,
                "ee_z": 0.7,
                "target_dx": 0.1,
                "target_dy": 0.0,
                "target_dz": 0.0,
                "shoulder_action": -0.1,
                "elbow_action": 0.0,
                "reward": 0.5,
                "total_reward": 0.8,
                "done": True,
            },
        ][:max_steps]

        with tempfile.TemporaryDirectory(prefix="rne_train_rollout_gif_test_") as temp:
            root = Path(temp)
            policy_path = root / "policy.json"
            rollout_path = root / "rollout.csv"
            gif_path = root / "house.gif"
            metadata_path = root / "house.json"
            _write_json(
                policy_path,
                train._policy_payload(
                    [0.0, 1.0, 0.0, 1.0],
                    best_reward=1.2,
                    training_iterations=3,
                ),
            )

            previous_argv = sys.argv
            try:
                sys.argv = [
                    "train.py",
                    "--policy-in",
                    str(policy_path),
                    "--eval-only",
                    "--rollout-csv",
                    str(rollout_path),
                    "--rollout-house-gif",
                    str(gif_path),
                    "--rollout-house-gif-metadata",
                    str(metadata_path),
                    "--report-house-gif-width",
                    "120",
                    "--report-house-gif-height",
                    "80",
                    "--report-house-gif-max-frames",
                    "2",
                    "--report-house-gif-fps",
                    "8",
                ]
                with contextlib.redirect_stdout(io.StringIO()):
                    train.main()
            finally:
                sys.argv = previous_argv

            self.assertTrue(rollout_path.is_file())
            self.assertTrue(gif_path.is_file())
            self.assertTrue(metadata_path.is_file())
            self.assertEqual(
                rollout_path.read_text(encoding="utf-8").splitlines()[0],
                ROLLOUT_CSV_HEADER,
            )
            data = gif_path.read_bytes()
            self.assertTrue(data.startswith(b"GIF89a"))
            self.assertEqual(data[-1:], b";")
            self.assertGreater(len(data), 1024)
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["schema_version"], 1)
            self.assertEqual(metadata["source"], {"kind": "rollout_csv", "path": str(rollout_path)})
            self.assertEqual(metadata["sample_count"], 2)
            self.assertEqual(metadata["frame_count"], 2)
            self.assertEqual(metadata["byte_size"], len(data))
            self.assertEqual(metadata["sha256"], f"sha256:{hashlib.sha256(data).hexdigest()}")

    def test_compare_reports_writes_ranked_outputs_and_copies_best_policy(self):
        with tempfile.TemporaryDirectory(prefix="rne_compare_test_") as temp:
            root = Path(temp)
            reports = root / "reports"
            _write_report(reports, "seed_0000", error=0.2, reward=5.0)
            _write_report(reports, "seed_0001", error=0.1, reward=7.0, house_gif=True)

            leaderboard_json = root / "leaderboard.json"
            leaderboard_csv = root / "leaderboard.csv"
            leaderboard_html = root / "leaderboard.html"
            best_policy = root / "best_policy.json"
            best_report = root / "best_report.json"
            best_house_gif = root / "best_rollout_house.gif"
            best_house_gif_metadata = root / "best_rollout_house.json"

            _run(
                [
                    sys.executable,
                    str(COMPARE),
                    str(reports),
                    "--html",
                    str(leaderboard_html),
                    "--json",
                    str(leaderboard_json),
                    "--csv",
                    str(leaderboard_csv),
                    "--best-policy-out",
                    str(best_policy),
                    "--best-report-out",
                    str(best_report),
                    "--best-house-gif-out",
                    str(best_house_gif),
                    "--best-house-gif-metadata-out",
                    str(best_house_gif_metadata),
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
            self.assertEqual(
                leaderboard["reports"][0]["rollout_house_gif_path"],
                "reports/seed_0001/rollout_house.gif",
            )
            self.assertEqual(
                leaderboard["reports"][0]["rollout_house_gif_metadata_path"],
                "reports/seed_0001/rollout_house.json",
            )
            self.assertEqual(leaderboard["reports"][0]["rollout_house_gif_width"], 2)
            self.assertEqual(leaderboard["reports"][0]["rollout_house_gif_height"], 2)
            self.assertEqual(leaderboard["reports"][0]["rollout_house_gif_frames"], 1)
            self.assertEqual(
                leaderboard["reports"][0]["rollout_house_gif_sample_count"], 1
            )
            self.assertEqual(leaderboard["reports"][0]["rollout_house_gif_fps"], 1.0)
            self.assertGreater(leaderboard["reports"][0]["rollout_house_gif_byte_size"], 0)
            self.assertTrue(
                leaderboard["reports"][0]["rollout_house_gif_sha256"].startswith(
                    "sha256:"
                )
            )
            self.assertIsNone(leaderboard["reports"][1]["rollout_house_gif_path"])
            self.assertIsNone(
                leaderboard["reports"][1]["rollout_house_gif_metadata_path"]
            )
            self.assertIsNone(leaderboard["reports"][1]["rollout_house_gif_width"])
            self.assertIsNone(leaderboard["reports"][1]["rollout_house_gif_sample_count"])
            self.assertTrue(leaderboard_csv.is_file())
            self.assertIn("rollout_house_gif_path", leaderboard_csv.read_text())
            self.assertIn("rollout_house_gif_metadata_path", leaderboard_csv.read_text())
            self.assertIn("rollout_house_gif_width", leaderboard_csv.read_text())
            self.assertIn("rollout_house_gif_sample_count", leaderboard_csv.read_text())
            self.assertIn("rollout_house_gif_sha256", leaderboard_csv.read_text())
            leaderboard_html_text = leaderboard_html.read_text()
            self.assertIn("GIF 2x2 / 1f /", leaderboard_html_text)
            self.assertIn('src="reports/seed_0001/rollout_house.gif"', leaderboard_html_text)
            self.assertIn('href="reports/seed_0001/rollout_house.json"', leaderboard_html_text)
            self.assertIn('alt="seed_0001 house GIF"', leaderboard_html_text)

            copied_policy = json.loads(best_policy.read_text(encoding="utf-8"))
            self.assertEqual(copied_policy["algorithm"], POLICY_ARTIFACT_ALGORITHM)
            self.assertEqual(
                best_house_gif.read_bytes(),
                (reports / "seed_0001" / "rollout_house.gif").read_bytes(),
            )
            copied_gif = best_house_gif.read_bytes()
            gif_metadata = json.loads(
                best_house_gif_metadata.read_text(encoding="utf-8")
            )
            self.assertEqual(gif_metadata["schema_version"], 1)
            self.assertEqual(
                gif_metadata["artifact"], "rne_mobile_manipulator_house_gif"
            )
            self.assertEqual(gif_metadata["gif_path"], "best_rollout_house.gif")
            self.assertEqual(
                gif_metadata["source"],
                {
                    "kind": "best_report",
                    "report": "seed_0001",
                    "manifest_path": "reports/seed_0001/manifest.json",
                    "rollout_house_gif_path": "reports/seed_0001/rollout_house.gif",
                },
            )
            self.assertEqual(gif_metadata["sample_count"], 1)
            self.assertEqual(gif_metadata["frame_count"], 1)
            self.assertEqual(gif_metadata["max_frames"], 1)
            self.assertEqual(gif_metadata["width"], 2)
            self.assertEqual(gif_metadata["height"], 2)
            self.assertEqual(gif_metadata["fps"], 1.0)
            self.assertEqual(gif_metadata["byte_size"], len(copied_gif))
            self.assertEqual(
                gif_metadata["sha256"],
                f"sha256:{hashlib.sha256(copied_gif).hexdigest()}",
            )

            summary = json.loads(best_report.read_text(encoding="utf-8"))
            self.assertEqual(summary["schema_version"], 1)
            self.assertEqual(summary["best"]["report"], "seed_0001")
            self.assertEqual(
                summary["best"]["rollout_house_gif_path"],
                "reports/seed_0001/rollout_house.gif",
            )
            self.assertEqual(
                summary["best"]["rollout_house_gif_metadata_path"],
                "reports/seed_0001/rollout_house.json",
            )
            self.assertEqual(summary["best"]["rollout_house_gif_width"], 2)
            self.assertEqual(summary["best"]["rollout_house_gif_sample_count"], 1)
            self.assertTrue(summary["best"]["rollout_house_gif_sha256"].startswith("sha256:"))

    def test_compare_reports_rejects_best_house_gif_copy_without_artifact(self):
        with tempfile.TemporaryDirectory(prefix="rne_best_house_gif_test_") as temp:
            root = Path(temp)
            report = _write_report(root, "seed_0000", error=0.1, reward=7.0)
            best_house_gif = root / "best_rollout_house.gif"

            result = _run(
                [
                    sys.executable,
                    str(COMPARE),
                    str(report),
                    "--best-house-gif-out",
                    str(best_house_gif),
                ],
                expect_success=False,
            )

            self.assertIn("best report has no rollout_house_gif artifact", result.stderr)
            self.assertFalse(best_house_gif.exists())

    def test_compare_reports_requires_best_house_gif_for_metadata_copy(self):
        with tempfile.TemporaryDirectory(prefix="rne_best_house_gif_metadata_test_") as temp:
            root = Path(temp)
            report = _write_report(root, "seed_0000", error=0.1, reward=7.0, house_gif=True)

            result = _run(
                [
                    sys.executable,
                    str(COMPARE),
                    str(report),
                    "--best-house-gif-metadata-out",
                    str(root / "best_rollout_house.json"),
                ],
                expect_success=False,
            )

            self.assertIn(
                "--best-house-gif-metadata-out requires --best-house-gif-out",
                result.stderr,
            )

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

    def test_compare_reports_rejects_invalid_house_gif_artifact(self):
        with tempfile.TemporaryDirectory(prefix="rne_house_gif_manifest_test_") as temp:
            report = _write_report(Path(temp), "seed_0000", house_gif=True)
            (report / "rollout_house.gif").write_bytes(b"not a gif")

            result = _run(
                [sys.executable, str(COMPARE), str(report)],
                expect_success=False,
            )

            self.assertIn("rollout_house_gif: GIF artifact is too small", result.stderr)

    def test_compare_reports_rejects_house_gif_metadata_mismatch(self):
        cases = [
            (
                "width",
                lambda metadata: metadata.__setitem__("width", 999),
                "rollout_house_gif: width mismatch",
            ),
            (
                "byte_size",
                lambda metadata: metadata.__setitem__("byte_size", 999),
                "rollout_house_gif: byte_size mismatch",
            ),
            (
                "sha256",
                lambda metadata: metadata.__setitem__("sha256", "sha256:wrong"),
                "rollout_house_gif: sha256 mismatch",
            ),
            (
                "sample_count",
                lambda metadata: metadata.__setitem__("sample_count", 999),
                "rollout_house_gif: sample_count mismatch",
            ),
        ]
        for name, metadata_mutator, expected in cases:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory(
                    prefix="rne_house_gif_metadata_test_"
                ) as temp:
                    report = _write_report(
                        Path(temp),
                        "seed_0000",
                        house_gif=True,
                        manifest_mutator=lambda manifest: metadata_mutator(
                            manifest["rollout_house_gif"]
                        ),
                    )

                    result = _run(
                        [sys.executable, str(COMPARE), str(report)],
                        expect_success=False,
                    )

                    self.assertIn(expected, result.stderr)

    def test_compare_reports_rejects_house_gif_metadata_artifact_mismatch(self):
        cases = [
            (
                "gif path",
                lambda metadata: metadata.__setitem__("gif_path", "other.gif"),
                "rollout_house_gif_metadata: gif_path does not point at rollout_house_gif",
            ),
            (
                "source path",
                lambda metadata: metadata["source"].__setitem__("path", "other.csv"),
                "rollout_house_gif_metadata: source.path does not point at rollout_csv",
            ),
            (
                "sha256",
                lambda metadata: metadata.__setitem__("sha256", "sha256:wrong"),
                "rollout_house_gif_metadata: sha256 mismatch",
            ),
        ]
        for name, metadata_mutator, expected in cases:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory(
                    prefix="rne_house_gif_metadata_artifact_test_"
                ) as temp:
                    report = _write_report(Path(temp), "seed_0000", house_gif=True)
                    metadata_path = report / "rollout_house.json"
                    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
                    metadata_mutator(metadata)
                    _write_json(metadata_path, metadata)

                    result = _run(
                        [sys.executable, str(COMPARE), str(report)],
                        expect_success=False,
                    )

                    self.assertIn(expected, result.stderr)

    def test_compare_reports_rejects_manifest_metric_mismatches(self):
        cases = [
            (
                "policy best reward",
                lambda manifest: manifest.__setitem__("best_reward", 11.0),
                "policy best_reward mismatch",
            ),
            (
                "policy training iterations",
                lambda manifest: manifest.__setitem__("training_iterations", 4),
                "policy training_iterations mismatch",
            ),
            (
                "rollout rows",
                lambda manifest: manifest.__setitem__("rollout_rows", 2),
                "rollout_rows mismatch",
            ),
            (
                "rollout schema version",
                lambda manifest: manifest.__setitem__("rollout_schema_version", 999),
                "unsupported rollout_schema_version",
            ),
            (
                "rollout schema hash",
                lambda manifest: manifest.__setitem__(
                    "rollout_schema_hash", "sha256:wrong"
                ),
                "rollout_schema_hash mismatch",
            ),
            (
                "final total reward",
                lambda manifest: manifest.__setitem__("final_total_reward", 11.0),
                "final_total_reward mismatch",
            ),
            (
                "final target error",
                lambda manifest: manifest.__setitem__("final_target_error", 0.2),
                "final_target_error mismatch",
            ),
        ]
        for name, manifest_mutator, expected in cases:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory(prefix="rne_manifest_test_") as temp:
                    report = _write_report(
                        Path(temp),
                        "seed_0000",
                        manifest_mutator=manifest_mutator,
                    )
                    result = _run(
                        [sys.executable, str(COMPARE), str(report)],
                        expect_success=False,
                    )
                    self.assertIn(expected, result.stderr)

    def test_compare_reports_rejects_invalid_rollout_csv(self):
        def update_manifest(report, mutator):
            manifest_path = report / "manifest.json"
            manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
            mutator(manifest)
            _write_json(manifest_path, manifest)

        cases = [
            (
                "header mismatch",
                lambda report: (report / "rollout.csv").write_text(
                    "step,total_reward,done\n0,10,false\n", encoding="utf-8"
                ),
                "rollout CSV header mismatch",
            ),
            (
                "step mismatch",
                lambda report: _write_rollout(
                    report / "rollout.csv",
                    ["1,0,0,0,0,0,0,0.1,0,0,1,0,10,10,false"],
                ),
                "rollout step mismatch",
            ),
            (
                "numeric invalid",
                lambda report: _write_rollout(
                    report / "rollout.csv",
                    ["0,0,0,0,0,0,0,bad,0,0,1,0,10,10,false"],
                ),
                "field 'target_dx' must be numeric",
            ),
            (
                "done invalid",
                lambda report: _write_rollout(
                    report / "rollout.csv",
                    ["0,0,0,0,0,0,0,0.1,0,0,1,0,10,10,maybe"],
                ),
                "field 'done' must be true or false",
            ),
            (
                "cumulative reward mismatch",
                lambda report: _write_rollout(
                    report / "rollout.csv",
                    ["0,0,0,0,0,0,0,0.1,0,0,1,0,5,7,false"],
                ),
                "rollout total_reward row 0 mismatch",
            ),
            (
                "rows after done",
                lambda report: (
                    _write_rollout(
                        report / "rollout.csv",
                        [
                            "0,0,0,0,0,0,0,0.1,0,0,1,0,10,10,true",
                            "1,0,0,0,0,0,0,0.1,0,0,0,0,0,10,false",
                        ],
                    ),
                    update_manifest(
                        report,
                        lambda manifest: manifest.__setitem__("rollout_rows", 2),
                    ),
                ),
                "rollout CSV has rows after done",
            ),
        ]
        for name, rollout_mutator, expected in cases:
            with self.subTest(name=name):
                with tempfile.TemporaryDirectory(prefix="rne_rollout_test_") as temp:
                    report = _write_report(Path(temp), "seed_0000")
                    rollout_mutator(report)
                    result = _run(
                        [sys.executable, str(COMPARE), str(report)],
                        expect_success=False,
                    )
                    self.assertIn(expected, result.stderr)

    def test_render_house_gif_writes_gif_from_rollout_csv(self):
        with tempfile.TemporaryDirectory(prefix="rne_house_gif_test_") as temp:
            root = Path(temp)
            rollout = root / "rollout.csv"
            gif_path = root / "house.gif"
            metadata_path = root / "house.json"
            _write_rollout(
                rollout,
                [
                    "0,-0.7,0,0,-0.35,0,0.35,0.9,0,0.4,0.5,0.1,0.2,0.2,false",
                    "1,-0.3,0,0,0.05,0,0.65,0.5,0,0.1,0.2,-0.1,0.4,0.6,false",
                    "2,0.1,0,0,0.45,0,0.8,0.1,0,0.0,0.0,-0.2,0.6,1.2,true",
                ],
            )

            _run(
                [
                    sys.executable,
                    str(RENDER_HOUSE_GIF),
                    str(rollout),
                    "--out",
                    str(gif_path),
                    "--metadata-out",
                    str(metadata_path),
                    "--max-frames",
                    "3",
                    "--width",
                    "120",
                    "--height",
                    "80",
                    "--fps",
                    "8",
                ]
            )

            data = gif_path.read_bytes()
            self.assertTrue(data.startswith(b"GIF89a"))
            self.assertEqual(data[-1:], b";")
            self.assertGreater(len(data), 1024)
            self.assertLess(len(data), 30_000)

            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["schema_version"], 1)
            self.assertEqual(metadata["artifact"], "rne_mobile_manipulator_house_gif")
            self.assertEqual(metadata["source"], {"kind": "rollout_csv", "path": str(rollout)})
            self.assertEqual(metadata["sample_count"], 3)
            self.assertEqual(metadata["frame_count"], 3)
            self.assertEqual(metadata["width"], 120)
            self.assertEqual(metadata["height"], 80)
            self.assertEqual(metadata["byte_size"], len(data))
            self.assertEqual(metadata["sha256"], f"sha256:{hashlib.sha256(data).hexdigest()}")
            result = _run(
                [
                    sys.executable,
                    str(RENDER_HOUSE_GIF),
                    "--verify-metadata",
                    str(metadata_path),
                ]
            )
            self.assertIn("house gif metadata verified", result.stdout)

    def test_render_house_gif_demo_can_write_rollout_csv(self):
        with tempfile.TemporaryDirectory(prefix="rne_house_gif_demo_test_") as temp:
            root = Path(temp)
            rollout = root / "demo_rollout.csv"
            gif_path = root / "house.gif"
            metadata_path = root / "house.json"

            _run(
                [
                    sys.executable,
                    str(RENDER_HOUSE_GIF),
                    "--demo",
                    "--demo-rollout-csv",
                    str(rollout),
                    "--out",
                    str(gif_path),
                    "--metadata-out",
                    str(metadata_path),
                    "--max-frames",
                    "5",
                    "--width",
                    "120",
                    "--height",
                    "80",
                    "--fps",
                    "8",
                ]
            )

            loaded = render_house_gif.load_samples(rollout)
            self.assertEqual(len(loaded), 90)
            self.assertEqual(loaded[0]["step"], 0)
            self.assertFalse(loaded[0]["done"])
            self.assertTrue(loaded[-1]["done"])
            self.assertEqual(
                rollout.read_text(encoding="utf-8").splitlines()[0],
                ROLLOUT_CSV_HEADER,
            )

            data = gif_path.read_bytes()
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(
                metadata["source"],
                {
                    "kind": "demo",
                    "task": "navigate_pick_place",
                    "rollout_csv_path": str(rollout),
                },
            )
            self.assertEqual(metadata["sample_count"], 90)
            self.assertEqual(metadata["frame_count"], 5)
            self.assertEqual(metadata["byte_size"], len(data))
            self.assertEqual(
                metadata["sha256"], f"sha256:{hashlib.sha256(data).hexdigest()}"
            )

    def test_house_gif_demo_writes_preview_bundle(self):
        with tempfile.TemporaryDirectory(prefix="rne_house_gif_bundle_test_") as temp:
            out_dir = Path(temp) / "house_demo"

            result = _run(
                [
                    sys.executable,
                    str(HOUSE_GIF_DEMO),
                    "--out-dir",
                    str(out_dir),
                    "--width",
                    "120",
                    "--height",
                    "80",
                    "--max-frames",
                    "5",
                    "--fps",
                    "8",
                ]
            )

            self.assertIn("house gif demo written", result.stdout)
            rollout = out_dir / "house_mobile_manipulator.csv"
            gif_path = out_dir / "house_mobile_manipulator.gif"
            metadata_path = out_dir / "house_mobile_manipulator.json"
            index_path = out_dir / "index.html"
            for path in (rollout, gif_path, metadata_path, index_path):
                self.assertTrue(path.is_file(), path)

            loaded = render_house_gif.load_samples(rollout)
            self.assertEqual(len(loaded), 90)
            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            self.assertEqual(metadata["gif_path"], "house_mobile_manipulator.gif")
            self.assertEqual(
                metadata["source"],
                {
                    "kind": "demo",
                    "task": "navigate_pick_place",
                    "rollout_csv_path": "house_mobile_manipulator.csv",
                },
            )
            self.assertEqual(metadata["frame_count"], 5)
            self.assertEqual(metadata["width"], 120)
            self.assertEqual(metadata["height"], 80)
            self.assertEqual(metadata["byte_size"], 5653)
            self.assertEqual(metadata["sha256"], HOUSE_GIF_DEMO_SMALL_GIF_SHA256)
            self.assertEqual(_file_sha256(gif_path), HOUSE_GIF_DEMO_SMALL_GIF_SHA256)
            self.assertEqual(_file_sha256(rollout), HOUSE_GIF_DEMO_SMALL_CSV_SHA256)
            self.assertEqual(
                _file_sha256(metadata_path), HOUSE_GIF_DEMO_SMALL_METADATA_SHA256
            )
            verified = render_house_gif.verify_metadata(metadata_path)
            self.assertEqual(verified["sha256"], metadata["sha256"])

            html = index_path.read_text(encoding="utf-8")
            self.assertIn('src="house_mobile_manipulator.gif"', html)
            self.assertIn('href="house_mobile_manipulator.csv"', html)
            self.assertIn('href="house_mobile_manipulator.json"', html)

    def test_house_gif_demo_check_mode_uses_temporary_bundle(self):
        result = _run(
            [
                sys.executable,
                str(HOUSE_GIF_DEMO),
                "--check",
                "--width",
                "120",
                "--height",
                "80",
                "--max-frames",
                "5",
                "--fps",
                "8",
            ]
        )

        self.assertIn("house gif demo check ok", result.stdout)
        self.assertIn("frames=5", result.stdout)
        self.assertIn("size=120x80", result.stdout)

    def test_readme_hero_metadata_verifies(self):
        metadata = json.loads(README_HERO_METADATA.read_text(encoding="utf-8"))
        gif_info = render_house_gif.inspect_gif(README_HERO_METADATA.with_name("rne-hero.gif"))

        self.assertEqual(
            metadata["artifact"], "rne_3d_mobile_manipulator_navigation_reach_hero"
        )
        self.assertEqual(metadata["gif_path"], "rne-hero.gif")
        self.assertEqual(metadata["poster_path"], "rne-hero.png")
        self.assertEqual(metadata["width"], gif_info["width"])
        self.assertEqual(metadata["height"], gif_info["height"])
        self.assertEqual(metadata["frame_count"], gif_info["frame_count"])
        self.assertEqual(metadata["byte_size"], gif_info["byte_size"])
        self.assertEqual(metadata["sha256"], gif_info["sha256"])
        self.assertEqual(
            metadata["source"],
            {
                "generator": "examples/32_lift_pick_place_hero",
                "kind": "wgpu_simulation",
                "physics": "MobileManipulatorSim/Rapier",
                "policy": "MobileReachHeroPolicy",
                "scene": "assets/scenes/mm_mobile.rne.scene.toml",
            },
        )

    def test_render_house_gif_verify_metadata_rejects_mismatch(self):
        with tempfile.TemporaryDirectory(prefix="rne_house_gif_verify_test_") as temp:
            root = Path(temp)
            rollout = root / "rollout.csv"
            gif_path = root / "house.gif"
            metadata_path = root / "house.json"
            _write_rollout(
                rollout,
                [
                    "0,-0.7,0,0,-0.35,0,0.35,0.9,0,0.4,0.5,0.1,0.2,0.2,false",
                    "1,0.1,0,0,0.45,0,0.8,0.1,0,0.0,0.0,-0.2,0.6,0.8,true",
                ],
            )
            _run(
                [
                    sys.executable,
                    str(RENDER_HOUSE_GIF),
                    str(rollout),
                    "--out",
                    str(gif_path),
                    "--metadata-out",
                    str(metadata_path),
                    "--max-frames",
                    "2",
                    "--width",
                    "120",
                    "--height",
                    "80",
                ]
            )

            metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
            metadata["sha256"] = "sha256:wrong"
            _write_json(metadata_path, metadata)
            result = _run(
                [
                    sys.executable,
                    str(RENDER_HOUSE_GIF),
                    "--verify-metadata",
                    str(metadata_path),
                ],
                expect_success=False,
            )

            self.assertIn("metadata sha256 mismatch", result.stderr)

    def test_render_house_gif_lzw_compresses_code_size_boundaries(self):
        with tempfile.TemporaryDirectory(prefix="rne_house_lzw_test_") as temp:
            gif_path = Path(temp) / "ramp.gif"
            pixels = list(range(24)) * 5

            render_house_gif.write_gif(
                gif_path,
                [pixels],
                width=10,
                height=12,
                fps=1,
            )

            data = gif_path.read_bytes()
            self.assertTrue(data.startswith(b"GIF89a"))
            self.assertEqual(data[-1:], b";")
            self.assertLess(len(data), 300)

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
                    "--report-house-gif",
                    "--report-house-gif-width",
                    "120",
                    "--report-house-gif-height",
                    "80",
                    "--report-house-gif-max-frames",
                    "4",
                    "--report-house-gif-fps",
                    "8",
                    "--dry-run",
                ]
            )

            self.assertIn("compare_reports.py", result.stdout)
            self.assertIn("--report-house-gif", result.stdout)
            self.assertIn("--report-house-gif-width 120", result.stdout)
            self.assertIn("--report-house-gif-height 80", result.stdout)
            self.assertIn("--report-house-gif-max-frames 4", result.stdout)
            self.assertIn("--report-house-gif-fps 8", result.stdout)
            self.assertIn("--best-house-gif-out", result.stdout)
            self.assertIn("best_rollout_house.gif", result.stdout)
            self.assertIn("--best-house-gif-metadata-out", result.stdout)
            self.assertIn("best_rollout_house.json", result.stdout)
            self.assertFalse((out / "sweep_manifest.json").exists())
            self.assertFalse((out / "leaderboard.json").exists())


if __name__ == "__main__":
    unittest.main()

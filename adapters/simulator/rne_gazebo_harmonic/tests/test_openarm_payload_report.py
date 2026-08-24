from __future__ import annotations

import importlib.util
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


SUITE = load_module("payload_suite_for_report", SCRIPT_DIR / "build_openarm_payload_suite.py")
REPORT = load_module("openarm_payload_report", SCRIPT_DIR / "build_openarm_payload_report.py")


class OpenArmPayloadReportTests(unittest.TestCase):
    def compile_fixtures(self, output: Path):
        return SUITE.compile_suite(
            SCRIPT_DIR / "openarm_payload_experiments.json",
            ROOT / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
            SCRIPT_DIR / "openarm_right.adapter.json",
            SCRIPT_DIR / "openarm_right.rne_actuation.json",
            output,
        )

    def test_model_integrity_is_recomputed_without_backend_traces(self) -> None:
        with tempfile.TemporaryDirectory() as fixture_dir, tempfile.TemporaryDirectory() as trace_dir:
            fixture_root = Path(fixture_dir)
            self.compile_fixtures(fixture_root)
            report = REPORT.build_report(
                fixture_root,
                Path(trace_dir),
                SCRIPT_DIR / "openarm_payload_experiments.json",
                ROOT / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
                ROOT
                / "docs/evidence/openarm-controller-lab/evidence/openarm-plant-state-feedback.controller.json",
            )
            self.assertEqual(report["status"], "incomplete")
            self.assertEqual(len(report["model_cases"]), 5)
            self.assertEqual(len(report["missing_traces"]), 15)
            self.assertTrue(
                all(case["check"]["status"] == "passed" for case in report["model_cases"])
            )

    def test_metrics_recompute_rmse_final_peak_and_iae(self) -> None:
        trace = {
            "observations": [
                {
                    "joint_reference_position_rad": [1.0, 0.0],
                    "joint_position_rad": [0.5, 0.0],
                },
                {
                    "joint_reference_position_rad": [1.0, 0.0],
                    "joint_position_rad": [1.25, 0.0],
                },
            ]
        }
        metrics = REPORT.controlled_joint_metrics(trace, 0, 0.1)
        self.assertAlmostEqual(metrics["tracking_rmse_rad"], (0.15625) ** 0.5)
        self.assertAlmostEqual(metrics["final_absolute_error_rad"], 0.25)
        self.assertAlmostEqual(metrics["maximum_absolute_error_rad"], 0.5)
        self.assertAlmostEqual(metrics["integral_absolute_error_rad_s"], 0.075)

    def test_declared_out_of_capacity_case_is_an_expected_failure(self) -> None:
        checks = [
            {"requirement_id": "payload.maximum_model_parameter_realization_delta", "status": "passed"},
            {"requirement_id": "payload.maximum_supported_mass_kg", "status": "failed"},
            {"requirement_id": "payload.requires_exact_replay", "status": "passed"},
        ]
        self.assertEqual(
            REPORT.outcome_status(checks, payload_supported=False),
            "failed_as_expected",
        )
        self.assertEqual(REPORT.outcome_status(checks, payload_supported=True), "failed")

    def test_tampered_model_is_rejected_before_metrics(self) -> None:
        with tempfile.TemporaryDirectory() as fixture_dir, tempfile.TemporaryDirectory() as trace_dir:
            fixture_root = Path(fixture_dir)
            self.compile_fixtures(fixture_root)
            urdf = fixture_root / "payload-0500g/openarm_v2_right.payload.urdf"
            urdf.write_bytes(urdf.read_bytes() + b"\n")
            with self.assertRaisesRegex(ValueError, "model_urdf_sha256 differs"):
                REPORT.build_report(
                    fixture_root,
                    Path(trace_dir),
                    SCRIPT_DIR / "openarm_payload_experiments.json",
                    ROOT
                    / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
                    ROOT
                    / "docs/evidence/openarm-controller-lab/evidence/openarm-plant-state-feedback.controller.json",
                )


if __name__ == "__main__":
    unittest.main()

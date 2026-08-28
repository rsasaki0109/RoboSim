from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_joint_loss_report.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("openarm_joint_loss_report", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPORT = load_module()


class OpenArmJointLossReportTests(unittest.TestCase):
    def test_supported_case_requires_every_check(self) -> None:
        checks = [
            {
                "requirement_id": "joint_loss.maximum_model_parameter_realization_delta",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.maximum_controlled_joint_rmse_rad",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.maximum_controlled_joint_final_error_rad",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.maximum_supported_viscous_damping_nm_s_per_rad",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.requires_exact_replay",
                "status": "passed",
            },
        ]
        self.assertEqual(REPORT.outcome_status(checks, supported=True), "passed")
        checks[1]["status"] = "failed"
        self.assertEqual(REPORT.outcome_status(checks, supported=True), "failed")

    def test_outside_envelope_distinguishes_margin_and_expected_boundary(self) -> None:
        checks = [
            {
                "requirement_id": "joint_loss.maximum_model_parameter_realization_delta",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.maximum_controlled_joint_rmse_rad",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.maximum_controlled_joint_final_error_rad",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.maximum_supported_viscous_damping_nm_s_per_rad",
                "status": "failed",
            },
            {
                "requirement_id": "joint_loss.requires_exact_replay",
                "status": "passed",
            },
        ]
        self.assertEqual(
            REPORT.outcome_status(checks, supported=False),
            "outside_declared_envelope",
        )
        checks[1]["status"] = "failed"
        self.assertEqual(
            REPORT.outcome_status(checks, supported=False),
            "expected_boundary_failure",
        )
        checks[4]["status"] = "failed"
        self.assertEqual(REPORT.outcome_status(checks, supported=False), "failed")

    def test_missing_structural_check_fails_closed(self) -> None:
        checks = [
            {
                "requirement_id": "joint_loss.maximum_controlled_joint_rmse_rad",
                "status": "failed",
            },
            {
                "requirement_id": "joint_loss.maximum_controlled_joint_final_error_rad",
                "status": "passed",
            },
            {
                "requirement_id": "joint_loss.maximum_supported_viscous_damping_nm_s_per_rad",
                "status": "failed",
            },
        ]
        self.assertEqual(REPORT.outcome_status(checks, supported=False), "failed")

    def test_urdf_parser_distinguishes_absent_and_explicit_dynamics(self) -> None:
        import tempfile

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "model.urdf"
            path.write_text('<robot><joint name="joint5" type="revolute"/></robot>')
            self.assertEqual(REPORT.urdf_joint_dynamics(path, "joint5"), (0.0, 0.0, False))
            path.write_text(
                '<robot><joint name="joint5" type="revolute"><dynamics damping="2.5" friction="0"/></joint></robot>'
            )
            self.assertEqual(REPORT.urdf_joint_dynamics(path, "joint5"), (2.5, 0.0, True))


if __name__ == "__main__":
    unittest.main()

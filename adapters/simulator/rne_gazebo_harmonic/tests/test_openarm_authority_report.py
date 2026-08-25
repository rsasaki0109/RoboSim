from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "adapters/simulator/rne_gazebo_harmonic/build_openarm_authority_report.py"


def load_module():
    spec = importlib.util.spec_from_file_location("openarm_authority_report", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPORT = load_module()


class OpenArmAuthorityReportTests(unittest.TestCase):
    def test_metrics_recompute_complete_trajectory(self) -> None:
        trace = {
            "observations": [
                {"joint_reference_position_rad": [1.0], "joint_position_rad": [0.5]},
                {"joint_reference_position_rad": [1.0], "joint_position_rad": [1.25]},
            ]
        }
        metrics = REPORT.metrics(trace, 0)
        self.assertAlmostEqual(metrics["tracking_rmse_rad"], (0.15625) ** 0.5)
        self.assertEqual(metrics["final_absolute_error_rad"], 0.25)

    def test_below_declared_authority_is_expected_only_when_performance_passes(self) -> None:
        checks = [
            {"requirement_id": "authority.maximum_controlled_joint_rmse_rad", "status": "passed"},
            {"requirement_id": "authority.minimum_supported_scale", "status": "failed"},
            {"requirement_id": "authority.requires_exact_replay", "status": "passed"},
        ]
        self.assertEqual(
            REPORT.outcome_status(checks, authority_supported=False),
            "failed_as_expected",
        )
        checks[0]["status"] = "failed"
        self.assertEqual(
            REPORT.outcome_status(checks, authority_supported=False),
            "failed_as_expected",
        )

    def test_actuator_evidence_does_not_call_commands_measurements(self) -> None:
        trace = {
            "observations": [
                {
                    "limited_effort_command_nm": [7.0],
                    "effort_saturated": [True],
                    "effort_measurement_available": [False],
                },
                {
                    "limited_effort_command_nm": [2.0],
                    "effort_saturated": [False],
                    "effort_measurement_available": [False],
                },
            ]
        }
        evidence = REPORT.actuator_evidence(trace, 0)
        self.assertEqual(evidence["kind"], "command_model_only")
        self.assertEqual(evidence["command_saturation_sample_count"], 1)
        self.assertEqual(evidence["command_saturation_fraction"], 0.5)
        self.assertEqual(evidence["measured_effort_sample_count"], 0)


if __name__ == "__main__":
    unittest.main()

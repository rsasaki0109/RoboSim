from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_joint_loss_controller_tuning_report.py"
)
SPEC = importlib.util.spec_from_file_location(
    "openarm_joint_loss_controller_tuning_report", SCRIPT
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OpenArmJointLossControllerTuningReportTests(unittest.TestCase):
    def test_selects_lowest_rmse_then_smallest_limit(self) -> None:
        outcomes = [
            {
                "status": "passed",
                "maximum_state_feedback_correction_rad": 0.05,
                "metrics": {"tracking_rmse_rad": 0.019},
            },
            {
                "status": "failed",
                "maximum_state_feedback_correction_rad": 0.04,
                "metrics": {"tracking_rmse_rad": 0.021},
            },
            {
                "status": "passed",
                "maximum_state_feedback_correction_rad": 0.08,
                "metrics": {"tracking_rmse_rad": 0.017},
            },
            {
                "status": "passed",
                "maximum_state_feedback_correction_rad": 0.06,
                "metrics": {"tracking_rmse_rad": 0.017},
            },
        ]
        self.assertIs(MODULE.select_outcome(outcomes), outcomes[3])

    def test_returns_none_without_a_passing_candidate(self) -> None:
        self.assertIsNone(
            MODULE.select_outcome(
                [
                    {
                        "status": "failed",
                        "maximum_state_feedback_correction_rad": 0.04,
                        "metrics": {"tracking_rmse_rad": 0.021},
                    }
                ]
            )
        )


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "adapters/simulator/rne_gazebo_harmonic/build_openarm_robustness_report.py"
SPEC = importlib.util.spec_from_file_location("openarm_robustness_report", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OpenArmRobustnessReportTests(unittest.TestCase):
    def test_first_violation_is_the_first_cumulative_iae_crossing(self) -> None:
        observations = [
            {
                "step": step,
                "sim_time_ticks": step * 100_000_000,
                "joint_position_rad": [0.0, 0.01],
                "joint_reference_position_rad": [0.0, 0.0],
            }
            for step in range(1, 61)
        ]
        metrics = {
            "contract": {"start_step": 1, "end_step": 10},
            "evaluation_end_step": 60,
            "recovery_check_value_s": 0.1,
        }
        requirements = {
            "controller.state.maximum_disturbance_peak_error_rad": {
                "id": "peak",
                "unit": "rad",
                "maximum": 0.05,
            },
            "controller.state.maximum_disturbance_recovery_time_s": {
                "id": "recovery",
                "unit": "s",
                "maximum": 1.0,
            },
            "controller.state.maximum_disturbance_iae_rad_s": {
                "id": "iae",
                "unit": "rad*s",
                "maximum": 0.02,
            },
        }
        failure = MODULE.first_requirement_violation(
            observations, metrics, 1, 10.0, requirements
        )
        self.assertIsNotNone(failure)
        self.assertEqual(failure["requirement_id"], "iae")
        self.assertEqual(failure["step"], 20)
        self.assertGreater(failure["observed"], 0.02)


if __name__ == "__main__":
    unittest.main()

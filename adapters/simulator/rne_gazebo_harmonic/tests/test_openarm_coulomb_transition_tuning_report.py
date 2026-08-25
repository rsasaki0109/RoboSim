from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_coulomb_transition_tuning_report.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("transition_tuning_report", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPORT = load_module()


class OpenArmCoulombTransitionTuningReportTests(unittest.TestCase):
    def test_selection_uses_largest_predeclared_passing_transition(self) -> None:
        outcomes = [
            {"transition_velocity_rad_s": 0.01, "status": "passed"},
            {"transition_velocity_rad_s": 0.04, "status": "failed"},
            {"transition_velocity_rad_s": 0.02, "status": "passed"},
        ]
        self.assertEqual(
            REPORT.select_candidate(outcomes)["transition_velocity_rad_s"], 0.02
        )

    def test_selection_returns_none_when_contract_remains_red(self) -> None:
        self.assertIsNone(
            REPORT.select_candidate(
                [{"transition_velocity_rad_s": 0.01, "status": "failed"}]
            )
        )


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/"
    "build_openarm_coulomb_controller_pole_tuning_report.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("coulomb_pole_report", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPORT = load_module()


class OpenArmCoulombControllerPoleTuningReportTests(unittest.TestCase):
    def test_selection_uses_rmse_then_maximum_pole(self) -> None:
        outcomes = [
            {"candidate_id": "a", "maximum_pole": 0.9, "metrics": {"tracking_rmse_rad": 0.01}, "status": "passed"},
            {"candidate_id": "b", "maximum_pole": 0.8, "metrics": {"tracking_rmse_rad": 0.01}, "status": "passed"},
        ]
        self.assertEqual(REPORT.select_candidate(outcomes)["candidate_id"], "b")


if __name__ == "__main__":
    unittest.main()

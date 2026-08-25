from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_coulomb_substep_tuning_report.py"
)


def load_module():
    spec = importlib.util.spec_from_file_location("substep_tuning_report", SCRIPT)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


REPORT = load_module()


class OpenArmCoulombSubstepTuningReportTests(unittest.TestCase):
    def test_selection_uses_smallest_passing_substep_count(self) -> None:
        outcomes = [
            {"physics_substeps_per_control_step": 1, "status": "failed"},
            {"physics_substeps_per_control_step": 10, "status": "passed"},
            {"physics_substeps_per_control_step": 5, "status": "passed"},
        ]
        self.assertEqual(
            REPORT.select_candidate(outcomes)["physics_substeps_per_control_step"], 5
        )


if __name__ == "__main__":
    unittest.main()

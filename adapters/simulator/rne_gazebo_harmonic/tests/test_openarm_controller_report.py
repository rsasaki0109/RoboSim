from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "adapters/simulator/rne_gazebo_harmonic/build_openarm_controller_report.py"
SPEC = importlib.util.spec_from_file_location("openarm_controller_report", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OpenArmControllerReportTests(unittest.TestCase):
    def test_fixed_requirement_is_not_derived_from_observed_controller_spread(self) -> None:
        requirement = {
            "id": "fixed",
            "gate": "closed_loop_performance",
            "unit": "s",
            "maximum": 3.5,
        }
        self.assertEqual(MODULE.check(requirement, 3.49)["status"], "passed")
        self.assertEqual(MODULE.check(requirement, 3.51)["status"], "failed")
        self.assertEqual(requirement["maximum"], 3.5)

    def test_controller_vector_reproduction_rejects_width_drift(self) -> None:
        self.assertEqual(MODULE.maximum_delta([1.0, 2.0], [1.0, 2.0]), 0.0)
        with self.assertRaisesRegex(ValueError, "width drifted"):
            MODULE.maximum_delta([1.0], [1.0, 2.0])


if __name__ == "__main__":
    unittest.main()

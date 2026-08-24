from __future__ import annotations

import importlib.util
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"
SCRIPT = SCRIPT_DIR / "build_openarm_robustness_suite.py"
SPEC = importlib.util.spec_from_file_location("openarm_robustness_suite", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
COMPILER = MODULE.load_controller_compiler(SCRIPT_DIR)


class OpenArmRobustnessSuiteTests(unittest.TestCase):
    def compile(self):
        return MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
        )

    def test_fixed_grid_compiles_deterministically(self) -> None:
        suite, controllers = self.compile()
        self.assertEqual((suite, controllers), self.compile())
        self.assertEqual(
            [case["offset_rad"] for case in suite["cases"]], []
        )
        self.assertEqual(
            [controller["disturbance_contract"]["offset_rad"] for controller in controllers.values()],
            [0.0, 0.03, 0.06, 0.09, 0.12],
        )

    def test_cases_change_only_identity_and_declared_bias(self) -> None:
        _, controllers = self.compile()
        normalized = []
        for controller in controllers.values():
            value = controller.copy()
            value["controller_id"] = "normalized"
            value["disturbance_contract"] = value["disturbance_contract"].copy()
            value["disturbance_contract"]["offset_rad"] = 0.0
            normalized.append(value)
        self.assertTrue(all(value == normalized[0] for value in normalized[1:]))

    def test_case_ids_are_unit_explicit_and_stable(self) -> None:
        self.assertEqual(MODULE.case_id(0.0), "bias-000mrad")
        self.assertEqual(MODULE.case_id(0.12), "bias-120mrad")
        with self.assertRaisesRegex(ValueError, "whole milliradians"):
            MODULE.case_id(0.0005)


if __name__ == "__main__":
    unittest.main()

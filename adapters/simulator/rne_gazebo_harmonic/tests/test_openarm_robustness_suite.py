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
RUNNER_SPEC = importlib.util.spec_from_file_location(
    "openarm_robustness_runner", SCRIPT_DIR / "run_openarm_trace.py"
)
assert RUNNER_SPEC is not None and RUNNER_SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(RUNNER_SPEC)
RUNNER_SPEC.loader.exec_module(RUNNER)


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

    def test_sensor_bias_grid_preserves_raw_sensor_and_disables_actuator_bias(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_position_measurement_bias",
        )
        self.assertEqual(suite["dimension_id"], "joint_position_measurement_bias")
        self.assertEqual(
            [controller["measurement_fault_contract"]["offset_rad"] for controller in controllers.values()],
            [0.0, 0.01, 0.02, 0.04, 0.06],
        )
        self.assertTrue(
            all(
                controller["disturbance_contract"]["offset_rad"] == 0.0
                for controller in controllers.values()
            )
        )
        controller = controllers["sensor-bias-010mrad"]
        width = len(controller["action_joint_order"])
        observation = {
            "joint_position_rad": [0.0] * width,
        }
        sample_ticks = controller["observation_contract"]["sample_period_ticks"]
        visible, bias = RUNNER.apply_measurement_bias(
            controller,
            observation,
            (controller["measurement_fault_contract"]["start_controller_step"] - 1)
            * sample_ticks,
        )
        joint_index = controller["action_joint_order"].index("openarm_right_joint5")
        self.assertEqual(visible[joint_index], 0.01)
        self.assertEqual(bias[joint_index], 0.01)
        self.assertTrue(
            all(
                controller["measurement_fault_contract"]["sensor_status"] == "nominal"
                for controller in controllers.values()
            )
        )

    def test_dropout_grid_has_bounded_hold_and_recovery_contract(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_feedback_publication_dropout",
        )
        self.assertEqual(suite["dimension_id"], "joint_feedback_publication_dropout")
        self.assertEqual(
            [
                controller["measurement_fault_contract"]["consecutive_dropped_frames"]
                for controller in controllers.values()
            ],
            [0, 1, 2, 3, 4],
        )
        controller = controllers["dropout-003frames"]
        contract = controller["observation_contract"]
        self.assertEqual(contract["maximum_age_ticks"], 3 * 16_666_667)
        self.assertEqual(
            contract["stale_observation_policy"],
            "hold_last_accepted_target_and_freeze_state",
        )
        self.assertEqual(
            contract["recovery_policy"], "resume_on_fresh_nominal_observation"
        )
        self.assertFalse(RUNNER.sensor_sample_published(controller, 3240))
        self.assertFalse(RUNNER.sensor_sample_published(controller, 3242))
        self.assertTrue(RUNNER.sensor_sample_published(controller, 3243))


if __name__ == "__main__":
    unittest.main()

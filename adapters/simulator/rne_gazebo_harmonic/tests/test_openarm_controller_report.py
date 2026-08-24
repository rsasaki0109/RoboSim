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

    def test_disturbance_metrics_localize_realization_and_recovery(self) -> None:
        contract = {
            "kind": "additive_actuator_target_bias_pulse_v1",
            "classification": "actuator_realization_error",
            "joint": "joint",
            "start_step": 10,
            "end_step": 20,
            "offset_rad": 0.03,
            "controller_visibility": "unobserved_except_through_typed_joint_feedback",
            "application_order": "after_controller_limits_before_backend_actuation",
        }
        observations = []
        for step in range(1, 161):
            active = 10 <= step <= 20
            error = 0.02 if active else 0.0
            observations.append(
                {
                    "step": step,
                    "joint_position_rad": [error],
                    "joint_reference_position_rad": [0.0],
                    "joint_controller_target_rad": [0.0],
                    "joint_position_target_rad": [0.03 if active else 0.0],
                    "joint_actuator_disturbance_rad": [0.03 if active else 0.0],
                    "actuator_disturbance_active": active,
                }
            )
        metrics = MODULE.disturbance_metrics(
            {"disturbance_contract": contract}, observations, 0, 10.0
        )
        self.assertIsNone(metrics["first_realization_mismatch"])
        self.assertAlmostEqual(metrics["peak_tracking_error_rad"], 0.02)
        self.assertEqual(metrics["recovery_step"], 21)
        self.assertAlmostEqual(metrics["recovery_time_s"], 0.1)
        self.assertAlmostEqual(metrics["iae_rad_s"], 0.022)


if __name__ == "__main__":
    unittest.main()

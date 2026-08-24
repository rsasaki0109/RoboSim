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
    def test_command_delay_contract_fails_at_first_delayed_application(self) -> None:
        controller = {
            "disturbance_contract": {
                "kind": "actuator_command_transport_delay_pulse_v1",
                "start_step": 10,
                "end_step": 20,
                "delay_steps": 3,
            }
        }
        observations = [
            {"step": step, "sim_time_ticks": step * 100}
            for step in range(1, 21)
        ]
        requirement = {
            "id": "controller.actuator.maximum_command_transport_delay_steps",
            "unit": "control_period_count",
            "maximum": 2,
        }
        violation = MODULE.command_delay_violation(
            controller, observations, requirement
        )
        self.assertEqual(violation["step"], 10)
        self.assertEqual(violation["source_step"], 7)
        self.assertEqual(violation["observed"], 3)
        controller["disturbance_contract"]["delay_steps"] = 2
        self.assertIsNone(
            MODULE.command_delay_violation(controller, observations, requirement)
        )

    def test_command_delay_source_step_is_recomputed_from_retained_targets(self) -> None:
        report_module = MODULE.load_controller_report_module(SCRIPT.parent)
        controller = {
            "disturbance_contract": {
                "kind": "actuator_command_transport_delay_pulse_v1",
                "joint": "openarm_right_joint5",
                "start_step": 3,
                "end_step": 4,
                "delay_steps": 2,
            }
        }
        observations = []
        for step in range(1, 8):
            commanded = step / 100.0
            applied = (step - 2) / 100.0 if 3 <= step <= 4 else commanded
            observations.append(
                {
                    "step": step,
                    "joint_controller_target_rad": [commanded],
                    "joint_position_target_rad": [applied],
                    "joint_actuator_disturbance_rad": [applied - commanded],
                    "actuator_disturbance_active": applied != commanded,
                    "joint_position_rad": [0.0],
                    "joint_reference_position_rad": [0.0],
                }
            )
        metrics = report_module.disturbance_metrics(
            controller, observations, 0, 60.0
        )
        self.assertIsNone(metrics["first_realization_mismatch"])
        self.assertEqual(
            metrics["realization_verification"]["relationship"],
            "applied_target_at_step_equals_controller_target_at_step_minus_delay_steps",
        )
        self.assertEqual(
            metrics["realization_verification"]["maximum_delta_rad"], 0.0
        )
        observations[2]["joint_position_target_rad"][0] += 0.001
        mismatch = report_module.disturbance_metrics(
            controller, observations, 0, 60.0
        )["first_realization_mismatch"]
        self.assertEqual(mismatch["step"], 3)
        self.assertEqual(mismatch["expected_source_step"], 1)

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

    def test_measurement_bias_is_checked_against_delayed_raw_observation(self) -> None:
        observations = []
        for step in range(1, 6):
            active = step == 4
            observations.append(
                {
                    "step": step,
                    "joint_position_rad": [step / 100.0],
                    "controller_observation_sequence": None if step < 3 else step - 2,
                    "joint_controller_observation_position_rad": (
                        [] if step < 3 else [(step - 2) / 100.0 + (0.02 if active else 0.0)]
                    ),
                    "joint_measurement_bias_rad": [0.02 if active else 0.0],
                    "measurement_bias_active": active,
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "additive_joint_position_bias_pulse_v1",
                "start_controller_step": 4,
                "end_controller_step": 4,
                "offset_rad": 0.02,
            }
        }
        metrics = MODULE.measurement_bias_metrics(controller, observations, 0)
        self.assertIsNotNone(metrics)
        self.assertEqual(metrics["maximum_realization_delta_rad"], 0.0)
        self.assertEqual(metrics["active_decision_count"], 1)
        self.assertIsNone(metrics["first_realization_mismatch"])

    def test_dropout_metrics_separate_publication_hold_and_recovery(self) -> None:
        observations = []
        for step in range(1, 7):
            rejected = step == 5
            observations.append(
                {
                    "step": step,
                    "sim_time_ticks": step * 10,
                    "sensor_sample_published": step not in {2, 3, 4},
                    "controller_observation_sequence": (
                        None if step <= 2 else (1 if step <= 5 else 5)
                    ),
                    "controller_observation_age_ticks": (
                        None if step <= 2 else (step - 1) * 10 if step <= 5 else 10
                    ),
                    "joint_position_rad": [step / 100.0],
                    "joint_controller_observation_position_rad": (
                        [] if step <= 2 else [0.01 if step <= 5 else 0.05]
                    ),
                    "joint_controller_target_rad": [0.2 if step >= 4 else 0.1],
                    "joint_integral_correction_rad": [0.03 if step >= 4 else 0.02],
                    "controller_rejected": rejected,
                    "controller_rejection_reason": (
                        "maximum_observation_age_ticks" if rejected else None
                    ),
                    "fail_safe_hold_active": rejected,
                    "controller_state_frozen": rejected,
                    "controller_recovered": step == 6,
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "joint_feedback_publication_dropout_burst_v1",
                "start_capture_sequence": 2,
                "consecutive_dropped_frames": 3,
            }
        }
        metrics = MODULE.availability_metrics(controller, observations)
        self.assertIsNotNone(metrics)
        self.assertTrue(metrics["publication_realization_matches"])
        self.assertEqual(metrics["maximum_controller_observation_age_ticks"], 40)
        self.assertEqual(metrics["rejected_decision_count"], 1)
        self.assertEqual(metrics["first_rejected_step"], 5)
        self.assertEqual(metrics["maximum_fail_safe_target_delta_rad"], 0.0)
        self.assertEqual(metrics["maximum_frozen_integral_delta_rad"], 0.0)
        self.assertEqual(metrics["recovery_decision_count"], 1)
        self.assertEqual(metrics["first_recovered_step"], 6)
        self.assertIsNone(metrics["first_controller_source_mismatch"])


if __name__ == "__main__":
    unittest.main()

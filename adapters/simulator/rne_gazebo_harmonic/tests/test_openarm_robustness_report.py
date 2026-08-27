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

    def test_command_rate_limit_is_recomputed_from_previous_applied_target(self) -> None:
        report_module = MODULE.load_controller_report_module(SCRIPT.parent)
        controller = {
            "disturbance_contract": {
                "kind": "actuator_command_slew_rate_limit_pulse_v1",
                "joint": "openarm_right_joint5",
                "start_step": 3,
                "end_step": 4,
                "maximum_rate_rad_s": 0.06,
            }
        }
        observations = []
        expected_applied = 0.0
        for step, commanded in enumerate([0.0, 0.0, 0.01, 0.02, 0.02, 0.02], 1):
            if 3 <= step <= 4:
                expected_applied = min(max(commanded, expected_applied - 0.001), expected_applied + 0.001)
            else:
                expected_applied = commanded
            observations.append(
                {
                    "step": step,
                    "joint_controller_target_rad": [commanded],
                    "joint_position_target_rad": [expected_applied],
                    "joint_actuator_disturbance_rad": [expected_applied - commanded],
                    "actuator_disturbance_active": expected_applied != commanded,
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
            "applied_target_delta_is_clamped_to_declared_rate_times_fixed_delta",
        )
        self.assertAlmostEqual(
            metrics["realization_verification"]["maximum_recomputed_applied_rate_rad_s"],
            0.06,
        )
        observations[3]["joint_position_target_rad"][0] += 0.001
        mismatch = report_module.disturbance_metrics(
            controller, observations, 0, 60.0
        )["first_realization_mismatch"]
        self.assertEqual(mismatch["step"], 4)

    def test_command_rate_limit_requirement_fails_below_minimum(self) -> None:
        controller = {
            "disturbance_contract": {
                "kind": "actuator_command_slew_rate_limit_pulse_v1",
                "start_step": 10,
                "end_step": 20,
                "maximum_rate_rad_s": 0.01,
            }
        }
        observations = [
            {"step": step, "sim_time_ticks": step * 100}
            for step in range(1, 21)
        ]
        requirement = {
            "id": "controller.actuator.minimum_command_slew_rate_rad_s",
            "unit": "rad/s",
            "minimum": 0.02,
        }
        violation = MODULE.command_rate_limit_violation(
            controller, observations, requirement
        )
        self.assertEqual(violation["step"], 10)
        self.assertEqual(violation["observed"], 0.01)
        self.assertEqual(violation["minimum"], 0.02)
        controller["disturbance_contract"]["maximum_rate_rad_s"] = 0.02
        self.assertIsNone(
            MODULE.command_rate_limit_violation(controller, observations, requirement)
        )

    def test_command_deadband_is_recomputed_from_previous_applied_target(self) -> None:
        report_module = MODULE.load_controller_report_module(SCRIPT.parent)
        controller = {
            "disturbance_contract": {
                "kind": "actuator_command_deadband_pulse_v1",
                "joint": "openarm_right_joint5",
                "start_step": 3,
                "end_step": 5,
                "deadband_rad": 0.002,
            }
        }
        commands = [0.0, 0.0, 0.001, 0.002, 0.003, 0.003]
        observations = []
        applied = 0.0
        for step, commanded in enumerate(commands, 1):
            if 3 <= step <= 5 and abs(commanded - applied) <= 0.002:
                next_applied = applied
            else:
                next_applied = commanded
            observations.append(
                {
                    "step": step,
                    "joint_controller_target_rad": [commanded],
                    "joint_position_target_rad": [next_applied],
                    "joint_actuator_disturbance_rad": [next_applied - commanded],
                    "actuator_disturbance_active": next_applied != commanded,
                    "joint_position_rad": [0.0],
                    "joint_reference_position_rad": [0.0],
                }
            )
            applied = next_applied
        metrics = report_module.disturbance_metrics(
            controller, observations, 0, 60.0
        )
        self.assertIsNone(metrics["first_realization_mismatch"])
        self.assertEqual(
            metrics["realization_verification"]["relationship"],
            "applied_target_holds_previous_value_within_declared_deadband",
        )
        self.assertEqual(
            metrics["realization_verification"]["maximum_recomputed_held_command_gap_rad"],
            0.002,
        )
        observations[2]["joint_position_target_rad"][0] = 0.001
        mismatch = report_module.disturbance_metrics(
            controller, observations, 0, 60.0
        )["first_realization_mismatch"]
        self.assertEqual(mismatch["step"], 3)

    def test_command_deadband_requirement_fails_above_maximum(self) -> None:
        controller = {
            "disturbance_contract": {
                "kind": "actuator_command_deadband_pulse_v1",
                "start_step": 10,
                "end_step": 20,
                "deadband_rad": 0.002,
            }
        }
        observations = [
            {"step": step, "sim_time_ticks": step * 100}
            for step in range(1, 21)
        ]
        requirement = {
            "id": "controller.actuator.maximum_command_deadband_rad",
            "unit": "rad",
            "maximum": 0.001,
        }
        violation = MODULE.command_deadband_violation(
            controller, observations, requirement
        )
        self.assertEqual(violation["step"], 10)
        self.assertEqual(violation["observed"], 0.002)
        controller["disturbance_contract"]["deadband_rad"] = 0.001
        self.assertIsNone(
            MODULE.command_deadband_violation(controller, observations, requirement)
        )

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

    def test_recovery_metrics_count_from_stale_rejection_until_fresh_resume(self) -> None:
        observations = []
        for step in range(1, 8):
            stale_rejected = step == 5
            recovery_hold = step == 6
            sequence = None if step <= 2 else (1 if step <= 5 else step - 1)
            observations.append(
                {
                    "step": step,
                    "sim_time_ticks": step * 10,
                    "sensor_sample_published": step not in {2, 3, 4},
                    "controller_observation_sequence": sequence,
                    "controller_observation_age_ticks": (
                        None if sequence is None else 40 if stale_rejected else 10
                    ),
                    "joint_position_rad": [step / 100.0],
                    "joint_reference_position_rad": [0.0],
                    "joint_controller_observation_position_rad": (
                        [] if sequence is None else [sequence / 100.0]
                    ),
                    "joint_controller_target_rad": [
                        0.3 if step == 7 else 0.2 if step >= 4 else 0.1
                    ],
                    "joint_integral_correction_rad": [0.03 if step >= 4 else 0.02],
                    "controller_rejected": stale_rejected or recovery_hold,
                    "controller_rejection_reason": (
                        "maximum_observation_age_ticks"
                        if stale_rejected
                        else "recovery_confirmation_pending"
                        if recovery_hold
                        else None
                    ),
                    "fail_safe_hold_active": stale_rejected or recovery_hold,
                    "controller_state_frozen": stale_rejected or recovery_hold,
                    "controller_recovered": step == 7,
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "joint_feedback_dropout_recovery_hold_v1",
                "start_capture_sequence": 2,
                "consecutive_dropped_frames": 3,
                "additional_recovery_hold_decisions": 1,
            }
        }
        metrics = MODULE.recovery_metrics(controller, observations, 0)
        self.assertIsNotNone(metrics)
        self.assertIsNone(metrics["first_hold_mismatch"])
        self.assertEqual(metrics["stale_rejected_decision_count"], 1)
        self.assertEqual(metrics["recovery_hold_rejected_decision_count"], 1)
        self.assertEqual(metrics["recovery_decision_count"], 2)
        self.assertEqual(metrics["first_recovered_step"], 7)
        requirements = {
            "controller.sensor.maximum_recovery_decisions": {
                "id": "recovery",
                "unit": "controller_decision_count",
                "maximum": 1,
            },
            "controller.sensor_recovery.maximum_controlled_joint_rmse_rad": {
                "id": "rmse",
                "unit": "rad",
                "maximum": 1.0,
            },
            "controller.sensor_recovery.maximum_controlled_joint_final_error_rad": {
                "id": "final",
                "unit": "rad",
                "maximum": 1.0,
            },
        }
        violation = MODULE.first_recovery_violation(
            metrics, observations, requirements
        )
        self.assertEqual(violation["requirement_id"], "recovery")
        self.assertEqual(violation["step"], 7)

    def test_rearm_metrics_find_missing_interburst_fresh_frame(self) -> None:
        observations = []
        for step in range(1, 8):
            sequence = None if step <= 2 else (1 if step <= 6 else 6)
            observations.append(
                {
                    "step": step,
                    "sim_time_ticks": step * 10,
                    "sensor_sample_published": step not in {2, 3, 4, 5},
                    "controller_observation_sequence": sequence,
                    "controller_observation_age_ticks": (
                        None if sequence is None else 10 * (step - sequence)
                    ),
                    "joint_position_rad": [step / 100.0],
                    "joint_reference_position_rad": [0.0],
                    "joint_controller_observation_position_rad": (
                        [] if sequence is None else [sequence / 100.0]
                    ),
                    "joint_controller_target_rad": [0.1],
                    "joint_integral_correction_rad": [0.02],
                    "controller_rejected": False,
                    "controller_rejection_reason": None,
                    "fail_safe_hold_active": False,
                    "controller_state_frozen": False,
                    "controller_recovered": False,
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "joint_feedback_repeated_dropout_bursts_v1",
                "start_capture_sequence": 2,
                "burst_length_frames": 2,
                "burst_count": 2,
                "interburst_fresh_frames": 0,
            }
        }
        metrics = MODULE.rearm_metrics(controller, observations, 0)
        self.assertIsNotNone(metrics)
        self.assertTrue(metrics["publication_realization_matches"])
        self.assertEqual(metrics["maximum_consecutive_dropout_frames"], 4)
        self.assertEqual(metrics["interburst_fresh_frames"], 0)
        self.assertIsNone(metrics["first_controller_source_mismatch"])
        requirements = {
            "controller.sensor.minimum_interburst_fresh_frames": {
                "id": "rearm",
                "unit": "consecutive_frame_count",
                "minimum": 1,
            },
            "controller.sensor.maximum_observation_age_ticks": {
                "id": "age",
                "unit": "tick",
                "maximum": 100,
            },
            "controller.sensor.maximum_fail_safe_target_delta_rad": {
                "id": "hold",
                "unit": "rad",
                "maximum": 1.0,
            },
            "controller.sensor.maximum_recovery_decisions": {
                "id": "recovery",
                "unit": "controller_decision_count",
                "maximum": 1,
            },
            "controller.sensor_rearm.maximum_controlled_joint_rmse_rad": {
                "id": "rmse",
                "unit": "rad",
                "maximum": 1.0,
            },
            "controller.sensor_rearm.maximum_controlled_joint_final_error_rad": {
                "id": "final",
                "unit": "rad",
                "maximum": 1.0,
            },
        }
        violation = MODULE.first_rearm_violation(metrics, observations, requirements)
        self.assertEqual(violation["requirement_id"], "rearm")
        self.assertEqual(violation["step"], 4)

    def test_quantization_metrics_recompute_visible_position(self) -> None:
        raw_positions = [0.001, 0.003, 0.005, 0.007, 0.009]
        observations = []
        for step, raw in enumerate(raw_positions, start=1):
            sequence = None if step == 1 else step - 1
            source_raw = None if sequence is None else raw_positions[sequence - 1]
            active = 3 <= step <= 4
            visible = []
            if source_raw is not None:
                expected = (
                    MODULE.math.copysign(
                        MODULE.math.floor(abs(source_raw) / 0.004 + 0.5) * 0.004,
                        source_raw,
                    )
                    if active
                    else source_raw
                )
                visible = [expected]
            observations.append(
                {
                    "step": step,
                    "sim_time_ticks": step * 10,
                    "controller_observation_sequence": sequence,
                    "controller_observation_age_ticks": None if sequence is None else 10,
                    "joint_position_rad": [raw],
                    "joint_reference_position_rad": [0.0],
                    "joint_controller_observation_position_rad": visible,
                    "controller_rejected": False,
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "joint_position_quantization_pulse_v1",
                "start_controller_step": 3,
                "end_controller_step": 4,
                "quantization_step_rad": 0.004,
            }
        }
        metrics = MODULE.quantization_metrics(controller, observations, 0)
        self.assertIsNotNone(metrics)
        self.assertEqual(metrics["active_decision_count"], 2)
        self.assertAlmostEqual(metrics["maximum_quantization_error_rad"], 0.001)
        self.assertEqual(metrics["maximum_realization_delta_rad"], 0.0)
        self.assertIsNone(metrics["first_realization_mismatch"])
        requirements = {
            "controller.sensor.maximum_position_quantization_step_rad": {
                "id": "quantization",
                "unit": "rad",
                "maximum": 0.002,
            },
            "controller.sensor.maximum_quantization_realization_delta_rad": {
                "id": "realization",
                "unit": "rad",
                "maximum": 1e-12,
            },
            "controller.sensor_quantization.maximum_controlled_joint_rmse_rad": {
                "id": "rmse",
                "unit": "rad",
                "maximum": 1.0,
            },
            "controller.sensor_quantization.maximum_controlled_joint_final_error_rad": {
                "id": "final",
                "unit": "rad",
                "maximum": 1.0,
            },
        }
        violation = MODULE.first_quantization_violation(
            metrics, observations, requirements
        )
        self.assertEqual(violation["requirement_id"], "quantization")
        self.assertEqual(violation["step"], 3)

    def test_latency_metrics_preserve_capture_time_and_find_ingress_boundary(self) -> None:
        observations = []
        for step in range(1, 7):
            sequence = None if step <= 3 else step - 3
            observations.append(
                {
                    "step": step,
                    "sim_time_ticks": step * 10,
                    "sensor_sample_published": True,
                    "controller_observation_sequence": sequence,
                    "controller_observation_age_ticks": None if sequence is None else 20,
                    "controller_bootstrap": sequence is None,
                    "controller_rejected": False,
                    "joint_position_rad": [step / 100.0],
                    "joint_reference_position_rad": [0.0],
                    "joint_controller_observation_position_rad": (
                        [] if sequence is None else [sequence / 100.0]
                    ),
                    "joint_controller_target_rad": [0.1],
                    "joint_integral_correction_rad": [0.0],
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "joint_feedback_controller_ingress_delay_v1",
                "delay_frames": 1,
            }
        }
        metrics = MODULE.latency_metrics(controller, observations, 0, 10)
        self.assertIsNotNone(metrics)
        self.assertIsNone(metrics["first_realization_mismatch"])
        self.assertEqual(metrics["bootstrap_decision_count"], 3)
        self.assertEqual(metrics["maximum_controller_observation_age_ticks"], 20)
        self.assertAlmostEqual(
            metrics["controlled_joint_rmse_rad"],
            (91 / 6) ** 0.5 / 100,
        )
        requirements = {
            "controller.sensor.maximum_controller_ingress_delay_frames": {
                "id": "ingress",
                "unit": "control_period_count",
                "maximum": 0,
            },
            "controller.sensor.maximum_observation_age_ticks": {
                "id": "age",
                "unit": "tick",
                "maximum": 30,
            },
            "controller.sensor_latency.maximum_controlled_joint_rmse_rad": {
                "id": "rmse",
                "unit": "rad",
                "maximum": 1.0,
            },
            "controller.sensor_latency.maximum_controlled_joint_final_error_rad": {
                "id": "final",
                "unit": "rad",
                "maximum": 1.0,
            },
        }
        violation = MODULE.first_latency_violation(
            metrics, observations, requirements
        )
        self.assertEqual(violation["requirement_id"], "ingress")
        self.assertEqual(violation["step"], 4)

    def test_jitter_metrics_recompute_schedule_and_first_peak(self) -> None:
        sequences = [None, None, 1, 1, 1, 4, 4, 4]
        ages = [None, None, 10, 20, 30, 10, 20, 30]
        observations = []
        for step, (sequence, age) in enumerate(zip(sequences, ages), 1):
            observations.append(
                {
                    "step": step,
                    "sim_time_ticks": step * 10,
                    "sensor_sample_published": True,
                    "controller_observation_sequence": sequence,
                    "controller_observation_age_ticks": age,
                    "controller_bootstrap": sequence is None,
                    "controller_rejected": False,
                    "joint_position_rad": [step / 100.0],
                    "joint_reference_position_rad": [0.0],
                    "joint_controller_observation_position_rad": (
                        [] if sequence is None else [sequence / 100.0]
                    ),
                    "joint_controller_target_rad": [0.1],
                    "joint_integral_correction_rad": [0.0],
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "joint_feedback_controller_ingress_jitter_pulse_v1",
                "maximum_jitter_frames": 2,
                "start_capture_sequence": 2,
                "end_capture_sequence": 6,
                "schedule": "maximum_delay_for_n_frames_then_nominal_v1",
            }
        }
        metrics = MODULE.jitter_metrics(controller, observations, 0, 10)
        self.assertIsNotNone(metrics)
        self.assertIsNone(metrics["first_realization_mismatch"])
        self.assertEqual(metrics["maximum_realized_jitter_frames"], 2)
        self.assertEqual(metrics["jittered_capture_count"], 4)
        requirements = {
            "controller.sensor.maximum_controller_ingress_jitter_frames": {
                "id": "jitter",
                "unit": "control_period_count",
                "maximum": 1,
            },
            "controller.sensor.maximum_observation_age_ticks": {
                "id": "age",
                "unit": "tick",
                "maximum": 30,
            },
            "controller.sensor_jitter.maximum_controlled_joint_rmse_rad": {
                "id": "rmse",
                "unit": "rad",
                "maximum": 1.0,
            },
            "controller.sensor_jitter.maximum_controlled_joint_final_error_rad": {
                "id": "final",
                "unit": "rad",
                "maximum": 1.0,
            },
        }
        violation = MODULE.first_jitter_violation(
            metrics, observations, requirements
        )
        self.assertEqual(violation["requirement_id"], "jitter")
        self.assertEqual(violation["step"], 5)

    def test_stale_age_metrics_separate_old_selection_hold_and_recovery(self) -> None:
        sequences = [None, None, 1, 2, 3, 1, 2, 3, 7, 8]
        observations = []
        for step, sequence in enumerate(sequences, 1):
            rejected = 6 <= step <= 8
            observations.append(
                {
                    "step": step,
                    "sim_time_ticks": step * 10,
                    "sensor_sample_published": True,
                    "controller_observation_sequence": sequence,
                    "controller_observation_age_ticks": (
                        None if sequence is None else (step - 1 - sequence) * 10
                    ),
                    "controller_bootstrap": sequence is None,
                    "controller_rejected": rejected,
                    "controller_rejection_reason": (
                        "maximum_observation_age_ticks" if rejected else None
                    ),
                    "fail_safe_hold_active": rejected,
                    "controller_state_frozen": rejected,
                    "controller_recovered": step == 9,
                    "joint_position_rad": [step / 100.0],
                    "joint_reference_position_rad": [0.0],
                    "joint_controller_observation_position_rad": (
                        [] if sequence is None else [sequence / 100.0]
                    ),
                    "joint_controller_target_rad": [0.1],
                    "joint_integral_correction_rad": [0.02],
                }
            )
        controller = {
            "measurement_fault_contract": {
                "kind": "joint_feedback_controller_stale_age_pulse_v1",
                "additional_stale_frames": 3,
                "start_controller_step": 6,
                "end_controller_step": 8,
                "selection_policy": "nth_older_available_publication_v1",
            }
        }
        metrics = MODULE.stale_age_metrics(controller, observations, 0, 10)
        self.assertIsNotNone(metrics)
        self.assertIsNone(metrics["first_realization_mismatch"])
        self.assertIsNone(metrics["first_hold_mismatch"])
        self.assertEqual(metrics["maximum_selected_stale_frames"], 3)
        self.assertEqual(metrics["maximum_controller_observation_age_ticks"], 40)
        self.assertEqual(metrics["rejected_decision_count"], 3)
        self.assertEqual(metrics["recovery_decision_count"], 1)
        requirements = {
            "controller.sensor.maximum_observation_age_ticks": {
                "id": "age",
                "unit": "tick",
                "maximum": 30,
            },
            "controller.sensor_stale.maximum_controlled_joint_rmse_rad": {
                "id": "rmse",
                "unit": "rad",
                "maximum": 1.0,
            },
            "controller.sensor_stale.maximum_controlled_joint_final_error_rad": {
                "id": "final",
                "unit": "rad",
                "maximum": 1.0,
            },
        }
        violation = MODULE.first_stale_age_violation(
            metrics, observations, requirements
        )
        self.assertEqual(violation["requirement_id"], "age")
        self.assertEqual(violation["step"], 6)


if __name__ == "__main__":
    unittest.main()

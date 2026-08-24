from __future__ import annotations

import sys
from pathlib import Path
import unittest


SCRIPT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR))
from openarm_actuation import (  # noqa: E402
    ActuationDiagnosticAccumulator,
    low_pass_velocity,
    realize_joint_command,
    realize_joint_command_diagnostic,
    validate_actuation,
)


def effort_config() -> dict:
    return {
        "actuation_mode": "effort_pd",
        "physics_substeps_per_control_step": 10,
        "effort_joint_indices": [0, 1],
        "position_gain_s_inv": 8.0,
        "maximum_velocity_rad_s": 2.0,
        "stiffness_nm_per_rad": [10.0, 20.0, 30.0],
        "damping_nm_s_per_rad": [1.0, 2.0, 3.0],
        "maximum_effort_nm": [4.0, 5.0, 6.0],
        "saturation_behavior": "clamp_each_joint_effort_before_pre_update",
        "failure_behavior": "reject_invalid_configuration_before_simulator_start",
    }


class OpenArmActuationTests(unittest.TestCase):
    def test_effort_pd_declares_limits_failure_behavior_and_substeps(self) -> None:
        self.assertEqual(
            validate_actuation(effort_config(), 3),
            ("effort_pd", 10, frozenset({0, 1})),
        )
        config = effort_config()
        config["failure_behavior"] = "continue"
        with self.assertRaisesRegex(ValueError, "failure behavior"):
            validate_actuation(config, 3)
        config = effort_config()
        config["derivative_filter_kind"] = "first_order_low_pass_backward_euler_v1"
        config["derivative_filter_time_constant_s"] = 0.0
        with self.assertRaisesRegex(ValueError, "time constant"):
            validate_actuation(config, 3)

    def test_effort_is_damped_and_saturated_per_joint(self) -> None:
        config = effort_config()
        self.assertEqual(
            realize_joint_command(
                config, "effort_pd", frozenset({0, 1}), 0, 1.0, 0.0, 0.0
            ),
            ("effort_nm", 4.0),
        )
        self.assertEqual(
            realize_joint_command(
                config, "effort_pd", frozenset({0, 1}), 1, 0.0, 0.0, 3.0
            ),
            ("effort_nm", -5.0),
        )

    def test_non_effort_joints_retain_bounded_velocity_servo(self) -> None:
        config = effort_config()
        self.assertEqual(
            realize_joint_command(
                config, "effort_pd", frozenset({0, 1}), 2, 1.0, 0.0, 0.0
            ),
            ("velocity_rad_s", 2.0),
        )

    def test_diagnostic_reports_actual_substep_effort_and_saturation(self) -> None:
        config = effort_config()
        accumulator = ActuationDiagnosticAccumulator(1)
        first = realize_joint_command_diagnostic(
            config, "effort_pd", frozenset({0}), 0, 1.0, 0.0, 0.0
        )
        second = realize_joint_command_diagnostic(
            config, "effort_pd", frozenset({0}), 0, 0.1, 0.0, 0.0
        )
        accumulator.record(0, first, 1.0, 3.0, 1.0)
        accumulator.record(0, second, 0.1, -2.0, 0.5)
        diagnostic = accumulator.finish(2, [0.05])
        self.assertEqual(diagnostic["joint_command_kind"], ["effort_nm"])
        self.assertEqual(diagnostic["joint_raw_command_peak_abs"], [10.0])
        self.assertEqual(diagnostic["joint_applied_command_mean"], [2.5])
        self.assertEqual(diagnostic["joint_saturation_substep_count"], [1])
        self.assertEqual(diagnostic["joint_saturation_fraction"], [0.5])
        self.assertEqual(diagnostic["joint_initial_position_error_rad"], [1.0])
        self.assertEqual(diagnostic["joint_final_position_error_rad"], [0.05])
        self.assertEqual(diagnostic["joint_measured_velocity_peak_abs_rad_s"], [3.0])
        self.assertEqual(
            diagnostic["joint_derivative_feedback_velocity_peak_abs_rad_s"], [1.0]
        )

    def test_diagnostic_rejects_missing_substeps(self) -> None:
        accumulator = ActuationDiagnosticAccumulator(1)
        command = realize_joint_command_diagnostic(
            effort_config(), "effort_pd", frozenset({0}), 0, 0.1, 0.0, 0.0
        )
        accumulator.record(0, command, 0.1, 0.0, 0.0)
        with self.assertRaisesRegex(ValueError, "substeps"):
            accumulator.finish(2, [0.0])

    def test_backward_euler_velocity_filter_is_bounded_and_deterministic(self) -> None:
        self.assertAlmostEqual(low_pass_velocity(0.0, 13.0, 0.02, 0.02), 6.5)
        self.assertAlmostEqual(low_pass_velocity(6.5, 13.0, 0.02, 0.02), 9.75)
        with self.assertRaisesRegex(ValueError, "filter"):
            low_pass_velocity(0.0, 1.0, 0.0, 0.02)

if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import sys
from pathlib import Path
import unittest


SCRIPT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR))
from openarm_actuation import realize_joint_command, validate_actuation  # noqa: E402


def effort_config() -> dict:
    return {
        "actuation_mode": "effort_pd",
        "physics_substeps_per_control_step": 10,
        "effort_joint_count": 2,
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
        self.assertEqual(validate_actuation(effort_config(), 3), ("effort_pd", 10, 2))
        config = effort_config()
        config["failure_behavior"] = "continue"
        with self.assertRaisesRegex(ValueError, "failure behavior"):
            validate_actuation(config, 3)

    def test_effort_is_damped_and_saturated_per_joint(self) -> None:
        config = effort_config()
        self.assertEqual(
            realize_joint_command(config, "effort_pd", 2, 0, 1.0, 0.0, 0.0),
            ("effort_nm", 4.0),
        )
        self.assertEqual(
            realize_joint_command(config, "effort_pd", 2, 1, 0.0, 0.0, 3.0),
            ("effort_nm", -5.0),
        )

    def test_non_effort_joints_retain_bounded_velocity_servo(self) -> None:
        config = effort_config()
        self.assertEqual(
            realize_joint_command(config, "effort_pd", 2, 2, 1.0, 0.0, 0.0),
            ("velocity_rad_s", 2.0),
        )


if __name__ == "__main__":
    unittest.main()

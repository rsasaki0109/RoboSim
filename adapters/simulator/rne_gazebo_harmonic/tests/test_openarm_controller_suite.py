from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "adapters/simulator/rne_gazebo_harmonic/build_openarm_controller_suite.py"
SPEC = importlib.util.spec_from_file_location("openarm_controller_suite", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
RUNNER_SCRIPT = ROOT / "adapters/simulator/rne_gazebo_harmonic/run_openarm_trace.py"
RUNNER_SPEC = importlib.util.spec_from_file_location("openarm_gazebo_runner", RUNNER_SCRIPT)
assert RUNNER_SPEC is not None and RUNNER_SPEC.loader is not None
RUNNER = importlib.util.module_from_spec(RUNNER_SPEC)
RUNNER_SPEC.loader.exec_module(RUNNER)
REPORT = (
    ROOT
    / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json"
)
MANIFEST = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_experiments.json"
)
LIMITS = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_right_pose_cycle.controller.json"
)


class OpenArmControllerSuiteTests(unittest.TestCase):
    def setUp(self) -> None:
        self.suite, self.controllers = MODULE.compile_suite(REPORT, MANIFEST, LIMITS)
        self.pid = self.controllers["pid"]
        self.state = self.controllers["state_feedback"]
        self.order = self.pid["action_joint_order"]
        self.joint_index = self.order.index(MODULE.JOINT)

    def test_compilation_is_deterministic_and_preserves_the_reference(self) -> None:
        self.assertEqual(
            (self.suite, self.controllers),
            MODULE.compile_suite(REPORT, MANIFEST, LIMITS),
        )
        self.assertEqual(self.pid["keyframes"], self.state["keyframes"])
        self.assertEqual(self.pid["intentional_failure"], self.state["intentional_failure"])
        self.assertEqual(self.suite["observation_latency_samples"], 1)

    def test_pid_controls_only_joint5_under_shared_limits(self) -> None:
        law = self.pid["feedback_law"]
        for field in (
            "position_error_gain",
            "velocity_damping_s",
            "integral_error_gain_s_inv",
            "maximum_integral_correction_rad",
            "maximum_correction_rad",
        ):
            nonzero = [index for index, value in enumerate(law[field]) if value != 0.0]
            self.assertEqual(nonzero, [self.joint_index])
        self.assertEqual(
            law["maximum_correction_rad"][self.joint_index],
            self.suite["shared_maximum_correction_rad"],
        )

    def test_state_feedback_is_controllable_and_places_stable_declared_poles(self) -> None:
        law = self.state["feedback_law"]
        model = law["identified_plant"]
        self.assertGreater(abs(model["controllability_determinant"]), 1e-10)
        self.assertEqual(
            model["observability_scope"],
            "dynamic_output_state_with_known_input_history_v1",
        )
        self.assertGreater(abs(model["observability_determinant"]), 1e-10)
        closed_loop = law["closed_loop_a"]
        for pole in law["desired_closed_loop_poles"]:
            characteristic = [
                [
                    pole * float(row == column) - closed_loop[row][column]
                    for column in range(4)
                ]
                for row in range(4)
            ]
            self.assertLess(abs(MODULE.determinant(characteristic)), 1e-8)
            self.assertLess(abs(pole), 1.0)
        self.assertGreater(law["integral_state_feedback_gain_s_inv"], 0.0)
        self.assertTrue(math.isfinite(law["maximum_integral_state_error_rad_s"]))

    def test_state_model_is_bound_to_retained_rapier_arx_without_refit(self) -> None:
        report = json.loads(REPORT.read_text(encoding="utf-8"))
        rapier = next(
            backend for backend in report["backends"] if backend["backend_id"] == "rne_rapier"
        )
        model = self.state["feedback_law"]["identified_plant"]
        self.assertEqual(model["source_backend_id"], "rne_rapier")
        self.assertEqual(model["arx_coefficients"], rapier["arx_model"]["coefficients"])
        self.assertEqual(model["source_report_sha256"], MODULE.sha256(REPORT))
        self.assertEqual(
            self.state["feedback_law"]["maximum_state_feedback_correction_rad"],
            self.suite["shared_maximum_correction_rad"],
        )

    def test_state_controller_uses_bounded_delayed_observation_prediction(self) -> None:
        width = len(self.order)
        reference = self.state["keyframes"][361]["joint_position_target_rad"].copy()
        observation = {
            "step": 359,
            "sim_time_ticks": 359 * self.state["observation_contract"]["sample_period_ticks"],
            "sensor_status": "nominal",
            "joint_position_rad": [0.0] * width,
            "joint_velocity_rad_s": [0.0] * width,
            "joint_position_target_rad": [0.0] * width,
        }
        integral = [0.0] * width
        previous_position: list[float | None] = [None] * width
        previous_input: list[float | None] = [None] * width
        previous_previous_input: list[float | None] = [None] * width
        previous_input[self.joint_index] = 0.03
        previous_previous_input[self.joint_index] = 0.01
        decision = RUNNER.controller_decision(
            self.state,
            reference,
            integral,
            previous_position,
            previous_input,
            previous_previous_input,
            observation,
            observation["sim_time_ticks"]
            + self.state["observation_contract"]["maximum_age_ticks"],
        )
        self.assertLessEqual(
            abs(decision["correction"][self.joint_index]),
            self.suite["shared_maximum_correction_rad"],
        )
        self.assertGreater(decision["integral_correction"][self.joint_index], 0.0)
        self.assertEqual(previous_position[self.joint_index], 0.0)
        self.assertEqual(
            previous_input[self.joint_index], decision["target"][self.joint_index]
        )


if __name__ == "__main__":
    unittest.main()

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

    def test_gazebo_initialization_has_bounded_cold_start_timeout(self) -> None:
        self.assertEqual(RUNNER.response_timeout_s(0), 120.0)
        self.assertEqual(RUNNER.response_timeout_s(1), 120.0)
        self.assertEqual(RUNNER.response_timeout_s(2), 30.0)
        with self.assertRaisesRegex(ValueError, "non-negative"):
            RUNNER.response_timeout_s(-1)

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
        self.assertEqual(MODULE.rate_limit_case_id(0.15), "rate-150mrad-s")
        with self.assertRaisesRegex(ValueError, "milliradians per second"):
            MODULE.rate_limit_case_id(0.0005)
        self.assertEqual(MODULE.deadband_case_id(0.001), "deadband-1000urad")
        with self.assertRaisesRegex(ValueError, "whole microradians"):
            MODULE.deadband_case_id(0.0000005)
        self.assertEqual(MODULE.sensor_latency_case_id(3), "sensor-latency-003frames")

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

    def test_command_delay_uses_the_declared_controller_source_step(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "actuator_command_delay",
        )
        self.assertEqual(suite["dimension_id"], "actuator_command_delay")
        self.assertEqual(
            [
                controller["disturbance_contract"]["delay_steps"]
                for controller in controllers.values()
            ],
            [0, 1, 2, 3, 4],
        )
        controller = controllers["delay-002steps"]
        width = len(controller["action_joint_order"])
        joint_index = controller["action_joint_order"].index("openarm_right_joint5")
        start_step = controller["disturbance_contract"]["start_step"]
        history = [[step / start_step] * width for step in range(1, start_step + 1)]
        current = history[-1]
        applied, disturbance = RUNNER.apply_actuator_disturbance(
            controller, start_step, current, history, history[:-1]
        )
        self.assertEqual(applied[joint_index], (start_step - 2) / start_step)
        self.assertEqual(
            disturbance[joint_index], (start_step - 2) / start_step - 1.0
        )
        self.assertTrue(
            all(
                applied[index] == current[index]
                for index in range(width)
                if index != joint_index
            )
        )

    def test_sensor_latency_grid_retains_capture_time_and_bounded_age_policy(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_feedback_controller_ingress_latency",
        )
        self.assertEqual(
            suite["dimension_id"], "joint_feedback_controller_ingress_latency"
        )
        self.assertEqual(
            [
                controller["measurement_fault_contract"]["delay_frames"]
                for controller in controllers.values()
            ],
            [0, 1, 2, 3, 4],
        )
        controller = controllers["sensor-latency-003frames"]
        self.assertEqual(
            controller["measurement_fault_contract"]["controller_visibility"],
            "delayed_nominal_publication_with_original_capture_timestamp",
        )
        self.assertEqual(
            controller["observation_contract"]["maximum_age_ticks"],
            3 * 16_666_667,
        )
        self.assertEqual(
            controller["observation_contract"]["stale_observation_policy"],
            "hold_last_accepted_target_and_freeze_state",
        )
        RUNNER.validate_measurement_fault(controller, 3600)
        self.assertEqual(RUNNER.controller_ingress_delay_frames(controller), 3)

    def test_sensor_jitter_grid_uses_deterministic_burst_schedule(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_feedback_controller_ingress_jitter",
        )
        self.assertEqual(
            suite["dimension_id"], "joint_feedback_controller_ingress_jitter"
        )
        self.assertEqual(
            [
                controller["measurement_fault_contract"]["maximum_jitter_frames"]
                for controller in controllers.values()
            ],
            [0, 1, 2, 3, 4],
        )
        controller = controllers["sensor-jitter-002frames"]
        contract = controller["measurement_fault_contract"]
        self.assertEqual(
            contract["schedule"], "maximum_delay_for_n_frames_then_nominal_v1"
        )
        RUNNER.validate_measurement_fault(controller, 3600)
        start = contract["start_capture_sequence"]
        self.assertEqual(
            [
                RUNNER.controller_ingress_delay_frames(controller, start + offset)
                for offset in range(6)
            ],
            [2, 2, 0, 2, 2, 0],
        )

    def test_sensor_stale_age_grid_selects_older_available_publication(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_feedback_controller_stale_age",
        )
        self.assertEqual(
            suite["dimension_id"], "joint_feedback_controller_stale_age"
        )
        self.assertEqual(
            [
                controller["measurement_fault_contract"]["additional_stale_frames"]
                for controller in controllers.values()
            ],
            [0, 1, 2, 3, 4],
        )
        controller = controllers["sensor-stale-003frames"]
        contract = controller["measurement_fault_contract"]
        self.assertEqual(
            contract["selection_policy"], "nth_older_available_publication_v1"
        )
        RUNNER.validate_measurement_fault(controller, 3600)
        self.assertEqual(
            RUNNER.controller_stale_offset_frames(
                controller, contract["start_controller_step"] - 1
            ),
            0,
        )
        self.assertEqual(
            RUNNER.controller_stale_offset_frames(
                controller, contract["start_controller_step"]
            ),
            3,
        )
        self.assertEqual(
            RUNNER.controller_stale_offset_frames(
                controller, contract["end_controller_step"] + 1
            ),
            0,
        )

    def test_sensor_recovery_grid_delays_only_fresh_observation_resume(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_feedback_dropout_recovery",
        )
        self.assertEqual(suite["dimension_id"], "joint_feedback_dropout_recovery")
        self.assertEqual(
            [
                controller["measurement_fault_contract"][
                    "additional_recovery_hold_decisions"
                ]
                for controller in controllers.values()
            ],
            [0, 1, 2, 3, 4],
        )
        controller = controllers["sensor-recovery-001decisions"]
        contract = controller["measurement_fault_contract"]
        self.assertEqual(contract["consecutive_dropped_frames"], 3)
        self.assertEqual(
            controller["observation_contract"]["recovery_policy"],
            "resume_after_configured_fresh_observations",
        )
        RUNNER.validate_measurement_fault(controller, 3600)
        self.assertFalse(RUNNER.sensor_sample_published(controller, 3240))
        self.assertFalse(RUNNER.sensor_sample_published(controller, 3242))
        self.assertTrue(RUNNER.sensor_sample_published(controller, 3243))

    def test_repeated_dropout_grid_rearms_on_one_fresh_frame(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_feedback_repeated_dropout_rearm",
        )
        self.assertEqual(
            suite["dimension_id"], "joint_feedback_repeated_dropout_rearm"
        )
        self.assertEqual(
            [
                controller["measurement_fault_contract"]["interburst_fresh_frames"]
                for controller in controllers.values()
            ],
            [4, 3, 2, 1, 0],
        )
        self.assertEqual(
            list(controllers),
            [
                "sensor-rearm-004fresh",
                "sensor-rearm-003fresh",
                "sensor-rearm-002fresh",
                "sensor-rearm-001fresh",
                "sensor-rearm-000fresh",
            ],
        )
        one_fresh = controllers["sensor-rearm-001fresh"]
        RUNNER.validate_measurement_fault(one_fresh, 3600)
        self.assertEqual(
            [RUNNER.sensor_sample_published(one_fresh, step) for step in range(3240, 3245)],
            [False, False, True, False, False],
        )
        no_fresh = controllers["sensor-rearm-000fresh"]
        RUNNER.validate_measurement_fault(no_fresh, 3600)
        self.assertEqual(
            [RUNNER.sensor_sample_published(no_fresh, step) for step in range(3240, 3244)],
            [False, False, False, False],
        )

    def test_sensor_quantization_grid_preserves_raw_feedback(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_position_measurement_quantization",
        )
        self.assertEqual(
            suite["dimension_id"], "joint_position_measurement_quantization"
        )
        self.assertEqual(
            [
                controller["measurement_fault_contract"]["quantization_step_rad"]
                for controller in controllers.values()
            ],
            [0.0, 0.001, 0.002, 0.004, 0.008],
        )
        controller = controllers["sensor-quantization-04000urad"]
        RUNNER.validate_measurement_fault(controller, 3600)
        width = len(controller["action_joint_order"])
        joint_index = controller["action_joint_order"].index("openarm_right_joint5")
        raw = [0.0] * width
        raw[joint_index] = -0.003
        observation = {"joint_position_rad": raw.copy()}
        contract = controller["measurement_fault_contract"]
        sample_ticks = controller["observation_contract"]["sample_period_ticks"]
        visible, error = RUNNER.apply_measurement_bias(
            controller,
            observation,
            (contract["start_controller_step"] - 1) * sample_ticks,
        )
        self.assertEqual(observation["joint_position_rad"][joint_index], -0.003)
        self.assertEqual(visible[joint_index], -0.004)
        self.assertAlmostEqual(error[joint_index], -0.001)

    def test_sensor_saturation_grid_clamps_controller_visible_position(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_position_measurement_saturation",
        )
        self.assertEqual(
            suite["dimension_id"], "joint_position_measurement_saturation"
        )
        self.assertEqual(
            [
                controller["measurement_fault_contract"][
                    "saturation_limit_abs_rad"
                ]
                for controller in controllers.values()
            ],
            [0.08, 0.06, 0.05, 0.04, 0.03],
        )
        controller = controllers["sensor-saturation-040mrad"]
        RUNNER.validate_measurement_fault(controller, 3600)
        width = len(controller["action_joint_order"])
        joint_index = controller["action_joint_order"].index("openarm_right_joint5")
        raw = [0.0] * width
        raw[joint_index] = 0.06
        observation = {"joint_position_rad": raw.copy()}
        contract = controller["measurement_fault_contract"]
        sample_ticks = controller["observation_contract"]["sample_period_ticks"]
        visible, error = RUNNER.apply_measurement_bias(
            controller,
            observation,
            (contract["start_controller_step"] - 1) * sample_ticks,
        )
        self.assertEqual(observation["joint_position_rad"][joint_index], 0.06)
        self.assertEqual(visible[joint_index], 0.04)
        self.assertAlmostEqual(error[joint_index], -0.02)

    def test_sensor_stuck_grid_holds_last_nominal_controller_value(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "joint_position_stuck_value",
        )
        self.assertEqual(suite["dimension_id"], "joint_position_stuck_value")
        self.assertEqual(
            [
                controller["measurement_fault_contract"][
                    "consecutive_stuck_frames"
                ]
                for controller in controllers.values()
            ],
            [0, 1, 2, 3, 4],
        )
        controller = controllers["sensor-stuck-003frames"]
        RUNNER.validate_measurement_fault(controller, 3600)
        width = len(controller["action_joint_order"])
        joint_index = controller["action_joint_order"].index("openarm_right_joint5")
        observations = []
        for sequence in range(1, 901):
            position = [0.0] * width
            position[joint_index] = sequence / 1000.0
            observations.append(
                {
                    "step": sequence,
                    "sensor_status": RUNNER.sensor_status_for_sequence(
                        controller, sequence
                    ),
                    "joint_position_rad": position,
                    "joint_velocity_rad_s": [0.0] * width,
                }
            )
        transformed = RUNNER.stuck_controller_observation(
            controller, observations, observations[-1]
        )
        self.assertEqual(transformed["sensor_status"], "stuck_value")
        self.assertEqual(transformed["step"], 900)
        self.assertEqual(transformed["joint_position_rad"][joint_index], 0.899)
        self.assertEqual(observations[-1]["joint_position_rad"][joint_index], 0.9)
        self.assertEqual(
            [RUNNER.sensor_status_for_sequence(controller, sequence) for sequence in range(899, 905)],
            ["nominal", "stuck_value", "stuck_value", "stuck_value", "nominal", "nominal"],
        )

    def test_command_rate_limit_uses_previous_applied_target_and_fixed_delta(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "actuator_command_rate_limit",
        )
        self.assertEqual(suite["dimension_id"], "actuator_command_rate_limit")
        self.assertEqual(
            [
                controller["disturbance_contract"]["maximum_rate_rad_s"]
                for controller in controllers.values()
            ],
            [0.4, 0.25, 0.15, 0.1, 0.05],
        )
        controller = controllers["rate-050mrad-s"]
        width = len(controller["action_joint_order"])
        joint_index = controller["action_joint_order"].index("openarm_right_joint5")
        start_step = controller["disturbance_contract"]["start_step"]
        previous = [0.0] * width
        current = [0.0] * width
        current[joint_index] = 0.1
        applied, disturbance = RUNNER.apply_actuator_disturbance(
            controller,
            start_step,
            current,
            [current] * start_step,
            [previous] * (start_step - 1),
        )
        maximum_delta_rad = 0.05 * RUNNER.FIXED_DELTA_TICKS / 1_000_000_000.0
        self.assertAlmostEqual(applied[joint_index], maximum_delta_rad)
        self.assertAlmostEqual(
            disturbance[joint_index], maximum_delta_rad - current[joint_index]
        )
        self.assertTrue(
            all(
                applied[index] == current[index]
                for index in range(width)
                if index != joint_index
            )
        )

    def test_command_deadband_holds_previous_applied_target_inside_band(self) -> None:
        suite, controllers = MODULE.compile_robustness_suite(
            COMPILER,
            SCRIPT_DIR / "openarm_robustness_experiments.json",
            ROOT / "docs/evidence/openarm-plant-lab/evidence/openarm-plant-lab-report.json",
            SCRIPT_DIR / "openarm_plant_experiments.json",
            SCRIPT_DIR / "openarm_right_pose_cycle.controller.json",
            SCRIPT_DIR / "openarm_controller_requirements.json",
            "actuator_command_deadband",
        )
        self.assertEqual(suite["dimension_id"], "actuator_command_deadband")
        self.assertEqual(
            [
                controller["disturbance_contract"]["deadband_rad"]
                for controller in controllers.values()
            ],
            [0.0, 0.00025, 0.0005, 0.001, 0.002],
        )
        controller = controllers["deadband-2000urad"]
        width = len(controller["action_joint_order"])
        joint_index = controller["action_joint_order"].index("openarm_right_joint5")
        start_step = controller["disturbance_contract"]["start_step"]
        previous = [0.0] * width
        current = [0.0] * width
        current[joint_index] = 0.001
        applied, disturbance = RUNNER.apply_actuator_disturbance(
            controller,
            start_step,
            current,
            [current] * start_step,
            [previous] * (start_step - 1),
        )
        self.assertEqual(applied[joint_index], 0.0)
        self.assertEqual(disturbance[joint_index], -0.001)
        self.assertTrue(
            all(
                applied[index] == current[index]
                for index in range(width)
                if index != joint_index
            )
        )


if __name__ == "__main__":
    unittest.main()

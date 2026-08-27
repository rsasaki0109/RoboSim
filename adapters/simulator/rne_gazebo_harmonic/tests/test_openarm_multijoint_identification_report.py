from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_multijoint_identification_report.py"
)
SPEC = importlib.util.spec_from_file_location("openarm_multijoint_report", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
MANIFEST = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_multijoint_identification_experiments.json"
)
REQUIREMENTS = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_multijoint_identification_requirements.json"
)


class OpenArmMultijointIdentificationReportTests(unittest.TestCase):
    def test_mimo_fit_uses_isolated_training_and_held_out_coupled_validation(self) -> None:
        def rollout(offset: float, count: int) -> tuple[list[list[float]], list[list[float]]]:
            inputs = [
                [
                    math.sin((index + offset) * 0.17)
                    + 0.2 * math.sin((index + offset) * 0.047),
                    math.cos((index + offset) * 0.11)
                    + 0.15 * math.sin((index + offset) * 0.071),
                ]
                for index in range(count)
            ]
            outputs = [[0.0, 0.0], [0.0, 0.0]]
            for index in range(2, count):
                outputs.append(
                    [
                        0.72 * outputs[-1][0]
                        - 0.08 * outputs[-2][0]
                        + 0.18 * inputs[index][0]
                        + 0.04 * inputs[index][1]
                        + 0.01,
                        0.61 * outputs[-1][1]
                        - 0.05 * outputs[-2][1]
                        + 0.03 * inputs[index][0]
                        + 0.21 * inputs[index][1]
                        - 0.005,
                    ]
                )
            return inputs, outputs

        training_inputs, training_outputs = rollout(0.0, 500)
        validation_inputs, validation_outputs = rollout(73.0, 240)
        training_velocities = [[0.0, 0.0]] + [
            [
                training_outputs[index][joint]
                - training_outputs[index - 1][joint]
                for joint in range(2)
            ]
            for index in range(1, len(training_outputs))
        ]
        validation_velocities = [[0.0, 0.0]] + [
            [
                validation_outputs[index][joint]
                - validation_outputs[index - 1][joint]
                for joint in range(2)
            ]
            for index in range(1, len(validation_outputs))
        ]
        rows, expected = MODULE.state_input_rows(
            training_inputs, training_outputs, training_velocities, 0
        )
        validation_rows, validation_expected = MODULE.state_input_rows(
            validation_inputs, validation_outputs, validation_velocities, 0
        )
        self.assertEqual(MODULE.matrix_rank(rows), 7)
        coefficients, covariance = MODULE.fit_model(rows, expected, 1e-10)
        metrics, predictions = MODULE.prediction_metrics(
            coefficients,
            covariance,
            rows,
            expected,
            validation_rows,
            validation_expected,
            20,
            1.959963984540054,
        )
        self.assertEqual(len(predictions), len(validation_expected))
        self.assertLess(metrics["one_step_prediction_rmse_rad"], 0.003)

    def test_coherence_recovers_a_deterministic_linear_response(self) -> None:
        rate = 60.0
        frequency = 0.75
        inputs = [
            0.04 * math.sin(2.0 * math.pi * frequency * index / rate)
            for index in range(480)
        ]
        outputs = [0.0] + [0.8 * inputs[index - 1] for index in range(1, 480)]
        observed = MODULE.coherence(inputs, outputs, rate, frequency, 240, 120)
        self.assertGreater(observed, 0.99)

    def test_fixture_declares_seven_independent_training_regions_and_no_refit(self) -> None:
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        analysis = manifest["analysis"]
        self.assertEqual(len(analysis["identified_joint_order"]), 7)
        self.assertEqual(len(analysis["training_segments"]), 7)
        validation = next(
            item
            for item in manifest["segments"]
            if item["id"] == analysis["validation_segment"]
        )
        self.assertEqual(validation["kind"], "coupled_multisine")
        self.assertEqual(
            [source["joint"] for source in validation["sources"]],
            analysis["identified_joint_order"],
        )

    def test_requirements_are_fixed_and_quality_gated(self) -> None:
        registry = json.loads(REQUIREMENTS.read_text(encoding="utf-8"))
        requirements = MODULE.requirement_map(registry)
        self.assertEqual(len(requirements), len(registry["requirements"]))
        self.assertEqual(
            requirements["identification.model.maximum_validation_rmse_rad"][
                "maximum"
            ],
            0.05,
        )
        self.assertTrue(
            all(item["gate"] in MODULE.QUALITY_GATES for item in requirements.values())
        )


if __name__ == "__main__":
    unittest.main()

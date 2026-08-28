from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "adapters/simulator/rne_gazebo_harmonic/build_openarm_plant_report.py"
SPEC = importlib.util.spec_from_file_location("openarm_plant_report", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
REQUIREMENTS = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_requirements.json"
)


class OpenArmPlantReportTests(unittest.TestCase):
    def test_complex_projection_recovers_sine_amplitude_and_phase(self) -> None:
        rate = 60.0
        frequency = 0.75
        phase = 0.4
        values = [
            0.2 * math.sin(2.0 * math.pi * frequency * index / rate + phase)
            for index in range(600)
        ]
        projection = MODULE.complex_projection(values, rate, frequency)
        self.assertAlmostEqual(2.0 * abs(projection), 0.2, places=4)

    def test_arx_fit_is_trained_without_validation_refit(self) -> None:
        inputs = [math.sin(index * 0.17) for index in range(300)]
        outputs = [0.0, 0.0]
        for index in range(2, len(inputs)):
            outputs.append(
                0.8 * outputs[-1]
                - 0.1 * outputs[-2]
                + 0.2 * inputs[index - 1]
                + 0.05 * inputs[index - 2]
                + 0.01
            )
        rows, expected = MODULE.arx_rows(inputs[:200], outputs[:200])
        coefficients = MODULE.fit_arx(rows, expected)
        validation_rows, validation_expected = MODULE.arx_rows(inputs[200:], outputs[200:])
        validation = MODULE.predict(coefficients, validation_rows, validation_expected)
        self.assertLess(validation["one_step_prediction_rmse_rad"], 1e-8)

    def test_requirement_checks_do_not_derive_limits_from_observations(self) -> None:
        requirement = {
            "id": "fixed",
            "gate": "closed_loop_performance",
            "unit": "rad",
            "maximum": 0.1,
        }
        self.assertEqual(MODULE.upper_check(requirement, 0.09)["status"], "passed")
        self.assertEqual(MODULE.upper_check(requirement, 0.11)["status"], "failed")
        self.assertEqual(
            MODULE.upper_check(requirement, 0.11)["gate"],
            "closed_loop_performance",
        )
        self.assertEqual(requirement["maximum"], 0.1)

    def test_registry_assigns_every_requirement_to_a_known_quality_gate(self) -> None:
        registry = json.loads(REQUIREMENTS.read_text(encoding="utf-8"))
        requirements = MODULE.requirement_map(registry)
        self.assertEqual(len(requirements), len(registry["requirements"]))
        self.assertTrue(
            all(
                requirement["gate"] in MODULE.QUALITY_GATES
                for requirement in requirements.values()
            )
        )


if __name__ == "__main__":
    unittest.main()

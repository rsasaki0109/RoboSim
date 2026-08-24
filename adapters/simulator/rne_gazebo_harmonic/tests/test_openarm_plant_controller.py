from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_plant_controller.py"
)
SPEC = importlib.util.spec_from_file_location("openarm_plant_controller", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
MANIFEST = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_plant_experiments.json"
)


class OpenArmPlantControllerTests(unittest.TestCase):
    def setUp(self) -> None:
        self.manifest = MODULE.load_manifest(MANIFEST)
        self.controller = MODULE.compile_controller(self.manifest)
        self.order = self.manifest["action_joint_order"]
        self.joint5 = self.order.index("openarm_right_joint5")

    def target(self, step: int) -> list[float]:
        return self.controller["keyframes"][step]["joint_position_target_rad"]

    def test_compilation_is_deterministic_contiguous_and_bounded(self) -> None:
        self.assertEqual(self.controller, MODULE.compile_controller(self.manifest))
        self.assertEqual(len(self.controller["keyframes"]), 3601)
        self.assertEqual(
            [frame["step"] for frame in self.controller["keyframes"]],
            list(range(3601)),
        )
        operating = self.manifest["operating_point_rad"]
        maximum_offset = max(
            abs(value - center)
            for frame in self.controller["keyframes"][301:]
            for value, center in zip(frame["joint_position_target_rad"], operating)
        )
        self.assertLessEqual(maximum_offset, 0.2)

    def test_conditioning_reaches_the_operating_point_without_an_initial_step(self) -> None:
        initial = self.manifest["initial_reference_rad"]
        operating = self.manifest["operating_point_rad"]
        self.assertEqual(self.target(0), initial)
        self.assertNotEqual(self.target(1), operating)
        self.assertEqual(self.target(240), operating)
        self.assertEqual(self.target(300), operating)

    def test_step_ramp_chirp_and_validation_are_distinct(self) -> None:
        self.assertEqual(self.target(360)[self.joint5], 0.0)
        self.assertEqual(self.target(361)[self.joint5], 0.12)
        self.assertEqual(self.target(660)[self.joint5], 0.12)
        self.assertEqual(self.target(661)[self.joint5], 0.0)
        self.assertAlmostEqual(self.target(1050)[self.joint5], 0.15)
        chirp = [self.target(step)[self.joint5] for step in range(1201, 2401)]
        validation = [self.target(step)[self.joint5] for step in range(2401, 3001)]
        self.assertGreater(max(chirp) - min(chirp), 0.15)
        self.assertGreater(max(validation) - min(validation), 0.08)
        self.assertNotEqual(chirp[:100], validation[:100])

    def test_coupling_uses_unique_source_frequencies_and_holds_joint5(self) -> None:
        coupling = self.manifest["segments"][-1]
        frequencies = [source["frequency_hz"] for source in coupling["sources"]]
        self.assertEqual(len(frequencies), len(set(frequencies)))
        self.assertTrue(
            all(
                math.isclose(self.target(step)[self.joint5], 0.0, abs_tol=1e-15)
                for step in range(3001, 3601)
            )
        )


if __name__ == "__main__":
    unittest.main()

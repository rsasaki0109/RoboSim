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
MULTIJOINT_MANIFEST = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_multijoint_identification_experiments.json"
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

    def test_multijoint_fixture_excites_each_arm_joint_then_all_together(self) -> None:
        manifest = MODULE.load_manifest(MULTIJOINT_MANIFEST)
        controller = MODULE.compile_controller(manifest)
        identified = manifest["analysis"]["identified_joint_order"]
        order = manifest["action_joint_order"]
        operating = manifest["operating_point_rad"]
        self.assertEqual(controller["keyframes"][-1]["step"], 4380)
        self.assertEqual(len(manifest["analysis"]["training_segments"]), 7)
        for segment_id, joint in zip(
            manifest["analysis"]["training_segments"], identified
        ):
            segment = next(
                item for item in manifest["segments"] if item["id"] == segment_id
            )
            midpoint = (segment["start_step"] + segment["end_step"]) // 2
            active = controller["keyframes"][midpoint]["joint_position_target_rad"]
            changed = [
                order[index]
                for index, (value, center) in enumerate(zip(active, operating))
                if not math.isclose(value, center, abs_tol=1e-12)
            ]
            self.assertEqual(changed, [joint])
        validation = next(
            item
            for item in manifest["segments"]
            if item["id"] == manifest["analysis"]["validation_segment"]
        )
        sample = controller["keyframes"][validation["start_step"] + 37][
            "joint_position_target_rad"
        ]
        self.assertTrue(
            all(
                not math.isclose(sample[order.index(joint)], operating[order.index(joint)], abs_tol=1e-12)
                for joint in identified
            )
        )


if __name__ == "__main__":
    unittest.main()

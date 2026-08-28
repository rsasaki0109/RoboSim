from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/build_openarm_joint_loss_controller_tuning.py"
)
SPEC = importlib.util.spec_from_file_location("openarm_joint_loss_controller_tuning", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)
MANIFEST = (
    ROOT
    / "adapters/simulator/rne_gazebo_harmonic/openarm_joint_loss_controller_tuning.json"
)
BASE = (
    ROOT
    / "docs/evidence/openarm-controller-lab/evidence/openarm-plant-state-feedback.controller.json"
)


class OpenArmJointLossControllerTuningTests(unittest.TestCase):
    def test_candidates_are_deterministic_and_change_only_declared_fields(self) -> None:
        base = json.loads(BASE.read_text(encoding="utf-8"))
        with tempfile.TemporaryDirectory() as first_raw, tempfile.TemporaryDirectory() as second_raw:
            first = Path(first_raw)
            second = Path(second_raw)
            suite_a = MODULE.compile_candidates(MANIFEST, BASE, first)
            suite_b = MODULE.compile_candidates(MANIFEST, BASE, second)
            self.assertEqual(suite_a, suite_b)
            self.assertEqual(len(suite_a["candidates"]), 4)
            for descriptor in suite_a["candidates"]:
                candidate = json.loads(
                    (first / descriptor["file"]).read_text(encoding="utf-8")
                )
                expected = json.loads(json.dumps(base))
                expected["controller_id"] = descriptor["controller_id"]
                expected["feedback_law"][
                    "maximum_state_feedback_correction_rad"
                ] = descriptor["maximum_state_feedback_correction_rad"]
                self.assertEqual(candidate, expected)
                self.assertEqual(
                    MODULE.sha256(first / descriptor["file"]), descriptor["sha256"]
                )

    def test_manifest_rejects_post_hoc_or_ambiguous_candidate_grids(self) -> None:
        manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
        for grid in ([0.04], [0.04, 0.04], [0.06, 0.04], [0.04, float("inf")]):
            invalid = json.loads(json.dumps(manifest))
            invalid["maximum_state_feedback_correction_grid_rad"] = grid
            with self.assertRaises(ValueError):
                MODULE.validate_manifest(invalid)


if __name__ == "__main__":
    unittest.main()

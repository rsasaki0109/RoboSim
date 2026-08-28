from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"
sys.path.insert(0, str(SCRIPT_DIR))


def load_module():
    path = SCRIPT_DIR / "build_openarm_coulomb_controller_pole_tuning.py"
    spec = importlib.util.spec_from_file_location("coulomb_pole_tuning", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TUNING = load_module()


class OpenArmCoulombControllerPoleTuningTests(unittest.TestCase):
    def test_compiler_recomputes_gain_for_each_frozen_pole_set(self) -> None:
        base = (
            ROOT
            / "docs/evidence/openarm-controller-lab/evidence/"
            "openarm-plant-state-feedback.controller.json"
        )
        with tempfile.TemporaryDirectory() as directory:
            suite = TUNING.compile_candidates(
                SCRIPT_DIR / "openarm_coulomb_controller_pole_tuning.json",
                base,
                Path(directory),
            )
            self.assertEqual(len(suite["candidates"]), 4)
            self.assertEqual(
                len({tuple(item["state_feedback_gain"]) for item in suite["candidates"]}),
                4,
            )
            self.assertTrue(
                all(
                    item["integral_state_feedback_gain_s_inv"] > 0.0
                    for item in suite["candidates"]
                )
            )


if __name__ == "__main__":
    unittest.main()

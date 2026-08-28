from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"
sys.path.insert(0, str(SCRIPT_DIR))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PAYLOAD = load_module("payload_for_transition", SCRIPT_DIR / "build_openarm_payload_suite.py")
TUNING = load_module(
    "openarm_coulomb_transition_tuning",
    SCRIPT_DIR / "build_openarm_coulomb_transition_tuning.py",
)


class OpenArmCoulombTransitionTuningTests(unittest.TestCase):
    def test_compiler_binds_predeclared_transition_to_robot_asset(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload_root = root / "payload"
            PAYLOAD.compile_suite(
                SCRIPT_DIR / "openarm_payload_experiments.json",
                ROOT / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
                SCRIPT_DIR / "openarm_right.adapter.json",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                payload_root,
            )
            output = root / "tuning"
            first = TUNING.compile_tuning(
                SCRIPT_DIR / "openarm_coulomb_transition_tuning.json",
                SCRIPT_DIR / "openarm_coulomb_friction_experiments.json",
                payload_root / "payload-0000g",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                output,
            )
            second = TUNING.compile_tuning(
                SCRIPT_DIR / "openarm_coulomb_transition_tuning.json",
                SCRIPT_DIR / "openarm_coulomb_friction_experiments.json",
                payload_root / "payload-0000g",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                output,
            )
            self.assertEqual(first, second)
            self.assertEqual(len(first["candidates"]), 4)
            candidate = first["candidates"][-1]
            self.assertEqual(candidate["transition_velocity_rad_s"], 0.05)
            self.assertGreaterEqual(
                candidate["kinetic_fraction_at_reference_velocity"], 0.95
            )
            robot_path = (
                output
                / candidate["candidate_id"]
                / "fixtures"
                / candidate["case_id"]
                / "openarm_payload.rne.robot.toml"
            )
            robot = robot_path.read_text()
            self.assertIn("coulomb_transition_velocity_rad_s = 0.050000000000000003", robot)


if __name__ == "__main__":
    unittest.main()

from __future__ import annotations

import importlib.util
import json
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


SUBSTEPS = load_module(
    "openarm_coulomb_substep_tuning",
    SCRIPT_DIR / "build_openarm_coulomb_substep_tuning.py",
)
PAYLOAD = load_module("payload_for_substeps", SCRIPT_DIR / "build_openarm_payload_suite.py")
COULOMB = load_module(
    "coulomb_for_substeps", SCRIPT_DIR / "build_openarm_coulomb_friction_suite.py"
)


class OpenArmCoulombSubstepTuningTests(unittest.TestCase):
    def test_compiler_preserves_exact_control_period(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            payload = root / "payload"
            PAYLOAD.compile_suite(
                SCRIPT_DIR / "openarm_payload_experiments.json",
                ROOT / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
                SCRIPT_DIR / "openarm_right.adapter.json",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                payload,
            )
            coulomb = root / "coulomb"
            COULOMB.compile_suite(
                SCRIPT_DIR / "openarm_coulomb_friction_experiments.json",
                payload / "payload-0000g",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                coulomb,
            )
            source = coulomb / "joint5-coulomb-0500mn"
            output = root / "substeps"
            suite = SUBSTEPS.compile_tuning(
                SCRIPT_DIR / "openarm_coulomb_substep_tuning.json", source, output
            )
            self.assertEqual(len(suite["candidates"]), 4)
            for candidate in suite["candidates"]:
                self.assertEqual(
                    sum(candidate["exact_substep_tick_partition"]), 16_666_667
                )
                config = json.loads(
                    (
                        output
                        / candidate["candidate_id"]
                        / "openarm_right.rne_actuation.json"
                    ).read_text()
                )
                self.assertEqual(
                    config["physics_substeps_per_control_step"],
                    candidate["physics_substeps_per_control_step"],
                )


if __name__ == "__main__":
    unittest.main()

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


PAYLOAD = load_module("payload_for_coulomb", SCRIPT_DIR / "build_openarm_payload_suite.py")
COULOMB = load_module(
    "openarm_coulomb_suite", SCRIPT_DIR / "build_openarm_coulomb_friction_suite.py"
)


class OpenArmCoulombFrictionSuiteTests(unittest.TestCase):
    def test_compiler_separates_portable_model_from_gazebo_realization(self) -> None:
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
            output = root / "coulomb"
            first = COULOMB.compile_suite(
                SCRIPT_DIR / "openarm_coulomb_friction_experiments.json",
                payload_root / "payload-0000g",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                output,
            )
            second = COULOMB.compile_suite(
                SCRIPT_DIR / "openarm_coulomb_friction_experiments.json",
                payload_root / "payload-0000g",
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                output,
            )
            self.assertEqual(first, second)
            self.assertEqual(len(first["cases"]), 5)
            case = first["cases"][-1]
            self.assertEqual(case["plant_coulomb_friction_nm"], 2.0)
            self.assertEqual(case["portable_model_realized_dynamics"], [10.0, 2.0])
            self.assertEqual(case["gazebo_runtime_model_realized_dynamics"], [10.0, 0.0])
            self.assertEqual(case["gazebo_adapter_realized_coulomb_friction_nm"], 2.0)
            case_dir = output / case["case_id"]
            config = json.loads((case_dir / "openarm_right.adapter.json").read_text())
            index = config["joint_order"].index(first["controlled_joint"])
            self.assertEqual(config["plant_coulomb_friction_nm"][index], 2.0)
            self.assertEqual(config["plant_coulomb_transition_velocity_rad_s"][index], 0.01)
            self.assertEqual(sum(config["plant_coulomb_friction_nm"]), 2.0)
            robot = (case_dir / "openarm_payload.rne.robot.toml").read_text()
            self.assertIn('path = "openarm_v2_right.coulomb.urdf"', robot)
            runtime = json.loads((case_dir / "runtime.json").read_text())
            self.assertEqual(
                [item["file"] for item in runtime["artifacts"] if item["role"] == "robot_model"],
                ["openarm_v2_right.payload.urdf"],
            )
            for item in runtime["artifacts"]:
                artifact = case_dir / item["file"]
                self.assertEqual(artifact.stat().st_size, item["size_bytes"])
                self.assertEqual(COULOMB.sha256(artifact), item["sha256"])

    def test_manifest_rejects_zero_transition_for_nonzero_grid(self) -> None:
        manifest = json.loads(
            (SCRIPT_DIR / "openarm_coulomb_friction_experiments.json").read_text()
        )
        manifest["coulomb_transition_velocity_rad_s"] = 0.0
        with self.assertRaisesRegex(ValueError, "unsupported"):
            COULOMB.validate_manifest(manifest)


if __name__ == "__main__":
    unittest.main()

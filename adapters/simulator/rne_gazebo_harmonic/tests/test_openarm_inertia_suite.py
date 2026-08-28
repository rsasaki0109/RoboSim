from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import unittest


SCRIPT_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SCRIPT_DIR))


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


PAYLOAD = load_module("payload_for_inertia", SCRIPT_DIR / "build_openarm_payload_suite.py")
COULOMB = load_module(
    "coulomb_for_inertia", SCRIPT_DIR / "build_openarm_coulomb_friction_suite.py"
)
INERTIA = load_module("openarm_inertia_suite", SCRIPT_DIR / "build_openarm_inertia_suite.py")


class OpenArmInertiaSuiteTests(unittest.TestCase):
    def compile_baseline(self, root: Path) -> Path:
        payload = root / "payload"
        PAYLOAD.compile_suite(
            SCRIPT_DIR / "openarm_payload_experiments.json",
            SCRIPT_DIR / "openarm_v2_right.rne.urdf",
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
        return coulomb / "joint5-coulomb-0500mn"

    def test_tensor_only_grid_is_deterministic_and_physically_valid(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            baseline = self.compile_baseline(root)
            first = root / "first"
            replay = root / "replay"
            suite = INERTIA.compile_suite(
                SCRIPT_DIR / "openarm_inertia_experiments.json", baseline, first
            )
            INERTIA.compile_suite(
                SCRIPT_DIR / "openarm_inertia_experiments.json", baseline, replay
            )
            self.assertEqual(
                [case["case_id"] for case in suite["cases"]],
                [
                    "joint5-inertia-01x",
                    "joint5-inertia-02x",
                    "joint5-inertia-04x",
                    "joint5-inertia-08x",
                    "joint5-inertia-16x",
                ],
            )
            self.assertEqual((first / "inertia-suite.json").read_bytes(), (replay / "inertia-suite.json").read_bytes())
            maximum = suite["cases"][-1]
            base = maximum["baseline_inertial"]
            realized = maximum["portable_realized_inertial"]
            self.assertEqual(realized["mass_kg"], base["mass_kg"])
            self.assertEqual(realized["center_of_mass_m"], base["center_of_mass_m"])
            for name, value in base["tensor_kg_m2"].items():
                self.assertAlmostEqual(realized["tensor_kg_m2"][name], 16.0 * value)
            self.assertEqual(realized, maximum["gazebo_realized_inertial"])
            robot = (first / maximum["case_id"] / INERTIA.ROBOT_FILE).read_text()
            self.assertIn('path = "openarm_v2_right.inertia.urdf"', robot)

    def test_invalid_tensor_and_manifest_fail_closed(self) -> None:
        bad_tensor = {"ixx": 1.0, "ixy": 0.0, "ixz": 0.0, "iyy": 1.0, "iyz": 0.0, "izz": 3.0}
        with self.assertRaisesRegex(ValueError, "physically realizable"):
            INERTIA.validate_physical_tensor(bad_tensor)
        manifest = json.loads((SCRIPT_DIR / "openarm_inertia_experiments.json").read_text())
        manifest["inertia_scale_grid"] = [1.0, 0.5, 2.0]
        with self.assertRaisesRegex(ValueError, "unsupported inertia"):
            INERTIA.validate_manifest(manifest)

if __name__ == "__main__":
    unittest.main()

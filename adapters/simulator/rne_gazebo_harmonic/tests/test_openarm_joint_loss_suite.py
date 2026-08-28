from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"


def load_module(name: str, path: Path):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


PAYLOAD = load_module(
    "payload_suite_for_joint_loss", SCRIPT_DIR / "build_openarm_payload_suite.py"
)
JOINT_LOSS = load_module(
    "openarm_joint_loss_suite", SCRIPT_DIR / "build_openarm_joint_loss_suite.py"
)


class OpenArmJointLossSuiteTests(unittest.TestCase):
    def test_compiler_changes_only_model_and_binds_exact_si_values(self) -> None:
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
            baseline = payload_root / "payload-0000g"
            output = root / "joint-loss"
            suite = JOINT_LOSS.compile_suite(
                SCRIPT_DIR / "openarm_joint_loss_experiments.json",
                baseline,
                SCRIPT_DIR / "openarm_right.rne_actuation.json",
                output,
            )

            self.assertEqual(len(suite["cases"]), 5)
            zero = suite["cases"][0]
            maximum = suite["cases"][-1]
            self.assertEqual(zero["plant_viscous_damping_nm_s_per_rad"], 0.0)
            self.assertFalse(zero["dynamics_element_present"])
            self.assertEqual(zero["model_urdf_sha256"], zero["source_model_sha256"])
            self.assertEqual(maximum["plant_viscous_damping_nm_s_per_rad"], 20.0)
            self.assertEqual(maximum["realized_viscous_damping_nm_s_per_rad"], 20.0)
            self.assertEqual(maximum["realized_coulomb_friction_nm"], 0.0)
            self.assertTrue(maximum["dynamics_element_present"])

            invariant_hash_fields = (
                "world_sha256",
                "robot_asset_config_sha256",
                "scene_config_sha256",
                "actuation_config_sha256",
                "adapter_config_sha256",
            )
            for field in invariant_hash_fields:
                self.assertEqual({case[field] for case in suite["cases"]}, {zero[field]})
            for case in suite["cases"]:
                case_dir = output / case["case_id"]
                fixture = json.loads((case_dir / "joint-loss-fixture.json").read_text())
                self.assertEqual(fixture, case)
                runtime = json.loads((case_dir / "runtime.json").read_text())
                for item in runtime["artifacts"]:
                    path = case_dir / item["file"]
                    self.assertEqual(path.stat().st_size, item["size_bytes"])
                    self.assertEqual(JOINT_LOSS.sha256(path), item["sha256"])

    def test_builder_rejects_existing_controlled_joint_dynamics(self) -> None:
        source = b'''<robot name="test"><joint name="joint5" type="revolute"><dynamics damping="1"/></joint></robot>'''
        with self.assertRaisesRegex(ValueError, "already declares dynamics"):
            JOINT_LOSS.build_urdf(source, "joint5", 2.0, 0.0)


if __name__ == "__main__":
    unittest.main()

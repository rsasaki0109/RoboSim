#!/usr/bin/env python3

from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"


def load_module():
    spec = importlib.util.spec_from_file_location(
        "openarm_physical_configuration_suite",
        SCRIPT_DIR / "build_openarm_physical_configuration_suite.py",
    )
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


MODULE = load_module()


class PhysicalConfigurationSuiteTests(unittest.TestCase):
    def compile(self, output: Path):
        return MODULE.compile_suite(
            SCRIPT_DIR / "openarm_physical_configuration_experiments.json",
            ROOT,
            output,
        )

    def test_compiler_uses_exact_official_product_presets(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            suite = self.compile(output)
            self.assertEqual(suite["configuration_order"], ["arm_only", "pinch_gripper"])
            self.assertAlmostEqual(
                suite["realized_gripper_mass_delta_kg"], 0.23969904736667047, places=14
            )
            manifest = json.loads(
                (SCRIPT_DIR / "openarm_physical_configuration_experiments.json").read_text()
            )
            for configuration in manifest["configurations"]:
                preset = ROOT / configuration["vendored_preset"]
                self.assertEqual(MODULE.sha256(preset), configuration["preset_sha256"])

    def test_arm_only_removes_only_the_official_end_effector_component(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            self.compile(output)
            arm = ET.parse(output / "arm_only" / MODULE.MODEL_FILE).getroot()
            gripper = ET.parse(output / "pinch_gripper" / MODULE.MODEL_FILE).getroot()
            for name in ("openarm_right_ee_link1", "openarm_right_ee_link2"):
                self.assertIsNone(arm.find(f"./link[@name='{name}']"))
                self.assertIsNotNone(gripper.find(f"./link[@name='{name}']"))
            for name in ("openarm_right_finger_joint1", "openarm_right_finger_joint2"):
                self.assertIsNone(arm.find(f"./joint[@name='{name}']"))
                self.assertIsNotNone(gripper.find(f"./joint[@name='{name}']"))
            self.assertEqual(len(arm.findall("./joint[@type='revolute']")), 7)
            self.assertEqual(len(gripper.findall("./joint[@type='revolute']")), 9)

    def test_shared_contract_controls_exactly_the_same_seven_arm_joints(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            self.compile(output)
            controller = json.loads((output / MODULE.CONTROLLER_FILE).read_text())
            task = json.loads((output / MODULE.TASK_FILE).read_text())
            actuation = json.loads((output / MODULE.ACTUATION_FILE).read_text())
            self.assertEqual(len(controller["action_joint_order"]), 7)
            self.assertEqual(task["action"]["tensors"][0]["shape"], [7])
            self.assertEqual(
                [item["joint_name"] for item in actuation["joints"]],
                controller["action_joint_order"],
            )
            for case in ("arm_only", "pinch_gripper"):
                adapter = json.loads((output / case / MODULE.ADAPTER_FILE).read_text())
                self.assertEqual(adapter["joint_order"], controller["action_joint_order"])

    def test_nonphysical_tensor_is_rejected(self) -> None:
        source = (
            ROOT / "assets/robots/openarm_description/openarm_v2_right.rne.urdf"
        ).read_bytes()
        root = ET.fromstring(source)
        tensor = root.find("./link/inertial/inertia")
        assert tensor is not None
        tensor.attrib["ixx"] = "-1"
        invalid = ET.tostring(root, encoding="utf-8")
        with self.assertRaisesRegex(ValueError, "positive definite"):
            MODULE.model_inertials(invalid)

    def test_preset_hash_tamper_fails_closed(self) -> None:
        manifest = json.loads(
            (SCRIPT_DIR / "openarm_physical_configuration_experiments.json").read_text()
        )
        manifest["configurations"][0]["preset_sha256"] = "0" * 64
        with self.assertRaisesRegex(ValueError, "preset hash drifted"):
            MODULE.validate_manifest(manifest, ROOT)


if __name__ == "__main__":
    unittest.main()

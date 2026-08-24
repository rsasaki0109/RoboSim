from __future__ import annotations

import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[4]
SCRIPT_DIR = ROOT / "adapters/simulator/rne_gazebo_harmonic"
SPEC = importlib.util.spec_from_file_location(
    "openarm_payload_suite", SCRIPT_DIR / "build_openarm_payload_suite.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class OpenArmPayloadSuiteTests(unittest.TestCase):
    def compile(self, output: Path):
        return MODULE.compile_suite(
            SCRIPT_DIR / "openarm_payload_experiments.json",
            ROOT / "assets/robots/openarm_description/openarm_v2_right.rne.urdf",
            SCRIPT_DIR / "openarm_right.adapter.json",
            output,
        )

    def test_suite_is_deterministic_and_unit_explicit(self) -> None:
        with tempfile.TemporaryDirectory() as first_dir, tempfile.TemporaryDirectory() as second_dir:
            first = self.compile(Path(first_dir))
            second = self.compile(Path(second_dir))
            self.assertEqual(first, second)
            self.assertEqual(
                [case["case_id"] for case in first["cases"]],
                ["payload-0000g", "payload-0100g", "payload-0250g", "payload-0500g", "payload-0750g"],
            )
            self.assertTrue(
                all("\\" not in case["fixture_file"] for case in first["cases"])
            )

    def test_declared_payload_is_lumped_into_hand_with_exact_rigid_body_inertia(self) -> None:
        with tempfile.TemporaryDirectory() as output_dir:
            output = Path(output_dir)
            suite = self.compile(output)
            case = suite["cases"][3]
            root = ET.parse(output / case["case_id"] / "openarm_v2_right.payload.urdf").getroot()
            link = root.find("./link[@name='openarm_right_ee_base_link']")
            base_mass = 0.52832
            base_com = [0.0137902, 0.0084073, -0.05938]
            base_inertia = {"ixx": 0.00021911, "ixy": 4.1683e-06, "ixz": 1.4043e-06, "iyy": 0.00015627, "iyz": -1.5124e-06, "izz": 0.0001659}
            expected_mass, expected_com, expected = MODULE.combined_inertial(
                base_mass, base_com, base_inertia, 0.5, [0.095, 0.0, -0.085], [0.08, 0.05, 0.04]
            )
            self.assertAlmostEqual(float(link.find("inertial/mass").attrib["value"]), expected_mass)
            actual_com = [float(v) for v in link.find("inertial/origin").attrib["xyz"].split()]
            for actual, wanted in zip(actual_com, expected_com):
                self.assertAlmostEqual(actual, wanted)
            inertia = link.find("inertial/inertia").attrib
            for name, wanted in expected.items():
                self.assertAlmostEqual(float(inertia[name]), wanted)
            self.assertIsNotNone(link.find("visual[@name='openarm_payload_visual']"))

    def test_baseline_omits_zero_mass_link_and_all_hashes_verify(self) -> None:
        with tempfile.TemporaryDirectory() as output_dir:
            output = Path(output_dir)
            suite = self.compile(output)
            baseline = suite["cases"][0]
            root = ET.parse(output / baseline["case_id"] / "openarm_v2_right.payload.urdf").getroot()
            self.assertIsNone(root.find(".//visual[@name='openarm_payload_visual']"))
            for case in suite["cases"]:
                case_dir = output / case["case_id"]
                runtime = json.loads((case_dir / "runtime.json").read_text(encoding="utf-8"))
                for item in runtime["artifacts"]:
                    path = case_dir / item["file"]
                    self.assertEqual(path.stat().st_size, item["size_bytes"])
                    self.assertEqual(MODULE.sha256(path), item["sha256"])
                self.assertEqual(
                    MODULE.sha256(case_dir / "openarm_v2_right.payload.urdf"),
                    case["model_urdf_sha256"],
                )


if __name__ == "__main__":
    unittest.main()

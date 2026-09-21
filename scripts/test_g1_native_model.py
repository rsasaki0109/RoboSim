"""Tests for native model preparation; no renderer or simulation required."""

import tempfile
import unittest
import xml.etree.ElementTree as ET
from collections import namedtuple
from pathlib import Path
from unittest.mock import patch

from g1_native_model import SOURCE, prepare

Usage = namedtuple("Usage", "total used free")


class ModelPreparationTest(unittest.TestCase):
    def test_only_empty_fixed_leaves_removed(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch(
                "g1_native_model.shutil.disk_usage",
                return_value=Usage(100, 0, 32 * 1024**3),
            ),
        ):
            output = Path(directory) / "model"
            audit = prepare(SOURCE, output)
            self.assertAlmostEqual(audit["declared_mass_kg"], 34.13385728)
            self.assertEqual(audit["movable_joint_count"], 23)
            self.assertEqual(len(audit["removed_empty_fixed_leaf_frames"]), 4)
            original = ET.parse(SOURCE).getroot()
            generated = ET.parse(output / "robot.urdf").getroot()
            for link in generated.findall("link"):
                source = original.find(f"link[@name='{link.get('name')}']")
                # Compare all physical numbers, independently of XML indentation.
                for subtree in ["inertial", "collision"]:

                    def values(element):
                        return [
                            (
                                e.tag,
                                {k: v for k, v in e.attrib.items() if k != "filename"},
                            )
                            for e in element.iter()
                        ]

                    self.assertEqual(
                        [values(x) for x in link.findall(subtree)],
                        [values(x) for x in source.findall(subtree)],
                    )
            self.assertFalse(audit["qualification_ready"])
            self.assertEqual(
                audit["multi_collision_links_merged_to_aabb"]["left_ankle_roll_link"], 4
            )
            with self.assertRaises(FileExistsError):
                prepare(SOURCE, output)

    def test_source_sole_profile_preserves_mass_and_matches_support_bounds(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch(
                "g1_native_model.shutil.disk_usage",
                return_value=Usage(100, 0, 32 * 1024**3),
            ),
        ):
            output = Path(directory) / "model"
            audit = prepare(SOURCE, output, source_soles=True)
            self.assertAlmostEqual(audit["declared_mass_kg"], 34.13385728)
            self.assertTrue(audit["source_sole_dimensions"])
            robot = ET.parse(output / "robot.urdf").getroot()
            foot = robot.find("link[@name='left_ankle_roll_link']")
            spheres = foot.findall("collision")
            self.assertEqual(len(spheres), 4)
            xs = [float(c.find("origin").get("xyz").split()[0]) for c in spheres]
            self.assertAlmostEqual(max(xs) - min(xs), 0.14)
            self.assertTrue(
                all(
                    float(c.find("geometry/sphere").get("radius")) == 0.002
                    for c in spheres
                )
            )
            self.assertFalse(audit["qualification_ready"])

    def test_physical_or_branch_massless_link_rejected(self):
        for content in ["<visual/>", ""]:
            with tempfile.TemporaryDirectory() as directory:
                source = Path(directory) / "source.urdf"
                source.write_text(
                    f'<robot name="bad"><link name="root">{content}</link></robot>'
                )
                with self.assertRaisesRegex(ValueError, "unresolved non-inertial"):
                    prepare(source, Path(directory) / "output")
                self.assertFalse((Path(directory) / "output").exists())

    def test_disk_reserve_prevents_writes(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch(
                "g1_native_model.shutil.disk_usage",
                return_value=Usage(100, 0, 29 * 1024**3),
            ),
        ):
            output = Path(directory) / "model"
            with self.assertRaisesRegex(RuntimeError, "30 GiB"):
                prepare(SOURCE, output)
            self.assertFalse(output.exists())


if __name__ == "__main__":
    unittest.main()

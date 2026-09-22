"""Tests for native model preparation; no renderer or simulation required."""

import tempfile
import unittest
import xml.etree.ElementTree as ET
from collections import namedtuple
from pathlib import Path
from unittest.mock import patch

import tomllib
from g1_native_model import SOURCE, prepare

Usage = namedtuple("Usage", "total used free")


class ModelPreparationTest(unittest.TestCase):
    def test_full_contact_requests_convex_meshes_and_preserves_sole_parts(self):
        with tempfile.TemporaryDirectory() as directory, patch(
            "g1_native_model.shutil.disk_usage", return_value=Usage(100, 0, 32 * 1024**3)
        ):
            output = Path(directory) / "model"
            audit = prepare(SOURCE, output, source_soles=True, independent_soles=True,
                            source_passive_loss=True, full_contact=True)
            config = tomllib.loads((output / "robot.rne.robot.toml").read_text())["urdf"]
            self.assertTrue(config["convex_mesh_collisions"])
            self.assertTrue(config["self_collisions"])
            self.assertTrue(config["preserve_collision_parts"])
            self.assertTrue(config["mesh_collisions"])
            self.assertEqual(audit["mesh_collision_elements_disabled"], 0)
            self.assertGreater(audit["convex_mesh_collision_elements"], 0)
            self.assertFalse(audit["qualification_ready"])
            with self.assertRaises(ValueError):
                prepare(SOURCE, Path(directory) / "invalid", full_contact=True)

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

    def test_independent_sole_profile_requests_compound_geometry(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch(
                "g1_native_model.shutil.disk_usage",
                return_value=Usage(100, 0, 32 * 1024**3),
            ),
        ):
            output = Path(directory) / "model"
            audit = prepare(SOURCE, output, source_soles=True, independent_soles=True)
            self.assertEqual(audit["multi_collision_links_merged_to_aabb"], {})
            self.assertEqual(
                audit["compound_link_part_counts"],
                {
                    "left_ankle_roll_link": 4,
                    "right_ankle_roll_link": 4,
                },
            )
            self.assertIn(
                "preserve_collision_parts = true",
                (output / "robot.rne.robot.toml").read_text(),
            )
            self.assertFalse(audit["qualification_ready"])

    def test_passive_profile_preserves_inertia_and_covers_all_movable_joints(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch(
                "g1_native_model.shutil.disk_usage",
                return_value=Usage(100, 0, 32 * 1024**3),
            ),
        ):
            output = Path(directory) / "model"
            audit = prepare(
                SOURCE, output, independent_soles=True, source_passive_loss=True
            )
            robot = ET.parse(output / "robot.urdf").getroot()
            original = ET.parse(SOURCE).getroot()
            config = tomllib.loads((output / "robot.rne.robot.toml").read_text())
            overrides = config["urdf"]["joint_passive_dynamics"]
            movable = [j for j in robot.findall("joint") if j.get("type") != "fixed"]
            self.assertEqual(len(overrides), 23)
            self.assertEqual(
                {x["joint"] for x in overrides}, {j.get("name") for j in movable}
            )
            self.assertTrue(
                all(x["coulomb_transition_velocity_rad_s"] == 0.1 for x in overrides)
            )
            for joint in movable:
                self.assertEqual(
                    joint.find("dynamics").attrib,
                    {"damping": "0.05", "friction": "0.2"},
                )
            self.assertTrue(
                all(
                    j.find("dynamics") is None
                    for j in robot.findall("joint")
                    if j.get("type") == "fixed"
                )
            )
            for link in robot.findall("link"):
                source = original.find(f"link[@name='{link.get('name')}']")
                self.assertEqual(
                    [e.attrib for e in link.find("inertial").iter()],
                    [e.attrib for e in source.find("inertial").iter()],
                )
            self.assertEqual(audit["passive_loss_joint_count"], 23)
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

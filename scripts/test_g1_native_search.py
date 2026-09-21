"""Test native search ranking, stable parallel selection, and disk guard."""

import copy
import json
import tempfile
import unittest
from collections import namedtuple
from pathlib import Path
from unittest.mock import patch

from g1_native_search import loss, search

Usage = namedtuple("Usage", "total used free")


def sample():
    return {
        "backend": "RoboSim/Rapier",
        "signed_rotation_rad": -6.283185307179586,
        "peak_joint_speed_ratio": 1.0,
        "max_joint_position_excess_rad": 0.0,
        "completed_maneuver_time_s": 5.0,
        "takeoff_s": 0.6,
        "final_second_max_base_speed_m_s": 0.02,
        "frames": [{"upright": 1.0, "base_translation_m": [0.0, 0.78, 0.0]}],
    }


class NativeSearchTest(unittest.TestCase):
    def test_collapsed_or_overspeed_flip_is_worse(self):
        good = sample()
        collapsed = copy.deepcopy(good)
        collapsed["completed_maneuver_time_s"] = 1.2
        collapsed["frames"][0]["base_translation_m"][1] = 0.2
        fast = copy.deepcopy(good)
        fast["peak_joint_speed_ratio"] = 2.0
        self.assertGreater(loss(collapsed), loss(good))
        self.assertGreater(loss(fast), loss(good))
        fast["peak_joint_speed_ratio"] = float("nan")
        self.assertEqual(loss(fast), 1e6)

    def test_parallel_selection_matches_serial_and_never_qualifies(self):
        def physics_fixture(command, **kwargs):
            input_path = Path(command[command.index("--native-candidate") + 1])
            output_path = Path(command[command.index("--native-output") + 1])
            p = json.loads(input_path.read_text())["parameters"][0]
            result = sample()
            result["signed_rotation_rad"] += abs(p - 2.1)
            output_path.write_text(json.dumps(result))

        with (
            tempfile.TemporaryDirectory() as directory,
            patch(
                "g1_native_search.shutil.disk_usage",
                return_value=Usage(100, 0, 32 * 1024**3),
            ),
            patch("g1_native_search.subprocess.run", side_effect=physics_fixture),
        ):
            base = Path(directory)
            binary = base / "binary"
            binary.write_text("fixture")
            scene = base / "scene"
            scene.write_text("fixture")
            seed = {"parameters": [2.0] * 16}
            serial = search(
                binary, scene, seed, base / "serial", 1, 1, [(0, 1.0, 3.0, 0.1)]
            )
            parallel = search(
                binary, scene, seed, base / "parallel", 1, 2, [(0, 1.0, 3.0, 0.1)]
            )
            self.assertEqual(serial, parallel)
            self.assertEqual(serial["candidate"]["parameters"][0], 2.1)
            self.assertFalse(serial["qualified_backflip"])
            self.assertEqual(seed["parameters"][0], 2.0)
            manifest = json.loads((base / "parallel/search.json").read_text())
            self.assertEqual(len(manifest["evaluations"]), 3)

    def test_disk_guard_does_not_create_campaign(self):
        with (
            tempfile.TemporaryDirectory() as directory,
            patch(
                "g1_native_search.shutil.disk_usage",
                return_value=Usage(100, 0, 29 * 1024**3),
            ),
        ):
            base = Path(directory)
            binary = base / "binary"
            binary.touch()
            with self.assertRaisesRegex(RuntimeError, "30 GiB"):
                search(binary, binary, {}, base / "output", 1, 1)
            self.assertFalse((base / "output").exists())


if __name__ == "__main__":
    unittest.main()

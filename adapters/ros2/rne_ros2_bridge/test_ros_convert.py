#!/usr/bin/env python3
"""Unit tests for ROS 2 conversion helpers (no ROS runtime required)."""

from __future__ import annotations

import struct
import unittest

from ros_convert import (
    make_clock_message,
    make_pointcloud2,
    make_tf_message,
    make_transform_stamped,
    sim_ticks_to_ros_time,
)


class RosConvertTests(unittest.TestCase):
    def test_sim_time_mapping(self) -> None:
        sec, nanosec = sim_ticks_to_ros_time(1_500_000_000)
        self.assertEqual(sec, 1)
        self.assertEqual(nanosec, 500_000_000)

    def test_clock_message(self) -> None:
        clock = make_clock_message(2_000_000_000)
        self.assertEqual(clock.clock.sec, 2)
        self.assertEqual(clock.clock.nanosec, 0)

    def test_pointcloud2_layout(self) -> None:
        cloud = make_pointcloud2([(1.0, 0.0, 0.5)], "lidar", 42)
        self.assertEqual(cloud.width, 1)
        self.assertEqual(cloud.point_step, 12)
        self.assertEqual(len(cloud.data), 12)
        x, y, z = struct.unpack("<fff", cloud.data)
        self.assertAlmostEqual(x, 1.0)
        self.assertAlmostEqual(y, 0.0)
        self.assertAlmostEqual(z, 0.5)

    def test_tf_message_contains_transforms(self) -> None:
        tf = make_tf_message(
            [
                make_transform_stamped("world", "base_link", (1.0, 0.0, 0.0), 10),
                make_transform_stamped("base_link", "lidar", (0.0, 0.2, 0.0), 10),
            ]
        )
        self.assertEqual(len(tf.transforms), 2)
        self.assertEqual(tf.transforms[1].child_frame_id, "lidar")


if __name__ == "__main__":
    unittest.main()

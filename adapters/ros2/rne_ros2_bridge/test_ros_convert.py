#!/usr/bin/env python3
"""Unit tests for ROS 2 conversion helpers (no ROS runtime required)."""

from __future__ import annotations

import math
import struct
import unittest

from ros_convert import (
    command_from_twist,
    make_clock_message,
    make_joint_state,
    make_laserscan,
    make_occupancy_grid,
    make_odometry,
    make_path,
    make_pointcloud2,
    make_tf_message,
    make_transform_stamped,
    sim_ticks_to_ros_time,
)
from geometry_msgs.msg import Twist


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

    def test_odometry_maps_pose_and_twist(self) -> None:
        message = make_odometry((1.5, -0.5, 0.7), 0.4, -0.2, "odom", "base_link", 30)
        self.assertEqual(message.header.frame_id, "odom")
        self.assertEqual(message.child_frame_id, "base_link")
        self.assertAlmostEqual(message.pose.pose.position.x, 1.5)
        self.assertAlmostEqual(message.twist.twist.linear.x, 0.4)
        self.assertAlmostEqual(message.twist.twist.angular.z, -0.2)
        # Pure-yaw quaternion: z = sin(yaw/2), w = cos(yaw/2).
        self.assertAlmostEqual(message.pose.pose.orientation.z, math.sin(0.35))
        self.assertAlmostEqual(message.pose.pose.orientation.w, math.cos(0.35))

    def test_occupancy_grid_layout(self) -> None:
        data = [0] * 20
        data[0] = 100
        message = make_occupancy_grid(4, 5, 0.5, (-1.0, -1.0, 0.0), data, "map", 5)
        self.assertEqual(message.info.width, 4)
        self.assertEqual(message.info.height, 5)
        self.assertAlmostEqual(message.info.resolution, 0.5)
        self.assertAlmostEqual(message.info.origin.position.x, -1.0)
        self.assertEqual(message.data[0], 100)
        with self.assertRaises(ValueError):
            make_occupancy_grid(4, 5, 0.5, (0.0, 0.0, 0.0), [0] * 19, "map", 0)

    def test_path_maps_waypoints(self) -> None:
        message = make_path([(0.0, 0.0, 0.0), (1.0, 1.0, 0.0)], "map", 7)
        self.assertEqual(len(message.poses), 2)
        self.assertAlmostEqual(message.poses[1].pose.position.y, 1.0)

    def test_laserscan_geometry(self) -> None:
        message = make_laserscan([1.0, 2.0, 3.0], 0.0, 0.1, 0.05, 10.0, "lidar", 9)
        self.assertEqual(list(message.ranges), [1.0, 2.0, 3.0])
        self.assertAlmostEqual(message.angle_max, 0.2)
        self.assertAlmostEqual(message.range_max, 10.0)

    def test_command_from_twist(self) -> None:
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = -0.1
        self.assertEqual(command_from_twist(twist), (0.5, -0.1))

    def test_joint_state_mapping(self) -> None:
        message = make_joint_state(
            ["left_wheel_joint", "right_wheel_joint"],
            [1.0, 1.5],
            [2.0, 2.5],
            "base_link",
            11,
        )
        self.assertEqual(list(message.name), ["left_wheel_joint", "right_wheel_joint"])
        self.assertEqual(list(message.position), [1.0, 1.5])
        self.assertEqual(list(message.velocity), [2.0, 2.5])
        self.assertEqual(message.header.frame_id, "base_link")


if __name__ == "__main__":
    unittest.main()

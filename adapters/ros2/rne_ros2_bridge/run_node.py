#!/usr/bin/env python3
"""Minimal ROS 2 bridge for Robot Native Engine.

Publishes:
- `/clock` from simulation time
- `/points` as `sensor_msgs/PointCloud2`
- `/tf` as `tf2_msgs/TFMessage`

When `rne_py` is installed, the node drives a headless diff-drive simulation.
Otherwise it publishes one synthetic frame for smoke testing.
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2
from tf2_msgs.msg import TFMessage

from ros_convert import (
    make_clock_message,
    make_pointcloud2,
    make_tf_message,
    make_transform_stamped,
)

try:
    import rne_py
except ImportError:
    rne_py = None


SIM_DT_NS = 1_000_000_000 // 60
SIM_STEPS = 300
MIN_FORWARD_X_M = 0.8


class RneBridgeNode(Node):
    """ROS 2 adapter node for RNE simulation outputs."""

    def __init__(self) -> None:
        super().__init__("rne_bridge")
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/points", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.sim_ticks = 0
        self.use_rne_sim = rne_py is not None
        if self.use_rne_sim:
            self.sim = rne_py.DiffDriveSim()
            self.obs = self.sim.reset()
            self.get_logger().info("Driving headless diff-drive via rne_py")
        else:
            self.get_logger().warn("rne_py not installed; publishing synthetic frame only")

    def publish_frame(self, base_xyz: tuple[float, float, float], points: list[tuple[float, float, float]]) -> None:
        """Publish one simulation frame to ROS 2 topics."""
        self.clock_pub.publish(make_clock_message(self.sim_ticks))
        self.cloud_pub.publish(make_pointcloud2(points, "lidar", self.sim_ticks))
        tf = make_tf_message(
            [
                make_transform_stamped("world", "base_link", base_xyz, self.sim_ticks),
                make_transform_stamped(
                    "base_link",
                    "lidar",
                    (0.0, 0.2, 0.0),
                    self.sim_ticks,
                ),
            ]
        )
        self.tf_pub.publish(tf)

    def spin_once(self) -> None:
        """Advance one simulation step or publish a synthetic sample."""
        if self.use_rne_sim:
            self.obs = self.sim.step(6.0, 6.0)
            base = (self.obs.base_x, self.obs.base_y, self.obs.base_z)
            distance = max(base[0], 0.1)
            points = [(distance, 0.0, 0.0), (distance, 0.5, 0.0), (distance, -0.5, 0.0)]
        else:
            base = (0.0, 0.25, 0.0)
            points = [(3.0, 0.0, 0.0), (3.0, 0.5, 0.0)]

        self.publish_frame(base, points)
        self.sim_ticks += SIM_DT_NS


def main() -> None:
    rclpy.init()
    node = RneBridgeNode()
    steps = SIM_STEPS if node.use_rne_sim else 1

    try:
        for step in range(steps):
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.01)
            if step % 60 == 59 and node.use_rne_sim:
                node.get_logger().info(
                    f"step {step + 1}: base_x={node.obs.base_x:.2f} m"
                )
            time.sleep(0.001)

        if node.use_rne_sim:
            node.get_logger().info(f"final base_x={node.obs.base_x:.2f} m")
            if node.obs.base_x < MIN_FORWARD_X_M:
                raise SystemExit(
                    f"expected forward motion from diff-drive policy (base_x={node.obs.base_x:.2f} m)"
                )
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

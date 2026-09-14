#!/usr/bin/env python3
"""Live rclpy round-trip test for the navigation mappings.

Publishes one `Odometry`, `LaserScan`, `OccupancyGrid`, and `Path` built by
`ros_convert` and checks that a subscriber receives each on the same node. This
exercises real ROS 2 serialization without requiring `simulation_interfaces` or
the `rne_py` bindings.
"""

from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

from ros_convert import (
    command_from_twist,
    make_laserscan,
    make_occupancy_grid,
    make_odometry,
    make_path,
)

TICKS = 1_500_000_000


class NavRoundTrip(Node):
    """Publishes and re-subscribes each navigation message once."""

    def __init__(self) -> None:
        super().__init__("rne_nav_roundtrip")
        self.received: dict[str, object] = {}
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.plan_pub = self.create_publisher(Path, "/plan", 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_subscription(Path, "/plan", self._on_plan, 10)

    def _on_odom(self, message: Odometry) -> None:
        self.received["odom"] = message

    def _on_scan(self, message: LaserScan) -> None:
        self.received["scan"] = message

    def _on_map(self, message: OccupancyGrid) -> None:
        self.received["map"] = message

    def _on_plan(self, message: Path) -> None:
        self.received["plan"] = message

    def publish_all(self) -> None:
        self.odom_pub.publish(make_odometry((1.25, -0.5, 0.3), 0.5, -0.1, "odom", "base_link", TICKS))
        self.scan_pub.publish(make_laserscan([1.0, 2.0, 3.0], 0.0, 0.1, 0.05, 10.0, "lidar", TICKS))
        border = [0] * 25
        for index in range(5):
            border[index] = 100
            border[20 + index] = 100
            border[index * 5] = 100
            border[index * 5 + 4] = 100
        self.map_pub.publish(make_occupancy_grid(5, 5, 0.5, (0.0, 0.0, 0.0), border, "map", TICKS))
        self.plan_pub.publish(make_path([(0.0, 0.0, 0.0), (1.0, 0.0, 0.0)], "map", TICKS))


def main() -> int:
    rclpy.init()
    node = NavRoundTrip()
    try:
        deadline = time.time() + 10.0
        while time.time() < deadline and len(node.received) < 4:
            node.publish_all()
            rclpy.spin_once(node, timeout_sec=0.1)

        missing = {"odom", "scan", "map", "plan"} - set(node.received)
        if missing:
            print(f"missing nav messages: {sorted(missing)}")
            return 1

        odom = node.received["odom"]
        assert abs(odom.pose.pose.position.x - 1.25) < 1e-9
        assert command_from_twist(Twist(linear=odom.twist.twist.linear, angular=odom.twist.twist.angular)) == (0.5, -0.1)
        assert len(node.received["map"].data) == 25
        assert len(node.received["plan"].poses) == 2
        print("nav round-trip passed")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

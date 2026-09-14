#!/usr/bin/env python3
"""Nav2-facing ROS 2 bridge for Robot Native Engine.

Unlike `run_node.py`, this node does not depend on `simulation_interfaces`: it
is a drop-in navigation target that a Nav2 stack can drive.

Publishes:
- `/clock` (`rosgraph_msgs/Clock`)
- `/odom` (`nav_msgs/Odometry`)
- `/tf` (`tf2_msgs/TFMessage`, `map -> odom -> base_link -> lidar`)
- `/scan` (`sensor_msgs/LaserScan`)
- `/map` (`nav_msgs/OccupancyGrid`)
- `/plan` (`nav_msgs/Path`)

Subscribes:
- `/cmd_vel` (`geometry_msgs/Twist`)

The base pose is integrated from `/cmd_vel` with a planar differential-drive
model, and a synthetic walled room provides the scan. Run it with:

```bash
source /opt/ros/jazzy/setup.bash
python3 nav_node.py
```
"""

from __future__ import annotations

import math
import os

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage

from ros_convert import (
    command_from_twist,
    make_clock_message,
    make_joint_state,
    make_laserscan,
    make_occupancy_grid,
    make_odometry,
    make_path,
    make_transform_stamped,
    make_tf_message,
)

BEAM_COUNT = 360
ROOM_HALF_X_M = 5.0
ROOM_HALF_Y_M = 3.0


class RneNavBridge(Node):
    """Integrates `/cmd_vel` and publishes Nav2-consumable navigation topics."""

    def __init__(self) -> None:
        super().__init__("rne_nav_bridge")
        self.declare_parameter("publish_period_s", 0.05)
        self.declare_parameter("range_max_m", 30.0)
        self.declare_parameter("map_file", os.environ.get("RNE_MAP_FILE", ""))
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        latching = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", latching)
        self.plan_pub = self.create_publisher(Path, "/plan", 10)
        self.joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)

        self.x_m = 0.0
        self.y_m = 0.0
        self.yaw_rad = 0.0
        self.linear_m_s = 0.0
        self.angular_rad_s = 0.0
        self.left_angle_rad = 0.0
        self.right_angle_rad = 0.0
        self.wheel_radius_m = 0.05
        self.track_width_m = 0.3
        self.sim_ticks = 0
        self.dt_s = float(self.get_parameter("publish_period_s").value)
        self.range_max_m = float(self.get_parameter("range_max_m").value)
        self.map_spec = self._load_map_spec() or self._synthetic_map_spec()
        self.plan = [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
        self.create_timer(self.dt_s, self.tick)
        self.get_logger().info("RNE nav bridge ready; publish /cmd_vel to drive")

    def handle_cmd_vel(self, message: Twist) -> None:
        """Stores the latest `/cmd_vel` command."""
        self.linear_m_s, self.angular_rad_s = command_from_twist(message)

    def tick(self) -> None:
        """Integrate one step and publish the navigation frame."""
        self.x_m += self.linear_m_s * math.cos(self.yaw_rad) * self.dt_s
        self.y_m += self.linear_m_s * math.sin(self.yaw_rad) * self.dt_s
        self.yaw_rad += self.angular_rad_s * self.dt_s
        half_track = self.track_width_m * 0.5
        left_rate = (self.linear_m_s - self.angular_rad_s * half_track) / self.wheel_radius_m
        right_rate = (self.linear_m_s + self.angular_rad_s * half_track) / self.wheel_radius_m
        self.left_angle_rad += left_rate * self.dt_s
        self.right_angle_rad += right_rate * self.dt_s
        self.sim_ticks += int(self.dt_s * 1_000_000_000)
        self.publish_frame()

    def publish_frame(self) -> None:
        ticks = self.sim_ticks
        pose = (self.x_m, self.y_m, self.yaw_rad)
        self.clock_pub.publish(make_clock_message(ticks))
        self.odom_pub.publish(
            make_odometry(
                pose,
                self.linear_m_s,
                self.angular_rad_s,
                "odom",
                "base_footprint",
                ticks,
            )
        )
        self.tf_pub.publish(
            make_tf_message(
                [
                    make_transform_stamped("map", "odom", (0.0, 0.0, 0.0), ticks),
                    make_transform_stamped(
                        "odom",
                        "base_footprint",
                        (self.x_m, self.y_m, 0.0),
                        ticks,
                        yaw=self.yaw_rad,
                    ),
                    make_transform_stamped("base_footprint", "base_link", (0.0, 0.0, 0.0), ticks),
                    make_transform_stamped("base_link", "lidar", (0.0, 0.2, 0.0), ticks),
                ]
            )
        )
        ranges = [self._ray_to_wall(self.x_m, self.y_m, angle) for angle in self._beam_angles()]
        self.scan_pub.publish(
            make_laserscan(
                ranges,
                0.0,
                2.0 * math.pi / BEAM_COUNT,
                0.05,
                self.range_max_m,
                "lidar",
                ticks,
            )
        )
        width, height, resolution, origin, data = self.map_spec
        self.map_pub.publish(
            make_occupancy_grid(width, height, resolution, origin, data, "map", ticks)
        )
        self.plan_pub.publish(make_path(self.plan, "map", ticks))
        half_track = self.track_width_m * 0.5
        left_rate = (self.linear_m_s - self.angular_rad_s * half_track) / self.wheel_radius_m
        right_rate = (self.linear_m_s + self.angular_rad_s * half_track) / self.wheel_radius_m
        self.joint_pub.publish(
            make_joint_state(
                ["left_wheel_joint", "right_wheel_joint"],
                [self.left_angle_rad, self.right_angle_rad],
                [left_rate, right_rate],
                "base_link",
                ticks,
            )
        )

    @staticmethod
    def _beam_angles() -> list[float]:
        return [2.0 * math.pi * beam / BEAM_COUNT for beam in range(BEAM_COUNT)]

    @staticmethod
    def _ray_to_wall(x: float, y: float, angle: float) -> float:
        dx, dy = math.cos(angle), math.sin(angle)
        best = float("inf")
        for bound, position, direction in (
            (ROOM_HALF_X_M, x, dx),
            (-ROOM_HALF_X_M, x, dx),
            (ROOM_HALF_Y_M, y, dy),
            (-ROOM_HALF_Y_M, y, dy),
        ):
            if abs(direction) > 1.0e-9:
                t = (bound - position) / direction
                if t > 0.0:
                    best = min(best, t)
        return best

    @staticmethod
    def _synthetic_map_spec() -> tuple[int, int, float, tuple[float, float, float], list[int]]:
        width = height = 20
        resolution = 0.5
        origin = (-5.0, -5.0, 0.0)
        data = [0] * (width * height)
        for x in range(width):
            data[x] = 100
            data[(height - 1) * width + x] = 100
        for y in range(height):
            data[y * width] = 100
            data[y * width + width - 1] = 100
        return width, height, resolution, origin, data

    def _load_map_spec(
        self,
    ) -> tuple[int, int, float, tuple[float, float, float], list[int]] | None:
        """Loads a `map_server` PGM+YAML pair when `map_file` is set."""
        map_file = str(self.get_parameter("map_file").value or "")
        if not map_file:
            return None
        if not os.path.isfile(map_file):
            self.get_logger().warn(f"map_file not found: {map_file}")
            return None
        try:
            width, height, resolution, origin, data = self._read_ros_map(map_file)
        except (OSError, ValueError) as error:
            self.get_logger().warn(f"failed to load map_file {map_file}: {error}")
            return None
        self.get_logger().info(
            f"loaded SLAM map {width}x{height} @ {resolution} m from {map_file}"
        )
        return width, height, resolution, origin, data

    @staticmethod
    def _read_ros_map(
        yaml_path: str,
    ) -> tuple[int, int, float, tuple[float, float, float], list[int]]:
        with open(yaml_path, "r", encoding="utf-8") as handle:
            text = handle.read()
        fields: dict[str, str] = {}
        for line in text.splitlines():
            if line.strip().startswith("#") or ":" not in line:
                continue
            key, value = line.split(":", 1)
            fields[key.strip()] = value.strip()
        image = fields.get("image")
        if not image:
            raise ValueError("map yaml has no image field")
        image_path = (
            image if os.path.isabs(image) else os.path.join(os.path.dirname(yaml_path), image)
        )
        resolution = float(fields.get("resolution", "0.05"))
        origin_text = fields.get("origin", "[0, 0, 0]").strip().strip("[]")
        ox, oy, oyaw = (float(part) for part in origin_text.split(","))
        width, height, pixels = RneNavBridge._read_pgm(image_path)
        # ROS map_server: 0 occupied, 254 free, 205 unknown -> RNE cell values.
        data = [-1 if value == 205 else (0 if value >= 250 else 100) for value in pixels]
        return width, height, resolution, (ox, oy, oyaw), data

    @staticmethod
    def _read_pgm(path: str) -> tuple[int, int, list[int]]:
        with open(path, "rb") as handle:
            raw = handle.read()
        position = 0

        def next_token() -> bytes:
            nonlocal position
            while position < len(raw):
                char = raw[position : position + 1]
                if char in b" \t\r\n":
                    position += 1
                    continue
                if char == b"#":
                    while position < len(raw) and raw[position : position + 1] != b"\n":
                        position += 1
                    continue
                break
            start = position
            while position < len(raw) and raw[position : position + 1] not in b" \t\r\n":
                position += 1
            return raw[start:position]

        magic = next_token()
        if magic != b"P5":
            raise ValueError(f"unsupported PGM magic {magic!r}")
        width = int(next_token())
        height = int(next_token())
        max_value = int(next_token())
        if max_value > 255:
            raise ValueError("16-bit PGM is not supported")
        if position < len(raw) and raw[position : position + 1] in b" \t\r\n":
            position += 1
        pixels = list(raw[position : position + width * height])
        if len(pixels) != width * height:
            raise ValueError("truncated PGM raster")
        return width, height, pixels


def main() -> None:
    rclpy.init()
    node = RneNavBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

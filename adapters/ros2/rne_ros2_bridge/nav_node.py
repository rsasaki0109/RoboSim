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

Actions (when `nav2_msgs` is available):
- `/navigate_to_pose` (`nav2_msgs/action/NavigateToPose`) with feedback and a
  spin recovery when progress stalls.

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
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.lifecycle import LifecycleNode, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan
from tf2_msgs.msg import TFMessage

try:
    from nav2_msgs.action import (
        BackUp,
        ComputePathToPose,
        FollowPath,
        FollowWaypoints,
        NavigateToPose,
        Spin,
    )
    from nav2_msgs.srv import LoadMap
    from rclpy.action import ActionServer, CancelResponse, GoalResponse

    HAS_ACTION = True
except ImportError:  # pragma: no cover - nav2_msgs is optional
    HAS_ACTION = False

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


class RneNavBridge(LifecycleNode):
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
        self.goal_active = False
        self.goal_tolerance_m = 0.15
        self.max_linear_m_s = 0.6
        self.max_angular_rad_s = 1.5
        self.k_yaw = 1.5
        self.slow_radius_m = 0.5
        self.stall_ticks = 20
        self.recovery_spin_s = 1.0
        self.recovery_spin_rad_s = 1.0
        self.action_servers = []
        self.load_map_service = None
        if HAS_ACTION:
            server_specs = [
                (NavigateToPose, "navigate_to_pose", self.execute_navigate),
                (ComputePathToPose, "compute_path_to_pose", self.execute_compute_path),
                (FollowPath, "follow_path", self.execute_follow_path),
                (FollowWaypoints, "follow_waypoints", self.execute_follow_waypoints),
                (Spin, "spin", self.execute_spin),
                (BackUp, "backup", self.execute_backup),
            ]
            for action_type, name, callback in server_specs:
                self.action_servers.append(
                    ActionServer(
                        self,
                        action_type,
                        name,
                        execute_callback=callback,
                        goal_callback=self.handle_goal,
                        cancel_callback=self.handle_cancel,
                    )
                )
            self.load_map_service = self.create_service(
                LoadMap, "load_map", self.handle_load_map
            )
            self.get_logger().info("Nav2 action servers ready")
        else:
            self.get_logger().warn("nav2_msgs unavailable; action servers disabled")
        self.create_timer(self.dt_s, self.tick)
        self.get_logger().info("RNE nav bridge ready; publish /cmd_vel to drive")

    def on_configure(self, state: object) -> object:
        """Lifecycle configure callback."""
        self.get_logger().info("configured")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: object) -> object:
        """Lifecycle activate callback."""
        self.get_logger().info("activated")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: object) -> object:
        """Lifecycle deactivate callback."""
        self.get_logger().info("deactivated")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: object) -> object:
        """Lifecycle cleanup callback."""
        self.get_logger().info("cleaned up")
        return TransitionCallbackReturn.SUCCESS

    def handle_cmd_vel(self, message: Twist) -> None:
        """Stores the latest `/cmd_vel` command."""
        self.linear_m_s, self.angular_rad_s = command_from_twist(message)

    def tick(self) -> None:
        """Integrate one `/cmd_vel` step and publish the navigation frame."""
        if self.goal_active:
            return
        self.step_base(self.linear_m_s, self.angular_rad_s)

    def step_base(self, linear_m_s: float, angular_rad_s: float) -> None:
        """Integrates one control step and publishes the navigation frame."""
        self.linear_m_s = linear_m_s
        self.angular_rad_s = angular_rad_s
        self.x_m += linear_m_s * math.cos(self.yaw_rad) * self.dt_s
        self.y_m += linear_m_s * math.sin(self.yaw_rad) * self.dt_s
        self.yaw_rad += angular_rad_s * self.dt_s
        half_track = self.track_width_m * 0.5
        left_rate = (linear_m_s - angular_rad_s * half_track) / self.wheel_radius_m
        right_rate = (linear_m_s + angular_rad_s * half_track) / self.wheel_radius_m
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

    def handle_goal(self, goal_request: object) -> object:
        """Accepts every NavigateToPose goal."""
        self.get_logger().info("accepted navigate_to_pose goal")
        return GoalResponse.ACCEPT

    def handle_cancel(self, goal_handle: object) -> object:
        """Accepts every cancellation request."""
        return CancelResponse.ACCEPT

    def _step_toward(self, target_x: float, target_y: float, distance: float) -> None:
        """Takes one heading-controller step toward a target."""
        dx = target_x - self.x_m
        dy = target_y - self.y_m
        heading = math.atan2(dy, dx)
        error = math.atan2(
            math.sin(heading - self.yaw_rad),
            math.cos(heading - self.yaw_rad),
        )
        angular = max(
            -self.max_angular_rad_s,
            min(self.max_angular_rad_s, self.k_yaw * error),
        )
        linear = self.max_linear_m_s if abs(error) < 0.6 else 0.0
        if distance < self.slow_radius_m:
            linear *= max(0.0, distance / self.slow_radius_m)
        self.step_base(linear, angular)

    def _pursue_to(self, goal_handle: object, target_x: float, target_y: float, publish) -> int:
        """Drives to `(target_x, target_y)` with a spin recovery on stall.

        Returns `0` on success, `1` when canceled, and `-1` on shutdown.
        """
        best_distance = float("inf")
        stall = 0
        recoveries = 0
        spin_remaining_s = 0.0
        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self.step_base(0.0, 0.0)
                goal_handle.canceled()
                return 1
            distance = math.hypot(target_x - self.x_m, target_y - self.y_m)
            if distance <= self.goal_tolerance_m:
                self.step_base(0.0, 0.0)
                return 0
            if distance < best_distance - 0.01:
                best_distance = distance
                stall = 0
            else:
                stall += 1
            if spin_remaining_s > 0.0:
                self.step_base(0.0, self.recovery_spin_rad_s)
                spin_remaining_s -= self.dt_s
            elif stall >= self.stall_ticks:
                stall = 0
                recoveries += 1
                spin_remaining_s = self.recovery_spin_s
                self.get_logger().warn(f"progress stalled; recovery spin {recoveries}")
                self.step_base(0.0, self.recovery_spin_rad_s)
            else:
                self._step_toward(target_x, target_y, distance)
            if publish is not None:
                publish(distance, recoveries)
            time.sleep(self.dt_s)
        return -1

    def _goal_result(self, goal_handle: object, result: object, code: int, name: str) -> object:
        """Maps an `_pursue_to` code onto an action result."""
        if code == 0:
            goal_handle.succeed()
            result.error_code = 0
            result.error_msg = ""
            self.get_logger().info(f"{name} reached goal")
        elif code == 1:
            result.error_code = 1
            result.error_msg = "canceled"
        else:
            result.error_code = -1
            result.error_msg = "shutdown"
        return result

    def execute_navigate(self, goal_handle: object) -> object:
        """Drives to a goal with feedback and a spin recovery on stall."""
        position = goal_handle.request.pose.pose.position
        target_x, target_y = float(position.x), float(position.y)
        self.plan = [(0.0, 0.0, 0.0), (target_x, target_y, 0.0)]
        self.goal_active = True
        feedback = NavigateToPose.Feedback()
        result = NavigateToPose.Result()
        self.get_logger().info(f"navigating to ({target_x:.2f}, {target_y:.2f})")

        def publish(distance: float, recoveries: int) -> None:
            feedback.distance_remaining = float(distance)
            feedback.number_of_recoveries = recoveries
            goal_handle.publish_feedback(feedback)

        try:
            code = self._pursue_to(goal_handle, target_x, target_y, publish)
        finally:
            self.goal_active = False
        return self._goal_result(goal_handle, result, code, "navigate_to_pose")

    def execute_compute_path(self, goal_handle: object) -> object:
        """Publishes a straight-line plan to the requested goal."""
        goal_pose = goal_handle.request.goal.pose.position
        target_x, target_y = float(goal_pose.x), float(goal_pose.y)
        result = ComputePathToPose.Result()
        self.plan = [
            (self.x_m, self.y_m, self.yaw_rad),
            (target_x, target_y, 0.0),
        ]
        result.path = make_path(self.plan, "map", self.sim_ticks)
        result.planning_time.sec = 0
        result.planning_time.nanosec = 0
        result.error_code = 0
        result.error_msg = ""
        self.plan_pub.publish(result.path)
        goal_handle.succeed()
        self.get_logger().info(f"computed path to ({target_x:.2f}, {target_y:.2f})")
        return result

    def execute_follow_path(self, goal_handle: object) -> object:
        """Follows an externally supplied `nav_msgs/Path`."""
        poses = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in goal_handle.request.path.poses
        ]
        self.plan = [(self.x_m, self.y_m, self.yaw_rad)] + [
            (x, y, 0.0) for x, y in poses
        ]
        self.goal_active = True
        feedback = FollowPath.Feedback()
        result = FollowPath.Result()
        error = 0

        def make_publish(target_x: float, target_y: float):
            def publish(_distance: float, _recoveries: int) -> None:
                feedback.distance_to_goal = float(
                    math.hypot(target_x - self.x_m, target_y - self.y_m)
                )
                feedback.speed = float(self.linear_m_s)
                goal_handle.publish_feedback(feedback)

            return publish

        try:
            for target_x, target_y in poses:
                error = self._pursue_to(
                    goal_handle, target_x, target_y, make_publish(target_x, target_y)
                )
                if error != 0:
                    break
        finally:
            self.goal_active = False
        return self._goal_result(goal_handle, result, error, "follow_path")

    def execute_follow_waypoints(self, goal_handle: object) -> object:
        """Drives through a list of waypoints, looping as requested."""
        poses = [
            (float(pose.pose.position.x), float(pose.pose.position.y))
            for pose in goal_handle.request.poses
        ]
        loops = max(1, int(goal_handle.request.number_of_loops))
        self.goal_active = True
        feedback = FollowWaypoints.Feedback()
        result = FollowWaypoints.Result()
        missed = []
        error = 0
        try:
            for _ in range(loops):
                for index, (target_x, target_y) in enumerate(poses):
                    feedback.current_waypoint = index
                    goal_handle.publish_feedback(feedback)
                    error = self._pursue_to(goal_handle, target_x, target_y, None)
                    if error != 0:
                        missed.append(index)
                        if error == 1:
                            break
                if error == 1:
                    break
        finally:
            self.goal_active = False
        result.missed_waypoints = missed
        if error == 1:
            result.error_code = 1
            result.error_msg = "canceled"
        else:
            result.error_code = 0
            result.error_msg = ""
            goal_handle.succeed()
        return result

    def execute_spin(self, goal_handle: object) -> object:
        """Rotates in place to the requested yaw."""
        target_yaw = float(goal_handle.request.target_yaw)
        feedback = Spin.Feedback()
        result = Spin.Result()
        self.goal_active = True
        elapsed_s = 0.0
        traveled_rad = 0.0
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.step_base(0.0, 0.0)
                    goal_handle.canceled()
                    result.error_code = 1
                    result.error_msg = "canceled"
                    return result
                error = math.atan2(
                    math.sin(target_yaw - self.yaw_rad),
                    math.cos(target_yaw - self.yaw_rad),
                )
                if abs(error) < 0.05:
                    self.step_base(0.0, 0.0)
                    break
                angular = max(
                    -self.max_angular_rad_s,
                    min(self.max_angular_rad_s, self.k_yaw * error),
                )
                self.step_base(0.0, angular)
                traveled_rad += abs(angular) * self.dt_s
                elapsed_s += self.dt_s
                feedback.angular_distance_traveled = float(traveled_rad)
                goal_handle.publish_feedback(feedback)
                time.sleep(self.dt_s)
        finally:
            self.goal_active = False
        result.total_elapsed_time.sec = int(elapsed_s)
        result.total_elapsed_time.nanosec = int((elapsed_s % 1.0) * 1.0e9)
        result.error_code = 0
        result.error_msg = ""
        goal_handle.succeed()
        self.get_logger().info(f"spin complete; traveled={traveled_rad:.3f} rad")
        return result

    def execute_backup(self, goal_handle: object) -> object:
        """Drives backward by the requested distance."""
        target_distance = abs(float(goal_handle.request.target.x))
        speed = abs(float(goal_handle.request.speed)) or self.max_linear_m_s
        feedback = BackUp.Feedback()
        result = BackUp.Result()
        self.goal_active = True
        elapsed_s = 0.0
        traveled_m = 0.0
        try:
            while rclpy.ok():
                if goal_handle.is_cancel_requested:
                    self.step_base(0.0, 0.0)
                    goal_handle.canceled()
                    result.error_code = 1
                    result.error_msg = "canceled"
                    return result
                if traveled_m >= target_distance:
                    self.step_base(0.0, 0.0)
                    break
                self.step_base(-speed, 0.0)
                traveled_m += speed * self.dt_s
                elapsed_s += self.dt_s
                feedback.distance_traveled = float(traveled_m)
                goal_handle.publish_feedback(feedback)
                time.sleep(self.dt_s)
        finally:
            self.goal_active = False
        result.total_elapsed_time.sec = int(elapsed_s)
        result.total_elapsed_time.nanosec = int((elapsed_s % 1.0) * 1.0e9)
        result.error_code = 0
        result.error_msg = ""
        goal_handle.succeed()
        self.get_logger().info(f"backup complete; traveled={traveled_m:.3f} m")
        return result

    def handle_load_map(self, request: object, response: object) -> object:
        """Loads a `map_server` PGM+YAML pair requested through `LoadMap`."""
        url = str(request.map_url or "")
        if url and os.path.isfile(url):
            try:
                width, height, resolution, origin, data = self._read_ros_map(url)
                self.map_spec = (width, height, resolution, origin, data)
                response.map = make_occupancy_grid(
                    width, height, resolution, origin, data, "map", self.sim_ticks
                )
                response.result = 0
                self.get_logger().info(f"loaded map from {url}")
                return response
            except (OSError, ValueError) as error:
                self.get_logger().warn(f"load_map failed: {error}")
        response.result = 1
        return response

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

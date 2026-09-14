#!/usr/bin/env python3
"""Minimal ROS 2 bridge for Robot Native Engine.

Publishes:
- `/clock` from simulation time
- `/points` as `sensor_msgs/PointCloud2`
- `/tf` as `tf2_msgs/TFMessage`
- `/odom` as `nav_msgs/Odometry`
- `/scan` as `sensor_msgs/LaserScan`
- `/map` as `nav_msgs/OccupancyGrid`
- `/plan` as `nav_msgs/Path`

Subscribes:
- `/cmd_vel` as `geometry_msgs/Twist`

Exposes `simulation_interfaces` services, action, and parameters when running
with `rne_py`. Without bindings, publishes one synthetic frame for smoke testing.
"""

from __future__ import annotations

import math
import os
import time
from typing import TYPE_CHECKING

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import LaserScan, PointCloud2
from simulation_interfaces.action import SimulateSteps
from simulation_interfaces.msg import (
    EntityInfo,
    EntityState,
    Result,
    SimulationState,
)
from simulation_interfaces.srv import (
    DeleteEntity,
    GetEntities,
    GetEntityInfo,
    GetEntityState,
    GetSimulationState,
    ResetSimulation,
    SetEntityState,
    SetSimulationState,
    SpawnEntity,
    StepSimulation,
)
from tf2_msgs.msg import TFMessage

from ros_convert import (
    command_from_twist,
    make_clock_message,
    make_laserscan,
    make_occupancy_grid,
    make_odometry,
    make_path,
    make_pointcloud2,
    make_tf_message,
    make_transform_stamped,
)
from sim_control import BridgeSim, SIM_DT_NS

try:
    import rne_py
except ImportError:
    rne_py = None

if TYPE_CHECKING:
    from rclpy.action.server import ServerGoalHandle


SIM_STEPS = 300
MIN_FORWARD_X_M = 0.8


class RneBridgeNode(Node):
    """ROS 2 adapter node for RNE simulation outputs."""

    def __init__(self) -> None:
        super().__init__("rne_bridge")
        self.declare_parameter("wheel_velocity_rad_s", 6.0)
        self.declare_parameter("real_time_factor", 1.0)
        self.clock_pub = self.create_publisher(Clock, "/clock", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/points", 10)
        self.tf_pub = self.create_publisher(TFMessage, "/tf", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.scan_pub = self.create_publisher(LaserScan, "/scan", 10)
        self.map_pub = self.create_publisher(OccupancyGrid, "/map", 10)
        self.plan_pub = self.create_publisher(Path, "/plan", 10)
        self.command: tuple[float, float] = (0.0, 0.0)
        self.create_subscription(Twist, "/cmd_vel", self.handle_cmd_vel, 10)
        self.map_spec = self._synthetic_map_spec()
        self.plan = self._synthetic_path()
        self.use_rne_sim = rne_py is not None
        self.callback_group = ReentrantCallbackGroup()
        self.real_time_factor = float(self.get_parameter("real_time_factor").value)
        self.entities: dict[str, EntityState] = {}
        self.entity_info: dict[str, EntityInfo] = {}
        self._seed_entities()
        self._register_entity_interfaces()
        if self.use_rne_sim:
            self.bridge = BridgeSim.new(rne_py.DiffDriveSim())
            self._register_simulation_interfaces()
            self.get_logger().info("Driving headless diff-drive via rne_py")
        else:
            self.sim_ticks = 0
            self.get_logger().warn("rne_py not installed; publishing synthetic frame only")

    @property
    def wheel_velocity(self) -> float:
        return float(self.get_parameter("wheel_velocity_rad_s").value)

    def handle_cmd_vel(self, message: Twist) -> None:
        """Stores the latest `/cmd_vel` command for the odometry twist."""
        self.command = command_from_twist(message)

    @staticmethod
    def _synthetic_map_spec() -> tuple[int, int, float, tuple[float, float, float], list[int]]:
        """Builds a small walled occupancy grid for smoke testing."""
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

    @staticmethod
    def _synthetic_path() -> list[tuple[float, float, float]]:
        """Builds a short straight plan for smoke testing."""
        return [(0.0, 0.0, 0.0), (1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]

    def _seed_entities(self) -> None:
        """Registers the diff-drive base as the initial entity."""
        state = EntityState()
        state.header.frame_id = "world"
        self.entities["base_link"] = state
        self.entity_info["base_link"] = EntityInfo(
            description="RNE differential-drive base", tags=[]
        )

    def _register_entity_interfaces(self) -> None:
        """Registers Gazebo-compatible entity spawn/delete/query services."""
        self.create_service(SpawnEntity, "/spawn_entity", self.handle_spawn_entity)
        self.create_service(DeleteEntity, "/delete_entity", self.handle_delete_entity)
        self.create_service(GetEntities, "/get_entities", self.handle_get_entities)
        self.create_service(GetEntityInfo, "/get_entity_info", self.handle_get_entity_info)
        self.create_service(GetEntityState, "/get_entity_state", self.handle_get_entity_state)
        self.create_service(SetEntityState, "/set_entity_state", self.handle_set_entity_state)

    @staticmethod
    def _success() -> Result:
        return Result(result=Result.RESULT_OK, error_message="")

    @staticmethod
    def _failure(code: int, message: str) -> Result:
        return Result(result=code, error_message=message)

    def handle_spawn_entity(
        self, request: SpawnEntity.Request, response: SpawnEntity.Response
    ) -> SpawnEntity.Response:
        """Adds an entity to the registry, honoring `allow_renaming`."""
        name = request.name or f"entity_{len(self.entities)}"
        if name in self.entities:
            if not request.allow_renaming:
                response.result = self._failure(
                    Result.RESULT_OPERATION_FAILED, f"entity {name} already exists"
                )
                return response
            suffix = 1
            while f"{name}_{suffix}" in self.entities:
                suffix += 1
            name = f"{name}_{suffix}"
        state = EntityState()
        state.header.frame_id = "world"
        state.pose = request.initial_pose.pose
        self.entities[name] = state
        self.entity_info[name] = EntityInfo(
            description=request.uri or "spawned entity", tags=[]
        )
        response.entity_name = name
        response.result = self._success()
        return response

    def handle_delete_entity(
        self, request: DeleteEntity.Request, response: DeleteEntity.Response
    ) -> DeleteEntity.Response:
        """Removes an entity from the registry."""
        if request.entity not in self.entities:
            response.result = self._failure(
                Result.RESULT_NOT_FOUND, f"entity {request.entity} not found"
            )
            return response
        del self.entities[request.entity]
        self.entity_info.pop(request.entity, None)
        response.result = self._success()
        return response

    def handle_get_entities(
        self, request: GetEntities.Request, response: GetEntities.Response
    ) -> GetEntities.Response:
        """Lists entities, optionally filtered by a name substring."""
        pattern = request.filters.filter
        response.entities = sorted(
            name for name in self.entities if not pattern or pattern in name
        )
        response.result = self._success()
        return response

    def handle_get_entity_info(
        self, request: GetEntityInfo.Request, response: GetEntityInfo.Response
    ) -> GetEntityInfo.Response:
        """Returns static info for an entity."""
        info = self.entity_info.get(request.entity)
        if info is None:
            response.result = self._failure(
                Result.RESULT_NOT_FOUND, f"entity {request.entity} not found"
            )
            return response
        response.info = info
        response.result = self._success()
        return response

    def handle_get_entity_state(
        self, request: GetEntityState.Request, response: GetEntityState.Response
    ) -> GetEntityState.Response:
        """Returns the pose/twist of an entity."""
        state = self.entities.get(request.entity)
        if state is None:
            response.result = self._failure(
                Result.RESULT_NOT_FOUND, f"entity {request.entity} not found"
            )
            return response
        response.state = state
        response.result = self._success()
        return response

    def handle_set_entity_state(
        self, request: SetEntityState.Request, response: SetEntityState.Response
    ) -> SetEntityState.Response:
        """Updates the pose/twist of an entity."""
        if request.entity not in self.entities:
            response.result = self._failure(
                Result.RESULT_NOT_FOUND, f"entity {request.entity} not found"
            )
            return response
        self.entities[request.entity] = request.state
        response.result = self._success()
        return response

    def _register_simulation_interfaces(self) -> None:
        self.create_service(
            ResetSimulation,
            "/reset_simulation",
            self.handle_reset_simulation,
            callback_group=self.callback_group,
        )
        self.create_service(
            GetSimulationState,
            "/get_simulation_state",
            self.handle_get_simulation_state,
            callback_group=self.callback_group,
        )
        self.create_service(
            SetSimulationState,
            "/set_simulation_state",
            self.handle_set_simulation_state,
            callback_group=self.callback_group,
        )
        self.create_service(
            StepSimulation,
            "/step_simulation",
            self.handle_step_simulation,
            callback_group=self.callback_group,
        )
        self.simulate_steps_server = ActionServer(
            self,
            SimulateSteps,
            "/simulate_steps",
            execute_callback=self.execute_simulate_steps,
            goal_callback=self.handle_simulate_steps_goal,
            cancel_callback=self.handle_simulate_steps_cancel,
            callback_group=self.callback_group,
        )

    def publish_frame(self, base_xyz: tuple[float, float, float], points: list[tuple[float, float, float]]) -> None:
        """Publish one simulation frame to ROS 2 topics."""
        sim_ticks = self.bridge.sim_ticks if self.use_rne_sim else self.sim_ticks
        self.clock_pub.publish(make_clock_message(sim_ticks))
        self.cloud_pub.publish(make_pointcloud2(points, "lidar", sim_ticks))
        tf = make_tf_message(
            [
                make_transform_stamped("world", "base_link", base_xyz, sim_ticks),
                make_transform_stamped(
                    "base_link",
                    "lidar",
                    (0.0, 0.2, 0.0),
                    sim_ticks,
                ),
            ]
        )
        self.tf_pub.publish(tf)

        base_pose = (base_xyz[0], base_xyz[1], 0.0)
        linear_m_s, angular_rad_s = self.command
        self.odom_pub.publish(
            make_odometry(base_pose, linear_m_s, angular_rad_s, "odom", "base_link", sim_ticks)
        )
        ranges = [math.hypot(point[0], point[1]) for point in points]
        self.scan_pub.publish(
            make_laserscan(ranges, 0.0, 0.1, 0.05, 30.0, "lidar", sim_ticks)
        )
        width, height, resolution, origin, data = self.map_spec
        self.map_pub.publish(
            make_occupancy_grid(width, height, resolution, origin, data, "map", sim_ticks)
        )
        self.plan_pub.publish(make_path(self.plan, "map", sim_ticks))

    def _observation_frame(self) -> tuple[tuple[float, float, float], list[tuple[float, float, float]]]:
        obs = self.bridge.observation
        base = (obs.base_x, obs.base_y, obs.base_z)
        distance = max(base[0], 0.1)
        points = [(distance, 0.0, 0.0), (distance, 0.5, 0.0), (distance, -0.5, 0.0)]
        return base, points

    def spin_once(self) -> None:
        """Advance one simulation step or publish a synthetic sample."""
        if self.use_rne_sim:
            if self.bridge.step_if_playing(self.wheel_velocity):
                base, points = self._observation_frame()
                self.publish_frame(base, points)
            return

        base = (0.0, 0.25, 0.0)
        points = [(3.0, 0.0, 0.0), (3.0, 0.5, 0.0)]
        self.publish_frame(base, points)
        self.sim_ticks += SIM_DT_NS

    def handle_reset_simulation(
        self, request: ResetSimulation.Request, response: ResetSimulation.Response
    ) -> ResetSimulation.Response:
        response.result = self.bridge.reset(request.scope)
        base, points = self._observation_frame()
        self.publish_frame(base, points)
        return response

    def handle_get_simulation_state(
        self, _request: GetSimulationState.Request, response: GetSimulationState.Response
    ) -> GetSimulationState.Response:
        response.state = self.bridge.playback_state()
        response.result = Result(result=Result.RESULT_OK, error_message="")
        return response

    def handle_set_simulation_state(
        self, request: SetSimulationState.Request, response: SetSimulationState.Response
    ) -> SetSimulationState.Response:
        response.result = self.bridge.set_playback(request.state.state)
        base, points = self._observation_frame()
        self.publish_frame(base, points)
        return response

    def handle_step_simulation(
        self, request: StepSimulation.Request, response: StepSimulation.Response
    ) -> StepSimulation.Response:
        error = self.bridge.step_while_paused(int(request.steps), self.wheel_velocity)
        if error is not None:
            response.result = error
            return response
        base, points = self._observation_frame()
        self.publish_frame(base, points)
        response.result = Result(result=Result.RESULT_OK, error_message="")
        return response

    def handle_simulate_steps_goal(self, goal_request: SimulateSteps.Goal) -> GoalResponse:
        if self.bridge.playback != SimulationState.STATE_PAUSED:
            return GoalResponse.REJECT
        if goal_request.steps == 0:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def handle_simulate_steps_cancel(self, _goal_handle: ServerGoalHandle) -> CancelResponse:
        return CancelResponse.ACCEPT

    def execute_simulate_steps(self, goal_handle: ServerGoalHandle) -> SimulateSteps.Result:
        steps = int(goal_handle.request.steps)
        feedback = SimulateSteps.Feedback()
        for completed in range(1, steps + 1):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return SimulateSteps.Result(
                    result=Result(
                        result=Result.RESULT_OPERATION_FAILED,
                        error_message="simulate_steps cancelled",
                    )
                )
            error = self.bridge.step_while_paused(1, self.wheel_velocity)
            if error is not None:
                goal_handle.abort()
                return SimulateSteps.Result(result=error)
            base, points = self._observation_frame()
            self.publish_frame(base, points)
            feedback.completed_steps = completed
            feedback.remaining_steps = steps - completed
            goal_handle.publish_feedback(feedback)

        goal_handle.succeed()
        return SimulateSteps.Result(result=Result(result=Result.RESULT_OK, error_message=""))


def main() -> None:
    rclpy.init()
    node = RneBridgeNode()
    steps = SIM_STEPS if node.use_rne_sim else 1

    try:
        for step in range(steps):
            node.spin_once()
            rclpy.spin_once(node, timeout_sec=0.01)
            if step % 60 == 59 and node.use_rne_sim:
                obs = node.bridge.observation
                node.get_logger().info(f"step {step + 1}: base_x={obs.base_x:.2f} m")
            time.sleep(0.001 / max(node.real_time_factor, 1.0e-6))

        if node.use_rne_sim:
            obs = node.bridge.observation
            node.get_logger().info(f"final base_x={obs.base_x:.2f} m")
            if obs.base_x < MIN_FORWARD_X_M:
                raise SystemExit(
                    f"expected forward motion from diff-drive policy (base_x={obs.base_x:.2f} m)"
                )

        hold_secs = float(os.environ.get("RNE_ROS2_HOLD_SECS", "0"))
        if hold_secs > 0:
            node.get_logger().info(
                f"holding ROS graph for {hold_secs:.0f}s (RNE_ROS2_HOLD_SECS)"
            )
            end = time.time() + hold_secs
            while time.time() < end and rclpy.ok():
                try:
                    node.spin_once()
                    rclpy.spin_once(node, timeout_sec=0.05)
                except rclpy.executors.ExternalShutdownException:
                    break
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()

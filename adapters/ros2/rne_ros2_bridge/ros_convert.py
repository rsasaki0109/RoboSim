"""Convert RNE-style values into ROS 2 Python messages."""

from __future__ import annotations

import math
import struct
from typing import Iterable, Sequence, Tuple

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseStamped,
    Quaternion,
    Transform,
    TransformStamped,
    Twist,
    Vector3,
)
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState, LaserScan, PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

Point3 = Tuple[float, float, float]
Pose2d = Tuple[float, float, float]


def sim_ticks_to_ros_time(ticks: int) -> Tuple[int, int]:
    """Maps RNE nanosecond ticks to `(sec, nanosec)`."""
    sec = ticks // 1_000_000_000
    nanosec = ticks % 1_000_000_000
    return int(sec), int(nanosec)


def make_clock_message(ticks: int) -> Clock:
    """Builds a `/clock` message from simulation ticks."""
    sec, nanosec = sim_ticks_to_ros_time(ticks)
    clock = Clock()
    clock.clock.sec = sec
    clock.clock.nanosec = nanosec
    return clock


def make_header(frame_id: str, ticks: int) -> Header:
    """Builds a ROS header from simulation ticks."""
    sec, nanosec = sim_ticks_to_ros_time(ticks)
    header = Header()
    header.frame_id = frame_id
    header.stamp.sec = sec
    header.stamp.nanosec = nanosec
    return header


def make_pointcloud2(points: Sequence[Point3], frame_id: str, ticks: int) -> PointCloud2:
    """Builds a `sensor_msgs/PointCloud2` XYZ cloud."""
    cloud = PointCloud2()
    cloud.header = make_header(frame_id, ticks)
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        _field("x", 0),
        _field("y", 4),
        _field("z", 8),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    cloud.data = b"".join(struct.pack("<fff", x, y, z) for x, y, z in points)
    return cloud


def make_transform_stamped(
    parent_frame: str,
    child_frame: str,
    translation: Point3,
    ticks: int,
    yaw: float = 0.0,
) -> TransformStamped:
    """Builds a `TransformStamped` message with an optional pure-yaw rotation."""
    transform = TransformStamped()
    transform.header = make_header(parent_frame, ticks)
    transform.child_frame_id = child_frame
    transform.transform = Transform(
        translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
        rotation=make_quaternion_from_yaw(yaw),
    )
    return transform


def make_tf_message(transforms: Iterable[TransformStamped]) -> TFMessage:
    """Builds a `tf2_msgs/TFMessage`."""
    message = TFMessage()
    message.transforms = list(transforms)
    return message


def _field(name: str, offset: int) -> PointField:
    field = PointField()
    field.name = name
    field.offset = offset
    field.datatype = PointField.FLOAT32
    field.count = 1
    return field


def make_quaternion_from_yaw(yaw: float) -> Quaternion:
    """Builds a pure-yaw `geometry_msgs/Quaternion`."""
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def make_pose(x: float, y: float, yaw: float) -> Pose:
    """Builds a planar `geometry_msgs/Pose` at `z = 0`."""
    return Pose(
        position=Point(x=x, y=y, z=0.0),
        orientation=make_quaternion_from_yaw(yaw),
    )


def make_odometry(
    pose: Pose2d,
    linear_m_s: float,
    angular_rad_s: float,
    frame_id: str,
    child_frame_id: str,
    ticks: int,
) -> Odometry:
    """Builds a `nav_msgs/Odometry` message for a differential base."""
    message = Odometry()
    message.header = make_header(frame_id, ticks)
    message.child_frame_id = child_frame_id
    message.pose.pose = make_pose(*pose)
    message.twist.twist = Twist(
        linear=Vector3(x=linear_m_s, y=0.0, z=0.0),
        angular=Vector3(x=0.0, y=0.0, z=angular_rad_s),
    )
    return message


def make_occupancy_grid(
    width: int,
    height: int,
    resolution: float,
    origin: Pose2d,
    data: Sequence[int],
    frame_id: str,
    ticks: int,
) -> OccupancyGrid:
    """Builds a `nav_msgs/OccupancyGrid` with row-major, x-fastest data."""
    if len(data) != width * height:
        raise ValueError(f"expected {width * height} cells, got {len(data)}")
    message = OccupancyGrid()
    message.header = make_header(frame_id, ticks)
    message.info.resolution = float(resolution)
    message.info.width = int(width)
    message.info.height = int(height)
    message.info.origin = make_pose(*origin)
    message.data = [int(value) for value in data]
    return message


def make_path(poses: Sequence[Pose2d], frame_id: str, ticks: int) -> Path:
    """Builds a `nav_msgs/Path` from planar poses."""
    message = Path()
    message.header = make_header(frame_id, ticks)
    for x, y, yaw in poses:
        stamped = PoseStamped()
        stamped.header = make_header(frame_id, ticks)
        stamped.pose = make_pose(x, y, yaw)
        message.poses.append(stamped)
    return message


def make_laserscan(
    ranges: Sequence[float],
    angle_min: float,
    angle_increment: float,
    range_min: float,
    range_max: float,
    frame_id: str,
    ticks: int,
) -> LaserScan:
    """Builds a `sensor_msgs/LaserScan` from ranges."""
    message = LaserScan()
    message.header = make_header(frame_id, ticks)
    message.angle_min = float(angle_min)
    message.angle_max = float(angle_min + angle_increment * max(len(ranges) - 1, 0))
    message.angle_increment = float(angle_increment)
    message.range_min = float(range_min)
    message.range_max = float(range_max)
    message.ranges = [float(value) for value in ranges]
    return message


def command_from_twist(message: Twist) -> Tuple[float, float]:
    """Extracts `(linear_m_s, angular_rad_s)` from a `geometry_msgs/Twist`."""
    return float(message.linear.x), float(message.angular.z)


def make_joint_state(
    names: Sequence[str],
    positions: Sequence[float],
    velocities: Sequence[float],
    frame_id: str,
    ticks: int,
    efforts: Sequence[float] | None = None,
) -> JointState:
    """Builds a `sensor_msgs/JointState` for the ros2_control boundary."""
    message = JointState()
    message.header = make_header(frame_id, ticks)
    message.name = list(names)
    message.position = [float(value) for value in positions]
    message.velocity = [float(value) for value in velocities]
    message.effort = [float(value) for value in (efforts or [])]
    return message


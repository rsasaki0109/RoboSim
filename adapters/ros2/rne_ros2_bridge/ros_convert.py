"""Convert RNE-style values into ROS 2 Python messages."""

from __future__ import annotations

import struct
from typing import Iterable, Sequence, Tuple

from geometry_msgs.msg import Transform, TransformStamped, Vector3, Quaternion
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage

Point = Tuple[float, float, float]


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


def make_pointcloud2(points: Sequence[Point], frame_id: str, ticks: int) -> PointCloud2:
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
    translation: Point,
    ticks: int,
) -> TransformStamped:
    """Builds a static `TransformStamped` message."""
    transform = TransformStamped()
    transform.header = make_header(parent_frame, ticks)
    transform.child_frame_id = child_frame
    transform.transform = Transform(
        translation=Vector3(x=translation[0], y=translation[1], z=translation[2]),
        rotation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0),
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

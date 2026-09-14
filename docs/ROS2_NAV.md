# ROS 2 Navigation Adapter

The `rne_adapter_ros2` crate maps RNE navigation and SLAM types to ROS 2 message
shapes without a ROS runtime dependency. The shapes in `messages.rs` mirror the
ROS interfaces field-for-field, so a thin `rclrs` / `rclpy` node can serialize
them directly.

This is Phase 4a of the navigation roadmap. The live publish/subscribe node is
provided by the `rne_ros2_bridge` Python node (`run_node.py`), which publishes
the topics below and subscribes `/cmd_vel`; `test_nav_roundtrip.py` exercises
the mappings through a live rclpy graph.

## Topic contract

| Topic | ROS type | RNE source | Mapping |
| --- | --- | --- | --- |
| `/scan` | `sensor_msgs/LaserScan` | `rne_data::PointCloud` + `rne_sensor::LidarSpec` | `pointcloud_to_laserscan` |
| `/odom` | `nav_msgs/Odometry` | `rne_nav::Pose2d` + `VelocityCommand2d` | `to_ros_odometry` |
| `/tf`, `/tf_static` | `tf2_msgs/TFMessage` | `rne_nav::TfBuffer` / `rne_world::FrameGraph` | `to_ros_tf_message_from_buffer`, `to_ros_tf_message` |
| `/map` | `nav_msgs/OccupancyGrid` | `rne_slam` map / `rne_nav::OccupancyGrid` | `to_ros_occupancy_grid` |
| `/plan` | `nav_msgs/Path` | `rne_nav::Path2d` | `to_ros_path` |
| `/cmd_vel` | `geometry_msgs/Twist` | `rne_nav::VelocityCommand2d` | `from_ros_twist` |

## Message shapes

`messages.rs` adds `RosPose`, `RosPoseWithCovariance`, `RosTwist`,
`RosTwistWithCovariance`, `RosOdometry`, `RosMapMetaData`, `RosOccupancyGrid`,
`RosPoseStamped`, and `RosPath` alongside the existing sensor and TF shapes.

## Conventions

- Planar poses map to `z = 0` with a pure-yaw quaternion; `yaw_to_pose` and
  `yaw_from_ros_quaternion` convert in both directions.
- Occupancy grid data is row-major with x fastest, matching both
  `nav_msgs/OccupancyGrid` and `rne_nav::OccupancyGrid`. Cell values use the ROS
  convention `-1` unknown, `0` free, `100` occupied.
- The grid `origin` is the pose of cell `(0, 0)`'s center in the map frame.
- `from_ros_twist` keeps only the forward linear and yaw angular components, so
  a Nav2 `cmd_vel` drives a differential base without lateral motion.
- TF messages use the most recent sample of every `TfBuffer` edge, sorted by
  frame ids for deterministic output.

## Boundary

Core crates stay ROS-free: `rne_nav` and `rne_slam` expose only RNE types, and
this adapter is the single place that knows ROS message layouts. The optional
`ros2` feature gates the runtime node; mapping functions have no feature gate.

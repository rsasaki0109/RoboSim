# ROS2 Adapters

This directory contains optional adapters between Robot Native Engine and external ecosystems.

## Rules

- Core crates under `crates/` must never depend on ROS2 runtime crates in this directory.
- URDF import lives in `crates/rne_urdf_import` (no ROS2 runtime dependency).
- `rne_adapter_ros2` maps RNE data to ROS2-compatible message shapes.
- Full ROS2 node publishing requires the optional `ros2` feature and a ROS2 installation.

## Crates

- `rne_adapter_ros2`: `/clock`, TF, PointCloud2, and navigation (`Odometry`,
  `OccupancyGrid`, `Path`, `Twist`) mapping helpers (Rust)
- `rne_ros2_bridge`: Python `rclpy` runtime node publishing `/clock`, `/points`,
  `/tf`, `/odom`, `/scan`, `/map`, `/plan` and subscribing `/cmd_vel`
- `rne_ros2_bridge/nav_node.py`: Nav2-facing node (no `simulation_interfaces`)
  that integrates `/cmd_vel` and publishes `/odom`, `/tf`, `/scan`, `/map`,
  `/plan`, `/joint_states`; `map_file` loads an RNE SLAM map
- `rne_ros2_node`: Native `rclrs` runtime node (same topics, headless `rne_ai` sim)

URDF import: `crates/rne_urdf_import`

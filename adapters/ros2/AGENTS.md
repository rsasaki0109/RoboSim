# ROS2 Adapters

This directory contains optional adapters between Robot Native Engine and external ecosystems.

## Rules

- Core crates under `crates/` must never depend on these adapters.
- `rne_urdf_import` has no ROS2 runtime dependency. It only parses URDF files.
- `rne_adapter_ros2` maps RNE data to ROS2-compatible message shapes.
- Full ROS2 node publishing requires the optional `ros2` feature and a ROS2 installation.

## Crates

- `rne_urdf_import`: URDF → RNE Robot/Link/Joint entities with collider/visual attach
- `rne_adapter_ros2`: `/clock`, TF, PointCloud2 mapping helpers (Rust)
- `rne_ros2_bridge`: Python `rclpy` runtime node publishing `/clock`, `/points`, `/tf`
- `rne_ros2_node`: Native `rclrs` runtime node (same topics, headless `rne_ai` sim)

# ROS2 Adapters

This directory contains optional adapters between Robot Native Engine and external ecosystems.

## Rules

- Core crates under `crates/` must never depend on ROS2 runtime crates in this directory.
- URDF import lives in `crates/rne_urdf_import` (no ROS2 runtime dependency).
- `rne_adapter_ros2` maps RNE data to ROS2-compatible message shapes.
- Full ROS2 node publishing requires the optional `ros2` feature and a ROS2 installation.

## Crates

- `rne_adapter_ros2`: `/clock`, TF, PointCloud2 mapping helpers (Rust)
- `rne_ros2_bridge`: Python `rclpy` runtime node publishing `/clock`, `/points`, `/tf`
- `rne_ros2_node`: Native `rclrs` runtime node (same topics, headless `rne_ai` sim)

URDF import: `crates/rne_urdf_import`

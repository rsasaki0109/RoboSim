# v0.1.0 Roadmap (post-release)

## Completed in v0.1.0

- [x] Core ECS + world + math
- [x] Rapier physics + determinism tests
- [x] Diff-drive robot + sensors + DataBus
- [x] Render skeleton + Python bindings
- [x] URDF import + ROS 2 Python bridge
- [x] GitHub Release

## v0.2 candidates

| Area | Task |
|------|------|
| Rendering | Mesh rendering, depth pass |
| Robot | URDF → collider/visual auto attach |
| ROS 2 | Native `rclrs` node when type-support is available |
| AI | Episode API, reward/termination |
| Assets | `.rne.scene.toml` / `.rne.robot.toml` format |

## Native ROS 2 (`rclrs`)

The Python bridge in `adapters/ros2/rne_ros2_bridge/` is the supported runtime path today.

A native Rust node requires `ros2-rust` message crates built into the ROS underlay.
On Jazzy, install generator support:

```bash
sudo apt install ros-jazzy-rosidl-generator-rs
```

Then build `ros2_rust` from source into the workspace overlay before enabling
the `ros2` feature on `rne_adapter_ros2`.

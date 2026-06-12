# v0.1.0 Roadmap (post-release)

## Completed in v0.1.0

- [x] Core ECS + world + math
- [x] Rapier physics + determinism tests
- [x] Diff-drive robot + sensors + DataBus
- [x] Render skeleton + Python bindings
- [x] URDF import + ROS 2 Python bridge
- [x] GitHub Release

## v0.2 candidates

| Area | Task | Status |
|------|------|--------|
| AI | Episode API, reward/termination | Done (`rne_ai`, `05_episode_diff_drive`) |
| Robot | URDF → collider/visual auto attach | Done (`rne_urdf_import`, `Visual`) |
| Assets | `.rne.scene.toml` / `.rne.robot.toml` format | Done (`rne_assets`, `06_scene_load`) |
| Rendering | Mesh rendering, depth pass | Done (`rne_render_wgpu`, `07_render_primitives`) |
| ROS 2 | Native `rclrs` node when type-support is available | Done (`rne_ros2_node`) |

## v0.3 candidates

| Area | Task | Status |
|------|------|--------|
| Integration | End-to-end scene + episode example | Done (`08_scene_episode`) |
| Rendering | URDF mesh load + wgpu draw | Planned |
| AI | Domain randomization, vectorized envs | Planned |
| Robot | Rapier joint-driven wheels | Planned |
| Agent | Agent Entity component + policy attach | Planned |
| ROS 2 | Optional CI for `rne_ros2_node` | Planned |
| Release | v0.2.0 tag + docs refresh | Planned |

## Native ROS 2 (`rclrs`)

Two runtime paths are available:

- **Python** (`adapters/ros2/rne_ros2_bridge/`): `rclpy` node, optional `rne_py` bindings
- **Rust** (`adapters/ros2/rne_ros2_node/`): native `rclrs` node with headless `rne_ai` sim

The native node requires ROS message crates from the underlay. On Jazzy:

```bash
sudo apt install ros-jazzy-rosidl-generator-rs ros-jazzy-test-msgs
source /opt/ros/jazzy/setup.bash
cd adapters/ros2/rne_ros2_node
./smoke_test.sh
```

Message crates use `"*"` version pins and are patched via `generate_cargo_config.sh`.
The node is built with `--manifest-path` and is not part of the core workspace CI.

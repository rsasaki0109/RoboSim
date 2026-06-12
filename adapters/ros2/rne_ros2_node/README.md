# Native ROS 2 Node (`rclrs`)

Rust bridge that publishes RNE simulation outputs to ROS 2 topics, mirroring the
Python node in `../rne_ros2_bridge/`.

## Topics

| Topic | Type | Source |
|-------|------|--------|
| `/clock` | `rosgraph_msgs/Clock` | RNE `SimTime` ticks |
| `/points` | `sensor_msgs/PointCloud2` | LiDAR-style XYZ cloud |
| `/tf` | `tf2_msgs/TFMessage` | `world → base_link → lidar` |

The node drives a headless diff-drive simulation via `rne_ai::DiffDriveSim` (no Python).

## Prerequisites

- ROS 2 Jazzy (or Humble) with:
  - `ros-jazzy-rosidl-generator-rs`
  - `ros-jazzy-test-msgs` (required by `rclrs` vendor linkage)
- Rust toolchain (same as the main workspace)

Message crates from the ROS underlay use `"*"` version constraints and must be
patched to local paths. This crate is **not** a workspace member; build it with
`--manifest-path` after generating `.cargo/config.toml`.

## Build and run

```bash
source /opt/ros/jazzy/setup.bash
cd adapters/ros2/rne_ros2_node

./generate_cargo_config.sh
cargo build --release --manifest-path Cargo.toml
./target/release/rne_ros2_node
```

Or use the smoke script (unit tests + build + 300-step motion check):

```bash
./smoke_test.sh
```

Verify in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /clock --once
ros2 topic echo /points --once
ros2 topic echo /tf --once
```

## Architecture

```
rne_ai::DiffDriveSim
        ↓
rne_adapter_ros2  (RosClock / RosPointCloud2 / RosTfMessage)
        ↓
convert.rs        (→ rosgraph_msgs / sensor_msgs / tf2_msgs)
        ↓
rclrs publishers  (/clock, /points, /tf)
```

## CI note

Core workspace CI (`cargo run -p xtask -- ci`) does not build this crate because it requires
a sourced ROS environment and patched message crates.

When ROS 2 Jazzy (or Humble) is installed locally:

```bash
cargo run -p xtask -- ci-ros2
# or
./adapters/ros2/rne_ros2_node/smoke_test.sh
```

GitHub Actions runs the same smoke script in `.github/workflows/ros2-node.yml` on changes
under `adapters/ros2/` and core simulation crates the node depends on.

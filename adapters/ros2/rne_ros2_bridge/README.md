# ROS 2 Bridge

Python adapter node that publishes RNE simulation outputs to ROS 2 topics.

## Topics

| Topic | Type | Source |
|-------|------|--------|
| `/clock` | `rosgraph_msgs/Clock` | RNE `SimTime` ticks |
| `/points` | `sensor_msgs/PointCloud2` | LiDAR-style XYZ cloud |
| `/tf` | `tf2_msgs/TFMessage` | `world → base_link → lidar` |

## Prerequisites

- ROS 2 (tested with Jazzy)
- Optional: `rne_py` built with maturin for live diff-drive simulation

## Run

```bash
source /opt/ros/jazzy/setup.bash

# optional: live simulation via Python bindings
cd /path/to/RoboSim
python3 -m venv .venv
.venv/bin/pip install maturin
.venv/bin/maturin develop -m crates/rne_py/Cargo.toml

# bridge node
cd adapters/ros2/rne_ros2_bridge
PYTHONPATH="../../.venv/lib/python3.12/site-packages:${PYTHONPATH:-}" python3 run_node.py
```

Verify in another terminal:

```bash
source /opt/ros/jazzy/setup.bash
ros2 topic echo /clock --once
ros2 topic echo /points --once
ros2 topic echo /tf --once
```

## Tests (no ROS runtime)

```bash
cd adapters/ros2/rne_ros2_bridge
python3 test_ros_convert.py
```

## Rust mapping layer

Message layout helpers also exist in Rust at `../rne_adapter_ros2/`.
The Python node is the supported runtime path when `rclrs` type-support
libraries are not installed.

## Future: native `rclrs` node

A native Rust ROS 2 node requires `ros2-rust` message crates built into
the ROS underlay. Track native runtime support behind the `ros2` feature
in `rne_adapter_ros2`.

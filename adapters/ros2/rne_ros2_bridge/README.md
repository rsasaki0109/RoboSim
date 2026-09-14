# ROS 2 Bridge

Python adapter node that publishes RNE simulation outputs to ROS 2 topics.

## Topics

| Topic | Type | Source |
|-------|------|--------|
| `/clock` | `rosgraph_msgs/Clock` | RNE `SimTime` ticks |
| `/points` | `sensor_msgs/PointCloud2` | LiDAR-style XYZ cloud |
| `/tf` | `tf2_msgs/TFMessage` | `world → base_link → lidar` |
| `/odom` | `nav_msgs/Odometry` | Base pose + `/cmd_vel` twist |
| `/scan` | `sensor_msgs/LaserScan` | Ranges derived from the point cloud |
| `/map` | `nav_msgs/OccupancyGrid` | Occupancy grid (synthetic walls, or a loaded RNE SLAM map) |
| `/plan` | `nav_msgs/Path` | Planned path |
| `/joint_states` | `sensor_msgs/JointState` | Differential wheel positions/velocities |

## Subscriptions

| Topic | Type | Use |
|-------|------|-----|
| `/cmd_vel` | `geometry_msgs/Twist` | Latest forward/yaw command; echoed in `/odom` twist |

The `/odom`, `/scan`, `/map`, and `/plan` shapes match the RNE navigation
mappings documented in `docs/ROS2_NAV.md`, so a Nav2 stack can consume them.

## Services (`simulation_interfaces`)

| Service | Type |
|---------|------|
| `/reset_simulation` | `simulation_interfaces/srv/ResetSimulation` |
| `/get_simulation_state` | `simulation_interfaces/srv/GetSimulationState` |
| `/set_simulation_state` | `simulation_interfaces/srv/SetSimulationState` |
| `/step_simulation` | `simulation_interfaces/srv/StepSimulation` |

## Action

| Action | Type |
|--------|------|
| `/simulate_steps` | `simulation_interfaces/action/SimulateSteps` |

## Parameters

| Name | Type | Default |
|------|------|---------|
| `wheel_velocity_rad_s` | `double` | `6.0` |

## Prerequisites

- ROS 2 (tested with Jazzy)
- `ros-jazzy-rclpy` (or distro equivalent)
- Optional: `rne_py` built with maturin for live diff-drive simulation

## Parameters (`nav_node.py`)

| Name | Type | Default |
|------|------|---------|
| `publish_period_s` | `double` | `0.05` |
| `range_max_m` | `double` | `30.0` |
| `map_file` | `string` | `RNE_MAP_FILE` or empty (synthetic map) |

Set `map_file` (or `RNE_MAP_FILE`) to a `map_server` PGM+YAML pair written by
`RNE_SLAM_MAP_DIR=... cargo run -p nav_slam_physics --example 98_nav_slam_physics`
to publish a real RNE SLAM map on `/map`.

## Run

```bash
source /opt/ros/jazzy/setup.bash

# Nav2-facing node (no simulation_interfaces): /odom, /tf, /scan, /map, /plan
python3 nav_node.py

# Full node with simulation_interfaces services
python3 run_node.py
```

`nav_smoke.sh` starts `nav_node.py` and verifies every navigation topic plus a
`/cmd_vel`-driven odometry update. `nav2_integration.sh` additionally launches
Nav2 and verifies a `NavigateToPose` goal is reached. `nav2_action_smoke.sh`
verifies the bridge's own `/navigate_to_pose` action server (feedback plus a spin
recovery) reaches a goal. `slam_map_smoke.sh` generates an RNE SLAM map and
checks `nav_node.py` republishes it on `/map`.
See `docs/ROS2_NAV2.md`.

For live simulation via Python bindings:
```bash
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
python3 test_sim_control.py
```

## Smoke test (ROS 2 + optional rne_py)

Builds `rne_py`, runs the bridge with live diff-drive simulation, and verifies
`/clock`, `/points`, and `/tf` with `ros2 topic echo`:

```bash
source /opt/ros/jazzy/setup.bash
./adapters/ros2/rne_ros2_bridge/smoke_test.sh
```

From the repo root:

```bash
cargo run -p xtask -- ci-ros2-bridge
```

GitHub Actions runs the same script in `.github/workflows/ros2-bridge.yml`.

Smoke tests set `RNE_ROS2_HOLD_SECS=60` so the bridge stays alive while `ros2 service call`
and topic probes run.
The native `rclrs` node uses the symmetric `ci-ros2` task.

## Rust mapping layer

Message layout helpers also exist in Rust at `../rne_adapter_ros2/`.
The Python node is the supported runtime path when `rclrs` type-support
libraries are not installed.

## Native `rclrs` node

A Rust-native bridge with the same topics lives at `../rne_ros2_node/`.
It uses `rne_ai::DiffDriveSim` directly (no Python bindings) and is built
with `--manifest-path` after sourcing ROS and running `generate_cargo_config.sh`.

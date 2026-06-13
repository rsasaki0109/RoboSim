# ROS 2 Bridge

Python adapter node that publishes RNE simulation outputs to ROS 2 topics.

## Topics

| Topic | Type | Source |
|-------|------|--------|
| `/clock` | `rosgraph_msgs/Clock` | RNE `SimTime` ticks |
| `/points` | `sensor_msgs/PointCloud2` | LiDAR-style XYZ cloud |
| `/tf` | `tf2_msgs/TFMessage` | `world → base_link → lidar` |

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
The native `rclrs` node uses the symmetric `ci-ros2` task.

## Rust mapping layer

Message layout helpers also exist in Rust at `../rne_adapter_ros2/`.
The Python node is the supported runtime path when `rclrs` type-support
libraries are not installed.

## Native `rclrs` node

A Rust-native bridge with the same topics lives at `../rne_ros2_node/`.
It uses `rne_ai::DiffDriveSim` directly (no Python bindings) and is built
with `--manifest-path` after sourcing ROS and running `generate_cargo_config.sh`.

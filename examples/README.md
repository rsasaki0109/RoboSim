# Examples

| Example | Command | Description |
|---------|---------|-------------|
| Hello world | `cargo run -p hello_world --example 00_hello_world` | ECS world + transform propagation |
| Falling cube | `cargo run -p falling_cube --example 01_falling_cube` | Rapier gravity + physics sync |
| Diff drive + LiDAR | `cargo run -p diff_drive_lidar --example 01_diff_drive_lidar` | Robot, sensors, DataBus |
| Render clear | `cargo run -p render_clear --example 02_render_clear` | wgpu off-screen clear render |
| URDF import | `cargo run -p urdf_import --example 03_urdf_import` | URDF → Robot/Link/Joint entities |
| Python policy | `.venv/bin/python examples/04_python_policy/run.py` | Python controls diff drive |
| Episode API | `cargo run -p episode_diff_drive --example 05_episode_diff_drive` | Reward, termination, log recording |
| Scene assets | `cargo run -p scene_load --example 06_scene_load` | `.rne.scene.toml` / `.rne.robot.toml` load |
| Render primitives | `cargo run -p render_primitives --example 07_render_primitives` | wgpu color + depth pass |
| Python episode | `.venv/bin/python examples/05_episode_diff_drive/run.py` | Episode API from Python |
| ROS 2 bridge | `adapters/ros2/rne_ros2_bridge/smoke_test.sh` | Publishes `/clock`, `/points`, `/tf` |

All Rust examples run headless and are suitable for CI.

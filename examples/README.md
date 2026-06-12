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
| Scene episode | `cargo run -p scene_episode --example 08_scene_episode` | Scene asset → episode → optional render |
| URDF mesh render | `cargo run -p urdf_mesh_render --example 09_urdf_mesh_render` | URDF mesh STL load + wgpu draw |
| Vectorized episode | `cargo run -p vectorized_episode --example 10_vectorized_episode` | Parallel envs + domain randomization |
| Agent policy | `cargo run -p agent_policy --example 11_agent_policy` | Agent entity + attachable policy (episode-owned world) |
| Shared-world agent | `cargo run -p shared_world_agent --example 12_shared_world_agent` | Agent entity in simulation ECS world |
| Multi-robot agent | `cargo run -p multi_robot_agent --example 13_multi_robot_agent` | Two agents, two robots, one shared world |
| Interactive viewer | `cargo run -p interactive_viewer --example 14_interactive_viewer` | winit window, WASD teleop, orbit camera (`--smoke` for headless) |
| Python episode | `.venv/bin/python examples/05_episode_diff_drive/run.py` | Episode API from Python |
| ROS 2 bridge | `adapters/ros2/rne_ros2_bridge/smoke_test.sh` | Publishes `/clock`, `/points`, `/tf` (Python) |
| ROS 2 bridge CI | `cargo run -p xtask -- ci-ros2-bridge` | Python bridge smoke + topic check (requires ROS 2 Jazzy/Humble) |
| ROS 2 native | `cargo run -p xtask -- ci-ros2` | `rclrs` node smoke test (requires ROS 2 Jazzy/Humble) |

All Rust examples run headless and are suitable for CI, except the interactive viewer which requires a display (use `--smoke` for headless verification).

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
| Interactive viewer | `cargo run -p interactive_viewer --example 14_interactive_viewer` | Scene asset load, URDF mesh visuals, WASD teleop, orbit camera, asset-defined LiDAR overlay (`L`), live hot reload; `--manipulator` / `--manipulator-mobile` for arm teleop (Q/E/Z/X) |
| Asset hot reload | `cargo run -p asset_hot_reload --example 15_asset_hot_reload -- --smoke` | Validate scene deps and reload on file change |
| Goal-conditioned agent | `cargo run -p goal_conditioned_agent --example 16_goal_conditioned_agent` | Goal-seeking policy, curriculum, shared-world goal agent |
| Multi-robot collision | `cargo run -p multi_robot_collision --example 17_multi_robot_collision` | Head-on contact detection from scene asset and built-in scenario |
| LiDAR render | `cargo run -p lidar_render --example 19_lidar_render` | LiDAR hits as sphere markers in wgpu scene |
| Mobile manipulator arm | `cargo run -p mobile_manipulator_arm --example 20_mobile_manipulator_arm -- --smoke` | URDF 2-DOF arm articulation under Rapier (Phase A) |
| Mobile manipulator reach | `cargo run -p mobile_manipulator_reach --example 21_mobile_manipulator_reach -- --smoke` | `MobileManipulatorSim` open-loop reach + DataBus joint state |
| 3D mobile manipulator hero | `cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero` | Steps the real lift pick-place simulation and renders README GIF/PNG media via wgpu |
| Quadruped standing | `cargo run -p quadruped_stand --example 36_quadruped_stand` | Headless 12-DoF URDF standing controller with four-foot contact impulses |
| Humanoid standing | `cargo run -p humanoid_stand --example 37_humanoid_stand` | Headless 12-DoF humanoid balance smoke with left/right foot loads |
| Unitree Go2 GIF | `cargo run -p unitree_go2_gif --example 38_unitree_go2_gif` | Loads the official Go2 URDF, steps 12 force-limited joints, and renders README GIF/PNG media |
| Unitree G1 GIF | `cargo run -p unitree_g1_gif --example 39_unitree_g1_gif` | Loads the official G1 23-DoF URDF, drives all articulated joints, and renders README GIF/PNG media |
| Mobile manipulator report GIF | `cargo run -p xtask -- house-gif-demo` | Dependency-free 2D report-artifact smoke; use `python examples/27_mobile_manipulator_rl/house_gif_demo.py --out-dir house_mobile_manipulator_demo` to keep CSV, GIF, metadata JSON, and HTML preview |
| Asset CLI | `cargo run -p rne_asset_cli -- validate assets/scenes/episode_diff_drive.rne.scene.toml --spawn` | Validate, inspect, watch asset files |
| Python episode | `.venv/bin/python examples/05_episode_diff_drive/run.py` | Episode API from Python |
| ROS 2 bridge | `adapters/ros2/rne_ros2_bridge/smoke_test.sh` | Publishes `/clock`, `/points`, `/tf` (Python) |
| ROS 2 bridge CI | `cargo run -p xtask -- ci-ros2-bridge` | Python bridge smoke + topic check (requires ROS 2 Jazzy/Humble) |
| ROS 2 native | `cargo run -p xtask -- ci-ros2` | `rclrs` node smoke test (requires ROS 2 Jazzy/Humble) |

All Rust examples run headless and are suitable for CI, except the interactive viewer which requires a display (use `--smoke` for headless verification).

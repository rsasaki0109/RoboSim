# Changelog

All notable changes to Robot Native Engine are documented in this file.

## [0.3.0] - 2026-06-12

### Added

- **Shared-world agents** (`12_shared_world_agent`): agent entities live in the simulation ECS world and drive diff-drive robots in-place
- **Multi-robot simulation** (`13_multi_robot_agent`): multiple robots in one `DiffDriveSim`, batched stepping, per-robot policies
- **Richer observations** (`DiffDriveObservation`): base yaw, wheel velocities, optional goal-relative `goal_delta_x_m`; `AgentGoal` component
- **Interactive viewer** (`14_interactive_viewer`, `rne_render_wgpu/viewer`): winit + wgpu window, WASD teleop, orbit camera (`--smoke` for headless CI)
- **Asset pipeline** (`15_asset_hot_reload`, `rne-asset`): hot reload via dependency mtime tracking, validate / inspect / watch CLI, `xtask asset`
- **ROS 2 Python bridge CI**: `ros2-bridge.yml`, `xtask ci-ros2-bridge`, enhanced smoke test with `rne_py` build and topic checks
- **CI**: repo asset validation and spawn smoke in core `xtask ci`

### Changed

- Python ROS 2 bridge smoke aligned with native node (300 steps, `MIN_FORWARD_X_M = 0.8`)
- `rne_py` bindings expose extended diff-drive observation fields

### Notes

- Interactive viewer requires a display; use `--smoke` or `RNE_SKIP_GPU` in headless environments
- Asset hot reload tracks scene, robot, and URDF dependency files by modification time

## [0.2.0] - 2026-06-13

### Added

- **AI / episodes** (`rne_ai`): reward, termination, log recording, scene-backed episodes
- **Domain randomization** and **vectorized envs** (`VectorizedDiffDriveEnv`, example `10_vectorized_episode`)
- **Agent Entity** with attachable policies (`11_agent_policy`)
- **Assets** (`rne_assets`): `.rne.scene.toml` / `.rne.robot.toml` loaders (example `06_scene_load`)
- **Rendering**: primitive color + depth pass (`07_render_primitives`), URDF STL mesh draw (`09_urdf_mesh_render`)
- **Robot**: URDF → collider/visual auto attach; **Rapier joint-driven** diff-drive wheels (`DiffDriveDriveMode::JointDriven`)
- **Integration**: end-to-end scene → episode → optional render (`08_scene_episode`)
- **ROS 2**: native `rclrs` node (`adapters/ros2/rne_ros2_node`); optional CI via `xtask ci-ros2` and GitHub Actions
- **CI**: GitHub Actions workflow for core workspace (`ci.yml`)
- Examples `05`–`11` and expanded determinism coverage for joint-driven physics

### Changed

- Default diff-drive simulation uses joint-driven Rapier wheels (scene assets still use kinematic mode)
- README and roadmap refreshed for v0.2 feature set

### Notes

- Core CI remains ROS-free: `cargo run -p xtask -- ci`
- Native ROS node still builds outside the workspace with `--manifest-path` and patched message crates
- Python bridge unchanged in `adapters/ros2/rne_ros2_bridge/`

## [0.1.0] - 2026-06-13

### Added

- Core crates: `rne_math`, `rne_core`, `rne_ecs`, `rne_world`
- Physics: `rne_physics`, `rne_physics_rapier` with determinism hash tests
- Robot framework: diff-drive spawn, actuator commands, kinematics
- Sensors and DataBus: IMU, LiDAR, wheel encoder, camera, `InMemoryDataBus`
- Logging: JSONL record/replay for actuator commands
- Rendering: `rne_render`, `rne_render_wgpu`, headless camera path
- Python bindings: `rne_py` with diff-drive policy example
- Adapters: URDF import, ROS 2 message mapping, Python ROS 2 bridge node
- Examples: hello world, falling cube, diff drive + LiDAR, render clear, URDF import
- Docs: architecture overview under `docs/architecture/`
- CI: `cargo run -p xtask -- ci` with dependency boundary lint

### Notes

- ROS 2 runtime publishing uses the Python bridge in `adapters/ros2/rne_ros2_bridge/`
- Native `rclrs` nodes require additional `ros2-rust` type-support packages

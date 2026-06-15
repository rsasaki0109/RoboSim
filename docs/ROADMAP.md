# Roadmap

## v0.6.1 (released)

Shipped 2026-06-12. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| AI | `MobileManipulatorSim::from_scene_path`, scene path helpers |
| Assets | `mm_minimal` / `mm_minimal_grasp` scenes, parallel-jaw gripper URDF |
| Examples | Example 22 grasp contact smoke |
| Adapters | ROS mobile manipulator loads `mm_mobile` scene by default |

## v0.6.0 (released)

Shipped 2026-06-12. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Physics | URDF → Rapier articulation (`attach_urdf_articulation`) |
| Assets | `mm_minimal` / `mm_mobile` URDF + scene assets |
| AI | `MobileManipulatorSim`, reach example, DataBus `JointState` |
| Rendering | Arm teleop in interactive viewer (`--manipulator`, `--manipulator-mobile`) |
| ROS 2 | `mobile_manipulator` mode: `/joint_states`, `/cmd_vel`, `/arm_joint_velocity` |

## v0.5.0 (released)

Shipped 2026-06-12. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Rendering | LiDAR hit visualization in wgpu and interactive viewer (`19_lidar_render`, `append_lidar_overlay`) |
| Rendering | Normal-based Lambert lighting in wgpu (`rne_render_wgpu`) |
| Sensors | Scene-defined LiDAR mounts and obstacles (`[lidar]`, `[[obstacles]]`) |
| ROS 2 | Native `/points` and `/scan` from simulation DataBus (`rne_ros2_node`) |

## v0.6 goal: mobile manipulator

Primary development target for v0.6 (shipped in v0.6.0). See [architecture/006_mobile_manipulator.md](architecture/006_mobile_manipulator.md).

| Phase | Area | Deliverable | Status |
|-------|------|-------------|--------|
| A | Physics | URDF → Rapier revolute chain, `JointMotor` on arm | Done (`attach_urdf_articulation`, parent-aware sync) |
| A | Assets | Minimal mobile-manipulator URDF + `.rne.scene.toml` | Done (`mm_minimal`, `mm_mobile` scene) |
| B | AI | `MobileManipulatorSim`, joint/EE observations, reach example | Done (`MobileManipulatorSim`, `21_mobile_manipulator_reach`) |
| B | Rendering | Arm teleop in interactive viewer | Done (`--manipulator`, `--manipulator-mobile`) |
| C | Manipulation | Gripper, wrist camera, pick/transport episodes | In progress (gripper + grasp smoke in v0.6.1) |
| D | ROS 2 | `/joint_states`, base + arm command topics | Done (`mm_mobile` mode: 4 joints, `/cmd_vel`, `/arm_joint_velocity`) |

### v0.6 candidates (detail)

| Area | Idea |
|------|------|
| Physics | Wire URDF joints to Rapier impulse joints (revolute + prismatic) |
| Robot | `[mobile_manipulator]` section in `.rne.robot.toml` |
| AI | `MobileManipulatorSim` with base velocity + arm joint commands |
| Sensors | `JointState` and end-effector pose on DataBus |
| Examples | `20_mobile_manipulator_reach` headless smoke |
| Rendering | Joint sliders + base teleop in `14_interactive_viewer` |
| Manipulation | Tabletop object spawn, reach/grasp termination (v0.7 stretch) |
| ROS 2 | Publish `/joint_states`; subscribe to arm trajectory or joint targets |

## v0.5 candidates

| Area | Idea |
|------|------|
| Rendering | LiDAR hit visualization in wgpu and interactive viewer | Done (`19_lidar_render`, `append_lidar_overlay`, `L` toggle) |
| Rendering | Simple normal-based lighting in wgpu fragment shader | Done (Lambert + ambient in `rne_render_wgpu`) |
| Sensors | Scene-defined LiDAR mounts (not demo-only wall spawn) | Done (`[lidar]` robot asset, `[[obstacles]]` scene asset) |
| ROS 2 | Publish `/scan` from native node when LiDAR is present | Done (`/points` + `/scan` from DataBus, `RNE_ROS2_SCENE_PATH`) |

## v0.4.0 (released)

Shipped 2026-06-12. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| AI | Goal-conditioned policies and curriculum (`16_goal_conditioned_agent`) |
| Physics | Multi-robot collision and interaction (`17_multi_robot_collision`) |
| ROS 2 | Services, actions, and parameter parity with native node |
| Rendering | URDF world transforms, orbit camera, multi-draw wgpu fix, sim-captured README hero |
| DevEx | `rne_urdf_import` promoted to core crate; CI boundary lint passes |

## v0.3.0 (released)

Shipped 2026-06-12. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| AI | Multi-robot episodes, richer observation spaces (`13_multi_robot_agent`) |
| Physics | Shared-world agents (agent ECS ↔ sim world) (`12_shared_world_agent`) |
| Rendering | Interactive viewer / camera teleop (`14_interactive_viewer`) |
| Assets | Hot reload, asset pipeline CLI (`15_asset_hot_reload`, `rne-asset`, `xtask asset`) |
| ROS 2 | Python bridge CI, topic parity with native node (`ros2-bridge.yml`, `xtask ci-ros2-bridge`) |

## v0.2.0 (released)

Shipped 2026-06-13. See [CHANGELOG.md](../CHANGELOG.md).

### Completed in v0.2 scope

| Area | Feature |
|------|---------|
| AI | Episode API, reward/termination (`rne_ai`, `05_episode_diff_drive`) |
| Robot | URDF → collider/visual auto attach (`rne_urdf_import`) |
| Assets | `.rne.scene.toml` / `.rne.robot.toml` (`rne_assets`, `06_scene_load`) |
| Rendering | Mesh rendering, depth pass (`07_render_primitives`) |
| ROS 2 | Native `rclrs` node (`rne_ros2_node`) |

### Also shipped in v0.2.0

| Area | Feature |
|------|---------|
| Integration | End-to-end scene + episode (`08_scene_episode`) |
| Rendering | URDF mesh load + wgpu draw (`09_urdf_mesh_render`) |
| AI | Domain randomization, vectorized envs (`10_vectorized_episode`) |
| Robot | Rapier joint-driven wheels (`DiffDriveDriveMode::JointDriven`) |
| Agent | Agent Entity + policy attach (`11_agent_policy`) |
| ROS 2 | Optional CI for `rne_ros2_node` (`.github/workflows/ros2-node.yml`, `xtask ci-ros2`) |
| CI | GitHub Actions core workspace job (`ci.yml`) |

## v0.1.0 (initial release)

- Core ECS + world + math
- Rapier physics + determinism tests
- Diff-drive robot + sensors + DataBus
- Render skeleton + Python bindings
- URDF import + ROS 2 Python bridge

## v0.4 candidates

| Area | Idea |
|------|------|
| AI | Goal-conditioned policies, curriculum / multi-task episodes | Done (`GoalSeekingPolicy`, `GoalCurriculum`, `16_goal_conditioned_agent`) |
| Rendering | Viewer + scene assets integration, URDF mesh in interactive mode | Done (`14_interactive_viewer`, `[visuals]` robot assets) |
| Physics | Multi-robot collision and interaction scenarios | Done (`multi_robot` helpers, `17_multi_robot_collision`) |
| ROS 2 | Services, actions, and parameter parity with native node | Done (`simulation_interfaces`, `wheel_velocity_rad_s`) |
| DevEx | Live asset reload wired into interactive viewer | Done (`14_interactive_viewer`) |

## Native ROS 2 (`rclrs`)

Two runtime paths are available:

- **Python** (`adapters/ros2/rne_ros2_bridge/`): `rclpy` node, optional `rne_py` bindings
- **Rust** (`adapters/ros2/rne_ros2_node/`): native `rclrs` node with headless `rne_ai` sim

On Jazzy:

```bash
sudo apt install ros-jazzy-rosidl-generator-rs ros-jazzy-test-msgs
source /opt/ros/jazzy/setup.bash
cargo run -p xtask -- ci-ros2
cargo run -p xtask -- ci-ros2-bridge
```

The native node is built with `--manifest-path` and is not part of the core workspace CI.

## Release checklist

After merging release changes:

```bash
cargo run -p xtask -- ci
git tag -a v0.6.0 -m "Robot Native Engine v0.6.0"
git push origin main --tags
gh release create v0.6.0 --title "v0.6.0" --notes-file CHANGELOG.md
```

Adjust the `gh release create` notes to the `[0.6.0]` section only if you prefer a shorter GitHub release body.

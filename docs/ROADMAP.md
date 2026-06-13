# Roadmap

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
| AI | Goal-conditioned policies, curriculum / multi-task episodes |
| Rendering | Viewer + scene assets integration, URDF mesh in interactive mode | Done (`14_interactive_viewer`, `[visuals]` robot assets) |
| Physics | Multi-robot collision and interaction scenarios |
| ROS 2 | Services, actions, and parameter parity with native node |
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
git tag -a v0.3.0 -m "Robot Native Engine v0.3.0"
git push origin main --tags
gh release create v0.3.0 --title "v0.3.0" --notes-file CHANGELOG.md
```

Adjust the `gh release create` notes to the `[0.3.0]` section only if you prefer a shorter GitHub release body.

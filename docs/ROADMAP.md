# Roadmap

## v0.13 goal: mobile clutter place E2E

Primary development target for v0.13. Closes the v0.12 stretch goal: full mobile
navigate → grasp → place on `mm_mobile_clutter` (example 34,
`IkMobileClutterPickPlacePolicy`). Fixed-base clutter (example 33) is the template;
the pinned `mobile_clutter_pick_place_center` config already exists for RL benches.

| Phase | Area | Deliverable | Status |
|-------|------|-------------|--------|
| A | Manipulation | Tune `IkMobileClutterPickPlacePolicy` carry/place; un-ignore `mobile_clutter_policy_completes_place` and `mobile_clutter_transport_script_places_cube_a` | Pending |
| B | Examples | Example 34 `--smoke` asserts grasp + place (mirrors example 33) | Pending |
| C | RL | `train_mobile_clutter.py` (CEM + replay) and PPO smoke on `mobile_clutter_pick_place_center`; wire into `xtask ci` | Pending |
| D | Release | README hero / CHANGELOG / ROADMAP update, ship v0.13.0 | Pending |

### v0.13 candidates

| Area | Idea |
|------|------|
| Physics | **mm_minimal settle fix (linux CI)**: the fixed-base arm has no position-hold motors (`configure_mobile_arm_motors` early-returns without a mobile base) and base/upper-arm collider interpenetration keeps injecting contact energy, so the idle pose is a sustained oscillation — chaotic, hence platform-divergent. Fix mirrors the merged mm_mobile work (interpenetration removal + spring-damper hold + anti-windup lead), then re-derive the affected fixed-base/lift open-loop test scripts (~15-17 tests across mm_minimal and mm_lift; do mm_minimal first, mm_lift second). Un-gate the 7 `cfg_attr(target_os = "linux", ignore)` rne_ai tests when done |
| Physics | Wire URDF prismatic joints to Rapier (carried-over architecture gap) |
| Perception | Wrist-camera grasp target estimation (visuomotor pick) — natural v0.14 follow-up |
| Scene diversity | Domain randomization + curriculum over clutter layouts |

## v0.11.0 goal: wrist RGB-D, clutter RL bench, scene diversity

Primary development target for v0.11. Shipped 2026-07-03. See [CHANGELOG.md](../CHANGELOG.md).

| Phase | Area | Deliverable | Status |
|-------|------|-------------|--------|
| A | Perception | Wrist RGB-D on DataBus + depth in observations; `VisuomotorReachPolicy` | Done (`ImageDepth`, scene-aware wrist sampling, depth obs fields) |
| B | RL | CEM pick-and-place + visuomotor reach bench; SB3 PPO smoke in CI | Done (`train_place.py`, `train_visuomotor.py`, `train_ppo.py` smokes) |
| C | Scene diversity | Clutter pick + mobile navigate-and-place episodes | Done (`clutter_pick_place`, `mobile_clutter_pick_place`, clutter scenes) |

### v0.12 candidates

| Area | Idea |
|------|------|
| RL | Converging clutter pick-and-place on SB3 PPO / CEM + reproducible bench + replay |
| Manipulation | Analytic IK for `mm_minimal` SCARA (mirrors `MmLiftKinematics`) |
| Scene diversity | Full mobile navigate → grasp → place E2E on clutter scenes | In progress (`IkMobileClutterPickPlacePolicy`, example 34) |
| Physics | Wire URDF prismatic joints to Rapier (architecture gap) |

## v0.12 goal: close the clutter pick-and-place loop

Shipped 2026-07-03. See [CHANGELOG.md](../CHANGELOG.md). Fixed-base clutter place E2E and RL
bench are complete; mobile navigate-and-place remains a stretch goal (example 34, policy skeleton).

| Phase | Area | Deliverable | Status |
|-------|------|-------------|--------|
| A | Kinematics | `MmMinimalKinematics` — analytic FK/IK for the `mm_minimal` SCARA chain | Done |
| B | Scene / E2E | `IkClutterPickPlacePolicy` + example `33_clutter_pick_place_e2e` | Done |
| C | RL | `train_clutter.py` + `train_clutter_ppo.py` converging bench + replay | Done |

## v0.12.0 (released)

Shipped 2026-07-03. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Kinematics | `MmMinimalKinematics` analytic FK/IK for `mm_minimal` SCARA |
| Manipulation | `IkClutterPickPlacePolicy`; example 33 fixed-base clutter place E2E |
| RL | `train_clutter.py` (CEM + replay); `train_clutter_ppo.py` (SB3 smoke) |
| Scenes | `clutter_place_center`; `mm_minimal_clutter` ground place target |
| Stretch | `IkMobileClutterPickPlacePolicy`, example 34 (mobile place tuning) |

## v0.11.0 goal: wrist RGB-D, clutter RL bench, scene diversity

Shipped 2026-07-03. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Perception | `ImageDepth` on DataBus; wrist depth obs fields; `VisuomotorReachPolicy` |
| RL | `train_place.py`, `train_visuomotor.py`, `train_ppo.py` smokes; pinned CPU torch in CI |
| Scenes | `mm_minimal_clutter` / `mm_mobile_clutter`; clutter + mobile navigate-place episodes |
| AI | Snapshot v2 with v1 restore; pre-grasp approach reward on Place tasks |

## v0.10.0 goal: arm IK & trajectory following

Primary development target for v0.10. Closes the one capability the mobile-manipulator
architecture doc still marks **Partial** ([architecture/006_mobile_manipulator.md](architecture/006_mobile_manipulator.md)):
analytic IK + joint-space trajectory following now exist; `IkLiftPickPlacePolicy` drives carry
via IK joint targets. `LiftPickPlacePolicy` remains as a scripted regression baseline.

| Phase | Area | Deliverable | Status |
|-------|------|-------------|--------|
| A | Kinematics | Analytic IK helper for the `mm_lift` lift+arm chain (pure, deterministic, seed-free), rustdoc + unit test | Done (`MmLiftKinematics`, `fk_ik_roundtrip`, `fk_matches_sim_at_idle`) |
| B | Control | Joint-space trajectory following: interpolate the IK solution into position-motor targets (`_rad` / `_m` / `_s` units), crate-approved tolerances | Done (`JointTrajectory`, `joint_tracking_action`, `ik_reaches_arbitrary_target`) |
| C | AI | Replace `LiftPickPlacePolicy` internals with IK-solved targets; existing determinism/golden tests guard regression | Done (`IkLiftPickPlacePolicy`; `LiftPickPlacePolicy` retained for regression) |
| D | Adapters | ROS 2 `trajectory_msgs/JointTrajectory` (subset) subscribe; expose IK / trajectory API via `rne_py` | Done (`step_hold_lift_joints` in `rne_py`; `/arm_joint_trajectory` 3-DOF on `RNE_ROS2_MODE=mm_lift`) |

**Why IK first:** it is the base every other theme rides on — perception (vision → target pose →
IK), RL (act in EE space), and scene diversity (reach an arbitrary object position) all get simpler
once IK exists.

### v0.10 candidates (detail)

| Area | Idea |
|------|------|
| Kinematics | Analytic IK for the column-lift + revolute arm; keep it out of core as a pure helper |
| Control | Interpolated joint-space trajectories driving the existing position motors |
| AI | `LiftPickPlacePolicy` re-expressed as IK-to-target instead of a fixed step sequence |
| ROS 2 | Subscribe `trajectory_msgs/JointTrajectory` (subset), map to arm joint targets |
| Python | `rne_py` IK / trajectory bindings alongside the existing episode API |

## v0.10.0 (released)

Shipped 2026-07-02. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Kinematics | `MmLiftKinematics` analytic FK/IK for `mm_lift` (sim shoulder sign corrected) |
| Control | Direct joint targets (`hold_lift_joints`), `JointTrajectory`, `joint_tracking_action` |
| AI | `ik_reaches_arbitrary_target`; `LiftPickPlacePolicy::kinematics()` helper |
| Python | `rne_py` `MmLiftKinematics`, `step_hold_lift_joints`, `MobileManipulatorSim("mm_lift")` |

## v0.9.0 (released)

Shipped 2026-07-02. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Manipulation | `mm_lift` column + sliding carriage; vertical lift position motor; top-down claw; full 3D pick → lift → carry → place |
| Control | Position (spring-damper) arm joints; per-motor `max_force`; per-world solver iterations for stiff chains |
| AI | `LiftPickPlacePolicy`; `lift_pick_place` episode; Place observation goal offset; reach curriculum + goal-conditioned reach |
| ROS 2 | `/lift_command` (`std_msgs/Float64`) drives the vertical lift |
| Rendering | Interactive viewer `--manipulator-lift` profile (`R` / `F` lift, arm + claw teleop) |
| Docs | README 3D pick-and-place showcase + `32_lift_pick_place_hero` sim-captured hero |

## v0.8.0 (released)

Shipped 2026-06-16. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Physics | Prismatic + fixed (weld) joints; deterministic backend iteration |
| Manipulation | Contact-triggered weld grasp, `Place` task, example 26 pick-and-place |
| AI / RL | `reach` task, CEM training loop (example 27), vectorized env (example 28), `rne_py` bindings + SB3 PPO integration |
| ROS 2 | Arm velocity / position / trajectory control + gripper command + `ee_link` TF |
| Assets | `mm_mobile` drive-wheel fix (was spinning in place) |

## v0.7.0 (released)

Shipped 2026-06-12. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| AI | `MobileManipulatorEpisode` (reach / grasp / transport / inspect tasks + rewards) |
| Sensors | `[wrist_camera]` on `mm_mobile`; wrist camera DataBus in sim |
| Rendering | Viewer wrist camera PiP (`P` toggle) on `--manipulator` profiles |
| ROS 2 | `/camera/image_raw` from wrist camera in `mobile_manipulator` mode |
| Examples | Example 25 episode smoke (inspect + transport termination) |

## v0.6.2 (released)

Shipped 2026-06-12. See [CHANGELOG.md](../CHANGELOG.md).

| Area | Feature |
|------|---------|
| Manipulation | Dynamic scene obstacles; transport helpers + `mm_minimal_transport` scene |
| Sensors | Wrist camera DataBus (`ImageRgb8`); example 24 wrist cam smoke |
| Examples | Example 23 transport smoke (finger contact + cube displacement) |
| Physics | Zero-velocity ECS→Rapier sync on spawn for repeatable initial EE pose |

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
| C | Manipulation | Gripper, wrist camera, pick/transport/place episodes | Done (contact-triggered weld grasp, `Place` task, example 26 pick-and-place) |
| D | ROS 2 | `/joint_states`, base + arm command topics | Done (`mm_mobile` mode: 4 joints, `/cmd_vel`, `/arm_joint_velocity`, `/gripper_command`, `ee_link` TF) |

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

| Area | Idea | Status |
|------|------|--------|
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

| Area | Idea | Status |
|------|------|--------|
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

After merging release changes (replace `0.9.0` with the version you are shipping):

```bash
cargo run -p xtask -- ci
git tag -a v0.9.0 -m "Robot Native Engine v0.9.0"
git push origin main --tags
gh release create v0.9.0 --title "v0.9.0" --notes-file CHANGELOG.md
```

Adjust the `gh release create` notes to the `[0.9.0]` section only if you prefer a shorter GitHub release body.

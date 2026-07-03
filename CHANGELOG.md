# Changelog

All notable changes to Robot Native Engine are documented in this file.

## [Unreleased]

## [0.12.0] - 2026-07-03

### Added

- **`MmMinimalKinematics`**: analytic FK/IK for the fixed-base `mm_minimal` SCARA arm
  (`mm_minimal_kinematics.rs`), with roundtrip tests, sim XZ parity, and reachability helper.
- **`IkClutterPickPlacePolicy`**: IK approach + tuned fixed-velocity carry toward
  `mm_minimal_clutter_place_target` (fixed-base ground target off the table edge).
  Example 33 `--smoke` asserts grasp and place on `clutter_cube_b`; 15 clutter unit
  tests cover grasp, carry tuning, and full scripted place E2E.
- **`train_clutter.py`**: CEM smoke on the `clutter_place_center` episode (approach reward +
  place progress, grasp assertion, deterministic replay on the best candidate).
- **`train_clutter_ppo.py`**: SB3 PPO integration smoke on `clutter_place_center`.
- **`clutter_pick_place_center`**: pinned center-cube config for reproducible clutter RL benches.
- **`IkMobileClutterPickPlacePolicy`**: diff-drive approach + IK arm pick/place for
  `mm_mobile_clutter` (example 34; full place E2E still tuning).
- **`mm_mobile_clutter_place_target`**: shared ground place target helper for mobile clutter episodes.
- **`mobile_clutter_pick_place_center`**: pinned `clutter_cube_a` config for mobile RL benches.
- **`xtask ci`**: runs `train_clutter.py` / `train_clutter_ppo.py --smoke` alongside existing RL smokes.

## [0.11.0] - 2026-07-03

### Added

- **`ImageDepth` DataBus payload** and paired wrist RGB-D sampling (`sample_camera_rgbd`,
  scene-aware headless depth via `scene_depth_probe`).
- **Wrist depth observations**: `wrist_depth_center_m`, `wrist_depth_min_m`, and
  `target_object_index` on `MobileManipulatorObservation` (Python bindings included).
- **`VisuomotorReachPolicy`**: goal-conditioned reach that scales arm velocity from wrist depth.
- **Clutter pick-and-place episodes**: `clutter_pick_place` and `mobile_clutter_pick_place`
  configs with `mm_minimal_clutter` / `mm_mobile_clutter` scenes; pre-grasp approach reward
  on Place tasks.
- **RL bench scripts**: `train_place.py` (CEM place smoke) and `train_visuomotor.py`
  (depth-conditioned reach smoke).
- **`xtask ci`**: validates clutter scenes and runs `rne_py` RL smokes (`run.py`,
  `train_place.py`, `train_visuomotor.py`, `train_ppo.py`).
- **`IkLiftPickPlacePolicy`**: pick-and-place state machine whose carry swing solves
  [`MmLiftKinematics`] targets and drives shoulder / elbow / lift at a fixed rate toward
  the IK joint solution. Example 31 and the `lift_pick_place` episode test use this
  policy; [`LiftPickPlacePolicy`] remains for scripted regression tests.
- **ROS 2 `mm_lift` mode** (`RNE_ROS2_MODE=mm_lift`): loads the `mm_lift` scene and exposes
  manipulator subscriptions including `/lift_command` and `/arm_joint_trajectory`.
- **3-DOF lift-arm trajectories**: when `lift_joint`, `shoulder_joint`, and `elbow_joint`
  appear in `/arm_joint_trajectory` or `/arm_joint_position`, the bridge drives
  `MobileManipulatorAction::hold_lift_joints` waypoint following.

### Fixed

- **Depth stream id**: `rne_ai` wrist depth uses `rne_sensor::CAMERA_DEPTH_STREAM_OFFSET`
  (single source of truth).
- **Place / reach progress rewards**: potential-based shaping (signed delta) instead of
  clamping progress at zero.
- **Mobile manipulator snapshot v2**: adds `wrist_depth_frame`; schema v1 checkpoints restore
  with `wrist_depth_frame` absent (`#[serde(default)]`).
- **Clutter scenes**: tabletop support in `mm_minimal_clutter` (cubes settle on table, stay clear
  of idle arm sweep); E2E covers gripper contact on all targets, weld grasp of the center cube,
  transport Place script parity, and mobile-base approach.
- **`xtask ci`**: pinned Python deps in `requirements-ci.txt` (CPU-only torch, gymnasium, SB3);
  set `RNE_SKIP_RL_SMOKES` to skip RL smokes locally.
- **`rne_py` checkpoint tests**: tolerate JSON float roundtrip on episode rewards.
- **RL smokes**: deterministic `random.seed(0)`; CEM smokes check best-iteration improvement
  (`max(history) > history[0]`).

## [0.10.0] - 2026-07-02

### Added

- **`MmLiftKinematics`**: analytic forward / inverse kinematics for the `mm_lift`
  column + 2R arm chain (pure, deterministic, seed-free). Matches the simulation
  shoulder sign convention. Tests: `fk_ik_roundtrip_for_reachable_targets`,
  `fk_matches_sim_at_idle`, `fk_shoulder_sign_matches_positive_velocity_swing`.
- **Direct lift-arm joint targets**: `MobileManipulatorAction::lift_joint_target`,
  `MobileManipulatorAction::hold_lift_joints()`, and
  `MobileManipulatorSim::set_lift_joint_targets()` drive lift / shoulder / elbow
  position motors to absolute targets (with raised stiffness for direct holds).
- **Joint-space trajectory helpers**: `JointTrajectory`, `joint_tracking_action`,
  and `hold_lift_joint_action` for position-motor tracking. Test
  `ik_reaches_arbitrary_target`.
- **`rne_py` IK bindings**: `MmLiftKinematics`, `MmLiftJointTarget`,
  `MmLiftGripperTarget`, `MobileManipulatorSim(mode="mm_lift")`,
  `step_hold_lift_joints()` on sim and episode, and `lift_position_m` on
  observations.

### Changed

- **`LiftPickPlacePolicy`**: exposes `kinematics()` and `default_place_target()` for
  IK-based controllers; carry swing remains the proven scripted shoulder rate until
  IK carry converges reliably under grasp load.

## [0.9.0] - 2026-07-02

### Added

- **README 3D pick-and-place showcase**: a sim-captured hero still of the `mm_lift` robot
  hoisting a grasped cube, generated by the new `32_lift_pick_place_hero` example, plus an
  updated highlights/feature list and run commands for the pick-and-place.

- **ROS 2 `/lift_command` topic**: the ROS 2 node now subscribes to `std_msgs/Float64` on
  `/lift_command` to drive the vertical lift (positive raises, negative lowers), alongside
  the existing `/cmd_vel`, `/gripper_command`, and arm topics. Verified by the ci-ros2 smoke.

- **`LiftPickPlacePolicy`**: a reusable scripted pick-and-place policy (state machine) for
  the `mm_lift` robot — lower → grasp → lift → swing → settle → lower → release. It implements
  `Policy<MobileManipulatorEpisode>` and is now the single source for the pick-and-place
  trajectory used by example 31 and the episode test (previously duplicated inline).
- **Configurable place location**: `LiftPickPlacePolicy::with_swing_steps` sets how far the
  carry swing rotates the arm, so the cube can be placed at different spots around the column
  (`total_steps()` reports the sequence length). Test `lift_place_swing_controls_drop_location`.

### Changed

- **Place tasks now expose a goal offset in the observation** (`target_d{x,y,z}_m`): before
  grasping it points from the gripper to the object (where to pick), and once grasped it
  points from the object to the place target (where to carry). Previously these were always
  zero for Place tasks, leaving a policy blind; this makes the pick-and-place observation-
  driven. Test `place_observation_points_at_object_then_target`.

### Added

- **Interactive viewer `--manipulator-lift` profile**: the redesigned `mm_lift` robot is now
  viewable/teleoperable in example 14, with `R` / `F` driving the vertical lift. Wired into
  `xtask ci` as a render smoke.

- **Lift pick-and-place episode** (`MobileManipulatorEpisodeConfig::lift_pick_place`): the
  full 3D pick-and-place as a first-class `Episode` (reward + success), on the `mm_lift_pick`
  scene with a place target. Exposed to Python as `MobileManipulatorEpisode("lift_place")`.
  The Python episode `step` now accepts a `lift_velocity_m_s` argument (default `0.0`, so
  existing 5-argument calls are unchanged) to drive the vertical lift. Test
  `lift_pick_place_episode_picks_carries_and_places`.

- **Full 3D pick-and-place** (manipulator-redesign phase 4, final): the `mm_lift` robot now
  performs an end-to-end pick→lift→carry→place — lower the top-down claw over a ground cube,
  grasp it, lift it, swing the arm to a new spot, lower it, and open to release. Test
  `lift_picks_carries_and_places_cube` and example
  `31_mobile_manipulator_lift_pick_place` (carries the cube ~1.1 m and releases it; wired
  into `xtask ci`). This completes the four-phase manipulator redesign (column base →
  controllable arm → top-down claw → pick-and-place).

- **Real 3D pick** (manipulator-redesign phase 3): the `mm_lift` gripper is redesigned as a
  **top-down claw** (two fingers hang down to straddle an object) so it can lower over a cube
  on the ground, grasp it (contact-triggered weld), and the lift raises it off the ground —
  the previous side-grip could not pick a ground object because its body collided with it.
  New `mm_lift_pick` scene + `mm_lift_pick_scene_path()` and test
  `lift_picks_cube_off_ground_and_raises_it`.

- **Per-motor force override** (`JointMotor.max_force`, default `0.0` = use the
  per-joint-type cap): a positive value overrides the cap for that motor, e.g. a heavy
  arm joint that needs more torque to track its target.

### Changed

- **Lift robot arm is now controllable** (manipulator-redesign phase 2): the arm revolute
  joints are position (spring-damper) motors with a raised torque cap, so the heavy arm
  moves to a commanded angle and *holds* it — a plain velocity motor was too weak to move
  or hold it. Fixed a geometry bug where the upper arm overlapped the carriage and jammed
  the shoulder; the arm now also settles perfectly straight. New test
  `lift_arm_tracks_and_holds_commanded_pose`.

- **Lift robot can now lower its gripper to the ground** (manipulator-redesign phase 1):
  `mm_lift` is rebuilt on a tall fixed **column** with the arm hanging from a sliding
  carriage, so the lift lowers the gripper from rest (~0.81 m) down to near ground
  (~0.26 m) and raises it to carry — the previous box base let the lift only go up. The
  arm also settles much straighter. New test `lift_lowers_gripper_toward_ground`; existing
  lift tests/smoke unchanged in intent.

### Added

- **Per-world solver iterations** (`PhysicsWorldDesc.solver_iterations`, default `0` =
  Rapier's default): a higher count stabilizes stiff articulated chains. The `mm_lift`
  robot's world uses 16 iterations so its tall lift+arm chain holds its pose instead of
  swinging chaotically (it was unstable at the default); other robots are unchanged.
  Covered by a new idle-pose-hold test.

- **Vertical lift (`mm_lift` robot)**: a fixed-base arm with a prismatic "torso" lift
  between the base and shoulder, so the whole SCARA arm can be raised and lowered.
  `MobileManipulatorSim::new_mm_lift()` loads it; `MobileManipulatorAction.lift_velocity_m_s`
  drives the lift (other robots ignore it). The lift is a **position (spring-damper) motor**,
  so it holds the ~6 kg arm against gravity at a commanded height without drift — vertical
  lifting was previously blocked by the velocity-only motor. Covered by a unit test
  (controllable, reversible vertical motion) and a replay-determinism test.
- **Example 30 lift smoke**: `30_mobile_manipulator_lift` raises the `mm_lift` arm with
  the vertical lift and checks the end-effector rises (wired into `xtask ci`)
- **Joint position motors**: `JointMotor` gains `stiffness` + `target_position` fields
  (both default `0.0`, so existing velocity motors are unchanged). A positive stiffness
  turns a joint into a spring-damper that holds a position target under load.
- **Tunable motor gain**: `JointMotor.gain` (default `1.0`) scales the velocity-tracking
  damping factor instead of the previously hardcoded `1.0`, letting a joint track its target
  more stiffly under load. Prismatic motors also get a higher force cap (150 N vs the 50 N
  revolute cap) so a lift can hold a multi-link arm.
- **Reach curriculum** (`MobileManipulatorEpisodeConfig::reach_curriculum` + `ReachCurriculum`):
  an easy→hard curriculum that widens the goal-conditioned reach target region as the
  policy accumulates successes; exposed to Python as `MobileManipulatorEpisode("reach_curriculum")`
  with a `curriculum_stage` getter
- **Example 29 curriculum smoke**: a goal-conditioned policy advances the reach curriculum
  to its final stage (wired into `xtask ci`)
- **Determinism test** for the mobile manipulator reach episode (replay world-state hash)
- **Goal-conditioned reach** (`MobileManipulatorEpisodeConfig::reach_randomized`): a fresh
  reachable target is sampled each episode and exposed in the observation as
  `target_d{x,y,z}_m`, so a policy must generalize. Exposed to Python as
  `MobileManipulatorEpisode("reach_random")`; example 27 `train.py` now learns a
  goal-conditioned policy across varied targets, and the gym env includes the goal offset.

## [0.8.0] - 2026-06-16

### Added

- **`MobileManipulatorEpisodeConfig::reach()`** dense-reward reach task (exposed to Python
  as `MobileManipulatorEpisode("reach")`); target placed so it needs active control
- **Example 27 training loop** (`train.py`): Cross-Entropy-Method policy optimization that
  learns the reach task end-to-end with no external deps (mean reward ~2 → ~12)
- **`VectorizedMobileManipulatorEnv`**: batched mobile-manipulator episodes for
  population-based / parallel RL rollouts (parity with `VectorizedDiffDriveEnv`), with
  example 28 evaluating a policy population in lock-step
- **`rne_py.VectorizedMobileManipulatorEnv`**: Python binding for the batched env; the
  example 27 CEM training loop now evaluates each candidate population through it
- **Example 27 `train_ppo.py`**: Stable-Baselines3 PPO integration on the reach gym env
  (the `train.py` CEM loop remains the dependency-free deterministic learning demo)
- **Prismatic joints**: `rne_physics::PrismaticJointDesc` + Rapier linear motor; URDF
  `type="prismatic"` joints now wire into the articulation (`UrdfArticulationAttached.prismatic_joints`)
- **Fixed (weld) joints**: `rne_physics::FixedJointDesc` welds a child to a parent at a
  relative pose; the Rapier backend creates and *removes* the joint as the component is
  inserted/dropped (release)
- **Contact-triggered grasping**: `MobileManipulatorSim` welds a graspable body to the
  end-effector when the gripper closes on it and releases it on open
  (`is_grasping`, `grasped_object`)
- **`MobileManipulatorTask::Place`** and **`MobileManipulatorEpisodeConfig::place()`**:
  pick up a cube, carry it, and set it down at a target location
- **Example 26 pick-and-place smoke**: full grasp → carry → release → settle cycle
- **`rne_py` mobile manipulator bindings**: `MobileManipulatorSim` / `MobileManipulatorEpisode`
  (place / transport / inspect) exposed to Python with `is_grasping`
- **Example 27 RL env**: gymnasium-style `MobileManipulatorPlaceEnv` wrapper + scripted
  smoke (degrades gracefully without `gymnasium` / `numpy`)
- **ROS 2 `/gripper_command`** (`std_msgs/Float64`): drives the gripper in
  `mobile_manipulator` mode (negative closes/grasps, positive opens/releases)
- **ROS 2 `ee_link` TF frame**: end-effector pose published on `/tf` relative to `base_link`
- **ROS 2 `/arm_joint_position`** (`sensor_msgs/JointState`): position-control the arm —
  the node drives `shoulder_joint` / `elbow_joint` toward the commanded positions with a
  clamped P-controller (a velocity command cancels the target)
- **ROS 2 `/arm_joint_trajectory`** (`trajectory_msgs/JointTrajectory`): follow a sequence
  of `shoulder_joint` / `elbow_joint` waypoints, advancing to the next when the current one
  is reached

### Fixed

- **ROS 2 node build**: `sensor_msgs/Image.is_bigendian` type mismatch (`bool` → `u8`)
  that broke `rne_ros2_node` compilation
- **`mm_mobile` drive wheels**: wheel joints were stacked vertically (`xyz="0 ±0.225 0"`)
  so only one wheel touched the ground and the base spun in place; relocated to a proper
  left/right diff-drive layout (`xyz="0 -0.15 ±0.225"`) so the base drives forward
- **URDF fixed joints**: were not wired to a physics joint, so a fixed-joint child link
  silently became a free-falling body; now wired as a rigid `FixedJointDesc` weld
  (recalibrated the affected `mm_minimal` reach/place demo targets)

### Changed

- **Deterministic physics backend iteration**: the Rapier backend now syncs bodies and
  joints (and writes transforms back) in a stable entity order, fixing run-to-run
  nondeterminism (previously flaky `shoulder_motor_moves_forearm`)
- **`xtask ci`**: example 26 pick-and-place smoke

## [0.7.0] - 2026-06-12

### Added

- **Viewer wrist camera PiP** (`P` toggle) on `--manipulator` profiles in example 14
- **ROS `/camera/image_raw`** from wrist camera DataBus in `mobile_manipulator` mode
- **`MobileManipulatorEpisode`** with reach / grasp / transport / inspect tasks and rewards
- **`MobileManipulatorTask`** and **`MobileManipulatorRewardConfig`**
- **Example 25 episode smoke**: inspect + transport termination
- **`body_within_zone_m`** transport helper for drop-zone checks
- **`[wrist_camera]`** on `mm_mobile` robot asset (forearm mount)

### Changed

- **`xtask ci`**: example 25 smoke; viewer smokes for `--manipulator` and `--manipulator-mobile`

## [0.6.2] - 2026-06-12

### Added

- **Dynamic scene obstacles** (`body_type = "dynamic"`) for graspable objects
- **`mm_minimal_transport` scene** and transport helpers (`displacement_m`, `body_moved_at_least_m`)
- **Example 23 transport smoke**: finger contact + cube displacement ≥ 2 cm
- **`[wrist_camera]` robot asset section** mounted on URDF arm links
- **Wrist camera DataBus** (`ImageRgb8`) in `MobileManipulatorSim`
- **Example 24 wrist cam smoke**: publishes 64×48 RGBA8 frames

### Changed

- **Physics init**: zero-velocity ECS→Rapier sync on spawn for repeatable initial EE pose
- **Example 21 smoke**: proportional reach with error-reduction criterion (no multi-attempt retry loop)
- **`xtask ci`**: smokes examples 23 and 24

## [0.6.1] - 2026-06-12

### Added

- **`MobileManipulatorSim::from_scene_path`**: load `mm_minimal` / `mm_mobile` from `.rne.scene.toml`
- **Scene path helpers**: `mm_minimal_scene_path`, `mm_mobile_scene_path`, `mm_minimal_grasp_scene_path`
- **`mm_minimal` scene asset** (`assets/scenes/mm_minimal.rne.scene.toml`)
- **Parallel-jaw gripper** on `mm_minimal` URDF (`left_finger_joint`, `right_finger_joint`)
- **`MobileManipulatorAction::gripper_velocity_rad_s`** and grasp contact helpers (`finger_contacts_named`)
- **`mm_minimal_grasp` scene** with tabletop cube obstacle
- **Example 22 grasp smoke**: finger contact with `grasp_cube` (`--smoke`)

### Changed

- **`new_mm_minimal` / `new_mm_mobile`** delegate to default scene assets
- **Interactive viewer**, **example 21**, and **ROS `mobile_manipulator` mode** load robots via scene paths
- **Viewer teleop**: `C` / `V` gripper close / open on manipulator profiles

## [0.6.0] - 2026-06-12

### Added

- **URDF arm articulation** (`attach_urdf_articulation`): revolute joints + `JointMotor` wired to Rapier
- **Minimal mobile manipulator asset** (`assets/robots/mm_minimal/`) and example `20_mobile_manipulator_arm`
- **`MobileManipulatorSim`**: 2-DOF arm environment with EE/joint observations and DataBus `JointState`
- **Reach example** (`21_mobile_manipulator_reach`): open-loop shoulder motion smoke test
- **`mm_mobile` URDF**: diff-drive base + 2-DOF arm (`MobileManipulatorSim::new_mm_mobile()`)
- **Interactive viewer arm teleop** (`14_interactive_viewer --manipulator`): Q/E/Z/X arm keys and EE HUD
- **ROS 2 `/joint_states`**: wheel joint state published from native `rne_ros2_node` bridge
- **ROS 2 mobile manipulator mode** (`RNE_ROS2_MODE=mobile_manipulator`): 4-joint `/joint_states`, `/cmd_vel`, `/arm_joint_velocity`
- **`mm_mobile` scene asset** (`assets/scenes/mm_mobile.rne.scene.toml`) with URDF robot spawn from `.rne.robot.toml`
- **URDF robot asset spawn** (`rne_assets`): `base_body_type`, `articulation`, and initial pose for `kind = "urdf"`
- **Mobile base drive helpers** (`mm_mobile_twist_to_wheel_velocities`, unified wheel sign in `MobileManipulatorSim`)

### Changed

- **Rapier physics sync** uses composed world transforms for parent/child link hierarchies
- **`xtask ci`**: validates `mm_mobile` / `mm_minimal` assets; smokes examples 20, 21, and viewer `--manipulator-mobile`

## [0.5.0] - 2026-06-12

### Added

- **LiDAR render helpers** (`rne_render::lidar`): sphere markers for ray hits via `RenderScene::append_lidar_points`
- **LiDAR render example** (`19_lidar_render`): diff-drive scan visualized in wgpu
- **Interactive viewer LiDAR overlay** (`14_interactive_viewer`): live hit markers and `L` toggle via `append_lidar_overlay()`
- **`DiffDriveObservation::lidar_points`** populated from DataBus in `rne_ai`
- **Normal-based wgpu lighting**: Lambert diffuse + ambient in the primitive fragment shader using vertex normals
- **Scene-defined LiDAR**: optional `[lidar]` robot section and `[[obstacles]]` in `.rne.scene.toml`
- **ROS 2 native LiDAR**: `rne_ros2_node` publishes DataBus hits on `/points` and `/scan` (`RNE_ROS2_SCENE_PATH`)

### Changed

- **Interactive viewer and ROS bridge** load LiDAR from scene assets instead of a demo-only API

## [0.4.0] - 2026-06-12

### Added

- **Goal-conditioned episodes** (`16_goal_conditioned_agent`): `GoalSeekingPolicy`, `GoalCurriculum`, and multi-task goal sampling
- **Multi-robot collision** (`17_multi_robot_collision`): shared-world contact scenarios and peer-relative observations
- **ROS 2 sim control parity**: `simulation_interfaces` services, `/simulate_steps` action, and `wheel_velocity_rad_s` parameter on both native `rclrs` and Python bridge nodes
- **README hero capture** (`18_readme_hero`, `docs/media/generate-hero.sh`): orbit-rendered PNG/GIF from the real wgpu simulator
- **`world_transform_of()`** for composed URDF / parent-child render transforms

### Changed

- **`rne_urdf_import` moved to `crates/`** so core workspace CI no longer depends on `adapters/ros2/`
- **Rendering**: physics-synced bases use yaw-only rotation; orbit camera helpers live in `rne_render_wgpu::camera` (no winit required)
- **wgpu multi-draw fix**: per-item draw uniforms use dynamic offsets so multi-link URDF scenes render correctly
- **Depth readback** uses `TextureAspect::DepthOnly` for reliable off-screen passes

### Fixed

- URDF mesh scenes no longer disappear when child links carry local rotations
- Interactive viewer and headless examples frame robots with `CameraOrbit` instead of a fixed offset camera

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

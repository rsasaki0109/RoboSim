# Mobile Manipulator Target

RNE v0.6+ treats **mobile manipulators** (mobile base + articulated arm + optional gripper) as the primary robot class to grow beyond diff-drive-only workflows.

## Goal

Simulate, control, and train on robots that **move and manipulate** in the same world:

- holonomic or differential mobile base
- multi-DOF arm (revolute and prismatic joints from URDF)
- end-effector pose / joint-state observations on the DataBus
- pick, reach, and transport episodes for embodied AI

Diff-drive remains supported; mobile manipulator work **extends** the existing robot-native model rather than replacing it.

## Target scenarios (v0.6 → v0.7)

| Scenario | Description | Success signal |
|----------|-------------|----------------|
| **Reach** | Move base + arm so EE touches a tabletop target | EE error &lt; 5 cm in headless CI |
| **Transport** | Grasp or push an object, drive to a drop zone | Object pose delta + contact events (ex. 23 smoke) |
| **Inspect** | Navigate to waypoint, pan wrist camera on fixture | Camera frame + base pose logged |
| **Teleop** | Keyboard/gamepad base + arm in interactive viewer | Live joint commands, stable physics |

Initial reference robot: **minimal URDF mobile manipulator** (diff base + 2–3 link arm + optional parallel gripper), checked in under `assets/robots/`. A larger open model (e.g. Fetch / TIAGo class) can follow once the pipeline is proven.

## What exists today

| Layer | Status |
|-------|--------|
| ECS `Robot` / `Link` / `Joint` / `Actuator` | Ready |
| URDF parse + spawn (revolute, prismatic, fixed) | Ready (ECS graph + visuals/colliders) |
| Rapier revolute joints + `JointMotor` | Ready (`attach_urdf_articulation`, joint-driven diff-drive) |
| Rapier prismatic joints | **Not wired** from URDF |
| URDF → Rapier articulated chain | Ready (`attach_urdf_articulation`, URDF robot assets with `articulation = true`) |
| Diff-drive kinematics + `DiffDriveSim` | Ready |
| Combined base + arm environment | Ready (`MobileManipulatorSim::new_mm_mobile`, `mm_mobile` scene asset) |
| Gripper / contact-rich manipulation | **Partial** (parallel jaw + grasp/transport smoke) |
| Wrist / head camera from scene assets | **Partial** (`[wrist_camera]` on `mm_minimal`, DataBus `ImageRgb8`) |
| Arm trajectory / IK | **Partial** (`MmLiftKinematics`, `IkLiftPickPlacePolicy`, direct joint targets; ROS 3-DOF trajectory on `mm_lift`) |
| ROS `/joint_states`, arm commands | Ready (`mobile_manipulator`: 2-DOF arm + `/arm_joint_trajectory`; `mm_lift`: 3-DOF lift-arm trajectory via `RNE_ROS2_MODE=mm_lift`) |

## Architecture (robot-native)

```
┌─────────────────────────────────────────────────────────┐
│  Agent / teleop / ROS adapter                           │
│    base cmd (twist or wheel vel) + arm joint targets    │
└──────────────────────────┬──────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────┐
│  MobileManipulatorSim (rne_ai)                          │
│    • sync base (diff-drive or omnidirectional)          │
│    • apply arm ActuatorCommands                         │
│    • step physics, sample sensors                       │
└──────────────────────────┬──────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────┐
│  ECS world                                              │
│    Robot → Link* → Joint* → Actuator*                   │
│    optional Sensor (wrist cam, EE force)                │
└──────────────────────────┬──────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────┐
│  Rapier (rne_physics_rapier)                            │
│    base rigid body + articulated impulse joints         │
└──────────────────────────┬──────────────────────────────┘
                           ▼
┌─────────────────────────────────────────────────────────┐
│  DataBus                                                │
│    JointState, EePose, Camera, Contact, Imu, …            │
└─────────────────────────────────────────────────────────┘
```

### New / extended concepts

| Concept | Purpose |
|---------|---------|
| `MobileManipulatorConfig` | base type, arm joint names, gripper joint, initial pose |
| `JointState` payload | named positions/velocities for all actuated joints |
| `EePose` payload | end-effector frame in world or base coordinates |
| `MobileManipulatorAction` | base velocity + arm joint deltas or absolute targets |
| `MobileManipulatorObservation` | base pose, joint state, EE pose, optional object relative pose |

Asset format: extend `.rne.robot.toml` with optional `[mobile_manipulator]` section **or** promote `kind = "urdf"` robots to full physics articulation when `[physics]` is enabled.

```toml
kind = "urdf"
model_name = "mm_minimal"

[urdf]
path = "mm_minimal/mm_minimal.urdf"

[mobile_manipulator]
base_joint = "base_to_arm"
arm_joints = ["shoulder", "elbow", "wrist"]
gripper_joints = ["finger_left", "finger_right"]
end_effector_link = "ee_link"
drive_mode = "diff_drive"   # or "omni", "fixed"

[physics]
articulated = true
base_body = "base_link"
```

## Implementation phases

### Phase A — Articulated URDF physics (v0.6.0 core)

1. **`attach_urdf_articulation()`** in `rne_urdf_import` — **Done**
   - Create Rapier revolute joints for every non-fixed URDF joint
   - Attach `JointMotor` on actuated joints
   - Parent-aware physics sync via `world_transform_of`

2. **Minimal asset** `assets/robots/mm_minimal/` + example `20_mobile_manipulator_arm` — **Done**

3. **Smoke test**: spawn, apply constant shoulder velocity, assert link motion + determinism — **Done**

### Phase B — Environment API (v0.6.x)

1. **`MobileManipulatorSim`** in `rne_ai` (parallel to `DiffDriveSim`)
2. **`MobileManipulatorObservation` / `Action`**
3. **Example `20_mobile_manipulator_reach`**: open-loop reach toward fixed target
4. **Interactive viewer**: arm joint sliders + existing base teleop

### Phase C — Manipulation & sensors (v0.7)

1. Wrist / head camera from scene assets
2. Gripper mimic joints or simple parallel jaw
3. Episode rewards: reach distance, grasp contact, object displacement
4. Optional analytic IK helper (not in core; start with joint-space policies)

### Phase D — Adapters

1. ROS: `/joint_states`, `/cmd_vel`, `trajectory_msgs/JointTrajectory` (subset)
2. Python: expose joint commands via `rne_py`
3. Record / replay arm trajectories on DataBus

## Non-goals (for now)

- Full MoveIt integration inside core (adapter-only later)
- Deformable objects, fluid, or dual-arm coordination
- GPU physics or alternate backends beyond Rapier
- Replacing diff-drive examples and CI paths

## CI strategy

Keep existing diff-drive + LiDAR CI green. Add:

- `cargo test -p rne_urdf_import articulation`
- `cargo run --example 20_mobile_manipulator_reach -- --smoke`
- Optional ROS smoke: `/joint_states` name count matches URDF actuated joints

## Related docs

- [Robot native model](002_robot_native.md)
- [DataBus](005_data_bus.md)
- [Roadmap](../ROADMAP.md)

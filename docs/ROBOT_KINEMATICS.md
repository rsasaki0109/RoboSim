# Robot Kinematics and Link Devices

RNE models a robot as a `Robot` / `Link` / `Joint` ECS graph. Before this
feature the graph only carried static transforms: `rne_world::propagate_transforms`
walks `Parent` / `Children` and multiplies each link's local `Transform3`, but it
never reads joint positions. Every joint-aware forward/inverse kinematics solver
was hard-coded for one body (`mm_lift`, `mm_minimal`, `so101`, G1).

This layer borrows the body-model separation used by Choreonoid (`Body`,
`JointPath`, `InverseKinematics`, `BodyCollisionDetector`, `Device`,
`BodyMotion`) and makes the same capabilities generic over any robot that is
described by `Link` and `Joint` components. It lives entirely in `rne_robot` and
does not require a physics backend, a renderer, ROS 2, or wall-clock time.

## Forward kinematics and Jacobian

`KinematicModel::from_robot(world, robot)` derives a model from the ECS graph:

- Links are ordered topologically (parents before children), so forward
  kinematics is a single pass.
- Movable joints (revolute, continuous, prismatic) define the degrees of
  freedom in a deterministic order exposed by
  `KinematicModel::movable_joint_entities` and `movable_joint_names`.
- Fixed joints are retained for the transform chain but contribute no DoF.

A joint connects a parent link to a child link. The child link's local
`Transform3` is the joint origin, and displacement is applied after it:

```
revolute / continuous: child = parent * origin * R(axis, q)
prismatic:             child = parent * origin * T(axis * q)
fixed:                 child = parent * origin
```

`axis` is expressed in the joint frame, matching URDF import.
`KinematicModel::jacobian(q, link, point_local)` then returns the 6×N geometric
Jacobian (linear xyz, angular xyz) for the chain from the base to the target
link.

## Inverse kinematics

`KinematicModel::inverse_kinematics(target, end_link, initial, options)` is a
damped least-squares solver. It clamps to joint limits each iteration and can
solve position only or full pose (`IkOptions::solve_orientation`). The result
reports the iteration count and residual position/orientation error.

## Self-collision

`SelfCollisionChecker` is the backend-neutral analogue of Choreonoid's
`BodyCollisionDetector`. It evaluates forward kinematics and tests collider
pairs directly from `Collider` components:

- spheres and capsules use closed-form point/segment distances;
- a segment against an oriented box is minimized by golden-section search;
- cuboid/cuboid uses the 15-axis separating-axis test;
- same-link and structurally adjacent link pairs are skipped, controlled by
  `SelfCollisionChecker::from_robot_with_min_link_distance`;
- explicit `CollisionGroups` are honored with the same mask semantics as the
  physics backends.

`SelfCollisionReport` lists every overlapping pair and its penetration depth.
Infinite plane colliders are ignored.

## Link devices

`Device { link, name, kind }` plus `LinkDevices` mirror Choreonoid's
`Device` / `DeviceList`. `rne_robot::spawn_device`, `attach_device`,
`detach_device`, `devices_of_link`, and `devices_of_kind` keep the link's index
in sync. The abstraction deliberately does not reference any concrete sensor or
actuator type, so `rne_robot` stays independent of `rne_sensor`.

## Motion and controller boundary

- `BodyMotion` and `JointTrack` store keyframed joint positions with linear,
  smooth-step, or cubic Hermite interpolation and optional time looping.
  `body_motion_from_world` seeds a motion from the current joint state.
- `Controller` (trait), `ControllerIoFrame`, and `ControllerOutput` define the
  read/write boundary a controller sees. `build_controller_io`,
  `apply_controller_output`, and `step_controller` connect it to the ECS while
  clamping commands to joint and actuator limits.

## Determinism

All ordering is explicit: links are sorted by name then entity index before
topological ordering, joints are sorted by child, and controller input frames
are sorted by joint name then entity index. No wall-clock time or global random
state is used, so kinematic evaluation is reproducible across runs.

## Limits

- Self-collision approximates anything that is not a sphere, capsule, or cuboid
  by ignoring it; plane colliders are skipped.
- The kinematic model requires a single connected link tree rooted at
  `Robot::base_link`; disconnected or multiply-parented links are rejected.
- Cuboid/capsule vs cuboid distances used by the self-collision checker are
  exact for convex primitives but do not model mesh collision geometry.

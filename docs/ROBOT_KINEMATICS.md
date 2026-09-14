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

## Mimic and passive joints

`MimicJoint { source, multiplier, offset }` mirrors the URDF `<mimic>` tag: the
joint's displacement is `multiplier * source.position + offset`. A mimic joint
is not an independent degree of freedom — `KinematicModel::dof`,
`movable_joint_entities`, and `joint_limits` exclude it, and forward kinematics
derives its displacement from the source. The geometric Jacobian applies the
chain rule: the mimic joint's screw contribution is scaled by `multiplier` and
accumulated into the source column. `mimic_joint_entities` and
`is_mimic_joint` expose the relationship.

`PassiveJoint` marks a joint that keeps its degree of freedom but is not
actuated; `KinematicModel::passive_joint_entities` lists them so callers can
exclude them from a planning group or controller.

`FloatingBase` on the base link prepends six DoF `(x, y, z, roll, pitch, yaw)`
to the model. `KinematicModel::base_dof` reports the count, and forward
kinematics, the Jacobian, joint limits, sampling, clamping, and `RobotState`
all include the base so a mobile manipulator can be planned as one vector.

## Inverse kinematics

`KinematicModel::inverse_kinematics(target, end_link, initial, options)` is a
damped least-squares solver. It clamps to joint limits each iteration and can
solve position only or full pose (`IkOptions::solve_orientation`). The result
reports the iteration count and residual position/orientation error.

## Solver boundary and robot state

`KinematicsSolver` is the MoveIt `KinematicsBase` analogue: a swappable inverse
kinematics solver that receives the model per call, so one solver serves any
robot. The built-in `DampedLeastSquaresSolver`
(`DAMPED_LEAST_SQUARES_SOLVER`) wraps the algorithm above, and
`JacobianTransposeSolver` (`JACOBIAN_TRANSPOSE_SOLVER`) is a second selectable
solver using `dq = gain * J^T e` with a backtracking line search, and
`AnalyticTwoLinkSolver` (`ANALYTIC_TWO_LINK_SOLVER`) is a closed-form solver for
a planar two-revolute chain (an IKFast-style analytic plugin).
`KinematicsSolverRegistry` addresses solvers by name in insertion order while
rejecting empty or duplicate names; `KinematicsSolver::search_position_ik`
adds seeded random restarts. `IkRequest` carries the end link, target pose,
seed, options, and an optional active-joint mask.

`RobotState` mirrors MoveIt's `RobotState`: `RobotState::from_world` snapshots
the current joint positions and velocities in degree-of-freedom order, exposes
named read/write accessors, computes forward kinematics, and seeds a solver
through `RobotState::solve_ik`.

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

`SelfCollisionChecker::attach_body` adds a collision body rigidly fixed to a
link (the MoveIt `AttachedBody` analogue) with optional `touch_links`. Attached
bodies are tested against other links, other attached bodies, and world objects,
so a grasped payload participates in planning and distance queries.

`segment_intersects_primitive` reports whether a line segment is occluded by a
sphere, capsule, or cuboid, and `SelfCollisionChecker::segment_blocked` /
`CollisionWorld::segment_blocked` apply it to robot links, attached bodies, and
world objects for line-of-sight queries.

`AllowedCollisionMatrix` is the MoveIt ACM analogue: explicit link pairs can be
skipped on top of the structural parent/child exclusion.
`SelfCollisionChecker::distance` returns the closest checked pair and its signed
distance (`distanceRobot`), while `SelfCollisionChecker::check_path` samples a
joint-space path and reports the first colliding configuration
(`isPathValid`). `signed_distance` exposes the pairwise signed distance, and
`CollisionWorld` holds world-space primitives tested against the robot's links
with `check` and `distance`. Objects can be named and managed with
`add_named_object`, `object`, and `remove_object` (the MoveIt `CollisionObject`
analogue). `MeshCollisionObject` adds triangle meshes (`add_mesh_object`), tested
against spheres/capsules with exact triangle distances and against segments with
ray/triangle intersection. `VoxelGridObject` adds an Octomap-style occupancy grid
(`add_voxel_grid` or `from_points`), tested as cuboids against the robot.

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

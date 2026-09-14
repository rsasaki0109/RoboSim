# Joint-Space Motion Planning

`rne_planning` adds a native, MoveIt-inspired motion-planning layer for
articulated robots. It is **not** a MoveIt or ROS port: no MoveIt, ROS 2,
physics-backend, renderer, or external planner dependency is added to core. The
layer builds on the generic `rne_robot` kinematic model, collision checker, and
kinematics solvers introduced in [ROBOT_KINEMATICS.md](../ROBOT_KINEMATICS.md).

## MoveIt mapping

| MoveIt concept | RNE type |
|----------------|----------|
| `RobotModel` / `RobotState` | `rne_robot::{KinematicModel, RobotState}` |
| `KinematicsBase` plugin | `rne_robot::KinematicsSolver` + `KinematicsSolverRegistry` |
| `CollisionEnv` / `AllowedCollisionMatrix` | `rne_robot::{SelfCollisionChecker, AllowedCollisionMatrix, CollisionWorld}` |
| `PlanningScene` | `rne_planning::PlanningScene` |
| SRDF planning group | `rne_planning::PlanningGroup` |
| `MotionPlanRequest` / `MotionPlanResponse` | `rne_planning::{MotionPlanRequest, MotionPlanResponse}` |
| Kinematic constraints | `rne_planning::GoalConstraint` |
| `RobotTrajectory` | `rne_planning::RobotTrajectory` |
| `PlannerInterface` / `PlanningPipeline` | `rne_planning::{MotionPlanner, PlannerRegistry, PlanningPipeline}` |
| `PlanningRequestAdapter` | `rne_planning::PlanningRequestAdapter` |
| `FixStartStateBounds` / `AddTimeParameterization` | `rne_planning::{FixStartStateBounds, AddTimeParameterization}` |
| `CartesianInterpolator` / Pilz `LIN` | `rne_planning::{CartesianPathPlanner, CartesianPathRequest}` |
| OMPL RRT-Connect / RRT* / Pilz `PTP` | `rne_planning::{RrtConnectPlanner, RrtStarPlanner, JointInterpolationPlanner}` |

## Crate boundary

`rne_planning` may depend only on `rne_robot`, `rne_ecs`, and `rne_math`. It
reads a robot from the ECS once when building a `PlanningScene` and then operates
on plain values, so planning is headless and backend-neutral. The `MotionPlanner`
trait is the in-process plugin boundary: external crates register additional
planners through `PlannerRegistry` without changing `rne_planning`, mirroring
MoveIt's planner plugin model without a dynamic-loading ABI.

## Pipeline

1. `PlanningScene::from_world` snapshots the kinematic model, self-collision
   colliders, and the built-in kinematics solvers.
2. `MotionPlanRequest` carries the start state, a `GoalConstraint`, and
   `PlanningOptions`. `GoalConstraint::Pose` / `Position` are resolved with a
   `KinematicsSolver`, selected by name from the scene registry. When
   `PlanningOptions::ik_restarts` is positive the solver uses seeded random
   restarts (`searchPositionIK`), so a reachable goal is not lost to a poor
   start seed.
3. The selected planner returns a validated `RobotTrajectory` or a
   `PlanningError`.
4. `PlanningPipeline` runs its adapter chain, validates the request, dispatches
   to a named planner, and runs the adapters again on the response.

## Request adapters

`PlanningRequestAdapter` is the MoveIt `PlanningRequestAdapter` analogue. The
built-in `FixStartStateBounds` clamps a start state into the model's joint
limits, `FixWorkspaceBounds` clamps a goal position into the scene workspace box,
`ValidateWorkspaceBounds` rejects an out-of-bounds goal,
`FixStartStateCollision` perturbs a colliding start state toward a nearby valid
one, and `SimplifyTrajectory` removes redundant waypoints whose shortcut is
collision free. `AddTimeParameterization` retimes a trajectory with
`time_parameterize_with_acceleration`. Each segment is timed so no joint exceeds
its `JointLimits::max_velocity` or its per-joint
`PlanningOptions::acceleration_limits`, both scaled by
`PlanningOptions::velocity_scaling_factor`. With acceleration limits the profile
is stop-and-go trapezoidal (triangular for short moves); without them it is
piecewise-constant velocity. A model with no finite velocity or acceleration
limit leaves the trajectory timing unchanged.

## Planning groups

`PlanningGroup` is the SRDF planning-group analogue. It names either a kinematic
chain (`PlanningGroup::chain`) or an explicit joint set (`PlanningGroup::joints`),
and stores the corresponding degree-of-freedom indices. A `MotionPlanRequest`
selects a group with `with_group`, and the scene resolves it by name.

Group-scoped planning restricts sampling, steering, and interpolation to the
group's joints; every other joint holds its start value. Pose and position goals
use `KinematicModel::inverse_kinematics_active`, which builds a reduced Jacobian
from the active columns so inactive joints cannot move. `PlanningGroup` also
exposes `project` / `embed` / `active_mask` for callers that manage state
directly.

## Constraints and path constraints

`GoalConstraint` covers joint, full-pose, position-only, and orientation-only
goals. Pose goals resolve through inverse kinematics with an explicit objective:
`IkOptions::solve_position` and `solve_orientation` select which Jacobian rows are
driven, so an orientation goal leaves position free. `PlanningOptions::ik_restarts`
enables seeded `searchPositionIK` restarts for goals that a single seed misses.

A `MotionPlanRequest` can carry a `PathConstraint` (orientation or position of an
end link). Planners check it at every sampled configuration through
`PlanningScene::is_motion_valid_with`, so an invalid path fails rather than
silently violating the constraint. The built-in planners all honor it.

`ConstraintSampler` is the MoveIt `ConstraintSampler` analogue: it draws a
collision-free configuration satisfying a goal constraint. Joint goals are
embedded and validated directly; pose, position, and orientation goals use seeded
random-restart inverse kinematics and reject solutions that collide.

`PathConstraint::Visibility` is the MoveIt `VisibilityConstraint` analogue: a
sensor link (its local `+X` axis) must point at a target within a tolerance, and
the line of sight must not be occluded by robot links, attached bodies, or world
objects. Occlusion uses `segment_intersects_primitive` (sphere / capsule
distance, cuboid box distance) through `SelfCollisionChecker::segment_blocked`
and `CollisionWorld::segment_blocked`, exposed as
`PlanningScene::line_of_sight_clear`.

## Floating base

`FloatingBase` on a robot's base link prepends six degrees of freedom to the
kinematic model as `(x, y, z, roll, pitch, yaw)` (fixed-axis roll-pitch-yaw),
the MoveIt virtual-joint analogue. `KinematicModel::base_dof`,
`movable_dof_names`, `joint_limits`, forward kinematics, the geometric Jacobian,
active-mask inverse kinematics, clamping, and random sampling all account for the
base, so a mobile manipulator plans its base and arm in one vector. Planar/other
virtual-joint types are not modeled; the base is fixed-axis only.

## SRDF import

`parse_srdf` reads the `<group>` elements of an SRDF document into
`SrdfGroup::Chain` (base/tip links) and `SrdfGroup::Joints` (named joints)
values; other SRDF elements are ignored, so a full MoveIt SRDF parses unchanged.
`PlanningScene::apply_srdf` resolves the names against the kinematic model and
registers the groups, which can then be selected with
`MotionPlanRequest::with_group`. `KinematicModel` exposes
`link_entity_by_name`, `joint_entity_by_name`, `joint_parent_link`, and
`joint_child_link` for the resolution.

## Mesh collision

`MeshCollisionObject` adds triangle geometry to a `CollisionWorld`.
`add_mesh_object` / `remove_mesh_object` / `mesh_object` manage them, and
`PlanningScene::add_mesh_collision_object` forwards to planning. Meshes are tested
against robot spheres and capsules with exact point/segment-to-triangle distance
and against world segments with ray/triangle intersection; cuboid-vs-mesh uses
the mesh AABB (conservative). This is a bounded approximation of MoveIt's FCL
mesh collision, not a full BVH.

`VoxelGridObject` is the Octomap analogue: a dense occupancy grid built from an
explicit bitmap or from a point cloud (`from_points`). Occupied voxels are tested
as cuboids against robot spheres, capsules, and cuboids, and block line-of-sight
segments. `CollisionWorld::{add_voxel_grid, remove_voxel_grid, voxel_grid}`
manage them and `PlanningScene::add_occupancy_map` accepts a point cloud.

## Attached bodies

`SelfCollisionChecker::attach_body` adds a `CollisionBody` (MoveIt
`AttachedBody`) rigidly fixed to a link. Attached bodies are tested against every
other link except the ones in `touch_links` (for example the gripper holding the
object), against each other, and — because they are appended to
`link_primitives` — against every `CollisionWorld` object. `PlanningScene`
forwards attach/detach so planners, path collision checks, and world distance
queries all see the grasped object. Reported pairs carry the attached body name
in `SelfCollisionPair::body_a` / `body_b`.

## Trajectory optimization

`optimize_trajectory` is the RNE reference to MoveIt's CHOMP. It lowers
`trajectory_cost`, a weighted sum of a smoothness term (squared joint
accelerations) and an obstacle term (squared clearance violation from
`SelfCollisionChecker::distance` and `CollisionWorld::distance`). Gradients are
finite differences over the active joints, with a backtracking step and joint
limits applied; endpoints stay fixed. `PlanningOptions` seeds the traversal and
the result is deterministic. Optimization is feasibility-gated: a feasible seed
never becomes infeasible, and an infeasible seed only returns a feasible result
if one is found (`trajectory_is_feasible`). `HybridPlanner` mirrors MoveIt's
hybrid planning: it runs RRT-Connect for a feasible path, then `optimize_trajectory`
to smooth it, and falls back to the global plan if the refinement is infeasible.
`StompPlanner` reports `NoPath` when it cannot produce a feasible trajectory.

## Demo media

`examples/102_motion_planning_media` loads the checked-in RNE-converted OpenArm v2
left arm (7-DOF, GLB meshes), plans a collision-free swing around a collision
object with RRT-Connect, and plays the trajectory through the wgpu renderer. It writes
`docs/media/motion-planning.gif` and `docs/media/motion-planning.png` directly
(no external generator); `-- --smoke` runs the planner headlessly and asserts the
obstacle blocks straight joint interpolation while RRT-Connect stays feasible.
The README "Native motion planning" section embeds both.

## MoveIt coverage

This is a native reimplementation that references MoveIt's architecture. Rough
coverage of MoveIt's useful, non-ROS core is about **77%**, weighted by
capability rather than line count:

| Subsystem | Coverage | Ported | Missing |
|-----------|----------|--------|---------|
| RobotModel / SRDF groups | ~90% | links, joints, DoF, limits, mimic/passive joints, planning groups (chain/joint), SRDF import, floating base | — |
| RobotState | ~60% | named joint state (base first), FK, IK seeding | attached bodies in state |
| Kinematics | ~80% | FK, Jacobian, DLS + Jacobian-transpose + analytic two-link IK, solver registry, group-scoped active-mask IK, seeded `searchPositionIK`, manipulability, joint-limit distance | general analytic/IKFast-class solvers |
| Collision | ~93% | self/world collision, named collision objects, triangle meshes, voxel occupancy maps, ACM, signed distance, path checks, attached bodies | FCL-grade BVH / exact mesh-mesh |
| Constraints | ~80% | joint, pose, position, orientation goals, path constraints, constraint sampler, visibility | — |
| Planning scene | ~60% | scene, collisions, solvers, planning groups, workspace bounds | scene diffs / apply |
| Planners | ~96% | joint interpolation (PTP), RRT-Connect, RRT*, informed RRT*, PRM, batch informed (BIT*-style), linear and circular Cartesian (LIN/CIRC), CHOMP and STOMP trajectory optimization, hybrid (global + CHOMP), group-scoped sampling | OMPL variants (KPIECE/SBL) |
| Pipeline / adapters | ~72% | pipeline, registry, fix-bounds, fix-workspace, fix-collision, trajectory simplification, time parameterization | constraint-resolving adapters |
| Trajectory | ~60% | timed trajectory, velocity- and acceleration-limited parameterization, CHOMP/STOMP optimization | jerk-limited parameterization, Cartesian timing |

Remaining useful ports: other OMPL variants (KPIECE/SBL), general
analytic/IKFast-class solvers, FCL-grade BVH collision, and planning-scene
diff/apply.

## Determinism

- Entity ordering inside the model, collision checker, and planners is explicit
  and deterministic.
- Sampling planners use a local xorshift generator seeded only by
  `PlanningOptions::seed`; no global random state or wall-clock time is read.
- Path collision checks interpolate a fixed number of steps
  (`PlanningOptions::collision_check_steps`).
- The same scene and request always produce the same trajectory.

## Limits

- `RrtConnectPlanner` is a bidirectional greedy tree planner, not a
  probabilistically complete RRT-Connect with nearest-neighbour indexing; it is
  intended for bounded, deterministic problems.
- `JointInterpolationPlanner` only succeeds when the straight joint-space line
  is collision-free.
- Goal poses are resolved by the built-in damped least-squares solver, which
  does not handle joint-angle wrapping; position goals are the recommended
  interface for redundant or winding chains.
- `RrtStarPlanner` selects the cheapest parent and rewires with exact cost
  propagation. It is asymptotically optimal in the limit of the sampling, but
  the returned result is only the best connection found within the iteration
  budget; nearest-neighbour search is linear rather than a spatial index.
- `CartesianPathPlanner` interpolates poses independently in translation and
  rotation, so full-pose motion on a non-redundant arm can leave the reachable
  pose manifold and stop early. It reports the completed `fraction`, matching
  MoveIt's `computeCartesianPath`; use position-only mode for planar chains.
  `circular_waypoints` builds a circular arc (Pilz `CIRC`) through a via pose for
  the same planner; collinear points are rejected.
- `searchPositionIK` restarts are not a completeness guarantee: the deterministic
  sampler is uniform over the declared limits and can still miss a solution
  within the restart budget. `manipulability` is a 6x6 `J J^T` determinant and
  `joint_limit_distance` ignores continuous joints.
- `PrmPlanner` connects each roadmap node only to its nearest
  `roadmap_neighbors` within `neighbor_radius`, so a disconnected roadmap returns
  `NoPath` even when a path exists; it is not a complete or asymptotically
  optimal PRM.

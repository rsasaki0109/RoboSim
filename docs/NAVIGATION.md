# Navigation Foundations

`rne_nav` is the ROS-free base for a SLAM / Nav2-style stack. It provides the
data structures and deterministic algorithms that planning, mapping, and
localization build on, without a physics backend, a renderer, or ROS 2. A ROS 2
adapter maps these types to `nav_msgs`, `sensor_msgs`, and `tf2` without
changing them.

This is Phase 1 of the navigation roadmap. It intentionally stops short of
path planning and scan matching; those arrive in later phases.

## Contents

| Type | Role |
| --- | --- |
| `OccupancyGrid` | Row-major 2D log-odds grid with world/grid projection and grid ray casting |
| `Costmap` | Planar cost surface with lethal, inscribed, and exponentially inflated costs |
| `TfBuffer` | Timestamped transform tree with shortest-path lookup and time interpolation |
| `LaserScan2d` | Planar scan payload and `integrate_scan` occupancy updates |
| `Path2d` | Ordered planar waypoints with arc-length and closest-point queries |
| `plan_path` | Grid A* / Dijkstra global planner over the costmap |
| `pure_pursuit_follow` | Lookahead path follower producing velocity commands |
| `MobileBase` | Differential / Ackermann / mecanum actuator with limits and fault handling |
| `EkfFusion` | 2D EKF fusing odometry, IMU yaw rate, and GPS position fixes |
| `DwaPlanner` | Dynamic-window local planner with rollout scoring |
| `RecoverySequence` | Nav2-style clear/spin/backup/wait recovery behaviors |
| `Sequence` / `Selector` | Minimal deterministic behavior tree for plan/control/recovery flow |
| `TiledOccupancyGrid` | Lazily allocated tiled occupancy map for large areas |
| `avoid_velocities` | Deterministic sampling sense-and-avoid for multiple robots |
| `integrate_point_cloud` | Projects a 3D point cloud (height band) into the occupancy grid |
| `ElevationMap` | 2.5D per-cell min/max/mean height map with slope and traversability queries |
| `NavMap` / `PendingScans` / `TfTree` | ECS resources |
| `integrate_pending_scans` | ECS system that drains the scan queue and refreshes the costmap |
| `NavGoal` | Planar goal component for future planners |

## Occupancy grid

`OccupancyGrid::new(width, height, resolution_m, origin)` creates an unknown
grid. Cells accumulate log-odds via `apply_occupied` / `apply_free` (defaults
`+0.85` / `-0.4`) clamped to `[-2.0, +3.5]`, so `probability`, `is_occupied`,
`is_free`, and `is_known` are deterministic functions of the update history.
`cell_value` returns the ROS-style map value (`-1` unknown, `0` free, `100`
occupied).

`raycast(from_m, to_m)` uses an Amanatides–Woo grid traversal and returns the
in-bounds cells a world-space ray crosses, which underlies both scan
integration and later ray-based clearing.

## Costmap

`Costmap::from_occupancy(grid, config)` seeds lethal cells from the occupied
threshold, runs a two-pass chamfer distance transform, and assigns:

* `254` lethal,
* `253` within the inscribed radius,
* an exponential decay between the inscribed and inflation radii,
* `0` free, `255` unknown.

## Transform tree

`TfBuffer` stores one edge per parent → child relationship. `set_transform`
inserts time-sorted samples and rejects a frame gaining a second parent.
`lookup(parent, child, time_s)` finds the shortest path through the tree,
composes edges in either direction, and interpolates each edge with linear
translation and spherical rotation interpolation. Extrapolation is an error
unless `set_allow_extrapolation(true)` is set. This covers
`map → odom → base_link → sensor` chains without `tf2`.

## Scan integration

`LaserScan2d` carries the angle range, range limits, and a nominal sensor
`FrameId`. `integrate_scan(grid, scan, sensor_pose_world, config)` casts every
`beam_stride`-th beam, clears free space along the path, and marks the endpoint
occupied on a return. No-return beams clear up to `no_return_range_m` (or the
scan max range). Integration is a pure function; the caller resolves the sensor
pose, typically through `TfBuffer::lookup`.

## Global planning

`plan_path(costmap, start_m, goal_m, config)` runs A* over the costmap with an
octile heuristic; `GlobalPlannerConfig::heuristic_weight = 0.0` degrades it to
Dijkstra. `cost_weight` folds the normalized cell cost into each edge, so the
path prefers clearance even when it is longer. Lethal cells are always blocked,
unknown cells are blocked unless `allow_unknown` is set, and the search is
bounded by `max_iterations`. The result is a `Path2d` whose endpoints are the
exact requested positions. Errors distinguish an outside-map or blocked start /
goal from a genuine `NoPath`.

`Path2d` supports arc-length queries (`point_at_distance`), closest-point
queries (`closest_point`), and `prune_collinear` for straightening planner
output.

## Path following

`pure_pursuit_follow(path, pose, config)` finds the closest path point, advances
a lookahead distance along the path, and returns a `VelocityCommand2d` from the
pure-pursuit curvature law. Forward speed scales down near the goal and on
sharp curvature; `FollowResult` reports the remaining distance and whether the
goal tolerance was reached.

## Local planning

`DwaPlanner::compute_command(costmap, path, pose, current)` samples the dynamic
window of reachable linear / angular velocities around `current`, rolls each
candidate forward with a constant-twist model, rejects rollouts that cross
lethal or unknown cells, and scores the survivors by heading, clearance, path
alignment, and speed. `DwaConfig` controls the window, sample counts, horizon,
weights, and footprint tolerance. The result is either the best
`VelocityCommand2d` or `Reached` when the goal tolerance is met.

## Elevation mapping

`ElevationMap` is the 2.5D companion to `integrate_point_cloud`. Points project
onto the navigation plane (`world X-Z`) and keep their `world Y` height, so every
cell stores a running minimum, maximum, and mean height plus a sample count.
`height_at` (mean height), `slope_at` (maximum slope to the `+x`/`+y` neighbours,
in radians), and `is_traversable` (known, intra-cell step within
`ElevationConfig::max_step_m`, slope within a limit) answer ground-robot
traversability queries. `to_obstacle_grid` converts large intra-cell steps into
a planar `OccupancyGrid` for the existing planner and costmap. Integration is
index-ordered and deterministic.

## Drive actuators

`MobileBase::new(drive, limits)` couples a `DriveKind`
(`DifferentialDrive`, `AckermannDrive`, or `MecanumDrive`) with `DriveLimits`
and a rate limiter. `command(desired, dt_s)` validates the request, clamps it to
the velocity and acceleration limits, and applies the dynamic window since the
previous command. It returns a `DriveOutput` with the applied
`VelocityCommand2d`, the wheel/steering `DriveActuation`, and a `DriveFault`:

* `DriveFault::None` — applied without clamping,
* `DriveFault::Saturated` — clamped by a velocity, steering, or acceleration
  limit,
* `DriveFault::Disabled` — `set_disabled(true)` emits a hard-stop zero command.

Non-finite commands and non-positive time steps are rejected with a
`DriveError`, and a disabled or saturated base never emits a non-finite wheel
setpoint, so every fault degrades to a safe stop. `DifferentialDrive` also
inverts wheel rates back to a body command for odometry.

## Recovery behaviors

When planning or control stalls, `RecoverySequence` runs Nav2-style actions in
order until one succeeds. `RecoveryAction` covers `ClearCostmap` (resets cells
within a radius toward free with `CLEAR_LOG_ODDS`), `Spin` (rotate through an
accumulated angle), `BackUp` (drive backward a bounded distance), and `Wait`.
Each action is executed by a stateful `RecoveryBehavior` that emits
`VelocityCommand2d` setpoints and reports `Running`, `Succeeded`, or `Failed`;
the sequence skips failed actions and returns `Succeeded` (retry planning) or
`Exhausted`. Progress counters make replay bit-identical.

## Behavior tree

`behavior_tree` provides the Nav2 BehaviorTree.CPP control flow without a
runtime dependency: `Sequence` ticks children until one fails, `Selector`
ticks until one succeeds, and both propagate `Running` while resuming at the
same child. `Condition` and `Action` leaves are built from closures and write
the `BtContext` (pose, command, tick count), so a plan/control/recovery tree is
deterministic and testable headless.

## State estimation

`EkfFusion` estimates `[x_m, y_m, yaw_rad, v_m_s, yaw_rate_rad_s]` with a
constant-velocity model and plain `f64` arithmetic (no random numbers, so replay
is bit-identical). `predict(dt_s)` advances the model and inflates the
covariance with `EkfConfig` process noise; measurements are fused through
`update_odometry` (forward velocity + yaw rate), `update_yaw_rate` (IMU only),
and `update_position` (a planar fix such as GPS, in the map frame). Angles wrap
to `(-pi, pi]` and the covariance is re-symmetrized after every update.

## Determinism

All iteration is index-ordered: beams by index, frames and neighbours sorted by
name, and the pending scan queue drained in insertion order. No wall-clock time
or global random state is used, so mapping a recorded sensor sequence reproduces
the same grid and can be hashed for replay tests.

## Roadmap

1. **Foundations** — grids, costmaps, transform tree, scan integration. *(done)*
2. **Planning and control** — A*/Dijkstra global planner, DWA local planner, pure-pursuit path following, drive actuators with limits/faults, recovery behaviors, and a behavior tree. *(done)*
3. **SLAM** — deterministic 2D scan matching, online occupancy mapping, and
   pose-graph optimization with loop closure in `rne_slam`. *(done)*
4. **ROS 2 adapter** — RNE→ROS message mappings *(done)* and the `rclpy` bridge
   node publishing `/odom`, `/scan`, `/map`, `/plan` and subscribing `/cmd_vel`
   *(done)*; a Nav2-compatible `NavigateToPose` action server with feedback and a
   spin recovery *(done)*; verified end to end against `nav2_bringup`.
5. **3D and estimation** — 2.5D `ElevationMap`, 3D point-to-point ICP in
   `rne_slam`, and the odometry/IMU/GPS `EkfFusion`. *(done)*
6. **Editor** — item tree, property editing, and motion timeline in the web
   viewer. *(skipped)*

## Tested end to end

- `examples/98_nav_slam_physics`: Rapier world + real `rne_sensor` LiDAR →
  `rne_slam` (SLAM 0.05 m vs odometry 0.19 m, 20 loop closures).
- `examples/100_nav_elevation_icp`: 2.5D elevation traversability and 3D ICP
  (mean residual 2e-16 m).
- `adapters/ros2/rne_ros2_bridge/nav2_integration.sh`: Nav2 reaches a
  `NavigateToPose` goal by driving the RNE base through `/cmd_vel`.
- `adapters/ros2/rne_ros2_bridge/nav2_action_smoke.sh`: the bridge's own
  `NavigateToPose` action server reaches a goal (`error_code: 0`).
- `tests/determinism/tests/nav_slam.rs`: mapping, planning, DWA, and SLAM
  reproduce exact poses and a stable grid hash.

## Limits

- The grid is full-resolution and dense; very large maps should be tiled.
- Scan integration uses a beam-cast model, not a beam-width likelihood field.
- The transform tree stores parent → child edges only; each frame has a single
  parent.

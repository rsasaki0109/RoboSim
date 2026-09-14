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
| `DwaPlanner` | Dynamic-window local planner with rollout scoring |
| `TiledOccupancyGrid` | Lazily allocated tiled occupancy map for large areas |
| `avoid_velocities` | Deterministic sampling sense-and-avoid for multiple robots |
| `integrate_point_cloud` | Projects a 3D point cloud (height band) into the occupancy grid |
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

## Determinism

All iteration is index-ordered: beams by index, frames and neighbours sorted by
name, and the pending scan queue drained in insertion order. No wall-clock time
or global random state is used, so mapping a recorded sensor sequence reproduces
the same grid and can be hashed for replay tests.

## Roadmap

1. **Foundations** — grids, costmaps, transform tree, scan integration. *(done)*
2. **Planning and control** — A*/Dijkstra global planner, DWA local planner, pure-pursuit path following. *(done; recovery behaviors pending)*
3. **SLAM** — deterministic 2D scan matching, online occupancy mapping, and
   pose-graph optimization with loop closure in `rne_slam`. *(done)*
4. **ROS 2 adapter** — RNE→ROS message mappings *(done)* and the `rclpy` bridge
   node publishing `/odom`, `/scan`, `/map`, `/plan` and subscribing `/cmd_vel`
   *(done)*; verified end to end against `nav2_bringup`.
5. **Editor** — item tree, property editing, and motion timeline in the web
   viewer. *(skipped)*

## Tested end to end

- `examples/98_nav_slam_physics`: Rapier world + real `rne_sensor` LiDAR →
  `rne_slam` (SLAM 0.05 m vs odometry 0.19 m, 20 loop closures).
- `adapters/ros2/rne_ros2_bridge/nav2_integration.sh`: Nav2 reaches a
  `NavigateToPose` goal by driving the RNE base through `/cmd_vel`.
- `tests/determinism/tests/nav_slam.rs`: mapping, planning, DWA, and SLAM
  reproduce exact poses and a stable grid hash.

## Limits

- The grid is full-resolution and dense; very large maps should be tiled.
- Scan integration uses a beam-cast model, not a beam-width likelihood field.
- The transform tree stores parent → child edges only; each frame has a single
  parent.

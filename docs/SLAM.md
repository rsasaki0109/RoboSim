# 2D SLAM

`rne_slam` is the deterministic, ROS-free online SLAM front-end. It builds on
`rne_nav` and turns a stream of `LaserScan2d` plus drifting odometry into a
corrected trajectory and an occupancy map. There is no renderer, physics
backend, `tf2`, or external SLAM dependency.

Phases 3a (front-end and online mapping) and 3b (pose-graph optimization and
loop closure) are implemented.

## Pipeline

For each scan:

1. **Predict** — the base pose is advanced by the odometry delta
   `previous_odom⁻¹ · odom`. The first scan seeds the map at the odometry pose.
2. **Match** — scan endpoints (downsampled to `SlamConfig::max_beams`) are
   scored against a `LikelihoodField` built from the map so far. A
   coarse-to-fine correlative search over `(x, y, yaw)` picks the best pose.
3. **Integrate** — the corrected pose plus `sensor_from_base` gives the sensor
   pose, and `rne_nav::integrate_scan` updates the occupancy grid.
4. **Record** — the corrected pose becomes a pose-graph node, connected to the
   previous node by an odometry edge.
5. **Loop close** — on a match, historical nodes are ranked by proximity to the
   current pose; the closest are matched and a high-scoring revisit adds a
   loop-closure edge and re-optimizes the graph. Two robust gates reject
   outliers: the matched pose must stay within
   `loop_max_translation_residual_m` / `loop_max_rotation_residual_rad` of the
   odometry prediction, and after optimization the loop edge must still satisfy
   the same bounds or it is rolled back.

`Slam2d::process(scan, odom_pose, sensor_from_base)` returns a `SlamUpdate` with
the corrected pose, whether matching applied, the match score, the number of
loop closures added, and the integration report. The live pose stays
scan-matched; the pose graph is refined in the background.

## Likelihood field

`LikelihoodField::from_occupancy(grid, config)` runs a chamfer distance transform
from obstacle cells and stores `exp(-d² / (2σ²))` clipped at
`max_distance_m`. Endpoints on an obstacle score near `1.0`, giving the matcher
a smooth basin to descend.

## Scan matching

`ScanMatcher::match_scan(field, points_sensor, sensor_from_base, initial_base)`
searches a linear and angular window, narrowing one coarse step per level. The
search order is fixed, so ties and results are deterministic. A match is
rejected when fewer than `min_valid_beams` endpoints land inside the field.
Setting `ScanMatchConfig::coarse_downsample_factor > 1` evaluates the first level
against a max-pooled `LikelihoodField` (multi-resolution match), widening the
basin before the fine levels refine it.

## Pose graph

`PoseGraph` stores poses as nodes and relative-pose constraints as edges.
`PoseGraphEdge::odometry` and `PoseGraphEdge::loop_closure` build the two edge
kinds, each carrying diagonal information. `PoseGraph::optimize(iterations,
damping, anchor)` runs Gauss-Newton with an additive `(x, y, yaw)` perturbation
model and a Cholesky solve, anchoring one node and fixing every node
disconnected from it. `PoseGraph::optimize_robust` adds a Huber kernel that
down-weights edges whose residual exceeds a threshold, so one grossly wrong loop
closure cannot drag the trajectory.

The residual is `e = m⁻¹ ∘ q_from⁻¹ ∘ q_to`, with

```
J_from = [[-cosθ, -sinθ,  d·sinθ],      J_to = [[ cosθ,  sinθ, 0],
          [ sinθ, -cosθ, -d·cosθ],              [-sinθ,  cosθ, 0],
          [ 0,     0,    -1     ]]              [ 0,     0,    1]]
```

where `θ = q_from.yaw + m.yaw` and `d = q_from - q_to`. A finite-difference unit
test pins these Jacobians so the solver cannot silently regress.

## Localization (AMCL)

`Amcl` is a deterministic Monte Carlo localization filter over a prior map. It
predicts the particle cloud from the odometry delta with motion noise, weights
each particle by the mean likelihood of its transformed scan endpoints, and
resamples (systematic, deterministic) when the effective sample size falls below
half. `Amcl::update(scan, odom, sensor_from_base)` returns the weighted-mean
estimate; the estimate converges near truth and is bit-identical across runs.

## 3D point-cloud ICP

`Icp3d::align(source, target, initial, config)` registers a 3D source cloud onto
a target cloud and returns the rigid `Transform3` (target ← source). It pairs
each (strided) source point with its brute-force nearest target point within
`max_correspondence_distance_m`, then recovers the incremental transform with
Horn's closed-form quaternion method. The largest eigenvector of the 4x4 Horn
matrix is found by power iteration after a Gershgorin shift, so the algebraically
largest eigenvector is selected deterministically even when the most negative
eigenvalue has a larger magnitude. `IcpResult` reports the transform, iteration
and correspondence counts, convergence, and the mean residual. This is the 3D
front-end used with the `rne_nav` elevation map for LiDAR/point-cloud mapping.

## 3D ICP odometry

`IcpOdometry` is the 3D LiDAR front-end. It keeps a voxel-downsampled map cloud
and estimates the sensor pose by aligning each new scan to it, using the
odometry delta as the motion prediction: the predicted world points are aligned
to the map with `Icp3d`, the resulting correction is composed onto the
prediction, and the scan is integrated at the corrected pose. `voxel_downsample`
collapses points to one centroid per voxel in deterministic (`BTreeMap`) order.
`IcpOdometryUpdate` reports the corrected pose, correspondence count, residual,
convergence, and whether ICP was applied. Feed the corrected pose into
`rne_nav::ElevationMap` for 3D/outdoor mapping.

`Slam3d` is the back-end on top: it adds keyframes when the robot moves beyond
`keyframe_translation_m` / `keyframe_rotation_rad`, keeps a 2.5D `(x, z, yaw)`
pose graph (world Y is up), and on a revisit within `loop_search_radius_m`
verifies the match with `Icp3d`. A low-residual match adds a loop-closure edge,
re-optimizes with the Huber kernel, syncs the keyframe poses, and rebuilds the
elevation map from every keyframe. `process(scan, odom_delta, sensor_from_base)`
returns a `Slam3dUpdate`; `graph()`, `elevation()`, `keyframe_count()`, and
`loop_closures()` expose the state.

## Pose-graph persistence

`to_graph_json` / `save_graph` write a pose graph as versioned `.rne.posegraph`
JSON (`RNE_POSE_GRAPH_FORMAT`, `RNE_POSE_GRAPH_VERSION`) carrying every node and
edge; `from_graph_json` / `load_graph` validate the format tag and version and
reject edges that reference missing nodes. `combine_graphs(base, other)` appends a
second session's nodes and edges after the first with offset indices — the
back-end primitive for multi-session mapping.

## Global relocalization

`GlobalRelocalizer` scores candidate `(x, y, yaw)` poses by the mean likelihood
of the scan endpoints under a `LikelihoodField` built from a prior map. A coarse
grid search over the map bounds is followed by `refine_levels` of local
refinement; the best pose is returned when its score clears `min_score`. It
needs no odometry, so it recovers from a kidnapped robot or an unlocalized
start, and the fixed search order makes the estimate reproducible.

## ECS glue
- `SlamState` wraps a `Slam2d` estimator as a resource.
- `PendingSlamScans` queues `(scan, odom_pose, sensor_from_base)`.
- `slam_step(world)` drains the queue and returns a `SlamStepReport`.

## Determinism

Odometry prediction, beam order, search order, candidate ranking, and map
updates are all index-ordered with no wall-clock time or random state, so a
recorded scan/odometry sequence reproduces the same map and trajectory hash.
`tests/determinism/tests/nav_slam.rs` runs mapping, planning, DWA, and SLAM
twice and compares exact poses and a stable FNV-1a hash of the occupancy grid.

## Lifelong mapping across sessions

A robot that works in one building maps it many times. `combine_graphs`
concatenates two session graphs, but it adds no constraint between them, so the
result is two disconnected trajectories in one container — two maps, not a
lifelong map. Worse, `PoseGraph::optimize` accepts a disconnected graph and
reports success, so a caller can believe two sessions were merged when the
second was never constrained at all. (`PoseGraphError::Disconnected` is declared
but never raised; see the limits below.)

`LifelongPoseGraph` adds the missing piece. Merging a session

1. rigidly aligns it into the map frame from the first correspondence, so the
   optimizer starts from a sensible linearization point rather than from two
   overlapping trajectories,
2. re-indexes its intra-session edges, which are relative measurements and so
   are unchanged by the alignment,
3. adds one inter-session edge per [`SessionConstraint`] — the lifelong
   equivalent of a loop closure, relating a pose from an earlier visit to a pose
   from the current one, and
4. re-optimizes with a Huber kernel so one bad correspondence cannot drag both
   trajectories.

Every node keeps the session it came from, because pruning old nodes, decaying
stale structure, and reporting per-session drift all need to know which visit
contributed what.

Measured on a six-node corridor whose second visit over-reports each step by
5 cm: concatenating and optimizing leaves the second session **bit-identical**
to its drifting input (mean node error 0.125 m), because nothing relates it to
the first. Merging the same session against two correspondences reduces that to
**0.0715 m** and yields one connected map. A session recorded in a rotated,
translated frame is aligned onto the prior trajectory to within 1e-6 m.

Still open for lifelong operation: the correspondences are supplied by the
caller rather than discovered by relocalizing the new session against the prior
map; occupancy fusion has no notion of time, so structure that moved between
visits accumulates in both places instead of decaying; and the graph is never
pruned, so it grows without bound across sessions.

## Limits

- Loop closure is pairwise against the likelihood field; there is no robust
  back-end with switchable constraints or outlier rejection yet.
- The matcher is correlative and brute-force within its window; a
  multi-resolution pyramid and descriptor-based place recognition are later
  optimizations.
- The map is a single-resolution dense grid; pose-graph optimization does not
  yet re-integrate the map after the trajectory changes.
- `PoseGraphError::Disconnected` is declared but never returned: the optimizer
  does not check that every node is reachable from the anchor, so a disconnected
  graph optimizes "successfully" while its detached components stay unrelated.
  `LifelongPoseGraph::is_connected` is the guard for the multi-session path;
  raising the error from the optimizer itself would change existing callers and
  has not been done.

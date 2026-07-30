# Deterministic traffic runtime

`rne_traffic` provides a backend-neutral kinematic path for large traffic
populations. It is intended for deterministic urban replay and background
traffic; selected Robot Entities can still use a full vehicle dynamics or
physics model.

## Runtime model

Each managed entity has:

- `TrafficActor` and a stable `rne_ecs::EntityUuid`;
- `TrafficRouteFollower`, containing route ID, longitudinal distance, current
  and desired speed, and vehicle length;
- `TrafficPose`, containing the sampled position and Y-up yaw.
- optionally, `TrafficDeparture`, containing the earliest simulation time at
  which the actor may move.

`TrafficRouteCatalog` stores validated open or closed 3D polylines by
`TrafficId`. Route distances and poses are in meters and radians. An open route
stops at its final point; a closed route wraps across its final-to-first
segment.

`advance_kinematic_traffic` receives `SimTime` and `SimDuration` explicitly. It
never reads wall-clock time. Before mutation it validates every actor, route
reference, UUID, numeric value, and control parameter. Invalid input therefore
cannot leave a partly advanced fleet.

## Deterministic car following

Actors are ordered by UUID, then grouped by route and longitudinal distance.
Each actor accelerates toward `desired_speed_m_s`, subject to configured
acceleration and braking limits. Its leader gap limits travel using vehicle
length, a fixed minimum bumper gap, and a speed-proportional time headway.
Updates are calculated from the same pre-step snapshot, so ECS insertion order
cannot affect the result.

The step report includes actor count, minimum observed gap, completed simulation
step metadata, and a stable FNV-1a hash over ordered follower and pose state.
The hash implementation, floating-point state, and actor order are explicit.

`shortest_lane_route` plans over schema-v1 directed connections. Cost is
centerline and connection-path distance; equal-cost alternatives use lane and
connection stable IDs as deterministic tie-breakers.
`materialize_lane_route` validates that exact lane/connection sequence and
assembles its centerlines and turn curves into one `TrafficRoute`.

`TrafficSignalControls` supplies stable route stop positions and current
aspects. The controlled step limits braking speed and clamps the vehicle front
before a red stop line. Signal phase scheduling remains a policy input, keeping
scenario timing and external traffic adapters out of the integrator.

Materialized routes retain the source connection ID and entry/exit distance of
each turn movement. `TrafficConflictControls::from_network_routes` groups all
movements in a junction conservatively, then uses the network's symmetric
connection conflicts to join any related groups across source junction IDs.
Consecutive movement spans from one route are merged so ownership cannot be
released midway through an intersection.
`advance_reserved_kinematic_traffic` grants each group to one UUID at a time,
using movement priority, estimated arrival time, and UUID as stable
tie-breakers. A not-yet-departed vehicle or one blocked by a red signal cannot
reserve. A vehicle without the reservation stops its front before the
connection path with an explicit safety setback; ownership is retained until
the vehicle rear clears the movement. Cross-route collision diagnostics use
oriented rectangles from vehicle length and configured width rather than only
center-point distance. This policy remains backend-neutral and requires no
renderer, physics engine, importer, or external traffic simulator.

Every step also reports average active speed, waiting actor count, maximum
per-route queue length, cumulative completed trips, cumulative waiting time,
and active reservation count. Metrics are derived from explicit simulation
time. They do not read the wall clock; wall time is used by examples only to
measure execution throughput.

## Acceptance coverage

`crates/rne_traffic/tests/fleet_replay.rs` runs 100 vehicles for 600 fixed
steps on a closed urban route. Forward and reverse ECS spawn order must produce
the committed state hash `5765881651073142143`, identical minimum gaps, and no
violation of the configured two-meter bumper gap.

Renderer-free Example 47 combines shortest routing, red-to-green stopping,
left and right turns, and 100 vehicles. It asserts zero signal violations, zero
bumper overlaps, spawn-order-identical replay hashes, and measured throughput
of at least 60 simulation steps per wall-clock second:

```bash
cargo run -p traffic_city_replay --example 47_traffic_city_replay
```

Example 46 runs this complete pipeline on the official PLATEAU Sanjo asset:
84 lanes become 26 junctions and 137 connections. A stable diversity selector
chooses eight reachable shortest paths with distinct origins and destinations
and real connection conflicts. One hundred compact cars, sedans, vans, and
buses receive `WorldRandom`-derived speeds, departure times, and initial gaps,
then run for 720 steps under 24 signal controls and junction reservations.
Forward and reverse spawn order must produce the same hash, with zero signal
violations, zero collisions, a two-meter minimum gap, and throughput above
60 Hz. The replay also asserts that reservations are exercised and reports
average speed, waiting time, queue length, and throughput. The generated
`docs/media/plateau-car.gif` contains 144 real wgpu frames over 12 seconds.
Optional batched debug layers show every lane, the chosen route, signal
positions, generated connections, and conflict points.

The follow-on `docs/media/plateau-lidar.gif` mounts a physics-aware 16-channel
LiDAR on the tracked vehicle and raycasts against official building collision
geometry, the other traffic actors, and their retroreflective licence plates. This integration remains example-side:
`rne_traffic` does not depend on sensors, physics, rendering, or PLATEAU.

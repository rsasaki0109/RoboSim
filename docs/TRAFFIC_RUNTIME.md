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

Signal programs and route choice are policy inputs: update an actor's desired
speed or route before the kinematic step. This keeps signal control, agent
policy, and external traffic adapters out of the backend-neutral integrator.

## Acceptance coverage

`crates/rne_traffic/tests/fleet_replay.rs` runs 100 vehicles for 600 fixed
steps on a closed urban route. Forward and reverse ECS spawn order must produce
the committed state hash `5765881651073142143`, identical minimum gaps, and no
violation of the configured two-meter bumper gap.

Example 46 uses the same `TrafficRoute`, `TrafficRouteFollower`, and
`advance_kinematic_traffic` API for the lead vehicle in the rendered official
PLATEAU Sanjo scene. A deterministic pre-step signal policy reduces desired
speed for red, then releases the vehicle on green. The generated
`docs/media/plateau-car.gif` contains 144 rendered frames over 12 seconds.

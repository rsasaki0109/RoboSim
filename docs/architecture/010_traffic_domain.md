# 010 — Backend-neutral traffic domain

## Status

Accepted for implementation.

## Context

RNE can import PLATEAU road surfaces and render a scripted car, but the importer
currently derives two straight lanes per surface and examples own their route,
turn, and signal logic. That cannot represent connected tiles, intersections,
grade separation, signal compliance, or deterministic multi-vehicle replay.

Traffic is part of the simulated world, not part of a renderer, physics backend,
PLATEAU parser, ROS 2 adapter, or external traffic simulator. Motor vehicles
remain Robot Entities; autonomous drivers remain Agent Entities. A traffic
component augments those entities instead of replacing the robot-native model.

## Decision

Add `rne_traffic`, a backend-neutral runtime domain for:

- stable traffic-network identifiers and source provenance;
- directed lanes, junctions, lane connections, turn geometry, and conflict
  relationships;
- signal groups, deterministic phase programs, and compliance state;
- route planning and lane-relative actor state;
- deterministic kinematic traffic stepping and replay diagnostics.

The crate's public data model uses explicit SI units and ordinary RNE/math
types. Externally visible collections use a canonical stable-ID order. Runtime
systems receive `SimTime`/`SimDuration` or fixed step indices explicitly; they
never read wall-clock time. Randomized behavior receives an explicit
`WorldRandom` stream and seed.

The first asset format is deterministic `.rne.traffic.json` schema v1.
Serialization is canonical and byte-identical for identical input. Every
imported or inferred value carries one of these authority classes:

- **Authoritative**: directly encoded by the source dataset;
- **Derived**: deterministically calculated from source geometry or semantics;
- **Synthetic**: supplied by an RNE scenario, such as a signal phase program
  absent from PLATEAU.

`rne_plateau` is an offline producer of this asset. It may parse PLATEAU
CityGML and depend on `rne_traffic`; `rne_traffic` must not depend on
`rne_plateau` or expose CityGML types.

## Dependency boundary

`rne_traffic` may depend on:

- `rne_math` for backend-neutral spatial values;
- `rne_core` for simulation time;
- `rne_ecs` and `rne_world` for components, resources, and stable world state;
- workspace serialization and error crates.

It must not depend on:

- `rne_robot`, `rne_ai`, `rne_sensor`, or application policies;
- `rne_physics`, Rapier, or another physics backend;
- `rne_render`, wgpu, or GPU handles;
- `rne_plateau`, XML/geospatial parsers, or CityGML types;
- ROS 2, adapters, SUMO, OpenDRIVE, or Lanelet2.

Offline importers and asset tooling point inward to `rne_traffic`. Robot,
agent, physics, rendering, and external-simulator integrations consume traffic
state through higher-level orchestration or adapters, preserving an acyclic
dependency graph.

## Entity ownership

- A network root has `TrafficNetworkRoot`.
- A road user has `TrafficActor` in addition to its Robot Entity components
  when it is a simulated vehicle.
- `TrafficRuntime` is per-world deterministic step state.
- Traffic events carry simulation time and stable entity UUIDs.

These foundation types intentionally do not define schema v1. Asset IDs,
topology records, and canonical serialization arrive in the next phase without
coupling the ECS markers to a particular importer.

## Runtime order

1. Apply signal phase and route decisions for the current `SimTime`.
2. Compute actor commands in stable actor-ID order.
3. Apply kinematic or backend-neutral robot commands.
4. Advance physics when a scenario opts into it.
5. Update lane-relative state and detect conflicts/violations.
6. Record a stable world-state hash.
7. Optionally extract render data.

Headless execution uses the same steps and is the acceptance path.

## Stage-one sequence

1. Establish this crate, boundary lint, ECS markers, runtime resource, and
   deterministic actor ordering.
2. Add `.rne.traffic.json` schema v1 and golden serialization.
3. Extend PLATEAU import through LOD2 and LOD3.1 road semantics.
4. Build deterministic topology, tile stitching, turns, and conflicts.
5. Add the headless urban replay and migrate the rendered PLATEAU car example.

## Consequences

- PLATEAU is one source of traffic semantics rather than a runtime dependency.
- Rendering and rigid-body simulation remain optional.
- Large traffic populations can use deterministic kinematics while selected
  Robot Entities opt into full physics.
- External traffic ecosystems require explicit adapters and cannot shape core
  types.
- Source uncertainty remains observable instead of being hidden by import
  heuristics.

Schema v1 is documented in
[`docs/TRAFFIC_ASSET.md`](../TRAFFIC_ASSET.md).

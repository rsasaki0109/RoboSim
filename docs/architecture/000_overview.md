# Architecture Overview

Robot Native Engine (RNE) is a robot-native simulation core written in Rust.

## Core principles

- **Robots are first-class entities**, not plugins.
- **ECS is authoritative** for world state.
- **ROS2 is optional** and lives only in `adapters/`.
- **Headless simulation** is the default path for CI and AI rollouts.

## Crate layers

```
rne_math → rne_core → rne_ecs → rne_world
                              ↘ rne_robot → rne_physics → rne_physics_rapier
                                                        ↘ rne_physics_conformance
                              ↘ rne_traffic
                              ↘ rne_data / rne_sensor / rne_render / rne_ai / rne_assets
rne_plugin_sdk (dependency-free author ABI) → rne_plugin ↔ runner
runner ↔ rne_data::transport ↔ native frontend (versioned framed sensor boundary)
adapters/ros2/* (optional)
tests/compatibility (release-facing typed readers; never a core dependency)
```

`rne_plugin_sdk` owns only dependency-free C-ABI constants, frames, and callback
signatures. `rne_plugin` hosts controller policies, not robot entities. Its
public control schema uses stable robot/joint names, fixed-step integer
timestamps, explicit units, deterministic ordering, and no renderer,
physics-backend, or adapter types. The runner owns capability negotiation and the
`created → configured → active → shutdown` lifecycle; C ABI v2-v3 is an
implementation boundary beneath that typed schema.

`rne_physics_conformance` is downstream of the backend-neutral physics trait
and ECS component contracts. It may construct canonical test worlds, but it
does not add behavior or vendor types to `rne_physics`, and no core crate
depends on it. Independently maintained backend crates use it as authoring and
certification tooling.

`rne_compatibility_suite` is a downstream release/test aggregator. It may
depend on public artifact owners and non-publishable conformance runners, but
no runtime or core crate may depend on it. Its installed binary verifies the
retained release corpus without changing artifact semantics or migrating
evidence in place.

`rne_data::transport` owns production frontend framing, capability negotiation,
typed RGB-D/LiDAR payload codecs, and frame+byte bounded latest-only queues.
Socket threads live in the runner and viewer; fixed-step simulation never
writes to a client, and a disconnect does not become a simulation command.

## Runtime pipeline

1. Control / AI action
2. Pre-physics sync
3. Fixed physics step
4. Post-physics sync
5. Sensor sampling
6. Data recording
7. Optional render extract/submit

See also:

- [Robot Native model](002_robot_native.md)
- [DataBus](005_data_bus.md)
- [Mobile manipulator target](006_mobile_manipulator.md)
- [Web viewer boundary](007_web_viewer.md)
- [Traffic domain](010_traffic_domain.md)

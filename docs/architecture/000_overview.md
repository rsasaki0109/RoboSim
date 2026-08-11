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
                              ↘ rne_traffic
                              ↘ rne_data / rne_sensor / rne_render / rne_ai / rne_assets
runner ↔ rne_plugin (versioned robot-native observation/action boundary)
adapters/ros2/* (optional)
```

`rne_plugin` hosts controller policies, not robot entities. Its public control
schema uses stable robot/joint names, fixed-step integer timestamps, explicit
units, deterministic ordering, and no renderer, physics-backend, or adapter
types. The runner owns capability negotiation and the
`created → configured → active → shutdown` lifecycle; C ABI v2-v3 is an
implementation boundary beneath that typed schema.

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

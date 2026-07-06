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
                              ↘ rne_data / rne_sensor / rne_render / rne_ai / rne_assets
adapters/ros2/* (optional)
```

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

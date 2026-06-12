# AGENTS.md

## Project

Robot Native Engine (RNE) is an open-source robot-native game engine.

Robots are not plugins. The core concepts are:

- World Entity
- Robot Entity
- Sensor Entity
- Actuator Entity
- Agent Entity
- Episode Entity

ROS2 is an adapter only. Do not add ROS2, rclrs, rclcpp, DDS, or ROS message dependencies to core crates.

## Repository Layout

- `crates/rne_core`: app, schedule, time, events, diagnostics
- `crates/rne_math`: vectors, transforms, units, spatial math
- `crates/rne_ecs`: ECS wrapper and shared entity conventions
- `crates/rne_world`: world entity, scene index, frame graph
- `crates/rne_robot`: robot/link/joint/actuator components and systems
- `crates/rne_physics`: physics backend traits only
- `crates/rne_physics_rapier`: Rapier implementation
- `crates/rne_sensor`: sensor traits, specs, outputs, noise models
- `crates/rne_render`: render traits only
- `crates/rne_render_wgpu`: wgpu renderer
- `crates/rne_asset`: asset database and import pipeline
- `crates/rne_data`: typed DataBus, stream IDs, frame payloads
- `crates/rne_ai`: agent, observation, action, reward, policy traits
- `crates/rne_plugin`: plugin manifest and loading interfaces
- `adapters/ros2`: ROS2 adapter. ROS2 dependencies are allowed here only.
- `examples`: runnable examples
- `tests`: integration, determinism, and golden tests
- `docs/adr`: architecture decision records

## Commands

Use these commands before submitting changes:

```bash
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace
cargo test --workspace --features headless
cargo run -p xtask -- ci
```

For a single crate:

```bash
cargo test -p rne_core
cargo clippy -p rne_core --all-targets -- -D warnings
```

## Coding Rules

- Keep core crates ROS2-free.
- Do not introduce global mutable state.
- Do not use wall-clock time inside simulation logic. Use SimClock.
- Do not make rendering required for simulation tests.
- Do not put physics-engine-specific handles in rne_robot or rne_world.
- Do not expose Rapier, MuJoCo, PhysX, or Bullet types through rne_physics public traits.
- All public structs must derive Debug where reasonable.
- Public APIs must have rustdoc.
- All new components go in `components.rs`.
- All new systems go in `systems.rs`.
- All new resources go in `resources.rs`.
- All new events go in `events.rs`.
- Use explicit units in field names: `_m`, `_rad`, `_s`, `_hz`, `_nm`.
- Prefer small immutable data structs.
- Use `unsafe` only with a `// SAFETY:` explanation and a test covering the invariant.
- Any new plugin must include a manifest and a minimal example.
- Any new adapter must not change core types to fit the external system.
- Any new sensor must define timestamp behavior, latency behavior, and noise behavior.
- Any new actuator must define limits and failure behavior.
- Any new feature must include tests and a short docs update.

## Testing Requirements

For each feature, add at least one of:

- Unit test in the crate
- Integration test in `tests/integration`
- Determinism test in `tests/determinism`
- Golden serialization test in `tests/golden`
- Runnable example in `examples`

Simulation logic must be testable in headless mode.

## Determinism Requirements

- Use deterministic ordering for entity iteration where results are externally visible.
- Do not use random numbers without an explicit seed from WorldRandom.
- Replay tests must compare stable hashes of world state.
- Floating point comparisons must use crate-approved tolerances.

## Module Boundary Rules

Allowed dependencies:

- `rne_math` depends on no RNE crate.
- `rne_core` may depend on `rne_math`.
- `rne_ecs` may depend on `rne_core` and `rne_math`.
- `rne_world` may depend on `rne_ecs`, `rne_core`, `rne_math`.
- `rne_robot` may depend on `rne_world`, `rne_ecs`, `rne_math`, `rne_core`.
- `rne_physics` may depend on `rne_robot`, `rne_world`, `rne_ecs`, `rne_math`.
- `rne_physics_rapier` may depend on `rne_physics` and Rapier.
- `rne_sensor` may depend on `rne_physics`, `rne_render`, `rne_data`.
- `rne_render_wgpu` may depend on `rne_render` and wgpu.
- `adapters/*` may depend on external ecosystems such as ROS2.

Forbidden:

- Core crates must not depend on `adapters/*`.
- Core crates must not depend on ROS2.
- `rne_robot` must not depend on Rapier, MuJoCo, PhysX, or Bullet.
- `rne_sensor` must not require a renderer unless the specific sensor is camera-like.

## Pull Request Definition of Done

A PR is done when:

- Code is formatted.
- Clippy passes with `-D warnings`.
- Tests pass.
- New public APIs have rustdoc.
- Architecture docs are updated if boundaries changed.
- A runnable example or test demonstrates the behavior.
- No forbidden dependency was added.

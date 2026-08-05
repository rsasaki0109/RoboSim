# OSS parity baseline

RNE is not trying to clone one simulator's GUI or file format. “Equivalent to
existing OSS” means that a contributor can describe a world, attach a robot and
sensors, run a fixed-step experiment, connect a controller, inspect the result,
and replay it without a commercial engine or a renderer.

The comparison baseline is deliberately workflow-oriented:

| Reference | Capabilities used as the baseline |
|---|---|
| [Choreonoid](https://www.choreonoid.org/en/documents/latest/simulation/concept.html) | World/body models, replaceable physics simulator items, vision sub-simulators, controller input/output, replay, and plugins |
| [Gazebo Sim](https://gazebosim.org/docs/latest/architecture/) | Entity/system simulation loop, physics separation, sensor systems with noise, GUI/transport plugins, and reusable worlds |
| [AWSIM](https://github.com/RobotecAI/AWSIM-AWF) and [Scenario Simulator v2](https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/ZeroMQ/) | Vehicle/sensor/environment/ROS 2 integration, scenario-driven execution, traffic co-simulation, and simulator-independent sensor APIs |

## Current RNE surface

| Capability | RNE today | Remaining parity gap |
|---|---|---|
| World and robot assets | `.rne.scene.toml`, `.rne.robot.toml`, URDF, OBJ, static glTF/GLB, PLATEAU import | SDF/MJCF/OpenSCENARIO import is not yet available |
| Fixed-step execution | `rne-asset simulate` and `rne-asset run` run a scene headlessly with an explicit rate and step count | Interactive pause/reset controls are still needed |
| Controller I/O | Typed `ActuatorCommand`, named joint velocity/effort and wheel paths, episode APIs, and an isolated ROS 2 adapter | Multi-joint trajectories and policy callbacks are still application-level APIs |
| Physics | Backend-neutral traits with Rapier rigid bodies, joints, articulation, contacts, and deterministic hashes | A second open backend and a public capability negotiation workflow are future work |
| Sensors | LiDAR, IMU, RGB-D/camera, wheel encoders, noise, latency, DataBus, and per-step replay stream summaries | Full typed payload export and a manifest-level sensor subscription list are future work |
| Rendering | Native wgpu, browser viewer, PBR materials, glTF maps, HDR/IBL, TAA | The renderer is not yet a first-class frontend of the headless runner |
| Scenario and traffic | Typed behavior contracts, deterministic traffic routing/signals, PLATEAU assets, multi-seed reports | OpenSCENARIO and external traffic-simulator adapters are future work |
| Replay and evaluation | Episode logs, stable hashes, vectorized checkpoints, behavior CI, JUnit/JSON reports, tagged wheel/joint `.rne-replay` actions, joint-state/sensor summaries, and browser interval inspection | Richer contact/failure annotations and full sensor payload streams are future work |
| Extension model | Backend-neutral traits and plugin manifests/interfaces | Runtime discovery/loading and a stable plugin ABI are future work |

## Delivered first slice

The first parity slice is the headless fixed-step runner and its versioned
manifest:

```bash
cargo run --release -p rne_asset_cli -- run \
  assets/runs/mesh_diff_drive.rne.run.toml
```

The manifest pins the scene reference, optional seed, fixed clock, controller,
determinism check, and optional replay output path. The controller can target
every matching URDF/ECS joint by name:

```bash
cargo run --release -p rne_asset_cli -- run \
  assets/runs/mm_minimal_joint_velocity.rne.run.toml
```

The runner applies typed wheel, named joint-velocity, or named joint-effort
commands through the actuator buffer, advances Rapier at a fixed simulation
rate, and records named joint state plus DataBus sensor stream count/sequence/
payload digests after each step. `determinism_check = true` repeats the complete
run and requires the final report, observations, and per-step physics hashes to
match. The command is headless; no GPU or ROS 2 installation is needed.

The direct form remains useful for one-off overrides:

```bash
cargo run --release -p rne_asset_cli -- simulate \
  assets/scenes/mesh_diff_drive.rne.scene.toml \
  --steps 600 --hz 60 --wheel-velocity-rad-s 6 \
  --determinism-check \
  --replay-out target/runs/mesh_diff_drive.rne-replay

# Re-run the recorded action schedule and verify every frame hash/observation
cargo run --release -p rne_asset_cli -- replay \
  target/runs/mesh_diff_drive.rne-replay
```

The `.rne-replay` file is versioned JSON. Version 1 now records tagged
wheel/joint actions while remaining able to read the original flat wheel-action
form. Each frame can also contain the selected base translation, deterministic
named joint state, sensor stream summaries, the physics hash, and the final
report. Replay comparison uses exact step/time/hash checks and a documented
`1e-12` relative-scale tolerance for floating-point observations.

The [web viewer](../web/rne_web_viewer/README.md) can inspect the same artifact
without rerunning the recorded policy or physics. Select the file in its Replay
inspector to scrub or play the interval and inspect the selected observation and
exact 64-bit physics and sensor payload hashes.

## Next parity order

1. Add full typed sensor payload export and manifest-level sensor subscriptions.
2. Add richer contact/failure annotations to the replay report.
3. Add a minimal OpenSCENARIO/traffic adapter after the native run/replay
   contract is stable.

This order closes the common simulator workflow first. Photoreal rendering,
large asset libraries, and GPU-scale parallelism remain separate capabilities,
not prerequisites for headless behavior testing.

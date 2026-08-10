# OSS workflow parity matrix

This matrix defines “equivalent to existing OSS” for RNE. The target is not
feature-count parity or a copy of another simulator's file format. A workflow
is at parity when a contributor can author it, run it headlessly, connect a
controller, inspect its result, and replay it with a stable acceptance result.

The reference workflows are deliberately complementary:

- [Choreonoid](https://choreonoid.org/en/manuals/latest/index.html): body
  models, controllers, devices/sensors, simulation, and replay.
- [Gazebo Sim](https://gazebosim.org/docs/latest/architecture/): an ECS
  simulation server, replaceable systems, plugins, transport, and a separate
  frontend.
- [AWSIM](https://github.com/autowarefoundation/AWSIM): an extensible,
  layered digital-twin simulator for vehicle, sensor, environment, and ROS 2
  workflows.
- [Scenario Simulator v2](https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/About/):
  scenario-driven traffic and sensor testing with simulator and scenario
  format boundaries.

## Acceptance rule

Every row must have all five pieces before it is marked **parity**:

1. A documented command or API path.
2. A runnable fixture or example.
3. A headless test that exercises the same simulation path.
4. A stable report, replay artifact, or hash that can fail CI.
5. A short user-facing document explaining the workflow.

“Partial” means the native RNE path works but an important reference workflow
is still missing. “Gap” means the capability is visible in the architecture
but a user cannot yet complete the workflow end to end.

## Current matrix

| Workflow | RNE proof point | Status | Parity gate |
|---|---|---|---|
| World and robot authoring | `.rne.scene.toml`, `.rne.robot.toml`, URDF, SDF, MJCF, OBJ, glTF/GLB, and PLATEAU import | Parity for the current asset slice | Load a fixture and report stable entity/model metadata |
| Fixed-step simulation | `rne-asset run` and `simulate` with explicit `SimClock` rate and step count | Parity | The same manifest produces the same final report and frame hashes |
| Controller and actuator I/O | Differential drive, named joint velocity/effort, trajectories, controller plugins, and C ABI discovery | Parity for the current controller slice | Controller commands appear in replay frames and affect the expected actuator state |
| Sensor simulation | LiDAR, IMU, RGB-D/camera, wheel encoders, noise, latency, DataBus, and typed payload subscriptions | Parity for the current sensor slice | Payload stream summaries and selected payloads replay with stable values |
| Physics selection | Rapier backend, analytic backend, capability negotiation, contact/failure reports | Parity for the current backend slice | A manifest selects a backend and records deterministic contact/failure data |
| Scenario authoring | OpenSCENARIO 1.0 subset, parameters, catalogs, speed, lane change, assigned route, signals, controlled fixed-step execution, and versioned scenario replay artifacts | Parity for the current scenario slice | A scenario manifest writes a `rne-scenario-replay` JSON record with engine compatibility and XOSC/network digests; `rne-asset replay` verifies those exact inputs before re-execution and final-result comparison |
| Native traffic runtime | Routing, signals, reservations, deterministic kinematics, flow metrics, and replay | Parity for the native traffic slice | Spawn-order-independent fleet hash, no forbidden gaps/collisions, stable metrics |
| SUMO road import | `rne-asset sumo-net` and direct `.net.xml` scenario networks | Parity for the import slice | Imported topology and signal wiring match the committed fixture expectations |
| External traffic co-simulation | `rne_traci::CoSimulation`, live vehicle mirroring, stable hash, explicit `TrafficPoseSource::External`, and opt-in `set_vehicle_speed_m_s` command return | Partial | Observe/synchronize and explicit SUMO speed control are deterministic; route/control ownership beyond speed remains future work |
| Runner control and remote inspection | stdin/TCP `pause`, `resume`, `step`, `reset`, `quit`, protocol-v1 live robot/traffic/sensor status snapshots, opt-in safety-capped source-resolution RGB-D TCP payloads, versioned command transcripts, and scenario result artifacts | Parity for the current runner-control slice | Process-level TCP E2E verifies source-resolution RGB/depth dimensions and byte lengths; absolute image/status limits prevent unbounded writes, while controlled artifacts replay consumed commands including reset/quit |
| Frontend and rendering | Native wgpu renderer, browser replay inspector, TCP frontend protocol, and `interactive_viewer --connect` projection for diff-drive, scenario traffic, generic URDF joints, RGB camera previews, GPU depth PiP, and LiDAR points | Partial | Bounded and opt-in full-resolution RGB/depth PiP are covered; binary video transport and a GPU-independent TCP-to-render smoke remain future work |
| Extension and integration | Backend-neutral traits, plugin manifests/C ABI, isolated ROS 2 adapter, external importer boundaries | Parity for the current extension slice | Add/remove an integration without changing core simulation types |
| CI and evaluation | `xtask parity`, stable hashes, robot and scenario replay artifacts, JSON/JUnit reports, determinism tests | Partial | The current catalog is green for robot/scenario replay/sensor/traffic/control, RGB-D TCP E2E, and frontend compile/remote-sensor projection slices; broader frontend smoke remains |

## Flagship workflows

These are the first end-to-end scenarios that every future frontend or transport
must preserve.

### Robot control and replay

```text
assets/runs/mesh_diff_drive.rne.run.toml
  → fixed-step differential-drive run
  → determinism check
  → target/runs/mesh_diff_drive.rne-replay
  → replay verification
```

### Sensor payload and replay

```text
assets/runs/mesh_diff_drive_lidar_payload.rne.run.toml
  → typed LiDAR payload subscription
  → per-frame stream/hash recording
  → replay verification
```

### Scenario and traffic

```text
assets/runs/sumo_cross.rne.run.toml
  → OpenSCENARIO 1.0 import
  → SUMO network import and route execution
  → signal/traffic metrics
  → deterministic final report
```

### Interactive runner control

```text
pause → step N → reset → step M → quit
  → TCP or stdin control transcript
  → replay artifact and status snapshots
  → rne-asset replay verifies the same final episode
```

## Delivery order

1. Keep this matrix and the flagship fixtures green.
2. Complete the scenario condition/metric surface and broaden frontend status
   projection to binary video transport and renderer-native sensor payloads.
3. Add explicit external-simulator control modes only where motion ownership is
   testable and deterministic; speed control is the first delivered SUMO path.
4. Extend `xtask parity` with the remaining GPU-independent frontend smoke
   transcript.
5. After parity, use deterministic replay, backend neutrality, and the Rust
   plugin ABI as RNE's differentiation axes.

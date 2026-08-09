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
| World and robot assets | `.rne.scene.toml`, `.rne.robot.toml`, URDF, OBJ, static glTF/GLB, PLATEAU import, minimal OpenSCENARIO 1.0 import, and minimal SDF (`rne_sdf`) and MJCF (`rne_mjcf`) model import → URDF | None for the current workflow slice |
| Fixed-step execution | `rne-asset simulate` and `rne-asset run` run a scene headlessly with an explicit rate and step count. `rne-asset run --control-stdin` accepts runner commands on stdin (`pause`, `resume`, `step N`, `reset`, `quit`) through a `rne_core` transport-neutral control state machine; `--control-port PORT` serves the same commands over a local TCP connection with live per-step observation snapshots for a GUI/frontend. `reset` rebuilds the world from the episode's initial conditions and `step N` advances exactly N frames before pausing again | No windowed renderer/GUI transport yet; the control protocol is line-based over stdin or TCP |
| Controller I/O | Typed `ActuatorCommand`, named joint velocity/effort/wheel paths, interpolated multi-joint position trajectories in run manifests, controller plugins invoked through a `rne_plugin` trait boundary (`[controller] kind = "plugin"`), episode APIs, and an isolated ROS 2 adapter | Application-level controllers still live in the code, not a loadable ABI |
| Physics | Backend-neutral traits with Rapier (full contacts, articulation, contact force) and an analytic deterministic backend (`rne_physics_analytic`, collision-free), selectable per run manifest with a public capability negotiation workflow (`[physics] backend` + `required_capabilities`) | None for the current workflow slice |
| Sensors | LiDAR, IMU, RGB-D/camera, wheel encoders, noise, latency, DataBus, per-step replay stream summaries, and full typed payload export with manifest-level sensor subscriptions | None for the current workflow slice |
| Rendering | Native wgpu, browser viewer, PBR materials, glTF maps, HDR/IBL, TAA | The renderer is not yet a first-class frontend of the headless runner |
| Scenario and traffic | Typed behavior contracts, deterministic traffic routing/signals, PLATEAU assets, multi-seed reports, minimal OpenSCENARIO 1.0 scenario execution (importer → versioned document → traffic runtime with parameter substitution, speed, lane-change, and assigned-route actions, vehicle catalogs, and network signal timing, wired into run manifests), offline SUMO `.net.xml` road-network import (`rne_sumo` converts SUMO lanes into the RNE Y-up frame, derives junctions/connections deterministically, and imports `connection` + `tlLogic` fixed-time signal programs matched to the derived connections), and scenario runs that reference a `.net.xml` road network directly (`rne-asset run` imports and routes on the fly; the `sumo_cross` fixture drives a vehicle through the imported intersection and the `signalized_cross` fixture holds it at a red stop line) | Live SUMO co-simulation (TraCI) is future work |
| Replay and evaluation | Episode logs, stable hashes, vectorized checkpoints, behavior CI, JUnit/JSON reports, tagged wheel/joint `.rne-replay` actions, joint-state/sensor summaries, per-step contact statistics, fall/failure annotations in the final report, and browser interval inspection | Full sensor payload streams for every sensor are opt-in via subscriptions |
| Extension model | Backend-neutral traits, plugin manifests/interfaces (`rne_plugin`), a controller-plugin boundary invoked by the runner, dynamic loading of controller plugins from shared libraries through a versioned C ABI (`rne_plugin::load_controller_library`), name-based runtime discovery (`rne_plugin::discover_controller_plugin`, or `[controller] plugin_paths` in a run manifest) with a built-in fallback, and authoring tooling (`rne-asset plugin new` scaffolds a compilable `cdylib` controller-plugin crate plus a manifest; `rne-asset plugin list` enumerates built-in and discoverable plugins) | None for the current workflow slice |

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

Run manifests can also request full typed sensor payload capture. A
`[[sensors]]` subscription selects sensors by entity name or kind, and the
recording then stores the complete IMU, LiDAR, camera (RGB+D), or wheel-encoder
payload for each frame in addition to the stream summaries:

```toml
[[sensors]]
kind = "lidar"

[[sensors]]
name = "wrist_camera"
```

A `joint_trajectory` controller drives named joints through time-indexed
position waypoints; the runner interpolates them per fixed step and records the
interpolated targets as the frame action:

```toml
[controller]
kind = "joint_trajectory"

[[controller.joint_trajectories]]
joint = "shoulder_joint"
waypoints = [
    { t_s = 0.0, position_rad = 0.0 },
    { t_s = 0.5, position_rad = 1.0 },
    { t_s = 1.0, position_rad = 0.0 },
]
```

```bash
cargo run --release -p rne_asset_cli -- run \
  assets/runs/mesh_diff_drive_lidar_payload.rne.run.toml

# Re-run and verify the recorded actions, observations, and every frame hash
cargo run --release -p rne_asset_cli -- replay \
  target/runs/mesh_diff_drive_lidar_payload.rne-replay
```

The typed payloads are JSON-encoded in the artifact, so the browser inspector and
other tools can read them without rerunning the sensor models.

Every frame also records contact statistics (active pair count and summed/max
normal impulse per step) and the final report annotates the run outcome: the
maximum concurrent contact pairs, the largest per-step contact impulse, the
minimum base height of the first differential-drive robot, and a `fell` failure
when that height drops below half of its initial value.

## Next parity order

1. Add a minimal OpenSCENARIO/traffic adapter after the native run/replay
   contract is stable. Delivered: the `rne_openscenario` importer parses a
   strict OpenSCENARIO 1.0 subset into a versioned `.rne.scenario.json`
   document, its executor drives the document over the traffic runtime, and
   run manifests can reference the scenario through `[scenario] xosc = ...` so
   `rne-asset run` executes it headlessly. Lane changes are supported as a
   `RelativeTargetLane` action that switches the actor to a synthetic parallel
   route, `AssignRouteAction` follows scripted waypoints, `${parameter}`
   references resolve from `ParameterDeclarations`, `CatalogReference` entities
   resolve from `VehicleCatalog` directories, and the network's fixed-time
   signal programs drive stop lines during the run.

This order closes the common simulator workflow first. Photoreal rendering,
large asset libraries, and GPU-scale parallelism remain separate capabilities,
not prerequisites for headless behavior testing.

## SUMO road-network import

`rne-sumo`-powered `rne-asset sumo-net` converts a SUMO `.net.xml` road network
into a `.rne.traffic.json` traffic asset. The importer maps `edge`/`lane`
geometry (2D `x,y` SUMO coordinates) into the RNE Y-up frame
(`[x, z, -y]`), converts `allow`/`disallow` road-user classes, and then lets
`rne_traffic::build_traffic_topology` deterministically derive junctions and
lane connections from the lane endpoints:

```bash
cargo run --release -p rne_asset_cli -- sumo-net \
  assets/networks/minimal_cross.net.xml \
  --out target/runs/minimal_cross.rne.traffic.json
```

Internal/connector edges are skipped, malformed input is rejected, and the
fixture network derives a four-way junction with seven movements. A run
manifest can reference the `.net.xml` directly as its OpenSCENARIO `LogicFile`:
`rne-asset run` imports the network (deriving topology) and routes the scenario
actors on it. `assets/runs/sumo_cross.rne.run.toml` spawns a vehicle on the
eastbound approach and drives it toward the intersection (206 m route, 10 m/s,
no collisions, deterministic).

`connection` and `tlLogic` elements are imported too: the `signalized_cross`
fixture assigns a fixed-time program (20 s green northbound, then 15 s green
eastbound), and the importer matches each `linkIndex` to the derived
connection with the same `(incoming, outgoing)` lane pair, building one RNE
`TrafficSignal` group per link and one phase per parsed phase. The signal
drives RNE stop-line control: a scenario actor on the eastbound approach is
held at the red stop line without violating the signal. Live SUMO
co-simulation (TraCI) remains future work.

## Runner control

`rne-asset run --control-stdin` drives a manifest interactively (or from a
script) without a GUI. The runner consults a transport-neutral control state
machine (`rne_core::control`) at every fixed-step boundary:

```bash
printf 'pause\nstep 5\nreset\nstep 3\nquit\n' | \
  cargo run --release -p rne_asset_cli -- run \
    assets/runs/mesh_diff_drive.rne.run.toml --control-stdin
```

- `pause` suspends the loop at the next step boundary; the runner blocks until
  the next command.
- `resume` continues advancing freely.
- `step N` advances exactly `N` frames, then pauses again. A step budget is
  never interrupted by queued commands, so scripts stay ordered.
- `reset` rebuilds the world from the episode's initial conditions (same seed)
  and restarts from step 0. The final report and replay artifact describe the
  current episode.
- `quit` ends the run gracefully; the current episode is still reported and
  written to the replay artifact.
- stdin EOF while paused is treated as `quit`, so piped scripts terminate
  deterministically.

The runner starts paused and waits for the first command, so a piped script
like `step N\nquit` advances exactly `N` frames regardless of timing.

For a GUI or renderer frontend, `--control-port PORT` serves the same commands
over a local TCP connection (port 0 picks an ephemeral port). On connect the
runner sends `ready paused`, acknowledges each command with `ok <state>`, and
streams `status step=<n> t=<t> state=<state> snapshot=<json>` after every
completed step. The snapshot is a compact single-line JSON observation
(`base`, `joints`, `sensors`), so a frontend can render the live state without
re-running physics. The test suite drives the binary over TCP and verifies the
reply protocol, the snapshot, and the resulting replay.

`--replay-out PATH` overrides the manifest's replay path, and determinism
re-checks are skipped in interactive mode.

## Dynamically loaded controller plugins

`[controller] kind = "plugin"` loads a controller from a shared library when the
manifest sets `library`:

```toml
[controller]
kind = "plugin"
plugin = "velocity_servo"
library = "../../target/debug/rne_plugin_example_velocity_servo.so"
joint = "shoulder_joint"
target_rad = 1.0
gain = 2.0
max_velocity_rad_s = 5.0
```

The host and the plugin communicate through a versioned C ABI
(`rne_plugin::cabi`, currently ABI version 2): the plugin exports
`rne_plugin_abi_version`, `rne_plugin_name`, `rne_controller_create`,
`rne_controller_destroy`, and `rne_controller_step`, and all data crosses the
boundary as `#[repr(C)]` values and NUL-terminated UTF-8 strings. A plugin
whose ABI version differs is rejected at load time.
`rne_plugin_example_velocity_servo` is the minimal reference implementation and
produces identical velocity commands to the built-in `VelocityServoController`;
a determinism test drives a scene with the loaded library and the built-in
implementation and requires byte-identical replay frames.

Plugins can also be discovered by name instead of an explicit path. A manifest
lists directories to search, and the runner loads the first shared library
whose file name contains the requested plugin name and whose `rne_plugin_name`
matches, falling back to the built-in registry when none does:

```toml
[controller]
kind = "plugin"
plugin = "velocity_servo"
plugin_paths = ["../../target/debug"]
joint = "shoulder_joint"
target_rad = 1.0
gain = 2.0
max_velocity_rad_s = 5.0
```

## Plugin authoring

`rne-asset plugin new` scaffolds a complete, compilable controller-plugin crate
implementing the C ABI, with a versioned `rne-plugin.json` manifest:

```bash
cargo run --release -p rne_asset_cli -- plugin new my_controller --dir plugins
cd plugins/my_controller && cargo build
```

The scaffolded crate starts as a velocity-servo policy (so it compiles and
loads immediately); replace `rne_controller_step` with your controller.
`rne-asset plugin list --path <dir>` enumerates the built-in `velocity_servo`
and any discoverable plugin libraries in the given directories. An
end-to-end test scaffolds, builds, and loads a plugin from the generated
source.

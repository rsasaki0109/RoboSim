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
| [AWSIM](https://github.com/autowarefoundation/AWSIM) and [Scenario Simulator v2](https://tier4.github.io/scenario_simulator_v2-docs/developer_guide/About/) | Vehicle/sensor/environment/ROS 2 integration, scenario-driven execution, traffic co-simulation, and simulator-independent sensor APIs |

## Current RNE surface

The detailed, executable acceptance criteria live in the
[OSS workflow parity matrix](OSS_PARITY_MATRIX.md). This page remains the
narrative status report; the matrix is the gate for deciding whether a
workflow is truly complete.

| Capability | RNE today | Remaining parity gap |
|---|---|---|
| World and robot assets | `.rne.scene.toml`, `.rne.robot.toml`, URDF, OBJ, static glTF/GLB, PLATEAU import, minimal OpenSCENARIO 1.0 import, and minimal SDF (`rne_sdf`) and MJCF (`rne_mjcf`) model import → URDF | None for the current workflow slice |
| Fixed-step execution | `rne-asset simulate` and `rne-asset run` run a scene or OpenSCENARIO manifest headlessly with an explicit rate and step count. `rne-asset run --control-stdin` accepts runner commands on stdin (`pause`, `resume`, `step N`, `reset`, `quit`) through a `rne_core` transport-neutral control state machine; legacy `--control-port PORT` remains supported. Production `--frontend-port PORT` uses explicit binary negotiation, finite socket deadlines, bounded queues, and reconnect-without-quit semantics. `reset` rebuilds the world from the episode's initial conditions and `step N` advances exactly N frames before pausing again | None for the current fixed-step and runner-control slice |
| Controller I/O | Typed `ActuatorCommand`, named joint velocity/effort/wheel paths, interpolated multi-joint position trajectories in run manifests, controller plugins invoked through a `rne_plugin` trait boundary (`[controller] kind = "plugin"`), dynamically loaded controller libraries through the versioned C ABI, episode APIs, and an isolated ROS 2 adapter | None for the current controller slice; sensor and agent plugin ABIs remain future scope |
| Physics | Backend-neutral traits with Rapier, a deterministic collision-free analytic backend, and a default-off MuJoCo 3.9 backend for rigid bodies, hinge/slide articulation, position/velocity/effort actuation, canonical contact force, and sensor overlaps; selectable per run manifest with public capability negotiation plus an executable canonical conformance report | MuJoCo raycasts and broader constraint/geometry coverage remain future scope; every currently advertised capability has unit-bearing evidence |
| Sensors | LiDAR, IMU, RGB-D/camera, wheel encoders, noise, latency, bounded DataBus retention, per-step replay stream summaries, full typed payload export, and framed lossless RGB-D/LiDAR frontend payloads preserving stream sequence/capture/availability timestamps | None for the current workflow slice |
| Rendering | Native wgpu, browser replay viewer, PBR materials, glTF maps, HDR/IBL, TAA, legacy `interactive_viewer --connect`, and production `interactive_viewer --frontend-connect` | Remote diff-drive, scenario traffic, and articulated joints project locally; production binary RGB/depth frames drive PiP and full LiDAR frames drive a bounded display overlay without stepping a second physics world |
| Scenario and traffic | Typed behavior contracts, deterministic traffic routing/signals, PLATEAU assets, multi-seed reports, multi-actor OpenSCENARIO 1.0 execution with per-kind routes and canonical action evidence, mixed runtime/external ownership metrics, versioned scenario replay, offline SUMO import, and recoverable live TraCI co-simulation | None for the M5 scenario-scale and TraCI recovery slice; broader OpenSCENARIO action coverage remains future scope |
| Replay and evaluation | Episode logs, stable hashes, vectorized checkpoints, behavior CI, JUnit/JSON reports, tagged wheel/joint `.rne-replay` actions, `rne-scenario-replay` XOSC/network/clock/result records, joint-state/sensor summaries, per-step contact statistics, fall/failure annotations in the final report, and browser interval inspection | Full sensor payload streams for every sensor are opt-in via subscriptions |
| Extension and compatibility model | Dependency-free controller author ABI (`rne_plugin_sdk`), host-side manifests/interfaces (`rne_plugin`), dynamic loading through a versioned C ABI, versioned offline controller and accelerator scaffolding, standalone plugin conformance (`rne-asset plugin check`), standalone accelerator process conformance (`rne-accelerator-conformance`), publishable external physics-backend conformance (`rne_physics_conformance`), content-addressed external hardware-process conformance (`rne-hardware-conformance`), a fixed 31-crate Rust API baseline, and an installed thirty-six-fixture corpus including both scaffold contracts and the plugin-conformance reader, distinct historical/current TaskSpec identities, TaskSpec-bound hardware-session evidence, all ten frontend protocol-v1 message families, renderer-capture evidence, provenance-bound state migration, historical checkpoint/replay decisions, and exact dataset/Failure Capsule ancestor retention | Independent third-party plugin and backend/adapter certification, future schema transitions, and the six-month stability window remain external v0.9/1.0 gates |

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

OpenSCENARIO manifests can write a deterministic scenario replay artifact just
like robot manifests. `assets/runs/scenario_speed.rne.run.toml` enables the
output by default:

```bash
cargo run --release -p rne_asset_cli -- run \
  assets/runs/scenario_speed.rne.run.toml

cargo run --release -p rne_asset_cli -- replay \
  target/runs/scenario_speed.rne-replay
```

The JSON discriminator is `rne-scenario-replay`. Schema version 4 records the
XOSC path, resolved traffic-network path, fixed-step options, the consumed
`pause`/`resume`/`step`/`reset`/`quit` command transcript, completed step count,
UUID-ordered actor state, canonical action evidence and result digest, mixed
pose ownership, minimum gap, signal/collision metrics, average speed, and
stable hash. Interactive artifacts are replayable too: `rne-asset replay` restores the
initial paused state, feeds the recorded commands back through the same
transport-neutral runner, and compares the final result.

The committed flagship checks can be run as one machine-readable catalog:

```bash
cargo run -p xtask -- parity
```

The default report is `artifacts/oss-parity/report.json` (ignored generated
output). CI uploads the same report so a failed workflow identifies the exact
robot, controller ABI/multi-robot ordering, physics, sensor, scenario, traffic,
or runner-control slice that regressed. Physics evidence is also written to
`artifacts/physics-conformance/report.json` by:

```bash
cargo run -p xtask -- physics-conformance
```

The catalog currently contains eight cases. Analytic covers both advertised
capabilities (2/2), and Rapier covers all five (5/5). Same-backend repeat runs
use canonical snapshot hashes; analytic-vs-Rapier and contact-rich checks use
the versioned SI-unit tolerance registry. The report also exposes Rapier's
current effective-mass convention: configured body mass is additional to the
default-density collider mass.

M5 scenario/traffic evidence is emitted by the release-mode scale gate:

```bash
cargo run -p xtask -- scenario-scale
```

The committed 2.5 km fixture expands one multi-actor event to 100 canonical
actions and runs 600 fixed steps. After an untimed warm-up, three repetitions
must reproduce the actor/action digest exactly; the slowest must sustain at
least 60 steps/s on the GitHub-hosted Windows parity runner. The JSON report at
`artifacts/scenario-scale/report.json` includes ownership counts, minimum gap,
all six violation-registry rows, timing samples, and zero unexplained
violations.

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
held at the red stop line without violating the signal.

## Live SUMO co-simulation

`rne_traci` is a minimal TraCI (Traffic Control Interface) client for driving
and observing a running SUMO process: it negotiates the API version, advances
simulation steps, lists vehicles, and reads their positions using SUMO's
big-endian TCP framing. Positions map into the RNE Y-up frame (`[x, 0, -y]`),
matching `rne_sumo`'s import frame, so SUMO vehicles land on the same network
geometry RNE imports. `rne_traci::CoSimulation` mirrors every live SUMO
vehicle into the RNE ECS as a `TrafficActor` with a `TrafficPose` and
`TrafficPoseSource::External`, so RNE sensors, logging, and rendering can
observe live SUMO traffic without the backend-neutral runtime overwriting its
poses. RNE-owned traffic actors can still run through the same runtime and
World.
`rne-asset co-sim <net.xml> --routes <rou.xml> --steps N` runs a headless SUMO
co-simulation (spawning SUMO, connecting over TraCI, stepping, and mirroring
vehicles) and reports a deterministic stable hash; `--determinism-check` runs
it twice and requires identical outcomes. CI installs `eclipse-sumo` so these
co-simulation tests run against a real SUMO process with a moving vehicle
(`sumo_cross_flow.rou.xml`). `CoSimulation::set_vehicle_speed_m_s` is the first
explicit control-return path: it sends a SUMO `vehicle.setSpeed` command only
when the caller asks for it, while SUMO remains the motion and routing
authority. The default `step` path is still observe-and-synchronize and never
derives commands from an RNE mirror pose; route assignment and richer control
ownership remain future work.

`CoSimulation` exposes connected, disconnected, and closed states. I/O or
protocol failure preserves the last complete mirror; bounded `recover` opens a
fresh endpoint connection and reads a complete sorted snapshot before commit,
without deliberately repeating the ambiguous `simulationStep`. Existing SUMO
IDs retain entity identity, additions/removals reconcile transactionally, and
session metrics expose failed steps, reconnect attempts, and successful
recoveries.

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

Scenario manifests use the same control flags. Their status snapshot contains
`positions_m`, `signal_violations`, `collisions`, `stable_hash`, and
`average_speed_m_s`; scenario reset and quit return the current partial episode
without running the non-interactive determinism re-check. A scenario
`--replay-out` record includes every command consumed by the session and can be
verified with the same `rne-asset replay` command.

For a GUI or renderer frontend, `--control-port PORT` serves the same commands
over a local TCP connection (port 0 picks an ephemeral port). On connect the
runner sends `ready paused protocol=1`, acknowledges each accepted command with
`ok <state>`, and
streams `status step=<n> t=<t> state=<state> snapshot=<json>` after every
completed step. The snapshot is a compact single-line JSON observation
(`base`, `base_yaw_rad`, `joints`, `sensors`, or scenario `positions_m` and
traffic metrics), so a frontend can render the live state without re-running
physics. The native wgpu frontend connects with:

Command acknowledgement is a queue-acceptance boundary; the following status
line is the applied-state boundary. TCP writes use a finite timeout. RGB-D
payloads are always bounded (1920x1080 per image even in opt-in source mode),
and snapshots above 32 MiB are replaced with a compact
`snapshot_limit_exceeded` status instead of an unbounded write.

```bash
cargo run -p interactive_viewer --example 14_interactive_viewer -- \
  --connect 127.0.0.1:9000 \
  assets/scenes/mesh_diff_drive.rne.scene.toml
```

Start the runner first with `rne-asset run ... --control-port 9000`; the viewer
starts attached and initially paused. Space toggles pause/resume, `N` advances
one frame, `T` advances ten frames, `R` resets, and Escape sends `quit`. The
viewer renders the local scene geometry while applying the runner's reported
pose, so the runner remains the only physics authority. For generic URDF
profiles, joint snapshots update only render-space link transforms from the
authored joint origins; the viewer never steps its local Rapier world while
connected. The test suite drives the binary over TCP and verifies the reply
protocol, the snapshot, and the resulting replay record.

`--replay-out PATH` overrides the manifest's replay path, and determinism
re-checks are skipped in interactive mode. The native viewer also accepts
scenario traffic snapshots and generic URDF joint snapshots:

```bash
# Scenario status: the first positions_m actor is projected into the local
# diff-drive scene for a lightweight traffic view.
cargo run -p interactive_viewer --example 14_interactive_viewer -- \
  --connect 127.0.0.1:9000 \
  assets/scenes/mesh_diff_drive.rne.scene.toml

# Articulated runner status: joint names/positions update render-only URDF
# link transforms without stepping a second physics simulation.
cargo run -p interactive_viewer --example 14_interactive_viewer -- \
  --connect 127.0.0.1:9000 --so101
```

The production frontend path is a separate framed binary protocol, so legacy
line clients keep working unchanged. Start the runner and native viewer with:

```bash
cargo run -p rne_asset_cli -- run \
  assets/runs/mm_minimal_joint_velocity.rne.run.toml \
  --frontend-port 9001

cargo run -p interactive_viewer --example 14_interactive_viewer -- \
  --frontend-connect 127.0.0.1:9001 --so101
```

The client sends its supported version range, capabilities, maximum payload,
and frame/byte queue budgets before any live data is sent. The server responds
with the effective contract or an explicit bounded rejection. Status remains a
compact JSON metadata payload inside a binary frame; RGB8, depth-f32 metres,
and LiDAR arrays use typed binary frames with stream sequence and simulation
capture/availability ticks.

The simulation thread only performs non-blocking latest-only queue publication.
Dedicated threads own finite-deadline socket I/O. Reliable acknowledgements
never evict one another; an over-budget client is disconnected instead. Sensor
and status frames replace stale frames by stable key, emit gap accounting, and
are discarded on disconnect. A later client reconnects to the same session and
resumes at current state without an offline backlog or an implicit `quit`.

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
(`rne_plugin::cabi`). The current ABI is v3 and the supported range is v2-v3.
Every plugin exports the v2 base symbols (`rne_plugin_abi_version`,
`rne_plugin_name`, `rne_controller_create`, `rne_controller_destroy`, and
`rne_controller_step`). ABI v3 additionally exports capability negotiation,
configure/reset/shutdown lifecycle hooks, and a robot-scoped fixed-step call.
All data crosses the boundary as `#[repr(C)]` values, integer masks/timestamps,
and NUL-terminated UTF-8 strings. Versions outside the supported range and
missing version-required symbols are rejected at load time.
`rne_plugin_example_velocity_servo` is the minimal reference implementation and
produces identical velocity commands to the built-in `VelocityServoController`;
the frozen `rne_plugin_legacy_v2_fixture` proves the oldest ABI still loads in
the newest runtime. Runner tests also require byte-identical replay frames for
loaded versus built-in controllers and spawn-order-independent robot-scoped
actions for a two-robot scene.

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
cargo run --release -p rne_asset_cli -- plugin new my_controller --dir plugins --schema 1
cd plugins/my_controller && cargo build
```

The scaffolded crate starts as an ABI-v3 velocity-servo policy (so it compiles
and loads immediately); replace `rne_controller_step_v3` with your controller.
`rne-asset plugin list --path <dir>` enumerates the built-in `velocity_servo`
and any discoverable plugin libraries in the given directories. An
end-to-end test scaffolds, builds, and loads a plugin from the generated
source.

The generated `rne-scaffold.json` content-addresses the exact schema-v1 author
files. The reader rejects extra, missing, modified, or symlinked entries, and
the compatibility corpus regenerates the fixed fixture to prevent generator
drift.

Before distribution, run the standalone conformance kit against the built
library and its manifest:

```bash
rne-asset plugin check \
  --library target/debug/libmy_controller.so \
  --manifest plugins/my_controller/rne-plugin.json \
  --output my_controller.conformance.json
```

Use the platform's `.dll` or `.dylib` name where appropriate. Report schema v1
content-addresses both inputs and checks manifest/binary identity, supported ABI
symbols, capability negotiation, fixed-step observation/action validation,
exact action replay after an identical seeded reset, and lifecycle shutdown.
Semantic failures still produce a machine-readable failed report and a nonzero
CLI exit.

The author-facing ABI definitions are isolated in the dependency-free
`rne_plugin_sdk` crate. `rne-asset plugin new` vendors that exact module into
the generated crate, so the first build works with `cargo build --offline` and
does not depend on RNE host crates or crates.io. The reference ABI-v3 plugin
uses the SDK directly; the frozen ABI-v2 fixture remains independent. See
[`PLUGIN_SDK.md`](PLUGIN_SDK.md) for ownership and upgrade rules.

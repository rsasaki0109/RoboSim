# Robot Native Model

RNE models robots with native entities instead of ROS nodes.

## First-class concepts

| Concept | Purpose |
|---------|---------|
| World Entity | gravity, time, random seed, scenario |
| Robot Entity | model metadata, base link, link/joint graph |
| Sensor Entity | IMU, LiDAR, camera, encoders |
| Actuator Entity | wheel motors, servos, grippers |
| Agent Entity | policy, teleop, external controller |
| Episode Entity | reset, reward, termination, recording |

## World randomness

World-level randomized behavior uses `WorldRandom`, seeded from the scene
`[world].seed`. Systems that need deterministic streams derive them from stable
`RandomStreamId` values instead of wall-clock time or global mutable state.
The main world stream is reserved for explicitly ordered bootstrap work; sensors,
agents, and domain randomization should keep their own derived RNG state or use
stable keyed samples. Mid-run snapshots persist `WorldRandomSnapshot`, plus any
RNG state owned by sensor, agent, or episode components. Built-in randomized
episodes expose `EpisodeRandomSnapshot` for their owned RNG stream.
Replay JSONL files can store these random checkpoints as `ReplayRandomSnapshot`
records; the payload is RNG state only and must be paired with a separate ECS or
physics snapshot for true mid-run resume. Restore APIs reject mismatched world
seeds and missing named RNG streams instead of silently continuing with a partial
checkpoint.
The built-in differential-drive simulator exposes `DiffDriveSimSnapshot` for an
in-memory completed-tick state checkpoint on the same scene topology. It records
local transforms, rigid-body velocities, actuator targets, joint motors, sensor
sequence state, latest sensor frames, world random state, simulation time, and
the next actuator command sequence.
`DiffDriveEpisodeSnapshot` wraps that simulator checkpoint with
`ReplayRandomSnapshot` and episode-local counters, reward state, active goal,
and curriculum progress so a running episode can resume from a completed tick.
The mobile-manipulator environment follows the same completed-tick checkpoint
contract with `MobileManipulatorSimSnapshot` and
`MobileManipulatorEpisodeSnapshot`; it additionally records grasp weld
components, lift motor targets, joint-state frames, wrist-camera frames, the
active task, and reach-curriculum progress.
Vectorized environments expose matching vector checkpoints that store the
per-environment episode checkpoints in stable environment-index order.
Bit-exact continuation for dynamic physics also requires backend-native
constraint/warm-start state; the current simulator snapshot is exact for
kinematic diff-drive and sufficient for logical episode state restoration in
dynamic environments.

Scene files may use bare integer seeds for the portable signed 64-bit TOML
range. Full `u64` seeds should be written as decimal or `0x` strings, for
example `seed = "0xffff_ffff_ffff_ffff"`.

Replay logs include explicit compatibility metadata for the replay format,
world seed, RNG algorithm, keyed random algorithm, stream derivation, and fixed
timestep. Replay consumers validate this metadata before deterministic playback
instead of silently running logs with mismatched algorithms.

Stateless sensor noise uses keyed random samples derived from the world seed,
the sensor-local seed, a stable stream id, the sensor sample sequence, and a
channel index. This keeps one sensor's noise sequence stable when unrelated
sensors are added or sampled in a different order.

## Robot components

- `Robot`: root metadata and base link reference
- `Link`: physical link on a robot
- `Joint`: parent/child link connection with axis and limits
- `Actuator`: command target applied to a joint or wheel

## Why not ROS in core

ROS2 topics, services, and TF are adapter concerns. The core publishes typed frames on the RNE DataBus so the same simulation can be consumed by Python, ROS2, files, or native Rust tools.

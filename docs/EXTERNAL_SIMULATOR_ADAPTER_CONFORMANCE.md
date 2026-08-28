# External simulator adapter conformance

`rne-simulator-conformance` is the process-isolated authoring and evidence kit
for Gazebo Sim, Choreonoid, and other simulators that execute an RNE TaskSpec.
The protocol contains only RNE-owned JSON values. Simulator SDK, ROS 2, DDS,
transport, entity handles, and physics types remain inside the adapter process.

This is not the `PhysicsBackend` interface. A physics backend advances RNE ECS
state. A simulator adapter owns a complete external world and translates the
ordered TaskSpec observation and action vectors at a fixed simulation step.

## Process contract

Each frame is one bounded JSON value followed by a newline. Protocol v1 is:

1. `open` binds the exact TaskSpec ID, TaskSpec SHA-256, flattened tensor
   widths, and `fixed_delta_ticks`;
2. `reset` applies a caller-owned seed and returns the initial observation at
   step zero and simulation time zero;
3. each `step` accepts one strictly increasing TaskSpec-ordered action and
   returns the observation after exactly one fixed simulation step;
4. the response carries the reached step, exact accumulated simulation-time
   ticks, terminal flags, and a same-runtime state digest;
5. `close` releases the isolated simulator session.

Unknown fields, wrong task digests, wrong widths, a one-tick delta mismatch,
cross-session requests, non-finite values, skipped action sequences, and steps
before reset fail closed. The protocol never uses wall-clock time as simulation
time.

## Runtime manifest

Every run retains an `rne_external_simulator_runtime_manifest` with the exact
simulator family and version, distribution, fixed delta, and three files in
canonical order:

- world: geometry, gravity, lights, and physics configuration;
- robot model: joints, collisions, actuators, and sensor definitions;
- adapter config: TaskSpec field-to-entity mapping and units.

The conformance runner rehashes each file and binds the manifest, files,
TaskSpec, adapter subject, and normalized launch arguments into its report.
A report without those exact retained bytes cannot pass the readiness audit.

## Run the kit

```bash
rne-simulator-conformance \
  --adapter ./rne-gazebo-adapter \
  --subject ./rne-gazebo-adapter \
  --runtime-manifest runtime.json \
  --task flagship.task.json \
  --output gazebo-conformance.json \
  --adapter-arg --runtime-manifest \
  --adapter-arg runtime.json
```

For an interpreted adapter, use its interpreter as `--adapter` and the exact
script as `--subject`. Argument hashing replaces an argument equal to the
subject path with `<adapter-subject>` and one equal to the runtime manifest
path with `<runtime-manifest>`, removing machine-specific parent directories.

The report has ten canonical checks:

| Check | Contract |
|---|---|
| `open_identity` | Handshake matches the retained simulator runtime identity |
| `task_binding` | Wrong TaskSpec digest is rejected before correct open |
| `fixed_delta_binding` | A one-tick fixed-step mismatch is rejected |
| `reset_origin` | Seeded reset returns the TaskSpec observation at step/time zero |
| `bounded_step` | One in-bounds action advances exactly one step |
| `fixed_step_progression` | Step and simulation time advance exactly for three actions |
| `deterministic_replay` | Two fresh same-runtime seeded traces are bit-identical |
| `action_sequence_rejection` | A skipped action sequence does not advance state |
| `session_isolation` | A foreign session cannot mutate the bound session |
| `width_rejection` | Wrong-width action is rejected before advancement |

The shipped mock adapter exercises the authoring path but cannot qualify as
independent evidence.

## Gazebo adapter mapping

Target Gazebo Harmonic first. Gazebo systems apply commands in
`ISystemPreUpdate` and read resulting state in `ISystemPostUpdate`; the official
[system plugin guide](https://gazebosim.org/api/sim/8/createsystemplugins.html)
defines those update phases. Run the server paused and advance one iteration
per accepted RNE action. Gazebo exposes fixed stepping through its world-control
service; see the official
[pause and run guide](https://gazebosim.org/api/sim/10/pause_run_simulation.html).

The adapter must translate only the flagship observation/action catalog. It
must not add Gazebo, ROS 2, DDS, or simulator-specific handles to an RNE core
crate. Gazebo's `simTime` reached after update is the response time; wall time,
real-time factor, and transport arrival time are diagnostic metadata only.

## Independent evidence

Submit `kind = "simulator_adapter"` through the external-system intake route.
The retained evidence pack includes:

- exact adapter subject;
- exact TaskSpec;
- normalized ordered adapter arguments;
- runtime manifest;
- world, robot model, and adapter config files;
- complete passing simulator conformance report;
- official RNE release archive used for the run;
- acyclic submission candidate and committed stdout/stderr logs;
- schema-v1 maintainer report from `external-simulator-check`.

Start from `release/external-simulator-submission-template.json`. The candidate
does not contain its own Git revision: commit the final candidate and logs,
then pass the clean repository's exact revision separately to the checker.
This avoids a self-referential digest while binding acceptance to immutable
Git state. The maintainer checker parses and rehashes the typed evidence but
does not execute the untrusted simulator adapter outside a sandbox.

An in-repository implementation, renamed mock, or copied reference cannot
satisfy the independent external-system gate.

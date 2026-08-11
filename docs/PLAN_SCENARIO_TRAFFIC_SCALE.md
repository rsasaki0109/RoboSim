# M5 scenario and traffic scale

## Goal

M5 turns the existing traffic-scale demonstrations into one enforceable
OpenSCENARIO and live-co-simulation contract. A reference OpenSCENARIO run must
drive 100 actors at a requested 60 Hz, mixed runtime/external ownership must be
observable without double integration, and a dropped TraCI connection must be
recoverable without guessing whether a failed simulation step should be sent a
second time.

The traffic runtime remains renderer-, physics-, importer-, and
external-simulator-neutral. Wall-clock time is used only by the benchmark
runner; simulation decisions continue to use `SimTime` and `SimDuration`.

## Existing baseline

- The OpenSCENARIO importer reads multiple `ScenarioObject` declarations, but
  rejects a `ManeuverGroup` containing more than one actor.
- The scenario executor sorts entity spawns and final poses by name, but
  derives one route from the first entity kind and gives every assigned-route
  action the same route ID.
- The traffic runtime leaves `TrafficPoseSource::External` actors untouched,
  but its step report and stable hash cover only runtime-integrated actors.
- `CoSimulation::step` applies mirror updates transactionally, but a caller
  cannot replace a failed TraCI connection and resynchronize the mirror without
  reconstructing the bridge.
- Renderer-free Examples 46 and 47 already prove 100 native actors above
  60 simulation steps per wall-clock second. M5 extends that gate through the
  OpenSCENARIO path rather than replacing those tests.

## Contract

### Multi-actor OpenSCENARIO

- A non-empty `ManeuverGroup/Actors` set may contain multiple unique
  `EntityRef` entries. Each supported event is expanded to every referenced
  actor in deterministic actor order.
- Every referenced actor must exist in `Entities`; duplicate references are
  rejected instead of applying an action twice.
- Runtime routes are derived per road-user kind, and every entity receives a
  route that admits its kind.
- Assigned routes use an entity-specific stable ID so simultaneous independent
  route actions cannot alias each other.
- Externally visible actor results contain names and are sorted by name. Entity
  declaration and ECS spawn order cannot change the final actor states or hash.

### Mixed ownership observation

Every completed native traffic step reports:

- runtime-owned, external-owned, and total actor counts;
- a canonical observed-state hash over all actor UUIDs, ownership tags, and
  poses;
- the existing runtime-only flow metrics and integration hash without changing
  their compatibility semantics.

External actors require a stable UUID and finite pose for canonical observation
but never require a route follower and are never advanced by the native
runtime. A mixed-world test proves that reversing spawn order preserves the
observed hash while the external pose remains byte-identical.

### Recoverable TraCI session

`CoSimulation` separates three phases:

1. request one SUMO simulation step;
2. read a complete, sorted vehicle snapshot;
3. commit that snapshot transactionally to ECS.

A connection created from an endpoint remembers that endpoint. After any I/O
or protocol failure, explicit recovery opens a fresh client and synchronizes
the current SUMO snapshot **without** issuing `simulationStep` again. The old
client and ECS mirror remain active until the replacement snapshot has been
read completely. Successful recovery increments a monotonic session generation
and reports created, updated, and removed mirror counts. A client injected by a
caller can be recovered with another injected client, while endpoint-less
automatic recovery returns a typed error.

### Urban scale budget

The committed reference builds and parses an OpenSCENARIO 1.0 document with
100 motor vehicles on a 2.5 km corridor, one shared 100-actor speed event, and
explicit staggered initial poses. It runs 720 fixed steps at a requested 60 Hz.

The scale runner fails unless:

- all 100 actors are present and receive the shared action;
- forward and reverse entity declaration order produce identical final actor
  states and stable hashes;
- signal violations and collisions are zero;
- minimum bumper gap is at least 2 m;
- one reference execution sustains at least 60 simulation steps per wall-clock
  second on the GitHub-hosted Windows/Linux CI class.

The JSON report records the fixture/schema versions, deterministic outcomes,
budget threshold, measured throughput, and benchmark-class label. Throughput
is evidence, never an input to simulation state or its stable hash.

## Delivery slices

### M5-A: multi-actor import and execution

- Expand actor-set events and validate all references.
- Derive routes per entity kind and isolate assigned route IDs.
- Add named ordered actor results and multi-actor parser/runtime tests.

### M5-B: ownership metrics

- Add canonical ownership counts and observed-state hashing.
- Preserve existing runtime-only flow/hash behavior.
- Add spawn-order and external-pose preservation tests.

### M5-C: TraCI recovery

- Extract snapshot read/commit from `CoSimulation::step`.
- Add endpoint and injected-client recovery paths with generation reports.
- Prove failed reads are transactional and recovery does not double-step.

### M5-D: scale report and CI

- Add the 100-actor OpenSCENARIO benchmark and deterministic acceptance tests.
- Emit a JSON report through `xtask scenario-scale`.
- Add the scale runner to OSS parity and upload its report in CI.

### M5-E: exit gates

- Run formatting, workspace Clippy with `-D warnings`, workspace tests,
  `xtask parity`, `xtask ci-headless`, and `xtask ci` from the locked graph.
- Validate the full GitHub Actions matrix on Windows and Linux.
- Merge the milestone and remove only its dedicated branch/worktree.

## Implementation status

- M5-A multi-actor import and execution: in progress.
- M5-B ownership metrics: pending.
- M5-C TraCI recovery: pending.
- M5-D scale report and CI: pending.
- M5-E full workspace/CI matrix: pending.

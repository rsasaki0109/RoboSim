# M5 scenario and traffic scale

## Goal

M5 turns the existing OpenSCENARIO, native traffic, and TraCI paths into one
enforceable headless scale contract. A committed reference scenario must run
100 actors at at least 60 simulation steps per wall-clock second on the CI
benchmark class, preserve canonical externally visible ordering, report mixed
native/external pose ownership, recover a disconnected TraCI session, and end
with zero unexplained violations.

Simulation code continues to use `SimClock`, `SimTime`, and `SimDuration`.
Wall-clock time is read only by the outer benchmark harness and is never an
input to simulation behavior or stable evidence.

## Starting baseline and gaps

- `rne_traffic` already advances 100 native actors deterministically, with a
  spawn-order-independent hash, explicit gaps, signal control, junction
  reservations, and flow metrics.
- renderer-free Example 47 already asserts at least 60 headless steps per
  second for 100 native vehicles, but it is not an OpenSCENARIO reference
  fixture and does not exercise mixed ownership or TraCI recovery.
- `rne_openscenario` imports multiple entities and applies actions in stable
  entity-name order, but execution derives one route from the first entity
  kind. Assigned routes share one synthetic ID and lane changes share one
  direction, so independent multi-actor route/action behavior is not yet an
  enforceable contract.
- `TrafficPoseSource::External` prevents double integration, but external
  actors are excluded from flow metrics and the existing fleet hash. A mixed
  step therefore cannot prove how many actors each subsystem owned or that the
  complete visible state was canonically ordered.
- `rne_traci::CoSimulation` mirrors each completed read transactionally, but a
  connection failure ends the session. It has no public disconnected state,
  bounded reconnect operation, recovery counters, or snapshot-only resync.

## Boundaries and non-goals

- `rne_traffic` remains backend-neutral and may depend only on `rne_core`,
  `rne_ecs`, `rne_math`, and `rne_world`. It must not depend on SUMO, TraCI,
  OpenSCENARIO, a renderer, or a physics backend.
- `rne_openscenario` owns scenario import and execution policy. It may consume
  `rne_traffic` APIs without moving scenario types into the traffic core.
- `rne_traci` owns TCP protocol and external-session recovery. SUMO remains the
  integration and routing authority for mirrored actors.
- ROS2 remains an adapter and is not part of this milestone.
- M5 does not add the complete OpenSCENARIO 1.x condition/action surface,
  continuous lane-change dynamics, distributed simulation, or a guarantee
  that unlike external simulators produce identical floating-point state.
- The 60 Hz gate measures headless throughput, not real-time sleeping or
  pacing. It never makes wall-clock time part of a replay hash.

## Contract

### Multi-entity scenario execution

Scenario execution has these deterministic rules:

1. Entity identity is the entity name plus a stable UUID derived from the
   canonical name order, never ECS insertion order.
2. Each distinct actor kind receives a compatible deterministically derived
   network route. An entity is never placed on a lane that excludes its kind.
3. Assigned routes and parallel lane-change routes have per-action stable IDs;
   one actor's route action cannot overwrite or alias another actor's route.
4. Due actions are applied in the total order `(start_time_s, entity_name,
   source_action_index)`. This order is recorded as externally visible action
   evidence.
5. Final actor snapshots are ordered by stable UUID and include name, actor
   kind, pose source, route ID when present, position, heading, and speed.
6. The scenario result digest covers the canonical final actor snapshots and
   the ordered action evidence. Reversing document entity or action insertion
   order without changing their semantic order produces the same result.

The reference fixture contains 100 actors with independent initial positions
and same-tick actions. Its replay test proves actor count, canonical order,
stable digest, zero collision/signal violations, and exact repeat execution.

### Mixed pose ownership metrics

Every traffic step reports one ownership snapshot:

- total visible traffic actors;
- runtime-owned and external-owned actor counts;
- runtime actors advanced by the native integrator;
- external poses observed but not advanced;
- actors missing the stable UUID or pose required for external visibility;
- a canonical visible-state digest over all valid traffic actors, including
  external actors.

Absence of `TrafficPoseSource` means `Runtime` for compatibility. Runtime-owned
actors require a route follower and are updated exactly once. External-owned
actors require a stable UUID and pose, may omit a route follower, and are never
mutated by native traffic integration. Invalid externally visible state fails
before any runtime actor is mutated.

The legacy native fleet digest remains available for native conformance. The
new visible-state digest is the M5 mixed-ownership evidence and changes when
either a native or external pose changes.

### Recoverable TraCI session

`rne_traci::CoSimulation` exposes a small explicit state machine:

```text
connected --I/O or protocol failure--> disconnected
disconnected --successful reconnect + snapshot resync--> connected
connected/disconnected --close--> closed
```

Recovery has these invariants:

- a failed TraCI read never partially mutates the ECS mirror;
- disconnect keeps the last complete mirror and stable actor mapping;
- reconnect is available only for endpoint-backed sessions and uses a bounded
  caller-supplied retry policy;
- the first successful connection performs a snapshot-only resync before the
  next simulation step, so recovery does not intentionally double-step SUMO;
- actors with the same SUMO IDs retain their RNE entities and UUIDs;
- actors added or removed while disconnected are reconciled in sorted SUMO ID
  order;
- session metrics report successful steps, failed steps, reconnect attempts,
  successful recoveries, and the current state;
- a closed session rejects step, recovery, and command calls deterministically.

A process-level mock test drops the first TCP connection after a completed
mirror, accepts a replacement connection, changes the vehicle set while
disconnected, and proves recovery without partial mutation or identity drift.

### Violation accounting

The M5 report classifies every violation; an unclassified aggregate is not
accepted. The initial registry is:

| ID | Unit | Meaning | Exit bound |
|---|---|---|---|
| `traffic.collision` | count | overlapping actor rectangles or negative same-route bumper gap | 0 |
| `traffic.signal` | count | front bumper crosses a red stop line | 0 |
| `traffic.ownership.invalid_state` | count | actor lacks stable UUID/pose or runtime follower | 0 |
| `traffic.ownership.double_integration` | count | external pose changed by the native step | 0 |
| `scenario.action.unapplied` | count | scheduled action due within the run was not applied | 0 |
| `traci.recovery.unreconciled` | count | mirror differs after successful resync | 0 |

The machine-readable report includes every registry row, measured count,
bound, status, and deterministic evidence. “Zero unexplained violations” means
all measured failures map to a registry row and every exit-bound row passes.

### Urban scale budget

The committed benchmark uses:

- exactly 100 actors;
- 600 fixed steps at 60 Hz simulation time;
- no renderer, GPU, ROS2, SUMO process, network service, or sleep;
- a release build on the GitHub-hosted `windows-latest` parity runner, which is
  the M5 CI benchmark class;
- three measured repetitions after one untimed warm-up;
- the minimum repetition throughput as the verdict;
- a required minimum of 60 completed simulation steps per wall-clock second.

The report records actor/step counts, elapsed nanoseconds and throughput for
each measured repetition, stable digests, violations, ownership metrics, and
the final verdict. Timing fields are diagnostic and excluded from deterministic
digest comparison; all state/ordering/violation evidence must match across
repetitions.

## Delivery slices

### M5-A: contract and canonical scenario evidence

- Freeze action ordering, actor snapshot, ownership, recovery, violation, and
  benchmark contracts.
- Add canonical scenario action events and final actor snapshots.
- Cover heterogeneous actors, same-tick actions, independent assigned routes,
  and both lane-change directions.

### M5-B: mixed ownership

- Add per-step ownership metrics and complete visible-state digest.
- Validate all visible actors transactionally before native mutation.
- Prove that external poses are observed, included in evidence, and never
  integrated by `rne_traffic`.

### M5-C: recoverable TraCI

- Add explicit session state and metrics.
- Add bounded reconnect and snapshot-only resync.
- Add disconnect/reconnect process tests and document failure behavior.

### M5-D: 100-actor reference and report

- Add a committed OpenSCENARIO urban-scale fixture and run manifest.
- Add a headless scale runner that emits deterministic JSON plus outer-harness
  timing evidence.
- Add `xtask scenario-scale` and upload its report from the parity CI job.

### M5-E: exit gates

- Add unit, integration, deterministic replay, report-schema, and process-level
  recovery tests.
- Run `cargo fmt --all`, workspace Clippy with `-D warnings`, workspace tests,
  `xtask ci-headless`, and `xtask ci` from the locked dependency graph.
- Update the roadmap, traffic runtime, OSS parity matrix, examples index, and
  changelog with measured M5 evidence.

## Implementation status

Local Windows x86_64 release evidence on 2026-08-12 completed all 100 actors
for 600 steps in every repetition. The slowest measured repetition sustained
1,177.9 steps/s against the 60 steps/s bound; minimum gap was 15.6 m, runtime
ownership was 100/100, actor/action result digest was
`6732886903736628512`, and every violation-registry count was zero. The named
GitHub-hosted runner remains the authoritative performance result.

- M5-A contract and canonical scenario evidence: complete.
- M5-B mixed ownership: complete.
- M5-C recoverable TraCI: complete.
- M5-D reference/report: complete locally; CI evidence pending.
- M5-E full workspace/CI matrix: in progress.

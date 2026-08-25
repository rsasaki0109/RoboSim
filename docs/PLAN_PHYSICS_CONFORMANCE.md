# M4 physics conformance

## Goal

M4 turns the existing dynamics tests into an enforceable backend contract. A
backend may advertise a physics capability only when a backend-neutral test
vector proves that capability, and the resulting evidence is emitted as a
stable, machine-readable report.

This milestone does not make unlike solvers bit-identical. Exact, versioned
snapshot hashes are used for repeat executions of a deterministic backend.
Analytic-vs-Rapier and contact-rich comparisons use named, unit-bearing,
documented tolerances.

## Current gaps

- Capability declarations and tests exist, but there is no catalog proving that
  every advertised capability has a conformance case.
- Dynamics tolerances are private constants spread across integration tests.
- `hash_physics_state` covers only quantized translations and relies on the
  standard-library hasher rather than a frozen algorithm.
- Contact pairs have no canonical snapshot representation for reports.
- Analytic and Rapier tests do not execute the same input vector or emit one
  comparison report.
- The analytic backend writes positions back to ECS but not integrated linear
  velocity, so a backend-neutral state snapshot cannot observe its complete
  rigid-body result.

## Contract

### Capability coverage

The conformance catalog maps each currently advertised capability to at least
one required case:

| Backend | Advertised capability | Required evidence |
|---|---|---|
| Analytic | `RigidBody` | fixed-step free fall updates pose and velocity in explicit SI units |
| Analytic | `KinematicBody` | an externally supplied pose remains authoritative across a fixed step |
| Analytic | `DeterministicStep` | two fresh executions produce the same canonical snapshot hash |
| Rapier | `RigidBody` | the same free-fall vector stays within the registered reference tolerance |
| Rapier | `KinematicBody` | the same external-pose vector stays within its named metre/radian tolerances |
| Rapier | `DeterministicStep` | two fresh executions produce the same canonical snapshot hash |
| Rapier | `Articulation` | revolute anchor and limit invariants remain bounded under load |
| Rapier | `ContactForce` | a resting body's reported impulse matches `mass * g * dt` within tolerance |
| Rapier | `RaycastBatch` | repeated queries return bounded hits in distance/entity order |
| Rapier | `JointEffortMeasurement` | the native accepted force/torque increment for direct effort actuation is retained after the step |
| MuJoCo | `JointEffortMeasurement` | a direct 2 N*m revolute command is retained as completed-step native actuator effort |

`GpuRigidBody` and `SoftBody` remain unadvertised. A coverage test fails when a
backend adds a capability without adding a catalog case.

### Canonical snapshot v1

`rne_physics` owns a backend-neutral snapshot of completed-step observable
state:

- schema version, simulation step, and simulation timestamp ticks;
- rigid-body type, entity identity, world translation/rotation, and linear and
  angular velocity;
- canonical contact entity pairs, oriented normals, and impulses in N·s;
- deterministic ordering and a frozen FNV-1a 64-bit digest over little-endian
  field encodings.

Non-finite state is rejected. Pair orientation is canonicalized before sorting,
so backend iteration order cannot change the snapshot. The snapshot is
observational evidence, not a promise to restore solver constraints or warm
starts.

### Tolerance registry v1

Every approximate assertion references a registry entry with:

- stable case and metric identifiers;
- explicit unit (`m`, `m/s`, `rad`, `N*s`, or unitless);
- absolute and relative bounds;
- a short solver/integration rationale.

Reports include the applicable registry entry and measured error. Exact hashes
are never used to claim cross-platform equivalence for contact-rich Rapier
state.

### Report v1

The conformance runner emits deterministic JSON containing:

- engine/report schema versions;
- backend names and sorted advertised capabilities;
- sorted case outcomes and unit-bearing metrics;
- canonical snapshot hashes where exact replay is valid;
- analytic-vs-Rapier free-fall deltas and tolerance decisions;
- a coverage verdict showing that every advertised capability has evidence.

Wall-clock timings and machine-specific paths are excluded. `xtask` writes the
report under `artifacts/physics-conformance/report.json` and fails when any case
or capability coverage check fails.

## Delivery slices

### M4-A: contract and snapshot

- Freeze capability names/order and the conformance case catalog.
- Add canonical physics snapshot v1 and frozen digest tests.
- Make analytic velocity write-back observable through the common state model.

### M4-B: shared vectors and tolerance registry

- Build one backend-neutral fixed-step rigid-body vector.
- Centralize named SI-unit tolerances and validation logic.
- Run exact repeatability checks separately from approximate solver comparison.

### M4-C: capability-specific validation

- Add Rapier articulation anchor/limit validation.
- Add canonical contact impulse validation.
- Add ordered repeated-raycast validation.
- Assert complete coverage of every backend's advertised capability set.

### M4-D: reports and CI integration

- Emit deterministic JSON and validate its schema/order.
- Add `xtask physics-conformance` and a parity/CI gate.
- Document measured contracts and backend limitations.

### M4-E: exit gates

- Unit, integration, report, determinism, Windows, and Linux checks pass.
- `cargo fmt --all`, workspace Clippy with `-D warnings`, workspace tests,
  `xtask ci-headless`, and `xtask ci` pass from the locked graph.
- No backend-specific type leaks through `rne_physics` public APIs.

## Implementation status

- M4-A contract and snapshot: complete.
- M4-B shared vectors and tolerance registry: complete.
- M4-C capability-specific validation: complete.
- M4-D reports and CI integration: complete.
- M4-E full workspace/CI matrix: complete.

## v2 follow-on for interchangeable dynamics

The v0.3 harness keeps the M4 measurements but removes the built-in-runner
assumption:

- `rne_physics::PhysicsBackendManifest` declares stable backend/engine versions,
  canonical capabilities, and the same-runtime repeatability class without
  exposing a native backend type;
- `run_backend_conformance` applies the shared capability catalog to any
  `PhysicsBackend` factory and fails closed when a capability vector or named
  tolerance profile is missing;
- conformance report schema v2 embeds the validated manifest and the actual
  runtime capability declaration so drift is visible in evidence;
- `JointState` is the backend-neutral completed-step articulation observable;
  Rapier writes reduced-coordinate revolute/prismatic state during ECS sync,
  and MuJoCo must implement the same contract before advertising articulation.

Backend-manifest schema v2 added the `kinematic_body` vocabulary. Analytic and
Rapier advertise it only after passing the shared external-pose vector. MuJoCo
does not advertise it: both `preflight_world` and the trait sync boundary reject
a kinematic entity before native model creation with a typed missing-capability
error.

Schema v3 adds `joint_effort_measurement`, which requires a completed-step,
backend-measured, unit-explicit realization of direct effort actuation. MuJoCo
and Rapier advertise it only after passing the shared direct 2 N*m revolute-effort
vector. Rapier measures the native `user_torque`/`user_force` increment accepted
before the step and retains it only after that step completes; motor-driven
position/velocity modes remain unavailable rather than copying their command.

MuJoCo rigid-body compilation now registers
`mujoco_free_fall_position_m_v1` and runs in the same catalog behind the
`rne_physics_conformance_suite/mujoco` feature. It also advertises `articulation` after
passing the shared revolute vector with unit-explicit `JointActuation` and
backend-neutral `JointState`; backend integration tests cover revolute and
prismatic position/velocity/effort behavior. MuJoCo now also passes the shared
`contact_force` vector: per-point solver force is integrated to N*s, aggregated
by canonical entity pair, and evaluated with
`mujoco_resting_impulse_n_s_v1`; sensor overlaps remain zero-impulse evidence.
Feature runs also compare Analytic-vs-MuJoCo and Rapier-vs-MuJoCo through named
position/velocity bounds. `rne-physics-divergence` tightens only the latter
position bound to 1 cm, finds the first violation at a stable fixed step, and
emits an existing-schema Behavior replay plus a deliberately failing report.
The MuJoCo Windows/Linux job packages both into a verified Failure Capsule.

The next articulation observable is optional completed-step joint-effort
evidence. `JointEffortMeasurement` keeps revolute N*m and prismatic N distinct;
absence remains different from measured zero. The joint-feedback sensor samples
it at the declared simulation capture time, applies the existing sensor latency,
adds no noise, and fails closed on non-finite values or unit-kind mismatch.
MuJoCo writes native `actuator_force` into this contract after each completed
step. Rapier remains explicitly unavailable because its multibody constraint
solver does not retain the solved motor impulse in public joint state. A backend
must not substitute the reconstructed bounded PD command for this measurement.
The follow-on conformance vector must compare command limit, actuator-space
effort, and any passive/implicit compensation as separate quantities.

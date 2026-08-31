# Mobility backend closed loop v1

This slice runs one exact `TaskSpec` through both Rapier and MuJoCo using the same
backend-neutral control path:

```text
motor voltage -> DC motor -> transmission -> wheel inertia
  -> transient combined-slip tire -> force-at-contact wrench
  -> backend rigid body -> completed contact kinematics and normal load
  -> next control step
```

The actor-visible observation is only the task-owned `command_phase`; the action is a
bounded motor terminal voltage. Chassis pose, velocity, contact load, tire force, motor
current, and wheel speed are explicitly privileged evidence fields. They are not actor
tensors. Both runs use seed `0`, a 1 ms fixed step, 300 settle steps, and 2,000 driven
steps.

## Contact and force convention

The backend reports a completed-step contact point, its carrier-point velocity, and normal
load. `evaluate_longitudinal_drive_path` subtracts the wheel circumferential velocity once,
updates motor/transmission/wheel/tire state, and emits a world-frame force-at-point wrench
for the following backend step. The wrench is absent when contact is absent. Native
tangential collider friction is zero so generic Coulomb friction cannot silently act as a
second tire model.

This is an equivalent single driven support path under a rigid 100 kg carrier, not yet a
geometric vehicle with physical wheel bodies or suspension. It proves the contact-to-tire-
to-wrench boundary and backend parity. It does not prove steering, yaw response, load
transfer, curb traversal, wheel lift recovery, or a named real vehicle profile.

## Self-verifying evidence

Each backend trace embeds the exact `PhysicsBackendManifest`, exact `TaskSpec`, seed,
fixed-step clock, ordered time series, SI-unit acceptance metrics, verdict, and content
digest. Validation recomputes the command schedule and rejects TaskSpec drift, non-finite
state, invalid timestamps, unstable support, excessive tilt, metric drift, verdict drift,
or digest tampering.

The cross-backend artifact retains both complete traces. It compares absolute gaps only in
declared units and tolerances; exact floating-point equality is deliberately not required.
The reference Windows run with Rapier 0.22 and MuJoCo 3.9.0 produced:

| metric | absolute gap | tolerance |
| --- | ---: | ---: |
| contact drive fraction | 0.0005 | 0.01 |
| final forward distance | 0.00424 m | 0.05 m |
| final forward velocity | 0.00174 m/s | 0.05 m/s |
| final lateral drift | 0.00000392 m | 0.01 m |
| maximum motor current | 0 A | 0.05 A |
| maximum tilt | 0.0000977 rad | 0.005 rad |
| maximum tire utilization | 0.0285 | 0.1 |
| maximum vertical displacement | 0.000200 m | 0.005 m |

These values are regression evidence for this fixture, not validation against hardware.

Run the comparison with the MuJoCo native library configured:

```text
cargo run -p rne_mobility_benchmark --features mujoco -- \
  --backend compare \
  --output rapier-vs-mujoco.json \
  --failure-replay mobility-divergence.rne-replay
```

The optional replay deliberately injects a stricter 1 mm forward-position bound while the
production 5 cm bound remains passing. The first actual backend divergence is retained as a
standard Behavior replay. It can be packaged and independently verified through the normal
Failure Capsule path:

```text
cargo run --locked -p xtask -- failure-capsule create \
  --replay mobility-divergence.rne-replay \
  --evidence rapier-vs-mujoco.json \
  --output mobility-divergence-capsule \
  --backend rapier-vs-mujoco \
  --backend-version rapier-0.22+mujoco-3.9.0

cargo run --locked -p xtask -- failure-capsule verify \
  mobility-divergence-capsule
```

The producer, feature-enabled unit test, Failure Capsule creator, and verifier all execute
headlessly. Rendering is neither required nor accepted as fidelity evidence.

## Research correspondence and next boundary

The tire hierarchy, slip-state rationale, and real-log identification requirements are
documented in
[`MOBILITY_COMBINED_SLIP_V1.md`](MOBILITY_COMBINED_SLIP_V1.md) and
[`MOBILITY_LONGITUDINAL_BENCHMARK_V1.md`](MOBILITY_LONGITUDINAL_BENCHMARK_V1.md).
Project Chrono motivates separating tire-force laws from general rigid-body contact, while
MuJoCo and PhysX document contact and vehicle-force interfaces rather than making a generic
contact solver a calibrated tire model. RNE preserves that separation through
`ContactPointSample` and `ExternalBodyWrench`.

The next fidelity gate replaces the equivalent support path with per-wheel rigid geometry,
steering, suspension/load-transfer state, anisotropic skid scrub, and named Ackermann and
differential-drive fixtures. Its acceptance must add yaw-rate, lateral acceleration,
wheel-load, steering, and lift/recontact evidence before any road-vehicle fidelity claim.

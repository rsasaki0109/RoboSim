# External product proof plan

Status: active execution plan
Started: 2026-08-23

## Decision

RNE is not competing to become another general-purpose robot simulator. Its
product wedge is a CI-native robot verification runtime:

> Run one typed robot task through simulation, recorded data, and bounded
> hardware paths, then package the first contract violation as portable,
> independently reproducible evidence.

The next product gate is external reproduction, not broader simulation
coverage. New robots, environments, importers, render features, and physics
features remain out of scope unless they are required to close one of the
gates below.

## Why this gate comes next

RNE already has a source-checkout flagship, interchangeable Rapier and MuJoCo
execution, typed TaskSpec artifacts, deterministic fault injection, replay
minimization, and Failure Capsules. Those are necessary foundations, but they
do not yet prove that a user outside this repository can obtain the product
outcome.

The native release bundle now rehearses twelve installed checks. The flagship
check uses `rne-flagship-proof`: one packaged command that reproduces the complete
indoor mobile-manipulation flagship success, expected failure, first contract
violation, browser inspector, verified Failure Capsule, and a SHA-256-bound
installed proof report. It executes the same proof on Rapier and a bundled,
pinned MuJoCo runtime. Clean tagged Windows/Linux archive evidence and the
15-minute external measurement remain the open parts of Gate 1; the public
intake path now verifies rather than trusts that submission.

## Flagship product contract

The retained flagship is indoor mobile manipulation. Its visual environment
may use the real-capture 3DGS asset, but the evidence contract does not depend
on presentation rendering.

One unchanged TaskSpec and controller must cover:

- a successful perception, grasp, lift, transport, and place run;
- an intentionally injected, deterministic sensor failure;
- the first violated typed contract and its simulation step;
- named SI-unit tolerances for cross-backend observations and outcomes;
- a minimized replay and self-contained browser inspector;
- a Failure Capsule binding the task, controller, model, configuration,
  backend identity, reports, replay, and content digests.

The 3DGS fixture becomes qualifying evidence only when it also binds the
metric world transform, camera intrinsics and extrinsics, collision alignment,
and semantic object identities. A visually plausible background alone is not
a product gate.

## Delivery gates

### Gate 1: installed proof

A clean Windows or Linux machine can use a release archive to run one command
that produces and verifies the complete flagship evidence without cloning the
source repository.

Acceptance criteria:

- median time from extracted archive to verified capsule is at most 15 minutes
  on the named reference machine;
- no Rust toolchain, ROS 2, renderer, separate MuJoCo installation, or network
  access is required after the archive has been obtained;
- the command returns a non-zero status if the expected failure, minimization,
  capsule verification, or browser artifact is absent;
- the release report and independent archive-install rehearsal name this exact
  workflow rather than grouping unrelated installed checks under a flagship
  label;
- Windows and Linux CI retain the generated report and capsule.

### Gate 2: backend differential proof

The unchanged flagship TaskSpec and controller execute through Rapier and
MuJoCo. A deliberate perturbation produces a report naming the first divergent
observable or semantic outcome.

Acceptance criteria:

- exact, tolerance, and outcome comparisons are explicitly separated;
- tolerance names and SI units are serialized in the report;
- solver-private state hashes are never claimed to match across backends;
- the differential report and both backend identities are bound into the
  Failure Capsule;
- a future external simulator can implement the same runner contract without
  changing core entity types.

The source/runtime-gated schema-v2 path now covers both successful execution
and the same minimized intentional blackout on Rapier and MuJoCo, including
zero-tolerance first-violation comparison and both verified replays. The active
release slice builds that path into the installed runner, bundles the pinned
official MuJoCo runtime and licenses, and makes the two-runtime proof mandatory
in archive rehearsal. Gate 2 closes only after both extracted-archive CI jobs
retain passing evidence.

The versioned, fixed-step external simulator process contract and conformance
catalog are now implemented outside core. A first real Gazebo Harmonic 8.15
adapter also passes the complete ten-check catalog headlessly with the official
positive-scale OpenArm v2 right-arm URDF: nine joint targets enter during
PreUpdate and eighteen position/velocity values leave during PostUpdate. The
next slice promotes the existing RNE OpenArm pose cycle into the same TaskSpec
and controller artifact, runs success and intentional tracking failure on both
Rapier and Gazebo, and packages their first tolerance violation. Gazebo, ROS 2,
DDS, and simulator-specific handles remain outside core crates.

### Gate 3: recorded and shadow proof

The same TaskSpec evaluates a timestamped recorded observation stream and a
process-isolated shadow session.

Acceptance criteria:

- observation and action ordering is identical to simulation;
- clock source, latency, drop, calibration, and unit metadata are explicit;
- a recorded failure is inspected through the same contract and capsule tools;
- transport or device failure terminates within declared bounds and fails
  closed.

### Gate 4: bounded physical proof

One named mobile manipulator completes a low-speed, bounded task under the same
contract. This is evidence for the selected adapter and operating envelope,
not a general safety certification.

Acceptance criteria:

- authority, workspace, speed, force, timeout, and emergency-stop bounds are
  documented and enforced outside core;
- simulation, shadow, and physical reports share one TaskSpec identity;
- at least one real failure or forced abort is retained and independently
  inspectable;
- no claim extends beyond the measured robot, adapter, and environment.

### Gate 5: independent adoption

The product hypothesis is accepted only after independent use.

Acceptance criteria:

- two external repositories reproduce distinct tasks and verifiable Failure
  Capsules without repository-author assistance;
- one third-party controller plugin passes the published conformance kit;
- one independently maintained simulator or hardware adapter passes its
  conformance kit;
- setup observations include elapsed time, failure points, and required manual
  interventions;
- the 15-minute time-to-proof target is measured, not inferred from CI.

## Immediate implementation slices

Work proceeds in this order, one mergeable slice at a time:

1. **Delivered:** rename the installed release report's generic check
   collection so it no longer implies that the indoor flagship is shipped.
2. **Delivered:** define a versioned installed-flagship proof report and one
   archive command that produces success, expected failure, browser inspector,
   and capsule.
3. **Delivered:** stage the minimum flagship assets and runner in the native
   release bundle.
4. **Delivered in the rehearsal contract; tagged evidence pending:** add
   Windows and Linux archive-install rehearsals for the proof command.
5. **Measurement path delivered; independent run pending:** use
   `rne-flagship-proof OUTPUT --cross-backend --measure-on MACHINE` to retain a
   schema-v1, proof-bound timing artifact, then collect it on a named external
   reference machine.
6. **Delivered:** ship the MuJoCo-enabled proof runner and pinned runtime in
   both native archives and require the cross-backend result in extracted
   release rehearsal.
7. **Delivered; first submission pending:** publish a copy-paste external
   reproduction guide, bind the proof to its producer executable and clean
   tagged archive, reject CI/placeholder measurements, and expose a validated
   public issue route.
8. Ask the first external operator to run the next tagged release without
   maintainer intervention and audit the retained report.

Each slice includes tests and a short documentation update. No unrelated demo
or subsystem expansion enters these slices.

## Sensor and control-dynamics hardening track

This track starts after the external simulator contract is shipped and remains
subordinate to the product-proof gates. It strengthens the existing indoor
mobile-lift and OpenArm validation fixtures; it does not justify adding unrelated
robots, scenes, sensor types, or engine-specific physics features.

### Sensor evidence

The sensor target is a reproducible measurement contract, not a larger catalog.
For camera, depth, lidar, IMU, and joint/actuator feedback already required by
the flagship, retain:

- simulation timestamp, sampling phase, rate, latency, jitter, drop policy, and
  queue capacity;
- explicit SI units, coordinate frame, intrinsics, extrinsics, distortion, and
  metric calibration identity;
- seeded bias, white noise, quantization, saturation, dropout, and stuck-value
  models with an explicit order of application;
- ground-truth alignment and named error metrics for simulation, recorded data,
  shadow, and bounded physical observations;
- deterministic fault injection whose first violated TaskSpec contract enters
  the same replay, browser inspector, and Failure Capsule path;
- observability-oriented coverage: each controller-required state names the
  sensor fields and calibration evidence from which it is estimated.

Acceptance requires a headless `sensor-validation-report` that binds TaskSpec,
sensor specs, calibration, model/config hashes, seed, latency/drop trace, named
unit-bearing tolerances, and the first failed field. At least one nominal and
one injected-failure case must compare Rapier, MuJoCo, recorded observations,
and later Gazebo without claiming renderer-private pixels are byte-identical.

### Control-engineering dynamics

Dynamics work targets closed-loop verification and system understanding rather
than novel solver features. The portable contract will cover:

- continuous and discrete plant identity, state/input/output ordering, sample
  time, operating point, SI units, and model provenance;
- actuator saturation, rate limit, deadband, delay, bandwidth, torque/force
  limits, and declared failure behavior;
- analytic or deterministic finite-difference linearization around a named
  operating point, with validity-range evidence;
- controllability and observability rank, poles/eigenvalues, damping, natural
  frequency, and closed-loop stability diagnostics;
- deterministic step, impulse, ramp, and chirp experiments with rise time,
  settling time, overshoot, steady-state error, IAE/ISE, gain margin, and phase
  margin where mathematically applicable;
- rollout-based plant identification with training/validation split hashes and
  residual metrics, never silently replacing the declared physical model;
- reference PID/state-feedback/LQR evaluations and controller-plugin results
  through the same action limits, delay, disturbance, replay, and Failure
  Capsule contracts;
- cross-backend and sim/recorded/shadow comparison at named unit-bearing
  tolerances, with the first control-contract divergence retained.

Acceptance requires a browser-readable `control-dynamics-report` that binds the
plant, operating point, controller, actuator/sensor contracts, experiment input,
backend/runtime, metrics, tolerance registry, state hashes, and replay. Every
analysis must run headless and from `SimClock`; rendering is never required.

### Ordered implementation slices

1. Define versioned sensor-validation and control-dynamics report schemas plus
   unit-bearing tolerance registries.
2. Make sensor sample phase, latency, noise, calibration, and fault order
   explicit for the existing flagship observation set.
3. Add actuator dynamics and deterministic plant experiment inputs without
   exposing backend-specific handles through `rne_robot` or `rne_physics`.
4. Add linearization, controllability/observability, time-domain, and
   frequency-domain analysis over recorded deterministic trajectories.
5. Evaluate the unchanged mobile-lift controller on Rapier and MuJoCo, then the
   Gazebo adapter, and package the first divergence into a Failure Capsule.
6. Reuse the same reports for recorded playback, shadow, and the bounded
   physical path before expanding to another robot or sensor family.

## Stop conditions

The strategy is reconsidered rather than hidden behind more implementation if
either condition persists after two external onboarding attempts:

- a new user cannot reach a verified capsule within 15 minutes because the
  workflow is intrinsically too complex; or
- users do not value cross-backend or sim-to-real failure evidence enough to
  integrate the TaskSpec contract.

In that case, gather the onboarding evidence, narrow the contract, and choose
a smaller user problem before adding more simulator features.

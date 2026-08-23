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

The native release bundle now rehearses eleven installed checks. The eleventh
is `rne-flagship-proof`: one packaged command that reproduces the complete
indoor mobile-manipulation flagship success, expected failure, first contract
violation, browser inspector, verified Failure Capsule, and a SHA-256-bound
installed proof report. Clean tagged Windows/Linux archive evidence and the
15-minute external measurement remain the open parts of Gate 1.

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
- no Rust toolchain, ROS 2, renderer, MuJoCo, or network access is required for
  the default reference proof;
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
zero-tolerance first-violation comparison and both verified replays. Gate 2 is
not complete until this two-runtime proof is consumable from the release path;
the default archive currently ships only the dependency-free Rapier runner.

After this gate, a bounded Gazebo adapter is the preferred external-simulator
spike. It should translate only the flagship observations and actions; it must
not introduce Gazebo, ROS 2, DDS, or simulator-specific handles into core
crates.

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
   `rne-flagship-proof OUTPUT --measure-on MACHINE` to retain a schema-v1,
   proof-bound timing artifact, then collect it on named external reference
   machines.
6. Publish a copy-paste external reproduction guide and ask the first external
   project to run it without maintainer intervention.

Each slice includes tests and a short documentation update. No unrelated demo
or subsystem expansion enters these slices.

## Stop conditions

The strategy is reconsidered rather than hidden behind more implementation if
either condition persists after two external onboarding attempts:

- a new user cannot reach a verified capsule within 15 minutes because the
  workflow is intrinsically too complex; or
- users do not value cross-backend or sim-to-real failure evidence enough to
  integrate the TaskSpec contract.

In that case, gather the onboarding evidence, narrow the contract, and choose
a smaller user problem before adding more simulator features.

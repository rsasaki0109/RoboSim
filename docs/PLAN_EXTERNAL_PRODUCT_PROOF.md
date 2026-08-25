# External product proof plan

Status: active execution plan
Started: 2026-08-23

## Decision

RNE is not competing to become another general-purpose robot simulator. Its
product wedge is a CI-native robot verification runtime:

> Run one typed robot task through simulation, recorded data, and bounded
> hardware paths, then package the first contract violation as portable,
> independently reproducible evidence.

Within that wedge, sensor evidence and control-engineering dynamics are the
two sustained technical investments. RNE should become unusually good at
explaining the complete sampled-data loop -- measurement, estimation,
controller decision, actuator realization, and physical response -- rather
than accumulating loosely validated sensor or controller features. This work
is part of the product definition and continues beyond the first OpenArm proof.

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
adapter passes the complete ten-check catalog headlessly with the official
positive-scale OpenArm v2 right-arm URDF. The existing OpenArm pose cycle is
now a content-addressed controller artifact compiled once into the exact action
trace consumed by both RNE/Rapier and Gazebo. Both 1,400-step success replays
are deterministic and pass the named 0.01 rad final tolerances; a deliberately
truncated action produces the same first violation at step 307 and the rejected
Gazebo step is proven not to advance state. The comparison report, browser
inspector, TaskSpec, controller, traces, runtime manifest, and replay are
packaged by the standard verified Failure Capsule tool. The next slice derives
time- and frequency-domain control metrics from these retained joint traces.
Gazebo, ROS 2, DDS, and simulator-specific handles remain outside core crates.

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

This is a first-class technical gate serving the product proof, not a side
catalog of simulation features. It strengthens the existing indoor mobile-lift
and OpenArm validation fixtures and must produce evidence reusable in
simulation, recorded playback, shadow, and bounded hardware operation. It does
not justify adding unrelated robots, scenes, sensor types, or engine-specific
physics features.

The engineering question is deliberately concrete:

> Given the same timestamped sensor observations, actuator contract, plant
> identity, controller, and reference trajectory, can RNE explain and reproduce
> the first closed-loop deviation across backends and later on hardware?

Every result must therefore separate four causes: measurement error, state
estimation error, actuator realization error, and plant-model error. A final
pose alone is not sufficient evidence.

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

Sensor work is delivered in dependency order:

1. **Joint and actuator feedback:** position, velocity, effort, command,
   saturation state, sample age, and status bits. This is the measurement path
   used to close the first OpenArm control loop.
2. **IMU:** timestamped orientation/angular velocity/linear acceleration,
   gravity convention, bias and random-walk state, saturation, mount transform,
   and stationary plus prescribed-motion validation.
3. **Camera and depth:** intrinsics, distortion, exposure timestamp, optical
   frame, depth scale and invalid-pixel policy, with geometric rather than
   byte-identical cross-renderer comparisons.
4. **Lidar only when required by the retained mobile task:** scan timing,
   per-ray pose convention, range/return policy, and map/fixture alignment.

Each sensor family needs a calibration fixture, deterministic golden stream,
nominal error budget, boundary case, and at least one injected failure. New
sensor APIs are not accepted until timestamp, latency, noise, saturation, and
failure behavior are testable without rendering.

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

This is a sustained investment area rather than a one-off OpenArm demo. RNE
will treat the complete sampled-data loop as the product surface: physical
plant, sensor sampling and transport, estimator, controller, actuator
realization, and safety limits. Controller algorithms remain plugins or
reference evidence; the engine owns the typed timing, units, constraints,
experiments, comparison, and replay contracts that make those algorithms
auditable.

The control track must also distinguish model classes honestly. Frequency-domain
margins are reported only for a declared linear operating model and validity
range. Nonlinear, saturated, hybrid, or time-varying behavior is evaluated with
bounded rollouts and explicit counterexamples rather than being summarized by
an invalid single margin. Solver-internal state may never stand in for a
controller-visible measurement or estimator output.

### Engineering quality gates

Sensor and dynamics work is promoted by measured gates rather than feature
presence. The requirements registry must assign each check to one of the
following gates and report the first failing gate:

| Gate | Required evidence |
|---|---|
| Measurement integrity | Timestamp/phase, units, frame and calibration provenance, latency distribution, saturation/drop/stuck behavior, and truth-aligned bias/RMSE are within named limits. Controller-visible samples remain separate from privileged truth. |
| Estimation validity | Required states have an observability argument, innovation and residual statistics, convergence/recovery time, and a deterministic sensor-fault case. An estimator may not silently substitute backend state. |
| Plant integrity | Mass, center of mass, inertia, friction/damping, transmission, limits, actuator realization, timestep, and operating point are retained from source to trace without an undocumented fallback. |
| Identification validity | Excitation is bounded and sufficiently informative; training and validation windows are disjoint; model order and delay are explicit; residual/prediction metrics and the valid operating region are reported. |
| Closed-loop performance | Stability evidence, tracking and disturbance-rejection metrics, margins where applicable, saturation/anti-windup exposure, and the smallest failing robustness case all satisfy fixed requirements. |
| Portability | The same observation/action/controller artifacts run through the advertised backends and recorded/shadow paths; differences are classified as measurement, estimation, realization, or plant divergence. |

A new sensor family cannot pass on clean truth-only fixtures, and a new
controller cannot pass on terminal pose alone. Promotion requires both a
nominal case and a deliberately failing case whose first violated requirement
is reproducible from retained artifacts.

Dynamics work follows a control-engineering model stack:

1. **Parameter integrity:** preserve mass, center of mass, inertia tensor,
   joint friction/damping, limits, transmission, and actuator parameters from
   source asset to backend evidence. Reject non-physical inertias and report
   every fallback or default.
2. **Open-loop characterization:** deterministic step, ramp, impulse, and chirp
   excitation with saturation-aware input records. Estimate delay, bandwidth,
   static gain, damping, natural frequency, and cross-axis coupling over named
   operating regions.
3. **Model identification and validation:** keep training and validation
   trajectories separate; retain their hashes; compare declared physics,
   linearized state-space, and fitted models using residual whiteness and
   unit-bearing prediction metrics.
4. **Controller synthesis and analysis:** establish PID first, then use
   state-feedback or LQR only when controllability, observability, operating
   range, and estimator assumptions are evidenced. Report stability margins,
   saturation exposure, anti-windup behavior, tracking metrics, and disturbance
   rejection.
5. **Closed-loop robustness:** sweep payload, inertia, friction, sensor bias,
   delay, drop, rate, and actuator degradation within declared bounds. Retain
   the smallest counterexample and its first violated contract.

### Tangible proof targets

This track must be visible as a working control and measurement laboratory, not
only as new APIs or unit tests. The following four artifacts are the user-facing
milestones, in dependency order:

1. **OpenArm feedback lab:** one headless command runs the same typed-feedback
   controller on Rapier, native MuJoCo, and Gazebo and produces an HTML report
   with synchronized reference, observation, command, saturation, latency, and
   backend-delta plots. A deterministic dropout must identify the same first
   unavailable observation and fail closed on all three runners.
2. **OpenArm plant lab:** bounded step, ramp, and chirp experiments produce
   per-joint time-domain metrics, empirical frequency response, cross-axis
   coupling, and training/validation data for a declared local plant model.
   Every plot must be reproducible from the retained machine-readable report;
   screenshots are not evidence.
3. **IMU estimator lab:** a stationary fixture and a prescribed-motion fixture
   show truth, raw measurement, and estimated state together, including bias,
   random walk, innovation, latency, and dropout recovery. The same fixture
   must emit both a passing report and a minimized estimator failure capsule.
4. **Robustness envelope:** a browser dashboard shows pass/fail regions over
   payload, friction, inertia, sensor delay/drop, and actuator degradation. A
   selected failing point opens its exact trace, first violated requirement,
   replay, and model/config hashes rather than only an aggregate heat map.

### Committed sensor and dynamics expansion

After the existing bias boundaries, work proceeds in the following order. Each
stage ends in a versioned experiment, fixed requirements, a passing case, a
smallest failing case, and portable evidence before the next stage starts.

1. **Sensor timing and availability:** sweep consecutive dropout length,
   latency, jitter, stale age, and recovery timing. Define fail-closed and
   bounded hold/predict policies explicitly, and prove that raw publication,
   controller-visible observation, and estimator state remain distinguishable.
2. **Actuator realization dynamics:** add measured command/position/velocity/
   effort provenance, authority loss, rate limiting, deadband, saturation,
   transport delay, and anti-windup evidence. A requested torque or position is
   never reported as realized effort without a qualifying measurement.
3. **Physical-plant uncertainty:** sweep payload mass and center of mass,
   inertia, joint friction/damping, transmission efficiency, and cross-axis
   coupling. Preserve the exact modified parameters in every trace and classify
   divergence as plant error rather than controller or sensor error.
4. **Identification and frequency response:** expand the isolated joint-5 lab
   into declared per-joint and coupled MIMO operating regions. Retain excitation
   sufficiency, coherence, residual whiteness, confidence bounds, and disjoint
   validation data; reject a model outside its evidenced range.
5. **State estimation:** evaluate velocity, disturbance, and IMU-derived state
   estimates using innovation/residual statistics, convergence and recovery
   time, and consistency measures such as NIS/NEES where their assumptions hold.
   No estimator may read privileged backend truth in its execution path.
6. **Constrained closed-loop control:** compare cascaded PID and justified
   state-space/LQR baselines first, then consider feed-forward, disturbance
   observers, or MPC only when the plant and estimator evidence supports them.
   Comparisons use identical observations, actuator limits, references, and
   perturbations and report tracking, effort, saturation, and robustness—not
   only terminal pose.

The immediate execution queue starts from the already retained actuator
realization diagnostics, Gazebo plant matching, payload sweep, and
actuator-authority envelope. The next slices are:

1. **Complete:** add a source-step-verifiable actuator command-transport delay across
   Rapier, MuJoCo, and Gazebo at the shared boundary after controller limits
   and before backend actuation;
2. **In progress:** the physical command slew-rate, command-deadband, and
   viscous joint-damping boundaries are complete; continue the actuator and
   plant envelope with Coulomb friction, inertia, and transmission-efficiency
   sweeps, retaining the smallest failing value for each mechanism;
3. expand sensor timing evidence from the completed fixed dropout case to
   latency, deterministic jitter, stale age, burst dropout, recovery policy,
   quantization, saturation, and stuck-value envelopes;
4. turn the joint-5 plant lab into per-joint and coupled operating-region
   identification with uncertainty, coherence, residual, and held-out
   prediction evidence;
5. compare constrained PID and justified state-space control using identical
   typed observations, limits, references, and perturbations;
6. then advance camera/depth and metric 3DGS calibration, followed by lidar
   only when a retained navigation task consumes it.

Sensor dropout/recovery at the typed controller boundary, actuator realization
diagnostics, payload robustness, and actuator-authority degradation are already
complete. Camera/depth and 3DGS calibration remain the next geometric-sensor
stage; they do not displace the joint-feedback and control-loop work above.

The labs use one versioned experiment manifest and one requirements registry.
The registry owns hard limits and engineering targets; report builders may not
derive pass thresholds from observed solver spread. Frequency-domain claims
must record excitation amplitude, sample period, window, leakage treatment,
frequency grid, estimator method, and coherence or an equivalent validity
measure. State-space claims must record state/output definitions, operating
point, discretization method, validity range, and controllability/observability
evidence.

Progress is counted by closed evidence loops. A milestone is not complete until
its manifest, deterministic fixture, report schema, browser rendering, nominal
case, intentional failure, replay/hash check, and short documentation example
are all present. This prevents sensor breadth or controller variety from
outrunning measurement quality.

The primary OpenArm benchmark is not considered complete until it includes a
payload-free baseline and at least one declared payload, multi-axis coupling,
sensor-in-the-loop feedback, effort saturation, a disturbance or parameter
variation, and backend comparisons using the same compiled action/controller
artifact. Pass/fail limits must come from URDF/actuator contracts or a named
requirements registry, never from observed backend spread.

The first OpenArm report is intentionally stricter than the existing final-pose
proof. It evaluates the complete trajectory with RMSE, IAE, ISE, terminal bias,
peak velocity, and measured position range for every joint. URDF position and
velocity limits are hard contracts with explicit SI-unit epsilon values. A run
that eventually reaches the goal still needs tuning when its transient response
overshoots a hard limit or exceeds the registered tracking bound; the threshold
must not be widened merely to make a backend pass.

The first plant-identification fixture isolates OpenArm right joint 5 before
moving the remaining arm joints with the joint-5 reference held constant. An
ARX(2,2) model is fitted only to the isolated, deterministic training window and
is evaluated without refitting on the coupled validation window. The baseline
evidence localized the original Rapier discrepancy: isolated tracking passed,
but coupled motion amplified joint-5 error and eventually crossed its URDF hard
position limit, while Gazebo remained within the same contracts. Passing the
URDF mass without its centre of mass and inertia tensor had changed the plant.
The corrected fixture passes both windows after binding exact URDF inertial
properties and an interior reset reference; no tolerance or actuator effort was
increased.

### Ordered implementation slices

1. **Complete -- physical parameter integrity:** preserve exact URDF mass,
   center of mass, and inertia tensor through the backend-neutral rigid-body
   contract; reject invalid tensors; eliminate implicit collider mass; bind the
   robot asset configuration into traces and capsules. Re-run the OpenArm
   joint-5 isolated/coupled identification and full pose-cycle regressions.
2. **Complete -- joint-feedback measurement contract:** version the observation
   schema for joint position, velocity, effort, command realization, saturation,
   sample phase, and age. Produce nominal, one-step-delay, dropout, stuck-value,
   and effort-saturation golden streams and the first headless
   `sensor-validation-report`.
3. **Complete -- sensor-in-the-loop control:** remove privileged
   backend-state feedback from the OpenArm reference controller. Run the same
   observation contract and controller artifact through Rapier, MuJoCo, and
   Gazebo, retaining each backend-derived command stream and the first
   measurement, realization, or plant divergence. All three runners now pass
   the same final-pose, controller-reproduction, and hard-dynamics contracts.
4. **Complete -- IMU contract and estimator fixture:** implement stationary and
   prescribed motion tests, seeded bias/noise/random walk, physical mount-frame
   and lever-arm validation, latency, saturation, dropout, and stuck-value cases,
   then add a deterministic complementary-filter reference estimator with
   innovation and consistency metrics.
5. **Complete -- open-loop plant suite:** add bounded step/ramp/chirp experiments and a
   versioned experiment manifest. Generate frequency-response data where
   applicable, time-domain metrics, coupling matrices, and train/validation
   datasets for the OpenArm joints and gripper.
6. **Complete -- model and controller suite:** add deterministic linearization
   and controllability/observability checks, validate the current ARX path, and
   compare PID plus one justified state-space baseline under identical limits,
   delay, and disturbance conditions.
7. **In progress -- robustness envelope:** execute seeded sweeps over payload, inertia,
   friction, sensor latency/bias/drop, and actuator degradation. Report the
   verified operating envelope and minimize the first failing case rather than
   averaging failures away.
8. **Geometric sensors:** validate the flagship camera/depth calibration and
   3DGS metric alignment with reprojection, depth, occlusion, and semantic-pose
   metrics. Add lidar only if the retained mobile task consumes it.
9. **Portable evidence:** package at least one measurement failure and one
   closed-loop dynamics failure into browser-readable Failure Capsules; replay
   both through recorded and shadow paths before expanding to another robot or
   sensor family.

Slice 2 now has an additive typed `JointFeedback` baseline with explicit SI
units, scheduled capture time, phase error, DataBus availability latency,
sequence-visible dropout, stuck-value status, and the unconstrained versus
limited actuator command. Backend effort is deliberately reported as
`Unavailable` until a backend supplies a qualifying measured realized effort;
command reconstruction is not mislabeled as measurement. The headless OpenArm
`sensor-validation-report` now binds the sensor contract and all input hashes,
retains deterministic nominal/replay/dropout/stuck stream hashes, identifies
drop sequence 307 at the first observable gap and stuck sequence 307 from its
first status, records 2,788 observable saturated channel-samples in the
sensor-in-the-loop run, and verifies the per-joint PID integrator anti-windup
bounds. Its static HTML companion is self-contained and browser-readable.
Slice 3 advances that measurement boundary into the controller execution path
described below.

The Rapier OpenArm trace now exercises the slice-3 observation boundary with a
one-control-period latency: scoring consumes only `latest_available` frames and
records capture, availability, consumption, and observation-age ticks. Each
frame also retains its position target, limited effort command, and saturation
state. A zero-delta calibration check against the backend reference is retained
separately from the controller-visible signal. The trace now passes the
TaskSpec's exact integer fixed-step duration into the plant instead of claiming
`16,666,667` ticks while running the generic 60 Hz integer-divided
`16,666,666`-tick default.

Slice 3 now has an artifact-defined, bounded joint-space PID reference
correction that consumes only typed, available joint feedback and clamps each
integral state independently. Rapier, native MuJoCo 3.9.0, and Gazebo 8.15 each
execute 1,798 feedback decisions after two declared bootstrap frames. The
cross-simulator report independently recomputes every correction and emitted
target from the retained observation and controller artifact, with zero timing
mismatches and at most `8.9e-16 rad` numerical reproduction delta. The final
reference errors are `0.004235 rad` for Rapier, `0.004991 rad` for native
MuJoCo, and `0.003525 rad` for Gazebo. The maximum pairwise final joint-position
delta is `0.006015 rad`; all remain inside the unchanged `0.01 rad` gates.
MuJoCo uses native implicit joint damping while preserving the same exact
bounded total-effort law, eliminating the unstable explicit-damping integration
without adding a backend-specific controller or effort limit.
Both native traces now use a versioned portable state digest that includes
articulated joint coordinates/velocities and rigid-body pose/velocity. Each
1,800-step run produces 1,800 distinct step digests and an exact replay-final
match; the report rejects a constant articulated-state hash and does not claim
that solver-private state is equal across backends.

Slice 4 adds a versioned `ImuFeedback` observation that names raw gyroscope and
specific-force units, scheduled capture, phase error, availability latency,
per-axis saturation, and stuck-value status without mislabeling truth
orientation as measurement. `ImuMount` binds the sensor axes and physical
lever arm to a rigid body; tangential and centripetal acceleration are included,
and invalid or missing calibration fails all due publication atomically.
Dropped and stuck observations advance the physical IMU state while affecting
only the declared output boundary.

The headless `rne-imu-validation` lab runs stationary and prescribed-roll
fixtures at 100 Hz through a complementary reference estimator. Its deterministic
JSON/HTML report records timing checks, phase-specific RMSE, maximum error,
normalized innovation squared, three-sigma coverage, complete trace hashes, and
the first sensor failure. The nominal stationary and motion RMSE values are
`0.000461 rad` and `0.001580 rad`, inside registered `0.01 rad` and `0.025 rad`
limits. Nominal reruns are hash-identical, while dropout and stuck-value cases
both localize the intended first violation at sequence 650.

Honoring the exact TaskSpec period exposed a real terminal tracking failure
(`0.0150 rad` against `0.01 rad`). The controller now reaches its unchanged
return-home target at step 1500 instead of 1580, leaving 300 steps for settling
inside the unchanged 1800-step episode. Without increasing tolerance, gain, or
effort limits, the feed-forward baseline produced Rapier/Gazebo final errors of
`0.005588 rad` and `0.001070 rad`, with a final cross-backend delta of
`0.006075 rad`. The sensor-in-the-loop successor improves those values as
recorded above without changing the TaskSpec tolerance or actuator effort
limits. The retained Rapier run contains 1800 typed observations with zero
sample-phase error and an exact one-period (`16,666,667 ticks`) observation age
for every frame.

Slice 6 retains the Rapier-identified ARX model without refitting, proves the
declared augmented system controllable and its dynamic output state observable
with known input history, and places four stable discrete poles. PID and
state-feedback use identical reference, observation latency, correction and
integral bounds, actuator limits, and a one-second `+0.03 rad` actuator-target
bias applied after controller limits. The controller sees that realization
error only through typed delayed feedback. Across Rapier, MuJoCo, and Gazebo,
state feedback limits the disturbance peak to `0.0111-0.0125 rad`, recovers
inside the fixed `0.005 rad` band in `0.20-0.233 s`, and holds IAE to
`0.00448-0.00576 rad*s`; each is inside the predeclared requirements. The
report independently reproduces controller output, disturbance, and applied
plant target for all 21,600 decisions with no realization mismatch.

Slice 7 now has its first measured dimension. A fixed
`[0.00, 0.03, 0.06, 0.09, 0.12] rad` actuator-target bias grid holds the
state-feedback controller and every other contract constant. Rapier identifies
`0.03 rad` as the last passing point and `0.06 rad` as the smallest grid
failure. MuJoCo and Gazebo reproduce that boundary: all three pass at
`0.03 rad`, then first fail only the unchanged `0.02 rad*s` IAE requirement at
`0.06 rad` while peak-error and recovery gates remain green. The Rapier trace
localizes the first cumulative crossing to step 3292 at `0.020305 rad*s` and
retains it as a dedicated behavior replay and verified 49-artifact Failure
Capsule. Slice 7 remains in progress until payload, inertia, friction, and
actuator-authority dimensions are evaluated under the same boundary rules.

The joint-position sensor-bias dimension now preserves raw typed feedback and
records the separate delayed position consumed by the controller. A fixed
`[0.00, 0.01, 0.02, 0.04, 0.06] rad` nominal-status bias grid yields
`0.01 rad` as the last pass and `0.02 rad` as the first failure on Rapier,
MuJoCo, and Gazebo. All three first fail the same `0.02 rad*s` IAE requirement;
the first crossing occurs at step 3303 on Rapier and step 3304 on MuJoCo and
Gazebo. Each boundary trace proves exactly 60 biased controller decisions and
at most `3.47e-18 rad` realization error. The retained 43-artifact Capsule is
bound to its producer and stops its replay at the first failed measurement.
The publication-dropout dimension uses a fixed `[0, 1, 2, 3, 4]` consecutive-
frame grid and a three-period maximum-age contract. All three backends pass two
drops and first fail three. At the failing boundary they reject exactly decision
3244 at `66,666,668 ticks`, freeze state, hold the previous accepted target with
zero delta, and recover in one decision on fresh sequence 3243. The earliest
violation is still capture sequence 3242, where the third consecutive
publication is absent; the minimum replay stops there. Physical parameter
sweeps, actuator-authority degradation, command delay, command slew-rate limits,
and command deadband are recorded below; friction/damping, inertia, and
transmission-efficiency boundaries remain open.

The first physical joint-loss dimension now separates URDF plant damping from
actuator servo damping. Its predeclared `[0, 2.5, 5, 10, 20] N*m*s/rad` joint-5
grid holds Coulomb friction at zero and binds identical TaskSpec, controller,
actuation, scene, world, adapter, and action hashes across Rapier, native
MuJoCo, and external Gazebo. All 15 runs are exact same-runtime replays with
zero independently parsed model-parameter delta. The shared controller passes
5 on every backend. The declared 10-point fails only MuJoCo RMSE
(`0.021031 rad > 0.02 rad`), and 20 fails all three. A browser report and
step-3600 minimum replay retain this honest `needs_tuning` boundary. Coulomb
friction, inertia, and transmission-efficiency dimensions remain open.

The command slew-rate fixture clamps each joint-5 applied target against the
previous applied target using the fixed control period. Its descending
`[0.40, 0.25, 0.15, 0.10, 0.05] rad/s` grid produces a real cross-backend
boundary rather than a declaration-only failure: `0.15 rad/s` passes on
Rapier, native MuJoCo, and Gazebo with 43/42/38 limited applications, while
`0.10 rad/s` first violates the fixed minimum at step 1298 with 60/59/57
limited applications. All six boundary traces have zero independently
recomputed realization delta. The deterministic browser-report SHA-256 is
`d4efb1cd9214420e24445a7804b2bb32532d6c8fa6ebfa0c7a5683e1a6f5f540`.

The command-deadband fixture recursively holds the previous applied joint-5
target whenever the controller-command gap is inside a declared physical
deadband. Its fixed `[0, 0.00025, 0.0005, 0.001, 0.002] rad` grid uses steps
882 through 941, where all three backends physically approach both sides of
the requirement boundary. Rapier, native MuJoCo, and Gazebo pass at `0.001 rad`
with 28/31/29 held applications and first fail only the fixed actuator
requirement at `0.002 rad` with 38/40/40 held applications. The maximum held
command gaps span `0.982-0.999 mrad` at the passing boundary and
`1.787-1.962 mrad` at the failing boundary. All six traces have zero
independently recomputed realization delta, while peak error, IAE, and recovery
remain green. The deterministic browser-report SHA-256 is
`258a62ac880a6d9c5d6c22fbae9a577cdc0b6934b7351640ee4449c3fb097475`.

The payload dimension now has a deterministic model-fixture compiler. It emits
the same per-case URDF, robot asset, scene, and Gazebo runtime manifest for a
fixed `[0, 0.10, 0.25, 0.50, 0.75] kg` grid. A rigid payload box is lumped into
`openarm_right_ee_base_link` with the parallel-axis theorem, so mass, center of
mass, and the complete inertia tensor change together instead of being
emulated as a target offset. Rapier and native MuJoCo show distinct,
deterministic responses to these model hashes. The payload runtime now also
fixes the Gazebo base to the world and uses bounded effort-PD realization over
ten physics substeps per control period. It runs the complete 3,600-decision
trace deterministically and responds to payload mass, replacing the previous
mass-insensitive velocity-servo result.
The adapter now records the command actually issued on every physics substep,
including raw/applied value, command kind, position error, and saturation count,
in a deterministic sidecar without extending the strict external-simulator wire
v1 schema. The baseline localized the failure: joints 3--7 saturated for roughly
`79--99.4%` of substeps and joint 5 saturated for `99.425%`. Replacing the
single global PD scale with an explicit per-joint gain map reduced the best
evaluated baseline joint-5 RMSE from `0.36322 rad` to `0.03276 rad`, final
joint-5 error to `0.01283 rad`, and overall final maximum error from `0.2125 rad`
to `0.06687 rad`, while keeping the URDF `7 N*m` wrist effort limit unchanged.
That intermediate candidate was still rejected by the fixed requirements; an
evaluated target-velocity feed-forward variant also regressed and was rejected.

Substep evidence then identified the mechanism rather than treating gain search
as sufficient. Across step, ramp, chirp, multisine, and coupling windows, joint
5 alternated at the `+/-7 N*m` limit while its control-period mean remained only
about `0.32--0.36 N*m`; measured velocity repeatedly reached the URDF
`20.944 rad/s` limit. A declared `0.02 s` backward-Euler first-order low-pass in
the derivative path reduced the derivative feedback peak to about `2.5 rad/s`.
With the same filter and a joint-5 stiffness of `120 N*m/rad`, the chirp window
saturation fell from `92.725%` to zero and RMSE from `0.02197 rad` to
`0.01175 rad`. Step-doublet saturation remains measurable at `0.1%`, preserving
a bounded nonlinear excitation case. The adapter records measured and filtered
velocity separately so this improvement cannot be mistaken for a sensor change.

The regenerated 15-trace matrix now passes. Rapier, MuJoCo, and Gazebo pass the
fixed joint-5 `0.02 rad` RMSE and `0.005 rad` final-error gates from zero through
the declared `0.50 kg` capacity. For Gazebo, RMSE spans `0.00504--0.00604 rad`,
final error stays at or below `0.00431 rad`, and saturation stays at or below
`0.128%`. The `0.75 kg` case passes every non-capacity requirement on all three
backends and is classified only as the expected capacity failure. The TaskSpec,
outer state-feedback controller, requirements, and URDF effort limits remain
unchanged. Two clean report builds produced the same SHA-256
`bc7796627627708034ca623d899fbd08d7a201be6d23584a97d1d4ee30602d88`.

The actuator-authority dimension now compiles the zero-payload fixture into a
fixed `[1.0, 0.8, 0.6, 0.4, 0.2]` joint-5 effort-scale grid. Each case emits a
strict native actuation config and matching Gazebo adapter/runtime manifest;
MuJoCo now accepts the same explicit `--actuation-config` boundary as Rapier.
All three backends pass the declared `0.6` supported minimum. At `0.2`
(`1.4 N*m`), MuJoCo is the first backend to cross the unchanged `0.02 rad` RMSE
gate at `0.02101 rad`; Rapier remains below the gate and Gazebo remains at
`0.00570 rad` while its substep saturation rises from `0.128%` to `5.125%`.
The report distinguishes the declared out-of-envelope status from the first
actual tracking failure instead of forcing the backends to share an invented
failure point. All 15 outcomes replay exactly and the deterministic report hash
is `7f48895bcd5f74afc1d5edd4b0acf0849810dd263a9c32a78cd9ec26072a476a`.

The actuator command-transport dimension now applies a fixed `[0, 1, 2, 3, 4]`
control-period delay to right joint 5 after controller limits and before each
backend's actuation boundary. The declared supported maximum is two periods;
three is the first out-of-envelope case on Rapier, native MuJoCo, and Gazebo at
step 3241, selecting controller source step 3238. The report independently
recomputes the source relationship from retained controller targets for all
six boundary traces. Every delayed application covers exactly 60 steps with
zero realization delta, while the performance peak, IAE, and recovery gates
remain green and distinct from the transport-contract result. All backend
rollouts replay exactly, and two clean report builds produced SHA-256
`ba623928997ffddf081538e622ee3882d2fb9363d7b3c210482d302c6068f7dd`.

### Track definition of done

The hardening track is complete only when:

- the OpenArm loop consumes typed sensor observations instead of hidden backend
  state, while every command is checked against the same actuator contract;
- one command produces nominal and deliberately failing sensor/control reports
  headlessly on Rapier, MuJoCo, and Gazebo;
- reports identify the first bad timestamp, field, command, or state and classify
  it as measurement, estimation, actuator, or plant divergence;
- deterministic reruns reproduce report and replay hashes, while declared
  stochastic tests reproduce from `WorldRandom` seeds;
- URDF limits and physical parameter checks pass without tolerance inflation or
  backend-specific exceptions;
- a payload/disturbance robustness envelope and frequency/time-domain control
  evidence are retained with named SI-unit requirements;
- the same schemas accept at least one recorded observation stream and one
  process-isolated shadow run; and
- both a sensor fault and a dynamics/control fault are independently inspectable
  from verified Failure Capsules.

## Stop conditions

The strategy is reconsidered rather than hidden behind more implementation if
either condition persists after two external onboarding attempts:

- a new user cannot reach a verified capsule within 15 minutes because the
  workflow is intrinsically too complex; or
- users do not value cross-backend or sim-to-real failure evidence enough to
  integrate the TaskSpec contract.

In that case, gather the onboarding evidence, narrow the contract, and choose
a smaller user problem before adding more simulator features.

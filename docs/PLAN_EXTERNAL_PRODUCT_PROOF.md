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
pinned MuJoCo runtime. The installed proof now also retains real controller-time
observations/actions for exact playback, Rapier-to-MuJoCo shadow comparison,
and a bounded disconnect, all under the same mobile-manipulation TaskSpec and
inside the verified Failure Capsule. Clean tagged Windows/Linux archive evidence and the
15-minute external measurement remain the open parts of Gate 1; the public
intake path now verifies rather than trusts that submission.

The first Windows archive diagnostic is now retained in
`docs/evidence/windows-installed-release-rehearsal-v1.json`. Two independent
deterministic ZIP writes produced identical bytes, all twelve checks passed
again from a separate extraction, and the extracted flagship reached its
verified 23-artifact capsule in 10.408 seconds. This deliberately remains
non-qualifying local evidence: the source commit was clean but not bound to an
expected release tag, and no independent external operator performed the run.

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

The Dr Johnson fixture now provides the first fail-closed partial geometric
baseline. It rehashes the official Deep Blending COLMAP inputs, two real
reference images, the committed splat manifest and PLY; verifies registered
camera reprojection, six semantic landmarks and floor alignment; and projects
the pickup collision proxy into the retained real-image rug polygon. A retained
same-camera RNE render now passes fixed raw-PSNR, luminance-correlation, and
gradient-correlation limits, with the validator recomputing every metric from
the content-addressed images. The same registered camera now produces a
content-bound alpha-composited proxy-depth frame: all six semantic landmarks
are observed with `0.148006` source-unit mean absolute error. A second real
camera adds 42 spatially distributed shared tracks; 40 match in both RNE depth
frames with `0.035302` source-unit depth-delta MAE and zero false occlusions
across 80 matched observations. Seven of eight contracts pass. It intentionally
remains non-qualifying, and the depth
report explicitly refuses a metre claim, because COLMAP scale lacks an
independent physical anchor; that anchor is the remaining geometric-sensor
deliverable and may not be inferred from visual plausibility.

The metric-anchor intake contract and operator procedure are now implemented.
They bind role-separated operator identity, method, UTC capture time, SI
distance and uncertainty, two exact registered COLMAP observations, and hashed
raw evidence. Both the fixture generator and Rust auditor recompute the scale
and fail closed. The upstream reconstruction archive has no metric metadata, so
completion now requires a genuine field measurement; no synthetic substitute
is accepted.

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

Current retained OpenArm evidence satisfies this software-side gate with one
exact Rapier playback, one strict Rapier-to-MuJoCo shadow divergence, and one
predeclared sequence-900 transport disconnect. All three runs use the same
TaskSpec/controller identity, suppress every submitted action, emit zero
actuator writes, and are independently replayed from a verified Failure
Capsule. This is recorded simulator evidence, not physical-device evidence;
Gate 4 still requires a bounded reference-device run.

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

This gate is currently open for a semantic reason, not only because physical
bytes have not yet been captured. The existing LeKiwi reference runner is bound
to `rne.lekiwi_so101.base_shadow.v1` at 30 Hz, while the installed flagship is
bound to `rne.flagship.mobile_lift_shared_aisle.v1` and its unchanged
`rne.ai.ik_mobile_lift_pick_place_policy.v1` controller at the simulation
control period. The former uses base velocity actions in `m/s` and `rad/s`; the
latter emits wheel velocity in `rad/s` plus arm, lift, and gripper actions. A
passing base-only LeKiwi physical-evidence manifest is therefore necessary
reference-device safety evidence, but it is not the same-contract Gate 4 proof
and must never be relabelled as such.

Gate 4 proceeds through these ordered implementation slices:

1. Define a versioned, content-addressed hardware projection contract that
   binds the full parent TaskSpec and controller bytes, the physical TaskSpec
   and profile bytes, every observation source, every action destination, and
   the exact transform configuration.
2. Make control-rate adaptation explicit. The retained report must name input
   and output rates, timestamp domains, hold/interpolation policy, maximum age,
   and the first dropped or stale sample; an implicit 60-to-30 Hz conversion is
   forbidden.
3. Make the wheel-to-kiwi realization explicit. Wheel radius, geometry,
   coordinate convention, unit conversion, saturation, and independently
   recomputed command residuals are hashed evidence rather than hidden adapter
   constants.
4. Assemble controller-visible observations in the unchanged parent TaskSpec
   order. Every element is classified as physical, calibrated physical,
   simulated, or unavailable; physical camera and joint channels retain their
   calibration and latency. Unavailable controller-required data fails closed.
5. Run an elevated physical shadow with the parent controller and zero actuator
   writes, then a low-speed live projection in which only predeclared base
   authority is enabled. Arm, lift, and gripper outputs remain explicitly
   suppressed until their own physical safety evidence exists.
6. Retain one passing bounded run and one intentional safety stop in a verified
   Failure Capsule. The capsule binds both TaskSpecs, controller, projection,
   profile, calibration, model/configuration hashes, wire trace, first contract
   violation, SI-unit tolerances, replay, and browser inspector.
7. Extend readiness acceptance only after the verifier replays that complete
   closure. The existing base-only `reference_hardware` artifact cannot satisfy
   this flagship-specific gate by itself.

The first action-boundary portion of slice 1 is now implemented as
`rne_hardware_lekiwi::flagship_projection`. It consumes all seven flattened
parent action elements, independently validates the parent and physical
TaskSpec limits, converts the two semantic wheel speeds through the canonical
mobile-manipulator radius and track width, and emits LeKiwi body-x/body-y/yaw
commands without clamping. Commands outside the `0.1 m/s` or `pi/6 rad/s`
physical envelope fail closed. The five arm/lift/gripper values are retained as
explicitly suppressed evidence beside a deterministic parent-action SHA-256.
This is not yet a live-ready bridge: content bindings for the complete
TaskSpec/controller/profile files and parent-order observation fusion remain
open portions of slices 1 and 4; the rate boundary is defined below.

The rate-boundary portion of slice 2 is now implemented as the wall-clock-free
`FlagshipLeKiwiRateScheduler`. It consumes exact zero-based parent sequences,
emits phase-zero even actions at an integer-exact `33,333,334 ns` period, and
retains intervening validated actions as explicit suppression evidence.
Missing, duplicate, reordered, overflowing, or invalid inputs fail without
advancing state. Parent-order observation fusion and complete file-content
bindings remain open; the scheduler alone does not authorize a live run.

The parent-order portion of slice 1 is now implemented as the stateful
`FlagshipLeKiwiObservationFuser`. It requires separately identified and
tick-stamped physical, localization, perception, traffic, and task-state
sources; validates freshness and monotonic continuity; applies an explicit
three-joint morphology calibration; and emits the exact 19-value release
observation with source-age, unused-physical-value, and deterministic-hash
evidence. It refuses zero-filled perception and never assumes SO-101 matches
the simulation arm. The remaining closure is to hash and rehash the actual
TaskSpec, controller, profile, calibration, localization, perception, traffic,
and task-state configuration files and bind these three boundary artifacts into
one shadow-run manifest. No actuation is authorized by fusion alone.

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

The third-party controller-plugin intake is now mechanically ready for that
independent run. Its schema-v1 acyclic candidate binds the exact release
archive, library, manifest, conformance report, command statuses, and committed
logs while keeping the containing Git revision separate. The maintainer-side
`external-plugin-check` rehashes every downloaded byte, requires clean
repository `HEAD`/origin and exact committed candidate/log bytes, validates the
typed passing report and negotiated controller identity, and emits the
registered schema-v1 staging report without loading the untrusted shared
library on the maintainer workstation. This closes an intake-tooling gap; it
does not count as the required independent plugin evidence until a genuinely
external owner submits and maintainers sandbox-rerun it.

The external simulator intake now has the same fail-closed path. Its acyclic
candidate retains the official release, adapter, TaskSpec, runtime manifest,
ordered world/robot/config files, normalized arguments, conformance report,
and committed logs. `external-simulator-check` binds those bytes to a clean
external Git revision and emits the registered schema-v1 maintainer report
without executing the untrusted adapter. Readiness v7 requires that complete
12-digest chain. This makes the next concrete adoption action an independent
Gazebo adapter submission rather than another in-repository simulator demo.

The in-repository Gazebo reference now also runs its ten-check process
conformance in dedicated Ubuntu 22.04 CI against pinned Gazebo Harmonic 8.15.0
packages. CI byte-compares the freshly generated report with the committed
report, closing the earlier gap where a semantically changed adapter could
leave a stale passing digest behind. This reference still does not count as
independent adoption; it makes the kit presented to the external operator
fresh and mechanically reproducible.

The two external-project slots now use an equally strong submission boundary.
`external-project-check` requires the official release archive, a clean
independently owned Git revision, an acyclic candidate, committed command logs,
the typed TaskSpec, and every verified Failure Capsule member. It additionally
requires the Capsule's `rne_task_spec` digest to equal the submitted TaskSpec.
Readiness manifest v8 rehashes the resulting seven-artifact chain and reparses
the maintainer report. This is intake readiness, not external adoption; both
slots remain empty until two unassisted external owners perform real tasks.

## Immediate implementation slices

Work proceeds in this order, one mergeable slice at a time:

1. **Delivered:** rename the installed release report's generic check
   collection so it no longer implies that the indoor flagship is shipped.
2. **Delivered:** define a versioned installed-flagship proof report and one
   archive command that produces success, expected failure, browser inspector,
   and capsule.
3. **Delivered:** stage the minimum flagship assets and runner in the native
   release bundle.
4. **Delivered and version-guarded; tagged evidence pending:** add Windows and
   Linux archive-install rehearsals for the proof command. The `v0.2.*` tag
   trigger now matches release `0.2.0`, and both packaging entry points reject
   future workflow/version drift before building an archive.
5. **Measurement path delivered; independent run pending:** use
   `rne-flagship-proof OUTPUT --cross-backend --measure-on MACHINE
   --verify-installed-bundle .` to retain a schema-v2 timing artifact covering
   exact installed-payload verification through the bound proof, then collect
   it on a named external reference machine.
6. **Delivered:** ship the MuJoCo-enabled proof runner and pinned runtime in
   both native archives and require the cross-backend result in extracted
   release rehearsal.
7. **Delivered; first submission pending:** ship an archive-only external
   reproduction quickstart and non-acceptance candidate JSON in every native
   bundle, bind the proof to its producer executable and clean tagged archive,
   reject CI/placeholder measurements, and expose a validated public issue
   route. Candidate schema v2 removes both impossible content-hash cycles: the
   candidate stays outside the proof bundle, and its containing Git revision is
   supplied separately. Maintainer verification rehashes the candidate, proof
   bundle, logs, release archive, and installed proof into report schema v2.
   The independent operator no longer needs an RNE source checkout; maintainers
   run the source-side acceptance checker only after submission.
8. Ask the first external operator to run the next tagged release without
   maintainer intervention and audit the retained report.

The tag publisher now promotes both platform attestation bundles and attested
archive-install reports from expiring Actions artifacts into permanent Release
assets. A deterministic release-level `SHA256SUMS` covers the four primary
archive/wheel assets and those four evidence files before publication. The
release exit gate rejects removal, publication before checksumming, or an
eight-file partial set. This makes the next `v0.2.0` tag durable enough for the
independent operator and the later 183-day readiness audit; it does not create
the tag or claim an external run occurred.

The installed runner now closes the remaining one-command ambiguity. Before it
creates output, it verifies the internal `SHA256SUMS` against the exact regular-
file graph and rejects extra files, symlinks, path escapes, duplicates, missing
members, and digest drift. The resulting schema-v1 verification report is bound
into installed-proof schema v4, time-to-proof schema v2, and the Failure Capsule;
the 15-minute interval therefore includes verification instead of beginning
after an unmeasured platform-specific checksum step.

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
2. **Complete:** the physical command slew-rate, command-deadband, viscous
   joint-damping, and regularized-Coulomb boundaries are complete. The Coulomb
   slice includes predeclared controller selection, a fixed 15-run
   Rapier/MuJoCo/Gazebo sweep, exact replay, a browser report, a minimum failure
   replay, and a verified Failure Capsule;
3. **Transmission complete; inertia pilot complete:** the motor-to-joint
   transmission-efficiency slice now separates bounded motor command from
   joint-side transmitted and measured effort across Rapier, MuJoCo, and
   Gazebo. Its fixed 15-run grid supports efficiency `>= 0.90`, retains `0.75`
   as the first predeclared-RMSE boundary failure, and emits exact replay, a
   browser report, and a verified Failure Capsule. A link-5-only inertia-tensor
   multiplier pilot over `[1, 2, 4, 8, 16]` remained performance-green and
   showed weak sensitivity on this trajectory; no boundary was manufactured
   with nonphysical multipliers. The successor now excites the seven-joint
   coupled mode across the official arm-only and pinch-gripper product presets,
   with source hashes and positive-definite tensor checks;
4. **Latency, jitter, stale-age selection, bounded recovery, and repeated re-arm complete:**
   controller-ingress latency has a fixed `[0, 1, 2, 3, 4]`-period sweep with
   original capture timestamps and a portable zero/one-period boundary.
   Deterministic jitter has a separate one/two-period boundary and an
   independently recomputed periodic schedule. Stale-age selection now proves
   the two/three-frame age boundary, 60-decision hold/freeze, and one-decision
   fresh recovery without changing publication or availability. The recovery
   successor fixes a three-publication trigger and sweeps additional
   fresh-observation holds `[0, 1, 2, 3, 4]`; all three backends recover in
   `[1, 2, 3, 4, 5]` decisions and first violate the one-decision requirement
   at step 3246. The repeated-burst successor fixes two two-frame gaps and
   sweeps interburst fresh frames `[4, 3, 2, 1, 0]`; one fresh frame passes and
   zero first fails the portable re-arm-spacing requirement at sequence 3242.
   The joint-5 position-quantization successor now retains raw typed feedback,
   sweeps `[0, 0.001, 0.002, 0.004, 0.008] rad`, and proves a portable
   `0.002/0.004 rad` boundary at controller step 3241 with zero realization
   delta. The joint-5 saturation successor retains raw feedback, sweeps the
   descending `[0.08, 0.06, 0.05, 0.04, 0.03] rad` symmetric range, and proves
   a portable `0.05/0.04 rad` boundary at the first observable clamp on
   controller step 941 with one real clamp per backend and zero realization
   delta. The status-aware stuck-value successor retains raw feedback, sweeps
   `[0, 1, 2, 3, 4]` held frames, and proves the portable two/three-frame
   boundary at controller step 904 with safe target hold, frozen state,
   one-decision recovery, and zero realization delta;
5. **Complete:** the seven-joint successor uses isolated multisine training
   regions and one simultaneous held-out coupled region on Rapier, MuJoCo, and
   Gazebo. Its 21 typed position/velocity/action models are full rank and retain
   uncertainty, coherence, residual diagnostics, coupled gain matrices, and
   fixed 149-check evidence without validation refit;
6. **Complete:** constrained PID and justified state-space control use identical
   typed observations, limits, references, and perturbations in the retained
   controller lab;
7. **Complete:** use the coupled operating region for the physically sourced,
   dynamically sensitive official arm-only versus pinch-gripper inertia
   experiment. Advance camera/depth and metric 3DGS calibration next. Add lidar
   only when a retained navigation task consumes it.

Sensor dropout/recovery at the typed controller boundary, actuator realization
diagnostics, payload robustness, and actuator-authority degradation are already
complete. Camera/depth and 3DGS calibration now have a fail-closed 7/8 Dr
Johnson fixture; an independent metric anchor remains the next geometric-sensor
stage. It does not displace the joint-feedback and control-loop work above.

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
MuJoCo keeps passive plant damping in native implicit joint dynamics while
realizing typed actuator damping inside the same exact bounded control law as
stiffness. Passive regularized-Coulomb effort is a separate generalized plant
force; it is not folded into native actuator-force measurement or used to
cancel damping outside the declared effort limit.
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

The multijoint identification successor now closes the next control-dynamics
slice. Seven isolated 480-step multisine regions train 21 backend/output models;
one simultaneous 720-step region is held out without refit. Rapier, native
MuJoCo, and Gazebo all produce full-rank `22/22` typed position/velocity/action
designs. Minimum diagonal coherence is `0.99485`, maximum validation RMSE and
cross-backend RMSE delta are approximately `0.0002073 rad`, and the largest 95%
prediction half-width is below `0.000562 rad`. The report retains raw residual
autocorrelation and permits its numerical-exactness path only below the fixed
`1e-8 rad` residual-RMSE floor. All 149 checks pass, and the three intentional
action-width failures agree at step 307. The coupled region is the excitation
basis for the completed physically sourced configuration experiment.

The physical-configuration successor uses exact upstream `right_arm` and
`right_arm_with_pinch_gripper` presets at commit
`1fba2cbc05001f05b4514120b70130b4ac06f409`, not an arbitrary tensor scale. The
official gripper adds `0.239699047367 kg`; every model link passes independent
positive-definite and rigid-body inertia checks. The same seven-axis TaskSpec,
controller, actuation, and byte-identical actions run both configurations on
Rapier, MuJoCo, and Gazebo with exact replay. Held-out response RMS deltas are
`0.0108432`, `0.0104996`, and `6.02431e-6 rad`; coupled-gain matrix Frobenius
deltas are `0.0544213`, `0.0541451`, and `2.71880e-5`. All 30 fixed checks pass,
and all six intentional failures remain at step 307. This closes the local
coupled-mode physical inertia gate without hiding Gazebo's lower sensitivity.

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
publication is absent; the minimum replay stops there. The controller-ingress
latency dimension retains every publication and capture timestamp while adding
`[0, 1, 2, 3, 4]` periods after typed availability. Total visible age is
therefore one through five periods. All three backends pass zero added periods
and first leave the declared envelope at one; Rapier and MuJoCo also fail the
fixed joint-5 performance gates there, while Gazebo remains performance-green.
At three and four added periods, the three-period maximum-age policy rejects
stale observations and proves zero-delta target hold with frozen controller
state. The portable first failure is localized to the first delayed controller
consumption at step 4 and retained as a minimum replay. The deterministic
jitter successor varies only controller-ingress availability over capture
sequences 3241–3300 using an `N`-delayed/one-nominal periodic schedule. Its
fixed `[0, 1, 2, 3, 4]`-period grid passes one and first fails two periods on
Rapier, native MuJoCo, and Gazebo. Both boundary points remain within the
three-period age limit with zero rejections and green joint-5 performance;
step 3244 retains capture sequence 3240 at `50,000,001 ticks`, independently
proving the first two-period jitter violation. Three and four periods exercise
the stale hold/freeze policy separately. The independent stale-age selection
dimension keeps all publications and base availability nominal, then selects
the `N`th older already-available sequence over controller steps 3243–3302.
Its fixed grid passes two additional frames at exactly `50,000,001 ticks` and
first fails three at step 3243 with sequence 3238 and `66,666,668 ticks` on all
three backends. The failing boundary rejects exactly 60 decisions, holds and
freezes with zero delta, then recovers on the latest available sequence in one
decision at step 3303. The independent dropout-recovery dimension fixes the
three-frame publication gap that enters fail-safe, then varies only additional
fresh-observation holds. Its grid yields one through five recovery decisions.
All three backends pass immediate recovery at step 3245 and first fail the
one-decision bound at step 3246 with one `recovery_confirmation_pending` hold.
The boundary retains one age rejection, zero target and integral-state delta
 across both frozen decisions, exact replay, and green joint-5 RMSE/final-error
 gates. The repeated-dropout re-arm successor fixes two two-frame bursts and
 varies only their fresh separator over `[4, 3, 2, 1, 0]`. Rapier, native
 MuJoCo, and Gazebo pass one fresh frame and first fail zero at sequence 3242;
 the zero-separator case merges into four missing publications while RMSE and
 final-error gates stay green. The quantization successor then rounds only the
 joint-5 controller-visible position over 60 decisions while retaining each raw
 typed source. All three backends pass `0.002 rad` and first fail `0.004 rad` at
 step 3241 with zero independently recomputed realization delta and green
 performance gates. The saturation successor then clamps only the same raw-
 source-derived controller-visible joint-5 position over steps 882--941. Its
 descending range grid passes `0.05 rad` and first fails `0.04 rad` at the
 first observable clamp on step 941
 on all three backends; the failing case contains one actual clamp per backend,
 zero realization delta, and green tracking gates. The status-aware stuck-value
 successor then holds only the controller-visible joint-5 sample while retaining
 raw typed feedback. All three backends pass two consecutive stuck frames and
 first fail three; the Rapier failure occurs at step 904 while consuming
 sequence 902 from held source 899. Every stuck decision safely holds the prior
 target and freezes state, the first fresh sample recovers in one decision, and
 independent reconstruction has zero realization delta. Physical parameter
sweeps, actuator-authority degradation, command delay, command slew-rate limits,
command deadband, viscous damping, and regularized Coulomb friction are recorded
below; the physically sourced, coupled-mode configuration boundary is complete.

The first physical joint-loss dimension now separates URDF plant damping from
actuator servo damping. Its predeclared `[0, 2.5, 5, 10, 20] N*m*s/rad` joint-5
grid holds Coulomb friction at zero. A separate predeclared
`[0.04, 0.05, 0.06, 0.08] rad` correction-limit grid uses only the MuJoCo
10-point as tuning data and deterministically selects `0.08 rad`; the selected
controller is then held unchanged across the complete validation grid. All 15
validation runs bind identical TaskSpec, controller, actuation, scene, world,
adapter, and action hashes across Rapier, native MuJoCo, and external Gazebo and
are exact same-runtime replays with zero independently parsed model-parameter
delta. The declared 10-point now passes at `0.013450`, `0.017185`, and
`0.009439 rad` RMSE. The first out-of-envelope point, 20, fails the same fixed
`0.02 rad` RMSE contract on all three at `0.021962`, `0.024173`, and
`0.021684 rad`. The browser report is `passed` and classifies those three rows
as `expected_boundary_failure`; the step-3600 replay retains the first Rapier
boundary failure. The portable regularized-Coulomb implementation now has a
complete 15-run Rapier/native-MuJoCo/Gazebo sweep over the frozen
`[0, 0.25, 0.5, 1, 2] N*m` grid. Transition width is a unit-bearing, hashed
`.rne.robot.toml` override because URDF cannot represent it. Earlier transition
and substep experiments did not recover the fixed `0.02 rad` RMSE gate. After
correcting the typed actuator law and freezing the current 19-substep fixture,
the predeclared `fast`, `baseline`, `medium`, and `slow` pole candidates produce
`0.016882`, `0.021732`, `0.030511`, and `0.039050 rad` on the Rapier 0.5 N*m
tuning case. The unchanged rule selects `fast`. That byte-identical controller
then passes all three backends through the declared 0.5 N*m envelope; model
realization and replay remain exact in all rows. Rapier and MuJoCo first fail
the unchanged RMSE gate at the 2 N*m out-of-envelope point (`0.023701` and
`0.023813 rad`), while Gazebo remains performance-green outside capacity. Every
actuator-effort row stays within the 7 N*m limit. The browser report is
`passed`; a step-3600 Rapier replay retains the first performance failure, and
a verified 30-artifact Failure Capsule binds the three focused traces, inputs,
hashes, diagnostics, and runner/report sources. The transmission-efficiency and
coupled-mode physical-configuration boundaries are complete.

The earlier exact-tick substep sweep is retained as negative evidence.
`[1, 2, 5, 10]` Rapier physics steps per 16,666,667-tick control period produce
`0.036139`, `0.047853`,
`0.324674`, and `0.084528 rad` RMSE. All replay and timing checks remain exact,
but none passes and higher substep counts degrade the force-based motor
response. No numerical setting was selected; the later fixed-fixture pole
selection and cross-backend validation above supersede it as the accepted
controller experiment.

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

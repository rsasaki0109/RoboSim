# Mobility Physical AI Foundation plan

Status: active, M3-B implemented; M3-C and later milestones remain

Implemented M0 evidence:

- wheel encoder payloads now read completed joint coordinates instead of actuator targets;
- kinematic wheels retain a realized integrated coordinate and restore it deterministically;
- `DiffDriveSim` installs per-wheel 60 Hz encoder streams and its observation/joint-state
  paths consume the latest available measurement rather than the newest command;
- `rne_ai::diff_drive_actor_observation` is a DataBus-only policy boundary that retains
  capture, availability, age, stream, and sequence metadata and fails closed when a
  required frame has not arrived;
- `DiffDriveEpisode` now consumes that strict actor frame from explicit localization,
  wheel-encoder, IMU, and LiDAR streams; mutating the live ECS pose cannot change its
  policy observation;
- sensor frames are captured after a completed physics tick with the completed tick's
  simulation timestamp, while the initial state is explicitly sampled at time zero;
- shared-world agents now use the same strict DataBus actor boundary for spawn, policy
  attachment, refresh, single-agent stepping, and multi-agent stepping. Exact ECS peer
  transforms remain available only through APIs documented as privileged diagnostics;
- a versioned canonical actor-input digest covers policy-visible values plus stream,
  sequence, capture, availability, and age metadata; identical seeded rollouts produce
  identical evidence digests.

The M0 policy-access inventory and exit-gate audit are complete. Any future peer feature
must be declared as a sensor or estimator stream rather than silently restoring exact peer
truth. The legacy localization stream remains a simulated measurement boundary backed by
pose truth; the M2-B estimator provides the sensor-only path and proves stale, delayed, or
missing inputs affect estimates without leaking ECS state.

M1-A is implemented: backend-neutral `DcMotorSpec`,
`TransmissionSpec`, and `WheelAssemblySpec` plus pure evaluators cover voltage/current
limits, back-EMF, optional inductance, explicit open/short failures, directional efficiency,
reflected inertia, and rolling resistance. The assumptions and identification requirements
are frozen in [`MOBILITY_PLANT_V1.md`](MOBILITY_PLANT_V1.md); no contact-backend integration
or tire-fidelity claim is included yet.

M1-B/M1-C are implemented. `ExternalBodyWrench` and the
`ExternalBodyWrench` physics capability define a one-step, world-frame force-at-point plus
free-moment boundary. Rapier implements it and conformance checks force response, lever-arm
moment, and automatic clearing. MuJoCo applies the same contract through a COM-shifted
Cartesian load, and the shared feature-enabled conformance catalog passes on both backends.
The exact semantics and primary-source basis are frozen in
[`MOBILITY_CONTACT_WRENCH_V1.md`](MOBILITY_CONTACT_WRENCH_V1.md). M1-D adds deterministic
point-contact load/kinematics acquisition and M1-E adds the transient combined-slip force
law; M3-B now composes those contracts into the first real backend loop.

M3-A adds a backend-neutral closed-loop longitudinal plant and a deterministic evidence
producer. Voltage drives motor current and torque through transmission and wheel inertia;
transient tire force accelerates the chassis and feeds back through wheel speed. Locked
rotor, acceleration, ice-like traction limiting, regenerative braking, open circuit, and
step convergence emit versioned SI-unit metrics and a stable content digest. Its scope and
research basis are frozen in
[`MOBILITY_LONGITUDINAL_BENCHMARK_V1.md`](MOBILITY_LONGITUDINAL_BENCHMARK_V1.md). This is
the analytic control baseline, not by itself a rigid-body fidelity claim.

M3-B now runs the exact same TaskSpec, seed, 1 ms clock, motor/transmission/wheel/tire
evaluator, contact acquisition, and external-wrench loop through Rapier and MuJoCo. Complete
traces retain backend manifests, privileged pose/velocity/contact telemetry, stability
metrics, unit-bearing cross-backend tolerances, and content digests. A deliberately strict
1 mm diagnostic records the first real position divergence as a Behavior replay and passes
through the standard Failure Capsule creator and verifier. The fixture is still an
equivalent single driven support path; per-wheel geometry, suspension/load transfer,
steering/yaw, skid scrub, split friction, grade, curb, and lift belong to M3-C. See
[`MOBILITY_BACKEND_CLOSED_LOOP_V1.md`](MOBILITY_BACKEND_CLOSED_LOOP_V1.md).

## North-star outcome

RNE must evaluate a policy through the same closed loop that exists on a real mobile
robot. A policy observes timestamped sensor frames, an estimator reconstructs state, a
controller requests actuator effort, the plant moves through wheel/ground interaction,
and sensors observe the result. Commanded state is never substituted for measured state.

```text
TaskSpec / policy
  -> estimator -> controller -> motor / transmission -> wheel / tire / ground
  -> rigid-body dynamics -> encoder / IMU / LiDAR / camera -> DataBus
  -> policy
```

The first benchmark release covers differential/skid-steer indoor robots and Ackermann
vehicles. The same TaskSpec, seeds, observations, actions, metrics, and evidence format
must run headlessly on Rapier and MuJoCo. Rendering is optional and is not evidence of
plant or sensor fidelity.

## Research decisions

The selected baseline is deliberately staged. One model cannot be simultaneously easy
to identify for a small indoor robot, valid at standstill, and sufficient for aggressive
road-vehicle handling.

| concern | primary evidence | RNE decision |
| --- | --- | --- |
| tire-force hierarchy | [Project Chrono tire models](https://api.chrono.projectchrono.org/wheeled_tire.html) separates rigid, handling, and finite-element tires and documents Fiala, Pac89/02, and TMeasy validity | expose a backend-neutral force-law contract; ship a low-cost transient combined-slip law before any high-parameter Pacejka profile |
| low-speed and combined slip | [TMeasy](https://doi.org/10.1080/00423110701776284) is a semi-physical low-frequency handling model; the published extension includes first-order force dynamics and standstill manoeuvres | use explicit relaxation state and low-speed regularization; do not divide raw lateral velocity by near-zero forward speed |
| mobile-robot friction | [MVSim paper](https://arxiv.org/abs/2302.11033) and [official physics documentation](https://mvsimulator.readthedocs.io/en/stable/physics.html) separate wheel-ground force models from planar collision dynamics and support differential/Ackermann plants | represent every driven/steered wheel, not only a chassis bicycle; include anisotropic skid-steer friction and spatially varying surface parameters |
| generic contact backends | [MuJoCo computation](https://mujoco.readthedocs.io/en/3.8.0/computation/) provides tangential, torsional, and rolling contact friction; [PhysX Vehicle](https://nvidia-omniverse.github.io/PhysX/physx/5.6.1/docs/Vehicles.html) separates slip, load, friction, and tire-force state | backends provide contact kinematics, normal load, and wrench application; an RNE force law must not expose backend handles or assume generic Coulomb contact is a tire model |
| motor and transmission | the [MuJoCo DC motor technical note](https://mujoco.readthedocs.io/en/latest/_static/dcmotor.pdf) derives back-EMF, current-limited torque-speed envelopes, optional inductance, thermal effects, cogging, and friction | M1 starts with an identifiable quasi-static DC motor plus gear ratio, efficiency, reflected inertia, current/voltage limit, viscous and Coulomb loss; electrical/thermal state is an opt-in fidelity tier |
| IMU stochastic errors | the [Kalibr IMU model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model) defines continuous-time noise density and bias random walk and their sample-period scaling | profiles store continuous-time SI densities and calibration matrices; sampling frequency changes must not silently change the physical noise process |
| sensor timing | [CARLA sensor reference](https://carla.readthedocs.io/en/latest/ref_sensors/) exposes sensor ticks, frame timestamps, transforms, LiDAR sweep geometry, attenuation, dropout, and camera distortion | every output retains capture time and availability time; sweep/readout timing, latency, phase, dropout, and calibration are observable contract fields |
| batched Physical AI | [MuJoCo MJX](https://mujoco.readthedocs.io/en/latest/mjx.html) uses batch dimensions for model randomization and high-throughput simulation | RNE randomizes typed physical parameters from explicit seeds and batches TaskSpec instances without changing single-world semantics |

These sources are model and interface references, not parity claims. Every RNE model
must state its assumptions, validity envelope, required identification data, and failure
conditions in its own rustdoc and benchmark evidence.

## Observation authority

Four data classes remain distinct:

1. **command**: controller request before plant limits and losses;
2. **measurement**: calibrated sensor output after sampling, quantization, faults, and
   transport delay;
3. **estimate**: state-estimator output derived only from available measurements;
4. **privileged truth**: simulator state for critics, metrics, and diagnostics only.

Actor policies and production controllers may consume measurement and estimate streams.
They must not read ECS transforms, rigid-body velocities, contact impulses, noise state, or
randomized truth. A privileged critic may consume explicitly declared truth tensors during
training. Evaluation reports both actor-visible and privileged schemas.

M0 policy-access inventory:

| execution path | actor input authority |
| --- | --- |
| `DiffDriveEpisode` and standalone diff-drive agents | strict localization, encoder, IMU, and LiDAR DataBus frame |
| vectorized diff-drive episodes | strict frame through each contained `DiffDriveEpisode` |
| shared-world single and multi-agent runners | strict frame refreshed only after available sensor publication |
| `DiffDriveSim::observe*`, peer helpers, contacts, separation, renderer | privileged diagnostic APIs; forbidden as actor input |
| SSL scripted behavior scenario | privileged task oracle outside the Mobility Physical AI benchmark; not a sensor-only policy claim |

The inventory is enforced by truth-mutation regressions for the built-in episode and
shared-world runner. Future policy-bearing paths must name their actor stream contract in
this table or fail review.

## Architecture boundaries

- `rne_robot`: motor, transmission, wheel, steering and mobility-plant components and
  commands; no physics-backend types.
- `rne_physics`: backend-neutral contact samples, normal loads, surface properties, and
  applied-wrench contracts.
- `rne_physics_rapier` / `rne_physics_mujoco`: contact acquisition, wrench application,
  completed-step joint/effort synchronization, and conformance evidence.
- `rne_sensor`: encoder, effort/current, IMU, LiDAR, and camera sampling pipelines.
- `rne_data`: unit-explicit versioned payloads, calibration metadata, capture/availability
  time, and stream status.
- `rne_ai`: actor/critic observation declarations, estimators, deterministic parameter
  randomization, batched rollout, rewards, and TaskSpec metrics.
- `rne_traffic`: topology and traffic semantics only; it does not own vehicle dynamics.
- adapters: ROS 2, recorded data, shadow, HIL, and live hardware translation only.

## Delivery milestones

### M0 — honest mobile closed loop

- Wheel encoders sample completed joint coordinates, never actuator targets.
- Kinematic wheels integrate an explicit realized coordinate; dynamic wheels use backend
  `JointState`.
- Mobile examples adopt typed joint feedback with command, coordinate, effort, capture
  phase, latency, dropout, and stuck status.
- A stalled-wheel fixture proves `commanded_velocity_rad_s != measured_velocity_rad_s`.
- A policy-access audit proves the controller cannot read privileged chassis truth.

Exit gate: headless deterministic tests reproduce the same DataBus bytes for the same seed,
and a command/measurement substitution regression fails.

### M1 — identifiable mobile plant v1

- `DcMotorSpec`: resistance, motor constant, supply voltage, current limit, rotor inertia,
  viscous/Coulomb loss, optional inductance, and explicit failure mode.
- `TransmissionSpec`: ratio, directional efficiency, backlash/deadband, compliance, and
  reflected inertia.
- `WheelAssemblySpec`: radius, inertia, rolling resistance, steering coordinate, and
  contact frame.
- `ContactPointSample` and `ExternalBodyWrench`: deterministic backend-neutral boundary.
- `CombinedSlipTireSpec`: longitudinal/lateral slip, relaxation length, load-sensitive
  force limits, friction ellipse, and regularized standstill behavior.
- Ackermann adds suspension/load-transfer state after the per-wheel contract is stable;
  skid-steer adds anisotropic lateral resistance and scrub loss.

Initial profiles use parameters obtainable from datasheets, coast-down, locked-rotor,
free-spin, straight-line acceleration/braking, and constant-radius tests. Pacejka and
deformable-soil models are extension profiles, not v1 prerequisites.

### M2 — proprioception and sensor-only estimation

- Encoder: counts/revolution, edge quantization, sampling phase, velocity-window method,
  index offset, direction inversion, dropout, stuck, and saturation.
- Motor feedback: terminal voltage, current, winding temperature when the corresponding
  plant tier exists; otherwise explicitly unavailable.
- Steering encoder: coordinate, rate, calibration, backlash contribution, and faults.
- IMU: reuse existing white-noise, bias, scale/misalignment, range, resolution, mounting,
  and lever-arm model; add profile import and Allan-deviation validation artifacts.
- Estimator: wheel/IMU odometry first, then optional GNSS/LiDAR corrections. Reset,
  initialization, covariance, stale inputs, and invalid outputs are explicit.

Exit gate: a benchmark controller receives only DataBus frames. Delayed or dropped frames
change the estimate and behavior without changing truth dynamics.

### M3 — Mobility Benchmark v1

Fixtures run under Rapier and MuJoCo with unit-bearing tolerances:

- motor locked-rotor, step, chirp, free-spin, and coast-down;
- straight acceleration/braking and reverse transition;
- constant-radius understeer and steering step;
- split-friction braking and acceleration;
- skid-steer pivot/arc with lateral scrub;
- grade, curb, rough surface, and wheel lift;
- encoder/IMU timing, quantization, bias, dropout, stuck, and saturation;
- sensor-only odometry with truth used only for final metrics.

Each run emits configuration, seed, backend/capability report, time series, stable hashes,
metric tolerances, and a Failure Capsule. Cross-backend agreement is required only inside
the declared validity envelope; unexplained exact equality is not a fidelity target.

### M4 — Physical AI API

- TaskSpec declares actor-visible, privileged-critic, and diagnostic-only tensors.
- Domain randomization covers mass/inertia, COM, motor constants, losses, tire/surface
  parameters, steering geometry, calibration, latency, and sensor faults.
- Random samples are recorded and reproducible from world/episode seeds.
- Vectorized rollout preserves single-world step ordering and produces per-environment
  evidence hashes.
- Curriculum difficulty changes physical ranges and tasks, not hidden controller access.

### M5 — sim-to-real proof

- Import recorded command/sensor/ground-truth logs through an adapter.
- Fit only declared identifiable parameters, with training/validation log separation.
- Replay the fitted profile in open loop, then shadow a controller on recorded streams,
  then run bounded HIL.
- Report trajectory, yaw-rate, wheel-speed, acceleration, current, timing, and dropout
  residuals with units and confidence intervals.
- Preserve the raw profile, fitted profile, tool version, data hash, residual plots, and
  failure cases in a reproducible evidence bundle.

Exit gate: Mobility Physical AI Benchmark v1 contains at least one differential/skid robot
and one Ackermann vehicle validated against real logs, with sensor-only policy evaluation
and cross-backend results.

## Planned pull-request sequence

1. M0-A: make legacy wheel encoder read realized coordinates and integrate kinematic wheel
   position; add stalled-wheel regression and this plan.
2. M0-B: add the strict DataBus actor-observation contract and measured wheel feedback.
3. M0-C: route the built-in episode through explicit localization, encoder, IMU, and LiDAR
   streams; align capture timestamps with completed ticks and add truth-mutation tests.
4. M0-D: migrate shared agents and peer observations, then add stable DataBus replay hashes.
5. M1-A: backend-neutral motor/transmission/wheel types and pure deterministic unit tests.
6. M1-B: contact/wrench contract plus Rapier implementation and conformance fixture.
7. M1-C: MuJoCo implementation of the same contract and cross-backend fixture.
8. M1-D: backend-neutral point-contact load/kinematics contract with Rapier/MuJoCo parity.
9. M1-E: transient combined-slip force law, split-friction and low-speed tests. Implemented
   as a pure backend-neutral force element; backend scenario integration is the next gate.
10. M2-A: encoder/steering/current payloads, calibration and fault pipeline. The
    additive incremental-encoder slice now provides finite counters, CPR/direction/
    zero/index calibration, count/time velocity reconstruction, timing, latency,
    dropout, stuck, and saturation; steering and current frontends remain.
11. M2-B: wheel/IMU estimator and sensor-only mobile TaskSpec. The additive estimator
    now consumes only available incremental-encoder and raw-IMU frames, reconstructs
    finite counters, exposes timing/gaps/faults and disagreement, and propagates planar
    covariance. Its actor observation and motor-voltage TaskSpec expose estimate,
    uncertainty, timing, health, and task goal without a truth tensor; truth-named
    reward/termination terms remain diagnostic scoring outside the actor contract.
12. M2-C: completed motor telemetry plus measured current/voltage/optional-temperature
    frontend. The additive path now models offset, seeded noise, measurement range,
    quantization, timing, latency, dropout, stuck, saturation, and explicit missing
    temperature without reading commands. Generic incremental encoders already cover
    revolute steering coordinates; integrated steering/backlash evidence remains.
13. M3-A: deterministic analytic longitudinal plant and stable benchmark report. Implemented
    as a control-oriented baseline; it does not yet claim rigid-body backend fidelity.
14. M3-B: implemented. The same exact TaskSpec runs through Rapier and MuJoCo with
    completed-contact feedback, next-step tire wrench application, complete time-series and
    capability evidence, unit-bearing comparison, diagnostic replay, and verified Failure
    Capsule.
15. M3-C: replace the equivalent support path with per-wheel differential/skid and
    Ackermann fixtures; add steering, suspension/load transfer, lateral scrub, split
    friction, grade, curb, roughness, lift/recontact, and sensor-only closed-loop metrics.
16. M4/M5: batched Physical AI observations/randomization, then real-log identification,
    recorded/shadow/HIL validation.

Every PR is independently headless-testable, documents new public contracts, runs format,
Clippy, workspace tests, and `xtask ci-headless`, and removes its isolated build directory
after evidence is captured.

## Explicit non-goals for v1

- claiming road-vehicle fidelity from a planar bicycle GIF;
- implementing a high-parameter tire formula without identification data;
- using generic collider friction as an undocumented tire model;
- adding ROS 2 or simulator-specific types to core crates;
- training actors on truth and relabeling the result sensor-only;
- measuring parity by visual similarity or exact cross-backend floating-point equality;
- adding unrelated robot/importer/showcase work before benchmark exit gates pass.

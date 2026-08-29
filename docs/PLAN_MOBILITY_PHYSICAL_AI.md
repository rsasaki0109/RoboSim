# Mobility Physical AI Foundation plan

Status: active, M0 in progress

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
- `WheelContactSample` and `WheelWrench`: deterministic backend-neutral boundary.
- `TransientCombinedSlip`: longitudinal/lateral slip, relaxation length, load-sensitive
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
2. M0-B: migrate the differential-drive example to `JointFeedback`; add actor-access audit.
3. M1-A: backend-neutral motor/transmission/wheel types and pure deterministic unit tests.
4. M1-B: contact/wrench contract plus Rapier implementation and conformance fixture.
5. M1-C: MuJoCo implementation of the same contract and cross-backend fixture.
6. M1-D: transient combined-slip force law, split-friction and low-speed tests.
7. M2-A: encoder/steering/current payloads, calibration and fault pipeline.
8. M2-B: wheel/IMU estimator and sensor-only mobile TaskSpec.
9. M3: benchmark matrix, tolerances, evidence bundle, and Failure Capsules.
10. M4/M5: batched Physical AI observations/randomization, then real-log identification,
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

# Mobility sensor-observed closed loop v1

This fixture closes RNE's first rigid-body mobility loop without giving the
controller or estimator direct access to physics state. The same exact TaskSpec,
sensor contract, estimator, and PI controller execute against Rapier and MuJoCo.
Backend truth is retained only under explicitly privileged fields for acceptance
scoring.

This is the sensor-observed subgate of M3-C. It is not yet a differential-drive,
skid-steer, or Ackermann vehicle-fidelity claim.

## Signal boundary

The one-way runtime path is:

```text
PI voltage command
  -> DC motor / transmission / wheel / transient tire force
  -> rigid-body backend and completed contact
  -> 2048-CPR wheel encoders + IMU + electrical feedback
  -> DataBus capture and availability times
  -> wheel/IMU estimator
  -> actor observation and next PI decision
```

The controller reads only `DataBus::latest_available` frames. It never reads a
`Transform3`, `RigidBody`, `JointState`, contact result, command echo, or future
frame. The estimator API likewise accepts no ECS world or truth state. The actor
observation contains estimate, covariance, provenance, health, measured motor
voltage/current, and target velocity; its TaskSpec rejects truth- or
privileged-named tensors.

Truth position and velocity are recorded separately for diagnostics and scoring.
That separation makes command, measurement, estimate, and privileged truth
independently inspectable instead of treating a physics-state read as a sensor.

## Frozen experiment

| property | value |
| --- | ---: |
| physics period | 1 ms |
| settle / driven duration | 0.3 s / 3.0 s |
| target velocity | 1.0 m/s |
| encoder resolution | 2048 decoded counts/revolution, signed 32-bit wrap |
| sensor capture rate | 100 Hz |
| capture-to-availability latency | exactly 2 ms |
| motor command range | -24 V to +24 V |
| controller | sensor-estimated-velocity PI, 15 V/(m/s) proportional and 20 V/m integral |

The encoder frontend quantizes completed wheel coordinates before reconstructing
velocity over a two-sample count/time window. The IMU frontend uses a fixed mount
transform, seeded continuous-time-density white noise, turn-on bias, bias
instability/rate walk, scale and cross-axis error, finite ranges, and
quantization. The electrical frontend adds calibration offset, seeded noise,
finite range, quantization, and the same availability latency. Dropout, stuck,
saturation, sequence gaps, capture time, and availability time remain observable
frontend state.

The estimator advances only for a newly available encoder pair. Re-reading a
stale frame cannot invent another integration step. A deterministic dropped left
encoder sequence is covered by a test: estimator health exposes the gap and the
controller holds until a complete synchronized set is available.

## Acceptance and reference evidence

Validation recomputes the exact contract, TaskSpec, ordered sensor sequences and
timestamps, actor observation shape, metrics, verdict, and content digest. It
rejects non-finite values, future or over-age inputs, truth leakage, tracking
outside the declared interval, metric drift, or artifact tampering.

The reference Windows run with Rapier 0.22 and MuJoCo 3.9.0 produced:

| metric | Rapier | MuJoCo | allowed cross-backend gap |
| --- | ---: | ---: | ---: |
| final estimated velocity | 0.920388 m/s | 0.920388 m/s | 0.10 m/s |
| final truth velocity | 0.925902 m/s | 0.925970 m/s | 0.10 m/s |
| truth velocity tracking error | 0.074098 m/s | 0.074030 m/s | 0.01 m/s |
| RMS position estimate error | 0.004319 m | 0.006422 m | 0.05 m |
| RMS velocity estimate error | 0.028119 m/s | 0.028506 m/s | 0.05 m/s |
| maximum controller input age | 0.002 s | 0.002 s | exact per trace |
| nominal estimator-health fraction | 0.996970 | 0.996970 | trace bound 0.95--1.0 |

The estimated-velocity acceptance interval is 0.85--1.15 m/s; truth velocity must
remain in 0.90--1.10 m/s and its absolute target error must not exceed 0.10 m/s.
These are regression bounds for the frozen fixture, not hardware validation.

Run the comparison with the MuJoCo native library configured:

```text
cargo run -p rne_mobility_benchmark --features mujoco -- \
  --backend sensor-compare \
  --output sensor-observed-comparison.json
```

Individual evidence is available with `sensor-rapier` and `sensor-mujoco`.
Rendering is neither required nor accepted as evidence.

## Seeded sensor reset experiment

`sensor-randomized-compare --seed 42` runs the same controller and nominal plant
with a reset sample retained in `contract.randomization`. Here `--seed` is the
lane-local episode seed; a batch caller derives it with `derive_episode_seed`.
The physics/noise seed remains zero to isolate the effect of sensor parameters.
Validation regenerates the reset sample and rejects parameter drift.

The bounded experiment samples a common baseline delay of 1, 2, or 3 ms,
then adds 0, 1, or 2 ms of keyed jitter for each capture. The jitter is common
to encoder, IMU, and current streams, modeling shared transport. Capture times
remain exactly 100 Hz; only DataBus availability changes. Frames already
published retain their original delay. The estimator accepts at most baseline
plus 2 ms of age. Independent stream jitter and capture-clock jitter remain open.

The physical gyro has a residual bias sampled uniformly in `[-0.002, 0.002)`
rad/s after the estimator's nominal calibration. Current offset is sampled in
`[-0.1, 0.1)` A. A single left-encoder attempted sequence in `50..=149` is
dropped after measurement, preserving internal encoder state and exposing a
sequence gap to the estimator. These are explicit engineering test ranges,
not distributions fitted to a named physical device.

The actor TaskSpec remains unchanged: reset parameters are retained in the
experiment contract, not supplied as actor observations. Repeatability tests
compare complete traces, require changing input ages and an observed sequence
gap, and run the same sampled contract on Rapier and MuJoCo with the existing
unit-bearing tolerances. Stuck/saturation randomization and camera/LiDAR reset
parameters remain subsequent work; the joint chassis reset is described below.

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
$env:TEMP = 'E:\RNE-build\tmp'
$env:TMP = $env:TEMP
$env:MUJOCO_DYNAMIC_LINK_DIR = 'E:\RoboSim-mujoco\lib'
$env:PATH = 'E:\RoboSim-mujoco\bin;' + $env:PATH
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend sensor-randomized-compare --seed 42 `
  --output E:\RNE-build\m3c-sensor\sensor-randomized-compare.json
```

Two independent seed-42 CLI runs, after correcting latency acceptance to use the
declared contract bounds, produced byte-identical 695,590-byte comparison files
(SHA-256 `B203D06E6BF3775271BF88C29BF6E1254ED9E3F86C9EE3CFDC0C7917DFF0151A`).
The sample used 3 ms base latency, up to 2 ms added jitter, a 0.000960431 rad/s
residual gyro bias, a 0.018579700 A current offset, and left-encoder dropout at
sequence 104. All cross-backend tolerances passed; the privileged final-distance
gap was 0.002387 m against 0.1 m. Both backend tasks also passed. These results demonstrate reproducibility for
this synthetic reset profile, not real-device calibration accuracy.

### Joint physical/sensor reset

`run_randomized_mobility_sensor_trace` uses the same explicit episode seed to sample
the existing physical profile and the sensor reset in separate keyed random domains.
The sampled mass/inertia, motor, transmission, wheel, tire, friction, and road grade
are applied before stepping. The controller gains and nominal wheel-radius calibration
remain fixed: the estimator does not receive the sampled physical radius. The actor
TaskSpec is unchanged; the frozen profile is retained only in the experiment contract.
Suspension stiffness/damping are retained but inactive in this rigid-support fixture.

Road-aligned gravity is applied to both the backend and the IMU through the new
`SensorGravity` resource. IMUs subtract that vector when computing specific force;
without the resource they preserve the legacy gravity default. This prevents a graded
physics world from silently producing flat-world accelerometer readings. The resource
is an environment input, not controller-visible data. Other fixtures must explicitly
set it when using non-default gravity.

This follows the episode-fixed hidden dynamics distinction in
[Peng et al., Sim-to-Real Transfer of Robotic Control with Dynamics Randomization](https://xbpeng.github.io/projects/SimToReal/SimToReal_2018.pdf).
It does not reproduce that paper's learned controller or establish real-world transfer.
The joint-reset tests exercise exact replay, profile integrity, unchanged actor tensors,
and Rapier/MuJoCo comparison without widening the existing acceptance tolerances.

Validation: both `joint_reset` tests passed with MuJoCo enabled; all 79 sensor
library tests passed, including a stationary IMU and zero-specific-force free fall
under a nonvertical gravity vector. Targeted sensor/Mobility Clippy also passed.
After setting the external-SSD environment above, generate the joint comparison with:

```powershell
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend mobility-sensor-randomized-compare --seed 42 `
  --output E:\RNE-build\m3c-sensor\joint-reset-a.json
```

`--seed` is the episode seed directly; `--lane-id`, `--episode-index`, and
`--num-envs` are rejected for this single-episode CLI. A batch caller must derive
lane-local seeds before invoking the runner. The CLI does not claim vectorized execution.

Two independent seed-42 joint CLI runs, after the latency-scoring correction,
produced byte-identical 705,897-byte outputs:
digest `fnv1a64:b5cc9293e13e63d9`, SHA-256
`B11CD5A148E4D20AF1BC1A36929E1BC61E8CBD6963365D55C268B6C859D4FA52`.
The physical sample used 98.608971 kg and -0.021837 rad grade. All seven existing
comparison tolerances passed, including a 0.002592 m privileged final-distance gap
and a 0.001227 m estimated-distance gap (each limited to 0.1 m).

**Backend agreement is not task success.** In this joint seed, both tasks fail the
unchanged velocity acceptance: final true speeds are 0.888280 and 0.888500 m/s,
below the 0.9 m/s lower bound. Their tracking errors exceed the 0.1 m/s limit.
The comparison's `passed` field describes only backend gaps; inspect each nested
trace's `passed` field for task acceptance. The CLI prints both verdicts separately
and preserves this reproducible failure instead of widening tolerances. The sensor-only
reset succeeds; the joint reset exposes a robustness gap for follow-on identification
and estimation work. Neither result establishes real-world fidelity.

### Voltage replay and remaining capsule gate

The failed joint trace is evidence, not yet a common `rne_failure_capsule` bundle.
That envelope requires an actual replay artifact. The existing generic actuator-log
replayer currently restores wheel velocity commands; this fixture commands motor
terminal voltage. Relabeling volts as wheel velocity or joint effort would change
the plant semantics and is not an acceptable bridge.

`replay_sensor_observed_trace` now reruns a validated trace on a fresh instance of
the same backend. It restores both reset profiles, starts with zero voltage, and
uses each timestamped recorded voltage for the next drive-path evaluation with
zero-order hold between decisions. The existing one-step wrench staging is unchanged.
The PI update is bypassed; sensors, estimation, drive dynamics, and backend physics
still execute. Decision times and the complete resulting trace, including privileged
scoring and its digest, must match. Recomputed hashes alone do not establish replay:
changing recorded voltages and rehashing the source is rejected when the physical
result differs. A reproducibly failed task remains failed after successful replay.

The remaining gate adds a bounded file reader/CLI and build provenance, then packages
the verified voltage replay and failed metric report as references in the common
capsule. The generic wheel-velocity replayer is not modified or used for this voltage
fixture. Cross-backend agreement and single-backend exact replay remain distinct checks.

Validation on Windows (2026-09-07): the frozen voltage-replay worktree completed
`cargo run -p xtask -- ci` with exit code 0, including workspace Clippy/tests,
example and RL smokes, headless checks, parity, fuzz, and 10/10 behavior seeds.
The separate MuJoCo-enabled voltage-replay tests passed for both backends.
Build output, CI log, and configured evidence root were on the external SSD;
some legacy tests still use short-lived checkout-local paths. This CI result
does not close the real-log identification, vectorized execution, or capsule gates.

## Research and OSS correspondence

- [Kalibr's official IMU noise model](https://github.com/ethz-asl/kalibr/wiki/IMU-Noise-Model)
  defines continuous-time accelerometer/gyroscope noise densities, bias random
  walks, and their sample-period scaling. RNE stores physical SI densities and
  advances seeded bias state at capture time, so changing sample rate cannot
  silently reuse a per-sample standard deviation as a physical parameter.
- [Kalibr](https://github.com/ethz-asl/kalibr) also treats spatial and temporal
  calibration as estimated quantities. RNE retains the IMU mount and calibrated
  gyro bias in the evidence contract; M5 must replace fixture values with fitted
  real-log profiles and residual evidence.
- [CARLA's official sensor reference](https://carla.readthedocs.io/en/latest/ref_sensors/)
  exposes per-sensor capture periods, frame timestamps, transforms, seeded IMU
  noise, and LiDAR noise/dropoff. RNE additionally separates capture time from
  availability time so transport/processing latency is enforceable at controller
  ingress rather than merely written as metadata.
- [Gazebo Sensors](https://gazebosim.org/api/sensors/9/classgz_1_1sensors_1_1Sensor.html)
  schedules sensor updates independently of the physics cycle, and its official
  sensor stack supplies noise models. RNE follows the independent schedule while
  requiring every frontend to declare latency and failure behavior.
- Borenstein and Feng, *Measurement and Correction of Systematic Odometry Errors
  in Mobile Robots*, distinguishes systematic wheel-geometry error from
  non-systematic contact error and introduces UMBmark calibration
  ([paper](https://ieeexplore.ieee.org/document/544770)). RNE exposes wheel
  calibration and encoder/gyro disagreement but does not yet claim UMBmark or
  surface-calibrated slip validation.
- Deray et al., *Joint on-manifold self-calibration of odometry model and sensor
  extrinsics using pre-integration*, estimates wheel geometry and sensor
  extrinsics with motion covariance
  ([paper and code](https://artivis.github.io/publication/deray-ecmr-19/)). RNE v1
  remains a smaller deterministic baseline and keeps online calibration as an
  explicit later gate.
- [Project Chrono's official tire hierarchy](https://api.projectchrono.org/wheeled_tire.html)
  separates rigid contact tires from handling models such as Fiala, TMeasy, and
  Pacejka and notes the need for transient slip states. RNE likewise keeps its
  transient tire-force element separate from generic rigid contact; this fixture
  must not be relabeled as a calibrated handling vehicle.
- Mandow et al., *Experimental kinematics for wheeled skid-steer mobile robots*
  ([DOI 10.1109/IROS.2007.4399139](https://doi.org/10.1109/IROS.2007.4399139)),
  models skid steering through experimentally identified slip/ICR behavior rather than
  assuming an ideal no-slip differential drive. Yi et al. then combine wheel encoders
  and a low-cost IMU for skid-steer motion and slip estimation
  ([DOI 10.1109/TRO.2009.2026506](https://doi.org/10.1109/TRO.2009.2026506)). RNE therefore
  retains all four physical encoder streams and IMU disagreement evidence instead of hiding
  front/rear wheel disagreement inside physics truth.
- [WPILib's DifferentialDriveOdometry](https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html)
  consumes left/right traveled distance plus gyro angle and explicitly warns that the result
  drifts. RNE's new `FourWheelSideEncoderFusion` is the preceding four-to-two measurement
  boundary: modular changes from the two encoders on each side are summed, the derived stream
  declares twice the physical counts per revolution and a signed 63-bit finite counter, and
  source latency, wrap, and sequence gaps remain observable before the existing wheel/IMU
  estimator consumes the pair.

## Remaining M3-C boundary

The four-wheel plant and side-fusion primitive are now connected to the sensor-only
estimator/controller in
[`MOBILITY_PER_WHEEL_SENSOR_CLOSED_LOOP_V1.md`](MOBILITY_PER_WHEEL_SENSOR_CLOSED_LOOP_V1.md).
The steering-feedback counterpart is documented in
[`MOBILITY_ACKERMANN_SENSOR_CLOSED_LOOP_V1.md`](MOBILITY_ACKERMANN_SENSOR_CLOSED_LOOP_V1.md).
Those fixtures have their own cross-backend and fault evidence; the seeded reset experiment
above currently applies only to this simpler longitudinal fixture, not automatically to
either successor.

The combined gate still requires evidence that identifies real physical and sensor
parameters and exercises them together under road and fault variation. M5 fits motor,
tire, geometry, sensor, and estimator parameters against training logs and scores
held-out real logs. Passing synthetic fixtures alone does not close that gate.

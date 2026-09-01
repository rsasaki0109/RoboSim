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

## Remaining M3-C boundary

The next gate replaces the equivalent driven support with named per-wheel
differential/skid and Ackermann fixtures. Acceptance must cover steering and yaw
response, lateral acceleration and scrub, per-wheel load transfer, split friction,
grade, roughness/curb interaction, lift/recontact, synchronized steering feedback,
and sensor-only closed-loop metrics. M5 then fits motor, tire, geometry, sensor,
and estimator parameters against training logs and scores held-out real logs.

The first half of that replacement is now implemented in
[`MOBILITY_PER_WHEEL_SKID_V1.md`](MOBILITY_PER_WHEEL_SKID_V1.md): four independent skid
wheel paths execute against both rigid-body backends. Its plant has not yet been connected
to this sensor-only estimator/controller, so the combined M3-C gate remains open.

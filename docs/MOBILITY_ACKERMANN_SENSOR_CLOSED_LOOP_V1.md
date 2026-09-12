# Ackermann sensor-only closed loop v1/v2

Status: implemented additive M3-C sensor/control subgate

This benchmark runs the same suspended four-wheel Ackermann plant, TaskSpec, sensor
contract, estimator, controller, seed, and 1 ms physics clock through Rapier and MuJoCo.
The current contract identity is `mobility_ackermann_sensor_closed_loop_v2`.
Unlike the open-loop dynamics fixture, the controller receives no chassis pose, velocity,
joint state, physics handle, contact, or command echo. Its only vehicle-state input is the
newest frame available on typed DataBus streams.

## Measured boundary

The physical frontend contains:

- four independent 2048-count/revolution wheel encoders;
- two 16384-count/revolution front steering encoders;
- four motor voltage/current/optional-temperature frontends;
- one body-mounted six-axis IMU;
- a 100 Hz capture schedule and 2 ms capture-to-availability delay for every stream.

Wheel and steering measurements are quantized from completed coordinates. Motor feedback
comes from completed motor telemetry, not requested voltage. The IMU includes calibrated
turn-on bias, scale/misalignment terms, stochastic error state, finite resolution, and
measurement ranges. Every frame retains capture time, availability time, sequence, and
frontend status. Injected wheel, steering, motor, and IMU drops are observable as independent
sequence gaps; the estimator waits for a synchronized fresh set, reports the gap in health,
and subsequently returns to nominal operation.

The steering profile follows the official
[AS5048A/AS5048B datasheet](https://look.ams-osram.com/m/287d7ad97d1ca22e/original/AS5048-DS000298.pdf):
14-bit angular output, programmable zero, roughly 11.25 kHz internal sampling, and roughly
100 us propagation delay. RNE uses the 14-bit quantization as a representative benchmark
profile while keeping the slower 100 Hz system-level capture and 2 ms transport budget
explicit rather than presenting them as chip limits.

The current frontend structure is based on the official
[TI INA240 datasheet](https://www.ti.com/lit/ds/symlink/ina240.pdf), which specifies
bidirectional PWM motor current sensing, finite gain/offset error, and 400 kHz bandwidth.
The benchmark's 10 mA output quantization, 20 mA offset, noise, range, and downstream timing
are declared RNE fixture parameters, not claims about an INA240-only circuit.

The IMU profile follows the official
[Bosch BMI088 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf):
16-bit axes, selectable ranges, finite resolution, and noise density. The continuous-time
error process remains defined by the RNE IMU contract and explicit seed.

## Estimator and controller

`rne_ai::AckermannImuOdometry` accepts only seven availability-time DataBus streams:
four wheel counters, two calibrated steering angles, and mounted-IMU feedback. It:

1. validates frontend status, finite counters, sequence advance, capture skew, and age;
2. reconstructs each wheel distance across finite counter wrap;
3. reconstructs curvature independently from left/right Ackermann steering geometry;
4. solves rear-axle-center travel from all four wheel paths;
5. fuses steering/wheel and IMU yaw increments and propagates planar covariance;
6. reports initialization, sequence-gap, saturation, steering-disagreement, and
   wheel/IMU-disagreement health without reading truth.

The actor observation contains estimate, covariance-relevant health/provenance, measured
steering/current, and target speed/yaw rate. Its action is four bounded motor voltages plus
a bounded center steering target. A PI speed controller and steering controller combining
kinematic feedforward with measured yaw-rate error use only the estimate and target.
In v2, that controller output passes through the shared 80 ms first-order steering
actuator with explicit 2.5 rad/s rate, travel, deadband, and stuck-failure semantics
before becoming the Ackermann joint target. The trace keeps the controller output,
completed actuator target, and measured steering encoder values separate and validates
the actuator target by deterministic step replay.
The TaskSpec declares privileged chassis distance, speed, and yaw rate in a separate
privileged-critic observation space. Decision/capture ticks are declared diagnostic-only.
Neither space is concatenated into the actor observation, and the privileged values are
used only for scoring in this benchmark.

## Reproduction and evidence

Keep Cargo and output paths on an external drive when local capacity is constrained:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3d-steering-actuator\target'
$env:MUJOCO_DYNAMIC_LINK_DIR = 'E:\RoboSim-mujoco\lib'
$env:PATH = 'E:\RoboSim-mujoco\bin;' + $env:PATH
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-sensor-rapier --output E:\RNE-build\m3d-steering-actuator\ackermann-sensor-rapier-v2.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-sensor-mujoco --output E:\RNE-build\m3d-steering-actuator\ackermann-sensor-mujoco-v2.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-sensor-compare --output E:\RNE-build\m3d-steering-actuator\ackermann-sensor-comparison-v2.json
```

The verified v2 comparison artifact is
`E:\RNE-build\m3d-steering-actuator\ackermann-sensor-comparison-v2.json`.
It is 2,396,192 bytes, has SHA-256
`6f488b583e0c00e7542edd726d747f04dfd559fd98cfb7da683e198ee24fad99`,
content digest `fnv1a64:6240923b09da7ef1`, and passes every bound:

| metric | Rapier | MuJoCo | absolute gap |
| --- | ---: | ---: | ---: |
| final forward distance | 2.116989 m | 2.120304 m | 0.003314 m |
| final estimated speed | 0.772789 m/s | 0.772850 m/s | 0.000062 m/s |
| final estimated yaw rate | 0.120209 rad/s | 0.119848 rad/s | 0.000361 rad/s |
| RMS speed estimation error | 0.033547 m/s | 0.033704 m/s | 0.000157 m/s |
| RMS yaw-rate estimation error | 0.001598 rad/s | 0.001599 rad/s | 0.000001 rad/s |
| RMS speed tracking error | 0.568626 m/s | 0.568067 m/s | — |
| RMS yaw-rate tracking error | 0.024815 rad/s | 0.024803 rad/s | — |

Repository tests require exact same-runtime Rapier repeatability, actor-schema truth
exclusion, recoverable wheel/steering/IMU dropout visibility and recovery, motor-drop
sequence evidence, mutation-digest rejection, and bounded Rapier/MuJoCo gaps.

Fatal motion-input faults use a separate fail-closed path. Wheel encoder stuck and finite
counter saturation, steering encoder stuck, and IMU stuck stop estimate-driven control and
emit a self-verifying capsule containing the exact TaskSpec, frontend/fault contract,
backend manifest, failure step, decision timestamp, every latest sensor sequence/status,
and a content digest. Generate any one of the eight v2 backend/fault artifacts with:

```powershell
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-sensor-failure-rapier --fault wheel-stuck `
  --output E:\RNE-build\m3d-steering-actuator\ackermann-rapier-wheel-stuck-failure-v2.json
```

Replace the backend suffix with `mujoco`, and select `wheel-stuck`, `wheel-saturated`,
`steering-stuck`, or `imu-stuck`. The retained external-SSD v2 evidence is:

| backend | failure | failed step | digest |
| --- | --- | ---: | --- |
| Rapier | wheel encoder stuck | 1992 | `fnv1a64:bceb36c0c4d71176` |
| MuJoCo | wheel encoder stuck | 1992 | `fnv1a64:148f8ed8c1d34030` |
| Rapier | wheel encoder saturated | 1712 | `fnv1a64:e688ffe74cb0e313` |
| MuJoCo | wheel encoder saturated | 1712 | `fnv1a64:0610c94ee2c6cbf1` |
| Rapier | steering encoder stuck | 1992 | `fnv1a64:78d4d45ee3f1e45d` |
| MuJoCo | steering encoder stuck | 1992 | `fnv1a64:09f33e4c9dcaf997` |
| Rapier | IMU stuck | 1992 | `fnv1a64:d10ce20e5eb51281` |
| MuJoCo | IMU stuck | 1992 | `fnv1a64:8dd77bb8ac5c2d67` |

## Explicit limits

This proves wiring, timing, truth separation, deterministic estimation, and cross-backend
closed-loop execution. It does not prove real-vehicle fidelity. Tire and suspension
parameters remain unidentified; the road is flat and rigid; steering backlash and thermal
current-sense drift are not yet identified; LiDAR/camera are not part of this low-level
control loop; grade, roughness, curb impact, lift/recontact, deterministic domain
randomization, and real-log residuals remain open gates.

# Ackermann sensor-only closed loop v1

Status: implemented additive M3-C sensor/control subgate

This benchmark runs the same suspended four-wheel Ackermann plant, TaskSpec, sensor
contract, estimator, controller, seed, and 1 ms physics clock through Rapier and MuJoCo.
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
Privileged chassis distance, speed, and yaw rate live only in separately named trace fields
used for scoring.

## Reproduction and evidence

Keep Cargo and output paths on an external drive when local capacity is constrained:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
$env:MUJOCO_DYNAMIC_LINK_DIR = 'E:\RoboSim-mujoco\lib'
$env:PATH = 'E:\RoboSim-mujoco\bin;' + $env:PATH
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-sensor-rapier --output E:\RNE-build\m3c-sensor\ackermann-sensor-rapier-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-sensor-mujoco --output E:\RNE-build\m3c-sensor\ackermann-sensor-mujoco-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-sensor-compare --output E:\RNE-build\m3c-sensor\ackermann-sensor-comparison-v1.json
```

The verified comparison artifact is
`E:\RNE-build\m3c-sensor\ackermann-sensor-comparison-v1.json`, digest
`fnv1a64:e8f2fe2738a55711`:

| metric | Rapier | MuJoCo | absolute gap |
| --- | ---: | ---: | ---: |
| final forward distance | 2.111777 m | 2.114389 m | 0.002612 m |
| final estimated speed | 0.753573 m/s | 0.750678 m/s | 0.002895 m/s |
| final estimated yaw rate | 0.118696 rad/s | 0.119607 rad/s | 0.000911 rad/s |
| RMS speed estimation error | 0.034167 m/s | 0.033748 m/s | 0.000419 m/s |
| RMS yaw-rate estimation error | 0.001650 rad/s | 0.001842 rad/s | 0.000192 rad/s |
| RMS speed tracking error | 0.569508 m/s | 0.568975 m/s | — |
| RMS yaw-rate tracking error | 0.019922 rad/s | 0.020200 rad/s | — |

Repository tests require exact same-runtime Rapier repeatability, actor-schema truth
exclusion, recoverable wheel/steering/IMU dropout visibility and recovery, motor-drop
sequence evidence, mutation-digest rejection, and bounded Rapier/MuJoCo gaps.

## Explicit limits

This proves wiring, timing, truth separation, deterministic estimation, and cross-backend
closed-loop execution. It does not prove real-vehicle fidelity. Tire and suspension
parameters remain unidentified; the road is flat and rigid; steering backlash and thermal
current-sense drift are not yet identified; fatal Ackermann sensor faults do not yet emit a
Failure Capsule; LiDAR/camera are not part of this low-level control loop; grade, roughness,
curb impact, lift/recontact, deterministic domain randomization, and real-log residuals
remain open gates.

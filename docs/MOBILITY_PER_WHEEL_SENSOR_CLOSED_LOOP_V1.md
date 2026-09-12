# Per-wheel skid sensor-only closed loop v1

Status: implemented additive M3-C subgate

This gate connects the four physical wheel paths from
[`MOBILITY_PER_WHEEL_SKID_V1.md`](MOBILITY_PER_WHEEL_SKID_V1.md) to real sensor
frontends, a DataBus-only estimator, and a yaw-rate controller. The exact
`mobility_per_wheel_sensor_yaw_rate_v1` TaskSpec executes against Rapier and MuJoCo.
Backend pose, contact, and rigid-body velocity never enter the estimator or controller.

## Signal boundary

```text
left/right voltage action
  -> four independent motor / transmission / wheel / tire paths
  -> rigid contact and chassis motion
  -> four 2048-CPR encoders + four motor-current frontends + mounted IMU
  -> capture timestamp + 2 ms availability latency
  -> four physical encoder streams
  -> modular four-to-two side fusion
  -> side-distance / IMU odometry
  -> yaw-rate PI actor
  -> next left/right voltage action
```

Each physical wheel retains its own count, wrap, sequence, motor current, motor noise seed,
and plant state. `FourWheelSideEncoderFusion` does not average simulator coordinates. It
reconstructs each finite counter change, sums the two changes on each side, and publishes a
signed 63-bit derived counter with twice the physical CPR. Re-reading the same source set
returns no update before stale-input checks, so time passing between 100 Hz captures cannot
turn an already-consumed frame into a controller fault.

The estimator receives only the two derived encoder frames and mounted IMU frame. During a
skid pivot, encoder yaw and IMU yaw intentionally disagree because lateral scrub is required.
The frozen 0.001 rad-per-update disagreement threshold corresponds to 0.1 rad/s at 100 Hz;
above it, yaw-rate estimation uses the IMU while retaining the wheel innovation and health
code as actor-visible slip evidence.

## Frozen experiment

| property | value |
| --- | ---: |
| physics period | 1 ms |
| sensor/controller period | 10 ms |
| capture-to-availability latency | 2 ms |
| settle / controlled duration | 0.5 s / 3.0 s |
| target yaw rate | 0.30 rad/s |
| physical encoders | 4 x 2048 CPR, signed 32-bit wrap |
| derived side encoders | 2 x 4096 CPR, signed 63-bit wrap |
| motor feedback | 4 independent measured-current streams |
| action | left/right motor terminal voltage, +/-24 V |
| controller | estimate-only yaw-rate PI, 21 V/(rad/s) proportional and 18 V/rad integral |

The TaskSpec exposes estimated yaw rate, wheel/IMU innovation, input age, health, four source
sequences, four measured currents, and task target. It rejects truth- or privileged-named
actor tensors. Integrated yaw and rigid-body yaw rate are stored only in explicitly privileged
scoring fields.

## Reference evidence

The verified artifact is generated outside the repository at:

```text
E:\RNE-build\m3c-sensor\per-wheel-sensor-comparison-v1.json
```

| metric | Rapier | MuJoCo | trace gate |
| --- | ---: | ---: | ---: |
| final estimated yaw rate | 0.4286 rad/s | 0.4111 rad/s | 0.15 to 0.50 rad/s |
| final privileged yaw rate | 0.4136 rad/s | 0.3910 rad/s | 0.15 to 0.50 rad/s |
| RMS yaw-rate estimation error | 0.0183 rad/s | 0.0192 rad/s | 0 to 0.25 rad/s |
| RMS truth tracking error | 0.2430 rad/s | 0.2466 rad/s | 0 to 0.25 rad/s |
| absolute integrated yaw | 0.4106 rad | 0.4241 rad | 0.3 to 3.0 rad |
| maximum measured motor current | 18.32 A | 17.90 A | 0.5 to 25 A |

Cross-backend gaps pass 0.20 rad/s final truth yaw rate, 0.50 rad integrated yaw,
0.15 rad/s RMS estimation error, and 0.15 rad/s RMS tracking error. Rapier replay is
exactly deterministic. Both traces and their comparison recompute TaskSpec, sensor contract,
full motor/transmission/wheel/tire profile, four station geometries, contact-load filter,
ordered decisions, metrics, verdict, and FNV-1a content digest.

A deterministic front-left encoder sequence drop is also executed on the physical plant.
Four-to-two fusion waits for a complete set, propagates the source gap into the derived side
sequence, and the estimator reports `InputSequenceGap` while retaining bounded sensor-only
control. A separate mutation test proves actor-evidence changes invalidate the trace digest.

The typed fault matrix now distinguishes recoverable transport/diagnostic faults from unsafe
motion-input faults:

| injected frontend fault | controller-visible evidence | runtime behavior |
| --- | --- | --- |
| encoder drop | physical and derived sequence gap, estimator health | hold action until next synchronized set |
| motor-current drop | motor sequence gap | continue; current is diagnostic input |
| motor-current stuck | `StuckValue` motor status and held measurement | continue with explicit degraded evidence |
| IMU drop | IMU sequence gap, estimator health | hold action until next synchronized set |
| IMU saturation | `Saturated` IMU status and `ImuSaturated` health | continue with encoder yaw fallback |
| encoder stuck / counter saturation | typed estimator error plus physical stream status | fail closed and emit a Failure Capsule |
| IMU stuck | typed estimator error plus mounted-IMU status | fail closed and emit a Failure Capsule |

Faults are applied after physical measurement and before declared output latency. They never
modify wheel state, rigid-body truth, or commands to manufacture the expected evidence.

Every fatal case emits a self-verifying JSON Failure Capsule containing the frozen backend,
TaskSpec, sensor/controller/fault contract, failure step and decision time, and latest sequence
and status evidence for all four encoders, all four motor channels, and the mounted IMU. Its
stable failure code is one of `encoder_stuck`, `encoder_saturated`, or `imu_stuck`; an FNV-1a
content digest detects any later mutation. Exact replay equality is tested independently for
all three cases.

Run the comparison with:

```text
cargo run -p rne_mobility_benchmark --features mujoco -- \
  --backend skid-sensor-compare \
  --output per-wheel-sensor-comparison-v1.json
```

Generate a fatal-input capsule (prefer an external output path for generated evidence):

```text
cargo run -p rne_mobility_benchmark --features mujoco -- \
  --backend skid-sensor-failure-rapier \
  --fault encoder-stuck \
  --output E:\RNE-build\m3c-sensor\encoder-stuck-capsule-v1.json
```

## Research correspondence and limits

Mandow et al. model skid-steer odometry through experimentally identified slip and
instantaneous-center behavior ([DOI 10.1109/IROS.2007.4399139](https://doi.org/10.1109/IROS.2007.4399139)).
Yi et al. combine four-wheel skid kinematics, wheel encoders, and a low-cost IMU for motion and
slip estimation ([DOI 10.1109/TRO.2009.2026506](https://doi.org/10.1109/TRO.2009.2026506)).
This gate implements the measurement boundary and explicit disagreement evidence, but it does
not claim their identified ICR model or EKF accuracy.

M3-C remains open for differential two-wheel-plus-caster behavior, identified suspension/load
transfer, Ackermann steering feedback, split friction, grade, curb, roughness, and
lift/recontact evidence. Fatal encoder and IMU input cases are now serialized rather than
retained only as test errors; future fixtures must use the same evidence contract.

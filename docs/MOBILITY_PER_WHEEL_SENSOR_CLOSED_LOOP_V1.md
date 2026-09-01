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
| controller | estimate-only yaw-rate PI |

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
| final estimated yaw rate | 0.1863 rad/s | 0.3363 rad/s | 0.15 to 0.50 rad/s |
| final privileged yaw rate | 0.1662 rad/s | 0.3412 rad/s | 0.15 to 0.50 rad/s |
| RMS yaw-rate estimation error | 0.0162 rad/s | 0.0093 rad/s | 0 to 0.25 rad/s |
| RMS truth tracking error | 0.2339 rad/s | 0.1703 rad/s | 0 to 0.25 rad/s |
| absolute integrated yaw | 0.4033 rad | 0.4687 rad | 0.3 to 3.0 rad |
| maximum measured motor current | 18.03 A | 20.08 A | 0.5 to 25 A |

Cross-backend gaps pass 0.20 rad/s final truth yaw rate, 0.50 rad integrated yaw,
0.15 rad/s RMS estimation error, and 0.15 rad/s RMS tracking error. Rapier replay is
exactly deterministic. Both traces and their comparison recompute TaskSpec, sensor contract,
full motor/transmission/wheel/tire profile, four station geometries, contact-load filter,
ordered decisions, metrics, verdict, and FNV-1a content digest.

A deterministic front-left encoder sequence drop is also executed on the physical plant.
Four-to-two fusion waits for a complete set, propagates the source gap into the derived side
sequence, and the estimator reports `InputSequenceGap` while retaining bounded sensor-only
control. A separate mutation test proves actor-evidence changes invalidate the trace digest.

Run the comparison with:

```text
cargo run -p rne_mobility_benchmark --features mujoco -- \
  --backend skid-sensor-compare \
  --output per-wheel-sensor-comparison-v1.json
```

## Research correspondence and limits

Mandow et al. model skid-steer odometry through experimentally identified slip and
instantaneous-center behavior ([DOI 10.1109/IROS.2007.4399139](https://doi.org/10.1109/IROS.2007.4399139)).
Yi et al. combine four-wheel skid kinematics, wheel encoders, and a low-cost IMU for motion and
slip estimation ([DOI 10.1109/TRO.2009.2026506](https://doi.org/10.1109/TRO.2009.2026506)).
This gate implements the measurement boundary and explicit disagreement evidence, but it does
not claim their identified ICR model or EKF accuracy.

M3-C remains open for stuck/saturation and motor/IMU fault cases on this exact plant,
differential two-wheel-plus-caster behavior, identified suspension/load transfer, Ackermann
steering feedback, split friction, grade, curb, roughness, and lift/recontact evidence.

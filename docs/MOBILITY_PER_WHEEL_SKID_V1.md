# Per-wheel skid backend gate v1

Status: implemented additive M3-C subgate

This gate replaces the single equivalent driven support used by the first mobility
backend traces with four named physical wheel stations. Each station owns independent
motor, transmission, wheel-inertia, transient-slip, and contact state. The same exact
`mobility_per_wheel_skid_pivot_v1` TaskSpec executes on Rapier and MuJoCo.

This is a backend and vehicle-plant gate. It is not yet the sensor-only differential-drive
policy gate, a compliant suspension claim, or an Ackermann handling claim.

## Runtime path

The one-way fixed-step path is:

```text
[left_voltage_v, right_voltage_v]
  -> four independent DC motor / transmission / wheel paths
  -> completed point contacts grouped by wheel entity
  -> raw solver load
  -> 20 ms load-bandwidth filter + 3x identified-load validity bound
  -> transient combined-slip tire law
  -> four next-step world-point wrenches
  -> rigid chassis backend
```

The wheel station contract is backend-neutral:

- `WheelStationSpec` declares the body-frame wheel center, zero-steer rolling/axle axes,
  steering axis, driven flag, and steering bound;
- `resolve_wheel_station_frame` rotates station axes with chassis pose and steering and
  includes `angular_velocity x lever_arm` in carrier velocity;
- contact identity remains the wheel entity, while a rigid station transmits its tire
  wrench to the owning free chassis at the same world point;
- motor, transmission, wheel, and tire states remain independent for all four wheels.

## Why raw contact force is not tire load

Rapier and MuJoCo solve rigid contact differently. A coplanar rigid four-point support is
statically indeterminate: one backend can retain four positive point loads every step,
while another can alternate a valid supporting subset and emit a one-step solver-force
spike. Feeding that raw impulse-equivalent value directly into rolling resistance and tire
friction makes a numerical contact detail act like a physical load cell.

The gate therefore preserves both signals:

- `wheel_raw_normal_load_n` is unmodified backend evidence;
- `wheel_normal_load_n` is the controller/force-element load, conditioned with a declared
  20 ms first-order bandwidth and bounded by the tire profile's declared three-times
  reference-load validity envelope.

The conditioner's bound is not a hidden cross-backend tolerance. It is the force-law
validity boundary already declared by `CombinedSlipTireSpec::maximum_load_ratio`. The raw
signal remains in the digest-protected trace for diagnostics and future suspension/contact
identification.

Rolling resistance is also step-bounded. Its Coulomb magnitude may stop the predicted wheel
coordinate within one explicit step, but cannot reverse it. This prevents low-speed sign
chatter from injecting energy when contact load changes rapidly.

## Evidence and acceptance

The pivot task settles for 0.5 s, then commands `+12 V` on the left and `-12 V` on the
right for 2 s at a 1 ms fixed step. Actor-visible input is only `command_phase`; pose,
velocity, yaw, contact load, and tire state are privileged scoring evidence.

The verified comparison artifact was produced at:

```text
E:\RNE-build\m3c-sensor\per-wheel-skid-comparison-v1.json
```

| metric | Rapier | MuJoCo | gate |
| --- | ---: | ---: | ---: |
| final absolute integrated yaw | 0.733 rad | 0.681 rad | 0.2 to 6.0 rad |
| final absolute yaw rate | 0.362 rad/s | 0.171 rad/s | 0.05 to 5.0 rad/s |
| horizontal pivot displacement | 0.0001 m | 0.040 m | 0 to 0.5 m |
| maximum absolute yaw rate | 0.547 rad/s | 0.660 rad/s | 0.1 to 5.0 rad/s |
| maximum total lateral scrub force | 654.6 N | 710.0 N | 1 to 2000 N |
| maximum conditioned wheel-load spread | 0.82 N | 563.9 N | 0 to 980.665 N |
| minimum per-wheel contact fraction | 1.00 | 0.67 | 0.5 to 1.0 |
| maximum tire utilization | 1.00 | 1.00 | 0.01 to 1.0 |

Cross-backend absolute gaps pass 0.5 rad integrated yaw, 0.5 rad/s final yaw rate,
0.1 m pivot displacement, and 100 N maximum scrub-force tolerances. Rapier replay is
byte-exact deterministic, and MuJoCo shares the exact TaskSpec, station order, fixed step,
seed, metrics, and content-integrity validation.

## Research alignment

The design follows the separation already established in the mobility plan:

- [Project Chrono tire models](https://api.chrono.projectchrono.org/wheeled_tire.html)
  separate tire force elements from generic rigid contact and provide distinct rigid,
  handling, and finite-element fidelity tiers;
- [MuJoCo computation](https://mujoco.readthedocs.io/en/3.8.0/computation/)
  exposes contact constraints and forces, not an identified vehicle tire model;
- [MVSim physics](https://mvsimulator.readthedocs.io/en/stable/physics.html) treats
  differential/Ackermann wheel-ground forces separately from chassis collision dynamics.

## Remaining M3-C boundary

This gate does not close M3-C. Remaining work is:

- connect four independent encoder/electrical channels and the IMU estimator to this plant;
- add differential two-wheel-plus-caster and anisotropic skid profiles;
- replace the explicit load conditioner with identified suspension travel, spring, damper,
  bump/rebound stop, lift, and recontact evidence;
- add Ackermann steering geometry, steering encoder, front/rear load transfer, lateral
  acceleration, constant-radius, steering-step, and split-friction cases;
- add grade, roughness, curb, wheel lift/recontact, and sensor-only closed-loop metrics.

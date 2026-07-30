# Vehicle dynamics

`rne_robot` provides two single-track ("bicycle") vehicle models that consume the same
[`AckermannDrive`] commands and differ only in how the chassis answers them.

## Kinematic model

`ackermann_kinematics` is the no-slip bicycle: yaw rate is `v / L * tan(delta)` and the
vehicle always travels exactly where the front wheels point. It is cheap, singularity
free, and correct at parking and urban speeds — and it makes every controller look
perfect, because the plant has no way to disobey.

## Dynamic model

Attaching a [`VehicleDynamics`] component opts a vehicle into the planar dynamic
bicycle model instead, integrated by `vehicle_dynamics`. Lateral tire forces are
finite, so understeer, oversteer, and the widening of a line with speed emerge from
the force balance rather than being scripted.

Per step, for forward speed `vx`, lateral speed `vy`, yaw rate `r`, steering `delta`,
distances `a`/`b` from the center of mass to the axles, and per-axle cornering
stiffness `C`:

```text
alpha_f = atan((vy + a r) / vx) - delta      front slip angle
alpha_r = atan((vy - b r) / vx)              rear slip angle
Fy      = clamp(-C alpha, +/- mu Fz)         linear tire, friction saturated
m (vy' + vx r) = Fyf cos(delta) + Fyr        lateral balance
Iz r'          = a Fyf cos(delta) - b Fyr    yaw balance
```

The axle loads `Fz` include longitudinal weight transfer `m ax h / L`, so braking
loads the front tires and throttle loads the rear — the same corner behaves
differently on and off the power. Saturation state and the slip angles of the last
step are exposed on the component for telemetry and evaluation.

### Low-speed blend

Slip angles divide by `vx` and become singular near standstill; this is the standard
failure mode of dynamic bicycle models in stop-and-go traffic. Below
[`VehicleDynamics::blend_low_speed_m_s`] the lateral states take the no-slip solution
directly, so the model parks and creeps exactly like the kinematic one and hands over
smoothly as speed rises.

### Exclusivity

`ackermann_kinematics` automatically skips any vehicle that carries
[`VehicleDynamics`]; running both integrators over one chassis would double-integrate
it. The two systems can therefore coexist in one schedule, and a mixed fleet picks
the model per vehicle by adding or omitting one component.

## Why this matters for control evaluation

A controller tuned against the kinematic plant sees no difference between a feasible
line and an impossible one. The dynamic plant enforces `v^2 / R <= mu g`: ask for more
and the front axle saturates, the yaw rate falls short of the no-slip value, and the
vehicle runs wide. That failure is precisely what a lateral controller must be
evaluated against.

## Comparison scenario

Example 49 drives two identical vehicles through the same course — a straight, an
18 m constant-radius sweeper, and an exit straight — with the same pure-pursuit
controller, the same 14 m/s cruise command, and the same physically derived braking
point. At that speed the friction-limited turn radius (`v^2 / (mu g)` ≈ 22 m) exceeds
the course radius, so the corner is beyond the dynamic car's grip but recoverably so.

```bash
cargo run --release -p vehicle_dynamics_compare --example 49_vehicle_dynamics
RNE_SKIP_GPU=1 cargo run -p vehicle_dynamics_compare --example 49_vehicle_dynamics
```

Committed results:

| vehicle | worst course error | behaviour |
| --- | --- | --- |
| kinematic | 0.78 m | tracks the sweeper as commanded |
| dynamic | 17.13 m | front axle saturates for 92 steps, runs wide, rejoins on the exit |

The maximum gap between the two vehicles reaches 28.1 m. In the rendered GIF the
dynamic trail turns red wherever the front axle is beyond its friction limit; the
two trails are pixel-identical on the entry straight, which is the regression check
that the models share their command shaping.

The acceptance tests require the low-speed paths of the two models to agree within
the center-of-mass offset bound, the line to widen with speed through real slip
angles without saturation, a hard corner to saturate the front axle and undershoot
the no-slip yaw rate, load transfer to preserve total weight, the world-frame
velocity to carry the lateral component a mounted sensor would observe, and two runs
to be bit-identical.

[`AckermannDrive`]: ../crates/rne_robot/src/components.rs
[`VehicleDynamics`]: ../crates/rne_robot/src/components.rs

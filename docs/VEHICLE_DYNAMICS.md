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

### Load-dependent cornering stiffness (opt-in)

`C` above is constant by default: load transfer moves the friction limit
`mu Fz`, but not the slope of the linear tire. Setting
[`VehicleDynamics::cornering_stiffness_load_sensitivity`] to a
`CorneringStiffnessLoadSensitivity` makes `C` itself track each axle's
instantaneous load, reusing [`CombinedSlipTireSpec`]'s load-ratio clamp and
`load_sensitivity_per_load_ratio` functional form rather than a second,
independently invented law — the slope sign is flipped, since cornering
stiffness rises with load where tire friction falls with it. The reference
load is each axle's own static (zero-acceleration) load, so the declared
`front_cornering_stiffness_n_rad` / `rear_cornering_stiffness_n_rad` values
keep their exact meaning there; a hard brake still raises front stiffness and
lowers rear (and throttle does the reverse), sub-linearly in the load ratio.

This is a **model refinement of a deliberately simple linear tire, not a
measurement**: no real vehicle or tire data was used to choose the affine
shape, only consistency with the codebase's existing tire law. The field is
absent (`None`) by default, which keeps every existing constant-stiffness
trajectory bit-for-bit identical, and it is skipped entirely when serializing
a `VehicleDynamics` that does not set it, so existing serialized artifacts,
goldens, and retained evidence digests are unaffected.

### Lateral load transfer (opt-in)

The single-track model has one tire per axle, so it cannot represent the left/right
load shift a corner produces. Setting [`VehicleDynamics::lateral_load_transfer`] to a
`LateralLoadTransferSpec` splits each axle into two equal-slip tires: the total lateral
transfer `m |vx r| h / track` is split front/rear by
`front_roll_stiffness_fraction`, each side carries `Fz / 2 +/- dFz`, and the axle force
is the sum of the two sides at their own loads. Because the friction limit `mu Fz` is
linear in load but the linear tire saturates the loaded side before the unloaded one,
the split reduces the axle's usable lateral force and shifts the balance between the
axles — the classical load-transfer understeer/oversteer effect. A front-biased
roll-stiffness fraction tends toward understeer.

The transfer uses the steady centripetal acceleration `vx r` from the current state, not
this step's forces, so the load/force relation stays explicit and loop-free, mirroring
the longitudinal path that uses the previous chassis acceleration. This is a
**deliberately lower-order model, not a measurement**: a mean track width, a single
roll-stiffness fraction, and no roll degree of freedom. The field is absent (`None`) by
default, which keeps the single-tire-per-axle model bit-for-bit identical, and it is
skipped when serializing a `VehicleDynamics` that does not set it, so existing
serialized artifacts, goldens, and retained evidence digests are unaffected. With zero
lateral transfer the two half-load tires reproduce the single tire exactly, because
halving and doubling are exact in binary floating point.

### Four-wheel model (opt-in)

The single-track model has one wheel per axle, so its left and right wheels share a slip
angle and a steer angle. Setting [`VehicleDynamics::four_wheel`] to a
`FourWheelVehicleSpec` replaces that axle abstraction with four explicit wheels. Each
front wheel gets an Ackermann steer angle blended by `ackermann_fraction` (`0.0` is
parallel steering, `1.0` is the exact geometry for the rear-axle path radius
`R = L / tan(delta)`). Each wheel carries its own normal load, its own slip angle from
`vy + r x` and `vx + r z` (so the outer wheel of a turn runs faster), and its own
friction-saturated lateral force. The axle force and yaw moment are the explicit
per-wheel sums `sum Fy_i cos(delta_i)` and `sum x_i Fy_i cos(delta_i)`. Lateral load
transfer is directional: the outer side is loaded according to the sign of `vx r`, using
the same per-axle value `m |vx r| h / track` as the single-track split. The per-wheel
slip angles and saturation flags are exposed on `wheel_slip_rad` / `wheel_saturated`, and
the axle slip fields become the per-wheel mean.

Like the other extensions this is a **deliberately lower-order model, not a
measurement**: no roll degree of freedom, a linear friction-saturated tire, and the
steered front tires' longitudinal force component and any aligning moment are omitted.
The field is absent (`None`) by default, which keeps the single-track model bit-for-bit
identical, and it is skipped when serializing a `VehicleDynamics` that does not set it.
When it is present, [`VehicleDynamics::lateral_load_transfer`] is ignored, because the
four-wheel spec carries its own track width and roll split.

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

<p align="center">
  <picture>
    <source media="(prefers-reduced-motion: reduce)" srcset="media/vehicle-dynamics.png">
    <img src="media/vehicle-dynamics.gif" alt="Kinematic and dynamic RNE vehicle models diverging through a tire-saturated corner" width="800">
  </picture>
</p>

```bash
cargo run --release -p vehicle_dynamics_compare --example 49_vehicle_dynamics
RNE_SKIP_GPU=1 cargo run -p vehicle_dynamics_compare --example 49_vehicle_dynamics
```

The first command reruns the headless comparison, renders procedural cars with their
recorded body headings and front-wheel steering, overlays live speed, slip-angle,
yaw-rate, and grip-state telemetry, and replaces both committed media files. The
second command exercises the exact simulation and assertions without requiring a
renderer. Temporary full-resolution frames live under `target/vehicle-dynamics` and
are removed after a successful encode; the committed GIF has a 4 MiB size budget.

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
to be bit-identical. The lateral-transfer tests additionally require the split axle to
reproduce the single tire bit-for-bit at zero transfer, to cost usable axle force when
one side saturates first, to measurably change the steady-turn yaw response, and to be
deterministic. The four-wheel tests require exact Ackermann steering to spread the front
slip angles more than parallel steering, the axle slip telemetry to be the per-wheel
mean, the model to measurably change the steady-turn yaw response versus the
single-track model, and the run to be deterministic.

[`AckermannDrive`]: ../crates/rne_robot/src/components.rs
[`VehicleDynamics`]: ../crates/rne_robot/src/components.rs

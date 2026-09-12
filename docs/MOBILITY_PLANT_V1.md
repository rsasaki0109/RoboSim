# Mobility plant v1 contracts

Status: M1-A plant contracts and the M1-E combined-slip force element are implemented;
cross-backend wheel/contact scenarios remain pending.

These contracts are backend-neutral inputs to the Mobility Physical AI foundation. They
do not expose Rapier, MuJoCo, or another solver type, and they do not claim vehicle fidelity
from a kinematic speed command.

## DC motor

`DcMotorSpec` is an identifiable equivalent circuit. Its quasi-static tier computes

```text
back_emf_v = back_emf_constant_v_s_rad * rotor_velocity_rad_s
current_a  = clamp((terminal_voltage_v - back_emf_v) / resistance_ohm)
torque_nm  = torque_constant_nm_a * current_a - viscous_loss - coulomb_loss
```

Terminal voltage and current are independently limited. At exact standstill, Coulomb
friction cancels electromagnetic torque only up to the declared friction magnitude. An
optional inductance advances current with the explicit equation `L dI/dt = V - k_e w - R I`;
the caller must choose a timestep appropriate for that electrical time constant.

`OpenCircuit` forces armature current and electromagnetic torque to zero. `ShortCircuit`
forces terminal voltage to zero, so back-EMF produces bounded braking current. Mechanical
shaft loss remains present in both failures.

Identify the v1 fields from terminal resistance or a locked-rotor test, torque/current and
speed/voltage datasheet points, current-limit evidence, free-spin current, and coast-down.
Thermal drift, magnetic saturation, inverter switching, commutation ripple, and cogging are
outside this tier and must be declared unavailable rather than fitted into unrelated fields.

## Transmission

`TransmissionSpec::ratio_motor_rad_per_wheel_rad` is signed and unit explicit. The static
map selects drive or backdrive efficiency from mechanical power direction and reports rotor
inertia reflected to the wheel as `J_motor * ratio^2`.

Backlash, torsional stiffness, and damping are declared for the next stateful driveline tier.
M1-A intentionally does not fake their transient response inside a rigid torque multiplier.
Ratio and efficiency can be taken from the gearbox datasheet; backlash and compliance need
direction-reversal and torque/deflection measurements.

## Steering actuator

`SteeringActuatorSpec` shapes a requested steering angle before it reaches a
backend joint-position constraint. It uses an exact first-order zero-order-hold
response, followed by explicit angular-rate and travel limits. A declared
deadband holds position for small command errors, while `Stuck` holds the last
completed position regardless of the next command.

This is a command-to-angle actuator model, not steering ground truth and not a
torque/current servo model. Identify its time constant, rate limit, deadband,
travel, and failure response from synchronized command and direct steering-angle
measurements. Backlash, compliance, load-dependent servo response, and linkage
forces remain unavailable until represented by a separately validated tier.

`identify_steering_actuator_first_order` provides the first evidence gate for the
unsaturated lag tier. It requires a uniform monotonic capture clock and direct
angle samples, fits only a declared leading training split, and evaluates frozen
one-step residuals on a later holdout split. The discrete fit is
`delta_angle = b * (command - angle)` with `tau = -dt / ln(1 - b)`.
Command echoes, insufficient command-error excitation, unstable response ratios,
out-of-travel samples, clock drift, nonphysical time constants, and excessive
training or holdout residuals fail explicitly. Callers must prequalify and retain
separate rate-saturated, deadband, backlash, and fault segments.

## Wheel assembly

`WheelAssemblySpec` declares unloaded radius, width, axle inertia, rolling resistance, and
an orthonormal local contact frame. Its standalone rolling-resistance law is
`Crr * normal_load_n * radius_m`, opposes completed motion, and returns zero at exact
standstill rather than inventing a direction.

The wheel spec is not a tire model. The point-contact contract supplies backend-neutral
contact kinematics and normal load; `CombinedSlipTireSpec` supplies transient combined-slip
forces and low-speed regularization. Generic collider friction must not be presented as
identified tire behavior or silently combined with the tire force element.

## Tire identification

`identify_combined_slip_tire_steady` implements a bounded first identification
gate for the low-order tire force element. It requires complete acquisitions with
monotonic capture clocks, relaxed longitudinal/lateral slip coordinates, measured
normal load, and independently measured longitudinal/lateral tire force. Each run
also carries a stable acquisition ID, condition ID, and an independently declared
road-friction scale.

The gate follows the staged measurement practice used for combined-slip models:
pure-longitudinal and pure-lateral training samples identify the two small-slip
stiffnesses and peak-friction coefficients, while combined-slip acquisitions are
held out and scored without refitting. See the on-vehicle force/slip measurement
and validation procedure in [Van Gennip and McPhee, 2018](https://doi.org/10.4271/2018-01-1339)
and the pure-before-combined split described for MF-Swift parameterization in
[Besselink et al., 2009](https://doi.org/10.2346/1.3133110). These references
support the evidence order; RNE does not claim to implement their Magic Formula.

The deterministic coarse-to-fine search is bounded by declared stiffness/friction
ranges and a fixed evaluation budget. Training must contain both small-slip and
peak-region excitation for each axis. Holdout must contain combined slip in at
least two condition IDs, with a declared minimum sample count met independently
by every condition. Acceptance requires pure-slip training RMS, pooled
combined-slip vector-force RMS, and every condition RMS to pass independently.
Duplicate acquisition IDs, invalid clocks, out-of-envelope load/slip, insufficient
excitation, non-finite fits, and residual failures are rejected.

This steady gate deliberately does not identify relaxation length, load sensitivity,
road-friction scale, low-speed regularization, camber, aligning moment, temperature,
wear, or pressure. Those template values must retain separate provenance. Trajectory,
IMU, odometry, or command logs without an independently justified tire-force
reconstruction cannot enter this gate.

## Evidence and validity

Pure deterministic tests cover locked rotor, voltage/current saturation, back-EMF,
inductive current state, open/short failures, directional transmission efficiency,
reflected inertia, steering response/rate/travel/failure behavior, invalid inputs, and
rolling-resistance sign. Synthetic identification tests additionally recover the
four frozen steady tire parameters from pure-slip training and reject a degraded
friction-condition holdout. Future benchmark profiles
must preserve the raw parameter source and run locked-rotor, free-spin, coast-down,
acceleration/braking, and direction-reversal fixtures before claiming a calibrated plant.

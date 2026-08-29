# Mobility plant v1 contracts

Status: M1-A model contracts implemented; wheel/contact coupling is pending M1-B.

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

## Wheel assembly

`WheelAssemblySpec` declares unloaded radius, width, axle inertia, rolling resistance, and
an orthonormal local contact frame. Its standalone rolling-resistance law is
`Crr * normal_load_n * radius_m`, opposes completed motion, and returns zero at exact
standstill rather than inventing a direction.

The wheel spec is not a tire model. M1-B supplies backend-neutral contact kinematics and
normal load; M1-D supplies transient combined-slip forces and low-speed regularization.
Until those stages land, generic collider friction must not be presented as identified
tire behavior.

## Evidence and validity

Pure deterministic tests cover locked rotor, voltage/current saturation, back-EMF,
inductive current state, open/short failures, directional transmission efficiency,
reflected inertia, invalid inputs, and rolling-resistance sign. Future benchmark profiles
must preserve the raw parameter source and run locked-rotor, free-spin, coast-down,
acceleration/braking, and direction-reversal fixtures before claiming a calibrated plant.

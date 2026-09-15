# Mobility longitudinal benchmark v1

This benchmark closes RNE's first deterministic control-oriented mobility loop:

```text
terminal voltage -> DC motor -> transmission -> wheel inertia
  -> transient tire slip -> road force -> chassis motion
```

It is a backend-neutral analytic plant for identification, controller development,
and regression testing. It is not a rigid-body vehicle, a Rapier/MuJoCo parity claim,
or a validated model of a particular robot or car.

## State and force balance

`LongitudinalMobilityPlantState` retains chassis position and velocity, representative
driven-wheel position and velocity, motor current, and transient tire slip. One driven
wheel is evaluated and its road force is multiplied by the declared identical driven
wheel count.

The motor sees `omega_motor = ratio * omega_wheel`. The existing motor and transmission
evaluators supply realized current, shaft torque, efficiency loss, and reflected rotor
inertia. The representative wheel and chassis then obey:

```text
(J_wheel + J_reflected) * domega_wheel/dt
  = T_drive + T_rolling - radius * F_tire

mass * dv/dt
  = driven_wheel_count * F_tire + F_aero - F_grade

F_aero  = -drag_coefficient * v * abs(v)
F_grade = mass * gravity * sin(grade)
```

Wheel-surface velocity relative to the road is computed from both dynamic states, so
traction and braking slip emerge from the closed loop. The combined-slip element applies
load-sensitive peak force, low-speed regularization, relaxation length, and a smooth
friction ellipse. Semi-implicit Euler advances velocities before positions at a fixed
simulation step.

## Optional longitudinal load transfer

By default the driven wheel's normal load is the constant
`normal_load_per_driven_wheel_n`, so `CombinedSlipTireSpec::load_sensitivity_per_load_ratio`
has nothing to react to in this plant: the load ratio never moves. `LongitudinalMobilityPlantSpec::longitudinal_load_transfer`
is an opt-in `Option<LongitudinalLoadTransferSpec>` that instead derives the driven
wheel's normal load from the chassis's own longitudinal acceleration each step:

```text
delta_F_z = m * a_x * h_cg / L
```

`h_cg` (`cg_height_m`) and `L` (`wheelbase_m`) are declared geometry; `driven_axle`
(`Front` or `Rear`) fixes the sign, since forward acceleration always loads the rear
axle and unloads the front regardless of which axle the plant's single driven-wheel
path represents. The transfer is applied to the driven axle's static total load
(`normal_load_per_driven_wheel_n * driven_wheel_count`) and split evenly across
`driven_wheel_count` wheels; the result is clamped to non-negative so an analytically
unloaded axle reads zero rather than a negative load.

`a_x` is the chassis acceleration `evaluate_longitudinal_mobility_plant` already
computed on the *previous* step (`state.previous_chassis_acceleration_m_s2`), not a
value solved jointly with this step's tire force. This keeps the evaluator a single
explicit step, matches its existing semi-implicit integration (a step's contact
evidence is always derived from already-completed motion), and already reflects
grade and aerodynamic drag, since those terms are baked into the acceleration the
loop computes. At rest, or with the field absent, this is exactly today's constant
load.

This is an analytic, rigid-body model: no suspension spring/damper dynamics, no pitch
state, and longitudinal transfer only -- no lateral or cornering transfer. It has not
been fitted or validated against measured vehicle data; `wheelbase_m` and
`cg_height_m` are declared geometry, not identified parameters. When
`longitudinal_load_transfer` is absent (the default), the plant is bit-for-bit
identical to its behavior before this field existed, which existing serialized specs
and evidence depend on: it decodes as `None` under `#[serde(default)]` and is never
emitted when absent.

## Research and OSS correspondence

- Li et al., *Estimation of Vehicle Dynamic Parameters Based on the Two-Stage
  Estimation Method*, gives the same longitudinal chassis and wheel balances
  `m * dv/dt = sum(F_x)` and `J * domega/dt = T - r * F_x`, and identifies load transfer,
  tire stiffness, mass, inertia, and center-of-gravity geometry as coupled parameters:
  https://doi.org/10.3390/s21113711
- Project Chrono separates rigid, semi-empirical handling, and finite-element tire
  fidelity. Its Pac02 and TMeasy descriptions retain contact-patch slip state equations
  for transient maneuvers, and its Fiala example exposes rolling resistance, stiffness,
  friction, and relaxation lengths. RNE follows this decomposition without claiming
  implementation parity:
  https://api.projectchrono.org/wheeled_tire.html
- Dominguez et al., *Longitudinal Dynamics Model Identification of an Electric Car
  Based on Real Response Approximation*, identifies propulsion, braking, and friction
  forces from full-speed-range vehicle tests and validates against an independent trip.
  This motivates RNE's requirement that a named hardware profile be fitted and validated
  from real logs rather than inferred from plausible defaults:
  https://arxiv.org/abs/2003.07738
- The RNE transient tire structure and its low-speed/combined-slip sources are documented
  separately in [`MOBILITY_COMBINED_SLIP_V1.md`](MOBILITY_COMBINED_SLIP_V1.md).

These references justify model structure and identification requirements. They do not
validate RNE's default parameters.

## Deterministic evidence

`rne-mobility-benchmark` emits a versioned JSON report with fixed-step metadata, SI-unit
metrics, per-case verdicts, and a stable FNV-1a content digest. The report validates its
schema, ordering, finite values, metric verdicts, aggregate verdict, and digest. The CLI
returns failure when any physical envelope fails.

The v1 matrix covers:

1. locked-rotor supply and current limiting;
2. coupled high-friction acceleration;
3. ice-like traction limiting versus the nominal road;
4. regenerative braking and speed reduction;
5. explicit motor open circuit;
6. trajectory convergence when the fixed step is halved.

The low-friction case deliberately uses a road scale of `0.05`. A scale of `0.2` remains
motor-limited for the default plant, so it cannot prove that the tire-force ceiling affects
the trajectory. The benchmark requires the low-friction tire to reach its force limit as
well as producing lower chassis speed and higher wheel-slip speed.

Run the producer headlessly with an isolated target directory:

```text
cargo run -p rne_mobility_benchmark -- --output mobility-report.json
```

## Validity envelope and omissions

The model is suitable for deterministic straight-line controller tests at the declared
fixed step, parameter sensitivity checks, and a first stage of real-log identification.
It currently assumes identical driven wheels, a rigid driveline ratio, flat longitudinal
contact, and no native backend tangential friction. Per-wheel normal load is prescribed
by default and can optionally follow an analytic longitudinal weight-transfer model (see
above); either way it is not solved from a suspended rigid body.

It does not yet model suspension, pitch, steering, yaw, lateral scrub, cornering/lateral
load transfer, backlash state, tire temperature or pressure, road roughness, ABS,
traction control, inverter switching, bus-voltage sag, battery state, or thermal limits.
The analytic plant itself does not inject tire forces into Rapier or MuJoCo. The separate
[`MOBILITY_BACKEND_CLOSED_LOOP_V1.md`](MOBILITY_BACKEND_CLOSED_LOOP_V1.md) fixture now
runs the shared drive-path evaluator through real Rapier and MuJoCo contact/wrench paths,
retains manifests and time series, applies unit-bearing tolerances, and proves the Failure
Capsule path with an injected diagnostic bound. It remains an equivalent single support
path, not yet a per-wheel suspension or steering model.

Later identification must separate fitting logs from validation logs and preserve raw-data
hashes, fitted parameters, residuals, and failure cases.

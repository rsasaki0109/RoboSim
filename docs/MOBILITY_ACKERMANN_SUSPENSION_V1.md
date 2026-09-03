# Ackermann suspension benchmark v1

Status: implemented additive M3-C dynamics subgate

This fixture replaces a planar bicycle or equivalent support point with an explicit
four-station multibody vehicle. The same
`mobility_ackermann_suspension_split_mu_v1` TaskSpec, seed, 1 ms clock, force laws,
sampling, and verdict code run through Rapier and MuJoCo.

## Physical contract

The model contains a rigid chassis plus four named FL/RL/FR/RR stations. Each station
has a vertical prismatic unsprung link, a steering-axis revolute link, a frictionless
normal-contact sphere, and independent motor, transmission, wheel-inertia, combined-slip
tire, and relaxation state. The front targets use inner/outer Ackermann geometry; the
rear steering axes are physically present but constrained to zero. Tire forces are
computed from solved load and contact-point velocity, then applied through the
backend-neutral one-step wrench contract. Generic collider friction is not relabelled as
a vehicle tire.

The suspension force uses one unit-explicit law for every backend:

```text
F = clamp(k (q_free - q) - c q_dot, -F_max, F_max)
```

The v1 fixture uses 200 kN/m stiffness, 15 kN s/m damping, 24 kg unsprung mass per
station, a 120 mm mechanical stroke, an explicit -52 mm initial ride coordinate, and a
preloaded -61 mm free-length coordinate. The force
is sent as prismatic effort, not as a backend position servo. Mechanical travel remains
separate from a declared 1 mm numerical joint-limit tolerance for iterative constraint
solvers. Equal and opposite prismatic forces act at the common joint anchors. Applying
them independently at body centers of mass creates a non-physical force couple for an
off-center suspension; a Rapier regression requires zero net world moment from the
internal force pair.

This decomposition follows the modular chassis/steering/suspension/wheel structure in
the [Project Chrono Vehicle whitepaper](https://projectchrono.org/assets/white_papers/chronoVehicle_IJVP.pdf)
and its [official suspension demo](https://github.com/projectchrono/chrono/blob/main/src/demos/mbs/demo_MBS_suspension.cpp),
which constructs explicit spindles, steering constraints, and spring-damper elements.
Chrono's official vehicle documentation also separates tire-force models from multibody
suspension and provides low-speed extensions rather than assuming a high-speed slip law
is valid at standstill; see the [official Chrono repository](https://github.com/projectchrono/chrono)
and the maintainers' [low/zero-speed tire-model note](https://github.com/projectchrono/chrono/discussions/575).
These are architecture and model references, not a claim that RNE reproduces Chrono.

## Maneuver and honest evidence

The vehicle first settles under gravity, then performs ramped all-wheel acceleration,
an Ackermann turn, and split-friction braking with the left road scale set to 0.45.
Scoring after the settle boundary requires:

- continuous four-wheel solved contact;
- non-trivial forward/lateral motion, steering, and yaw rate;
- suspension motion during the driven phases, excluding initial drop;
- front/rear and left/right load change relative to the settled baseline;
- a split-friction utilization difference;
- bounded SI-unit cross-backend gaps and mutation-detecting digests.

The trace intentionally labels chassis pose and velocity as privileged truth. This is an
open-loop plant/dynamics subgate, and its TaskSpec state tensors are explicitly named
`diagnostic_*`; it does not yet claim sensor-only Ackermann control.

Generate the external-SSD evidence with:

```powershell
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-suspension-rapier --output ackermann-suspension-rapier-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-suspension-mujoco --output ackermann-suspension-mujoco-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend ackermann-suspension-compare --output ackermann-suspension-comparison-v1.json
```

The repository tests require exact same-runtime Rapier repeatability, passing individual
Rapier/MuJoCo traces, passing unit-bearing comparison bounds, Ackermann inner/outer
geometry, valid suspension-force evaluation, and trace-tamper rejection.

The verified comparison artifact is stored outside the repository at
`E:\RNE-build\m3c-sensor\ackermann-suspension-comparison-v1.json` with digest
`fnv1a64:681b40b5910206a4`:

| metric | Rapier | MuJoCo | absolute gap |
| --- | ---: | ---: | ---: |
| forward displacement | 0.881665 m | 0.884145 m | 0.002480 m |
| lateral displacement | 0.142206 m | 0.142809 m | 0.000603 m |
| maximum yaw rate | 0.064793 rad/s | 0.064903 rad/s | 0.000110 rad/s |
| front/rear load shift from settled baseline | 126.197 N | 141.615 N | 15.418 N |
| left/right load change from settled baseline | 62.824 N | 71.237 N | 8.413 N |
| split-friction utilization gap | 0.118561 | 0.119906 | — |
| driven-phase suspension range | 0.240922 mm | 0.276690 mm | 0.035768 mm |
| minimum wheel contact fraction | 1.000 | 1.000 | — |

## Explicit limits

This is not real-vehicle validation. The wheel contact shapes are sphere proxies on a
flat rigid plane, tire rotation is an explicit force-element state rather than mesh roll,
and the parameters have not yet been identified from logs. Road roughness, curb impact,
wheel lift/recontact, steering backlash, actuator/sensor calibration, sensor-only
estimation, deterministic domain randomization, and real-log residuals remain subsequent
M3-C through M5 gates.

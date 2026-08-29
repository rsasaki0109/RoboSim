# Mobility combined-slip tire v1

This is RNE's first backend-neutral tire force element. It turns completed point-contact
evidence into a deterministic one-step `ExternalBodyWrench`; Rapier and MuJoCo types remain
inside their adapters. The model is deliberately low-order and identifiable. It is not
presented as Pacejka, full TMeasy, or a validated road-vehicle tire.

## Coordinates and update

`aggregate_wheel_contact_patch` sums contact normal load and computes load-weighted world
point, road-to-wheel normal, and wheel-surface velocity relative to road. It normalizes the
result for either side of the canonically ordered contact pair. Contact samples are evidence
from the completed physics step, so applying the returned wrench has a documented one-step
delay.

For longitudinal surface speed `v_surface_x`, wheel circumferential speed `v_circ`, and
positive regularization speed `v_num`, the target slip coordinate is:

```text
kappa_target = -v_surface_x / (abs(v_circ) + v_num)
tan(alpha)_target = -v_surface_y / (abs(v_circ) + v_num)
```

This sign makes a driven wheel whose bottom surface moves rearward generate forward force.
Each coordinate follows an exact first-order relaxation update with time constant
`relaxation_length / transport_speed`; a zero relaxation length is explicitly instantaneous.
Wheel lift resets both retained coordinates and all forces.

Small-slip stiffness scales with normal load inside the declared load envelope. Peak
longitudinal and lateral friction decrease with normalized load and are multiplied by an
explicit road-friction scale. The two linear force demands share a smooth ellipse through
`scale = tanh(demand) / demand`, keeping combined utilization at or below one without a hard
force discontinuity. A zero road scale produces zero force.

## Research basis

- [Project Chrono TMeasy implementation](https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.cpp)
  computes longitudinal and lateral slips from contact-patch velocity with a positive
  low-speed denominator, combines load-dependent stiffness and peak forces, and carries
  explicit transient/standstill handling. RNE adopts the observable structure, not its
  parameterization or name.
- [Project Chrono tire-model documentation](https://api.projectchrono.org/development/wheeled_tire.html)
  separates terrain contact from force-producing tire models and distinguishes rigid,
  handling, and finite-element fidelity tiers. RNE follows that separation at the physics
  boundary.
- [Bernard and Clover, *Tire Modeling for Low-Speed and High-Speed Calculations*, SAE
  950311](https://saemobilus.sae.org/papers/tire-modeling-low-speed-high-speed-calculations-950311)
  identifies the singular low-speed slip-coordinate problem and motivates stateful tire
  treatment rather than an unguarded division by vehicle speed.
- [Zhang et al., *A New Tire Model with an Application in Vehicle Dynamics Studies*,
  AVEC 2018](https://ddl.stanford.edu/sites/g/files/sbiybj25996/files/media/file/zhang_2018_avec_0.pdf)
  discusses combined-slip friction-circle constraints and low-speed continuity in a Fiala
  family model.

## Evidence and validity envelope

Unit tests cover canonical contact orientation, load-weighted aggregation, traction and
lateral-force signs, combined-force bounds, split-friction scaling, load sensitivity,
finite standstill behavior, relaxation transients, lift reset, deterministic repetition,
and wrench coordinates. Parameter profiles must preserve their source and be identified
from real force/slip or vehicle-test logs before RNE makes a fidelity claim.

The v1 validity envelope excludes camber thrust, aligning moment, pressure and temperature,
transient footprint geometry, turn slip, enveloping terrain contact, deformable soil, and
parking-static bristle force. Native collider tangential friction must be disabled or
declared as a separate contribution when this force element is active; otherwise friction
is counted twice. Backend conformance of split friction, wheel lift, and force application
is the next integration gate.

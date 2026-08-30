# Mobility contact and wrench boundary v1

This document freezes the first backend-neutral output boundary between a mobility
force-element model and a rigid-body solver. It does not claim a tire model yet.

## Contract

`ExternalBodyWrench` carries an entity, a world-frame application point in meters,
a world-frame force in newtons, and a world-frame free moment in newton-meters.
`PhysicsBackend::apply_external_body_wrench` accepts the command after ECS-to-backend
synchronization and before `step`. An accepted command affects exactly the next completed
step and is then cleared. Non-finite input, missing bodies, and non-dynamic targets fail
without partially applying a load.

`PhysicsCapability::ExternalBodyWrench` makes support explicit. It refines
`RigidBody`; a backend that does not advertise it uses the trait's fail-closed default.
The conformance vector applies 120 N at a 1 m lever arm plus a 60 N*m free moment to a
2 kg body with a declared 1 kg*m^2 inertia. It checks linear and angular response and then
checks that neither load persists into the following step.

Rapier implements the boundary with its native force-at-point and torque operations.
MuJoCo maps the application-point force to a center-of-mass torque with
`(point - COM) x force`, adds the declared free moment, writes the resulting world-frame
Cartesian load, and clears it immediately after the completed step. No backend handle or
numeric type crosses the `rne_physics` API.

## Research basis

- [Rapier rigid-body forces and impulses](https://rapier.rs/docs/user_guides/rust/rigid_body_forces_and_impulses/)
  distinguishes persistent force from impulse and provides world-point force application.
  RNE clears accepted loads after one step so caller timing is portable rather than inheriting
  Rapier's persistent-user-force behavior.
- [MuJoCo computation: contacts](https://mujoco.readthedocs.io/en/3.9.0/computation/index.html)
  defines point contacts with a global contact point and frame, and separates constraint
  force/torque from applied forces. M1-C will map this RNE wrench to MuJoCo's Cartesian
  external-load path without changing the contract.
- [Project Chrono tire models](https://api.projectchrono.org/development/wheeled_tire.html)
  separates tire-terrain contact data from the tire force/moment reported back to the vehicle.
  It also warns that handling models use specific contact approximations and slip definitions.
- [Project Chrono `ChTMeasyTire`](https://api.projectchrono.org/classchrono_1_1vehicle_1_1_t_measy_tire.html)
  exposes contact data and tire-frame force/moment separately, reinforcing the same boundary.

## Deliberate limitations

- Contact point, normal load, and surface velocity are standardized by
  `ContactPointSample` in the following M1-D slice. Material state and tire-frame
  aggregation remain intentionally outside this v1 wrench contract.
- The existing pair-aggregated `ContactEvent` normal impulse is diagnostic evidence, not a
  per-wheel contact patch model.
- Generic collider Coulomb friction is not called a tire model.
- This contract contains no relaxation state, longitudinal/lateral slip, friction ellipse,
  load sensitivity, suspension, thermal state, or deformable terrain.
- Callers must submit multiple wrenches in stable entity order when floating-point accumulation
  is externally visible.

## Next proof

The next slice consumes generic `ContactPointSample` values to form a wheel contact patch,
then implements transient combined slip. Rapier and MuJoCo already execute this exact
output-wrench vector through the shared conformance catalog.

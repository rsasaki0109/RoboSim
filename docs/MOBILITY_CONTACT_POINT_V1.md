# Mobility contact-point input boundary v1

This contract supplies the input half of a backend-neutral mobility force element. It is
generic physics evidence rather than a wheel or tire type, so `rne_physics` remains usable
by other contact-driven models.

## Contract

`ContactPointSample` reports one solved, non-sensor contact point from the last completed
step. Entities are canonically ordered by stable entity index. The point and relative
surface velocity are world-frame values; the unit normal points from A to B; velocity is
explicitly `v_B(point) - v_A(point)`; and normal load is a finite, positive step-average
force in newtons. `PhysicsBackend::contact_points` returns samples in deterministic order.

`PhysicsCapability::ContactPointKinematics` refines `ContactForce`. Backends that do not
advertise it fail closed through the trait default. The shared conformance fixture settles
a 2 kg cuboid on a plane and verifies canonical orientation, finite unit normals, near-zero
relative surface speed, and the sum of point loads against supported weight using named,
unit-bearing backend tolerances.

Rapier transforms each geometric manifold point from collider-local coordinates, evaluates
both rigid-body velocities at the world point, and divides the solved normal impulse by the
completed fixed-step duration. MuJoCo reads each native contact point and normal force, then
uses its world-oriented object velocity plus `omega x (point - geom_origin)` for each
surface velocity. Neither backend-native handle nor contact structure crosses the API.

## Research basis

- [Rapier advanced collision detection](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/)
  distinguishes local geometric contacts, transient solver contacts, and solved impulses
  retained on tracked contacts. This is why RNE derives point force from solved impulse and
  fixed-step duration instead of exposing a transient native solver contact.
- [Rapier integration parameters](https://rapier.rs/docs/user_guides/rust/integration_parameters/)
  documents iterative velocity/position solves and warm starting. RNE therefore records a
  named tolerance rather than claiming exact body-weight equality.
- [MuJoCo contact types](https://mujoco.readthedocs.io/en/stable/APIreference/APItypes.html)
  defines contact position in global coordinates and the contact-frame normal direction.
- [MuJoCo `mj_contactForce`](https://mujoco.readthedocs.io/en/3.3.3/APIreference/APIfunctions.html#mj-contactforce)
  returns per-contact 6D force/torque in the contact frame; RNE takes its normal component
  and keeps all coordinate-frame conversion inside the adapter.
- [MuJoCo contact-force computation](https://mujoco.readthedocs.io/en/3.3.0/programming/simulation.html)
  explains the transposed contact frame and pyramidal/elliptic friction-cone conversion.
  Tangential solver force is deliberately not standardized in v1.
- [Project Chrono tire models](https://api.projectchrono.org/development/wheeled_tire.html)
  separates terrain-contact information from tire-force output and documents that different
  handling models use different contact approximations.

## Deliberate limitations

- Samples are completed-step evidence, so a force element using them has an explicit
  one-step coupling delay. The delay is deterministic and will be part of the tire validity
  envelope rather than hidden by backend callbacks.
- Contact-point identity is not stable across steps or backends. Wheel-patch aggregation and
  relaxation state are keyed by wheel entity, never by native manifold index.
- Tangential native solver impulse, material lookup, road texture, camber, and tire-frame
  axes are not in this contract.
- A custom tire model must use a declared native-friction policy to avoid double-counting
  tangential force. Generic collider friction is not a tire model.

## Next proof

M1-E aggregates samples per wheel, defines low-speed-safe longitudinal/lateral slip and
normal load, evaluates an identifiable combined-slip law with relaxation state, and returns
one-step `ExternalBodyWrench` commands. Split-friction and wheel-lift fixtures will exercise
both Rapier and MuJoCo under named tolerances.

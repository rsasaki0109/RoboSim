# Differential-drive trailing-caster benchmark v1

Status: implemented additive M3-C dynamics subgate

This fixture replaces an equivalent support point with an explicit three-point
multibody robot: two rigid driven-wheel stations and one passive trailing caster. The
same `mobility_diff_drive_trailing_caster_v1` TaskSpec, 1 ms clock, open-loop voltage
sequence, motor/transmission/tire laws, contact conditioning, and scoring code run through
Rapier and MuJoCo.

## Why the caster is explicit

Wu et al., [“Steering-angle computation for the multibody modelling of
differential-driving mobile robots with a caster”](https://doi.org/10.1177/1729881418820166),
show that a platform-only kinematic model can be adequate for low-level navigation, but
the caster steering angle and multibody terms matter for dynamic, high-speed, or
heavy-load operation. The v1 contract therefore records rather than hides:

- mount location and mechanical trail;
- bracket and wheel masses;
- swivel and axle inertias;
- swivel and rolling damping;
- caster rolling and combined-slip tire state.

The maneuver settles under gravity, enters a ramped differential arc, straightens, then
ramps through zero into reverse. Evidence includes all three support loads, contact
participation, chassis pose and velocity, wrapped and unwrapped caster swivel, swivel
rate, caster roll rate, lateral force, friction utilization, and backend identity.

## Contact and backend boundary

RNE consumes solved contact points and normal loads and applies the next-step tire wrench
through backend-neutral `ContactPointSample` and `ExternalBodyWrench` contracts. This is
consistent with MuJoCo's documented contact-force/Jacobian formulation and Rapier's
contact graph and solver-force reporting:

- [MuJoCo computation: contacts and constraint forces](https://mujoco.readthedocs.io/en/latest/computation/)
- [Rapier advanced collision detection and contact graph](https://rapier.rs/docs/user_guides/rust/advanced_collision_detection/)

Two MuJoCo compilation invariants are required by this fixture. RNE `CollisionGroups`
are compiled to deterministic pairwise `<exclude>` entries because MuJoCo's native
`contype`/`conaffinity` acceptance rule is not identical to RNE's bilateral mask rule.
Zero-friction colliders compile with `condim="1"`; retaining tangential constraint rows
with zero friction capacity produced an energy-injecting degeneracy under tire wrenches.

## Acceptance evidence

Each trace carries unit-bearing min/max verdicts and a mutation-detecting digest. Both
backends must pass:

- caster and both drive wheels remain in solved contact for at least half the driven run;
- acceleration unloads and reverse braking reloads the caster;
- horizontal displacement, caster lateral force, swivel angle, and swivel rate are
  non-trivial and bounded;
- the cross-backend gaps in load recovery, displacement, swivel, and lateral force remain
  inside declared SI-unit tolerances.

The tests also require same-runtime Rapier byte-for-byte repeatability, validate the
MuJoCo/Rapier comparison, reject trace mutation, verify collision-group compilation, and
verify normal-only compilation for frictionless contact.

The verified comparison artifact is generated outside the repository at
`E:\RNE-build\m3c-sensor\diff-caster-comparison-v1.json`:

| metric | Rapier | MuJoCo | cross-backend gap |
| --- | ---: | ---: | ---: |
| caster contact fraction | 1.000 | 1.000 | — |
| caster load recovery | 124.669 N | 125.335 N | 0.666 N |
| final horizontal displacement | 0.9541 m | 0.9595 m | 0.0055 m |
| maximum caster lateral force | 8.406 N | 8.632 N | 0.225 N |
| maximum caster swivel | 0.5646 rad | 0.5602 rad | 0.0044 rad |
| maximum caster swivel rate | 0.9709 rad/s | 0.9565 rad/s | — |
| minimum drive-wheel contact fraction | 1.000 | 1.000 | — |

Run and emit evidence with:

```powershell
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend diff-caster-rapier --output diff-caster-rapier-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend diff-caster-mujoco --output diff-caster-mujoco-v1.json
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend diff-caster-compare --output diff-caster-comparison-v1.json
```

## Explicit limits

This is a dynamics fixture, not a full real-robot validation claim. The drive and caster
contacts use sphere proxies on a flat rigid plane; suspension compliance, wheel profile,
roughness, curb impact, split friction, lift/recontact, closed-loop sensor-only control,
and real-log parameter identification remain later M3-C/M5 gates. Three support points
avoid the over-constrained uneven-ground problem; four-or-more-wheel vehicles still need
identified suspension before comparable load-transfer claims are justified.

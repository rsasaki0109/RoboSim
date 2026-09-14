# 012 — Backend-neutral articulated-body dynamics

## Status

Implemented (Phase 0 of the native legged-control foundation).

## Context

RNE's legged and mobile-manipulation work needs model-based quantities —
joint-space inertia, gravity and velocity bias, center of mass, and the
center-of-mass Jacobian — that currently live only inside physics backends or
are inferred numerically. Scripted joint-space gaits hit a steering and
disturbance-rejection ceiling (`docs/GO2_LOCOMOTION.md`, `docs/G1_LOCOMOTION.md`);
the honest next step is a model-based gait schedule (ZMP, capture point,
whole-body control, centroidal MPC). Every one of those controllers needs the
same primitive: deterministic articulated-body dynamics that does not depend on
Rapier, MuJoCo, or a renderer.

External libraries (Pinocchio, RBDL, Crocoddyl) demonstrate the algorithm set
but are C++ dependencies with their own math conventions and no place in the
core crate graph.

## Decision

Add `rne_dynamics`, a backend-neutral, deterministic articulated-body dynamics
crate derived directly from the [`rne_robot`] link/joint graph.

The crate owns:

- spatial vector algebra (motion `[linear; angular]`, force `[force; torque]`)
  and `6x6` spatial transforms and inertias;
- `ArticulatedModel`: a topological tree with per-link spatial inertia, joint
  motion subspaces, and gravity, wrapping a `KinematicModel` for ordering and
  forward kinematics;
- composite-rigid-body mass matrix (`mass_matrix`);
- recursive Newton-Euler inverse dynamics (`rnea`) and the bias
  (`non_linear_effects`, `gravity_torque`);
- forward dynamics via a deterministic dense solve (`forward_dynamics`);
- center of mass (`center_of_mass`) and the center-of-mass Jacobian
  (`com_jacobian`);
- world-frame link motion and bias acceleration (`link_motions`) and a
  base-twist-consistent spatial Jacobian (`frame_jacobian`) for task-space
  control. `com_jacobian` and `frame_jacobian` use the same body-twist base
  convention as `mass_matrix` and `rnea`, unlike `rne_robot`'s roll-pitch-yaw
  Jacobian, so the two must not be mixed.

The crate does not own a physics backend, a renderer, contacts, or a solver
policy. It reads `RigidBody` / `RigidBodyInertia` components through `rne_physics`
and never mutates the world.

### Floating base

When the base link carries `FloatingBase`, the model gains six degrees of
freedom. The base configuration is `(x, y, z, roll, pitch, yaw)` with fixed-axis
roll-pitch-yaw, while the base generalized velocity and acceleration are the
body-frame spatial twist and its derivative. This mirrors the free-flyer
separation between configuration and tangent space used by native dynamics
libraries: `q` drives the transforms, `qd` / `qdd` drive the Newton-Euler
recursion. Gravity is folded into the base acceleration (`a_0 = -g_body`) so the
recursive pass yields `g(q)` directly.

## Dependency boundary

`rne_dynamics` may depend on:

- `rne_math` for vectors and transforms;
- `rne_ecs`, `rne_world`, and `rne_robot` for the model graph;
- `rne_physics` for backend-neutral `RigidBody` and `RigidBodyInertia`;
- error crates used across the workspace.

It must not depend on Rapier, MuJoCo, wgpu, ROS 2, adapters, or an external
dynamics library. It must not require a renderer or wall-clock time. It is the
matrix and bias layer that `rne_legged`, `rne_ai`, and control code can share.

## Validation

Unit tests in the crate pin:

- the two-link point-mass mass matrix against its closed form;
- the two-link gravity torque against `d V / d q`;
- linearity of `rnea` in `qdd` (`rnea(q, qd, qdd) = M qdd + rnea(q, qd, 0)`);
- symmetry and positive definiteness of `M`;
- floating-body `M` equal to its `6x6` spatial inertia and the floating gravity
  wrench against a hand derivation;
- energy conservation of a torque-free double pendulum under RK4, which
  exercises the Coriolis / centrifugal bias.

`examples/103_dynamics_diagnostics` spawns the 12-DoF Unitree Go2, enables the
floating base, and checks that the `18x18` mass matrix is symmetric and positive,
that gravity and the center of mass are finite, and that a free-fall forward
step satisfies `M qdd + g = 0` to solver tolerance and produces `-9.81 m/s²`
base acceleration.

## Limitations and follow-ups

- Mimic joints are folded into kinematics but contribute no independent
  velocity or acceleration in the dynamics recursion.
- Only the dense solve is provided; an ABA / articulated-body forward pass and
  analytical derivatives are later work needed for DDP-class solvers.
- Centroidal momentum matrix, contact constraints, and constrained forward
  dynamics are deliberate follow-ups.
- The floating-base tangent separation means callers must integrate base pose
  and base velocity with an explicit mapping rather than `q += qd * dt`.

## Consequences

- Model-based controllers can be tested headless and deterministically without a
  physics backend.
- A single dynamics source of truth now backs planning, control, and AI work.
- The remaining gap to Pinocchio/Crocoddyl is derivatives and constrained
  dynamics; this crate is the prerequisite, not the replacement.

# 014 — Backend-neutral whole-body control

## Status

Implemented (Phase C of the v0.16 model-based legged-control goal).

## Context

`rne_dynamics` provides the mass matrix, inverse dynamics, center of mass, and
bias accelerations; `rne_legged` provides a deterministic center-of-mass
trajectory and ZMP reference. Something must turn those templates into the joint
torques that realize them on a floating-base robot while the feet stay fixed and
the contact forces respect friction. That is whole-body control.

The reference formulations are task-space inverse dynamics and hierarchical
whole-body control (Khatib operational space; Sentis and Khatib; Del Prete's
TSID): solve for the robot accelerations and contact wrenches that satisfy the
floating-base equations of motion and the contact constraints while driving
task-space objectives.

## Decision

Add `rne_wbc`, a deterministic weighted inverse-dynamics whole-body controller.

The controller solves a least-squares problem over the stacked unknowns
`[qdd; f_1 .. f_c]`:

- the floating-base equations of motion
  `(M qdd + h - sum_c J_c^T f_c)[0..6] = 0` as high-weight rows;
- contact no-slip `J_c qdd = -Jdot_c qd` at each active contact point;
- an optional center-of-mass task with feed-forward acceleration and
  position/velocity feedback;
- an optional joint posture task;
- Tikhonov regularization on the joint accelerations and contact forces.

The contact no-slip and dynamics rows carry large weights, so they act as
constraints; the tasks shape the remaining degrees of freedom. The normal
equations are solved after column equilibration, which keeps the system
well-conditioned when the mass matrix mixes large base inertias with small link
inertias. Contact forces are projected into the Coulomb friction cone and the
actuated joint torques are recovered from
`S^T tau = M qdd + h - sum_c J_c^T f_c`, then clipped to any configured limits.

The crate owns contact points, friction cones, the task definitions, the
weighted solve, and the solution diagnostics. It does not own a physics backend,
a renderer, a contact schedule, or a state estimator.

### Dynamics convention

The controller uses `rne_dynamics::frame_jacobian`, whose floating-base columns
map the *body-twist* generalized velocity used by `mass_matrix` and `rnea`.
`rne_robot::KinematicModel::jacobian` uses roll-pitch-yaw base rates instead, so
the two must not be mixed; `com_jacobian` was moved onto the dynamics
convention for the same reason. `link_motions` supplies the bias acceleration
`Jdot qd` without an explicit Jacobian derivative.

## Dependency boundary

`rne_wbc` may depend on `rne_dynamics`, `rne_robot`, `rne_ecs`, `rne_world`, and
`rne_math`. It must not depend on a renderer, physics backend, ROS 2, or an
external control stack, and it must not mutate simulation state.

## Validation

Unit tests pin friction-cone projection, a floating body supporting its own
weight through two contacts (contact forces sum to `m g`, base wrench residual
below half a newton, contact acceleration residual below `1e-3`), rejection of
invalid inputs, and determinism.

`examples/105_whole_body_control` runs the controller on the floating-base 12-DoF
Unitree Go2 with four foot contacts and reports:

```text
standing: mass=27.09kg vertical_force=265.72N (weight=265.72N)
standing: base_residual=8.950e-9 contact_residual=6.558e-8 max_torque=4.41Nm saturated=false
com task: requested_x=1.000 achieved=(0.769, -0.011, 0.000) m/s²
```

The base equations of motion are satisfied to `1e-8` and the realized contact
forces support the exact body weight. The Go2 URDF is Z-up and is rotated to the
Y-up world before the solve.

## Limitations and follow-ups

- The dynamics and contact rows are high-weight soft constraints, not hard
  inequalities, and the friction cone is enforced by projection after the solve
  rather than inside it. A hierarchical or active-set QP with hard friction and
  torque inequalities is the next fidelity tier.
- Contacts are point forces without contact moments, and the active set is fixed
  for a solve; a contact schedule and touchdown / lift-off switching are not yet
  part of the controller.
- The controller is a single-shot solve. Driving it from `rne_legged` at the
  simulation rate, with a running `rne_ai` or `rne_physics` loop, is the next
  integration step toward a walking robot.

## Consequences

- The template trajectory from `rne_legged` can now be realized as joint torques
  on the same model that `rne_dynamics` evaluates.
- The remaining gap to a walking robot is the closed-loop scheduling and QP
  fidelity, not the projection of tasks onto contacts.

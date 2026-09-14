# 013 — Backend-neutral legged walking templates

## Status

Implemented (Phase B of the v0.16 model-based legged-control goal).

## Context

RNE's first legged foundation used scripted joint-space gaits and learned
torque overlays. `docs/GO2_LOCOMOTION.md` and `docs/G1_LOCOMOTION.md` record the
ceiling those gaits reach: steering and disturbance rejection need foot
placement coordinated with body dynamics, which a joint-space kinematic schedule
cannot express. The honest next step is a model-based gait schedule.

The reference algorithms are well established and deterministic:

- the Linear Inverted Pendulum Model (LIPM) and ZMP, with Kajita's preview
  control;
- the Divergent Component of Motion (DCM) and capture point (Pratt,
  Englsberger), with the closed-form DCM foot placement;
- the XCoM/capture-point orbit used for steady-speed footstep placement.

None of these require a physics backend, a renderer, or ROS 2, and all are
specified by exact closed-form equations, which fits the determinism and
replay requirements of the core crates.

## Decision

Add `rne_legged`, a deterministic template-planner crate that turns a footstep
request into a center-of-mass trajectory and ZMP reference.

The crate owns:

- `Horizontal` planar geometry in the Y-up world's `X`/`Z` plane;
- `LimpParams`, `LimpState`, `capture_point`, `dcm_step`,
  `propagate_constant_zmp`, and `footstep_from_dcm`;
- `ZmpPreviewController`, which solves the discrete algebraic Riccati equation
  by deterministic fixed iteration and derives the preview gains from the
  affine value-function recursion;
- `FootstepPlan`, `GaitSchedule`, and `ZmpSegment` with smoothstep
  double-support transitions and dedicated start/finish weight shifts;
- `WalkingPattern` and `plan_walking_pattern`, which run the preview
  controller independently on the `X` and `Z` axes.

The crate does not own a physics backend, contact solver, whole-body controller,
renderer, or asset pipeline.

### Model

The LIPM keeps the center of mass at a constant height `h` with natural
frequency `omega = sqrt(g / h)`. The DCM is `xi = com + com_vel / omega`. Under
a constant ZMP `p` the closed form is

```text
com(t) = p + (com0 - p) cosh(omega t) + (com_vel0 / omega) sinh(omega t)
xi(t)  = p + (xi0 - p) exp(omega t)
```

Placing the foot at `p = (xi0 exp(omega T) - xi_end) / (exp(omega T) - 1)`
drives the DCM to `xi_end` over a single-support interval `T`; setting
`xi_end = xi0` recovers the capture-point step that brings the center of mass to
rest above the foot.

### ZMP preview control

The LIPM is discretized with jerk input,
`x = [com, com_vel, com_acc]`, and ZMP output `p = com - (h/g) com_acc`. The
controller minimizes ZMP tracking error and jerk weight, giving
`u = -K x + sum_i f_i p_ref(k + i)`. The feedback `K` comes from the
steady-state Riccati solution; the preview gains come from iterating the
affine term `C Q` by `M = A^T - A^T P B B^T / (R + B^T P B)`.

The footstep plan produces a ZMP reference that holds each stance foot during
single support and moves between consecutive stance feet with a cubic
smoothstep during double support. The initial and final transitions use a longer
weight shift, so the reference is trackable from rest.

## Dependency boundary

`rne_legged` may depend on:

- `rne_math` for vectors and transforms;
- `rne_dynamics`, `rne_robot`, `rne_ecs`, and `rne_world` for later model-aware
  extensions;
- `serde` and `thiserror` used across the workspace.

It must not depend on a renderer, physics backend, ROS 2, or an external
planning/control stack. A physics backend or whole-body controller consumes the
planned center-of-mass trajectory; the planner never mutates simulation state.

## Validation

Unit tests pin:

- the capture point and DCM closed forms against their analytic expressions;
- `footstep_from_dcm` inverting `dcm_step` exactly;
- a capture-point step bringing the center of mass to rest;
- the preview controller stabilizing and tracking a constant reference;
- deterministic, finite patterns whose steady ZMP tracking error stays within
  a centimeter and whose feet advance monotonically.

`examples/104_legged_pattern` plans an eight-step Go2-scale walk and reports:

```text
walk: footsteps=10 duration=4.44s samples=888 com=(0.000, 0.000) -> (0.800, -0.002)
zmp tracking: overall=0.0602 m steady(after 1s)=0.0072 m
bounds: max_com_speed=0.668 m/s max_dcm_offset=0.806 m
push recovery: dcm=(0.000, 0.070) step=(0.000, 0.070) rest_com=(0.000, 0.070) rest_speed=0.0000
```

The 6 cm overall error is the initial weight-shift transient; the steady gait
tracks the ZMP within 7 mm.

## Limitations and follow-ups

- The template is planar and constant-height; it does not yet consume terrain
  elevation or a costmap. A footstep planner over `rne_nav` elevation and cost
  maps is the Phase D follow-up.
- The plan is open loop: there is no whole-body controller, contact force
  distribution, or state feedback. Phase C adds the hierarchical-QP whole-body
  layer that realizes the trajectory on the articulated model.
- The preview controller tracks the reference ZMP only; capture-point footstep
  adjustment under a measured disturbance is provided as a primitive but is not
  yet closed around a running robot.
- Only straight-line plans are generated; turning follows once the footstep
  planner is terrain-aware.

## Consequences

- A deterministic, backend-free walking pattern can be generated and tested
  headless and replayed from a request.
- The crate is the reference trajectory source that the whole-body controller
  and, later, a learned residual policy can consume.
- The open gap to a walking robot is now a control and contact problem, not a
  gait-shape problem.

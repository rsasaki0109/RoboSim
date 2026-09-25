# Multi-floor navigation

Every other map in this repository is one horizontal surface. That is the right
model for a floor and the wrong model for a building, and the difference is not
a bigger grid: floors do not share a coordinate plane, and a robot crosses
between them only at specific places, through a device that takes time and can
refuse. An indoor service robot that cannot leave its floor is a fundamentally
different product from one that can.

This document covers the three pieces that close that gap, and the physical
constraints each of them was measured against.

## Building maps and cross-floor routes

`rne_nav::BuildingMap` is a set of `Floor`s — each with its own costmap and its
world elevation — joined by `FloorTransition`s. A transition names where a robot
boards, where it ends up, what kind of crossing it is (elevator, stairs, ramp),
what the crossing costs in seconds, and whether it may be taken in reverse.

`plan_building_route` searches that graph. Within-floor legs are delegated to
the ordinary 2D `plan_path`, so obstacles, inflation and unknown-cell policy
behave exactly as they do for single-floor navigation; crossings are taken
whole, because a robot cannot stop halfway up a lift. The result is a sequence
of `RouteLeg::Drive` and `RouteLeg::Cross`.

Two deliberate choices:

- **A leg that cannot be planned removes that edge rather than failing the
  search.** A blocked corridor should make the planner prefer another staircase,
  not report the whole building unreachable.
- **The caller decides how a metre of floor compares with a second of waiting**
  (`RouteCosts::drive_cost_per_meter`). Otherwise the planner cannot tell
  whether a far, fast ramp beats a near, slow lift.

Expansion is in cost order with ties broken by node index, so the same building
and endpoints always replan identically.

### A freshly created grid is unknown, not free

An `OccupancyGrid` that has never been observed is entirely *unknown*, and the
default `GlobalPlannerConfig` refuses to route through unknown space. A floor
must be declared free, not merely left empty, or even a same-floor route is
`Unreachable`. Both the tests and example 120 mark their floors free explicitly.

## Elevators

`rne_nav::Elevator` is the timing of a shaft as a pure state machine: `Idle`,
`Opening`, `DoorsOpen`, `Closing`, `Moving`. It holds no physics handles and
reads no wall-clock time, so a boarding sequence replays exactly. A caller
drives it with `update(dt_s)` and applies `car_height_m()` and
`door_opening_m()` to whatever bodies represent the car and doors.

The contract a boarding behaviour actually needs is `is_boardable(floor)`: the
car must be at that floor **and** the doors fully open. Being parked with the
doors still moving is not boardable, which is what keeps a robot from driving
into a closing door.

### The car accelerates, because an instant stop throws its passengers

An early version changed speed instantly. Measured on a rider standing on the
car, stopping from 1 m/s launches an unattached body `v^2 / 2g` into the air —
about 5 cm. The car now follows a trapezoidal profile and comes to rest, which
also lengthens a 7 m ascent from 8.0 s to 9.25 s: 1.25 s to reach 1 m/s over
0.625 m, the same again to brake, and 5.75 m of cruise.

### Riding works through normal contact, once the car is commanded

A body standing on the car needs no special support — it is carried by ordinary
normal contact. But this only holds if the car is *commanded* as a kinematic
body rather than teleported. Writing a kinematic body's pose straight into the
backend with `set_position` leaves its velocity at zero from the solver's point
of view. Two measurements exposed it:

| | with `set_position` | with `set_next_kinematic_position` |
| --- | --- | --- |
| Rider clearance drift over a 7 m ascent | 0.0708 m | **0.0000 m** |
| Cargo carried by a platform moving 0.6 m laterally | −0.0003 m (left behind) | carried |

`rne_physics::CommandedKinematicPose` opts a kinematic body into
`set_next_kinematic_position`, and the elevator car carries it.

The marker is **opt-in rather than the default**, which was itself a measured
decision. Making it unconditional broke the OpenArm showcase: that demo grasps
a block by switching it to a kinematic body and writing its pose to follow the
gripper, and a commanded body arrives a step late with momentum handed to it,
so the left gripper never achieved its contact-gated re-grasp. Teleport
semantics are correct for a carried object — it should arrive exactly where it
is put — and commanded semantics are correct for a platform, so the caller
says which it means. With the marker in place the showcase reproduces its
original digest and the ride stays exact.

## Call buttons

`rne_nav::CallButton` is the building's own control, modelled honestly: a robot
that summons the car by calling an API has demonstrated nothing, whereas one
that has to reach a 2 cm target with enough force, and no more, has.

It accepts only contacts on its face — inside the face radius, and between the
face and the declared plunger travel — sums their force, actuates with
hysteresis so a fingertip resting near the threshold does not chatter, and
reports `just_pressed()` as an **edge**, so one physical press produces exactly
one elevator call however long the presser rests on it.

Like the support-polygon code, it takes contacts as plain values
(`ButtonContact`), so `rne_nav` stays free of any physics backend.

### A press needs a dynamic body

A kinematic presser against a fixed button produces **no contact force at all**:
neither body can move, so the solver never resolves the pair. Example 119 drives
a dynamic fingertip with a known external wrench instead. The face then reads
27 N on impact and 3.75 N settled against a 3 N drive; the excess is Rapier's
penetration-recovery term, which is a push the button genuinely receives.

## Examples

| Example | What it shows |
| --- | --- |
| 118 `elevator_ride` | A rider boards the car and rides 7 m across three floors with zero clearance drift; the car never travels with its doors open and never reports a boardable floor while parked elsewhere |
| 119 `elevator_call_button` | A dynamic fingertip presses the call button through solved contact forces; one press per contact however long it is held, and the car answers |
| 120 `multi_floor_mission` | A 1F-to-3F delivery planned through a `BuildingMap` and executed on the real elevator, routed onto the far lift because a wall seals the near lift's 2F landing |

Example 120's measured run: 5 legs at cost 60.25, 20.25 m driven, 22.65 s in
lifts, 2 crossings, and an identical replan.

## Open

- **Pressing the button with an arm.** Example 120 uses a driven fingertip, not
  a manipulator. The standalone SO-101 scene cannot hold a commanded pose, for
  reasons traced below; that is an arm-control problem, separate from the
  button.

  An earlier revision of this section reported that "all six joints read
  0.000 rad against non-zero targets". **That reading was an artifact.**
  `assets/robots/so101.rne.robot.toml` sets `articulation = true` but not
  `multibody = true`, so its six joints are impulse joints rather than
  reduced-coordinate ones, and impulse joints carry no `JointState` — which is
  what `named_joint_position` reports. The joints were moving; the accessor had
  nothing to read.

  Setting `multibody = true` alone makes the scene produce NaN on the first
  step, because SO-101 joints carry a non-identity origin `rpy` and the joint
  frames must include it (`use_joint_origin_rpy = true`, as
  `mm_mobile_so101.rne.robot.toml` does and documents). With both flags the
  scene is stable and reports real joint angles.

  What remains unexplained is tracking: with the asset corrected, a shoulder
  commanded to 0.5 rad settles near 0.147 rad, and that residual does not
  respond to stiffness (200-4000), to solver iterations (16, 32, 64, 128, 256 —
  it plateaus), or to disabling self-collisions. So it is not a convergence
  problem. The corrected asset is **not** committed: enabling the multibody path
  also changes realized effort from an exact 1.0 N·m to 0.9723 N·m, which
  `direct_effort_actuation_retains_ceiling_and_clamps_command` pins exactly, and
  a change that weakens a pinned assertion without delivering a working arm is
  not worth making.
- **Floor transitions in scene assets.** Buildings are constructed in code;
  there is no `.rne.scene.toml` representation of floors or transitions yet.
- **Localization across floors.** A robot in a moving lift is in a featureless
  box with no odometry cues; nothing here addresses relocalizing on arrival.

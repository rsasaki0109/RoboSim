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

### Buildings are data

A building built in code cannot be shared, reviewed, or swapped without a
recompile. `BuildingDescription` gives it a versioned `.rne.building` file:
which floors exist, where their maps live, and how a robot crosses between
them.

```json
{
  "format": "rne.building",
  "version": 1,
  "name": "office three floor",
  "floors": [
    { "id": 0, "name": "1F", "elevation_m": 0.0, "map": "1f.rne.map", "inflation_radius_m": 0.2 }
  ],
  "transitions": [
    { "name": "far lift", "kind": "Elevator", "from": 0, "to": 1,
      "from_point_m": [10.0, 10.0, 0.0], "to_point_m": [10.0, 10.0, 0.0],
      "cost_s": 20.0, "bidirectional": true }
  ]
}
```

Three decisions worth naming:

- **Floors reference their maps by path rather than embedding them**, so a
  building file stays readable and the maps remain ordinary `.rne.map` files
  that the SLAM and navigation tools already produce. Paths resolve against the
  building file's own directory, so a site directory can be copied whole.
- **Inflation is part of the description, not a caller default.** Two robots
  with different footprints need different inflation over the same map, and a
  building that silently picked one would plan routes the other cannot drive.
- **Load-time validation is the same validation planning uses.** A transition
  whose endpoint falls outside its floor fails when the file is loaded, naming
  the floor, rather than surfacing later as an unexplained routing failure.

`assets/buildings/office_three_floor/` is a committed example; example 120
loads it, and `--emit-site` regenerates it.

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
- **Being carried is not modelled.** `reacquire_floor` answers the question on
  arrival; nothing represents the ride itself, during which the robot's building
  pose is unknowable from its own sensors and its floor-frame pose is the only
  valid one.

## Knowing which floor you arrived on

A robot riding a lift is not navigating: its wheels are still, its odometry
reports no motion, and its scan sees the same car walls the whole way. Yet its
pose in the building changes by a whole storey. When the doors open it has to
answer a question odometry cannot.

`rne_slam::reacquire_floor` answers it from a bounded hypothesis set: the robot
knows which lift it boarded and therefore which floors that lift serves, so the
candidates are one pose per served floor near that lift's alighting point, each
matched locally rather than searched across the building.

**The hard part is not finding a match; it is that floors look alike.** An
office landing on 3F and on 4F can be identical to a planar scanner, and a
matcher asked for its best candidate returns one with high confidence. Placing a
robot on the wrong floor is worse than admitting ignorance — it will navigate
confidently to a room that is not there. So an identification is accepted only
when the winner beats the runner-up by a declared margin, the runner-up is
always reported as the evidence for the answer, and identical floors return
`FloorAmbiguity::Ambiguous` rather than a guess.

### The margin is the defence; the score floor is weak

`min_score` is a sanity floor and not much more. Mean scan likelihood is
dominated by whatever the candidate floors have in common: a map missing a
**full-height interior partition** still scores 0.83 against a scan taken on the
partitioned floor, because the shared outer walls carry most of the beams. A
caller with only one candidate has no margin to fall back on and has to raise
`min_score` deliberately.

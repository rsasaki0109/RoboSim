# ADR 028: Native joint-space motion planning inspired by MoveIt

## Status

Accepted.

## Context

RNE targets mobile manipulators but deliberately keeps MoveIt and ROS out of
core ([006_mobile_manipulator.md](../architecture/006_mobile_manipulator.md)
lists "Full MoveIt integration inside core" as a non-goal). Manipulation work
still needs the capabilities MoveIt is built around: a robot state, swappable
inverse kinematics, collision world queries, goal constraints, trajectories, and
a planner pipeline. Those capabilities must be deterministic, headless, and
usable without a physics backend.

`rne_robot` already had a kinematic model with forward/inverse kinematics and a
self-collision checker, and `rne_nav` has 2D grid planning, but there was no
articulated joint-space planner or scene abstraction.

## Decision

Reference MoveIt's architecture and reimplement it natively, without adding any
MoveIt, ROS, physics-backend, renderer, or external planner dependency to core.

1. In `rne_robot`, add a `KinematicsSolver` boundary with
   `KinematicsSolverRegistry` (the `KinematicsBase` analogue), a `RobotState`
   (`RobotState`), an `AllowedCollisionMatrix`, signed distance queries, path
   collision checking, and a `CollisionWorld` for static objects.
2. Add a new `rne_planning` crate that depends only on `rne_robot`, `rne_ecs`,
   and `rne_math`, and provides `PlanningScene`, `MotionPlanRequest`,
   `MotionPlanResponse`, `GoalConstraint`, `RobotTrajectory`, a `MotionPlanner`
   trait, a `PlannerRegistry`, a `PlanningPipeline`, and built-in
   `JointInterpolationPlanner` and `RrtConnectPlanner` implementations.
3. Treat the `MotionPlanner` trait as the plugin boundary. Additional planners
   are registered in-process; a dynamic-loading ABI is out of scope until a
   concrete need appears.

All sampling uses an explicit seed. No wall-clock time or global random state is
read. The layer stays backend-neutral: it reads ECS state once to build a scene
and then operates on plain values.

## Consequences

- Manipulation planning is available headlessly and deterministically, and can be
  tested without a renderer or physics engine.
- `rne_planning` adds a crate and a workspace member; module boundaries and
  architecture docs must list it.
- The built-in planners are intentionally modest. `RrtConnectPlanner` is a
  deterministic greedy tree planner rather than a complete OMPL-class planner,
  and the built-in damped least-squares solver does not handle joint-angle
  wrapping, so position goals are preferred for redundant chains.
- Because core does not depend on MoveIt, ROS, or an external planner, covering
  more planning algorithms means adding native implementations or new
  `MotionPlanner` registrations.

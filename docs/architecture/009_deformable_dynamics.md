# 009 — Backend-neutral deformable dynamics

## Status

Accepted for implementation.

## Context

RNE needs deterministic cable and cloth simulation for robot manipulation. The
current Rapier backend exposes rigid bodies, articulations, and contacts but no
native deformable-body implementation. Modeling a cable or cloth as many rigid
bodies would leak backend behavior into robot tasks, scale poorly, and make
replay sensitive to joint-island ordering.

The public physics traits must remain backend neutral. Rendering must remain
optional, ROS2 must remain an adapter, and headless tests must exercise the same
simulation logic as rendered examples.

## Decision

Add `rne_deformable`, a backend-neutral deterministic Extended Position Based
Dynamics (XPBD) solver.

The crate owns:

- deformable particles and topology;
- distance, bending-distance, and pin constraints;
- fixed-substep, fixed-iteration sequential constraint solving;
- collision projection against backend-neutral plane, cuboid, sphere, and
  Y-axis capsule primitives;
- cable and rectangular cloth builders;
- stable state hashing and CPU geometry extraction.

The crate does not own a rigid-body backend, renderer, asset parser, policy, or
ROS2 integration. Rigid and kinematic collider poses are sampled into plain
`DeformableCollider` values before a deformable step. The MVP uses one-way
coupling: rigid bodies affect particles, but particle impulses do not affect
rigid bodies.

When enabled, particle self-collision rebuilds a deterministic uniform-grid
broadphase once per substep after distance-constraint convergence. Candidate
pairs are sorted by particle index, directly constrained neighbors are
excluded, and rigid colliders are projected once more after pair separation.
Cloth additionally builds a deterministic triangle-AABB grid and solves
non-adjacent vertex-triangle contacts with barycentric inverse-mass weighting.

`DeformableAttachment` stores stable particle indices and anchors expressed in
another ECS entity's local frame. During world stepping those anchors become
temporary pin constraints, then are removed without changing scene-authored
pins. Proximity acquisition requires a distinct unpinned particle for every
contact point and is all-or-nothing; removing the component releases the body.
This permits deterministic robot manipulation without adding backend handles
or rigid reaction impulses to deformable state.

Constraint arrays are evaluated in insertion order. Colliders are evaluated in
caller-provided stable order. Parallel reductions and wall-clock time are not
used. Every step receives an explicit simulation duration and gravity vector.

## Dependency boundary

`rne_deformable` may depend on:

- `rne_math` for vectors and transforms;
- `rne_ecs` and `rne_world` for deterministic ECS stepping and poses;
- `rne_physics` for backend-neutral collider shapes;
- `bevy_ecs` only for component derives;
- serialization and error crates used across the workspace.

It must not depend on Rapier, wgpu, ROS2, adapters, or `rne_ai`.

Scene assets may depend on `rne_deformable` to spawn cable and cloth components.
`rne_ai` may depend on it to sequence robot-driven attachment and fixed-step
deformable updates after a rigid backend step.
Render integration consumes extracted CPU geometry; solver state never stores
GPU handles.

## MVP sequence

1. Cable particles, distance/bending constraints, pins, gravity, and replay hash.
2. Plane/cuboid/sphere/capsule collision and positional friction.
3. Scene asset schema, ECS spawn, and cable render example.
4. Rectangular cloth structural/shear/bending constraints and dynamic normals.
5. Cloth drape scene, headless test, and wgpu example.

Deterministic self-collision and kinematic robot attachment are implemented
follow-ups layered on the same sequential solver. Tearing,
deformable-deformable collision, and full two-way rigid coupling remain
explicit follow-up work.

## Consequences

- Cable and cloth behavior can be tested without Rapier or rendering.
- The same solver state can produce headless hashes and rendered geometry.
- One-way coupling is insufficient for lightweight rigid bodies reacting to
  cloth or cable forces; a later backend-neutral impulse exchange is required.
- Sequential XPBD prioritizes deterministic behavior and clarity over maximum
  GPU throughput.

# ADR 029: Extensible collider shapes and offline deterministic collision bake

## Status

Accepted.

## Context

`rne_physics::ColliderShape` originally modelled only primitives (sphere,
cuboid, capsule, plane) and was `Copy`. Imported meshes therefore had no true
collision geometry, so `rne_urdf_import` approximated every mesh collision with
a single axis-aligned bounding box. That over-fills concave shapes and degrades
manipulation and locomotion contact.

Supporting real mesh collision needs variable-size data (hull points, triangle
indices, height samples, child shapes). A `Copy` enum cannot hold it, and a
physics backend must not receive engine-specific handles through the neutral
trait. Separately, mesh convex decomposition is expensive and, in available Rust
implementations (for example a work-in-progress CoACD port), non-deterministic,
which conflicts with RNE's determinism contract.

## Decision

1. Extend `ColliderShape` with `ConvexHull`, `TriMesh`, `HeightField`, and
   `Compound` (`CompoundPart`) variants. Variable-size payloads live behind
   `Arc<[T]>`, so `ColliderShape` and `Collider` drop `Copy` and become cheap to
   clone. `rne_deformable::DeformableCollider` and `rne_robot`'s internal link
   collider follow.
2. Rapier converts each variant with `SharedShape::convex_hull`, `trimesh`,
   `heightfield`, and `compound`, guarding the parry convex-hull builder against
   too-few-point inputs and returning a typed
   `PhysicsError::InvalidColliderShape` instead of panicking.
3. MuJoCo rejects non-primitive colliders with a typed compile error until
   Mesh/hfield compilation lands. Analytic ignores colliders. Deformable
   contact, self-collision, and the URDF AABB fallback approximate non-primitive
   shapes as conservative bounding spheres.
4. Add an offline `rne_collision_bake` crate that decomposes a triangle mesh
   into a `Compound` of axis-aligned boxes using a fully deterministic
   voxel-and-merge algorithm (surface voxelization with a triangle/box
   separating-axis test, then a lexicographic greedy merge). Bakes are written
   as a versioned `.rne.collision.json` sidecar.
5. `rne_urdf_import` loads a sidecar next to a mesh collision element and uses
   the scaled compound in place of the AABB fallback. `rne-asset bake-collision`
   authors sidecars. The runtime never decomposes geometry.

No physics backend, renderer, ROS, or external geometry library becomes a
dependency of a core crate: Rapier is the only backend that consumes the new
variants, and the bake crate is an offline tool with no runtime dependents.

## Consequences

- Mesh collision is no longer limited to a bounding box, improving contact
  fidelity for imported robots and obstacles without a runtime decomposition
  cost.
- Dropping `Copy` is a source-breaking change for downstream `Collider` /
  `ColliderShape` copies; dependents must clone or borrow.
- The built-in decomposer is intentionally simple: it marks the mesh surface, so
  parts may overlap and the interior is not filled. It is adequate for collision
  but not for volume queries. A higher-quality convex-hull backend (CoACD, when
  it becomes published and deterministic) can be added behind a feature without
  changing the artifact format.
- MuJoCo, deformable contact, and self-collision remain conservative
  approximations for non-primitive shapes until exact projections and Mesh
  compilation are implemented.

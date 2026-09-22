# 032: Additive convex contact geometry

Status: Accepted

The G1 full-body contact check cannot use the current mesh-to-AABB importer:
boxes fill space outside the source collision meshes, particularly around
folded limbs. A convex hull of each source mesh is needed before optimizing
and qualifying native full-body contact. This change supplies the backend
geometry contract; connecting it to G1 asset import remains a separate step.

`rne_physics::ConvexCollider` is an additive component. Its deterministically
ordered vertices use entity-local meters. It replaces the companion
`Collider` geometry and offset at creation, retaining material, sensor and
collision groups. Existing `ColliderShape` variants, public struct literals,
physics traits and error enums are unchanged. A companion primitive remains
available to consumers that only understand the older geometry contract.

Rapier validates the vertices and builds a positive-volume 3D hull. Missing
companions, simultaneous compound geometry, nonfinite/unrepresentable
coordinates and degenerate hulls produce `InitializationFailed`. The fallible
Parry hull routine is used because the convenience hull constructor panics on
some degenerate clouds. Hull topology and shape allocation occur at creation,
not on each simulation step. Geometry edits after synchronization are not
supported. With declared inertia the collider density stays zero, preserving
physical link mass and inertia.

No backend-specific types or new external dependencies enter core APIs. Other
backends are not made convex-contact-conformant by this component, and must
not be used to qualify the convex contact model. Full G1 import, body/self
contact screening and backflip validation are still pending. In particular,
this addition does not promote the earlier foot-only GIF to qualification.

Tests raycast through the empty corner of a tetrahedron's bounding box and
through its actual sloped face, verify the link identity and authored mass,
and reject empty, coincident, planar, nonfinite and conflicting geometry.

The additive `attach_urdf_convex_colliders` importer API now loads STL point
clouds, applies mesh scale followed by collision-origin rotation/translation,
and stores sorted unique local vertices. The opt-in TOML extension
`urdf.convex_mesh_collisions = true` invokes it after ordinary collider import.
It requires `mesh_collisions = true`. Primitive links retain their geometry;
multiple/mixed collision elements on a mesh link fail explicitly rather than
being fused into a hull. Missing meshes and conflicting compounds fail too.
Existing public asset/spawn structs remain unchanged. Import tests verify
transformed vertices, explicit opt-in, wrong option types and mixed-shape
rejection. G1's 21 mesh links use this path; both soles retain four primitives.

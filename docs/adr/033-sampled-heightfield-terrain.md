# 033: Additive sampled heightfield terrain

Status: Accepted

Every legged, wheeled and manipulation scene in this repository stands on a
flat `ColliderShape::Plane` or a large cuboid. Rough ground, ramps and
undulating terrain cannot be expressed at all, so foot-clearance, slope
stability and disturbance work has no ground to run against. Compound cuboids
can already build discrete steps, but not a continuous sampled surface.

`rne_physics::HeightfieldCollider` is an additive component, following the
`CompoundCollider` and `ConvexCollider` precedent rather than extending
`ColliderShape`. `ColliderShape` is `Copy` and carries no heap storage, so a
sampled grid cannot be a variant of it, and adding one would break the frozen
Rust API baseline for `rne_physics`. The component replaces the companion
`Collider` geometry and offset at creation and retains its material, sensor and
collision groups; the companion stays a bounding approximation for consumers
that do not understand terrain.

Samples are row-major with `rows` samples along entity-local X and `columns`
samples along entity-local Z, so index `row * columns + column` is one cell
corner. `scale_m.x` and `scale_m.z` are the total horizontal extents and
`scale_m.y` multiplies the stored heights, which are meters at a scale of 1.
The patch is centered on the entity.

Parry indexes heightfield samples with the matrix column along X and the matrix
row along Z, which is the transpose of the authored layout. The transpose lives
in `rne_physics_rapier::convert::heightfield_to_shared` so the authoring
convention stays backend-neutral, and a raycast test over an X-only ramp pins
the convention: a transposed grid moves the slope onto Z and changes every
off-diagonal sample.

A heightfield is an open surface with no volume, so it cannot supply a dynamic
body's mass properties. Rapier synchronization rejects a non-`Fixed` body, a
missing companion `Collider`, simultaneous compound or convex geometry, grids
below two samples per axis, height counts that disagree with the declared grid,
nonfinite or `f32`-unrepresentable heights, and non-positive horizontal extents.
All of these produce `InitializationFailed` before a collider is created.
Geometry edits after synchronization are not supported.

Because the patch is finite, a body that leaves its edge falls instead of
finding more floor. That is the declared behavior, not a defect, and the
example asserts rest poses stay inside the patch so an off-edge fall is never
reported as a terrain contact.

Two consequences of an open surface were measured rather than assumed when
replacing an existing scene's ground. The scene ground box is a solid 1 m thick
volume that pushes out a link spawned slightly beneath its surface; a
heightfield does not, so a Go2 whose foot links spawn marginally below zero had
those links fall freely, reaching -122.79 m after 5 s — exactly the free-fall
distance — while its calves rested on the terrain. `attach_ground_heightfield`
therefore documents that the patch must be authored below every spawned link.
Backend-level contact reporting itself is unaffected: a sphere on a flat patch
produces one contact event and six contact points with nonzero normal forces.

The second consequence is more limiting and is recorded here so it is not
rediscovered. Because the surface is open, a penetration that happens *during*
a run is also permanent: the body is never pushed back out and falls away. The
Unitree Go2's foot colliders are 22 mm spheres, and at the scene's default 60 Hz
they tunnel through a sloped patch and fall past -4 m within seconds. Stepping
at 240 Hz bounds per-step penetration enough to survive a 1.75 s run but not a
7 s one, and setting Parry's `HeightFieldFlags::FIX_INTERNAL_EDGES` did not
prevent tunnelling either, while it did change the rolling trajectory in example
116; that flag is therefore deliberately not set, and the measurement is
recorded in `heightfield_to_shared` so it is not retried blindly.

The consequence for users is explicit: this component is suitable for terrain
contact with bodies substantially thicker than the per-step penetration, and is
**not** currently suitable for qualifying legged locomotion with small foot
geometry. Example 117 therefore checks for tunnelling and reports the affected
slopes as invalid rather than scoring them. Closing that gap needs a ground
representation a foot cannot fall out of — a solid volume beneath the sampled
surface, thicker foot collision geometry, or continuous collision detection,
none of which the physics contract exposes today. That remains open.

No backend-specific types or new external dependencies enter core APIs. Other
backends are not made terrain-conformant by this component and must not be used
to qualify terrain contact. Scene-TOML authoring of terrain is deliberately not
part of this change: `SceneCollisionAsset` is also `Copy` and frozen, so terrain
scenes need their own schema entry, which remains open.

Tests cover row-major indexing and sample accessors, validation of degenerate
grids, heights and extents, serde round-tripping of the row-major layout, the
raycast axis convention, non-unit horizontal and vertical scaling with an
off-patch miss, a sphere rolling along the sampled slope rather than a plane,
and every rejection path. Example 116 samples an analytic ramp-plus-ripple
surface, checks the reported contact height against that same analytic function,
and settles three bodies on it.

# 029: Opt-in compound contact primitives

Status: Accepted

URDF links may have multiple disconnected collision elements. The legacy
importer merges them into one AABB, filling empty space between sole spheres
and changing contact behavior. G1 backflip comparison requires retaining the
individual sole shapes without changing existing scenes.

`rne_physics` defines backend-neutral `ColliderPart` and `CompoundCollider`
components. A compound shares the companion `Collider`'s material, sensor and
collision-group behavior, and replaces its geometry at collider creation.
Part poses are relative to the entity, independent of the companion offset.
Part order follows source URDF order. Planes, empty parts and invalid geometry
are rejected by Rapier before stepping. Geometry is creation-time data, as with
the existing collider shape; live geometry replacement is not added here.

`preserve_collision_parts` in the URDF asset/spawn configuration defaults to
false. When enabled, multiple imported shapes become compound parts. Mesh
parts still use the existing AABB approximation; this is not convex-mesh or
full-body collision qualification. A companion bounding collider remains for
consumers that inspect the older component.

Rapier creates a compound shared shape on a single rigid body. With declared
inertia, collider density remains zero, preserving authored mass/COM/inertia.
Contacts and queries retain the original link entity identity. Other backends
are not made compound-contact-conformant by this change and must not be used
to validate this contact model. No vendor types enter public physics traits,
and no dependency boundary changes.

Verification covers a ray through the empty gap, hits on both spheres,
preservation of declared mass after the first physics step, rejection of an
empty compound, URDF origin/order preservation, and opt-in asset propagation.
Native G1 runs separately check the two four-part feet and standing/flip behavior.

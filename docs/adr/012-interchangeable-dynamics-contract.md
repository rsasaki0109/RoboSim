# ADR 012: Interchangeable dynamics conformance contract

## Status

Accepted for the v0.3 interchangeable-dynamics milestone.

## Context

The first physics conformance report proved Analytic and Rapier capabilities,
but the aggregate function selected both concrete backends directly. Rapier's
joint motor application and reduced-coordinate observation also lived behind
backend-specific helper methods. A third backend could implement
`PhysicsBackend` without being able to run the same complete catalog.

## Decision

`rne_physics` owns a versioned `PhysicsBackendManifest` containing only stable
backend and engine identifiers, versions, canonical `PhysicsCapability` values,
and a same-runtime repeatability class. Native engine handles and types remain
private to backend crates.

The publish-disabled built-in conformance suite owns comparisons among RNE's
shipped backends. Its generic `run_backend_conformance` accepts a manifest and
any `PhysicsBackend` factory.
Every advertised capability must map to a shared vector and a named,
unit-bearing tolerance where numeric comparison is required. Missing vectors,
unregistered profiles, manifest/runtime drift, duplicate capabilities, and
non-canonical ordering are deterministic failures.

Completed-step articulation state crosses the backend boundary as the
unit-explicit `JointState` ECS component. Applying `JointMotor` commands belongs
to `sync_from_ecs`; publishing `JointState` belongs to `sync_to_ecs`. A runner
therefore receives the same semantics through the trait and through any
backend convenience helper.

Aggregate report schema v2 embeds each manifest and the actual runtime
capability declaration. The schema, manifest, and tolerance-registry versions
are compiled constants checked against `release/contracts.toml` and a committed
golden JSON shape.

Backend-manifest schema v2 added `kinematic_body` as a refinement of
`rigid_body`. A backend may advertise it only when the catalog's shared
external-pose vector passes. Analytic and Rapier do so; MuJoCo's compiler maps a
kinematic entity to a typed missing-capability error before allocating native
model state.

## Consequences

Analytic, Rapier, and feature-gated MuJoCo now run through one generic catalog
without exposing backend-only observation APIs. Experimental backends can call
the kit before promotion and receive an honest failing report for missing
profiles. MuJoCo has joined with `rigid_body`, `articulation`, and
`contact_force` while remaining default-off. Rapier and MuJoCo consume the same
unit-explicit `JointActuation` commands and synchronize the same backend-neutral
joint state; invalid command units or values fail before stepping. MuJoCo
contact-point forces are integrated over the fixed step and reduced to the same
canonical entity-pair evidence as Rapier, without exposing MuJoCo model, data,
geometry, or contact types through `rne_physics`.

The feature-gated aggregate includes Analytic-vs-MuJoCo and
Rapier-vs-MuJoCo comparisons with named position/velocity bounds. A separate
fault-injection case tightens only the Rapier-vs-MuJoCo position bound, records
the first violating fixed step in the existing Behavior replay schema, and
packages that replay with report schema v2 in a Failure Capsule. Diagnostic
failure evidence therefore does not weaken or silently replace the production
conformance contract.

For independently maintained Rust backends, the publishable
`rne_physics_conformance` authoring crate owns a distinct external report schema
v1. Catalog v2 keeps ten checks in fixed order, including completed-step native
realization of direct joint-effort actuation, uses a backend-independent tolerance
registry that authors cannot widen, accepts arbitrary stable backend IDs, and
fails unsupported GPU/soft-body claims explicitly. Each report hashes the exact
implementation subject and canonical manifest; Failure Capsule verification
requires the matching subject bytes. The built-in aggregate package is named
`rne_physics_conformance_suite` so the public authoring crate has the stable
`rne_physics_conformance` registry name.

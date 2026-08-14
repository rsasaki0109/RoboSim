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

The publish-disabled conformance kit owns catalog execution. Its generic
`run_backend_conformance` accepts a manifest and any `PhysicsBackend` factory.
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

## Consequences

Analytic, Rapier, and feature-gated MuJoCo now run through one generic catalog
without exposing backend-only observation APIs. Experimental backends can call
the kit before promotion and receive an honest failing report for missing
profiles. MuJoCo has joined with `rigid_body` while remaining default-off, and
cannot claim articulation until it synchronizes the same backend-neutral joint
state.

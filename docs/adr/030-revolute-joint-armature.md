# ADR 030: Revolute generalized armature

- Status: Accepted
- Date: 2026-09-22

## Context

The G1 screening plant adds 0.01 kg·m² to each actuated joint coordinate.
Matching its link masses, spatial inertia and sole geometry does not reproduce
this term. Adding link spatial inertia instead changes floating-base coupling
and is not the same generalized inertia. Rapier 0.22 exposes read-only
factorized generalized inertia and no public armature setter.

## Decision

Add the backend-neutral `RevoluteJointArmature { inertia_kg_m2 }` ECS component
on an articulated child link. Rapier requires a realized revolute multibody
joint and finite, nonnegative inertia representable in its f32 scalar type.
Unsupported or invalid configurations fail through the existing error API.
Zero/absence preserves the original dynamics; removing the component restores
zero armature at the next synchronization. No existing public struct or enum
changes shape. Other backends do not yet implement this component.

Vendor the already-used Apache-2.0 Rapier 0.22.0 package under
`third_party/rapier3d`, with license and upstream file hashes. The implementation
patch is confined to its multibody module. Add a validated coordinate setter,
a zero-initialized armature vector, and its diagonal to both acceleration and
constraint mass matrices before permutation/factorization:

`M_effective(q) = M_spatial(q) + diag(armature)`.

The added term is constant in joint coordinates, so it adds no configuration
derivative, damping, gravitational load, or applied force. It affects direct
torque, motor/limit constraints and contact response through the same inverse
inertia. Vector entries follow joint growth, append, split and root-type
changes. The patch skips the new addition for zero entries to retain legacy
arithmetic. Do not modify the shared Cargo registry in place.

The vendor manifest has an independent workspace for its tests, and the engine
uses a workspace-level Cargo patch. The new dependency source is explicit in
Cargo.lock. Normal release packaging must retain this directory; published
crate consumers do not inherit Cargo workspace patches automatically. The backend feature `experimental-armature` is off by default and is enabled
by example 114. Enabling it requires this patch; default backend builds still
compile against unmodified crates.io Rapier and reject nonzero armature.
Native armature support therefore currently belongs to this repository build.

## Verification and limits

Backend tests compare the undamped analytic response for fixed and floating
bases, using both direct torque and torque-limited force-based motors. Tests
retain spatial mass, exercise removing the component and reject invalid values
or unsupported placement. Vendor tests cover coordinate remapping and setter
validation. Native G1 probes compare zero and 0.01 kg·m² with the same candidate.

The native controller's articulated model still uses spatial inertia for
COM/kinematics; no claim is made that its inverse-dynamics or optimal-control
mass matrix incorporates this new component. Model parity also requires
contact, actuator, friction and collision coverage checks. Armature support
alone is not evidence of a successful or hardware-ready backflip.

The vendor subsequently also incorporates the independent
[free-root COM correction](031-multibody-free-root-com.md). Zero-armature
compatibility above refers to the armature addition itself; recordings before
that numerical fix keep their historical source hashes.

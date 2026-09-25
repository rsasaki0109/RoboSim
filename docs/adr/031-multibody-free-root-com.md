# ADR 031: Free-root coordinates at the center of mass

- Status: Accepted
- Date: 2026-09-22

## Context

The native G1 investigation found that Rapier 0.22's multibody free-root
Jacobian uses root COM linear velocity, but its free-joint translation
coordinates track the body origin. With a nonzero local COM, integrating
angular velocity consequently moves the physical COM without the reported
linear velocity. A zero-gravity welded-pair regression with coincident COMs
0.2 m from their origins drifts approximately 0.192 m in 0.1 s at 10 rad/s,
despite zero COM linear velocity and no applied forces.

## Decision

Extend the existing repository-local Rapier patch in the same multibody
module. The free root's second joint frame is placed at its local COM and
translation coordinates locate the world COM. Re-express coordinates without
moving the body when reading poses, changing local COM, or switching fixed and
dynamic roots. Generalized velocities, mass Jacobians and physical link
inertias retain their COM convention. No engine public API changes.

The correction applies to all builds using the vendored crate, independently
of `experimental-armature`. Default backend consumers can still compile
against unmodified crates.io Rapier, but do not receive this numerical fix.
This is a behavioral fix, so old native recordings retain their original
source/binary hashes and must not be compared as bit-identical trajectories.

## Verification and limits

A backend integration regression checks that both welded bodies rotate about
a stationary COM. Vendor tests cover zero and nonzero COM, translating and
rotating motion, body-pose reads, fixed/dynamic transitions and changed COM
without a pose jump. The old armature acceleration/remapping tests remain.
The f32-to-f64 quaternion normalization regression is a separate correction.

Neither correction proves a native backflip. Contact coverage, complete
landing qualification and hardware feasibility remain separate requirements.

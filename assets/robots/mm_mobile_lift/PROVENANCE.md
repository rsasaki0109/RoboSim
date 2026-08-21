# `mm_mobile_lift` visual provenance

## Current state

This directory currently ships the project-owned URDF only. Its visual
geometry is made from simple URDF primitives; no authored GLB/GLTF mesh,
texture, or active robot-visual manifest is committed yet. The schema and
validator are implemented in `rne_assets`; an asset manifest will be added
with the authored meshes so a design placeholder cannot be mistaken for
finished visual evidence.

The current physics and robot source is:

- [`mm_mobile_lift.urdf`](mm_mobile_lift.urdf)
- SHA-256: `sha256:315ff1a0800a760a28b44cbbd7b903ecf7a3635d6a90b664db2002883b85df0d`
- Model: `mm_mobile_lift`
- Contents: 10 links and 9 joints

The source URDF is part of this repository and is distributed under the
repository's MIT/Apache-2.0 licensing terms. No third-party mesh, texture,
scan, or external robot package is bundled by this visual-contract slice.

## Visual authoring requirements

The next visual slice may add link-scoped GLB/GLTF files and PBR textures only
after recording, for every imported source, the following information:

- source URL or repository and immutable revision;
- upstream license and required attribution/notice text;
- local modifications and coordinate/scale conversion;
- file SHA-256 and the exact link/LOD mapping;
- whether the asset is authored, generated, or redistributed.

Unlicensed or unverifiable mesh bytes must not be added to this directory.

## Scope boundary

The visual mesh is presentation-only. Adding it must not change the existing
URDF collision geometry, declared masses/inertia, joint axes, joint limits,
joint types, sensor mounts, or simulation policy. Runtime rendering should
follow the post-physics link transforms, while headless simulation must remain
valid when the visual mesh is absent.

The contract's target budgets are LOD0 at 120,000 triangles or fewer, LOD1 at
40,000 triangles or fewer, at most eight material slots, 2K maximum textures,
and 16 MiB total texture bytes. These are acceptance limits for the next
authoring slice, not measurements of the current primitive fallback.

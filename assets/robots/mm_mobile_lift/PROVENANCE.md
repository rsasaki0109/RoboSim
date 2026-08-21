# `mm_mobile_lift` visual provenance

## Current state

This directory ships a project-authored, deterministic visual pack alongside
the physics URDF. The active visual contract is
[`mm_mobile_lift.visual.toml`](mm_mobile_lift.visual.toml); it maps all ten
links to LOD0/LOD1 GLBs under [`meshes/`](meshes/). The GLBs contain rounded
and bevelled surfaces, multiple material parts, and embedded metallic-
roughness PBR maps (base color, normal, metallic-roughness, emissive, and
occlusion).

The current physics and robot source is:

- [`mm_mobile_lift.urdf`](mm_mobile_lift.urdf)
- SHA-256: `sha256:489fa0e7ef67fa471d2909695d3e3a98658eab3f250262d7005c670469cf702b`
- Model: `mm_mobile_lift`
- Contents: 10 links and 9 joints

The source URDF and all generated visual bytes are authored in this
repository and are distributed under the repository's MIT/Apache-2.0
licensing terms. No third-party mesh, texture, scan, or external robot
package is bundled.

## Reproducible authoring

Regenerate the complete pack from the repository root with:

```text
python tools/generate_mm_mobile_lift_visuals.py
```

The generator uses only the Python standard library and writes no timestamps,
random identifiers, host paths, or network-fetched data. It is the source of
truth for the generated GLBs; the manifest records the link/LOD mapping and
the validator checks every mesh before use. A second run must produce byte-
identical files (the release evidence records the SHA-256 values).

## Visual authoring requirements

The visual bytes are **authored/generated**, not redistributed. There is no
upstream URL, revision, or attribution requirement. For every generated file,
the source is the deterministic generator at
[`tools/generate_mm_mobile_lift_visuals.py`](../../../tools/generate_mm_mobile_lift_visuals.py);
the coordinate convention is `rne_y_up_x_forward`, scale is `[1, 1, 1]`, and
the exact LOD mapping is pinned in the manifest. The generator output is
checked into the repository so README captures do not depend on a toolchain
or network service being present.

## Scope boundary

The visual mesh is presentation-only. Adding it must not change the existing
URDF collision geometry, declared masses/inertia, joint axes, joint limits,
joint types, sensor mounts, or simulation policy. Runtime rendering should
follow the post-physics link transforms, while headless simulation must remain
valid when the visual mesh is absent.

The manifest-wide budgets are LOD0 at 120,000 triangles or fewer, LOD1 at
40,000 triangles or fewer, 80 material-homogeneous parts, 2K maximum textures,
and 16 MiB total decoded texture bytes. The generated pack measures 13,188
LOD0 triangles, 5,132 LOD1 triangles, 38 parts per LOD, 8 px maps, and 97,280
decoded texture bytes. Collision geometry, joint axes/limits, and inertial
values remain unchanged from the physics asset.

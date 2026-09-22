# Collision Bake

Offline approximate convex decomposition for mesh colliders. The runtime never
decomposes geometry: an offline tool writes a deterministic `.rne.collision.json`
sidecar next to a mesh, and importers load it as a `ColliderShape::Compound`.

## Why

URDF/SDF/MJCF mesh collision previously fell back to a single axis-aligned
bounding box, which over-fills concave shapes and produces poor contact. The bake
pipeline replaces that fallback with a compound of convex boxes.

The decomposition is deterministic by design (voxel grid + lexicographic greedy
merge), so baked artifacts are reproducible and can be committed. A
higher-quality convex-hull backend such as CoACD can be added later behind a
feature without changing the artifact format; note that the current CoACD-rs
implementation is unpublished, manifold-only, and uses non-deterministic RNG,
which is why it is not a default dependency.

## Artifact

`rne_collision_bake::CollisionBake` (schema version 1, kind
`rne_collision_bake`) stores:

- `source_triangle_count`, `config`, `part_count`;
- `shape`: a `ColliderShape::Compound` of `Cuboid` parts in the mesh's local
  frame.

A sidecar path is `<mesh path>.collision.json` (for example
`base_link.stl.collision.json`).

## Authoring

```bash
cargo run -p rne_asset_cli -- bake-collision \
  --mesh assets/robots/demo/meshes/base_link.stl \
  --out  assets/robots/demo/meshes/base_link.stl.collision.json \
  --max-cells-per-axis 16 \
  --max-parts 2048
```

`--max-cells-per-axis` trades fidelity for part count. The tool fails closed when
the merged part count exceeds `--max-parts` or the voxel grid exceeds the
internal budget.

## Consumption

`rne_urdf_import` checks for a sidecar when resolving a mesh collision element.
When present and parseable, the baked compound is scaled into the mesh frame and
used instead of the AABB fallback; otherwise behavior is unchanged.
`rne_asset_cli` can also inspect and validate the artifact as JSON.

## Determinism and limits

- Pure integer/f64 grid arithmetic with fixed iteration order; identical inputs
  produce byte-identical artifacts.
- Surface voxelization (triangle/box separating-axis test) marks the mesh shell.
  The decomposition is an approximation: parts may overlap and the interior is
  not filled. This is adequate for collision but not for volume queries.
- Non-primitive shapes remain conservative bounding spheres in deformable
  contact and self-collision until exact projections land.
- MuJoCo rejects compound colliders until Mesh/hfield compilation is added.

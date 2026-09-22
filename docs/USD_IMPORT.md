# USD Import

Minimal, offline ASCII Universal Scene Description (`.usda`) import. The
importer is a strict subset, like `rne_sdf` and `rne_mjcf`: unsupported
constructs are rejected with a clear error instead of being silently dropped.
There is no layer composition, references, payloads, variants, or binary
`.usdc`/`.usdz` support.

## Supported subset

- `#usda 1.0` magic on the first non-empty line.
- `def`/`over`/`class` `Xform` and `Mesh` prims nested by braces, with an
  optional parenthesized metadata block after the prim name.
- `double3`/`float3` `xformOp:translate` and `matrix4d` `xformOp:transform`
  (rotation + translation; scale/shear is ignored, rotation is normalized).
- `point3f[] points`, `int[] faceVertexCounts`, `int[] faceVertexIndices`, and
  `float3[] primvars:displayColor`.
- Unknown attributes are skipped by consuming a balanced value.

Transforms compose parent to child. When both translate and matrix ops are
present on one prim, the translation is added to the matrix translation; a full
`xformOpOrder` is not modelled.

## Output

`rne_usd::parse_usda` / `parse_usda_file` return a `UsdScene` of world-space
`UsdMesh` values (vertices, triangle indices, optional color). Polygon faces are
fan-triangulated. `UsdMesh::to_obj` and `UsdScene::write_obj_files` write
Wavefront OBJ, which the existing mesh pipeline consumes.

```bash
cargo run -p rne_asset_cli -- usd-import path/to/asset.usda --out-dir out/meshes
```

The importer never mutates core types to fit USD; it only produces native
meshes and values.

## Limits

- One ASCII layer, no composition/references/binary formats.
- Rigid transforms only; scale and shear are ignored rather than baked.
- Mesh collision is not produced here; pair the OBJ output with
  `rne-asset bake-collision` (or an explicit scene collision shape) as needed.

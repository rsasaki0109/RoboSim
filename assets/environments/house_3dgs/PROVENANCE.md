# House 3DGS fixture provenance

`house_3dgs_fixture.ply` is a deterministic, procedural Gaussian cloud made
for RNE's visual-only House showcase. It is not a scan and contains no
third-party geometry, photographs, textures, or learned model weights. The
cloud intentionally contains point-dense surfaces for a floor, three walls,
ceiling edge, window glazing/frame, rug, sofa, coffee table, kitchen island,
stools, cabinet/television, and plant so that RGB colour and proxy depth stay
legible at the 640x480 capture size.

Regenerate it from the repository root with:

```text
python tools/generate_house_3dgs.py
```

The generator is deterministic and dependency-free. The checked-in sidecar
`house_3dgs_fixture.metadata.json` records the generator hash, PLY hash, byte
size, point count, and semantic group counts. The manifest consumed by the
renderer is `house_3dgs.rne.splat.toml`.

## Coordinate and rendering contract

- Right-handed world coordinates, Y-up, metres; the RNE camera looks along
  local `-Z`.
- The room shell spans approximately `x,z = [-3.2, 3.2]` and `y = [0, 3]`.
- The manifest uses `rne.gaussian_splat.v1`; 3DGS is a visual background only.
- Physics, collision, navigation, and robot link transforms remain owned by
  their normal RNE scene/assets. A mesh foreground may be composited through
  `HybridRenderScene`.


# PLATEAU CityGML import

RNE imports a bounded PLATEAU CityGML tile offline. PLATEAU-specific XML and
geospatial types never enter simulation core crates: the converter emits normal
RNE scene assets, building and road OBJ/MTL meshes, copied textures, and a
stable metadata sidecar.

The importer prefers semantic `bldg:Building` LOD2 boundary surfaces, falls back
to building LOD1 solids, and imports `tran:Road` LOD1 surfaces
from the [Project PLATEAU LOD2 building specification](https://www.mlit.go.jp/plateaudocument02/tocC/tocC_02/tocC_02_02/).
It supports `gml:Polygon` exterior `gml:LinearRing` geometry expressed with
3D `gml:posList` or `gml:pos` coordinates. Untextured polygon interior
rings are triangulated deterministically; textured polygons currently require
one exterior-ring UV list and therefore reject interior rings explicitly.

LOD2 `RoofSurface`, `WallSurface`, `GroundSurface`, `OuterCeilingSurface`,
`OuterFloorSurface`, and `ClosureSurface` classifications are preserved.
`app:ParameterizedTexture` targets are matched by polygon ID; their exterior
ring UV order is written to OBJ `vt` records. Referenced PNG/JPEG files must
use safe relative paths. They are copied into the generated asset bundle and
sampled as sRGB base color by the wgpu renderer.
The mapping follows PLATEAU's
[`ParameterizedTexture` model](https://www.mlit.go.jp/plateaudocument/toc4/toc4_22/toc4_22_03/toc4_22_03_01/)
and [lower-left UV convention](https://www.mlit.go.jp/plateaudocument/toc9/toc9_05/toc9_05_02/).

## Convert a tile

```bash
cargo run -p rne_plateau_import -- \
  path/to/tile.gml \
  --output target/plateau/533946 \
  --tile-name 533946
```

The output contains:

```text
533946/
├── 533946.rne.scene.toml
├── 533946.plateau.json
├── meshes/
    ├── plateau_building_0000_<stable-id>.obj
    ├── plateau_building_0000_<stable-id>.mtl
    ├── plateau_road_0000_<stable-id>.obj
    └── ...
└── textures/
    └── appearance_0000.png
```

The JSON sidecar preserves each `gml:id`, name, function, measured height,
generated entity name and mesh path, LOD, semantic surface counts, texture
paths, local translation, world-space bounds, and triangle count. Buildings are
sorted by `gml:id`; generated scene, metadata,
filenames, vertices, and triangle order are deterministic.

Each `tran:Road/lod1MultiSurface` polygon becomes a visual road mesh. For a
straight surface whose length is at least 1.5 times its width, the importer also
derives two opposing lane centerlines in stable principal-axis order. These
lanes are explicitly marked as derived approximations: LOD1 describes the whole
road boundary and does not provide authoritative lane separation.

## Coordinates

`--coordinate-mode auto` recognizes common geographic PLATEAU CRS identifiers,
including EPSG:6697. Geographic triples are interpreted as latitude, longitude,
and ellipsoidal height. A local tangent approximation maps longitude to `+X`,
height to `+Y`, and latitude to `-Z`. Explicit projected mode preserves the
source's horizontal tuple order: axis 1 maps to `+X`, height to `+Y`, and axis 2
to `-Z`. Reorder projected coordinates before import when their CRS axis order
does not match the desired RNE axes.

By default, the source origin is the tile bounds center at its minimum height.
For adjacent tiles that must share an exact origin, provide it explicitly:

```bash
--coordinate-mode geographic --origin 35.6812,139.7671,0
```

The local tangent conversion is intended for bounded city tiles, not
country-scale reprojection. Pre-tile large datasets or supply projected
coordinates before import.

## Physics and headless use

Each building becomes a fixed RNE object. Its visual uses the generated LOD1 or
LOD2 OBJ/MTL, while collision uses a deterministic axis-aligned bounding box.
This keeps
headless physics inexpensive and avoids exposing CityGML or renderer-specific
types through physics traits. Collision follows the imported building bounds; it
does not reproduce concave footprints. Road meshes are visual surfaces over the
scene's fixed ground collider.

The repository includes a synthetic CC0 CityGML fixture and tests that verify:

- stable `gml:id` ordering and metadata;
- byte-identical repeated conversion;
- OBJ loading through the normal RNE mesh pipeline;
- LOD2 semantic surface counts, UV preservation, and Appearance texture decoding;
- headless scene spawning with fixed building colliders;
- deterministic road mesh and opposing-lane derivation;
- bounded Ackermann commands and 60 Hz SimClock-driven vehicle replay;
- geographic and projected Y-up coordinate conversion.

## Official PLATEAU traffic traversal GIF

The runnable example converts the checked-in official Sanjo City 2025 mesh,
loads 213 buildings and 59 road surfaces headlessly, then renders two
SimClock-driven Ackermann cars accelerating, steering, and braking on a
derived road pair beside the textured LOD2 Kita-Sanjo Station:

```bash
cargo run -p plateau_drone_gif --example 46_plateau_drone_gif
```

It writes the eight-second car-follow `docs/media/plateau-car.gif` animation
and matching reduced-motion PNG. Frames are rasterized at 1280×720 and downsampled to a
960×540 GIF. A deterministic CPU presentation pass uses the renderer's linear
depth buffer for atmospheric perspective, replaces empty pixels with a
sky-to-horizon gradient, and applies restrained color balance and vignette.
This pass changes presentation pixels only; simulation and camera depth outputs
remain unchanged. Set `RNE_SKIP_GPU=1` to run only the conversion and headless
load smoke.

The official road LOD1 geometry has zero elevation while building geometry
uses surveyed absolute elevations. For this visualization the example
deterministically places every building's lowest AABB face on the road datum.
The source meshes and textures are unchanged. Approximate lane markings are
example-authored overlays. Buildings and vehicles use the wgpu renderer's
2048×2048 directional shadow map with 3×3 percentage-closer filtering; the
light projection is fitted to scene bounds and snapped to shadow texels to
limit shimmer between frames. The data subset, source URL, and CC BY 4.0
attribution are recorded beside the example.

This presentation strategy follows the official
[PLATEAU daytime visualization tutorial](https://www.mlit.go.jp/plateau/learning/tpc26-1/),
which recommends texture, fog, lighting, and kitbashed or extruded detail to
improve city footage. RNE feeds the example's procedural CC0 texture through
the same CityGML Appearance → OBJ/MTL → wgpu path used by file imports. It also keeps
derived lane markings explicitly approximate: the
[PLATEAU road LOD guidance](https://www.mlit.go.jp/plateaudocument02/tocD/tocD_02/_007b6849-33c2-5206-74a3-025bfbf0bdcd/)
states that LOD1 represents a road as a surface, while internal road
classification begins at LOD2.

## Phase 1 limits

- Building LOD1 solids and LOD2 boundary surfaces are supported; terrain,
  vegetation, signals, `GeoreferencedTexture`, and LOD2 traffic-area semantics
  are not.
- Appearance support is limited to polygon-targeted `ParameterizedTexture`
  PNG/JPEG images with one exterior-ring UV list.
- Derived lanes currently support elongated straight road polygons. Curved
  centerlines, intersections, and authoritative `tran:TrafficArea` lanes are
  future work.
- Interior rings are supported for untextured polygons.
- One bounded CityGML file per invocation.
- Static AABB collision rather than triangle-mesh collision.
- Geographic conversion uses a local tangent approximation rather than a full
  geodetic projection library.

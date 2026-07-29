# PLATEAU CityGML import

RNE imports a bounded PLATEAU CityGML tile offline. PLATEAU-specific XML and
geospatial types never enter simulation core crates: the converter emits normal
RNE scene assets, a canonical `.rne.traffic.json` network, building and road
OBJ/MTL meshes, copied textures, and a stable metadata sidecar.

The importer prefers semantic `bldg:Building` LOD2 boundary surfaces, falls back
to building LOD1 solids, and imports `tran:Road` LOD1 surfaces plus LOD2/LOD3
`TrafficArea` and `AuxiliaryTrafficArea` surfaces
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
├── 533946.rne.traffic.json
├── 533946.plateau.json
├── meshes/
    ├── plateau_building_0000_<stable-id>.obj
    ├── plateau_building_0000_<stable-id>.mtl
    ├── plateau_road_0000_<stable-id>.obj
    └── ...
└── textures/
    └── appearance_0000.png
```

The JSON sidecar preserves each `gml:id`, name, class, every function, measured height,
generated entity name and mesh path, LOD, semantic surface counts, texture
paths, local translation, world-space bounds, and triangle count. Buildings are
sorted by `gml:id`; generated scene, metadata,
filenames, vertices, and triangle order are deterministic.

Each `tran:Road/lod1MultiSurface` polygon becomes a visual road mesh. For a
straight surface whose length is at least 1.5 times its width, the importer also
derives two opposing lane centerlines in stable principal-axis order. These
lanes are explicitly marked as derived approximations: LOD1 describes the whole
road boundary and does not provide authoritative lane separation.

## Road semantics

When a road contains inline `tran:TrafficArea` or
`tran:AuxiliaryTrafficArea`, the importer selects `lod3MultiSurface` before
`lod2MultiSurface`, preserves each area's `gml:id`, direct `tran:class`, and all
direct `tran:function` codes, and builds the road mesh from those non-overlapping
semantic surfaces. Road-level class and function codes are read only from direct
children, so an area's function cannot be mistaken for the containing road's
function.

The mapping follows the official PLATEAU
[`TrafficArea_function.xml`](https://www.mlit.go.jp/plateaudocument/toc4/toc4_03/toc4_03_04/toc4_03_04_01/_trafficarea_function_xml/)
definitions:

| LOD | Code | Meaning | RNE import |
|-----|------|---------|------------|
| 2 / 3.0 | `1000` | carriageway | two opposing derived driving lanes |
| 2 / 3.x | `1020` | carriageway intersection | semantic area preserved; topology deferred |
| 2 / 3.x | `2000` | sidewalk area | one derived bicycle/pedestrian path |
| 3.1 | `1010` | explicit vehicle lane | one derived driving centerline |

For LOD2 and LOD3.1, auxiliary code `3000` represents an island, median, or
tram stop island according to the official
[`AuxiliaryTrafficArea_function.xml`](https://www.mlit.go.jp/plateaudocument/toc4/toc4_03/toc4_03_04/toc4_03_04_01/AuxiliaryTrafficArea_function_xml/).
It is preserved in metadata and mesh geometry but is not emitted as a
traversable lane.

PLATEAU's [LOD3.1 road definition](https://www.mlit.go.jp/plateaudocument/toc4/toc4_03/toc4_03_01/toc4_03_01_04/%E4%BA%A4%E9%80%9A%E9%81%93%E8%B7%AF%E3%83%A2%E3%83%87%E3%83%AB_lod3_1%E3%81%AE%E5%AE%9A%E7%BE%A9_/)
states that code `1010` separates vehicle lanes. The polygon is
source-authoritative, but PLATEAU CityGML does not provide the travel direction
used by RNE. Therefore the emitted centerline, width, and canonical
principal-axis direction remain `derived` with `heuristic` accuracy and a
human-readable method. Road and area raw codes remain available in the PLATEAU
metadata sidecar. No inferred value is labeled authoritative.

The traffic file is validated and serialized by `rne_traffic` schema v1.
Network and lane IDs are namespaced by the sanitized `--tile-name`, all
geometry is in the same local `map` frame as the scene, and repeated imports of
identical input are byte-identical.

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
- byte-identical `.rne.traffic.json` output and schema validation;
- OBJ loading through the normal RNE mesh pipeline;
- LOD2 semantic surface counts, UV preservation, and Appearance texture decoding;
- headless scene spawning with fixed building colliders;
- deterministic road mesh and opposing-lane derivation;
- road-level class/function preservation, LOD2 traffic/auxiliary areas, and
  LOD3.1 code-`1010` lane extraction;
- bounded Ackermann commands and 60 Hz SimClock-driven vehicle replay;
- geographic and projected Y-up coordinate conversion.

## Official PLATEAU traffic traversal GIF

The runnable example converts the checked-in official Sanjo City 2025 mesh and
loads 213 buildings and 59 road surfaces headlessly. Its 84 imported directed
lanes pass through `build_traffic_topology`, producing 26 junctions, 137
connections, and 128 symmetric conflict pairs. `shortest_lane_route` selects a
16-lane, 731 m path with three left and seven right turns, and
`materialize_lane_route` converts the lane/connection sequence into runtime
geometry. One hundred textured CC0 Kenney sedans follow it under three
fixed-time red/green controls:

```bash
cargo run -p plateau_drone_gif --example 46_plateau_drone_gif
```

It writes the twelve-second car-follow `docs/media/plateau-car.gif` animation
and matching reduced-motion PNG. Frames are rasterized at 1280×720 and downsampled to a
960×540 GIF. A deterministic CPU presentation pass uses the renderer's linear
depth buffer for atmospheric perspective, replaces empty pixels with a
sky-to-horizon gradient, and applies restrained color balance and vignette.
This pass changes presentation pixels only; simulation and camera depth outputs
remain unchanged. Set `RNE_SKIP_GPU=1` to run only the conversion and headless
acceptance replay. That replay runs 720 steps twice with opposite ECS spawn
orders and requires the same stable state hash, zero red-light violations, zero
collisions, a two-meter minimum bumper gap, and at least 60 simulation steps per
wall-clock second.

Set `RNE_TRAFFIC_DEBUG` to a comma-separated selection of `lanes`, `route`,
`signals`, `connections`, and `conflicts` (or `all`) to render batched debug
overlays. `RNE_RENDER_FRAME_COUNT=1` and
`RNE_MEDIA_DIR=target/plateau-debug-media` provide a fast one-frame
visualization smoke without replacing README media.

The official road LOD1 geometry has zero elevation while building geometry
uses surveyed absolute elevations. For this visualization the example
deterministically places every building's lowest AABB face on the road datum.
The source meshes and textures are unchanged. Approximate lane markings are
example-authored overlays. A low grass-colored ground receiver removes the
empty-sky gaps between imported surfaces. Concrete sidewalks and curbs follow
the selected road's derived principal axis and width. The procedural
intersection asphalt bridges the small gap between independently derived road
surfaces; stop and crosswalk markings make the approximation explicit. Three
signals render the deterministic red/green phases consumed by the traffic
runtime. CC0 procedural street trees, streetlights, and guardrails are
sampled at fixed longitudinal
fractions; a candidate is omitted whenever its clearance disc intersects an
imported building collision AABB. Tests verify stable placement and non-overlap.
Buildings, streetscape props, and vehicles use the wgpu renderer's
2048×2048 directional shadow map with 3×3 percentage-closer filtering; the
light projection is fitted to scene bounds and snapped to shadow texels to
limit shimmer between frames. The tracked sedan uses one cached body mesh and
four wheel instances: wheel spin comes from integrated distance, the front pair
receives route steering, and rear lamp intensity follows actual deceleration.
The remaining 99 vehicles use one-draw body-mesh LODs so the full fleet stays
within the renderer item budget. Red and blue body palettes preserve the source
windows, lamps, trim, and shading. The PLATEAU data attribution, Kenney CC0 notice, and
procedural streetscape CC0 dedication are recorded beside the example.

This presentation strategy follows the official
[PLATEAU daytime visualization tutorial](https://www.mlit.go.jp/plateau/learning/tpc26-1/),
which recommends texture, fog, lighting, and kitbashed or extruded detail to
improve city footage. RNE feeds the Kenney OBJ/MTL palette and the example's
procedural CC0 texture through the same textured-mesh path used by CityGML
Appearance imports. It also keeps derived lane markings explicitly approximate: the
[PLATEAU road LOD guidance](https://www.mlit.go.jp/plateaudocument02/tocD/tocD_02/_007b6849-33c2-5206-74a3-025bfbf0bdcd/)
states that LOD1 represents a road as a surface, while internal road
classification begins at LOD2.

## Phase 1 limits

- Building LOD1 solids and LOD2 boundary surfaces are supported; terrain,
  vegetation, signals, and `GeoreferencedTexture` are not.
- Appearance support is limited to polygon-targeted `ParameterizedTexture`
  PNG/JPEG images with one exterior-ring UV list.
- Derived centerlines currently support elongated road/traffic-area polygons.
  Curved centerlines, tile stitching, intersections, and travel-direction
  attribution are handled by the topology phase rather than the semantic
  reader.
- Interior rings are supported for untextured polygons.
- One bounded CityGML file per invocation.
- Static AABB collision rather than triangle-mesh collision.
- Geographic conversion uses a local tangent approximation rather than a full
  geodetic projection library.

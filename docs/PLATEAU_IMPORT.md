# PLATEAU CityGML import

RNE imports a bounded PLATEAU CityGML tile offline. PLATEAU-specific XML and
geospatial types never enter simulation core crates: the converter emits normal
RNE scene assets, building and road OBJ meshes, and a stable metadata sidecar.

The importer targets `bldg:Building` LOD1 solids and `tran:Road` LOD1 surfaces
from the [Project PLATEAU standard product specification](https://www.mlit.go.jp/plateaudocument/).
It supports `gml:Polygon` exterior `gml:LinearRing` geometry expressed with
3D `gml:posList` or `gml:pos` coordinates. Polygon interior rings are rejected
with a feature-specific diagnostic instead of being silently dropped.

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
└── meshes/
    ├── plateau_building_0000_<stable-id>.obj
    ├── plateau_road_0000_<stable-id>.obj
    └── ...
```

The JSON sidecar preserves each `gml:id`, name, function, measured height,
generated entity name and mesh path, local translation, world-space bounds, and
triangle count. Buildings are sorted by `gml:id`; generated scene, metadata,
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

Each building becomes a fixed RNE object. Its visual uses the generated LOD1
OBJ, while collision uses a deterministic axis-aligned bounding box. This keeps
headless physics inexpensive and avoids exposing CityGML or renderer-specific
types through physics traits. Phase 1 collision follows the LOD1 envelope; it
does not reproduce concave footprints. Road meshes are visual surfaces over the
scene's fixed ground collider.

The repository includes a synthetic CC0 CityGML fixture and tests that verify:

- stable `gml:id` ordering and metadata;
- byte-identical repeated conversion;
- OBJ loading through the normal RNE mesh pipeline;
- headless scene spawning with fixed building colliders;
- deterministic road mesh and opposing-lane derivation;
- bounded Ackermann commands and 60 Hz SimClock-driven vehicle replay;
- geographic and projected Y-up coordinate conversion.

## Drone and traffic traversal GIF

The runnable example converts the synthetic tile, loads it headlessly, then
renders a deterministic drone path over the buildings while two SimClock-driven
Ackermann cars accelerate, steer, and brake along imported opposing lanes:

```bash
cargo run -p plateau_drone_gif --example 46_plateau_drone_gif
```

It writes both aerial `docs/media/plateau-drone.gif` and eight-second
car-follow `docs/media/plateau-car.gif` animations, with matching
reduced-motion PNGs. Frames are rasterized at 1280×720 and downsampled to a
960×540 GIF. A deterministic CPU presentation pass uses the renderer's linear
depth buffer for atmospheric perspective, replaces empty pixels with a
sky-to-horizon gradient, and applies restrained color balance and vignette.
This pass changes presentation pixels only; simulation and camera depth outputs
remain unchanged. Set `RNE_SKIP_GPU=1` to run only the conversion and headless
load smoke.

For presentation, the example deterministically generates a larger CC0
PLATEAU-style showcase containing ten varied-height buildings and a 90-meter
road. Sidewalks, curbs, markings, road wear, batched facade windows, rooftop
equipment, trees, streetlights, signals, and contact shadows are
example-authored render overlays rather than imported CityGML semantics. Round
primitive dimensions and the presentation pass have unit tests, while the
traffic replay remains exact and SimClock-driven. The showcase license is
recorded beside the example.

This presentation strategy follows the official
[PLATEAU daytime visualization tutorial](https://www.mlit.go.jp/plateau/learning/tpc26-1/),
which recommends texture, fog, lighting, and kitbashed or extruded detail to
improve LOD1 city footage. RNE uses procedural solid-color detail here because
the renderer does not yet ingest CityGML appearance textures. It also keeps
derived lane markings explicitly approximate: the
[PLATEAU road LOD guidance](https://www.mlit.go.jp/plateaudocument02/tocD/tocD_02/_007b6849-33c2-5206-74a3-025bfbf0bdcd/)
states that LOD1 represents a road as a surface, while internal road
classification begins at LOD2.

## Phase 1 limits

- Building solids and road surfaces at LOD1 only; no terrain, vegetation,
  textures, signals, or LOD2 traffic-area semantics.
- Derived lanes currently support elongated straight road polygons. Curved
  centerlines, intersections, and authoritative `tran:TrafficArea` lanes are
  future work.
- Exterior polygon rings only.
- One bounded CityGML file per invocation.
- Static AABB collision rather than triangle-mesh collision.
- Geographic conversion uses a local tangent approximation rather than a full
  geodetic projection library.

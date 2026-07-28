# PLATEAU CityGML import

RNE imports a bounded PLATEAU CityGML tile offline. PLATEAU-specific XML and
geospatial types never enter simulation core crates: the converter emits normal
RNE scene assets, per-building OBJ meshes, and a stable metadata sidecar.

Phase 1 targets `bldg:Building` LOD1 solids from the
[Project PLATEAU standard product specification](https://www.mlit.go.jp/plateaudocument/).
It supports `gml:Polygon` exterior `gml:LinearRing` geometry expressed with
3D `gml:posList` or `gml:pos` coordinates. Polygon interior rings are rejected
with a building-specific diagnostic instead of being silently dropped.

## Convert a tile

```bash
cargo run -p rne_plateau_import -- \
  path/to/533946_bldg_6697_op.gml \
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
    └── ...
```

The JSON sidecar preserves each `gml:id`, name, function, measured height,
generated entity name and mesh path, local translation, world-space bounds, and
triangle count. Buildings are sorted by `gml:id`; generated scene, metadata,
filenames, vertices, and triangle order are deterministic.

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
does not reproduce concave footprints.

The repository includes a synthetic CC0 CityGML fixture and tests that verify:

- stable `gml:id` ordering and metadata;
- byte-identical repeated conversion;
- OBJ loading through the normal RNE mesh pipeline;
- headless scene spawning with fixed building colliders;
- geographic and projected Y-up coordinate conversion.

## Drone and traffic traversal GIF

The runnable example converts the synthetic tile, loads it headlessly, then
renders a deterministic drone path over the buildings while two scripted cars
travel in opposite directions along the collision-clear center road:

```bash
cargo run -p plateau_drone_gif --example 46_plateau_drone_gif
```

It writes both aerial `docs/media/plateau-drone.gif` and car-follow
`docs/media/plateau-car.gif` animations, with matching reduced-motion PNGs.
Set `RNE_SKIP_GPU=1` to run only the conversion and headless load smoke.

## Phase 1 limits

- Building LOD1 solids only; no road CityGML, terrain, vegetation, textures, or
  LOD2. The Phase 1 GIF road and cars follow an example-authored route; importing
  `tran:Road` geometry and deriving vehicle lanes is the next PLATEAU phase.
- Exterior polygon rings only.
- One bounded CityGML file per invocation.
- Static AABB collision rather than triangle-mesh collision.
- Geographic conversion uses a local tangent approximation rather than a full
  geodetic projection library.

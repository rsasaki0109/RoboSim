# RNE traffic asset schema v1

RNE traffic networks use UTF-8 JSON files ending in `.rne.traffic.json`.
Schema v1 is owned by `rne_traffic` and is independent of PLATEAU, rendering,
physics backends, ROS 2, and external traffic simulators.

Every document starts with:

```json
{
  "schema": "rne.traffic",
  "schema_version": 1,
  "network": {}
}
```

The committed reference document is
[`tests/golden/traffic/schema_v1_reference.rne.traffic.json`](../tests/golden/traffic/schema_v1_reference.rne.traffic.json).

## Stable IDs and references

`TrafficId` values are non-empty ASCII identifiers using letters, digits,
`-`, `_`, `.`, `~`, `:`, `/`, and `#`. Importers should namespace source IDs,
for example:

```text
plateau:53394525/road-main#lane-0
```

Network, lane, junction, connection, signal, signal-group, and signal-phase IDs
share one global namespace. Array positions are never identities. All
lane/junction/connection/signal references must resolve, signal group membership
must agree in both directions, and conflict relationships must be symmetric.

## Provenance and accuracy

Every network, lane, junction, connection, and signal carries `provenance`.
Signal timing has independent provenance because physical signal geometry may
come from a dataset while its phase program is scenario-authored.

Authority classes:

- `authoritative`: directly encoded by the source;
- `derived`: deterministically calculated from source data and accompanied by
  a non-empty `method`;
- `synthetic`: authored by an RNE scenario.

Accuracy classes are `surveyed`, `modeled`, `derived`, `heuristic`,
`scenario_authored`, and `unknown`. Optional horizontal and vertical values use
meters. Authoritative and derived records require at least one source reference.

This distinction prevents a PLATEAU-derived centerline or an RNE-authored signal
program from appearing as source-authoritative data.

## Geometry and units

All geometry uses the document's `coordinate_frame`. Schema v1 supports
`rne_y_up`: right-handed RNE coordinates with Y up. Field suffixes are explicit:

- positions, centerlines, widths, and accuracy: `_m`;
- speeds: `_m_s`;
- signal facing angle: `_rad`;
- phase durations and offsets: `_s`.

Numbers must be finite. Lane widths, speed limits, and phase durations must be
positive. A lane centerline and connection path each require at least two
points.

## Network records

- `lanes` are directed centerlines with lane type, allowed actor classes, width,
  optional speed, and source road class/function values.
- `junctions` represent intersections, merges, splits, roundabouts, or tile
  boundaries.
- `connections` join an incoming lane to an outgoing lane and hold movement
  paths, conflicts, and optional signal control.
- `signals` hold optional physical placement, connection groups, and an
  optional ordered fixed-time phase program.

Phase order is semantic and is preserved. Every phase must specify exactly one
aspect for every group in its signal.

## Canonical serialization

Use `save_traffic_asset` or `canonical_traffic_asset_bytes`; do not call
`serde_json` directly for generated assets. The canonical writer:

1. sorts top-level records by stable ID;
2. sorts set-like source, actor, road-function, conflict, group, connection,
   and group-aspect lists;
3. preserves directed point order and signal phase order;
4. normalizes negative zero to `0.0`;
5. validates the complete reference graph;
6. writes pretty JSON with one trailing newline.

The golden tests prove that shuffled set-like input and negative zero produce
byte-identical output, and that parse/serialize round trips preserve those
bytes.

Lane-only assets can be converted into junctions, connection paths, and
conflicts with the deterministic builder described in
[`docs/TRAFFIC_TOPOLOGY.md`](TRAFFIC_TOPOLOGY.md).

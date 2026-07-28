# Deterministic traffic topology

`rne_traffic::TopologyBuilder` converts one or more lane-only traffic networks
into one canonical network with junctions, directed lane connections, sampled
turn paths, and symmetric conflict relationships. It is backend-neutral and
runs without rendering or physics.

## Inputs and coordinate convention

Every input must be valid traffic schema v1, use the same coordinate frame, and
contain lanes only. Existing junctions, connections, or signals are rejected
instead of being overwritten. Lane IDs must be globally unique across the
inputs.

Directions come from the first and last non-degenerate horizontal segment of
each directed centerline. RNE uses a right-handed Y-up frame, so movement
classification is performed in the XZ plane. Actor classes on the incoming and
outgoing lanes must overlap for a connection to be possible.

## Construction

The default process is deterministic:

1. Sort lanes and endpoints by stable `TrafficId`.
2. Cluster horizontally nearby endpoints whose vertical separation is within
   `max_grade_separation_m`.
3. Pair incoming lane ends with compatible outgoing lane starts.
4. Omit U-turns unless `allow_u_turns` is enabled.
5. Classify straight, left, right, and U-turn movements from endpoint headings.
6. Classify each cluster as a T intersection, cross intersection, merge, split,
   generic intersection, or cross-network tile boundary.
7. Sample every movement as a cubic Bézier path.
8. Detect at-grade path intersections and emit symmetric conflict IDs.
9. Derive stable junction and connection IDs from source IDs, attach derived
   provenance, validate schema v1, and canonicalize the output.

A one-in/one-out cluster is a `tile_boundary` only when it joins lanes from
different source networks. Vertical separation prevents bridge and tunnel
lanes from becoming an at-grade junction or conflict.

## Configuration

`TopologyBuildConfig::default()` uses:

- `endpoint_snap_m = 5.0`;
- `max_grade_separation_m = 1.0`;
- 20-degree straight and U-turn tolerances;
- 12 Bézier segments;
- a handle length of 45 percent of endpoint separation, with a 1-meter minimum;
- `conflict_clearance_m = 0.25`;
- U-turn generation disabled.

All distances are meters and angles are radians. Invalid, non-finite, or
out-of-range values are rejected when the builder is created.

## Guarantees and limits

For identical lane content and configuration, source network order and lane
array order do not affect canonical output bytes. Generated IDs use a fixed
128-bit hash implementation with a committed test vector; they do not depend
on Rust's process-randomized hash state.

The builder is geometric rather than regulatory. It does not infer turn
restrictions, lane-change links, roundabouts, signal placement, priority, or
signal programs. Those require authoritative source attributes or an explicit
scenario policy. It also deliberately rejects already-topologized networks;
incremental editing belongs in a separate operation.

The integration suite covers cross and T intersections, curved approaches,
tile stitching, grade separation, sampled endpoints, symmetric conflicts,
configuration validation, and byte-identical output under shuffled input.

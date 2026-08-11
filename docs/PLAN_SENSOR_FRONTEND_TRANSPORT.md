# M3 production sensor/frontend transport

## Goal

M3 replaces ad-hoc bulk sensor data in the runner's line-oriented status
messages with a versioned, framed binary transport. The simulation remains the
only authority: a slow, disconnected, or incompatible frontend must not block a
fixed step or retain unbounded memory.

The existing `--control-port` protocol v1 remains supported during the 1.0
transition. The production protocol is exposed separately and shares the
transport-neutral runner-control state machine.

## Architecture

- `rne_data::transport` owns the platform-neutral wire contract, negotiation,
  typed RGB-D/LiDAR codecs, frame limits, and bounded egress queue.
- `rne_core::control` remains transport-neutral and contains no sensor or socket
  types.
- `rne-asset` owns the TCP listener and maps DataBus frames into the wire
  contract. Socket reads/writes never run on the simulation thread.
- Native and browser frontends decode the same protocol contract. Adapters map
  decoded RNE payloads into their external ecosystems; external types never
  enter core crates.

## Protocol v1

Every frame has a fixed 32-byte little-endian header:

| Offset | Field | Type |
|---:|---|---|
| 0 | magic (`RNEF`) | `[u8; 4]` |
| 4 | protocol major | `u16` |
| 6 | protocol minor | `u16` |
| 8 | message kind | `u16` |
| 10 | flags | `u16` |
| 12 | payload length | `u32` |
| 16 | monotonically increasing transport sequence | `u64` |
| 24 | run session id | `u64` |

The client sends `ClientHello` first with its supported version range,
capabilities, maximum accepted payload, and queue budget. The server replies
with `ServerHello` or a bounded `Reject`. No status or sensor frame is sent
before negotiation succeeds.

Protocol v1 carries:

- control commands and acknowledgements;
- compact status metadata;
- RGBA8 images;
- little-endian linear-depth `f32` images in metres;
- LiDAR XYZ/intensity/ray/return/channel/timestamp arrays;
- drop/gap notices with cumulative queue statistics.

Dimensions, element counts, payload lengths, finite numeric requirements, and
LiDAR parallel-array alignment are validated before allocation or publication.

## Backpressure and reconnect semantics

- The egress queue has explicit frame and byte limits.
- Control acknowledgements and negotiation messages are reliable. If a client
  cannot accept them within its negotiated budget, that client is disconnected;
  simulation continues.
- Status and sensor messages are latest-only by stable stream key. A newer frame
  replaces an older queued frame for the same key.
- When another latest-only frame must be evicted, the queue records a drop and
  emits a bounded gap notice when possible.
- Socket writes have finite deadlines and run off the simulation thread.
- Disconnect does not imply `quit`. The listener accepts a later client for the
  same run session. The new handshake reports the current transport sequence and
  drop counters; live delivery resumes from the latest state rather than
  retaining an offline backlog.

## Delivery slices

### M3-A: frozen wire contract

- Frame encoder/decoder with strict size and kind validation.
- Explicit version/capability negotiation and rejection reasons.
- Golden byte tests that are identical on Windows and Linux.

### M3-B: production sensor payloads

- Lossless bounded codecs for RGB8, depth-f32, and LiDAR payloads.
- DataBus metadata preserves stream id, sensor sequence, capture ticks, and
  available ticks.
- Malformed dimensions, non-finite values, and misaligned LiDAR attributes are
  rejected.

### M3-C: bounded delivery

- Frame+byte bounded queue with reliable and latest-only classes.
- Deterministic drop accounting and gap messages.
- Bounded DataBus retention in the runner.

### M3-D: runner/frontend integration

- Reconnecting binary TCP server with finite I/O deadlines.
- `rne-asset` command-line surface and native viewer client.
- Legacy `--control-port` compatibility tests remain green.

### M3-E: exit gates

- A slow client cannot delay simulation progress or exceed queue limits.
- A disconnected client creates no retained backlog and can reconnect to the
  same session.
- RGB-D and LiDAR reference payloads decode byte-for-byte and retain timestamps.
- Unsupported protocol ranges and capabilities fail explicitly.
- Unit, integration, golden, parity, Windows, and Linux headless checks pass.
- `cargo fmt --all`, workspace Clippy with `-D warnings`, workspace tests,
  `xtask ci-headless`, and `xtask ci` pass from the locked graph.

## Implementation status

- M3-A frozen wire contract: done locally.
- M3-B production RGB-D/LiDAR payloads: done locally.
- M3-C bounded delivery and DataBus retention: done locally.
- M3-D reconnecting runner and native frontend: done locally.
- M3-E full workspace/CI matrix: done locally on 2026-08-11.

The locked graph passes `cargo fmt --all`, workspace Clippy with
`-D warnings`, `cargo test --workspace`, `xtask ci-headless`, and the complete
`xtask ci` pipeline. The parity catalog contains 20 passing checks, including
wire golden bytes, process-level RGB-D/LiDAR delivery, slow-client progress,
same-session reconnect, and native-viewer sensor projection.

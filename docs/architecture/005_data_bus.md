# DataBus

The RNE DataBus is a typed publish/subscribe bus for sensor and recorder data.

## Core types

- `StreamId`: logical stream identifier
- `Frame<T>`: timestamped payload with sequence numbers
- `FramePayload`: marker trait for typed payloads

## Payloads

- `ImuSample`
- `PointCloud`
- `ImageRgb8`
- `WheelEncoderSample`

## Design rules

- Simulation time comes from `SimClock`, never wall clock.
- Sequence numbers are monotonic per stream.
- Latency is modeled with explicit simulation duration ticks.
- Adapters such as ROS2 subscribe to DataBus outputs rather than changing core types.
- Production runners use bounded per-stream retention. A lagging cursor resumes
  at the oldest retained sequence and the bus exposes cumulative eviction counts.

## Production frontend transport

`rne_data::transport` defines the renderer- and socket-independent production
wire contract. Its fixed 32-byte little-endian frame header carries protocol
version, message kind, payload length, transport sequence, and run session id.
The first frame is an explicit capability/version/limit offer; the server
answers with a selected contract or a bounded rejection.

RGBA8, linear-depth f32, and LiDAR payload codecs preserve DataBus stream id,
sensor sequence, capture ticks, and available ticks. Dimensions and parallel
LiDAR attributes are validated before allocation. The bounded egress queue has
both frame and byte limits: control acknowledgements are reliable, while
status and sensor streams are latest-only and report deterministic gaps.

Socket ownership stays in the runner/frontend. Disconnect never changes
simulation state or implies `quit`; the same run session can negotiate a later
connection without retaining an offline backlog.

## Streaming dataset evidence

`rne_data::dataset` records DataBus output into dataset bundle schema v1
without retaining the complete run. Each record preserves stream sequence,
capture ticks, availability ticks, payload kind, and payload SHA-256. The
manifest freezes calibration, field units, latency/noise behavior, seeds,
assets, and explicit gap semantics; it also hashes the complete record shard.
Dataset-native codecs carry IMU, planar transforms, TaskSpec-ordered actions,
task outcomes, and ground-truth annotations through the same typed `Frame<T>`
and embedded timestamp metadata contract.

`rne_data::offline` verifies depth prediction against ground truth with two
streaming scans and no renderer dependency. See
[Dataset bundle v1](../DATASET_BUNDLE.md) and
[ADR 015](../adr/015-streaming-dataset-bundle.md).

## Example flow

```
Sensor ECS component
  → sample_sensors()
  → Frame<T> on InMemoryDataBus
  → optional rne_log / framed frontend / rne_adapter_ros2 mapping
  → optional streaming dataset bundle / headless offline evaluation
```

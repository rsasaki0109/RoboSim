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

## Example flow

```
Sensor ECS component
  → sample_sensors()
  → Frame<T> on InMemoryDataBus
  → optional rne_log / rne_adapter_ros2 mapping
```

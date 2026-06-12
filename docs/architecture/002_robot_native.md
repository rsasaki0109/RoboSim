# Robot Native Model

RNE models robots with native entities instead of ROS nodes.

## First-class concepts

| Concept | Purpose |
|---------|---------|
| World Entity | gravity, time, random seed, scenario |
| Robot Entity | model metadata, base link, link/joint graph |
| Sensor Entity | IMU, LiDAR, camera, encoders |
| Actuator Entity | wheel motors, servos, grippers |
| Agent Entity | policy, teleop, external controller |
| Episode Entity | reset, reward, termination, recording |

## Robot components

- `Robot`: root metadata and base link reference
- `Link`: physical link on a robot
- `Joint`: parent/child link connection with axis and limits
- `Actuator`: command target applied to a joint or wheel

## Why not ROS in core

ROS2 topics, services, and TF are adapter concerns. The core publishes typed frames on the RNE DataBus so the same simulation can be consumed by Python, ROS2, files, or native Rust tools.

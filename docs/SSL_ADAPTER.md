# SSL simulation-protocol adapter

Adapter for the official RoboCup SSL
[simulation protocol](https://github.com/RoboCup-SSL/ssl-simulation-protocol)
UDP ports. Core crates stay protobuf-free; field geometry scoring stays in
example 76 / `rne.ssl.small_pitch_2v2.v1`. Example 87 couples decoded commands
into that plant.

## Ports

| Port | Message in | Message out |
|---|---|---|
| 10300 | `SimulatorCommand` (ball teleport subset) | `SimulatorResponse` |
| 10301 | `RobotControl` (blue) | `RobotControlResponse` |
| 10302 | `RobotControl` (yellow) | `RobotControlResponse` |

## Run

```bash
cargo test -p rne_adapter_ssl
cargo run --locked -p ssl_adapter_smoke --example 80_ssl_adapter_smoke -- --smoke
cargo run --locked -p ssl_physics_coupling --example 87_ssl_physics_coupling -- --smoke
```

Tests bind ephemeral ports on `127.0.0.1` so CI does not need the real 10300–10302.

## In this slice

- prost encode/decode for `RobotControl` and a ball-teleport `SimulatorCommand`
- typed `SslMoveCommand` / `SslParsedRobotControl` mapping
- kinematics helpers that map SSL local/global/wheel velocities onto
  differential-drive stand-in wheel speeds and SSL ball teleports onto RNE Y-up
- one-shot UDP serve helpers for loopback smoke
- example 87 applies decoded UDP commands to `SslSmallPitchScenario`

## Not in this slice

- Full `SimulatorConfig`, robot teleport, or `google.protobuf.Any` custom feedback
- ssl-vision multicast, Game Controller, AutoRef
- Persistent multi-client simulator process
- Omniwheel kiwi drive (diff-drive stand-ins only)
- Kick / chip / dribbler actuation

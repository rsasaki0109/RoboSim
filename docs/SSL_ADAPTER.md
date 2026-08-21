# SSL simulation-protocol adapter

Wire spike for the official RoboCup SSL
[simulation protocol](https://github.com/RoboCup-SSL/ssl-simulation-protocol)
UDP ports. Core crates stay protobuf-free; field geometry scoring stays in
example 76 / `rne.ssl.small_pitch_2v2.v1`.

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
```

Tests bind ephemeral ports on `127.0.0.1` so CI does not need the real 10300–10302.

## In this spike

- prost encode/decode for `RobotControl` and a ball-teleport `SimulatorCommand`
- typed `SslMoveCommand` / `SslParsedRobotControl` mapping
- one-shot UDP serve helpers for loopback smoke

## Not in this spike

- Full `SimulatorConfig`, robot teleport, or `google.protobuf.Any` custom feedback
- ssl-vision multicast, Game Controller, AutoRef
- Physics coupling to `ssl_small_pitch` (next slice)
- Persistent multi-client simulator process

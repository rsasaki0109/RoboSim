# SSL Adapters

Optional adapters between Robot Native Engine and the RoboCup Small Size League
simulation protocol.

## Rules

- Core crates under `crates/` must never depend on this directory or on SSL
  protobuf / UDP protocol crates.
- Contests scoring for field geometry lives in `rne_ai` (`ssl_small_pitch`).
  This adapter only speaks the wire protocol.
- Do not change core types to fit grSim or ssl-simulation-protocol messages.

## Crates

- `rne_adapter_ssl`: UDP ports 10300–10302 encode/decode helpers, kinematics
  mapping onto differential-drive stand-ins, and loopback smoke for
  `RobotControl` / minimal `SimulatorCommand`. Example 87 couples those
  commands into `ssl_small_pitch`.

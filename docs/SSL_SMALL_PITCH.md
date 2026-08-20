# RoboCup SSL small-pitch 2v2

This slice scores **SSL Division B field geometry** with four robots and a
golf-ball. It is not a [grSim](https://github.com/RoboCup-SSL/grSim) clone
and does not speak the [SSL simulation protocol](https://github.com/RoboCup-SSL/ssl-simulation-protocol) UDP ports.

## What is in v1

| Official check | Judge |
|---|---|
| Division B pitch 9 m × 6 m | `SslSmallPitch` |
| Ball fully across the goal line, inside a 1.0 m mouth, 0.18 m depth | `SslBallRegion::YellowGoal` / `BlueGoal` |
| Ball fully across a sideline or end line outside the mouth | `OutOfField` |
| Ball faster than 6.5 m/s | `legal_ball_speed` |
| 180 mm robot cylinder | scene cuboids of radius 0.09 m |

A scripted blue attacker pushes the ball into the yellow goal. The injected
fault drives it out over a sideline.

## Run

```bash
cargo test -p rne_ai ssl_small_pitch
cargo run --locked -p ssl_small_pitch --example 76_ssl_small_pitch -- --smoke
```

## Not in this slice

- protobuf / ports 10300–10302 (that belongs in `adapters/ssl`, later)
- ssl-vision cameras, Game Controller, AutoRef
- omniwheel kiwi drive (diff-drive stand-ins)
- 11 vs 11
- kick / chip / dribbler hardware

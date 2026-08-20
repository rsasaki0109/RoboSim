# Tsukuba Challenge confirmation run

This slice scores the **2026 confirmation-run checklist** with the official
stop geometry. It is not a 2.2 km photoreal Kenkyugakuen clone.

Official source: [Tsukuba Challenge 2026 tasks](https://tsukubachallenge.jp/2026/regulations/tasks).

## What is in v1

The confirmation run (~400 m in the real event) is a safety check, not the
full 2.2 km / 100 min course. The analog is a few meters long so the same
judges run headlessly at 60 Hz.

| Official check | Judge |
|---|---|
| Two crosswalks: stop within 1.5 m of the road edge, none past the edge | `evaluate_tsukuba_road_edge_stop` + consecutive rest |
| Detect the marshal's green cone: stop or avoid; contact with the cone fails | `no_cone_contact` |
| Operator e-stop: wheels to rest | `e_stop_asserted` and `stopped` |
| No roadway entry | `no_roadway_entry` |
| Complete the confirmation segment | `confirmation_complete` |

The 1 m / 0.5 m **stop-line** box is implemented as
`evaluate_tsukuba_stop_line` for the later full-run / signal-crossing slice.
Confirmation itself uses the road-edge 1.5 m rule.

The marshal e-stop is applied after the cone is handled, while the robot is
still at rest, so the scripted success never drives through the obstacle.

## Run

```bash
cargo test -p rne_ai tsukuba
cargo run --locked -p tsukuba_confirmation --example 75_tsukuba_confirmation -- --smoke
```

`--smoke` requires a clean scripted success and a cone-hit failure on
`no_cone_contact`.

## Assets

- `assets/scenes/tsukuba_confirmation.rne.scene.toml`
- `assets/robots/tsukuba_confirmation.rne.robot.toml`
- `assets/tasks/tsukuba_confirmation.task.json` (`rne.tsukuba.confirmation.v1`)

## Not in this slice

- 2.2 km city hall loop, 100 min budget, hotel clockwise circuit
- Pedestrian-signal optional task B
- Unmapped parking A, delivery D1/D2, other-robot plate E
- RGB-D perception of the cone (the fail condition is geometric contact)
- Official Kenkyugakuen surveyed PLATEAU tile packaging (visual fixture backdrop is example 83; see TSUKUBA_PLATEAU_BACKDROP.md)

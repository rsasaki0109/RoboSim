# Tsukuba full run (shortened)

This slice scores a **shortened full-run analog** with official stop-line geometry.
It is not the 2.2 km / 100 min Kenkyugakuen loop.

Official source: [Tsukuba Challenge 2026 tasks](https://tsukubachallenge.jp/2026/regulations/tasks).

## What is in v1

Three signalized crossings on one scaled sidewalk segment (~10 m):

| Official check | Judge |
|---|---|
| Stop inside the 1 m / 0.5 m stop-line box at each crossing | `evaluate_tsukuba_stop_line` + consecutive rest |
| Wait for pedestrian green before crossing | `signal_wait_at_crossings` |
| No roadway entry | `no_roadway_entry` |
| Reach the goal marker at rest | `full_run_complete` |

The confirmation-run slice (example 75) keeps the road-edge 1.5 m rule and marshal
cone/e-stop checks. This slice uses the stop-line box that confirmation already
implements as a shared judge.

## Run

```bash
cargo test -p rne_ai tsukuba_full_run
cargo run --locked -p tsukuba_full_run --example 79_tsukuba_full_run -- --smoke
```

`--smoke` requires a clean scripted success and a skip-stops failure on
`first_stop_line_stop`.

## Assets

- `assets/scenes/tsukuba_full_run.rne.scene.toml`
- `assets/robots/tsukuba_confirmation.rne.robot.toml` (shared diff-drive robot)
- `assets/tasks/tsukuba_full_run.task.json` (`rne.tsukuba.full_run.v1`)

## Not in this slice

- 2.2 km city hall loop, hotel circuit, 100 min budget
- Optional tasks A/B/D/E from the regulations
- RGB-D cone or signal detection (geometry-only judges)
- Kenkyugakuen PLATEAU scenery (see example 78 for optional 3DGS background)

# Tsukuba confirmation + PLATEAU backdrop

This slice composites a **visual-only** PLATEAU CityGML fixture behind the
analytic Tsukuba Challenge confirmation sidewalk. Contest scoring stays in
example 75 (`rne.tsukuba.confirmation.v1`); the backdrop never enters the
judges or the headless confirmation plant.

## Run

```bash
cargo run --locked -p tsukuba_plateau_backdrop --example 83_tsukuba_plateau_backdrop -- --smoke
```

`--smoke` imports the committed LOD1 fixture, checks building/road counts, and
asserts the confirmation TaskSpec identity is unchanged. Without `--smoke`, a
GPU capture writes a PNG under `target/rne-tsukuba-plateau-backdrop/` when
wgpu is available.

## Assets

- `assets/environments/tsukuba_plateau_backdrop.rne.env.toml`
- Fixture: `crates/rne_plateau/tests/fixtures/plateau_lod1_minimal.gml`
- Confirmation scene/task remain the analytic assets used by example 75

Replace the fixture GML with a Kenkyugakuen PLATEAU tile later; keep
`confirmation_task_id` and do not wire imported collision into the scoring
world.

## Not in this slice

- Changing confirmation judges or TaskSpec digests
- Spawning PLATEAU collision into the confirmation physics plant
- Official Kenkyugakuen surveyed tile packaging in-repo

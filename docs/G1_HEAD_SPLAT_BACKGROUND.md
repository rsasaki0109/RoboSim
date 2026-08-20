# G1 head camera × 3DGS background

Hybrid viewer/dataset capture: the official Unitree G1 `head_link` camera looks
out over a Tsukuba Gaussian-splat sidewalk while the G1 meshes draw in the
foreground.

This does **not** change contest scoring (example 75) or the full RGB-D DataBus
sensor contract (example 71).

## Run

```bash
cargo run --locked -p g1_head_splat_background --example 81_g1_head_splat_background -- --smoke
```

`--smoke` loads the dynamic G1, resolves `head_link`, validates the Tsukuba
confirmation splat manifest, and writes
`target/rne-g1-head-splat/g1_head_splat_capture.json` without requiring a GPU.

Without `--smoke`, a GPU path writes `g1_head_splat.png` plus the same report
with an `rgba_hash`.

## Assets

- G1: `assets/scenes/unitree_g1_dynamic.rne.scene.toml`
- Splat: `assets/environments/tsukuba_confirmation.rne.splat.toml`
- Meshes: `assets/robots/g1_description`

## Not in this slice

- Kenkyugakuen preferred-PLY auto-swap (see Tsukuba 3DGS docs / example 78)
- Depth from the splat pass (foreground mesh depth only)
- Streaming into `rne_sensor` DataBus frames

# Tsukuba 3DGS background

This slice adds optional **3D Gaussian Splatting** backgrounds for viewer and
dataset capture. The Tsukuba Challenge 2026 confirmation-run **scoring stays
headless and analytic** in example 75; nothing here changes contest contracts.

## Architecture

- `rne_render`: `GaussianSplatEnvironment`, `HybridRenderScene`, manifest loader
- `rne_render_3dgs`: `wgpu-3dgs-viewer` adapter + hybrid compositor
- `rne_render_wgpu`: `BackgroundRenderPass` hook and `render_hybrid_scene_camera`

Splat assets are **visual-only**. Physics, task markers, and `rne_ai` behavior
contracts are unchanged.

## Run

```bash
cargo test -p rne_render gaussian_splat
cargo test -p rne_render_3dgs
cargo run --locked -p tsukuba_3dgs_background --example 78_tsukuba_3dgs_background -- --smoke
```

Without `--smoke`, the example writes `target/tsukuba_hybrid.png` when a GPU is available.

## Assets

- `assets/environments/tsukuba_confirmation.rne.splat.toml`
- `assets/environments/tsukuba_confirmation_fixture.ply` (tiny committed fixture)

Replace the PLY with a real Kenkyugakuen scan later; keep the manifest
`environment_id` and record `renderer_identity` in dataset manifests.

## Not in this slice

- RGB-D depth from splats (mesh proxy depth only on the foreground pass)
- Cross-GPU pixel-hash determinism for splat captures
- Dynamic splat updates or learned scene changes

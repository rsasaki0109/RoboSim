# Tsukuba 3DGS background

This slice adds optional **3D Gaussian Splatting** backgrounds for viewer and
dataset capture. The Tsukuba Challenge 2026 confirmation-run **scoring stays
headless and analytic** in example 75; nothing here changes contest contracts.

## Architecture

- `rne_render`: `GaussianSplatEnvironment`, `HybridRenderScene`, manifest loader,
  `GaussianSplatCaptureReport`
- `rne_render_3dgs`: `wgpu-3dgs-viewer` adapter + hybrid compositor
- `rne_render_wgpu`: `BackgroundRenderPass` hook and `render_hybrid_scene_camera`

Splat assets are **visual-only**. Physics, task markers, and `rne_ai` behavior
contracts are unchanged.

## Run

```bash
cargo test -p rne_render gaussian_splat
cargo test -p rne_render_3dgs
cargo run --locked -p tsukuba_3dgs_background --example 78_tsukuba_3dgs_background -- --smoke
cargo run --locked -p tsukuba_3dgs_background --example 78_tsukuba_3dgs_background -- --smoke --environment kenkyugakuen
```

Without `--smoke`, the example writes `target/tsukuba_hybrid.png` and
`target/tsukuba_splat_capture.json` when a GPU is available.

## Kenkyugakuen swap

| File | Role |
|---|---|
| `assets/environments/tsukuba_kenkyugakuen.rne.splat.toml` | `environment_id = tsukuba.kenkyugakuen.v1` |
| `preferred_ply_path = tsukuba_kenkyugakuen.ply` | Drop a real scan here (gitignored) |
| `ply_path = tsukuba_confirmation_fixture.ply` | CI stand-in when the preferred file is absent |

When `tsukuba_kenkyugakuen.ply` is present, the loader uses it and sets
`standin = false`. Otherwise smoke keeps the fixture and reports `standin =
true`. Override any time with `--ply PATH` or `RNE_SPLAT_PLY`.

Capture reports record `environment_id`, `renderer_identity`, `ply_sha256`, and
`standin` so dataset manifests can refuse stand-in captures.

## Assets

- `assets/environments/tsukuba_confirmation.rne.splat.toml`
- `assets/environments/tsukuba_kenkyugakuen.rne.splat.toml`
- `assets/environments/tsukuba_confirmation_fixture.ply` (tiny committed fixture)

## Not in this slice

- Shipping a multi-GB Kenkyugakuen PLY in git
- RGB-D depth from splats (mesh proxy depth only on the foreground pass)
- Cross-GPU pixel-hash determinism for splat captures
- Dynamic splat updates or learned scene changes

# RNE Web Viewer

Browser-viewable MVP for Robot Native Engine: renders the embedded `mm_minimal` robot scene with the native `rne_render_wgpu` stack (WebGPU with WebGL2 fallback). No JavaScript renderer and no physics stepping — arm motion is a deterministic sine sweep driven by a per-frame counter.

## Prerequisites

```powershell
$env:Path += ";$env:USERPROFILE\.cargo\bin"
rustup target add wasm32-unknown-unknown
cargo install trunk
```

## Run locally

From this directory:

```powershell
trunk serve
```

Then open the URL printed by Trunk (default `http://127.0.0.1:8080`).

## Build static bundle

```powershell
trunk build --release
```

Output lands in `dist/` (WASM + `index.html`).

## Controls

- **Left drag**: orbit camera
- **Mouse wheel**: zoom

## Animation note

Joint angles follow `sin(frame_index)` with a fixed period in frames. The browser's `requestAnimationFrame` only schedules redraws; animation phase comes from an integer frame counter, not wall-clock simulation time.

## Workspace checks

```powershell
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy -p rne_web_viewer --target wasm32-unknown-unknown -- -D warnings
trunk build
cargo test --workspace
```

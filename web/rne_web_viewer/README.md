# RNE Web Viewer

Browser-viewable MVP for Robot Native Engine: renders the embedded `mm_minimal` robot scene with the native `rne_render_wgpu` stack (WebGPU with WebGL2 fallback). The replay inspector reads a versioned `.rne-replay` artifact in the browser and exposes a timeline, selected observations, and per-step hashes without rerunning policy code or physics.

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
- **Replay file**: choose a `.rne-replay` or JSON artifact in the Replay inspector
- **Replay slider / Play**: inspect or step through the recorded interval

## Animation note

Joint angles follow `sin(frame_index)` with a fixed period in frames. The browser's `requestAnimationFrame` only schedules redraws; animation phase comes from an integer frame counter, not wall-clock simulation time.

## Replay inspection

Generate an artifact with the headless runner, then start this viewer and select
the file from `target/runs/`:

```powershell
cargo run --release -p rne_asset_cli -- run assets/runs/mesh_diff_drive.rne.run.toml
trunk serve
```

The panel validates schema version, frame count, sequential timestamps, and
finite wheel actions before enabling the timeline. It displays the exact
64-bit physics hash as hexadecimal; the browser does not execute the recorded
actions or invoke a simulator.

## Workspace checks

```powershell
cargo fmt --all
cargo clippy --workspace --all-targets -- -D warnings
cargo clippy -p rne_web_viewer --target wasm32-unknown-unknown -- -D warnings
trunk build
cargo test --workspace
```

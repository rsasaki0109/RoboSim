# README showcase acceptance contract

The front-page animations are executable product evidence, not render-only
mockups. Each capture replays the same deterministic scenario used by its
GPU-free smoke command, then builds the visible actors from post-step world or
observation state.

## Common media contract

- Simulation entries use fixed steps and an explicit seed. Dataset-viewer
  entries use fixed indexed camera states. Capture code does not use wall-clock
  time to advance either path.
- Every entry has a headless smoke path, a GPU capture path, a 960 x 540 poster,
  machine-readable metadata, source provenance, and license files.
- GIF and poster bytes, SHA-256 digests, dimensions, README references, metadata
  evidence, provenance paths, and license paths are checked by
  `cargo run -p xtask -- showcase-media-check`.
- Each GIF is at most 5 MB. The five front-page GIFs together are capped at
  12 MB for a mobile-friendly README.
- Visual-only overlays may improve legibility, but their transforms and task
  state must be rebuilt from the simulation and declared in metadata.

## Catalog and tracked bytes

[`docs/media/showcase.toml`](media/showcase.toml) is the schema-v2 source of
  truth for the real-indoor hero and the 2 x 2 environment grid.

| Showcase | GIF / poster | GIF bytes | poster bytes | poster size |
| --- | --- | ---: | ---: | ---: |
| Real indoor 3DGS mobile manipulation | `house-mobile-manipulation.gif` / `.png` | 291,944 | 910,890 | 960 x 540 |
| Tsukuba Challenge | `showcase-tsukuba.gif` / `.png` | 2,262,164 | 18,815 | 960 x 540 |
| Factory inspection | `showcase-factory.gif` / `.png` | 1,830,619 | 52,228 | 960 x 540 |
| Office AGV delivery | `showcase-office.gif` / `.png` | 1,935,863 | 18,240 | 960 x 540 |
| PLATEAU UAV RGB-D flight | `showcase-uav.gif` / `.png` | 4,329,461 | 439,474 | 960 x 540 |

The current GIF total is **10,650,051 bytes**, below the 12,000,000-byte
combined ceiling. `showcase-media-check` verifies the exact total; regeneration
must update the manifest's sizes and hashes in the same change.

## Task gates

| Showcase | Required simulation evidence |
| --- | --- |
| Real indoor 3DGS mobile manipulation | Real floor-level friction grasp; terminated without truncation; lift clearance at least 0.20 m; payload transport at least 2.0 m; placement error at most 0.10 m; all ten authored PBR links synchronized with zero recorded transform error; no synthetic room furniture is rendered. |
| Tsukuba Challenge | Three stop lines and three signal waits complete; no roadway entry or unstopped overshoot; headless and capture replay digests match. |
| Factory inspection | Official G1 articulation completes all three markers upright; at least 20 mesh items are rendered; replay digest matches. |
| Office AGV delivery | Yield, dock pickup, desk delivery, and desk placement complete; no contact, corridor exit, or early drop; replay digest matches. |
| PLATEAU UAV RGB-D flight | Visible `MultirotorFlight` entity travels at least 60 m; RMS position error at most 1.0 m; altitude error at most 0.6 m; building clearance at least 2.0 m; zero collisions; onboard RGB-D and replay hashes are deterministic. |

The indoor hero uses the photo-derived Voxel51/Graphdeco Dr Johnson capture
under Apache-2.0. Its published COLMAP cameras establish the transform into
RNE's Y-up metric simulation frame. The colour renderer applies that same
manifest transform, and the ground plus a 3 cm collision-only pickup support
share the measured rug/floor frame, so the scan is an executable room rather
than a viewer backdrop. The collision support is never rendered: the robot
picks directly at rug level without inventing furniture absent from the capture.
The committed derivative keeps every tenth upstream position, DC colour,
opacity, scale, and rotation record byte-for-byte. The UAV view uses official
PLATEAU Sanjo City geometry, a controlled visible airframe, and synchronized
onboard RGB-D rather than a free-flying render camera. Tsukuba combines the full-run scenario
with the PLATEAU test fixture. Factory uses Unitree G1 meshes under its bundled
BSD-3-Clause notice. Office uses a repository-authored scene and synchronized
render overlays. Exact source, conversion, hashes, and licenses are recorded in
the manifest and metadata.

Indoor calibration uses the optional `rotation_xyzw = [x, y, z, w]` splat-
manifest field. The loader rejects non-finite or zero quaternions, normalizes a
valid value, and does not allow it together with a non-zero `rotation_y_rad`.

## Regeneration

Run the renderer-independent gates first:

```bash
cargo run --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --smoke
cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment all
cargo run --locked -p plateau_drone_gif --example 46_plateau_drone_gif -- --smoke
```

Then regenerate on a machine with wgpu and ffmpeg:

```bash
cargo run --release --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --capture
cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment all
cargo run --release --locked -p plateau_drone_gif --example 46_plateau_drone_gif
python tools/prepare_showcase_uav.py
cargo run -p xtask -- showcase-media-check
```

GPU capture remains opt-in so simulation CI stays renderer-independent. The
committed metadata records sampled simulation steps, phase labels, replay
digests, camera parameters, render hashes, and encoding settings needed to
audit each artifact.

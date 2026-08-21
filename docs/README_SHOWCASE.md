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
truth for the House hero and the 2 x 2 environment grid.

| Showcase | GIF / poster | GIF bytes | poster bytes | poster size |
| --- | --- | ---: | ---: | ---: |
| House 3DGS mobile manipulation | `house-mobile-manipulation.gif` / `.png` | 1,812,372 | 319,602 | 960 x 540 |
| Tsukuba Challenge | `showcase-tsukuba.gif` / `.png` | 2,262,164 | 18,815 | 960 x 540 |
| Factory inspection | `showcase-factory.gif` / `.png` | 1,830,619 | 52,228 | 960 x 540 |
| Office AGV delivery | `showcase-office.gif` / `.png` | 1,935,863 | 18,240 | 960 x 540 |
| Real-world 3DGS Sakura | `showcase-real-3dgs.gif` / `.png` | 3,983,559 | 363,639 | 960 x 540 |

The current GIF total is **11,824,577 bytes**, below the 12,000,000-byte
combined ceiling. Regeneration must update the manifest's exact sizes and
hashes in the same change.

## Task gates

| Showcase | Required simulation evidence |
| --- | --- |
| House 3DGS mobile manipulation | Real friction grasp; terminated without truncation; lift clearance at least 0.20 m; payload transport at least 2.0 m; placement error at most 0.10 m; all ten authored PBR links synchronized with zero recorded transform error. |
| Tsukuba Challenge | Three stop lines and three signal waits complete; no roadway entry or unstopped overshoot; headless and capture replay digests match. |
| Factory inspection | Official G1 articulation completes all three markers upright; at least 20 mesh items are rendered; replay digest matches. |
| Office AGV delivery | Yield, dock pickup, desk delivery, and desk placement complete; no contact, corridor exit, or early drop; replay digest matches. |
| Real-world 3DGS Sakura | The committed non-stand-in PLY has 59,045 byte-for-byte copied records from the pinned CC0 Scaniverse source; no synthetic splats are added; the fixed 48-step camera/depth path replays to the same digest. |

The House background is the repository-authored procedural 3D Gaussian splat
fixture. Tsukuba combines the full-run scenario with the PLATEAU test fixture.
Factory uses Unitree G1 meshes under their bundled BSD-3-Clause notice. Office
uses a repository-authored scene and synchronized render overlays. The Sakura
entry is real iPad Pro + Scaniverse data released CC0 by WakuFactory, reduced by
copying every fourth Gaussian record byte-for-byte. Exact source, conversion,
hash, and license paths are recorded in the manifest and metadata.

## Regeneration

Run the renderer-independent gates first:

```bash
cargo run --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --smoke
cargo run --locked -p showcase_captures --example 90_showcase_captures -- --smoke --environment all
cargo run --locked -p real_scan_3dgs_showcase --example 91_real_scan_3dgs_showcase -- --smoke
```

Then regenerate on a machine with wgpu and ffmpeg:

```bash
cargo run --release --locked -p house_mobile_lift_hero --example 89_house_mobile_lift_hero -- --capture
cargo run --release --locked -p showcase_captures --example 90_showcase_captures -- --capture --environment all
cargo run --release --locked -p real_scan_3dgs_showcase --example 91_real_scan_3dgs_showcase -- --capture
cargo run -p xtask -- showcase-media-check
```

GPU capture remains opt-in so simulation CI stays renderer-independent. The
committed metadata records sampled simulation steps, phase labels, replay
digests, camera parameters, render hashes, and encoding settings needed to
audit each artifact.

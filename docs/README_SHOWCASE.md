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
| Real indoor 3DGS mobile manipulation | `house-mobile-manipulation.gif` / `.png` | 1,158,109 | 737,087 | 960 x 540 |
| OpenArm v2 bimanual control | `showcase-openarm.gif` / `.png` | 1,983,788 | 56,163 | 960 x 540 |
| Factory inspection | `showcase-factory.gif` / `.png` | 2,638,709 | 88,762 | 960 x 540 |
| Office AGV delivery | `showcase-office.gif` / `.png` | 1,078,053 | 96,581 | 960 x 540 |
| PLATEAU UAV RGB-D flight | `showcase-uav.gif` / `.png` | 4,329,461 | 439,474 | 960 x 540 |

The current GIF total is **11,483,570 bytes**, below the 12,000,000-byte
combined ceiling. `showcase-media-check` verifies the exact total; regeneration
must update the manifest's sizes and hashes in the same change.

## Task gates

| Showcase | Required simulation evidence |
| --- | --- |
| Real indoor 3DGS mobile manipulation | Both finger pads closed on the block (each face within 5 mm, level with its centre) before it is held, with the friction assist off; while held, the pads stay within 1 cm of its faces and level with its centre, measured every step; terminated without truncation; lift clearance at least 0.20 m; payload transport at least 1.5 m; placement error at most 0.10 m; all ten authored PBR links synchronized with zero recorded transform error; no synthetic room furniture is rendered; rendered wrist RGB-D performs known-robot self masking, payload segmentation, depth back-projection, and analytic-IK correction without payload-truth controller inputs; all 45 post-physics samples show the detected reticle; task telemetry and the 2D trace use the same samples. |
| OpenArm v2 bimanual control | Both official seven-axis arms and two-finger grippers expose 18 force-limited actuators and declared URDF inertias; a typed 18-axis sensor publishes every control step with exactly one-period latency; delayed state feedback drives explicit PD effort targets over 19 physics substeps; every commanded keypose is an inverse-kinematics solution against the real URDF chain, and between keyposes each gripper moves in a straight line solved by IK every control step with the spare degree of freedom pulled toward a hanging posture; the right gripper closes on a real dynamic block only once two distinct fingertip contacts gate the grasp, then the left gripper receives the same contact-gated handoff and places the block on a marked pad, releasing it back to ordinary dynamics; each end effector travels at least 0.16 m; each gripper changes aperture by at least 0.015 m; proximal final tracking error is at most 0.13 rad; at least 45 official visual mesh parts resolve; headless and capture replay digests match. |
| Factory inspection | The belt stops each of three dynamic parts within 3 cm of the inspection point; the G1's right fingertips (a contact box over the hand mesh's fingertip vertices, on the forearm) meet each part within 2 cm of its top-face centre and 5 mm of its height, with contact reported for at least 80% of the hold and the part moved under 5 mm; no part leaves the belt; the G1 drifts under 5 cm and tilts under 8°; at least 20 mesh items are rendered; replay digest matches. |
| Office AGV delivery | Yield, dock pickup, desk delivery, and desk placement complete; no contact, corridor exit, or early drop; replay digest matches. |
| PLATEAU UAV RGB-D flight | Visible `MultirotorFlight` entity travels at least 60 m; RMS position error at most 1.0 m; altitude error at most 0.6 m; building clearance at least 2.0 m; zero collisions; onboard RGB-D and replay hashes are deterministic. |

The indoor hero uses the photo-derived Voxel51/Graphdeco Dr Johnson capture
under Apache-2.0. Its published COLMAP cameras establish the transform into
RNE's Y-up metric simulation frame. The colour renderer applies that same
manifest transform, and the ground plus a 3 cm collision-only pickup support
share the measured rug/floor frame, so the scan is an executable room rather
than a viewer backdrop. The collision support is never rendered: the robot
picks directly at rug level without inventing furniture absent from the capture.
The wrist RGB-D inset is rendered from each sampled camera pose against that
same 3DGS and robot foreground. Its target reticle comes from RGB-D payload
detection rather than truth projection. During final pickup alignment, the
controller removes the known robot foreground, back-projects detected depth,
and feeds the perceived target to analytic IK; truth is retained only for the
recorded perception error. Task phase, grasp/transport telemetry, and top-down
trace all derive from the same 45 post-physics rollout samples.
The committed derivative keeps every tenth upstream position, DC colour,
opacity, scale, and rotation record byte-for-byte. The UAV view uses official
PLATEAU Sanjo City geometry, a controlled visible airframe, and synchronized
onboard RGB-D rather than a free-flying render camera. OpenArm uses the official
v2.0 description and a pinned Apache-2.0 model derivative, split into two
reduced-coordinate articulations and reassembled at the original pedestal mount
transforms. Its visible control panel is synchronized from the same post-step
typed-feedback age, tracking error, effort utilization, and saturation state
used by the controller. Every commanded keypose is solved offline against the
real URDF chain, so the trajectory actually reaches the block and the pad
instead of an arbitrary joint-space sweep. The carried block is a real
dynamic Rapier body: it switches to a kinematic pose-follower only once two
distinct fingertip sensors report contact, rides parented to whichever
gripper is holding it, and is handed back to ordinary dynamics on release so
it drops and settles under gravity. The workbench frame, floor, partitions,
and props are render-only lab dressing rebuilt each frame from fixed world
constants. Factory uses Unitree G1 meshes under its bundled
BSD-3-Clause notice. Office uses a repository-authored scene and synchronized
render overlays. Exact source, conversion, hashes, and licenses are recorded in
the manifest and metadata.

Indoor calibration uses the optional `rotation_xyzw = [x, y, z, w]` splat-
manifest field. The loader rejects non-finite or zero quaternions, normalizes a
valid value, and does not allow it together with a non-zero `rotation_y_rad`.

## Indoor hero validation evidence

A fail-closed fixture binds two real frames, COLMAP cameras, six registered
landmarks, the floor plane, the pickup collision proxy, a same-camera
real-versus-RNE RGB observation, and deterministic single-/multi-view depth
evidence; the proxy projects onto the captured rug instead of arbitrary room
space. The fixture passes 7/8 geometric-sensor contracts. Its RGB observation
records 13.05 dB raw PSNR, 0.927 luminance correlation, and 0.688 gradient
correlation; alpha-composited source-unit depth matches 6/6 semantic
landmarks at 0.148 mean absolute error, and 40/42 two-camera tracks at
0.0353 depth-delta MAE with 0/80 false occlusions. It remains explicitly
non-qualifying -- and does not call reconstruction-unit depths metres --
until an independent physical scale anchor is retained.

- [Validation fixture](../assets/environments/voxel51_drjohnson_3dgs/drjohnson.validation.json)
- [Multi-view depth evidence](../assets/environments/voxel51_drjohnson_3dgs/IMG_6292-IMG_6293.multiview-depth.json)

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

# README showcase acceptance contract

The README animations are product evidence, not render-only mockups. Every
committed animation must be generated from the same deterministic simulation
path exercised by its headless smoke gate. Rendering may add cameras, lighting,
trails, labels, and a reduced-motion poster, but it must not replace or retime
the simulated robot state.

## Common media contract

- Simulation advances at a fixed 60 Hz from an explicit seed and never reads
  wall-clock time.
- A GPU-free smoke path validates the complete motion before the GPU capture
  starts.
- Externally visible actors, actions, and metric records use canonical order.
- Every animation has a 960 x 540 or larger poster, a regeneration command,
  and a stable outcome digest or exact replay comparison.
- A README GIF targets at most 5 MB. The five showcase GIFs together target at
  most 20 MB so the repository front page remains usable on mobile networks.
- Camera motion is smooth, keeps the task subject visible, and shows enough
  environment context to make translation and rotation unambiguous.

## Catalog and tracked bytes

[`docs/media/showcase.toml`](media/showcase.toml) is the catalog for the five
front-page GIF/poster pairs. It records the exact README locations, current
byte sizes and SHA-256 values, poster dimensions, and the smoke/capture command
for each entry. The checked-in snapshot is:

| Showcase | GIF / poster | GIF bytes | poster bytes | poster size | README GIF line |
| --- | --- | ---: | ---: | ---: | ---: |
| Mobile manipulation | `rne-hero.gif` / `rne-hero.png` | 3,400,413 | 132,026 | 960 x 540 | 25 |
| G1 biped locomotion | `unitree-g1-learned-stride.gif` / `.png` | 1,800,144 | 49,092 | 960 x 540 | 33 |
| Go2 quadruped locomotion | `go2-torque-turn.gif` / `.png` | 3,813,018 | 77,052 | 960 x 540 | 43 |
| Urban vehicle | `plateau-car.gif` / `.png` | 3,989,457 | 377,902 | 1280 x 720 | 51 (also 203) |
| Urban UAV | `plateau-uav.gif` / `.png` | 4,439,169 | 488,139 | 1280 x 720 | 61 |

The current GIF total is 17,442,201 bytes (under the 20,000,000-byte combined
ceiling); every poster is at least 960 x 540. Regeneration must update the
catalog's observed sizes and hashes in the same change. The PLATEAU vehicle and
UAV entries intentionally share one capture command because they are produced
from the same city run.

## Task gates

| Showcase | Required simulation evidence |
| --- | --- |
| Mobile manipulation | At least 0.9 m payload transport, at least 12 grasped steps, release after grasp, final placement error at most 0.20 m, end-effector error at most 0.05 m, and no non-planar base motion. |
| G1 biped locomotion | Dynamic official 23-DoF model, no fall, at least 0.12 m commanded-window progress, bounded height/tilt/torque, and exact replay. The caption must state that sustained integrated heading hold remains outside the current envelope (v0.2.1 pins 8 s mean yaw-rate sign only). |
| Go2 quadruped locomotion | Dynamic official 12-DoF model, all-joint torque walking, two late transport windows, bounded height/tilt/torque, and exact replay. Steering evidence must preserve forward transport rather than pivoting or stalling. |
| Urban vehicle | The shared PLATEAU city runs 100 actors for 600 or more 60 Hz steps with zero collisions, signal violations, ownership errors, and double integration. The featured vehicle must remain within 2.0 m of its route. |
| Urban UAV | A rendered quadrotor follows the shared PLATEAU route through bounded acceleration, speed, yaw rate, and tilt; RMS position error is at most 1.0 m, maximum altitude error is at most 0.6 m, minimum building clearance is positive, no collision occurs, and replay digests match. |

## Shared urban presentation

The vehicle and UAV captures use one city definition and one coordinate frame.
The scene combines official PLATEAU geometry with road pavement, sidewalks,
curbs, lane and crossing markings, signals, streetlights, guardrails, varied
facades and roofs, vegetation, and a populated deterministic traffic layer.
The vehicle capture uses a road-level tracking camera with onboard RGB-D
insets; the UAV capture shows the aircraft body and rotors from a chase camera
and a gimballed nadir-forward RGB-D inset mounted on the airframe.

## Regeneration

Run the GPU-free gates first:

```bash
cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero -- --smoke
cargo run -p g1_stride_gif --example 63_g1_stride_gif -- --smoke
cargo run -p go2_turn_gif --example 60_go2_turn_gif -- --smoke
cargo run -p plateau_drone_gif --example 46_plateau_drone_gif -- --smoke
cargo run -p xtask -- scenario-scale
```

Then regenerate the committed captures on a machine with wgpu and ffmpeg:

```bash
cargo run -p lift_pick_place_hero --example 32_lift_pick_place_hero
cargo run -p g1_stride_gif --example 63_g1_stride_gif
cargo run -p go2_turn_gif --example 60_go2_turn_gif
cargo run -p plateau_drone_gif --example 46_plateau_drone_gif
cargo run -p xtask -- hero-media-check
```

The PLATEAU command produces both the vehicle and UAV media from the same
imported city. `RNE_RENDER_FRAME_COUNT=1` plus an `RNE_MEDIA_DIR` outside
`docs/media` is the canonical fast visual preview. Headless smoke commands
remain part of `xtask ci-smoke`; GPU capture is intentionally opt-in so
simulation CI stays renderer independent.

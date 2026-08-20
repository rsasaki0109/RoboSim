# G1 workbench mission

This slice is the geometric analog of
[Grove-G1](https://github.com/Adyansh04/grove-g1): drive to a workbench, get
the object into the arm window, pick, and place. It is **not** a ROS 2, Nav2,
or MoveIt port.

## What Grove does, and what RNE scores

Grove parks Nav2 within **0.5 m**, then closes the last half-metre because the
arm window is about **0.2 m**. Manipulation on Grove can pin the pelvis
(`pin_pelvis`). RNE does the same split:

1. Dynamic factory G1 walks until it is inside 0.5 m of `inspection_parts_check`.
2. The existing Dex3 workcell (fixed pelvis) picks and places the cube.

DDS, SLAM Toolbox, and `rt/arm_sdk` stay out of core. A later adapter can speak
those if a hardware G1 is in the loop.

## Run

```bash
cargo test -p rne_ai workbench_mission
cargo run --locked -p g1_workbench_mission --example 77_g1_workbench_mission -- --smoke
```

`--smoke` requires a clean mission and a skip-approach run that fails
`park_within_0_5_m`.

## Not in this slice

- Whole-body loco-manipulation on one unpinned plant
- Learned stride during the approach (inspection gait only, so marker radii hold)
- Object detection; the Dex3 cube pose is scene-authored, as in Grove today
- Vision-language actions

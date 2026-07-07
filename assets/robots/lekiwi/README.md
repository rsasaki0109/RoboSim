# LeKiwi mobile base assets

Vendored from [SIGRobotics-UIUC/LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi)
(`URDF/LeKiwi.urdf` and selected `URDF/meshes/*.stl`).

- **License:** Apache-2.0 (see `LICENSE`)
- **URDF:** `lekiwi_base.urdf` — reduced base-only model (no SO-ARM100 arm, cameras, or standoff-only cosmetics beyond the structural stack)
- **Meshes:** 22 STL files (~5.0 MB). The three 4" omni-wheel bodies (~15 MB each upstream) are replaced with `<cylinder>` primitives (`r=0.0508 m`, `length=0.025 m`) sized from the LeKiwi BOM (101.6 mm diameter wheels).
- **Orientation:** upstream Z-up CAD; scene spawn applies `initial_rotation_rpy = [-π/2, 0, 0]` on `base_link` (renamed from `base_plate_layer1-v5`).
- **Arm mount:** `arm_mount` link at the upstream `Base_08q-v1` attachment pose on `base_plate_layer2-v3` for bolting on a vendored SO-101 later.

Used by the LeRobot LeKiwi mobile-manipulator tutorials.

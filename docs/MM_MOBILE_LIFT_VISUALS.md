# `mm_mobile_lift` visual contract and authored PBR pack

The active manifest is
[`assets/robots/mm_mobile_lift/mm_mobile_lift.visual.toml`](../assets/robots/mm_mobile_lift/mm_mobile_lift.visual.toml).
It maps all ten URDF links to link-frame GLBs, with a detailed LOD0 for the
README hero and a lower-cost LOD1 for interactive/runtime use:

| Link group | Authored detail |
| --- | --- |
| Base and wheels | Bevelled chassis, lift rails, status light, tire tread, rim and hub rings |
| Carriage and arm | Rounded covers, actuator gears, bearing collars, cable channels, and fasteners |
| Wrist and gripper | Joint rings, electronics housing, cyan status lamp, knuckles, friction ribs |

Every generated GLB has multiple material-homogeneous parts and embedded
metallic-roughness PBR maps (base color, normal, metallic-roughness, emissive,
and occlusion). The geometry is authored in the manifest's
`rne_y_up_x_forward` frame and includes the original URDF visual offsets in
each link mesh. The URDF therefore attaches each mesh at identity scale and
the post-physics link transform remains the sole source of motion; a fixed or
prismatic joint never receives a render-only corrective transform.

Regenerate the pack from the repository root with:

```text
python tools/generate_mm_mobile_lift_visuals.py
python tools/generate_mm_mobile_lift_visuals.py --check
```

The generator is standard-library-only and deterministic. The validator
(`cargo run -p xtask -- showcase-media-check` plus the `rne_assets` tests)
loads every LOD and enforces the path, scale, material, texture, and triangle
budgets. See [`PROVENANCE.md`](../assets/robots/mm_mobile_lift/PROVENANCE.md)
for license, source, SHA-256, and the visual/physics boundary.

The contract does not make rendering required for headless simulation.
Collision geometry, joints, limits, and inertial values remain owned by the
physics URDF and are intentionally unchanged by the visual replacement.

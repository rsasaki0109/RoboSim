# `mm_mobile_lift` visual contract and authored PBR pack

The active manifest is
[`assets/robots/mm_mobile_lift/mm_mobile_lift.visual.toml`](../assets/robots/mm_mobile_lift/mm_mobile_lift.visual.toml).
It maps all ten URDF links to link-frame GLBs, with a detailed LOD0 for the
README hero and a lower-cost LOD1 for interactive/runtime use:

| Link group | Authored detail |
| --- | --- |
| Base and wheels | U-shaped chassis (rear body and two side cheeks with a full-height slot between them), white top covers, bumpers, caster wheels, lidar, status strip; tire tread, rim and hub rings |
| Mast and carriage | 70 mm T-slot aluminium mast standing in the chassis slot with a top cap and cable duct; a carriage that wraps the mast on guide rollers and cantilevers forward to the shoulder drive |
| SCARA arm | Inner link between shoulder and elbow drives, outer link stacked above it at the elbow, wrist drive and spline quill, white covers, fasteners and cable |
| Wrist and gripper | Rotary wrist actuator, parallel-jaw body on finger rails, wrist RGB-D camera on a bracket at the camera frame, aluminium finger blades with ribbed rubber pads |

The lift joint sits 0.15 m ahead of the base origin, and for a floor pick the
carriage travels below the top of the chassis. The mast and the chassis slot
are placed so the carriage rides the mast through that travel rather than
passing through the shell. The outer arm link is drawn 0.055 m above the
inner one, as a SCARA stacks them; the joints and the tool frame are the
URDF's.

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

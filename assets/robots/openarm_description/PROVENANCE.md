# OpenArm v2.0 model provenance

This directory vendors the generated OpenArm v2.0 bimanual URDF and the
corresponding official collision meshes from
[`enactic/openarm_description`](https://github.com/enactic/openarm_description).

- Upstream repository: `https://github.com/enactic/openarm_description`
- Upstream commit: `1fba2cbc05001f05b4514120b70130b4ac06f409`
- Retrieved: 2026-08-23
- Upstream license: Apache-2.0 (`LICENSE.openarm_description`)
- Source URDF: `assets/robot/openarm_v2.0/urdf/example/v2.urdf`

RNE's checked-in derivative removes the ROS 2 control-only XML blocks and
splits the common-pedestal bimanual tree into left and right reduced-coordinate
articulations. The fixed pedestal is reassembled by the RNE scene at the exact
upstream mount transforms, avoiding a backend-specific branched-root
assumption while preserving the visible bimanual system. It uses
the official simplified STL collision geometry and converted GLB copies of the
official multi-part Collada arm and gripper visuals for native rendering. The
current force-limited joint-control showcase retains the collision meshes for
audit but disables arm mesh colliders; it does not claim grasp/contact evidence.
Joint topology, origins, axes, limits, inertials, mesh scale, and source visual
geometry are otherwise retained from the generated upstream model.
`openarm_v2.rne.urdf` retains the complete transformed source for audit;
the two `openarm_v2_{left,right}.rne.urdf` runtime derivatives contain one
arm subtree each. The large pedestal visual alone uses its official simplified STL to
keep the repository and README capture bounded.

The deterministic conversion was performed with Python 3 and trimesh 4.12.2:

```python
scene = trimesh.load(source_dae, force="scene")
target_glb.write_bytes(scene.export(file_type="glb"))
```

The conversion preserves each source scene's geometry parts, transforms, and
material colours; the committed showcase checker and metadata bind the final
rendered bytes.

The vendored mesh set is intentionally bounded to the eleven collision STL and
ten rendered GLB files needed by this bimanual configuration. Both arms reuse
the same official arm and pinch-gripper geometry; left-hand mirroring remains
encoded by the URDF scale.

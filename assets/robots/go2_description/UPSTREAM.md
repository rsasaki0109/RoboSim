# Unitree Go2 model provenance

- Source: `unitreerobotics/unitree_ros`
- Commit: `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`
- Upstream path: `robots/go2_description`
- License: BSD-3-Clause; see `LICENSE.unitree_ros`

The URDF and seven DAE visual meshes are vendored unchanged. RNE applies the
Z-up to Y-up conversion through `unitree_go2.rne.robot.toml`; core URDF types
are not modified to fit Unitree.

`scripts/convert_unitree_collada.py` generates the derived binary STL meshes
and `go2_description.rne.urdf` consumed by RNE's current mesh loader.

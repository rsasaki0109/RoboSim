# Unitree G1 model provenance

- Source: `unitreerobotics/unitree_ros`
- Commit: `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`
- Upstream path: `robots/g1_description/g1_23dof.urdf`
- License: BSD-3-Clause; see `LICENSE.unitree_ros`

The 23-DoF URDF and the 29 STL files it references are vendored unchanged.
RNE applies the Z-up to Y-up conversion in `unitree_g1.rne.robot.toml`.
The fixed-base media scene disables collision; the dynamic standing scene keeps
primitive contacts while excluding mesh-AABB approximations through asset settings.

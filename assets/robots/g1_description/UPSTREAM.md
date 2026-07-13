# Unitree G1 model provenance

- Source: `unitreerobotics/unitree_ros`
- Commit: `d96d8f63ae17a7108d4f7229c00ef875ba7129c9`
- Upstream paths:
  - `robots/g1_description/g1_23dof.urdf`
  - `robots/g1_description/g1_29dof_with_hand.urdf`
- License: BSD-3-Clause; see `LICENSE.unitree_ros`

The 23-DoF and 29-DoF-with-Dex3 URDFs and the STL files they reference are
vendored unchanged. Shared meshes are stored once; the integrated model adds
the waist/wrist links and seven actuated Dex3 joints per hand.
RNE applies the Z-up to Y-up conversion in its G1 robot asset descriptors. The
29-DoF + Dex3 task keeps the upstream model unchanged, disables mesh-derived
collision approximations, and adds backend-neutral fingertip sensor bodies in
the Episode for deterministic two-sided contact gating.

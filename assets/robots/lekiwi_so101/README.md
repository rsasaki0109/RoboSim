# LeKiwi + SO-101 composite

Mobile manipulator built from vendored `lekiwi_base.urdf` and `so101.urdf`.

- **Mount:** `so101_mount_joint` rigidly attaches `so101_base_link` to `arm_mount` using the upstream LeKiwi `Base_08q` → first-servo offset (`xyz="-0.02975 -0.04565 0.0278"`, `rpy="π/2 0 0"`).
- **Collisions:** SO-101 mesh collision elements are omitted (visuals kept) to avoid mesh-AABB overlap with the base deck at spawn.
- **Meshes:** `../lekiwi/` and `../so101/` relative paths resolve from this directory.
- **Regenerate:** `python scripts/gen_lekiwi_so101_urdf.py`

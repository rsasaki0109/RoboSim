# External URDF ingestion

RNE imports a **minimal URDF subset** for real-world robot models (not only hand-written fixtures). The importer lives in `rne_urdf_import`; scene loading uses `rne_assets` URDF robot assets.

## Supported geometry

| URDF element | Visual | Collision / physics |
|--------------|--------|---------------------|
| `box` | yes | cuboid collider |
| `sphere` | yes | sphere collider |
| `cylinder` | yes | Y-axis capsule collider |
| `mesh` (STL) | yes (`package://` or URDF-relative) | **AABB cuboid fallback** when `mesh_assets_root` is set |

Mesh collision does not add a mesh collider shape to `rne_physics`. At import time the STL is loaded, scaled, and replaced with a `ColliderShape::Cuboid` centered on the mesh axis-aligned bounding box (center folded into `Collider.local_offset`).

When `UrdfSpawnConfig.mesh_assets_root` is `None`, mesh `<collision>` elements are skipped (legacy behavior).

## Supported joints

| Type | Parsed | ECS `Joint` | Rapier articulation |
|------|--------|---------------|---------------------|
| `fixed` | yes | `Fixed` | `FixedJointDesc` weld |
| `revolute` | yes | `Revolute` | `RevoluteJointDesc` + motor; optional angle limits |
| `continuous` | yes | `Continuous` | revolute motor (no limits) |
| `prismatic` | yes | `Prismatic` | `PrismaticJointDesc` + motor (linear limits parsed only) |

### `<limit lower upper velocity effort>`

Stored on `UrdfJoint.limit` as `UrdfJointLimit` (`lower`, `upper`, `max_velocity_rad_s`, `max_effort_nm`). Units follow URDF convention: radians / rad/s / N·m for revolute joints; meters / m/s / N for prismatic.

Revolute limits are copied into:

- `rne_robot::Joint.limits`
- `rne_physics::RevoluteJointDesc::{lower_rad, upper_rad}` → Rapier joint limits when both are present

Continuous joints ignore limits.

### `<mimic joint multiplier offset>`

Parsed into `UrdfJoint.mimic` (`UrdfJointMimic`). **Not simulated** — mimic couplings are metadata only; actuators must drive leader joints explicitly.

## Multi-visual links

External URDFs (e.g. SO-101) often declare several `<visual>` elements per link. The importer attaches:

- one `rne_render::Visual` when a link has a single visual, or
- `rne_render::LinkVisuals` (a `Vec<Visual>`) when there are multiple.

`build_visual_render_scene` emits one render item per visual.

## Asset layout

Vendored robots live under `assets/robots/<name>/`:

```
assets/robots/so101/
  so101.urdf
  meshes/*.stl
  LICENSE
  README.md
```

Mesh URIs use `package://so101/meshes/<file>.stl` or plain paths relative to the URDF directory (`meshes/foo.stl`). The package root is the URDF parent directory (`assets/robots/<name>/`), set automatically via `UrdfSpawnConfig.mesh_assets_root` when spawning from `.rne.robot.toml`.

URDF robot assets may set `initial_rotation_rpy` (roll-pitch-yaw radians) on the base link; LeKiwi uses `[-π/2, 0, 0]` to rest the upstream Z-up CAD model on the Y-up ground plane.

## Vendored models

| Asset | Source | License | Notes |
|-------|--------|---------|-------|
| `so101` | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | Apache-2.0 | ~15 MB STL set from `Simulation/SO101/assets` |
| `cart_minimal` | RNE-authored | project license | Primitive diff-drive cart (continuous wheel joints) |
| `lekiwi` | [SIGRobotics-UIUC/LeKiwi](https://github.com/SIGRobotics-UIUC/LeKiwi) | Apache-2.0 | Reduced base-only URDF (~5 MB / 22 STLs); omni wheel bodies replaced with cylinders |

### LeKiwi reduction strategy

Upstream `LeKiwi.urdf` is a 45-link / 44-joint assembly (~61 MB meshes). `lekiwi_base.urdf` keeps the two-layer base plates, battery stack, three STS3215 drive servos, omni mounts, and an `arm_mount` frame at the SO-ARM100 attachment pose. Arm links (`SO_ARM100_*`, `STS3215_03a*`, wrist, cameras, standoffs-only cosmetics) are dropped. The three ~15 MB omni-wheel STL bodies are substituted with `<cylinder radius="0.0508" length="0.025">` (4" wheel per BOM).

### Kiwi-drive kinematics

`UrdfSceneSim::step_kiwi` maps a planar body twist `(vx_m_s, vz_m_s, wz_rad_s)` to the three continuous drive joints using per-wheel geometry derived from upstream mount origins on `base_plate_layer1-v5`, transformed into the RNE XZ ground plane after `initial_rotation_rpy = [-π/2, 0, 0]`:

```
ωᵢ = (-sin(θᵢ)·vx + cos(θᵢ)·vz + Rᵢ·ωz) / r
```

| Constant | Value | Derivation |
|----------|-------|------------|
| `r` (`LEKIWI_WHEEL_RADIUS_M`) | 0.0508 m | 4" omni wheel diameter (LeKiwi BOM) |
| `θ₀` | 1.768 rad | `atan2(0.10, -0.02)` — mount `drive_motor_mount-v11-2` |
| `θ₁` | −0.281 rad | `atan2(-0.02268, 0.07928)` — mount `drive_motor_mount-v11-1` |
| `θ₂` | −2.347 rad | `atan2(-0.05732, -0.05928)` — mount `drive_motor_mount-v11` |
| `R₀` | 0.102 m | ‖(x, z)‖ of mount −2 |
| `R₁` | 0.082 m | ‖(x, z)‖ of mount −1 |
| `R₂` | 0.083 m | ‖(x, z)‖ of mount base |

Mount URDF offsets `(x, y, 0)` map to world `(x, 0, -y)` under the spawn rotation.

## Examples & viewer

- `cargo run -p urdf_import --example 03_urdf_import` — inline fixture import
- `cargo run -p external_urdf --example 35_external_urdf` — SO-101 + cart + LeKiwi scenes
- `cargo run -p interactive_viewer --example 14_interactive_viewer -- --so101`
- `cargo run -p interactive_viewer --example 14_interactive_viewer -- --cart`
- `cargo run -p interactive_viewer --example 14_interactive_viewer -- --lekiwi`

## Intentionally unsupported (skipped)

- `inertial`, `transmission`, `gazebo`, material references by name only
- Non-STL meshes (DAE, OBJ, PLY)
- Mimic joint physics
- Prismatic motor limits in Rapier (parsed only; see `docs/ROADMAP.md`)

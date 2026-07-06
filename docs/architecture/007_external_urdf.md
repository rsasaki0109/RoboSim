# External URDF ingestion

RNE imports a **minimal URDF subset** for real-world robot models (not only hand-written fixtures). The importer lives in `rne_urdf_import`; scene loading uses `rne_assets` URDF robot assets.

## Supported geometry

| URDF element | Visual | Collision / physics |
|--------------|--------|---------------------|
| `box` | yes | cuboid collider |
| `sphere` | yes | sphere collider |
| `cylinder` | yes | Y-axis capsule collider |
| `mesh` (STL) | yes (`package://` or relative) | **AABB cuboid fallback** when `mesh_assets_root` is set |

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

Mesh URIs use `package://so101/meshes/<file>.stl`. The package root is the URDF parent directory (`assets/robots/so101`), set automatically via `UrdfSpawnConfig.mesh_assets_root` when spawning from `.rne.robot.toml`.

## Vendored models

| Asset | Source | License | Notes |
|-------|--------|---------|-------|
| `so101` | [TheRobotStudio/SO-ARM100](https://github.com/TheRobotStudio/SO-ARM100) | Apache-2.0 | ~15 MB STL set from `Simulation/SO101/assets` |
| `cart_minimal` | RNE-authored | project license | Primitive diff-drive cart (continuous wheel joints) |

## Examples & viewer

- `cargo run -p urdf_import --example 03_urdf_import` — inline fixture import
- `cargo run -p external_urdf --example 35_external_urdf` — SO-101 + cart scenes
- `cargo run -p interactive_viewer --example 14_interactive_viewer -- --so101`
- `cargo run -p interactive_viewer --example 14_interactive_viewer -- --cart`

## Intentionally unsupported (skipped)

- `inertial`, `transmission`, `gazebo`, material references by name only
- Non-STL meshes (DAE, OBJ, PLY)
- Mimic joint physics
- Prismatic motor limits in Rapier (parsed only; see `docs/ROADMAP.md`)

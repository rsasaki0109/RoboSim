# Web viewer boundary

The browser viewer lives in `web/rne_web_viewer/` and is **not** part of the core crate graph. Core simulation, physics, and rendering traits remain platform-agnostic; the web app is a thin host that:

- embeds scene/robot/URDF assets at compile time (`include_str!`)
- spawns ECS entities through `rne_assets` in-memory parsers
- drives a kinematic joint sweep from a frame counter (visual only)
- presents frames via `rne_render_wgpu` on a canvas (WebGPU / WebGL2)

## Allowed dependencies

The web crate may depend on core crates (`rne_math`, `rne_ecs`, `rne_world`, `rne_robot`, `rne_physics` traits, `rne_render`, `rne_render_wgpu`, `rne_assets`, `rne_urdf_import`) and browser tooling (`wasm-bindgen`, `winit`, `wgpu` with `webgl`).

## Forbidden

- No core crate may depend on `web/*`.
- No ROS2 / adapter code in the viewer.
- No wall-clock time inside simulation-oriented logic (the viewer uses frame-count animation only).

## Core touch points

Small, documented helpers were added for embedded assets:

- `rne_assets::parse_scene_bundle_with_sources` — parse scene + robot TOML from memory
- `rne_assets::spawn_scene_bundle` — spawn with optional embedded URDF map and kinematic mode (`wire_articulation: false`)
- `rne_render::load_stl_bytes` — parse STL without `std::fs` (for future mesh scenes)
- `rne_render_wgpu::InteractiveViewer::new_async` — async GPU init required on `wasm32`

See also [Architecture overview](000_overview.md).

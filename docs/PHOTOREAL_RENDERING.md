# Photoreal render foundation

RNE keeps photoreal rendering in the render layer. Simulation and headless
tests do not require a GPU or a renderer.

`rne_render::PbrMaterial` carries linear base color, roughness, metallic, and
emissive values plus optional tangent-space normal and linear roughness maps.
`rne_render_wgpu` uploads those values to its Cook–Torrance directional-light
path. Normal maps use screen-space UV derivatives to build a tangent frame, so
the core mesh format stays backend-neutral and does not require a tangent
buffer. OBJ materials populate `map_Bump` and `map_Ns` when present; `Ns` is
converted to a GGX roughness estimate and the legacy shininess map is inverted
into a linear roughness map. STL and primitive items use flat-normal and
white-roughness fallback maps.

The G1 stride hero uses a render-only UV mesh for the test-bay floor and the
tileable base-color asset at
`examples/63_g1_stride_gif/assets/photoreal_test_bay/concrete_floor_basecolor.png`.
Its corresponding linear maps are
`concrete_floor_normal.png` and `concrete_floor_roughness.png` in the same
directory. The maps are sampled with repeat addressing and are not part of the
the physics scene. Normal/roughness maps are now part of the material path;
HDR environment lighting and temporal anti-aliasing are later rendering
increments.

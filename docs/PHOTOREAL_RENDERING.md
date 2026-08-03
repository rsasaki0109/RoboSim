# Photoreal render foundation

RNE keeps photoreal rendering in the render layer. Simulation and headless
tests do not require a GPU or a renderer.

`rne_render::PbrMaterial` carries linear base color, roughness, metallic, and
emissive values. `rne_render_wgpu` uploads those values to its Cook–Torrance
directional-light path, while mesh base-color textures remain optional. OBJ
diffuse colors and textures populate the same material path; STL and primitive
items use the documented defaults.

The G1 stride hero uses a render-only UV mesh for the test-bay floor and the
tileable base-color asset at
`examples/63_g1_stride_gif/assets/photoreal_test_bay/concrete_floor_basecolor.png`.
The floor is deliberately not part of the physics scene. Normal/roughness
texture maps, HDR environment lighting, and temporal anti-aliasing are later
rendering increments.

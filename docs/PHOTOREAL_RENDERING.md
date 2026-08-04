# Photoreal render foundation

RNE keeps photoreal rendering in the render layer. Simulation and headless
tests do not require a GPU or a renderer.

`rne_render::PbrMaterial` carries linear base color, roughness, metallic, and
emissive values plus optional tangent-space normal and linear roughness maps.
It also carries the glTF packed metallic-roughness, emissive, and occlusion
maps. `rne_render_wgpu` uploads those values to its Cook–Torrance
directional-light path. Normal maps use screen-space UV derivatives to build a
tangent frame, so the core mesh format stays backend-neutral and does not
require a tangent buffer. OBJ materials populate `map_Bump` and `map_Ns` when
present; `Ns` is converted to a GGX roughness estimate and the legacy
shininess map is inverted into a linear roughness map. STL and primitive items
use flat-normal and white-map fallback textures.

## HDR environment lighting

`EnvironmentMap::load` reads a Radiance `.hdr` equirectangular image into a
backend-neutral linear RGB32F map. `EnvironmentLighting` adds explicit map
intensity, diffuse/specular image-based-lighting strengths, and world-Y
rotation. The WGPU backend samples the map for the sky background, diffuse
ambient response, and view-dependent specular response while retaining the
existing directional shadow path. Environment textures are cached by immutable
map identity, so repeated frames do not re-upload the HDR pixels.

Applications opt in without changing simulation or headless APIs:

```rust
let map = Arc::new(EnvironmentMap::load("studio.hdr")?);
backend.set_environment(EnvironmentLighting::from_map(map));
```

The G1 photoreal capture accepts the same path through `RNE_HDRI_PATH` and
optional `RNE_HDRI_INTENSITY` / `RNE_HDRI_ROTATION_RAD` variables. No HDRI is
vendored in the repository; applications remain responsible for the source
map's license and attribution.

## Temporal anti-aliasing

`rne_render_wgpu::TaaSettings` enables a deterministic Halton camera-jitter
sequence, depth-based reprojection, neighborhood history clamping, and a
configurable feedback factor. It is opt-in so existing captures and headless
render paths remain unchanged:

```rust
backend.set_taa(TaaSettings::enabled());
```

The history is discarded when the scene's visible transforms or material
colors change, which prevents moving-robot ghosting. Camera motion is
reprojected through the previous view-projection matrix; static scenes gain
the strongest edge-quality improvement. The G1 capture exposes the same path
with `RNE_TAA=1` and optional `RNE_TAA_FEEDBACK` / `RNE_TAA_JITTER_PX`.

## Prefiltered image-based lighting

When an HDR environment is first uploaded, `rne_render_wgpu` builds a small,
deterministic prefiltered representation. Five GGX/Hammersley specular levels
cover `128x64` through `8x4`, while a `32x16` cosine-weighted diffuse map
captures low-frequency irradiance. The original HDR texture remains available
for the sky, and the primitive shader selects the prefiltered specular level
from material roughness and the diffuse map for ambient response. Environment
map identity caching means the CPU prefilter and GPU uploads happen once per
immutable map instance.

The prefilter is generated on demand and does not alter the backend-neutral
`EnvironmentMap` API. Applications still only need to call
`backend.set_environment(...)`; no extra asset or configuration is required.

## glTF/GLB asset path

`rne_render::load_mesh_parts` and `RenderScene::resolve_mesh_assets` accept
static `.gltf` and `.glb` assets in addition to STL and OBJ. The importer walks
the selected glTF scene in deterministic node order, bakes node transforms into
the CPU mesh, preserves one render part per material primitive, and keeps
material textures in the backend-neutral `ImageFrame` representation. GLB
buffers and images are imported through the same path as external glTF
resources.

The v0.3-A material mapping follows the glTF metallic-roughness convention:

| glTF input | RNE material field | GPU treatment |
|------------|--------------------|---------------|
| `baseColorFactor` / texture | `base_color_rgba` / scene base-color texture | factor is linear; texture uses sRGB format |
| `metallicFactor`, `roughnessFactor` | `metallic`, `roughness` | clamped; optional packed map uses B/G channels |
| `normalTexture.scale` | `normal_strength` | tangent-space map, linear format |
| `occlusionTexture.strength` | `occlusion_strength` | R channel, linear format |
| `emissiveFactor` / texture | `emissive_rgb` / `emissive_texture` | factor is linear; texture uses sRGB format |

`load_mesh_parts` keeps the static bind-pose path compatible with existing
callers. Applications that need humanoid motion can use `load_gltf_scene` to
preserve the node hierarchy, inverse-bind matrices, `JOINTS_0`/`WEIGHTS_0`,
and linear or step TRS animation clips. `GltfSceneAsset::sample_part` applies
the selected clip and returns a fresh CPU-deformed `TriangleMesh`, while
`sample_part_for_gpu` keeps bind-pose vertices and returns immutable joint
matrices and weights for the WGPU backend. `GltfAnimationPlayer` advances from
simulation deltas and exposes the same GPU sampling path; dynamic mesh items
therefore update their storage-buffer pose without accumulating vertex
deformation. The WGPU main and shadow passes consume the same `JOINTS_0` /
`WEIGHTS_0` vertex attributes, so animated silhouettes and shadows stay aligned.
Cubic-spline animation, morph targets, and non-triangle primitive modes remain
explicit unsupported cases.

Example 69 exercises this path with the pinned, attributed
`assets/fixtures/rigged_figure/RiggedFigure.glb` humanoid fixture. Run
`cargo run -p gltf_humanoid_gpu --example 69_gltf_humanoid_gpu -- --smoke` for
the GPU-free loader/player check; the default invocation renders two animation
frames to `target/rne-gltf-humanoid`. The asset's source, credit, license, and
SHA-256 are recorded in `assets/fixtures/rigged_figure/ASSET_LICENSE.md`.

Example 69 intentionally vendors the attributed GLB fixture above; no other
third-party scene or texture asset is added. The loader preserves source pixel
data but does not replace an imported asset's
license or attribution requirements; applications should keep the glTF asset's
provenance beside their package and verify redistribution terms before shipping
it. The generated unit-test images are synthetic fixtures.

## Unitree G1 photoreal capture

Example 70 connects the official Unitree G1 URDF/STL visual hierarchy to the
photoreal path without moving robot-specific types into the renderer. It loads
the vendored BSD-3-Clause model recorded in
`assets/robots/g1_description/UPSTREAM.md`, settles the same dynamic scene used
by the headless G1 gait examples, resolves all 29 visual mesh parts through
`MeshRenderCache`, and adds a render-only calibration room. The room reuses the
mapped concrete floor from example 63, so base color, tangent-space normal, and
linear roughness maps are exercised together with the G1 materials.

Run the deterministic GPU-free check with:

```text
cargo run -p g1_photoreal_capture --example 70_g1_photoreal_capture -- --smoke
```

The default invocation writes twelve PNG frames and an animated GIF below
`target/rne-g1-photoreal/`. `RNE_HDRI_PATH` enables an external Radiance HDR
environment, while `RNE_TAA=1` enables deterministic temporal accumulation;
the optional `RNE_HDRI_INTENSITY`, `RNE_HDRI_ROTATION_RAD`,
`RNE_TAA_FEEDBACK`, and `RNE_TAA_JITTER_PX` variables tune those paths. HDRI
files remain external so their licensing and attribution stay with the
application that supplies them.

The G1 stride hero uses a render-only UV mesh for the test-bay floor and the
tileable base-color asset at
`examples/63_g1_stride_gif/assets/photoreal_test_bay/concrete_floor_basecolor.png`.
Its corresponding linear maps are
`concrete_floor_normal.png` and `concrete_floor_roughness.png` in the same
directory. The maps are sampled with repeat addressing and are not part of the
physics scene. Normal/roughness maps and glTF PBR maps are now part of the
material path. HDR environment lighting, prefiltered image-based lighting, and
temporal anti-aliasing are available through the opt-in WGPU path. The
backend-neutral glTF path supports deterministic CPU skinning, GPU skinning
payload sampling, and a simulation-driven TRS animation player. Morph targets
remain a future increment.

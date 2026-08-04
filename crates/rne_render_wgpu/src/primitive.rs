use crate::environment_filter::{prefilter_environment, SPECULAR_MIP_LEVELS};
use crate::taa::{TaaSettings, TemporalAntiAliasing};

const SHADER: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    light_view_proj: mat4x4<f32>,
    light_ambient: vec4<f32>,
    diffuse_shadow: vec4<f32>,
    camera_position: vec4<f32>,
    environment: vec4<f32>,
}

struct DrawUniform {
    model: mat4x4<f32>,
    normal_col0: vec4<f32>,
    normal_col1: vec4<f32>,
    normal_col2: vec4<f32>,
    color: vec4<f32>,
    base_color: vec4<f32>,
    material_params: vec4<f32>,
    emissive: vec4<f32>,
    map_params: vec4<f32>,
    skinning: vec4<f32>,
}

@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(1) @binding(0) var<uniform> draw: DrawUniform;
@group(2) @binding(0) var base_color_texture: texture_2d<f32>;
@group(2) @binding(1) var base_color_sampler: sampler;
@group(3) @binding(0) var shadow_texture: texture_depth_2d;
@group(3) @binding(1) var shadow_sampler: sampler_comparison;
@group(4) @binding(0) var normal_texture: texture_2d<f32>;
@group(4) @binding(1) var normal_sampler: sampler;
@group(5) @binding(0) var roughness_texture: texture_2d<f32>;
@group(5) @binding(1) var roughness_sampler: sampler;
@group(6) @binding(0) var metallic_roughness_texture: texture_2d<f32>;
@group(6) @binding(1) var metallic_roughness_sampler: sampler;
@group(7) @binding(0) var emissive_texture: texture_2d<f32>;
@group(7) @binding(1) var emissive_sampler: sampler;
@group(8) @binding(0) var occlusion_texture: texture_2d<f32>;
@group(8) @binding(1) var occlusion_sampler: sampler;
@group(9) @binding(0) var environment_texture: texture_2d<f32>;
@group(9) @binding(1) var prefiltered_environment_texture: texture_2d<f32>;
@group(9) @binding(2) var diffuse_environment_texture: texture_2d<f32>;

struct SkinStorage {
    mesh_transform: mat4x4<f32>,
    joint_matrices: array<mat4x4<f32>>,
}

@group(10) @binding(0) var<storage, read> skin: SkinStorage;

fn mat3_from_mat4(matrix: mat4x4<f32>) -> mat3x3<f32> {
    return mat3x3<f32>(matrix[0].xyz, matrix[1].xyz, matrix[2].xyz);
}

fn skinned_position(
    position: vec3<f32>,
    joints: vec4<u32>,
    weights: vec4<f32>,
) -> vec4<f32> {
    if (draw.skinning.x < 0.5) {
        return vec4<f32>(position, 1.0);
    }
    let source = vec4<f32>(position, 1.0);
    var result = vec4<f32>(0.0);
    result += skin.joint_matrices[joints.x] * source * weights.x;
    result += skin.joint_matrices[joints.y] * source * weights.y;
    result += skin.joint_matrices[joints.z] * source * weights.z;
    result += skin.joint_matrices[joints.w] * source * weights.w;
    return skin.mesh_transform * result;
}

fn skinned_normal(
    normal: vec3<f32>,
    joints: vec4<u32>,
    weights: vec4<f32>,
) -> vec3<f32> {
    if (draw.skinning.x < 0.5) {
        return normal;
    }
    var result = vec3<f32>(0.0);
    result += mat3_from_mat4(skin.joint_matrices[joints.x]) * normal * weights.x;
    result += mat3_from_mat4(skin.joint_matrices[joints.y]) * normal * weights.y;
    result += mat3_from_mat4(skin.joint_matrices[joints.z]) * normal * weights.z;
    result += mat3_from_mat4(skin.joint_matrices[joints.w]) * normal * weights.w;
    return mat3_from_mat4(skin.mesh_transform) * result;
}

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
    @location(1) world_normal: vec3<f32>,
    @location(2) texcoord: vec2<f32>,
    @location(3) light_clip_position: vec4<f32>,
    @location(4) world_position: vec3<f32>,
}

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
    @location(2) texcoord: vec2<f32>,
    @location(3) joints: vec4<u32>,
    @location(4) weights: vec4<f32>,
) -> VertexOutput {
    var out: VertexOutput;
    let local_position = skinned_position(position, joints, weights);
    let local_normal = skinned_normal(normal, joints, weights);
    let world = draw.model * local_position;
    out.clip_position = camera.view_proj * world;
    out.light_clip_position = camera.light_view_proj * world;
    out.color = draw.color;
    out.world_position = world.xyz;

    let normal_matrix = mat3x3<f32>(
        draw.normal_col0.xyz,
        draw.normal_col1.xyz,
        draw.normal_col2.xyz,
    );
    var world_normal = normal_matrix * local_normal;
    if (dot(world_normal, world_normal) < 1e-8) {
        world_normal = vec3<f32>(0.0, 1.0, 0.0);
    } else {
        world_normal = normalize(world_normal);
    }
    out.world_normal = world_normal;
    out.texcoord = texcoord;
    return out;
}

fn shadow_visibility(input: VertexOutput, ndotl: f32) -> f32 {
    let light_ndc = input.light_clip_position.xyz / input.light_clip_position.w;
    let uv = vec2<f32>(
        light_ndc.x * 0.5 + 0.5,
        0.5 - light_ndc.y * 0.5,
    );
    if (uv.x <= 0.0 || uv.x >= 1.0 || uv.y <= 0.0 || uv.y >= 1.0 ||
        light_ndc.z <= 0.0 || light_ndc.z >= 1.0) {
        return 1.0;
    }
    let dimensions = vec2<f32>(textureDimensions(shadow_texture));
    let texel = 1.0 / dimensions;
    let bias = max(0.00035, 0.0015 * (1.0 - ndotl));
    var visibility = 0.0;
    for (var y = -1; y <= 1; y = y + 1) {
        for (var x = -1; x <= 1; x = x + 1) {
            let offset = vec2<f32>(f32(x), f32(y)) * texel;
            visibility += textureSampleCompare(
                shadow_texture,
                shadow_sampler,
                uv + offset,
                light_ndc.z - bias,
            );
        }
    }
    return visibility / 9.0;
}

fn distribution_ggx(ndoth: f32, roughness: f32) -> f32 {
    let alpha = roughness * roughness;
    let alpha_squared = alpha * alpha;
    let denominator = ndoth * ndoth * (alpha_squared - 1.0) + 1.0;
    return alpha_squared / max(3.14159265 * denominator * denominator, 1e-5);
}

fn geometry_schlick_ggx(ndotv: f32, roughness: f32) -> f32 {
    let k = (roughness + 1.0) * (roughness + 1.0) / 8.0;
    return ndotv / max(ndotv * (1.0 - k) + k, 1e-5);
}

fn geometry_smith(ndotv: f32, ndotl: f32, roughness: f32) -> f32 {
    return geometry_schlick_ggx(ndotv, roughness) *
        geometry_schlick_ggx(ndotl, roughness);
}

fn fresnel_schlick(cos_theta: f32, f0: vec3<f32>) -> vec3<f32> {
    return f0 + (vec3<f32>(1.0) - f0) * pow(1.0 - cos_theta, 5.0);
}

fn reinhard_tonemap(color: vec3<f32>) -> vec3<f32> {
    return color / (vec3<f32>(1.0) + color);
}

fn rotate_environment_direction(direction: vec3<f32>, rotation_rad: f32) -> vec3<f32> {
    let cosine = cos(rotation_rad);
    let sine = sin(rotation_rad);
    return vec3<f32>(
        cosine * direction.x - sine * direction.z,
        direction.y,
        sine * direction.x + cosine * direction.z,
    );
}

fn wrap_environment_index(index: i32, size: i32) -> i32 {
    let wrapped = index % size;
    return select(wrapped, wrapped + size, wrapped < 0);
}

fn sample_prefiltered_level(direction: vec3<f32>, level: i32) -> vec3<f32> {
    let rotated = rotate_environment_direction(
        normalize(direction),
        camera.environment.z,
    );
    let u = atan2(rotated.z, rotated.x) / 6.28318530 + 0.5;
    let v = acos(clamp(rotated.y, -1.0, 1.0)) / 3.14159265;
    let dimensions = textureDimensions(prefiltered_environment_texture, level);
    let width = max(i32(dimensions.x), 1);
    let height = max(i32(dimensions.y), 1);
    let x = u * f32(width) - 0.5;
    let y = v * f32(height) - 0.5;
    let x0 = wrap_environment_index(i32(floor(x)), width);
    let x1 = wrap_environment_index(x0 + 1, width);
    let y0 = clamp(i32(floor(y)), 0, height - 1);
    let y1 = clamp(y0 + 1, 0, height - 1);
    let fx = fract(x);
    let fy = clamp(fract(y), 0.0, 1.0);
    let c00 = textureLoad(prefiltered_environment_texture, vec2<i32>(x0, y0), level).rgb;
    let c10 = textureLoad(prefiltered_environment_texture, vec2<i32>(x1, y0), level).rgb;
    let c01 = textureLoad(prefiltered_environment_texture, vec2<i32>(x0, y1), level).rgb;
    let c11 = textureLoad(prefiltered_environment_texture, vec2<i32>(x1, y1), level).rgb;
    return mix(mix(c00, c10, fx), mix(c01, c11, fx), fy);
}

fn sample_prefiltered_environment(direction: vec3<f32>, lod: f32) -> vec3<f32> {
    let max_level = 4.0;
    let clamped_lod = clamp(lod, 0.0, max_level);
    let level0 = i32(floor(clamped_lod));
    let level1 = min(level0 + 1, 4);
    let blend = fract(clamped_lod);
    return mix(
        sample_prefiltered_level(direction, level0),
        sample_prefiltered_level(direction, level1),
        blend,
    );
}

fn sample_diffuse_environment(direction: vec3<f32>) -> vec3<f32> {
    let rotated = rotate_environment_direction(
        normalize(direction),
        camera.environment.z,
    );
    let u = atan2(rotated.z, rotated.x) / 6.28318530 + 0.5;
    let v = acos(clamp(rotated.y, -1.0, 1.0)) / 3.14159265;
    let dimensions = textureDimensions(diffuse_environment_texture);
    let width = max(i32(dimensions.x), 1);
    let height = max(i32(dimensions.y), 1);
    let x = u * f32(width) - 0.5;
    let y = v * f32(height) - 0.5;
    let x0 = wrap_environment_index(i32(floor(x)), width);
    let x1 = wrap_environment_index(x0 + 1, width);
    let y0 = clamp(i32(floor(y)), 0, height - 1);
    let y1 = clamp(y0 + 1, 0, height - 1);
    let fx = fract(x);
    let fy = clamp(fract(y), 0.0, 1.0);
    let c00 = textureLoad(diffuse_environment_texture, vec2<i32>(x0, y0), 0).rgb;
    let c10 = textureLoad(diffuse_environment_texture, vec2<i32>(x1, y0), 0).rgb;
    let c01 = textureLoad(diffuse_environment_texture, vec2<i32>(x0, y1), 0).rgb;
    let c11 = textureLoad(diffuse_environment_texture, vec2<i32>(x1, y1), 0).rgb;
    return mix(mix(c00, c10, fx), mix(c01, c11, fx), fy);
}

fn fallback_tangent_frame(normal: vec3<f32>) -> mat3x3<f32> {
    var reference = vec3<f32>(0.0, 1.0, 0.0);
    if (abs(normal.y) > 0.9) {
        reference = vec3<f32>(1.0, 0.0, 0.0);
    }
    let tangent = normalize(cross(reference, normal));
    let bitangent = normalize(cross(normal, tangent));
    return mat3x3<f32>(tangent, bitangent, normal);
}

fn tangent_frame(normal: vec3<f32>, world_position: vec3<f32>, uv: vec2<f32>) -> mat3x3<f32> {
    let position_dx = dpdx(world_position);
    let position_dy = dpdy(world_position);
    let uv_dx = dpdx(uv);
    let uv_dy = dpdy(uv);
    let determinant = uv_dx.x * uv_dy.y - uv_dx.y * uv_dy.x;
    if (abs(determinant) < 1e-5) {
        return fallback_tangent_frame(normal);
    }
    let tangent = normalize(
        (position_dx * uv_dy.y - position_dy * uv_dx.y) / determinant,
    );
    let bitangent = normalize(
        (position_dy * uv_dx.x - position_dx * uv_dy.x) / determinant,
    );
    return mat3x3<f32>(tangent, bitangent, normal);
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    let light_dir = normalize(camera.light_ambient.xyz);
    let geometric_normal = normalize(input.world_normal);
    let geometric_ndotl = max(dot(geometric_normal, light_dir), 0.0);
    let visibility = shadow_visibility(input, geometric_ndotl);
    let shadowed = mix(1.0 - camera.diffuse_shadow.y, 1.0, visibility);
    let uv = vec2<f32>(input.texcoord.x, 1.0 - input.texcoord.y);
    let texture_color = textureSample(
        base_color_texture,
        base_color_sampler,
        uv,
    );
    let tangent_normal_sample = textureSample(normal_texture, normal_sampler, uv).xyz * 2.0 - 1.0;
    let tangent_normal = normalize(vec3<f32>(
        tangent_normal_sample.xy * clamp(draw.material_params.w, 0.0, 2.0),
        tangent_normal_sample.z,
    ));
    let normal = normalize(
        tangent_frame(geometric_normal, input.world_position, uv) * tangent_normal,
    );
    let mapped_ndotl = max(dot(normal, light_dir), 0.0);
    let albedo = input.color.rgb * draw.base_color.rgb * texture_color.rgb;
    let roughness_sample = textureSample(roughness_texture, roughness_sampler, uv).r;
    let metallic_roughness_sample = textureSample(
        metallic_roughness_texture,
        metallic_roughness_sampler,
        uv,
    );
    let roughness = clamp(
        mix(draw.material_params.x, roughness_sample, draw.material_params.z) *
            mix(1.0, metallic_roughness_sample.g, draw.map_params.x),
        0.04,
        1.0,
    );
    let metallic = clamp(
        draw.material_params.y * mix(1.0, metallic_roughness_sample.b, draw.map_params.x),
        0.0,
        1.0,
    );
    let view_dir = normalize(camera.camera_position.xyz - input.world_position);
    let half_dir = normalize(view_dir + light_dir);
    let ndotv = max(dot(normal, view_dir), 0.0);
    let ndoth = max(dot(normal, half_dir), 0.0);
    let vdoth = max(dot(view_dir, half_dir), 0.0);
    let f0 = mix(vec3<f32>(0.04), albedo, metallic);
    let fresnel = fresnel_schlick(vdoth, f0);
    let distribution = distribution_ggx(ndoth, roughness);
    let geometry = geometry_smith(ndotv, mapped_ndotl, roughness);
    let specular = fresnel * distribution * geometry /
        max(4.0 * ndotv * mapped_ndotl, 1e-4);
    let diffuse = (vec3<f32>(1.0) - fresnel) * (1.0 - metallic) * albedo /
        3.14159265;
    let direct = (diffuse + specular) * mapped_ndotl * camera.diffuse_shadow.x * 3.2 * shadowed;
    let occlusion_sample = textureSample(occlusion_texture, occlusion_sampler, uv).r;
    let occlusion = mix(
        1.0,
        occlusion_sample,
        clamp(draw.map_params.y, 0.0, 1.0) * draw.map_params.z,
    );
    let emissive_sample = textureSample(emissive_texture, emissive_sampler, uv).rgb;
    let emissive = draw.emissive.rgb *
        mix(vec3<f32>(1.0), emissive_sample, draw.map_params.w);
    let environment_factor = camera.environment.w;
    let environment_diffuse = sample_diffuse_environment(normal) *
        camera.environment.x * environment_factor * albedo * (1.0 - metallic) * occlusion;
    let environment_reflection = reflect(-view_dir, normal);
    let environment_fresnel = fresnel_schlick(ndotv, f0);
    let environment_specular = sample_prefiltered_environment(environment_reflection, roughness * 4.0) *
        camera.environment.y * environment_factor * environment_fresnel * occlusion;
    let ambient = albedo * camera.light_ambient.w * (1.0 - metallic) * occlusion;
    let hdr_color = ambient + environment_diffuse + environment_specular + direct + emissive;
    return vec4<f32>(
        reinhard_tonemap(hdr_color),
        input.color.a * draw.base_color.a * texture_color.a,
    );
}
"#;

const SKY_SHADER: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
    inv_view_proj: mat4x4<f32>,
    light_view_proj: mat4x4<f32>,
    light_ambient: vec4<f32>,
    diffuse_shadow: vec4<f32>,
    camera_position: vec4<f32>,
    environment: vec4<f32>,
}

@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(1) @binding(0) var environment_texture: texture_2d<f32>;

struct SkyVertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) ndc_position: vec2<f32>,
}

@vertex
fn vs_sky(@builtin(vertex_index) vertex_index: u32) -> SkyVertexOutput {
    var positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    var output: SkyVertexOutput;
    output.clip_position = vec4<f32>(positions[vertex_index], 1.0, 1.0);
    output.ndc_position = positions[vertex_index];
    return output;
}

fn rotate_environment_direction(direction: vec3<f32>, rotation_rad: f32) -> vec3<f32> {
    let cosine = cos(rotation_rad);
    let sine = sin(rotation_rad);
    return vec3<f32>(
        cosine * direction.x - sine * direction.z,
        direction.y,
        sine * direction.x + cosine * direction.z,
    );
}

fn wrap_environment_index(index: i32, size: i32) -> i32 {
    let wrapped = index % size;
    return select(wrapped, wrapped + size, wrapped < 0);
}

fn sample_environment(direction: vec3<f32>) -> vec3<f32> {
    let rotated = rotate_environment_direction(
        normalize(direction),
        camera.environment.z,
    );
    let u = atan2(rotated.z, rotated.x) / 6.28318530 + 0.5;
    let v = acos(clamp(rotated.y, -1.0, 1.0)) / 3.14159265;
    let dimensions = textureDimensions(environment_texture);
    let width = max(i32(dimensions.x), 1);
    let height = max(i32(dimensions.y), 1);
    let x = u * f32(width) - 0.5;
    let y = v * f32(height) - 0.5;
    let x0 = wrap_environment_index(i32(floor(x)), width);
    let x1 = wrap_environment_index(x0 + 1, width);
    let y0 = clamp(i32(floor(y)), 0, height - 1);
    let y1 = clamp(y0 + 1, 0, height - 1);
    let fx = fract(x);
    let fy = clamp(fract(y), 0.0, 1.0);
    let c00 = textureLoad(environment_texture, vec2<i32>(x0, y0), 0).rgb;
    let c10 = textureLoad(environment_texture, vec2<i32>(x1, y0), 0).rgb;
    let c01 = textureLoad(environment_texture, vec2<i32>(x0, y1), 0).rgb;
    let c11 = textureLoad(environment_texture, vec2<i32>(x1, y1), 0).rgb;
    return mix(mix(c00, c10, fx), mix(c01, c11, fx), fy);
}

fn reinhard_tonemap(color: vec3<f32>) -> vec3<f32> {
    return color / (vec3<f32>(1.0) + color);
}

@fragment
fn fs_sky(input: SkyVertexOutput) -> @location(0) vec4<f32> {
    let near_clip = vec4<f32>(input.ndc_position, 0.0, 1.0);
    let far_clip = vec4<f32>(input.ndc_position, 1.0, 1.0);
    let near_world = camera.inv_view_proj * near_clip;
    let far_world = camera.inv_view_proj * far_clip;
    let near_position = near_world.xyz / near_world.w;
    let far_position = far_world.xyz / far_world.w;
    let direction = normalize(far_position - near_position);
    let color = sample_environment(direction) * camera.environment.w;
    return vec4<f32>(reinhard_tonemap(color), 1.0);
}
"#;

const SHADOW_SHADER: &str = r#"
struct ShadowUniform {
    light_view_proj: mat4x4<f32>,
}

struct DrawUniform {
    model: mat4x4<f32>,
    normal_col0: vec4<f32>,
    normal_col1: vec4<f32>,
    normal_col2: vec4<f32>,
    color: vec4<f32>,
    base_color: vec4<f32>,
    material_params: vec4<f32>,
    emissive: vec4<f32>,
    map_params: vec4<f32>,
    skinning: vec4<f32>,
}

struct SkinStorage {
    mesh_transform: mat4x4<f32>,
    joint_matrices: array<mat4x4<f32>>,
}

@group(0) @binding(0) var<uniform> shadow: ShadowUniform;
@group(1) @binding(0) var<uniform> draw: DrawUniform;
@group(2) @binding(0) var<storage, read> skin: SkinStorage;

fn skinned_position(
    position: vec3<f32>,
    joints: vec4<u32>,
    weights: vec4<f32>,
) -> vec4<f32> {
    if (draw.skinning.x < 0.5) {
        return vec4<f32>(position, 1.0);
    }
    let source = vec4<f32>(position, 1.0);
    var result = vec4<f32>(0.0);
    result += skin.joint_matrices[joints.x] * source * weights.x;
    result += skin.joint_matrices[joints.y] * source * weights.y;
    result += skin.joint_matrices[joints.z] * source * weights.z;
    result += skin.joint_matrices[joints.w] * source * weights.w;
    return skin.mesh_transform * result;
}

@vertex
fn vs_shadow(
    @location(0) position: vec3<f32>,
    @location(3) joints: vec4<u32>,
    @location(4) weights: vec4<f32>,
) -> @builtin(position) vec4<f32> {
    let local_position = skinned_position(position, joints, weights);
    return shadow.light_view_proj * draw.model * local_position;
}
"#;

use bytemuck::{Pod, Zeroable};
use rne_math::{Mat4, Transform3, Vec3};
use rne_render::{
    Camera, CameraPassOutput, DepthFrame, EnvironmentLighting, EnvironmentMap, ImageFrame,
    RenderError, RenderScene, RenderTarget, SkinningData, TriangleMesh, VisualShape,
};
use std::collections::HashMap;
use std::sync::Arc;
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
    texcoord: [f32; 2],
    joints: [u16; 4],
    weights: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
    inv_view_proj: [[f32; 4]; 4],
    light_view_proj: [[f32; 4]; 4],
    light_ambient: [f32; 4],
    diffuse_shadow: [f32; 4],
    camera_position: [f32; 4],
    environment: [f32; 4],
}

/// Default directional light for primitive shading (world space, not normalized).
pub const DEFAULT_LIGHT_DIR: [f32; 3] = [0.35, 0.9, 0.25];
/// Minimum lit channel multiplier applied away from the light.
pub const DEFAULT_AMBIENT: f32 = 0.28;
/// Additional diffuse multiplier at surfaces facing the light.
pub const DEFAULT_DIFFUSE: f32 = 0.72;
/// Fraction of direct illumination removed by a fully shadowed surface.
pub const DEFAULT_SHADOW_STRENGTH: f32 = 0.82;

fn default_camera_uniform(
    view_proj: rne_math::Mat4,
    inv_view_proj: rne_math::Mat4,
    light_view_proj: rne_math::Mat4,
    camera_position: Vec3,
    environment: &EnvironmentLighting,
) -> CameraUniform {
    let environment = environment.sanitized();
    CameraUniform {
        view_proj: mat4_to_cols(view_proj),
        inv_view_proj: mat4_to_cols(inv_view_proj),
        light_view_proj: mat4_to_cols(light_view_proj),
        light_ambient: [
            DEFAULT_LIGHT_DIR[0],
            DEFAULT_LIGHT_DIR[1],
            DEFAULT_LIGHT_DIR[2],
            DEFAULT_AMBIENT,
        ],
        diffuse_shadow: [DEFAULT_DIFFUSE, DEFAULT_SHADOW_STRENGTH, 0.0, 0.0],
        camera_position: [
            camera_position.x as f32,
            camera_position.y as f32,
            camera_position.z as f32,
            1.0,
        ],
        environment: [
            environment.diffuse_strength,
            environment.specular_strength,
            environment.rotation_rad,
            if environment.map.is_some() {
                environment.intensity
            } else {
                0.0
            },
        ],
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct ShadowUniform {
    light_view_proj: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct DrawUniform {
    model: [[f32; 4]; 4],
    normal_col0: [f32; 4],
    normal_col1: [f32; 4],
    normal_col2: [f32; 4],
    color: [f32; 4],
    base_color: [f32; 4],
    material_params: [f32; 4],
    emissive: [f32; 4],
    map_params: [f32; 4],
    skinning: [f32; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct SkinStorageHeader {
    mesh_transform: [[f32; 4]; 4],
}

struct BuiltPrimitiveMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
}

pub struct PrimitiveRenderer {
    pipeline: wgpu::RenderPipeline,
    sky_pipeline: wgpu::RenderPipeline,
    shadow_pipeline: wgpu::RenderPipeline,
    camera_layout: wgpu::BindGroupLayout,
    environment_layout: wgpu::BindGroupLayout,
    shadow_camera_layout: wgpu::BindGroupLayout,
    skin_layout: wgpu::BindGroupLayout,
    fallback_skin: GpuSkin,
    draw_bind_group: wgpu::BindGroup,
    draw_uniform_stride: u32,
    box_mesh: BuiltPrimitiveMesh,
    sphere_mesh: BuiltPrimitiveMesh,
    cylinder_mesh: BuiltPrimitiveMesh,
    camera_buffer: wgpu::Buffer,
    shadow_camera_buffer: wgpu::Buffer,
    draw_buffer: wgpu::Buffer,
    mesh_cache: HashMap<usize, GpuMesh>,
    texture_layout: wgpu::BindGroupLayout,
    fallback_texture: GpuTexture,
    texture_cache: HashMap<usize, GpuTexture>,
    fallback_normal_texture: GpuTexture,
    normal_texture_cache: HashMap<usize, GpuTexture>,
    fallback_roughness_texture: GpuTexture,
    roughness_texture_cache: HashMap<usize, GpuTexture>,
    fallback_metallic_roughness_texture: GpuTexture,
    metallic_roughness_texture_cache: HashMap<usize, GpuTexture>,
    fallback_emissive_texture: GpuTexture,
    emissive_texture_cache: HashMap<usize, GpuTexture>,
    fallback_occlusion_texture: GpuTexture,
    occlusion_texture_cache: HashMap<usize, GpuTexture>,
    fallback_environment: GpuEnvironmentTexture,
    environment_texture_cache: HashMap<usize, GpuEnvironmentTexture>,
    taa: TemporalAntiAliasing,
    _shadow_texture: wgpu::Texture,
    shadow_view: wgpu::TextureView,
    shadow_bind_group: wgpu::BindGroup,
}

struct GpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    index_format: wgpu::IndexFormat,
    skin: Option<GpuSkin>,
}

struct GpuSkin {
    _buffer: wgpu::Buffer,
    bind_group: wgpu::BindGroup,
}

struct GpuTexture {
    _texture: wgpu::Texture,
    bind_group: wgpu::BindGroup,
}

struct GpuEnvironmentTexture {
    _texture: wgpu::Texture,
    _prefiltered_texture: wgpu::Texture,
    _diffuse_texture: wgpu::Texture,
    bind_group: wgpu::BindGroup,
}

/// Color and depth views for an on-screen or off-screen render pass.
pub struct PrimitiveRenderViews<'a> {
    /// Width of the color and depth attachments in pixels.
    pub width: u32,
    /// Height of the color and depth attachments in pixels.
    pub height: u32,
    /// Color attachment view.
    pub color_view: &'a wgpu::TextureView,
    /// Depth attachment view.
    pub depth_view: &'a wgpu::TextureView,
}

/// Inputs for rendering a scene into existing GPU views.
pub struct PrimitiveSurfacePass<'a> {
    /// GPU device.
    pub device: &'a wgpu::Device,
    /// GPU queue.
    pub queue: &'a wgpu::Queue,
    /// Camera parameters.
    pub camera: &'a Camera,
    /// Camera world transform.
    pub view: &'a Transform3,
    /// Scene primitives to draw.
    pub scene: &'a RenderScene,
    /// HDR environment lighting and background settings.
    pub environment: &'a EnvironmentLighting,
    /// Clear color for empty pixels.
    pub clear_color: [f32; 4],
    /// Render targets.
    pub targets: &'a PrimitiveRenderViews<'a>,
}

/// Inputs for one off-screen primitive render pass.
pub struct PrimitiveRenderPass<'a> {
    /// GPU device.
    pub device: &'a wgpu::Device,
    /// GPU queue.
    pub queue: &'a wgpu::Queue,
    /// Output target dimensions.
    pub target: RenderTarget,
    /// Camera parameters.
    pub camera: &'a Camera,
    /// Camera world transform.
    pub view: &'a Transform3,
    /// Scene primitives to draw.
    pub scene: &'a RenderScene,
    /// HDR environment lighting and background settings.
    pub environment: &'a EnvironmentLighting,
    /// Clear color for empty pixels.
    pub clear_color: [f32; 4],
}

impl PrimitiveRenderer {
    pub fn new(
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        color_format: wgpu::TextureFormat,
    ) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rne_primitive_shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });
        let sky_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rne_environment_sky_shader"),
            source: wgpu::ShaderSource::Wgsl(SKY_SHADER.into()),
        });
        let shadow_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rne_shadow_shader"),
            source: wgpu::ShaderSource::Wgsl(SHADOW_SHADER.into()),
        });

        let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("rne_camera_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX_FRAGMENT,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });

        let draw_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("rne_draw_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: true,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let shadow_camera_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_shadow_camera_layout"),
                entries: &[wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::VERTEX,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                }],
            });
        let skin_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("rne_skin_storage_layout"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Storage { read_only: true },
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let texture_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("rne_base_color_texture_layout"),
            entries: &[
                wgpu::BindGroupLayoutEntry {
                    binding: 0,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: true },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 1,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
            ],
        });
        let shadow_texture_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_shadow_texture_layout"),
                entries: &[
                    wgpu::BindGroupLayoutEntry {
                        binding: 0,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Texture {
                            sample_type: wgpu::TextureSampleType::Depth,
                            view_dimension: wgpu::TextureViewDimension::D2,
                            multisampled: false,
                        },
                        count: None,
                    },
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Comparison),
                        count: None,
                    },
                ],
            });
        let environment_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_environment_texture_layout"),
                entries: &[0, 1, 2].map(|binding| wgpu::BindGroupLayoutEntry {
                    binding,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                }),
            });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("rne_primitive_pipeline_layout"),
            bind_group_layouts: &[
                &camera_layout,
                &draw_layout,
                &texture_layout,
                &shadow_texture_layout,
                &texture_layout,
                &texture_layout,
                &texture_layout,
                &texture_layout,
                &texture_layout,
                &environment_layout,
                &skin_layout,
            ],
            push_constant_ranges: &[],
        });

        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("rne_primitive_pipeline"),
            layout: Some(&pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x3,
                            offset: 0,
                            shader_location: 0,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x3,
                            offset: 12,
                            shader_location: 1,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x2,
                            offset: 24,
                            shader_location: 2,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Uint16x4,
                            offset: 32,
                            shader_location: 3,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x4,
                            offset: 40,
                            shader_location: 4,
                        },
                    ],
                }],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: color_format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState::default(),
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });
        let sky_pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("rne_environment_sky_pipeline_layout"),
            bind_group_layouts: &[&camera_layout, &environment_layout],
            push_constant_ranges: &[],
        });
        let sky_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("rne_environment_sky_pipeline"),
            layout: Some(&sky_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &sky_shader,
                entry_point: Some("vs_sky"),
                buffers: &[],
                compilation_options: Default::default(),
            },
            fragment: Some(wgpu::FragmentState {
                module: &sky_shader,
                entry_point: Some("fs_sky"),
                targets: &[Some(wgpu::ColorTargetState {
                    format: color_format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
                compilation_options: Default::default(),
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: None,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });
        let shadow_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("rne_shadow_pipeline_layout"),
                bind_group_layouts: &[&shadow_camera_layout, &draw_layout, &skin_layout],
                push_constant_ranges: &[],
            });
        let shadow_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("rne_shadow_pipeline"),
            layout: Some(&shadow_pipeline_layout),
            vertex: wgpu::VertexState {
                module: &shadow_shader,
                entry_point: Some("vs_shadow"),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as wgpu::BufferAddress,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x3,
                            offset: 0,
                            shader_location: 0,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Uint16x4,
                            offset: 32,
                            shader_location: 3,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x4,
                            offset: 40,
                            shader_location: 4,
                        },
                    ],
                }],
                compilation_options: Default::default(),
            },
            fragment: None,
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                strip_index_format: None,
                front_face: wgpu::FrontFace::Ccw,
                cull_mode: Some(wgpu::Face::Back),
                ..Default::default()
            },
            depth_stencil: Some(wgpu::DepthStencilState {
                format: wgpu::TextureFormat::Depth32Float,
                depth_write_enabled: true,
                depth_compare: wgpu::CompareFunction::Less,
                stencil: wgpu::StencilState::default(),
                bias: wgpu::DepthBiasState {
                    constant: 2,
                    slope_scale: 2.0,
                    clamp: 0.0,
                },
            }),
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        let box_mesh = upload_primitive(device, "rne_box", &unit_cube());
        let sphere_mesh = upload_primitive(device, "rne_sphere", &unit_sphere());
        let cylinder_mesh = upload_primitive(device, "rne_cylinder", &unit_cylinder());
        let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_camera_uniform"),
            size: std::mem::size_of::<CameraUniform>() as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let shadow_camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_shadow_camera_uniform"),
            size: std::mem::size_of::<ShadowUniform>() as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let draw_uniform_stride =
            uniform_stride(device.limits().min_uniform_buffer_offset_alignment);
        let draw_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_draw_uniform"),
            size: (draw_uniform_stride * MAX_SCENE_ITEMS) as wgpu::BufferAddress,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let draw_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_draw_bind_group"),
            layout: &draw_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::Buffer(wgpu::BufferBinding {
                    buffer: &draw_buffer,
                    offset: 0,
                    size: std::num::NonZeroU64::new(std::mem::size_of::<DrawUniform>() as u64),
                }),
            }],
        });
        let fallback_skin = upload_skinning(
            device,
            &skin_layout,
            &SkinningData {
                mesh_transform: Mat4::IDENTITY,
                joints: vec![[0; 4]],
                weights: vec![[0.0; 4]],
                joint_matrices: vec![Mat4::IDENTITY],
            },
        );
        let fallback_texture = upload_texture(
            device,
            queue,
            &texture_layout,
            "rne_white_texture",
            &ImageFrame::from_rgba8(1, 1, vec![255, 255, 255, 255]),
            wgpu::TextureFormat::Rgba8UnormSrgb,
        );
        let fallback_normal_texture = upload_texture(
            device,
            queue,
            &texture_layout,
            "rne_flat_normal_texture",
            &ImageFrame::from_rgba8(1, 1, vec![128, 128, 255, 255]),
            wgpu::TextureFormat::Rgba8Unorm,
        );
        let fallback_roughness_texture = upload_texture(
            device,
            queue,
            &texture_layout,
            "rne_white_roughness_texture",
            &ImageFrame::from_rgba8(1, 1, vec![255, 255, 255, 255]),
            wgpu::TextureFormat::Rgba8Unorm,
        );
        let fallback_metallic_roughness_texture = upload_texture(
            device,
            queue,
            &texture_layout,
            "rne_white_metallic_roughness_texture",
            &ImageFrame::from_rgba8(1, 1, vec![255, 255, 255, 255]),
            wgpu::TextureFormat::Rgba8Unorm,
        );
        let fallback_emissive_texture = upload_texture(
            device,
            queue,
            &texture_layout,
            "rne_white_emissive_texture",
            &ImageFrame::from_rgba8(1, 1, vec![255, 255, 255, 255]),
            wgpu::TextureFormat::Rgba8UnormSrgb,
        );
        let fallback_occlusion_texture = upload_texture(
            device,
            queue,
            &texture_layout,
            "rne_white_occlusion_texture",
            &ImageFrame::from_rgba8(1, 1, vec![255, 255, 255, 255]),
            wgpu::TextureFormat::Rgba8Unorm,
        );
        let fallback_environment = upload_environment_texture(
            device,
            queue,
            &environment_layout,
            "rne_fallback_environment",
            &EnvironmentMap::solid([0.0, 0.0, 0.0, 1.0]).expect("valid fallback environment"),
        );
        let shadow_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_directional_shadow_map"),
            size: wgpu::Extent3d {
                width: SHADOW_MAP_SIZE,
                height: SHADOW_MAP_SIZE,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let shadow_view = shadow_texture.create_view(&wgpu::TextureViewDescriptor::default());
        let shadow_sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("rne_shadow_comparison_sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            address_mode_w: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            compare: Some(wgpu::CompareFunction::LessEqual),
            ..Default::default()
        });
        let shadow_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_shadow_texture_bind_group"),
            layout: &shadow_texture_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(&shadow_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&shadow_sampler),
                },
            ],
        });

        Self {
            pipeline,
            sky_pipeline,
            shadow_pipeline,
            camera_layout,
            environment_layout,
            shadow_camera_layout,
            skin_layout,
            fallback_skin,
            draw_bind_group,
            draw_uniform_stride,
            box_mesh,
            sphere_mesh,
            cylinder_mesh,
            camera_buffer,
            shadow_camera_buffer,
            draw_buffer,
            mesh_cache: HashMap::new(),
            texture_layout,
            fallback_texture,
            texture_cache: HashMap::new(),
            fallback_normal_texture,
            normal_texture_cache: HashMap::new(),
            fallback_roughness_texture,
            roughness_texture_cache: HashMap::new(),
            fallback_metallic_roughness_texture,
            metallic_roughness_texture_cache: HashMap::new(),
            fallback_emissive_texture,
            emissive_texture_cache: HashMap::new(),
            fallback_occlusion_texture,
            occlusion_texture_cache: HashMap::new(),
            fallback_environment,
            environment_texture_cache: HashMap::new(),
            taa: TemporalAntiAliasing::new(device, color_format),
            _shadow_texture: shadow_texture,
            shadow_view,
            shadow_bind_group,
        }
    }

    /// Enables or configures temporal anti-aliasing for subsequent frames.
    pub fn set_taa(&mut self, settings: TaaSettings) {
        self.taa.set_settings(settings);
    }

    /// Discards accumulated temporal history before the next frame.
    pub fn reset_taa_history(&mut self) {
        self.taa.reset_history();
    }

    fn primitive_mesh_for(&self, shape: &VisualShape) -> &BuiltPrimitiveMesh {
        match shape {
            VisualShape::Sphere { .. } => &self.sphere_mesh,
            VisualShape::Cylinder { .. } => &self.cylinder_mesh,
            VisualShape::Box { .. } | VisualShape::Mesh { .. } | VisualShape::DynamicMesh => {
                &self.box_mesh
            }
        }
    }

    /// Renders a scene into existing color and depth views without CPU readback.
    pub fn render_to_views(&mut self, pass: PrimitiveSurfacePass<'_>) -> Result<(), RenderError> {
        let device = pass.device;
        let queue = pass.queue;
        let camera = pass.camera;
        let view = pass.view;
        let scene = pass.scene;
        let environment = pass.environment;
        let clear_color = pass.clear_color;
        let targets = pass.targets;
        let base_view_proj = camera.view_projection(view);
        let scene_key = scene_temporal_key(scene);
        let taa_frame = self.taa.begin_frame(
            device,
            targets.width,
            targets.height,
            base_view_proj,
            scene_key,
        );
        let view_proj = if taa_frame.enabled {
            taa_frame.current_view_proj
        } else {
            base_view_proj
        };
        let inv_view_proj = view_proj.inverse();
        let light_view_proj = directional_light_view_projection(scene);
        queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::bytes_of(&default_camera_uniform(
                view_proj,
                inv_view_proj,
                light_view_proj,
                view.translation,
                environment,
            )),
        );
        queue.write_buffer(
            &self.shadow_camera_buffer,
            0,
            bytemuck::bytes_of(&ShadowUniform {
                light_view_proj: mat4_to_cols(light_view_proj),
            }),
        );

        if scene.items.len() > MAX_SCENE_ITEMS as usize {
            return Err(RenderError::RenderFailed(format!(
                "scene item count {} exceeds limit {MAX_SCENE_ITEMS}",
                scene.items.len()
            )));
        }

        let mut draw_bytes = vec![0_u8; self.draw_uniform_stride as usize * scene.items.len()];
        for (index, item) in scene.items.iter().enumerate() {
            let model = item.transform.to_matrix();
            let normal_cols = normal_matrix_cols(model);
            let material = item.material.sanitized();
            let uniform = DrawUniform {
                model: mat4_to_cols(model),
                normal_col0: normal_cols[0],
                normal_col1: normal_cols[1],
                normal_col2: normal_cols[2],
                color: item.color_rgba,
                base_color: material.base_color_rgba,
                material_params: [
                    material.roughness,
                    material.metallic,
                    f32::from(material.roughness_texture.is_some()),
                    material.normal_strength,
                ],
                emissive: [
                    material.emissive_rgb[0],
                    material.emissive_rgb[1],
                    material.emissive_rgb[2],
                    0.0,
                ],
                map_params: [
                    f32::from(material.metallic_roughness_texture.is_some()),
                    material.occlusion_strength,
                    f32::from(material.occlusion_texture.is_some()),
                    f32::from(material.emissive_texture.is_some()),
                ],
                skinning: [
                    f32::from(
                        item.mesh
                            .as_ref()
                            .is_some_and(|mesh| mesh.skinning.is_some()),
                    ),
                    0.0,
                    0.0,
                    0.0,
                ],
            };
            let offset = index * self.draw_uniform_stride as usize;
            draw_bytes[offset..offset + std::mem::size_of::<DrawUniform>()]
                .copy_from_slice(bytemuck::bytes_of(&uniform));
        }
        queue.write_buffer(&self.draw_buffer, 0, &draw_bytes);

        if taa_frame.enabled {
            self.taa
                .prepare(queue, taa_frame, targets.width, targets.height);
        }

        let dynamic_meshes = scene
            .items
            .iter()
            .map(|item| {
                if item.shape == VisualShape::DynamicMesh {
                    item.mesh
                        .as_ref()
                        .map(|mesh| upload_mesh(device, mesh, &self.skin_layout))
                } else {
                    None
                }
            })
            .collect::<Vec<_>>();
        for item in &scene.items {
            if let Some(mesh) = &item.mesh {
                let _ = self.gpu_mesh(device, mesh);
            }
            if let Some(texture) = &item.base_color_texture {
                let key = Arc::as_ptr(texture) as usize;
                if !self.texture_cache.contains_key(&key) {
                    let uploaded = upload_texture(
                        device,
                        queue,
                        &self.texture_layout,
                        "rne_base_color_texture",
                        texture,
                        wgpu::TextureFormat::Rgba8UnormSrgb,
                    );
                    self.texture_cache.insert(key, uploaded);
                }
            }
            if let Some(texture) = &item.material.normal_texture {
                let key = Arc::as_ptr(texture) as usize;
                if !self.normal_texture_cache.contains_key(&key) {
                    let uploaded = upload_texture(
                        device,
                        queue,
                        &self.texture_layout,
                        "rne_normal_texture",
                        texture,
                        wgpu::TextureFormat::Rgba8Unorm,
                    );
                    self.normal_texture_cache.insert(key, uploaded);
                }
            }
            if let Some(texture) = &item.material.roughness_texture {
                let key = Arc::as_ptr(texture) as usize;
                if !self.roughness_texture_cache.contains_key(&key) {
                    let uploaded = upload_texture(
                        device,
                        queue,
                        &self.texture_layout,
                        "rne_roughness_texture",
                        texture,
                        wgpu::TextureFormat::Rgba8Unorm,
                    );
                    self.roughness_texture_cache.insert(key, uploaded);
                }
            }
            if let Some(texture) = &item.material.metallic_roughness_texture {
                let key = Arc::as_ptr(texture) as usize;
                if !self.metallic_roughness_texture_cache.contains_key(&key) {
                    let uploaded = upload_texture(
                        device,
                        queue,
                        &self.texture_layout,
                        "rne_metallic_roughness_texture",
                        texture,
                        wgpu::TextureFormat::Rgba8Unorm,
                    );
                    self.metallic_roughness_texture_cache.insert(key, uploaded);
                }
            }
            if let Some(texture) = &item.material.emissive_texture {
                let key = Arc::as_ptr(texture) as usize;
                if !self.emissive_texture_cache.contains_key(&key) {
                    let uploaded = upload_texture(
                        device,
                        queue,
                        &self.texture_layout,
                        "rne_emissive_texture",
                        texture,
                        wgpu::TextureFormat::Rgba8UnormSrgb,
                    );
                    self.emissive_texture_cache.insert(key, uploaded);
                }
            }
            if let Some(texture) = &item.material.occlusion_texture {
                let key = Arc::as_ptr(texture) as usize;
                if !self.occlusion_texture_cache.contains_key(&key) {
                    let uploaded = upload_texture(
                        device,
                        queue,
                        &self.texture_layout,
                        "rne_occlusion_texture",
                        texture,
                        wgpu::TextureFormat::Rgba8Unorm,
                    );
                    self.occlusion_texture_cache.insert(key, uploaded);
                }
            }
        }

        let environment_bind_group = self
            .environment_texture(device, queue, environment)
            .bind_group
            .clone();

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_camera_bind_group"),
            layout: &self.camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: self.camera_buffer.as_entire_binding(),
            }],
        });
        let shadow_camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_shadow_camera_bind_group"),
            layout: &self.shadow_camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: self.shadow_camera_buffer.as_entire_binding(),
            }],
        });

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("rne_scene_encoder"),
        });

        let taa_scene_view = if taa_frame.enabled {
            Some(self.taa.scene_view())
        } else {
            None
        };
        let scene_color_view = taa_scene_view.unwrap_or(targets.color_view);

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("rne_directional_shadow_pass"),
                color_attachments: &[],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: &self.shadow_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.shadow_pipeline);
            pass.set_bind_group(0, &shadow_camera_bind_group, &[]);
            for (index, item) in scene.items.iter().enumerate() {
                pass.set_bind_group(
                    1,
                    &self.draw_bind_group,
                    &[index as u32 * self.draw_uniform_stride],
                );
                if let Some(gpu_mesh) = &dynamic_meshes[index] {
                    let skin_bind_group = gpu_mesh
                        .skin
                        .as_ref()
                        .map(|skin| &skin.bind_group)
                        .unwrap_or(&self.fallback_skin.bind_group);
                    pass.set_bind_group(2, skin_bind_group, &[]);
                    pass.set_vertex_buffer(0, gpu_mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(gpu_mesh.index_buffer.slice(..), gpu_mesh.index_format);
                    pass.draw_indexed(0..gpu_mesh.index_count, 0, 0..1);
                } else if let Some(mesh) = &item.mesh {
                    let gpu_mesh = self
                        .mesh_cache
                        .get(&(Arc::as_ptr(mesh) as usize))
                        .expect("mesh uploaded before shadow pass");
                    let skin_bind_group = gpu_mesh
                        .skin
                        .as_ref()
                        .map(|skin| &skin.bind_group)
                        .unwrap_or(&self.fallback_skin.bind_group);
                    pass.set_bind_group(2, skin_bind_group, &[]);
                    pass.set_vertex_buffer(0, gpu_mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(gpu_mesh.index_buffer.slice(..), gpu_mesh.index_format);
                    pass.draw_indexed(0..gpu_mesh.index_count, 0, 0..1);
                } else {
                    let primitive = self.primitive_mesh_for(&item.shape);
                    pass.set_bind_group(2, &self.fallback_skin.bind_group, &[]);
                    pass.set_vertex_buffer(0, primitive.vertex_buffer.slice(..));
                    pass.set_index_buffer(
                        primitive.index_buffer.slice(..),
                        wgpu::IndexFormat::Uint16,
                    );
                    pass.draw_indexed(0..primitive.index_count, 0, 0..1);
                }
            }
        }

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("rne_scene_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: scene_color_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: f64::from(clear_color[0]),
                            g: f64::from(clear_color[1]),
                            b: f64::from(clear_color[2]),
                            a: f64::from(clear_color[3]),
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: Some(wgpu::RenderPassDepthStencilAttachment {
                    view: targets.depth_view,
                    depth_ops: Some(wgpu::Operations {
                        load: wgpu::LoadOp::Clear(1.0),
                        store: wgpu::StoreOp::Store,
                    }),
                    stencil_ops: None,
                }),
                timestamp_writes: None,
                occlusion_query_set: None,
            });

            if environment.is_enabled() {
                pass.set_pipeline(&self.sky_pipeline);
                pass.set_bind_group(0, &camera_bind_group, &[]);
                pass.set_bind_group(1, &environment_bind_group, &[]);
                pass.draw(0..3, 0..1);
            }
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &camera_bind_group, &[]);
            pass.set_bind_group(3, &self.shadow_bind_group, &[]);
            pass.set_bind_group(9, &environment_bind_group, &[]);

            for (index, item) in scene.items.iter().enumerate() {
                pass.set_bind_group(
                    1,
                    &self.draw_bind_group,
                    &[index as u32 * self.draw_uniform_stride],
                );
                let texture = item
                    .base_color_texture
                    .as_ref()
                    .and_then(|texture| self.texture_cache.get(&(Arc::as_ptr(texture) as usize)))
                    .unwrap_or(&self.fallback_texture);
                let normal_texture = item
                    .material
                    .normal_texture
                    .as_ref()
                    .and_then(|texture| {
                        self.normal_texture_cache
                            .get(&(Arc::as_ptr(texture) as usize))
                    })
                    .unwrap_or(&self.fallback_normal_texture);
                let roughness_texture = item
                    .material
                    .roughness_texture
                    .as_ref()
                    .and_then(|texture| {
                        self.roughness_texture_cache
                            .get(&(Arc::as_ptr(texture) as usize))
                    })
                    .unwrap_or(&self.fallback_roughness_texture);
                let metallic_roughness_texture = item
                    .material
                    .metallic_roughness_texture
                    .as_ref()
                    .and_then(|texture| {
                        self.metallic_roughness_texture_cache
                            .get(&(Arc::as_ptr(texture) as usize))
                    })
                    .unwrap_or(&self.fallback_metallic_roughness_texture);
                let emissive_texture = item
                    .material
                    .emissive_texture
                    .as_ref()
                    .and_then(|texture| {
                        self.emissive_texture_cache
                            .get(&(Arc::as_ptr(texture) as usize))
                    })
                    .unwrap_or(&self.fallback_emissive_texture);
                let occlusion_texture = item
                    .material
                    .occlusion_texture
                    .as_ref()
                    .and_then(|texture| {
                        self.occlusion_texture_cache
                            .get(&(Arc::as_ptr(texture) as usize))
                    })
                    .unwrap_or(&self.fallback_occlusion_texture);
                pass.set_bind_group(2, &texture.bind_group, &[]);
                pass.set_bind_group(4, &normal_texture.bind_group, &[]);
                pass.set_bind_group(5, &roughness_texture.bind_group, &[]);
                pass.set_bind_group(6, &metallic_roughness_texture.bind_group, &[]);
                pass.set_bind_group(7, &emissive_texture.bind_group, &[]);
                pass.set_bind_group(8, &occlusion_texture.bind_group, &[]);

                if let Some(gpu_mesh) = &dynamic_meshes[index] {
                    let skin_bind_group = gpu_mesh
                        .skin
                        .as_ref()
                        .map(|skin| &skin.bind_group)
                        .unwrap_or(&self.fallback_skin.bind_group);
                    pass.set_bind_group(10, skin_bind_group, &[]);
                    pass.set_vertex_buffer(0, gpu_mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(gpu_mesh.index_buffer.slice(..), gpu_mesh.index_format);
                    pass.draw_indexed(0..gpu_mesh.index_count, 0, 0..1);
                } else if let Some(mesh) = &item.mesh {
                    let gpu_mesh = self
                        .mesh_cache
                        .get(&(Arc::as_ptr(mesh) as usize))
                        .expect("mesh uploaded before render pass");
                    let skin_bind_group = gpu_mesh
                        .skin
                        .as_ref()
                        .map(|skin| &skin.bind_group)
                        .unwrap_or(&self.fallback_skin.bind_group);
                    pass.set_bind_group(10, skin_bind_group, &[]);
                    pass.set_vertex_buffer(0, gpu_mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(gpu_mesh.index_buffer.slice(..), gpu_mesh.index_format);
                    pass.draw_indexed(0..gpu_mesh.index_count, 0, 0..1);
                } else {
                    let primitive = self.primitive_mesh_for(&item.shape);
                    pass.set_bind_group(10, &self.fallback_skin.bind_group, &[]);
                    pass.set_vertex_buffer(0, primitive.vertex_buffer.slice(..));
                    pass.set_index_buffer(
                        primitive.index_buffer.slice(..),
                        wgpu::IndexFormat::Uint16,
                    );
                    pass.draw_indexed(0..primitive.index_count, 0, 0..1);
                }
            }
        }

        if taa_frame.enabled {
            self.taa
                .encode(device, &mut encoder, targets.depth_view, targets.color_view);
        }

        queue.submit(Some(encoder.finish()));
        self.taa.commit(taa_frame);
        Ok(())
    }

    pub fn render(
        &mut self,
        pass: PrimitiveRenderPass<'_>,
    ) -> Result<CameraPassOutput, RenderError> {
        let target = pass.target;
        let camera = pass.camera;
        let view = pass.view;
        let scene = pass.scene;
        let environment = pass.environment;
        let clear_color = pass.clear_color;
        let device = pass.device;
        let queue = pass.queue;
        let color_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_color_target"),
            size: wgpu::Extent3d {
                width: target.width.max(1),
                height: target.height.max(1),
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Rgba8UnormSrgb,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::COPY_SRC
                | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let color_view = color_texture.create_view(&wgpu::TextureViewDescriptor::default());

        let depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_depth_target"),
            size: wgpu::Extent3d {
                width: target.width.max(1),
                height: target.height.max(1),
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: wgpu::TextureFormat::Depth32Float,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::COPY_SRC
                | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        self.render_to_views(PrimitiveSurfacePass {
            device,
            queue,
            camera,
            view,
            scene,
            environment,
            clear_color,
            targets: &PrimitiveRenderViews {
                width: target.width,
                height: target.height,
                color_view: &color_view,
                depth_view: &depth_view,
            },
        })?;

        let color_buffer;
        let depth_buffer;
        {
            let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("rne_scene_readback_encoder"),
            });
            let bytes_per_row = align_to(target.width * 4, wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
            let buffer_size = bytes_per_row as u64 * target.height as u64;
            color_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rne_color_readback"),
                size: buffer_size,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });
            depth_buffer = device.create_buffer(&wgpu::BufferDescriptor {
                label: Some("rne_depth_readback"),
                size: buffer_size,
                usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
                mapped_at_creation: false,
            });

            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfo {
                    texture: &color_texture,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::All,
                },
                wgpu::TexelCopyBufferInfo {
                    buffer: &color_buffer,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(bytes_per_row),
                        rows_per_image: Some(target.height),
                    },
                },
                wgpu::Extent3d {
                    width: target.width,
                    height: target.height,
                    depth_or_array_layers: 1,
                },
            );
            encoder.copy_texture_to_buffer(
                wgpu::TexelCopyTextureInfo {
                    texture: &depth_texture,
                    mip_level: 0,
                    origin: wgpu::Origin3d::ZERO,
                    aspect: wgpu::TextureAspect::DepthOnly,
                },
                wgpu::TexelCopyBufferInfo {
                    buffer: &depth_buffer,
                    layout: wgpu::TexelCopyBufferLayout {
                        offset: 0,
                        bytes_per_row: Some(bytes_per_row),
                        rows_per_image: Some(target.height),
                    },
                },
                wgpu::Extent3d {
                    width: target.width,
                    height: target.height,
                    depth_or_array_layers: 1,
                },
            );
            queue.submit(Some(encoder.finish()));
        }

        let color = map_color_buffer(device, &color_buffer, target)?;
        let depth = map_depth_buffer(device, &depth_buffer, target, camera)?;
        Ok(CameraPassOutput { color, depth })
    }

    fn gpu_mesh(&mut self, device: &wgpu::Device, mesh: &Arc<TriangleMesh>) -> &GpuMesh {
        let key = Arc::as_ptr(mesh) as usize;
        if !self.mesh_cache.contains_key(&key) {
            let uploaded = upload_mesh(device, mesh, &self.skin_layout);
            self.mesh_cache.insert(key, uploaded);
        }
        self.mesh_cache.get(&key).expect("mesh uploaded into cache")
    }

    fn environment_texture(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        lighting: &EnvironmentLighting,
    ) -> &GpuEnvironmentTexture {
        let Some(map) = lighting.map.as_ref() else {
            return &self.fallback_environment;
        };
        let key = Arc::as_ptr(map) as usize;
        if !self.environment_texture_cache.contains_key(&key) {
            let uploaded = upload_environment_texture(
                device,
                queue,
                &self.environment_layout,
                "rne_environment_texture",
                map,
            );
            self.environment_texture_cache.insert(key, uploaded);
        }
        self.environment_texture_cache
            .get(&key)
            .expect("environment texture uploaded")
    }
}

fn upload_primitive(
    device: &wgpu::Device,
    label: &str,
    mesh: &(Vec<Vertex>, Vec<u16>),
) -> BuiltPrimitiveMesh {
    let (vertices, indices) = mesh;
    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(&format!("{label}_vertices")),
        contents: bytemuck::cast_slice(vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some(&format!("{label}_indices")),
        contents: bytemuck::cast_slice(indices),
        usage: wgpu::BufferUsages::INDEX,
    });
    BuiltPrimitiveMesh {
        vertex_buffer,
        index_buffer,
        index_count: indices.len() as u32,
    }
}

fn upload_mesh(
    device: &wgpu::Device,
    mesh: &TriangleMesh,
    skin_layout: &wgpu::BindGroupLayout,
) -> GpuMesh {
    let vertices: Vec<Vertex> = mesh
        .positions
        .iter()
        .zip(mesh.normals.iter())
        .zip(mesh.texcoords.iter())
        .enumerate()
        .map(|(index, ((position, normal), texcoord))| Vertex {
            position: *position,
            normal: *normal,
            texcoord: *texcoord,
            joints: mesh
                .skinning
                .as_ref()
                .and_then(|skinning| skinning.joints.get(index).copied())
                .unwrap_or([0; 4]),
            weights: mesh
                .skinning
                .as_ref()
                .and_then(|skinning| skinning.weights.get(index).copied())
                .unwrap_or([0.0; 4]),
        })
        .collect();

    let use_u32 = mesh.indices.len() > u16::MAX as usize;
    let (index_bytes, index_format, index_count) = if use_u32 {
        (
            bytemuck::cast_slice(&mesh.indices).to_vec(),
            wgpu::IndexFormat::Uint32,
            mesh.indices.len() as u32,
        )
    } else {
        let indices_u16: Vec<u16> = mesh.indices.iter().map(|index| *index as u16).collect();
        (
            bytemuck::cast_slice(&indices_u16).to_vec(),
            wgpu::IndexFormat::Uint16,
            indices_u16.len() as u32,
        )
    };

    let vertex_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("rne_mesh_vertices"),
        contents: bytemuck::cast_slice(&vertices),
        usage: wgpu::BufferUsages::VERTEX,
    });
    let index_buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("rne_mesh_indices"),
        contents: &index_bytes,
        usage: wgpu::BufferUsages::INDEX,
    });

    GpuMesh {
        vertex_buffer,
        index_buffer,
        index_count,
        index_format,
        skin: mesh
            .skinning
            .as_ref()
            .map(|skinning| upload_skinning(device, skin_layout, skinning)),
    }
}

fn upload_skinning(
    device: &wgpu::Device,
    layout: &wgpu::BindGroupLayout,
    skinning: &SkinningData,
) -> GpuSkin {
    let header = SkinStorageHeader {
        mesh_transform: mat4_to_cols(skinning.mesh_transform),
    };
    let joint_matrices = skinning
        .joint_matrices
        .iter()
        .copied()
        .map(mat4_to_cols)
        .collect::<Vec<_>>();
    let mut bytes = bytemuck::bytes_of(&header).to_vec();
    bytes.extend_from_slice(bytemuck::cast_slice(&joint_matrices));
    let buffer = device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
        label: Some("rne_skin_storage"),
        contents: &bytes,
        usage: wgpu::BufferUsages::STORAGE | wgpu::BufferUsages::COPY_DST,
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("rne_skin_bind_group"),
        layout,
        entries: &[wgpu::BindGroupEntry {
            binding: 0,
            resource: buffer.as_entire_binding(),
        }],
    });
    GpuSkin {
        _buffer: buffer,
        bind_group,
    }
}

fn upload_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    layout: &wgpu::BindGroupLayout,
    label: &str,
    image: &ImageFrame,
    format: wgpu::TextureFormat,
) -> GpuTexture {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width: image.width.max(1),
            height: image.height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &image.rgba8,
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(image.width * 4),
            rows_per_image: Some(image.height),
        },
        wgpu::Extent3d {
            width: image.width,
            height: image.height,
            depth_or_array_layers: 1,
        },
    );
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some(label),
        address_mode_u: wgpu::AddressMode::Repeat,
        address_mode_v: wgpu::AddressMode::Repeat,
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::Sampler(&sampler),
            },
        ],
    });
    GpuTexture {
        _texture: texture,
        bind_group,
    }
}

fn upload_environment_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    layout: &wgpu::BindGroupLayout,
    label: &str,
    map: &EnvironmentMap,
) -> GpuEnvironmentTexture {
    let prefiltered = prefilter_environment(map);
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(label),
        size: wgpu::Extent3d {
            width: map.width,
            height: map.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba32Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        bytemuck::cast_slice(&map.rgba32f),
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(map.width * 4 * std::mem::size_of::<f32>() as u32),
            rows_per_image: Some(map.height),
        },
        wgpu::Extent3d {
            width: map.width,
            height: map.height,
            depth_or_array_layers: 1,
        },
    );

    let prefiltered_base = &prefiltered.specular_levels[0];
    let prefiltered_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(&format!("{label}_specular_prefiltered")),
        size: wgpu::Extent3d {
            width: prefiltered_base.width,
            height: prefiltered_base.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: SPECULAR_MIP_LEVELS,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba32Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    for (level_index, level) in prefiltered.specular_levels.iter().enumerate() {
        queue.write_texture(
            wgpu::TexelCopyTextureInfo {
                texture: &prefiltered_texture,
                mip_level: level_index as u32,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            bytemuck::cast_slice(&level.rgba32f),
            wgpu::TexelCopyBufferLayout {
                offset: 0,
                bytes_per_row: Some(level.width * 4 * std::mem::size_of::<f32>() as u32),
                rows_per_image: Some(level.height),
            },
            wgpu::Extent3d {
                width: level.width,
                height: level.height,
                depth_or_array_layers: 1,
            },
        );
    }

    let diffuse_texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some(&format!("{label}_diffuse_prefiltered")),
        size: wgpu::Extent3d {
            width: prefiltered.diffuse.width,
            height: prefiltered.diffuse.height,
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Rgba32Float,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        wgpu::TexelCopyTextureInfo {
            texture: &diffuse_texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        bytemuck::cast_slice(&prefiltered.diffuse.rgba32f),
        wgpu::TexelCopyBufferLayout {
            offset: 0,
            bytes_per_row: Some(prefiltered.diffuse.width * 4 * std::mem::size_of::<f32>() as u32),
            rows_per_image: Some(prefiltered.diffuse.height),
        },
        wgpu::Extent3d {
            width: prefiltered.diffuse.width,
            height: prefiltered.diffuse.height,
            depth_or_array_layers: 1,
        },
    );

    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    let prefiltered_view = prefiltered_texture.create_view(&wgpu::TextureViewDescriptor::default());
    let diffuse_view = diffuse_texture.create_view(&wgpu::TextureViewDescriptor::default());
    let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some(label),
        layout,
        entries: &[
            wgpu::BindGroupEntry {
                binding: 0,
                resource: wgpu::BindingResource::TextureView(&view),
            },
            wgpu::BindGroupEntry {
                binding: 1,
                resource: wgpu::BindingResource::TextureView(&prefiltered_view),
            },
            wgpu::BindGroupEntry {
                binding: 2,
                resource: wgpu::BindingResource::TextureView(&diffuse_view),
            },
        ],
    });
    GpuEnvironmentTexture {
        _texture: texture,
        _prefiltered_texture: prefiltered_texture,
        _diffuse_texture: diffuse_texture,
        bind_group,
    }
}

fn map_color_buffer(
    device: &wgpu::Device,
    buffer: &wgpu::Buffer,
    target: RenderTarget,
) -> Result<ImageFrame, RenderError> {
    let bytes_per_row = align_to(target.width * 4, wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
    let rgba8 = map_buffer_to_vec(buffer, device, target, bytes_per_row)?;
    Ok(ImageFrame::from_rgba8(target.width, target.height, rgba8))
}

fn map_depth_buffer(
    device: &wgpu::Device,
    buffer: &wgpu::Buffer,
    target: RenderTarget,
    camera: &Camera,
) -> Result<DepthFrame, RenderError> {
    let bytes_per_row = align_to(target.width * 4, wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
    let raw = map_buffer_to_vec(buffer, device, target, bytes_per_row)?;
    let mut depth_m = Vec::with_capacity((target.width * target.height) as usize);
    for chunk in raw.chunks_exact(4) {
        let z = f32::from_le_bytes([chunk[0], chunk[1], chunk[2], chunk[3]]);
        depth_m.push(linearize_depth(
            z,
            camera.near_m as f32,
            camera.far_m as f32,
        ));
    }
    Ok(DepthFrame::new(target.width, target.height, depth_m))
}

fn map_buffer_to_vec(
    buffer: &wgpu::Buffer,
    device: &wgpu::Device,
    target: RenderTarget,
    bytes_per_row: u32,
) -> Result<Vec<u8>, RenderError> {
    let slice = buffer.slice(..);
    let (sender, receiver) = std::sync::mpsc::channel();
    slice.map_async(wgpu::MapMode::Read, move |result| {
        let _ = sender.send(result);
    });
    device.poll(wgpu::Maintain::Wait);
    receiver
        .recv()
        .map_err(|_| RenderError::RenderFailed("readback channel closed".into()))?
        .map_err(|error| RenderError::RenderFailed(error.to_string()))?;

    let mapped = slice.get_mapped_range();
    let mut bytes = vec![0_u8; target.rgba8_len()];
    for y in 0..target.height as usize {
        let src_start = y * bytes_per_row as usize;
        let dst_start = y * target.width as usize * 4;
        let row_len = target.width as usize * 4;
        bytes[dst_start..dst_start + row_len]
            .copy_from_slice(&mapped[src_start..src_start + row_len]);
    }
    drop(mapped);
    buffer.unmap();
    Ok(bytes)
}

fn linearize_depth(depth: f32, near: f32, far: f32) -> f32 {
    if depth >= 1.0 {
        return far;
    }
    (near * far) / (far - depth * (far - near))
}

fn directional_light_view_projection(scene: &RenderScene) -> Mat4 {
    let (bounds_min, bounds_max) =
        scene_world_bounds(scene).unwrap_or((Vec3::splat(-10.0), Vec3::splat(10.0)));
    let center = (bounds_min + bounds_max) * 0.5;
    let radius_m = ((bounds_max - bounds_min).length() * 0.5).max(1.0);
    let light_direction = Vec3::new(
        f64::from(DEFAULT_LIGHT_DIR[0]),
        f64::from(DEFAULT_LIGHT_DIR[1]),
        f64::from(DEFAULT_LIGHT_DIR[2]),
    )
    .normalize();
    let light_eye = center + light_direction * (radius_m * 2.0 + 20.0);
    let light_view = Mat4::look_at_rh(light_eye, center, Vec3::Z);

    let mut light_min = Vec3::splat(f64::INFINITY);
    let mut light_max = Vec3::splat(f64::NEG_INFINITY);
    for corner in aabb_corners(bounds_min, bounds_max) {
        let light_position = light_view.transform_point3(corner);
        light_min = light_min.min(light_position);
        light_max = light_max.max(light_position);
    }

    let width_m = (light_max.x - light_min.x + SHADOW_BOUNDS_MARGIN_M * 2.0).max(1.0);
    let height_m = (light_max.y - light_min.y + SHADOW_BOUNDS_MARGIN_M * 2.0).max(1.0);
    let texel_x_m = width_m / f64::from(SHADOW_MAP_SIZE);
    let texel_y_m = height_m / f64::from(SHADOW_MAP_SIZE);
    let center_x = (((light_min.x + light_max.x) * 0.5) / texel_x_m).round() * texel_x_m;
    let center_y = (((light_min.y + light_max.y) * 0.5) / texel_y_m).round() * texel_y_m;
    let left = center_x - width_m * 0.5;
    let right = center_x + width_m * 0.5;
    let bottom = center_y - height_m * 0.5;
    let top = center_y + height_m * 0.5;
    let near = (-light_max.z - SHADOW_BOUNDS_MARGIN_M).max(0.1);
    let far = (-light_min.z + SHADOW_BOUNDS_MARGIN_M).max(near + 1.0);

    Mat4::orthographic_rh(left, right, bottom, top, near, far) * light_view
}

fn scene_world_bounds(scene: &RenderScene) -> Option<(Vec3, Vec3)> {
    let mut bounds_min = Vec3::splat(f64::INFINITY);
    let mut bounds_max = Vec3::splat(f64::NEG_INFINITY);
    let mut has_position = false;

    for item in &scene.items {
        let model = item.transform.to_matrix();
        if let Some(mesh) = &item.mesh {
            for position in &mesh.positions {
                let world = model.transform_point3(Vec3::new(
                    f64::from(position[0]),
                    f64::from(position[1]),
                    f64::from(position[2]),
                ));
                if world.is_finite() {
                    bounds_min = bounds_min.min(world);
                    bounds_max = bounds_max.max(world);
                    has_position = true;
                }
            }
        } else {
            for corner in aabb_corners(Vec3::splat(-0.5), Vec3::splat(0.5)) {
                let world = model.transform_point3(corner);
                if world.is_finite() {
                    bounds_min = bounds_min.min(world);
                    bounds_max = bounds_max.max(world);
                    has_position = true;
                }
            }
        }
    }

    has_position.then_some((bounds_min, bounds_max))
}

fn aabb_corners(min: Vec3, max: Vec3) -> [Vec3; 8] {
    [
        Vec3::new(min.x, min.y, min.z),
        Vec3::new(max.x, min.y, min.z),
        Vec3::new(min.x, max.y, min.z),
        Vec3::new(max.x, max.y, min.z),
        Vec3::new(min.x, min.y, max.z),
        Vec3::new(max.x, min.y, max.z),
        Vec3::new(min.x, max.y, max.z),
        Vec3::new(max.x, max.y, max.z),
    ]
}

fn mat4_to_cols(matrix: Mat4) -> [[f32; 4]; 4] {
    let cols = matrix.to_cols_array_2d();
    [
        [
            cols[0][0] as f32,
            cols[0][1] as f32,
            cols[0][2] as f32,
            cols[0][3] as f32,
        ],
        [
            cols[1][0] as f32,
            cols[1][1] as f32,
            cols[1][2] as f32,
            cols[1][3] as f32,
        ],
        [
            cols[2][0] as f32,
            cols[2][1] as f32,
            cols[2][2] as f32,
            cols[2][3] as f32,
        ],
        [
            cols[3][0] as f32,
            cols[3][1] as f32,
            cols[3][2] as f32,
            cols[3][3] as f32,
        ],
    ]
}

fn scene_temporal_key(scene: &RenderScene) -> u64 {
    const OFFSET: u64 = 0xcbf2_9ce4_8422_2325;
    const PRIME: u64 = 0x0000_0001_0000_01b3;
    let mut hash = OFFSET;
    let mut mix = |value: u64| {
        hash ^= value;
        hash = hash.wrapping_mul(PRIME);
    };

    mix(scene.items.len() as u64);
    for item in &scene.items {
        for value in item.transform.to_matrix().to_cols_array() {
            mix(value.to_bits());
        }
        for value in item.color_rgba {
            mix(u64::from(value.to_bits()));
        }
        mix(u64::from(item.material.roughness.to_bits()));
        mix(u64::from(item.material.metallic.to_bits()));
        for value in item.material.emissive_rgb {
            mix(u64::from(value.to_bits()));
        }
    }
    hash
}

fn normal_matrix_cols(model: Mat4) -> [[f32; 4]; 3] {
    let cols = model.inverse().transpose().to_cols_array_2d();
    [
        [cols[0][0] as f32, cols[0][1] as f32, cols[0][2] as f32, 0.0],
        [cols[1][0] as f32, cols[1][1] as f32, cols[1][2] as f32, 0.0],
        [cols[2][0] as f32, cols[2][1] as f32, cols[2][2] as f32, 0.0],
    ]
}

fn align_to(value: u32, alignment: u32) -> u32 {
    value.div_ceil(alignment) * alignment
}

// Sizes the per-draw dynamic-offset uniform buffer (stride x count, ~256 KB at
// 1024). The Sanjo capture's static tile + 100-vehicle fleet + colormapped point
// cloud overlay legitimately exceeds the previous 512.
const MAX_SCENE_ITEMS: u32 = 1_024;
const SHADOW_MAP_SIZE: u32 = 2_048;
const SHADOW_BOUNDS_MARGIN_M: f64 = 3.0;

fn uniform_stride(alignment: u32) -> u32 {
    align_to(std::mem::size_of::<DrawUniform>() as u32, alignment)
}

fn unit_cube() -> (Vec<Vertex>, Vec<u16>) {
    let p = [
        [-0.5, -0.5, -0.5],
        [0.5, -0.5, -0.5],
        [0.5, 0.5, -0.5],
        [-0.5, 0.5, -0.5],
        [-0.5, -0.5, 0.5],
        [0.5, -0.5, 0.5],
        [0.5, 0.5, 0.5],
        [-0.5, 0.5, 0.5],
    ];
    let faces: [([usize; 4], [f32; 3]); 6] = [
        ([0, 1, 2, 3], [0.0, 0.0, -1.0]),
        ([4, 5, 6, 7], [0.0, 0.0, 1.0]),
        ([4, 0, 3, 7], [-1.0, 0.0, 0.0]),
        ([1, 5, 6, 2], [1.0, 0.0, 0.0]),
        ([3, 2, 6, 7], [0.0, 1.0, 0.0]),
        ([4, 5, 1, 0], [0.0, -1.0, 0.0]),
    ];

    let mut vertices = Vec::new();
    let mut indices = Vec::new();
    for (face, normal) in faces {
        let base = vertices.len() as u16;
        for corner in face {
            vertices.push(Vertex {
                position: p[corner],
                normal,
                texcoord: [0.0, 0.0],
                joints: [0; 4],
                weights: [0.0; 4],
            });
        }
        indices.extend_from_slice(&[base, base + 1, base + 2, base, base + 2, base + 3]);
    }

    (vertices, indices)
}

/// Unit cylinder aligned with +Z, radius 0.5, height 1.0 centered at the origin.
fn unit_cylinder() -> (Vec<Vertex>, Vec<u16>) {
    const SEGMENTS: usize = 24;
    let mut vertices = Vec::with_capacity(SEGMENTS * 2 + 2);
    let mut indices = Vec::new();

    for ring in [-0.5_f32, 0.5] {
        for segment in 0..SEGMENTS {
            let angle = std::f32::consts::TAU * segment as f32 / SEGMENTS as f32;
            let x = angle.cos() * 0.5;
            let y = angle.sin() * 0.5;
            vertices.push(Vertex {
                position: [x, y, ring],
                normal: [angle.cos(), angle.sin(), 0.0],
                texcoord: [0.0, 0.0],
                joints: [0; 4],
                weights: [0.0; 4],
            });
        }
    }

    for segment in 0..SEGMENTS {
        let next = (segment + 1) % SEGMENTS;
        let bottom = segment as u16;
        let bottom_next = next as u16;
        let top = (SEGMENTS + segment) as u16;
        let top_next = (SEGMENTS + next) as u16;
        indices.extend_from_slice(&[bottom, top, bottom_next, bottom_next, top, top_next]);
    }

    let bottom_center = vertices.len() as u16;
    vertices.push(Vertex {
        position: [0.0, 0.0, -0.5],
        normal: [0.0, 0.0, -1.0],
        texcoord: [0.0, 0.0],
        joints: [0; 4],
        weights: [0.0; 4],
    });
    let top_center = vertices.len() as u16;
    vertices.push(Vertex {
        position: [0.0, 0.0, 0.5],
        normal: [0.0, 0.0, 1.0],
        texcoord: [0.0, 0.0],
        joints: [0; 4],
        weights: [0.0; 4],
    });

    for segment in 0..SEGMENTS {
        let next = (segment + 1) % SEGMENTS;
        indices.extend_from_slice(&[bottom_center, next as u16, segment as u16]);
        indices.extend_from_slice(&[
            top_center,
            (SEGMENTS + segment) as u16,
            (SEGMENTS + next) as u16,
        ]);
    }

    (vertices, indices)
}

/// Unit sphere with radius 0.5 centered at the origin.
fn unit_sphere() -> (Vec<Vertex>, Vec<u16>) {
    const RINGS: usize = 16;
    const SEGMENTS: usize = 24;
    let mut vertices = Vec::with_capacity((RINGS + 1) * (SEGMENTS + 1));
    let mut indices = Vec::new();

    for ring in 0..=RINGS {
        let v = ring as f32 / RINGS as f32;
        let phi = v * std::f32::consts::PI;
        let y = phi.cos();
        let ring_radius = phi.sin();
        for segment in 0..=SEGMENTS {
            let u = segment as f32 / SEGMENTS as f32;
            let theta = u * std::f32::consts::TAU;
            let x = ring_radius * theta.cos();
            let z = ring_radius * theta.sin();
            let normal = [x, y, z];
            vertices.push(Vertex {
                position: [x * 0.5, y * 0.5, z * 0.5],
                normal,
                texcoord: [0.0, 0.0],
                joints: [0; 4],
                weights: [0.0; 4],
            });
        }
    }

    let stride = SEGMENTS + 1;
    for ring in 0..RINGS {
        for segment in 0..SEGMENTS {
            let current = (ring * stride + segment) as u16;
            let next = current + 1;
            let below = current + stride as u16;
            let below_next = below + 1;
            indices.extend_from_slice(&[current, below, next, next, below, below_next]);
        }
    }

    (vertices, indices)
}

#[cfg(test)]
mod mesh_tests {
    use super::{
        directional_light_view_projection, unit_cube, unit_cylinder, unit_sphere, RenderScene,
        Transform3, Vec3, VisualShape, SHADER, SHADOW_SHADER, SKY_SHADER,
    };
    use crate::taa::{COPY_SHADER, TAA_SHADER};
    use rne_render::RenderSceneItem;

    #[test]
    fn shaders_validate_without_gpu() {
        for (label, source) in [
            ("primitive", SHADER),
            ("shadow", SHADOW_SHADER),
            ("sky", SKY_SHADER),
            ("taa", TAA_SHADER),
            ("taa-present", COPY_SHADER),
        ] {
            let module = naga::front::wgsl::parse_str(source)
                .unwrap_or_else(|error| panic!("{label} WGSL parse failed: {error}"));
            let mut validator = naga::valid::Validator::new(
                naga::valid::ValidationFlags::all(),
                naga::valid::Capabilities::all(),
            );
            validator
                .validate(&module)
                .unwrap_or_else(|error| panic!("{label} WGSL validation failed: {error}"));
        }
    }

    #[test]
    fn primitive_meshes_have_triangles() {
        for mesh in [unit_cube(), unit_cylinder(), unit_sphere()] {
            assert!(!mesh.0.is_empty());
            assert!(mesh.1.len() >= 3);
        }
    }

    #[test]
    fn directional_shadow_projection_contains_scene_bounds() {
        let item = RenderSceneItem {
            transform: Transform3 {
                translation: Vec3::new(12.0, 4.0, -8.0),
                scale: Vec3::new(6.0, 8.0, 10.0),
                ..Transform3::IDENTITY
            },
            shape: VisualShape::Box { size_m: Vec3::ONE },
            color_rgba: [1.0; 4],
            mesh: None,
            base_color_texture: None,
            material: Default::default(),
        };
        let scene = RenderScene { items: vec![item] };
        let projection = directional_light_view_projection(&scene);

        assert!(projection.is_finite());
        assert_eq!(
            projection,
            directional_light_view_projection(&scene),
            "the fitted projection must be deterministic"
        );
        for corner in super::aabb_corners(Vec3::splat(-0.5), Vec3::splat(0.5)) {
            let world = scene.items[0]
                .transform
                .to_matrix()
                .transform_point3(corner);
            let clip = projection.transform_point3(world);
            assert!((-1.0..=1.0).contains(&clip.x));
            assert!((-1.0..=1.0).contains(&clip.y));
            assert!((0.0..=1.0).contains(&clip.z));
        }
    }
}

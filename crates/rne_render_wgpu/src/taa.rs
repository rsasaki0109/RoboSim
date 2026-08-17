//! Temporal anti-aliasing for the WGPU render path.

use bytemuck::{Pod, Zeroable};
use rne_math::Mat4;

pub(crate) const PACKED_DEPTH_FORMAT: wgpu::TextureFormat = wgpu::TextureFormat::Rgba8Unorm;
pub(crate) const PACKED_DEPTH_CLEAR: wgpu::Color = wgpu::Color {
    r: 0.0,
    g: 0.0,
    b: 128.0 / 255.0,
    a: 63.0 / 255.0,
};

pub(crate) const TAA_SHADER: &str = r#"
struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    var out: VertexOutput;
    let positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    let uvs = array<vec2<f32>, 3>(
        vec2<f32>(0.0, 1.0),
        vec2<f32>(2.0, 1.0),
        vec2<f32>(0.0, -1.0),
    );
    out.position = vec4<f32>(positions[vertex_index], 0.0, 1.0);
    out.uv = uvs[vertex_index];
    return out;
}

struct TaaUniform {
    current_inv_view_proj: mat4x4<f32>,
    previous_view_proj: mat4x4<f32>,
    settings: vec4<f32>,
    resolution: vec4<f32>,
}

@group(0) @binding(0) var current_color: texture_2d<f32>;
@group(0) @binding(1) var history_color: texture_2d<f32>;
@group(0) @binding(2) var current_depth: texture_2d<f32>;
@group(0) @binding(3) var color_sampler: sampler;
@group(0) @binding(4) var<uniform> taa: TaaUniform;

fn sample_current(uv: vec2<f32>) -> vec4<f32> {
    return textureSampleLevel(current_color, color_sampler, clamp(uv, vec2<f32>(0.0), vec2<f32>(1.0)), 0.0);
}

fn reproject_uv(uv: vec2<f32>, depth: f32) -> vec2<f32> {
    let current_ndc = vec4<f32>(uv.x * 2.0 - 1.0, 1.0 - uv.y * 2.0, depth, 1.0);
    let world_h = taa.current_inv_view_proj * current_ndc;
    if abs(world_h.w) < 0.00001 {
        return uv;
    }
    let world = world_h / world_h.w;
    let previous_clip = taa.previous_view_proj * world;
    if previous_clip.w <= 0.00001 {
        return uv;
    }
    let previous_ndc = previous_clip.xyz / previous_clip.w;
    return vec2<f32>(previous_ndc.x * 0.5 + 0.5, 0.5 - previous_ndc.y * 0.5);
}

fn unpack_depth(packed: vec4<f32>) -> f32 {
    let bytes = vec4<u32>(round(clamp(packed, vec4<f32>(0.0), vec4<f32>(1.0)) * 255.0));
    let bits = bytes.x | (bytes.y << 8u) | (bytes.z << 16u) | (bytes.w << 24u);
    return bitcast<f32>(bits);
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    let current = sample_current(input.uv);
    if taa.settings.y < 0.5 {
        return current;
    }

    let pixel = vec2<i32>(
        i32(clamp(floor(input.position.x), 0.0, taa.resolution.x - 1.0)),
        i32(clamp(floor(input.position.y), 0.0, taa.resolution.y - 1.0)),
    );
    let depth = unpack_depth(textureLoad(current_depth, pixel, 0));
    let history_uv = reproject_uv(input.uv, depth);
    if any(history_uv < vec2<f32>(0.0)) || any(history_uv > vec2<f32>(1.0)) {
        return current;
    }

    let texel = taa.resolution.zw;
    var minimum = current.rgb;
    var maximum = current.rgb;
    for (var y: i32 = -1; y <= 1; y = y + 1) {
        for (var x: i32 = -1; x <= 1; x = x + 1) {
            let neighbor = sample_current(input.uv + vec2<f32>(f32(x), f32(y)) * texel).rgb;
            minimum = min(minimum, neighbor);
            maximum = max(maximum, neighbor);
        }
    }

    let history = textureSampleLevel(history_color, color_sampler, history_uv, 0.0);
    let clamped_history = clamp(history.rgb, minimum, maximum);
    let feedback = clamp(taa.settings.x, 0.0, 0.98);
    return vec4<f32>(mix(current.rgb, clamped_history, feedback), current.a);
}
"#;

pub(crate) const COPY_SHADER: &str = r#"
struct VertexOutput {
    @builtin(position) position: vec4<f32>,
    @location(0) uv: vec2<f32>,
}

@vertex
fn vs_main(@builtin(vertex_index) vertex_index: u32) -> VertexOutput {
    var out: VertexOutput;
    let positions = array<vec2<f32>, 3>(
        vec2<f32>(-1.0, -1.0),
        vec2<f32>(3.0, -1.0),
        vec2<f32>(-1.0, 3.0),
    );
    let uvs = array<vec2<f32>, 3>(
        vec2<f32>(0.0, 1.0),
        vec2<f32>(2.0, 1.0),
        vec2<f32>(0.0, -1.0),
    );
    out.position = vec4<f32>(positions[vertex_index], 0.0, 1.0);
    out.uv = uvs[vertex_index];
    return out;
}

@group(0) @binding(0) var source: texture_2d<f32>;
@group(0) @binding(1) var source_sampler: sampler;

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    return textureSampleLevel(source, source_sampler, input.uv, 0.0);
}
"#;

/// Opt-in temporal anti-aliasing settings for the WGPU backend.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TaaSettings {
    /// Whether temporal accumulation is enabled.
    pub enabled: bool,
    /// Fraction of the reprojected history blended into the current sample.
    pub feedback: f32,
    /// Halton-sequence camera-jitter amplitude in pixels.
    pub jitter_scale_px: f32,
}

impl Default for TaaSettings {
    fn default() -> Self {
        Self::disabled()
    }
}

impl TaaSettings {
    /// Returns disabled settings, preserving the pre-TAA render path.
    pub const fn disabled() -> Self {
        Self {
            enabled: false,
            feedback: 0.9,
            jitter_scale_px: 0.75,
        }
    }

    /// Returns conservative settings suitable for a static or slowly moving view.
    pub const fn enabled() -> Self {
        Self {
            enabled: true,
            feedback: 0.9,
            jitter_scale_px: 0.75,
        }
    }

    /// Clamps values to a finite range accepted by the shader.
    pub fn sanitized(self) -> Self {
        Self {
            enabled: self.enabled,
            feedback: finite_or(self.feedback, 0.9).clamp(0.0, 0.98),
            jitter_scale_px: finite_or(self.jitter_scale_px, 0.75).clamp(0.0, 2.0),
        }
    }
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct TaaUniform {
    current_inv_view_proj: [[f32; 4]; 4],
    previous_view_proj: [[f32; 4]; 4],
    settings: [f32; 4],
    resolution: [f32; 4],
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct TaaFrame {
    pub(crate) enabled: bool,
    pub(crate) current_view_proj: Mat4,
    previous_view_proj: Mat4,
    history_valid: bool,
    scene_key: u64,
}

impl TaaFrame {
    fn disabled(view_proj: Mat4, scene_key: u64) -> Self {
        Self {
            enabled: false,
            current_view_proj: view_proj,
            previous_view_proj: view_proj,
            history_valid: false,
            scene_key,
        }
    }
}

/// Internal GPU resources and deterministic state for temporal accumulation.
pub(crate) struct TemporalAntiAliasing {
    settings: TaaSettings,
    pipeline: wgpu::RenderPipeline,
    present_pipeline: wgpu::RenderPipeline,
    bind_group_layout: wgpu::BindGroupLayout,
    present_bind_group_layout: wgpu::BindGroupLayout,
    sampler: wgpu::Sampler,
    uniform_buffer: wgpu::Buffer,
    format: wgpu::TextureFormat,
    size: Option<(u32, u32)>,
    _scene_texture: Option<wgpu::Texture>,
    scene_view: Option<wgpu::TextureView>,
    _reprojection_depth_texture: Option<wgpu::Texture>,
    reprojection_depth_view: Option<wgpu::TextureView>,
    resolved_texture: Option<wgpu::Texture>,
    resolved_view: Option<wgpu::TextureView>,
    history_texture: Option<wgpu::Texture>,
    history_view: Option<wgpu::TextureView>,
    history_valid: bool,
    previous_view_proj: Mat4,
    previous_scene_key: u64,
    frame_index: u64,
}

impl TemporalAntiAliasing {
    pub(crate) fn new(device: &wgpu::Device, format: wgpu::TextureFormat) -> Self {
        let taa_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rne_taa_shader"),
            source: wgpu::ShaderSource::Wgsl(TAA_SHADER.into()),
        });
        let copy_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rne_taa_present_shader"),
            source: wgpu::ShaderSource::Wgsl(COPY_SHADER.into()),
        });

        let bind_group_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("rne_taa_layout"),
            entries: &[
                texture_entry(0),
                texture_entry(1),
                wgpu::BindGroupLayoutEntry {
                    binding: 2,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Texture {
                        sample_type: wgpu::TextureSampleType::Float { filterable: false },
                        view_dimension: wgpu::TextureViewDimension::D2,
                        multisampled: false,
                    },
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 3,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                    count: None,
                },
                wgpu::BindGroupLayoutEntry {
                    binding: 4,
                    visibility: wgpu::ShaderStages::FRAGMENT,
                    ty: wgpu::BindingType::Buffer {
                        ty: wgpu::BufferBindingType::Uniform,
                        has_dynamic_offset: false,
                        min_binding_size: None,
                    },
                    count: None,
                },
            ],
        });
        let present_bind_group_layout =
            device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
                label: Some("rne_taa_present_layout"),
                entries: &[
                    texture_entry(0),
                    wgpu::BindGroupLayoutEntry {
                        binding: 1,
                        visibility: wgpu::ShaderStages::FRAGMENT,
                        ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                        count: None,
                    },
                ],
            });

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("rne_taa_pipeline_layout"),
            bind_group_layouts: &[&bind_group_layout],
            push_constant_ranges: &[],
        });
        let present_pipeline_layout =
            device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
                label: Some("rne_taa_present_pipeline_layout"),
                bind_group_layouts: &[&present_bind_group_layout],
                push_constant_ranges: &[],
            });

        let pipeline = fullscreen_pipeline(
            device,
            "rne_taa_pipeline",
            &pipeline_layout,
            &taa_shader,
            format,
        );
        let present_pipeline = fullscreen_pipeline(
            device,
            "rne_taa_present_pipeline",
            &present_pipeline_layout,
            &copy_shader,
            format,
        );

        let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
            label: Some("rne_taa_sampler"),
            address_mode_u: wgpu::AddressMode::ClampToEdge,
            address_mode_v: wgpu::AddressMode::ClampToEdge,
            mag_filter: wgpu::FilterMode::Linear,
            min_filter: wgpu::FilterMode::Linear,
            mipmap_filter: wgpu::FilterMode::Nearest,
            ..Default::default()
        });
        let uniform_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_taa_uniform"),
            size: std::mem::size_of::<TaaUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });

        Self {
            settings: TaaSettings::default(),
            pipeline,
            present_pipeline,
            bind_group_layout,
            present_bind_group_layout,
            sampler,
            uniform_buffer,
            format,
            size: None,
            _scene_texture: None,
            scene_view: None,
            _reprojection_depth_texture: None,
            reprojection_depth_view: None,
            resolved_texture: None,
            resolved_view: None,
            history_texture: None,
            history_view: None,
            history_valid: false,
            previous_view_proj: Mat4::IDENTITY,
            previous_scene_key: 0,
            frame_index: 0,
        }
    }

    pub(crate) fn set_settings(&mut self, settings: TaaSettings) {
        let settings = settings.sanitized();
        if settings != self.settings {
            self.settings = settings;
            self.reset_history();
        }
    }

    pub(crate) fn reset_history(&mut self) {
        self.history_valid = false;
        self.previous_view_proj = Mat4::IDENTITY;
        self.previous_scene_key = 0;
        self.frame_index = 0;
    }

    pub(crate) fn begin_frame(
        &mut self,
        device: &wgpu::Device,
        width: u32,
        height: u32,
        view_proj: Mat4,
        scene_key: u64,
    ) -> TaaFrame {
        if !self.settings.enabled {
            self.reset_history();
            return TaaFrame::disabled(view_proj, scene_key);
        }

        self.ensure_targets(device, width.max(1), height.max(1));
        let jitter_px = halton_jitter(self.frame_index, self.settings.jitter_scale_px);
        let current_view_proj = jitter_view_projection(view_proj, jitter_px, width, height);
        let history_valid = self.history_valid && self.previous_scene_key == scene_key;
        let previous_view_proj = if history_valid {
            self.previous_view_proj
        } else {
            current_view_proj
        };
        TaaFrame {
            enabled: true,
            current_view_proj,
            previous_view_proj,
            history_valid,
            scene_key,
        }
    }

    pub(crate) fn prepare(&self, queue: &wgpu::Queue, frame: TaaFrame, width: u32, height: u32) {
        let uniform = TaaUniform {
            current_inv_view_proj: mat4_to_cols(frame.current_view_proj.inverse()),
            previous_view_proj: mat4_to_cols(frame.previous_view_proj),
            settings: [
                self.settings.feedback,
                f32::from(frame.history_valid),
                0.0,
                0.0,
            ],
            resolution: [
                width as f32,
                height as f32,
                1.0 / width.max(1) as f32,
                1.0 / height.max(1) as f32,
            ],
        };
        queue.write_buffer(&self.uniform_buffer, 0, bytemuck::bytes_of(&uniform));
    }

    pub(crate) fn scene_view(&self) -> &wgpu::TextureView {
        self.scene_view
            .as_ref()
            .expect("TAA targets must be initialized before scene_view")
    }

    pub(crate) fn reprojection_depth_view(&self) -> &wgpu::TextureView {
        self.reprojection_depth_view
            .as_ref()
            .expect("TAA targets must be initialized before reprojection_depth_view")
    }

    pub(crate) fn encode(
        &self,
        device: &wgpu::Device,
        encoder: &mut wgpu::CommandEncoder,
        reprojection_depth_view: &wgpu::TextureView,
        target_view: &wgpu::TextureView,
    ) {
        let scene_view = self
            .scene_view
            .as_ref()
            .expect("TAA scene target must be initialized");
        let resolved_view = self
            .resolved_view
            .as_ref()
            .expect("TAA resolved target must be initialized");
        let resolved_texture = self
            .resolved_texture
            .as_ref()
            .expect("TAA resolved texture must be initialized");
        let history_texture = self
            .history_texture
            .as_ref()
            .expect("TAA history texture must be initialized");
        let history_view = self
            .history_view
            .as_ref()
            .expect("TAA history target must be initialized");

        let bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_taa_bind_group"),
            layout: &self.bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(scene_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::TextureView(history_view),
                },
                wgpu::BindGroupEntry {
                    binding: 2,
                    resource: wgpu::BindingResource::TextureView(reprojection_depth_view),
                },
                wgpu::BindGroupEntry {
                    binding: 3,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
                wgpu::BindGroupEntry {
                    binding: 4,
                    resource: self.uniform_buffer.as_entire_binding(),
                },
            ],
        });

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("rne_taa_resolve_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: resolved_view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &bind_group, &[]);
            pass.draw(0..3, 0..1);
        }

        encoder.copy_texture_to_texture(
            wgpu::TexelCopyTextureInfo {
                texture: resolved_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::TexelCopyTextureInfo {
                texture: history_texture,
                mip_level: 0,
                origin: wgpu::Origin3d::ZERO,
                aspect: wgpu::TextureAspect::All,
            },
            wgpu::Extent3d {
                width: self.size.map_or(1, |size| size.0),
                height: self.size.map_or(1, |size| size.1),
                depth_or_array_layers: 1,
            },
        );

        let present_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_taa_present_bind_group"),
            layout: &self.present_bind_group_layout,
            entries: &[
                wgpu::BindGroupEntry {
                    binding: 0,
                    resource: wgpu::BindingResource::TextureView(resolved_view),
                },
                wgpu::BindGroupEntry {
                    binding: 1,
                    resource: wgpu::BindingResource::Sampler(&self.sampler),
                },
            ],
        });
        let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("rne_taa_present_pass"),
            color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                view: target_view,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Clear(wgpu::Color::BLACK),
                    store: wgpu::StoreOp::Store,
                },
            })],
            depth_stencil_attachment: None,
            timestamp_writes: None,
            occlusion_query_set: None,
        });
        pass.set_pipeline(&self.present_pipeline);
        pass.set_bind_group(0, &present_bind_group, &[]);
        pass.draw(0..3, 0..1);
    }

    pub(crate) fn commit(&mut self, frame: TaaFrame) {
        if frame.enabled {
            self.previous_view_proj = frame.current_view_proj;
            self.previous_scene_key = frame.scene_key;
            self.history_valid = true;
            self.frame_index = self.frame_index.wrapping_add(1);
        }
    }

    fn ensure_targets(&mut self, device: &wgpu::Device, width: u32, height: u32) {
        if self.size == Some((width, height)) {
            return;
        }

        let scene_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_taa_scene_texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: self.format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let reprojection_depth_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_taa_reprojection_depth_texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: PACKED_DEPTH_FORMAT,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::TEXTURE_BINDING,
            view_formats: &[],
        });
        let resolved_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_taa_resolved_texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: self.format,
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT
                | wgpu::TextureUsages::TEXTURE_BINDING
                | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let history_texture = device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_taa_history_texture"),
            size: wgpu::Extent3d {
                width,
                height,
                depth_or_array_layers: 1,
            },
            mip_level_count: 1,
            sample_count: 1,
            dimension: wgpu::TextureDimension::D2,
            format: self.format,
            usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
            view_formats: &[],
        });

        self.scene_view = Some(scene_texture.create_view(&wgpu::TextureViewDescriptor::default()));
        self.reprojection_depth_view =
            Some(reprojection_depth_texture.create_view(&wgpu::TextureViewDescriptor::default()));
        self.resolved_view =
            Some(resolved_texture.create_view(&wgpu::TextureViewDescriptor::default()));
        self.history_view =
            Some(history_texture.create_view(&wgpu::TextureViewDescriptor::default()));
        self._scene_texture = Some(scene_texture);
        self._reprojection_depth_texture = Some(reprojection_depth_texture);
        self.resolved_texture = Some(resolved_texture);
        self.history_texture = Some(history_texture);
        self.size = Some((width, height));
        self.reset_history();
    }
}

fn texture_entry(binding: u32) -> wgpu::BindGroupLayoutEntry {
    wgpu::BindGroupLayoutEntry {
        binding,
        visibility: wgpu::ShaderStages::FRAGMENT,
        ty: wgpu::BindingType::Texture {
            sample_type: wgpu::TextureSampleType::Float { filterable: true },
            view_dimension: wgpu::TextureViewDimension::D2,
            multisampled: false,
        },
        count: None,
    }
}

fn fullscreen_pipeline(
    device: &wgpu::Device,
    label: &'static str,
    layout: &wgpu::PipelineLayout,
    shader: &wgpu::ShaderModule,
    format: wgpu::TextureFormat,
) -> wgpu::RenderPipeline {
    device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some(label),
        layout: Some(layout),
        vertex: wgpu::VertexState {
            module: shader,
            entry_point: Some("vs_main"),
            buffers: &[],
            compilation_options: Default::default(),
        },
        fragment: Some(wgpu::FragmentState {
            module: shader,
            entry_point: Some("fs_main"),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: None,
                write_mask: wgpu::ColorWrites::ALL,
            })],
            compilation_options: Default::default(),
        }),
        primitive: wgpu::PrimitiveState::default(),
        depth_stencil: None,
        multisample: wgpu::MultisampleState::default(),
        multiview: None,
        cache: None,
    })
}

fn halton_jitter(index: u64, scale_px: f32) -> [f32; 2] {
    [
        (halton(index + 1, 2) - 0.5) * scale_px,
        (halton(index + 1, 3) - 0.5) * scale_px,
    ]
}

fn halton(mut index: u64, base: u64) -> f32 {
    let mut result = 0.0_f32;
    let mut fraction = 1.0_f32 / base as f32;
    while index != 0 {
        result += (index % base) as f32 * fraction;
        index /= base;
        fraction /= base as f32;
    }
    result
}

fn jitter_view_projection(view_proj: Mat4, jitter_px: [f32; 2], width: u32, height: u32) -> Mat4 {
    let mut jittered = view_proj;
    jittered.w_axis.x += 2.0 * f64::from(jitter_px[0]) / width.max(1) as f64;
    jittered.w_axis.y -= 2.0 * f64::from(jitter_px[1]) / height.max(1) as f64;
    jittered
}

fn mat4_to_cols(matrix: Mat4) -> [[f32; 4]; 4] {
    matrix
        .to_cols_array_2d()
        .map(|column| column.map(|value| value as f32))
}

fn finite_or(value: f32, fallback: f32) -> f32 {
    if value.is_finite() {
        value
    } else {
        fallback
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn default_taa_is_disabled() {
        assert_eq!(TaaSettings::default(), TaaSettings::disabled());
    }

    #[test]
    fn taa_settings_are_shader_safe() {
        let settings = TaaSettings {
            enabled: true,
            feedback: f32::NAN,
            jitter_scale_px: f32::INFINITY,
        }
        .sanitized();
        assert_eq!(settings.feedback, 0.9);
        assert_eq!(settings.jitter_scale_px, 0.75);
    }

    #[test]
    fn taa_depth_reprojection_unpacks_a_portable_color_attachment() {
        assert!(TAA_SHADER.contains("var current_depth: texture_2d<f32>"));
        assert!(TAA_SHADER.contains("unpack_depth(textureLoad(current_depth, pixel, 0))"));
        assert!(!TAA_SHADER.contains("texture_depth_2d"));
    }

    #[test]
    fn halton_jitter_is_deterministic_and_bounded() {
        let first = halton_jitter(0, 0.75);
        assert_eq!(first, halton_jitter(0, 0.75));
        for index in 0..32 {
            let jitter = halton_jitter(index, 0.75);
            assert!(jitter
                .into_iter()
                .all(|value| (-0.375..=0.375).contains(&value)));
        }
    }

    #[test]
    fn jitter_changes_clip_translation_without_changing_projection_depth() {
        let projection = Mat4::perspective_rh(0.8, 1.5, 0.1, 100.0);
        let jittered = jitter_view_projection(projection, [0.5, -0.25], 100, 80);
        assert!((jittered.w_axis.x - projection.w_axis.x - 0.01).abs() < 1e-12);
        assert!((jittered.w_axis.y - projection.w_axis.y - 0.00625).abs() < 1e-12);
        assert_eq!(jittered.z_axis, projection.z_axis);
    }
}

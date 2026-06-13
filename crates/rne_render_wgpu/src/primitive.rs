const SHADER: &str = r#"
struct CameraUniform {
    view_proj: mat4x4<f32>,
}

struct DrawUniform {
    model: mat4x4<f32>,
    color: vec4<f32>,
}

@group(0) @binding(0) var<uniform> camera: CameraUniform;
@group(1) @binding(0) var<uniform> draw: DrawUniform;

struct VertexOutput {
    @builtin(position) clip_position: vec4<f32>,
    @location(0) color: vec4<f32>,
}

@vertex
fn vs_main(
    @location(0) position: vec3<f32>,
    @location(1) normal: vec3<f32>,
) -> VertexOutput {
    _ = normal;
    var out: VertexOutput;
    let world = draw.model * vec4<f32>(position, 1.0);
    out.clip_position = camera.view_proj * world;
    out.color = draw.color;
    return out;
}

@fragment
fn fs_main(input: VertexOutput) -> @location(0) vec4<f32> {
    return input.color;
}
"#;

use bytemuck::{Pod, Zeroable};
use rne_math::Mat4;
use rne_math::Transform3;
use rne_render::{
    Camera, CameraPassOutput, DepthFrame, ImageFrame, RenderError, RenderScene, RenderTarget,
    TriangleMesh, VisualShape,
};
use std::collections::HashMap;
use std::sync::Arc;
use wgpu::util::DeviceExt;

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct Vertex {
    position: [f32; 3],
    normal: [f32; 3],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct CameraUniform {
    view_proj: [[f32; 4]; 4],
}

#[repr(C)]
#[derive(Clone, Copy, Pod, Zeroable)]
struct DrawUniform {
    model: [[f32; 4]; 4],
    color: [f32; 4],
}

struct BuiltPrimitiveMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
}

pub struct PrimitiveRenderer {
    pipeline: wgpu::RenderPipeline,
    camera_layout: wgpu::BindGroupLayout,
    draw_bind_group: wgpu::BindGroup,
    draw_uniform_stride: u32,
    box_mesh: BuiltPrimitiveMesh,
    sphere_mesh: BuiltPrimitiveMesh,
    cylinder_mesh: BuiltPrimitiveMesh,
    camera_buffer: wgpu::Buffer,
    draw_buffer: wgpu::Buffer,
    mesh_cache: HashMap<usize, GpuMesh>,
}

struct GpuMesh {
    vertex_buffer: wgpu::Buffer,
    index_buffer: wgpu::Buffer,
    index_count: u32,
    index_format: wgpu::IndexFormat,
}

/// Color and depth views for an on-screen or off-screen render pass.
pub struct PrimitiveRenderViews<'a> {
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
    /// Clear color for empty pixels.
    pub clear_color: [f32; 4],
}

impl PrimitiveRenderer {
    pub fn new(device: &wgpu::Device, color_format: wgpu::TextureFormat) -> Self {
        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("rne_primitive_shader"),
            source: wgpu::ShaderSource::Wgsl(SHADER.into()),
        });

        let camera_layout = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("rne_camera_layout"),
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

        let pipeline_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("rne_primitive_pipeline_layout"),
            bind_group_layouts: &[&camera_layout, &draw_layout],
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

        let box_mesh = upload_primitive(device, "rne_box", &unit_cube());
        let sphere_mesh = upload_primitive(device, "rne_sphere", &unit_sphere());
        let cylinder_mesh = upload_primitive(device, "rne_cylinder", &unit_cylinder());
        let camera_buffer = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_camera_uniform"),
            size: std::mem::size_of::<CameraUniform>() as wgpu::BufferAddress,
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

        Self {
            pipeline,
            camera_layout,
            draw_bind_group,
            draw_uniform_stride,
            box_mesh,
            sphere_mesh,
            cylinder_mesh,
            camera_buffer,
            draw_buffer,
            mesh_cache: HashMap::new(),
        }
    }

    fn primitive_mesh_for(&self, shape: &VisualShape) -> &BuiltPrimitiveMesh {
        match shape {
            VisualShape::Sphere { .. } => &self.sphere_mesh,
            VisualShape::Cylinder { .. } => &self.cylinder_mesh,
            VisualShape::Box { .. } | VisualShape::Mesh { .. } => &self.box_mesh,
        }
    }

    /// Renders a scene into existing color and depth views without CPU readback.
    pub fn render_to_views(&mut self, pass: PrimitiveSurfacePass<'_>) -> Result<(), RenderError> {
        let device = pass.device;
        let queue = pass.queue;
        let camera = pass.camera;
        let view = pass.view;
        let scene = pass.scene;
        let clear_color = pass.clear_color;
        let targets = pass.targets;
        let view_proj = camera.view_projection(view);
        queue.write_buffer(
            &self.camera_buffer,
            0,
            bytemuck::bytes_of(&CameraUniform {
                view_proj: mat4_to_cols(view_proj),
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
            let uniform = DrawUniform {
                model: mat4_to_cols(item.transform.to_matrix()),
                color: item.color_rgba,
            };
            let offset = index * self.draw_uniform_stride as usize;
            draw_bytes[offset..offset + std::mem::size_of::<DrawUniform>()]
                .copy_from_slice(bytemuck::bytes_of(&uniform));
        }
        queue.write_buffer(&self.draw_buffer, 0, &draw_bytes);

        let camera_bind_group = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("rne_camera_bind_group"),
            layout: &self.camera_layout,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: self.camera_buffer.as_entire_binding(),
            }],
        });

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("rne_scene_encoder"),
        });

        {
            let mut pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("rne_scene_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: targets.color_view,
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

            pass.set_pipeline(&self.pipeline);
            pass.set_bind_group(0, &camera_bind_group, &[]);

            for (index, item) in scene.items.iter().enumerate() {
                pass.set_bind_group(
                    1,
                    &self.draw_bind_group,
                    &[index as u32 * self.draw_uniform_stride],
                );

                if let Some(mesh) = &item.mesh {
                    let gpu_mesh = self.gpu_mesh(device, mesh);
                    pass.set_vertex_buffer(0, gpu_mesh.vertex_buffer.slice(..));
                    pass.set_index_buffer(gpu_mesh.index_buffer.slice(..), gpu_mesh.index_format);
                    pass.draw_indexed(0..gpu_mesh.index_count, 0, 0..1);
                } else {
                    let primitive = self.primitive_mesh_for(&item.shape);
                    pass.set_vertex_buffer(0, primitive.vertex_buffer.slice(..));
                    pass.set_index_buffer(
                        primitive.index_buffer.slice(..),
                        wgpu::IndexFormat::Uint16,
                    );
                    pass.draw_indexed(0..primitive.index_count, 0, 0..1);
                }
            }
        }

        queue.submit(Some(encoder.finish()));
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
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
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
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT | wgpu::TextureUsages::COPY_SRC,
            view_formats: &[],
        });
        let depth_view = depth_texture.create_view(&wgpu::TextureViewDescriptor::default());

        self.render_to_views(PrimitiveSurfacePass {
            device,
            queue,
            camera,
            view,
            scene,
            clear_color,
            targets: &PrimitiveRenderViews {
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
        self.mesh_cache
            .entry(key)
            .or_insert_with(|| upload_mesh(device, mesh))
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

fn upload_mesh(device: &wgpu::Device, mesh: &TriangleMesh) -> GpuMesh {
    let vertices: Vec<Vertex> = mesh
        .positions
        .iter()
        .zip(mesh.normals.iter())
        .map(|(position, normal)| Vertex {
            position: *position,
            normal: *normal,
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

fn align_to(value: u32, alignment: u32) -> u32 {
    value.div_ceil(alignment) * alignment
}

const MAX_SCENE_ITEMS: u32 = 256;

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
    });
    let top_center = vertices.len() as u16;
    vertices.push(Vertex {
        position: [0.0, 0.0, 0.5],
        normal: [0.0, 0.0, 1.0],
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
    use super::{unit_cube, unit_cylinder, unit_sphere};

    #[test]
    fn primitive_meshes_have_triangles() {
        for mesh in [unit_cube(), unit_cylinder(), unit_sphere()] {
            assert!(!mesh.0.is_empty());
            assert!(mesh.1.len() >= 3);
        }
    }
}

//! wgpu render backend implementation.

use crate::primitive::{PrimitiveRenderPass, PrimitiveRenderer};
use pollster::block_on;
use rne_math::Transform3;
use rne_render::{
    Camera, CameraPassOutput, ImageFrame, RenderBackend, RenderError, RenderScene, RenderTarget,
};

/// wgpu-backed renderer with off-screen render targets.
pub struct WgpuRenderBackend {
    device: wgpu::Device,
    queue: wgpu::Queue,
    primitive: Option<PrimitiveRenderer>,
}

impl WgpuRenderBackend {
    /// Creates a renderer using the default GPU adapter.
    pub fn new() -> Result<Self, RenderError> {
        Self::with_backends(wgpu::Backends::all())
    }

    /// Creates a renderer restricted to the given backend mask.
    pub fn with_backends(backends: wgpu::Backends) -> Result<Self, RenderError> {
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends,
            ..Default::default()
        });

        let adapter = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::LowPower,
            compatible_surface: None,
            force_fallback_adapter: false,
        }))
        .ok_or(RenderError::NoAdapter)?;

        let (device, queue) = block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("rne_render_wgpu"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
            },
            None,
        ))
        .map_err(|error| RenderError::InitFailed(error.to_string()))?;

        Ok(Self {
            device,
            queue,
            primitive: None,
        })
    }

    fn render_clear_inner(
        &self,
        target: RenderTarget,
        clear_color: [f32; 4],
    ) -> Result<ImageFrame, RenderError> {
        let texture = self.device.create_texture(&wgpu::TextureDescriptor {
            label: Some("rne_clear_target"),
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

        let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut encoder = self
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor {
                label: Some("rne_clear_encoder"),
            });

        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("rne_clear_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
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
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
        }

        read_color_texture(&self.device, &self.queue, &texture, target)
    }
}

impl RenderBackend for WgpuRenderBackend {
    fn render_clear(
        &mut self,
        target: RenderTarget,
        clear_color: [f32; 4],
    ) -> Result<ImageFrame, RenderError> {
        self.render_clear_inner(target, clear_color)
    }

    fn render_scene_camera(
        &mut self,
        camera: &Camera,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
    ) -> Result<CameraPassOutput, RenderError> {
        if self.primitive.is_none() {
            self.primitive = Some(PrimitiveRenderer::new(
                &self.device,
                wgpu::TextureFormat::Rgba8UnormSrgb,
            ));
        }
        let renderer = self.primitive.as_mut().expect("primitive renderer");
        renderer.render(PrimitiveRenderPass {
            device: &self.device,
            queue: &self.queue,
            target: camera.render_target(),
            camera,
            view,
            scene,
            clear_color,
        })
    }
}

fn read_color_texture(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    texture: &wgpu::Texture,
    target: RenderTarget,
) -> Result<ImageFrame, RenderError> {
    let bytes_per_row = align_to(target.width * 4, wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
    let buffer_size = bytes_per_row as u64 * target.height as u64;
    let buffer = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("rne_readback"),
        size: buffer_size,
        usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
        mapped_at_creation: false,
    });

    let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
        label: Some("rne_clear_readback_encoder"),
    });
    encoder.copy_texture_to_buffer(
        wgpu::TexelCopyTextureInfo {
            texture,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        wgpu::TexelCopyBufferInfo {
            buffer: &buffer,
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
    map_rgba8_buffer(device, &buffer, target, bytes_per_row)
}

fn map_rgba8_buffer(
    device: &wgpu::Device,
    buffer: &wgpu::Buffer,
    target: RenderTarget,
    bytes_per_row: u32,
) -> Result<ImageFrame, RenderError> {
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
    let mut rgba8 = vec![0_u8; target.rgba8_len()];
    for y in 0..target.height as usize {
        let src_start = y * bytes_per_row as usize;
        let dst_start = y * target.width as usize * 4;
        let row_len = target.width as usize * 4;
        rgba8[dst_start..dst_start + row_len]
            .copy_from_slice(&mapped[src_start..src_start + row_len]);
    }
    drop(mapped);
    buffer.unmap();

    Ok(ImageFrame::from_rgba8(target.width, target.height, rgba8))
}

fn align_to(value: u32, alignment: u32) -> u32 {
    value.div_ceil(alignment) * alignment
}

#[cfg(test)]
fn unique_colors(rgba8: &[u8]) -> usize {
    use std::collections::HashSet;
    rgba8
        .chunks_exact(4)
        .map(|px| (px[0], px[1], px[2], px[3]))
        .collect::<HashSet<_>>()
        .len()
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_math::{Quat, Vec3};
    use rne_render::{hash_depth_f32, hash_rgba8, RenderScene, RenderSceneItem, VisualShape};
    use std::path::PathBuf;
    use std::sync::Arc;

    #[test]
    fn wgpu_clear_render_produces_image() {
        if std::env::var("RNE_SKIP_GPU").is_ok() {
            return;
        }

        let mut backend = match WgpuRenderBackend::new() {
            Ok(backend) => backend,
            Err(RenderError::NoAdapter) => return,
            Err(error) => panic!("{error}"),
        };

        let frame = backend
            .render_clear(RenderTarget::new(8, 8), [0.1, 0.2, 0.3, 1.0])
            .expect("clear render");

        assert_eq!(frame.width, 8);
        assert_eq!(frame.height, 8);
        assert_eq!(frame.rgba8.len(), 8 * 8 * 4);
        assert_ne!(hash_rgba8(&frame.rgba8), 0);
    }

    #[test]
    fn wgpu_scene_render_produces_color_and_depth() {
        if std::env::var("RNE_SKIP_GPU").is_ok() {
            return;
        }

        let mut backend = match WgpuRenderBackend::new() {
            Ok(backend) => backend,
            Err(RenderError::NoAdapter) => return,
            Err(error) => panic!("{error}"),
        };

        let camera = Camera::new(64, 48, std::f64::consts::FRAC_PI_4);
        let view = Transform3::from_translation_rotation(Vec3::new(0.0, 1.0, 3.0), Quat::IDENTITY);
        let scene = RenderScene {
            items: vec![RenderSceneItem {
                transform: Transform3 {
                    translation: Vec3::new(0.0, 0.25, 0.0),
                    rotation: Quat::IDENTITY,
                    scale: Vec3::new(0.5, 0.3, 0.4),
                },
                shape: VisualShape::Box {
                    size_m: Vec3::new(0.5, 0.3, 0.4),
                },
                color_rgba: [0.8, 0.2, 0.2, 1.0],
                mesh: None,
            }],
        };

        let output = backend
            .render_scene_camera(&camera, &view, &scene, [0.05, 0.08, 0.12, 1.0])
            .expect("scene render");

        let min_depth = output
            .depth
            .depth_m
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min);
        let center = (output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize;
        eprintln!(
            "unit scene: color_hash={:#018x} min_depth={min_depth:.3} center_depth={:.3} unique_colors={}",
            hash_rgba8(&output.color.rgba8),
            output.depth.depth_m[center],
            unique_colors(&output.color.rgba8)
        );

        assert_ne!(hash_rgba8(&output.color.rgba8), 0);
        assert_ne!(hash_depth_f32(&output.depth.depth_m), 0);
        assert!(output
            .depth
            .depth_m
            .iter()
            .any(|depth| *depth < camera.far_m as f32));
    }

    #[test]
    fn wgpu_mesh_scene_render_produces_depth() {
        if std::env::var("RNE_SKIP_GPU").is_ok() {
            return;
        }

        let mut backend = match WgpuRenderBackend::new() {
            Ok(backend) => backend,
            Err(RenderError::NoAdapter) => return,
            Err(error) => panic!("{error}"),
        };

        let package_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_render/tests/fixtures/mesh_diff_drive");
        let mesh = Arc::new(
            rne_render::load_stl(&package_root.join("meshes/base_link.stl")).expect("load stl"),
        );
        let scene = RenderScene {
            items: vec![RenderSceneItem {
                transform: Transform3 {
                    translation: Vec3::new(0.0, 0.25, 0.0),
                    rotation: Quat::IDENTITY,
                    scale: Vec3::ONE,
                },
                shape: VisualShape::Mesh {
                    path: "package://mesh_diff_drive/meshes/base_link.stl".into(),
                    scale: Vec3::ONE,
                },
                color_rgba: [0.35, 0.55, 0.95, 1.0],
                mesh: Some(mesh),
            }],
        };

        let camera = Camera::new(64, 48, std::f64::consts::FRAC_PI_4);
        let view = Transform3::from_translation_rotation(Vec3::new(0.0, 1.5, 4.0), Quat::IDENTITY);
        let output = backend
            .render_scene_camera(&camera, &view, &scene, [0.05, 0.08, 0.12, 1.0])
            .expect("mesh scene render");

        let min_depth = output
            .depth
            .depth_m
            .iter()
            .copied()
            .fold(f32::INFINITY, f32::min);
        let center = (output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize;
        eprintln!(
            "mesh scene: color_hash={:#018x} min_depth={min_depth:.3} center_depth={:.3} unique_colors={}",
            hash_rgba8(&output.color.rgba8),
            output.depth.depth_m[center],
            unique_colors(&output.color.rgba8)
        );

        assert_ne!(hash_rgba8(&output.color.rgba8), 0);
        assert_ne!(hash_depth_f32(&output.depth.depth_m), 0);
    }

    #[test]
    fn wgpu_urdf_primitive_scene_renders_visible_geometry() {
        if std::env::var("RNE_SKIP_GPU").is_ok() {
            return;
        }

        let mut backend = match WgpuRenderBackend::new() {
            Ok(backend) => backend,
            Err(RenderError::NoAdapter) => return,
            Err(error) => panic!("{error}"),
        };

        let xml = include_str!(
            "../../../adapters/ros2/rne_urdf_import/tests/fixtures/minimal_diff_drive.urdf"
        );
        let urdf = rne_urdf_import::parse_urdf(xml).expect("parse URDF");
        let mut world = rne_ecs::World::new();
        let spawned = rne_urdf_import::spawn_urdf_robot(&mut world, &urdf).expect("spawn URDF");

        let mut scene = RenderScene::new();
        for entity in spawned.links.values() {
            let Some(visual) = world.get::<rne_render::Visual>(*entity).cloned() else {
                continue;
            };
            let world_transform = world
                .get::<rne_world::Transform3>(*entity)
                .copied()
                .unwrap_or_default();
            scene.items.push(RenderScene::item_from_visual(
                world_transform,
                visual.shape,
                visual.color_rgba,
                visual.local_offset,
            ));
        }

        let camera = Camera::new(128, 96, std::f64::consts::FRAC_PI_4);
        let view = Transform3::from_translation_rotation(Vec3::new(0.0, 1.5, 4.0), Quat::IDENTITY);
        let output = backend
            .render_scene_camera(&camera, &view, &scene, [0.05, 0.08, 0.12, 1.0])
            .expect("urdf scene render");

        let center = (output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize;
        let center_depth = output.depth.depth_m[center];
        eprintln!(
            "urdf scene: color_hash={:#018x} center_depth={center_depth:.3} unique_colors={}",
            hash_rgba8(&output.color.rgba8),
            unique_colors(&output.color.rgba8)
        );

        assert!(
            center_depth < camera.far_m as f32,
            "expected geometry in center pixel, got depth {center_depth}"
        );
        assert!(
            unique_colors(&output.color.rgba8) > 4,
            "expected shaded scene colors, got only clear color"
        );
    }

    #[test]
    fn wgpu_orbit_camera_renders_focused_box() {
        if std::env::var("RNE_SKIP_GPU").is_ok() {
            return;
        }

        let mut backend = match WgpuRenderBackend::new() {
            Ok(backend) => backend,
            Err(RenderError::NoAdapter) => return,
            Err(error) => panic!("{error}"),
        };

        let focus = Vec3::new(0.5, 0.25, 0.0);
        let orbit = crate::CameraOrbit {
            focus,
            ..crate::CameraOrbit::default()
        };
        let scene = RenderScene {
            items: vec![RenderSceneItem {
                transform: Transform3 {
                    translation: focus,
                    rotation: Quat::IDENTITY,
                    scale: Vec3::new(0.5, 0.3, 0.4),
                },
                shape: VisualShape::Box {
                    size_m: Vec3::new(0.5, 0.3, 0.4),
                },
                color_rgba: [0.8, 0.2, 0.2, 1.0],
                mesh: None,
            }],
        };

        let camera = Camera::new(320, 240, std::f64::consts::FRAC_PI_4);
        let view = orbit.camera_transform();
        let output = backend
            .render_scene_camera(&camera, &view, &scene, [0.05, 0.08, 0.12, 1.0])
            .expect("orbit render");

        let center = (output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize;
        eprintln!(
            "orbit box: color_hash={:#018x} center_depth={:.3} unique_colors={}",
            hash_rgba8(&output.color.rgba8),
            output.depth.depth_m[center],
            unique_colors(&output.color.rgba8)
        );

        assert!(
            output.depth.depth_m[center] < camera.far_m as f32,
            "orbit camera should render focused box in center"
        );
    }

    #[test]
    fn wgpu_orbit_camera_renders_focused_mesh() {
        if std::env::var("RNE_SKIP_GPU").is_ok() {
            return;
        }

        let mut backend = match WgpuRenderBackend::new() {
            Ok(backend) => backend,
            Err(RenderError::NoAdapter) => return,
            Err(error) => panic!("{error}"),
        };

        let package_root = PathBuf::from(env!("CARGO_MANIFEST_DIR"))
            .join("../rne_render/tests/fixtures/mesh_diff_drive");
        let mesh = Arc::new(
            rne_render::load_stl(&package_root.join("meshes/base_link.stl")).expect("load stl"),
        );
        let focus = Vec3::new(0.5, 0.25, 0.0);
        let orbit = crate::CameraOrbit {
            focus,
            ..crate::CameraOrbit::default()
        };
        let scene = RenderScene {
            items: vec![RenderSceneItem {
                transform: Transform3 {
                    translation: focus,
                    rotation: Quat::IDENTITY,
                    scale: Vec3::ONE,
                },
                shape: VisualShape::Mesh {
                    path: "package://mesh_diff_drive/meshes/base_link.stl".into(),
                    scale: Vec3::ONE,
                },
                color_rgba: [0.35, 0.55, 0.95, 1.0],
                mesh: Some(mesh),
            }],
        };

        let camera = Camera::new(320, 240, std::f64::consts::FRAC_PI_4);
        let view = orbit.camera_transform();
        let output = backend
            .render_scene_camera(&camera, &view, &scene, [0.05, 0.08, 0.12, 1.0])
            .expect("orbit mesh render");

        let center = (output.depth.height / 2 * output.depth.width + output.depth.width / 2) as usize;
        eprintln!(
            "orbit mesh: color_hash={:#018x} center_depth={:.3} unique_colors={}",
            hash_rgba8(&output.color.rgba8),
            output.depth.depth_m[center],
            unique_colors(&output.color.rgba8)
        );

        assert!(
            output.depth.depth_m[center] < camera.far_m as f32,
            "orbit camera should render focused mesh in center"
        );
    }
}

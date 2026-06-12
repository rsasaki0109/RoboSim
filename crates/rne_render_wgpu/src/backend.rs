//! wgpu render backend implementation.

use pollster::block_on;
use rne_render::{ImageFrame, RenderBackend, RenderError, RenderTarget};

/// wgpu-backed renderer with off-screen render targets.
pub struct WgpuRenderBackend {
    device: wgpu::Device,
    queue: wgpu::Queue,
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

        Ok(Self { device, queue })
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

        let bytes_per_row = align_to(target.width * 4, wgpu::COPY_BYTES_PER_ROW_ALIGNMENT);
        let buffer_size = bytes_per_row as u64 * target.height as u64;
        let buffer = self.device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("rne_readback"),
            size: buffer_size,
            usage: wgpu::BufferUsages::COPY_DST | wgpu::BufferUsages::MAP_READ,
            mapped_at_creation: false,
        });

        encoder.copy_texture_to_buffer(
            wgpu::TexelCopyTextureInfo {
                texture: &texture,
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

        self.queue.submit(Some(encoder.finish()));

        let slice = buffer.slice(..);
        let (sender, receiver) = std::sync::mpsc::channel();
        slice.map_async(wgpu::MapMode::Read, move |result| {
            let _ = sender.send(result);
        });
        self.device.poll(wgpu::Maintain::Wait);

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
}

impl RenderBackend for WgpuRenderBackend {
    fn render_clear(
        &mut self,
        target: RenderTarget,
        clear_color: [f32; 4],
    ) -> Result<ImageFrame, RenderError> {
        self.render_clear_inner(target, clear_color)
    }
}

fn align_to(value: u32, alignment: u32) -> u32 {
    value.div_ceil(alignment) * alignment
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_render::hash_rgba8;

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
}

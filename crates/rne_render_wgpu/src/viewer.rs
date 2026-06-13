//! Interactive winit window backed by wgpu.

use crate::primitive::{PrimitiveRenderViews, PrimitiveRenderer, PrimitiveSurfacePass};
use pollster::block_on;
use rne_math::{Quat, Transform3, Vec3};
use rne_render::{Camera, RenderError, RenderScene};
use std::sync::Arc;
use thiserror::Error;
use winit::window::Window;

/// Errors while creating or presenting an interactive viewer.
#[derive(Debug, Error)]
pub enum ViewerError {
    /// No compatible GPU adapter was found.
    #[error("no compatible GPU adapter")]
    NoAdapter,
    /// GPU device initialization failed.
    #[error("GPU init failed: {0}")]
    InitFailed(String),
    /// Surface or swapchain operation failed.
    #[error("surface error: {0}")]
    Surface(String),
}

impl From<RenderError> for ViewerError {
    fn from(error: RenderError) -> Self {
        match error {
            RenderError::NoAdapter => Self::NoAdapter,
            RenderError::InitFailed(message) => Self::InitFailed(message),
            other => Self::Surface(other.to_string()),
        }
    }
}

/// Orbit camera around a world-space focus point.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraOrbit {
    /// Horizontal orbit angle in radians.
    pub yaw_rad: f64,
    /// Vertical orbit angle in radians.
    pub pitch_rad: f64,
    /// Distance from the focus point in meters.
    pub distance_m: f64,
    /// Point the camera looks at.
    pub focus: Vec3,
}

impl Default for CameraOrbit {
    fn default() -> Self {
        Self {
            yaw_rad: 0.0,
            pitch_rad: 0.55,
            distance_m: 4.0,
            focus: Vec3::ZERO,
        }
    }
}

impl CameraOrbit {
    /// Builds a camera world transform looking at the focus point.
    pub fn camera_transform(&self) -> Transform3 {
        let pitch = self.pitch_rad.clamp(0.15, 1.45);
        let yaw = self.yaw_rad;
        let horizontal = self.distance_m * pitch.sin();
        let eye = Vec3::new(
            self.focus.x + horizontal * yaw.sin(),
            self.focus.y + self.distance_m * pitch.cos(),
            self.focus.z + horizontal * yaw.cos(),
        );
        let forward = (self.focus - eye).normalize_or_zero();
        let rotation = if forward.length_squared() > f64::EPSILON {
            Quat::from_rotation_arc(-Vec3::Z, forward)
        } else {
            Quat::IDENTITY
        };

        Transform3::from_translation_rotation(eye, rotation)
    }
}

/// winit window with a wgpu swapchain for interactive scene rendering.
pub struct InteractiveViewer {
    window: Arc<Window>,
    surface: wgpu::Surface<'static>,
    device: wgpu::Device,
    queue: wgpu::Queue,
    config: wgpu::SurfaceConfiguration,
    primitive: PrimitiveRenderer,
    depth_texture: wgpu::Texture,
    depth_view: wgpu::TextureView,
}

impl InteractiveViewer {
    /// Creates a viewer bound to an existing winit window.
    pub fn new(window: Arc<Window>) -> Result<Self, ViewerError> {
        let size = window.inner_size();
        let width = size.width.max(1);
        let height = size.height.max(1);

        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor {
            backends: wgpu::Backends::all(),
            ..Default::default()
        });
        let surface = instance
            .create_surface(window.clone())
            .map_err(|error| ViewerError::Surface(error.to_string()))?;

        let adapter = block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
            power_preference: wgpu::PowerPreference::HighPerformance,
            compatible_surface: Some(&surface),
            force_fallback_adapter: false,
        }))
        .ok_or(ViewerError::NoAdapter)?;

        let (device, queue) = block_on(adapter.request_device(
            &wgpu::DeviceDescriptor {
                label: Some("rne_interactive_viewer"),
                required_features: wgpu::Features::empty(),
                required_limits: wgpu::Limits::default(),
                memory_hints: wgpu::MemoryHints::Performance,
            },
            None,
        ))
        .map_err(|error| ViewerError::InitFailed(error.to_string()))?;

        let surface_caps = surface.get_capabilities(&adapter);
        let format = surface_caps
            .formats
            .iter()
            .copied()
            .find(wgpu::TextureFormat::is_srgb)
            .unwrap_or(surface_caps.formats[0]);

        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            width,
            height,
            present_mode: wgpu::PresentMode::Fifo,
            alpha_mode: surface_caps.alpha_modes[0],
            view_formats: vec![],
            desired_maximum_frame_latency: 2,
        };
        surface.configure(&device, &config);

        let primitive = PrimitiveRenderer::new(&device, format);
        let (depth_texture, depth_view) = create_depth_target(&device, width, height);

        Ok(Self {
            window,
            surface,
            device,
            queue,
            config,
            primitive,
            depth_texture,
            depth_view,
        })
    }

    /// Returns the underlying winit window.
    pub fn window(&self) -> &Window {
        &self.window
    }

    /// Returns the current drawable size in pixels.
    pub fn size(&self) -> (u32, u32) {
        (self.config.width, self.config.height)
    }

    /// Builds a pinhole camera matching the window size.
    pub fn camera(&self) -> Camera {
        Camera::new(
            self.config.width,
            self.config.height,
            std::f64::consts::FRAC_PI_4,
        )
    }

    /// Resizes the swapchain and depth target.
    pub fn resize(&mut self, width: u32, height: u32) {
        if width == 0 || height == 0 {
            return;
        }

        self.config.width = width;
        self.config.height = height;
        self.surface.configure(&self.device, &self.config);

        let (depth_texture, depth_view) = create_depth_target(&self.device, width, height);
        self.depth_texture = depth_texture;
        self.depth_view = depth_view;
    }

    /// Renders a scene to the window and presents the frame.
    pub fn render(
        &mut self,
        view: &Transform3,
        scene: &RenderScene,
        clear_color: [f32; 4],
    ) -> Result<(), ViewerError> {
        let output = self
            .surface
            .get_current_texture()
            .map_err(|error| ViewerError::Surface(error.to_string()))?;
        let color_view = output
            .texture
            .create_view(&wgpu::TextureViewDescriptor::default());

        let camera = self.camera();
        self.primitive
            .render_to_views(PrimitiveSurfacePass {
                device: &self.device,
                queue: &self.queue,
                camera: &camera,
                view,
                scene,
                clear_color,
                targets: &PrimitiveRenderViews {
                    color_view: &color_view,
                    depth_view: &self.depth_view,
                },
            })
            .map_err(ViewerError::from)?;

        output.present();
        Ok(())
    }
}

fn create_depth_target(
    device: &wgpu::Device,
    width: u32,
    height: u32,
) -> (wgpu::Texture, wgpu::TextureView) {
    let texture = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("rne_viewer_depth"),
        size: wgpu::Extent3d {
            width: width.max(1),
            height: height.max(1),
            depth_or_array_layers: 1,
        },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::Depth32Float,
        usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
        view_formats: &[],
    });
    let view = texture.create_view(&wgpu::TextureViewDescriptor::default());
    (texture, view)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn camera_orbit_produces_finite_transform() {
        let orbit = CameraOrbit {
            yaw_rad: 0.7,
            pitch_rad: 0.4,
            distance_m: 3.0,
            focus: Vec3::new(1.0, 0.25, 0.0),
        };
        let transform = orbit.camera_transform();
        assert!(transform.translation.x.is_finite());
        assert!(transform.translation.y.is_finite());
        assert!(transform.translation.z.is_finite());
    }
}

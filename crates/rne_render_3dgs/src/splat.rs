//! Gaussian splat background loading and hybrid compositing.

use crate::camera::RneSplatCamera;
use glam::UVec2;
use rne_math::Transform3;
use rne_render::{
    Camera, CameraPassOutput, GaussianSplatEnvironment, HybridRenderScene, RenderError,
};
use rne_render_wgpu::{BackgroundRenderPass, WgpuRenderBackend};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use thiserror::Error;
use wgpu_3dgs_viewer::{Gaussians, Viewer};

/// Error while loading or rendering a Gaussian splat background.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum GaussianSplatError {
    /// The PLY file could not be opened or parsed.
    #[error("failed to load Gaussian splat PLY {path}: {message}")]
    Ply {
        /// Source path.
        path: String,
        /// Error text.
        message: String,
    },
    /// GPU initialization for the splat viewer failed.
    #[error("failed to initialize Gaussian splat viewer: {0}")]
    Init(String),
}

/// GPU-resident Gaussian splat background for hybrid compositing.
pub struct GaussianSplatBackground {
    viewer: Viewer,
    environment: GaussianSplatEnvironment,
}

impl GaussianSplatBackground {
    /// Loads a splat cloud from an environment manifest entry.
    pub fn from_environment(
        device: &wgpu::Device,
        environment: &GaussianSplatEnvironment,
    ) -> Result<Self, GaussianSplatError> {
        const MIN_STORAGE_BUFFERS_PER_COMPUTE_STAGE: u32 = 9;
        let limits = device.limits();
        if limits.max_storage_buffers_per_shader_stage < MIN_STORAGE_BUFFERS_PER_COMPUTE_STAGE {
            return Err(GaussianSplatError::Init(format!(
                "adapter exposes {} storage buffers per compute stage; 3DGS requires at least {MIN_STORAGE_BUFFERS_PER_COMPUTE_STAGE}",
                limits.max_storage_buffers_per_shader_stage
            )));
        }
        let gaussians = load_gaussians(&environment.ply_path)?;
        let viewer = Viewer::new(device, wgpu::TextureFormat::Rgba8UnormSrgb, &gaussians)
            .map_err(|error| GaussianSplatError::Init(error.to_string()))?;
        Ok(Self {
            viewer,
            environment: environment.clone(),
        })
    }

    /// Stable renderer identity from the manifest.
    #[must_use]
    pub fn renderer_identity(&self) -> &str {
        &self.environment.renderer_identity
    }

    /// Environment identifier from the manifest.
    #[must_use]
    pub fn environment_id(&self) -> &str {
        &self.environment.environment_id
    }
}

impl BackgroundRenderPass for GaussianSplatBackground {
    fn render_background(
        &mut self,
        device: &wgpu::Device,
        queue: &wgpu::Queue,
        color_view: &wgpu::TextureView,
        camera: &Camera,
        view: &Transform3,
        clear_color: [f32; 4],
    ) -> Result<(), RenderError> {
        let _ = self.environment.transform;
        let splat_camera = RneSplatCamera::from_rne(camera, view);
        splat_camera.upload(
            &mut self.viewer,
            queue,
            UVec2::new(camera.width, camera.height),
        );

        let mut encoder = device.create_command_encoder(&wgpu::CommandEncoderDescriptor {
            label: Some("rne_gaussian_splat_background_encoder"),
        });
        {
            let _pass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("rne_gaussian_splat_clear_pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: color_view,
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
        self.viewer.render(&mut encoder, color_view);
        queue.submit(Some(encoder.finish()));
        Ok(())
    }
}

/// Loads a [`GaussianSplatBackground`] from a validated environment manifest.
pub fn load_gaussian_splat_background(
    device: &wgpu::Device,
    environment: &GaussianSplatEnvironment,
) -> Result<GaussianSplatBackground, GaussianSplatError> {
    GaussianSplatBackground::from_environment(device, environment)
}

/// Renders a hybrid splat background plus mesh foreground capture.
pub fn render_hybrid_scene_camera(
    backend: &mut WgpuRenderBackend,
    background: &mut GaussianSplatBackground,
    camera: &Camera,
    view: &Transform3,
    hybrid: &HybridRenderScene,
    clear_color: [f32; 4],
) -> Result<CameraPassOutput, RenderError> {
    backend.render_hybrid_scene_camera(background, camera, view, &hybrid.foreground, clear_color)
}

fn load_gaussians(path: &Path) -> Result<Gaussians, GaussianSplatError> {
    let file = File::open(path).map_err(|error| GaussianSplatError::Ply {
        path: path.display().to_string(),
        message: error.to_string(),
    })?;
    let mut reader = BufReader::new(file);
    Gaussians::read_ply(&mut reader).map_err(|error| GaussianSplatError::Ply {
        path: path.display().to_string(),
        message: error.to_string(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use rne_render::validate_gaussian_splat_manifest;

    #[test]
    fn tsukuba_fixture_ply_parses_for_viewer() {
        let environment = validate_gaussian_splat_manifest(
            &rne_render::tsukuba_confirmation_splat_manifest_path(),
        )
        .expect("manifest");
        load_gaussians(&environment.ply_path).expect("fixture ply");
    }
}

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
        // The viewer owns Gaussians in the PLY's reconstruction-local frame.
        // Move the world-space camera into that frame so the manifest transform
        // affects the colour pass exactly as it does the proxy-depth pass.
        let splat_view = environment_local_camera(view, &self.environment.transform);
        let splat_camera = RneSplatCamera::from_rne(camera, &splat_view);
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

fn environment_local_camera(world_camera: &Transform3, environment: &Transform3) -> Transform3 {
    let mut local_camera = environment.inverse().mul_transform(world_camera);
    // Camera view construction intentionally ignores scale. Keeping it at one
    // also makes the returned value an ordinary pose rather than an affine
    // environment transform.
    local_camera.scale = rne_math::Vec3::ONE;
    local_camera
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
    use rne_math::{Quat, Vec3};
    use rne_render::validate_gaussian_splat_manifest;

    #[test]
    fn environment_transform_moves_camera_into_ply_frame() {
        let environment = Transform3 {
            translation: Vec3::new(4.0, 2.0, -3.0),
            rotation: Quat::from_rotation_y(std::f64::consts::FRAC_PI_2),
            scale: Vec3::splat(0.5),
        };
        let local_expected = Transform3::from_translation_rotation(
            Vec3::new(2.0, 1.0, -5.0),
            Quat::from_rotation_x(-0.3),
        );
        let world_camera = environment.mul_transform(&local_expected);

        let actual = environment_local_camera(&world_camera, &environment);

        assert!((actual.translation - local_expected.translation).length() < 1.0e-12);
        assert!(actual
            .rotation
            .abs_diff_eq(local_expected.rotation, 1.0e-12));
        assert_eq!(actual.scale, Vec3::ONE);
    }

    #[test]
    fn tsukuba_fixture_ply_parses_for_viewer() {
        let environment = validate_gaussian_splat_manifest(
            &rne_render::tsukuba_confirmation_splat_manifest_path(),
        )
        .expect("manifest");
        load_gaussians(&environment.ply_path).expect("fixture ply");
    }
}

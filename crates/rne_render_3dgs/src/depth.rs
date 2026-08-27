//! CPU alpha-composited proxy depth for hybrid RGB-D capture spikes.
//!
//! Full volumetric splat depth is not available from the color-only
//! `wgpu-3dgs-viewer` pass. This module projects anisotropic Gaussian covariance
//! into screen space and front-to-back alpha-composites linear depth so dataset
//! captures can fill background depths where the mesh pass only sees empty space.

use crate::GaussianSplatError;
use glam::{Mat4, Vec2, Vec3, Vec4Swizzles};
use rne_math::Transform3;
use rne_render::{Camera, DepthFrame};
use std::fs::File;
use std::io::BufReader;
use std::path::Path;
use wgpu_3dgs_viewer::{Gaussian, Gaussians};

/// Loads Gaussian means from a PLY and rasterizes a nearest-depth proxy frame.
pub fn splat_proxy_depth_from_ply(
    ply_path: &Path,
    camera: &Camera,
    view: &Transform3,
    environment_transform: &Transform3,
) -> Result<DepthFrame, GaussianSplatError> {
    let gaussians = load_gaussians(ply_path)?;
    Ok(splat_proxy_depth_from_gaussians(
        &gaussians.gaussians,
        camera,
        view,
        environment_transform,
    ))
}

/// Alpha-composites projected anisotropic Gaussians into a linear depth buffer.
#[must_use]
pub fn splat_proxy_depth_from_gaussians(
    gaussians: &[Gaussian],
    camera: &Camera,
    view: &Transform3,
    environment_transform: &Transform3,
) -> DepthFrame {
    let width = camera.width.max(1);
    let height = camera.height.max(1);
    let far = camera.far_m as f32;
    let near = camera.near_m as f32;
    let pixel_count = (width * height) as usize;

    let view_matrix = Camera::view_matrix(view).as_mat4();
    let projection = camera.projection_matrix().as_mat4();
    let env = transform_to_mat4(environment_transform);
    let projection_context = ProjectionContext {
        environment: env,
        view: view_matrix,
        projection,
        width,
        height,
    };

    let mut projected = Vec::with_capacity(gaussians.len());
    for (source_index, gaussian) in gaussians.iter().enumerate() {
        if f32::from(gaussian.color.w) < 8.0 {
            continue;
        }
        let world = projection_context
            .environment
            .transform_point3(gaussian.pos);
        let view_pos = view_matrix.transform_point3(world);
        let depth = -view_pos.z;
        if !(depth.is_finite() && depth >= near && depth <= far) {
            continue;
        }
        let clip = projection * view_pos.extend(1.0);
        if clip.w.abs() < 1.0e-6 {
            continue;
        }
        let ndc = clip.xyz() / clip.w;
        if ndc.x.abs() > 1.0 || ndc.y.abs() > 1.0 || ndc.z < 0.0 || ndc.z > 1.0 {
            continue;
        }
        let center_px = ndc_to_pixel(ndc, width, height);
        let covariance_px =
            projected_covariance_px(gaussian, world, center_px, &projection_context);
        projected.push(ProjectedGaussian {
            source_index,
            center_px,
            covariance_px,
            depth,
            opacity: f32::from(gaussian.color.w) / 255.0,
        });
    }
    projected.sort_by(|left, right| {
        left.depth
            .total_cmp(&right.depth)
            .then(left.source_index.cmp(&right.source_index))
    });
    let mut accumulated_weight = vec![0.0_f32; pixel_count];
    let mut weighted_depth = vec![0.0_f32; pixel_count];
    let mut transmittance = vec![1.0_f32; pixel_count];
    for gaussian in projected {
        stamp_alpha_composited_depth(
            &mut accumulated_weight,
            &mut weighted_depth,
            &mut transmittance,
            width,
            height,
            &gaussian,
        );
    }
    let depth_m = accumulated_weight
        .into_iter()
        .zip(weighted_depth)
        .map(|(weight, weighted)| {
            if weight >= 0.10 {
                weighted / weight
            } else {
                far
            }
        })
        .collect();
    DepthFrame::new(width, height, depth_m)
}

#[derive(Clone, Copy, Debug)]
struct ProjectedGaussian {
    source_index: usize,
    center_px: Vec2,
    covariance_px: [f32; 3],
    depth: f32,
    opacity: f32,
}

#[derive(Clone, Copy, Debug)]
struct ProjectionContext {
    environment: Mat4,
    view: Mat4,
    projection: Mat4,
    width: u32,
    height: u32,
}

/// Composites mesh and splat proxy depths with nearest-surface wins.
#[must_use]
pub fn composite_mesh_and_splat_depth(
    mesh: &DepthFrame,
    splat: &DepthFrame,
    far_m: f32,
) -> DepthFrame {
    assert_eq!(mesh.width, splat.width);
    assert_eq!(mesh.height, splat.height);
    assert_eq!(mesh.depth_m.len(), splat.depth_m.len());
    let depth_m = mesh
        .depth_m
        .iter()
        .zip(splat.depth_m.iter())
        .map(|(mesh_depth, splat_depth)| {
            let mesh_hit = mesh_depth.is_finite() && *mesh_depth < far_m * 0.999;
            let splat_hit = splat_depth.is_finite() && *splat_depth < far_m * 0.999;
            match (mesh_hit, splat_hit) {
                (true, true) => mesh_depth.min(*splat_depth),
                (true, false) => *mesh_depth,
                (false, true) => *splat_depth,
                (false, false) => far_m,
            }
        })
        .collect();
    DepthFrame::new(mesh.width, mesh.height, depth_m)
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

fn transform_to_mat4(transform: &Transform3) -> Mat4 {
    transform.to_matrix().as_mat4()
}

fn ndc_to_pixel(ndc: Vec3, width: u32, height: u32) -> Vec2 {
    Vec2::new(
        (ndc.x * 0.5 + 0.5) * width as f32,
        (1.0 - (ndc.y * 0.5 + 0.5)) * height as f32,
    )
}

fn project_world_to_pixel(
    world: Vec3,
    view: &Mat4,
    projection: &Mat4,
    width: u32,
    height: u32,
) -> Option<Vec2> {
    let view_pos = view.transform_point3(world);
    let clip = *projection * view_pos.extend(1.0);
    if clip.w.abs() < 1.0e-6 {
        return None;
    }
    Some(ndc_to_pixel(clip.xyz() / clip.w, width, height))
}

fn projected_covariance_px(
    gaussian: &Gaussian,
    center_world: Vec3,
    center_px: Vec2,
    context: &ProjectionContext,
) -> [f32; 3] {
    let rotation = glam::Mat3::from_quat(gaussian.rotation);
    let mut covariance_xx = 0.1;
    let mut covariance_xy = 0.0;
    let mut covariance_yy = 0.1;
    for source_axis in [
        rotation.x_axis * gaussian.scale.x,
        rotation.y_axis * gaussian.scale.y,
        rotation.z_axis * gaussian.scale.z,
    ] {
        let world_axis = context.environment.transform_vector3(source_axis);
        let Some(endpoint_px) = project_world_to_pixel(
            center_world + world_axis,
            &context.view,
            &context.projection,
            context.width,
            context.height,
        ) else {
            continue;
        };
        let screen_axis = endpoint_px - center_px;
        covariance_xx += screen_axis.x * screen_axis.x;
        covariance_xy += screen_axis.x * screen_axis.y;
        covariance_yy += screen_axis.y * screen_axis.y;
    }
    [covariance_xx, covariance_xy, covariance_yy]
}

fn stamp_alpha_composited_depth(
    accumulated_weight: &mut [f32],
    weighted_depth: &mut [f32],
    transmittance: &mut [f32],
    width: u32,
    height: u32,
    gaussian: &ProjectedGaussian,
) {
    let [covariance_xx, covariance_xy, covariance_yy] = gaussian.covariance_px;
    let determinant = covariance_xx * covariance_yy - covariance_xy * covariance_xy;
    if !(determinant.is_finite() && determinant > 1.0e-8) {
        return;
    }
    let inverse_xx = covariance_yy / determinant;
    let inverse_xy = -covariance_xy / determinant;
    let inverse_yy = covariance_xx / determinant;
    let radius_x = (2.0 * covariance_xx.sqrt()).ceil().clamp(1.0, 16.0) as i32;
    let radius_y = (2.0 * covariance_yy.sqrt()).ceil().clamp(1.0, 16.0) as i32;
    let center_x = gaussian.center_px.x.floor() as i32;
    let center_y = gaussian.center_px.y.floor() as i32;
    for y in center_y - radius_y..=center_y + radius_y {
        for x in center_x - radius_x..=center_x + radius_x {
            if x < 0 || y < 0 || x >= width as i32 || y >= height as i32 {
                continue;
            }
            let offset_x = x as f32 + 0.5 - gaussian.center_px.x;
            let offset_y = y as f32 + 0.5 - gaussian.center_px.y;
            let mahalanobis_sq = inverse_xx * offset_x * offset_x
                + 2.0 * inverse_xy * offset_x * offset_y
                + inverse_yy * offset_y * offset_y;
            if mahalanobis_sq > 4.0 {
                continue;
            }
            let index = (y as u32 * width + x as u32) as usize;
            if transmittance[index] <= 1.0e-3 {
                continue;
            }
            let alpha = (gaussian.opacity * (-0.5 * mahalanobis_sq).exp()).min(0.99);
            if alpha < 1.0 / 255.0 {
                continue;
            }
            let contribution = transmittance[index] * alpha;
            accumulated_weight[index] += contribution;
            weighted_depth[index] += contribution * gaussian.depth;
            transmittance[index] *= 1.0 - alpha;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use glam::{Quat, U8Vec4, Vec3};
    use rne_math::{Quat as RneQuat, Transform3, Vec3 as RneVec3};
    use rne_render::hash_depth_f32;

    fn gaussian_at(pos: Vec3) -> Gaussian {
        Gaussian {
            rotation: Quat::IDENTITY,
            pos,
            color: U8Vec4::new(200, 200, 200, 255),
            sh: [Vec3::ZERO; 15],
            scale: Vec3::splat(0.05),
        }
    }

    #[test]
    fn proxy_depth_hits_a_point_in_front_of_the_camera() {
        let camera = Camera::new(64, 48, std::f64::consts::FRAC_PI_4);
        let view = Transform3 {
            translation: RneVec3::new(0.0, 0.0, 0.0),
            rotation: RneQuat::IDENTITY,
            scale: RneVec3::ONE,
        };
        // Camera looks down -Z; place a Gaussian 2 m ahead.
        let depth = splat_proxy_depth_from_gaussians(
            &[gaussian_at(Vec3::new(0.0, 0.0, -2.0))],
            &camera,
            &view,
            &Transform3::IDENTITY,
        );
        let center = (depth.height / 2 * depth.width + depth.width / 2) as usize;
        assert!(
            (depth.depth_m[center] - 2.0).abs() < 0.15,
            "center depth {}",
            depth.depth_m[center]
        );
        assert_ne!(hash_depth_f32(&depth.depth_m), 0);
    }

    #[test]
    fn low_opacity_front_splat_does_not_force_nearest_depth() {
        let camera = Camera::new(64, 48, std::f64::consts::FRAC_PI_4);
        let view = Transform3::IDENTITY;
        let mut front = gaussian_at(Vec3::new(0.0, 0.0, -1.0));
        front.color.w = 8;
        let back = gaussian_at(Vec3::new(0.0, 0.0, -2.0));
        let depth =
            splat_proxy_depth_from_gaussians(&[front, back], &camera, &view, &Transform3::IDENTITY);
        let center = (depth.height / 2 * depth.width + depth.width / 2) as usize;
        assert!(
            depth.depth_m[center] > 1.8 && depth.depth_m[center] < 2.0,
            "alpha-composited center depth {}",
            depth.depth_m[center]
        );
    }

    #[test]
    fn composite_prefers_closer_surface() {
        let mesh = DepthFrame::new(2, 1, vec![1.0, 50.0]);
        let splat = DepthFrame::new(2, 1, vec![3.0, 2.0]);
        let composed = composite_mesh_and_splat_depth(&mesh, &splat, 100.0);
        assert_eq!(composed.depth_m, vec![1.0, 2.0]);
    }

    #[test]
    fn fixture_ply_produces_finite_proxy_depths() {
        let environment = rne_render::validate_gaussian_splat_manifest(
            &rne_render::tsukuba_confirmation_splat_manifest_path(),
        )
        .expect("manifest");
        let camera = Camera::new(80, 60, std::f64::consts::FRAC_PI_4);
        // Match example 82 / 78 sidewalk orbit so the tiny fixture is in frame.
        let view = rne_render_wgpu::CameraOrbit {
            focus: RneVec3::new(3.75, 0.25, 0.0),
            yaw_rad: -1.35,
            pitch_rad: 1.05,
            distance_m: 5.5,
        }
        .camera_transform();
        let depth = splat_proxy_depth_from_ply(
            &environment.ply_path,
            &camera,
            &view,
            &environment.transform,
        )
        .expect("proxy depth");
        assert!(
            depth
                .depth_m
                .iter()
                .any(|value| *value < camera.far_m as f32 * 0.999),
            "expected at least one finite splat proxy depth"
        );
        let again = splat_proxy_depth_from_ply(
            &environment.ply_path,
            &camera,
            &view,
            &environment.transform,
        )
        .expect("proxy depth replay");
        assert_eq!(depth.hash_depth(), again.hash_depth());
    }
}

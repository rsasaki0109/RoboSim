//! CPU proxy depth from Gaussian means for hybrid RGB-D capture spikes.
//!
//! Full volumetric splat depth is not available from the color-only
//! `wgpu-3dgs-viewer` pass. This module projects Gaussian centers into a linear
//! depth buffer so dataset captures can fill background depths where the mesh
//! pass only sees empty space.

use crate::GaussianSplatError;
use glam::{Mat4, Vec4Swizzles};
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

/// Projects Gaussian centers into a linear depth buffer (meters).
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
    let mut depth_m = vec![far; (width * height) as usize];

    let view_matrix = Camera::view_matrix(view).as_mat4();
    let projection = camera.projection_matrix().as_mat4();
    let env = transform_to_mat4(environment_transform);

    for gaussian in gaussians {
        if f32::from(gaussian.color.w) < 8.0 {
            continue;
        }
        let world = env.transform_point3(gaussian.pos);
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
        let px = ((ndc.x * 0.5 + 0.5) * width as f32).floor() as i32;
        let py = ((1.0 - (ndc.y * 0.5 + 0.5)) * height as f32).floor() as i32;
        let radius_px = splat_stamp_radius_px(gaussian, depth, camera);
        stamp_min_depth(&mut depth_m, width, height, px, py, radius_px, depth);
    }

    DepthFrame::new(width, height, depth_m)
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

fn splat_stamp_radius_px(gaussian: &Gaussian, depth_m: f32, camera: &Camera) -> i32 {
    let extent_m = gaussian.scale.x.max(gaussian.scale.y).max(gaussian.scale.z);
    let fov_y = camera.fov_y_rad as f32;
    let pixels =
        (extent_m / depth_m.max(1.0e-3)) * (camera.height as f32) / (2.0 * (fov_y * 0.5).tan());
    pixels.round().clamp(1.0, 8.0) as i32
}

fn stamp_min_depth(
    depth_m: &mut [f32],
    width: u32,
    height: u32,
    center_x: i32,
    center_y: i32,
    radius_px: i32,
    depth: f32,
) {
    let radius_sq = radius_px * radius_px;
    for dy in -radius_px..=radius_px {
        for dx in -radius_px..=radius_px {
            if dx * dx + dy * dy > radius_sq {
                continue;
            }
            let x = center_x + dx;
            let y = center_y + dy;
            if x < 0 || y < 0 || x >= width as i32 || y >= height as i32 {
                continue;
            }
            let index = (y as u32 * width + x as u32) as usize;
            if depth < depth_m[index] {
                depth_m[index] = depth;
            }
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

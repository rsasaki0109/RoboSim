//! Backend-neutral physically based material parameters.

use crate::image::ImageFrame;
use std::sync::Arc;

/// Material parameters shared by render backends.
///
/// The values describe a metallic-roughness workflow. `base_color_rgba` is
/// linear-space albedo and opacity; textures are sampled separately by the
/// backend and multiplied by this value. Backends should use [`Self::sanitized`]
/// before uploading the parameters to a GPU.
#[derive(Clone, Debug, PartialEq)]
pub struct PbrMaterial {
    /// Linear-space base color and opacity.
    pub base_color_rgba: [f32; 4],
    /// Perceptual surface roughness in the inclusive `[0, 1]` range.
    pub roughness: f32,
    /// Metallic factor in the inclusive `[0, 1]` range.
    pub metallic: f32,
    /// Linear-space emissive RGB contribution.
    pub emissive_rgb: [f32; 3],
    /// Optional tangent-space normal map sampled with mesh UV coordinates.
    pub normal_texture: Option<Arc<ImageFrame>>,
    /// Optional linear roughness map sampled with mesh UV coordinates.
    pub roughness_texture: Option<Arc<ImageFrame>>,
    /// Strength applied to tangent-space normal-map XY components.
    pub normal_strength: f32,
    /// Optional packed glTF metallic-roughness texture. Green is roughness;
    /// blue is metallic, both in linear space.
    pub metallic_roughness_texture: Option<Arc<ImageFrame>>,
    /// Optional sRGB glTF emissive texture.
    pub emissive_texture: Option<Arc<ImageFrame>>,
    /// Optional linear glTF occlusion texture sampled from the red channel.
    pub occlusion_texture: Option<Arc<ImageFrame>>,
    /// Strength applied to the occlusion texture in the inclusive `[0, 1]`
    /// range.
    pub occlusion_strength: f32,
}

impl Default for PbrMaterial {
    fn default() -> Self {
        Self {
            base_color_rgba: [1.0; 4],
            roughness: 0.7,
            metallic: 0.0,
            emissive_rgb: [0.0; 3],
            normal_texture: None,
            roughness_texture: None,
            normal_strength: 1.0,
            metallic_roughness_texture: None,
            emissive_texture: None,
            occlusion_texture: None,
            occlusion_strength: 1.0,
        }
    }
}

impl PbrMaterial {
    /// Creates a metallic-roughness material with explicit parameters.
    pub const fn new(
        base_color_rgba: [f32; 4],
        roughness: f32,
        metallic: f32,
        emissive_rgb: [f32; 3],
    ) -> Self {
        Self {
            base_color_rgba,
            roughness,
            metallic,
            emissive_rgb,
            normal_texture: None,
            roughness_texture: None,
            normal_strength: 1.0,
            metallic_roughness_texture: None,
            emissive_texture: None,
            occlusion_texture: None,
            occlusion_strength: 1.0,
        }
    }

    /// Attaches optional tangent-space normal and linear roughness textures.
    pub fn with_texture_maps(
        mut self,
        normal_texture: Option<Arc<ImageFrame>>,
        roughness_texture: Option<Arc<ImageFrame>>,
    ) -> Self {
        self.normal_texture = normal_texture;
        self.roughness_texture = roughness_texture;
        self
    }

    /// Attaches packed metallic-roughness, emissive, and occlusion maps.
    pub fn with_pbr_texture_maps(
        mut self,
        metallic_roughness_texture: Option<Arc<ImageFrame>>,
        emissive_texture: Option<Arc<ImageFrame>>,
        occlusion_texture: Option<Arc<ImageFrame>>,
        occlusion_strength: f32,
    ) -> Self {
        self.metallic_roughness_texture = metallic_roughness_texture;
        self.emissive_texture = emissive_texture;
        self.occlusion_texture = occlusion_texture;
        self.occlusion_strength = occlusion_strength;
        self
    }

    /// Sets the tangent-space normal-map scale used by a backend.
    pub fn with_normal_strength(mut self, normal_strength: f32) -> Self {
        self.normal_strength = normal_strength;
        self
    }

    /// Returns finite, shader-safe material parameters.
    pub fn sanitized(&self) -> Self {
        Self {
            base_color_rgba: self
                .base_color_rgba
                .map(|value| finite_or(value, 1.0).max(0.0)),
            roughness: finite_or(self.roughness, 0.7).clamp(0.04, 1.0),
            metallic: finite_or(self.metallic, 0.0).clamp(0.0, 1.0),
            emissive_rgb: self
                .emissive_rgb
                .map(|value| finite_or(value, 0.0).max(0.0)),
            normal_texture: self.normal_texture.clone(),
            roughness_texture: self.roughness_texture.clone(),
            normal_strength: finite_or(self.normal_strength, 1.0).clamp(0.0, 2.0),
            metallic_roughness_texture: self.metallic_roughness_texture.clone(),
            emissive_texture: self.emissive_texture.clone(),
            occlusion_texture: self.occlusion_texture.clone(),
            occlusion_strength: finite_or(self.occlusion_strength, 1.0).clamp(0.0, 1.0),
        }
    }
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
    fn default_material_is_non_metallic_and_rough() {
        let material = PbrMaterial::default();
        assert_eq!(material.base_color_rgba, [1.0; 4]);
        assert_eq!(material.roughness, 0.7);
        assert_eq!(material.metallic, 0.0);
        assert_eq!(material.emissive_rgb, [0.0; 3]);
        assert!(material.normal_texture.is_none());
        assert!(material.roughness_texture.is_none());
        assert_eq!(material.normal_strength, 1.0);
        assert!(material.metallic_roughness_texture.is_none());
        assert!(material.emissive_texture.is_none());
        assert!(material.occlusion_texture.is_none());
        assert_eq!(material.occlusion_strength, 1.0);
    }

    #[test]
    fn sanitization_clamps_invalid_shader_inputs() {
        let material = PbrMaterial::new(
            [f32::NAN, -1.0, 0.5, 1.0],
            f32::INFINITY,
            -2.0,
            [f32::NAN, -1.0, 2.0],
        )
        .sanitized();
        assert_eq!(material.base_color_rgba, [1.0, 0.0, 0.5, 1.0]);
        assert_eq!(material.roughness, 0.7);
        assert_eq!(material.metallic, 0.0);
        assert_eq!(material.emissive_rgb, [0.0, 0.0, 2.0]);
        assert!(material.normal_texture.is_none());
        assert!(material.roughness_texture.is_none());
        assert_eq!(material.normal_strength, 1.0);
        assert!(material.metallic_roughness_texture.is_none());
        assert!(material.emissive_texture.is_none());
        assert!(material.occlusion_texture.is_none());
        assert_eq!(material.occlusion_strength, 1.0);
    }

    #[test]
    fn texture_maps_are_preserved_through_sanitization() {
        let normal = Arc::new(ImageFrame::from_rgba8(1, 1, vec![128, 128, 255, 255]));
        let roughness = Arc::new(ImageFrame::from_rgba8(1, 1, vec![180, 180, 180, 255]));
        let metallic_roughness = Arc::new(ImageFrame::from_rgba8(1, 1, vec![0, 128, 200, 255]));
        let emissive = Arc::new(ImageFrame::from_rgba8(1, 1, vec![12, 24, 48, 255]));
        let occlusion = Arc::new(ImageFrame::from_rgba8(1, 1, vec![160, 160, 160, 255]));
        let material = PbrMaterial::default()
            .with_texture_maps(Some(Arc::clone(&normal)), Some(Arc::clone(&roughness)))
            .with_pbr_texture_maps(
                Some(Arc::clone(&metallic_roughness)),
                Some(Arc::clone(&emissive)),
                Some(Arc::clone(&occlusion)),
                0.65,
            );
        let sanitized = material.sanitized();
        assert_eq!(sanitized.normal_texture, Some(normal));
        assert_eq!(sanitized.roughness_texture, Some(roughness));
        assert_eq!(
            sanitized.metallic_roughness_texture,
            Some(metallic_roughness)
        );
        assert_eq!(sanitized.emissive_texture, Some(emissive));
        assert_eq!(sanitized.occlusion_texture, Some(occlusion));
        assert_eq!(sanitized.occlusion_strength, 0.65);
    }
}

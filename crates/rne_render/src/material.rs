//! Backend-neutral physically based material parameters.

/// Material parameters shared by render backends.
///
/// The values describe a metallic-roughness workflow. `base_color_rgba` is
/// linear-space albedo and opacity; textures are sampled separately by the
/// backend and multiplied by this value. Backends should use [`Self::sanitized`]
/// before uploading the parameters to a GPU.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct PbrMaterial {
    /// Linear-space base color and opacity.
    pub base_color_rgba: [f32; 4],
    /// Perceptual surface roughness in the inclusive `[0, 1]` range.
    pub roughness: f32,
    /// Metallic factor in the inclusive `[0, 1]` range.
    pub metallic: f32,
    /// Linear-space emissive RGB contribution.
    pub emissive_rgb: [f32; 3],
}

impl Default for PbrMaterial {
    fn default() -> Self {
        Self {
            base_color_rgba: [1.0; 4],
            roughness: 0.7,
            metallic: 0.0,
            emissive_rgb: [0.0; 3],
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
        }
    }

    /// Returns finite, shader-safe material parameters.
    pub fn sanitized(self) -> Self {
        Self {
            base_color_rgba: self
                .base_color_rgba
                .map(|value| finite_or(value, 1.0).max(0.0)),
            roughness: finite_or(self.roughness, 0.7).clamp(0.04, 1.0),
            metallic: finite_or(self.metallic, 0.0).clamp(0.0, 1.0),
            emissive_rgb: self
                .emissive_rgb
                .map(|value| finite_or(value, 0.0).max(0.0)),
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
    }
}

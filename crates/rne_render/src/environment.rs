//! HDR environment maps and backend-neutral environment-lighting settings.

use image::DynamicImage;
use std::path::Path;
use std::sync::Arc;
use thiserror::Error;

/// An equirectangular high-dynamic-range environment map.
///
/// Pixels are stored in row-major order as linear, non-negative RGBA `f32`
/// values. The alpha channel is retained for a stable four-channel GPU upload
/// and does not affect lighting.
#[derive(Clone, Debug, PartialEq)]
pub struct EnvironmentMap {
    /// Image width in pixels.
    pub width: u32,
    /// Image height in pixels.
    pub height: u32,
    /// Linear RGBA pixels, with four values per pixel.
    pub rgba32f: Vec<f32>,
}

/// Error returned while constructing or loading an [`EnvironmentMap`].
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum EnvironmentMapError {
    /// The supplied dimensions and pixel buffer do not agree.
    #[error(
        "environment map dimensions {width}x{height} require {expected} float values, got {actual}"
    )]
    InvalidLength {
        /// Image width.
        width: u32,
        /// Image height.
        height: u32,
        /// Required number of float values.
        expected: usize,
        /// Supplied number of float values.
        actual: usize,
    },
    /// The map has no pixels.
    #[error("environment map dimensions must be non-zero")]
    Empty,
    /// A pixel contained a non-finite or negative value.
    #[error("environment map pixel {pixel} channel {channel} is not finite and non-negative")]
    InvalidPixel {
        /// Zero-based pixel index.
        pixel: usize,
        /// Zero-based channel index.
        channel: usize,
    },
    /// The image file could not be decoded.
    #[error("failed to decode environment map {path}: {message}")]
    Decode {
        /// Source path.
        path: String,
        /// Decoder error.
        message: String,
    },
}

impl EnvironmentMap {
    /// Creates an equirectangular map from linear RGBA32F pixels.
    pub fn from_rgba32f(
        width: u32,
        height: u32,
        rgba32f: Vec<f32>,
    ) -> Result<Self, EnvironmentMapError> {
        if width == 0 || height == 0 {
            return Err(EnvironmentMapError::Empty);
        }
        let expected = width as usize * height as usize * 4;
        if rgba32f.len() != expected {
            return Err(EnvironmentMapError::InvalidLength {
                width,
                height,
                expected,
                actual: rgba32f.len(),
            });
        }
        for (index, pixel) in rgba32f.chunks_exact(4).enumerate() {
            for (channel, value) in pixel.iter().enumerate() {
                if !value.is_finite() || *value < 0.0 {
                    return Err(EnvironmentMapError::InvalidPixel {
                        pixel: index,
                        channel,
                    });
                }
            }
        }
        Ok(Self {
            width,
            height,
            rgba32f,
        })
    }

    /// Creates a one-pixel environment useful for fallbacks and tests.
    pub fn solid(rgba32f: [f32; 4]) -> Result<Self, EnvironmentMapError> {
        Self::from_rgba32f(1, 1, rgba32f.to_vec())
    }

    /// Loads a Radiance HDR (`.hdr`) equirectangular environment map.
    ///
    /// The decoded RGB values are converted to linear RGBA32F pixels. Image
    /// orientation follows the source file; the WGPU backend applies the
    /// equirectangular lookup convention at sample time.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, EnvironmentMapError> {
        let path = path.as_ref();
        let path_string = path.display().to_string();
        let decoded = image::open(path).map_err(|error| EnvironmentMapError::Decode {
            path: path_string.clone(),
            message: error.to_string(),
        })?;
        Self::from_dynamic_image(decoded).map_err(|error| match error {
            EnvironmentMapError::Decode { message, .. } => EnvironmentMapError::Decode {
                path: path_string,
                message,
            },
            other => other,
        })
    }

    fn from_dynamic_image(image: DynamicImage) -> Result<Self, EnvironmentMapError> {
        let rgb = image.to_rgb32f();
        let mut rgba32f = Vec::with_capacity(rgb.width() as usize * rgb.height() as usize * 4);
        for pixel in rgb.pixels() {
            rgba32f.extend_from_slice(&[pixel[0], pixel[1], pixel[2], 1.0]);
        }
        Self::from_rgba32f(rgb.width(), rgb.height(), rgba32f)
    }
}

/// Environment-lighting controls shared by renderer backends.
#[derive(Clone, Debug, PartialEq)]
pub struct EnvironmentLighting {
    /// Optional equirectangular HDR environment map.
    pub map: Option<Arc<EnvironmentMap>>,
    /// Overall HDR map intensity multiplier.
    pub intensity: f32,
    /// Diffuse image-based-lighting multiplier.
    pub diffuse_strength: f32,
    /// Specular image-based-lighting multiplier.
    pub specular_strength: f32,
    /// Rotation around the world Y axis in radians.
    pub rotation_rad: f32,
}

impl Default for EnvironmentLighting {
    fn default() -> Self {
        Self {
            map: None,
            intensity: 1.0,
            diffuse_strength: 0.35,
            specular_strength: 0.25,
            rotation_rad: 0.0,
        }
    }
}

impl EnvironmentLighting {
    /// Creates enabled environment lighting from an HDR map.
    pub fn from_map(map: Arc<EnvironmentMap>) -> Self {
        Self {
            map: Some(map),
            ..Self::default()
        }
    }

    /// Returns a shader-safe copy with finite, bounded controls.
    pub fn sanitized(&self) -> Self {
        Self {
            map: self.map.clone(),
            intensity: finite_or(self.intensity, 1.0).max(0.0),
            diffuse_strength: finite_or(self.diffuse_strength, 0.35).max(0.0),
            specular_strength: finite_or(self.specular_strength, 0.25).max(0.0),
            rotation_rad: finite_or(self.rotation_rad, 0.0),
        }
    }

    /// Returns whether the map contributes visible background or lighting.
    pub fn is_enabled(&self) -> bool {
        self.map.is_some() && self.intensity.is_finite() && self.intensity > 0.0
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
    fn rgba32f_map_validates_dimensions_and_pixels() {
        let map = EnvironmentMap::from_rgba32f(2, 1, vec![1.0; 8]).expect("valid map");
        assert_eq!(map.width, 2);
        assert_eq!(map.height, 1);

        assert!(matches!(
            EnvironmentMap::from_rgba32f(2, 1, vec![1.0; 4]),
            Err(EnvironmentMapError::InvalidLength { .. })
        ));
        assert!(matches!(
            EnvironmentMap::from_rgba32f(1, 1, vec![-1.0, 0.0, 0.0, 1.0]),
            Err(EnvironmentMapError::InvalidPixel {
                pixel: 0,
                channel: 0
            })
        ));
    }

    #[test]
    fn lighting_sanitization_keeps_map_and_clamps_controls() {
        let map = Arc::new(EnvironmentMap::solid([2.0, 1.0, 0.5, 1.0]).expect("solid map"));
        let lighting = EnvironmentLighting {
            map: Some(Arc::clone(&map)),
            intensity: f32::NAN,
            diffuse_strength: -1.0,
            specular_strength: f32::INFINITY,
            rotation_rad: f32::NEG_INFINITY,
        }
        .sanitized();
        assert_eq!(lighting.map, Some(map));
        assert_eq!(lighting.intensity, 1.0);
        assert_eq!(lighting.diffuse_strength, 0.0);
        assert_eq!(lighting.specular_strength, 0.25);
        assert_eq!(lighting.rotation_rad, 0.0);
    }

    #[test]
    fn disabled_default_does_not_enable_environment() {
        assert!(!EnvironmentLighting::default().is_enabled());
    }
}

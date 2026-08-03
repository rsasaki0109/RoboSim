//! Deterministic CPU prefiltering for HDR environment lighting.

use rne_render::EnvironmentMap;

pub(crate) const SPECULAR_MIP_LEVELS: u32 = 5;
const SPECULAR_BASE_WIDTH: u32 = 128;
const SPECULAR_BASE_HEIGHT: u32 = 64;
const DIFFUSE_WIDTH: u32 = 32;
const DIFFUSE_HEIGHT: u32 = 16;
const SPECULAR_SAMPLES: u32 = 32;
const DIFFUSE_SAMPLES: u32 = 64;

pub(crate) struct EnvironmentLevel {
    pub(crate) width: u32,
    pub(crate) height: u32,
    pub(crate) rgba32f: Vec<f32>,
}

pub(crate) struct PrefilteredEnvironment {
    pub(crate) specular_levels: Vec<EnvironmentLevel>,
    pub(crate) diffuse: EnvironmentLevel,
}

pub(crate) fn prefilter_environment(map: &EnvironmentMap) -> PrefilteredEnvironment {
    let specular_levels = (0..SPECULAR_MIP_LEVELS)
        .map(|level| {
            let divisor = 1_u32 << level;
            let width = (SPECULAR_BASE_WIDTH / divisor).max(1);
            let height = (SPECULAR_BASE_HEIGHT / divisor).max(1);
            let roughness = level as f32 / (SPECULAR_MIP_LEVELS - 1) as f32;
            EnvironmentLevel {
                width,
                height,
                rgba32f: prefilter_specular_level(map, width, height, roughness),
            }
        })
        .collect();

    EnvironmentLevel {
        width: DIFFUSE_WIDTH,
        height: DIFFUSE_HEIGHT,
        rgba32f: prefilter_diffuse(map, DIFFUSE_WIDTH, DIFFUSE_HEIGHT),
    }
    .into_prefiltered(specular_levels)
}

impl EnvironmentLevel {
    fn into_prefiltered(self, specular_levels: Vec<EnvironmentLevel>) -> PrefilteredEnvironment {
        PrefilteredEnvironment {
            specular_levels,
            diffuse: self,
        }
    }
}

fn prefilter_specular_level(
    map: &EnvironmentMap,
    width: u32,
    height: u32,
    roughness: f32,
) -> Vec<f32> {
    let pixel_count = width as usize * height as usize;
    let mut output = Vec::with_capacity(pixel_count * 4);
    let sample_count = if roughness <= f32::EPSILON {
        1
    } else {
        SPECULAR_SAMPLES
    };

    for y in 0..height {
        for x in 0..width {
            let direction = equirectangular_direction(
                (x as f32 + 0.5) / width as f32,
                (y as f32 + 0.5) / height as f32,
            );
            let mut color = [0.0_f32; 3];
            let mut weight_sum = 0.0_f32;
            let (tangent, bitangent) = tangent_basis(direction);

            for sample_index in 0..sample_count {
                let light_direction = if sample_count == 1 {
                    direction
                } else {
                    let xi = [
                        (sample_index as f32 + 0.5) / sample_count as f32,
                        radical_inverse_vdc(sample_index),
                    ];
                    let half_vector =
                        importance_sample_ggx(xi, roughness, direction, tangent, bitangent);
                    let reflected = add(
                        scale(half_vector, 2.0 * dot(direction, half_vector)),
                        scale(direction, -1.0),
                    );
                    normalize(reflected)
                };
                let weight = dot(direction, light_direction).max(0.0);
                if weight > 0.0 {
                    let sample = sample_environment(map, light_direction);
                    color[0] += sample[0] * weight;
                    color[1] += sample[1] * weight;
                    color[2] += sample[2] * weight;
                    weight_sum += weight;
                }
            }

            if weight_sum > 0.0 {
                color = scale(color, 1.0 / weight_sum);
            }
            output.extend_from_slice(&[color[0], color[1], color[2], 1.0]);
        }
    }
    output
}

fn prefilter_diffuse(map: &EnvironmentMap, width: u32, height: u32) -> Vec<f32> {
    let pixel_count = width as usize * height as usize;
    let mut output = Vec::with_capacity(pixel_count * 4);
    for y in 0..height {
        for x in 0..width {
            let normal = equirectangular_direction(
                (x as f32 + 0.5) / width as f32,
                (y as f32 + 0.5) / height as f32,
            );
            let (tangent, bitangent) = tangent_basis(normal);
            let mut color = [0.0_f32; 3];
            for sample_index in 0..DIFFUSE_SAMPLES {
                let xi = [
                    (sample_index as f32 + 0.5) / DIFFUSE_SAMPLES as f32,
                    radical_inverse_vdc(sample_index),
                ];
                let phi = std::f32::consts::TAU * xi[0];
                let cos_theta = (1.0 - xi[1]).sqrt();
                let sin_theta = xi[1].sqrt();
                let local = [sin_theta * phi.cos(), cos_theta, sin_theta * phi.sin()];
                let direction = normalize(add(
                    add(scale(tangent, local[0]), scale(normal, local[1])),
                    scale(bitangent, local[2]),
                ));
                let sample = sample_environment(map, direction);
                color[0] += sample[0];
                color[1] += sample[1];
                color[2] += sample[2];
            }
            let scale = 1.0 / DIFFUSE_SAMPLES as f32;
            output.extend_from_slice(&[color[0] * scale, color[1] * scale, color[2] * scale, 1.0]);
        }
    }
    output
}

fn importance_sample_ggx(
    xi: [f32; 2],
    roughness: f32,
    normal: [f32; 3],
    tangent: [f32; 3],
    bitangent: [f32; 3],
) -> [f32; 3] {
    let alpha = roughness * roughness;
    let alpha_squared = alpha * alpha;
    let phi = std::f32::consts::TAU * xi[0];
    let cos_theta = ((1.0 - xi[1]) / (1.0 + (alpha_squared - 1.0) * xi[1])).sqrt();
    let sin_theta = (1.0 - cos_theta * cos_theta).max(0.0).sqrt();
    let local = [sin_theta * phi.cos(), sin_theta * phi.sin(), cos_theta];
    normalize(add(
        add(scale(tangent, local[0]), scale(bitangent, local[1])),
        scale(normal, local[2]),
    ))
}

fn sample_environment(map: &EnvironmentMap, direction: [f32; 3]) -> [f32; 3] {
    let direction = normalize(direction);
    let u = direction[2].atan2(direction[0]) / std::f32::consts::TAU + 0.5;
    let v = direction[1].clamp(-1.0, 1.0).acos() / std::f32::consts::PI;
    let x = u * map.width as f32 - 0.5;
    let y = v * map.height as f32 - 0.5;
    let x0 = x.floor() as i32;
    let y0 = y.floor() as i32;
    let fx = x - x.floor();
    let fy = (y - y.floor()).clamp(0.0, 1.0);
    let x1 = x0 + 1;
    let y1 = y0 + 1;
    let c00 = map_pixel(map, x0, y0);
    let c10 = map_pixel(map, x1, y0);
    let c01 = map_pixel(map, x0, y1);
    let c11 = map_pixel(map, x1, y1);
    let top = mix(c00, c10, fx);
    let bottom = mix(c01, c11, fx);
    mix(top, bottom, fy)
}

fn map_pixel(map: &EnvironmentMap, x: i32, y: i32) -> [f32; 3] {
    let width = map.width as i32;
    let height = map.height as i32;
    let x = x.rem_euclid(width) as usize;
    let y = y.clamp(0, height - 1) as usize;
    let index = (y * map.width as usize + x) * 4;
    [
        map.rgba32f[index],
        map.rgba32f[index + 1],
        map.rgba32f[index + 2],
    ]
}

fn equirectangular_direction(u: f32, v: f32) -> [f32; 3] {
    let theta = (u - 0.5) * std::f32::consts::TAU;
    let phi = v * std::f32::consts::PI;
    let sin_phi = phi.sin();
    [sin_phi * theta.cos(), phi.cos(), sin_phi * theta.sin()]
}

fn tangent_basis(normal: [f32; 3]) -> ([f32; 3], [f32; 3]) {
    let reference = if normal[1].abs() < 0.9 {
        [0.0, 1.0, 0.0]
    } else {
        [1.0, 0.0, 0.0]
    };
    let tangent = normalize(cross(reference, normal));
    (tangent, normalize(cross(normal, tangent)))
}

fn radical_inverse_vdc(mut bits: u32) -> f32 {
    bits = bits.rotate_left(16);
    bits = ((bits & 0x5555_5555) << 1) | ((bits & 0xAAAA_AAAA) >> 1);
    bits = ((bits & 0x3333_3333) << 2) | ((bits & 0xCCCC_CCCC) >> 2);
    bits = ((bits & 0x0F0F_0F0F) << 4) | ((bits & 0xF0F0_F0F0) >> 4);
    bits as f32 * 2.328_306_4e-10
}

fn dot(a: [f32; 3], b: [f32; 3]) -> f32 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn cross(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    ]
}

fn add(a: [f32; 3], b: [f32; 3]) -> [f32; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn scale(value: [f32; 3], factor: f32) -> [f32; 3] {
    [value[0] * factor, value[1] * factor, value[2] * factor]
}

fn normalize(value: [f32; 3]) -> [f32; 3] {
    let length = dot(value, value).sqrt().max(f32::EPSILON);
    scale(value, 1.0 / length)
}

fn mix(a: [f32; 3], b: [f32; 3], amount: f32) -> [f32; 3] {
    [
        a[0] + (b[0] - a[0]) * amount,
        a[1] + (b[1] - a[1]) * amount,
        a[2] + (b[2] - a[2]) * amount,
    ]
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn prefilter_is_deterministic_and_has_expected_levels() {
        let map = EnvironmentMap::from_rgba32f(
            4,
            2,
            vec![
                1.0, 0.5, 0.25, 1.0, 0.2, 0.4, 0.8, 1.0, 0.8, 0.3, 0.1, 1.0, 0.1, 0.7, 0.9, 1.0,
                0.3, 0.6, 0.2, 1.0, 0.9, 0.4, 0.2, 1.0, 0.6, 0.2, 0.8, 1.0, 0.4, 0.8, 0.1, 1.0,
            ],
        )
        .expect("valid map");
        let first = prefilter_environment(&map);
        let second = prefilter_environment(&map);
        assert_eq!(first.specular_levels.len(), SPECULAR_MIP_LEVELS as usize);
        assert_eq!(first.specular_levels[0].width, SPECULAR_BASE_WIDTH);
        assert_eq!(first.specular_levels[4].height, 4);
        assert_eq!(first.diffuse.width, DIFFUSE_WIDTH);
        assert_eq!(
            first.specular_levels[2].rgba32f,
            second.specular_levels[2].rgba32f
        );
        for (level_index, level) in first.specular_levels.iter().enumerate() {
            for (value_index, value) in level.rgba32f.iter().enumerate() {
                assert!(
                    value.is_finite() && *value >= 0.0,
                    "invalid prefilter value at level {level_index}, index {value_index}: {value}"
                );
            }
        }
    }

    #[test]
    fn constant_environment_stays_constant_through_prefiltering() {
        let map =
            EnvironmentMap::from_rgba32f(2, 2, [2.0, 1.0, 0.5, 1.0].repeat(4)).expect("valid map");
        let filtered = prefilter_environment(&map);
        for level in filtered
            .specular_levels
            .iter()
            .chain(std::iter::once(&filtered.diffuse))
        {
            for pixel in level.rgba32f.chunks_exact(4) {
                assert!((pixel[0] - 2.0).abs() < 1e-4);
                assert!((pixel[1] - 1.0).abs() < 1e-4);
                assert!((pixel[2] - 0.5).abs() < 1e-4);
                assert_eq!(pixel[3], 1.0);
            }
        }
    }
}

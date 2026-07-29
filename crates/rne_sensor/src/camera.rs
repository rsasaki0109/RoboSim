//! Physics-aware RGB camera sensor specification and sampling.
//!
//! The model is renderer-independent: the backend supplies an ideal pinhole render and
//! a linear depth buffer, and every sensor effect is applied on top of those buffers.
//! Nothing here depends on a particular render backend, and no effect is pushed down
//! into one. This mirrors how [`crate::lidar`] stays physics-backend-neutral.
//!
//! # Pipeline
//!
//! ```text
//! render (optionally per rolling-shutter band)
//!   -> lens distortion   (optical, geometric; resamples color and depth)
//!   -> vignetting        (optical, cos^4 falloff)
//!   -> exposure gain     (electronic)
//!   -> shot + read noise (electronic)
//! ```
//!
//! Optical effects precede electronic ones because that is the order light actually
//! encounters them.
//!
//! # Determinism and units
//!
//! Every stochastic effect draws from a disjoint slot of a [`SensorNoiseKey`]-derived
//! stream, so a given key always reproduces the same frame. No wall-clock time is read.
//!
//! Channel values are treated as linear radiance proxies in `[0, 1]`. The render backend
//! returns 8-bit RGBA, so exposure and noise operate on a quantized approximation of
//! linear light rather than on true photon counts. This is a deliberate simplification;
//! a backend that grows a high-dynamic-range surface can replace it without changing
//! this API.
//!
//! # Defaults
//!
//! [`CameraSpec::default`] disables every effect, and a default spec produces
//! byte-identical output to a plain backend render.

use crate::noise::gaussian_pair;
use crate::SensorNoiseKey;
use rne_core::{mix64, KeyedRandom, SimTime};
use rne_data::{ImageDepth, ImageRgb8};
use rne_render::{
    pass::CameraPassOutput, Camera, DepthFrame, ImageFrame, RenderBackend, RenderScene,
};
use rne_world::Transform3;
use serde::{Deserialize, Serialize};

const CAMERA_RANDOM_DOMAIN_V1: u64 = 0x3143_414D_4152_454E;
/// Random slots reserved per pixel; three color channels consume two slots each.
const PIXEL_SLOT_STRIDE: u64 = 8;
/// Rec. 709 luminance weights used by auto exposure.
const LUMA_WEIGHTS: [f64; 3] = [0.2126, 0.7152, 0.0722];

/// RGB + depth sample from one camera capture.
#[derive(Clone, Debug, PartialEq)]
pub struct CameraRgbdSample {
    /// RGBA8 color image.
    pub rgb: ImageRgb8,
    /// Matching linear depth image in meters.
    pub depth: ImageDepth,
}

/// Brown-Conrady lens distortion coefficients.
///
/// All coefficients are dimensionless and act on normalized pinhole coordinates. Zero
/// coefficients are an exact identity: the image is returned untouched rather than
/// resampled.
#[derive(Clone, Copy, Debug, Default, PartialEq, Serialize, Deserialize)]
pub struct CameraDistortion {
    /// First radial coefficient. Negative values produce barrel distortion.
    pub k1: f64,
    /// Second radial coefficient.
    pub k2: f64,
    /// Third radial coefficient.
    pub k3: f64,
    /// First tangential coefficient.
    pub p1: f64,
    /// Second tangential coefficient.
    pub p2: f64,
}

impl CameraDistortion {
    /// Returns true when the model applies no distortion at all.
    pub fn is_identity(&self) -> bool {
        self.k1 == 0.0 && self.k2 == 0.0 && self.k3 == 0.0 && self.p1 == 0.0 && self.p2 == 0.0
    }

    /// Maps ideal normalized coordinates to distorted normalized coordinates.
    pub fn distort(&self, x: f64, y: f64) -> (f64, f64) {
        let r2 = x * x + y * y;
        let radial = 1.0 + self.k1 * r2 + self.k2 * r2 * r2 + self.k3 * r2 * r2 * r2;
        (
            x * radial + 2.0 * self.p1 * x * y + self.p2 * (r2 + 2.0 * x * x),
            y * radial + self.p1 * (r2 + 2.0 * y * y) + 2.0 * self.p2 * x * y,
        )
    }
}

/// Sensor pose at the start and end of one frame readout.
///
/// A rolling-shutter sensor reads rows sequentially, so a moving camera captures the
/// top and bottom of a frame from different poses. Interpolating across this sweep
/// reproduces the skew and wobble real CMOS sensors show.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraSweep {
    /// Sensor pose when the first row is read out.
    pub start: Transform3,
    /// Sensor pose when the last row is read out.
    pub end: Transform3,
}

impl CameraSweep {
    /// Creates a sweep between two sensor poses.
    pub fn new(start: Transform3, end: Transform3) -> Self {
        Self { start, end }
    }

    /// Creates a sweep for a sensor that does not move during readout.
    pub fn stationary(pose: Transform3) -> Self {
        Self {
            start: pose,
            end: pose,
        }
    }

    /// Returns true when the sensor pose is identical across the sweep.
    pub fn is_stationary(&self) -> bool {
        self.start == self.end
    }

    /// Returns the interpolated sensor pose at `fraction` through the readout.
    pub fn pose_at(&self, fraction: f64) -> Transform3 {
        if self.is_stationary() {
            return self.start;
        }
        let fraction = fraction.clamp(0.0, 1.0);
        Transform3 {
            translation: self.start.translation.lerp(self.end.translation, fraction),
            rotation: self.start.rotation.slerp(self.end.rotation, fraction),
            scale: self.start.scale.lerp(self.end.scale, fraction),
        }
    }
}

/// RGB camera parameters.
///
/// Defaults describe an ideal pinhole camera with no distortion, a global shutter, unit
/// gain and no noise, so existing configurations are unaffected.
#[derive(Clone, Copy, Debug, PartialEq, Serialize, Deserialize)]
#[serde(default)]
pub struct CameraSpec {
    /// Output width in pixels.
    pub width: u32,
    /// Output height in pixels.
    pub height: u32,
    /// Vertical field of view in radians.
    pub fov_y_rad: f64,
    /// Deterministic render seed.
    pub seed: u64,
    /// Lens distortion coefficients.
    pub distortion: CameraDistortion,
    /// Time to read out every row of the frame, in seconds.
    ///
    /// `0.0` models a global shutter: every row shares the capture instant.
    pub readout_time_s: f64,
    /// Number of horizontal bands rendered independently for rolling shutter.
    ///
    /// `1` renders the frame once. Higher values trade render cost for a finer
    /// approximation of continuous row-by-row readout.
    pub rolling_shutter_bands: u16,
    /// Manual exposure compensation in stops; gain is `2^exposure_ev`.
    pub exposure_ev: f64,
    /// Target mean luminance for auto exposure in `[0, 1]`; `0.0` disables it.
    pub auto_exposure_target_luminance: f64,
    /// Maximum absolute stops auto exposure may apply.
    pub auto_exposure_max_ev: f64,
    /// Photon shot-noise scale; noise variance grows linearly with signal.
    pub shot_noise_scale: f64,
    /// Constant read-noise standard deviation in normalized signal units.
    pub read_noise_stddev: f64,
    /// Vignetting strength in `[0, 1]`; `0.0` disables the cos^4 falloff.
    pub vignette_strength: f64,
}

impl Default for CameraSpec {
    fn default() -> Self {
        Self {
            width: 64,
            height: 48,
            fov_y_rad: std::f64::consts::FRAC_PI_4,
            seed: 0,
            distortion: CameraDistortion::default(),
            readout_time_s: 0.0,
            rolling_shutter_bands: 1,
            exposure_ev: 0.0,
            auto_exposure_target_luminance: 0.0,
            auto_exposure_max_ev: 4.0,
            shot_noise_scale: 0.0,
            read_noise_stddev: 0.0,
            vignette_strength: 0.0,
        }
    }
}

impl CameraSpec {
    /// Returns the number of rolling-shutter bands, at least one.
    pub fn effective_band_count(&self) -> u16 {
        self.rolling_shutter_bands.max(1)
    }

    /// Returns the readout time of `row` relative to the start of the frame, in seconds.
    ///
    /// Row zero is read at the capture time reported by `Frame.capture_time`; the last
    /// row is read [`Self::readout_time_s`] later. Sensor output latency is separate and
    /// is applied by [`crate::Sensor::latency`].
    pub fn row_time_s(&self, row: u32) -> f64 {
        if self.height <= 1 || self.readout_time_s <= 0.0 {
            return 0.0;
        }
        self.readout_time_s * f64::from(row.min(self.height - 1)) / f64::from(self.height - 1)
    }

    /// Returns true when no sensor effect would modify the rendered image.
    pub fn is_ideal(&self) -> bool {
        self.distortion.is_identity()
            && self.vignette_strength <= 0.0
            && self.exposure_ev == 0.0
            && self.auto_exposure_target_luminance <= 0.0
            && self.shot_noise_scale <= 0.0
            && self.read_noise_stddev <= 0.0
    }

    /// Returns the focal length in pixels implied by the vertical field of view.
    fn focal_length_px(&self) -> f64 {
        let half = (self.fov_y_rad * 0.5).tan();
        if half.abs() <= f64::EPSILON {
            return f64::from(self.height.max(1)) * 0.5;
        }
        f64::from(self.height.max(1)) * 0.5 / half
    }
}

/// Samples an RGB camera attached to the given entity transform.
pub fn sample_camera<R: RenderBackend + ?Sized>(
    render: &mut R,
    transform: &Transform3,
    spec: &CameraSpec,
    sim_time: SimTime,
) -> ImageRgb8 {
    sample_camera_rgbd(render, transform, spec, sim_time, &RenderScene::new()).rgb
}

/// Samples RGB and depth using scene geometry when provided.
pub fn sample_camera_rgbd<R: RenderBackend + ?Sized>(
    render: &mut R,
    transform: &Transform3,
    spec: &CameraSpec,
    sim_time: SimTime,
    scene: &RenderScene,
) -> CameraRgbdSample {
    sample_camera_rgbd_keyed(
        render,
        transform,
        spec,
        sim_time,
        scene,
        SensorNoiseKey::new(0, spec.seed, 0, 0),
    )
}

/// Samples RGB and depth with stateless deterministic sensor noise.
pub fn sample_camera_rgbd_keyed<R: RenderBackend + ?Sized>(
    render: &mut R,
    transform: &Transform3,
    spec: &CameraSpec,
    sim_time: SimTime,
    scene: &RenderScene,
    noise_key: SensorNoiseKey,
) -> CameraRgbdSample {
    sample_camera_rgbd_swept(
        render,
        &CameraSweep::stationary(*transform),
        spec,
        sim_time,
        scene,
        noise_key,
    )
}

/// Samples a frame whose rows are read out from a moving sensor pose.
///
/// Use this when the platform moves appreciably within [`CameraSpec::readout_time_s`]:
/// each band is rendered from the pose interpolated at its own readout time, which is
/// the rolling-shutter skew a real CMOS sensor produces. A stationary sweep or a single
/// band reduces exactly to a global-shutter capture.
pub fn sample_camera_rgbd_swept<R: RenderBackend + ?Sized>(
    render: &mut R,
    sweep: &CameraSweep,
    spec: &CameraSpec,
    sim_time: SimTime,
    scene: &RenderScene,
    noise_key: SensorNoiseKey,
) -> CameraRgbdSample {
    let camera = Camera::new(spec.width, spec.height, spec.fov_y_rad);
    let output = render_frame(render, &camera, sweep, spec, sim_time, scene);

    let mut rgba8 = output.color.rgba8;
    let mut depth_m = output.depth.depth_m;
    apply_sensor_response(&mut rgba8, &mut depth_m, spec, noise_key);

    CameraRgbdSample {
        rgb: ImageRgb8::from_rgba8(output.color.width, output.color.height, rgba8),
        depth: ImageDepth::new(output.depth.width, output.depth.height, depth_m),
    }
}

/// Renders the frame, splitting it into rolling-shutter bands when required.
fn render_frame<R: RenderBackend + ?Sized>(
    render: &mut R,
    camera: &Camera,
    sweep: &CameraSweep,
    spec: &CameraSpec,
    sim_time: SimTime,
    scene: &RenderScene,
) -> CameraPassOutput {
    let bands = spec.effective_band_count();
    if bands <= 1 || sweep.is_stationary() || spec.height == 0 {
        return render_pass(render, camera, &sweep.start, spec, sim_time, scene);
    }

    let mut color: Option<ImageFrame> = None;
    let mut depth: Option<DepthFrame> = None;
    for band in 0..bands {
        // Sample the sweep at the middle of each band's readout window.
        let fraction = (f64::from(band) + 0.5) / f64::from(bands);
        let pass = render_pass(
            render,
            camera,
            &sweep.pose_at(fraction),
            spec,
            sim_time,
            scene,
        );
        let (first_row, last_row) = band_rows(spec.height, bands, band);

        let color_target = color.get_or_insert_with(|| pass.color.clone());
        copy_rows(
            &mut color_target.rgba8,
            &pass.color.rgba8,
            spec.width as usize * 4,
            first_row,
            last_row,
        );
        let depth_target = depth.get_or_insert_with(|| pass.depth.clone());
        copy_rows(
            &mut depth_target.depth_m,
            &pass.depth.depth_m,
            spec.width as usize,
            first_row,
            last_row,
        );
    }

    match (color, depth) {
        (Some(color), Some(depth)) => CameraPassOutput { color, depth },
        _ => render_pass(render, camera, &sweep.start, spec, sim_time, scene),
    }
}

fn render_pass<R: RenderBackend + ?Sized>(
    render: &mut R,
    camera: &Camera,
    pose: &Transform3,
    spec: &CameraSpec,
    sim_time: SimTime,
    scene: &RenderScene,
) -> CameraPassOutput {
    let view = rne_math::Transform3 {
        translation: pose.translation,
        rotation: pose.rotation,
        scale: pose.scale,
    };

    if scene.items.is_empty() {
        let frame = render
            .render_camera(camera, &view, [0.05, 0.08, 0.12, 1.0], sim_time, spec.seed)
            .expect("camera render");
        CameraPassOutput {
            color: frame,
            depth: DepthFrame::new(
                spec.width,
                spec.height,
                vec![camera.far_m as f32; (spec.width * spec.height) as usize],
            ),
        }
    } else {
        render
            .render_scene_camera(camera, &view, scene, [0.05, 0.08, 0.12, 1.0])
            .expect("camera scene render")
    }
}

/// Returns the half-open row range owned by `band`.
fn band_rows(height: u32, bands: u16, band: u16) -> (usize, usize) {
    let height = height as usize;
    let bands = usize::from(bands.max(1));
    let band = usize::from(band);
    let first = height * band / bands;
    let last = height * (band + 1) / bands;
    (first, last.min(height))
}

fn copy_rows<T: Copy>(
    target: &mut [T],
    source: &[T],
    row_stride: usize,
    first: usize,
    last: usize,
) {
    if row_stride == 0 {
        return;
    }
    let start = first * row_stride;
    let end = (last * row_stride).min(target.len()).min(source.len());
    if start >= end {
        return;
    }
    target[start..end].copy_from_slice(&source[start..end]);
}

/// Applies the optical and electronic sensor response in place.
fn apply_sensor_response(
    rgba8: &mut Vec<u8>,
    depth_m: &mut Vec<f32>,
    spec: &CameraSpec,
    noise_key: SensorNoiseKey,
) {
    if spec.is_ideal() || spec.width == 0 || spec.height == 0 {
        return;
    }

    if !spec.distortion.is_identity() {
        *rgba8 = distort_color(rgba8, spec);
        *depth_m = distort_depth(depth_m, spec);
    }
    if spec.vignette_strength > 0.0 {
        apply_vignette(rgba8, spec);
    }
    let gain = exposure_gain(rgba8, spec);
    if gain != 1.0 {
        apply_gain(rgba8, gain);
    }
    if spec.shot_noise_scale > 0.0 || spec.read_noise_stddev > 0.0 {
        apply_noise(rgba8, spec, noise_key);
    }
}

/// Returns the source sample position in pixels for an output pixel.
///
/// Output pixel coordinates are treated as ideal pinhole coordinates, and the forward
/// Brown-Conrady model gives the position to read from the rendered image. Positions
/// outside the rendered image clamp to the edge rather than introducing black borders
/// that later stages would then amplify.
fn source_position(spec: &CameraSpec, px: u32, py: u32) -> (f64, f64) {
    let focal_px = spec.focal_length_px();
    let center_x = f64::from(spec.width) * 0.5;
    let center_y = f64::from(spec.height) * 0.5;
    let x = (f64::from(px) + 0.5 - center_x) / focal_px;
    let y = (f64::from(py) + 0.5 - center_y) / focal_px;
    let (x_d, y_d) = spec.distortion.distort(x, y);
    (
        x_d * focal_px + center_x - 0.5,
        y_d * focal_px + center_y - 0.5,
    )
}

fn distort_color(rgba8: &[u8], spec: &CameraSpec) -> Vec<u8> {
    let width = spec.width as usize;
    let height = spec.height as usize;
    let mut output = vec![0_u8; width * height * 4];

    for py in 0..spec.height {
        for px in 0..spec.width {
            let (source_x, source_y) = source_position(spec, px, py);
            let target = (py as usize * width + px as usize) * 4;
            for channel in 0..4 {
                output[target + channel] =
                    bilinear_channel(rgba8, width, height, source_x, source_y, channel);
            }
        }
    }
    output
}

fn distort_depth(depth_m: &[f32], spec: &CameraSpec) -> Vec<f32> {
    let width = spec.width as usize;
    let height = spec.height as usize;
    let mut output = vec![0.0_f32; width * height];

    for py in 0..spec.height {
        for px in 0..spec.width {
            let (source_x, source_y) = source_position(spec, px, py);
            // Nearest neighbour: blending depth across a discontinuity would invent
            // surfaces that do not exist.
            let sx = (source_x.round().max(0.0) as usize).min(width.saturating_sub(1));
            let sy = (source_y.round().max(0.0) as usize).min(height.saturating_sub(1));
            let target = py as usize * width + px as usize;
            if let Some(value) = depth_m.get(sy * width + sx) {
                output[target] = *value;
            }
        }
    }
    output
}

fn bilinear_channel(
    rgba8: &[u8],
    width: usize,
    height: usize,
    x: f64,
    y: f64,
    channel: usize,
) -> u8 {
    if width == 0 || height == 0 {
        return 0;
    }
    let x = x.clamp(0.0, (width - 1) as f64);
    let y = y.clamp(0.0, (height - 1) as f64);
    let x0 = x.floor() as usize;
    let y0 = y.floor() as usize;
    let x1 = (x0 + 1).min(width - 1);
    let y1 = (y0 + 1).min(height - 1);
    let tx = x - x0 as f64;
    let ty = y - y0 as f64;

    let at = |row: usize, column: usize| -> f64 {
        rgba8
            .get((row * width + column) * 4 + channel)
            .map(|value| f64::from(*value))
            .unwrap_or(0.0)
    };

    let top = at(y0, x0) * (1.0 - tx) + at(y0, x1) * tx;
    let bottom = at(y1, x0) * (1.0 - tx) + at(y1, x1) * tx;
    (top * (1.0 - ty) + bottom * ty).round().clamp(0.0, 255.0) as u8
}

/// Applies a cos^4 illumination falloff away from the optical axis.
fn apply_vignette(rgba8: &mut [u8], spec: &CameraSpec) {
    let strength = spec.vignette_strength.clamp(0.0, 1.0);
    let width = spec.width as usize;
    let half_extent = f64::from(spec.height.max(1)) * 0.5;
    let tan_half = (spec.fov_y_rad * 0.5).tan();
    let center_x = f64::from(spec.width) * 0.5;
    let center_y = f64::from(spec.height) * 0.5;

    for py in 0..spec.height {
        for px in 0..spec.width {
            let tan_x = (f64::from(px) + 0.5 - center_x) / half_extent * tan_half;
            let tan_y = (f64::from(py) + 0.5 - center_y) / half_extent * tan_half;
            let cos_theta = 1.0 / (1.0 + tan_x * tan_x + tan_y * tan_y).sqrt();
            let factor = 1.0 - strength * (1.0 - cos_theta.powi(4));
            let offset = (py as usize * width + px as usize) * 4;
            for channel in 0..3 {
                if let Some(value) = rgba8.get_mut(offset + channel) {
                    *value = (f64::from(*value) * factor).round().clamp(0.0, 255.0) as u8;
                }
            }
        }
    }
}

/// Returns the linear gain applied by manual or automatic exposure.
fn exposure_gain(rgba8: &[u8], spec: &CameraSpec) -> f64 {
    if spec.auto_exposure_target_luminance > 0.0 {
        let mean = mean_luminance(rgba8);
        let limit = 2.0_f64.powf(spec.auto_exposure_max_ev.max(0.0));
        if mean <= f64::EPSILON {
            return limit;
        }
        return (spec.auto_exposure_target_luminance / mean).clamp(1.0 / limit, limit);
    }
    2.0_f64.powf(spec.exposure_ev)
}

fn mean_luminance(rgba8: &[u8]) -> f64 {
    if rgba8.len() < 4 {
        return 0.0;
    }
    let pixels = rgba8.len() / 4;
    let mut total = 0.0;
    for pixel in rgba8.chunks_exact(4) {
        total += LUMA_WEIGHTS[0] * f64::from(pixel[0])
            + LUMA_WEIGHTS[1] * f64::from(pixel[1])
            + LUMA_WEIGHTS[2] * f64::from(pixel[2]);
    }
    total / (pixels as f64 * 255.0)
}

fn apply_gain(rgba8: &mut [u8], gain: f64) {
    for pixel in rgba8.chunks_exact_mut(4) {
        for value in pixel.iter_mut().take(3) {
            *value = (f64::from(*value) * gain).round().clamp(0.0, 255.0) as u8;
        }
    }
}

/// Adds signal-dependent shot noise and a constant read-noise floor.
fn apply_noise(rgba8: &mut [u8], spec: &CameraSpec, noise_key: SensorNoiseKey) {
    let random = KeyedRandom::new(
        noise_key.root_seed,
        CAMERA_RANDOM_DOMAIN_V1 ^ mix64(noise_key.sensor_seed),
    );
    let shot_scale = spec.shot_noise_scale.max(0.0);
    let read_stddev = spec.read_noise_stddev.max(0.0);

    for (pixel_index, pixel) in rgba8.chunks_exact_mut(4).enumerate() {
        let base_slot = (pixel_index as u64).wrapping_mul(PIXEL_SLOT_STRIDE);
        for (channel, value) in pixel.iter_mut().take(3).enumerate() {
            let signal = f64::from(*value) / 255.0;
            // Photon arrivals are Poisson, so the noise variance scales with the signal.
            // The read-noise floor is signal-independent and adds in quadrature.
            let variance = shot_scale * signal + read_stddev * read_stddev;
            if variance <= 0.0 {
                continue;
            }
            let slot = base_slot + (channel as u64) * 2;
            let (sample, _) = gaussian_pair(&random, noise_key, slot);
            let noisy = signal + sample * variance.sqrt();
            *value = (noisy.clamp(0.0, 1.0) * 255.0).round() as u8;
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use rne_math::{Quat, Vec3};
    use rne_render::HeadlessRenderBackend;

    fn gradient_frame(spec: &CameraSpec) -> Vec<u8> {
        let mut rgba8 = Vec::with_capacity((spec.width * spec.height * 4) as usize);
        for py in 0..spec.height {
            for px in 0..spec.width {
                let value = ((px + py) % 200 + 28) as u8;
                rgba8.extend_from_slice(&[value, value / 2 + 40, 200 - value / 2, 255]);
            }
        }
        rgba8
    }

    fn test_spec() -> CameraSpec {
        CameraSpec {
            width: 32,
            height: 24,
            ..CameraSpec::default()
        }
    }

    #[test]
    fn camera_sensor_returns_image_payload() {
        let mut backend = HeadlessRenderBackend::new();
        let spec = CameraSpec {
            width: 16,
            height: 12,
            ..CameraSpec::default()
        };
        let image = sample_camera(
            &mut backend,
            &Transform3::default(),
            &spec,
            SimTime::from_ticks(10),
        );

        assert_eq!(image.width, 16);
        assert_eq!(image.height, 12);
        assert_eq!(image.rgba8.len(), 16 * 12 * 4);
    }

    #[test]
    fn default_spec_is_an_exact_no_op() {
        let spec = test_spec();
        assert!(spec.is_ideal());

        let original = gradient_frame(&spec);
        let mut rgba8 = original.clone();
        let mut depth_m = vec![7.5_f32; (spec.width * spec.height) as usize];
        let original_depth = depth_m.clone();

        apply_sensor_response(
            &mut rgba8,
            &mut depth_m,
            &spec,
            SensorNoiseKey::new(1, 2, 3, 4),
        );

        assert_eq!(rgba8, original);
        assert_eq!(depth_m, original_depth);
    }

    #[test]
    fn legacy_serialized_spec_deserializes_with_ideal_defaults() {
        let legacy = r#"{"width":320,"height":180,"fov_y_rad":0.8,"seed":7}"#;
        let spec: CameraSpec = serde_json::from_str(legacy).expect("legacy camera spec");

        assert_eq!(spec.width, 320);
        assert_eq!(spec.height, 180);
        assert_eq!(spec.seed, 7);
        assert!(spec.is_ideal());
        assert_eq!(spec.effective_band_count(), 1);
        assert_eq!(spec.auto_exposure_max_ev, 4.0);
    }

    #[test]
    fn zero_distortion_is_the_identity_mapping() {
        let spec = test_spec();
        assert!(spec.distortion.is_identity());

        for (x, y) in [(0.0, 0.0), (0.3, -0.7), (-1.2, 0.4)] {
            let (dx, dy) = spec.distortion.distort(x, y);
            assert_relative_eq!(dx, x, epsilon = 1e-15);
            assert_relative_eq!(dy, y, epsilon = 1e-15);
        }

        // With no distortion the resampling grid is the pixel grid itself.
        let (source_x, source_y) = source_position(&spec, 7, 5);
        assert_relative_eq!(source_x, 7.0, epsilon = 1e-9);
        assert_relative_eq!(source_y, 5.0, epsilon = 1e-9);
    }

    #[test]
    fn brown_conrady_matches_a_hand_computed_sample() {
        let distortion = CameraDistortion {
            k1: 0.1,
            k2: 0.02,
            k3: 0.0,
            p1: 0.01,
            p2: -0.005,
        };
        let (x, y) = (0.4_f64, -0.3_f64);

        let r2 = x * x + y * y;
        let radial = 1.0 + 0.1 * r2 + 0.02 * r2 * r2;
        let expected_x = x * radial + 2.0 * 0.01 * x * y + (-0.005) * (r2 + 2.0 * x * x);
        let expected_y = y * radial + 0.01 * (r2 + 2.0 * y * y) + 2.0 * (-0.005) * x * y;

        let (dx, dy) = distortion.distort(x, y);
        assert_relative_eq!(dx, expected_x, epsilon = 1e-15);
        assert_relative_eq!(dy, expected_y, epsilon = 1e-15);
    }

    #[test]
    fn barrel_distortion_samples_inward_and_pincushion_outward() {
        let barrel = CameraSpec {
            distortion: CameraDistortion {
                k1: -0.3,
                ..CameraDistortion::default()
            },
            ..test_spec()
        };
        let pincushion = CameraSpec {
            distortion: CameraDistortion {
                k1: 0.3,
                ..CameraDistortion::default()
            },
            ..test_spec()
        };
        let ideal = test_spec();

        let centre_x = f64::from(ideal.width) * 0.5;
        let ideal_offset = (source_position(&ideal, 0, 0).0 - centre_x).abs();
        let barrel_offset = (source_position(&barrel, 0, 0).0 - centre_x).abs();
        let pincushion_offset = (source_position(&pincushion, 0, 0).0 - centre_x).abs();

        // Barrel distortion magnifies the centre, so a corner reads from nearer the axis.
        assert!(barrel_offset < ideal_offset);
        assert!(pincushion_offset > ideal_offset);
    }

    #[test]
    fn distortion_changes_the_image_but_preserves_its_size() {
        let spec = CameraSpec {
            distortion: CameraDistortion {
                k1: -0.25,
                ..CameraDistortion::default()
            },
            ..test_spec()
        };
        let original = gradient_frame(&spec);
        let mut rgba8 = original.clone();
        let mut depth_m = (0..spec.width * spec.height)
            .map(|index| index as f32 * 0.01)
            .collect::<Vec<_>>();
        let original_depth = depth_m.clone();

        apply_sensor_response(
            &mut rgba8,
            &mut depth_m,
            &spec,
            SensorNoiseKey::new(1, 1, 1, 1),
        );

        assert_eq!(rgba8.len(), original.len());
        assert_eq!(depth_m.len(), original_depth.len());
        assert_ne!(rgba8, original);
        assert_ne!(depth_m, original_depth);
        // Depth stays a resampling of real depths rather than a blend of them.
        assert!(depth_m.iter().all(|value| original_depth.contains(value)));
    }

    #[test]
    fn vignette_darkens_corners_more_than_the_centre() {
        let spec = CameraSpec {
            vignette_strength: 1.0,
            fov_y_rad: 1.2,
            ..test_spec()
        };
        let mut rgba8 = vec![200_u8; (spec.width * spec.height * 4) as usize];
        for pixel in rgba8.chunks_exact_mut(4) {
            pixel[3] = 255;
        }

        apply_vignette(&mut rgba8, &spec);

        let at = |px: u32, py: u32| {
            let offset = (py as usize * spec.width as usize + px as usize) * 4;
            rgba8[offset]
        };
        let centre = at(spec.width / 2, spec.height / 2);
        let corner = at(0, 0);

        assert!(
            corner < centre,
            "corner {corner} must be darker than centre {centre}"
        );
        assert!(centre > 190);
        // Alpha is an optical no-op.
        assert!(rgba8.chunks_exact(4).all(|pixel| pixel[3] == 255));
    }

    #[test]
    fn exposure_scales_brightness_monotonically() {
        let spec = test_spec();
        let base = gradient_frame(&spec);
        let unchanged = mean_luminance(&base);

        let mut brighter = base.clone();
        let brighter_gain = exposure_gain(
            &brighter,
            &CameraSpec {
                exposure_ev: 1.0,
                ..spec
            },
        );
        apply_gain(&mut brighter, brighter_gain);

        let mut darker = base.clone();
        let darker_gain = exposure_gain(
            &darker,
            &CameraSpec {
                exposure_ev: -1.0,
                ..spec
            },
        );
        apply_gain(&mut darker, darker_gain);

        assert!(mean_luminance(&darker) < unchanged);
        assert!(mean_luminance(&brighter) > unchanged);
        // One stop down halves the signal, up to quantization.
        assert_relative_eq!(
            mean_luminance(&darker),
            unchanged * 0.5,
            max_relative = 0.02
        );
    }

    #[test]
    fn auto_exposure_drives_mean_luminance_toward_the_target() {
        let spec = CameraSpec {
            auto_exposure_target_luminance: 0.5,
            auto_exposure_max_ev: 4.0,
            ..test_spec()
        };
        let mut rgba8 = vec![30_u8; (spec.width * spec.height * 4) as usize];
        for pixel in rgba8.chunks_exact_mut(4) {
            pixel[3] = 255;
        }
        let before = mean_luminance(&rgba8);

        let gain = exposure_gain(&rgba8, &spec);
        apply_gain(&mut rgba8, gain);
        let after = mean_luminance(&rgba8);

        assert!(gain > 1.0);
        assert!(after > before);
        assert_relative_eq!(after, 0.5, max_relative = 0.05);
    }

    #[test]
    fn auto_exposure_respects_the_stop_limit() {
        let spec = CameraSpec {
            auto_exposure_target_luminance: 1.0,
            auto_exposure_max_ev: 1.0,
            ..test_spec()
        };
        let rgba8 = vec![1_u8; (spec.width * spec.height * 4) as usize];

        // The scene is far darker than the target, so the gain saturates at +1 stop.
        assert_relative_eq!(exposure_gain(&rgba8, &spec), 2.0, epsilon = 1e-12);
    }

    #[test]
    fn sensor_noise_is_repeatable_and_key_dependent() {
        let spec = CameraSpec {
            shot_noise_scale: 0.01,
            read_noise_stddev: 0.02,
            ..test_spec()
        };
        let base = gradient_frame(&spec);
        let key = SensorNoiseKey::new(11, 22, 33, 44);

        let mut first = base.clone();
        apply_noise(&mut first, &spec, key);
        let mut second = base.clone();
        apply_noise(&mut second, &spec, key);
        let mut other = base.clone();
        apply_noise(
            &mut other,
            &spec,
            SensorNoiseKey {
                sample_index: 45,
                ..key
            },
        );

        assert_eq!(first, second);
        assert_ne!(first, other);
        assert_ne!(first, base);
        // Noise never touches alpha.
        assert!(first.chunks_exact(4).all(|pixel| pixel[3] == 255));
    }

    #[test]
    fn shot_noise_grows_with_signal_but_read_noise_does_not() {
        let key = SensorNoiseKey::new(5, 6, 7, 8);
        let shot_only = CameraSpec {
            shot_noise_scale: 0.05,
            ..test_spec()
        };
        let read_only = CameraSpec {
            read_noise_stddev: 0.05,
            ..test_spec()
        };
        let pixels = (shot_only.width * shot_only.height) as usize;

        let deviation = |spec: &CameraSpec, level: u8| {
            let mut rgba8 = Vec::with_capacity(pixels * 4);
            for _ in 0..pixels {
                rgba8.extend_from_slice(&[level, level, level, 255]);
            }
            apply_noise(&mut rgba8, spec, key);
            let total: f64 = rgba8
                .chunks_exact(4)
                .map(|pixel| (f64::from(pixel[0]) - f64::from(level)).abs())
                .sum();
            total / pixels as f64
        };

        // Shot noise scales with the square root of the signal.
        assert!(deviation(&shot_only, 200) > deviation(&shot_only, 20) * 1.5);
        // Read noise is a constant floor, so it barely changes with signal level.
        let dark = deviation(&read_only, 60);
        let bright = deviation(&read_only, 200);
        assert!((bright - dark).abs() < dark * 0.5);
    }

    #[test]
    fn rolling_shutter_reduces_to_a_global_shutter_when_stationary() {
        let spec = CameraSpec {
            width: 24,
            height: 16,
            readout_time_s: 0.02,
            rolling_shutter_bands: 8,
            ..CameraSpec::default()
        };
        let pose = Transform3::from_translation_rotation(Vec3::new(0.0, 0.0, 2.0), Quat::IDENTITY);
        let key = SensorNoiseKey::new(2, 3, 4, 5);

        let mut backend = HeadlessRenderBackend::new();
        let swept = sample_camera_rgbd_swept(
            &mut backend,
            &CameraSweep::stationary(pose),
            &spec,
            SimTime::from_ticks(3),
            &RenderScene::new(),
            key,
        );
        let mut backend = HeadlessRenderBackend::new();
        let global = sample_camera_rgbd_keyed(
            &mut backend,
            &pose,
            &CameraSpec {
                rolling_shutter_bands: 1,
                readout_time_s: 0.0,
                ..spec
            },
            SimTime::from_ticks(3),
            &RenderScene::new(),
            key,
        );

        assert_eq!(swept, global);
    }

    #[test]
    fn band_rows_tile_the_frame_without_gaps_or_overlap() {
        let height = 17_u32;
        let bands = 5_u16;
        let mut covered = 0_usize;
        let mut previous_end = 0_usize;

        for band in 0..bands {
            let (first, last) = band_rows(height, bands, band);
            assert_eq!(first, previous_end);
            assert!(last >= first);
            covered += last - first;
            previous_end = last;
        }

        assert_eq!(previous_end, height as usize);
        assert_eq!(covered, height as usize);
    }

    #[test]
    fn band_rendering_composites_each_band_from_its_own_pose() {
        // A synthetic backend that paints every pixel with the camera's x position, so
        // each band is identifiable in the composited frame.
        struct PoseCodedBackend {
            width: u32,
            height: u32,
        }

        impl RenderBackend for PoseCodedBackend {
            fn render_clear(
                &mut self,
                target: rne_render::RenderTarget,
                _clear_color: [f32; 4],
            ) -> Result<ImageFrame, rne_render::RenderError> {
                Ok(ImageFrame {
                    width: target.width,
                    height: target.height,
                    rgba8: vec![0; (target.width * target.height * 4) as usize],
                })
            }

            fn render_camera(
                &mut self,
                _camera: &Camera,
                view: &rne_math::Transform3,
                _clear_color: [f32; 4],
                _sim_time: SimTime,
                _seed: u64,
            ) -> Result<ImageFrame, rne_render::RenderError> {
                let code = view.translation.x.round().clamp(0.0, 255.0) as u8;
                Ok(ImageFrame {
                    width: self.width,
                    height: self.height,
                    rgba8: (0..self.width * self.height)
                        .flat_map(|_| [code, code, code, 255])
                        .collect(),
                })
            }
        }

        let spec = CameraSpec {
            width: 4,
            height: 8,
            readout_time_s: 0.01,
            rolling_shutter_bands: 4,
            ..CameraSpec::default()
        };
        let mut backend = PoseCodedBackend {
            width: spec.width,
            height: spec.height,
        };
        let start = Transform3::from_translation_rotation(Vec3::ZERO, Quat::IDENTITY);
        let end = Transform3::from_translation_rotation(Vec3::new(80.0, 0.0, 0.0), Quat::IDENTITY);

        let sample = sample_camera_rgbd_swept(
            &mut backend,
            &CameraSweep::new(start, end),
            &spec,
            SimTime::ZERO,
            &RenderScene::new(),
            SensorNoiseKey::new(0, 0, 0, 0),
        );

        // Band i is rendered at fraction (i + 0.5) / 4, so x = 10, 30, 50, 70.
        let row_code = |row: u32| sample.rgb.rgba8[(row * spec.width * 4) as usize];
        assert_eq!(row_code(0), 10);
        assert_eq!(row_code(2), 30);
        assert_eq!(row_code(4), 50);
        assert_eq!(row_code(6), 70);
        // Rows within one band share a pose.
        assert_eq!(row_code(0), row_code(1));
    }

    #[test]
    fn row_readout_times_span_the_configured_window() {
        let spec = CameraSpec {
            height: 100,
            readout_time_s: 0.033,
            ..CameraSpec::default()
        };

        assert_relative_eq!(spec.row_time_s(0), 0.0, epsilon = 1e-15);
        assert_relative_eq!(spec.row_time_s(99), 0.033, epsilon = 1e-15);
        // Out-of-range rows clamp instead of extrapolating.
        assert_relative_eq!(spec.row_time_s(500), 0.033, epsilon = 1e-15);

        // A global shutter reports a single instant for every row.
        let global = CameraSpec {
            readout_time_s: 0.0,
            ..spec
        };
        assert_relative_eq!(global.row_time_s(50), 0.0, epsilon = 1e-15);
    }
}

# Physics-aware camera

`rne_sensor` models a camera independently of any specific render backend. The backend
supplies an ideal pinhole render and a matching linear depth buffer; every sensor effect
is applied on top of those buffers inside `rne_sensor`. No effect is pushed down into
`rne_render_wgpu`, and swapping the renderer does not change the sensor model. This
mirrors how [physics-aware LiDAR](LIDAR_SIMULATION.md) stays physics-backend-neutral.

`CameraSpec::default()` disables every effect. A default spec produces byte-identical
output to a plain backend render, so existing scenes and tests are unaffected.

## Pipeline

```text
render (optionally once per rolling-shutter band)
  -> lens distortion   (optical, geometric; resamples color and depth)
  -> vignetting        (optical, cos^4 falloff)
  -> exposure gain     (electronic)
  -> shot + read noise (electronic)
```

Optical effects precede electronic ones because that is the order light actually
encounters them: the lens forms the image, then the sensor converts and reads it.

## Lens distortion

`CameraDistortion` holds Brown-Conrady coefficients acting on normalized pinhole
coordinates. For ideal coordinates `(x, y)` with `r^2 = x^2 + y^2`:

```text
radial = 1 + k1 r^2 + k2 r^4 + k3 r^6
x_d    = x radial + 2 p1 x y + p2 (r^2 + 2 x^2)
y_d    = y radial + p1 (r^2 + 2 y^2) + 2 p2 x y
```

Negative `k1` produces barrel distortion, positive `k1` pincushion. Normalization uses
the focal length implied by the vertical field of view, `f = (height / 2) / tan(fov_y / 2)`,
with square pixels and the principal point at the image centre.

Output pixel coordinates are treated as ideal coordinates, and the forward model gives
the position to read from the rendered image. Color is sampled bilinearly. **Depth is
sampled nearest-neighbour**: blending depth across a discontinuity would invent surfaces
that do not exist. Source positions outside the rendered image clamp to the edge rather
than introducing black borders that the later stages would then amplify.

All-zero coefficients are an exact identity — the buffers are returned untouched rather
than resampled, so there is no interpolation loss when distortion is disabled.

## Rolling shutter

A CMOS sensor reads rows sequentially, so a moving camera captures the top and bottom of
a frame from different poses. `CameraSweep` describes the sensor pose at the start and
end of readout, exactly as `LidarSweep` does for one LiDAR revolution.

`sample_camera_rgbd_swept` splits the frame into `rolling_shutter_bands` horizontal
bands and renders each from the pose interpolated at the middle of its readout window,
then composites the bands into one frame. Bands tile the image with no gaps or overlap.

Two configurations reduce to a global shutter exactly, with a single render call:

* `rolling_shutter_bands <= 1`, or
* a stationary sweep.

`readout_time_s` is the time to read every row. Band count is a cost/fidelity knob: more
bands approximate continuous row-by-row readout more finely at the price of more renders.

## Timestamp, latency, and noise behavior

Sampling is driven only by explicit `SimTime`. `Frame.capture_time` is the instant row
zero is read; `CameraSpec::row_time_s(row)` gives the offset of any later row within
`readout_time_s`. Sensor output latency is separate: `Sensor.latency_ticks` produces
`Frame.available_time`. No wall-clock time is read anywhere in the model.

Every stochastic effect draws from a disjoint slot of a `SensorNoiseKey`-derived stream,
built from `WorldRandom.seed`, the camera-local seed, the DataBus stream id, and the scan
sequence. Each pixel owns a fixed slot block, so noise replays exactly for a given key
and does not change when unrelated entities are spawned in a different order.

## Exposure

Manual exposure applies a gain of `2^exposure_ev`, so the default of `0.0` is unit gain.

Auto exposure activates when `auto_exposure_target_luminance > 0`. It measures mean Rec.
709 luminance over the frame and applies

```text
gain = clamp(target / mean, 2^-max_ev, 2^+max_ev)
```

where `max_ev` is `auto_exposure_max_ev`. A fully black frame saturates at the positive
limit rather than dividing by zero.

## Sensor noise

Photon arrivals are Poisson-distributed, so noise variance grows with the signal. For a
normalized signal `s` the model uses a Gaussian whose variance combines a
signal-dependent shot term and a signal-independent read floor in quadrature:

```text
sigma = sqrt(shot_noise_scale * s + read_noise_stddev^2)
s'    = clamp(s + N(0, 1) * sigma, 0, 1)
```

Noise applies to the three color channels; alpha is left untouched.

## Vignetting

Illumination falls off away from the optical axis. For a pixel whose direction makes an
angle `theta` with the axis:

```text
factor = 1 - vignette_strength * (1 - cos^4(theta))
```

`vignette_strength = 0` disables it; `1.0` applies the full cos^4 law. `cos(theta)` is
derived from the pixel's tangent-space offset and the vertical field of view.

## Known simplification

The render backend returns 8-bit RGBA, so exposure and noise operate on a quantized
approximation of linear light rather than on true photon counts, and channel values are
treated as linear radiance proxies. This is a deliberate simplification of the same kind
as the LiDAR model's first-order extinction: a backend that grows a high-dynamic-range
surface can replace it without changing this API.

# Physics-aware LiDAR

`rne_sensor` models LiDAR independently of rendering, traffic, geospatial
importers, ROS 2, and any specific physics backend. Geometry comes from the
backend-neutral `PhysicsBackend::raycast` contract. An application or offline
importer can attach `LidarMaterial` to a hit entity without changing its visual
or contact material.

The design follows the realism dimensions described by
[ImmersiVERSE's physics-aware LiDAR work](https://www.linkedin.com/posts/immersiverse-sl_building-physics-aware-lidar-simulation-for-activity-7486066379075342338-Rk-i):
non-visual material response, physical return intensity, beam and sensor noise,
weather, and seeded domain randomization. It remains a compact deterministic
model rather than an electromagnetic or Mie-scattering solver.

## Scan geometry

A scan is a grid of `ray_count` azimuth columns by `channel_count` elevation
channels. Channel elevations are spaced uniformly between `min_elevation_rad`
and `max_elevation_rad`, which describes VLP-16/32 and Ouster OS-class sensors.
The defaults (`channel_count = 1`, zero elevation limits) keep legacy
single-plane 2D configurations behaving exactly as before.

Columns are emitted sequentially over `rotation_period_s`. `sample_lidar_swept`
takes a `LidarSweep` — the sensor pose at the start and end of the revolution —
and casts each column from the pose interpolated at its own emission time. A
moving platform therefore produces the motion distortion real spinning scanners
exhibit, rather than an idealized instantaneous snapshot. `rotation_period_s = 0`
models an instantaneous scan and suppresses distortion entirely.

Every point records its emission offset in `PointCloud.timestamps_s`, so
downstream code can de-skew the cloud with its own ego-motion estimate.

## Material and return model

`LidarMaterial` contains normalized reflectivity, transmissivity, roughness, and
a retroreflective gain. Values authored with `LidarMaterial::new` are clamped to
`[0, 1]`. Convenience presets cover clear glass, dry asphalt, concrete, painted
metal, retroreflective road signage, and licence plates.

Returned energy follows the single-scattering LiDAR equation reduced to the
terms a simulator can evaluate from geometry and material data. For range `r`,
incidence cosine `c`, roughness `q`, reflectivity `rho`, retroreflective gain
`G`, incident energy `E`, minimum range `r0`, and atmospheric extinction `beta`:

```text
p     = 1 + 4 (1 - q)
retro = 1 + (G - 1) c^8
lap   = 1 - exp(-(r / r0)^2)
I     = E rho c^p retro (10 m / r)^2 lap exp(-2 beta r) + sensor noise
```

* `(10 m / r)^2` is inverse-square spreading, normalized so a Lambertian unit
  reflector at 10 m under normal incidence returns full scale.
* `lap` is the transmitter/receiver geometric overlap form factor, which
  suppresses returns inside the crossover range. Without it the inverse-square
  term alone would diverge at close range.
* `retro` models corner-cube sheeting: road signs and licence plates return one
  to two orders of magnitude more energy, but only near normal entrance angles.
* The factor of two in the exponential represents the outgoing and returning
  paths.

Energy above `saturation_intensity` is clipped. The clipped excess optionally
blooms into neighbouring azimuth columns of the same channel, scaled by
`bloom_gain` and limited to `bloom_column_radius` — the halo real detectors show
around retroreflectors.

After a surface, remaining energy is multiplied by transmissivity, clamped so
reflection plus transmission cannot exceed one. Later ray hits can therefore
produce second or later returns through glass. `max_returns` limits work and
output size. Rapier returns all intersections sorted by distance and stable
entity ID; other physics backends must follow the same ordering contract.

## Beam footprint and mixed pixels

`beam_divergence_rad` is the full divergence angle. Two models consume it:

* `beam_sample_count = 1` treats divergence as an uncorrelated Gaussian pointing
  jitter. This is the cheap model and costs no extra raycasts.
* `beam_sample_count > 1` integrates the footprint by casting sub-rays spread
  across the divergence cone on a golden-angle spiral. The first return is
  reported at the energy-weighted mean of the sub-ray ranges, and its intensity
  is scaled by the fraction of the footprint that returned energy.

When the footprint spans a depth discontinuity larger than
`mixed_pixel_threshold_m`, the return is a *mixed pixel*: a single blended range
between the two surfaces with reduced intensity. These are the stray points that
appear on object silhouettes in real scans, and — unlike glass transmission —
they occur at every hard edge.

## Noise, weather, and occlusion

`LidarSpec` defines explicit SI fields for:

- minimum and maximum range in meters;
- wavelength in nanometers;
- full beam divergence in radians and footprint sub-sample count;
- Gaussian range noise in meters;
- Gaussian normalized-intensity noise;
- an additive solar ambient noise floor;
- return dropout probability, saturation intensity, and minimum intensity;
- fog extinction in inverse meters;
- rain and snow rates in millimeters per hour;
- dust concentration in milligrams per cubic meter.

The first-order extinction approximation is:

```text
beta = fog
     + 0.0001 rain
     + 0.00002 dust
     + 0.00015 snow
```

Weather affects a scan in three physically distinct ways:

1. **Attenuation.** `exp(-2 beta r)` is the ensemble-average energy loss.
2. **Backscatter.** With `backscatter_probability_scale > 0`, a ray may return
   from an aerosol particle instead of a surface. The range is drawn by
   inverting the truncated exponential free-path distribution, so returns
   cluster near the sensor exactly as fog returns do. Particles beyond the first
   hard surface are not illuminated, and the spurious return is merged into the
   ray's return list by distance.
3. **Discrete occlusion.** Rain and snow particles are large enough to swallow a
   whole pulse, which appears as isolated missing returns rather than a uniform
   intensity loss. Fog and dust particles are too small and are excluded from
   `LidarAtmosphere::occlusion_per_m`.

`LidarDomainRandomization` supplies inclusive per-scan ranges for all four
weather terms. The ranges are sampled statelessly from `WorldRandom.seed`, the
LiDAR-local seed, DataBus stream ID, and scan sequence. Every stochastic effect
draws from a disjoint slot in that keyed stream, so range, intensity, beam,
dropout, backscatter, and weather samples replay exactly and do not change when
unrelated entities are spawned in a different order.

The attenuation and dropout structure is comparable to the distance-based
intensity, atmospheric attenuation, dropout, and range noise documented by
[CARLA's LiDAR sensor](https://carla.readthedocs.io/en/latest/ref_sensors/).
NVIDIA's RTX LiDAR documentation likewise distinguishes material-aware returns
and atmospheric sensor modeling from ordinary visual fog; see the
[Omniverse LiDAR extension](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/omni_sensors_docs/lidar_extension.html).

## Point cloud attributes

`PointCloud.points_m` remains available to existing consumers. Physics-aware
scans additionally populate aligned arrays:

| array | meaning |
| --- | --- |
| `intensities` | normalized return intensity |
| `ray_indices` | azimuth column index |
| `return_indices` | one-based return index within the ray |
| `channel_indices` | elevation channel (ring) index |
| `timestamps_s` | emission offset from the start of the scan |

Empty attribute arrays remain valid for legacy point clouds, and legacy
serialized clouds deserialize unchanged.

The ROS 2 adapter grows the `PointCloud2` layout to match the attributes the
cloud actually carries, using the `ring` and `time` field names of the Velodyne
and Ouster drivers so existing de-skewing nodes consume it unchanged:

| attributes present | fields | `point_step` |
| --- | --- | --- |
| points only | `x y z` | 12 |
| + intensity | `x y z intensity` | 16 |
| + channel indices | `x y z intensity ring` | 20 |
| + timestamps | `x y z intensity ring time` | 24 |

## Time, latency, and failure behavior

Sampling is driven only by explicit `SimTime`. `Frame.capture_time` is the scan
time. `Sensor.latency_ticks` produces `Frame.available_time`, and consumers do
not need wall-clock time. Within a scan, `PointCloud.timestamps_s` carries the
sub-scan emission offsets. `LidarFailureBehavior::DropRay` omits one ray after a
backend query error; `DropScan` returns an empty cloud for that scan. Invalid
zero-ray, zero-return, or non-positive-range configurations also return an empty
cloud without partially publishing malformed attributes.

## Sanjo acceptance scenario

Example 46 imports the official Sanjo PLATEAU tile, runs the deterministic
100-vehicle/eight-route traffic scenario, mounts a 905 nm 16-channel spinning
LiDAR (600 azimuth columns, +/-15 degrees vertical, 9,600 rays per revolution)
on the tracked vehicle, and raycasts against 213 imported building colliders,
the other 99 vehicles, and their retroreflective licence plates. Concrete,
glass, asphalt, painted-metal, and retroreflective properties are non-visual ECS
data. The scanner sweeps one revolution per rendered frame, so each column is
cast from the interpolated pose of the moving host vehicle.

The headless acceptance test captures twelve frames twice with opposite
traffic-collider spawn order and requires:

- stable hash `13248311255248989536`;
- identical point, multiple-return, and saturated-return counts;
- aligned point attributes;
- at least one transmitted later return;
- at least one saturated retroreflective return;
- returns on all sixteen elevation channels spanning more than two meters
  vertically;
- non-negative emission times spanning less than one revolution.

The full example reports column and channel counts, return count,
multiple-return count, saturated-return count, mean intensity, scan duration,
stable hash, traffic safety KPIs, and measured scan throughput. Its wgpu output
colors the cloud with the turbo intensity colormap real point-cloud
viewers use and writes `docs/media/plateau-lidar.gif` plus
the reduced-motion `docs/media/plateau-lidar.png` poster. The committed
144-frame capture contains 910,993 returns, including 104,901 later returns and
885 saturated returns, with mean normalized intensity `0.060`, a per-scan
duration of `0.0832 s`, and stable full-capture hash `178647125583897092`.
The reduced azimuth grid raises the measured contended-host capture rate from
10.6 Hz to 15.4 Hz while preserving all sixteen vertical channels, three
beam-footprint samples, material transmission, and the 12 Hz presentation rate.

The capture world adds a large ground-plane collider with a dim diffuse grass
material under the whole tile: the imported road surfaces only cover the
carriageway, and without ground everywhere the downward elevation channels
return nothing over grass and sidewalks, leaving the signature concentric
rings as a partial crescent. The atmosphere matches the rendered clear sunny
day — trace haze and dust, no precipitation, no aerosol backscatter — so no
returns float in visibly clear air; rain and backscatter remain covered by the
`rne_sensor` unit tests.

The mount rides a [dynamic-bicycle](VEHICLE_DYNAMICS.md) chassis rather than the
kinematic traffic actor itself: the tracked vehicle's traffic trajectory becomes
a ghost that a `VehicleDynamics` vehicle chases with pure pursuit and a short
steering-actuator lag, staying within 0.19 m of it at these urban speeds. The
swap lives in the example layer, so `rne_traffic` keeps its backend-free
contract and the other 99 actors are bit-identical to the pure-traffic replay.

### Onboard camera

The same vehicle carries a forward RGB-D camera behind the windshield, tilted
`0.045 rad` down and rendered at 320x180 with a `0.716 rad` vertical field of
view. It is a second sensor on the same actor rather than a presentation camera:
it observes the world without the LiDAR debug overlay, and its pose is derived
from the vehicle heading exactly like the LiDAR mount.

Both capture paths run the full [physics-aware camera](CAMERA_SIMULATION.md)
model, so the insets show real sensor output rather than a clean render: barrel
distortion, an 8-band rolling shutter swept across the platform motion, auto
exposure, shot and read noise, and vignetting.

Capture runs twice for different purposes:

* **Headless.** `HeadlessRenderBackend` resolves geometry through the shared
  scene depth probe rather than rasterizing, which gives a GPU-free deterministic
  acceptance signal — center and minimum depth per frame plus a stable capture
  hash. This is what CI checks.
* **wgpu.** The real renderer produces the RGB frame and linear depth buffer that
  are composited into the GIF as two picture-in-picture insets. The depth inset
  reuses the LiDAR intensity ramp — yellow near, green mid, blue far — so both
  sensors read against one legend.

The committed 144-frame headless capture reports a nearest observed depth of
`12.53 m`, a mean center depth of `43.60 m`, and stable hash
`10455576295794772416`. The acceptance test additionally requires the capture to
repeat exactly, to fill every pixel of the configured resolution, and to change
between frames as the host vehicle drives.

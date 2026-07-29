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

## Material and return model

`LidarMaterial` contains normalized reflectivity, transmissivity, and
roughness. Values authored with `LidarMaterial::new` are clamped to `[0, 1]`.
Convenience presets cover clear glass, dry asphalt, concrete, and painted
metal.

For range `r`, incidence cosine `c`, roughness `q`, reflectivity `rho`,
incident energy `E`, and atmospheric extinction `beta`, normalized intensity
is:

```text
p = 1 + 4 (1 - q)
I = E rho c^p / (1 + (r / 10 m)^2) exp(-2 beta r) + sensor noise
```

The factor of two in the exponential represents the outgoing and returning
paths. After a surface, remaining energy is multiplied by transmissivity,
clamped so reflection plus transmission cannot exceed one. Later ray hits can
therefore produce second or later returns through glass. `max_returns` limits
work and output size. Rapier returns all intersections sorted by distance and
stable entity ID; other physics backends must follow the same ordering
contract.

`PointCloud.points_m` remains available to existing consumers. Physics-aware
scans additionally populate aligned `intensities`, `ray_indices`, and
one-based `return_indices` arrays. Empty attribute arrays remain valid for
legacy point clouds.

## Beam, noise, and weather

`LidarSpec` defines explicit SI fields for:

- minimum and maximum range in meters;
- wavelength in nanometers;
- full beam divergence in radians;
- Gaussian range noise in meters;
- Gaussian normalized-intensity noise;
- return dropout probability and minimum intensity;
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

`LidarDomainRandomization` supplies inclusive per-scan ranges for all four
weather terms. The ranges are sampled statelessly from `WorldRandom.seed`,
the LiDAR-local seed, DataBus stream ID, and scan sequence. Range, intensity,
beam, dropout, and weather samples therefore replay exactly and do not change
when unrelated entities are spawned in a different order.

The attenuation and dropout structure is comparable to the distance-based
intensity, atmospheric attenuation, dropout, and range noise documented by
[CARLA's LiDAR sensor](https://carla.readthedocs.io/en/latest/ref_sensors/).
NVIDIA's RTX LiDAR documentation likewise distinguishes material-aware returns
and atmospheric sensor modeling from ordinary visual fog; see the
[Omniverse LiDAR extension](https://docs.omniverse.nvidia.com/isaacsim/latest/features/sensors_simulation/omni_sensors_docs/lidar_extension.html).

## Time, latency, and failure behavior

Sampling is driven only by explicit `SimTime`. `Frame.capture_time` is the scan
time. `Sensor.latency_ticks` produces `Frame.available_time`, and consumers do
not need wall-clock time. `LidarFailureBehavior::DropRay` omits one ray after a
backend query error; `DropScan` returns an empty cloud for that scan. Invalid
zero-ray, zero-return, or non-positive-range configurations also return an
empty cloud without partially publishing malformed attributes.

## Sanjo acceptance scenario

Example 46 imports the official Sanjo PLATEAU tile, runs the deterministic
100-vehicle/eight-route traffic scenario, mounts a 905 nm 360-ray LiDAR on the
tracked vehicle, and raycasts against 213 imported building colliders plus the
other 99 vehicles. Concrete, glass, asphalt, and painted-metal properties are
non-visual ECS data. The headless acceptance test captures twelve frames twice
with opposite traffic-collider spawn order and requires:

- stable hash `6499951982825043854`;
- identical point and multiple-return counts;
- aligned point attributes;
- at least one transmitted later return.

The full example reports return count, multiple-return count, mean intensity,
stable hash, traffic safety KPIs, and measured scan throughput. Its wgpu output
uses three intensity color bands and writes `docs/media/plateau-lidar.gif` plus
the reduced-motion `docs/media/plateau-lidar.png` poster. The committed
144-frame capture contains 36,993 returns, including 1,296 later returns, with
mean normalized intensity `0.036` and stable full-capture hash
`2990180753787339583`.

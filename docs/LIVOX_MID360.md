# Livox Mid-360 on the Unitree Go2

`rne_sensor::livox` models the Livox Mid-360 from real recordings rather than
from a guessed scan pattern. Livox does not publish the pattern, so it was
recovered from `livox_ros_driver2` output of a Go2-mounted unit and checked
against a second recording the fit never saw.

```rust
let spec = livox_mid360_spec();
let rig: LidarRigOcclusion = serde_json::from_str(&std::fs::read_to_string(
    "assets/sensors/livox_mid360/go2_rig_occlusion.json",
)?)?;
let cloud = sample_livox_mid360(
    &backend, physics_world, &world, &sweep, &spec,
    &LivoxMid360Pattern::new(), frame_index, Some(&rig), noise_key,
);
```

## Source recordings

Two ROS 2 bags from the same Go2, `EIL_Box` (107 s) and `EIL_Mask2` (34 s),
each with `/livox/lidar` (`sensor_msgs/PointCloud2`, xfer format 0:
`x y z intensity t`), `/livox/imu`, and `/leg_twist`. The model uses the first
400 and 300 frames. The recordings are private lab data and are not
redistributed; `tools/prepare_livox_mid360_model.py` regenerates every derived
file from them.

## What the recordings establish

| Property | Measured | How |
| --- | --- | --- |
| Emission order | message order; `t` is not | azimuth advances +1.32° per firing across every packet and frame boundary, while `t` jumps −270 to +795 µs at packet boundaries with zero correlation (r = 0.006) |
| Lines | 4, fired in turn | point `n` belongs to line `n % 4`; the four points of a firing sit about 1.8° apart |
| Firing period | 20.00 µs (19.997 and 20.002 µs in the two bags) | stamp span over point count |
| Packets | 96 points; frames of 207–210 packets | driver output |
| Rotor | 1.303185° per firing = 181.0 rev/s | coherence maximum of line-0 azimuth |
| Elevation nod | period 5100.66 firings = 0.1020 s | coherence maximum of line-0 elevation |
| Frame period | 99.97 ms; `header.stamp` ≈ first point `t` + 50 ms | header stamps |
| No-return slots | 38–43 % of each frame, kept as `(0, 0, 0)` | driver output |
| Mount | upside down, floor 0.440–0.454 m along sensor `+z`, tilt 0.9–2.4° | floor plane fit, 8.4 mm RMS, three windows |
| Self returns | 5.49 % / 5.48 % of all slots within 0.35 m | both bags; per-bin range agrees to 1 mm (median) |
| Mount posts | four, at −136°, −46°, +44°, +134° | 100 % no-return, 6° wide at low elevation, 16° near 45°, in both bags |
| Floor range spread | 8–11 mm robust σ within 2 m | per-frame plane fit; includes floor roughness and gait motion |
| Floor intensity | median 0–3 of 255 | the floor in these rooms is dark |

The elevation nod takes 2 % longer than a frame, so each frame starts at a
different nod phase. That is the non-repetitive behavior: in the model, 1°×1°
coverage of the field of view is 61.9 % after one frame and 97.6 % after ten.

## The pattern model

Each line's direction is a function of two phases, rotor `a = 0.0227449 n` and
nod `p = 0.0012318 n` for global firing `n`. In the rotor-aligned frame the
direction is a 2D Fourier series with rotor harmonics up to 3 and nod
harmonics up to 6 (91 terms per component). The fit alternates between the
shape and a per-frame phase offset:

| Evaluation | Median | p90 | p99 |
| --- | --- | --- | --- |
| EIL_Box, frames 0–299 (fit) | 0.095° | 0.283° | 0.582° |
| EIL_Box, frames 300–399 (held out) | 0.108° | 0.304° | 0.560° |
| EIL_Mask2, frames 150–299 (other recording, fixture) | 0.128° | 0.353° | 2.37° |

The Livox datasheet quotes an angular precision below 0.15°. The test
`pattern_matches_held_out_real_returns` checks the Rust evaluation against the
EIL_Mask2 fixture.

The per-frame phase offsets wander by about 1° (1σ) and stay within about ±2°
over 40 s. That moves points along the scan path without changing coverage, so
`LivoxMid360Pattern` leaves them at zero by default;
`with_phase_offsets_rad` reproduces a recorded frame exactly.

## Rig occlusion

`assets/sensors/livox_mid360/go2_rig_occlusion.json` holds 2°×2° bins in
Livox-frame azimuth and elevation, each with a probability of no return (the
mount posts inside the 0.1 m blind zone), a probability of a self return, and
the self-return range. The no-return part is the excess over each elevation
row's environment baseline, taking the lower of the two recordings, so only
what is fixed to the sensor remains. The self-return part is the mean of the
two recordings. Across ten simulated frames, 5.37 % of slots return from the
rig; the recordings show 5.48–5.49 %.

## Sensor settings

`livox_mid360_spec()` uses the datasheet where it has a figure: 905 nm, 0.1 m
blind zone, 70 m maximum range at 80 % reflectivity, and first returns only.
The detection threshold puts a 10 % Lambertian target at 40 m at the 50 %
detection point. Range noise (1 cm) and pointing jitter (0.09°) are the
measured floor spread and pattern residual, so both are upper bounds.

Timestamps, latency, and noise follow [`LIDAR_SIMULATION.md`](LIDAR_SIMULATION.md):
point `n` of a frame is stamped `n × 5 µs` after the frame's first firing, frame
latency is the owning sensor's `latency_ticks`, and rig draws use a keyed
stream disjoint from the other noise, so a frame replays exactly.

## What is not established

- **Timestamp jitter.** Real `t` values carry packet-level host jitter (−270
  to +795 µs). The model emits ideal emission times; reproducing the jitter
  only matters for de-skew studies and is not modeled.
- **No-return slots in the output.** The driver keeps no-return slots as zero
  points; `PointCloud` holds returns only. A Livox-format ROS 2 publisher
  would have to re-insert them.
- **Floor and wall materials.** The dark-floor dropout (no-return rates up to
  74 % at 0.4–0.8 m) comes from the environment, not the sensor. It has not
  yet been calibrated into `LidarMaterial` values.
- **Other units.** Both recordings come from one Mid-360 on one Go2. The rotor
  and nod rates agree to 0.03 % between them, but a different unit or firmware
  may differ.

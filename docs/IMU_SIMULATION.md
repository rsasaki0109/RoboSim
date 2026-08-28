# Physics-aware IMU

`rne_sensor` models an inertial measurement unit as a real part behaves, not as a
convenient readout of simulator state. The model is independent of the physics backend
and the renderer, like [physics-aware LiDAR](LIDAR_SIMULATION.md) and
[physics-aware camera](CAMERA_SIMULATION.md).

## What an IMU actually measures

A gyroscope reports angular rate about its own axes. An accelerometer reports **specific
force** — the non-gravitational acceleration it experiences — also in the sensor body
frame:

```text
omega_body = R^T * omega_world
f_body     = R^T * (a_world - g_world)
```

Two consequences are easy to get wrong and are worth stating explicitly:

* A device **at rest reads `+9.81 m/s^2` along its up axis**, not zero and not `-9.81`.
  The mounting surface pushes it up against gravity, and that reaction is what the proof
  mass feels.
* The output is in the **body frame**. Rolling the device ninety degrees moves the `1 g`
  reading onto a different axis.

Proper acceleration is derived from the change in world velocity across the sample
interval, using explicit `SimClock` time. No wall-clock time is read anywhere.

## Mount and timestamp contract

The validation path uses an explicit `ImuMount`. Its `body_from_sensor` transform
defines both sensor-axis orientation and the physical offset from the rigid-body
origin. At an offset `r`, the sampled acceleration includes the complete rigid
mount contribution

```text
a_sensor = a_origin + alpha x r + omega x (omega x r)
```

so a rotating body cannot silently treat an off-axis sensor as if it were at the
origin. Missing, scaled, non-finite, or non-unit mounts fail before any due frame
or sensor state is published.

`ImuFeedback` is the versioned raw observation contract. It names gyroscope units
as `rad/s` and accelerometer **specific force** as `m/s^2`, retains scheduled and
actual capture timing plus DataBus availability latency, and exposes per-axis
saturation. Sequence gaps and `stuck_value` status make injected failures
observable. Orientation is intentionally absent: it is estimator output or
separately labeled validation truth, not a raw IMU measurement.

## Error model

Errors follow the decomposition used for Allan-variance characterization of MEMS parts
(IEEE Std 952 and the MEMS stochastic-error literature). For a true measurement `x` over
a sample interval `dt`:

```text
measured = M x + b_turn_on + b_gm + b_rrw + n_white
```

| term | meaning | Allan deviation signature |
| --- | --- | --- |
| `M` | scale factor and axis misalignment, small-angle | — |
| `b_turn_on` | fixed turn-on bias, constant for a run | constant offset |
| `b_gm` | bias instability, first-order Gauss-Markov with correlation time `tau` | flat minimum |
| `b_rrw` | rate random walk | rising `+1/2` slope tail |
| `n_white` | angle or velocity random walk | falling `-1/2` slope |

The scale and misalignment matrix is the usual small-angle form:

```text
    | 1 + sx    -mz      my  |
M = |   mz    1 + sy    -mx  |
    |  -my      mx    1 + sz |
```

White noise is a density: its standard deviation over an interval scales as
`sigma / sqrt(dt)`, so a slower sample rate averages it down. The Gauss-Markov bias
advances as

```text
b <- b * exp(-dt / tau) + sigma_b * sqrt(1 - exp(-2 dt / tau)) * N(0, 1)
```

which is stationary — it wanders but stays bounded. Rate random walk instead accumulates
`K * sqrt(dt) * N(0, 1)` every step and grows without bound. That difference is what
separates a sensor whose bias you can calibrate out from one whose bias you cannot.

Output is finally **saturated** to the configured measurement range and then
**quantized** to the configured resolution, in that order, matching how a real part clips
before its analog-to-digital converter rounds.

## Determinism and state

Bias instability and rate random walk are time-correlated, so they need state. `ImuState`
carries that state on the sensor entity — the two bias processes plus the previous world
velocity — and `sample_sensors` writes it back with the rest of the sampling state. Every
random draw comes from a disjoint slot of a `SensorNoiseKey`-derived stream built from
`WorldRandom.seed`, the sensor seed, the DataBus stream id, and the sample counter.
Replaying the same sample sequence reproduces the same drift exactly.

`sample_imu` and `sample_imu_keyed` stay stateless. They model a device with no proper
acceleration and no bias evolution, which is the right answer for a static probe.
`sample_imu_stateful` runs the full model.

`sample_imu_stateful_diagnostic` adds mount-aware truth for validation without
placing truth into the raw payload. `sample_imu_feedback_sensors` applies the
typed schedule, latency, dropout, and stuck-value contract in deterministic
entity order. A dropped sample still advances the physical bias and kinematic
state; a stuck sample freezes only the published values.

`ImuSpec::default()` is an ideal sensor: every error term is zero and measurements pass
through untouched.

## Dead-reckoning acceptance scenario

Example 48 drives a vehicle around a 20 m circular arc at 8 m/s for twelve seconds and
integrates its IMU with a textbook strapdown update — attitude propagated by the measured
body rate, specific force rotated into the world frame, gravity restored, then integrated
twice with the trapezoid rule. Nothing corrects the estimate, so the error it accumulates
is exactly the error the sensor model produces.

```bash
cargo run --release -p imu_dead_reckoning --example 48_imu_dead_reckoning
RNE_SKIP_GPU=1 cargo run -p imu_dead_reckoning --example 48_imu_dead_reckoning
```

The IMU runs at 240 Hz, an exact multiple of the 12 Hz frame rate. That constraint is
asserted at compile time: if the rates do not divide evenly, integrated time and frame
time diverge and the mismatch shows up as drift that the sensor did not cause.

Committed results over 96.0 m travelled:

| run | final position error | fraction of distance |
| --- | --- | --- |
| ideal `ImuSpec::default()` | 0.160 m | 0.17 % |
| consumer-grade MEMS parameters | 5.54 m | 5.8 % |

The ideal figure is pure integration error and bounds how much of the modeled run is
numerical rather than physical. Stable hash of the modeled measurement stream is
`14927116494763557730`.

The acceptance tests require the ideal run to integrate back onto the truth, the modeled
run to drift further, the error to more than double between the first quarter and the
end, and two runs with the same key sequence to be bit-identical.

## Headless measurement and estimator validation

The CI-native IMU lab runs four seconds stationary and eight seconds of prescribed
roll motion at 100 Hz. A small reference complementary filter consumes only
`ImuFeedback`; truth is used afterward for scoring. The JSON and self-contained
HTML reports include stationary/motion RMSE, maximum error, normalized innovation
squared, the fraction inside a declared three-sigma band, timestamp mismatches,
trace hashes, and the first sequence/kind of an intentional failure.

```bash
cargo run -p showcase_captures --bin rne-imu-validation -- \
  --output target/imu-validation
```

The nominal trace is run twice and must hash identically. Registered gates are
`0.01 rad` stationary RMSE and `0.025 rad` prescribed-motion RMSE. The fixture
also injects one missing sequence and one stuck-value failure at sequence 650;
both must be localized at that exact observation boundary. This lab is the
sensor-side input to the OpenArm step/ramp/chirp, plant-identification, PID/state-
space, and robustness slices in the product-proof plan.

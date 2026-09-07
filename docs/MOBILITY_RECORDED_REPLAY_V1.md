# Recorded mobility source replay v1

Status: additive NCLT DataBus replay implemented; estimator integration and physical
accuracy validation remain pending. See the [source audit](MOBILITY_REAL_LOG_SOURCE_AUDIT.md)
for acquisition hashes, source definitions and limitations.

The next estimator's lag, integration and missing-interval contract is specified in
[measured-velocity estimator design](MOBILITY_MEASURED_VELOCITY_ESTIMATOR_DESIGN.md).

`recorded_nclt::replay::NcltReplay` reads both original CSV inputs through the bounded
readers, before publishing anything. It does not accept mutable parsed structs as
proof of source identity. Source wheel speeds and filtered IMU samples implement
`FramePayload` outside core crates, without ROS dependencies or fabricated counts.

## Time and observation contract

- One origin is the minimum first Unix timestamp across both streams. Subtract it
  with integer arithmetic and convert microseconds to nanosecond ticks. Preserve
  source timestamps inside payloads; independent stream origins would erase skew.
- Each stream has an explicit constant replay delay from zero through one second.
  Validate all final source-time/delay ranges with checked arithmetic at construction.
  These are experimental delivery delays, not measured sensor latency.
- `Frame.capture_time` is a source-time replay surrogate, not a certification of the
  sensor's physical capture instant. `available_time` is that surrogate plus delay.
  Original timing uncertainty, fault status and filtering remain unknown or unchanged.
- `advance_to` publishes only due samples, ordered by availability, wheels before
  IMU on ties. All events at the same boundary are applied before observation.
  `observation` uses `latest_available` on the private DataBus, not privileged truth.
- Backward time fails before state changes. Repeating the same time is allowed.
  Missing first deliveries remain `None`. After exhaustion, the latest samples retain
  their original timestamps so consumers can detect staleness; no auto-reset occurs.
- The bus retains one frame per stream. Jumping over boundaries intentionally skips
  intermediate observations. Visit `next_delivery_time` to inspect each distinct
  delivery boundary. Reconstruct from the original inputs for reset.

No frame rotation, interpolation, gravity correction, noise injection, or estimator
integration is hidden in replay. In particular, the raw-count `WheelImuOdometry`
contract is not satisfied by this source. A separate velocity-input estimator needs
explicit integration/gap/frame/uncertainty policies.

## Executable real-log path

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
cargo run -p rne_mobility_benchmark --bin rne-nclt-audit -- replay E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10\wheels.csv E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10\ms25.csv 10000
```

The final argument is the explicitly chosen common delay in microseconds (10 ms in
this example); the library supports distinct wheel/IMU delays. The CLI is read-only.
It traverses every delivery boundary and hashes both latest-available observations,
including their source values, sequence, capture/availability and decision ticks.
The digest also binds input hashes, shared origin and replay policy, with a v1
domain separator. Entity allocation is excluded. Reports explicitly deny physical
accuracy and measured-latency claims. This is not a world-state hash, signed evidence,
Failure Capsule, or cross-backend physics comparison.

Synthetic tests cover shared origin, delayed and tied delivery, no future leakage,
unaltered IMU axes, deterministic reconstruction, large-step/latest-frame behavior,
backward rejection without mutation, malformed source/policy, checked arithmetic
and policy-bound CLI digests. No dataset download is required by CI.

The acquired 2013-01-10 files were replayed at the example's 10 ms delay: 42,276
wheel and 48,324 IMU samples yielded 90,600 distinct delivery boundaries. Shared
origin was 1357847237276758 microseconds; final decision was 1025990101000 ticks.
Two complete CLI runs produced identical reports and observation SHA-256
`43ba215e8e0e2783bb8398dc81f36bba4e059ed96ef161a48462aece560ff5f9`.
This proves reproducibility for this source/policy, not physical accuracy.

## Causal pairing audit for the next estimator

A read-only merge of the original timestamp columns pairs each wheel time with the
latest IMU time less than or equal to it (never a future sample). All 42,276 wheel
rows have such an IMU predecessor. Source-time gaps are: minimum 1 us, lower median
10,800 us, upper-index 95th percentile 24,362 us, maximum 68,089 us. The sorted
zero-based indices used are `floor((N-1)*0.5)` and `ceil((N-1)*0.95)` respectively.
There are 22,492 gaps above 10 ms, 3,306 above 20 ms, and 98 above 50 ms.

These are inter-stream recorded-time separations, not measured transport latency,
clock synchronization error or proven sensor dropout. For the example's equal
10 ms replay delays, decision-time age at a wheel delivery additionally includes
that replay delay. Unequal delays require pairing by actual availability, not this
source-only merge. An estimator must declare its stale/gap policy before evaluation;
do not choose a permissive threshold just to make every sample pass. A nearest-time
join could select future IMU samples and is not a causal online baseline.

## Validation result

Formatting, Clippy and MuJoCo-enabled tests passed (105 library tests, one main
CLI test and two audit CLI tests). The full `xtask ci` passed its lint, workspace,
smoke, RL and headless stages, then exited with failure in OSS parity: the existing
`binary_frontend_streams_lossless_rgbd_with_sim_timestamps` hit its 10-second socket
read timeout. No code or timeout was changed. Re-running the complete control test
target passed 10/10; re-running all OSS parity checks passed 22/22. The previously
unreached fuzz and Behavior stages then passed 361 cases and 10/10 seeds.

This is stage-complete validation with a recorded retry, not a clean first-pass
full-CI result. The timeout's root cause remains unproven. Evidence is retained under
`E:\RNE-build\m3c-sensor`: `nclt-replay-mujoco-tests.log`, `nclt-replay-ci.log`,
`nclt-replay-parity-first-failure.json`, `nclt-replay-parity-retry.log`,
`nclt-replay-fuzz.log` and `nclt-replay-behavior.log`.

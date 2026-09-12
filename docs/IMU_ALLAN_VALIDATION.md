# IMU Allan-deviation validation

Status: statistic implementation, exact fixtures and synthetic stateful IMU-output
checks passed. Frontend capture-contract projection is implemented; physical-profile
import and calibration remain pending.

`rne_sensor::allan::overlapping_allan_deviation` accepts scalar measurements with
actual capture ticks, an exact positive sample period, and strictly increasing
positive averaging factors. It rejects nonfinite values, missing/nonuniform captures,
overflowing time arithmetic, and oversized input. It never interpolates, drops
records, rounds requested factors, or uses frame availability time as capture time.
Input limits are 1,000,000 samples and 64 factors. Runtime is O(samples * factors),
memory O(samples). The output unit follows the input; variance uses its square.

For N rate samples and m-sample means, the statistic is half the mean squared
difference of adjacent means over N-2m+1 overlapping positions. A compensated,
centered prefix sum reduces numerical accumulation error. Centering removes only
a constant, not a fitted trend. One-pair results are explicit and are not labelled
as statistically sufficient. Pair count is not independent degrees of freedom.
No confidence interval or automatic noise-profile fit is claimed.

The implementation convention was checked against
[AllanTools' source](https://github.com/aewallin/allantools/blob/ddc5bb5a46cbdc245a347ebe4f1f3c872a7371f7/allantools/allantools.py):
its rate-to-phase conversion prepends zero, yielding N+1 phase values from N rate
values, and its overlapping phase statistic therefore uses N-2m+1 pairs. RNE does
not adopt its input trimming. Tests use an independent direct-mean oracle and
hand-computed constant, linear-ramp and alternating sequences, plus offset/scale
invariance and explicit invalid-input rejection. No external package is required.

## Feedback capture contract

`allan::feedback::analyze_imu_feedback` connects a capture-ordered segment of typed
DataBus `Frame<ImuFeedback>` values to six-axis Allan statistics. Before projection
it requires one stream/entity, a supported schema, positive contiguous sequences,
finite values, nominal status, clear saturation flags, and consistent capture phase.
Actual and scheduled times must each advance by the declared exact period.
Availability must be no earlier than capture and no later than the explicit
observation cutoff; the compatibility timestamp must match availability. Delivery
latencies may vary or reorder without replacing capture time in the computation.

Segments may explicitly start after startup, but internal gaps are rejected rather
than bridged. No frames are sorted, discarded or repaired. The returned statistics
retain stream/entity and first/last sequence and capture coordinates. Acceptance
does not establish stationarity: actual motion also contributes to Allan deviation.
Tests use the real frontend and DataBus with a changing angular-rate signal, and
exercise dropout, stuck output, availability cutoffs and corrupt metadata. Actual
gyro and accelerometer range clipping is rejected, while the unclipped gyro prefix
is accepted with an averaging factor that fits its length. This
is an offline analysis boundary, not a live controller or physical certificate.

## IMU parameter interpretation

The existing IMU recurrence is unchanged. Only incorrect documentation is corrected:

- `random_walk` is white-noise density N, in measurement-unit / sqrt(Hz).
  Positive-interval sample noise scales as N/sqrt(dt).
- `rate_random_walk` is K in measurement-unit / sqrt(s). Its increments scale as
  K*sqrt(dt). For gyro rates, K is rad/s^1.5, not (rad/s)/s^1.5.
- The compatibility-named `bias_instability` parameter is the stationary standard
  deviation of a first-order Gauss-Markov bias. It is not a flicker-noise B parameter
  or a flat Allan-deviation level. Existing serialized profiles are not reinterpreted.

The primary [inertial noise analysis reference](https://www.mathworks.com/help/nav/ug/inertial-sensor-noise-analysis-using-allan-variance.html)
distinguishes white, Brownian rate-walk and flicker contributions. Its continuous-time
rate-walk Allan variance K²*tau/3 is an asymptotic guide for this discrete generator.
For adjacent means of m instantaneous discrete random-walk samples with independent
increment variance K²*dt, summing squared increment weights instead gives
K²*dt*(2m²+1)/(6m). At m=1 this is K²*dt/2. An independent increment-weight calculation
matched that expression for m=1,2,4,8,16,64; it must not be mistaken for a sensor run.

## Remaining qualification

The actual stateful IMU sampler now passes noise-only tests for all six axes, sample
periods 10 and 40 ms, averaging factors 1/4/16/64, and seeds 11/29/47/83. Each seed
contributes 8,192 retained samples after an explicit time-zero initialization.
White, rate-walk and GM terms are isolated. Each axis's variance is averaged across
the four seeds and checked separately against its discrete expectation with a
predeclared 25% relative regression tolerance. The GM oracle sums the stationary
AR(1) covariance matrix of adjacent averages, independently of the prefix-sum
statistic. The test does not average axes together or claim a confidence level.
On 2026-09-08, all seven focused Allan tests, all 86 sensor crate tests, doc-tests
and sensor all-target Clippy passed. Full `cargo run -p xtask -- ci` subsequently
passed with exit 0 for frozen commit `6d7e7d1c7e7dc4ee390d72a973e424ad46b80ae6`,
including OSS parity, 361 fuzz cases and 10/10 Behavior CI seeds. Log:
`E:\RNE-build\m3c-sensor\imu-allan-v1-ci.log`.

Remaining work: physical profile validation. The acquired long-duration source and
its nonuniform timing and thermal limitations are documented in
[IPIN static IMU source audit](IMU_IPIN_SOURCE_AUDIT.md).
Handle startup explicitly: time-zero white noise uses a compatibility fallback, while time-zero GM
draws a stationary bias; a positive-time zero-state start is different. Check frontend
capture timestamps, latency, gaps and status before applying uniform-series analysis.
Disable legacy post-quantization noise in noise-only fixtures; test range/quantization
and faults separately. Synthetic agreement is not physical sensor calibration.

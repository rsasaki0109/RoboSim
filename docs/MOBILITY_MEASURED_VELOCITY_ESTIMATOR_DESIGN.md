# Measured wheel-speed / gyro estimator design

Status: implementation contract selected; not yet implemented or validated.
This extends [recorded source replay](MOBILITY_RECORDED_REPLAY_V1.md), not the
incremental-counter estimator. No ROS dependency is proposed.

## Time semantics first

Do not multiply an entire previous interval by the newly arrived velocity. For
example, a 1 m/s observation at source time 0 followed by 2 m/s at time 1 implies
1 m traveled over [0,1) under a zero-order hold, not 2 m. Delivery delay does not
authorize applying the second measurement retroactively over that interval.

The first baseline will use a fixed output lag for the *configured replay*:

- Let `L` be the maximum of the two explicitly configured replay delays. At decision
  time `D`, the completed source-time horizon is `H = D - L`, only when `D >= L`.
- Admit only frames actually delivered by `D`. Retain distinct sequence numbers
  and source times in bounded queues, never reach into the replay's private arrays.
- Process source events through `H` in timestamp order, applying all same-time
  updates together. Integrate the previously held values up to each event, then
  replace them. Never use an event after `H` to interpolate an earlier interval.
- Report `estimate_time_ticks = H` separately from `decision_time_ticks = D` and
  `output_lag_ticks = L`. Do not extrapolate this result to the current time without
  an explicitly separate prediction model and error assessment.
- The runner must visit every delivery boundary; a sequence gap is an input error,
  not permission to infer missing history from the latest frame. Include the final
  source timestamp plus `L` as a flush boundary, which can follow the last delivery
  when the stream delays differ. No invented sample is needed to flush history.
- Timestamp reversal, contradictory duplicate frames, queue exhaustion or a delay
  exceeding the declared bound must fail before mutating the accepted state.

This bound is known in the controlled replay, not established for the real capture.
Unknown or unbounded real delay requires a different policy. For comparison,
[robot_localization's official documentation](https://docs.ros.org/en/kinetic/api/robot_localization/html/state_estimation_nodes.html#smooth-lagged-data)
describes restoring historical filter state and processing measurements again when
lagged data arrives, with bounded history configured separately. That is a useful
later out-of-sequence filtering path, not evidence that this baseline implements it.

## Measurement and geometry contract

- Inputs are measured left/right *linear wheel speeds* and filtered IMU angular
  velocity. No encoder resolution, raw counts, motor command or privileged pose
  is fabricated. Mean wheel speed is a no-longitudinal-slip motion assumption,
  not a direct chassis-speed measurement.
- The first model is explicitly planar. NCLT's source body axes are forward/right/
  down; preserve this basis for the local estimate. Gyro z is a planar yaw-rate
  approximation, not the exact heading derivative on a pitched/rolled Segway.
  Report this limitation and do not claim full 3D state estimation.
- Initial position and heading are caller-declared local coordinates, not global
  ground truth. Bias/scale calibration values must also be explicit; zero bias is
  an assumption unless supported by a separate calibration capture.
- Integrate constant held forward speed and yaw rate over each accepted interval
  using the planar rigid-motion exponential, with a stable zero-yaw-rate limit.
  Cover straight, reverse, clockwise/counterclockwise and near-zero-rate cases.
- Any future conversion into RNE's Y-up pose must be a separately tested proper
  basis rotation, including yaw sign, not an Euler-angle column permutation.

## Missing data and uncertainty

Require caller-selected maximum hold age for each stream, recorded in evidence
before scoring. Split integration at sample-expiry boundaries, not merely at the
next delivery. Before both streams initialize, emit no valid pose interval. Never
hold stale velocities indefinitely or interpolate across an unobserved gap.

After a gap, a fresh pair may initialize a new local segment. Do not silently join
segments across unknown motion. Report segment identity, valid integrated duration,
unobserved duration and sample ages. A segment origin is a computational convention,
not a claim that the vehicle returned to zero. Long-session pose coverage must fail
when continuity is unobserved, even if later segments are healthy.

Unknown measurement covariance stays unknown; do not supply zero covariance or a
statistical confidence bound inferred from synthetic tests. A subsequent calibrated
filter must propagate measurement/process uncertainty, including wheel slip and
bias, before claiming calibrated confidence or estimator accuracy.

## Required evidence

Test a delayed step input against the no-lookahead example, unequal delays, source
time ties, initial missing data, stale intervals, recovery segments, sequence gaps,
backward calls without mutation, bounded buffers and terminal flushing. Repeated
runs must reproduce the complete observation/estimate/coverage digest. Changing
only delivery delays must preserve estimates at matching source horizons under
the declared bound, while changing decision-time availability as specified.

Run the acquired NCLT session only after synthetic timing/geometry tests pass.
Report coverage and local-segment output without a global accuracy score. Do not
use odometry-interpolated NCLT poses as an independent reference. Independent
reference qualification, calibrated uncertainty, source-to-RNE basis tests and
cross-backend estimator experiments remain required work toward the full goal.

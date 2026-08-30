# Mobility wheel/IMU odometry v1

`WheelImuOdometry` is RNE's first sensor-only mobile-state estimator. It consumes
two `IncrementalEncoderFeedback` streams and one `ImuFeedback` stream. Its public
update API cannot accept an ECS world, transform, rigid-body state, actuator
command, or ground truth. Truth belongs only in benchmark scoring.

## Contract

At each estimator decision time, v1:

1. selects only frames whose DataBus availability time has elapsed;
2. rejects stale or excessively skewed capture timestamps;
3. rejects stuck IMU/encoder values and saturated encoder counters;
4. reconstructs signed finite-counter differences across wrap;
5. rejects a modular count change above a configured physical bound, because
   motion beyond half a counter range is directionally ambiguous;
6. converts counts to left/right contact distance using declared CPR, direction,
   radius, and track width;
7. integrates bias-corrected IMU yaw rate over actual capture time;
8. blends encoder and gyro yaw increments, increasing gyro weight when their
   innovation crosses the declared disagreement/slip threshold;
9. integrates the center distance at midpoint heading and propagates a full 3x3
   planar pose covariance;
10. emits pose, twist, the two source yaw increments, innovation, health, sequence
    gaps, capture/decision times, and maximum input age.

Initialization records counter and time baselines and emits no invented motion.
Repeated encoder pairs and non-advancing capture times fail closed. Sequence gaps
remain explicit even when the finite counters permit an unambiguous accumulated
displacement. IMU saturation falls back to encoder yaw for that update.

The covariance is first-order dead-reckoning propagation from declared per-update
wheel-distance, encoder-yaw, and gyro-rate uncertainty. It is useful as visible
estimator evidence, but it is not a claim that unmodelled slip is Gaussian.

## Research and OSS correspondence

- WPILib's official `DifferentialDriveOdometry` documents periodic pose updates
  from left/right encoder distance and gyro heading, recommends distance rather
  than velocity integration for low-CPR encoders, and explicitly warns that
  encoder/gyro dead reckoning drifts under contact:
  https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/differential-drive-odometry.html
- `robot_localization` is the production OSS comparison for timestamped queues,
  sensor-specific fusion, differential measurements, and the distinction between
  locally continuous `odom` estimates and globally corrected `map` estimates:
  https://docs.ros.org/en/noetic/api/robot_localization/html/state_estimation_nodes.html
- Deray et al., *Joint on-manifold self-calibration of odometry model and sensor
  extrinsics using pre-integration*, treats wheel radii and wheel separation as
  calibration parameters and carries motion covariance and calibration Jacobians:
  https://artivis.github.io/publication/deray-ecmr-19/
- Potokar et al., *Robust Preintegrated Wheel Odometry for Off-road Autonomous
  Ground Vehicles*, models 3D wheel preintegration and slip for off-road motion:
  https://www.cs.cmu.edu/~kaess/pub/Potokar24ral.pdf
- Lee et al., *Visual-Inertial-Wheel Odometry with Online Calibration*, derives
  wheel/IMU preintegration and covariance while estimating wheel and sensor
  calibration parameters:
  https://copland.udel.edu/~ghuang/papers/tr_wheel-vio.pdf

RNE v1 intentionally stays smaller than these research estimators. It supplies a
deterministic, inspectable baseline that can run in headless tests and on recorded
DataBus frames. The source comparison prevents the baseline from being mistaken
for a tightly coupled inertial navigation or localization system.

## Known omissions and next gates

- no accelerometer integration, roll/pitch/gravity state, gyro-bias state, or
  zero-velocity update;
- no out-of-sequence rewind/replay; inputs must be synchronized within the
  configured capture-time bound;
- no EKF/UKF, factor graph, wheel preintegration, or online intrinsic/extrinsic
  calibration;
- no lateral/longitudinal slip state; disagreement only changes health and yaw
  weighting;
- no GNSS, visual, LiDAR, or landmark correction, so global drift is unavoidable;
- no covariance inflation model learned from real residuals yet.

The M2-B baseline includes a mobile TaskSpec whose actor receives this estimate,
uncertainty, timing, health, and goal while truth is restricted to named scoring
and termination terms. M3 must now run that contract against an integrated plant,
measure drift, innovation, fault response, latency, and truth error across surfaces
and both physics backends. M5 must identify calibration, noise, and disagreement
thresholds from held-out real logs.

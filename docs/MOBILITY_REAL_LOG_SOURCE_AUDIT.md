# Mobility real-log source audit

Status: source screening and first sensor archive inspection, 2026-09-07.
One NCLT sensor archive acquired externally; no physical validation completed.
Other candidate descriptions are not verified channel manifests.

## Selection and limits

| Primary source | Potential RNE use | Evidence gap / decision |
| --- | --- | --- |
| [Michigan NCLT](https://robots.engin.umich.edu/nclt/index.html) | Wheel/IMU replay and estimator timing checks | First bounded format-inspection candidate; not independent drivetrain or suspension validation. |
| [Driving Data of a Real F1tenth Car](https://zenodo.org/records/12536536) | Ackermann dynamic-model identification candidate | Search-indexed repository description explicitly targets identification. Record returned HTTP 429; file schema, size, license and reference independence remain unverified. No download selected yet. |
| [KAIST Complex Urban Dataset](https://sites.google.com/view/complex-urban-dataset/home) | Navigation sensor replay candidate | Official page lists LiDAR, stereo and position sensors, but does not establish the actuator channels needed here. It declares CC BY-NC-SA 4.0; do not bundle under RNE's license. Not selected for dynamics identification. |

NCLT's official update history says left/right wheel velocities were added to sensor
archives in August 2018. Its March 2019 update says the approximately 100 Hz
ground-truth poses use odometry interpolation between SLAM graph poses. Thus those
interpolated poses are not independent wheel-odometry reference measurements.
The page provides a script to inspect the original graph-node poses. Its 2013-01-10
entry lists a 21 MB sensor archive, separate from much larger images and LiDAR.
The page declares ODbL and Database Contents licensing. Verify the actual archive
and applicable terms before acquisition; listed size is not an enforced byte limit.

For NCLT, inspect original node timestamps and the reference construction first.
Even graph-node poses require an audit of shared estimator inputs before being
called independent truth. Do not differentiate an odometry-interpolated trajectory
and report the result as independently measured velocity or slip. Published wheel
velocity is not automatically raw encoder counts. A Segway capture also does not
establish skid-steer, trailing-caster, or Ackermann validity.

## Acquired NCLT evidence

Acquired directly from the official page's
[2013-01-10 sensor archive](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/sensor_data/2013-01-10_sen.tar.gz):

- external path: `E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10_sen.tar.gz`;
- exact length: 21,669,014 bytes; download enforced a 32 MiB streaming cap;
- SHA-256: `3ec1a5ac27ee716e6e5da0cd4e8fee96241d6a21ac63d14a844c40d2410d5ffb`;
- no filesystem extraction, images, or LiDAR acquisition; listing shows regular
  sensor CSV files, one README and their containing directory;
- checksum records the bytes received, not a publisher signature or attestation.

The archive README specifies `wheels.csv` as timestamp, left speed, right speed
(m/s), and `kvh.csv` as timestamp plus heading (rad). KVH heading must not be
silently treated as angular velocity. The README does not itself specify the wheel
timestamp unit, capture/receipt distinction, calibration uncertainty or raw encoder
counts. Resolve these before publishing a normalized sensor contract.

A complete in-memory scan of `wheels.csv` via `tar -xOf` found:

| Check | Observed value |
| --- | --- |
| Rows, each with three parseable fields | 42,276 |
| First / last integer timestamp | 1357847237325466 / 1357848263256859 |
| Minimum / maximum adjacent timestamp difference | 14 / 201267 (source timestamp units) |
| Non-increasing timestamps | 0 |
| Non-finite wheel speeds | 0 |
| Maximum absolute wheel speed | 2.334337 m/s |

These checks establish format and ordering only. They do not prove physical
accuracy, sensor jitter, latency, dropout behavior or synchronization. Retain the
irregular timestamps instead of assigning a constant sample period. The initial
three-row preview closed its pipe early and `tar` reported a broken pipe; the
subsequent complete scan checked `tar` exit status zero before computing this table.

Official scripts were read as text, not executed:

- [read_ground_truth.py](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/python/read_ground_truth.py)
  obtains graph-node times from the covariance file and samples the trajectory at
  those times. It labels position NED, not RNE's world frame.
- [read_odom.py](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/python/read_odom.py)
  reads timestamp, position and roll/pitch/heading; it is not a wheel-count reader.
- [read_ms25.py](https://s3.us-east-2.amazonaws.com/nclt.perl.engin.umich.edu/python/read_ms25.py)
  identifies timestamp, magnetic field, acceleration and rotation-rate columns,
  with plot units microseconds, Gauss, m/s squared and rad/s respectively.

Next inspect documented sensor frames, time semantics and reference construction;
do not infer these from similar-looking columns. A wheel/IMU replay importer remains
unimplemented, and no suspension or drivetrain identification result is claimed.

## Identification design

[Gonultas et al., IROS 2023](https://arxiv.org/abs/2308.03898v2) reports
gradient-based identification of a front-steered vehicle and real F1TENTH lane-keeping
validation. This supports including a control-level validation experiment after
fitting; it does not establish that the separate Zenodo record above has the same
vehicle, measurements, or parameters. Neither source is evidence that RNE currently
matches real dynamics.

Before implementing a dataset-specific importer, resolve:

- immutable source/version, exact filenames, lengths, checksums and license;
- measured versus commanded steering and drive quantities, units and sign;
- clock domains, capture versus logging time, synchronization uncertainty,
  missing intervals and resampling already performed;
- measured versus estimated pose/velocity and all inputs used to build references;
- vehicle dimensions, drive topology, mass and available calibration artifacts;
- excitation sufficient for the chosen parameters, including confounded or
  unobservable parameters that must remain fixed or explicitly unidentified.

Without voltage/current and calibrated drive information, trajectory agreement
alone cannot identify motor electrical parameters. Do not manufacture missing
force, current, steering measurements or timestamp uncertainty from model output.
Without independent reference measurements, report replay consistency rather than
physical accuracy.

The existing [suspension identification gate](MOBILITY_SUSPENSION_IDENTIFICATION_V1.md)
requires strut displacement, velocity and generalized force plus acquisition evidence.
None of these candidate descriptions establishes that contract. Keep its physical
dataset status pending; do not relax it to accept generic driving trajectories.

## Bounded execution plan

1. Inspect NCLT's official format-reading scripts as text, without running them.
   Resolve exact sensor archive URL and licensing; inspect file listing before extraction.
2. Acquire only selected small sensor/reference files under an explicitly validated
   external SSD root. Set a hard streaming byte cap, use create-new destinations,
   hash original bytes, and record final URL and acquisition metadata. Reject path
   traversal, links and extraction-size overruns. No camera/LiDAR download for this slice.
3. Produce a channel/timestamp audit before normalization. Implement the offline
   importer outside core crates, with malformed-input, unit, ordering and deterministic
   replay tests. Preserve original timestamps; leave unavailable latency unknown.
4. Revisit the F1TENTH record metadata when available. Select a dynamic-model experiment
   only after verifying input/output semantics and identifiability. Split by complete
   runs before fitting; reserve a separate final test, not adjacent time samples reused
   across tuning and evaluation. Freeze SI metrics and acceptance budgets in advance.
5. Feed validated parameters to both backends and report real-reference residuals
   separately from Rapier/MuJoCo agreement. Agreement between backends is not a
   substitute for agreement with independently measured behavior.

Raw captures, converted datasets, caches and reports stay on the external SSD and
are not committed. CI uses small synthetic format fixtures clearly labeled synthetic;
passing those tests never changes physical-validation status. This plan adds no ROS
dependencies to core crates and does not require rendering.

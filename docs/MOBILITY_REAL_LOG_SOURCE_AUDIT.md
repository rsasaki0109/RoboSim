# Mobility real-log source audit

Status: source screening and first sensor archive inspection, 2026-09-07.
One NCLT sensor archive acquired externally; no physical validation completed.
Other candidate descriptions are not verified channel manifests.

## Selection and limits

| Primary source | Potential RNE use | Evidence gap / decision |
| --- | --- | --- |
| [Michigan NCLT](https://robots.engin.umich.edu/nclt/index.html) | Wheel/IMU replay and estimator timing checks | First bounded format-inspection candidate; not independent drivetrain or suspension validation. |
| [Driving Data of a Real F1tenth Car](https://zenodo.org/records/12536536) | Velocity-command response identification candidate | Official API and README now inspected after earlier access errors. Command is body-frame; VICON velocity is world-frame. CC BY 4.0 declared. Bag contents remain uninspected. |
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

The paper's Sections 3, 4 and 7 and Tables 4/8 were inspected in the
[author-hosted manuscript](https://s3.us-east-2.amazonaws.com/publications.perl.engin.umich.edu/ncarlevaris-2015a.pdf).
UTIME denotes Unix microseconds. The body axes are forward/right/down, with its
origin at the axle center; the IMU has identity mounting rotation but a nonzero
lever arm. Microstrain values use its internal filter. Odometry fuses wheel,
FOG and IMU inputs: it is not an independent reference for their validation.
`odometry_mu.csv` represents relative image-event motion, whereas the 100 Hz file
is relative to the run start. Do not integrate or compare these interchangeably.

The 2018 wheel addition inherits the dataset timestamp convention; its README does
not independently document acquisition-clock semantics. No measured arrival latency
or synchronization uncertainty is inferred. A source-time replay must explicitly
distinguish its scheduling policy from physical sensor latency.

## Executable ingestion

`rne_mobility_benchmark::recorded_nclt` now reads wheel and `ms25` CSVs through
bounded `Read` inputs. Each requires nonempty input, exact column counts, 16-digit
integer timestamps, strictly increasing times and finite numeric values. Limits are
64 MiB per input, one million rows and 1,024 bytes per line. Invalid inputs fail as
a whole. The output binds parsed samples to SHA-256 of the exact uncompressed bytes,
including original line endings. Synthetic tests do not include redistributed data.

Values remain in source coordinates and units (including magnetic field in Gauss).
No resampling, gravity compensation, lever-arm correction, encoder-count synthesis,
or measurement-noise estimation occurs. There is no DataBus replay bridge yet;
this reader is not a replacement for that remaining work.

The CLI is read-only and reports timestamps, interval extrema and a source digest:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
cargo run -p rne_mobility_benchmark --bin rne-nclt-audit -- wheels E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10\wheels.csv
cargo run -p rne_mobility_benchmark --bin rne-nclt-audit -- imu E:\RoboSim-external-data\mobility-nclt-2013-01-10\2013-01-10\ms25.csv
```

Only these two regular CSV members were subsequently extracted from the hash-checked
archive into a previously absent external directory: 1,483,062 and 8,951,816 bytes.
The initial no-extraction audit above describes the earlier inspection stage.
The default benchmark binary remains `rne-mobility-benchmark`; adding this audit
tool does not change existing `cargo run -p rne_mobility_benchmark` behavior.

Both real files passed the CLI. Wheel counts/times match the initial scan. IMU
has 48,324 rows spanning 1357847237276758 through 1357848263255151 microseconds,
with adjacent intervals from 3,134 to 72,076 microseconds. Both file digests match
an independent PowerShell `Get-FileHash -Algorithm SHA256` computation:

- wheels: `5387898ac211c17502c23e0a231ced7b479c3123717317b3527ff9bd66d0a029`;
- IMU: `9e504bbbc410c53a25c2da670fd2f03d3228154f1016026bd09e48123380498f`.

Passing the ingestion checks is not a sensor-quality or dynamics acceptance gate.

Validation of this ingestion slice: formatting and crate Clippy passed, the four
reader tests and one audit-summary test passed, and MuJoCo-enabled tests passed
(101 library tests plus one test in each CLI). The full `xtask ci` completed with
exit code zero, including headless, OSS parity, 361 fuzz cases and 10/10 Behavior
seeds. Logs are external: `E:\RNE-build\m3c-sensor\nclt-mujoco-tests.log` and
`E:\RNE-build\m3c-sensor\nclt-ci.log`. Two repeated audit invocations for each real
CSV produced identical output, with hashes independently checked by PowerShell.

Next connect qualified source-frame samples to explicit replay scheduling and
estimator inputs, with frame/time tests. No physical identification result is claimed.

### Replay boundary review (not yet implemented)

Inspection of `rne_ai::WheelImuOdometry::update` shows that it requires
`IncrementalEncoderFeedback` and integrates differences of `raw_count`, not a wheel
velocity field. Its `WheelImuOdometryConfig` also requires counter resolution and
width. NCLT cannot satisfy this input contract without invented measurements.
The older `WheelEncoderSample` also requires a realized angular position, which the
recorded wheel-speed file does not provide. Neither payload is an honest shortcut.

Similarly, `ImuFeedback` declares raw specific force, sample-phase error and known
saturation status. Internally filtered `ms25` values and unavailable status metadata
must not be relabeled nominal raw feedback. Keep the existing incremental-encoder
estimator unchanged until an explicit measured-velocity estimation path is designed.

The next replay slice should therefore:

- publish source-typed wheel-speed and filtered-IMU payloads through the existing
  extensible `FramePayload`/DataBus boundary, without adding NCLT dependencies to core;
- retain source Unix timestamps in payloads and use one shared integer origin for
  both streams, converting elapsed microseconds to nanosecond ticks with checked
  subtraction and multiplication; separate stream origins would erase real skew;
- declare replay availability delay as a chosen transport experiment, never measured
  latency; preserve original source values, frames, filtering and unknown quality;
- validate both streams before publishing, use deterministic tie ordering, and test
  missing inputs, distinct start times, ties, delayed availability and reset/replay;
- drive consumers through `latest_available`, never `latest`, and preserve source
  digests plus replay policy in evidence. Reconstruct from original bytes when
  provenance matters: public parsed structs can be modified after ingestion;
- add a separately identified wheel-speed/gyro integration path with explicit
  interpolation, gap, frame and uncertainty policies before claiming estimator replay.

This is an integration requirement, not an additional completed benchmark gate.

## Identification design

The F1TENTH record's [official API](https://zenodo.org/api/records/12536536)
lists nine bags, from 84,985,680 to 192,694,601 bytes, and declares CC BY 4.0.
The [README](https://zenodo.org/api/records/12536536/files/ReadMe.md/content)
describes separate continuous runs, body-frame `/cmd_vel` commands and world-frame
`/vrpn_client_node/Car_2_Tracking/twist` VICON measurements. Its opening paragraph
instead spells the command topic `/vel_cmd`; verify actual bag connections rather
than choosing from prose. No bag was downloaded during this metadata inspection.

This supports investigating aggregate command-to-motion response. It does not yet
establish measured steering, motor voltage/current, wheel forces or clock uncertainty.
For longitudinal signed speed, a world-frame velocity norm loses reversal and
lateral-motion information. Require orientation/frame evidence for projection or
declare a narrower speed-magnitude metric, without calling it signed velocity.
Inspect measurement derivation and timestamp alignment before treating VICON output
as a qualified reference. Split by entire runs before identification.

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

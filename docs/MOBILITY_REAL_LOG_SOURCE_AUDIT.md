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
or measurement-noise estimation occurs. The additive
[source-time DataBus replay](MOBILITY_RECORDED_REPLAY_V1.md) preserves that boundary;
measured-velocity state estimation remains separate follow-up work.

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

### Replay and estimator boundary review

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

The replay contract and remaining estimator requirements are:

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
than choosing from prose. That initial metadata inspection did not download a bag;
the subsequent bounded acquisition below supersedes the metadata-only status.

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

### F1TENTH bounded acquisition and preliminary byte audit

One run was acquired from the official API content link, with a 90,000,000-byte
download cap, into the previously absent external directory
`E:\RoboSim-external-data\mobility-f1tenth-12536536`. No ROS runtime, bagpy, image
extraction or additional package installation was needed. The bag itself includes
scan messages; they were not extracted into a separate dataset.

- File: `ex-hard-r2_2023-06-12-19-59-52.bag`, exactly 84,985,680 bytes.
- Official MD5, verified: `1f0e930d8d0bf2106fac9d34a94b9c21`.
- Locally computed SHA-256:
  `3ba7b5c13da68227bf8af27370e7205f142dca4ca3340c8b1b51feba70cc22ac`.
- Attribution: Giannis Badakis, Michalis Galanis and Zengjie Zhang,
  *Driving Data of a Real F1tenth Car*, Zenodo record 12536536, CC BY 4.0.

A read-only, bounded standard-library byte audit, following the record layout in
the [official ROS implementation](https://github.com/ros/ros_comm/blob/noetic-devel/tools/rosbag/src/rosbag/bag.py),
found ROS bag v2, 105 uncompressed chunks and 25 connections. Message definitions
were treated as text, not executed. Selected message counts from walking chunks
agree with the connection-index totals:

| Topic | Type | Messages |
| --- | --- | ---: |
| `/cmd_vel` | `geometry_msgs/Twist` | 9,853 |
| `/commands/motor/speed` | `std_msgs/Float64` | 9,855 |
| `/sensors/core` | `vesc_msgs/VescStateStamped` | 6,032 |
| `/vrpn_client_node/Car_2_Tracking/pose` | `geometry_msgs/PoseStamped` | 13,426 |
| `/vrpn_client_node/Car_2_Tracking/twist` | `geometry_msgs/TwistStamped` | 13,402 |

The VESC embedded definition includes input voltage, motor/input current, electrical
RPM and duty cycle. Thus telemetry is present, contrary to what the metadata alone
could establish. Presence is **not qualification**: preliminary decoding found input
voltage and PCB temperature identically zero, duty cycle from -0.194 to 0.569 despite
the embedded comment's 0-to-1 range, and fault values spanning 0 through 255 despite
only codes 0 through 6 being declared. Treat these as unresolved decoding/driver/
record-quality anomalies, not physical fault diagnoses or usable motor calibration.

VICON pose/twist report `world`; VESC's frame string is empty. For VICON pose,
`bag_time - header_stamp` ranges from -63.775767420 to -62.780972095 s; for VICON
twist, -63.775357363 to -62.906228216 s; for VESC, -63.775798446 to -63.445731478 s.
These are differences between recorded clock fields, not negative physical latency.
The unstamped command messages have no capture timestamp. Do not erase these
differences by independently zeroing each stream or fitting an unexplained offset.

Next, validate decoding through an independent reader and synthetic format fixtures,
inspect the acquisition/driver time conventions, and determine whether a justified
common-clock mapping exists. Verify VICON velocity derivation and orientation before
body projection. Electrical RPM needs pole-pair/transmission/sign calibration; servo
commands are not measured steering. Do not fit voltage-driven electrical parameters
from the zero-voltage channel. No parameter fit, held-out score or physical pass is
claimed from this preliminary inspection.

Independent reader check: the already available `rosbags` 0.11.3 ROS1 reader and
its generated typed deserializer reproduced all five selected message counts and
all three header-versus-bag timestamp ranges exactly. It independently confirmed
6,032/6,032 zero input-voltage values and 5,861/6,032 fault values outside codes
0 through 6 (observed range 0 through 255). This rules out the initial manual byte
parser as the sole explanation, not driver/firmware or recording defects.
The missing `lz4` 4.4.5 dependency (99 kB wheel) was installed without pip caching
only under the external acquisition directory's `reader-deps`; no existing Python
environment was modified. Calls used `-B` to avoid bytecode cache writes. Reader
agreement does not qualify the channel or resolve clock synchronization.

Reference angular velocity also needs qualification. The
[published Noetic driver source](https://docs.ros.org/en/noetic/api/vrpn_client_ros/html/vrpn__client__ros_8cpp_source.html)
converts `vel_quat` to roll/pitch/yaw and assigns these directly to twist angular
fields without dividing by `vel_quat_dt`. Its header stamp can use either server
time or ROS current time. The bag's exact deployed driver version and configuration
are not established, so this is a concrete investigation lead, not proof that the
same defect produced this capture. Do not treat the angular fields as calibrated
rad/s, silently apply a guessed sample-rate multiplier, or infer synchronization
from the topic/type names. Independently checked pose differences and documented
server/driver conventions are required before selecting a yaw-rate reference.

The independent audit is reproducible with `scripts/audit_f1tenth_source.py`.
It verifies the pinned size and SHA-256 before parsing; it rejects different
captures instead of silently assuming their schemas or calibration. It checks
decoded counts against the source connection index and prints a stable JSON digest.
Only the five listed channels are deserialized; no dataset is exported or fitted.
Use the external dependency directory and disable bytecode writes:

```powershell
$env:TEMP = 'E:\RNE-build\tmp'
$env:TMP = $env:TEMP
$env:PYTHONPATH = 'E:\RoboSim-external-data\mobility-f1tenth-12536536\reader-deps'
python -B scripts/audit_f1tenth_source.py E:\RoboSim-external-data\mobility-f1tenth-12536536\ex-hard-r2_2023-06-12-19-59-52.bag
python -B -m unittest discover -s scripts -p test_audit_f1tenth_source.py -v
```

The chosen Python must provide `rosbags==0.11.3` and its dependencies. Synthetic
audit tests do not import rosbags and do not require physical data. The captured
report is `E:\RNE-build\m3c-sensor\f1tenth-independent-audit.json`, with digest
`f0427731ef2388e97dc29fa1906385fa18db57786a4409c9e97673193d418710`.

### Pose/twist consistency diagnostic

`scripts/audit_f1tenth_reference.py` uses the same pinned source and independent
reader, preserving header timestamps and the declared `world` frame. It compares
successive pose differences with the latest twist at or before the interval end.
The policy was fixed before evaluating: pose intervals 1..50 ms and twist age at
most 20 ms. These are retrospective interval averages versus endpoint samples,
not identical measurements or online observations. No command-time mapping is used.

There are 13,225 paired intervals, 148 excluded pose intervals and 52 missing/stale
twist pairs. World-planar velocity-difference RMS is 0.850751351 m/s. Pose-heading
rate RMS is 1.466426217 rad/s, while recorded angular-z RMS is 0.012536505 in its
unqualified units. The unscaled angular difference RMS is 1.455188522. A diagnostic
through-origin scale is 104.952102266; it is neither applied nor accepted as a
calibration. Timing, finite-difference noise, interval-versus-point comparison and
driver conventions may contribute. These results do not establish independent
accuracy and do not identify the capture's exact driver version or fault cause.

Run with the same external environment as above:

```powershell
python -B scripts/audit_f1tenth_reference.py E:\RoboSim-external-data\mobility-f1tenth-12536536\ex-hard-r2_2023-06-12-19-59-52.bag
python -B -m unittest discover -s scripts -p 'test_audit_f1tenth*.py' -v
```

The initial eight synthetic audit tests passed. Schema-v1 evidence is
`E:\RNE-build\m3c-sensor\f1tenth-reference-audit.json`, digest
`ef74cece68e453d8d24ede31e27a350bfaf738b5caeb6f36a25d365b93eecacb`.
Next, distinguish sample/clock jitter from velocity derivation effects with
interval-integrated comparisons; do not select a larger smoothing window merely
to make residuals pass. Independent reference calibration and command-clock
qualification remain prerequisites for physical identification.

The schema-v2 diagnostic additionally integrates zero-order-held world-frame
twist over each complete pose interval, splitting at source events and the same
20 ms expiry. Any positive-duration uncovered part rejects the entire pair; an
observation at the endpoint cannot fill an earlier gap. The original endpoint
diagnostic remains in the report without changing its policy or metrics.

For 13,194 fully observed intervals (148 interval exclusions, 83 coverage exclusions),
planar displacement-difference RMS is 0.003055050 m. This has different units and
a different accepted population from the 0.850751351 m/s endpoint comparison;
do not describe the two numbers as an accuracy improvement. Pose-heading increment
RMS is 0.012544661 rad, whereas integrated recorded angular-z RMS is 0.000127213
in unqualified integrated units. The exploratory through-origin angular scale is
95.680780098 and remains unapplied. The angular discrepancy persists with the
interval-integrated comparison; its cause and a valid correction remain unproven.

All 11 synthetic audit tests passed. Schema-v2 evidence is
`E:\RNE-build\m3c-sensor\f1tenth-reference-integral-audit.json`, digest
`c9050e373ff763e0e9ea390adbf5ca185d5383f6b9a5925e1f6beb72b32fa49d`.
Neither diagnostic performs clock synchronization, independent reference
qualification, electrical identification or an acceptance-threshold fit.

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

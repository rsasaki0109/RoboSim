# Mobility real-log source audit

Status: source screening and bounded NCLT, F1TENTH and DDMR inspection, 2026-09-08.
Data resides on external storage; no physical calibration qualification completed.
Uninspected candidate descriptions are not verified channel manifests.

## Selection and limits

### Additional electrical-identification candidate (2026-09-09)

The authors' [OpenMCT DC motor dataset, version 1](https://data.mendeley.com/datasets/5xvg43r9r8/1)
(DOI `10.17632/5xvg43r9r8.1`, published 2026-05-11) declares CC BY 4.0,
13 raw logs and over 52,000 rows. Its description lists PWM, speed, loop interval,
raw/filtered current sensing and digital-multimeter current when enabled, together
with calibration, APRBS, PI/discrete-controller and chirp experiments. This is a
bench motor candidate, not a differential/skid or Ackermann vehicle validation.
The linked SSRN article returned HTTP 403 during screening; its contents were not
reviewed. No raw file, calibration table or processing script has been downloaded
or executed, and file sizes/hashes remain unknown.

The public site's client bundle subsequently established the anonymous listing
route `/public-api/datasets/5xvg43r9r8/files?folder_id=root&version=1`
(Accept `application/vnd.mendeley-public-dataset.1+json`) and folder route
`/public-api/datasets/5xvg43r9r8/folders/1`. These returned metadata successfully;
the guessed `/versions/1` route did not exist. Root documentation is individually
available: `DATA_DESCRIPTION.md` 15,112 bytes, `DATASET_METADATA.md` 2,535 bytes,
`LICENSE` 385 bytes and `README.md` 2,841 bytes. The listing declares SHA-256
`f015801923f2fd17b833159f1de2f64b60450913d5ad1c774a5a3118e86f5479`
for `DATA_DESCRIPTION.md` (file ID `85b0dbe5-c0ec-4a7b-bcc9-82aa44688b22`).
This is server-declared metadata, not yet a locally verified content hash.
Folder metadata separates current calibration, static characterization,
system identification, continuous/discrete validation and optional characterization.
Read these small documentation files before selecting any experiment log; the
root documentation sizes do not bound the experiment folders or complete dataset.

Documentation-only inspection subsequently read the data description, hardware
metadata, calibration README and calibration MATLAB source (without executing it).
The declared hardware is Teensy 4.0, DRV8874 IPROPI, TSINY ts-25GA370H-20 and
Siglent SDM3045X. Speed is RPM, `DT_ms` is a loop interval, and PWM is a command;
`CURRENT_RAW` is ADC counts while `CURRENT_AVG` already uses firmware filtering
and an earlier calibration. Missing DMM fields use `nan` and sample ID `-1`.
The DMM is asynchronous and reused IDs do not represent independent measurements.

The inspected [calibration script](https://data.mendeley.com/public-files/datasets/5xvg43r9r8/files/2ba66e22-88b5-4fd7-88ea-59433626db22/file_downloaded)
constructs time by cumulative loop intervals, takes absolute DMM current, selects
age <= 10 ms and one minimum-age row per DMM ID, then removes residuals beyond
4 * 1.4826 * MAD after an initial linear fit. Its final RMSE is computed on the
retained fitting rows, not independent holdout. Negative calibrated values are
clipped for displayed output. An RNE audit must retain rejected-row counts,
unclipped residuals and DMM identity; this processing is not signed current or
clock-calibration evidence. The fit mask also lacks an explicit nonnegative-age
check, so raw age validity needs independent inspection.

The calibration folder listing declares `raw_data.txt` 258,643 bytes, file ID
`187827aa-3750-4d23-b513-ddf00d64480c`. Next acquire this individual log with a
300,000-byte hard bound and verify its server-declared SHA-256, then audit all
rows before fitting.

Bounded acquisition completed: the exact 258,643 bytes matched server SHA-256
`9dfa5b4ffaea999ef3792537b3a17611feb6ee3da0add47c63857cb11f0179a3` and were saved
without overwrite at
`E:\RoboSim-external-data\mobility-openmct-5xvg43r9r8-v1\current-calibration-raw.txt`.
A preliminary PowerShell CSV inspection (not yet a strict production reader)
found 4,558 rows, two missing DMM rows, 3,947 distinct valid DMM IDs and 609 reused
rows. Reused IDs retained identical DMM time/current in this inspection. There
were no negative ages; valid ages ranged 0.8–57.6 ms. Nonnegative ADC and age
0–10 ms selected 1,952 rows with 1,952 distinct DMM IDs before residual rejection.
All loop-interval fields were exactly 20 ms; cumulative interval sum is 91.16 s,
not an independently measured acquisition duration. ADC counts ranged 0–296.
These observations do not yet reproduce the fit, certify timing or provide an
independent validation capture. The next reader must validate exact headers,
column count, finite required channels, missing-DMM tuples, integer IDs and
source ordering rather than relying on permissive `ConvertFrom-Csv` parsing.

`recorded_openmct::read_openmct` now implements an offline strict reader with
8 MiB/100,000-row/1,024-byte-line bounds and exact-byte hashing. It retains signed
DMM current, original RPM, PWM commands, raw ADC and filtered source current
separately. The documented missing tuple becomes `None`, never zero. Reused DMM
IDs must retain bit-identical current/time; ID or DMM-time regression is rejected.
No age selection, absolute-value conversion, calibration, filtering or resampling
occurs in ingestion. Metadata dates are retained as declarations, not converted
to a simulation clock. Focused synthetic tests cover preservation and rejection;
validation is in progress. The read-only `openmct_audit` example has now read the
physical file successfully through this strict reader: 4,558 rows, two missing
DMM rows, 3,947 distinct IDs, 609 reused rows, age range 0.8–57.6 ms and the exact
SHA-256 above matched the preliminary audit. The first two synthetic tests and
all-target Clippy passed. After adding byte/row/line-bound and read-error tests,
all three focused tests passed (0.33 s), together with all-target Clippy. Full
workspace validation subsequently passed as recorded below. This is successful
ingestion, not calibration acceptance.

```text
cargo run -p rne_mobility_benchmark --example openmct_audit -- <external raw-log.txt>
```

The frozen reader implementation (Git blob
`4f204f9b36401ddbd13565bdc673abe7c5397a90`) completed `cargo run -p xtask -- ci`
with exit 0. Mobility library: 167 passed, zero failures, one ignored long training
job (182.73 s). Workspace lint/tests, headless, OSS parity, 361 fuzz cases and
Behavior CI 10/10 seeds passed. External log:
`E:\RNE-build\m3c-sensor\openmct-ingestion-v1-ci.log`, SHA-256
`b5e8bf2b92538f82526819353828dd744aed4fc76bb754c6f195571a99619feb`.
This default-feature run does not establish MuJoCo feature coverage. Negative
results remain: heading CEM score -10 equals baseline; clutter PPO -1.37 is below
random 2.01; mobile CEM does not place; flagship evidence is `cross_backend=false`.

A separate read-only PowerShell recomputation on the acquired raw log reproduced
the author's selection and linear/MAD-refit arithmetic: 1,952 candidates, 1,654
retained and 298 rejected; slope 1.1860067240413787 mA/count, intercept
-1.4498528146829202 mA. Unclipped retained-fit RMSE was 5.037074098976796 mA,
whereas all-candidate RMSE under that same refit was 12.773130702197356 mA.
Initial OLS all-candidate RMSE was 12.413588291154817 mA. This independent
implementation check used permissive CSV parsing after strict ingestion succeeded;
it is not yet a reusable RNE calibration API or an independent validation capture.
Do not interpret rejected samples as known sensor faults or fit residuals as a
calibrated noise distribution. A future tested reproduction must retain every
candidate and its selection reason, and report excluded and retained errors.

Next acquisition gate: obtain an explicit file listing and bounded individual raw
logs on external storage; inspect units, measured versus commanded voltage, motor
and load identity, clock construction, current-reference synchronization and
calibration residuals before choosing identifiable parameters. Freeze separate
excitation/validation runs before fitting. A PWM channel alone is not measured
terminal voltage; do not infer physical electrical constants from it without the
missing conversion and instrumentation evidence.

The [AutoDRIVE Nigel author repository](https://github.com/Tinker-Twins/AutoDRIVE-Nigel-Dataset)
is a separate Ackermann candidate with timestamp, steering, tick-count and inertial
columns. Its README declares approximately 1.50 GB for the camera-free dataset
and 66 GB for the full dataset. Neither was downloaded. The linked Zenodo record
returned HTTP 429. Physical versus simulator capture provenance and calibration
are not established by the inspected README, so it is not accepted as real-log
qualification. Do not clone the full repository merely to inspect its schema.

| Primary source | Potential RNE use | Evidence gap / decision |
| --- | --- | --- |
| [Michigan NCLT](https://robots.engin.umich.edu/nclt/index.html) | Wheel/IMU replay and estimator timing checks | First bounded format-inspection candidate; not independent drivetrain or suspension validation. |
| [Driving Data of a Real F1tenth Car](https://zenodo.org/records/12536536) | Velocity-command response identification candidate | One bag inspected with independent readers; electrical channels and reference timing/units remain unqualified. See the acquired evidence below. CC BY 4.0 declared. |
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

## Acquired DDMR format evidence

The author repository's [pinned CSV](https://github.com/RAI-Techno/ddmr_control/blob/6291b0d7faa5c7b7deb475e833c115c84d7123da/Data%20and%20Codes/Data.csv)
was acquired with a 20,000,000-byte streaming bound on 2026-09-08.
The exact 19,461,206 bytes matched Git blob SHA-1
`427680e90bfde9d41b09068c78dafc807273e907` (including the Git blob header).
Local SHA-256 is `c278dde8bfc38974bb2b1cc054160349da51456817c898ea2e052656aa75f60e`.
Original file: `E:\RoboSim-external-data\mobility-ddmr-6291b0d7\Data.csv`.
No notebooks or model weights were executed.

A complete read-only CSV scan found 338,550 rows, each with five finite numeric
fields: time (s), left/right voltage (V), left/right speed (rad/s). No malformed
rows or non-increasing times occurred. Times span 0 to 3385.4900000000002 s.
All adjacent differences match 0.01 s within 1 ns (decimal extrema
0.0099999999997 and 0.0100000000003 s). Both voltage columns range from
-5.808 to 10.664 V. Left speed ranges from -16.379961696524322 to
33.17992241090824 rad/s; right from -17.639958750103116 to
32.759923393048645 rad/s. Values and timestamps were not resampled or repaired.

These are file-format checks only. A regular time column does not establish
hardware capture timing. Commanded versus measured terminal voltage, encoder
processing, clock construction, capture instrumentation and dataset-specific
license scope still need qualification. There are no current or independent
body-reference columns in this CSV. Do not use these checks to claim electrical,
slip, suspension or real-world model accuracy. Keep raw data external and do not
bundle it into the repository.

### DDMR capture and evaluation qualification

Read-only source inspection on 2026-09-08 reached the following decision:
**retain as an unqualified command-to-wheel-response candidate, not an electrical
parameter or vehicle-slip calibration dataset.**

The pinned [identifier notebook](https://github.com/RAI-Techno/ddmr_control/blob/6291b0d7faa5c7b7deb475e833c115c84d7123da/Data%20and%20Codes/System_Identification_Model.ipynb),
code cell 4, consumes the first 336,000 rows, ignores the time column, constructs
150-row two-voltage windows, and targets the two speeds at each window's last row.
It prepends an artificial zero-input/zero-target example. It then splits the
constructed windows chronologically (60% training, then 67% of the remainder for
validation), without a boundary purge. Adjacent windows across a partition share
149 input rows. This is shared input history, not proof that target values leaked.
The reported R-squared is not independent-capture or free-running physical-model
validation. The notebook neither records hardware data nor establishes voltage
measurement or encoder decoding. No notebook code or weights were executed.

The complete GitHub file trees were inspected for `ddmr_control` at
`6291b0d7faa5c7b7deb475e833c115c84d7123da`, the linked
[exploration project](https://github.com/RAI-Techno/drl_autonomous_exploration/tree/2624fe1a9d04770552bc6acea9b6e963bdc00a4e),
and its linked [LilyBot project](https://github.com/RAI-Techno/lilybot/tree/eb059ec6564267c7e245cdde75a1617f18d1f7b7).
The trees were not truncated. No DDMR capture firmware or CSV recording script
was identified there. LilyBot's `lily_go.launch` starts Gazebo with simulated
time, not an instrumented real motor acquisition path. Its parts list links a
Yahboom ROS expansion board and DFRobot FIT0493 motor. This narrows hardware
candidates but does not pin the components/firmware used for the CSV capture.
The DDMR root LICENSE identifies Apache License 2.0; no separate dataset license
file appeared in the inspected tree. Keep the data external pending redistribution
review rather than relabeling it as RNE-owned data.

The [motor vendor](https://www.dfrobot.com/product-1462.html) lists 34:1 gearing
and quadrature feedback with 374 pulses per output revolution. Whether acquisition
counts one or multiple edges, and whether speed uses a fixed interval, remain
unknown. Do not substitute a guessed 374 or 1496 counts/revolution into the recorded
sensor contract. The [board documentation](https://www.yahboom.net/study/ROS-Driver-Board)
lists encoder capture and PWM control tutorials, but does not bind a firmware
version, configuration or voltage conversion to this dataset.

The `recorded_ddmr` module now provides a bounded, source-hashed offline reader
preserving the five recorded columns and unknown capture/receipt semantics.
It retains the original time token alongside f64 seconds, accepts the exact
unquoted numeric header with optional UTF-8 BOM and LF/CRLF, and rejects malformed,
nonfinite, unordered or oversized input. Limits are 32 MiB, 500,000 records and
512 bytes per data line. No automatic time conversion or voltage calibration occurs.

`DdmrSeries::split` takes explicit raw-row cut indices before window construction.
Each read-only partition builds its own past-only prediction windows. Positive
history and future-horizon lengths are mandatory; a horizon of one targets the
next recorded row, not necessarily a fixed number of seconds. Short partitions
fail instead of yielding an empty evaluation. No warm-up history crosses a split.
The `ddmr_source_check` example reports source integrity and window counts only:

```text
cargo run -p rne_mobility_benchmark --example ddmr_source_check -- <Data.csv> 203130 270840 150 1
```

For the pinned 338,550-row source these explicit cuts are 60%/20%/20%. They are an
ingestion smoke configuration, not a tuned model or a frozen performance result.
Synthetic tests cover source spelling/negative zero, line-ending hash differences,
disjoint histories and future labels, split/window boundary errors, byte/row/line
bounds, invalid numeric input and I/O errors.

On 2026-09-08 the three focused reader/split tests and crate all-target Clippy
(`-D warnings`, default features) passed. The example read all 338,550 real rows,
reported the exact byte count and SHA-256 recorded above, and generated 202,980
training windows plus 67,560 each for validation and test. It fitted no model and
reported all physical-calibration, timing and voltage qualification flags false.
This implementation postdates the full CI for commit `3dbc897`; that earlier
full CI is not regression evidence for this reader.
The subsequent MuJoCo-enabled crate library regression completed with 133 passed,
0 failed and 2 intentionally ignored long-job tests. This run covered the reader
and split, before adding the response identifier.

The exploratory response identifier and its fixed protocol are described in
[`MOBILITY_DDMR_RESPONSE_EXPERIMENT.md`](MOBILITY_DDMR_RESPONSE_EXPERIMENT.md).
Any response fit must use raw-row-disjoint chronological training/validation/test
segments, build windows only inside each segment, and disclose that one capture is
not independent-session validation. Freeze model choice on training/validation
only and compare held-out SI-unit errors with a persistence baseline. Do not fit
motor resistance, torque constant, current dynamics or tire slip from these five
columns. Physical parameter acceptance still needs a qualified acquisition source.

## Acquired NCLT archive

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

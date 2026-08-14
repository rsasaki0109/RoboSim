# Dataset bundle v1

RNE dataset bundles preserve a simulation run as bounded, renderer-neutral
evidence. The producer writes one record at a time; verification and offline
metrics stream records back without loading the run or starting a renderer.

This is the first v0.5 slice. It freezes the bundle, timing, calibration,
noise, gap, payload, and depth-evaluation contracts. RGB8, depth-f32, and
LiDAR use the existing lossless `rne_data::transport` codecs. IMU, planar
transform, ActionSpec-ordered f64 action, task outcome, and numeric
ground-truth annotation use dataset-specific v1 codecs. A headless integration
fixture records and decodes all of them in one content-addressed shard.

## Bundle layout

```
capture.rne-dataset/
  manifest.json
  records.rnedata
```

`manifest.json` schema v1 declares:

- TaskSpec digest, fixed simulation step, and world seed;
- streams sorted by numeric `StreamId`;
- payload schema, source entity, coordinate frame, dtype, and unit fields;
- calibration, latency, deterministic noise model, and noise seed;
- content-addressed assets and typed domain-randomization decisions;
- exact shard length, SHA-256, aggregate counts, and per-stream sequence
  boundaries.

The additional run payload encodings are:

| Stream | Encoding | Stable contents |
|---|---|---|
| IMU | `rne.dataset.imu.v1` | angular velocity and linear acceleration, f64 XYZ |
| transform | `rne.dataset.pose2d.v1` | position XYZ in metres and yaw in radians |
| action | `rne.dataset.action_f64.v1` | finite flat f64 values in ActionSpec order |
| task outcome | `rne.dataset.task_outcome.v1` | episode/step, reward totals, terminated/truncated/success flags |
| annotation | `rne.dataset.ground_truth_f64.v1` | class id, instance id, and manifest-ordered finite f64 values |

Each embeds the same stream, sequence, capture, and availability metadata as
the outer record. A mismatch is rejected during writing or streaming read.

The manifest has a self-excluding `content_sha256`: compact JSON is hashed with
that field empty. This detects accidental or partial edits. It is not a
signature; signed provenance is a later ecosystem milestone.

`records.rnedata` starts with `RNEDATA1` and a fixed schema-v1 header. Every
record has a fixed 80-byte little-endian header containing record kind,
`StreamId`, sequence, capture ticks, availability ticks, payload length, and
payload SHA-256. The manifest also hashes the complete shard, including all
record headers. A payload is limited to the same 32 MiB ceiling as the
production frontend transport.

Writers publish `records.rnedata.partial` and `manifest.json.partial` first,
then rename them only after flushing and validating the complete bundle. The
target directory must not exist, so evidence is never overwritten implicitly.

## Time, latency, and missing data

All time values are simulation nanosecond ticks. Dataset code does not read a
wall clock.

- `capture_ticks` says when the measurement was sampled.
- `available_ticks` says when a controller could consume it.
- fixed-latency streams require the exact declared delta on every record;
  per-frame latency is allowed only up to its declared bound.
- capture time cannot move backwards within a stream.
- sequences begin at zero and must be contiguous.

A dropped frame is not the same as absent data. A `Gap` record declares the
first missing sequence and a non-zero dropped count. Skipping a sequence
without a gap fails both writing and reading. Shard summaries separately count
physical records, materialized samples, and logical drops.

## Calibration and noise

RGB, depth, LiDAR, and IMU streams require both contracts, including noiseless
sources:

- calibration has a versioned model, reference frame, and finite parameters;
- noise has a versioned model, explicit seed, and finite parameters.

Parameter keys carry units, for example `fx_px` or `bias_stddev_rad_s`. Public
payload field units are independently listed in stream order. Unknown manifest
fields and non-finite numeric values fail closed.

## Headless verification and evaluation

Verify a complete bundle:

```powershell
cargo run -p xtask -- dataset-check artifacts/capture.rne-dataset
```

Compare a predicted depth stream with a ground-truth depth stream and write a
content-addressed report:

```powershell
cargo run -p xtask -- dataset-evaluate-depth `
  artifacts/capture.rne-dataset 10 11 0.02 artifacts/depth-eval.json
```

The evaluator scans the shard twice and retains at most one selected frame per
stream. Sequence, capture time, dimensions, explicit gaps, and non-negative
depth are checked before deterministic MAE, RMSE, and maximum error are
computed. `verify_depth_pair_report` recomputes the metrics from the bundle;
changing a report and merely replacing its self-hash is rejected.

Canonical manifest and evaluation shapes live in
`tests/golden/datasets/`. Integration tests also freeze the complete
IMU/transform/action/outcome/annotation shard digest, corrupt a record digest,
inject an unknown manifest field, omit a sequence, reject non-finite actions,
and forge internally consistent report metrics to prove fail-closed behavior.

## Reference robot capture

Example 73 runs a real renderer-free diff-drive episode through the kinematic
drive system, Rapier world queries, IMU/LiDAR samplers, and the scene-aware
headless RGB-D camera pipeline. It records the committed TaskSpec, action,
task outcome, base transform, IMU, LiDAR, RGB, sensor depth, and ideal depth
streams into one bundle:

```powershell
cargo run -p diff_drive_dataset_capture `
  --example 73_diff_drive_dataset_capture -- `
  artifacts/datasets/diff-drive-reference.rne-dataset --verify-golden
cargo run -p xtask -- dataset-check `
  artifacts/datasets/diff-drive-reference.rne-dataset
cargo run -p xtask -- dataset-evaluate-depth `
  artifacts/datasets/diff-drive-reference.rne-dataset `
  401 402 0.01 artifacts/datasets/diff-drive-depth-evaluation.json
```

DataBus sensor counters begin at one, while bundle sequences are defined as
dataset-local and zero-based. The capture adapter therefore renumbers records
while retaining physical sensor values, capture time, availability time,
stream ID, and noise seed. LiDAR positions and intensities are stored at the
manifest-declared 1 micrometre and 1 ppm resolutions before lossless encoding;
this removes irrelevant platform-math ULP differences without hiding a
sensor-scale difference. Camera depth is likewise stored at a declared
0.1-millimetre resolution. The reference sensor applies a declared 5 mm fixed
depth bias and 3 ms output latency; its paired ideal stream shares sequence,
capture time, dimensions, scene, and calibration. The bundle-local
`depth-evaluation.json` is written only after its metrics have been recomputed
from all 17 pairs (52,224 pixels), and the 10 mm gate must pass. Reference
asset line endings are fixed to LF. The v2 summary freezes manifest, complete
shard, evaluation-report SHA-256 values, all eight stream counts, zero dropped
records, and the successful terminal outcome. The v1 summary remains as a
compatibility fixture. The selected one-metre goal is retained as a typed,
seed-bearing run decision instead of being inferred from the trajectory.

`xtask ci-headless` regenerates this capture in a fresh temporary target and
checks the golden. The Windows/Linux evidence matrix performs the same capture
from a clean checkout, verifies the complete bundle, independently regenerates
the depth report, and uploads both for inspection. A digest mismatch fails the
job rather than being normalized away.

## Compatibility

Dataset bundle schema v1, dataset-native payload schema v1, and
offline-evaluation schema v1 are registered in `release/contracts.toml`.
Ordered stream/field arrays, enum values, binary
headers, units, and digest construction are semantic. Readers reject an
unknown schema instead of guessing. A future lossless migration must retain
the original bundle and cannot fabricate samples or turn a gap into a frame.

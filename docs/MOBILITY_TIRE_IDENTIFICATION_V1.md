# Mobility tire identification and physical acquisition v1

Status: deterministic identification and acquisition contracts implemented; no physical
dataset is qualified yet.

## Claim boundary

RNE's current low-order tire law is a transient, load-sensitive combined-slip force element.
The v1 identifier fits only steady longitudinal/lateral stiffness and peak-friction values.
It does not fit relaxation length, load sensitivity, road scale, temperature, pressure,
wear, camber, or aligning moment. A successful synthetic fixture proves only deterministic
software plumbing.

Physical qualification requires all three artifacts:

1. a `rne_mobility_tire_identification_dataset` with complete acquisition-level
   training/holdout separation;
2. a `rne_mobility_tire_acquisition_manifest` bound to that exact dataset SHA-256;
3. every raw, calibration, derivation, road-characterization, and procedure file referenced
   by the manifest under an explicitly supplied evidence root.

The standalone identification result schema v2 calls its provenance bit
`recorded_source_claim`; it cannot emit `physical_measurement`. Only the joined qualification
artifact produced after all three inputs replay successfully carries
`physical_measurement: true`.

Byte identity and self-consistency do not prove that a capture happened or that a calibration
authority is genuine. Review of the retained records remains an external physical-evidence
step.

## Measurement contract

Each complete acquisition has one immutable ID, road/tire condition ID, tire installation ID,
raw-container segment ID, and road-friction scale. Multiple runs may share one raw container,
but a file/segment pair cannot be reused under another acquisition ID. The manifest requires
these converted channels in canonical order:

| Channel | Unit | RNE convention |
|---|---:|---|
| longitudinal slip ratio | `1` | positive wheel surface speed minus carrier-forward speed |
| lateral slip tangent | `1` | negative carrier lateral velocity divided by transport speed |
| normal load | `N` | positive contact-normal load on the wheel |
| longitudinal force | `N` | positive wheel-forward force |
| lateral force | `N` | positive wheel-lateral force |

Every channel declares its sensor identity, measured/derived origin, raw source rate,
converted common-row rate, resolution, expanded uncertainty, and a content-addressed
calibration or derivation record. Derived channels must use the explicit derivation-procedure
class; measured channels cannot use it. Raw source rates may differ, but converted rates must
match and cannot exceed their source rates.

The acquisition must use a shared hardware, IEEE 1588 PTP, or GNSS-disciplined timestamp
domain and declare no more than 1 ms inter-channel uncertainty. PTP is a clock protocol, not
proof of achieved synchronization, so the measured uncertainty remains mandatory. IEEE's
[1588-2019 overview](https://standards.ieee.org/ieee/61588/10624/1588/6825/) defines the
network synchronization scope.

Raw capture may be MDF4, MCAP, rosbag2, or a documented immutable CSV export. ASAM describes
[MDF](https://www.asam.net/standards/detail/mdf/) as a post-measurement storage format and
explicitly supports multiple channel rates; choosing a container does not waive channel,
clock, or calibration evidence.

## Split and fitting protocol

Pure longitudinal and pure lateral acquisitions form training. Combined-slip acquisitions
from at least two conditions are holdout and never refit the model. Both small-slip and
peak-region excitation are required, and every holdout condition has an independent sample
floor and residual gate. This staged evidence order follows the on-vehicle force/slip
measurement and validation approach of
[Van Gennip and McPhee](https://doi.org/10.4271/2018-01-1339); RNE does not claim their
Magic Formula parameterization.

The road-friction scale is never inferred by the v1 fit. Every run instead binds an
instrumented reference tire, calibrated friction trailer, or calibrated bench-surface record.
Reusing a path with a different size or digest is rejected.

## Transient relaxation subgate

`identify_tire_relaxation_length` identifies longitudinal or lateral relaxation length only
after the steady stiffness, peak friction, load sensitivity, and road scale have been frozen.
Its physical rows contain capture time, regularized transport speed, kinematic target slip,
normal load, road-friction scale, and independently measured force on the selected axis. It
does not accept an internal `relaxed_slip` state as if that state were a sensor.

For pure-axis data below a declared force-utilization ceiling, the identifier inverts RNE's
steady law to reconstruct the measured relaxation state:

```text
force = peak * tanh(stiffness * load_ratio * relaxed_slip / peak)
```

It then fits the exact distance-domain first-order update used by the runtime:

```text
x[k+1] = target[k] + (x[k] - target[k]) * exp(-speed[k] * dt[k] / length)
```

The target and speed are zero-order held over each interval. Saturated force rows, low-speed
rows, weakly excited transitions, invalid clocks, and out-of-envelope load/slip are rejected or
excluded according to the frozen spec. Complete acquisition IDs remain disjoint; fitting uses
training only, followed by pooled and per-condition holdout gates. Longitudinal and lateral
lengths are separate calls and may not be silently tied.

This inverse is intentionally not used near force saturation, where `atanh` is ill-conditioned.
It also does not infer the steady tire parameters simultaneously: joint fitting would hide
parameter non-identifiability and must use a separate protocol.

## CLI

The identification fixture is deliberately non-physical:

```text
cargo run -p rne_mobility_benchmark -- --backend tire-identification-fixture --output dataset.json
cargo run -p rne_mobility_benchmark -- --backend tire-identification --input dataset.json --output result.json
```

A recorded dataset is admitted to the physical gate only with its manifest and external
evidence root:

```text
cargo run -p rne_mobility_benchmark -- \
  --backend tire-acquisition-verify \
  --input dataset.json \
  --acquisition-manifest acquisition.json \
  --evidence-root E:/rne-tire-capture \
  --output verified-acquisition.json
```

The verifier bounds manifest and artifact sizes, rejects unknown fields and incomplete or
unordered runs/channels, confines canonicalized paths to the supplied root, streams hashes
without loading large raw captures into memory, and detects file growth, truncation, or digest
drift. Its output is the joined physical-qualification artifact, not a copy of the manifest.

## Remaining physical gate

No repository fixture currently passes as genuine physical evidence. Completion requires an
appropriately licensed retained capture, reviewed calibration and synchronization records,
conversion into the frozen schema, train/holdout execution, and portable application of the
identified tire profile to the shared Rapier/MuJoCo TaskSpec. The relaxation-length software
fit exists, but its owned artifact/acquisition binding and physical execution remain open.
Load-sensitivity identification still requires a separate excitation and validation protocol.

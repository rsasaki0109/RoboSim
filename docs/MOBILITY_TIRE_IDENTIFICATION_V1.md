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

## Load-sensitivity identification subgate

The runtime peak-friction ratio decreases linearly with normalized load until the already
declared minimum-friction clamp:

```text
q = normal_load / reference_load
friction_ratio = max(1 - load_sensitivity * (q - 1), minimum_friction_ratio)
```

`identify_tire_load_sensitivity` is a separate staged fit. It freezes the preceding stiffnesses,
reference-load peak friction, road scale, relaxation parameters, and clamp, then searches only
`load_sensitivity_per_load_ratio`. Retained training samples must have combined slip on both axes,
must bracket the reference load, and must cover a declared minimum load-ratio span. Complete
acquisitions remain disjoint across train and holdout. After fitting, pooled and per-condition
holdout vector-force residuals are evaluated without refitting.

This follows the model-structure lesson from the original
[TMeasy paper](https://doi.org/10.1080/00423110701776284): load-dependent tire characteristics
are distinct physical parameters rather than generic collider friction. Project Chrono's
[official TMeasy implementation](https://github.com/projectchrono/chrono/blob/main/src/chrono_vehicle/wheeled_vehicle/tire/ChTMeasyTire.h)
likewise retains separate nominal-load and twice-nominal-load force characteristics and
interpolates them using `q = Fz / Fz_nom`. RNE does not reproduce TMeasy's full quadratic
interpolation; the present one-parameter law remains RNE's lower-order identifiable profile.

The subgate rejects data whose load range would enter the minimum-friction clamp for any allowed
candidate, because that flat region cannot identify the slope. Its deterministic synthetic tests
recover only a known software fixture and reject unbracketed training or degraded holdout. An
owned, bounded dataset/result artifact replays this fit and binds its frozen steady evidence,
split, provenance, and self-hash. A `rne_mobility_tire_load_sensitivity_acquisition_manifest`
now binds this dataset's complete train/holdout runs to retained physical evidence beneath an
explicit evidence root, reusing the steady manifest's five-channel `TireSignalKind` contract
because the load-sweep dataset records the exact same longitudinal-slip/lateral-slip/normal-load/
longitudinal-force/lateral-force channels. Its qualification streams and rehashes every retained
raw capture, road-friction record, and calibration/derivation record and binds them to the exact
recomputed load-sensitivity fit, mirroring the steady and transient physical gates. Integration
into the four-manifest identified-profile physical gate and cross-backend application of the
fitted coefficient are described below.

The transient path owns its input rather than borrowing process memory. A
`rne_mobility_tire_relaxation_dataset` freezes one axis, owns and replays the preceding steady
dataset and fit evidence, then uses only that fit's tire law with complete transient
training/holdout acquisitions, provenance, and a self-excluding SHA-256. Its
`rne_mobility_tire_relaxation_result` replays the fit and binds the exact dataset, fit contract,
residuals, and source label. The result deliberately calls the provenance bit
`recorded_source_claim`; it is not physical qualification.

Physical qualification additionally requires an axis-aware transient acquisition manifest.
Each run binds synchronized transport speed, kinematic target slip, normal load, and measured
axis force channels, plus raw-segment identity, road-friction evidence, calibration or
derivation records, logger identity, and the exact RNE commit. Only the joined qualification
can emit `physical_measurement: true`, after all referenced files have been streamed and
hashed beneath the explicitly supplied evidence root.

## Backend-neutral profile application

An `rne_mobility_identified_tire_profile` v1 accepts exactly one longitudinal and one lateral
relaxation chain. Profile v2 additionally owns and replays one load-sensitivity dataset/result.
All three staged chains must share the exact same owned steady dataset and steady fit. V2 starts
from that steady result and replaces only the fitted load-sensitivity coefficient and the two
axis-specific relaxation lengths; substituted stiffness, friction, load-sensitivity, or
relaxation values fail validation. The v1 decoder remains accepted for existing three-manifest
physical evidence.

Mobility backend trace schema v2 now retains the complete backend-neutral plant beside the
TaskSpec. The identified-profile runner requires that retained plant's tire element to equal
the replayed profile bit-for-bit. Rapier and MuJoCo receive the same TaskSpec, seed, fixed step,
motor, transmission, wheel, road, and identified tire profile. Their comparison retains the
existing unit-bearing tolerances rather than claiming bitwise state equality across solvers.
The bundled profiles are synthetic and therefore demonstrate application plumbing, not physical
qualification. The v2 fixture is applied unchanged by the same Rapier/MuJoCo runner and retained
inside the backend trace/comparison artifact.

Physical execution has one additional fail-closed boundary. An
`rne_mobility_physical_tire_application_request` embeds the exact identified profile plus its
required acquisition manifests, and is itself versioned to match the profile:

- schema v1 binds profile v1 to exactly three manifests: the shared steady-force dataset,
  longitudinal relaxation, and lateral relaxation. This is byte-compatible with every existing
  three-manifest request and remains the only path accepted for profile v1.
- schema v2 binds profile v2 to all four manifests: the same three plus the load-sweep
  acquisition manifest bound to the profile's owned `TireLoadSensitivityDataset`.

The two versions are mutually exclusive. Request validation fails closed if `schema_version`
disagrees with `profile.schema_version`, if a v1 request carries a load-sensitivity manifest, or
if a v2 request lacks one. Qualification streams and rehashes every retained raw capture,
road-friction record, calibration or derivation record, and acquisition procedure beneath one
explicit evidence root, regardless of version. The resulting
`rne_mobility_physically_qualified_tire_profile` binds every required qualification (three for
v1, four for v2) to the exact profile digest and copies the request's schema version. Only that
joined artifact can enter the physical Rapier/MuJoCo comparison; a `recorded_source_claim` alone
is rejected, and files are always rehashed at qualification time rather than trusting a
previously serialized `physical_measurement` value.

An identified profile can also be applied jointly with an identified suspension
strut. `identified_suspension_tire` fits one suspension
`SuspensionIdentificationDataset`, replays one `IdentifiedTireProfileEvidence`,
and executes the fitted strut plus this profile's `CombinedSlipTireSpec` on the
shared suspended four-wheel road task on both backends. Its combined
`physical_measurement` is the conjunction of the suspension declaration and this
profile's `recorded_source_claim` and is still not qualification; see
[`MOBILITY_SUSPENSION_IDENTIFICATION_V1.md`](MOBILITY_SUSPENSION_IDENTIFICATION_V1.md).

## CLI

The identification fixture is deliberately non-physical:

```text
cargo run -p rne_mobility_benchmark -- --backend tire-identification-fixture --output dataset.json
cargo run -p rne_mobility_benchmark -- --backend tire-identification --input dataset.json --output result.json
cargo run -p rne_mobility_benchmark -- --backend tire-load-sensitivity-fixture --output load-sensitivity.json
cargo run -p rne_mobility_benchmark -- --backend tire-load-sensitivity-identification --input load-sensitivity.json --output load-sensitivity-result.json
cargo run -p rne_mobility_benchmark -- --backend tire-relaxation-fixture --output transient.json
cargo run -p rne_mobility_benchmark -- --backend tire-relaxation-identification --input transient.json --output transient-result.json
cargo run -p rne_mobility_benchmark -- --backend identified-tire-profile-fixture --output tire-profile.json
cargo run -p rne_mobility_benchmark -- --backend load-sensitive-tire-profile-fixture --output load-sensitive-profile.json
cargo run -p rne_mobility_benchmark -- --backend identified-tire-rapier --input load-sensitive-profile.json --output load-sensitive-rapier.json
cargo run -p rne_mobility_benchmark -- --backend identified-tire-rapier --input tire-profile.json --output rapier-application.json
cargo run -p rne_mobility_benchmark --features mujoco -- --backend identified-tire-compare --input load-sensitive-profile.json --output cross-backend-application.json
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

cargo run -p rne_mobility_benchmark -- \
  --backend tire-load-sensitivity-acquisition-verify \
  --input load-sensitivity.json \
  --acquisition-manifest load-acquisition.json \
  --evidence-root E:/rne-tire-capture \
  --output verified-load-acquisition.json

cargo run -p rne_mobility_benchmark -- \
  --backend tire-relaxation-acquisition-verify \
  --input transient.json \
  --acquisition-manifest transient-acquisition.json \
  --evidence-root E:/rne-tire-capture \
  --output verified-transient-acquisition.json
```

After producing the required axis manifests, the CLI validates and binds them with the shared
steady manifest and exact identified profile. It writes the sealed physical-application request,
so an operator never has to calculate or transcribe its self-hash. `physical-tire-request`
detects the profile's schema version from `--input` and requires the matching manifest set: a
profile v1 input accepts only `--steady-acquisition-manifest`,
`--longitudinal-acquisition-manifest`, and `--lateral-acquisition-manifest` and produces a
schema-v1 request; a profile v2 input additionally requires
`--load-acquisition-manifest` and produces a schema-v2 request. The same request is then used for
the standalone file gate and the cross-backend execution gate, unchanged by version:

```text
cargo run -p rne_mobility_benchmark -- \
  --backend physical-tire-request \
  --input tire-profile.json \
  --steady-acquisition-manifest steady-acquisition.json \
  --longitudinal-acquisition-manifest longitudinal-acquisition.json \
  --lateral-acquisition-manifest lateral-acquisition.json \
  --output physical-tire-request.json

cargo run -p rne_mobility_benchmark -- \
  --backend physical-tire-request \
  --input load-sensitive-profile.json \
  --steady-acquisition-manifest steady-acquisition.json \
  --load-acquisition-manifest load-acquisition.json \
  --longitudinal-acquisition-manifest longitudinal-acquisition.json \
  --lateral-acquisition-manifest lateral-acquisition.json \
  --output physical-tire-request-v2.json

cargo run -p rne_mobility_benchmark -- \
  --backend physical-tire-qualify \
  --input physical-tire-request-v2.json \
  --evidence-root E:/rne-tire-capture \
  --output physically-qualified-profile.json

cargo run -p rne_mobility_benchmark --features mujoco -- \
  --backend physical-tire-compare \
  --input physical-tire-request-v2.json \
  --evidence-root E:/rne-tire-capture \
  --output physical-cross-backend-application.json
```

`physical-tire-compare` re-runs the complete file qualification immediately before simulation,
then applies the qualified profile to the same retained plant and TaskSpec on Rapier and MuJoCo.
The emitted comparison contains the joined qualification and both backend traces rather than
trusting a previously serialized boolean. `physical-tire-request` validates exact dataset digests,
axis identities, source kinds, and manifest self-hashes before it writes anything; it does not
read the referenced captures or assert physical qualification.

The physical request schema is versioned to exactly match the profile it carries.
`physical-tire-request` rejects a load-sensitive profile v2 given only the three legacy
manifests, and rejects a profile v1 given a `--load-acquisition-manifest` it cannot bind. This
fail-closed boundary prevents a recorded-source label or a valid software fit from being mistaken
for physical load-sensitivity evidence, in either direction.

The verifier bounds manifest and artifact sizes, rejects unknown fields and incomplete or
unordered runs/channels, confines canonicalized paths to the supplied root, streams hashes
without loading large raw captures into memory, and detects file growth, truncation, or digest
drift. Its output is the joined physical-qualification artifact, not a copy of the manifest.

## Remaining physical gate

No repository fixture currently passes as genuine physical evidence. Completion requires an
appropriately licensed retained capture, reviewed calibration and synchronization records,
conversion into the frozen schema, train/holdout execution, and portable application of the
identified tire profile to the shared Rapier/MuJoCo TaskSpec. The relaxation-length software
fit, owned artifact/acquisition binding, the now four-manifest physical application gate, and
shared Rapier/MuJoCo execution path exist for both profile v1 and profile v2, but no genuine
physical capture has passed either complete chain yet. Acquiring and independently reviewing the
retained steady, load-sweep, and transient files is now the remaining evidence step for the
existing profiles; every current load-sweep, steady, and transient acquisition fixture used by
the automated test suite is synthetic or process-test bytes explicitly labeled as such, never a
genuine capture. Software profile v2 and the shared Rapier/MuJoCo application path already bind
and execute the fitted load-sensitivity coefficient once a request supplies its four qualified
manifests.

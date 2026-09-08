# Suspension identification v1

Status: implemented additive M3-C/M5 identification-and-application contract; physical dataset pending

This subgate adds a backend-neutral path from timestamped suspension measurements to
the linear force law already consumed by the Rapier and MuJoCo Ackermann plants:

```text
F = k (x_eq - x) - c x_dot
```

Each input sample contains capture time, suspension position, suspension velocity, and
generalized strut force with units in its field name. The identifier fits stiffness
`k`, viscous damping `c`, and unloaded equilibrium position `x_eq`. It performs no
wall-clock access or random resampling.

## Identification and validation contract

The force equation is rewritten as a three-coefficient linear regression. Position,
velocity, and force are centered before solving the two-variable normal equation,
reducing intercept conditioning error. A near-singular position/velocity excitation
matrix fails with `RankDeficient`; a constant or mutually dependent excitation is not
accepted as evidence.

Every fifth sample is reserved before fitting. The frozen v1 split therefore prevents
the optimizer from seeing its holdout points. Acceptance independently requires:

- strictly increasing finite capture times and finite SI measurements;
- at least 80 training and 20 holdout samples;
- 50–500 kN/m stiffness, 0.5–50 kN s/m damping, and -0.20–0.10 m equilibrium position;
- finite training and holdout force RMSE no greater than 25 N;
- complete dataset/result digest binding and deterministic recomputation.

Finite input samples can still overflow during force prediction or residual
squaring. Non-finite residual metrics fail with `ResidualExceeded`, including
NaN cancellation between overflowing spring and damping terms. Regression tests
exercise these cases using held-out samples so that the fitted coefficients
remain unchanged; ordinary finite fits retain the existing arithmetic.

The regression was first observed as an actual successful Rust result containing
`holdout_rmse_n: NaN`. After rejection was added, all 53 `rne_robot` library tests
and all 12 MuJoCo-enabled benchmark tests matching `suspension` passed, as did
both crates' all-target Clippy checks with warnings denied. The benchmark test
also seals and decodes the finite JSON dataset before requiring the typed
`ResidualExceeded` error. These focused checks do not constitute a new full CI
run or qualify a physical dataset.

This design is consistent with an experimental quarter-car ARX study that uses
accelerometers above and below the suspension and linear least-squares estimation on
real vehicle test data: [A Quarter Car ARX Model Identification Based on Real Car Test
Data](https://jtec.utem.edu.my/jtec/article/view/2413). The bounded regression and
holdout contract also leaves a direct upgrade path to robust bounded nonlinear least
squares; SciPy's [official `least_squares`
documentation](https://docs.scipy.org/doc/scipy/reference/generated/scipy.optimize.least_squares.html)
defines bound constraints and robust losses such as `soft_l1` and Huber. RNE v1 uses
ordinary least squares intentionally because the current strut law is linear and the
baseline must remain dependency-free and exactly reproducible. Robust loss and
uncertainty intervals remain follow-up work for physical logs.

## Bounded external-data path

The CLI accepts a regular JSON file no larger than 8 MiB, rejects unknown fields, and
verifies the dataset digest before fitting. `source_kind` is one of
`synthetic_fixture`, `recorded_bench`, or `recorded_vehicle`. The result derives
`physical_measurement` from that enum, so the built-in generator cannot accidentally
claim measurement status.

The provenance label is still a declaration, not a signature, trusted timestamp, or
independent attestation. A qualifying physical result must later bind the raw logger,
instrument calibration, robot identity, acquisition procedure, and immutable source
hash through the external-evidence path.

The physical acquisition manifest now makes that boundary executable. A
`recorded_bench` or `recorded_vehicle` dataset does not pass it unless all of the
following are present and internally consistent:

- exact vehicle/rig, strut, logger, logger-software, capture, and RNE commit identities;
- synchronized position, velocity, and force channels in canonical SI units and sign;
- sample rate, resolution, expanded uncertainty, and calibration/derivation class for
  every channel;
- a shared hardware, IEEE 1588 PTP, or GNSS-disciplined clock with no more than 1 ms
  declared inter-channel timestamp uncertainty;
- SHA-256 and exact byte length for the raw capture, acquisition procedure, and every
  calibration or derivation artifact;
- streamed rehashing of those files beneath an explicitly supplied external evidence
  root, including canonical-path containment checks.

This follows the data-integrity shape of [ASAM MDF](https://www.asam.net/standards/detail/mdf/),
which retains raw values, conversion formulas, timestamps, and interpretation metadata.
[NI's bridge guidance](https://www.ni.com/white-paper/11368/en/) identifies excitation,
filtering, offset nulling, and shunt calibration as parts of a load-cell measurement
chain, while [NIST GMP 13](https://www.nist.gov/document/gmp-13-ensuring-traceability-20190621pdf)
requires the SI reference, traceability and uncertainty statements, results, and
documented procedure. MCAP is also admitted as a raw container because its
[official message contract](https://github.com/foxglove/mcap/blob/main/cpp/mcap/include/mcap/types.hpp)
separates log and publish timestamps. Container choice alone never qualifies a capture.

Given a populated manifest and its referenced files on the external SSD, verify the
complete acquisition boundary before fitting:

```powershell
cargo run -p rne_mobility_benchmark -- `
  --backend suspension-acquisition-verify `
  --input E:\RNE-data\capture-001\suspension-dataset.json `
  --acquisition-manifest E:\RNE-data\capture-001\acquisition-manifest.json `
  --evidence-root E:\RNE-data\capture-001
```

Manifest validation and file hashing are qualifications performed by RNE, not an
independent accreditation or a cryptographic signature from the instrument operator.

Generate and fit the contract-only fixture on the external SSD:

```powershell
$env:CARGO_TARGET_DIR = 'E:\RNE-build\m3c-sensor'
$env:TEMP = 'E:\RNE-build\tmp'
$env:TMP = 'E:\RNE-build\tmp'
cargo run -p rne_mobility_benchmark -- `
  --backend suspension-identification-fixture `
  --output E:\RNE-build\m3c-sensor\suspension-identification-synthetic-dataset-v1.json
cargo run -p rne_mobility_benchmark -- `
  --backend suspension-identification `
  --input E:\RNE-build\m3c-sensor\suspension-identification-synthetic-dataset-v1.json `
  --output E:\RNE-build\m3c-sensor\suspension-identification-synthetic-result-v1.json
$env:MUJOCO_DYNAMIC_LINK_DIR = 'E:\RoboSim-mujoco\lib'
$env:PATH = 'E:\RoboSim-mujoco\bin;' + $env:PATH
cargo run -p rne_mobility_benchmark --features mujoco -- `
  --backend identified-road-compare `
  --input E:\RNE-build\m3c-sensor\suspension-identification-synthetic-dataset-v1.json `
  --output E:\RNE-build\m3c-sensor\identified-suspension-road-comparison-v1.json
```

Verified fixture evidence:

| field | value |
| --- | ---: |
| source | `synthetic_fixture` (not physical measurement) |
| samples | 400 (320 train / 80 holdout) |
| dataset digest | `fnv1a64:58aee35ac91cc8e0` |
| result digest | `fnv1a64:baa4b1d2b14c163a` |
| identified stiffness | 199999.728 N/m |
| identified damping | 15000.051 N s/m |
| identified equilibrium position | -0.061000042 m |
| training force RMSE | 0.632 N |
| holdout force RMSE | 0.636 N |
| maximum absolute holdout residual | 1.010 N |

## Identification-to-simulation application

The `identified-road-compare` path closes the software handoff that previously ended at
the fit result. It replaces only stiffness, damping, and equilibrium position in the
portable `SuspensionStrutSpec`; travel, axis, force limit, and unsprung mass remain the
declared road-benchmark geometry. That exact spec is then supplied to both Rapier and
MuJoCo under one road `TaskSpec`. The resulting artifact embeds and binds the source
dataset, recomputed identification evidence, applied strut, both full traces, comparison
metrics, and aggregate verdict.

Verified synthetic application evidence on the external SSD:

| field | value |
| --- | ---: |
| artifact size | 581634 bytes |
| artifact digest | `fnv1a64:24240c35ed465d9e` |
| independent rerun SHA-256 | `B16B626083CF96D83B9312822AE3F1E0E7B9B868BEC51B7BB6B4535FDDDC82E7` (byte-identical) |
| physical measurement | `false` |
| Rapier trace digest | `fnv1a64:bb50c7f4f5810cc4` |
| MuJoCo trace digest | `fnv1a64:fe49cf4096cdc704` |
| forward displacement gap | 0.1682 m (limit 0.5 m) |
| curb normal-impulse gap | 39.2579 N s (limit 50 N s) |
| suspension-velocity gap | 0.2436 m/s (limit 1.0 m/s) |
| vertical-acceleration RMS gap | 9.3157 m/s² (limit 10 m/s²) |
| lift / recontact event gaps | 2 / 2 events (limits 20 / 20) |

The FNV digests detect ordinary artifact drift; they are not cryptographic signatures.
This run demonstrates deterministic parameter transport and backend application, not
vehicle calibration or physical fidelity.

## Remaining physical gate

This fixture proves the schema, split, solver, residual calculation, provenance
propagation, determinism, and tamper rejection. It does not pass the physical acquisition
manifest because its source is synthetic, and it does not identify the RNE vehicle.
M3-C/M5 remain open until a physical bench or vehicle log with calibrated force,
position, velocity, and timing is retained externally; the resulting parameters must
then pass this same two-backend application path and a separately captured road profile.
Confidence/conditioning evidence, outlier-robust fitting, tire parameter
identification, and recorded/shadow/HIL comparison are also still required.

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
`ResidualExceeded` error. These focused checks do not qualify a physical dataset.

Full regression checkpoint: commit `c48c85e` subsequently completed
`cargo run -p xtask -- ci` with exit code 0, including workspace lint/tests,
smoke/RL, headless, OSS parity, 361 fuzz cases and Behavior CI 10/10 seeds.
External log: `E:\RNE-build\m3c-sensor\suspension-finite-v1-ci.log`, SHA-256
`459f8e6d76837652c2e944159c08f4a883bb266d3350336d76092061f15848ed`.
This validates the software revision, not real-world suspension calibration.

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

### Real-data candidate disposition (2026-09-08)

The TU Dresden primary record for [3-component servo-hydraulic test bench
measurements](https://opara.zih.tu-dresden.de/items/eec1959c-6ec6-4ee3-af05-5419f1d2adb6)
(DOI `10.25532/OPARA-151`, 2021-12-17, CC BY 4.0) lists
`Heindel2021_dataset.zip` (529.24 MB). It describes three inertia-compensated
force channels and three displacement channels from an oil-filled hydro-mount
rig, with nonlinear damping, stiffness, and cross-direction pendulum coupling.
The archive has not been downloaded or its sample layout inspected here.

Disposition: candidate for model-discrepancy/virtual-sensing research, **not**
qualified linear-strut or RNE vehicle calibration. The landing-page description
does not establish the synchronized SI velocity, sample clock, instrument
uncertainty, or calibration artifacts required by the acquisition gate. Do not
invent velocity timestamps or relabel this rig as the benchmark vehicle.
Before retaining any archive on the external SSD, inspect available metadata
for those requirements and budget both archive and extraction sizes; preserve
raw-source hashes and distinguish derived velocity from measured velocity.

### Identification validation upgrade

`rne_robot::systems::suspension_residual_timing` evaluates a frozen fit on one
acquisition without refitting or joining run boundaries. It retains interval
min/max, caller-declared absolute interval tolerance, mean force residual and
optional lag-one autocorrelation. Following [NIST's definition](https://www.itl.nist.gov/div898/handbook/eda/section3/eda35c.htm),
the centered adjacent-product sum uses full-run centered energy as denominator.
Only intervals matching the first within the declared tolerance admit this
statistic; irregular timing and constant residuals return `None`, not zero.
No interpolation, independence verdict, effective sample size or uncertainty
interval is inferred.

The benchmark library's separate `rne_suspension_timing_evidence` schema 1
embeds the unchanged excitation/run evidence, a caller-declared interval
tolerance, and ordered training/holdout timing diagnostics. Every acquisition
is evaluated separately using the frozen training fit. The strict decoder and
encoder rerun the embedded request and compare the complete envelope; both use
the same 8 MiB bound. This is replay integrity, not source authentication or
physical qualification. Each acquisition must contain at least three samples.
The library API is available as `identify_suspension_timing`,
`encode_suspension_timing`, and `decode_suspension_timing`. The CLI backend
`suspension-timing` takes a whole-run request via `--input` and requires explicit
`--interval-tolerance-s` justified by clock evidence. Backend
`suspension-timing-verify` takes saved timing evidence via `--input` and rejects
a tolerance override: it replays the embedded declaration. Existing artifact
schemas and acceptance gates are unchanged.

The additive `rne_robot::systems::suspension_training_excitation` diagnostic
accepts training acquisitions only. It returns centered position/velocity RMS
in SI units, correlation, and the L2 condition number of the centered design
after normalizing each column to unit norm. For correlation `rho`, its Gram
matrix has eigenvalues `1 +/- |rho|`, so the design condition is
`sqrt((1+|rho|)/(1-|rho|))`, not the squared Gram condition. The norm convention
follows [NumPy's official condition-number documentation](https://numpy.org/doc/stable/reference/generated/numpy.linalg.cond.html).
Zero-energy columns or `1-|rho| <= 8*EPSILON` return no finite condition, rather
than serializing infinity. Scaling before centering avoids direct SI squaring
overflow. RMS magnitudes remain separate because unit normalization can hide
insufficient excitation amplitude. Forces are validated but do not enter the
calculation. It does not change residual acceptance gates and is not parameter
uncertainty evidence.

The separate `rne_suspension_excitation_evidence` v1 envelope embeds unchanged
whole-run evidence plus these training-only diagnostics. CLI modes
`suspension-excitation` (whole-run request input) and `suspension-excitation-verify`
(envelope input) use the same bounded 8 MiB intake and compact output. Verification
reexecutes both the fit and diagnostics and compares the complete envelope.
Changing holdout forces cannot change the diagnostic; editing a stored diagnostic
is rejected. Neither an acceptable condition number nor a residual verdict
qualifies instrument calibration or independent physical trials.

Excitation-slice checks (2026-09-08): all 59 `rne_robot` library tests and
all-target Clippy passed. Tests include orthogonal/collinear/constant designs,
an analytic near-collinear condition, force independence, extreme finite scales,
and invalid inputs. The MuJoCo-enabled benchmark library passed 163 tests with
zero failures and two ignored long training jobs (250.30 s); all-target Clippy
passed after the envelope/CLI addition. The four run-artifact tests include
diagnostic recomputation and tamper rejection. These are focused and benchmark
regression checks. Subsequently, commit
`6deb13e36acfb273f5cf79178c17308f68b2c440` completed the full
`cargo run -p xtask -- ci` with exit code 0, with tracked files unchanged
through execution. The run ended with OSS parity, 361 fuzz cases across nine
boundaries and Behavior CI 10/10 seeds. External log:
`E:\RNE-build\m3c-sensor\suspension-excitation-v1-ci.log`, SHA-256
`8299f7b9ec95a499031c5e64faa22ffcd2f46b56eae9fedb4fb35fc5917208ae`.
This does not qualify real measurements or supply parameter uncertainty/HIL evidence.

Inspection of `identify_suspension_strut` confirms that v1 holds out every
`holdout_stride`th point from a single ordered sequence. This prevents direct
use of held-out samples in fitting, but does not establish independent trials:
adjacent training points can share autocorrelated noise and operating conditions.
The existing synthetic fixture and its deterministic hashes remain useful
software regression evidence, not a generalization result for physical logs.

Before claiming physical identification, add a separately versioned validation
path with whole acquisition runs held out (or explicitly bounded contiguous
time blocks when independent runs are unavailable). Freeze the split before
fitting, report excitation/conditioning from training data only, and evaluate
force residuals by held-out run and operating range. Any uncertainty estimate
must state its assumptions about correlated samples and measured-input error;
an IID least-squares interval alone must not certify this gate. Tests must prove
that changing holdout forces cannot alter fitted parameters and that run IDs
cannot occur in both training and validation.

The additive `identify_suspension_strut_runs` API now accepts explicit complete
training and holdout acquisitions. IDs must be unique across and within roles;
empty runs, non-finite samples and non-increasing per-run clocks are rejected.
Clocks may restart between runs. All supplied training samples enter the fit;
all holdout samples are excluded. The shared arithmetic preserves v1 sample
ordering and results when given equivalent partitions. `holdout_stride` remains
a validated compatibility field in the reused spec, but does not choose points
in the run API. Counts and RMSE are pooled/sample-weighted, not per-run gates.

`identify_suspension_strut_runs_report` additionally retains each run's ID, sample
count, force RMSE, maximum absolute residual and RMSE verdict in caller order.
It uses the role's existing training/holdout RMSE bound for each run; any failing
run sets the report verdict to false even when pooled residuals pass. Minimum
sample counts still apply to pooled roles, and maximum absolute residual is a
diagnostic, not a separate gate. Pooled fit failures still return an error.

The benchmark now supplies separate v1 `rne_suspension_run_request` and
`rne_suspension_run_evidence` kinds through `--backend suspension-run-identification`
and `--backend suspension-run-verify`, respectively, with `--input` and `--output`.
Requests embed `spec`, `training` and `holdout`; each run contains an
`acquisition_id` and an existing v1 `dataset`. The bound is 8 MiB, 64 total runs
and 100,000 combined samples. Numeric IDs, dataset IDs and exact serialized
sample captures must be unique across the whole request. Changing a source label
does not evade exact sample duplication checks; altered or overlapping captures
are not detected by this check.

Evidence embeds the request, its typed compact-JSON SHA-256, and the full report.
The CLI emits revalidated compact JSON without a trailing newline, bounded by
the same 8 MiB limit as the decoder; pretty-print expansion is not used here.
The verifier actually reruns identification and compares the entire result;
it does not trust a stored hash or verdict. A valid report may have `passed=false`:
successful CLI execution establishes processing/integrity, not model acceptance.
SHA-256 is integrity binding, not authentication. No acquisition-manifest checks
are implied by this path, and recorded source labels remain unverified declarations.

This is not physical qualification. Training-only conditioning, uncertainty
and acquisition-manifest integration remain to implement. Distinct caller IDs
do not detect duplicated raw captures or establish independent measurements.

Whole-run slice validation (2026-09-08): `rne_robot --lib` passed 56 tests;
its all-target Clippy passed with warnings denied. The MuJoCo-enabled mobility
benchmark library passed 162 tests with zero failures and two explicitly ignored
long training jobs (220.66 s); its all-target Clippy also passed. This includes
v1 suspension regressions, whole-run split isolation, failed-run retention,
rehashed split/report tampering rejection and compact evidence roundtrips.
The two long training jobs were not rerun.

Full workspace checkpoint: `fdecb2edb6d94cc9ce78d5133bb7239c52b968e9`
completed `cargo run -p xtask -- ci` with exit code 0, with tracked files fixed
through execution. This included workspace lint/tests, smoke/RL checks,
headless, OSS parity, 361 fuzz cases across nine boundaries and Behavior CI
10/10 seeds. External log:
`E:\RNE-build\m3c-sensor\suspension-runs-v1-ci.log`, SHA-256
`a891981ca2266ccdfc5391b04f698ecae18eda006727cf914821453ab4366155`.
This is regression evidence, not physical dataset qualification or actual HIL.

This fixture proves the schema, split, solver, residual calculation, provenance
propagation, determinism, and tamper rejection. It does not pass the physical acquisition
manifest because its source is synthetic, and it does not identify the RNE vehicle.
M3-C/M5 remain open until a physical bench or vehicle log with calibrated force,
position, velocity, and timing is retained externally; the resulting parameters must
then pass this same two-backend application path and a separately captured road profile.
Confidence/conditioning evidence, outlier-robust fitting, tire parameter
identification, and recorded/shadow/HIL comparison are also still required.

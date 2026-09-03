# Suspension identification v1

Status: implemented additive M3-C/M5 identification-contract subgate; physical dataset pending

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
- training and holdout force RMSE no greater than 25 N;
- complete dataset/result digest binding and deterministic recomputation.

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

## Remaining physical gate

This fixture proves the schema, split, solver, residual calculation, provenance
propagation, determinism, and tamper rejection. It does not identify the RNE vehicle.
M3-C/M5 remain open until a physical bench or vehicle log with calibrated force,
position, velocity, and timing is retained externally; the resulting parameters are
then applied unchanged to both backends and evaluated on a separately captured road
profile. Confidence/conditioning evidence, outlier-robust fitting, tire parameter
identification, and recorded/shadow/HIL comparison are also still required.

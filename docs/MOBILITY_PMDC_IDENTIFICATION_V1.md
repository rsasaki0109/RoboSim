# PMDC effective-identification protocol v1

## Scope

This protocol was frozen before reading development response values. It binds the exact
Mendeley v2 workbook, lossless training/development record streams, whole-run split,
signal reconstruction, candidate models, selection rule and one-shot development gates.
The source experiment and acquisition equations are described in the authors'
[Data in Brief article](https://pmc.ncbi.nlm.nih.gov/articles/PMC8752902/).

The result may identify only effective coefficient ratios convolved with the current
aggregation, encoder processing, H-bridge and timing path. It must not be labelled as an
independent estimate of armature resistance, inductance, torque constant, back-EMF
constant, rotor inertia, gearbox efficiency or sensor calibration.

## Immutable data boundary

- Source workbook SHA-256:
  `85203c4b3ad6fbdd05221e1be7fd41ce733376c0f316d7fd5542b604a6854605`.
- Training record-stream SHA-256:
  `8de4971cbfc8bfd26e3244ead3357ad6950f56a8d57759a7ceab44f0feb12e5b`.
- Development record-stream SHA-256:
  `a9e559ee901221a3663274c4f20939b202491426145580d8e1bd278aa0d04dec`.
- PRBS9 Motor A trials 1--8 are training; trial 9 is one-shot development.
- Final headers `A18101` and `A20112` remain sealed. Protocol v1 does not accept final
  input and no implementation may open those response rows.

Every training fold leaves out one complete run. Samples are never randomly split,
clipped or removed. Timestamps are not resampled and signals are not implicitly filtered.

## Observation construction

Terminal input is the reported `MotorVoltage = VoltageB1 - VoltageA1`. Current is the
reported ten-reading aggregate `Current`, not adjacent `rawCurrent`. Output-shaft speed is
reconstructed from backward encoder-count difference and the actual adjacent source
timestamps:

```text
omega_rpm[k] = -(count[k] - count[k-1]) * 60
               / (1800 * 17 * (time_us[k] - time_us[k-1]) * 1e-6)
```

The first row of each run has no reconstructed speed and is excluded by construction,
not as an outlier. Nonpositive time differences, missing values, formula cells and
nonfinite arithmetic invalidate the run. The source `Velocity` channel remains available
as a diagnostic but is not the model target because the authors calculate it on a fixed
10 ms grid despite the recorded clock variation.

## Candidate equations and identifiability

The simpler electrical candidate is the quasi-static effective relation

```text
I[k] = q_v V[k] + q_w omega[k] + q_0
```

with required signs `q_v > 0` and `q_w < 0`. The nested dynamic candidate is

```text
I[k+1] - I[k] = dt[k] * (a_v V[k] + a_i I[k] + a_w omega[k] + a_0)
```

with `a_v > 0`, `a_i < 0`, and `a_w < 0`. Under an ideal, independently calibrated
acquisition path these resemble `1/L`, `-R/L`, and `-Ke/L`; this dataset does not justify
that inversion, so the fitted values remain effective coefficients.

Both candidates use the same mechanical equation:

```text
omega[k+1] - omega[k]
  = dt[k] * (b_i I[k] + b_w omega[k] + b_s sign(omega[k]) + b_0)
```

with `b_i > 0`, `b_w <= 0`, and `b_s <= 0`. It can expose ratios resembling
`Kt/J`, `-b/J`, and `-tau_c/J`; it cannot separate numerator and inertia or identify
gearbox efficiency. Samples with exactly zero speed use `sign(0) = 0`.

## Deterministic training selection

Each fit uses f64 Householder QR with column pivoting; equal pivot magnitudes select the
lowest original column index. A design is rank deficient when a retained diagonal is at
most `1e-10` times the largest diagonal. No ridge term or coefficient clamp is allowed.

For each held-out training run, current one-step RMSE is divided by that run's P95 minus
P5. Quantiles use Hyndman-Fan type 7 linear interpolation. A zero/nonfinite range is an
invalid fold. The cross-validation score is the arithmetic mean over the eight runs.
Select the quasi-static candidate unless the dynamic candidate has valid physical signs
and improves mean current NRMSE by at least `0.02` absolute. Invalid signs or rank failure
reject a candidate; they are not repaired.

## One-shot development gates

After selecting and refitting on all eight training runs, begin coupled rollout from the
first observed current and reconstructed speed of development trial 9. Apply reported
terminal voltage and actual source intervals without teacher forcing. All normalization
ranges below are training aggregate P95 minus P5 using type 7 quantiles:

| Metric | Maximum |
|---|---:|
| current rollout RMSE / training current range | 0.20 |
| output-speed rollout RMSE / training speed range | 0.15 |
| absolute current mean error / training current range | 0.05 |
| absolute output-speed mean error / training speed range | 0.05 |

All four gates must pass. Repeated executions must reproduce coefficients within
`1e-12` absolute. A failure is evidence that protocol v1 failed, not permission to tune
against trial 9 and rerun the same gate. Any revised protocol must declare trial 9 exposed
and keep the two final runs sealed until an independent development source is selected.

The executable frozen representation and drift tests are in
`tests/mobility_benchmark/src/recorded_pmdc.rs`. Model fitting and development evaluation
remain unimplemented at this checkpoint; therefore no accuracy or physical-parameter
qualification is claimed.

# DDMR recorded wheel-response experiment

Status: exploratory empirical response, not physical motor/tire calibration.

This protocol is fixed before running the real-data response fit. The source is
the external DDMR CSV qualified only for format in
[the source audit](MOBILITY_REAL_LOG_SOURCE_AUDIT.md). No dataset is bundled.

## Fixed protocol

- Source SHA-256: `c278dde8bfc38974bb2b1cc054160349da51456817c898ea2e052656aa75f60e`.
- Raw records: 338,550. Training rows `[0,203130)`, validation `[203130,270840)`,
  test `[270840,338550)`, all zero-based. Build transitions within each partition.
- Each wheel independently uses `w[k+1] = a*w[k] + b*u[k] + c`.
  Coefficient units are dimensionless, (rad/s)/V and rad/s. There is no cross-wheel
  coupling, tire-slip state, current state, or physical-parameter interpretation.
- Fit centered ordinary least squares on training only. Reject constant regressors,
  normalized determinant <= 1e-10 and nonfinite arithmetic. Do not clip coefficients
  or enforce stability; report whether each fitted pole has magnitude below one.
- Source interval assumption: 0.01 s, absolute tolerance 1e-9 s. A one-row input
  delay is an explicit model assumption, not a measured hardware latency.
- No model-order, input-delay, offset, hyperparameter or split search. Validation
  and test results cannot revise this fit. Future model selection needs a distinct
  protocol and must disclose reuse of an already inspected test recording.

## Two different evaluations

One-step prediction uses the last measured speed at every transition. Its baseline
persists that measured speed. Free-running simulation uses only the partition's
first measured speed to initialize its recurrence, then consumes recorded voltages
without further measured-speed correction. Its baseline holds the initial speed.
Neither mode uses another partition's history or estimates initial conditions from
future labels. Both report RMSE and maximum absolute error in rad/s over every
transition, separately for each wheel. There is no pass threshold selected from
these results. Execution errors remain explicit, with unavailable metrics rather
than fabricated zero errors.

The distinction follows the primary
[system-identification documentation on prediction and simulation](https://www.mathworks.com/help/ident/ug/definition-simulation-and-prediction.html):
prediction can use previous measured outputs whereas simulation does not use them
after initialization. Low one-step error alone is not evidence of simulator fidelity.

The recorded voltage may be a command or a converted PWM quantity rather than a
terminal-voltage measurement. Source times and encoder processing are unqualified.
Even a good free-running fit would establish only within-recording empirical
response, not independent-session validation, unbiased parameter estimation,
closed-loop robustness, drivetrain electrical identification or slip fidelity.
The empirical model does not replace either RNE physics backend.

## Reproduction and checks

```text
cargo test -p rne_mobility_benchmark recorded_ddmr --lib
cargo run -p rne_mobility_benchmark --example ddmr_response_experiment -- <Data.csv> <new-output.json>
```

The example requires the pinned source, writes with `create_new` and records source,
reader, model and experiment hashes plus compiler/build metadata. A dirty build is
reported honestly. It fits once, then evaluates each partition in both modes.
Output files belong on external storage. Synthetic tests recover known coefficients,
check partition independence, and perturb one held-out label to prove that it does
not correct free-running state. Rank, clock and overflow failures are tested.

## First real-data result (2026-09-08)

The fixed example completed with no evaluation execution errors. It fitted
203,129 training transitions and evaluated 67,709 transitions in each held-out
partition. Coefficients `[a,b,c]` were:

- Left: `[0.9743711510194542, 0.07115268839145696, 0.05541272651116269]`.
- Right: `[0.9749575335430659, 0.07150112418378084, 0.037101345578472]`.

Both scalar poles are inside the unit circle. This is not a physical validation.
RMSE below is in rad/s, left/right; no failed intervals were removed.

| Partition | One-step model | Last-measurement baseline | Free-running model | Initial-speed baseline |
| --- | --- | --- | --- | --- |
| Training | 0.438550 / 0.421038 | 0.456472 / 0.439131 | 2.529647 / 2.441151 | 14.570763 / 14.560123 |
| Validation | 0.423114 / 0.426014 | 0.439469 / 0.442010 | 2.664322 / 2.762577 | 11.935306 / 11.345669 |
| Test | 0.421267 / 0.410315 | 0.438780 / 0.432001 | 2.628668 / 2.649987 | 10.851621 / 10.652072 |

Test maximum absolute errors were 3.782979/6.479964 rad/s for one-step and
11.457624/11.635549 rad/s for free-run. One-step improvement over persistence is
small; substantially larger free-running error prevents treating the prediction
score as evidence of a high-fidelity simulator. The weak initial-speed baseline
is explicitly different from the one-step baseline. No model was retuned after
these results. Unknown capture instrumentation remains a separate validity gap.

Complete evidence: `E:\RoboSim-external-data\mobility-ddmr-6291b0d7\response-experiment-v1.json`.
SHA-256: `72ac819e2a987a249f5447ba201c14a6d61d36bbde16ea65079f0df8ed7b51c2`.
This was a dirty build based on `3dbc897`, with exact experiment, reader and
response source hashes in the artifact. Six focused DDMR tests and default-feature
crate all-target Clippy passed. Full regression for the new response code remains
pending; the earlier 133-test MuJoCo regression predates this addition.

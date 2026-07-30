# Controller evaluation

`rne_ai::control_eval` turns recorded closed-loop runs into the standard quantities a
control engineer reads off a tracking plot, and aggregates them across seeds so a claim
like "controller A beats controller B" carries a mean and a spread instead of an
anecdote from one lucky run.

## Metrics

The input is deliberately plain data — `ControlTrackingSample { time_s,
tracking_error_m, command, saturated, violation }` — not a trait over environments. Any
loop that can log those five values can be evaluated: the kinematic plant, the dynamic
plant, or hardware telemetry read back from a log.

`ControlMetrics::from_samples(samples, settling_band_m)` computes:

| metric | meaning |
| --- | --- |
| `rms_error_m` | root-mean-square tracking error |
| `max_error_m` | worst tracking error |
| `settling_time_s` | first time after which the error never leaves the band again; `None` when the run never settles, which is itself a result |
| `overshoot_m` | worst error after the band is first entered |
| `steady_state_error_m` | mean error over the settled tail |
| `effort` | time integral of the command magnitude |
| `smoothness` | total variation of the command; lower is smoother |
| `saturated_fraction` | fraction of steps with a saturated actuator or tire |
| `violation_count` | steps that broke a hard constraint |

Settling is computed backward from the end of the run, so a temporary dip into the band
does not count — the error must stay inside.

`ControlEvalReport::from_seed_metrics` aggregates per-seed metrics into mean, standard
deviation, min, and max per metric (a `MetricSpread`), counts total violations and
unsettled seeds, and serializes to pretty JSON for artifacts. Seeds live in a
`BTreeMap`, so reports are byte-stable regardless of evaluation order.

## Actuator lag

`VehicleDynamics::steering_lag_s` adds a first-order lag ahead of the steering rate
limit: the steering column follows the command with time constant `tau` instead of
reaching it within one tick. This is the phase loss that destabilizes aggressively
tuned controllers on hardware while they look fine against an instant plant. `0.0`
reduces exactly to the previous behaviour.

## Example 50: same controller, ten seeds, two verdicts

One pure-pursuit controller runs the sweeper course from example 49 at 12.5 m/s. Each
of ten seeds perturbs, through deterministic `KeyedRandom` draws:

| condition | range |
| --- | --- |
| tire-road friction | 0.72 – 0.95 |
| initial lateral offset | ±1.5 m |
| steering actuator lag | 50 – 180 ms |

The corner demands ~8.7 m/s² of lateral acceleration, which sits inside the randomized
grip range `mu g` = 7.1 – 9.3 m/s²: low-friction seeds saturate and understeer while
high-friction seeds hold the line. Committed results:

| plant | RMS error | worst seed | saturated | unsettled |
| --- | --- | --- | --- | --- |
| kinematic | 0.507 ± 0.066 m | 1.38 m | 0 % | 0 / 10 |
| dynamic | 5.03 ± 2.29 m | 16.3 m | 59 % | 7 / 10 |

The same controller, the same commands: the no-slip plant reports a competent
controller, the dynamic plant exposes what it lacks — no speed adaptation ahead of the
corner and no compensation for actuator lag. The ±2.29 m spread across seeds is the
reason multi-seed evaluation exists; any single seed would have told a different story.

```bash
cargo run --release -p control_eval_demo --example 50_control_eval
RNE_SKIP_GPU=1 cargo run -p control_eval_demo --example 50_control_eval
```

Both reports are written to `target/control-eval/*.json`. The rendered GIF fans all
ten dynamic-seed trails over the shared course; the spread through the corner is the
statistics made visible.

The acceptance tests require seed conditions to be deterministic and bounded, a
repeated run to be bit-identical, the kinematic plant to flatter the controller (lower
RMS, zero saturation), the randomized conditions to move the metrics, and the report to
serialize with every seed present.

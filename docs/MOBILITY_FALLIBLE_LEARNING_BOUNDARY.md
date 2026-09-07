# Mobility fallible learning boundary

Status: implementation contract for the next learning integration; not implemented.
This follows the fixed sensor environment and independent noise reset work. It does
not replace real-log calibration, hardware evidence or measured throughput gates.

## Observed mismatch

`rne_ai::Episode::step/reset` return `EpisodeStep` directly, without a failure
result. `EpisodeStep::termination`, generic vectorized `success_count` and portable
batch `success_count` interpret termination as success. Existing portable and
generic batches consume that infallible episode interface. `rne_py` depends on
`rne_ai`, not on the mobility benchmark package or its backend implementations.

In contrast, `SensorFixedEnvironment` can fail after partial physical progress and
require reset. `SensorFixedBatch` retains outcomes for every lane; action preflight
is atomic but physical execution is not. Its fixed task ends by horizon truncation,
not success. These semantics cannot be faithfully encoded by catching an error and
returning a synthetic successful terminal transition.

## Required implementation boundary

Add an opt-in fallible learning adapter; do not change the semantics of existing
infallible episodes or force physics backends to implement them. Keep benchmark
and backend types out of `rne_ai` public contracts and do not make `rne_py` depend
on a benchmark/test package just to expose this environment.

- Provide TaskSpec-ordered, shape-checked actor tensors and validity/freshness
  masks. No world reference, reset parameters, true velocity or physical hash may
  enter policy inference. Reset roots are explicit evaluator configuration.
- Keep successful transition rewards and horizon truncation separate from actor
  observation tensors. A tracking-task horizon must not be labeled success.
- Distinguish batch preflight rejection from per-lane execution failure. Preserve
  which lanes advanced and which require reset; never insert zero reward, fake
  observations or synthetic terminal transitions for a failed execution.
- A failed lane is excluded from learning updates until explicitly reset. Healthy
  lane results remain available, with original lane/episode identities and clocks.
  Do not discard all healthy results because a neighboring lane failed.
- Partial reset must not advance other lanes or enter the rollout as an action.
  Preserve the existing explicit episode index and independent noise-root policy.
- Bind the actual actions, resets and outcomes into replay evidence. A learner
  checkpoint additionally needs policy/optimizer and learner-RNG state; the current
  action replay is not already a resumable training checkpoint.

## Demonstration and acceptance

First connect a real policy update loop to persistent sensor-only batch stepping,
not pre-generated actions or a batch of completed reference-controller episodes.
Demonstrate training on declared train seeds and evaluate once on disjoint held-out
reset/noise seeds, without changing parameters after examining held-out results.
Retain unsuccessful episodes. An improved training reward is not real calibration.

Tests must inject an invalid action and a post-physics failure, verify the exact
lane/mutation/reset behavior, and prove that failed steps never become optimizer
samples. Compare worker counts and partial resets using recorded actions. Execute
both Rapier and MuJoCo; keep within-backend replay separate from SI-unit cross-
backend tolerances. Measure actual batch stepping and policy-update throughput
with rendering disabled and external SSD storage. Python exposure, if needed,
must preserve these fallible contracts rather than silently coercing them into
the current infallible API. Full-workspace CI is required after implementation.

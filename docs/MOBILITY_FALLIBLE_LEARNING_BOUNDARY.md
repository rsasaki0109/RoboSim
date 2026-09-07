# Mobility fallible learning boundary

Status: fallible transition projection and a reference online policy-update loop
implemented; broader learner integration and full validation remain incomplete.
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

## Implemented transition projection

`SensorFixedBatch::step_learning` steps the persistent batch and returns separate
`transitions` and `failures` collections. A `LearningTransition` binds before/after
TaskSpec-ordered sensor-only tensors, applied voltage, evaluator reward, lane and
episode identity, exact interval timestamps and horizon truncation. It has no
success flag, physical parameters, world handle or physical-state hash. Actor
tensor shapes and finite values are checked against the fixed TaskSpec.

Preflight rejection advances no physics. After stepping, a failed lane has only a
`LearningLaneFailure` diagnostic, never a zero-reward or fake terminal transition.
Healthy lane transitions are retained. Invalid post-step output also invalidates
that lane until explicit reset. Existing `reset_lanes` does not create a learning
transition; the next transition retains its actual lane-local clock and episode.

The adapter does not reinterpret `rne_ai::Episode`, automatically reset, call an
optimizer, or expose the benchmark through `rne_py`. Failure exclusion from this
transition collection alone is not proof of correct learner integration.
Tests cover invalid actions, a complete 330-step horizon,
post-physics errors/panics, healthy-lane retention and recovery after partial reset.
Both initial targeted tests and crate Clippy passed before the reference learner
was added. Updated backend and regression checks are tracked below.

## Reference learner (integration diagnostic)

`SensorTableLearner::train_step` now chooses an action from current sensor tensors,
executes the persistent batch, and updates Q values only for valid transitions.
The bounded voltage grid is [0, 3, 12, 18, 24] V. The table keys contain the
10 ms episode index, estimate-valid mask and estimated speed quantized at 0.25 m/s
and clipped to [-2, 2] m/s. Missing estimates have a dedicated key. No physical
parameters, reference speed measurement or privileged velocity enters the actor.
The fixed task's target schedule is implicit in episode time; this is not a
general-purpose observation encoder. Exploration uses an independent explicit
seed and lane/global-decision coordinates (the caller must not reuse a decision
index across resets). Greedy evaluation neither explores nor updates.

The update follows [Watkins and Dayan's Q-learning rule](https://doi.org/10.1007/BF00992698)
with alpha=0.1, discount=0.99 and epsilon=0.2. The objective here is explicitly the
finite 330-action return: horizon continuation is zero, without labeling it
success. This is not the bootstrap policy for an arbitrary continuing task.
Delayed/quantized observations are not assumed Markov, constant alpha does not
meet the usual diminishing-step assumptions, and no convergence claim is made.
Update validation is transactional; physical stepping is not. Execution failure
diagnostics never enter the learner. This small CPU reference is an integration
diagnostic, not evidence of improved control, scalability or sim-to-real transfer.
Held-out policy-quality evaluation, measured throughput, checkpointing and full
regression validation remain required by the acceptance plan above.

Current focused checks passed: four reference-learner tests with the `mujoco`
feature, both fallible-projection tests, and crate Clippy including MuJoCo. The
online tests execute 330 steps in two actual worlds for each backend, repeat with
one versus two workers, compare the resulting tables exactly within each backend,
and verify 660 updates with nonzero learned values. A separate numerical test
checks the Bellman update, zero finite-horizon continuation, changed greedy action
and read-only evaluation. Invalid sample updates are atomic. Injected post-step
errors and panics in lane 1 produce exactly two updates from healthy lanes; an
explicit partial reset then permits three further updates. These tests measure
integration correctness, not task success or generalization. Full CI is in
progress; its log is on the external SSD at
`E:\RNE-build\m3c-sensor\reference-learner-ci.log`.

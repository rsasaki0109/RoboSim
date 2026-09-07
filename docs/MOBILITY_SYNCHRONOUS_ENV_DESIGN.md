# Mobility synchronous environment: implementation contract

Status: design for the next implementation slice, **not an implemented API**.
The existing episode-parallel runner and policy callback are documented in
[sensor episode batch v1](MOBILITY_SENSOR_EPISODE_BATCH_V1.md).

## Evidence driving the design

The current observed loop advances physics by 1 ms, samples sensor frontends,
then invokes its policy only when a newly available encoder pair can update the
estimator. It holds the previous voltage otherwise. Its TaskSpec's nominal
control period is 10 ms; that does not make callback intervals fixed.

In the external `episode-rapier-workers-1.json` evidence (root 42, episode 0),
each of three lanes has 329 decisions. Lane 0 intervals range 8–19 ms, lane 1
8–20 ms, lane 2 8–22 ms. First decisions occur at 2, 1 and 3 ms respectively;
the last decisions precede the 3.3 s physical horizon. Therefore a callback
counter is neither a common physical-time barrier nor the terminal timestamp.

[Farama's autoreset specification](https://farama.org/Vector-Autoreset-Mode)
distinguishes next-step, same-step and disabled reset modes; replay handling
depends on which mode is used. We select explicit reset initially, preserving
terminal observations without introducing ignored actions during autoreset.
[MuJoCo's rollout documentation](https://mujoco.readthedocs.io/en/stable/python.html#rollout)
describes parallel trajectory execution with worker-local data. This informs
ownership, but trajectory batching alone does not supply our feedback step API.

## Required implementation

1. Extract a backend-neutral persistent single-world runtime from `observed.rs`.
   Own the backend, physics world ID, ECS, sensor bus, estimator, drive state,
   pending wrench and simulation counter. Do not rebuild/replay a prefix for
   each action and do not use wall-clock timing in the state machine.
2. Preserve the existing event-driven reference/replay execution path exactly.
   Existing voltage traces and hashes are regression evidence, not something to
   silently rewrite to accommodate a new control schedule.
3. Give the fixed-period environment a distinct TaskSpec/trace contract. A call
   advances exactly ten 1 ms ticks, holds one validated voltage throughout,
   and returns at a common physical-time boundary. Wrench staging remains
   explicit. The horizon is 330 control steps / 3.3 s, including settling.
4. Sample/deliver frontends and update the estimator at physics ticks. Returning
   the latest available estimate at a control boundary must not fabricate a new
   sensor capture. Expose validity, capture/availability provenance, age and
   whether an estimate is new. At reset, an unavailable delayed measurement is
   explicitly invalid, not silently obtained from truth or future frames.
5. Return fallible transitions. Non-finite or out-of-bounds actions are rejected
   before any state mutation. Backend/sensor faults are errors, never task
   success. A solver error after mutation poisons the lane until explicit reset;
   do not claim rollback. Horizon truncation is separate from task acceptance.
6. Reset reconstructs all owned state, including queues, estimator history and
   pending force. Explicit `(root_seed, lane_id, episode_index)` derives the
   reset seed. Record the fixed WorldRandom noise seed honestly until a separate
   noise-stream contract is introduced. No hidden radius enters the estimator.
7. Build batched stepping over these persistent runtimes. Validate the entire
   action batch and lane mask before advancing any lane. Keep stable lane IDs;
   no auto-reset in v1. Retain per-lane failure/terminal status when another
   lane fails. Do not promise an atomic rollback of physical solver operations.
8. Separate actor results from privileged evaluator results. Truth-based reward
   calculation and final physics hash belong to the evaluator; runtime/world
   access must not be embedded in actor observations. Report transition duration
   and define reward accumulation over the same interval before training use.

The existing `rne_ai::Episode` and portable batch APIs assume infallible stepping.
Do not convert solver errors into panic or invented terminal success to fit them;
establish a fallible adapter boundary before reusing checkpoint infrastructure.

## Acceptance tests before claiming completion

- Existing Rapier and MuJoCo event-driven full traces remain exactly unchanged.
- Fixed-step timestamps are common across lanes despite different jitter/dropout.
- No observation references a capture/delivery later than the returned time.
- Invalid input leaves a replay-equivalent lane unchanged; solver failure marks
  the lane unusable until reset. A mixed batch reports which lanes advanced.
- Repeating an explicit reset tuple clears delayed frames and stale actions and
  reproduces the full trace, including after a prior failed episode.
- Terminal observation, duration, final physics hash and truncation occur at the
  horizon even when the final sensor capture precedes it. Post-terminal step
  rejects input until reset, and reset is not counted as an action transition.
- Lane width, worker count and partial reset of other lanes do not alter a
  continuing lane. Replay includes reset operations and per-lane episode seeds.
- Run both backends, within-backend exact replay and cross-backend SI-unit
  tolerances; retain unsuccessful tasks. Run workspace CI on the final code.

This design does not close the broader goal's per-wheel/Ackermann randomization,
real-log calibration, independent noise resets, HIL or throughput gates.

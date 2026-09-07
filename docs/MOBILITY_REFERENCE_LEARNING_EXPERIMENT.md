# Fixed sensor reference-learning experiment v1

Status: preregistered diagnostic protocol; no evaluation results yet.

This experiment exercises actual online updates and a frozen sensor-only policy
on persistent physical worlds. It does not replace real-log calibration, safety
qualification, actuator/sensor coverage, or cross-backend SI tolerance gates.
The table learner is deliberately small and partially observed; poor performance
must be reported, not repaired by changing this protocol after evaluation.

## Frozen setup

- Train only on Rapier: 32 explicit episodes per lane, four lanes, two workers.
  Episode indices are 0..31; 330 actions per episode; 42,240 updates if all steps
  execute successfully. Physical root 1,310,001; noise root 2,310,001; exploration
  root 3,310,001. A global decision index continues across episode resets.
- Freeze `SensorTableLearner` after training. Keep its existing voltage grid,
  state bins, alpha, discount and exploration settings unchanged. Do not select
  among training checkpoints using evaluation outcomes.
- Evaluate eight fresh lanes at episode index zero, two workers, physical root
  4,310,001 and noise root 5,310,001. Check actual derived physical and noise seed
  sets for disjointness from training before execution.
- Run the same held-out seeds on Rapier and MuJoCo. Compare the frozen learner
  (greedy, no updates), its untrained greedy initialization, and a sensor-only PI
  baseline with fixed gains Kp=15 V s/m, Ki=40 V/m, integral clamp +/-2 m and
  voltage clamp +/-24 V. The PI baseline updates every 10 ms from the latest
  estimate (zero voltage when no estimate); this sampling differs from the older
  event-driven PI benchmark and must not be conflated with its results.
- No held-out retuning or rerunning to select favorable samples. Subsequent
  algorithm changes require a separately declared protocol and fresh held-out
  seeds. Infrastructure correction, if necessary, must be documented explicitly.

## Evidence to retain

Emit all per-lane returns, accumulated tracking error in meters, valid action
counts, horizon-completion flags and execution-error diagnostics, including
unsuccessful runs. Horizon completion is not task success. Reward at each step
is minus privileged tracking-error integral minus 0.001, kept outside actor
tensors. The accumulated error can be recovered as `-return - 0.001 * steps`.
Do not invent final ground-truth velocity from a sensor estimate.

A solver failure aborts further stepping of that batch, records every lane's
partial result, and invalidates a completed-performance comparison; no failed
lane is silently dropped or reset into a replacement evaluation episode.

Record actual training wall duration and completed updates per second with
rendering disabled. This includes action selection, physical stepping, tensor
projection and learning updates; report batch construction/reset time separately
or identify its inclusion. Wall-clock measurements belong to the external
experiment harness, never simulation logic. Small CPU throughput does not imply
GPU scalability. Retain the code revision/diff identity and pinned protocol along
with the results on the external SSD. Report paired comparisons on all eight
lanes, not only a favorable mean. Do not interpret within-backend determinism as
cross-backend agreement or sim-to-real accuracy.

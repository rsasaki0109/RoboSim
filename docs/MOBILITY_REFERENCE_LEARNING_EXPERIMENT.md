# Fixed sensor reference-learning experiment v1

Status: preregistered diagnostic executed once; results recorded below.

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

## Observed result (2026-09-08)

Protocol and executable source were committed as
`07e7cbd14ccac87de6b6bbb8236d34406a61f1bb` before evaluation. The run exited 0.
No held-out retuning or second evaluation was performed. The complete JSON,
including all 32 x 4 training returns and all 48 evaluation lane records, is at
`E:\RNE-build\m3c-sensor\reference-learning-v1.json`, SHA-256
`58951fc386fc31820476a17089de8ca4f8d34c22349e95f9779e9e3680d19091`.

Training performed 42,240 real action-transition updates in 17.7231089 s:
2,383.329 updates/s, including world construction, resets and all loop work.
This is one local CPU measurement in the development profile (optimized with
debug information), after the existing CI process ended, not a repeated hardware
benchmark or a GPU throughput claim. All 48 evaluation runs completed 330 actions
without execution errors. Their horizon flags mean completion, not task success.
The reward-to-integral identity was checked for every lane with 1e-10 tolerance.

Mean tracking-error integral (m; lower is better):

| Policy | Rapier | MuJoCo |
|---|---:|---:|
| Untrained greedy | 2.984363 | 2.984068 |
| Trained, frozen greedy | 1.160770 | 1.146238 |
| Fixed PI 15/40 | 0.504832 | 0.503883 |

All paired lane values (m, rounded to six decimal places):

| Lane | Rapier untrained | Rapier trained | Rapier PI | MuJoCo untrained | MuJoCo trained | MuJoCo PI |
|---|---:|---:|---:|---:|---:|---:|
| 0 | 2.778118 | 0.782403 | 0.430187 | 2.772069 | 1.035872 | 0.427872 |
| 1 | 3.466795 | 1.838152 | 0.645714 | 3.474087 | 1.880876 | 0.648066 |
| 2 | 3.075091 | 1.365665 | 0.472712 | 3.078807 | 1.182034 | 0.472623 |
| 3 | 2.731519 | 0.874421 | 0.470939 | 2.726929 | 0.803087 | 0.468670 |
| 4 | 2.763810 | 0.832032 | 0.464443 | 2.757585 | 0.806955 | 0.462054 |
| 5 | 3.125384 | 1.355555 | 0.553009 | 3.133191 | 1.273157 | 0.553336 |
| 6 | 2.999910 | 1.136900 | 0.473411 | 2.999877 | 1.169867 | 0.471620 |
| 7 | 2.934279 | 1.101031 | 0.528242 | 2.930002 | 1.018059 | 0.526818 |

The trained table improved over zero-initialized greedy actions in every paired
lane on both backends, but was worse than PI in every lane. This supports a
working online-learning integration and limited held-out improvement over a weak
baseline, **not** a superior controller. The policies choose their own actions on
each backend, so these totals are not the same-action cross-backend SI conformance
gate. No real-world accuracy, final-speed success, calibrated uncertainty, learned
checkpoint restoration or HIL behavior is established by this experiment.

Next work must preserve this negative PI comparison. Broader policy algorithms
need a new preregistered evaluation; fixed-step action/learner checkpoint replay
and common Failure Capsules remain missing. These gaps remain part of the full
Mobility Foundation goal rather than being waived by a runnable learning demo.

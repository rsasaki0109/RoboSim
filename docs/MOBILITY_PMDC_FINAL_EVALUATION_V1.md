# PMDC final-evaluation protocol v1

This contract was frozen after the one-shot trial-9 development pass and before either
final response run was converted or read. It binds the selected training evidence
`dc9995c...361d`, the exposed development evidence `1cf7b5...24c71`, source trials 10
and 11, headers `A18101` and `A20112`, and 2,009 source samples per run. Model refitting,
candidate reselection and threshold changes are forbidden.

The executable Rust contract has SHA-256
`c8ed0ce2f34fa90fd1797b42dee02f2aa721760f504d9590c1e3b659220fca53`.
Its `pmdc_final_protocol` headless example emits the full contract without accepting or
opening a data path.

Each final run uses the identical actual-timestamp encoder reconstruction and free-run
rollout initialized from its first observation. The four development metrics and maximums
are unchanged. Every metric must pass at three aggregation levels:

1. each complete run independently;
2. pooled residual sums across both runs, normalized by the unchanged training scale;
3. the worse of the two run-normalized values.

The final verdict is the conjunction of all per-run, pooled and worst-run gates. Conversion
and evaluation each use exclusive creation and may occur only once. A failed evaluation is
retained rather than replaced or tuned away.

The existing common `rne_log::FailureCapsule` requires a fixed simulation timestep and a
simulation timestamp. These physical records have an explicitly nonuniform source clock;
resampling is forbidden. Protocol v1 therefore refuses to mislabel the recorded evaluation
as a fixed-step simulation capsule. A failed final result must retain its content-bound
evaluation evidence, but common Failure Capsule packaging remains fail-closed until RNE
supports a variable-source-timestamp recorded replay clock. That infrastructure gap is
part of the active Mobility Physical AI goal, not a reason to weaken this contract.

The executable contract and drift tests are in `recorded_pmdc::final_protocol`. The
`pmdc_final_protocol` example prints its canonical SHA-256 without accessing final data.
After that pre-data contract commit, the dedicated sealer was separately committed and
then executed exactly once. It exclusively created one external-SSD artifact containing
both complete final runs: 2,009 records from trial 10 and 2,009 from trial 11. The artifact
is 1,261,025 bytes with SHA-256 `94c28290...772d2`; its 4,018 canonical record lines have
SHA-256 `722af621...1409`. These identities are frozen in Rust independently of the
protocol digest, so the already-bound protocol cannot silently change.

The final evaluator was then frozen in `recorded_pmdc::final_evaluation` before its first
real execution. It re-verifies the exact training identification and final artifact,
performs no refit, and records twelve ordered gate objects. Each `per_run` gate retains
two named scalar values; each `pooled` and `worst_run` gate retains one scalar value.
The evidence validator also requires every worst-run value to equal the maximum of its
two per-run values. The headless writer reserves a new output path before opening either
input, writes pass or fail evidence, and only then returns a failing exit status when a
gate is missed.

The sole final evaluation passed all twelve gates over 2,007 rollout comparisons in each
run. Values below are dimensionless and use the unchanged training scales:

| Metric | Trial 10 | Trial 11 | Pooled | Worst run | Maximum |
| --- | ---: | ---: | ---: | ---: | ---: |
| Current rollout NRMSE | 0.177537 | 0.180339 | 0.178943 | 0.180339 | 0.20 |
| Output-speed rollout NRMSE | 0.014450 | 0.014655 | 0.014553 | 0.014655 | 0.15 |
| Current signed-bias fraction | 0.037847 | 0.042101 | 0.039974 | 0.042101 | 0.05 |
| Output-speed signed-bias fraction | 0.006282 | 0.006551 | 0.006416 | 0.006551 | 0.05 |

The retained 4,791-byte evidence file has SHA-256 `59f0f967...d2c32` and content digest
`5f4e0b8a...a520b`; both are frozen in Rust. A first command with an incorrect training
filename failed at the training-file `stat` before reading either input or computing any
metric. Its verified zero-byte reservation is retained as
`pmdc-final-evaluation-v1.failed-preflight-empty` rather than erased. The corrected sole
evaluation did not refit, reselect, change thresholds, resample timestamps, qualify
individual physical constants, or create a misleading fixed-clock Failure Capsule.

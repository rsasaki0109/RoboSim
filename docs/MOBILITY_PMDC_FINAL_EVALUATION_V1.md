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

At this checkpoint the final partition has been read and losslessly sealed, but its
response values have not been evaluated and no final metric or verdict exists.

# Portable task CPU scaling benchmark

Run the deterministic CPU reference task at the v0.4 batch-width checkpoints:

```text
cargo run --locked -p xtask -- task-scale
```

The default report is `artifacts/task-scale/report.json`. It records the full
TaskSpec, backend, precision, operating system, architecture, CPU identity,
logical CPU count, 32 warm-up steps, 256 measured steps, and samples for 1, 16,
256, and 4096 lanes.

This is an outer-harness throughput measurement. Wall-clock time never enters
the episode or runner, and no timing value enters a correctness digest. Every
batch width executes the same lane-zero episode seed and action sequence; report
generation fails if its observation/reward/termination replay digest differs
from the single-environment run.

Use `--warmup-steps`, `--measured-steps`, or `--json` to run an exploratory
measurement. Comparisons must retain the report's task, backend, precision,
hardware, warm-up, measured-step, and batch-size metadata. The benchmark is a
portable-runner baseline, not a claim that the CPU reference task matches a
GPU physics workload.

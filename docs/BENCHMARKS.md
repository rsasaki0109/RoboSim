# Deterministic benchmark reports

The benchmark aggregator combines the existing physics-conformance and
scenario-scale reports without reimplementing either producer:

```text
cargo run --locked -p xtask -- benchmark
```

Missing producer reports are generated automatically. Use `--no-generate` to
require reports that already exist, or pass `--physics-report`,
`--scenario-report`, and `--output` to select explicit paths. The stable report
is written to `artifacts/benchmarks/report.json` by default.

The schema-v1 artifact has kind `rne_benchmark_report`. Its cases and evidence
are sorted, evidence files are recorded with repository-relative paths and
SHA-256 digests, and `content_digest` is the SHA-256 of canonical JSON with
the digest field itself removed. Each case records the exact nanosecond
`fixed_delta_ticks` used by `SimDuration`, plus its derived explicit-unit
`fixed_delta_s` projection (`simulation_hz` uses rounded-rate integer
division; seconds use nearest-nanosecond rounding). It contains no timings,
host metadata, timestamps, stdout, or absolute paths. Each case embeds the
applicable `rne_core::DeterminismContract`.

Timing data is deliberately separate and optional:

```text
cargo run --locked -p xtask -- benchmark --timings
```

This writes `artifacts/benchmarks/timings.json`; use `--timings-output` for a
different location. Timing and host fields in that artifact are volatile and
must not be used to compare stable benchmark report bytes.

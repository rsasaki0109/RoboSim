# Capability report

RNE's advertised workflow surface is available as a deterministic, machine-readable
report. Generate it from the repository root with:

```bash
cargo run --locked -p xtask -- capability-report
```

The default output is `artifacts/capability-report/report.json`. A different path
can be selected with `--output PATH` (the `--json PATH` spelling is retained as an
alias for consistency with other `xtask` reports).

The complete release evidence path is available through `cargo run --locked -p
xtask -- evidence`; see [EVIDENCE_QUICKSTART.md](EVIDENCE_QUICKSTART.md).

The schema-v1 report uses the stable top-level discriminator
`kind = "rne_capability_report"`, records the RNE release version and exact Git
commit, and emits the 13 capabilities in the same explicit order as
`docs/OSS_PARITY_MATRIX.md`. Every capability includes its current status
(`complete`, `parity`, or `partial`) and one or more committed evidence paths paired
with the contributor-facing command that exercises the path. The command validates
that every path is a repository file and is tracked by Git before writing the report;
schema-v1 deserialization rejects unknown fields.

The report deliberately omits timestamps, host paths, and command output. It is
therefore suitable for release metadata, CI catalogues, and downstream tooling;
runtime test results remain the responsibility of the referenced commands and the
dedicated `xtask parity` report.

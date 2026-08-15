# Compatibility fixture corpus

RNE retains a small executable corpus of public release artifacts. Its purpose
is to catch accidental reader breakage before release and to let users rerun
the same check from an extracted native bundle.

Run it from a source checkout:

```bash
cargo run --locked -p rne_compatibility_suite -- \
  --output artifacts/compatibility/report.json
```

Run the installed form from the bundle root:

```bash
./bin/rne-compatibility --root . \
  --output compatibility-fixture-report.json
```

The strict registry is `release/compatibility-fixtures.toml`. Schema v1
contains nine fixtures:

| Contract | Retained artifact |
|---|---|
| Task contract | TaskSpec v1 |
| Batch execution | portable batch checkpoint v2 |
| Replay | generic replay v1 |
| Dataset | bundle manifest v1 and depth evaluation v1 |
| Failure evidence | Failure Capsule v1 |
| Hardware safety | process-mock conformance v1 |
| Physics | built-in conformance v2 and external-backend conformance v1 |

For every entry the runner:

1. resolves a canonical relative path without following it outside the corpus;
2. bounds the file at 4 MiB and requires a regular file;
3. verifies the SHA-256 of canonical compact JSON;
4. deserializes and semantically validates it with the current typed reader;
5. changes the version to a deterministic future value and requires rejection;
6. adds an unknown top-level field and requires rejection.

The report has no timestamps, host paths, random IDs, or timing data. The same
registry and fixture content therefore produce identical report bytes on Linux
and Windows. Canonical JSON hashing deliberately ignores indentation and line
endings while preserving every parsed field and value.

## Change policy

Append a fixture when a candidate-stable artifact becomes release-facing or
when an older supported artifact needs explicit retention. Update its typed
dispatch, tests, golden report, contract registry, compatibility documentation,
and release bundle together. Do not silently replace an older fixture with a
newer shape; keep both while the older version is supported.

The current corpus is the first v0.9 slice, not a complete 1.0 declaration. It
does not yet cover every frontend frame, C ABI binary, scenario replay, dataset
payload, Python call shape, or migration outcome. Those surfaces must gain
fixtures before their candidate freeze, and the independent-use and six-month
stability gates remain mandatory.

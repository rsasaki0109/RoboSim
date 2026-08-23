# ADR 027: Archive-bound time-to-proof evidence

## Status

Accepted.

## Context

Deterministic correctness reports must remain byte-stable, while the product
goal requires a measured time from invoking an extracted release command to a
verified mobile-manipulation Failure Capsule. A CI duration alone does not name
the measured hardware or bind the result to the generated proof. A standalone
timing JSON could also be copied beside an unrelated release archive.

## Decision

The installed `rne-flagship-proof` command accepts `--measure-on MACHINE` and
emits `rne_time_to_proof_report` schema v1 outside the deterministic proof
index. The report records the operator-supplied machine label, detected OS and
architecture, elapsed milliseconds from process start through the verified
capsule and SHA-256-bound installed proof report, and the fixed 900,000 ms
acceptance target. It binds both proof roots by relative path, size, and
SHA-256.

Archive-install rehearsal invokes this measured path and independently checks
its identity, threshold, and bound files. `rne_archive_install_rehearsal`
schema v2 adds the timing-report digest alongside the archive, release report,
checksum manifest, and schema-v6 installed verdicts. Tagged release provenance
therefore signs a subject that binds the exact archive and measurement. Release
workflows retain the complete proof directory.

CI machine labels are packaging diagnostics, not independent adoption evidence.
The external gate requires the same report from a user outside the repository
on specifically named reference hardware.

## Consequences

- Wall-clock measurements never enter deterministic correctness hashes.
- A changed proof, capsule, timing report, or archive breaks the signed evidence
  chain.
- Archive-install schema v1 remains historical evidence but cannot satisfy the
  current time-to-proof gate.
- Trust in a submitted machine label remains a provenance and review question;
  the schema makes the claim explicit and content-addressed rather than trying
  to infer external independence.

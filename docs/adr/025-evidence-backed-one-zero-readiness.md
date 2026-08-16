# ADR 025: Evidence-backed 1.0 readiness

## Status

Accepted.

## Context

RNE has machine-readable release, conformance, compatibility, Failure Capsule,
and physical-hardware reports, but the final 1.0 conditions previously existed
only as prose. Individual in-repository demonstrations cannot prove external
adoption, a six-month stability interval, an independent implementation, or a
maintainable long-term support promise. A star count is even less direct.

Using the current date inside the audit would also violate reproducibility: the
same evidence could produce different output on two days without any input
change.

## Decision

Add `xtask release-readiness` and a strict schema-v1 TOML tracker. The command
requires an explicit `--as-of YYYY-MM-DD`, verifies typed source reports and
their exact SHA-256 digests, and emits a fixed-order schema-v1 JSON report.
Unmet facts are `not_met`; malformed or tampered supplied evidence is an error.

The candidate surface is the immutable Rust public-API baseline commit and Git
tree. Stability requires both 183 calendar days and a 183-day aggregate span
across at least two independently owned external projects, with no unplanned
break declared. Linux and Windows release evidence must reproduce one retained
tagged revision. The compatibility report must validate against the exact
current fixture registry. Physical evidence is delegated to the full LeKiwi
verifier rather than a checklist-shaped surrogate.

`release-check` validates the committed tracker and contract registration, but
normal 0.x development does not require unavailable external evidence. The
explicit `--require-eligible` mode is reserved for promotion and fails unless
all nine checks pass.

## Consequences

- RNE can report incremental progress without claiming 1.0 readiness.
- External evidence can be kept as a content-addressed pack outside the source
  checkout, including on removable storage.
- No tag, release, support promise, or adoption claim is created by the tool.
- Evidence authenticity and organizational independence still require human
  review; the tool verifies identities, structure, provenance, and bytes.
- Changes to the readiness report require a registered schema transition and
  golden update.

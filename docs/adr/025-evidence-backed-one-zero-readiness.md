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

Add `xtask release-readiness` and a strict versioned TOML tracker. The command
requires an explicit `--as-of YYYY-MM-DD`, verifies typed source reports and
their exact SHA-256 digests, and emits a fixed-order schema-v1 JSON report.
Unmet facts are `not_met`; malformed or tampered supplied evidence is an error.

The candidate surface is the immutable Rust public-API baseline commit and Git
tree. Stability requires both 183 calendar days and a 183-day aggregate span
across at least two independently owned external projects, with no unplanned
break declared. Linux and Windows release evidence must reproduce one retained
tagged revision. The compatibility report must validate against the exact
current fixture registry, equal a fresh execution of every current typed
reader, and retain source revisions, trees, declarations, and golden blobs
that still verify through Git history. A report whose passing fields were only
edited into place is therefore insufficient. Physical evidence is delegated
to the full LeKiwi verifier rather than a checklist-shaped surrogate.

`release-check` validates the committed tracker and contract registration, but
normal 0.x development does not require unavailable external evidence. The
explicit `--require-eligible` mode is reserved for promotion and fails unless
all nine checks pass.

Release commands also contain a version-triggered interlock. Any 1.x or later
`release-check`, `release-bundle`, or `release-exit` requires an external
manifest path and explicit assessment date from the environment, reruns the
typed audit, writes a promotion report, and stops unless it is eligible. This
guards the source, platform-package, and aggregate-publication paths instead of
assuming one of them called another.

Tracker schema v2 makes every external identity immutable by requiring its
40-character source revision and rejects report-only certification. Controller
reports are rebound to retained library and manifest bytes. Physics reports are
rebound to the exact implementation artifact or source bundle. Hardware reports
are rebound to the adapter, TaskSpec, normalized launch arguments, negotiated
task identity, and flattened widths. These checks reuse the report's own
content-addressed subject fields instead of trusting its aggregate verdict.

Tracker schema v3 closes the platform-package side of the same substitution
problem. Every platform retains the archive, extracted release report,
SHA256SUMS, and a separate archive-install wrapper. The wrapper fixes their
byte identities and the independent nine-check result. Both the archive and
wrapper are `actions/attest@v4` subjects and each receives a fresh strict
verification receipt during readiness evaluation. The gate reconstructs the
checksum member graph instead of trusting either report's aggregate status.

## Consequences

- RNE can report incremental progress without claiming 1.0 readiness.
- External evidence can be kept as a content-addressed pack outside the source
  checkout, including on removable storage.
- No tag, release, support promise, or adoption claim is created by the tool.
- Changing package metadata to 1.x cannot bypass the external-evidence audit.
- Evidence authenticity and organizational independence still require human
  review; the tool verifies identities, structure, provenance, and bytes.
- A passing conformance JSON cannot be moved onto different plugin, backend, or
  adapter bytes inside the readiness pack.
- A compatibility JSON cannot substitute for executing the registered corpus;
  promotion replays all readers and requires an exact report match.
- The committed tracker may count historical compatibility because its
  retained report is rehashed, source-provenance checked, and reproduced by a
  fresh 24-reader replay; this does not imply any external adoption claim.
- A release or install report from another build cannot be paired with a
  signed archive; the signed wrapper and checksum graph bind all three.
- Changes to the readiness report require a registered schema transition and
  golden update.

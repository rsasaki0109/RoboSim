# ADR 019: Historical migration fixtures

- Status: Accepted
- Date: 2026-08-15

## Context

Current-schema golden files prove that readers and writers agree today, but do
not prove that a supported old artifact still restores to the same present
state. Unit tests for mobile-manipulator snapshot v1 changed a current in-memory
value rather than reading the JSON shape that users actually retained.

## Decision

Retain strict migration-case JSON in the installed compatibility corpus. The
first case stores a complete mobile-manipulator snapshot v1 without the wrist
depth field introduced in v2 or the grasp-retarget field introduced in v3. The
current runtime must restore it, reject unknown fields and schema v4, normalize
it to snapshot v3, and match every serialized state value within `1e-9` before
matching the registered state digest.

Expose the current and minimum supported snapshot schema constants through
`rne_ai` and register them in `release/contracts.toml`. Bundle the exact scene,
robot manifest, and URDF needed by the migration case so extracted release
tests do not depend on source-checkout paths.

## Consequences

- Compatibility evidence now covers an actual old-to-current state transition.
- Floating-point reconstruction is explicit and bounded instead of being
  mislabeled as byte-lossless.
- Future migration cases must retain their source artifact, expected outcome,
  tolerance where applicable, and current normalized-state digest.
- One passing migration does not imply that every historical format is covered;
  broader history and the long stability window remain separate gates.

ADR 021 extends this mechanism with provenance-bound, nonzero v1 and v2 source
artifacts while retaining the original case.

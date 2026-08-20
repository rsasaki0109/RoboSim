# ADR 021: Provenance-bound historical migration matrix

- Status: Accepted
- Date: 2026-08-16

## Context

ADR 019 introduced one executable snapshot-v1 to snapshot-v3 migration case,
but its source payload was synthesized from a current zero-step snapshot by
removing newer fields. That proved the reader mechanism, not that a nontrivial
artifact emitted by historical code still restores. It also skipped the
intermediate v2 schema that introduced wrist depth state.

## Decision

Retain the original case and append two strict schema-v2 migration fixtures.
Their complete source snapshots are emitted after seven fixed simulation ticks
by ancestor revisions `47525b127a77cbffa9da27b1e0c127ee673aa641`
(snapshot schema v1) and `2255cbefec9d1eb5040603fbb119a290ad855191`
(snapshot schema v2). Each fixture records the exact source commit, Git tree,
workspace version, scene, generation step count, canonical source digest, and
tolerance-normalized current snapshot digest.

The v1 source must contain nonzero joint-state and RGB sensor frames while
omitting depth and grasp-retarget fields. The v2 source must additionally
contain a populated wrist-depth frame and must still omit grasp-retarget state.
The current runtime restores each source as schema v3, compares every retained
serialized value at the existing `1e-9` tolerance, and rejects future schemas,
unknown fields, provenance retargeting, or digest drift.

Installed bundles carry both complete migration fixtures and the scene assets
needed to execute them without Git. Source `release-check` additionally uses a
full repository history to verify that both source commits remain ancestors of
`HEAD`, their trees match, their source declares the registered snapshot
schema, and the scene exists at that revision. The release-contract CI checkout
therefore uses `fetch-depth: 0`.

## Consequences

- Migration evidence now spans the actual v1 and v2 serializer shapes and
  exercises timestamped sensor state after simulation has advanced.
- A rewritten or pruned source commit, retargeted provenance record, lost v2
  depth frame, or changed restored state fails the release gate.
- The installed corpus grows from fifteen to seventeen fixtures while keeping
  the original case for audit continuity.
- These cases cover one evolving state family. They do not prove every replay,
  dataset, protocol, TaskSpec, or Failure Capsule migration, external adoption,
  or the six-month stability requirement.

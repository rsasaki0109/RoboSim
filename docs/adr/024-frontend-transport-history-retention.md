# ADR 024: Frontend transport history retention

- Status: Accepted
- Date: 2026-08-16

## Context

The installed compatibility corpus decoded a current protocol-v1
`ClientHello`, but it did not bind those bytes to the protocol's real Git
history. A regenerated current fixture could therefore continue to pass after
an accidental same-version wire change. This left frontend transport behind
the provenance guarantees already applied to TaskSpec, dataset, replay,
checkpoint, snapshot, and Failure Capsule artifacts.

Protocol v1 was introduced by commit
`1a38391362ece24cc73c0e1470a51bd7f933e6fc`, tree
`3117bd4949f19c36a1fba66524b97bd4bd1af3d4`. Later commits added the bounded
streaming runner and hardened payload and negotiation limits without changing
the frame encoding. The first committed full `ClientHello` golden was added by
commit `be53f16347beb7df822850748d0e01ce41d227a0`, tree
`78a68abd73fb4564793559d8e75e021ad5090129`.

## Decision

Retain `tests/golden/protocol/frontend-transport-v1.json` from `be53f16` in a
schema-v1 historical-decision wrapper. The source golden has Git blob
`2eecf4edc03fa10c78dc950453f7adcde70bdb6a` and canonical digest
`sha256:998640e79945057bb755182009c397f3537996be583ec9f74c048fd1c6dcda71`;
the current golden must remain the same blob.

The installed verifier must:

- decode and re-encode the complete frame and `ClientHello` payload exactly;
- reproduce the recorded negotiated capabilities and byte/frame limits;
- reject corrupt magic, unknown message kinds, truncation, trailing bytes,
  and an incompatible protocol major;
- reject a future fixture schema and unknown top-level fields; and
- bind the decision to the source revision, tree, workspace version, artifact
  digest, and `same_schema_frontend_transport` reason.

Source `release-check` separately verifies that the protocol introduction and
golden revisions remain ancestors of `HEAD` with their exact trees. It checks
the introduction's protocol-major/minor declarations and platform-independent
header vector, then verifies the source and current full goldens resolve to the
recorded blob. Extracted bundles need no Git database because they carry the
content-addressed wrapper and compiled identities.

## Consequences

- The installed corpus grows from twenty-three to twenty-four fixtures.
- A same-version wire change can no longer be hidden by regenerating the
  current frontend golden.
- Payload-limit and queue hardening remain compatible implementation changes;
  they do not justify a protocol-major bump.
- This case retains one negotiated `ClientHello`. Additional message families,
  any future protocol v2 transition, and independent external frontend use
  remain separate 1.0 gates.

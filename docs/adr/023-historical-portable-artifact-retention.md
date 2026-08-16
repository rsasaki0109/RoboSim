# ADR 023: Historical portable artifact retention

- Status: Accepted
- Date: 2026-08-16

## Context

The compatibility corpus retained current TaskSpec, dataset, and Failure
Capsule v1 goldens, but that proved only that today's readers accepted today's
fixtures. All three contracts still use their original schema number. Without
source provenance, a fixture could be silently replaced while continuing to
claim v1 compatibility.

Dataset evidence had an additional gap. The retained manifest described a
streaming shard, but the installed compatibility runner did not possess or
read the historical shard. Manifest-only acceptance cannot prove binary record
framing, payload hashes, explicit gaps, timestamp ordering, or headless offline
evaluation.

## Decision

Extend schema-v1 `rne_historical_compatibility_decision` with optional embedded
source files and an expected result digest. Existing checkpoint and replay
decisions omit both fields and retain their canonical hashes.

Add three same-schema retention cases:

- TaskSpec v1 from commit
  `70a9ff35afbf0215803dd288103bdda79fa46891`, tree
  `94459bcb0c5090921bf6edbcf6f63246ebdd6a40`;
- dataset bundle v1 from commit
  `aecafb62c99f432b2a76956575f4562c6047a6bc`, tree
  `0bc9d2d48185282da31dc80eb8857d84012a5928`;
- Failure Capsule v1 from commit
  `61d6c813e79d7eac6a8ab212776d620069f98905`, tree
  `5dac12166fe39da5a1207426f3e7520851e415d2`.

The JSON sources are the exact golden blobs committed by those revisions.
TaskSpec and Failure Capsule must deserialize, validate, and serialize to the
same semantic JSON. Their future-schema and unknown-field mutations must fail.

The dataset case additionally embeds the exact 736-byte `records.rnedata`
emitted by the ancestor writer. The installed runner reconstructs a temporary
bundle and requires two streams, six records, four samples, two explicit drops,
the original shard and manifest digests, and the exact retained depth-pair
evaluation digest. It then flips one shard bit and requires streaming
verification to fail.

Source `release-check` verifies every revision remains an ancestor of `HEAD`,
matches its recorded tree and workspace version, declares schema v1, and
contains the original golden. For the dataset it also verifies the historical
integration-test generation recipe. Extracted bundles use the embedded bytes
and compiled identities without requiring Git.

## Consequences

- The installed corpus grows from twenty to twenty-three fixtures.
- Same-schema compatibility is now tied to the introducing implementation,
  not merely to a current fixture with the same version number.
- Dataset history covers its binary artifact and offline behavior rather than
  only manifest deserialization.
- These cases do not invent v2 migrations. If any family gains a new public
  schema, it must add a lossless migration or an explicit required-rerun
  decision using real old artifacts.
- Broader frontend protocol history and external six-month adoption remain
  independent 1.0 gates.

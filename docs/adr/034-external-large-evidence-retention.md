# ADR 034: External retention for large docs/evidence traces

## Status

Accepted.

## Context

`docs/evidence` holds raw R&D trace artifacts for the openarm and G1
contact-backflip campaigns: roughly 465 MB of the repository's roughly 750 MB
tracked tree, spread across 579 raw JSON rollout/trace files. Individual
success-trace files run 10-14 MB each. None of this is generated at build or
test time; it is a permanent historical record checked in as evidence for
past findings (see the openarm-*-lab and g1-contact-backflip directories).

Every clone, fetch, and CI checkout pays for this weight even though almost
none of it is read by the build, by `xtask ci`/`ci-headless`, or by any
committed report reader — it exists purely as durable evidence a human or a
future audit can go back and inspect. Left unconstrained, the campaigns this
evidence supports keep growing and each one adds more multi-megabyte raw
traces, so the tracked tree's size is unbounded and grows independently of
the engine's own source and test surface.

Deleting or rewriting the already-tracked history is out of scope for this
ADR: the existing files remain exactly as committed, and this decision does
not touch, move, or re-host any of them. The problem is purely about what
happens to *new* raw evidence from here on.

## Decision

Going forward, a file added under `docs/evidence` that exceeds 1 MiB
(`MAX_INLINE_EVIDENCE_BYTES`, 1,048,576 bytes) must not be committed inline.
Instead it is stored externally (for example, as an asset attached to a
GitHub Release) and represented in Git by a small `<name>.pointer.json`
sidecar with a fixed schema:

```json
{
  "kind": "rne_docs_evidence_pointer",
  "schema_version": 1,
  "original_filename": "dropout-004frames-rapier-success-trace.json",
  "sha256": "sha256:<64 lowercase hex characters>",
  "size_bytes": 14563210,
  "url": "https://github.com/rsasaki0109/RoboSim/releases/download/<tag>/<asset>"
}
```

The pointer's `sha256` and `size_bytes` describe the externally stored file,
not the pointer file itself. `url` must be an `https://` retrieval location.
Verification is: fetch `url`, hash the downloaded bytes with SHA-256, and
check the digest and byte length against the pointer's `sha256` and
`size_bytes`. A mismatch means the external asset is not the evidence the
pointer claims it is.

Evidence already tracked before this ADR is exempt: `xtask` ships a
historical allowlist registry, `release/docs-evidence-retention.toml`
(schema version 1), generated once from the current tree as a list of
`{path, sha256, size_bytes}` entries for every `docs/evidence` file already
above the threshold (61 entries at the time of writing). A file at or below
1 MiB never needs an allowlist entry, regardless of when it was added.

Enforcement is `xtask`'s `docs_evidence_retention` check
(`cargo run -p xtask -- docs-evidence-check`), wired into
`lint-boundaries` and therefore into `ci-lint` / `ci`. For every file under
`docs/evidence` it exceeds the threshold, the check requires either:

- a `*.pointer.json` sidecar with a valid schema (checked structurally, not
  by fetching the network), or
- an exact, digest-matching entry in the historical allowlist registry.

The check fails closed: a new large raw file with neither a pointer nor a
matching historical entry breaks the build, as does an allowlisted file whose
bytes no longer match its recorded digest (historical evidence must not be
modified in place) or a registry entry that no longer has a file on disk
(the registry must track the tree exactly). The registry itself is validated
for a clean schema version, forward-slash paths confined to `docs/evidence`,
canonical `sha256:`-prefixed digests, and no duplicate or sub-threshold
entries.

`.gitattributes` marks `docs/evidence/**` as `-diff linguist-generated=true`
so GitHub collapses diffs against these files by default and excludes them
from language statistics; this is presentation only and changes no stored
bytes.

## Consequences

- New raw evidence traces stop growing the tracked tree; only a pointer's
  few hundred bytes are committed, no matter how large the underlying trace.
- Historical evidence committed before this ADR keeps working exactly as
  before: nothing was moved, deleted, or re-hashed against a different
  location, and the allowlist snapshot exists purely to let CI's threshold
  check keep passing.
- Reviewers and CI can no longer see the raw bytes of newly added large
  evidence inline in a diff; verifying it requires fetching the external
  asset and rehashing, which the pointer schema makes mechanical.
- The size threshold is a repository-wide policy for `docs/evidence` only;
  extending it to other directories, or lowering it, is a future ADR.
- A contributor who commits a large raw file directly (no pointer, no
  allowlist entry) gets a clear, fail-closed CI error naming this ADR rather
  than a silent, unbounded tree-size increase.

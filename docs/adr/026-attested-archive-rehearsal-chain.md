# ADR 026: Attested archive-install rehearsal chain

## Status

Accepted.

## Context

The native archive had signed provenance, while `release-report.json` and the
post-extraction install rehearsal were supplied to the 1.0 gate as separate
files. Their own SHA-256 references prevented unnoticed mutation, but did not
prove that they described the signed archive beside them. A valid passing
report from another build could be substituted without breaking the archive's
attestation.

## Decision

Keep installed-rehearsal schema v4 inside the archive. After deterministic
archive creation and fresh extraction, `release-install-smoke` must also receive
the archive path and emit `rne_archive_install_rehearsal` schema v1. The outer
report binds:

- the canonical platform archive file name, size, and SHA-256;
- the extracted bundle root;
- exact `release-report.json` and `SHA256SUMS` identities; and
- the independently rerun ordered nine-check schema-v4 result.

The verifier reconstructs `SHA256SUMS` from the release report's sorted member
list plus `release-report.json`, requires the inner and outer rehearsal bytes
and verdicts to agree, and rejects noncanonical paths or digests. Release jobs
attest the outer report alongside the archive and wheel. Readiness manifest v3
retains separate strict receipts and freshly runs `gh attestation verify` for
both the archive and outer report against the same workflow, tag, and commit.

## Consequences

- An archive/report swap changes either a signed subject digest or the bound
  checksum graph and fails closed.
- The cycle-free inner schema-v4 report remains compatible; archive identity
  lives only in the post-archive wrapper.
- Each platform readiness entry retains seven files rather than five.
- The wrapper attests successful execution by the trusted release workflow; it
  does not replace independent review of GitHub, Sigstore, or runner trust.

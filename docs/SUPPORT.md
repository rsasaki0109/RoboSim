# Support policy

## Current status

RNE is a pre-1.0 project. Maintainers review bug reports and patches on a
best-effort basis, but the 0.x releases do not promise a response time,
remediation time, or maintenance period. Supported interfaces and migration
rules are defined by the [compatibility policy](COMPATIBILITY.md); CI and
release-rehearsal platforms describe tested configurations, not a support
service commitment.

Security reports should use GitHub's private vulnerability reporting for the
repository. Do not include undisclosed vulnerability details in a public
issue. P0 and P1 release blockers remain visible in `release/blockers.toml`.

## Required 1.0 commitment

RNE cannot be promoted to 1.0 until a maintainer with authority to provide the
commitment publishes all of the following:

- the maintainer or maintenance group responsible for the commitment;
- an unambiguous support period;
- a public HTTPS policy describing the covered stable surfaces, supported
  release lines and platforms, security-reporting route, compatibility and
  deprecation rules, and any exclusions.

The published policy must be consistent with `docs/COMPATIBILITY.md` and the
1.0 candidate's verified release artifacts. A project issue, roadmap entry,
draft text, CI result, or contributor intention is not a support commitment.

The evidence tracker records this decision in the `[support]` table of
`release/one-zero-readiness.toml`. Until the decision is authorized and the
HTTPS policy is published, `committed` stays `false` and the maintainer,
support-period, and policy fields stay empty. The readiness audit rejects a
partially populated uncommitted table and rejects an incomplete committed
table. This document therefore clarifies the gate without claiming that its
final commitment already exists.

## Reporting and compatibility

Public defects and feature requests belong in the repository's issue tracker.
Security-sensitive reports use private vulnerability reporting. Artifact or
API compatibility questions should include the RNE version, platform, relevant
schema or ABI version, and a minimal reproducible input where disclosure is
safe.

Compatibility promises, historical-reader behavior, and migration procedures
remain authoritative in `docs/COMPATIBILITY.md`. The evidence required to
promote a candidate is defined separately in `docs/ONE_ZERO_READINESS.md`.

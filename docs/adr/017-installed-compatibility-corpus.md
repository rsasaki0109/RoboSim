# ADR 017: Installed compatibility fixture corpus

- Status: Accepted
- Date: 2026-08-15

## Context

RNE had many crate-local golden tests and a machine-readable list of schema
versions, but no release artifact proved that the installed readers still
accepted a retained cross-domain history. A source-only test also could not
show that the shipped binary and bundle contained the promised fixtures.

## Decision

Keep a strict content-addressed registry under `release/` and implement its
runner in the downstream, non-publishable `rne_compatibility_suite` package.
Ship the `rne-compatibility` binary and the exact registered files in native
bundles. Both release CI and post-extraction rehearsal must run it.

Each JSON fixture is hashed after canonical parsing so evidence is independent
of indentation and platform line endings. The current typed reader must accept
the original value and reject deterministic future-schema and unknown-field
mutations. Reports contain stable relative paths and no host or timing data.

No core, simulation, adapter, or public authoring crate may depend on the
compatibility suite. It is allowed to aggregate those crates because it sits at
the release/test boundary.

## Consequences

- Accidental reader drift fails one centralized release gate and names the
  exact contract that changed.
- Extracted bundles can prove compatibility without the source checkout.
- Retaining a supported old schema consumes explicit fixture and test cost.
- The corpus proves only its listed artifacts; external adoption and the full
  candidate-surface inventory remain separate 1.0 gates.

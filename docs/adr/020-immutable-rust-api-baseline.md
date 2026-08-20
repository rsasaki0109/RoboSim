# ADR 020: Immutable Rust API baseline

- Status: Accepted
- Date: 2026-08-16

## Context

The SemVer job covered every publishable Rust crate, but selected
`origin/<base>` for pull requests and `HEAD^` for pushes. That catches a break
once, yet the comparison point moves after merge and no longer proves that the
candidate surface retained an older accepted API.

## Decision

Commit `release/rust-api-baseline.toml` with schema version 1. It records the
exact baseline commit, its Git tree, cargo-semver-checks version 0.49.0, and the
manifest path of all 31 publishable crates. Release checks require exact
coverage and ordering, match every path to current Cargo metadata, verify each
manifest exists in the baseline commit, verify the commit/tree pair, and
require the baseline to remain an ancestor of `HEAD`.

CI reads this registry directly and applies patch compatibility rules to every
package shard. There is no missing-package bootstrap or fallback to a moving
branch parent. The registry is included in native bundles as audit metadata;
the source comparison remains a clean-checkout CI gate.

Patch releases cannot retarget the baseline. A pre-1.0 retarget requires a
minor release, migration notes, a new ADR, and evidence that the candidate
still passed against the previous baseline before changing the registry.
After the initial bootstrap, CI rejects registry changes relative to the
pull-request base or push parent while the release remains 0.1.0. History must
preserve the baseline commit; merge strategies that discard it are
incompatible with this gate.

## Consequences

- A breaking change cannot become invisible merely because the next CI run
  starts from a newer parent.
- Package additions, removals, and path moves require an explicit baseline
  decision instead of being silently skipped.
- This starts the Rust stability clock but does not prove six months of use or
  external adoption.

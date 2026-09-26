# ADR 035: Retarget the Rust API baseline for 0.4.0

- Status: Accepted
- Date: 2026-09-26
- Supersedes the registry split in [ADR 033](033-additive-package-api-baselines.md)

## Context

[ADR 020](020-immutable-rust-api-baseline.md) froze the Rust public API at
`release/rust-api-baseline.toml`, and 0.3.0 pinned it to main commit
`d0139576c232fed66d2f0b97ea7b01b0773b83b8` (2026-09-15). Every CI shard
compares the working tree against that revision under patch rules.

The gate has been failing since around PR #290 and was failing on every pull
request merged after it, including #325, #326, #327 and #328. Nothing enforced
it: the aggregate `workspace` job reports `scope ci has a non-success required
job`, but that is a report, not a block, so each further breaking change landed
into an already-red signal and was indistinguishable from the previous ones.

The largest single contribution is #318 `chore: remove confirmed-dead pub API`,
which deleted or narrowed items verified to have no callers anywhere in the
repository. That PR deferred the retarget deliberately — its changelog entry
says to "retarget them at the next minor release the way PR #275 did, using
this entry for the removed-item list" — so this ADR completes work #318 planned
rather than repairing a mistake it made. What went wrong is the interval: the
gate was already red before #318 and stayed red for eight more merges
afterwards, so anything else that broke the frozen surface in between arrived
without a distinguishable signal.

Measured against the 0.3.0 baseline, 132 commits accumulated **94 breaking
changes** across 17 crates:

| Class | Count |
| --- | ---: |
| `pub` item removed from the public surface | 80 |
| ⤷ demoted to `pub(crate)`, still present | 39 |
| ⤷ deleted outright | 41 |
| `pub` field added to a struct-literal-constructible struct | 10 |
| type stopped deriving `Copy` | 3 |
| associated const became `#[doc(hidden)]` | 1 |

The full item list is in [COMPATIBILITY.md](../COMPATIBILITY.md#0-4-0-rust-api-baseline-retarget).

**Restoring the previous surface is not available.** The three lost `Copy`
derives are `Collider`, `ColliderShape` and `DeformableCollider`, and
`ColliderShape` lost `Copy` because it gained `HeightField`, `TriMesh`,
`ConvexHull` and `Compound` variants carrying owned geometry. Restoring `Copy`
means deleting heightfield and mesh collision support. The ten added fields are
likewise the four-wheel vehicle dynamics, load transfer, traffic car-following
and URDF fixed-joint welding features. A revert-everything path would undo
shipped, wanted capability to satisfy a version number.

## Decision

Treat 0.4.0 as the breaking pre-1.0 minor that absorbs these changes, exactly
as 0.3.0 absorbed the changes that accumulated after the 0.2.0 freeze.

- Bump every workspace package, exact internal dependency requirement, release
  registry, xtask release constant, bundle identity, Python API contract
  version, evidence-campaign template and installation document from 0.3.0 to
  0.4.0.
- Retarget `release/rust-api-baseline.toml` to
  `aa4aa7b46bcb3486042b346be6dfdd14b7cbcf6a` / tree
  `41e0c9db2aee1bc65c92804a3fd022984d8f1629`, the tip of main at bump time, and
  move the 1.0 readiness candidate to the same commit/tree pair.
- **Fold `release/rust-api-additions-v1.toml` back into the single registry and
  delete it.** ADR 033 created it because `rne_collision_bake` and `rne_usd` did
  not exist at the 0.3.0 baseline; at the 0.4.0 baseline they do, so its
  precondition ("an addition must not have existed at the original baseline") is
  no longer satisfiable and its validation now fails by construction. One
  registry again covers all 36 publishable packages, and the CI shard no longer
  selects a different registry for one shard.
- **Do not merge while `semver` or `workspace` is red.** A retarget clears
  accumulated debt exactly once; it is not a way to keep clearing it. A breaking
  change to a frozen API is a decision that needs a minor bump and an ADR, and
  it can only be recognised as one if the gate is green before it lands. If a
  pull request turns `semver` red, either the change is wrong or it needs its
  own baseline decision — merging it and moving on is neither.

## Consequences

- The gate is green again, so the next breaking change is visible as a change
  rather than as one more line in an existing failure.
- 0.4.0 is not source-compatible with 0.3.0 for the 94 listed items. Pre-1.0
  minors may break; this one does.
- ADR 020's stated precondition for a retarget — "evidence that the candidate
  still passed against the previous baseline" — cannot be met here, because the
  breakage is already in main. That clause is written for a retarget made
  *before* a break lands. This retarget is made after, which is precisely the
  situation the no-merge-while-red rule above exists to prevent recurring.
- This changes no numerical physics gate, no artifact schema and no runtime
  behaviour. It is a versioning and registry decision.
- Nothing here establishes external adoption or a support commitment; the
  1.0 readiness manifest still records `support.committed = false`.

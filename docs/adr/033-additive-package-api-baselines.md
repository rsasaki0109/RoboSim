# ADR 033: Additive API baselines for newly introduced packages

- Status: Accepted
- Date: 2026-09-22

## Context

Main commit `cd6848d555ab74d7735908887126f5e366bebc88` introduced publishable
`rne_collision_bake` and `rne_usd` without adding them to the release inventory.
The required release gate rejects this mismatch. Neither manifest exists at the
original frozen API baseline. Retargeting the entire registry would erase the
compatibility history of existing crates.

## Decision

Keep `release/rust-api-baseline.toml` byte-for-byte unchanged. Register the two
new packages in `release/rust-api-additions-v1.toml`, using the same version-1
registry format, cargo-semver-checks version, explicit manifest paths and exact
commit/tree pair. Their first baseline is the introducing main commit above,
with tree `bc439dab3ce492f15788485aea9df8cbedc61f96`.

This is the explicit new-package baseline decision required by ADR 020. It is
not a retarget of any existing API or a waiver of its failures. The release
version remains 0.3.0. Subsequent edits to either registry are subject to the
same within-release immutability guard. Future additions require another
explicit baseline decision; there is no automatic missing-package bootstrap.

Release validation requires exact, disjoint coverage of the public package
inventory, matching current manifest paths, available ancestral commits and
matching trees. An addition must not have existed at the original baseline.
The SemVer matrix selects the additional registry only for the two new packages;
all previous shards retain their original comparison. Both registries are
included in native release bundles.

## Validation and limitations

Regression tests reject missing entries, existing-package replacements, wrong
trees, missing historical manifests and resetting an already-existing package.
The CI workflow contract checks both registry paths and exact package coverage.
This registration does not resolve previously reported breaking fields in other
crates, change numerical physics gates or establish external adoption/readiness.

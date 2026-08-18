# Release rehearsal

`.github/workflows/release.yml` builds and independently rehearses one native
bundle on Linux x86-64 and Windows x86-64. Each job builds the pinned ABI3
Python wheel, invokes `xtask release-bundle`, creates a deterministic archive,
extracts it into a fresh directory, and invokes `release-install-smoke` against
installed artifacts only while naming the exact source archive.

The bundle contains the CLI, standalone physics, hardware-adapter, accelerator,
compatibility, and scenario-scale conformance binaries, the fixed-binding
hardware and accelerator process mocks,
the reference controller shared library, dependency-free Rust and C plugin SDKs,
plugin/physics/hardware/accelerator external-conformance guides, compatibility and install
documentation, locked dependency SBOM,
artifact-attestation policy, Rust API baseline, Python API manifest, locked
dependency graph, Failure Capsule authoring guides, replay fixtures, provenance
report, and `SHA256SUMS`. Installed-rehearsal schema v5 runs ten frozen checks:
robot replay, scenario replay, physics conformance, external hardware-adapter
conformance, accelerator protocol conformance, the 100-actor scale case, standalone controller-plugin
conformance, the installed compatibility corpus, a fresh wheel installation,
and exact Python public-API verification. Schema v4 remains historical evidence
for the same set without accelerator process conformance; it cannot be
relabelled as v5. The hardware check executes nine
TaskSpec/protocol/safety cases using installed binaries only. The controller
check runs `rne-asset plugin check` against the
reference binary and against a fresh scaffold built offline with warnings
denied; the scaffold SDK must match the bundled SDK byte-for-byte.
The accelerator check executes all nine JSONL exchanges against the bundled
mock, then generates a dependency-free adapter scaffold from the installed CLI
and runs the same process kit against it. Both eleven-check content-addressed
reports must bind the exact manifest, runtime contract, TaskSpec, checkpoint,
and clean shutdown. The scaffold README must retain its explicit warning that
the fixture is authoring-path evidence, not independent accelerator evidence;
its canonical `rne-scaffold.json` must validate the exact schema-v1 file set.
The robot-replay check also uses the installed `rne-asset` binary to create and
verify a content-addressed Failure Capsule from a retained failed behavior
replay and TaskSpec. This proves the external-project evidence authoring path
without relying on source-only `xtask`.
The Python check compares all 24 public exports, constructors, methods,
properties, constants, and text signatures, then writes a stable schema-v1
report.
The independent run additionally emits
`rne_archive_install_rehearsal` schema v1. This outer report binds the archive
file name, byte length, and SHA-256 to the extracted bundle root,
`release-report.json`, canonical `SHA256SUMS`, and the complete inner schema-v5
rehearsal. Validation reconstructs the checksum graph and requires the staged
and independently extracted verdict maps to be identical.
After all ten checks pass, xtask deletes the tool-owned wheel virtual
environment and controller and accelerator scaffolds. `release-bundle`
additionally deletes its
internal `.rehearsal-<target>` directory and target-local copied supply-chain
evidence only after `release-report.json` and `SHA256SUMS` verify. The bundle,
archive-bound reports, replays, conformance reports, and Failure Capsule remain.
Any failed rehearsal keeps all transient directories for diagnosis. Every
cleanup target must be the expected real directory directly beneath its owned
parent; path escapes, symlinks, and regular files are rejected.
The compatibility corpus includes provenance-bound, sensor-bearing snapshot-v1
and snapshot-v2 restores into snapshot-v3, plus the retained original v1 case.
It also includes a provenance-bound vectorized checkpoint-v1 restore and real
scenario replay v2/v3 required-rerun decisions.
The TaskSpec, streaming dataset bundle, and Failure Capsule v1 cases bind their
introducing revisions. Dataset verification materializes the embedded original
shard and must reject a one-bit mutation.
The frontend case binds protocol v1's introducing revision and first committed
full `ClientHello` golden; installed rehearsal repeats byte-exact decode,
re-encode, negotiation, and malformed-input rejection.
The installed binary must have the bundled mobile-manipulator scene and URDF
needed to reproduce both historical outcomes without a source checkout. Source
release CI separately checks the exact source commits, trees, schema constants,
and ancestry using a full-history checkout.
The bundled Rust API registry is audit evidence; source CI performs the actual
31-crate `cargo-semver-checks` comparison because it requires both source trees.

## Local rehearsal

From a clean source checkout with the pinned tools:

```bash
python -m pip install "maturin==1.13.3"
maturin build --locked --release --features extension-module \
  --manifest-path crates/rne_py/Cargo.toml --out artifacts/wheels
cargo run --locked -p xtask -- release-bundle \
  --target x86_64-unknown-linux-gnu \
  --wheel artifacts/wheels/rne_py-0.1.0-*.whl \
  --python python
```

Use target `x86_64-pc-windows-msvc` on Windows. A tag build also passes
`--expected-tag v0.1.0`; the bundle reports `reproducible = true` only when the
worktree is clean and that exact tag identifies `HEAD`. `--allow-dirty` exists
for local development only and is never used by release CI.

After creating and extracting the deterministic archive, rerun:

```bash
cargo run --locked -p xtask -- release-install-smoke \
  --archive artifacts/release/rne-0.1.0-x86_64-unknown-linux-gnu.tar.gz \
  --bundle-dir artifacts/extracted/rne-0.1.0-x86_64-unknown-linux-gnu \
  --output-dir artifacts/extracted-evidence \
  --python python
```

## Signed provenance

Tag pushes and manual release rehearsals use `actions/attest@v4` to create a
signed SLSA v1 provenance attestation for the native archive, Python wheel, and
archive-bound install report. Each platform job copies the action's exact `bundle-path` into its
retained workflow artifact. Pull-request jobs deliberately do not mint
attestations. The publish job must successfully run `gh attestation verify`
against those local bundles for all four cross-platform release assets and both
archive-install reports before `gh release create`; an unsigned,
digest-mismatched, cross-tag, cross-commit, or wrong-workflow subject cannot be
accepted by the workflow.

The trust policy is machine-readable in
`release/artifact-attestation.toml`. `xtask release-check` rejects drift in the
provider, issuer, repository, workflow, predicate type, subjects, Action
version, OIDC permissions, event condition, local-bundle retention, exact
workflow certificate identity, source and signer revisions, or
publish-before-verify ordering.
Consumers should follow [RELEASE_INSTALL.md](RELEASE_INSTALL.md) and verify the
downloaded asset against `rsasaki0109/RoboSim` before extraction. GitHub's
[artifact attestation documentation](https://docs.github.com/en/actions/how-tos/secure-your-work/use-artifact-attestations/use-artifact-attestations)
describes the underlying Sigstore verification model.

## Aggregate release decision

The Linux and Windows jobs feed the `release_candidate` aggregate. It runs
`xtask release-exit --scope release` against the frozen
`release/exit-matrix.toml` contract and uploads a schema-v1 verdict containing
the tested commit, Cargo.lock digest, clean-checkout status, required job
results, and P0/P1 blocker decision. Tagged publication depends on this
aggregate, so both native rehearsals and their attestation steps must pass.

For any 1.x release, `release-check`, both platform `release-bundle` commands,
and the aggregate `release-exit` additionally rerun the evidence-backed 1.0
gate. `RNE_ONE_ZERO_READINESS_MANIFEST` points to the complete evidence pack
and `RNE_ONE_ZERO_READINESS_AS_OF` supplies its explicit assessment date. The
workflow must make that pack available to every release job and retain
`artifacts/release-readiness/promotion-report.json`. Missing or ineligible
evidence fails before packaging or the aggregate verdict. See
[ONE_ZERO_READINESS.md](ONE_ZERO_READINESS.md).

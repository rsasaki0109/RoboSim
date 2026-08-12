# M6: 1.0 RC hardening plan

M6 turns the audited M0-M5 engine into an installable `v1.0.0-rc.1` release
candidate. It does not widen the simulation domain. It freezes the supported
surface, proves compatibility and supply-chain policy, hardens untrusted input
boundaries, and produces independently reproducible release evidence.

## Release contract

The candidate is accepted only when all of the following are true:

- the workspace release version is exactly `1.0.0-rc.1`;
- the minimum supported Rust version (MSRV) is `1.88.0`, the highest declared
  MSRV in the locked dependency graph;
- core crates remain ROS2-free and renderer-independent headless workflows stay
  available;
- every public library enables the `missing_docs` lint and workspace rustdoc
  completes with warnings denied;
- SemVer compatibility is checked against the frozen M6 baseline on every
  subsequent release-candidate change;
- the locked dependency graph has no unaccepted RustSec advisory, unapproved
  license, untrusted source, or unexplained duplicate-version exception;
- deterministic fuzz-smoke campaigns cover the import and transport parsers,
  while `cargo-fuzz` targets remain available for longer manual campaigns;
- release bundles include the CLI, controller-plugin example, licenses,
  compatibility and migration policy, dependency SBOM, SHA-256 checksums, and
  a machine-readable provenance report;
- installed bundle and Python-wheel smoke tests run the flagship robot replay,
  scenario replay, physics conformance, and 100-actor scale workflows;
- two independent clean-checkout rehearsals pass, one on Linux and one on
  Windows;
- the release-blocker registry contains no open P0 or P1 item.

Timing and build paths never participate in deterministic payload hashes.
Release evidence records them separately as diagnostics.

## Compatibility policy

The frozen 1.0 surface includes:

- public Rust APIs in publishable `rne_*` libraries;
- controller C ABI v3 and its documented v2 compatibility floor;
- runner frontend protocol v1;
- `.rne.scene.toml`, `.rne.robot.toml`, `.rne.run.toml`, traffic JSON, plugin
  manifest, replay, Behavior CI, physics-conformance, and scenario-scale schema
  versions;
- the Python ABI3 interface for Python 3.9 and newer.

Before `1.0.0`, an RC may reject an incompatible artifact only with a typed or
actionable compatibility error. After `1.0.0`, stable public Rust API or schema
breakage requires a major release; additive schema changes require tolerant
readers or an explicit version bump. Simulation outcomes may change only when
called out in release notes with updated deterministic evidence.

## Supply-chain and license policy

`cargo-deny` is the authoritative graph policy. CI pins cargo-deny `0.20.2`
and cargo-audit `0.22.2`. Accepted production licenses are:

- `MIT`, `Apache-2.0`, `Apache-2.0 WITH LLVM-exception`;
- `BSD-2-Clause`, `BSD-3-Clause`, `ISC`, `Zlib`, `0BSD`;
- `Unicode-3.0`, `NCSA`, `CC0-1.0`, and `Unlicense`.

Copyleft dependencies are rejected from the default release graph. A future
exception must name the exact crate/version, feature reachability, owner,
rationale, and expiry. Registry dependencies must come from crates.io; git and
unknown registry sources are denied. The release report records the Cargo.lock
digest and a sorted dependency SBOM.

## Fuzz boundary

The stable CI campaign is deterministic and bounded. A fixed seed manifest
generates empty, truncated, oversized, invalid UTF-8, delimiter-heavy, and
seeded byte mutations for:

- OpenSCENARIO XML and scenario replay JSON;
- SDF, MJCF, SUMO network, native traffic, and URDF import inputs;
- runner/frontend framed transport payloads and negotiation messages.

Every parser must return a value or an ordinary error without panic, excessive
allocation, wall-clock waiting, filesystem escape, or state mutation before
validation. Corpus and campaign digests are stable. `cargo-fuzz` targets mirror
the stable campaign for longer sanitizer-backed runs outside the PR budget.

## Release artifacts

Each platform bundle is named
`rne-1.0.0-rc.1-<target>.<zip|tar.gz>` and contains:

- `rne-asset`;
- the velocity-servo controller-plugin shared library and manifest;
- `README.md`, `CHANGELOG.md`, `LICENSE-MIT`, and `LICENSE-APACHE`;
- `COMPATIBILITY.md` and the release-blocker registry;
- `sbom.cargo.json`, `release-report.json`, and `SHA256SUMS`.

The report pins release version, Git commit, target, Rust/Cargo versions,
Cargo.lock digest, bundle member digests, public schema/ABI floors, audit
verdicts, fuzz campaign digest, and flagship workflow verdicts. It must not
claim reproducibility if the worktree is dirty or the expected tag does not
point at the tested commit.

## Work packages

### M6-A: freeze and migration policy

- Add this plan, `docs/COMPATIBILITY.md`, and a machine-readable blocker
  registry.
- Set version `1.0.0-rc.1` and MSRV `1.88.0`.
- Deny missing public docs and warnings in workspace rustdoc.
- Add SemVer/API checks and version/schema consistency tests.

### M6-B: supply-chain evidence

- Add `deny.toml` with explicit licenses, sources, bans, and exceptions.
- Run pinned cargo-deny and cargo-audit in CI.
- Emit a sorted dependency SBOM and lockfile digest from the locked graph.

### M6-C: parser and protocol hardening

- Add deterministic stable-toolchain fuzz-smoke campaigns and regression
  corpora.
- Add matching cargo-fuzz targets for importer and framed-transport boundaries.
- Gate panic freedom, input limits, corpus digest, and report schema.

### M6-D: artifact and install rehearsal

- Add cross-platform bundle assembly, SHA-256 manifest, provenance report, and
  installed-bundle smoke tests.
- Build and install the ABI3 Python wheel in the rehearsal environment.
- Add tag/manual release workflow with Linux and Windows artifacts.

### M6-E: final exit matrix

- Run format, boundaries, Clippy, workspace tests, rustdoc, headless, Behavior
  CI, OSS parity, supply-chain checks, fuzz-smoke, and release rehearsals from
  the locked graph.
- Complete two clean-checkout rehearsals on Linux and Windows.
- Record measured evidence in the roadmap, changelog, and this plan.
- Publish and merge only with zero open P0/P1 blocker and every required check
  green.

## Initial audit (2026-08-12)

- M0-M5 are merged and their GitHub checks are green.
- Workspace version is `0.14.0-rc.1`; no MSRV is declared.
- 27 of 28 library roots deny missing docs; `rne_py` is the exception to audit.
- No release workflow, compatibility policy, blocker registry, cargo-deny
  policy, advisory job, SBOM, or fuzz workspace exists.
- `cargo package -p rne_core --no-verify` fails because internal path
  dependencies have no registry version requirement.
- The locked graph's highest declared dependency MSRV is `1.88.0`.
- cargo-audit, cargo-deny, and cargo-semver-checks are not installed locally.

## Implementation status

- M6-A freeze and migration policy: complete.
- M6-B supply-chain evidence: complete.
- M6-C parser/protocol hardening: complete.
- M6-D artifacts and install rehearsal: pending.
- M6-E final exit matrix: pending.

## M6-A evidence (2026-08-12)

- All 107 workspace packages declare Rust `1.88.0`; `cargo +1.88.0 check
  --locked --workspace --all-targets` passed on Windows in 4m36s.
- Exactly 27 libraries are publishable. Their internal dependencies use exact
  `=1.0.0-rc.1` requirements, and one multi-package Cargo invocation assembled
  all 27 crate archives before any registry publication.
- `cargo run --locked -p xtask -- release-check --allow-dirty` passed: release
  metadata, compiled compatibility constants, the blocker registry, workspace
  rustdoc with warnings denied, and all crate archives were accepted.
- Workspace rustdoc found and fixed six stale/private links that ordinary builds
  did not exercise; all 28 library roots now deny missing documentation.
- `cargo-semver-checks 0.49.0` checked `rne_ai` against `origin/main` while
  forcing patch compatibility: 223 checks passed and 30 were inapplicable.
  GitHub CI covers all 27 public crates in seven parallel package groups.
- The xtask release-contract and blocker-registry unit suite passed 10/10 tests.

## M6-B evidence (2026-08-12)

- `cargo-deny 0.20.2` and `cargo-audit 0.22.2` are pinned in CI. The
  crates.io-only source policy, explicit license allowlist, wildcard and
  duplicate-version bans, and advisory policy all pass against the locked
  530-package graph.
- The only accepted advisory is the unmaintained `paste 1.0.15`
  (`RUSTSEC-2024-0436`) build-time macro reached through Rapier's
  nalgebra/simba graph. Its owner, exact reachability, mitigation, rationale,
  and 2027-02-12 expiry are recorded in
  `release/supply-chain-exceptions.toml`; all allowed duplicate versions use
  the same time-bounded evidence format.
- Security maintenance advanced `crossbeam-epoch`, `anyhow`, `memmap2`,
  `pyo3`, `wayland-scanner`, and `quick-xml`. The PyO3 binding was migrated to
  the current attachment and extraction APIs, and unused winit font features
  removed the unmaintained `ttf-parser` path.
- All 75 private example, test, and tooling manifests now carry the workspace
  license, and their internal RNE dependencies use exact workspace-managed
  requirements. Direct PNG users share the workspace's maintained PNG line.
- `cargo run --locked -p xtask -- supply-chain` validates policy/registry
  agreement, runs both pinned audit tools, and emits a deterministically sorted
  `sbom.cargo.json` plus `cargo-lock.sha256`. Two independent generations had
  identical SHA-256 digests. `--generate-only` reproduces the evidence without
  requiring the audit binaries.
- The supply-chain CI job uploads both evidence files and is required by the
  aggregate workspace gate. Its xtask tests passed 13/13; warning-free
  workspace check and Clippy passed on Rust 1.95, and the locked all-target
  workspace check passed on the declared Rust 1.88 MSRV.

## M6-C evidence (2026-08-12)

- The stable-toolchain campaign covers nine explicit boundaries:
  OpenSCENARIO XML, scenario replay JSON, SDF, MJCF, SUMO network XML, native
  traffic JSON, URDF, framed transport payloads, and transport negotiation.
  Its 361 fixed cases include valid and committed regression corpus entries,
  empty/truncated/invalid-UTF-8/delimiter-heavy inputs, a 64 KiB limit probe,
  excessive MJCF nesting, and 32 seeded mutations per boundary.
- Two independent report generations were byte-identical. The schema-v1 report
  recorded zero panics, corpus digest
  `sha256:1e5c61f2b084e5369f8c79e188b7128573c45c6249cbfbeb773f2a1bd3d7218e`,
  and campaign digest
  `sha256:0d5918b43c5d4403a31a811fe1dfef10714e88d6a648f95f7a09fda9f87f7477`.
- Public in-memory and file parsers reject oversized input before parsing or
  full-file allocation. SDF and MJCF use 16 MiB input limits, OpenSCENARIO and
  URDF use 64 MiB, SUMO/native traffic use 128 MiB, and framed transport keeps
  its 32 MiB absolute payload ceiling even if a caller offers a larger limit.
- OpenSCENARIO parameter substitution is single-pass, deterministic, and
  output-bounded. Catalog directories reject absolute/parent traversal, cache
  resolved entries, cap each catalog file at 8 MiB, and cap a scan at 1,024
  files and 64 MiB. Canonical directory and file paths are verified so symlinks
  cannot escape the scenario directory. MJCF rejects body nesting past 128
  levels before unbounded recursion.
- `cargo +nightly fuzz run importers` and `cargo +nightly fuzz run transport`
  call the same boundary functions without panic catching for longer sanitizer
  campaigns. Both cargo-fuzz binaries compile warning-free; usage and regression
  promotion are documented in `docs/FUZZING.md`.
- `cargo run --locked -p xtask -- fuzz-smoke` validates coverage, accepted valid
  seeds, accounting, limits, schema, and panic freedom before writing the report.
  Its required CI job uploads that report and feeds the aggregate workspace
  gate. Campaign tests passed 3/3, all affected parser/transport unit and
  integration tests passed, and affected workspace plus cargo-fuzz targets pass
  Clippy with warnings denied.

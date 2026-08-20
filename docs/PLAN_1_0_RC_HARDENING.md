# M6: 0.1 release hardening plan

M6 turns the audited M0-M5 engine into an installable `v0.1.0` release. It does
not widen the simulation domain. It freezes the supported
surface, proves compatibility and supply-chain policy, hardens untrusted input
boundaries, and produces independently reproducible release evidence.

## Release contract

The candidate is accepted only when all of the following are true:

- the workspace release version is exactly `0.1.0`;
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

The frozen 0.1 surface includes:

- public Rust APIs in publishable `rne_*` libraries;
- controller C ABI v3 and its documented v2 compatibility floor;
- runner frontend protocol v1;
- `.rne.scene.toml`, `.rne.robot.toml`, `.rne.run.toml`, traffic JSON, plugin
  manifest, replay, Behavior CI, physics-conformance, and scenario-scale schema
  versions;
- the Python ABI3 interface for Python 3.9 and newer.

While the project remains below `1.0.0`, incompatible public API or schema
changes must carry a typed/actionable compatibility error, migration notes, and
updated deterministic evidence. Once `1.0.0` is declared, stable public Rust
API or schema breakage requires a major release; additive schema changes require
tolerant readers or an explicit version bump. Simulation outcomes may change
only when called out in release notes with updated deterministic evidence.

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
`rne-0.1.0-<target>.<zip|tar.gz>` and contains:

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
- Set version `0.1.0` and MSRV `1.88.0`.
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
- M6-D artifacts and install rehearsal: complete.
- M6-E final exit matrix: complete.

## M6-A evidence (2026-08-12)

- All 107 workspace packages declare Rust `1.88.0`; `cargo +1.88.0 check
  --locked --workspace --all-targets` passed on Windows in 4m36s.
- Exactly 27 libraries are publishable. Their internal dependencies use exact
  `=0.1.0` requirements, and one multi-package Cargo invocation assembled
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

## M6-D evidence (2026-08-12)

- `xtask release-bundle` assembles one native, versioned directory from the
  locked graph and rejects non-host targets, unsafe target names, dirty release
  builds unless explicitly allowed for local development, and mismatched RC
  tags. Schema-v1 provenance records the exact commit, target, Rust/Cargo
  versions, Cargo.lock digest, contract floors, payload digests, supply-chain
  verdicts, fuzz digest, and six workflow verdicts. Reproducibility remains
  false unless a clean worktree is built from the exact `v0.1.0` tag.
- A real Windows x86-64 bundle was built with Rust 1.95.0, Python 3.11,
  maturin 1.13.3, cargo-deny 0.20.2, and cargo-audit 0.22.2. Its installed-only
  rehearsal passed robot replay, scenario replay, all advertised analytic and
  Rapier physics capabilities, the 100-actor/600-step scale case, dynamic
  discovery of the velocity-servo controller DLL, and fresh installation and
  execution of the `cp39-abi3` wheel. The scale case's slowest of three local
  samples measured 2,505.86 headless steps/s against the 60 steps/s floor.
- The Windows directory was compressed with sorted members, fixed ZIP metadata,
  and maximum DEFLATE compression, expanded into a fresh path, verified against
  the exact-member `SHA256SUMS`, and passed all six checks again without
  source-tree binaries or Python packages. Checksum validation rejects modified,
  missing, duplicate, traversal, symbolic-link, and unlisted members.
- The required release workflow repeats assembly, native archive extraction,
  checksum verification, wheel installation, and all six checks on clean Linux
  and Windows runners. A verified `v0.1.0` tag can publish exactly two
  archives and two standalone wheels only after both platform jobs pass.
- The artifact command unit suite passes 17/17 tests, affected crates pass
  Clippy with warnings denied, and the complete bundle plus extracted-bundle
  rehearsals pass locally. Two consecutive assemblies produced byte-identical
  provenance, install, and checksum reports. The release report intentionally
  records `reproducible=false` for this untagged dirty development run.

## M6-E implementation evidence (2026-08-12)

- `release/exit-matrix.toml` freezes exactly 14 CI jobs and the Linux/Windows
  native rehearsal jobs, including their runner class, clean-checkout
  requirement, and exact command. Graph-building commands must use `--locked`.
  `xtask release-check` rejects drift
  between that contract and either workflow, including path filters that could
  silently omit a pull-request rehearsal.
- The `workspace` and `release_candidate` aggregate jobs run even after a
  dependency failure, pass every `needs.*.result` to `xtask release-exit`, and
  upload schema-v1 evidence. Reports pin the tested commit and Cargo.lock
  digest and independently record clean checkout, all-green jobs, and zero
  open P0/P1 blockers. Tag publication now depends on `release_candidate`.
- The extended xtask unit suite passes 22/22 tests and warning-free Clippy on
  Rust 1.95; the new code also checks successfully on the Rust 1.88 MSRV.
  Local positive probes generated complete CI and release reports; because the
  implementation worktree was dirty they correctly reported
  `release_eligible=false`. A synthetic Windows failure returned exit code 1
  and recorded the failed job rather than producing a false-green aggregate.
- The complete local matrix passed on the final working content: format and
  warning-free all-target workspace Clippy; all workspace unit, integration,
  process, plugin-scaffold, and doc tests; warning-free rustdoc and all 27
  public crate archives; 31 render plus 52 sensor headless tests; 10/10
  Behavior CI seeds; all example/media and 11 Python RL smokes; the 531-package
  supply-chain policy; and all 361 fuzz-smoke cases. OSS parity passed 22/22
  checks in 516.784 s. Its three 100-actor samples had identical hashes and a
  slowest 3,246.41 steps/s against the 60 steps/s floor, with zero unexplained
  violation.
- PR #162 passed the final clean-checkout matrix and merged as `a4e230f`.
  GitHub Actions CI run `31577873319` passed every required job, including
  three workspace-test shards, 22/22 Windows parity checks, and the new
  `workspace` aggregate. Release run `31577873221` passed Linux in 6m41s,
  Windows in 12m27s, and the new `release_candidate` aggregate in 3m54s.
- The uploaded CI and release schema-v1 reports both identify tested commit
  `f14799361bbd65d86b2bba3b94713b12cf414017` and Cargo.lock SHA-256
  `d5453946485aab7ef3bbac1968188b4fb789e986f94e0adf4808d41ac6b249a8`.
  Both independently record a clean checkout, zero open P0/P1 blockers, every
  required dependency as `success`, and `release_eligible=true`. This closes
  the M6-E acceptance matrix.

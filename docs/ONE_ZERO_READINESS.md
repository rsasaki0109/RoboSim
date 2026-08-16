# RNE 1.0 readiness evidence

RNE does not become 1.0 because a date arrived or a star count was reached.
The promotion gate is an evidence-backed audit of external use, extension,
hardware, release, compatibility, blocker, and support commitments. The
committed tracker intentionally remains ineligible until those facts exist.

## Run the progress audit

The date is mandatory so the report never depends on wall-clock time:

```powershell
cargo run --locked -p xtask -- release-readiness `
  --as-of 2026-08-16 `
  --output E:/RoboSim-readiness/report.json
```

Without `--require-eligible`, unmet checks are written as `not_met` and the
command succeeds. Malformed, missing, path-escaping, symlinked, oversized, or
SHA-mismatched evidence always fails. Add `--require-eligible` only at the 1.0
promotion gate; it exits unsuccessfully unless every check passes.

The default tracker is `release/one-zero-readiness.toml`. An evidence pack can
live outside the repository, including on an external disk:

```powershell
cargo run --locked -p xtask -- release-readiness `
  --manifest E:/RoboSim-readiness/one-zero-readiness.toml `
  --as-of 2027-02-15 `
  --output E:/RoboSim-readiness/report.json `
  --require-eligible
```

Evidence paths use forward slashes and are relative to the selected manifest.
Each reference contains an exact lowercase `sha256:` digest. The audit reads
regular files no larger than 64 MiB and rejects symlinks or paths outside the
pack. It does not download URLs, create tags, or infer evidence from GitHub
stars.

## Fixed checks

| Check | Passing evidence |
|---|---|
| `stability_window` | At least 183 calendar days since the immutable Rust API baseline, at least 183 days of observed external-project use, and zero declared unplanned breaks |
| `external_projects` | At least two distinct repositories, owned outside the RNE repository owner, each with a valid TaskSpec and fully verifiable Failure Capsule produced without repository-author assistance |
| `third_party_plugin` | At least one externally owned controller plugin with a passing typed conformance report |
| `external_system` | At least one externally owned physics backend or hardware adapter with a passing typed conformance report |
| `reference_hardware` | A full LeKiwi physical-evidence manifest accepted by the safety and provenance verifier |
| `release_artifacts` | Linux x86-64 and Windows x86-64 archives, attestations, verification output, release reports, and nine-check installed rehearsals from the same clean tagged revision |
| `historical_compatibility` | A passing report for the exact committed compatibility registry with at least 24 distinct checks and fail-closed future/unknown-field cases |
| `p0_p1_blockers` | `release/blockers.toml` is structurally valid and has no open P0/P1 entry |
| `support_commitment` | A named maintainer, support period, and HTTPS policy are explicitly committed |

The report schema is registered as `evidence.one_zero_readiness_report = 1` in
`release/contracts.toml`. Its check order is fixed and a committed golden
captures the current honest baseline. `manifest_sha256` binds the complete
normalized input, including external identities and support fields. On
2026-08-16, only the blocker check is passing: the project is `eligible=false`
with 1 of 9 checks satisfied.

## Evidence-pack shape

The committed tracker pins the candidate surface and thresholds. Evidence is
added only after independent results exist. Place the two optional top-level
references before `[candidate]`; repeated entries use TOML arrays:

```toml
reference_hardware = { path = "hardware/lekiwi-evidence.json", sha256 = "sha256:<64-lowercase-hex>" }
compatibility_report = { path = "compatibility/report.json", sha256 = "sha256:<64-lowercase-hex>" }

[[external_project]]
id = "independent-project-a"
owner = "external-owner"
repository = "https://example.invalid/project-a"
first_used_on = "2026-08-16"
last_verified_on = "2027-02-15"
author_assistance = false
task_spec = { path = "projects/a/task.json", sha256 = "sha256:<64-lowercase-hex>" }
failure_capsule = { path = "projects/a/capsule/capsule.json", sha256 = "sha256:<64-lowercase-hex>" }

[[third_party_plugin]]
id = "external-controller"
owner = "external-owner"
repository = "https://example.invalid/controller"
report = { path = "plugins/controller-report.json", sha256 = "sha256:<64-lowercase-hex>" }

[[external_system]]
id = "external-physics"
owner = "external-owner"
repository = "https://example.invalid/physics"
kind = "physics_backend" # or "hardware_adapter"
report = { path = "systems/physics-report.json", sha256 = "sha256:<64-lowercase-hex>" }
```

Each `platform_release` also declares `platform`, `revision`, and `tag`, then
references `archive`, `attestation`, `attestation_verification`,
`release_report`, and `install_report`. Both platforms must resolve to the same
retained tag and commit. The release report must say the checkout was clean,
tag-matched, reproducible, supply-chain clean, and passing all nine installed
workflows.

External evidence should be reviewed for real independence before its digest
is accepted. A structurally valid self-authored report is not a substitute for
third-party adoption.

## 1.x promotion interlock

Every `release-check`, `release-bundle`, and `release-exit` invocation is wired
to the same guard. For 0.x it performs no external-evidence read. For any 1.x
or later version it requires these environment variables before release work
can proceed:

```text
RNE_ONE_ZERO_READINESS_MANIFEST=/absolute/path/to/one-zero-readiness.toml
RNE_ONE_ZERO_READINESS_AS_OF=YYYY-MM-DD
RNE_ONE_ZERO_READINESS_OUTPUT=/absolute/path/to/promotion-report.json # optional
```

The output defaults to `artifacts/release-readiness/promotion-report.json`.
The guard reruns the full typed audit; a prewritten JSON report alone is not an
input and cannot unlock a release. Missing variables, malformed dates,
ineligible evidence, or tampered referenced files stop all three release
paths. A future 1.0 workflow must securely provision the complete evidence pack
to its clean Linux, Windows, and aggregate jobs and retain the generated report.
The current 0.1 release workflow intentionally has no such variables and cannot
silently become a 1.x publisher by changing package metadata alone.

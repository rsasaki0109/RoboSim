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

## Initialize an external evidence pack

Start from the committed, deliberately ineligible baseline instead of copying
paths and digests by hand:

```powershell
cargo run --locked -p xtask -- readiness-pack init `
  --output E:/RoboSim-readiness
```

The command validates the immutable candidate identity, then atomically
publishes `one-zero-readiness.toml` and the retained compatibility report. It
refuses an existing output directory. The new pack therefore begins at the same
honest `2/9`, `eligible=false` state as the source tracker.

Stage each independently produced file before referencing it in the manifest:

```powershell
cargo run --locked -p xtask -- readiness-pack stage `
  --pack E:/RoboSim-readiness `
  --source D:/external-controller/controller-report.json `
  --path plugins/external-controller/controller-report.json
```

The successful command prints a paste-ready reference:

```toml
{ path = "plugins/external-controller/controller-report.json", sha256 = "sha256:<64-lowercase-hex>" }
```

`stage` accepts only a regular non-symlink file at or below the audit's 64 MiB
limit. Its destination must be a contained forward-slash relative path; parent
symlinks, the readiness manifest itself, temporary-name collisions, and any
existing destination fail closed. Bytes are copied through a private temporary
name and hashed before the final name is published.

This helper establishes file identity only. A human must still review external
ownership and independence, add the appropriate typed entry shown below, and
run `release-readiness`. Staging a file cannot change a readiness check by
itself.

Independent projects and extension authors submit the complete metadata and
artifact checklist through the fixed
[external evidence intake](EXTERNAL_EVIDENCE_INTAKE.md). Its issue forms are a
review queue only; accepted bytes still pass this audit from an external pack.

Evidence paths use forward slashes and are relative to the selected manifest.
Each reference contains an exact lowercase `sha256:` digest. The audit reads
regular files no larger than 64 MiB and rejects symlinks or paths outside the
pack. It does not download URLs, create tags, or infer evidence from GitHub
stars.

Manifest v4 requires the accelerator evidence collection to be explicit. Use
`accelerator_adapter = []` at the top level while no entries are retained;
replace that empty array with one or more `[[accelerator_adapter]]` tables when
evidence is accepted.

## Fixed checks

| Check | Passing evidence |
|---|---|
| `stability_window` | At least 183 calendar days since the immutable Rust API baseline, at least 183 days of observed external-project use, and zero declared unplanned breaks |
| `external_projects` | At least two distinct repositories, owned outside the RNE repository owner, each with a valid TaskSpec and fully verifiable Failure Capsule produced without repository-author assistance |
| `third_party_plugin` | At least one externally owned controller plugin whose passing typed report is rebound to the exact retained library and manifest bytes |
| `external_system` | At least one externally owned physics backend or hardware adapter whose passing typed report is rebound to the exact retained implementation subject; hardware also retains its TaskSpec and normalized launch arguments. Audited accelerator adapters are reported separately and do not satisfy this check |
| `reference_hardware` | A full LeKiwi physical-evidence manifest accepted by the safety and provenance verifier |
| `release_artifacts` | Linux x86-64 and Windows x86-64 archives plus archive-bound ten-check install reports, both freshly Sigstore-verified from retained bundles; extracted release reports and SHA256SUMS must reconstruct the same clean tagged artifact graph |
| `historical_compatibility` | A retained report exactly equal to a fresh execution of at least 36 registered typed-reader checks, including fail-closed accelerator capability/status/protocol/process/conformance/scale/scaffold, the controller scaffold, future/unknown-field mutations, and verified historical Git revision/tree/blob provenance |
| `p0_p1_blockers` | `release/blockers.toml` is structurally valid and has no open P0/P1 entry |
| `support_commitment` | A named maintainer, unambiguous support period, and published HTTPS policy are explicitly committed; an uncommitted table must contain no partial claims |

The manifest, report, and attestation receipt schemas are registered as
`evidence.one_zero_readiness_manifest = 4`,
`evidence.one_zero_readiness_report = 1`, and
`evidence.github_attestation_verification = 1` in `release/contracts.toml`.
The archive-bound install result is separately registered as
`evidence.archive_install_rehearsal_report = 1`.
The report check order is fixed and a committed golden captures the current
honest baseline. `manifest_sha256` binds the complete normalized input,
including external identities and support fields. The retained 36-check
compatibility report is byte-for-byte equal to a fresh typed-reader replay and
the blocker registry is clean, so the committed 2026-08-16 baseline is
`eligible=false` with 2 of 9 checks satisfied. The remaining seven checks still
require real external, physical, signed-release, elapsed-time, or maintainer
evidence.

The pre-1.0 status and required contents of the final maintainer commitment are
documented in [the support policy](SUPPORT.md). The tracker remains
`committed = false` with all other support fields empty until an authorized
maintainer publishes that policy; draft language cannot make the check pass.

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
revision = "<40-lowercase-hex-commit>"
first_used_on = "2026-08-16"
last_verified_on = "2027-02-15"
author_assistance = false
task_spec = { path = "projects/a/task.json", sha256 = "sha256:<64-lowercase-hex>" }
failure_capsule = { path = "projects/a/capsule/capsule.json", sha256 = "sha256:<64-lowercase-hex>" }

[[third_party_plugin]]
id = "external-controller"
owner = "external-owner"
repository = "https://example.invalid/controller"
revision = "<40-lowercase-hex-commit>"
library = { path = "plugins/external-controller.dll", sha256 = "sha256:<64-lowercase-hex>" }
manifest = { path = "plugins/rne-plugin.json", sha256 = "sha256:<64-lowercase-hex>" }
report = { path = "plugins/controller-report.json", sha256 = "sha256:<64-lowercase-hex>" }

[[external_system]]
id = "external-physics"
owner = "external-owner"
repository = "https://example.invalid/physics"
revision = "<40-lowercase-hex-commit>"
kind = "physics_backend"
subject = { path = "systems/physics-source.tar.zst", sha256 = "sha256:<64-lowercase-hex>" }
report = { path = "systems/physics-report.json", sha256 = "sha256:<64-lowercase-hex>" }

[[external_system]]
id = "external-hardware"
owner = "external-owner"
repository = "https://example.invalid/hardware"
revision = "<40-lowercase-hex-commit>"
kind = "hardware_adapter"
subject = { path = "systems/adapter.py", sha256 = "sha256:<64-lowercase-hex>" }
task_spec = { path = "systems/task.json", sha256 = "sha256:<64-lowercase-hex>" }
# Exact normalized list hashed by the report. An argument equal to the adapter
# subject path is represented by the runner as "<adapter-subject>".
adapter_arguments = ["<adapter-subject>", "--sandbox", "isolated-v1"]
report = { path = "systems/hardware-report.json", sha256 = "sha256:<64-lowercase-hex>" }

[[accelerator_adapter]]
id = "external-accelerator"
owner = "external-owner"
repository = "https://example.invalid/accelerator"
revision = "<40-lowercase-hex-commit>"
subject = { path = "accelerators/adapter.py", sha256 = "sha256:<64-lowercase-hex>" }
task_spec = { path = "accelerators/task.json", sha256 = "sha256:<64-lowercase-hex>" }
accelerator_manifest = { path = "accelerators/accelerator.toml", sha256 = "sha256:<64-lowercase-hex>" }
runtime_contract = { path = "accelerators/runtime.toml", sha256 = "sha256:<64-lowercase-hex>" }
adapter_arguments = ["-m", "external_adapter", "--mode", "conformance"]
report = { path = "accelerators/process-report.json", sha256 = "sha256:<64-lowercase-hex>" }
```

Manifest v4 adds the separately audited `accelerator_adapter` entries. Each
entry is fail-closed and content-addressed, but only `external_system` entries
of kind `physics_backend` or `hardware_adapter` count toward 1.0 eligibility;
v3 evidence therefore cannot be relabelled without adding and revalidating the
new manifest shape.

Compatibility evidence must be assessed from a complete source checkout whose
Git object database contains the registered historical commits. The gate
rehashes the report, validates its registry identity, replays the complete
corpus from the current checkout, rechecks historical source provenance, and
requires field-for-field equality with the retained report. Editing a report
to say `passed` cannot replace this replay.

The committed report lives at
`release/evidence/compatibility-report-v1.json`. It is deliberately retained
separately from the compatibility golden even though their current bytes are
identical: the golden detects output drift, while the readiness reference and
SHA-256 identify the evidence admitted to the promotion audit. Any reader,
fixture, registry, or historical-provenance change must regenerate and review
both roles explicitly. Native release bundles stage the manifest and this
retained report together; the full promotion replay still runs from a source
checkout with the registered Git history.

Manifest v3 retains v2's rejection of report-only external certification.
Plugin verification
rehashes the retained library and manifest, compares their file names and the
library size, validates the plugin manifest, and requires its name to equal the
negotiated controller identity. Physics verification rehashes the exact
implementation artifact or deterministic source bundle named by the report.
Hardware verification additionally rehashes and validates the TaskSpec,
recomputes the normalized argument-list digest, and checks the negotiated task
identity and flattened observation/action widths. External repository commits
are fixed as lowercase 40-character revisions. Remote repository ownership and
the social fact of independence still require human review; exact subject
binding cannot turn an in-repository reference into third-party adoption.

Each `platform_release` also declares `platform`, `revision`, and `tag`, then
references `archive`, `attestation`, `archive_attestation_verification`,
`release_report`, `checksum_manifest`, `install_report`, and
`install_attestation_verification`. Both platforms must resolve to the same
retained tag and commit. The release report must say the checkout was clean,
tag-matched, reproducible, supply-chain clean, and passing all ten installed
workflows. `install_report` is the strict
`rne_archive_install_rehearsal` schema-v1 wrapper: it fixes the archive file,
size, digest, extracted bundle root, release report, checksum manifest, and
schema-v5 ten-check rehearsal. `attestation` is the exact JSON Sigstore bundle
emitted by `actions/attest@v4`; both verification fields are strict
`rne_github_attestation_verification` schema-v1 receipts, not raw CLI output.

The audit does not trust either receipt by itself. For every platform it reruns
`gh attestation verify --bundle` over both the referenced archive and the
archive-install report and pins the
repository, exact workflow certificate identity, tag ref, source commit,
signer commit, GitHub OIDC issuer, SLSA v1 predicate, and self-hosted-runner
rejection. Each verification must return exactly one in-toto subject with the
expected SHA-256. The gate regenerates both stable receipts, compares every
field, rehashes the extracted reports, and proves that `SHA256SUMS` equals the
release report's complete member graph plus the report itself. It also requires
the staged and independently extracted ten-check verdicts to agree. Missing
`gh`, failed signature or transparency verification, a bundle/archive/report
swap, unknown fields, or any policy drift fails closed. Store all seven files
per platform together in the external evidence pack; the committed release
workflow retains the action's exact `bundle-path` for this purpose. Manifest v2
lacks these distinct subjects and receipts and cannot be relabelled as v3.

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

# External evidence intake

RNE remains 0.x until independently produced evidence passes the same typed
readiness audit used by release tooling. A GitHub issue is only a review queue:
it is never itself evidence, and opening one cannot change a readiness result.
Stars, forks, self-authored reference implementations, screenshots, and copied
JSON reports do not satisfy an external-use gate.

The machine-readable route registry is
`release/external-evidence-intake.toml`. Registry v9 binds the archive-only
installed flagship quickstart and acyclic candidate templates for the
flagship, external-project, third-party controller-plugin, and simulator
adapter routes. It
continues to distinguish qualifying systems from audited nonqualifying
accelerator evidence. Validate the registry, guides, template, and all
required issue-form fields before publishing a release:

```bash
cargo run --locked -p xtask -- external-intake-check
```

## Common submission rules

All routes require:

1. a public repository owned outside the RNE repository owner;
2. an exact lowercase 40-character source revision;
3. the exact RNE 0.x release, archive checksum, and platform used;
4. immutable downloads for every required artifact, with file name, byte size,
   and lowercase SHA-256;
5. complete reproduction or conformance commands and their exit status;
6. permission for maintainers to retain and redistribute the submitted bytes
   as readiness evidence.

Use a GitHub release asset or another immutable, unauthenticated download. Do
not put credentials, private URLs, personal data, robot access tokens, or
unredistributable vendor files in an issue or artifact. Maintainers download
fresh copies, review ownership and provenance, stage them into an external
pack with `xtask readiness-pack stage`, and rerun `xtask release-readiness`.
The audit rehashes and parses the retained bytes; it does not trust issue text.

Compute digests with one of:

```powershell
Get-FileHash -Algorithm SHA256 path/to/file
```

```bash
sha256sum path/to/file
```

## `installed_flagship_reproduction`

Use
`.github/ISSUE_TEMPLATE/installed-flagship-reproduction.yml` to measure the
official installed flagship on one named Windows or Linux x86_64 machine. One
accepted independent run is required for the installed-proof gate. RNE CI,
repository-author runs, copied reports, and placeholder machine labels do not
qualify.

Download a clean tagged native archive, verify its published checksum, extract
it, and run from the extracted bundle root:

```bash
./bin/rne-flagship-proof flagship-proof --cross-backend \
  --measure-on "community-lab-desktop-a" --verify-installed-bundle .
```

On Windows use `bin\\rne-flagship-proof.exe`. The complete archive-only operator
procedure and candidate JSON are in
[`EXTERNAL_FLAGSHIP_REPRODUCTION.md`](EXTERNAL_FLAGSHIP_REPRODUCTION.md) and
`release/external-flagship-submission-template.json`. Preserve the archive, the
exact extracted bundle, and the complete `flagship-proof` directory without
editing their contents.

After submission, a maintainer freshly downloads the retained bytes and, from
a pinned checkout of the same RNE release, generates the versioned external
report (PowerShell line continuations are shown):

```powershell
cargo run --locked -p xtask -- external-flagship-check `
  --archive path/to/rne-RELEASE-TARGET.zip `
  --bundle-dir path/to/rne-RELEASE-TARGET `
  --proof-dir path/to/extracted-proof-bundle/flagship-proof `
  --proof-bundle path/to/independent-proof-bundle.zip `
  --evidence-repo-dir path/to/clean-external-evidence-repository `
  --submission path/to/external-flagship-submission.json `
  --revision 0123456789abcdef0123456789abcdef01234567 `
  --output external-flagship-reproduction.json
```

The operator never needs an RNE source checkout, and maintainer verification
does not retroactively provide author assistance to the independent run. The
checker fails closed unless the release is clean, tagged, reproducible,
and internally checksum-consistent. Candidate schema v2 keeps the repository
revision outside the committed JSON and the candidate outside the proof
bundle, avoiding both content-hash cycles. The checker records the separately
supplied evidence revision and rehashes the candidate, proof bundle, release
archive, logs, checksum manifest, and packaged producer executable,
requires the evidence checkout's clean `HEAD`, origin URL, and committed
candidate/log bytes to match that revision,
verifies the proof report and Failure Capsule, requires
Rapier and bundled MuJoCo success plus intentional failure, checks all named SI
tolerances and the exact first violation, matches the timing platform to the
archive, and enforces the 15-minute limit. The emitted schema-v2 report binds
the independently owned repository revision and all qualifying artifacts.
`author_assistance=false` is mandatory.

## `external_project`

Use
`.github/ISSUE_TEMPLATE/external-project-evidence.yml` for a real project that
uses RNE to define and reproduce its own task. Two distinct external
repositories are required for 1.0.

The submitted revision must retain a valid `TaskSpec`, every member of a
complete `Failure Capsule`, the acyclic
`release/external-project-submission-template.json` candidate, and complete
stdout/stderr logs. The Capsule's `rne_task_spec` member must have the exact
digest of the separately submitted TaskSpec. Record
`first_used_on` and `last_verified_on` as explicit `YYYY-MM-DD` dates.

This route uniquely requires `author_assistance=false`. Documentation,
published examples, and the installed CLI may be used, but an RNE repository
author must not directly perform, debug, or repair the submitted reproduction.
If direct help is needed, fix the public documentation or tooling first and
restart the qualifying run independently. This prevents a maintainer-operated
demo from being relabelled as external adoption.

Failure Capsule creation and verification use:

```bash
rne-asset failure-capsule create \
  --replay path/to/failure.rne-replay \
  --evidence path/to/task.json \
  --output artifacts/failure-capsule \
  --backend backend-name \
  --backend-version backend-version
rne-asset failure-capsule verify \
  artifacts/failure-capsule
```

The command is shipped in each native release bundle. Run it from the extracted
release root (which retains the release lockfile), or from a locked Rust
project root. It does not require the RNE source tree. The external project's
immutable repository revision remains a separate mandatory submission field.

Commit the TaskSpec, complete Capsule directory, candidate, and distinct logs
to the external repository. Keep the containing 40-character Git revision out
of the candidate so the candidate does not hash the commit that contains
itself. After independently downloading the exact official release archive, a
maintainer verifies and binds the submission without executing project code:

```powershell
cargo run --locked -p xtask -- external-project-check `
  --release-archive path/to/rne-0.2.0-TARGET.zip `
  --task path/to/clean-external-project/evidence/task.json `
  --capsule-dir path/to/clean-external-project/evidence/failure-capsule `
  --submission path/to/clean-external-project/external-project-submission.json `
  --evidence-repo-dir path/to/clean-external-project `
  --revision 0123456789abcdef0123456789abcdef01234567 `
  --output external-project-maintainer-report.json
```

The checker requires a clean `HEAD`, matching public origin, committed bytes,
three or more successful reproduction commands, no author assistance, valid
usage dates, the exact official release archive name and digest, a typed
TaskSpec, and a fully verified Failure Capsule. The readiness audit reparses
the maintainer report and rehashes the release, TaskSpec, Capsule manifest and
all Capsule members, candidate, and logs. A copied `capsule.json` or an issue
claim therefore cannot satisfy either external-project slot.

See [Trust evidence quickstart](EVIDENCE_QUICKSTART.md),
[TaskSpec v1](task-spec-v1.md), and [Failure Capsule](FAILURE_CAPSULE.md).

## `third_party_plugin`

Use
`.github/ISSUE_TEMPLATE/third-party-plugin-evidence.yml` for a controller
plugin implemented and owned outside the RNE repository owner. Submit the exact
shared library, `rne-plugin.json`, and complete conformance report. A copied or
renamed reference plugin does not count.

Run the installed kit, replacing the platform-specific library name:

```bash
rne-asset plugin check \
  --library path/to/libcontroller.so \
  --manifest path/to/rne-plugin.json \
  --output path/to/controller-conformance.json
```

The readiness audit validates both typed files, rehashes the exact library,
checks its file name and size, and requires the negotiated controller identity
to match the manifest. See [Controller plugin SDK](PLUGIN_SDK.md).

Complete `release/external-plugin-submission-template.json` only after the
release archive, controller library, manifest, conformance report, and command
logs have their final digests. Commit the candidate and both logs to the
external repository; pass the exact 40-character revision separately so the
candidate does not hash or identify the commit that contains itself. The
candidate is a review input, never accepted evidence by itself.

After downloading the immutable bytes, a maintainer runs:

```powershell
cargo run --locked -p xtask -- external-plugin-check `
  --release-archive path/to/rne-0.2.0-TARGET.zip `
  --library path/to/controller.dll `
  --manifest path/to/rne-plugin.json `
  --report path/to/controller-conformance.json `
  --submission path/to/external-plugin-submission.json `
  --evidence-repo-dir path/to/clean-external-plugin-repository `
  --revision 0123456789abcdef0123456789abcdef01234567 `
  --output external-controller-plugin-submission.json
```

The checker rejects dirty or mismatched Git state, origin drift, unknown JSON
fields, mutable-looking URLs, nonzero commands, path escapes, symlinks,
oversized files, platform/library mismatches, digest drift, failed or malformed
conformance reports, and manifest/controller identity drift. It deliberately
does not load the untrusted shared library on the maintainer workstation;
maintainers rerun executable conformance only inside an appropriate sandbox.
The exact checker output is retained as `maintainer_report`; readiness manifest
v7 reparses it and rehashes all seven submitted artifacts, so the qualifying
gate cannot silently fall back to the older library/manifest/report-only path.

## `external_system`

Use
`.github/ISSUE_TEMPLATE/external-system-evidence.yml` for an independently
maintained `physics_backend`, `hardware_adapter`, `simulator_adapter`, or
`accelerator_adapter`. Exactly one passing physics backend, hardware adapter,
or simulator adapter is required for 1.0;
an accelerator adapter is audited ecosystem evidence but cannot satisfy that
gate.

Physics submissions include the exact implementation binary or deterministic
source bundle named by the report and the complete
`rne_external_physics_backend_conformance_report`. The public Rust kit owns all
nine vectors and tolerances. See
[External physics backend conformance](EXTERNAL_PHYSICS_BACKEND_CONFORMANCE.md).

Hardware submissions additionally include the exact TaskSpec and normalized
adapter argument list. Use `"<adapter-subject>"` where the runner normalized an
argument equal to the submitted subject path. Conformance must run against a
sandbox, simulator, process mock, or correctly isolated HIL rig with explicit
authorization:

```bash
rne-hardware-conformance \
  --adapter path/to/adapter \
  --subject path/to/tested-subject \
  --task path/to/task.json \
  --allow-hil \
  --output path/to/hardware-conformance.json \
  --adapter-arg first-normalized-argument
```

This process-protocol pass is not physical safety certification. See
[External hardware adapter conformance](HARDWARE_ADAPTER_CONFORMANCE.md).

Simulator submissions include the exact TaskSpec, normalized adapter argument
list, runtime manifest, and the world, robot-model, and adapter-config bytes in
manifest order. Run the installed fixed-step kit:

```bash
rne-simulator-conformance \
  --adapter path/to/adapter \
  --subject path/to/tested-subject \
  --runtime-manifest path/to/runtime.json \
  --task path/to/task.json \
  --output path/to/simulator-conformance.json
```

Then complete `release/external-simulator-submission-template.json` with the
official RNE release archive, adapter, TaskSpec, runtime manifest, three runtime
artifacts in manifest order, report, normalized arguments, and successful
commands. Commit that acyclic candidate and its distinct stdout/stderr logs to
the clean external repository; provide its 40-character Git revision outside
the candidate.

After downloading the immutable files, a maintainer runs:

```powershell
cargo run --locked -p xtask -- external-simulator-check `
  --release-archive path/to/rne-0.2.0-TARGET.tar.gz `
  --adapter path/to/adapter.py `
  --task path/to/task.json `
  --runtime-manifest path/to/runtime.json `
  --runtime-artifact path/to/world.sdf `
  --runtime-artifact path/to/robot.sdf `
  --runtime-artifact path/to/adapter.json `
  --report path/to/simulator-conformance.json `
  --submission path/to/external-simulator-submission.json `
  --evidence-repo-dir path/to/clean-external-simulator-repository `
  --revision 0123456789abcdef0123456789abcdef01234567 `
  --output external-simulator-maintainer-report.json
```

The checker does not execute the untrusted adapter. It validates clean Git
provenance and committed logs, rehashes every byte, reparses the TaskSpec,
runtime manifest and passing conformance report, and rebinds the fixed step,
vector widths, handshake identity, argument hash, and canonical runtime files.
Readiness manifest v7 requires this maintainer report and all twelve retained
digests before a simulator can satisfy the external-system gate.

The readiness audit rehashes every retained file, binds the handshake simulator
identity and exact fixed step to the TaskSpec and runtime manifest, and reruns
report validation. See
[External simulator adapter conformance](EXTERNAL_SIMULATOR_ADAPTER_CONFORMANCE.md).

Accelerator submissions retain the exact adapter subject, TaskSpec,
`accelerator.toml`, `runtime.toml`, normalized ordered argument list, and the
complete process-conformance report. Run the installed standalone kit:

```bash
rne-accelerator-conformance \
  --manifest path/to/accelerator.toml \
  --runtime path/to/runtime.toml \
  --task path/to/task.json \
  --adapter path/to/adapter \
  --subject path/to/tested-subject \
  --output path/to/accelerator-process-report.json \
  --adapter-arg first-normalized-argument
```

The readiness audit parses all four typed inputs, validates the complete
nine-exchange transcript, and independently rehashes all retained bytes and
the normalized JSON argument array. See
[Accelerator process protocol](ACCELERATOR_PROTOCOL.md).

## Maintainer acceptance

For every submission, a maintainer:

1. verifies external repository ownership and the exact commit;
2. downloads each immutable artifact without submitter credentials;
3. recomputes file names, sizes, and SHA-256 values;
4. reruns the corresponding typed verifier or conformance kit;
5. stages exact bytes into an external-disk readiness pack without overwriting
   prior evidence;
6. adds a reviewed manifest entry and runs `release-readiness` for an explicit
   assessment date.

Malformed, unavailable, mutable, oversized, symlinked, path-escaping, or
SHA-mismatched input is rejected. A structurally valid submission can still be
rejected after human independence review. Accepted evidence does not create a
tag or support promise, and it does not bypass the remaining stability,
hardware, signed-release, compatibility, blocker, or support gates.

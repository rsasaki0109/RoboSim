# Installed flagship reproduction quickstart

This is the bundle-local path for an independent operator to reproduce RNE's
indoor mobile-manipulation proof. It requires only the official native archive;
do not clone the RNE source repository and do not accept hands-on debugging
from an RNE repository author during a qualifying run.

Running these steps creates a **candidate submission**, not accepted evidence.
Maintainers independently download, hash, parse, and replay the retained bytes
before a readiness result can change.

## 1. Record immutable identity

Record the archive download URL, file name, byte size, published SHA-256,
release target, operating system, and a stable non-sensitive machine label.
Verify the archive's published provenance and digest before extraction. Keep
the downloaded archive unchanged.

Download the release-level `SHA256SUMS` beside the archive and verify the
archive's matching entry before extraction. The same Release page retains the
platform-specific attestation bundle and archive-install report, allowing a
different machine and later readiness audit to reverify the exact published
bytes after the workflow artifact retention window has expired.

Create a new public evidence repository owned outside the RNE repository owner.
The repository author, machine operator, and commands must be real; do not use
RNE CI, a copied report, or placeholder metadata. Do not record the final Git
revision yet; it is created only after the proof-bundle digest and candidate
JSON are fixed.

## 2. Run from the extracted bundle

Extract without flattening the top-level directory and change into that
directory. On Linux:

```bash
./bin/rne-flagship-proof flagship-proof --cross-backend \
  --measure-on "community-lab-desktop-a" --verify-installed-bundle .
test -f flagship-proof/installed-proof-report.json
test -f flagship-proof/time-to-proof-report.json
test -f flagship-proof/failure-capsule/capsule.json
```

On Windows, run the same fail-closed bundle verification and proof path:

```powershell
.\bin\rne-flagship-proof.exe flagship-proof --cross-backend `
  --measure-on "community-lab-desktop-a" --verify-installed-bundle .
Test-Path flagship-proof\installed-proof-report.json
Test-Path flagship-proof\time-to-proof-report.json
Test-Path flagship-proof\failure-capsule\capsule.json
```

All commands must exit successfully. Preserve stdout, stderr, every exact
command, its exit status, and elapsed output in files under the evidence
repository. Do not edit anything under `flagship-proof`.

`--verify-installed-bundle .` runs before output creation. It rejects missing,
extra, modified, duplicate, escaping, or symlinked members against the internal
`SHA256SUMS`, writes `installed-bundle-verification.json`, and includes that
report in both the installed proof and Failure Capsule. The 15-minute timer now
covers this complete verification rather than starting after a separate manual
checksum command.

## 3. Retain and submit exact bytes

First create an immutable proof archive containing the complete unmodified
`flagship-proof` directory and the reproduction logs. **Do not put the candidate
submission JSON in this archive**: that JSON contains the archive SHA-256, so
including it would create an impossible self-referential digest.

Copy `release/external-flagship-submission-template.json` into the external
evidence repository and replace every `null` with the measured value. Store
`reproduction.commands` and `reproduction.exit_statuses` as equally sized JSON
arrays with at least three entries and only zero exit statuses. Keep
`candidate_status` unchanged. Candidate schema v2 deliberately has no Git
revision field, because a file cannot contain the hash of the commit containing
that same file. Commit the completed candidate JSON and logs, then record that
commit's exact lowercase 40-character ID.

Use lowercase `windows` or `linux`, architecture `x86_64`, raw lowercase
64-character SHA-256 values without a `sha256:` prefix, and repository-relative
log paths. For example:

```json
"reproduction": {
  "commands": ["verify archive", "extract archive", "run installed proof"],
  "exit_statuses": [0, 0, 0],
  "stdout_log_path": "logs/stdout.txt",
  "stderr_log_path": "logs/stderr.txt"
}
```

Publish two immutable, unauthenticated downloads:

1. the unchanged official native release archive;
2. the proof archive containing the complete unmodified `flagship-proof`
   directory and reproduction logs, but not the candidate JSON.

Record the URL, file name, byte size, and lowercase SHA-256 for both downloads
in the candidate JSON before committing it. Open the installed flagship
reproduction issue linked from `docs/EXTERNAL_EVIDENCE_INTAKE.md` and paste the
same identities and commands.

Do not add credentials, personal data, robot tokens, private URLs, or
unredistributable vendor files. Do not claim that the submission has passed.

## 4. Maintainer verification boundary

The independent operator does not need an RNE source checkout. After
submission, a maintainer checks out the exact matching release tag and runs
`xtask external-flagship-check` against freshly downloaded archive,
proof-bundle, candidate, and log bytes. That checker binds the separately
supplied external repository revision, candidate, both archives, extracted
bundle, complete proof, timing report, producer executable, Rapier/MuJoCo
results, logs, and Failure Capsule into the qualifying report. Only the
subsequent readiness audit can accept the evidence.

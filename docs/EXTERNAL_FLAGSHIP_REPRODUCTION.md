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

Create a new public evidence repository owned outside the RNE repository owner.
Commit a reproduction log, then record its exact lowercase 40-character commit
ID. The repository author, machine operator, and commands must be real; do not
use RNE CI, a copied report, or placeholder metadata.

## 2. Run from the extracted bundle

Extract without flattening the top-level directory and change into that
directory. On Linux:

```bash
sha256sum --check SHA256SUMS
./bin/rne-flagship-proof flagship-proof --cross-backend \
  --measure-on "community-lab-desktop-a"
test -f flagship-proof/installed-proof-report.json
test -f flagship-proof/time-to-proof-report.json
test -f flagship-proof/failure-capsule/capsule.json
```

On Windows, verify every `SHA256SUMS` entry with
`Get-FileHash -Algorithm SHA256`, then run:

```powershell
.\bin\rne-flagship-proof.exe flagship-proof --cross-backend `
  --measure-on "community-lab-desktop-a"
Test-Path flagship-proof\installed-proof-report.json
Test-Path flagship-proof\time-to-proof-report.json
Test-Path flagship-proof\failure-capsule\capsule.json
```

All commands must exit successfully. Preserve stdout, stderr, exit status, and
elapsed output. Do not edit anything under `flagship-proof`.

## 3. Retain and submit exact bytes

Copy `release/external-flagship-submission-template.json` into the external
evidence repository and replace every `null` with the measured value. Keep
`candidate_status` unchanged. Publish two immutable, unauthenticated downloads:

1. the unchanged official native release archive;
2. an archive containing the complete unmodified `flagship-proof` directory,
   the completed submission JSON, and the reproduction log.

Record the URL, file name, byte size, and lowercase SHA-256 for both downloads.
Open the installed flagship reproduction issue linked from
`docs/EXTERNAL_EVIDENCE_INTAKE.md` and paste the same identities and commands.

Do not add credentials, personal data, robot tokens, private URLs, or
unredistributable vendor files. Do not claim that the submission has passed.

## 4. Maintainer verification boundary

The independent operator does not need an RNE source checkout. After
submission, a maintainer checks out the exact matching release tag and runs
`xtask external-flagship-check` against freshly downloaded bytes. That checker
binds the external repository revision, archive, extracted bundle, complete
proof, timing report, producer executable, Rapier/MuJoCo results, and Failure
Capsule into the qualifying report. Only the subsequent readiness audit can
accept the evidence.

# Install a 1.0 RC bundle

Robot Native Engine release bundles are native, self-contained rehearsal
artifacts for Linux x86-64 and Windows x86-64. Select the archive whose target
matches the host:

- `rne-0.1.0-x86_64-unknown-linux-gnu.tar.gz`
- `rne-0.1.0-x86_64-pc-windows-msvc.zip`

Extract the archive without flattening its top-level directory. Before running
anything, first verify that the archive itself has GitHub/Sigstore provenance
bound to this repository, then verify every extracted entry in `SHA256SUMS`:

```bash
REVISION="$(gh api repos/rsasaki0109/RoboSim/commits/v0.1.0 --jq .sha)"
gh attestation verify rne-0.1.0-x86_64-unknown-linux-gnu.tar.gz \
  -R rsasaki0109/RoboSim \
  --cert-identity https://github.com/rsasaki0109/RoboSim/.github/workflows/release.yml@refs/tags/v0.1.0 \
  --source-ref refs/tags/v0.1.0 \
  --source-digest "$REVISION" \
  --signer-digest "$REVISION" \
  --cert-oidc-issuer https://token.actions.githubusercontent.com \
  --predicate-type https://slsa.dev/provenance/v1 \
  --deny-self-hosted-runners
gh attestation verify rne_py-0.1.0-cp39-abi3-manylinux_2_*.whl \
  -R rsasaki0109/RoboSim \
  --cert-identity https://github.com/rsasaki0109/RoboSim/.github/workflows/release.yml@refs/tags/v0.1.0 \
  --source-ref refs/tags/v0.1.0 \
  --source-digest "$REVISION" \
  --signer-digest "$REVISION" \
  --cert-oidc-issuer https://token.actions.githubusercontent.com \
  --predicate-type https://slsa.dev/provenance/v1 \
  --deny-self-hosted-runners
```

Use the downloaded Windows ZIP and wheel paths in the same commands on
Windows. Verification must identify `https://token.actions.githubusercontent.com`
as issuer, this repository as source, and `.github/workflows/release.yml` as the
build workflow and the requested tag as source ref, and must reject self-hosted
builders. Replace `REVISION` with the exact 40-character commit resolved by the
verified tag. Maintainer readiness audits additionally pass the retained action
bundle through `--bundle` so the evidence pack remains independently replayable.
The committed policy is
`release/artifact-attestation.toml`; GitHub's verifier checks the signed SLSA
provenance and subject digest. Checksums remain a separate, offline integrity
layer after extraction. On Linux:

```bash
cd rne-0.1.0-x86_64-unknown-linux-gnu
sha256sum --check SHA256SUMS
./bin/rne-asset --version
```

On Windows, compare each manifest digest with `Get-FileHash -Algorithm SHA256`.
`release-report.json` records the tested commit, Rust/Cargo versions, target,
Cargo.lock digest, schema/ABI floors, supply-chain and fuzz verdicts, and every
bundle-member digest. `reproducible` is true only for a clean build whose exact
`v0.1.0` tag points to the tested commit.
The retained `Cargo.lock` also lets the installed Failure Capsule author record
the exact release graph during bundle rehearsal; when run from an external
project, the same CLI records that project's own lockfile and Git revision.
The extracted `release/rust-api-baseline.toml` records the immutable source
commit/tree and complete 31-crate manifest set used by SemVer CI; it is audit
metadata and does not require shipping either source tree in the native bundle.

## Installed workflows

The bundle includes the fixtures and validation binaries used by the release
rehearsal. From its top-level directory:

```bash
./bin/rne-asset run assets/runs/mesh_diff_drive.rne.run.toml \
  --replay-out robot.rne-replay
./bin/rne-asset replay robot.rne-replay
./bin/rne-asset failure-capsule create \
  --replay tests/golden/replays/behavior-replay-v1.json \
  --evidence assets/tasks/diff_drive_goal.task.json \
  --output failure-capsule \
  --backend external-project \
  --backend-version 0.1.0
./bin/rne-asset failure-capsule verify failure-capsule
./bin/rne-asset run assets/runs/scenario_speed.rne.run.toml \
  --replay-out scenario.rne-replay
./bin/rne-asset replay scenario.rne-replay
./bin/rne-physics-conformance --output physics-conformance.json
./bin/rne-scenario-scale --output scenario-scale.json
./bin/rne-hardware-conformance \
  --adapter ./bin/rne-hardware-mock-device \
  --adapter-arg --device-id \
  --adapter-arg installed-mock-v1 \
  --adapter-arg --expected-task-id \
  --adapter-arg rne.diff_drive.sensor_goal.v1 \
  --adapter-arg --observation-width \
  --adapter-arg 9 \
  --adapter-arg --action-width \
  --adapter-arg 2 \
  --task assets/tasks/diff_drive_goal.task.json \
  --allow-hil \
  --output hardware-adapter-conformance.json
./bin/rne-accelerator-conformance \
  --adapter ./bin/rne-accelerator-protocol-mock \
  --adapter-arg --transcript \
  --adapter-arg tests/golden/accelerators/protocol-transcript-v1.json \
  --subject ./bin/rne-accelerator-protocol-mock \
  --manifest adapters/mjx/accelerator.toml \
  --runtime adapters/mjx/runtime.toml \
  --task adapters/mjx/fixtures/free-fall-task-spec-v1.json \
  --output accelerator-protocol-conformance.json
./bin/rne-asset plugin list --path lib
./bin/rne-asset plugin check \
  --library lib/librne_plugin_example_velocity_servo.so \
  --manifest lib/rne-plugin.json \
  --output controller-plugin-conformance.json
./bin/rne-compatibility --root . --output compatibility-fixture-report.json
```

The hardware command's `--allow-hil` is safe here because the target is the
bundled deterministic process mock; do not reuse it with an unisolated physical
robot. The accelerator mock is dependency-free and exercises protocol behavior,
not GPU availability or promotion performance. Use `.exe` on Windows and
`lib/rne_plugin_example_velocity_servo.dll` as the controller library. These
commands are headless and require neither ROS2 nor a renderer. Capsule creation
refuses successful replays, existing destinations, symlinks, path escapes,
malformed known evidence, and digest mismatches.
The compatibility command reads only the retained registry and fixtures under
the extracted bundle, verifies their canonical JSON digests, runs the current
typed readers, and checks fail-closed future-schema and unknown-field handling.
Its historical matrix restores complete sensor-bearing snapshot-v1 and
snapshot-v2 artifacts as snapshot-v3 and verifies source and restored-state
digests. It also restores the legacy vectorized checkpoint v1 exactly and
requires typed rerun decisions for scenario replay v2/v3 rather than fabricating
v4 evidence. It additionally retains ancestor TaskSpec and Failure Capsule v1
artifacts exactly and reconstructs the ancestor dataset bundle from its embedded
binary shard before rerunning stream, gap, digest, and offline-evaluation checks.
The first committed protocol-v1 `ClientHello` golden is also retained with its
source commit/tree and must decode, re-encode, and negotiate exactly through the
installed current transport reader.
Git history is required only by source `release-check`.

The bundle includes the dependency-free authoring module at
`sdk/rust/rne_plugin_sdk.rs`. To prove the installed authoring path with no
registry dependencies (a Rust toolchain is required):

```bash
./bin/rne-asset plugin new release_controller --dir authoring
cargo build --offline \
  --manifest-path authoring/release_controller/Cargo.toml
./bin/rne-asset plugin check \
  --library authoring/release_controller/target/debug/librelease_controller.so \
  --manifest authoring/release_controller/rne-plugin.json \
  --output authoring/release_controller/conformance.json
```

Use the corresponding `.exe` and `.dll` names on Windows. The generated
`src/rne_plugin_sdk.rs` is byte-identical to the module included in the bundle.
C and C++ controller authors can instead include `sdk/c/rne_plugin_sdk.h`; the
installed compatibility corpus verifies its registered 64-bit ABI-v3 layout
and required symbols.

## Python ABI3 wheel

The matching `rne_py` wheel is present under `wheels/` and is also attached as a
standalone release asset. Install it into a fresh Python 3.9-or-newer virtual
environment, then run the bundled smoke:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --no-index --no-deps wheels/rne_py-*.whl
.venv/bin/python python-wheel-smoke.py
.venv/bin/python python-api-compat.py \
  --fixture sdk/python/rne_py-api-v1.json \
  --output python-api-report.json
```

On Windows the interpreter is `.venv\Scripts\python.exe`. The wheel is built
with PyO3 `abi3-py39`; no Rust toolchain is needed to install it.

## Reproduce the release rehearsal

From a clean source checkout with Rust 1.95.0, Python 3.11, maturin 1.13.3,
cargo-deny 0.20.2, and cargo-audit 0.22.2:

```bash
python3 -m pip install maturin==1.13.3
maturin build --locked --release --features extension-module \
  --manifest-path crates/rne_py/Cargo.toml --out artifacts/wheels
cargo run --locked -p xtask -- release-bundle \
  --target x86_64-unknown-linux-gnu \
  --wheel artifacts/wheels/rne_py-0.1.0-*.whl
```

`release-install-smoke --archive ARCHIVE --bundle-dir PATH --output-dir
EMPTY_PATH` independently checks `SHA256SUMS`, installs the bundled wheel, and
reruns all ten schema-v5 installed-artifact checks, including accelerator
process conformance, the compatibility corpus, and the exact Python API
manifest. Its schema-v1 outer report records the
exact archive and extracted release/checksum identities; tagged release CI
attests that report as a separate subject.

On success, the command retains the archive-bound report, replays,
conformance reports, compatibility/Python API reports, and Failure Capsule, but
removes its tool-owned wheel virtual environment and generated controller
scaffold. A failed check retains those transient directories for debugging.
Cleanup is bounded to the exact expected child directories and refuses
symlinks, regular files, or paths outside the selected output directory.

Built-in `rne_ai` scene helpers locate the release `assets/` directory at run
time by walking upward from the process working directory and executable. They
do not embed the checkout used to compile the library, so a shared Cargo target
or relocated extracted bundle cannot retain a deleted build-machine path. Run
packaged Python examples from the extracted release root (or pass an explicit
scene path when embedding RNE in another layout).

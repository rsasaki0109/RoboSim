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
gh attestation verify rne-0.1.0-x86_64-unknown-linux-gnu.tar.gz \
  -R rsasaki0109/RoboSim \
  --signer-workflow rsasaki0109/RoboSim/.github/workflows/release.yml \
  --source-ref refs/tags/v0.1.0 \
  --deny-self-hosted-runners
gh attestation verify rne_py-0.1.0-cp39-abi3-manylinux_2_*.whl \
  -R rsasaki0109/RoboSim \
  --signer-workflow rsasaki0109/RoboSim/.github/workflows/release.yml \
  --source-ref refs/tags/v0.1.0 \
  --deny-self-hosted-runners
```

Use the downloaded Windows ZIP and wheel paths in the same commands on
Windows. Verification must identify `https://token.actions.githubusercontent.com`
as issuer, this repository as source, and `.github/workflows/release.yml` as the
build workflow and the requested tag as source ref, and must reject self-hosted
builders. The committed policy is
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

## Installed workflows

The bundle includes the fixtures and validation binaries used by the release
rehearsal. From its top-level directory:

```bash
./bin/rne-asset run assets/runs/mesh_diff_drive.rne.run.toml \
  --replay-out robot.rne-replay
./bin/rne-asset replay robot.rne-replay
./bin/rne-asset run assets/runs/scenario_speed.rne.run.toml \
  --replay-out scenario.rne-replay
./bin/rne-asset replay scenario.rne-replay
./bin/rne-physics-conformance --output physics-conformance.json
./bin/rne-scenario-scale --output scenario-scale.json
./bin/rne-asset plugin list --path lib
./bin/rne-asset plugin check \
  --library lib/librne_plugin_example_velocity_servo.so \
  --manifest lib/rne-plugin.json \
  --output controller-plugin-conformance.json
```

Use `.exe` on Windows and
`lib/rne_plugin_example_velocity_servo.dll` as the controller library. These
commands are headless and require neither ROS2 nor a renderer.

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

## Python ABI3 wheel

The matching `rne_py` wheel is present under `wheels/` and is also attached as a
standalone release asset. Install it into a fresh Python 3.9-or-newer virtual
environment, then run the bundled smoke:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install --no-index --no-deps wheels/rne_py-*.whl
.venv/bin/python python-wheel-smoke.py
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

`release-install-smoke --bundle-dir PATH --output-dir EMPTY_PATH` independently
checks `SHA256SUMS`, installs the bundled wheel, and reruns all six frozen
installed-artifact checks.

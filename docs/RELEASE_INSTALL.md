# Install a 1.0 RC bundle

Robot Native Engine release bundles are native, self-contained rehearsal
artifacts for Linux x86-64 and Windows x86-64. Select the archive whose target
matches the host:

- `rne-1.0.0-rc.1-x86_64-unknown-linux-gnu.tar.gz`
- `rne-1.0.0-rc.1-x86_64-pc-windows-msvc.zip`

Extract the archive without flattening its top-level directory. Before running
anything, verify every entry listed in `SHA256SUMS`. On Linux:

```bash
cd rne-1.0.0-rc.1-x86_64-unknown-linux-gnu
sha256sum --check SHA256SUMS
./bin/rne-asset --version
```

On Windows, compare each manifest digest with `Get-FileHash -Algorithm SHA256`.
`release-report.json` records the tested commit, Rust/Cargo versions, target,
Cargo.lock digest, schema/ABI floors, supply-chain and fuzz verdicts, and every
bundle-member digest. `reproducible` is true only for a clean build whose exact
`v1.0.0-rc.1` tag points to the tested commit.

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
```

Use `.exe` on Windows. These commands are headless and require neither ROS2 nor
a renderer.

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
  --wheel artifacts/wheels/rne_py-1.0.0rc1-*.whl
```

`release-install-smoke --bundle-dir PATH --output-dir EMPTY_PATH` independently
checks `SHA256SUMS`, installs the bundled wheel, and reruns all six frozen
installed-artifact checks.

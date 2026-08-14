# Release rehearsal

`xtask release-artifacts` is the single entry point for a native release
bundle. It builds `rne-asset` and the velocity-servo controller library,
copies the compatibility/license/blocker documents and a relative-path asset
fixture, embeds the locked Cargo SBOM, builds the ABI3 Python wheel, writes a
provenance report plus `SHA256SUMS`, and runs the installed-bundle smoke.

Run the supply-chain evidence first, then assemble the native bundle:

```text
cargo run --locked -p xtask -- supply-chain
cargo run --locked -p xtask -- release-artifacts --output artifacts/release
```

Tagged CI adds `--require-tag`, which requires `HEAD` to be exactly
`v0.1.0`; manual dispatch intentionally leaves that check off.

The command emits `rne-0.1.0-<target>.zip` on Windows and
`rne-0.1.0-<target>.tar.gz` on Unix. The staging directory is retained
under the output directory for inspection. `release-smoke --bundle` accepts
either that staging directory or its parent output directory:

```text
cargo run --locked -p xtask -- release-smoke \
  --bundle artifacts/release --skip-python
```

During assembly the installed CLI runs the robot replay and OpenSCENARIO
replay fixtures. The normal (non-`--skip-flagship`) path also runs the locked
physics-conformance and release-mode 100-actor scenario-scale gates and stores
their JSON reports under `reports/` in the bundle.

The normal command installs the wheel into a fresh temporary venv and checks
that `rne_py.DiffDriveSim` advances deterministically. `--skip-python` is only
for hosts without maturin/Python; CI always runs the wheel rehearsal.

The report records the Cargo.lock digest, target/tool versions, ABI/schema
floors, every static bundle member digest, supply-chain verdict, and smoke
verdicts. Reproducibility is true only for a clean checkout whose `HEAD` is
the expected `v0.1.0` tag; a dirty/manual rehearsal is explicitly marked
non-reproducible.

For a dirty developer checkout, `--allow-dirty` and (only when the local
supply-chain report is known to be stale) `--allow-stale-supply-chain` make the
exception explicit. Release CI never uses those overrides.

For the complete M6-E gate, run:

```text
cargo run --locked -p xtask -- release-exit \
  --output artifacts/release-exit/report.json
```

This records every required stage—format/boundaries/Clippy, workspace tests,
examples, Python RL, headless sensors, fuzz-smoke, OSS parity, Behavior CI,
release contract/rustdoc, supply-chain, blockers, and bundle rehearsal—with
durations and errors. It exits non-zero unless the checkout is clean, the
expected tag is present, every stage passes, and the report is release-ready.
Manual workflow dispatch uses `--allow-untagged` only to exercise a clean
untagged checkout; the report remains non-reproducible and cannot publish.

`.github/workflows/release.yml` runs the complete `release-exit` matrix on
Ubuntu and Windows for tag pushes and manual dispatch. Tagged runs create a
draft prerelease only after both platform bundles have uploaded successfully.

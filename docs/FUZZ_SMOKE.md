# Parser and frontend fuzzing

RNE has two complementary fuzzing paths:

- `cargo run --locked -p xtask -- fuzz-smoke` is the deterministic bounded
  campaign used by CI. It exercises OpenSCENARIO and replay JSON, SDF, MJCF,
  SUMO, URDF, and frontend transport boundaries with empty, truncated,
  oversized, invalid-UTF-8, delimiter-heavy, and seeded byte-mutation cases.
  The campaign catches panics, runs twice, and pins the corpus SHA-256 in the
  `xtask` source. Its report is written to
  `artifacts/fuzz-smoke/report.json`.
- `cargo fuzz` targets under `fuzz/fuzz_targets/` provide longer sanitizer-backed
  campaigns for the same public entry points. The `fuzz` directory is an
  independent Cargo workspace so the libFuzzer dependency never enters the
  release workspace graph.

The importers reject inputs above 256 KiB before XML/JSON parsing. The framed
transport additionally enforces its absolute 32 MiB payload limit even when a
caller supplies a larger per-call limit. Fuzz targets intentionally stop before
the bounded campaign limit; the deterministic smoke campaign includes explicit
oversized cases to verify the ordinary error path.

Example longer runs:

```bash
cargo install cargo-fuzz
cargo fuzz run --manifest-path fuzz/Cargo.toml openscenario_xml
cargo fuzz run --manifest-path fuzz/Cargo.toml frontend_transport
```

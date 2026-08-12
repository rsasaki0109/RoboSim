# Parser and protocol fuzzing

RNE keeps a deterministic stable-toolchain campaign in the ordinary workspace
and matching sanitizer-backed targets under `fuzz/`. Both paths cover
OpenSCENARIO and scenario replay, SDF, MJCF, SUMO, native traffic, URDF, and the
runner/frontend framed transport and negotiation payloads.

Run the bounded release gate and write its schema-v1 evidence report with:

```bash
cargo run --locked -p xtask -- fuzz-smoke
```

The command uses a fixed seed, a 64 KiB per-case campaign limit, committed
regressions, valid format seeds, truncation and delimiter cases, invalid UTF-8,
an over-limit probe, and 32 deterministic mutations per boundary. It fails if
any parser panics, any valid seed is rejected, boundary coverage changes, or
case accounting is inconsistent. Timing and filesystem paths are not hashed.

For longer sanitizer-backed campaigns, install `cargo-fuzz` and use a nightly
toolchain:

```bash
cargo +nightly fuzz run importers
cargo +nightly fuzz run transport
```

The cargo-fuzz targets call the same boundary functions as the stable campaign
without catching panics, so crashes remain visible to libFuzzer and sanitizers.
Promote every minimized crash into `tests/fuzz_smoke/corpus/regressions.txt`
before fixing it; the next stable report then records the changed corpus and
campaign digests.

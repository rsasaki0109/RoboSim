# MJX-Warp accelerator adapter

MJX-Warp is the one selected v0.4 accelerator target. It remains experimental
and default-off. The production boundary will be an out-of-process Python
adapter that consumes TaskSpec v1 and emits the same lane metadata and replay
evidence as the CPU reference runner. No JAX, Warp, CUDA, MuJoCo, or Python type
may enter an RNE core crate.

The selection, rejected alternatives, local preflight, and promotion gates are
recorded in [ADR 014](../../docs/adr/014-mjx-warp-accelerator-selection.md).
Validate the machine-readable decision with:

```text
cargo run --locked -p xtask -- accelerator-check
```

Selection is not a throughput claim. Promotion requires a Linux NVIDIA runner,
TaskSpec shape/dtype/unit/order agreement, lane-zero outcome conformance against
CPU MuJoCo, batch measurements at 1/16/256/4096, checkpoint/failure evidence,
and an explicit unsupported-feature report.

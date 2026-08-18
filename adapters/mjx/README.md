# MJX-Warp accelerator adapter

MJX-Warp is the one selected v0.4 accelerator target. It remains experimental
and default-off. The production boundary is an out-of-process Python JSONL
adapter that consumes TaskSpec v1 and emits the same lane metadata and portable
replay-checkpoint shape as the CPU reference runner. No JAX, Warp, CUDA, MuJoCo,
or Python type enters an RNE core crate.

The selection, rejected alternatives, local preflight, and promotion gates are
recorded in [ADR 014](../../docs/adr/014-mjx-warp-accelerator-selection.md).
Validate the machine-readable decision with:

```text
cargo run --locked -p xtask -- accelerator-check
```

That command validates the manifest and the free-fall TaskSpec/MJCF binding in
Rust, then runs the dependency-free protocol and subprocess tests. The real
server always selects MJX-Warp unless a test explicitly opts into the fake:

```text
python adapters/mjx/serve.py
python adapters/mjx/serve.py --backend fake --allow-test-backend  # tests only
```

The wire operations are `probe`, `create`, `reset_lanes`, `step`, `checkpoint`,
`restore`, `close`, and `shutdown`. Requests and responses are finite canonical
JSON objects separated by newlines. See
[the protocol contract](../../docs/ACCELERATOR_PROTOCOL.md).

The native `rne-accelerator-conformance` binary can launch any compatible
external JSONL process without a shell and emit an eleven-check,
content-addressed report. `rne-accelerator-protocol-mock` provides the
dependency-free installed-bundle rehearsal subject; passing that mock proves
the process contract, not GPU availability. See the standalone command and
third-party adapter argument rules in the protocol contract.

Generate the local contract-test conformance report with:

```text
cargo run --locked -p xtask -- accelerator-conformance --backend fake --allow-test-backend --output artifacts/accelerator/conformance.json
```

Omit the fake flags on the Linux NVIDIA evidence runner. A fake report carries
`evidence_class = "contract_test"`; only the real runtime can emit
`evidence_class = "accelerator"`.

`cargo run --locked -p xtask -- accelerator-scale` measures all four widths
after 32 warm-up steps for 256 measured steps. Timing uses the host monotonic
clock only in the evidence runner; it never enters simulation state or replay
digests. The report identifies its boundary as `python_session_api` and proves
the lane-zero digest is unchanged by batch width.

The promotion runtime is pinned in `runtime.toml` and
`requirements-gpu.txt`: Linux x86-64, Python 3.12, CUDA 13, NVIDIA driver 580
or newer, JAX/JAXlib/CUDA plugin 0.10.2, MuJoCo/MJX 3.9.0, and Warp 1.12.1.
MuJoCo 3.9.0 deliberately matches RNE's CPU MuJoCo backend. Install only on the
Linux evidence runner:

```text
python3.12 -m venv .venv-mjx
.venv-mjx/bin/python -m pip install -r adapters/mjx/requirements-gpu.txt
```

The adapter probes the OS, architecture, Python, driver, installed package
versions, JAX backend, and device list before creating a session. It never
silently falls back to CPU.

The manual `MJX-Warp evidence` workflow targets only a trusted self-hosted
runner carrying the `rne-mjx-warp` label. It is not triggered by pull requests.
It installs the pins, runs widths 1/16/256/4096, verifies an injected divergence
fails, and uploads the reports plus `nvidia-smi` and `pip freeze` provenance.

Selection is not a throughput claim. Promotion requires a Linux NVIDIA runner,
TaskSpec shape/dtype/unit/order agreement, lane-zero outcome conformance against
CPU MuJoCo, batch measurements at 1/16/256/4096, checkpoint/failure evidence,
and an explicit unsupported-feature report.

The protocol, deterministic fake, runtime probe, and production free-fall
MJX-Warp code path are implemented. The production path has not been promoted:
this Windows host exposes CPU-only JAX, so GPU stepping and throughput evidence
must still be produced on the required Linux NVIDIA runner.

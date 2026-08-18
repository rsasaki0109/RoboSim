# Accelerator protocol v1

RNE accelerator adapters run outside core crates. The selected `mjx_warp`
adapter communicates over UTF-8 JSON Lines on stdin/stdout. stdout contains
protocol messages only; diagnostics go to stderr. JAX, Warp, CUDA, MuJoCo, and
Python objects never cross the boundary.
The v1 free-fall binding is f64; the production process explicitly enables JAX
x64 and rejects the session if that precision cannot be provided.

## Envelope and lifecycle

Every request contains exactly `kind = "rne_accelerator_request"`,
`schema_version = 1`, an unsigned `request_id`, and an `operation`. Every
response echoes the ID and contains either `ok = true` plus `result`, or
`ok = false` plus a stable `{code, message, details}` error. Unknown fields,
NaN/infinity, lines over 16 MiB, unknown operations, and schema mismatches fail
before simulation.

The operations are:

- `probe`: returns capability-report v1 without creating a session;
- `create`: binds an exact TaskSpec, root seed, supported batch width, and reset mode;
- `reset_lanes`: accepts only non-empty, strictly increasing, unique lane IDs;
- `step`: accepts one exact flat action per stable lane;
- `checkpoint` and `restore`: use portable batch-checkpoint v2 and replay its
  chronological step/reset operation log;
- `close`: destroys one session;
- `shutdown`: destroys all sessions and exits cleanly.

The process permits at most eight sessions, each with at most 100,000 replay
operations. It accepts only the evidence widths 1, 16, 256, and 4096. A terminal
observation is returned before auto-reset; the reset occurs immediately before
that lane's next batch step. A non-finite physics state is never serialized and
fails closed with `non_finite_state`.

## Task binding

The initial binding is `rne.physics.free_fall.mjx.v1`. Its committed TaskSpec
and MJCF live under `adapters/mjx/fixtures`. `xtask accelerator-check` parses
and validates the TaskSpec using the Rust `rne_ai` implementation as well as the
Python boundary, checks the bounded MJCF identity, and runs the JSONL subprocess
tests. The action is the exact no-op `[0.0]`; observations are position Y in
metres followed by velocity Y in metres per second.

The binding is deliberately small because it can be compared directly with
RNE's existing CPU MuJoCo free-fall conformance case. Adding another task
requires a new exact binding and evidence; the adapter never guesses tensor
order, units, bounds, or unsupported model behavior.

## Runtime and promotion

The production launcher defaults to `mjx_warp`. It lazily imports JAX, MuJoCo,
MJX-Warp, and Warp only after a request and requires JAX's selected backend to
be `gpu`. Missing packages, import failures, and CPU-only JAX return a stable
unavailable reason. The deterministic fake backend exists only for process
contract tests and requires both `--backend fake` and
`--allow-test-backend`.

Implementation does not imply availability or a throughput claim. Promotion
still requires Linux NVIDIA evidence for real stepping, CPU MuJoCo outcome
parity, lane isolation, all four widths, exact checkpoint replay, injected
divergence/Failure Capsule, and pinned package/driver versions.

`xtask accelerator-conformance` emits conformance-report v1. It records the
TaskSpec and normalized-MJCF SHA-256 digests, f64 CPU MuJoCo discrete reference,
named tolerances, actual lane-zero outcome/digest, checkpoint schema, runtime,
and a digest over all report content. The dependency-free fake can prove report
and fault-detection plumbing, but its `contract_test` evidence class cannot pass
an accelerator promotion gate.

Runtime-contract v1 pins the promotion line to Linux x86-64, Python 3.12,
CUDA 13, NVIDIA driver 580 or newer, JAX/JAXlib/CUDA plugin 0.10.2,
MuJoCo/MJX 3.9.0, and Warp 1.12.1. The MuJoCo line matches the existing CPU
backend. `probe` returns unavailable before stepping if any host or package pin
differs, if JAX does not select `gpu`, or if the expected device cannot be
enumerated.

`xtask accelerator-scale` emits scale-report v1 for widths 1, 16, 256, and
4096. It records warm-up/measured counts, elapsed nanoseconds, transitions per
second, lane-zero episode metadata/digest, runtime contract, and report digest.
The monotonic clock is used only around runner calls and cannot affect physics,
reset order, checkpoint state, or replay digests. The declared measurement
boundary is the blocking Python session API, not JSONL serialization.

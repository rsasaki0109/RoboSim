# Trust evidence quickstart

This walkthrough starts from a clean checkout, runs the complete v0.2 trust
gate, and verifies a portable Failure Capsule. The default build is CPU-only
and does not require MuJoCo, a renderer, ROS 2, SUMO, or Python.

## Prerequisites

- Git
- the Rust toolchain declared by `rust-toolchain.toml`
- a clean checkout of the source revision to be recorded

Confirm that tracked files are clean:

```bash
git status --short
```

The command should print nothing. Generated files under `artifacts/` are
ignored and do not affect the source revision.

## Generate and verify the bundle

From the repository root, run:

```bash
cargo run --locked -p xtask -- evidence
```

The command executes these producers in order:

1. the audited capability report;
2. backend-neutral physics conformance;
3. the release-mode scenario-scale source report;
4. the timing-free benchmark aggregate;
5. the committed expected-failure Behavior fixture and minimizer;
6. Failure Capsule creation and verification.

Nothing is published until every producer and verifier succeeds. The completed
directory is moved into place as `artifacts/evidence/`; a failed run leaves a
clearly named `.evidence.partial-PID` directory beside it for diagnosis.

The command refuses to overwrite an existing output directory. To repeat the
run without deleting evidence, choose another directory:

```bash
cargo run --locked -p xtask -- evidence --output artifacts/evidence-repeat
```

## Inspect the result

The stable inventory is:

```text
artifacts/evidence/manifest.json
```

It records the source commit and SHA-256 references for:

- `capability-report.json`;
- `physics-conformance.json`;
- `benchmark-report.json`;
- `failure-capsule/capsule.json`.

Scenario timing samples are stored separately in `benchmark-timings.json` and
do not participate in stable correctness hashes. The bundle retains the
Behavior JSON/JUnit outputs; the capsule carries the byte-stable minimized
replay, physics report, and benchmark report.

Verify the capsule again in a fresh process:

```bash
cargo run --locked -p xtask -- failure-capsule verify artifacts/evidence/failure-capsule
```

Then replay the minimized failure independently:

```bash
cargo run --locked -p xtask -- behavior-replay artifacts/evidence/behavior-ci/replays/unitree_g1_dex3_invalid_tray-seed-0-minimized.rne-replay
```

The replay is an intentionally failing fixture. Success means that the same
contract, seed, first violating step, and state digest were reproduced; it does
not mean the robot behavior passed.

## Stability boundaries

- Capability and benchmark reports are byte-stable for the same commit and
  committed evidence inputs.
- Physics evidence follows its embedded exact or SI-unit tolerance contracts.
- Failure Capsule metadata includes the compiler target and Rust version, so a
  capsule produced on Windows is not claimed to be byte-identical to one
  produced on Linux. Both must verify and reproduce the same declared failure.
- The aggregate manifest is a run inventory. Its hashes intentionally reveal
  any platform-specific capsule provenance.

The committed JSON examples under `tests/golden/evidence/` freeze the schema-v1
shapes used by the Rust readers and writers. Schema changes must update the
contract registry, goldens, compatibility notes, and tests together.

# Failure Capsule

Failure Capsules are portable, deterministic envelopes around a failed replay.
The envelope stores immutable run/build/backend metadata and sorted references
to replay/evidence files. It does not duplicate replay actions and does not
embed archive bytes, so the same `capsule.json` can later be transported as a
directory, archive, or remote bundle.

The `xtask` module is wired by the top-level dispatcher as:

```text
cargo run -p xtask -- failure-capsule create \
  --replay artifacts/behavior-ci/replays/failure.rne-replay \
  --evidence artifacts/behavior-ci/report.json \
  --output artifacts/failure-capsules/failure-1 \
  --backend rapier \
  --backend-version 0.22

cargo run -p xtask -- failure-capsule verify \
  artifacts/failure-capsules/failure-1
```

`create` accepts either the generic `rne_log::ReplayArtifact` JSON schema or
the `rne_ai::BehaviorReplayArtifact` JSON schema. The source replay is copied
to `replay/`, optional evidence is copied to `evidence/`, and each reference
gets a lowercase SHA-256 digest. The destination must not already exist;
creation never overwrites an existing capsule directory.

Hardware and shadow failures keep that replay requirement: a hardware trace is
not mislabeled as simulation time or a simulation action schema. Supply the
corresponding simulation/behavior failure replay and add the portable TaskSpec,
hardware session evidence, and shadow comparison as evidence:

```text
cargo run -p xtask -- failure-capsule create \
  --replay artifacts/shadow-run/failure.rne-replay \
  --evidence assets/tasks/diff_drive_goal.task.json \
  --evidence artifacts/shadow-run/hardware-session.json \
  --evidence artifacts/shadow-run/shadow-comparison.json \
  --output artifacts/failure-capsules/shadow-failure \
  --backend shadow \
  --backend-version wire-v1
```

Known hardware session, wire, shadow, mock-conformance, and external adapter-
conformance evidence retains its concrete kind and schema in artifact
references. Creation and verification both require every session, wire trace,
and shadow report to have a matching TaskSpec evidence file. Session evidence
is reconstructed from its wire trace and gateway events; normalized
hardware/simulation vectors replay through the shadow comparator to recompute
first divergence, aggregates, and verdict. An external adapter report is
accepted only when the capsule also contains bytes matching both its TaskSpec
SHA-256 and adapter-subject SHA-256; a successful handshake additionally
requires the negotiated TaskSpec identity. Missing subject evidence,
inconsistent top-level session metadata, or a tampered summary fails before a
capsule is accepted.

For the LeKiwi reference path, pass the complete
`rne_lekiwi_reference_session` output from `rne-lekiwi-session` in place of a
bare nested session. Creation and verification preserve its concrete kind,
validate the exact embedded profile and Ready device identity, reconstruct the
nested wire/gateway session, and require a separate matching TaskSpec evidence
file. A mock-prefixed device identity remains mock evidence inside the capsule.

Physics conformance reports retain their
`rne_physics_conformance_report` kind and report schema version in the capsule
instead of being flattened to generic evidence. The v0.3 fault-injection proof
generates a replay through the existing Behavior replay schema, then packages
and verifies both artifacts:

```text
cargo run -p rne_physics_conformance --features mujoco \
  --bin rne-physics-divergence -- \
  --report artifacts/physics-divergence-source/conformance-report.json \
  --replay artifacts/physics-divergence-source/divergence.rne-replay

cargo run -p xtask -- failure-capsule create \
  --replay artifacts/physics-divergence-source/divergence.rne-replay \
  --evidence artifacts/physics-divergence-source/conformance-report.json \
  --output artifacts/physics-divergence-capsule \
  --backend rapier-vs-mujoco \
  --backend-version rapier-0.22+mujoco-3.9.0

cargo run -p xtask -- failure-capsule verify \
  artifacts/physics-divergence-capsule
```

The production Rapier-vs-MuJoCo free-fall comparison remains within its named
10 cm tolerance. The diagnostic deliberately injects a 1 cm bound, records
both backend observations through the first violating step, and marks only that
fault-injection case as failed. This keeps an expected solver difference
distinct from a production conformance regression.

Generic replays must carry `final_report.failure`; successful generic replays
are not converted into failure capsules. Their fixed timestep is derived from
the recorded frame timestamps and every frame must match the same fixed-step
sequence. Behavior replay minimization provenance remains in the copied
behavior replay. Because its metadata does not include original step/action
counts, the capsule deliberately leaves `minimization` unset instead of
inventing counts.

`verify` validates `capsule.json`, requires a replay reference, rejects
absolute/parent-traversal paths, rejects symlink escapes, checks that every
referenced file exists and matches its digest, and invokes the known replay
reader/schema validator for generic and behavior replay kinds. Known physics
conformance evidence is also checked for matching kind and schema metadata. The
verifier invokes the known TaskSpec, hardware-session, LeKiwi
reference-session, wire-trace, and shadow-report validators. It does not trust
host paths recorded in the capsule.

The capsule schema is intentionally independent of transport. A future archive
or object-store adapter can package the same relative paths without changing
`capsule.json`.

The committed end-to-end fixture is generated by `cargo run --locked -p xtask
-- evidence`; see [EVIDENCE_QUICKSTART.md](EVIDENCE_QUICKSTART.md) for the clean
checkout reproduction flow.

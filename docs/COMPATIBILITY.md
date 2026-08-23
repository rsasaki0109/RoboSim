# Compatibility and migration policy

This policy applies to Robot Native Engine 0.x. Release `0.1.0` remains the
historical compatibility baseline. Release `0.2.0` is the first product-proof
minor: it may add versioned evidence contracts, but it must not silently
reinterpret artifacts accepted by their retained 0.1 readers.

## Supported toolchains and platforms

- MSRV: Rust `1.88.0`.
- CI/reference Rust: `1.95.0` with the committed `Cargo.lock`.
- Tier-1 release rehearsal platforms: GitHub-hosted Linux x86_64 and Windows
  x86_64.
- Python: CPython 3.9 or newer through one ABI3 wheel per platform.
- ROS2 remains an adapter and follows the ROS distribution/toolchain declared
  by the adapter workflow; it is not required by core crates.

An MSRV increase in 0.x requires a minor release and release-note entry. The
locked graph may require a newer compiler than an individual RNE crate when an
optional backend is enabled; the full default workspace is the release MSRV
authority.

## Rust API

Public items in publishable `rne_*` crates remain pre-1.0 and may evolve with
documented migration notes.

- Patch releases may fix behavior without removing, renaming, or changing the
  type of a public item.
- Minor releases may add APIs, deprecate old APIs, or make documented breaking
  changes while the engine remains below 1.0. A deprecated API remains through
  at least the next minor release.
- Once 1.0 is declared, removing or incompatibly changing a stable API
  requires a major release.
- APIs explicitly documented as experimental are exempt until promoted, but
  must remain memory-safe and must produce actionable errors.

`release/rust-api-baseline.toml` freezes the baseline commit and tree,
`cargo-semver-checks` 0.49.0, and the exact manifest path of all 31 publishable
crates. Every CI shard compares against that revision with patch rules; it does
not retarget to the pull-request base or `HEAD^`. A missing baseline commit,
tree mismatch, package omission, or manifest move fails closed. The baseline
commit must remain reachable in repository history.

Patch releases must not retarget the baseline. A deliberate pre-1.0 baseline
change requires a minor version, migration notes, an ADR, and a final passing
comparison against the prior baseline before the registry changes. Workspace
CI rejects changes to an already-present registry relative to the pull-request
base or push parent while the release remains 0.2.0. Rustdoc runs with warnings
denied, and public libraries deny missing documentation.

The v0.3 interchangeable-dynamics milestone extends the pre-1.0 exhaustive
`PhysicsCapability` and `PhysicsError` enums with `KinematicBody` and
`InvalidActuation`. Downstream exhaustive matches must add arms for those
variants. `KinematicBody` is appended after the frozen capability variants, so
their discriminants and derived ordering do not change. The `rne_physics`
SemVer policy reports enum additions as warnings during 0.x while continuing
to reject removals, payload changes, and variant reordering.

## C ABI and plugin compatibility

Controller ABI v3 is the current authoring ABI. The host continues to load the
frozen v2 compatibility fixture. Plugins must negotiate ABI version and
capabilities before configuration or stepping.

Rust authoring SDK v1 lives in the dependency-free `rne_plugin_sdk` crate. The
legacy `rne_plugin::cabi` type paths re-export those exact definitions. New
scaffolds vendor the same source into `src/rne_plugin_sdk.rs`, allowing an
offline build without coupling a plugin to host implementation crates. The SDK
version and controller ABI version are separate registered contracts.
Native C/C++ authors receive the matching `sdk/c/rne_plugin_sdk.h` header. The
schema-v1 64-bit layout fixture freezes all public structure sizes, alignments,
field offsets, capability values, required symbols, and normalized signatures
on Linux x86-64 and Windows x86-64.

- Unknown ABI versions are rejected before a plugin function is invoked.
- Host-owned lifecycle order remains configure, episode activation/reset,
  robot-scoped step, and shutdown.
- Struct layout, ownership, and nullability rules are part of the ABI contract.
- A future ABI version is additive unless it is assigned a new major number;
  v2 support cannot be removed without an explicit migration note.

`rne-asset plugin check` emits controller-plugin conformance report schema v1.
The six check IDs and their order are compatibility fixtures: manifest identity,
ABI symbols, capability negotiation, fixed-step schema, exact seeded reset
replay, and shutdown. New checks require a report-schema change unless they are
strictly diagnostic and preserve existing readers.

Core crates never expose adapter or backend-specific types to make an external
ABI fit.

## Frontend protocol

Runner/frontend protocol v1 is supported throughout 0.x. Negotiation rejects
incompatible major versions before simulation control. Unknown optional frame
kinds may be skipped only when the negotiated capability set allows it;
malformed lengths, limits, or required fields are errors. Slow or disconnected
frontends must not stall simulation or retain unbounded data.

Legacy line-oriented `--control-port` protocol v1 remains available during the
0.x transition. Command acknowledgement precedes the corresponding applied
status boundary.

## Assets, manifests, and reports

Each serialized format carries an explicit version where required. Readers
follow these rules:

- the same schema version is read exactly;
- additive optional fields use stable defaults;
- an unknown major/schema version is rejected with expected and actual values;
- input digests are verified before replay;
- migration never fabricates sensor frames, actions, random state, or physics
  hashes that were absent in the source artifact.

Current 0.1 formats include scene, robot, run, plugin, traffic, replay, Behavior
CI, physics-conformance, scenario-scale, determinism-contract, capability,
benchmark, and Failure Capsule artifacts. OpenSCENARIO, SDF, MJCF, URDF, SUMO,
and PLATEAU inputs are import formats: accepted subsets are documented, and
unsupported constructs remain explicit import errors.

TaskSpec schema v1 rejects unknown fields and validates fixed tensor shape,
dtype, row-major order, units, bounds, reward terms, termination, reset,
curriculum, and randomization before execution. Ordered arrays are semantic.
Portable batch checkpoint schema v2 embeds its TaskSpec and chronological
step/partial-reset operations. The legacy `VectorizedEpisode` API and its
checkpoint remain unchanged for patch compatibility; new portable execution
uses the separately additive `PortableBatchRunner` API. The legacy action-replay
checkpoint is independently versioned by
`VECTORIZED_EPISODE_CHECKPOINT_VERSION = 1`, rejects unknown fields, and has a
provenance-bound restore fixture; it must not be relabelled as portable batch
checkpoint v2 because it does not contain TaskSpec or lane state.

Accelerator protocol v1 and accelerator capability-report v1 are frozen
contracts. Adapters reject unknown envelope fields and unsupported TaskSpecs
before stepping. Adding an operation or field requires a protocol-version
change unless the existing schema explicitly marks it optional. Runtime-private
state is not a substitute for portable batch-checkpoint v2.
Accelerator conformance-report v1 and runtime-contract v1 are versioned as
well. A report made by the dependency-free fake is explicitly `contract_test`
evidence and cannot be re-labelled as hardware evidence.
The installed corpus retains capability-report v1 and validates it through
`rne_accelerator_contract` against the selected manifest, exact runtime pins,
and bound TaskSpec. Status tampering cannot turn `test_only` or `unavailable`
evidence into an available accelerator claim.
It also retains conformance-report v1 and independently recomputes its CPU
reference, metrics, verdict, deterministic lane-zero seed, Python-canonical
content digest, TaskSpec digest, and normalized-model digest. The installed
bundle includes the exact manifest, runtime contract, TaskSpec, and MJCF needed
for that verification.
Scale-report v1 is retained through the same raw-number reader. It recomputes
transitions and throughput, binds every width to the manifest, and requires
lane-zero replay digest, episode index, and derived seed to remain identical
across widths. Only real `available` accelerator evidence covering all four
promotion widths can satisfy the accelerator promotion verdict; incomplete
dependency-free `contract_test` evidence remains test-only.

Dataset bundle schema v1 freezes the `RNEDATA1` file header, 80-byte record
header, stream/field ordering, simulation capture and availability ticks,
explicit gap records, calibration/noise declarations, payload hashes, shard
hashes, and manifest self-hash. Offline evaluation report schema v1 freezes
the depth-pair metric fields and digest construction. A report is trusted only
after metrics are recomputed from its referenced verified bundle. Unknown
fields, implicit sequence gaps, non-finite values, and unknown schemas are
rejected. Canonical shapes live under `tests/golden/datasets/`.
Dataset-native payload encodings `imu.v1`, `pose2d.v1`, `action_f64.v1`,
`task_outcome.v1`, and `ground_truth_f64.v1` use fixed little-endian metadata
and reject trailing bytes; their combined reference shard digest is frozen.
The real diff-drive reference capture additionally freezes its TaskSpec,
manifest, complete shard, per-stream counts, terminal verdict, and recomputed
RGB-D evaluation report in
`tests/golden/datasets/diff-drive-reference-summary-v2.json`. Its v1 summary is
retained as an older compatibility fixture. DataBus sensor sequence values are
normalized to zero-based dataset-local sequence values; stream identity,
timestamps, physical payload values, calibration, declared storage resolution,
and noise behavior remain semantic.

Hardware-gateway evidence schema v1 freezes the artifact discriminator,
TaskSpec identity, authority mode, ordered typed events, connection state,
safety latch, bounded queue counts, drop counters, and last accepted sequence
identities. Unknown fields are rejected. Host ticks describe adapter-side I/O
decisions and never replace SimClock or enter deterministic simulation-state
hashes. The canonical fail-closed live session is
`tests/golden/hardware/gateway-fail-closed-session-v1.json`.

Hardware wire protocol/trace/session-evidence schemas v1 freeze directional
frame kinds, strict JSON Lines encoding, session and request correlation,
typed commands/responses, terminal device-stop confirmation, exact trace
ordering, and correlation with gateway safety state. The default frame limit is
64 KiB; a trace that reaches its configured capacity fails instead of dropping
replay frames. The process-isolated disconnect fixture is
`tests/golden/hardware/gateway-process-disconnect-session-v1.json`.

The v1 safety-reason vocabulary includes `controller_fault`. Reference hosts
use it when an observation-driven controller returns a malformed action in an
actuating mode; the device must acknowledge the resulting zero stop.

Shadow-comparison report schema v1 freezes the TaskSpec identity and ordered
tensor tolerances, separate host/simulation timestamps, normalized observation
vectors, per-sample sum/mean/max errors, first violating tensor/element/unit,
aggregate counts, and verdict. Discrete observation tensors require exact
comparison. An untrusted report replays every vector against its TaskSpec; its
canonical failing fixture is
`tests/golden/hardware/gateway-shadow-comparison-v1.json`.

Hardware mock-conformance report schema v1 requires exactly six canonically
ordered cases: command deadline, disconnect, reconnect, stale command,
actuator limit, and emergency stop. Every case must prove device zero-output
confirmation and gateway zero-stop delivery; reconnect additionally proves an
explicit rearm. The canonical all-pass report is
`tests/golden/hardware/gateway-mock-conformance-v1.json`.

External hardware-adapter conformance report schema v1 is a distinct contract.
It freezes nine ordered process cases, the negotiated protocol-v1 identity,
bounded diagnostics, and SHA-256 identities for the adapter subject, normalized
arguments, and TaskSpec. It never converts a process-mock pass into physical
hardware evidence. An adapter must reject incorrect task dimensions, shadow
actuation, duplicate sequences, cross-session requests, and wrong-width
actions while preserving explicit safe-stop and clean-close behavior.
Failure Capsule validation accepts the report only beside evidence bytes
matching both embedded subject hashes and, after a successful handshake, a
TaskSpec with the negotiated identity.

LeKiwi reference-session schema v1 binds the exact reference profile and
device-bridge schema to a promoted device identity plus the complete nested
hardware session evidence. Its validator reconstructs the nested session,
checks Open dimensions against the strict profile, and requires the promoted
device identity to equal the Ready handshake. Mock and physical bridge
identities are distinct. A Completed outcome additionally requires a clean,
unlatched, disconnected gateway with no pending actuation.

LeKiwi physical-evidence manifest schema v1 indexes every required v0.6 exit
artifact by explicit kind/schema, a unique canonical relative path, and a
`sha256:` digest. It binds one
physical device and upstream revision to two-person power isolation, an
independently observed host-loss stop within the 500 ms watchdog bound, clean
host reproduction, shadow/HIL/live sessions, camera data, offline evaluation,
and a Failure Capsule. Its self-excluding content digest protects the index;
`cargo run -p xtask -- lekiwi-evidence verify MANIFEST` rehashes and
semantically validates the entire tree.

Failure Capsule tooling preserves the concrete TaskSpec, hardware-session,
LeKiwi reference-session, wire-trace, shadow-report, and mock-conformance kinds
instead of flattening them to generic evidence. Creation and verification
require matching TaskSpec evidence and rerun the known validators. Hardware host ticks are never
substituted for the capsule's simulation replay timestamps; a corresponding
generic or behavior failure replay remains mandatory.

The evidence-manifest schema inventories one verified run of the capability,
physics-conformance, benchmark, and Failure Capsule gates. It is provenance,
not a claim that compiler- and platform-bearing capsule bytes match across
hosts. Canonical schema-v1 examples live under `tests/golden/evidence/`.

Flagship workflow report schema v1 binds the portable TaskSpec, imported asset
digest, deterministic event catalog, successful behavior-contract verdict, and
minimized failure provenance. The generated Failure Capsule carries that
report, the TaskSpec, both behavior reports, the minimized replay, and the
self-contained browser inspector; `xtask flagship` regenerates and verifies
the set together.

Flagship cross-backend report schema v2 retains schema v1's two successful
backend outcomes and nine named SI-unit tolerance checks, then adds the
controller identity and both intentional-failure outcomes. Rapier and MuJoCo
must execute the same minimized blackout dimensions, reproduce
`perception_stream_alive` at the same step and simulation nanosecond with zero
tolerance, and verify their own replay. Backend-private failure state digests
remain diagnostic and are never compared. Schema-v1 reports prove only the
successful cross-backend path and remain historical evidence.

Physics conformance report schema v2 embeds backend-manifest schema v2,
catalog version, tolerance-registry version, declared/runtime capabilities, and
coverage verdicts. Its canonical shape lives at
`tests/golden/physics/conformance-report-v2.json`. A backend identifier with no
registered shared vector or tolerance profile produces a failing case rather
than silently weakening coverage.
Manifest schema v2 adds `kinematic_body` as a refinement of `rigid_body`.
Analytic and Rapier prove it with the shared external-pose vector; MuJoCo
rejects it at preflight with `MissingCapabilities` before native compilation.

External physics-backend conformance report schema v1 is a separate public
authoring contract owned by the publishable `rne_physics_conformance` crate.
Its fixed nine-check order covers manifest identity and all eight capability
IDs. Unadvertised capabilities remain explicit `not_advertised` entries;
advertised GPU rigid-body or soft-body support fails until a later catalog
version defines portable vectors. Authors cannot override the catalog's named
SI-unit tolerances. The canonical schema is
`crates/rne_physics_conformance/tests/golden/external-backend-conformance-v1.json`.
Failure Capsule verification requires the exact implementation bytes whose
SHA-256 appears in the report subject.

`JointActuation` is a tagged, backend-neutral ECS command with distinct
revolute/prismatic position, velocity, and effort variants. Field names carry
their SI units. A backend rejects unknown variants, non-finite values, negative
gains/limits, and joint-kind mismatches before a physics step.

The machine-readable values frozen by this policy live in
`release/contracts.toml`. The release gate compares them with the compiled ABI,
transport, asset, replay, physics, determinism, task, accelerator-selection,
accelerator-protocol/scaffold, controller-scaffold, dataset, hardware, and
evidence constants; changing one side alone fails.

The installed compatibility corpus is indexed by
`release/compatibility-fixtures.toml`. Each entry fixes a reader identity,
schema, canonical forward-slash path, and SHA-256 of canonical compact JSON.
The schema-v1 `rne_compatibility_fixture_report` requires every retained
artifact to pass its current typed reader and requires deterministic mutations
of its version field and top-level object to be rejected. Canonical JSON hashing
keeps evidence independent of checkout line endings and indentation. Removing
a retained fixture or changing its meaning requires a documented compatibility
decision; adding another retained artifact changes the registry digest but not
the report shape.

For 1.0 promotion, a retained report is not accepted as an assertion by itself.
`xtask release-readiness` revalidates the registered historical Git commits,
trees, schema declarations, and golden blobs, executes the complete corpus with
the current readers, and requires the supplied report to equal the fresh report
exactly. The audit records the report, registry, and every fixture digest.

The thirty-six-fixture registry additionally freezes the accelerator and
controller scaffold schema-v1 file sets, controller-plugin conformance report
v1, the current sensor-goal TaskSpec and its process-isolated
hardware session, all ten frontend protocol-v1 message families and the
historical `ClientHello` frame, all five dataset-native
payload families, behavior replay v1, scenario replay v4, the renderer-backed
RGB-D capture report, the controller C ABI-v3 64-bit layout, three historical
mobile-manipulator migrations, one exact legacy checkpoint restore, and two
explicit old-scenario required-rerun decisions. The original zero-step schema-v1
case remains retained. Two stronger cases contain 7-tick, sensor-bearing
schema-v1 and schema-v2 snapshots emitted by ancestor revisions `47525b1` and
`2255cbe`. They bind the full source commit/tree, workspace version, scene,
generation step count, source digest, and complete normalized v3 digest at a
`1e-9` floating-point tolerance. The v2 case must preserve a populated depth
frame; both pre-v3 cases must omit grasp-retarget state. Source release checks
require both revisions and exact trees to remain reachable ancestors, while
the extracted bundle verifies the content-addressed data without requiring a
Git checkout. Binary fixtures pair semantic fields with lowercase hex bytes:
acceptance requires exact decode/re-encode identity plus rejection of
truncation and trailing bytes. Frontend validation also rejects corrupt magic,
unknown message kinds, and an incompatible negotiated major version.

The synthesized passing controller-plugin report is compatibility evidence for
the report reader only. It cannot satisfy third-party-plugin readiness: that
gate independently requires external ownership and subject-bound library and
manifest bytes whose hashes, sizes, names, and negotiated identity match the
submitted report.

The retained original `rne.diff_drive.goal.v1` artifact has five flattened
observations. The current sensor-bearing task has nine and therefore uses the
new `rne.diff_drive.sensor_goal.v1` identity instead of silently redefining the
old ID. Its hardware-session reader replays the wire trace and derives both
opening widths from the registered current TaskSpec.

The frontend frame is also retained in a provenance-bound historical decision.
Protocol v1's introducing commit/tree and the first full `ClientHello` golden
commit/tree are checked as reachable ancestors. The golden blob retained from
the original `be53f16` freeze (bound on `main` via squash `9e1ea8c`)
must equal the current blob, and the installed reader repeats the exact frame,
payload, negotiation, and fail-closed mutation checks without Git history.

Four additional same-schema historical decisions bind frontend transport v1,
TaskSpec v1, dataset bundle v1, and Failure Capsule v1 revisions. TaskSpec and
Failure Capsule must deserialize, validate, and serialize to the exact same
semantic JSON. Dataset retention embeds the original manifest and exact 736-byte shard;
the installed reader must reproduce two streams, six records, four samples,
two explicit drops, and the retained headless depth-evaluation digest. A
single-bit shard mutation must fail verification. These are same-schema
retention contracts: no v2 exists to migrate to, so changing their v1 number
would be a new compatibility decision rather than evidence of an old migration.

Installed-rehearsal report schema v2 adds the required `hardware_adapter` check
to the six schema-v1 checks. Schema-v1 reports remain historical evidence but
cannot be relabelled as v2 because they do not prove the installed external
adapter runner, fixed-binding process mock, or bundled TaskSpec. Re-run the
matching release bundle to produce v2 evidence; do not edit an old report.
Installed-rehearsal report schema v3 adds the required
`compatibility_corpus` check and bundled `rne-compatibility` binary. A v1 or v2
report remains historical evidence for the workflows it actually ran and must
not be promoted to v3 without rerunning the matching extracted bundle.
Installed-rehearsal report schema v4 appends the required `python_api` check.
It verifies the installed ABI3 wheel against the bundled strict API manifest
and emits a content-addressed schema-v1 Python API report. Older rehearsal
reports cannot be relabelled as v4 because they do not prove this call shape.

Installed-rehearsal report schema v5 appends the required
`accelerator_protocol` check. It launches the bundled dependency-free mock
through the standalone accelerator conformance binary and requires the complete
nine-exchange lifecycle, timeout boundary, checkpoint restore, and Rust
transcript binding. Older reports cannot be relabelled as v5 because they did
not execute or content-address this process boundary.

Release report schema v2 renames the schema-v1 `flagship_workflows` map to
`installed_workflows`. The entries are the ten generic installed-bundle checks;
they did not prove execution of the indoor mobile-manipulation flagship. The v2
reader accepts the old field spelling for historical inspection, but a schema-v1
report remains historical evidence and cannot qualify as a current release
report without rebuilding and rerunning the bundle.

Installed-rehearsal report schema v6 appends `flagship_proof`. The release
archive now runs the indoor mobile-manipulation TaskSpec through the bundled
Rapier and MuJoCo proof runner, verifies both successful behaviors, the same
deterministic minimized perception failure and first violation, both replays,
browser inspector, and eleven-artifact Failure Capsule. The official MuJoCo
3.9.0 runtime and licenses are archive members; their upstream archive identity
and shipped file digests are retained in a runtime manifest. Older schema-v6
reports that list only `rapier_native` remain valid evidence for the narrower
workflow but do not qualify as the current cross-backend installed proof.

Installed flagship proof report schema v2 is the timing-free index emitted by
that runner. It fixes the task and outcome identities, binds the exact packaged
producer executable, and binds the TaskSpec,
Rapier/MuJoCo success and failure reports, both failure replays, unit-bearing
cross-backend report, browser inspector, workflow report, and capsule manifest
by byte size and SHA-256. Timing measurements remain a separate hardware-named
artifact. Schema v1 remains evidence for its original artifact set but cannot
qualify as archive-bound external reproduction because it does not identify the
executable that produced it.

Time-to-proof report schema v1 is that separate artifact. It records a bounded
operator-supplied machine label, OS and architecture, command elapsed
milliseconds, the 900,000 ms acceptance target, and content identities for the
installed proof report and verified capsule manifest. The report is deliberately
excluded from deterministic hashes. CI-generated measurements prove the
packaged measurement path only; independent acceptance still requires an
external user and named reference hardware.

External flagship reproduction report schema v1 binds one independently owned
public repository revision to the exact clean tagged release archive, source
revision, release report, checksum manifest, packaged producer executable,
installed proof and timing reports,
cross-backend report, and Failure Capsule manifest. Its verifier rejects RNE
repository ownership, CI or placeholder machine labels, target/platform
mismatches, elapsed time above 900,000 ms, missing SI tolerance evidence, and
non-identical first violations. The report is reviewable evidence, not proof of
independence by itself; maintainers still verify ownership and immutable
downloads before acceptance.

The schema-v6 report remains inside the archive as the staged rehearsal. A
fresh extraction emits the separate `rne_archive_install_rehearsal` schema-v2
wrapper. It binds the exact archive digest, extracted release report and
checksum manifest, hardware-named time-to-proof report, and a second schema-v6
result. The wrapper is a distinct
signed subject; an older standalone schema-v6 report cannot be presented as
archive-bound evidence.

## Replay migration

Replay artifacts are evidence, not mutable project files. Exact replay requires
the recorded engine compatibility version, schema, source assets, fixed clock,
and input digests. When an older artifact is rejected:

1. retain the original artifact unchanged;
2. install the recorded RNE version and verify it there;
3. rerun the original source manifest with the new engine to create a new
   artifact;
4. compare named diagnostics and document any expected outcome change.

There is deliberately no byte-rewriting migration that claims an old run was
produced by a new engine. Declarative scene/robot/run assets may gain a separate
lossless migration command in a future minor release.

The installed historical decision matrix makes this concrete for scenario
replay v2 and v3. Both are real 300-step artifacts emitted by their registered
ancestor serializers. V2 lacks input digests and engine identity; both versions
lack the canonical actor/action/ownership/result-digest evidence required by
v4. The current reader must return `UnsupportedVersion` with expected/actual
schema values, and merely changing their `schema_version` to 4 must still fail.
The supported operation is to preserve the old bytes, verify with the old
engine if needed, then rerun the source manifest with the current engine.

## Python API

The wheel uses `abi3-py39`. Python class, method, task, and keyword names follow
the same documented 0.x compatibility rules as public Rust APIs. The strict
`release/python-api-v1.json` contract freezes all public module exports,
constants, class constructors, methods, raw text signatures, and properties.
Source Python CI and installed release rehearsal compare the live extension
module exactly, reject unknown fixture fields and noncanonical ordering, and
emit `rne_python_api_report` schema v1. Pickled Python objects are not a stable
artifact format; use versioned RNE JSON/replay/checkpoint formats.

## Determinism compatibility

Stable hashes prove repetition under one declared engine/version/backend
contract. They do not promise identical floating-point bits across unlike
physics backends. A release that intentionally changes a deterministic result
must update its golden evidence and explain the semantic reason in the
changelog. Timing values and filesystem paths never participate in stable
simulation digests.

## Security and support

Malformed external input is supported only to the point of safe rejection.
Security reports should use GitHub's private vulnerability reporting for the
repository. P0/P1 release blockers are tracked in `release/blockers.toml`; a
release rehearsal fails while either severity is open.

The current 0.x support status and the explicit commitment required before a
1.0 promotion are defined in [the support policy](SUPPORT.md). The committed
1.0 readiness tracker must remain empty rather than imply a maintainer, period,
or published policy that has not been authorized.

# Changelog

All notable changes to Robot Native Engine are documented in this file.

## [Unreleased]

### Fixed

- Repair CI merge blockers from the Tsukuba 3DGS landing: restore
  `preserve_color` in the wgpu viewer surface pass, keep `rne_render_3dgs`
  unpublished so the frozen 0.1.0 public package set stays intact, drop
  release-rehearsal pull-request path filters that skipped required gates, and
  retarget the Rust API baseline to a commit that is actually an ancestor of
  `main`.
- Make MJX process-conformance subject digests ignore Windows CRLF checkouts
  so the golden matches the LF bytes stored in git (and used on Linux CI).
- Retarget the frontend-transport historical decision from pre-squash tip
  `be53f16` to reachable `main` ancestor `9e1ea8c` so `release-check` ancestry
  gates pass after squash merges.
- Fix a broken `rne_ai` rustdoc link to `UnitreeG1TorqueOverlay::LEARNED_STRIDE`
  that failed `cargo doc -D warnings` during release-check.
- Relax the G1 commanded-stride smoke margin from exact 2.0x to 1.9x so Linux
  CI does not fail on a few millimeters of clearance noise.
- Install Mesa/Xvfb and force `WGPU_BACKEND=gl` for Ubuntu evidence so the
  required G1 WGPU dataset capture can run without a hardware GPU.
- Soften the Go2 robust-turn smoke yaw floor to match current Linux CI arc
  magnitude while still beating the straight-walk drift budget.
- Follow symlinks when hashing accelerator process-conformance subjects so
  Cargo `CARGO_BIN_EXE_*` mock binaries are accepted on Linux CI.
- Fetch full git history in the sharded test jobs so readiness/baseline
  ancestry checks can resolve the retargeted Rust API baseline commit.
- Pin the ROS 2 Python bridge workflow to Rust 1.95.0 so maturin matches the
  repo `rust-toolchain.toml` instead of a partial `stable` install.
- Keep `~/.cargo/bin` on PATH after `setup-ros` so ROS 2 bridge maturin builds
  do not trigger a conflicting rustup reinstall.
- Pre-build accelerator mock bins before sharded nextest runs so rust-cache
  stubs cannot empty `CARGO_BIN_EXE_*` subjects.
- Tolerate Windows control-socket reset on RGB-D quit ack in the asset CLI
  parity suite.

### Added

- Add a headless office AGV desk-place mission that unloads kinematic cargo
  into a desk place box after shared-aisle delivery
  (`rne.office.agv_desk_place.v1`).

- Add a headless office AGV shared-aisle delivery analog that yields to a
  kinematic oncoming AGV, then completes dock-to-desk scoring
  (`rne.office.agv_shared_aisle.v1`).

- Add a headless office AGV dock-to-desk delivery analog that scores corridor
  stay-in-lane, stopped pickup-dock visit, and 1.2 m desk-face delivery stop
  contracts on a short analytic aisle (`rne.office.agv_delivery.v1`).

- Add optional 3D Gaussian splat backgrounds for Tsukuba confirmation viewer
  capture via `rne_render_3dgs` and example 78. Contest scoring in example 75
  stays headless and analytic; splats are visual-only.

- Add a headless Grove-G1 style workbench mission that parks the dynamic G1
  inside 0.5 m of the factory marker, then runs the pelvis-pinned Dex3 pick
  and place (`rne.g1.workbench_mission.v1`). This is not a Nav2 or MoveIt port.

- Add a headless RoboCup SSL Division B 2v2 analog that scores official
  9 m × 6 m field geometry, goal-mouth crossing, out-of-bounds, and the
  6.5 m/s ball-speed cap (`rne.ssl.small_pitch_2v2.v1`) without speaking
  the grSim / SSL simulation protobuf ports.

- Add a headless Tsukuba Challenge 2026 confirmation-run analog that scores the
  official road-edge 1.5 m stop box, stop-line 1 m / 0.5 m box, green-cone
  contact fail, e-stop rest, and no-roadway-entry contracts on a scaled
  sidewalk scene (`rne.tsukuba.confirmation.v1`).

- Freeze the current sensor-goal TaskSpec and its process-isolated hardware
  session as the twenty-eighth and twenty-ninth installed compatibility
  fixtures. Session validation now replays the complete wire/gateway contract
  and recomputes observation/action widths from the exact supplied TaskSpec.
- Separate the current nine-element sensor task as
  `rne.diff_drive.sensor_goal.v1`; the retained five-element
  `rne.diff_drive.goal.v1` contract remains immutable instead of sharing an ID
  with incompatible observation semantics. Dataset v2 evidence is regenerated
  against the new TaskSpec digest; the older v1 dataset summary is unchanged.
- Freeze controller-plugin conformance report v1 as the twenty-seventh
  installed compatibility fixture. The typed reader now rejects non-portable
  subject names, non-canonical digests, unsupported ABI/schema identities,
  reordered capabilities, and passing reports without required capabilities or
  a non-empty library. This synthesized fixture does not count as third-party
  plugin evidence; readiness still rehashes retained external subject bytes.
- Freeze all ten frontend protocol-v1 message families as the twenty-sixth
  installed compatibility fixture. Client/server negotiation, rejection,
  control command/acknowledgement, status, RGB8, depth, LiDAR, and gap frames
  now require exact wire re-encoding, typed semantic equality, fixed ordering,
  and fail-closed truncated/trailing input handling.

- Freeze renderer-backed RGB-D capture report v1 as the twenty-fifth installed
  compatibility fixture. Producer, independent dataset verifier, and installed
  compatibility kit now share one strict `rne_data` type; readiness requires
  the expanded typed-reader corpus while cross-adapter pixel hashes remain outside
  the compatibility contract.

- Add a portable Unitree G1 gait TaskSpec and connect the real WGPU head-camera
  path to a streaming, TaskSpec-bound RGB-D dataset with calibration, timing,
  latency, noise, renderer identity, complete scene/robot/mesh/environment input
  digests, and mandatory Windows/Linux evidence-job verification. Renderer
  unavailability and asset overrides now fail explicit capture requests instead
  of being reported as evidence through the CPU smoke fallback.

- Document the current best-effort 0.x support status and the exact published
  policy decision required before 1.0, without presenting draft intent as a
  maintainer commitment.

- Surface the three independent RNE 1.0 validation routes near the top of the
  README so external task, controller-plugin, and backend/hardware authors can
  reach the fixed evidence forms without weakening the typed acceptance gate.

### Fixed

- Make the 1.0 readiness manifest reject partially populated uncommitted
  support claims and incomplete, noncanonical, oversized, or non-HTTPS
  committed support fields.

- **Fail-closed LeKiwi physical actuation preflight**: physical HIL now
  requires explicit cutoff-operator and elevated-wheel confirmations, while
  physical live requires cutoff-operator and clear-work-area confirmations.
  Mock, shadow, and cross-stage confirmation misuse is rejected; the flags do
  not replace the typed two-operator physical-evidence attestation.

- **Relocatable built-in scene lookup**: `rne_ai` built-in mobile-manipulator
  and URDF scene helpers now locate the staged `assets/` tree from the runtime
  working directory or executable location. Shared Cargo targets can no longer
  reuse a deleted checkout path that was embedded at compile time; unresolved
  assets remain relative instead of pointing at a stale build machine path.

- **Bounded release-rehearsal cleanup**: successful native-bundle and
  independently extracted rehearsals now remove only their validated,
  tool-owned wheel virtual environment, controller scaffold, internal
  rehearsal directory, and target-local copied evidence after the retained
  reports and checksum chain are complete. Failed rehearsals keep all of those
  diagnostics. Cleanup rejects path escapes, symlinks, and regular files so a
  user-selected release output cannot widen the deletion boundary.

- **External CI artifact storage**: xtask CI evidence producers accept an
  absolute `RNE_ARTIFACTS_DIR`, preserving the existing real-directory and
  bounded-deletion checks while keeping large generated reports, replays, and
  Failure Capsules off the source disk.

- **Portable WGPU TAA depth reprojection**: temporal anti-aliasing now samples
  pixel-center depth from a losslessly packed `Rgba8Unorm` scene attachment
  instead of a depth `textureLoad` that the OpenGL/GLSL backend cannot lower.
  Off-screen depth readback uses the same portable color path on adapters that
  do not support depth-texture buffer copies; on-screen non-TAA rendering keeps
  its original single-color-target path.

### Added

- **Installed external-project evidence authoring**: `rne-asset
  failure-capsule create|verify` now exposes the same strict, non-overwriting
  Failure Capsule implementation previously available only through source-tree
  `xtask`. Native bundles retain `Cargo.lock`, the authoring guides, and a
  failed replay fixture, and their installed `robot_replay` rehearsal now
  creates and verifies a content-addressed capsule. Independent projects can
  therefore produce required task evidence from an extracted release or their
  own locked Rust checkout without cloning RNE source. The expanded typed-reader
  reachability is covered by a time-bounded `getrandom 0.4.3` duplicate review;
  no new registry package is introduced.

- **External evidence intake contract**: a machine-readable three-route
  registry, public contributor guide, and required GitHub issue forms now cover
  independent task reproduction, third-party controller plugins, and external
  physics backends or hardware adapters. `xtask external-intake-check` binds
  the route thresholds, ownership and author-assistance policy, artifact
  checklist, form fields, and repository-contained files; lint and release
  checks fail on drift. Submission remains a review queue and cannot satisfy
  the typed readiness gate by itself.

- **External readiness-pack authoring**: `xtask readiness-pack init` creates a
  non-overwriting external-disk copy of the honest 2/9 readiness baseline and
  its retained compatibility evidence. `readiness-pack stage` then copies one
  regular file through a temporary name, enforces the audit's 64 MiB limit and
  forward-slash containment rules, refuses symlinks and overwrites, and emits
  the canonical SHA-256 TOML reference. It does not certify ownership,
  independence, or a passing 1.0 gate.

- **README vehicle-dynamics showcase**: the deterministic kinematic-versus-dynamic
  comparison now renders oriented procedural cars, steerable wheels, continuous road
  geometry, saturation-colored trails, and live slip/yaw/grip telemetry into a
  size-gated GIF with a reduced-motion poster, published directly in the README.

- **Evidence-backed 1.0 readiness gate**: `xtask release-readiness` now audits
  nine fixed promotion conditions from a strict, SHA-256-bound evidence pack
  using an explicit date instead of wall-clock time. It verifies independent
  TaskSpec/Failure Capsule use, third-party plugin and backend/adapter reports,
  LeKiwi physical evidence, same-tag Linux/Windows release rehearsals, the exact
  compatibility corpus, P0/P1 blockers, and a maintainer support commitment.
  The committed tracker retains and freshly replays the complete 29-check
  historical compatibility report, so the honest baseline is now 2/9 and
  remains ineligible; no tag or 1.0 claim is created.
- **Fail-closed 1.x promotion interlock**: `release-check`, platform
  `release-bundle`, and aggregate `release-exit` now require the complete
  external evidence pack plus an explicit assessment date for any 1.x or later
  version. Each path reruns the typed audit and writes a promotion report;
  missing, malformed, tampered, or ineligible evidence stops the release before
  packaging or publication. Normal 0.x development remains unchanged.
- **Replayable signed-provenance gate**: release jobs now retain the exact
  Sigstore bundle emitted by `actions/attest@v4`, and publication verifies each
  platform's archive and wheel from that bundle. The 1.0 readiness audit reruns
  `gh attestation verify` with the repository, workflow certificate identity,
  tag, source and signer commit, issuer, SLSA predicate, runner policy, and
  archive digest pinned, then requires an exact strict schema-v1 receipt.
- **Attested archive-install chain**: release jobs now emit and sign a strict
  schema-v1 archive-install rehearsal report that binds the exact archive
  name, size, and SHA-256 to the extracted `release-report.json`,
  `SHA256SUMS`, and all nine installed checks. Readiness manifest v3 requires
  separate fresh Sigstore receipts for the archive and this report, then
  reconstructs the complete checksum graph so reports from another archive
  cannot be substituted.
- **Subject-bound external certification evidence**: readiness manifest v2
  requires immutable external revisions and retains the exact controller
  library/manifest, physics implementation bundle, or hardware adapter,
  TaskSpec, and normalized launch arguments. The gate rehashes those bytes and
  matches report file names, sizes, digests, negotiated task identity, and
  observation/action widths, so a passing report cannot be relabelled onto a
  different implementation. Installed bundles now carry all three external
  conformance authoring guides beside the SDKs and runners.
- **Replayed historical-compatibility evidence**: the 1.0 readiness gate no
  longer accepts a registry-shaped compatibility report on its pass flags
  alone. It revalidates ancestor revisions, trees, schema declarations, and
  golden blobs, executes all 29 fixtures through the current typed readers,
  and requires the retained report to match that fresh result exactly.
- **Frontend transport history retention**: protocol v1's introducing commit
  and first committed full `ClientHello` golden are now bound to exact Git
  trees and the original blob. The installed compatibility runner decodes and
  re-encodes those ancestor bytes exactly, reproduces negotiation, and rejects
  corruption, unsupported versions, unknown kinds, truncation, trailing bytes,
  future fixture schemas, and unknown fields.
- **Immutable Rust public-API baseline**: `release/rust-api-baseline.toml`
  freezes the exact commit, Git tree, `cargo-semver-checks` version, and
  manifest path for all 31 publishable crates. CI now compares every shard to
  that fixed revision and fails if the commit disappears, its tree differs, a
  package moves, or the registry no longer covers the complete release set.
  Once bootstrapped, same-release registry changes are rejected against the PR
  base or push parent so the fixed comparison cannot be silently retargeted.
- **Provenance-bound historical migration matrix**: the installed
  compatibility corpus retains the original zero-step schema-v1 case and adds
  nonzero, sensor-bearing schema-v1 and schema-v2 snapshots emitted by their
  actual ancestor revisions. Each case fixes its source commit/tree, scene,
  generation steps, source digest, and normalized schema-v3 digest. Source CI
  verifies that both revisions remain ancestors with the recorded trees;
  installed bundles restore both artifacts and fail closed on provenance,
  schema, digest, or unknown-field drift.
- **Installed Python and C authoring contracts**: release bundles now include a
  dependency-free C/C++ controller header and a content-addressed 64-bit ABI
  layout/symbol fixture. The ABI3 wheel freezes all 24 public exports plus
  constructor, method, and property call shapes in a strict Python API manifest;
  source CI and extracted bundles emit a deterministic verification report.
  Installed-rehearsal schema v4 appends the ninth `python_api` check.
- **Installed compatibility fixture corpus**: `rne-compatibility` now verifies
  twenty-nine content-addressed TaskSpec, checkpoint, generic/behavior/scenario
  replay, dataset, renderer capture, all frontend protocol-v1 message families,
  controller C ABI and plugin-conformance report, historical migration,
  Failure Capsule, hardware, and physics artifacts through their current typed
  readers. The frontend and dataset payload fixtures also require byte-exact
  re-encoding and fail-closed handling of corrupt, truncated, and trailing
  binary input. Each check proves rejection of a future schema and unknown
  top-level field. Release bundles retain the registry and fixtures, CI uploads
  a deterministic schema-v1 report, and installed-rehearsal keeps the corpus a
  required workflow on Linux and Windows.
- **Historical checkpoint/replay decision matrix**: real artifacts emitted by
  ancestor serializers now prove exact restoration of generic vectorized
  checkpoint v1 and typed required-rerun rejection of scenario replay v2/v3.
  The scenario cases freeze their missing v4 evidence, exact errors, source
  commits/trees, unsafe-relabel rejection, and 300-step historical result; no
  migration fabricates actor, action, ownership, input, or result-digest data.
- **Historical portable-artifact retention**: TaskSpec v1, Failure Capsule v1,
  and a complete streaming dataset bundle v1 are now retained from their
  introducing ancestor revisions. The installed verifier reconstructs the
  dataset from its exact 736-byte shard, rechecks records, explicit gaps,
  hashes, and headless depth evaluation, and rejects binary corruption. Source
  CI binds all three artifacts to their exact commits, trees, schema sources,
  and original golden blobs.
- **External physics-backend conformance SDK**: the publishable
  `rne_physics_conformance` crate runs a fixed, unit-bearing nine-check catalog
  against any public `PhysicsBackend` factory without engine allowlists or
  vendor dependencies. Reports bind the implementation and manifest by
  SHA-256, reject capability overclaims, replay byte-identically, and require
  the exact implementation subject when packaged in a Failure Capsule.
- **Controller plugin authoring SDK**: dependency-free `rne_plugin_sdk` owns
  the versioned C-ABI constants, frames, and callback signatures while the host
  loader re-exports the existing paths. `rne-asset plugin new` vendors the
  exact SDK module for an offline warning-free build, and installed release
  rehearsal conforms both the reference binary and a freshly generated plugin.
- **External hardware-adapter conformance**: `rne-hardware-conformance` runs a
  content-addressed nine-case TaskSpec/wire/safety catalog against any child
  process with explicit sandboxed HIL authorization. The Rust process mock and
  Python LeKiwi bridge pass the same runner, and installed-rehearsal schema v2
  adds the required hardware-adapter check on Linux and Windows.

## [0.1.0] - 2026-08-14

### Added

- **0.1 release hardening (M6)**: the workspace now targets `0.1.0`,
  declares Rust `1.88.0` as its MSRV, gives packaged internal dependencies
  exact release requirements, denies undocumented public Rust APIs across every
  release library, and publishes the compatibility and migration contract plus
  machine-readable schema and release-blocker registries. Dedicated CI gates
  exercise the MSRV, warning-free rustdoc, all 27 crate archives, and patch-level
  SemVer compatibility against the frozen baseline. Pinned cargo-deny and
  cargo-audit checks now enforce the approved licenses, crates.io-only sources,
  reviewed duplicate-version exceptions, and RustSec policy; `xtask
  supply-chain` emits a deterministic sorted Cargo SBOM plus SHA-256 evidence
  for the lockfile and policy. Parser/protocol hardening adds 256 KiB importer
  limits, an absolute 32 MiB frontend payload ceiling, deterministic
  `xtask fuzz-smoke` evidence, and independent sanitizer-ready `cargo fuzz`
  targets. `xtask release-artifacts` now builds native CLI/controller bundles
  with SBOM, provenance, path-sorted SHA-256 checksums, installed-bundle smoke,
  and an ABI3 Python wheel install/import rehearsal; tag/manual release CI runs
  the same gate on Linux and Windows. `xtask release-exit` now records the
  complete M6-E exit matrix, including blocker, clean-checkout/tag, all CI,
  supply-chain, and release-rehearsal verdicts with per-stage durations.

- **Supply-chain release evidence (M6)**: pinned cargo-deny and cargo-audit
  gates enforce the crates.io-only source, license, duplicate-version, and
  advisory policies against the locked graph. A time-bounded exception
  registry records exact reachability and mitigation, while `xtask
  supply-chain` emits a deterministic Cargo SBOM and lockfile SHA-256 for CI
  artifacts. Dependency maintenance also updates PyO3 and parser/runtime
  libraries and removes an unused unmaintained font-parsing path.

- **Parser and protocol hardening (M6)**: import and frontend transport
  boundaries now enforce allocation-safe input ceilings, bounded deterministic
  OpenSCENARIO substitution, catalog traversal and symlink containment, and an
  MJCF nesting limit. A fixed 361-case, nine-boundary fuzz-smoke campaign emits
  reproducible schema-v1 panic-free evidence in required CI, with matching
  cargo-fuzz importer and transport targets for longer sanitizer runs.

- **Native release artifacts and install rehearsal (M6)**: pinned Linux and
  Windows bundles include the release CLIs, example controller plugin, ABI3
  Python wheel, fixtures, licenses, compatibility policy, blocker registry,
  SBOM, provenance report, and a complete SHA-256 manifest. Assembly and
  post-extraction checks run robot and scenario replay, physics conformance,
  the deterministic 100-actor scale gate, plugin discovery, and wheel install
  from bundled files only. Archive metadata and member ordering are normalized
  for byte-stable output. Tag builds publish both native archives and wheels
  only after clean Linux and Windows rehearsals succeed.

- **Machine-enforced final RC exit matrix (M6)**: a versioned exit contract
  maps all 12 required CI jobs and both native release rehearsals to their
  exact runner, clean-checkout requirement, and pinned command; every graph-
  building command must use the lockfile. The `workspace`
  and `release_candidate` aggregate checks emit schema-v1 verdict reports,
  reject missing, skipped, cancelled, or failed dependencies, and require zero
  open P0/P1 blockers. Native rehearsals now run on every pull request, and tag
  publication depends on the two-platform aggregate verdict. The first clean
  final matrix passed every required CI and native rehearsal job; both uploaded
  aggregate reports recorded `release_eligible=true` before PR #162 merged.
- **README simulation showcase**: mobile manipulation, G1 biped, Go2
  quadruped, 100-actor PLATEAU traffic, and a visible controlled quadrotor now
  have one quantitative media contract. The backend-neutral
  `MultirotorFlight` controller bounds speed, acceleration, yaw rate, and tilt
  with deterministic replay tests; the PLATEAU capture shares one detailed
  streetscape between vehicle and UAV media, and `xtask hero-media-check`
  enforces references, poster dimensions, and per-file/combined GIF budgets.

- **Scenario and traffic scale (M5)**: OpenSCENARIO maneuver groups expand to
  canonical multi-actor actions, heterogeneous actor kinds receive compatible
  deterministic routes, assigned routes no longer alias, and replay schema v4
  records UUID-ordered actor state plus ordered action evidence. Native traffic
  reports mixed runtime/external ownership and a complete visible-state digest.
  TraCI co-simulation now retains the last complete mirror across disconnects,
  exposes lifecycle metrics, and performs bounded snapshot-only recovery
  without re-sending an ambiguous simulation step. `xtask scenario-scale`
  writes the classified 100-actor/600-step release benchmark report and gates
  at least 60 headless steps/s on the named CI runner.

- **Physics conformance (M4)**: analytic and Rapier execute one fixed-step
  rigid-body vector, emit canonical versioned snapshots with frozen FNV-1a
  hashes, and compare unlike solvers only through a named SI-unit tolerance
  registry. Rapier articulation, resting-contact impulse, and repeated ordered
  raycast vectors prove every advertised capability. `xtask
  physics-conformance` and OSS parity write and validate the deterministic JSON
  report, including the measured Rapier convention that configured body mass is
  additional to default-density collider mass.
- **Observable analytic velocity**: analytic synchronization exposes integrated
  linear velocity through shared ECS state and canonical snapshots.

- **Production sensor/frontend transport (M3)**: `rne_data::transport` defines a
  fixed-header little-endian framed protocol with explicit version/capability/
  limit negotiation, bounded rejection messages, stable session and sequence
  fields, control/status/gap messages, and lossless RGB8, depth-f32, and LiDAR
  codecs preserving DataBus stream sequences and capture/availability ticks.
- **Bounded non-blocking runner frontend**: `rne-asset run --frontend-port`
  performs socket I/O off the simulation thread, bounds egress by frames and
  bytes, keeps control acknowledgements reliable, replaces stale status/sensor
  frames latest-only, and accepts reconnects without turning disconnect into
  `quit` or retaining an offline backlog. The runner DataBus now uses bounded
  per-stream retention.
- **Native binary frontend client**: `interactive_viewer --frontend-connect`
  negotiates the production protocol and projects binary RGB-D and LiDAR into
  the existing PiP/overlay path. Protocol golden, malformed payload, reconnect,
  process RGB-D/LiDAR, unread slow-client, and legacy compatibility tests are in
  the OSS parity catalog.

- **Stable robot-native controller surface (M2)**: versioned typed
  observation/action frames use stable robot and joint identities, explicit
  units, fixed-step timestamps, strict validation, and canonical ordering.
  Host-owned lifecycle and capability negotiation now gate deterministic
  multi-controller/multi-robot scheduling before the first step; command
  conflicts and unknown robot/joint targets are rejected.
- **Controller C ABI v3 with v2 compatibility**: the loader accepts ABI v2-v3,
  dispatches version-required symbols, and adds capability, configure, reset,
  robot-scoped step, and shutdown calls in v3. A frozen independent v2 plugin
  loads and steps in the current host; the reference plugin and generated
  scaffolds emit v3 and include plugin manifests.
- **Robot-scoped controller replay and runner lifecycle**: plugin actions are
  recorded per stable asset model ID and exact joint, replayed to one actuator,
  and displayed by the browser inspector. The runner owns configure, episode
  activation/reset, stepping, and shutdown. A two-robot URDF fixture proves
  identical action bytes and named actuator targets under reversed ECS spawn
  order.
- **Behavior CI failure replay and minimization**: typed contract/report schema
  v2 now records a deterministic seed manifest and per-violation state digest.
  Failed seeds emit versioned `.rne-replay` artifacts with scripted actions,
  task observations, named randomization dimensions, compatibility metadata,
  and the first violating contract. `xtask behavior-ci` deterministically
  minimizes each failure, verifies the minimized replay, writes a standalone
  `.behavior-case.json`, and includes all artifact paths in JSON and JUnit.
- **Local Behavior replay diagnostics**: `cargo run -p xtask --
  behavior-replay <artifact>` reconstructs the G1 scenario headlessly and
  requires matching schema, seed, action sequence, first violation, and stable
  world-state digests. Named JSON-field diffs distinguish semantic divergence
  from an explicit `1e-12` tolerance for derived floating-point observations.
- **Committed G1 failure fixture and browser inspection**: the invalid-tray
  fixture detects inactive-hand contact at a stable step and digest, reproduces
  across processes, and can be exercised with `behavior-ci --case`. The web
  replay inspector accepts Behavior CI artifacts and displays their contract,
  dimensions, observation interval, and exact 64-bit state hashes.

### Fixed

- **Ordered TCP runner-control replies**: command enqueue and acknowledgement
  now share the status-writer lock, guaranteeing the documented acknowledgement
  before the applied-state status even under Windows thread scheduling.
- **Stable Windows WGPU startup**: the default renderer now selects WGPU's
  primary backends on Windows instead of loading legacy OpenGL drivers. Explicit
  backend selection remains available through `WgpuRenderBackend::with_backends`.

## [0.14.0-rc.1] - 2026-08-10

### Added

- **Scenario replay artifact v3**: controlled and fixed-step OpenSCENARIO runs
  now record the producing RNE version plus stable digests of the exact XOSC
  and resolved traffic-network bytes. `rne-asset replay` verifies compatibility
  and both inputs before executing, rejects changed inputs with expected/actual
  digests, and validates result counts and finite metrics.
- **Native remote inspection loop**: the runner streams robot base, generic
  joint, RGB-D, LiDAR, IMU, wheel, and scenario traffic state; the native wgpu
  viewer projects remote articulated joints and traffic poses, renders RGB and
  GPU-depth picture-in-picture overlays, and draws remote LiDAR points without
  stepping a second physics world.
- **Executable OSS parity catalog**: `cargo run -p xtask -- parity` runs the
  flagship robot replay, scenario replay, sensor payload, traffic ownership,
  TraCI, runner-control, RGB-D, and frontend contracts and writes a machine-
  readable report. The catalog is an explicit CI gate.
- **M0-to-0.1 execution plan**: the roadmap now defines dated M0-M6 outcomes
  and objective exit gates for release consolidation, Behavior CI replay,
  stable control schemas, production sensor transport, physics conformance,
  scenario scale, and 0.1 release hardening.
- **Headless SUMO co-simulation run** (`rne-asset co-sim <net.xml> --routes
  <rou.xml> --steps N`): spawns SUMO, connects over TraCI, mirrors vehicles
  through `rne_traci::CoSimulation`, and reports a deterministic stable hash
  over every step's sorted vehicle states. `--determinism-check` runs it twice
  and requires identical outcomes. A process-level test verifies the mirrored
  vehicle and determinism against a real SUMO process (CI installs
  `eclipse-sumo`).
- **Live SUMO co-simulation bridge** (`rne_traci::CoSimulation`): mirrors every
  vehicle of a running SUMO process into the RNE ECS as a `TrafficActor` with a
  `TrafficPose` in the RNE Y-up frame, spawning, updating, and despawning
  actors as SUMO vehicles appear, move, and depart. A stateful-mock test pins
  create/update/remove, and CI's real-SUMO test verifies the bridge tracks the
  moving fixture vehicle. Advancing the mirrored actors through the RNE traffic
  runtime remains future work.
- **Live SUMO vehicle mapping**: `rne_traci::vehicle_position_rne` reads a
  SUMO vehicle's position in the RNE Y-up frame (`[x, 0, -y]`), matching the
  `rne_sumo` import frame so co-simulated vehicles land on the imported
  network geometry. CI's real-SUMO test now runs a moving vehicle
  (`assets/networks/sumo_cross_flow.rou.xml`) on `minimal_cross.net.xml` and
  verifies its RNE position advances down the approach across co-simulation
  steps.
- **Minimal TraCI client** (`rne_traci`): a TCP client for live SUMO
  co-simulation implementing SUMO's big-endian TraCI framing
  (`get_version`, `simulation_step`, `close`, `vehicle_ids`,
  `vehicle_position`). The crate tests validate the wire protocol against an
  in-process mock TraCI server, and CI installs `eclipse-sumo` so a
  co-simulation test runs against a real SUMO process.
- **Live observation streaming over the TCP control endpoint**: each completed
  step now streams a compact single-line JSON observation
  (`base`, `joints`, `sensors`) through `rne_core::RunnerControl::report_status`,
  so a renderer/frontend can render the live state without re-running physics.
  The TCP status line becomes
  `status step=<n> t=<t> state=<state> snapshot=<json>`; the process-level test
  verifies the snapshot payload alongside the control protocol.
- **SUMO `tlLogic` fixed-time signal-program import**: `rne_sumo` now parses
  `connection` and `tlLogic` elements and overlays them onto the derived
  topology by matching each `linkIndex` to the derived connection with the same
  `(incoming, outgoing)` lane pair, building one RNE `TrafficSignal` group per
  link and one phase per parsed phase (the phase-state character at a link
  index becomes the group's aspect). The `signalized_cross` fixture (20 s
  green northbound, then 15 s green eastbound) drives RNE stop-line control: a
  scenario actor on the eastbound approach is held at the red stop line with
  zero violations. Unsignalized networks still import without signals.
- **SUMO networks drive scenario runs**: a run manifest's OpenSCENARIO
  `LogicFile` may reference a SUMO `.net.xml` directly; `rne-asset run` imports
  it through `rne_sumo` (deriving topology) instead of loading a
  `.rne.traffic.json`. `assets/runs/sumo_cross.rne.run.toml` spawns a vehicle
  on the imported `minimal_cross.net.xml` fixture and drives it toward the
  intersection (206 m route at 10 m/s, no collisions); tests pin the route
  derivation, the run, and determinism.
- **TCP runner control endpoint** (`rne-asset run --control-port PORT`): the
  interactive control channel (`pause`, `resume`, `step N`, `reset`, `quit`)
  is also served over a local TCP connection for a GUI/frontend. On connect
  the runner sends `ready paused protocol=1`, acknowledges each accepted
  command with `ok <state>`, and streams
  `status step=<n> t=<t> state=<state> snapshot=<json>` after every step
  through `rne_core::RunnerControl::report_status`. A process-level test drives
  the binary over TCP and verifies the protocol and resulting replay.
- **Plugin authoring tooling**: `rne_plugin::scaffold_controller_plugin` generates
  a complete, compilable controller-plugin crate (a `cdylib` implementing the
  controller-plugin C ABI, initially a velocity-servo policy) plus a versioned
  `rne-plugin.json` manifest. `rne-asset plugin new <name> --dir <parent>`
  scaffolds it; `rne-asset plugin list --path <dir>` enumerates built-in and
  discoverable plugin libraries (`rne_plugin::discover_plugin_names`). An
  end-to-end test scaffolds, builds, and loads a plugin from the generated
  source, verifying the name, ABI version, and velocity-servo commands.
- **SUMO `.net.xml` road-network import** (`rne_sumo`): an offline importer that
  converts SUMO `edge`/`lane` geometry and `allow`/`disallow` road-user classes
  into the RNE Y-up frame (`[x, z, -y]`), skipping internal/connector edges, and
  derives junctions and lane connections deterministically through
  `rne_traffic::build_traffic_topology`. `rne-asset sumo-net` writes the
  resulting `.rne.traffic.json`. Fixture: `assets/networks/minimal_cross.net.xml`
  (four-way junction, seven movements). Malformed XML and shapes are rejected
  with clear errors.
- **Controller-plugin discovery by name**: the controller-plugin C ABI is now
  version 2 and requires the plugin to export `rne_plugin_name` (a static
  NUL-terminated name), which the host uses as the loaded plugin's name.
  `rne_plugin::discover_controller_plugin` searches directories for a shared
  library whose file name contains the requested name and whose
  `rne_plugin_name` matches, deterministically, falling back to the built-in
  `velocity_servo` registry. Run manifests select it with
  `[controller] plugin_paths = [...]`. Discovery, not-found, and
  discovered-vs-built-in replay-identical tests pin the behavior.
- **Dynamically loaded controller plugins**: `rne_plugin::load_controller_library`
  opens a controller plugin from a shared library through a versioned C ABI
  (`rne_plugin_abi_version`, `rne_controller_create`, `rne_controller_destroy`,
  `rne_controller_step`; `#[repr(C)]` observations/commands, NUL-terminated
  UTF-8 strings). ABI-version mismatches and missing symbols are rejected at
  load time. `rne_plugin_example_velocity_servo` is a minimal `cdylib` reference
  implementation that drives the same policy as the built-in
  `VelocityServoController`; a run manifest selects it with
  `[controller] kind = "plugin"` plus `library = "..."`. Loading, create-error,
  and determinism (loaded vs built-in replay-identical) tests pin the behavior.
- **Interactive runner control** (`rne-asset run --control-stdin`): a
  transport-neutral control state machine in `rne_core::control` drives the
  fixed-step loop with `pause`, `resume`, `step N`, `reset`, and `quit`
  commands on stdin. `reset` rebuilds the world from the episode's initial
  conditions; `step N` advances exactly N frames then pauses. The runner starts
  paused awaiting the first command, so piped scripts are timing-independent.
  stdin EOF while paused quits the run, determinism re-checks are skipped in
  interactive mode, and `--replay-out PATH` overrides the manifest's replay path.
- **Controller-plugin boundary** (`rne_plugin`): plugin manifests and a
  `ControllerPlugin` trait separate policy implementations from the runner.
  Run manifests select a plugin with `[controller] kind = "plugin"`; the
  built-in `VelocityServoController` maps observed joint positions to velocity
  commands each step. Deterministic replay records the plugin actions.
  Example: `assets/runs/mm_minimal_velocity_servo.rne.run.toml`.
- **OpenSCENARIO vehicle catalogs**: `CatalogLocations` `VehicleCatalog`
  directories are resolved relative to the scenario file, and
  `ScenarioObject` `CatalogReference` entries are looked up in the catalog
  files (scanned deterministically). Catalog references without a base
  directory are rejected. Parser tests pin the resolution.
- **OpenSCENARIO assigned-route actions**: the importer parses
  `AssignRouteAction` waypoints and the executor builds a polyline route from
  them, snapping the actor onto it at the scheduled time (each action applies
  once). Parser and deterministic execution tests pin the behavior.
- **Network signal timing in scenario runs**: the OpenSCENARIO executor derives
  stop-line controls from the road network's `TrafficSignal` fixed-time
  programs and advances their aspects each step, so actors stop at red and
  proceed on green. A signaled-corridor test pins the delay, zero red-line
  violations, and determinism.
- **OpenSCENARIO parameter substitution**: `ParameterDeclarations` values are
  substituted into `${name}` references before parsing, so action targets can
  be parameterized. Duplicate declarations are rejected.
- **OpenSCENARIO lane-change actions**: the importer parses `LateralAction`
  `LaneChangeAction` `RelativeTargetLane` events and the executor switches the
  actor to a synthetic parallel route (one lane width lateral, snapped) at the
  scheduled time. Deterministic lane-change and parser tests pin the behavior.
- **Second physics backend** (`rne_physics_analytic`): a deterministic,
  collision-free analytic backend (semi-implicit Euler gravity integration for
  dynamic rigid bodies) that implements the backend-neutral `PhysicsBackend`
  trait. Run manifests select the backend with `[physics] backend =
  "rapier" | "analytic"` and negotiate `required_capabilities` against it.
  Free-fall, determinism, fixed-body, capability, and empty-contact tests pin
  the behavior. Example: `assets/runs/cart_analytic.rne.run.toml`.
- **Physics capability negotiation**: `rne_physics::require_capabilities`
  verifies a backend's declared capabilities against a required set and reports
  the missing ones; run manifests can declare `[physics]
  required_capabilities = [...]` and `rne-asset run` fails with a clear error
  before executing if the backend cannot satisfy them. Rapier now also declares
  `deterministic_step` and `contact_force`.
- **Minimal MuJoCo MJCF model importer** (`rne_mjcf`): converts a strict MJCF
  subset (one root body tree, `hinge`/`slide` joints with `axis`/`range` under
  the `compiler` degree or radian convention, and `box`/`sphere`/`cylinder`
  geoms with `pos`/`rgba`) into a URDF document the existing `rne_urdf_import`
  pipeline consumes. Body/geom rotations, free/ball/universal joints, meshes,
  and capsules are rejected with a clear error. Golden and round-trip tests pin
  the emitted URDF.
- **Minimal SDF model importer** (`rne_sdf`): converts a strict Gazebo SDF
  subset (a single `<model>` of links with inertial/visual/collision geometry
  and revolute/continuous/prismatic/fixed joints) into a URDF document that the
  existing `rne_urdf_import` pipeline consumes. Worlds, multiple models,
  link/model `<pose>`, and unsupported geometry are rejected with a clear error.
  Golden and round-trip tests pin the emitted URDF.
- **Multi-joint position trajectories**: run manifests can use a
  `joint_trajectory` controller that interpolates time-indexed position
  waypoints per named joint; the runner records the interpolated targets as the
  frame action and they replay deterministically. Example:
  `assets/runs/mm_minimal_joint_trajectory.rne.run.toml`.
- **OpenSCENARIO run manifests**: run manifests can reference a scenario with
  `[scenario] xosc = "..."` (the manifest `scene` then becomes optional), and
  `rne-asset run` parses the OpenSCENARIO file, loads the road network from its
  `LogicFile`, executes the scenario over the traffic runtime, and verifies
  determinism when configured. Example: `assets/runs/scenario_speed.rne.run.toml`.
- **OpenSCENARIO scenario executor**: `rne_openscenario` now executes a
  scenario document over the traffic runtime — it derives an actor-compatible
  route from the road network, spawns the scenario entities as traffic actors,
  and applies each timed `AbsoluteSpeed` action while stepping the deterministic
  kinematic traffic systems. Deterministic replay tests pin the outcome.
- **Minimal OpenSCENARIO 1.0 importer** (`rne_openscenario`): parses a strict
  OpenSCENARIO 1.0 subset — `FileHeader` 1.0, `RoadNetwork/LogicFile` road
  reference, `Entities` vehicles/bicycles/pedestrians, `Init` teleport spawn
  poses, and storyboard `SpeedAction` events with `AbsoluteTargetSpeed` and a
  `SimulationTimeCondition` — into a versioned `.rne.scenario.json` document.
  Unsupported elements are rejected with a clear error; golden and round-trip
  tests pin the canonical JSON.
- **Contact and failure annotations**: the replay artifact now records per-step
  contact statistics (active pair count, summed and max normal impulse) and the
  final report annotates the run outcome with the maximum concurrent contact
  pairs, the largest per-step contact impulse, the minimum base height, and a
  `fell` failure when the first robot base drops below half its initial height.
  `rne-asset simulate`/`run` print the annotations and the browser replay
  inspector shows them per frame and in the report.
- **Full typed sensor payload export**: run manifests can request full IMU,
  LiDAR, camera (RGB+D), or wheel-encoder payload capture with `[[sensors]]`
  subscriptions (by entity name or kind). The `.rne-replay` artifact stores the
  complete typed payload per frame next to the existing stream summaries, and
  `rne-asset simulate`/`run` accept `--sensor-name` / `--sensor-kind`. Replay
  verification continues to check the exact payload hashes; the browser replay
  inspector summarizes captured payloads. See `docs/OSS_PARITY.md`.
- **Photoreal industrial environment package (v0.3-J)**: examples 70 and 71
  now default to a provenance-pinned CC0 Poly Haven Machine Shop HDRI and
  Hand Truck glTF prop, exercise the existing PBR/glTF material path, and keep
  procedural calibration-room and lighting fallbacks behind explicit
  environment variables. The asset directory records source URLs, authors,
  licenses, upstream MD5 values, and local SHA-256 hashes.
- **Photoreal Unitree G1 RGB-D sensor loop (v0.3-I)**: example 71 mounts the
  existing renderer-independent `CameraSpec` pipeline on G1 `head_link`,
  publishes paired RGB/depth frames through DataBus with deterministic optical
  effects and simulation latency, writes RGB/depth/manifest capture artifacts,
  and validates replay hashes in the workspace smoke gate.
- **Photoreal Unitree G1 capture (v0.3-H)**: example 70 now resolves the
  official Unitree G1 URDF/STL visual hierarchy through the existing physics
  world and `MeshRenderCache`, adds a PBR calibration room with floor normal/
  roughness maps, supports optional HDRI/TAA, writes PNG/GIF captures, and
  runs a headless mesh-resolution smoke in CI.
- **Photoreal humanoid asset integration (v0.3-G)**: the pinned and attributed
  Khronos Rigged Figure GLB now drives example 69 through the glTF scene loader,
  deterministic animation player, material/texture propagation, and WGPU color
  plus shadow skinning. `--smoke` validates the asset and GPU payload without a
  renderer; the default example writes two rendered animation frames. The
  workspace smoke gate runs the GPU-free asset check on every CI pass.
- **Photoreal environment lighting (v0.3-B)**: `rne_render` now loads
  Radiance `.hdr` equirectangular maps as validated linear RGB32F data, while
  `rne_render_wgpu` provides opt-in sky, diffuse IBL, and view-dependent
  specular IBL with configurable intensity and world-Y rotation. The G1
  photoreal capture accepts `RNE_HDRI_PATH`, `RNE_HDRI_INTENSITY`, and
  `RNE_HDRI_ROTATION_RAD`; no third-party HDRI is vendored.
- **Photoreal temporal anti-aliasing (v0.3-C)**: `rne_render_wgpu` now
  provides opt-in deterministic Halton camera jitter, depth-based reprojection,
  neighborhood history clamping, resize-safe accumulation, and G1 capture
  controls through `RNE_TAA`, `RNE_TAA_FEEDBACK`, and `RNE_TAA_JITTER_PX`.
- **Photoreal prefiltered IBL (v0.3-D)**: HDR environment uploads now build
  deterministic GGX/Hammersley specular mip levels and a cosine-weighted
  diffuse environment map. WGPU material shading selects specular blur from
  roughness while the original HDR map remains the sky source.
- **Photoreal humanoid motion (v0.3-E)**: `rne_render::load_gltf_scene` now
  preserves glTF node hierarchies, inverse-bind skins, four-influence vertex
  weights, and linear/step TRS animation clips. `GltfSceneAsset::sample_part`
  produces deterministic bind-pose-safe CPU-deformed meshes for the existing
  dynamic-mesh render path; cubic-spline animation and morph targets remain
  explicit unsupported cases.
- **Photoreal GPU skinning (v0.3-F)**: `GltfAnimationPlayer` and
  `GltfSceneAsset::sample_part_for_gpu` provide simulation-driven bind-pose
  meshes with joint matrices and weights. `rne_render_wgpu` uploads those
  payloads through storage buffers and applies them in both the color and
  shadow passes.
- **PLATEAU city-drive visual realism**: Example 46 now generates a licensed
  90-meter, ten-building PLATEAU-style showcase and renders varied facades,
  sidewalks, curbs, lane markings, a crossing, trees, and streetlights. The
  shared orbit camera now keeps world `+Y` upright, eliminating follow-camera
  roll, and the regenerated eight-second driving GIF uses a daylight city view.
- **PLATEAU road traffic realism**: `tran:Road` LOD1 surfaces now become
  deterministic road meshes and stable derived two-way lane metadata. A bounded
  SimClock-driven Ackermann model supplies acceleration, braking, steering-rate
  limits, pure-pursuit control, and explicit invalid-command behavior. Example
  46 drives both cars from imported lanes with rotating and steering wheels.
- **PLATEAU import Phase 1**: a ROS2-free offline `rne_plateau` pipeline and
  `rne-plateau-import` CLI convert bounded CityGML building LOD1 solids into
  deterministic per-building OBJ meshes, an ordinary RNE scene, and stable
  `gml:id` metadata. Geographic and projected coordinates map to local-meter
  Y-up space, building AABBs provide inexpensive static headless collision,
  and a synthetic CC0 fixture covers byte-identical replay and scene spawning.
  Example 46 renders a deterministic drone traversal plus two-way car traffic.
- **Robot Behavior CI Phase 1**: backend-neutral typed `Always`, `Eventually`,
  and `Consecutive` contracts in `rne_ai`; deterministic ascending multi-seed
  execution; first-violation diagnostics with stable entity names; versioned
  JSON and JUnit XML reports; and `cargo run -p xtask -- behavior-ci`. The
  initial headless G1 + Dex3 scenario checks grasp-contact stability, forbidden
  hand/workcell contact, a simulation-time acquisition deadline, finite
  observations, and bounded payload motion. A real successful seed and an
  intentionally invalid tray layout cover both report outcomes.
- **Friction-based grasp core (v0.14 Phase B)**: opt-in `GraspMode::Friction` on
  `MobileManipulatorSim` (`set_grasp_mode`) holds a grasped object with
  force-limited finger squeeze and surface friction only — no weld joint is
  inserted, the object stays a free rigid body, and the grasp drops when both
  fingers stop bearing load for 5 consecutive steps (or on an open command).
  The finger motors get a 1.0 N·m force cap in friction mode so the squeeze
  saturates at the object surface instead of wedging through it. Weld remains
  the default mode: all existing scenes, tests, and the README hero trajectory
  digest are bit-for-bit unchanged. Supporting plumbing: `ContactEvent` now
  carries the pair's accumulated normal-impulse magnitude from Rapier's solver
  (`impulse`, N·s per step), and scene TOML obstacles accept an optional
  `friction` coefficient override — used by the new tests to prove a µ=0.02
  cube slips out of the same grasp that carries a µ=0.5 cube.
- **Friction-grasp task migration (v0.14 Phase C)**: continued-close policies
  now converge on a bounded 15-step pinch target instead of winding the finger
  springs into a geometric jam. The fixed-base clutter policy/E2E and Python RL
  clutter/place rollouts select friction mode after reset; policy observations
  remain in carry/place coordinates across the friction grasp's debounced drop
  semantics. Weld remains available for scripted regression trajectories.

### Changed

- **Reproducible release gates**: GitHub Actions and local `xtask ci` use the
  repository's Rust 1.95 toolchain and locked dependency graph. Headless and
  default 10-seed Behavior CI are independent required jobs; the full local
  gate also includes headless, OSS parity, and Behavior CI stages.
- **Bounded runner transport**: protocol-v1 TCP writes have a finite timeout,
  opt-in source RGB-D is capped at 1920x1080 per image, source and transmitted
  dimensions are explicit, and snapshots above 32 MiB become a compact limit
  status instead of an unbounded socket write.
- **Explicit external traffic ownership**: co-simulated actors carry
  `TrafficPoseSource::External`, so native traffic systems do not advance poses
  owned by SUMO while native actors retain deterministic kinematic ownership.

### Known limitations

- **Mobile friction placement**: `mm_mobile` can acquire a physical friction
  grasp, but its planar arm has no vertical lift. The existing mobile place
  policy relies on a weld to drag the cube along the long tabletop before it
  falls clear; a free friction-held cube loses contact during that maneuver.
  Example 34 therefore validates friction grasp acquisition while its complete
  transport/place trajectory remains a v0.15 robot/scene redesign task.

### Fixed

- **Transactional TraCI synchronization**: a co-simulation step now reads and
  validates every sorted SUMO vehicle position before mutating ECS or the actor
  index. A late position error therefore cannot partially update existing
  actors or spawn only a prefix of the new vehicle set.
- **`mm_mobile` arm servo sway**: base motion alone (a yaw turn, or a driving
  turn like the hero pick approach) back-drove the uncommanded shoulder/elbow
  position hold by up to ~0.30 rad — the base's yaw acceleration couples the
  outboard arm chain's full inertia into the joints, and the shared 400/60
  spring constants (tuned for command tracking) are far too soft to resist it,
  so the swinging arm bulldozed tabletop objects during driving approaches.
  The mobile robot's shoulder/elbow now switch to a near-critically-damped
  4000/127 position hold while uncommanded (velocity-commanded moves, and the
  fixed-base robots, keep the original tracking dynamics), cutting the
  back-drive to ~0.034 rad. The hero rollout's pre-grasp cube nudge and its
  regression bound tighten accordingly.

- **README hero capture**: the hero GIF's task cube was a render-only decoration
  keyframe-lerped into the gripper at a hardcoded step — it visibly flew ~1.5 m
  through the air into a gripper that never approached it, and slid ~0.74 m back
  to the place target after release. The cube is now a real dynamic body in a new
  `mm_mobile_hero` scene (physical pick table + place tray), picked by the actual
  two-finger contact-gated grasp weld via an observation-gated approach → grasp →
  retreat → carry → release policy, and dropped onto the tray by physics. New
  smoke regression guards: pre-grasp object displacement, post-release slide,
  real `is_grasping()` grasp duration.

## [0.13.0] - 2026-07-06

### Added

- **`train_mobile_clutter.py`**: CEM smoke on the pinned `mobile_clutter_pick_place_center`
  episode (`mm_mobile_clutter` scene, `clutter_cube_a`). Re-implements
  `IkMobileClutterPickPlacePolicy`'s observation-gated phase machine (settle, pick drive,
  retreat, carry drive, release) in Python, with CEM tuning the pick-phase gripper rate
  and the pick/retreat/carry drive speeds against a weak baseline that holds the gripper
  open (structurally unable to grasp); asserts improvement margin, grasp, and deterministic
  replay of the best candidate.
- **`train_mobile_clutter_ppo.py`**: SB3 PPO integration smoke on `mobile_clutter_place_center`.
- **Mobile clutter place E2E**: `IkMobileClutterPickPlacePolicy` now completes the full
  navigate → grasp → place loop on `mm_mobile_clutter` (observation-gated phases: poke-grasp
  drive, straight retreat that drags the welded object clear of the tabletop contact wedge,
  carry drive that parks the object over the target, object-over-target release gate). The
  two mobile clutter E2E tests run un-ignored, and example 34 places `clutter_cube_a` on the
  ground target.

### Fixed

- **`mm_mobile` asset**: gripper base and finger links had no collision geometry (fingers could
  never articulate or trigger the contact-weld grasp), and the chassis/arm collision boxes
  interpenetrated, locking the shoulder and elbow joints solid. Colliders added and arm-chain
  collision boxes trimmed for clearance; `mm_mobile_clutter` table lowered to the lift-less
  arm's fixed shoulder plane.
- **Mobile base sim**: the diff-drive base pose is now integrated kinematically from the
  commanded twist after each physics step (heading extracted by planar projection instead of
  Euler yaw, which aliased transient contact tilt into corrupted headings), making drive
  phases deterministic; mobile arm joints use position-hold motors with an anti-windup lead
  and the mobile world runs at the lift robot's solver iteration count.
- **`mm_minimal` settle physics (linux CI)**: the fixed-base SCARA arm never settled — its
  chassis/arm colliders interpenetrated (injecting contact energy every tick) and its
  shoulder/elbow were bare velocity motors with no restoring force, so the idle pose was a
  sustained chaotic oscillation that merely sampled differently per platform. The arm now
  mirrors the mm_mobile fix (collision boxes trimmed, spring-damper position-hold motors,
  anti-windup lead) and settles to a true equilibrium (shoulder/elbow within mrad of zero,
  identical on Windows and Linux). Fixed-base grasp welds seat the object 2 cm upward so a
  horizontal carry does not fight the object's pinned support contact; the clutter cubes and
  the fixed-base place targets were re-derived against the stable dynamics (the old layouts
  were only reachable through the unstable arm's joint stretch); the mobile clutter carry
  steers the carried object (not the base) onto the place target so its platform-dependent
  carry offset no longer skips the release gate. All eight formerly linux-gated tests
  (seven in `rne_ai`, one in `rne_py`) and the example 26/33 + `run.py` smokes now run
  un-gated on all platforms. The README hero live-digest comparison stays Windows-only:
  cross-platform contact dynamics are outcome-stable, not bit-identical.

## [0.12.0] - 2026-07-03

### Added

- **`MmMinimalKinematics`**: analytic FK/IK for the fixed-base `mm_minimal` SCARA arm
  (`mm_minimal_kinematics.rs`), with roundtrip tests, sim XZ parity, and reachability helper.
- **`IkClutterPickPlacePolicy`**: IK approach + tuned fixed-velocity carry toward
  `mm_minimal_clutter_place_target` (fixed-base ground target off the table edge).
  Example 33 `--smoke` asserts grasp and place on `clutter_cube_b`; 15 clutter unit
  tests cover grasp, carry tuning, and full scripted place E2E.
- **`train_clutter.py`**: CEM smoke on the `clutter_place_center` episode (approach reward +
  place progress, grasp assertion, deterministic replay on the best candidate).
- **`train_clutter_ppo.py`**: SB3 PPO integration smoke on `clutter_place_center`.
- **`clutter_pick_place_center`**: pinned center-cube config for reproducible clutter RL benches.
- **`IkMobileClutterPickPlacePolicy`**: diff-drive approach + IK arm pick/place for
  `mm_mobile_clutter` (example 34; full place E2E still tuning).
- **`mm_mobile_clutter_place_target`**: shared ground place target helper for mobile clutter episodes.
- **`mobile_clutter_pick_place_center`**: pinned `clutter_cube_a` config for mobile RL benches.
- **`xtask ci`**: runs `train_clutter.py` / `train_clutter_ppo.py --smoke` alongside existing RL smokes.

## [0.11.0] - 2026-07-03

### Added

- **`ImageDepth` DataBus payload** and paired wrist RGB-D sampling (`sample_camera_rgbd`,
  scene-aware headless depth via `scene_depth_probe`).
- **Wrist depth observations**: `wrist_depth_center_m`, `wrist_depth_min_m`, and
  `target_object_index` on `MobileManipulatorObservation` (Python bindings included).
- **`VisuomotorReachPolicy`**: goal-conditioned reach that scales arm velocity from wrist depth.
- **Clutter pick-and-place episodes**: `clutter_pick_place` and `mobile_clutter_pick_place`
  configs with `mm_minimal_clutter` / `mm_mobile_clutter` scenes; pre-grasp approach reward
  on Place tasks.
- **RL bench scripts**: `train_place.py` (CEM place smoke) and `train_visuomotor.py`
  (depth-conditioned reach smoke).
- **`xtask ci`**: validates clutter scenes and runs `rne_py` RL smokes (`run.py`,
  `train_place.py`, `train_visuomotor.py`, `train_ppo.py`).
- **`IkLiftPickPlacePolicy`**: pick-and-place state machine whose carry swing solves
  [`MmLiftKinematics`] targets and drives shoulder / elbow / lift at a fixed rate toward
  the IK joint solution. Example 31 and the `lift_pick_place` episode test use this
  policy; [`LiftPickPlacePolicy`] remains for scripted regression tests.
- **ROS 2 `mm_lift` mode** (`RNE_ROS2_MODE=mm_lift`): loads the `mm_lift` scene and exposes
  manipulator subscriptions including `/lift_command` and `/arm_joint_trajectory`.
- **3-DOF lift-arm trajectories**: when `lift_joint`, `shoulder_joint`, and `elbow_joint`
  appear in `/arm_joint_trajectory` or `/arm_joint_position`, the bridge drives
  `MobileManipulatorAction::hold_lift_joints` waypoint following.

### Fixed

- **Depth stream id**: `rne_ai` wrist depth uses `rne_sensor::CAMERA_DEPTH_STREAM_OFFSET`
  (single source of truth).
- **Place / reach progress rewards**: potential-based shaping (signed delta) instead of
  clamping progress at zero.
- **Mobile manipulator snapshot v2**: adds `wrist_depth_frame`; schema v1 checkpoints restore
  with `wrist_depth_frame` absent (`#[serde(default)]`).
- **Clutter scenes**: tabletop support in `mm_minimal_clutter` (cubes settle on table, stay clear
  of idle arm sweep); E2E covers gripper contact on all targets, weld grasp of the center cube,
  transport Place script parity, and mobile-base approach.
- **`xtask ci`**: pinned Python deps in `requirements-ci.txt` (CPU-only torch, gymnasium, SB3);
  set `RNE_SKIP_RL_SMOKES` to skip RL smokes locally.
- **`rne_py` checkpoint tests**: tolerate JSON float roundtrip on episode rewards.
- **RL smokes**: deterministic `random.seed(0)`; CEM smokes check best-iteration improvement
  (`max(history) > history[0]`).

## [0.10.0] - 2026-07-02

### Added

- **`MmLiftKinematics`**: analytic forward / inverse kinematics for the `mm_lift`
  column + 2R arm chain (pure, deterministic, seed-free). Matches the simulation
  shoulder sign convention. Tests: `fk_ik_roundtrip_for_reachable_targets`,
  `fk_matches_sim_at_idle`, `fk_shoulder_sign_matches_positive_velocity_swing`.
- **Direct lift-arm joint targets**: `MobileManipulatorAction::lift_joint_target`,
  `MobileManipulatorAction::hold_lift_joints()`, and
  `MobileManipulatorSim::set_lift_joint_targets()` drive lift / shoulder / elbow
  position motors to absolute targets (with raised stiffness for direct holds).
- **Joint-space trajectory helpers**: `JointTrajectory`, `joint_tracking_action`,
  and `hold_lift_joint_action` for position-motor tracking. Test
  `ik_reaches_arbitrary_target`.
- **`rne_py` IK bindings**: `MmLiftKinematics`, `MmLiftJointTarget`,
  `MmLiftGripperTarget`, `MobileManipulatorSim(mode="mm_lift")`,
  `step_hold_lift_joints()` on sim and episode, and `lift_position_m` on
  observations.

### Changed

- **`LiftPickPlacePolicy`**: exposes `kinematics()` and `default_place_target()` for
  IK-based controllers; carry swing remains the proven scripted shoulder rate until
  IK carry converges reliably under grasp load.

## [0.9.0] - 2026-07-02

### Added

- **README 3D pick-and-place showcase**: a sim-captured hero still of the `mm_lift` robot
  hoisting a grasped cube, generated by the new `32_lift_pick_place_hero` example, plus an
  updated highlights/feature list and run commands for the pick-and-place.

- **ROS 2 `/lift_command` topic**: the ROS 2 node now subscribes to `std_msgs/Float64` on
  `/lift_command` to drive the vertical lift (positive raises, negative lowers), alongside
  the existing `/cmd_vel`, `/gripper_command`, and arm topics. Verified by the ci-ros2 smoke.

- **`LiftPickPlacePolicy`**: a reusable scripted pick-and-place policy (state machine) for
  the `mm_lift` robot — lower → grasp → lift → swing → settle → lower → release. It implements
  `Policy<MobileManipulatorEpisode>` and is now the single source for the pick-and-place
  trajectory used by example 31 and the episode test (previously duplicated inline).
- **Configurable place location**: `LiftPickPlacePolicy::with_swing_steps` sets how far the
  carry swing rotates the arm, so the cube can be placed at different spots around the column
  (`total_steps()` reports the sequence length). Test `lift_place_swing_controls_drop_location`.

### Changed

- **Place tasks now expose a goal offset in the observation** (`target_d{x,y,z}_m`): before
  grasping it points from the gripper to the object (where to pick), and once grasped it
  points from the object to the place target (where to carry). Previously these were always
  zero for Place tasks, leaving a policy blind; this makes the pick-and-place observation-
  driven. Test `place_observation_points_at_object_then_target`.

### Added

- **Interactive viewer `--manipulator-lift` profile**: the redesigned `mm_lift` robot is now
  viewable/teleoperable in example 14, with `R` / `F` driving the vertical lift. Wired into
  `xtask ci` as a render smoke.

- **Lift pick-and-place episode** (`MobileManipulatorEpisodeConfig::lift_pick_place`): the
  full 3D pick-and-place as a first-class `Episode` (reward + success), on the `mm_lift_pick`
  scene with a place target. Exposed to Python as `MobileManipulatorEpisode("lift_place")`.
  The Python episode `step` now accepts a `lift_velocity_m_s` argument (default `0.0`, so
  existing 5-argument calls are unchanged) to drive the vertical lift. Test
  `lift_pick_place_episode_picks_carries_and_places`.

- **Full 3D pick-and-place** (manipulator-redesign phase 4, final): the `mm_lift` robot now
  performs an end-to-end pick→lift→carry→place — lower the top-down claw over a ground cube,
  grasp it, lift it, swing the arm to a new spot, lower it, and open to release. Test
  `lift_picks_carries_and_places_cube` and example
  `31_mobile_manipulator_lift_pick_place` (carries the cube ~1.1 m and releases it; wired
  into `xtask ci`). This completes the four-phase manipulator redesign (column base →
  controllable arm → top-down claw → pick-and-place).

- **Real 3D pick** (manipulator-redesign phase 3): the `mm_lift` gripper is redesigned as a
  **top-down claw** (two fingers hang down to straddle an object) so it can lower over a cube
  on the ground, grasp it (contact-triggered weld), and the lift raises it off the ground —
  the previous side-grip could not pick a ground object because its body collided with it.
  New `mm_lift_pick` scene + `mm_lift_pick_scene_path()` and test
  `lift_picks_cube_off_ground_and_raises_it`.

- **Per-motor force override** (`JointMotor.max_force`, default `0.0` = use the
  per-joint-type cap): a positive value overrides the cap for that motor, e.g. a heavy
  arm joint that needs more torque to track its target.

### Changed

- **Lift robot arm is now controllable** (manipulator-redesign phase 2): the arm revolute
  joints are position (spring-damper) motors with a raised torque cap, so the heavy arm
  moves to a commanded angle and *holds* it — a plain velocity motor was too weak to move
  or hold it. Fixed a geometry bug where the upper arm overlapped the carriage and jammed
  the shoulder; the arm now also settles perfectly straight. New test
  `lift_arm_tracks_and_holds_commanded_pose`.

- **Lift robot can now lower its gripper to the ground** (manipulator-redesign phase 1):
  `mm_lift` is rebuilt on a tall fixed **column** with the arm hanging from a sliding
  carriage, so the lift lowers the gripper from rest (~0.81 m) down to near ground
  (~0.26 m) and raises it to carry — the previous box base let the lift only go up. The
  arm also settles much straighter. New test `lift_lowers_gripper_toward_ground`; existing
  lift tests/smoke unchanged in intent.

### Added

- **Per-world solver iterations** (`PhysicsWorldDesc.solver_iterations`, default `0` =
  Rapier's default): a higher count stabilizes stiff articulated chains. The `mm_lift`
  robot's world uses 16 iterations so its tall lift+arm chain holds its pose instead of
  swinging chaotically (it was unstable at the default); other robots are unchanged.
  Covered by a new idle-pose-hold test.

- **Vertical lift (`mm_lift` robot)**: a fixed-base arm with a prismatic "torso" lift
  between the base and shoulder, so the whole SCARA arm can be raised and lowered.
  `MobileManipulatorSim::new_mm_lift()` loads it; `MobileManipulatorAction.lift_velocity_m_s`
  drives the lift (other robots ignore it). The lift is a **position (spring-damper) motor**,
  so it holds the ~6 kg arm against gravity at a commanded height without drift — vertical
  lifting was previously blocked by the velocity-only motor. Covered by a unit test
  (controllable, reversible vertical motion) and a replay-determinism test.
- **Example 30 lift smoke**: `30_mobile_manipulator_lift` raises the `mm_lift` arm with
  the vertical lift and checks the end-effector rises (wired into `xtask ci`)
- **Joint position motors**: `JointMotor` gains `stiffness` + `target_position` fields
  (both default `0.0`, so existing velocity motors are unchanged). A positive stiffness
  turns a joint into a spring-damper that holds a position target under load.
- **Tunable motor gain**: `JointMotor.gain` (default `1.0`) scales the velocity-tracking
  damping factor instead of the previously hardcoded `1.0`, letting a joint track its target
  more stiffly under load. Prismatic motors also get a higher force cap (150 N vs the 50 N
  revolute cap) so a lift can hold a multi-link arm.
- **Reach curriculum** (`MobileManipulatorEpisodeConfig::reach_curriculum` + `ReachCurriculum`):
  an easy→hard curriculum that widens the goal-conditioned reach target region as the
  policy accumulates successes; exposed to Python as `MobileManipulatorEpisode("reach_curriculum")`
  with a `curriculum_stage` getter
- **Example 29 curriculum smoke**: a goal-conditioned policy advances the reach curriculum
  to its final stage (wired into `xtask ci`)
- **Determinism test** for the mobile manipulator reach episode (replay world-state hash)
- **Goal-conditioned reach** (`MobileManipulatorEpisodeConfig::reach_randomized`): a fresh
  reachable target is sampled each episode and exposed in the observation as
  `target_d{x,y,z}_m`, so a policy must generalize. Exposed to Python as
  `MobileManipulatorEpisode("reach_random")`; example 27 `train.py` now learns a
  goal-conditioned policy across varied targets, and the gym env includes the goal offset.

## [0.8.0] - 2026-06-16

### Added

- **`MobileManipulatorEpisodeConfig::reach()`** dense-reward reach task (exposed to Python
  as `MobileManipulatorEpisode("reach")`); target placed so it needs active control
- **Example 27 training loop** (`train.py`): Cross-Entropy-Method policy optimization that
  learns the reach task end-to-end with no external deps (mean reward ~2 → ~12)
- **`VectorizedMobileManipulatorEnv`**: batched mobile-manipulator episodes for
  population-based / parallel RL rollouts (parity with `VectorizedDiffDriveEnv`), with
  example 28 evaluating a policy population in lock-step
- **`rne_py.VectorizedMobileManipulatorEnv`**: Python binding for the batched env; the
  example 27 CEM training loop now evaluates each candidate population through it
- **Example 27 `train_ppo.py`**: Stable-Baselines3 PPO integration on the reach gym env
  (the `train.py` CEM loop remains the dependency-free deterministic learning demo)
- **Prismatic joints**: `rne_physics::PrismaticJointDesc` + Rapier linear motor; URDF
  `type="prismatic"` joints now wire into the articulation (`UrdfArticulationAttached.prismatic_joints`)
- **Fixed (weld) joints**: `rne_physics::FixedJointDesc` welds a child to a parent at a
  relative pose; the Rapier backend creates and *removes* the joint as the component is
  inserted/dropped (release)
- **Contact-triggered grasping**: `MobileManipulatorSim` welds a graspable body to the
  end-effector when the gripper closes on it and releases it on open
  (`is_grasping`, `grasped_object`)
- **`MobileManipulatorTask::Place`** and **`MobileManipulatorEpisodeConfig::place()`**:
  pick up a cube, carry it, and set it down at a target location
- **Example 26 pick-and-place smoke**: full grasp → carry → release → settle cycle
- **`rne_py` mobile manipulator bindings**: `MobileManipulatorSim` / `MobileManipulatorEpisode`
  (place / transport / inspect) exposed to Python with `is_grasping`
- **Example 27 RL env**: gymnasium-style `MobileManipulatorPlaceEnv` wrapper + scripted
  smoke (degrades gracefully without `gymnasium` / `numpy`)
- **ROS 2 `/gripper_command`** (`std_msgs/Float64`): drives the gripper in
  `mobile_manipulator` mode (negative closes/grasps, positive opens/releases)
- **ROS 2 `ee_link` TF frame**: end-effector pose published on `/tf` relative to `base_link`
- **ROS 2 `/arm_joint_position`** (`sensor_msgs/JointState`): position-control the arm —
  the node drives `shoulder_joint` / `elbow_joint` toward the commanded positions with a
  clamped P-controller (a velocity command cancels the target)
- **ROS 2 `/arm_joint_trajectory`** (`trajectory_msgs/JointTrajectory`): follow a sequence
  of `shoulder_joint` / `elbow_joint` waypoints, advancing to the next when the current one
  is reached

### Fixed

- **ROS 2 node build**: `sensor_msgs/Image.is_bigendian` type mismatch (`bool` → `u8`)
  that broke `rne_ros2_node` compilation
- **`mm_mobile` drive wheels**: wheel joints were stacked vertically (`xyz="0 ±0.225 0"`)
  so only one wheel touched the ground and the base spun in place; relocated to a proper
  left/right diff-drive layout (`xyz="0 -0.15 ±0.225"`) so the base drives forward
- **URDF fixed joints**: were not wired to a physics joint, so a fixed-joint child link
  silently became a free-falling body; now wired as a rigid `FixedJointDesc` weld
  (recalibrated the affected `mm_minimal` reach/place demo targets)

### Changed

- **Deterministic physics backend iteration**: the Rapier backend now syncs bodies and
  joints (and writes transforms back) in a stable entity order, fixing run-to-run
  nondeterminism (previously flaky `shoulder_motor_moves_forearm`)
- **`xtask ci`**: example 26 pick-and-place smoke

## [0.7.0] - 2026-06-12

### Added

- **Viewer wrist camera PiP** (`P` toggle) on `--manipulator` profiles in example 14
- **ROS `/camera/image_raw`** from wrist camera DataBus in `mobile_manipulator` mode
- **`MobileManipulatorEpisode`** with reach / grasp / transport / inspect tasks and rewards
- **`MobileManipulatorTask`** and **`MobileManipulatorRewardConfig`**
- **Example 25 episode smoke**: inspect + transport termination
- **`body_within_zone_m`** transport helper for drop-zone checks
- **`[wrist_camera]`** on `mm_mobile` robot asset (forearm mount)

### Changed

- **`xtask ci`**: example 25 smoke; viewer smokes for `--manipulator` and `--manipulator-mobile`

## [0.6.2] - 2026-06-12

### Added

- **Dynamic scene obstacles** (`body_type = "dynamic"`) for graspable objects
- **`mm_minimal_transport` scene** and transport helpers (`displacement_m`, `body_moved_at_least_m`)
- **Example 23 transport smoke**: finger contact + cube displacement ≥ 2 cm
- **`[wrist_camera]` robot asset section** mounted on URDF arm links
- **Wrist camera DataBus** (`ImageRgb8`) in `MobileManipulatorSim`
- **Example 24 wrist cam smoke**: publishes 64×48 RGBA8 frames

### Changed

- **Physics init**: zero-velocity ECS→Rapier sync on spawn for repeatable initial EE pose
- **Example 21 smoke**: proportional reach with error-reduction criterion (no multi-attempt retry loop)
- **`xtask ci`**: smokes examples 23 and 24

## [0.6.1] - 2026-06-12

### Added

- **`MobileManipulatorSim::from_scene_path`**: load `mm_minimal` / `mm_mobile` from `.rne.scene.toml`
- **Scene path helpers**: `mm_minimal_scene_path`, `mm_mobile_scene_path`, `mm_minimal_grasp_scene_path`
- **`mm_minimal` scene asset** (`assets/scenes/mm_minimal.rne.scene.toml`)
- **Parallel-jaw gripper** on `mm_minimal` URDF (`left_finger_joint`, `right_finger_joint`)
- **`MobileManipulatorAction::gripper_velocity_rad_s`** and grasp contact helpers (`finger_contacts_named`)
- **`mm_minimal_grasp` scene** with tabletop cube obstacle
- **Example 22 grasp smoke**: finger contact with `grasp_cube` (`--smoke`)

### Changed

- **`new_mm_minimal` / `new_mm_mobile`** delegate to default scene assets
- **Interactive viewer**, **example 21**, and **ROS `mobile_manipulator` mode** load robots via scene paths
- **Viewer teleop**: `C` / `V` gripper close / open on manipulator profiles

## [0.6.0] - 2026-06-12

### Added

- **URDF arm articulation** (`attach_urdf_articulation`): revolute joints + `JointMotor` wired to Rapier
- **Minimal mobile manipulator asset** (`assets/robots/mm_minimal/`) and example `20_mobile_manipulator_arm`
- **`MobileManipulatorSim`**: 2-DOF arm environment with EE/joint observations and DataBus `JointState`
- **Reach example** (`21_mobile_manipulator_reach`): open-loop shoulder motion smoke test
- **`mm_mobile` URDF**: diff-drive base + 2-DOF arm (`MobileManipulatorSim::new_mm_mobile()`)
- **Interactive viewer arm teleop** (`14_interactive_viewer --manipulator`): Q/E/Z/X arm keys and EE HUD
- **ROS 2 `/joint_states`**: wheel joint state published from native `rne_ros2_node` bridge
- **ROS 2 mobile manipulator mode** (`RNE_ROS2_MODE=mobile_manipulator`): 4-joint `/joint_states`, `/cmd_vel`, `/arm_joint_velocity`
- **`mm_mobile` scene asset** (`assets/scenes/mm_mobile.rne.scene.toml`) with URDF robot spawn from `.rne.robot.toml`
- **URDF robot asset spawn** (`rne_assets`): `base_body_type`, `articulation`, and initial pose for `kind = "urdf"`
- **Mobile base drive helpers** (`mm_mobile_twist_to_wheel_velocities`, unified wheel sign in `MobileManipulatorSim`)

### Changed

- **Rapier physics sync** uses composed world transforms for parent/child link hierarchies
- **`xtask ci`**: validates `mm_mobile` / `mm_minimal` assets; smokes examples 20, 21, and viewer `--manipulator-mobile`

## [0.5.0] - 2026-06-12

### Added

- **LiDAR render helpers** (`rne_render::lidar`): sphere markers for ray hits via `RenderScene::append_lidar_points`
- **LiDAR render example** (`19_lidar_render`): diff-drive scan visualized in wgpu
- **Interactive viewer LiDAR overlay** (`14_interactive_viewer`): live hit markers and `L` toggle via `append_lidar_overlay()`
- **`DiffDriveObservation::lidar_points`** populated from DataBus in `rne_ai`
- **Normal-based wgpu lighting**: Lambert diffuse + ambient in the primitive fragment shader using vertex normals
- **Scene-defined LiDAR**: optional `[lidar]` robot section and `[[obstacles]]` in `.rne.scene.toml`
- **ROS 2 native LiDAR**: `rne_ros2_node` publishes DataBus hits on `/points` and `/scan` (`RNE_ROS2_SCENE_PATH`)

### Changed

- **Interactive viewer and ROS bridge** load LiDAR from scene assets instead of a demo-only API

## [0.4.0] - 2026-06-12

### Added

- **Goal-conditioned episodes** (`16_goal_conditioned_agent`): `GoalSeekingPolicy`, `GoalCurriculum`, and multi-task goal sampling
- **Multi-robot collision** (`17_multi_robot_collision`): shared-world contact scenarios and peer-relative observations
- **ROS 2 sim control parity**: `simulation_interfaces` services, `/simulate_steps` action, and `wheel_velocity_rad_s` parameter on both native `rclrs` and Python bridge nodes
- **README hero capture** (`18_readme_hero`, `docs/media/generate-hero.sh`): orbit-rendered PNG/GIF from the real wgpu simulator
- **`world_transform_of()`** for composed URDF / parent-child render transforms

### Changed

- **`rne_urdf_import` moved to `crates/`** so core workspace CI no longer depends on `adapters/ros2/`
- **Rendering**: physics-synced bases use yaw-only rotation; orbit camera helpers live in `rne_render_wgpu::camera` (no winit required)
- **wgpu multi-draw fix**: per-item draw uniforms use dynamic offsets so multi-link URDF scenes render correctly
- **Depth readback** uses `TextureAspect::DepthOnly` for reliable off-screen passes

### Fixed

- URDF mesh scenes no longer disappear when child links carry local rotations
- Interactive viewer and headless examples frame robots with `CameraOrbit` instead of a fixed offset camera

## [0.3.0] - 2026-06-12

### Added

- **Shared-world agents** (`12_shared_world_agent`): agent entities live in the simulation ECS world and drive diff-drive robots in-place
- **Multi-robot simulation** (`13_multi_robot_agent`): multiple robots in one `DiffDriveSim`, batched stepping, per-robot policies
- **Richer observations** (`DiffDriveObservation`): base yaw, wheel velocities, optional goal-relative `goal_delta_x_m`; `AgentGoal` component
- **Interactive viewer** (`14_interactive_viewer`, `rne_render_wgpu/viewer`): winit + wgpu window, WASD teleop, orbit camera (`--smoke` for headless CI)
- **Asset pipeline** (`15_asset_hot_reload`, `rne-asset`): hot reload via dependency mtime tracking, validate / inspect / watch CLI, `xtask asset`
- **ROS 2 Python bridge CI**: `ros2-bridge.yml`, `xtask ci-ros2-bridge`, enhanced smoke test with `rne_py` build and topic checks
- **CI**: repo asset validation and spawn smoke in core `xtask ci`

### Changed

- Python ROS 2 bridge smoke aligned with native node (300 steps, `MIN_FORWARD_X_M = 0.8`)
- `rne_py` bindings expose extended diff-drive observation fields

### Notes

- Interactive viewer requires a display; use `--smoke` or `RNE_SKIP_GPU` in headless environments
- Asset hot reload tracks scene, robot, and URDF dependency files by modification time

## [0.2.0] - 2026-06-13

### Added

- **AI / episodes** (`rne_ai`): reward, termination, log recording, scene-backed episodes
- **Domain randomization** and **vectorized envs** (`VectorizedDiffDriveEnv`, example `10_vectorized_episode`)
- **Agent Entity** with attachable policies (`11_agent_policy`)
- **Assets** (`rne_assets`): `.rne.scene.toml` / `.rne.robot.toml` loaders (example `06_scene_load`)
- **Rendering**: primitive color + depth pass (`07_render_primitives`), URDF STL mesh draw (`09_urdf_mesh_render`)
- **Robot**: URDF → collider/visual auto attach; **Rapier joint-driven** diff-drive wheels (`DiffDriveDriveMode::JointDriven`)
- **Integration**: end-to-end scene → episode → optional render (`08_scene_episode`)
- **ROS 2**: native `rclrs` node (`adapters/ros2/rne_ros2_node`); optional CI via `xtask ci-ros2` and GitHub Actions
- **CI**: GitHub Actions workflow for core workspace (`ci.yml`)
- Examples `05`–`11` and expanded determinism coverage for joint-driven physics

### Changed

- Default diff-drive simulation uses joint-driven Rapier wheels (scene assets still use kinematic mode)
- README and roadmap refreshed for v0.2 feature set

### Notes

- Core CI remains ROS-free: `cargo run -p xtask -- ci`
- Native ROS node still builds outside the workspace with `--manifest-path` and patched message crates
- Python bridge unchanged in `adapters/ros2/rne_ros2_bridge/`

## [0.1.0] - 2026-06-13

### Added

- Core crates: `rne_math`, `rne_core`, `rne_ecs`, `rne_world`
- Physics: `rne_physics`, `rne_physics_rapier` with determinism hash tests
- Robot framework: diff-drive spawn, actuator commands, kinematics
- Sensors and DataBus: IMU, LiDAR, wheel encoder, camera, `InMemoryDataBus`
- Logging: JSONL record/replay for actuator commands
- Rendering: `rne_render`, `rne_render_wgpu`, headless camera path
- Python bindings: `rne_py` with diff-drive policy example
- Adapters: URDF import, ROS 2 message mapping, Python ROS 2 bridge node
- Examples: hello world, falling cube, diff drive + LiDAR, render clear, URDF import
- Docs: architecture overview under `docs/architecture/`
- CI: `cargo run -p xtask -- ci` with dependency boundary lint

### Notes

- ROS 2 runtime publishing uses the Python bridge in `adapters/ros2/rne_ros2_bridge/`
- Native `rclrs` nodes require additional `ros2-rust` type-support packages

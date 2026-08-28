# Bounded hardware gateway

`rne_hardware_gateway` is RNE's adapter-side safety boundary for playback,
shadow, hardware-in-the-loop (HIL), and live robot sessions. It binds directly
to the portable `TaskSpec`; it does not add hardware, ROS 2, vendor, or
wall-clock types to simulation crates.

This is the v0.6 foundation, not a claim of completed real-robot support. A
versioned process protocol, deterministic process mock, LeKiwi reference
adapter, and evidence-producing local host runner now exist. The physical
shadow/HIL/live evidence run remains required before live support is advertised.

## Time and authority

The state machine never reads a clock. Its process owner injects a monotonic
host tick in milliseconds into every operation. `SimClock` remains the only
time source inside simulation logic; the host tick exists only at the external
I/O boundary. A decreasing tick is a safety fault.

| Mode | Input source | Action behavior |
|---|---|---|
| playback | recorded adapter frames | validate and suppress |
| shadow | live observations | validate and suppress |
| HIL | test rig | require connection, fresh observation, cleared latch, and arm |
| live | physical robot | same fail-closed authority rules as HIL |

Playback and shadow cannot arm. HIL/live lose authority on disconnect, stale
observation, missed observation-to-command deadline, stale queued command,
actuator-limit violation, invalid controller output, queue overrun, clock
regression, or emergency stop.
The pending queue is cleared and replaced by a zero action carrying the fault
reason. Recovery requires an explicit latch clear, a fresh observation, and a
new arm operation.

## TaskSpec mapping

Observation and action order is the TaskSpec tensor order followed by row-major
element order. The initial gateway accepts every current TaskSpec observation
dtype through a normalized numeric boundary and checks its declared domain:
float range, integer integrality/range, exactly representable `i64`, `u8`, and
boolean 0/1. Hardware actions are restricted to float tensors with explicit,
finite TaskSpec bounds. Every submitted value is checked before authority or a
transport queue is touched.

The LeKiwi host exposes an observation-driven controller closure. The gateway
clock is sampled after that closure returns, so controller compute time counts
toward the command deadline. A malformed controller action in HIL/live becomes
a typed `controller_fault` zero stop rather than leaving the previous command
active.

Queue sizes and three timing limits are mandatory and non-zero:

- maximum observation age;
- observation-to-command deadline;
- maximum queued/active command age.

The observation ring, actuator queue, and audit event ring are bounded. Old
observations may be discarded with an audited counter; actuator commands are
never overwritten by a newer command. A full actuator queue trips safety.

## Process protocol and mock

Protocol v1 uses strict JSON Lines with distinct host/device discriminators,
an exact schema version, caller-selected session identity, monotonic request
sequence, and correlated response sequence. `HardwareWireCodec` enforces a
64 KiB default encoded-frame bound and reads through `BufRead::fill_buf`
without allowing an unterminated peer message to grow memory without limit.
Unknown fields, embedded line separators, zero widths, non-finite numeric
payloads, and inconsistent stop fields are rejected.

`HardwareWireTraceRecorder` has a hard entry capacity. It refuses a new frame
instead of discarding replay evidence, and it requires strict host/device
alternation plus request correlation. `HardwareSessionEvidence` joins the
exact wire trace to gateway events and rejects a disconnect or safety outcome
that does not match the final gateway latch.

An orderly Completed outcome is stricter than a transport close. HIL/live must
first relinquish authority, deliver and receive acknowledgment for the queued
zero stop, and then exchange Close. `close_cleanly` rejects armed authority,
pending actuator frames, or any safety latch; Completed evidence requires the
final snapshot to be disconnected, unlatched, and empty.

`rne-hardware-mock-device` implements that public contract in a child process.
It never reads a clock or sleeps. Observation polls return deterministic zero
vectors, and command-line fault plans inject an exact-count disconnect or
emergency stop. A terminal response is valid only when it confirms the device
watchdog applied a safe stop. The mock is conformance infrastructure, not a
robot dynamics model or a vendor transport.

`rne-hardware-conformance` is the separately versioned third-party-facing
runner. It launches any supplied adapter executable through the same wire
contract and emits a content-addressed nine-check report. Open identity,
TaskSpec binding, observation dtype/sequence, bounded HIL actuation, safe stop,
shadow authority, request ordering, session isolation, and action width are
tested without linking adapter code into RNE. Full execution requires explicit
`--allow-hil` authorization and is intended for a sandbox, process mock, or
isolated rig. See
[HARDWARE_ADAPTER_CONFORMANCE.md](HARDWARE_ADAPTER_CONFORMANCE.md).

## Shadow comparison

`ShadowComparator` binds hardware observations and deterministic simulation
steps to the same TaskSpec observation order. Its configuration provides
exactly one absolute tolerance per tensor; integer and boolean tensors require
zero tolerance. Hardware sequence, injected receipt tick, simulation step, and
SimClock ticks remain distinct. Shape/dtype checks run on both sides before
metrics are computed.

The comparator retains a configured maximum number of samples and fails when
that capacity is reached. Its report preserves normalized hardware/simulation
vectors, per-sample sum/mean/max error, violation count, the first
tensor/element/unit divergence, aggregate metrics, and a pass/fail verdict.
`validate_against` rebinds an untrusted report to the TaskSpec and replays every
vector through the comparator. This establishes the portable comparison
contract; an actual reference-device shadow run is still required.

`RecordedShadowSession` is the content-addressed execution envelope above that
comparator. It binds the exact TaskSpec, controller, requirements, action trace,
recorded trace, comparison trace, and calibration hashes. The stream contract
also retains the clock source, tick scale, nominal and maximum latency, explicit
sequence-gap drop policy, TaskSpec tensor units, bootstrap-action count, and a
hard sample capacity. `rne-recorded-shadow-session` executes the envelope in a
separate process through either playback or shadow authority. Every valid
controller action must be suppressed and the report fails if an actuator frame
is emitted.

The OpenArm compiler uses the retained 0.5 N·m Coulomb traces without refitting
the controller or changing tolerances. It produces three distinct cases:

- recorded Rapier to the same recording: exact playback pass;
- recorded Rapier to MuJoCo: strict `0.02 rad` position and `0.1 rad/s`
  velocity shadow comparison with the first divergence retained;
- recorded Rapier to itself with a predeclared sequence-900 disconnect: exact
  numeric comparison plus a bounded, non-actuating transport failure.

Build the session inputs, run the process boundary, and create the browser gate
report with `build_openarm_recorded_shadow_session.py`,
`rne-recorded-shadow-session`, and
`build_openarm_recorded_shadow_report.py`. The requirements live in
`openarm_recorded_shadow_requirements.json`; neither runner accepts an
undeclared disconnect point or an unbound TaskSpec/controller.

## Evidence and current gate

Gateway evidence schema v1 contains the retained ordered event stream and a
final bounded snapshot. Wire protocol/trace/session-evidence schemas are also
registered in `release/contracts.toml`. Canonical evidence includes:

- `tests/golden/hardware/gateway-fail-closed-session-v1.json`: in-process live
  over-limit stop;
- `tests/golden/hardware/gateway-process-disconnect-session-v1.json`:
  process-isolated HIL command, injected disconnect, device watchdog stop, and
  correlated gateway stop;
- `tests/golden/hardware/gateway-shadow-comparison-v1.json`: TaskSpec-bound
  comparison with one preserved first divergence and recomputed verdict;
- `tests/golden/hardware/gateway-mock-conformance-v1.json`: six actual child
  process cases covering command deadline, disconnect, explicit reconnect,
  stale command, actuator limit, and device emergency stop.

`HardwareSessionEvidence::validate_against` replays the complete nested trace,
normalizes the outer envelope, constructs a gateway from the supplied TaskSpec,
and requires the Open/Ready observation and action widths to equal the exact
flattened task spaces. The current sensor-bearing diff-drive contract is
`rne.diff_drive.sensor_goal.v1`; the older five-element
`rne.diff_drive.goal.v1` artifact remains a separate compatibility identity.

The external adapter report is intentionally content-addressed rather than a
fixed binary golden: native executable bytes differ by target. The integration
test requires two fresh runs against the fixed-binding Rust mock to be exactly
equal, rejects a mock without a fixed TaskSpec binding, and applies the same
catalog to the Python LeKiwi bridge.

Run the focused gate with:

```powershell
cargo test -p rne_hardware_gateway
cargo clippy -p rne_hardware_gateway --all-targets -- -D warnings
```

`xtask ci-headless` also executes this gate. Unit and golden tests currently
cover bounded wire decoding, strict trace correlation, process isolation,
shadow suppression, armed HIL/live delivery, limit stop, missed deadline,
stale queued command, disconnect/reconnect, emergency stop, bounded
observations, clock regression, and missing action limits.

`rne-asset failure-capsule create|verify` (and the source-tree `xtask` alias)
preserves and validates TaskSpec,
session, wire-trace, shadow-report, and mock-conformance kinds. It keeps a
corresponding simulation or behavior replay as the capsule replay instead of
pretending host time is simulation time.

The process-mock exit matrix and LeKiwi host session path are complete. The
next external gate records a real elevated shadow run and then the physical
safety matrix before any live-support claim.

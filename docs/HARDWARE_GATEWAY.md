# Bounded hardware gateway

`rne_hardware_gateway` is RNE's adapter-side safety boundary for playback,
shadow, hardware-in-the-loop (HIL), and live robot sessions. It binds directly
to the portable `TaskSpec`; it does not add hardware, ROS 2, vendor, or
wall-clock types to simulation crates.

This is the v0.6 foundation, not a claim of real-robot support. A vendor
transport, process-level mock, reference robot, and completed hardware evidence
run remain required before live support is advertised.

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
actuator-limit violation, queue overrun, clock regression, or emergency stop.
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

Queue sizes and three timing limits are mandatory and non-zero:

- maximum observation age;
- observation-to-command deadline;
- maximum queued/active command age.

The observation ring, actuator queue, and audit event ring are bounded. Old
observations may be discarded with an audited counter; actuator commands are
never overwritten by a newer command. A full actuator queue trips safety.

## Evidence and current gate

Gateway evidence schema v1 contains the retained ordered event stream and a
final bounded snapshot. Its contract is registered in
`release/contracts.toml`; the canonical over-limit live session is
`tests/golden/hardware/gateway-fail-closed-session-v1.json`.

Run the focused gate with:

```powershell
cargo test -p rne_hardware_gateway
cargo clippy -p rne_hardware_gateway --all-targets -- -D warnings
```

`xtask ci-headless` also executes this gate. Unit and golden tests currently
cover shadow suppression, armed live delivery, limit stop, missed deadline,
stale queued command, disconnect/reconnect, emergency stop, bounded
observations, clock regression, and missing action limits.

The next slices are a versioned process protocol and mock, observation/action
trace recording, shadow-vs-simulation comparison, Failure Capsule integration,
and only then a selected reference hardware adapter.

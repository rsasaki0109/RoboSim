# ADR 016: Keep hardware authority bounded and outside core

## Status

Accepted for the v0.6 sim-to-real foundation. The adapter-side state machine,
bounded process protocol, deterministic process mock, and golden disconnect
session were implemented on 2026-08-15. The TaskSpec-bound shadow comparator
and its first-divergence golden are also implemented. The six-case child-process
fault matrix and LeKiwi profile-bound host runner pass; physical-hardware
evidence remains open.

## Context

The portable TaskSpec gives simulation, datasets, Python, and accelerators one
ordered observation/action contract. Hardware adds a different class of risk:
wall-clock deadlines, transport disconnects, old observations, commands that
wait too long in a queue, actuator limits, and emergency-stop authority.

Putting those concerns into `rne_core` would make deterministic simulation
depend on external time and I/O. Letting each vendor adapter invent its own
safety behavior would make shadow, HIL, and live evidence incomparable.

## Decision

`adapters/hardware/rne_hardware_gateway` owns a vendor-neutral, TaskSpec-bound
state machine. The owner injects monotonic host ticks; the state machine does
not read the wall clock. Playback and shadow validate actions without actuator
authority. HIL and live require a connection, a fresh TaskSpec-shaped
observation, an explicitly cleared safety latch, and an explicit arm operation.

All observation, actuation, and event queues have configured hard bounds.
Actions require explicit TaskSpec bounds. Disconnect, stale data, deadline
miss, over-limit action, queue overrun, clock regression, and emergency stop
remove authority, clear pending commands, and queue a typed zero-action stop.
Reconnect never restores authority implicitly.

Gateway event/snapshot evidence is schema-versioned and has a committed golden
shape. It records decisions made from injected host ticks, but those ticks do
not participate in simulation determinism or stable simulation-state hashes.

The process boundary uses a versioned, byte-bounded JSON Lines protocol with
separate host/device kinds, session and request correlation, strict payloads,
and a bounded trace that never overwrites evidence. Terminal device responses
must confirm a device-side safe stop. A deterministic child-process mock
implements the same public contract without clocks, sleeps, vendor types, or
network dependencies.

Normal completion is a separate transition from fault disconnect. An
actuating session must disarm, deliver its resulting zero stop, receive the
device acknowledgment, and exchange Close before `close_cleanly` can produce a
disconnected, unlatched, empty final snapshot. Completed session evidence
rechecks those conditions.

Shadow comparison is a bounded evidence operation, not a new authority mode.
It compares TaskSpec-normalized hardware and simulation observations using one
ordered tolerance per tensor, keeps host and simulation timestamps distinct,
and preserves the first field outside tolerance. Discrete dtypes compare
exactly. Reports can be rebound to the TaskSpec and their aggregate verdicts
recomputed without hardware access.

Failure Capsules retain their existing simulation/behavior replay requirement.
Hardware session and shadow artifacts are typed evidence beside that replay,
never a synthetic simulation clock. Capsule creation and verification require
a matching TaskSpec and rerun session, wire-trace, and shadow-report validation.

## Consequences

Core crates remain ROS 2-, vendor-, network-, and wall-clock-free. A ROS 2,
C/Python, or vendor process can share the same authority contract without
changing TaskSpec or simulation types. Tests can cover every timing boundary
without sleeping.

The gateway is not itself a physical-hardware claim. The process mock proves
command deadline, disconnect, reconnect, stale command, limit, and emergency
stop behavior, but the selected reference robot must still supply an actual
shadow run and v0.6 exit evidence. Future protocol revisions may not weaken
byte/queue/deadline/limit, device-stop confirmation, comparison, capsule
validation, or explicit-rearm invariants.

LeKiwi + SO-101 is the selected v0.6 reference robot. Its brand-specific
adapter pins LeRobot v0.6.0, freezes the nine-value observation and three-value
base-action mapping, and adds a direct Python device process with an independent
500 ms watchdog. Reference profile v1 grants no arm actuation: normal base
commands hold the latest measured arm pose and safety frames call the base-stop
operation directly. Camera payloads remain typed dataset streams outside the
numeric wire. The injected-clock host runner records the exact profile, bridge
identity, bounded wire trace, gateway decisions, and terminal state in one
validated artifact. Mock and physical ready identities are deliberately
distinct. This selection and process conformance do not replace the still
required physical shadow, HIL, and live evidence.

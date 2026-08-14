# ADR 016: Keep hardware authority bounded and outside core

## Status

Accepted for the v0.6 sim-to-real foundation. The adapter-side state machine
and golden fail-closed session were implemented on 2026-08-15; process and
physical-hardware evidence remain open.

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

## Consequences

Core crates remain ROS 2-, vendor-, network-, and wall-clock-free. A ROS 2,
C/Python, or vendor process can share the same authority contract without
changing TaskSpec or simulation types. Tests can cover every timing boundary
without sleeping.

The gateway is not itself a physical-hardware claim. A process protocol, mock
disconnect/reconnect suite, trace/Failure Capsule bridge, shadow comparison,
and selected reference robot must still supply v0.6 exit evidence. A future
protocol may wrap these types, but may not weaken their queue, deadline, limit,
or explicit-rearm invariants.

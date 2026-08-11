# Stable Controller Control Surface Plan

Status: M2-A through M2-D implemented; all local exit gates passed

## Goal

M2 turns the current single-robot velocity-servo callback into a stable,
robot-native controller boundary. The boundary remains independent of ROS 2,
rendering, and any physics backend. A controller sees fixed-step observations,
returns typed actions, negotiates required capabilities before activation, and
can address several robots without depending on ECS allocation or spawn order.

## M2-A: Versioned observation and action schemas

`rne_plugin::control` defines controller schema version 1:

- `ControllerObservationFrame` records `step`, integer `sim_time_ticks`, and
  robot observations;
- `ControllerRobotObservation` uses a stable `robot_id` and named joint
  position/optional velocity values with explicit units;
- `ControllerActionFrame` answers one observation step with robot-scoped named
  joint-velocity commands;
- constructors canonicalize robots by `robot_id` and joints by name;
- validators reject unsupported versions, empty/NUL-containing identifiers,
  duplicates, non-canonical deserialized ordering, non-finite values, step
  mismatches, and actions targeting an unobserved robot or joint.

The strict ordering rule makes serialized frames and controller scheduling
independent of ECS entity IDs and input insertion order.

## M2-B: Lifecycle and capability negotiation

Implemented with a controller descriptor containing a sorted capability set
and a host-owned `created -> configured -> active -> shutdown` lifecycle. Configuration must
fail before stepping when a controller cannot consume every required
observation or produce every required action. Reset remains a deterministic
simulation event and receives fixed-step metadata, never wall-clock time.

## M2-C: Multi-robot scheduling

Implemented with a scheduler that invokes controllers in stable
controller/robot ID order, validates every returned action against the paired observation, rejects
conflicting commands, and emits one canonical action frame. A reversed-spawn
fixture must produce byte-identical action frames and equivalent final named
robot state.

## M2-D: C ABI compatibility

Implemented ABI v3 capability, lifecycle, and robot-scoped fixed-step symbols
while retaining the ABI-v2 symbol layouts as the oldest supported contract.
The loader dispatches by the plugin-reported ABI version. Tests build an
independent frozen ABI-v2 fixture and prove that it loads and runs in the newest
host, while the current example and generated scaffold exercise ABI v3 and its
negotiation path. Both plugin crates include committed manifests.

The asset runner now owns scheduler lifecycle, canonicalizes stable asset model
IDs independently of URDF internal names, records robot-scoped replay actions,
and resolves those actions back to exact robot/joint actuator targets. A dual
URDF fixture is spawned in both declaration orders and must produce identical
action JSON and named actuator state.

## M2 exit gate

- a controller built against ABI v2 loads and produces the expected command in
  the current runtime;
- unsupported capability requirements fail during configuration, before the
  first simulation step;
- multi-robot outputs are byte-identical when scene spawn order is reversed;
- no ROS 2, adapter, renderer, or physics-backend type enters the public
  controller schemas;
- formatting, workspace Clippy/tests, headless CI, full CI, and the parity
  catalog pass from the locked dependency graph.

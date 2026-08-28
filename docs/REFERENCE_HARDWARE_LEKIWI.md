# LeKiwi + SO-101 reference hardware

Status: selected profile, device bridge, profile-bound evidence runner, and
machine-verifiable physical-evidence manifest; physical shadow/HIL/live
evidence has not yet been captured.

This base-only profile is the safety prerequisite for the bounded physical
flagship, not the flagship proof itself. It runs
`rne.lekiwi_so101.base_shadow.v1`, whereas the release flagship runs
`rne.flagship.mobile_lift_shared_aisle.v1` with
`rne.ai.ik_mobile_lift_pick_place_policy.v1`. The rate, observation, and action
spaces differ. Gate 4 therefore additionally requires the explicit full-task
projection, elevated parent-controller shadow, bounded live success/failure,
and complete artifact closure defined in the
[external product proof plan](PLAN_EXTERNAL_PRODUCT_PROOF.md#gate-4-bounded-physical-proof).

## Selection

RNE v0.6 uses LeKiwi with the SO-101 follower arm as its first affordable
reference robot. This choice reuses the repository's vendored LeKiwi base,
SO-101 arm, composite URDF, kiwi-drive kinematics, and headless simulation
coverage. It also has a maintained upstream implementation with explicit motor
IDs, calibration, observation/action features, disconnect behavior, and a
device watchdog.

The adapter pins [Hugging Face LeRobot release
v0.6.0](https://github.com/huggingface/lerobot/releases/tag/v0.6.0), source
revision 30da8e687a6dfc617fcd94afc367ac7071c376ce. The pinned
[LeKiwi implementation](https://github.com/huggingface/lerobot/blob/v0.6.0/src/lerobot/robots/lekiwi/lekiwi.py)
defines motor IDs 1 through 6 for the arm, 7 through 9 for the base, six arm
position features, and x/y/theta velocity features. The pinned
[configuration](https://github.com/huggingface/lerobot/blob/v0.6.0/src/lerobot/robots/lekiwi/config_lekiwi.py)
enables degree units, torque disable on disconnect, a 500 ms host watchdog,
and a 30 Hz maximum loop frequency. Assembly and calibration remain governed
by the upstream [LeKiwi hardware guide](https://github.com/huggingface/lerobot/blob/v0.6.0/docs/source/lekiwi.mdx).

No LeRobot, Feetech, ZeroMQ, serial, camera, or wall-clock type enters an RNE
core crate. The only shared control boundary is TaskSpec plus hardware wire v1.

## Versioned contract

The exact profile is frozen in
tests/golden/hardware/lekiwi-reference-profile-v1.json and the portable task in
assets/tasks/lekiwi_so101_base.task.json. The executable source of truth is
rne_hardware_lekiwi::lekiwi_reference_profile_v1.

`rne-lekiwi-session` runs the same Open -> poll -> gateway -> optional actuator
write -> zero-stop -> Close lifecycle in mock, shadow, HIL, and live tests. Its
versioned output embeds the exact profile, device identity, complete bounded
wire trace, gateway events, and final snapshot. A Completed artifact is valid
only when the gateway is disarmed, has no pending actuation or safety latch,
the device acknowledged the final zero stop in HIL/live, and the explicit
Close response was followed by a clean local disconnect.

| Direction | RNE tensor | Shape | Unit | Upstream fields |
|---|---|---:|---|---|
| Observation | arm_joint_position_rad | 5 | rad | shoulder pan/lift, elbow flex, wrist flex/roll positions in deg |
| Observation | gripper_position_pct | scalar | pct | arm_gripper.pos in 0..100 |
| Observation | base_linear_velocity_m_s | 2 | m/s | x.vel, y.vel |
| Observation | base_angular_velocity_rad_s | scalar | rad/s | theta.vel in deg/s |
| Action | base_linear_velocity_m_s | 2 | m/s | x.vel, y.vel |
| Action | base_angular_velocity_rad_s | scalar | rad/s | theta.vel in deg/s |

The front 640 by 480 camera rotated 180 degrees and wrist 480 by 640 camera
rotated 90 degrees are declared as dataset streams. They are intentionally not
encoded into the bounded numeric process wire.

## Flagship action projection

`rne_hardware_lekiwi::flagship_projection` schema v1 is the first executable
same-contract boundary. It validates the complete seven-element flagship
controller action, converts its left/right wheel velocities from `rad/s` to a
body-x velocity in `m/s` and yaw rate in `rad/s`, and independently validates
the resulting three-element LeKiwi base action. The transform uses the
flagship model's canonical `0.1 m` wheel radius and `0.45 m` track width.

The projection never clamps an unsafe command. It fails closed when either
TaskSpec envelope is exceeded and records the five arm/lift/gripper elements
as explicitly suppressed values with a deterministic parent-action hash.
Registration as `hardware.flagship_lekiwi_action_projection = 1` freezes this
artifact shape; it does not make the bridge live-ready. Full-file content
bindings and parent-order observation fusion remain required before elevated
flagship shadow or actuation; the deterministic rate boundary is defined below.

The next executable boundary is
`rne_hardware_lekiwi::flagship_rate::FlagshipLeKiwiRateScheduler`. It accepts
exactly ordered zero-based controller sequences, validates every action through
the projection above, emits phase-zero even sequences, and records intervening
odd sequences as explicitly suppressed. Its `33,333,334 ns` write period is
exactly two `16,666,667 ns` simulation ticks and therefore never exceeds the
30 Hz device ceiling. Duplicate, missing, out-of-order, invalid, and overflowed
inputs fail without advancing state. This wall-clock-free rate proof still does
not supply the parent-order observation required to execute the controller.

## Safety case

The initial physical profile does not grant arm actuation. A normal base
command re-sends the latest measured six arm/gripper values so the arm holds
position. A gateway safety frame calls stop_base directly; it never interprets
the gateway's zero vector as a six-joint target.

The pinned v0.6.0 relative-target option is left disabled. That release looks
up the suffixed action keys in an unsuffixed present-position map when the
option is enabled. The RNE bridge avoids that inconsistent path entirely:
TaskSpec exposes no arm action, the bridge constructs arm fields only from the
latest finite device observation, and any missing observation rejects base
actuation.

The complete layered limits are:

- base x and y: at most 0.1 m/s per axis;
- base yaw: at most pi/6 rad/s, or 30 deg/s;
- observation age: 100 ms;
- observation-to-command deadline: 75 ms;
- active command age: 100 ms;
- independent device-process watchdog: 500 ms;
- base stop before close or disconnect, followed by torque disable;
- explicit safety-latch clear, fresh observation, and rearm after a trip;
- a reachable physical power-isolation switch operated by a second person.

The software emergency stop is defense in depth, not a substitute for physical
power isolation. First motion must use blocks or a stand that keeps every wheel
off the floor. The work area must be clear before floor motion.

## Prepare the device

Use a dedicated Python environment on the Raspberry Pi. Pin both the release
and its source revision:

    git clone https://github.com/huggingface/lerobot.git
    cd lerobot
    git checkout 30da8e687a6dfc617fcd94afc367ac7071c376ce
    python3 -m venv .venv
    . .venv/bin/activate
    python -m pip install --disable-pip-version-check -e ".[lekiwi]"

Follow the upstream guide to assign motor IDs and calibrate. Record the
calibration file digest, device serials, camera identities, power configuration,
LeRobot revision, Python version, OS image, and RNE commit in the run record.

Copy the bridge from
adapters/hardware/rne_hardware_lekiwi/python/rne_hardware_lekiwi_device.py to
the device checkout. It imports LeRobot only when not in mock mode and refuses
an installed version other than 0.6.0.

Before touching hardware, exercise the exact host lifecycle with the bundled
dependency-free mock:

    mkdir -p artifacts
    cargo run -p rne_hardware_lekiwi --bin rne-lekiwi-session -- \
      --mock --mode shadow --samples 60 \
      --session-id rne.lekiwi.mock.shadow.001 \
      --output artifacts/lekiwi-mock-shadow.json

The command refuses to overwrite an existing artifact. Every child exchange
has a finite host response timeout. A device/gateway safety terminal still
writes valid evidence and exits with code 3. An incomplete exchange, such as a
host I/O timeout with no response, cannot be relabelled as a complete trace and
exits without a session artifact.

## Staged validation

Each stage starts a new process and a new RNE session. Never reuse authority
after a safety trip.

1. Process conformance: run cargo test -p rne_hardware_lekiwi on a clean host.
   This proves strict wire parsing, unit conversion, shadow authority denial,
   arm hold, base stop, and an independently firing watchdog with a mock
   backend.
2. Playback: verify the selected TaskSpec, profile golden, recorded calibration,
   and camera metadata without connecting hardware.
3. Elevated shadow: connect the device with all wheels raised. Run the host in
   shadow mode for 1,800 samples and confirm that the evidence contains no
   Actuate host frame.
4. Elevated HIL: use only zero then low-amplitude base commands. Inject command
   deadline, host-process termination, reconnect, stale command, limit, and
   emergency-stop cases. Confirm both the gateway stop and physical wheel stop.
5. Floor live: start at 0.02 m/s in a clear bounded area, then increase only
   within the profile limits. A second operator owns the physical cutoff.

The bundled host launches the bridge as a local child process. On the prepared
Raspberry Pi, capture elevated shadow evidence with:

    mkdir -p artifacts
    cargo run -p rne_hardware_lekiwi --bin rne-lekiwi-session -- \
      --physical-session --mode shadow --samples 1800 \
      --session-id <run-id> --robot-id <calibration-id> --port /dev/ttyACM0 \
      --bridge ./rne_hardware_lekiwi_device.py \
      --output artifacts/lekiwi-elevated-shadow.json

HIL/live additionally require the explicit `--allow-actuation` switch and a
separate `--confirm-cutoff-operator` acknowledgement that the second operator
is present at the reachable physical cutoff. Elevated HIL also requires
`--confirm-wheels-elevated`; floor live instead requires
`--confirm-clear-work-area`. The CLI rejects these acknowledgements in mock,
shadow, or the wrong physical stage. They are a fail-closed launch preflight,
not evidence that the procedure happened: the final manifest still requires
the distinct operators and typed power-isolation diagnostic. Begin with zero
commands, then use the bounded `--action-vx-m-s`, `--action-vy-m-s`, and
`--action-wz-rad-s` flags only after the staged safety checks above.

The bundled CLI makes four safety paths repeatable without changing the bridge:

    # Observation accepted, then controller exceeds the 75 ms deadline.
    rne-lekiwi-session ... --mode hil --allow-actuation \
      --confirm-cutoff-operator --confirm-wheels-elevated \
      --samples 1 --controller-delay-ms 80 --output deadline.json

    # Host does not refresh before both gateway and device watchdog bounds.
    rne-lekiwi-session ... --mode hil --allow-actuation \
      --confirm-cutoff-operator --confirm-wheels-elevated \
      --samples 2 --sample-period-ms 600 --output watchdog.json

    # Gateway rejects the over-limit action and sends only a zero stop.
    rne-lekiwi-session ... --mode hil --allow-actuation \
      --confirm-cutoff-operator --confirm-wheels-elevated \
      --samples 1 --action-vx-m-s 0.100001 --output limit.json

    # Operator emergency stop after one acknowledged zero command.
    rne-lekiwi-session ... --mode hil --allow-actuation \
      --confirm-cutoff-operator --confirm-wheels-elevated \
      --samples 1 --emergency-stop-after-samples 1 --output emergency.json

The first floor-live command replaces `--confirm-wheels-elevated` with
`--confirm-clear-work-area` and must retain `--confirm-cutoff-operator`:

    rne-lekiwi-session ... --mode live --allow-actuation \
      --confirm-cutoff-operator --confirm-clear-work-area \
      --samples 1 --action-vx-m-s 0.02 --output live-first-motion.json

Replace `...` with the physical-session, session-id, robot-id, port, and bridge
arguments from the shadow command. Each safety terminal writes validated
evidence and exits 3. Host-process termination and reconnect remain external
procedures: terminate one live host under the physical observer, confirm the
independent device stop, then start a new process and session ID. A terminated
request with no device response is intentionally not called a complete wire
trace.

Standard input and output are exclusively hardware wire v1 JSON Lines. Run the
bundled CLI locally on the Pi. The Rust runner exposes a transport-neutral
interface for a separately reviewed SSH transport, but this release does not
claim one. Standard error may be retained as diagnostics but is not protocol
evidence.

## Seal and verify the physical evidence set

Copy `docs/examples/lekiwi-physical-evidence-draft-v1.json` into the evidence
directory. Also copy the two operator-diagnostic JSON examples into its
`diagnostics` directory. Replace every `REPLACE_*` value, keep every artifact
path relative to the manifest, and change each attestation and diagnostic
boolean only after the physical procedure was performed. The host-termination latency plus measurement
uncertainty must be at most the pinned 500 ms device-watchdog deadline. The
primary and cutoff operators must be different declared operators.

The Failure Capsule requires a standalone copy of the wire trace as well as
the complete profile-bound session. Extract it without hand-editing JSON:

    cargo run -p xtask -- lekiwi-evidence extract-trace \
      artifacts/lekiwi-elevated-shadow.json artifacts/lekiwi-shadow-trace.json

After every referenced file exists, `seal` hashes the exact bytes, rejects
escaping paths and symlinks, computes the self-excluding manifest digest, and
refuses to overwrite its output. The draft and sealed output stay in the same
directory so relative paths cannot silently change meaning:

    cargo run -p xtask -- lekiwi-evidence seal \
      artifacts/physical-evidence-draft.json \
      artifacts/physical-evidence.json

Then run the semantic verifier:

    cargo run -p xtask -- lekiwi-evidence verify \
      artifacts/physical-evidence.json

Verification rehashes every indexed file and rejects a mock identity, mixed
physical device IDs, reused session IDs, an Actuate frame in shadow, fewer than
1,800 shadow observations, a failing or incomplete shadow comparison, wrong
HIL stop reasons, a reconnect without a fresh completed session, floor motion
above 0.02 m/s per linear axis, a live success with no motion, and a live
failure that completed normally. It also streams and hashes the complete
camera dataset, requires calibrated non-empty front and wrist RGB streams,
binds a passing offline depth evaluation to that dataset, and fully verifies
the Failure Capsule directory. The typed power-isolation and host-termination
diagnostics must agree with the manifest, the terminated request must not be
presented as a complete session, and the reconnect ID must equal the fresh
completed HIL session. The capsule must contain the exact indexed
TaskSpec, shadow session and comparison bytes, a standalone wire trace, and a
simulation failure replay from the same RNE commit.

## Required exit evidence

The v0.6 reference-device claim remains open until one source commit has all of
the following:

- the exact reference profile and TaskSpec;
- a complete `rne_lekiwi_reference_session` artifact whose physical device ID
  begins with `rne.lekiwi_so101.physical.v1:`; mock identity is never physical
  evidence;
- an elevated shadow comparison with declared per-tensor tolerances;
- all six safety cases on the physical device, including explicit reconnect and
  rearm;
- a successful low-speed floor live session and an intentionally failed session;
- front and wrist dataset streams with timestamp, latency, calibration, and
  payload-hash verification;
- a Failure Capsule containing the task, session, trace, comparison, and
  corresponding simulation failure replay;
- device inventory, calibration digest, operator checklist, and an explicit
  statement that physical power isolation was tested;
- reproduction from a clean host checkout.

These requirements are represented by
`rne_lekiwi_physical_evidence_manifest` schema v1. A structurally valid
manifest is still not a pass: only the command above emits the final verified
line after replaying every nested contract.

A profile golden or mock process pass is not physical evidence. Until these
artifacts exist, roadmap language must say that real-hardware evidence remains
open. Even after this base-only manifest passes, the same-contract flagship
physical gate remains open until the separately hashed projection and
parent-controller evidence pass; the two claims must not be conflated.

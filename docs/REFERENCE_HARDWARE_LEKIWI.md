# LeKiwi + SO-101 reference hardware

Status: selected and executable profile; physical shadow/HIL/live evidence not
yet captured.

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

## Staged validation

Each stage starts a new process and a new RNE session. Never reuse authority
after a safety trip.

1. Process conformance: run cargo test -p rne_hardware_lekiwi on a clean host.
   This proves strict wire parsing, unit conversion, shadow authority denial,
   arm hold, base stop, and an independently firing watchdog with a mock
   backend.
2. Playback: verify the selected TaskSpec, profile golden, recorded calibration,
   and camera metadata without connecting hardware.
3. Elevated shadow: connect the device with all wheels raised. Open shadow
   mode, poll observations for 1,800 samples, and confirm that the bridge rejects
   any non-safety actuation.
4. Elevated HIL: use only zero then low-amplitude base commands. Inject command
   deadline, host-process termination, reconnect, stale command, limit, and
   emergency-stop cases. Confirm both the gateway stop and physical wheel stop.
5. Floor live: start at 0.02 m/s in a clear bounded area, then increase only
   within the profile limits. A second operator owns the physical cutoff.

The production device command is:

    python rne_hardware_lekiwi_device.py --robot-id <calibration-id> --port /dev/ttyACM0

Standard input and output are exclusively hardware wire v1 JSON Lines. Run the
process locally on the Pi or behind an unbuffered SSH standard-I/O transport.
Standard error may be retained as diagnostics but is not protocol evidence.

## Required exit evidence

The v0.6 reference-device claim remains open until one source commit has all of
the following:

- the exact reference profile and TaskSpec;
- a complete bounded wire trace and correlated gateway session evidence;
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

A profile golden or mock process pass is not physical evidence. Until these
artifacts exist, roadmap language must say that real-hardware evidence remains
open.

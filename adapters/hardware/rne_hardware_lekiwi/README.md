# RNE LeKiwi reference adapter

This adapter selects LeKiwi + SO-101 as RNE's affordable v0.6 reference
hardware. It is brand-specific and stays outside every core crate.

The v1 contract is deliberately base-only for actuation:

- observations contain five arm angles, gripper position, planar velocity, and
  yaw rate in explicit RNE units;
- actions contain x/y velocity and yaw rate, limited to 0.1 m/s per axis and
  30 deg/s;
- normal base commands hold the six latest measured arm/gripper values;
- every safety frame calls the independent base-stop path and never converts
  zeroes into arm position targets;
- front and wrist color frames remain out-of-band dataset streams.

Inspect the committed profile or TaskSpec:

    cargo run -p rne_hardware_lekiwi --bin rne-lekiwi-profile
    cargo run -p rne_hardware_lekiwi --bin rne-lekiwi-profile -- --task-only

Run the dependency-free child-process conformance suite:

    cargo test -p rne_hardware_lekiwi

Run the complete profile-bound host path against the Python mock and write a
non-overwriting evidence artifact:

    cargo run -p rne_hardware_lekiwi --bin rne-lekiwi-session -- --mock --mode shadow --samples 60 --session-id rne.lekiwi.mock.shadow.001 --output lekiwi-shadow-session.json

The session host injects monotonic host ticks into the gateway and preserves
Open, every observation/action decision, the final zero stop, Close, and the
gateway event stream. Mock ready responses use
`rne.lekiwi_so101.mock.v1`; the physical bridge uses a distinct
`rne.lekiwi_so101.physical.v1:<robot-id>` identity.

The Python bridge is pinned to LeRobot 0.6.0. Its mock mode has no third-party
Python dependency:

    python adapters/hardware/rne_hardware_lekiwi/python/rne_hardware_lekiwi_device.py --mock

Do not use live mode from this README alone. The physical setup, staged
authority procedure, required power isolation, and evidence checklist are in
[the reference-hardware runbook](../../../docs/REFERENCE_HARDWARE_LEKIWI.md).
The repository does not claim a real-hardware pass until the resulting
artifacts are committed and independently verified.

The final evidence set uses a versioned manifest rather than a handwritten
checklist. `cargo run -p xtask -- lekiwi-evidence seal DRAFT OUTPUT` hashes and
seals it; `cargo run -p xtask -- lekiwi-evidence verify MANIFEST` replays all
nested session, dataset, comparison, and Failure Capsule contracts. See the
runbook for the required draft fields and stage semantics.

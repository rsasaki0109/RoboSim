# External hardware adapter conformance

`rne-hardware-conformance` tests an arbitrary child process through the public
hardware JSON Lines protocol. The runner does not link the adapter, vendor SDK,
ROS 2, or transport implementation. It binds the process to one validated
`TaskSpec`, launches a fresh process per case, bounds every response wait, and
kills only the child it launched after a timeout or protocol failure.

The report is deterministic and machine-readable. It hashes the exact TaskSpec,
the adapter implementation selected by `--subject`, and normalized process
arguments. For a native adapter, `--adapter` is also the default subject. For an
interpreted adapter, use the interpreter as `--adapter` and the script as
`--subject`; an argument exactly matching the subject path is normalized before
the argument digest so checkout location does not change the contract.

## Safety boundary

Full conformance sends one normal action inside TaskSpec limits. Therefore the
CLI refuses to run without `--allow-hil`. This flag is authorization only for a
sandbox, simulator, process mock, or correctly isolated HIL rig. Never point the
command at a powered physical robot without completing that robot's authority
and power-isolation runbook. The kit never selects `live` mode.

A conformance pass proves process-protocol behavior. It does not prove physical
watchdog timing, wiring, power isolation, calibration, payload capacity, or
real-device safety. Those remain separate hardware evidence.

## Canonical cases

Report schema v1 contains nine checks in fixed order:

1. exact open identity and TaskSpec dimensions;
2. rejection and recovery from an incorrect open-time TaskSpec width;
3. two increasing, dtype-valid observations;
4. one gateway-produced, TaskSpec-bounded HIL action followed by safe stop;
5. explicit zero-output safe-stop acknowledgment;
6. denial of normal actuation in shadow mode while safe stop remains available;
7. rejection of a duplicate request sequence;
8. rejection of a cross-session request;
9. rejection of a wrong-width action before actuation.

Every passing case also requires a correlated response, a safe stop where
authority was opened, an acknowledged close, zero exit status, and completion
within the configured timeout. Semantic failures produce a valid report with a
nonzero CLI exit; unreadable inputs or invalid configuration are command errors.

## Run the bundled reference process

From an extracted Linux release bundle:

```bash
./bin/rne-hardware-conformance \
  --adapter ./bin/rne-hardware-mock-device \
  --adapter-arg --device-id \
  --adapter-arg external-mock-v1 \
  --adapter-arg --expected-task-id \
  --adapter-arg rne.diff_drive.goal.v1 \
  --adapter-arg --observation-width \
  --adapter-arg 9 \
  --adapter-arg --action-width \
  --adapter-arg 2 \
  --task assets/tasks/diff_drive_goal.task.json \
  --allow-hil \
  --output hardware-adapter-conformance.json
```

Use the corresponding `.exe` paths on Windows. `--adapter-arg` deliberately
accepts values beginning with `--`, so launcher-specific flags cross the CLI
without a shell command string.

The source gate also runs the same public catalog against the dependency-free
Rust process mock and the independently implemented Python LeKiwi mock bridge.
Release rehearsal runs only installed native binaries and the bundled TaskSpec.

## Third-party evidence

Publish the report together with the hashed adapter subject and TaskSpec. A
reviewer should verify the hashes, confirm that the subject is the implementation
actually launched, inspect every canonical check, and confirm the test ran
against a sandbox or isolated HIL target. RNE's 1.0 external-use gate additionally
requires an independently maintained adapter or backend; an in-repository mock
pass is reference evidence, not that external adoption claim.

Failure Capsules recognize this report kind only when those exact adapter and
TaskSpec bytes are included as evidence. The capsule verifier checks both
subject hashes and the negotiated TaskSpec identity before accepting the set.

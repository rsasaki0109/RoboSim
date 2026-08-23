# Gazebo Harmonic OpenArm adapter

This process adapter executes the official OpenArm v2 right-arm model in
Gazebo Harmonic and translates the RNE external-simulator JSONL contract into
fixed-step Gazebo commands. It is an adapter, not an RNE physics backend: no
Gazebo, ROS 2, DDS, or simulator-specific type enters a core crate.

The example contract exposes nine joint positions and velocities as its
observation and accepts nine joint-position targets as its action. A bounded
proportional servo becomes Gazebo joint-velocity commands during PreUpdate;
positions and velocities are sampled during PostUpdate. Every accepted action
advances exactly one 16,666,667 ns contract step.

## Requirements

- Ubuntu 22.04 amd64
- Gazebo Harmonic (`gz-sim8`)
- `python3-gz-sim8`
- a release build containing `rne-simulator-conformance`

The checked runtime manifest records the exact locally validated Gazebo
version. Regenerate it when deliberately qualifying another patch version.

## Run conformance

From the repository root on Ubuntu:

```bash
cargo build --locked -p rne_hardware_gateway --bin rne-simulator-conformance
target/debug/rne-simulator-conformance \
  --adapter /usr/bin/python3 \
  --subject adapters/simulator/rne_gazebo_harmonic/rne_gazebo_harmonic_adapter.py \
  --runtime-manifest adapters/simulator/rne_gazebo_harmonic/runtime.json \
  --task adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json \
  --timeout-ms 15000 \
  --output artifacts/gazebo-openarm-conformance/report.json \
  --adapter-arg adapters/simulator/rne_gazebo_harmonic/rne_gazebo_harmonic_adapter.py \
  --adapter-arg --runtime-manifest \
  --adapter-arg adapters/simulator/rne_gazebo_harmonic/runtime.json \
  --adapter-arg --task \
  --adapter-arg adapters/simulator/rne_gazebo_harmonic/openarm_right_joint_tracking.task.json
```

Gazebo Harmonic's DART backend currently reports that it cannot create the
URDF gripper's mimic constraint. The adapter therefore commands and observes
both finger joints explicitly. The upstream bimanual URDF also uses negative
mesh scales for its mirrored left arm, which DART rejects; this first fixture
intentionally qualifies the official positive-scale right-arm URDF only.

This in-repository adapter proves real external-simulator execution but does
not count as independent third-party adapter evidence.

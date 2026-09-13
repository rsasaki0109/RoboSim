# Unitree G1 joint-space locomotion RL

This is the Python learning boundary for the **joint-space** G1 locomotion
episode, `rne.unitree_g1.joint_locomotion.v1`. It replaces the earlier
three-parameter "stepper" gait, which could only shuffle in place, with a
twelve-leg-joint action space that can actually step.

The native Rust episode owns the physics, observation, reward, deterministic
seed, and termination. Python only adapts it to Gymnasium and runs PPO.

## Contract

- **Action** (12): normalized leg-joint offsets in `[-1, 1]` around the nominal
  stance, order left hip pitch/roll/yaw, knee, ankle pitch/roll, then right.
  Targets are `nominal + action * 0.25 rad`, tracked by the hybrid plant (eight
  proximal joints under torque PD, four ankles position-servoed).
- **Observation** (46): base angular velocity (3), projected gravity (3),
  velocity command (2), leg joint position (12), leg joint velocity (12),
  previous action (12), gait clock sin/cos (2).
- **Reward**: forward/yaw velocity tracking (exp kernel), upright, height,
  lateral-slip and action-rate/magnitude penalties, swing air-time bonus,
  alive bonus, and a fall penalty.

The observation/reward recipe follows published humanoid RL locomotion stacks
(`unitree_rl_gym`, `mujoco_playground`): projected gravity + angular velocity +
clock observations, velocity-tracking reward, and a feet air-time bonus.

## Build and run

```text
.venv/bin/maturin develop -m crates/rne_py/Cargo.toml --release
.venv/bin/python examples/93_g1_joint_locomotion_rl/run.py --smoke
.venv/bin/python examples/93_g1_joint_locomotion_rl/train_ppo.py --smoke
```

## Train

```text
.venv/bin/python examples/93_g1_joint_locomotion_rl/train_ppo.py \
    --timesteps 20000000 --n-envs 8 --repeat 4 --command 0.4
```

`--vec native` (default) uses `rne_py.UnitreeG1JointBatch`, which steps every
environment on the available cores in one Rust call with action repeat
(`--repeat`, the usual OSS decimation). `--vec subproc` and `--vec dummy` keep
the per-env Python paths for comparison.

The trainer prints a deterministic evaluation: forward displacement, mean
speed, and minimum pelvis height over one episode. The pre-joint stepper pinned
`0.0276 m/s`, so any policy clearly above that is genuine stepping rather than
in-place transport.

## Status

This is a working learning boundary, not a solved walk. On an 8-core CPU host
the native batch reaches ~2700 physics ticks/s in isolation, but end-to-end PPO
is ~262 env-steps/s because the CPU policy update dominates, so a 20M-step run
is ~20 hours. A GPU PPO path (or a distilled policy) is the next step; see
`docs/G1_LOCOMOTION.md` and `docs/PLAN_LEGGED_LOCOMOTION_FRONTIER.md`.

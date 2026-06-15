# 27 — Mobile manipulator RL environment

A gymnasium-style reinforcement-learning wrapper around the RNE mobile manipulator,
built on the `rne_py` Python bindings (`MobileManipulatorEpisode`). The same physics,
contact-triggered weld grasp, and `Place` reward used by the Rust example 26 are driven
from Python here.

## Build the bindings

```bash
.venv/bin/pip install maturin
.venv/bin/maturin develop -m crates/rne_py/Cargo.toml
```

## Run the smokes

```bash
# gym-style env wrapper + scripted pick-and-place rollout
.venv/bin/python examples/27_mobile_manipulator_rl/run.py --smoke

# self-contained training loop that LEARNS the reach task (no external deps)
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --smoke
```

`run.py` wraps the episode as a gymnasium-style env (scripted pick-and-place rollout).
`gymnasium` and `numpy` are optional: with them installed the env subclasses
`gymnasium.Env` and exposes `action_space` / `observation_space`; without them it returns
plain Python lists.

## Training loop (`train.py`)

`train.py` optimizes a small linear state-feedback policy on the dense-reward **reach**
task (`MobileManipulatorEpisode("reach")`) with the Cross-Entropy Method (CEM) — a
derivative-free RL algorithm needing only `rne_py` and the standard library. The mean
episode reward climbs from a failing policy (~2) to a solved reach (~12) over a handful of
iterations, demonstrating an end-to-end learning loop without `torch`/`gymnasium`:

```bash
.venv/bin/python examples/27_mobile_manipulator_rl/train.py          # 20 iterations
.venv/bin/python examples/27_mobile_manipulator_rl/train.py --smoke  # short, asserts learning
```

The reach target is placed where the arm only reaches it under active control (passive
settling fails), so the reward signal is meaningful for learning. Each CEM iteration
evaluates the whole candidate population in lock-step with
`rne_py.VectorizedMobileManipulatorEnv`, advancing every policy with a single batched
`step` call (each policy's reward is frozen the moment its episode ends).

## Spaces

- **Action** (`shape=(5,)`): `[left_wheel, right_wheel, shoulder, elbow, gripper]`
  velocities (rad/s; gripper is m/s, negative closes).
- **Observation** (`shape=(12,)`): base pose `(x, y, z, yaw)`, end-effector `(x, y, z)`,
  `shoulder`, `elbow`, `gripper` joint positions, `wrist_camera_pixels`,
  `joint_state_count`.

## Training with Stable-Baselines3

`stable-baselines3` is not a project dependency; install it into the venv to train:

```bash
.venv/bin/pip install gymnasium numpy stable-baselines3
```

```python
from stable_baselines3 import PPO
from run import MobileManipulatorPlaceEnv

env = MobileManipulatorPlaceEnv("place")
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)
```

Note: the `mm_minimal` arm is a horizontal SCARA, so `Place` is a horizontal carry and
release calibrated to a deterministic target. Reward is shaped by the object's horizontal
distance to that target plus a success bonus on a settled placement.

"""Gymnasium-style Unitree Go2 gait wrapper and headless smoke."""

import math
import os
import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with: "
        ".venv/bin/maturin develop -m crates/rne_py/Cargo.toml --release"
    )

try:
    import gymnasium as gym
    import numpy as np

    _HAS_GYM = True
    _Base = gym.Env
except ImportError:  # pragma: no cover - exercised without optional RL extras
    _HAS_GYM = False
    _Base = object


ACTION_DIM = 5
OBSERVATION_DIM = 21
ACTION_LOW = [-0.0, 0.0, -0.8, -0.3, -0.5]
ACTION_HIGH = [0.3, 0.4, 0.8, 0.3, 0.5]


class UnitreeGo2GaitEnv(_Base):
    """RL environment backed by the native deterministic Go2 episode."""

    metadata = {"render_modes": []}

    def __init__(self, max_steps=600, seed=1):
        super().__init__()
        self._episode = rne_py.UnitreeGo2GaitEpisode(max_steps, seed)
        if _HAS_GYM:
            self.action_space = gym.spaces.Box(
                low=np.asarray(ACTION_LOW, dtype=np.float32),
                high=np.asarray(ACTION_HIGH, dtype=np.float32),
                dtype=np.float32,
            )
            self.observation_space = gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(OBSERVATION_DIM,), dtype=np.float32
            )

    def _wrap_observation(self, observation):
        if len(observation) != OBSERVATION_DIM:
            raise RuntimeError(f"unexpected Go2 observation length: {len(observation)}")
        if not all(math.isfinite(value) for value in observation):
            raise RuntimeError("Go2 observation contains a non-finite value")
        if _HAS_GYM:
            return np.asarray(observation, dtype=np.float32)
        return observation

    def reset(self, *, seed=None, options=None):
        del seed, options
        result = self._episode.reset()
        return self._wrap_observation(result.observation), {}

    def step(self, action):
        values = list(action)
        if len(values) != ACTION_DIM:
            raise ValueError(f"expected {ACTION_DIM} action values, got {len(values)}")
        result = self._episode.step(
            stride_rad=float(values[0]),
            foot_lift_rad=float(values[1]),
            roll_correction_rad=float(values[2]),
            pitch_correction_rad=float(values[3]),
            lateral_extension_rad=float(values[4]),
        )
        return (
            self._wrap_observation(result.observation),
            float(result.reward),
            bool(result.terminated),
            bool(result.truncated),
            {"step_in_episode": self._episode.step_in_episode},
        )


def main():
    env = UnitreeGo2GaitEnv(max_steps=64, seed=6602)
    observation, _ = env.reset()
    action = [0.12, 0.16, 0.0, 0.0, 0.0]
    total_reward = 0.0
    steps = 0
    while steps < 32:
        observation, reward, terminated, truncated, _ = env.step(action)
        total_reward += reward
        steps += 1
        if terminated or truncated:
            break
    if "--smoke" in sys.argv:
        if len(observation) != OBSERVATION_DIM or not math.isfinite(total_reward):
            raise SystemExit("locomotion RL smoke failed: invalid rollout")
        backend = "gymnasium" if _HAS_GYM else "list-fallback"
        print(
            f"locomotion RL smoke ok: reward={total_reward:.3f} steps={steps} backend={backend}"
        )
        return
    print(f"Go2 gait rollout: reward={total_reward:.3f} steps={steps}")


if __name__ == "__main__":
    main()

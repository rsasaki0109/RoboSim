"""Gymnasium-style Unitree Go2 gait wrapper and headless smoke."""

import json
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


def _tensor_size(tensor):
    size = 1
    for dimension in tensor["shape"]:
        size *= dimension
    return size


def _space_size(space):
    return sum(_tensor_size(tensor) for tensor in space["tensors"])


def _space_dtype(space):
    dtypes = {tensor["dtype"] for tensor in space["tensors"]}
    if len(dtypes) != 1:
        raise RuntimeError(f"flat Gymnasium space requires one dtype, got {sorted(dtypes)}")
    dtype = dtypes.pop()
    if dtype not in {"f32", "f64"}:
        raise RuntimeError(f"unsupported Gymnasium floating dtype: {dtype}")
    return dtype


def _space_bounds(space):
    lower = []
    upper = []
    for tensor in space["tensors"]:
        size = _tensor_size(tensor)
        bounds = tensor["bounds"]
        if bounds is None:
            lower.extend([-math.inf] * size)
            upper.extend([math.inf] * size)
            continue
        for destination, source in ((lower, bounds["lower"]), (upper, bounds["upper"])):
            if len(source) == 1:
                destination.extend(source * size)
            elif len(source) == size:
                destination.extend(source)
            else:
                raise RuntimeError("TaskSpec bounds do not match flattened tensor size")
    return lower, upper


class UnitreeGo2GaitEnv(_Base):
    """RL environment backed by the native deterministic Go2 episode."""

    metadata = {"render_modes": []}

    def __init__(self, max_steps=600, seed=1):
        super().__init__()
        self._max_steps = max_steps
        self._episode = rne_py.UnitreeGo2GaitEpisode(max_steps, seed)
        canonical = rne_py.canonical_task_spec_json(self._episode.task_spec_json())
        self.task_spec = json.loads(canonical)
        if self.task_spec["schema_version"] != rne_py.TASK_SPEC_SCHEMA_VERSION:
            raise RuntimeError("Python and native TaskSpec schema versions differ")
        self.action_dim = _space_size(self.task_spec["action"])
        self.observation_dim = _space_size(self.task_spec["observation"])
        action_dtype = _space_dtype(self.task_spec["action"])
        observation_dtype = _space_dtype(self.task_spec["observation"])
        if _HAS_GYM:
            numpy_dtypes = {"f32": np.float32, "f64": np.float64}
            action_low, action_high = _space_bounds(self.task_spec["action"])
            observation_low, observation_high = _space_bounds(
                self.task_spec["observation"]
            )
            self.action_space = gym.spaces.Box(
                low=np.asarray(action_low, dtype=numpy_dtypes[action_dtype]),
                high=np.asarray(action_high, dtype=numpy_dtypes[action_dtype]),
                dtype=numpy_dtypes[action_dtype],
            )
            self.observation_space = gym.spaces.Box(
                low=np.asarray(observation_low, dtype=numpy_dtypes[observation_dtype]),
                high=np.asarray(observation_high, dtype=numpy_dtypes[observation_dtype]),
                dtype=numpy_dtypes[observation_dtype],
            )

    def _wrap_observation(self, observation):
        if len(observation) != self.observation_dim:
            raise RuntimeError(f"unexpected Go2 observation length: {len(observation)}")
        if not all(math.isfinite(value) for value in observation):
            raise RuntimeError("Go2 observation contains a non-finite value")
        if _HAS_GYM:
            return np.asarray(observation, dtype=self.observation_space.dtype)
        return observation

    def reset(self, *, seed=None, options=None):
        del options
        if _HAS_GYM:
            super().reset(seed=seed)
        if seed is not None:
            self._episode = rne_py.UnitreeGo2GaitEpisode(self._max_steps, seed)
        result = self._episode.reset()
        return self._wrap_observation(result.observation), {
            "task_id": self.task_spec["task_id"]
        }

    def step(self, action):
        values = list(action)
        if len(values) != self.action_dim:
            raise ValueError(f"expected {self.action_dim} action values, got {len(values)}")
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
            {
                "step_in_episode": self._episode.step_in_episode,
                "task_id": self.task_spec["task_id"],
            },
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
        if len(observation) != env.observation_dim or not math.isfinite(total_reward):
            raise SystemExit("locomotion RL smoke failed: invalid rollout")
        backend = "gymnasium" if _HAS_GYM else "list-fallback"
        print(
            f"locomotion RL smoke ok: reward={total_reward:.3f} steps={steps} backend={backend}"
        )
        return
    print(f"Go2 gait rollout: reward={total_reward:.3f} steps={steps}")


if __name__ == "__main__":
    main()

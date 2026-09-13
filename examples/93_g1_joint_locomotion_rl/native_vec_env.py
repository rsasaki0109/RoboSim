"""Stable-Baselines3 VecEnv backed by the native parallel G1 batch.

`rne_py.UnitreeG1JointBatch` steps every environment on the available cores in
one Rust call, which avoids the per-step Python round-trip that caps the
`SubprocVecEnv` path. Actions are held for `repeat` ticks (decimation), the
usual OSS locomotion control rate.
"""

import json
import math

import numpy as np
from stable_baselines3.common.vec_env import VecEnv

import rne_py
from gymnasium import spaces


def _tensor_size(tensor):
    size = 1
    for dimension in tensor["shape"]:
        size *= dimension
    return size


def _space_size(space):
    return sum(_tensor_size(tensor) for tensor in space["tensors"])


class NativeG1VecEnv(VecEnv):
    """Vectorized G1 joint-locomotion environment over one Rust batch call."""

    def __init__(
        self,
        n_envs=8,
        seed=1,
        command_forward_m_s=0.4,
        command_yaw_rate_rad_s=0.0,
        max_steps=500,
        repeat=4,
    ):
        self._batch = rne_py.UnitreeG1JointBatch(
            n_envs, seed, command_forward_m_s, command_yaw_rate_rad_s, max_steps
        )
        canonical = rne_py.canonical_task_spec_json(self._batch.task_spec_json())
        self.task_spec = json.loads(canonical)
        self.action_dim = _space_size(self.task_spec["action"])
        self.observation_dim = _space_size(self.task_spec["observation"])
        self._repeat = repeat
        self._actions = None
        self._n_envs = n_envs
        self.render_mode = None
        action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(self.action_dim,), dtype=np.float64
        )
        observation_space = spaces.Box(
            low=-math.inf, high=math.inf, shape=(self.observation_dim,), dtype=np.float64
        )
        super().__init__(n_envs, observation_space, action_space)

    def reset(self):
        result = self._batch.reset()
        return np.asarray(result.observations, dtype=np.float64)

    def step_async(self, actions):
        self._actions = np.asarray(actions, dtype=np.float64)

    def step_wait(self):
        result = self._batch.step(
            [list(action) for action in self._actions], self._repeat
        )
        observations = np.asarray(result.observations, dtype=np.float64)
        rewards = np.asarray(result.rewards, dtype=np.float64)
        dones = np.logical_or(
            np.asarray(result.terminated), np.asarray(result.truncated)
        )
        infos = [
            {"episode_index": index} for index in result.episode_indices
        ]
        return observations, rewards, dones, infos

    def close(self):
        return None

    def get_images(self):
        return [None] * self.num_envs

    def env_is_wrapped(self, wrapper_class, indices=None):
        return [False] * self.num_envs

    def env_method(self, method_name, *method_args, indices=None, **method_kwargs):
        raise NotImplementedError("NativeG1VecEnv has no per-env Python methods")

    def get_attr(self, attr_name, indices=None):
        value = getattr(self, attr_name, None)
        return [value] * self._n_envs

    def set_attr(self, attr_name, value, indices=None):
        setattr(self, attr_name, value)

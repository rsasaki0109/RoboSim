"""Gymnasium-style RL wrapper around the RNE mobile manipulator.

Exposes ``MobileManipulatorPlaceEnv`` with a ``reset()`` / ``step(action)`` API
suitable for reinforcement learning. ``gymnasium`` and ``numpy`` are optional: when
available the env subclasses ``gymnasium.Env`` and exposes action/observation spaces;
otherwise it degrades to a plain class returning Python lists, so the smoke runs in a
bare interpreter with only ``rne_py`` installed.

Run the smoke::

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/run.py --smoke
"""

import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/pip install maturin\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

try:
    import gymnasium as gym
    import numpy as np

    _HAS_GYM = True
    _Base = gym.Env
except ImportError:  # pragma: no cover - exercised only without gymnasium installed
    _HAS_GYM = False
    _Base = object

# Action layout: [left_wheel, right_wheel, shoulder, elbow, gripper] (rad/s; gripper m/s).
ACTION_DIM = 5
OBSERVATION_DIM = 18


def _observation_to_list(obs):
    return [
        obs.base_x,
        obs.base_y,
        obs.base_z,
        obs.base_yaw,
        obs.ee_x,
        obs.ee_y,
        obs.ee_z,
        obs.shoulder_position,
        obs.elbow_position,
        obs.gripper_position,
        float(obs.wrist_camera_pixels),
        float(obs.joint_state_count),
        # Goal-relative end-effector offset (zero unless a reach goal is set).
        obs.target_dx,
        obs.target_dy,
        obs.target_dz,
        obs.wrist_depth_center_m,
        obs.wrist_depth_min_m,
        float(obs.target_object_index),
    ]


class MobileManipulatorPlaceEnv(_Base):
    """RL environment wrapping ``rne_py.MobileManipulatorEpisode``."""

    metadata = {"render_modes": []}

    def __init__(self, task: str = "place"):
        super().__init__()
        self._episode = rne_py.MobileManipulatorEpisode(task)
        if _HAS_GYM:
            self.action_space = gym.spaces.Box(
                low=-6.0, high=6.0, shape=(ACTION_DIM,), dtype=np.float32
            )
            self.observation_space = gym.spaces.Box(
                low=-np.inf, high=np.inf, shape=(OBSERVATION_DIM,), dtype=np.float32
            )

    def _wrap_obs(self, obs_list):
        if _HAS_GYM:
            return np.asarray(obs_list, dtype=np.float32)
        return obs_list

    def reset(self, *, seed=None, options=None):
        step = self._episode.reset()
        return self._wrap_obs(_observation_to_list(step.observation)), {}

    def step(self, action):
        a = list(action)
        step = self._episode.step(
            left_wheel_velocity_rad_s=a[0],
            right_wheel_velocity_rad_s=a[1],
            shoulder_velocity_rad_s=a[2],
            elbow_velocity_rad_s=a[3],
            gripper_velocity_rad_s=a[4],
        )
        info = {
            "is_grasping": self._episode.is_grasping,
            "total_reward": self._episode.total_reward,
        }
        return (
            self._wrap_obs(_observation_to_list(step.observation)),
            step.reward,
            step.terminated,
            step.truncated,
            info,
        )


def run_scripted_place(env) -> bool:
    """IK clutter pick-place rollout; returns True if the episode terminates."""
    policy = rne_py.IkClutterPickPlacePolicy()
    step = env._episode.reset()
    for _ in range(policy.total_steps()):
        left, right, shoulder, elbow, gripper, lift = policy.act(step.observation)
        step = env._episode.step(
            left_wheel_velocity_rad_s=left,
            right_wheel_velocity_rad_s=right,
            shoulder_velocity_rad_s=shoulder,
            elbow_velocity_rad_s=elbow,
            gripper_velocity_rad_s=gripper,
            lift_velocity_m_s=lift,
        )
        if step.terminated:
            return True
    return False


def main():
    smoke = "--smoke" in sys.argv
    env = MobileManipulatorPlaceEnv("place")
    placed = run_scripted_place(env)
    reward = env._episode.total_reward
    steps = env._episode.step_in_episode

    if smoke:
        if placed:
            backend = "gymnasium" if _HAS_GYM else "list-fallback"
            print(
                f"rl smoke ok: placed cube, reward={reward:.2f} steps={steps} backend={backend}"
            )
            return
        sys.exit("smoke failed: scripted pick-and-place did not terminate")

    print(f"placed={placed} reward={reward:.2f} steps={steps} gymnasium={_HAS_GYM}")


if __name__ == "__main__":
    main()

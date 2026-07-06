"""Stable-Baselines3 PPO integration for the mobile navigate-grasp-place clutter task.

Uses the gymnasium wrapper on ``mobile_clutter_place_center`` (pinned `mm_mobile_clutter`
scene, `clutter_cube_a`). The smoke only verifies the SB3 pipeline runs end-to-end on the
longer (~1600-step budget) mobile episode; learning quality is a tuning concern (see
``train_mobile_clutter.py`` for the deterministic CEM demo that actually solves the task).

    .venv/bin/pip install gymnasium numpy stable-baselines3
    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_mobile_clutter_ppo.py --smoke
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import rne_py  # noqa: F401
    from run import MobileManipulatorPlaceEnv
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

try:
    from stable_baselines3 import PPO
except ImportError:
    sys.exit(
        "stable-baselines3 is not installed. Install the RL extras with:\n"
        "  .venv/bin/pip install gymnasium numpy stable-baselines3"
    )


def make_env():
    return MobileManipulatorPlaceEnv("mobile_clutter_place_center")


def evaluate(model, episodes=3):
    env = make_env()
    total = 0.0
    for _ in range(episodes):
        obs, _ = env.reset()
        done = False
        while not done:
            if model is None:
                action = env.action_space.sample()
            else:
                action, _ = model.predict(obs, deterministic=True)
            obs, reward, terminated, truncated, _ = env.step(action)
            total += reward
            done = terminated or truncated
    return total / episodes


def main():
    smoke = "--smoke" in sys.argv
    timesteps = 3000 if smoke else 60000

    random_reward = evaluate(None)
    model = PPO("MlpPolicy", make_env(), verbose=0, seed=0, device="cpu")
    model.learn(total_timesteps=timesteps)
    trained_reward = evaluate(model)

    print(
        "PPO mobile clutter: "
        f"random={random_reward:.2f} trained={trained_reward:.2f} "
        f"timesteps={timesteps}"
    )

    if smoke:
        if math.isfinite(trained_reward):
            print(
                "mobile clutter ppo smoke ok: SB3 PPO trained end-to-end on "
                "mobile_clutter_place_center"
            )
            return
        sys.exit(
            "smoke failed: PPO mobile clutter evaluation did not produce a finite reward"
        )


if __name__ == "__main__":
    main()

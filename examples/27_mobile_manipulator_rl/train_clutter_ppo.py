"""Stable-Baselines3 PPO integration for the clutter place task.

Uses the gymnasium wrapper on ``clutter_place_center`` (pinned center cube).
The smoke only verifies the SB3 pipeline runs end-to-end; learning quality is
a tuning concern (see ``train_clutter.py`` for the deterministic CEM demo).

    .venv/bin/pip install gymnasium numpy stable-baselines3
    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_clutter_ppo.py --smoke
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
    return MobileManipulatorPlaceEnv("clutter_place_center")


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
        "PPO clutter: "
        f"random={random_reward:.2f} trained={trained_reward:.2f} "
        f"timesteps={timesteps}"
    )

    if smoke:
        if math.isfinite(trained_reward):
            print("clutter ppo smoke ok: SB3 PPO trained end-to-end on clutter_place_center")
            return
        sys.exit("smoke failed: PPO clutter evaluation did not produce a finite reward")


if __name__ == "__main__":
    main()

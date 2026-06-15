"""Stable-Baselines3 PPO integration for the mobile manipulator reach task.

Shows the gymnasium env wrapper plugged into a standard deep-RL library. This is the
*integration* example; the dependency-free `train.py` (Cross-Entropy Method) is the
deterministic learning demo. PPO on this task needs a long run and some tuning to beat a
random policy — the `--smoke` mode only checks the training pipeline runs end-to-end.

Install the extras first:

    .venv/bin/pip install gymnasium numpy stable-baselines3
    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_ppo.py          # full run
    .venv/bin/python examples/27_mobile_manipulator_rl/train_ppo.py --smoke  # short CI run
"""

import math
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

try:
    import rne_py  # noqa: F401  (imported for the helpful error if missing)
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
    return MobileManipulatorPlaceEnv("reach")


def evaluate(model, episodes=5):
    """Mean episode reward of `model` (or a random policy when `model` is None)."""
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
    timesteps = 4000 if smoke else 80000

    random_reward = evaluate(None)
    model = PPO("MlpPolicy", make_env(), verbose=0, seed=0, device="cpu")
    model.learn(total_timesteps=timesteps)
    trained_reward = evaluate(model)

    print(
        f"PPO reach: random={random_reward:.2f} trained={trained_reward:.2f} "
        f"timesteps={timesteps}"
    )

    if smoke:
        # Smoke only verifies the SB3 integration runs end-to-end and produces a
        # finite policy evaluation (learning quality is a tuning/horizon concern;
        # see train.py for the deterministic learning demo).
        if math.isfinite(trained_reward):
            print("ppo smoke ok: SB3 PPO trained end-to-end on the reach gym env")
            return
        sys.exit("smoke failed: PPO evaluation did not produce a finite reward")


if __name__ == "__main__":
    main()

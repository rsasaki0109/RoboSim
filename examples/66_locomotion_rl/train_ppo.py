"""Stable-Baselines3 PPO integration smoke for the native Go2 gait episode."""

import math
import sys

try:
    from stable_baselines3 import PPO
except ImportError:
    sys.exit("stable-baselines3 is not installed; run xtask ci or install the RL requirements")

from run import UnitreeGo2GaitEnv


def evaluate(model, episodes=1):
    total = 0.0
    for episode_index in range(episodes):
        env = UnitreeGo2GaitEnv(max_steps=96, seed=6602 + episode_index)
        observation, _ = env.reset()
        done = False
        while not done:
            action, _ = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, _ = env.step(action)
            total += float(reward)
            done = terminated or truncated
    return total / episodes


def main():
    smoke = "--smoke" in sys.argv
    env = UnitreeGo2GaitEnv(max_steps=96, seed=6602)
    model = PPO(
        "MlpPolicy",
        env,
        verbose=0,
        seed=6602,
        device="cpu",
        n_steps=32,
        batch_size=32,
        n_epochs=1,
    )
    model.learn(total_timesteps=256 if smoke else 4096)
    reward = evaluate(model)
    print(f"locomotion PPO: reward={reward:.3f} smoke={smoke}")
    if smoke:
        if not math.isfinite(reward):
            raise SystemExit("locomotion PPO smoke failed: non-finite evaluation")
        print("locomotion PPO smoke ok: SB3 trained against the native Go2 episode")


if __name__ == "__main__":
    main()

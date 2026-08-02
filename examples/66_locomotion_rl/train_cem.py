"""Small deterministic CEM smoke over the native Go2 gait episode."""

import math
import random
import sys

from run import UnitreeGo2GaitEnv


DEFAULT_MEAN = [0.12, 0.16, 0.0, 0.0, 0.0]
LOW = [0.0, 0.0, -0.8, -0.3, -0.5]
HIGH = [0.3, 0.4, 0.8, 0.3, 0.5]


def evaluate(action, horizon=48, seed=6602):
    env = UnitreeGo2GaitEnv(max_steps=horizon + 4, seed=seed)
    env.reset()
    total = 0.0
    for _ in range(horizon):
        _, reward, terminated, truncated, _ = env.step(action)
        total += float(reward)
        if terminated or truncated:
            break
    return total


def cem_smoke():
    rng = random.Random(6602)
    mean = list(DEFAULT_MEAN)
    sigma = [0.05, 0.05, 0.15, 0.08, 0.10]
    best = -math.inf
    for _ in range(2):
        candidates = []
        for _ in range(4):
            action = [
                max(lo, min(hi, rng.gauss(center, spread)))
                for center, spread, lo, hi in zip(mean, sigma, LOW, HIGH)
            ]
            candidates.append((evaluate(action), action))
        candidates.sort(reverse=True, key=lambda item: item[0])
        elites = candidates[:2]
        best = max(best, elites[0][0])
        mean = [sum(item[1][index] for item in elites) / len(elites) for index in range(5)]
        sigma = [
            max(0.01, (sum((item[1][index] - mean[index]) ** 2 for item in elites) / len(elites)) ** 0.5)
            for index in range(5)
        ]
    if not math.isfinite(best):
        raise RuntimeError("CEM produced a non-finite locomotion score")
    return best, mean


def main():
    best, mean = cem_smoke()
    if "--smoke" in sys.argv:
        print(f"locomotion CEM smoke ok: best_reward={best:.3f} mean={[round(x, 4) for x in mean]}")
        return
    print(f"Go2 gait CEM: best_reward={best:.3f} mean={mean}")


if __name__ == "__main__":
    main()

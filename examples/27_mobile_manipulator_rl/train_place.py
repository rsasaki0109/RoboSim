"""CEM training smoke for the tabletop pick-and-place task.

Demonstrates learning signal from the pre-grasp approach reward on the `place`
episode. Needs only ``rne_py`` and the standard library.

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_place.py --smoke
"""

import random
import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

ACTION_LIMIT = 6.0
EPISODE_STEPS = 400
PARAM_DIM = 4  # shoulder_bias, elbow_bias, gripper_close, shoulder_gain


def clamp(value, limit=ACTION_LIMIT):
    return max(-limit, min(limit, value))


def act(params, obs):
    shoulder_bias, elbow_bias, gripper_close, shoulder_gain = params
    return [
        0.0,
        0.0,
        clamp(shoulder_bias + shoulder_gain * obs.target_dx),
        clamp(elbow_bias + 2.0 * obs.target_dz),
        clamp(gripper_close),
    ]


def rollout(params):
    episode = rne_py.MobileManipulatorEpisode("place")
    step = episode.reset()
    for _ in range(EPISODE_STEPS):
        action = act(params, step.observation)
        step = episode.step(*action)
        if step.terminated or step.truncated:
            break
    return episode.total_reward


def cem_smoke():
    population = 12
    elite = 4
    iterations = 6
    mean = [0.5, 0.0, -2.5, 2.0]
    std = [0.8, 0.8, 0.5, 1.0]
    history = []

    for _ in range(iterations):
        candidates = []
        for _ in range(population):
            params = [random.gauss(mean[i], std[i]) for i in range(PARAM_DIM)]
            candidates.append((rollout(params), params))
        candidates.sort(key=lambda item: item[0], reverse=True)
        elites = candidates[:elite]
        history.append(elites[0][0])
        mean = [sum(item[1][i] for item in elites) / elite for i in range(PARAM_DIM)]
        std = [max(0.2, s * 0.9) for s in std]

    return history


def main():
    random.seed(0)
    smoke = "--smoke" in sys.argv
    history = cem_smoke()
    print(
        f"place CEM: start={history[0]:.2f} end={history[-1]:.2f} "
        f"history={[round(x, 2) for x in history]}"
    )
    if smoke:
        if max(history) > history[0]:
            print("place smoke ok: CEM improved pick-and-place reward")
            return
        sys.exit("smoke failed: place CEM did not improve reward")


if __name__ == "__main__":
    main()

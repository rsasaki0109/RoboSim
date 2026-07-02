"""CEM visuomotor reach training smoke using wrist depth observations.

Uses the goal-conditioned `reach_random` task and scales arm commands from
``wrist_depth_center_m``. Needs only ``rne_py`` and the standard library.

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train_visuomotor.py --smoke
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
EPISODE_STEPS = 300
PARAM_DIM = 4  # shoulder_gain, elbow_gain, depth_bias, depth_gain


def clamp(value, limit=ACTION_LIMIT):
    return max(-limit, min(limit, value))


def depth_scale(obs, depth_bias, depth_gain):
    if obs.wrist_depth_center_m <= 0.0:
        return 1.0
    raw = depth_bias / obs.wrist_depth_center_m * depth_gain
    return max(0.35, min(1.5, raw))


def act(params, obs):
    shoulder_gain, elbow_gain, depth_bias, depth_gain = params
    scale = depth_scale(obs, max(0.2, depth_bias), max(0.5, depth_gain))
    return [
        0.0,
        0.0,
        clamp(shoulder_gain * obs.target_dx * scale),
        clamp(elbow_gain * obs.target_dz * scale),
        0.0,
    ]


def rollout(params):
    episode = rne_py.MobileManipulatorEpisode("reach_random")
    step = episode.reset()
    for _ in range(EPISODE_STEPS):
        action = act(params, step.observation)
        step = episode.step(*action)
        if step.terminated or step.truncated:
            break
    return episode.total_reward


def cem_smoke():
    population = 16
    elite = 4
    iterations = 8
    mean = [2.5, 3.0, 0.55, 1.0]
    std = [1.0, 1.0, 0.15, 0.3]
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
        std = [max(0.15, s * 0.85) for s in std]

    return history


def main():
    random.seed(0)
    smoke = "--smoke" in sys.argv
    history = cem_smoke()
    print(
        "visuomotor CEM: "
        f"start={history[0]:.2f} end={history[-1]:.2f} history={[round(x, 2) for x in history]}"
    )
    if smoke:
        if max(history) > history[0]:
            print("visuomotor smoke ok: depth-conditioned CEM improved reach reward")
            return
        sys.exit("smoke failed: visuomotor CEM did not improve reward")


if __name__ == "__main__":
    main()

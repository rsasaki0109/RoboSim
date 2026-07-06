"""CEM training smoke for the tabletop pick-and-place task.

Demonstrates learning signal on the `place` episode: the cube spawns between the
gripper fingers (grasping is immediate under the stable arm dynamics), so the
learnable structure is the carry schedule — how fast and how long to sweep the
arm, and when to release — and the CEM must discover the timing that parks the
cube over the place target before opening the gripper (success bonus +10).
Needs only ``rne_py`` and the standard library.

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
PARAM_DIM = 4  # carry_rate, stop_frac, release_frac, elbow_rate


def clamp(value, limit=ACTION_LIMIT):
    return max(-limit, min(limit, value))


def rollout(params):
    """Scheduled grasp-carry-release rollout: sweep until ``stop_frac``, keep the
    gripper closed until ``release_frac``, then open and let the cube settle."""
    carry_rate, stop_frac, release_frac, elbow_rate = params
    stop_t = int(max(0.0, min(1.0, stop_frac)) * EPISODE_STEPS)
    release_t = int(max(0.0, min(1.0, release_frac)) * EPISODE_STEPS)
    episode = rne_py.MobileManipulatorEpisode("place")
    episode.reset()
    for t in range(EPISODE_STEPS):
        gripper = -2.5 if t < release_t else 3.0
        shoulder = carry_rate if t < stop_t else 0.0
        elbow = elbow_rate if t < stop_t else 0.0
        step = episode.step(0.0, 0.0, clamp(shoulder), clamp(elbow), gripper)
        if step.terminated or step.truncated:
            break
    return episode.total_reward


def cem_smoke():
    population = 12
    elite = 4
    iterations = 6
    # Deliberately poor start (slow sweep, release far too late) so the smoke
    # has headroom to demonstrate improvement.
    mean = [0.1, 0.15, 0.9, 0.0]
    std = [0.3, 0.25, 0.25, 0.3]
    history = []

    for _ in range(iterations):
        candidates = []
        for _ in range(population):
            params = [random.gauss(mean[i], std[i]) for i in range(PARAM_DIM)]
            candidates.append((rollout(params), params))
        candidates.sort(key=lambda item: item[0], reverse=True)
        elites = candidates[:elite]
        # Track the elite MEAN: it is robust to a single lucky draw in the
        # first iteration, unlike the per-iteration best.
        history.append(sum(item[0] for item in elites) / elite)
        mean = [sum(item[1][i] for item in elites) / elite for i in range(PARAM_DIM)]
        std = [max(0.05, s * 0.9) for s in std]

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
        # Require a solid margin so cross-platform reward jitter cannot fake
        # (or hide) the learning signal.
        if max(history) > history[0] + 1.0:
            print("place smoke ok: CEM improved pick-and-place reward")
            return
        sys.exit("smoke failed: place CEM did not improve reward")


if __name__ == "__main__":
    main()

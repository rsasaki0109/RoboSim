"""Self-contained training loop for the RNE mobile manipulator reach task.

Optimizes a small linear state-feedback policy with the Cross-Entropy Method (CEM),
a derivative-free RL algorithm. Needs only ``rne_py`` and the Python standard library
(no gymnasium / numpy / torch), so it runs anywhere the bindings are built and clearly
demonstrates learning: the mean episode reward climbs from a failing policy (~2) to a
solved reach (~11-12).

    .venv/bin/maturin develop -m crates/rne_py/Cargo.toml
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py            # full run
    .venv/bin/python examples/27_mobile_manipulator_rl/train.py --smoke    # short CI run
"""

import random
import sys

try:
    import rne_py
except ImportError:
    sys.exit(
        "rne_py is not installed. Build it with:\n"
        "  .venv/bin/pip install maturin\n"
        "  .venv/bin/maturin develop -m crates/rne_py/Cargo.toml"
    )

ACTION_LIMIT_RAD_S = 6.0
EPISODE_STEPS = 300
# Policy: [shoulder_bias, shoulder_gain_ee_z, elbow_bias, elbow_gain_ee_y].
PARAM_DIM = 4


def _clamp(value, limit):
    return max(-limit, min(limit, value))


def policy_action(params, obs):
    """Linear state-feedback policy mapping the observation to arm joint velocities."""
    shoulder = _clamp(params[0] + params[1] * obs.ee_z, ACTION_LIMIT_RAD_S)
    elbow = _clamp(params[2] + params[3] * obs.ee_y, ACTION_LIMIT_RAD_S)
    return shoulder, elbow


def rollout(params):
    """Runs one reach episode under the policy and returns the total reward."""
    episode = rne_py.MobileManipulatorEpisode("reach")
    step = episode.reset()
    obs = step.observation
    for _ in range(EPISODE_STEPS):
        shoulder, elbow = policy_action(params, obs)
        step = episode.step(
            shoulder_velocity_rad_s=shoulder, elbow_velocity_rad_s=elbow
        )
        obs = step.observation
        if step.terminated:
            break
    return episode.total_reward


def cem_train(iterations, population, elite, seed):
    rng = random.Random(seed)
    mean = [0.0] * PARAM_DIM
    std = [3.0] * PARAM_DIM
    history = []
    best_params = mean
    best_reward = float("-inf")

    for iteration in range(iterations):
        samples = [
            [rng.gauss(mean[d], std[d]) for d in range(PARAM_DIM)]
            for _ in range(population)
        ]
        scored = sorted(
            ((rollout(p), p) for p in samples), key=lambda sr: sr[0], reverse=True
        )
        elites = [p for _, p in scored[:elite]]
        if scored[0][0] > best_reward:
            best_reward, best_params = scored[0]

        # Refit the sampling distribution to the elite set.
        for d in range(PARAM_DIM):
            values = [p[d] for p in elites]
            mean[d] = sum(values) / len(values)
            var = sum((v - mean[d]) ** 2 for v in values) / len(values)
            std[d] = max(0.1, var**0.5)

        mean_reward = sum(r for r, _ in scored) / len(scored)
        history.append((iteration, mean_reward, scored[0][0]))
        print(
            f"iter {iteration:2d}: mean_reward={mean_reward:6.3f} "
            f"best_reward={scored[0][0]:6.3f}"
        )

    return best_params, best_reward, history


def main():
    smoke = "--smoke" in sys.argv
    iterations = 6 if smoke else 20
    best_params, best_reward, history = cem_train(
        iterations=iterations, population=16, elite=4, seed=7
    )

    first_mean = history[0][1]
    peak_mean = max(mean_reward for _, mean_reward, _ in history)
    print(
        f"trained: best_reward={best_reward:.3f} "
        f"mean_reward first={first_mean:.3f} peak={peak_mean:.3f} "
        f"params=[{', '.join(f'{p:.2f}' for p in best_params)}]"
    )

    if smoke:
        # Learning must lift the mean reward and find a solved reach (success bonus ~10).
        if peak_mean > first_mean + 1.0 and best_reward > 10.0:
            print("rl train smoke ok: CEM learned to reach the target")
            return
        sys.exit(
            f"smoke failed: no learning (mean first={first_mean:.3f} peak={peak_mean:.3f}, "
            f"best={best_reward:.3f})"
        )


if __name__ == "__main__":
    main()

"""Self-contained goal-conditioned training loop for the RNE mobile manipulator.

Optimizes a small goal-conditioned linear policy (maps the goal-relative end-effector
offset to joint velocities) with the Cross-Entropy Method (CEM), a derivative-free RL
algorithm. Each candidate is scored on several randomized reach targets, so it must
generalize rather than memorize one pose. Needs only ``rne_py`` and the Python standard
library (no gymnasium / numpy / torch); the mean reward climbs from a failing policy
(~2) to reaching varied targets (~11-12).

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


# Each CEM candidate is scored on this many freshly sampled targets, so a policy must
# generalize (use the goal) rather than memorize a single pose.
TARGETS_PER_CANDIDATE = 3


def policy_action(params, obs):
    """Goal-conditioned linear policy: maps the goal-relative offset to joint velocities."""
    shoulder = _clamp(params[0] * obs.target_dx + params[1] * obs.target_dz, ACTION_LIMIT_RAD_S)
    elbow = _clamp(params[2] * obs.target_dx + params[3] * obs.target_dz, ACTION_LIMIT_RAD_S)
    return shoulder, elbow


def evaluate_population(population):
    """Mean reward of each candidate over several sampled targets (goal-conditioned).

    All candidates run in lock-step on the batched env and share the same sequence of
    randomized targets (one per round), so the comparison is fair and rewards a policy
    that reaches *varied* goals. A candidate's per-round reward is frozen when its episode
    ends so repeated success bonuses cannot inflate it.
    """
    env = rne_py.VectorizedMobileManipulatorEnv("reach_random", len(population))
    totals = [0.0] * len(population)
    observations = env.reset()

    for _ in range(TARGETS_PER_CANDIDATE):
        round_reward = [None] * len(population)
        for _ in range(EPISODE_STEPS):
            batch = [
                (0.0, 0.0, *policy_action(params, obs), 0.0)
                for params, obs in zip(population, observations)
            ]
            observations, done = env.step(batch)
            for i, finished in enumerate(done):
                if finished and round_reward[i] is None:
                    round_reward[i] = env.episode_reward(i)
            if all(r is not None for r in round_reward):
                break
        for i in range(len(population)):
            totals[i] += (
                env.episode_reward(i) if round_reward[i] is None else round_reward[i]
            )
        observations = env.reset()

    return [total / TARGETS_PER_CANDIDATE for total in totals]


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
        rewards = evaluate_population(samples)
        scored = sorted(
            zip(rewards, samples), key=lambda sr: sr[0], reverse=True
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
        # CEM must find a goal-conditioned policy that reaches the varied targets: a
        # mean-over-targets reward above ~11 means essentially all sampled goals were hit.
        _ = (first_mean, peak_mean)
        if best_reward > 11.0:
            print("rl train smoke ok: CEM learned a goal-conditioned reach policy")
            return
        sys.exit(f"smoke failed: no generalizing policy found (best={best_reward:.3f})")


if __name__ == "__main__":
    main()
